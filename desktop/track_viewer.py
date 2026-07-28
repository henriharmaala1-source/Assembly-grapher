#!/usr/bin/env python3
"""
Side-by-side tracker viewer — run it with no arguments.

    python3 track_viewer.py

Everything is driven from the window: pick a video from the built-in browser,
drag a box around a target, watch all three trackers run the same footage from
the same designation at the same time.

  NCC     this project's classical pixel tracker (simtrack)
  CSRT    OpenCV's CSR-DCF -- a discriminative correlation filter that learns
          online, no network and no model file. Needs opencv-contrib-python;
          if it is missing the viewer simply runs without it.
  LEARNED the ONNX Siamese network (siamese_onnx)
  GATED   the network leading, with its OWN private NCC as fallback on frames
          where the network declines to answer

GATED gets its own pair of tracker instances rather than reusing the two above,
because a shared instance would be re-designated by the gate and the standalone
comparison would silently stop being standalone.

Why a viewer at all
-------------------
Every number this project has on the learned tracker came from synthetic 1/f
clips. The contrast cliff, the ~5 px/frame boundary, the 93% for gating — all
synthetic. This exists to put the same three trackers on REAL footage where the
failure can be watched rather than summarised, because the one thing the
battery cannot tell you is what losing the lock actually looks like.

No command-line control by design: paths, designation, playback and export are
all in the window.

Controls
  drag on the video ...... designate (all three seed from the same box)
  click the scrub bar .... seek (resets the trackers — they are causal)
  SPACE .................. play / pause          . or , ... step
  F ...................... fullscreen            R ....... clear the lock
  O ...................... back to the browser   S ....... save annotated mp4
  Q / ESC ................ quit
"""
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import eval_tracker as et                      # noqa: E402
import simtrack as st                          # noqa: E402
from siamese_onnx import SiameseOnnxTracker    # noqa: E402

def _find_model():
    # The model already ships as an Android asset; don't keep a second copy.
    for c in (os.path.join(HERE, 'vittrack.onnx'),
              os.path.join(HERE, os.pardir, 'android-tracker', 'app', 'src',
                           'main', 'assets', 'vittrack.onnx')):
        if os.path.exists(c):
            return os.path.abspath(c)
    return os.path.join(HERE, 'vittrack.onnx')


MODEL = _find_model()
VIDEO_EXT = ('.mp4', '.avi', '.mov', '.mkv', '.m4v', '.webm')

COL_NCC = (80, 220, 90)        # green
COL_LEARN = (240, 170, 60)     # blue-ish
COL_GATE = (90, 90, 255)       # red
COL_CSRT = (60, 230, 240)      # yellow

# CSRT (CSR-DCF) lives in opencv_contrib, which the plain opencv-python wheel
# does not ship. It scored 88% on the battery against 70% for this project's
# tracker and 83% for the network, so it belongs in the comparison -- but its
# absence must not stop the viewer running.
HAVE_CSRT = hasattr(cv2, 'TrackerCSRT_create')
COL_GT = (200, 200, 200)
FONT = cv2.FONT_HERSHEY_SIMPLEX


# --------------------------------------------------------------------------- ui
class Btn:
    def __init__(self, label, key=None):
        self.label, self.key = label, key
        self.rect = (0, 0, 0, 0)

    def draw(self, img, x, y, h=34, pad=14, on=False):
        (tw, _), _ = cv2.getTextSize(self.label, FONT, 0.5, 1)
        w = tw + pad * 2
        self.rect = (x, y, w, h)
        cv2.rectangle(img, (x, y), (x + w, y + h), (55, 55, 60) if not on else (40, 110, 170), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), (120, 120, 130), 1)
        cv2.putText(img, self.label, (x + pad, y + h - 12), FONT, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        return x + w + 8

    def hit(self, mx, my):
        x, y, w, h = self.rect
        return x <= mx <= x + w and y <= my <= y + h


def list_dir(path, limit=400):
    """One directory's sub-folders and videos, for the in-window browser.

    Navigable rather than a recursive scan of the script folder: real footage
    lives in ~/Downloads or wherever yt-dlp left it, and a browser that can only
    see next to the script would mean copying every clip in before it could be
    opened. Returns [(is_dir, label, fullpath)] with '..' first.
    """
    out = []
    parent = os.path.dirname(os.path.abspath(path))
    if parent and parent != os.path.abspath(path):
        out.append((True, '..', parent))
    try:
        names = sorted(os.listdir(path), key=str.lower)
    except OSError:
        return out
    for n in names:
        if n.startswith('.') or n == '__pycache__':
            continue
        full = os.path.join(path, n)
        if os.path.isdir(full):
            out.append((True, n + '/', full))
    for n in names:
        if n.lower().endswith(VIDEO_EXT):
            out.append((False, n, os.path.join(path, n)))
        if len(out) >= limit:
            break
    return out


# ----------------------------------------------------------------- tracker set
class TriTracker:
    """The three tracked outputs, all seeded from one designation."""

    def __init__(self, model_path):
        self.cues = st.CUESETS.get('FUSE3', ['edge', 'chroma', 'none'])
        self.model = model_path
        self.have_model = os.path.exists(model_path)
        self.reset()

    def _mk_learned(self):
        return SiameseOnnxTracker(self.model) if self.have_model else None

    def reset(self):
        self.ncc = None
        self.csrt = None
        self.learn = None
        self.g_ncc = None
        self.g_learn = None
        self.out = {}
        self.ms = {}
        self.gate_fallback = False
        self.armed = False

    def designate(self, bgr, yuv, cx, cy, size):
        size = max(8.0, float(size))
        rect = (int(cx - size / 2), int(cy - size / 2), int(size), int(size))
        self.ncc = st.Tracker(self.cues); self.ncc.designate(yuv, cx, cy, size)
        self.g_ncc = st.Tracker(self.cues); self.g_ncc.designate(yuv, cx, cy, size)
        if HAVE_CSRT:
            self.csrt = cv2.TrackerCSRT_create()
            self.csrt.init(bgr, rect)
        self.learn = self._mk_learned()
        self.g_learn = self._mk_learned()
        for t in (self.learn, self.g_learn):
            if t is not None:
                t.init(bgr, rect)
        self.armed = True
        self.out = {}
        self.gate_fallback = False

    def update(self, bgr, yuv):
        if not self.armed:
            return
        t0 = time.perf_counter()
        bx, by, bs, conf, state, ax, ay = self.ncc.update(yuv)
        self.ms['NCC'] = 1000 * (time.perf_counter() - t0)
        self.out['NCC'] = (bx, by, bs, f"{state} {conf*100:.0f}%")

        if self.csrt is not None:
            t0 = time.perf_counter()
            try:
                ok, box = self.csrt.update(bgr)
            except cv2.error:
                ok, box = False, (0, 0, 0, 0)     # CSRT throws if the box leaves the frame
            self.ms['CSRT'] = 1000 * (time.perf_counter() - t0)
            if ok:
                self.out['CSRT'] = (box[0] + box[2] / 2, box[1] + box[3] / 2,
                                    max(box[2], box[3]), "LOCKED")
            else:
                # No confidence to report: unlike the network, CSRT exposes no
                # score, so a failure is all-or-nothing. That is exactly why it
                # cannot be the one that drives a confidence gate.
                self.out['CSRT'] = self.out.get('CSRT', (0, 0, 0, ''))[:3] + ("LOST",)

        if self.learn is not None:
            t0 = time.perf_counter()
            ok, box = self.learn.update(bgr)
            self.ms['LEARNED'] = 1000 * (time.perf_counter() - t0)
            self.out['LEARNED'] = (box[0] + box[2] / 2, box[1] + box[3] / 2,
                                   max(box[2], box[3]),
                                   ("LOCKED" if ok else "no-report") +
                                   f" {self.learn.getTrackingScore():.2f}")

        # GATED: network leads, private NCC covers the frames it declines.
        if self.g_learn is not None:
            t0 = time.perf_counter()
            gbx, gby, gbs, gconf, gstate, _, _ = self.g_ncc.update(yuv)
            ok, box = self.g_learn.update(bgr)
            if ok:
                cx = box[0] + box[2] / 2; cy = box[1] + box[3] / 2
                sz = max(box[2], box[3])
                self.gate_fallback = False
                # Keep the fallback parked on the target while it is unused, so
                # the frame it is needed it is not somewhere else entirely.
                if np.hypot(gbx - cx, gby - cy) > gbs:
                    self.g_ncc.designate(yuv, cx, cy, sz)
                lbl = f"net {self.g_learn.getTrackingScore():.2f}"
            else:
                cx, cy, sz = gbx, gby, gbs
                self.gate_fallback = True
                lbl = f"FALLBACK {gstate}"
            self.ms['GATED'] = 1000 * (time.perf_counter() - t0)
            self.out['GATED'] = (cx, cy, sz, lbl)


# ---------------------------------------------------------------------- viewer
class Viewer:
    W, H = 1920, 1080
    MODE_BROWSE, MODE_PLAY = 0, 1

    def __init__(self):
        self.canvas = np.zeros((self.H, self.W, 3), np.uint8)
        self.mode = self.MODE_BROWSE
        self.browse_dir = HERE
        self.entries = list_dir(self.browse_dir)
        self.scroll = 0
        self.cap = None
        self.path = None
        self.frame = None
        self.idx = 0
        self.n = 0
        self.playing = False
        self.fullscreen = False
        self.drag = None
        self.tri = TriTracker(MODEL)
        self.gt = None
        self.rows = []
        self.video_rect = (0, 0, 1, 1)
        self.scrub_rect = (0, 0, 1, 1)
        self.msg = ""
        self.rec = None
        self.btn = {k: Btn(k) for k in
                    ('PLAY', 'STEP', 'CLEAR', 'OPEN', 'SAVE', 'FULL', 'QUIT')}

    # -- video ------------------------------------------------------------
    def open(self, path):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            self.msg = f"cannot open {os.path.basename(path)}"
            return
        self.cap, self.path = cap, path
        self.n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.idx = -1
        self.tri.reset()
        self.gt = None
        lab = os.path.splitext(path)[0] + '.csv'
        if os.path.exists(lab):
            try:
                # Ground truth, when the clip has it: the label track makes the
                # comparison quantitative instead of a vibe.
                self.gt, _ = et.read_labels(lab, max(1, self.n))
                probe = cv2.VideoCapture(path)
                ok, pf = probe.read(); probe.release()
                if ok and pf.shape[1] > self.MAX_W:
                    k = self.MAX_W / pf.shape[1]      # labels are in SOURCE px
                    self.gt = [None if g is None else (g[0]*k, g[1]*k, g[2]*k)
                               for g in self.gt]
            except Exception:
                self.gt = None
        self.advance()
        self.mode = self.MODE_PLAY
        self.playing = False
        self.msg = os.path.basename(path) + ("   [labels]" if self.gt else "")

    # Real footage is 1080p or 4K. The learned tracker resamples to fixed crops
    # so it does not care, but the classical one runs a full-FRAME optical-flow
    # grid every update, and in Python that scales straight into the seconds.
    # Cap the decode width; the whole pipeline (designation, boxes, labels) then
    # works in the same reduced space, so nothing needs rescaling downstream.
    MAX_W = 960

    def advance(self):
        if self.cap is None:
            return False
        ok, f = self.cap.read()
        if not ok:
            self.playing = False
            return False
        if f.shape[1] > self.MAX_W:
            k = self.MAX_W / f.shape[1]
            f = cv2.resize(f, (self.MAX_W, int(round(f.shape[0] * k))),
                           interpolation=cv2.INTER_AREA)
        self.frame = f
        self.idx += 1
        if self.tri.armed:
            self.tri.update(f, et.bgr_to_yuvdict(f))
        if self.rec is not None:
            self.rec.write(cv2.resize(self.compose(), (self.W // 2, self.H // 2)))
        return True

    def seek(self, i):
        if self.cap is None:
            return
        i = int(max(0, min(i, max(0, self.n - 1))))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        self.idx = i - 1
        # The trackers are causal: a jump invalidates their state, so the honest
        # thing is to drop the lock rather than silently carry a stale one.
        self.tri.reset()
        self.advance()

    # -- drawing ----------------------------------------------------------
    def compose(self):
        c = self.canvas
        c[:] = (24, 24, 27)
        if self.frame is None:
            return c
        fh, fw = self.frame.shape[:2]
        top, bottom = 70, 150
        avail_w, avail_h = self.W - 40, self.H - top - bottom
        s = min(avail_w / fw, avail_h / fh)
        vw, vh = int(fw * s), int(fh * s)
        vx, vy = (self.W - vw) // 2, top + (avail_h - vh) // 2
        self.video_rect = (vx, vy, vw, vh)
        c[vy:vy + vh, vx:vx + vw] = cv2.resize(self.frame, (vw, vh))

        def to_view(x, y):
            return int(vx + x * s), int(vy + y * s)

        if self.gt is not None and 0 <= self.idx < len(self.gt) and self.gt[self.idx]:
            gx, gy, gs = self.gt[self.idx]
            p0 = to_view(gx - gs / 2, gy - gs / 2); p1 = to_view(gx + gs / 2, gy + gs / 2)
            cv2.rectangle(c, p0, p1, COL_GT, 1)

        for i, (name, col) in enumerate((('NCC', COL_NCC), ('CSRT', COL_CSRT),
                                         ('LEARNED', COL_LEARN), ('GATED', COL_GATE))):
            r = self.tri.out.get(name)
            if not r:
                continue
            cx, cy, sz, _ = r
            p0 = to_view(cx - sz / 2, cy - sz / 2); p1 = to_view(cx + sz / 2, cy + sz / 2)
            th = 3 if name == 'GATED' else 2
            cv2.rectangle(c, p0, p1, col, th)
            # Stagger the labels. When two trackers agree their boxes coincide,
            # and overlapping text renders as unreadable mush at exactly the
            # moment you most want to know which is which.
            cv2.putText(c, name, (p0[0], max(12, p0[1] - 6 - i * 14)),
                        FONT, 0.45, col, 1, cv2.LINE_AA)

        if self.drag:
            (x0, y0), (x1, y1) = self.drag
            cv2.rectangle(c, (x0, y0), (x1, y1), (255, 255, 255), 1)

        self.draw_hud(c)
        self.draw_controls(c)
        return c

    def draw_hud(self, c):
        cv2.putText(c, self.msg, (20, 30), FONT, 0.6, (200, 200, 210), 1, cv2.LINE_AA)
        if not self.tri.armed:
            cv2.putText(c, "drag a box around a target to lock all three",
                        (20, 56), FONT, 0.6, (150, 190, 240), 1, cv2.LINE_AA)
            return
        y = 56
        for name, col in (('NCC', COL_NCC), ('CSRT', COL_CSRT),
                          ('LEARNED', COL_LEARN), ('GATED', COL_GATE)):
            r = self.tri.out.get(name)
            if not r:
                continue
            cx, cy, _, lbl = r
            err = ""
            if self.gt is not None and 0 <= self.idx < len(self.gt) and self.gt[self.idx]:
                gx, gy, _ = self.gt[self.idx]
                err = f"   err {np.hypot(cx - gx, cy - gy):5.1f}px"
            cv2.putText(c, f"{name:<8} {lbl:<22}{self.tri.ms.get(name, 0):5.1f} ms{err}",
                        (20, y), FONT, 0.55, col, 1, cv2.LINE_AA)
            y += 24
        if self.tri.gate_fallback:
            cv2.putText(c, "GATE: network declined -> classical", (20, y),
                        FONT, 0.55, (120, 200, 255), 1, cv2.LINE_AA)

    def draw_controls(self, c):
        bar_y = self.H - 96
        x0, x1 = 20, self.W - 20
        self.scrub_rect = (x0, bar_y, x1 - x0, 14)
        cv2.rectangle(c, (x0, bar_y), (x1, bar_y + 14), (60, 60, 66), -1)
        if self.n > 1:
            px = int(x0 + (x1 - x0) * self.idx / max(1, self.n - 1))
            cv2.rectangle(c, (x0, bar_y), (px, bar_y + 14), (70, 130, 200), -1)
            cv2.rectangle(c, (px - 2, bar_y - 4), (px + 2, bar_y + 18), (240, 240, 240), -1)
        cv2.putText(c, f"{self.idx + 1}/{self.n}", (x1 - 110, bar_y - 8),
                    FONT, 0.5, (190, 190, 200), 1, cv2.LINE_AA)
        x = 20; y = self.H - 60
        self.btn['PLAY'].label = 'PAUSE' if self.playing else 'PLAY'
        for k in ('PLAY', 'STEP', 'CLEAR', 'OPEN', 'SAVE', 'FULL', 'QUIT'):
            on = (k == 'SAVE' and self.rec is not None)
            x = self.btn[k].draw(c, x, y, on=on)
        if not HAVE_CSRT:
            cv2.putText(c, "no CSRT — pip install opencv-contrib-python to compare it",
                        (20, self.H - 108), FONT, 0.45, (120, 170, 255), 1, cv2.LINE_AA)
        if not self.tri.have_model:
            cv2.putText(c, "vittrack.onnx not found next to this script — NCC only",
                        (x + 20, y + 22), FONT, 0.5, (120, 170, 255), 1, cv2.LINE_AA)

    def draw_browser(self):
        c = self.canvas
        c[:] = (24, 24, 27)
        cv2.putText(c, "pick a video", (40, 60), FONT, 0.9, (235, 235, 240), 2, cv2.LINE_AA)
        cv2.putText(c, self.browse_dir, (40, 92), FONT, 0.5, (140, 140, 150), 1, cv2.LINE_AA)
        self.rows = []
        y = 140
        for i in range(self.scroll, min(len(self.entries), self.scroll + 22)):
            is_dir, label, full = self.entries[i]
            r = (40, y - 22, self.W - 80, 30)
            self.rows.append((r, is_dir, full))
            cv2.rectangle(c, (r[0], r[1]), (r[0] + r[2], r[1] + r[3]), (38, 38, 43), -1)
            col = (170, 200, 245) if is_dir else (215, 215, 225)
            cv2.putText(c, label, (52, y), FONT, 0.55, col, 1, cv2.LINE_AA)
            y += 34
        if len(self.entries) <= 1:
            cv2.putText(c, "nothing here — use '..' to go up", (40, 150),
                        FONT, 0.6, (150, 190, 240), 1, cv2.LINE_AA)
        x = 40; yb = self.H - 60
        for k in ('OPEN', 'QUIT'):
            self.btn[k].label = 'HOME' if k == 'OPEN' else 'QUIT'
            x = self.btn[k].draw(c, x, yb)
        if self.msg:
            cv2.putText(c, self.msg, (40, self.H - 90), FONT, 0.55,
                        (120, 170, 255), 1, cv2.LINE_AA)
        return c

    # -- input ------------------------------------------------------------
    def on_mouse(self, ev, mx, my, flags, _):
        if self.mode == self.MODE_BROWSE:
            if ev == cv2.EVENT_LBUTTONDOWN:
                for (rx, ry, rw, rh), is_dir, p in self.rows:
                    if rx <= mx <= rx + rw and ry <= my <= ry + rh:
                        if is_dir:
                            self.browse_dir = p
                            self.entries = list_dir(p); self.scroll = 0
                        else:
                            self.open(p)
                        return
                if self.btn['OPEN'].hit(mx, my):
                    self.browse_dir = os.path.expanduser('~')
                    self.entries = list_dir(self.browse_dir); self.scroll = 0
                elif self.btn['QUIT'].hit(mx, my):
                    self.quit = True
            elif ev == cv2.EVENT_MOUSEWHEEL:
                self.scroll = max(0, min(max(0, len(self.entries) - 1),
                                         self.scroll + (-1 if flags > 0 else 1)))
            return

        if ev == cv2.EVENT_LBUTTONDOWN:
            for k, b in self.btn.items():
                if b.hit(mx, my):
                    self.action(k); return
            sx, sy, sw, sh = self.scrub_rect
            if sx <= mx <= sx + sw and sy - 8 <= my <= sy + sh + 8:
                self.seek((mx - sx) / max(1, sw) * (self.n - 1)); return
            vx, vy, vw, vh = self.video_rect
            if vx <= mx <= vx + vw and vy <= my <= vy + vh:
                self.drag = [(mx, my), (mx, my)]
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag:
            self.drag[1] = (mx, my)
        elif ev == cv2.EVENT_LBUTTONUP and self.drag:
            (x0, y0), (x1, y1) = self.drag
            self.drag = None
            vx, vy, vw, vh = self.video_rect
            fh, fw = self.frame.shape[:2]
            s = vw / fw
            cx = ((x0 + x1) / 2 - vx) / s
            cy = ((y0 + y1) / 2 - vy) / s
            size = max(abs(x1 - x0), abs(y1 - y0)) / s
            if size < 6:            # a click, not a drag: use a sensible default
                size = 48
            self.tri.designate(self.frame, et.bgr_to_yuvdict(self.frame), cx, cy, size)
            self.msg = f"locked at ({cx:.0f},{cy:.0f}) size {size:.0f}px"

    def action(self, k):
        if k == 'PLAY':
            self.playing = not self.playing
        elif k == 'STEP':
            self.playing = False; self.advance()
        elif k == 'CLEAR':
            self.tri.reset(); self.msg = "lock cleared"
        elif k == 'OPEN':
            self.mode = self.MODE_BROWSE; self.playing = False
            self.entries = list_dir(self.browse_dir)
        elif k == 'SAVE':
            self.toggle_record()
        elif k == 'FULL':
            self.fullscreen = not self.fullscreen
            cv2.setWindowProperty('tracker', cv2.WND_PROP_FULLSCREEN,
                                  cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)
        elif k == 'QUIT':
            self.quit = True

    def toggle_record(self):
        if self.rec is not None:
            self.rec.release(); self.rec = None; self.msg = "saved"
            return
        out = os.path.splitext(self.path)[0] + '_compare.mp4'
        self.rec = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 25,
                                   (self.W // 2, self.H // 2))
        self.msg = f"recording -> {os.path.basename(out)}"

    # -- loop -------------------------------------------------------------
    def run(self):
        self.quit = False
        cv2.namedWindow('tracker', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('tracker', 1280, 720)
        cv2.setMouseCallback('tracker', self.on_mouse)
        while not self.quit:
            if self.mode == self.MODE_BROWSE:
                cv2.imshow('tracker', self.draw_browser())
            else:
                if self.playing:
                    self.advance()
                cv2.imshow('tracker', self.compose())
            k = cv2.waitKey(15 if self.playing else 30) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord(' '):
                self.action('PLAY')
            elif k in (ord('.'), ord(',')):
                self.action('STEP')
            elif k == ord('f'):
                self.action('FULL')
            elif k == ord('r'):
                self.action('CLEAR')
            elif k == ord('o'):
                self.action('OPEN')
            elif k == ord('s'):
                self.action('SAVE')
        if self.rec is not None:
            self.rec.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    Viewer().run()
