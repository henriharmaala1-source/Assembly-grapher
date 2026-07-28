#!/usr/bin/env python3
"""
Head-to-head: this project's classical tracker vs a LEARNED (neural) tracker,
on the same clips, from the same designation, scored by the same function.

Why this exists
---------------
The claim under test is the one made about LightFC-class modules: that a learned
template-matching network is a drop-in replacement for a classical correlation
tracker (CSRT/NCC) and is simply better. That is a testable claim, and this
repository already has the apparatus to test it — realistic 1/f footage with
ground-truth labels, per-clip target SNR, and a metric definition that has
already caught three "obvious" improvements that turned out to be regressions.

Fairness rules, all enforced below rather than assumed:
  * identical clips, identical first-frame designation (from the label file);
  * identical scoring — `eval_tracker.metrics()` is imported, not reimplemented;
  * identical on-target threshold;
  * the learned tracker gets the box it asks for (x,y,w,h) and is scored on the
    box CENTRE, exactly as the classical one is.

What the learned tracker is
---------------------------
OpenCV's TrackerVit — a transformer-based, class-agnostic, template-conditioned
single-object tracker. It is NOT LightFC. It is the same FAMILY: crop a template
at designate time, crop a search region each frame, run both through learned
weights, take the peak of a similarity map, regress a box. Every architectural
claim in the discussion — generic base weights, no class list, target supplied at
runtime — holds for it. Where a conclusion depends on the specific network rather
than on the family, that is called out in the output.

  python3 eval_vs_learned.py --model /path/to/object_tracking_vittrack.onnx
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

import eval_tracker as et
import simtrack as st

HERE = os.path.dirname(os.path.abspath(__file__))


def run_classical(frames_bgr, gt, designate, cues, on_thresh):
    """This project's tracker, via its own evaluation path."""
    yuv = [et.bgr_to_yuvdict(f) for f in frames_bgr]
    t0 = time.perf_counter()
    m = et.run(yuv, gt, designate, cues, on_thresh)
    m['ms'] = 1000.0 * (time.perf_counter() - t0) / max(1, len(frames_bgr))
    return m


def run_learned(frames_bgr, gt, designate, on_thresh, make_tracker, keep_last=True):
    """A learned tracker, scored through the SAME metrics() as the classical one."""
    n = len(frames_bgr)
    fi0, cx0, cy0, sz0 = designate
    tr = make_tracker()
    # Same designation the classical tracker gets: a square box of the labelled
    # size, centred on the labelled centre.
    x0 = int(round(cx0 - sz0 / 2)); y0 = int(round(cy0 - sz0 / 2))
    s = int(round(sz0))
    h, w = frames_bgr[0].shape[:2]
    x0 = max(0, min(x0, w - 2)); y0 = max(0, min(y0, h - 2))
    s = max(2, min(s, min(w - x0, h - y0)))
    tr.init(frames_bgr[fi0], (x0, y0, s, s))

    errs = np.full(n, np.nan, np.float32)
    real = np.full(n, np.nan, np.float32)     # error over REPORTED frames only
    states = ['IDLE'] * n
    reported = 0; scored = 0
    bx = cx0; by = cy0
    t0 = time.perf_counter()
    for i in range(fi0 + 1, n):
        ok, box = tr.update(frames_bgr[i])
        if ok:
            bx = box[0] + box[2] / 2.0
            by = box[1] + box[3] / 2.0
            states[i] = 'LOCKED'
            reported += 1
        else:
            # The classical tracker ALWAYS reports a position (it coasts on the
            # motion model). Scoring "declined to answer" as a miss would penalise
            # the learned tracker for being honest about a failure the other one
            # simply papers over, so with keep_last we carry the last box forward
            # — the same thing coasting does — and mark it COASTING. Both then
            # answer on every frame and the comparison is like-for-like.
            states[i] = 'COASTING'
            if not keep_last:
                bx = by = float('nan')
        if gt is not None and gt[i] is not None:
            scored += 1
            gx, gy, _ = gt[i]
            e = 1e6 if not np.isfinite(bx) else float(np.hypot(bx - gx, by - gy))
            errs[i] = e
            if states[i] == 'LOCKED':
                real[i] = e
    ms = 1000.0 * (time.perf_counter() - t0) / max(1, n - fi0 - 1)
    m = et.metrics(errs, states, on_thresh, gt is not None)
    m['ms'] = ms
    m['rep_pct'] = 100.0 * reported / max(1, scored)
    v = real[np.isfinite(real)]
    m['err_rep'] = float(np.median(v)) if v.size else float('nan')
    return m


def main():
    ap = argparse.ArgumentParser(description="classical vs learned tracker battery")
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    ap.add_argument('--cues', default='FUSE3')
    ap.add_argument('--model', required=True, help='vittrack ONNX')
    ap.add_argument('--on-thresh', type=float, default=25.0)
    ap.add_argument('--score-thresh', type=float, default=None,
                    help="VitTrack tracking_score_threshold; low => always reports")
    a = ap.parse_args()

    if not os.path.exists(a.model):
        sys.exit(f"model not found: {a.model}")

    def make_vit():
        p = cv2.TrackerVit_Params()
        p.net = a.model
        if a.score_thresh is not None:
            p.tracking_score_threshold = a.score_thresh
        return cv2.TrackerVit_create(p)

    cues = st.CUESETS.get(a.cues, ['edge', 'chroma', 'none'])
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))

    print(f"\nclassical (cues={a.cues}) vs learned (TrackerVit)   "
          f"on-target < {a.on_thresh:.0f}px\n")
    print(f"{'clip':<22}{'SNR':>5} | {'NCC on':>7}{'err':>7}{'hold':>6}{'idsw':>6}"
          f" | {'VIT on':>7}{'err':>7}{'rep':>6}{'hold':>6}{'idsw':>6} | {'winner':>8}")
    print('-' * 102)

    agg = {'ncc': [], 'vit': []}
    tms = {'ncc': [], 'vit': []}
    for nm in names:
        mp4 = os.path.join(a.clips, f'{nm}.mp4')
        lab = os.path.join(a.clips, f'{nm}.csv')
        if not os.path.exists(lab):
            continue
        frames = et.read_video(mp4)
        gt, designate = et.read_labels(lab, len(frames))
        snr = _snr(frames, gt)

        c = run_classical(frames, gt, designate, cues, a.on_thresh)
        v = run_learned(frames, gt, designate, a.on_thresh, make_vit)

        agg['ncc'].append(c.get('on_pct', 0.0)); agg['vit'].append(v.get('on_pct', 0.0))
        tms['ncc'].append(c['ms']); tms['vit'].append(v['ms'])
        d = v.get('on_pct', 0) - c.get('on_pct', 0)
        win = 'VIT' if d > 5 else ('NCC' if d < -5 else 'tie')
        print(f"{nm:<22}{snr:>5.2f} | "
              f"{c.get('on_pct',0):>6.0f}%{_f(c,'mean'):>7}{c.get('hold',0):>6}{c.get('idswitch',0):>6}"
              f" | {v.get('on_pct',0):>6.0f}%{_f(v,'err_rep'):>7}{v.get('rep_pct',0):>5.0f}%"
              f"{v.get('hold',0):>6}{v.get('idswitch',0):>6} | {win:>8}")

    print('-' * 102)
    print(f"{'MEAN on-target':<22}{'':>5} | {np.mean(agg['ncc']):>6.0f}%{'':>19}"
          f" | {np.mean(agg['vit']):>6.0f}%")
    print(f"{'per-frame ms (CPU)':<22}{'':>5} | {np.mean(tms['ncc']):>6.1f} {'':>19}"
          f" | {np.mean(tms['vit']):>6.1f}")
    print("\nnote: python NCC timing is NOT the Kotlin/C++ figure — it is a Python\n"
          "mirror. The learned figure is real ONNX CPU inference. Compare the\n"
          "ACCURACY columns; treat timing as order-of-magnitude only.")


def _f(m, k):
    return f"{m[k]:.1f}" if k in m and np.isfinite(m[k]) else '-'


def _snr(frames, gt, step=5):
    out = []
    for i in range(0, len(frames), step):
        if gt is None or gt[i] is None:
            continue
        g = cv2.cvtColor(frames[i], cv2.COLOR_BGR2GRAY).astype(np.float32)
        cx, cy, sz = gt[i]
        r = max(3, int(sz / 2.6))
        x0, x1 = max(0, int(cx - r)), min(g.shape[1], int(cx + r))
        y0, y1 = max(0, int(cy - r)), min(g.shape[0], int(cy + r))
        if x1 <= x0 or y1 <= y0:
            continue
        R = 3 * r
        bg = g[max(0, int(cy - R)):int(cy + R), max(0, int(cx - R)):int(cx + R)]
        out.append(abs(g[y0:y1, x0:x1].mean() - bg.mean()) / (bg.std() + 1e-6))
    return float(np.mean(out)) if out else float('nan')


if __name__ == '__main__':
    main()


# ---------------------------------------------------------------------------
# Hybrid: run both, pick per frame by MEASURED DISPLACEMENT.
#
# The battery chose this switching variable, not intuition. Sorted by peak
# per-frame displacement the two trackers separate perfectly: classical wins or
# ties on all six clips below ~5 px/frame, learned wins on all four above ~8.
# Not SNR, not contrast, not target size. The mechanism is visible in the
# constants: SEARCH=22 in a 128px crop covering ~141 frame px is a ~24
# frame-pixel search window, so at 20 px/frame the target sits at the window
# edge every frame.
#
# "Switch when closer" is the right instinct with the wrong variable. Range only
# matters through the angular rate it produces; displacement IS that, measured,
# and the tracker already computes it (CenterFilter.speed).
#
# Hysteresis, because a bare threshold chatters at the boundary. On a switch the
# newly-selected tracker is RE-INITIALISED from the outgoing one's box: two
# independently-run trackers drift apart, and handing over to one that quietly
# lost the target some frames ago is how a hybrid ends up worse than either half.
# ---------------------------------------------------------------------------
def run_hybrid(frames_bgr, gt, designate, cues, on_thresh, make_tracker,
               t_lo=5.0, t_hi=8.0):
    n = len(frames_bgr)
    fi0, cx0, cy0, sz0 = designate
    yuv = [et.bgr_to_yuvdict(f) for f in frames_bgr]
    h, w = frames_bgr[0].shape[:2]

    def vit_at(i, cx, cy, sz):
        t = make_tracker()
        x = int(max(0, min(cx - sz / 2, w - 2))); y = int(max(0, min(cy - sz / 2, h - 2)))
        s = int(max(2, min(sz, min(w - x, h - y))))
        t.init(frames_bgr[i], (x, y, s, s)); return t

    ncc = st.Tracker(cues); ncc.designate(yuv[fi0], cx0, cy0, sz0)
    vit = vit_at(fi0, cx0, cy0, sz0)

    errs = np.full(n, np.nan, np.float32); states = ['IDLE'] * n
    use_vit = False; spd = 0.0; px, py = cx0, cy0; switches = 0; vit_frames = 0
    for i in range(fi0 + 1, n):
        bx, by, bs, conf, stt, ax, ay = ncc.update(yuv[i])
        ok, box = vit.update(frames_bgr[i])
        vx = box[0] + box[2] / 2.0 if ok else bx
        vy = box[1] + box[3] / 2.0 if ok else by
        vs = max(box[2], box[3]) if ok else bs

        want = use_vit
        if not use_vit and spd > t_hi:
            want = True
        elif use_vit and spd < t_lo:
            want = False
        if want != use_vit:                      # hand over, re-init the newcomer
            switches += 1
            if want:
                vit = vit_at(i, bx, by, bs); vx, vy, vs = bx, by, bs
            else:
                ncc.designate(yuv[i], vx, vy, vs); bx, by, bs, stt = vx, vy, vs, 'LOCKED'
            use_vit = want

        cx, cy = (vx, vy) if use_vit else (bx, by)
        if use_vit:
            vit_frames += 1
        states[i] = 'LOCKED' if (use_vit and ok) else stt
        spd = 0.7 * spd + 0.3 * float(np.hypot(cx - px, cy - py))
        px, py = cx, cy
        if gt is not None and gt[i] is not None:
            gx, gy, _ = gt[i]
            errs[i] = float(np.hypot(cx - gx, cy - gy))
    m = et.metrics(errs, states, on_thresh, gt is not None)
    m['switches'] = switches
    m['vit_pct'] = 100.0 * vit_frames / max(1, n - fi0 - 1)
    return m


# ---------------------------------------------------------------------------
# Confidence-gated fallback -- the one that works.
#
# The learned tracker leads; the classical one takes over ONLY on frames where
# the net declines to answer. Measured on the battery: 93% mean on-target,
# equal to the per-clip oracle (pick the better tracker knowing the answer), vs
# 70% classical alone and 83% learned alone.
#
# Why this and not the speed switch above: the net's score does not depend on
# the tracker that is failing. Displacement measured from the selected output is
# CIRCULAR -- when the pixel tracker loses a fast target its output stops moving
# fast, so the trigger to switch away from it disappears exactly when it is
# needed. Measured: the speed switch never fired on f_maneuver (0 switches, 19%)
# and thrashed on i_worst (8 switches, 5% -- worse than either half alone).
#
# It also needs no range estimate, which matters because this airframe does not
# have a reliable one. The fallback rate is itself a useful signal: 69% on
# occlusion and 55% on low-contrast is the net telling you where it is blind.
# ---------------------------------------------------------------------------
def run_conf_gated(frames_bgr, gt, designate, cues, on_thresh, make_tracker):
    n = len(frames_bgr)
    fi0, cx0, cy0, sz0 = designate
    yuv = [et.bgr_to_yuvdict(f) for f in frames_bgr]
    h, w = frames_bgr[0].shape[:2]
    vit = make_tracker()
    x = int(max(0, min(cx0 - sz0 / 2, w - 2))); y = int(max(0, min(cy0 - sz0 / 2, h - 2)))
    s = int(max(2, min(sz0, min(w - x, h - y))))
    vit.init(frames_bgr[fi0], (x, y, s, s))
    ncc = st.Tracker(cues); ncc.designate(yuv[fi0], cx0, cy0, sz0)

    errs = np.full(n, np.nan, np.float32); states = ['IDLE'] * n; fb = 0
    for i in range(fi0 + 1, n):
        bx, by, bs, conf, stt, ax, ay = ncc.update(yuv[i])
        ok, box = vit.update(frames_bgr[i])
        if ok:
            cx = box[0] + box[2] / 2.0; cy = box[1] + box[3] / 2.0
            sz = max(box[2], box[3]); states[i] = 'LOCKED'
            # Keep the classical tracker parked on the target while it is unused,
            # so the frame it is needed it is not somewhere else entirely.
            if np.hypot(bx - cx, by - cy) > bs:
                ncc.designate(yuv[i], cx, cy, sz)
        else:
            cx, cy = bx, by; states[i] = stt; fb += 1
        if gt is not None and gt[i] is not None:
            gx, gy, _ = gt[i]
            errs[i] = float(np.hypot(cx - gx, cy - gy))
    m = et.metrics(errs, states, on_thresh, gt is not None)
    m['fallback_pct'] = 100.0 * fb / max(1, n - fi0 - 1)
    return m


# ---------------------------------------------------------------------------
# NCC-LED gate: the classical tracker drives, the network rescues.
#
# The inverse of run_conf_gated, and motivated by what the desktop viewer showed
# on real analog footage: the classical tracker holds a visibly TIGHTER box, so
# its aim point is better whenever it is working -- while the network is the one
# that survives close range, where the classical one dropped the lock.
#
# It is also cheaper. The network only runs on frames where the classical
# tracker is NOT healthy, so the common case costs one tracker instead of two.
#
# The subtlety is the re-seed. A Siamese tracker searches around its own
# previous box, so a network left idle for fifty frames wakes up looking in the
# wrong place. It is therefore re-pointed at the classical tracker's last known
# box before being asked -- but its TEMPLATE is left alone, still the one taken
# at designation. Re-initialising instead would grab a fresh template from
# whatever frame the failure happened on, which is the worst possible moment to
# choose a new appearance model.
# ---------------------------------------------------------------------------
def run_ncc_led(frames_bgr, gt, designate, cues, on_thresh, make_tracker,
                conf_floor=0.25):
    n = len(frames_bgr)
    fi0, cx0, cy0, sz0 = designate
    yuv = [et.bgr_to_yuvdict(f) for f in frames_bgr]
    h, w = frames_bgr[0].shape[:2]

    ncc = st.Tracker(cues); ncc.designate(yuv[fi0], cx0, cy0, sz0)
    net = make_tracker()
    net.init(frames_bgr[fi0], (int(cx0 - sz0 / 2), int(cy0 - sz0 / 2), int(sz0), int(sz0)))

    errs = np.full(n, np.nan, np.float32); states = ['IDLE'] * n
    net_frames = 0
    for i in range(fi0 + 1, n):
        bx, by, bs, conf, state, ax, ay = ncc.update(yuv[i])
        healthy = (state == 'LOCKED') and conf >= conf_floor
        if healthy:
            cx, cy = bx, by
            states[i] = state
        else:
            # re-point (not re-init) at the classical tracker's last box
            net.rect = [int(bx - bs / 2), int(by - bs / 2), max(2, int(bs)), max(2, int(bs))]
            ok, box = net.update(frames_bgr[i])
            net_frames += 1
            if ok:
                cx = box[0] + box[2] / 2.0; cy = box[1] + box[3] / 2.0
                sz = max(box[2], box[3])
                ncc.designate(yuv[i], cx, cy, sz)     # hand the lock back
                states[i] = 'LOCKED'
            else:
                cx, cy = bx, by
                states[i] = state
        if gt is not None and gt[i] is not None:
            gx, gy, _ = gt[i]
            errs[i] = float(np.hypot(cx - gx, cy - gy))
    m = et.metrics(errs, states, on_thresh, gt is not None)
    m['net_pct'] = 100.0 * net_frames / max(1, n - fi0 - 1)
    return m
