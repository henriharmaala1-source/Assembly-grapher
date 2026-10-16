#!/usr/bin/env python3
"""
P0-B — real-footage eval harness for the lock-on tracker.

Replays a clip through the SAME tracker as simtrack.py (imported, so it can't
drift out of sync) and reports the metrics that actually matter for "does it
hold": centre error, on-target %, longest hold-time, re-acquire count, and
identity-switches (confidently locked onto the WRONG thing).

Unlike simtrack.py (synthetic scenarios), this runs on recorded video — analog
dongle clips behave nothing like the clean synthetic renders, so every tuning
decision should ultimately be checked here, not just in the sim.

Usage
-----
  # 1. Sanity-check the harness end-to-end on a built-in synthetic scenario:
  python3 eval_tracker.py --synthetic reacq

  # 2. Real clip WITH ground-truth labels (measures centre error etc.):
  python3 eval_tracker.py --video clip.mp4 --labels clip.csv [--out annotated.mp4]

  # 3. Real clip WITHOUT labels — designate a box on frame 0, get state-only
  #    stability metrics (hold-time, re-acquire count) + an annotated video to eyeball:
  python3 eval_tracker.py --video clip.mp4 --init 320,240,80 --out annotated.mp4

Label format (CSV, optional header row "frame,cx,cy,size"):
  frame_index,centre_x,centre_y[,size]      # one row per labelled frame
Frames without a row are interpolated for scoring; the first labelled row is the
designation. `size` (box side, px) is optional — defaults to 64.

Labelling a real clip cheaply: dump frames (`--dump-frames dir`), click centres in
any image tool, or run a heavier offline tracker (SAM2/CSRT) to auto-label, then
hand-correct. The harness only needs the CSV.
"""
import argparse, csv, os, sys
import numpy as np
import simtrack as st


def bgr_to_yuvdict(bgr):
    """OpenCV BGR frame -> the {y,u,v} dict the tracker consumes (centred chroma)."""
    import cv2
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV).astype(np.float32)
    return {'y': yuv[:, :, 0].copy(),
            'u': yuv[:, :, 1] - 128.0,
            'v': yuv[:, :, 2] - 128.0}


def read_video(path):
    import cv2
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"cannot open video: {path}")
    frames = []
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        frames.append(bgr)
    cap.release()
    if not frames:
        sys.exit(f"no frames decoded from {path}")
    return frames


def read_labels(path, n):
    """CSV -> per-frame (cx,cy,size) with linear interpolation between labels.
    Returns (gt list length n or None entries, designate=(frame,cx,cy,size))."""
    rows = []
    with open(path) as f:
        for r in csv.reader(f):
            if not r or r[0].strip().lower() in ('frame', 'idx', '#'):
                continue
            fi = int(float(r[0])); cx = float(r[1]); cy = float(r[2])
            sz = float(r[3]) if len(r) > 3 else 64.0
            rows.append((fi, cx, cy, sz))
    if not rows:
        sys.exit(f"no label rows in {path}")
    rows.sort()
    gt = [None] * n
    for k in range(len(rows)):
        fi, cx, cy, sz = rows[k]
        if 0 <= fi < n:
            gt[fi] = (cx, cy, sz)
        if k + 1 < len(rows):                       # interpolate to the next label
            fj, cxj, cyj, szj = rows[k + 1]
            for m in range(fi + 1, min(fj, n)):
                t = (m - fi) / (fj - fi)
                gt[m] = (cx + (cxj - cx) * t, cy + (cyj - cy) * t, sz + (szj - sz) * t)
    return gt, rows[0]


def metrics(errs, states, on_thresh, had_gt):
    """errs: per-frame centre error (np.nan where no GT). states: tracker state str."""
    out = {}
    valid = ~np.isnan(errs) if had_gt else np.zeros(len(errs), bool)
    if had_gt and valid.any():
        e = errs[valid]
        out['mean'] = float(e.mean()); out['p90'] = float(np.percentile(e, 90))
        out['max'] = float(e.max())
        on = (errs < on_thresh) & valid
        out['on_pct'] = 100.0 * on.sum() / valid.sum()
        # longest consecutive on-target run (hold-time, in frames)
        best = cur = 0
        for v in on:
            cur = cur + 1 if v else 0; best = max(best, cur)
        out['hold'] = best
        # re-acquire: off-target -> on-target transitions
        reacq = 0; prev = True
        for v in on:
            if v and not prev:
                reacq += 1
            prev = v
        out['reacq'] = reacq
        # identity switch: confidently LOCKED but far from GT (tracking the wrong thing)
        locked = np.array([s == 'LOCKED' for s in states])
        out['idswitch'] = int(((~on) & locked & valid).sum())
    # state-only stats (always available)
    lk = sum(s in ('LOCKED', 'COASTING', 'SEARCHING') for s in states)
    out['active_pct'] = 100.0 * lk / max(1, len(states))
    out['lost'] = sum(s == 'LOST' or s == 'IDLE' for s in states)
    return out


def run(frames_yuv, gt, designate, cues, on_thresh, out_path=None, bgr_frames=None):
    n = len(frames_yuv)
    tr = st.Tracker(cues)
    fi0, cx0, cy0, sz0 = designate
    tr.designate(frames_yuv[fi0], cx0, cy0, sz0)
    errs = np.full(n, np.nan, np.float32)
    states = ['IDLE'] * n
    boxes = [None] * n
    for i in range(fi0 + 1, n):
        bx, by, bs, conf, stt, ax, ay = tr.update(frames_yuv[i])
        states[i] = stt
        boxes[i] = (bx, by, bs)
        if gt is not None and gt[i] is not None:
            gx, gy, _ = gt[i]
            errs[i] = float(np.hypot(bx - gx, by - gy))
    if out_path and bgr_frames is not None:
        write_annotated(out_path, bgr_frames, boxes, states, gt)
    return metrics(errs, states, on_thresh, gt is not None)


def write_annotated(path, bgr_frames, boxes, states, gt):
    import cv2
    h, w = bgr_frames[0].shape[:2]
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), 30, (w, h))
    col = {'LOCKED': (60, 230, 40), 'COASTING': (0, 180, 255),
           'SEARCHING': (70, 70, 255), 'LOST': (0, 0, 200), 'IDLE': (120, 120, 120)}
    for i, bgr in enumerate(bgr_frames):
        img = bgr.copy(); b = boxes[i]; s = states[i]
        if b is not None:
            bx, by, bs = b; c = col.get(s, (200, 200, 200))
            cv2.rectangle(img, (int(bx - bs / 2), int(by - bs / 2)),
                          (int(bx + bs / 2), int(by + bs / 2)), c, 2)
            cv2.putText(img, s, (int(bx - bs / 2), int(by - bs / 2) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
        if gt is not None and gt[i] is not None:
            gx, gy, _ = gt[i]
            cv2.drawMarker(img, (int(gx), int(gy)), (255, 255, 255), cv2.MARKER_CROSS, 12, 1)
        vw.write(img)
    vw.release()
    print(f"  wrote annotated video -> {path}")


def main():
    ap = argparse.ArgumentParser(description="lock-on tracker eval harness (P0-B)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--video', help="video clip to replay")
    src.add_argument('--synthetic', help="built-in scenario name (harness self-test): "
                     + ",".join(st.SCEN))
    ap.add_argument('--labels', help="ground-truth CSV: frame,cx,cy[,size]")
    ap.add_argument('--init', help="cx,cy,size to designate on frame 0 (no-GT mode)")
    ap.add_argument('--init-frame', type=int, default=0)
    ap.add_argument('--cues', default='FUSE3', help="cue set: " + ",".join(st.CUESETS))
    ap.add_argument('--on-thresh', type=float, default=25.0, help="on-target centre-error px")
    ap.add_argument('--out', help="write an annotated mp4")
    ap.add_argument('--dump-frames', help="dump input frames as PNGs into this dir (to label), then exit")
    args = ap.parse_args()

    cues = st.CUESETS.get(args.cues, ['edge', 'chroma', 'none'])
    bgr_frames = None

    if args.synthetic:
        if args.synthetic not in st.SCEN:
            sys.exit(f"unknown scenario; choose from {list(st.SCEN)}")
        # Reseed so the self-test is REPRODUCIBLE. Note: synthetic mode is a harness
        # sanity check (does the metric pipeline work), NOT a tracker-quality
        # benchmark — the synthetic background is seed-sensitive and behaves nothing
        # like analog footage. Real clips (--video) are the actual eval.
        st.rng = np.random.RandomState(1234)
        st.BGT = st.rng.rand(240, 320).astype(np.float32) * 40 + 60
        frames_raw, gtraw = st.SCEN[args.synthetic]()
        frames_yuv = frames_raw
        g0 = gtraw[0]
        gt = [(gx, gy, 44.0) for (gx, gy) in gtraw]
        designate = (0, g0[0], g0[1], 44.0)
    else:
        bgr_frames = read_video(args.video)
        if args.dump_frames:
            import cv2
            os.makedirs(args.dump_frames, exist_ok=True)
            for i, f in enumerate(bgr_frames):
                cv2.imwrite(os.path.join(args.dump_frames, f"{i:05d}.png"), f)
            print(f"dumped {len(bgr_frames)} frames -> {args.dump_frames}"); return
        frames_yuv = [bgr_to_yuvdict(f) for f in bgr_frames]
        n = len(frames_yuv)
        if args.labels:
            gt, dz = read_labels(args.labels, n)
            designate = dz
        elif args.init:
            cx, cy, sz = (float(x) for x in args.init.split(','))
            gt = None
            designate = (args.init_frame, cx, cy, sz)
        else:
            sys.exit("real video needs --labels (for error metrics) or --init (state-only)")

    m = run(frames_yuv, gt, designate, cues, args.on_thresh, args.out, bgr_frames)

    print(f"\ntracker eval — cues={args.cues}  frames={len(frames_yuv)}  on<{args.on_thresh:.0f}px")
    if 'mean' in m:
        print(f"  centre err : mean {m['mean']:.1f}  p90 {m['p90']:.1f}  max {m['max']:.1f} px")
        print(f"  on-target  : {m['on_pct']:.0f}%   hold-time {m['hold']} frames   "
              f"re-acquires {m['reacq']}   id-switches {m['idswitch']}")
    else:
        print("  (no ground truth — state-only metrics)")
    print(f"  active     : {m['active_pct']:.0f}% of frames tracking   lost/idle {m['lost']}")


if __name__ == '__main__':
    main()
