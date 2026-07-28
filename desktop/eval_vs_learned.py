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
