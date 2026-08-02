#!/usr/bin/env python3
"""
Paired A/B with a perturbation ensemble — the instrument that should have been
used for every decision in this project.

WHY. eval_noisefloor.py showed a single battery run has a spread of 18 points on
the mean and up to 67 on one clip, under perturbations that cannot matter
physically. Differences of 2-11 points have been acted on. A single run cannot
resolve them.

THE FIX is pairing, not more clips. The chaos is deterministic: a tiny input
change decides which frame re-locks and the trajectory diverges from there. So
run BOTH arms under the SAME perturbation, and the divergence largely cancels in
the difference. Repeat over N perturbations and the standard error of the paired
delta collapses even though each individual arm is still wildly noisy.

Perturbation: the designation box jittered by U(-0.25, +0.25) px. Below any
plausible human or labelling precision, and below the tracker's own sub-pixel
refine, so it cannot carry information — but it is enough to reseed the chaos.

    python3 eval_ab.py                       # the shipped fixes vs unfixed
    python3 eval_ab.py --seeds 12
"""
import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _one(job):
    """One arm, one perturbation seed, over the whole battery."""
    label, cfg, seed, clipdir, names = job
    sys.path.insert(0, HERE)
    import importlib
    import eval_tracker as et
    import simtrack as st
    importlib.reload(st)          # a pool worker must not inherit a polluted module
    et.st = st
    for k, v in cfg.items():
        setattr(st, k, v)
    cues = st.CUESETS['FUSE3']
    rng = np.random.RandomState(seed)
    out = []
    for nm in names:
        frames = et.read_video(os.path.join(clipdir, nm + '.mp4'))
        gt, dz = et.read_labels(os.path.join(clipdir, nm + '.csv'), len(frames))
        yuv = [et.bgr_to_yuvdict(x) for x in frames]
        fi0, cx0, cy0, sz0 = dz
        jx, jy = rng.uniform(-0.25, 0.25, 2)      # same draw order in both arms
        out.append(et.run(yuv, gt, (fi0, cx0 + jx, cy0 + jy, sz0),
                          cues, 25.0).get('on_pct', 0.0))
    return label, seed, out


def paired(cfgA, cfgB, seeds, clipdir, names, labels=('A', 'B')):
    jobs = []
    for s in range(seeds):
        jobs.append((labels[0], cfgA, 1000 + s, clipdir, names))
        jobs.append((labels[1], cfgB, 1000 + s, clipdir, names))
    with mp.Pool(max(1, os.cpu_count() or 2), maxtasksperchild=1) as p:
        got = p.map(_one, jobs)
    A = {s: v for l, s, v in got if l == labels[0]}
    B = {s: v for l, s, v in got if l == labels[1]}
    return A, B


def report(A, B, names, labels):
    ss = sorted(A)
    ma = np.array([np.mean(A[s]) for s in ss])
    mb = np.array([np.mean(B[s]) for s in ss])
    d = mb - ma
    n = len(d)
    se = d.std(ddof=1) / np.sqrt(n) if n > 1 else float('nan')
    print(f"{'seed':>6}{labels[0]:>10}{labels[1]:>10}{'delta':>9}")
    for i, s in enumerate(ss):
        print(f"{s:>6}{ma[i]:>9.1f}%{mb[i]:>9.1f}%{d[i]:>+9.1f}")
    print('-' * 35)
    print(f"{'mean':>6}{ma.mean():>9.1f}%{mb.mean():>9.1f}%{d.mean():>+9.2f}")
    print(f"\n  paired delta   {d.mean():+.2f} +/- {se:.2f} (SE over {n} draws)")
    if n > 1 and se > 0:
        print(f"  t              {d.mean()/se:+.2f}")
    print(f"  positive in    {int((d > 0).sum())} of {n} draws")
    print(f"  single-arm sd  {ma.std(ddof=1):.2f} / {mb.std(ddof=1):.2f} points"
          "   <- why one run proves nothing")
    print(f"\n  per-clip wins for {labels[1]} (of {n} draws):")
    for i, nm in enumerate(names):
        wa = np.array([A[s][i] for s in ss]); wb = np.array([B[s][i] for s in ss])
        w = int((wb > wa).sum()); l = int((wb < wa).sum())
        flag = '  <<' if (w >= n - 1 or l >= n - 1) and w != l else ''
        print(f"    {nm:<22}{w:>3}W {l:>3}L   median {np.median(wa):>5.0f}%"
              f" -> {np.median(wb):>5.0f}%{flag}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    ap.add_argument('--seeds', type=int, default=8)
    a = ap.parse_args()
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))

    # The three correctness fixes, together. The two geometry ones MUST be
    # measured together: they are biases of opposite sign that partly cancel in
    # the unfixed code, so fixing either alone is measurably worse than fixing
    # neither (bias (+0.98,+0.44) unfixed, (+2.92,+3.30) and (-2.25,-3.25) for
    # each alone, (-0.22,+0.00) for both).
    unfixed = dict(LATTICE_FIX=0, GRID_SYM=0, KF_DEGEN_GUARD=0)
    fixed = dict(LATTICE_FIX=1, GRID_SYM=1, KF_DEGEN_GUARD=1)
    A, B = paired(unfixed, fixed, a.seeds, a.clips, names, ('unfixed', 'fixed'))
    report(A, B, names, ('unfixed', 'fixed'))


if __name__ == '__main__':
    main()
