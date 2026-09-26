#!/usr/bin/env python3
"""
How large a difference can this battery actually resolve?

Every design decision in this project is made by running ten clips and comparing
one number. Two results suggest that number is noisier than it looks:

  - sweeping LK_MAX_PTS from 6 to 30 gave 72,72,69,77,74,67,74,68,75 -- jagged
    and non-monotonic, for what ought to be a smooth knob
  - capping the coasting search by 15% moved f_maneuver from 68% to 8%, and a
    60% cap scored better than a 30% cap

Both look like trajectory divergence rather than a real response: a tiny change
decides which frame re-locks, and everything downstream diverges. If that is
what is happening, then a 5-point difference in the mean may carry no
information at all, and several recent decisions rest on differences that size.

This measures it directly instead of arguing about it. Each variant below is a
SEMANTICALLY NULL perturbation -- a change no one would expect to affect
tracking quality:

  designation moved by one pixel   an operator drawing a box is not pixel-exact
  designation resized by 2%        likewise
  luma noise at sigma=0.5          below the quantisation step of 8-bit video

Ground truth is untouched, so every variant is scored against the same target.
The spread of the resulting means IS the battery's noise floor. Any measured
difference smaller than that spread is indistinguishable from having drawn the
designation box one pixel to the left.

    python3 eval_noisefloor.py
"""
import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

#        label      dx  dy  size    noise seed (0 = none)
VARIANTS = [
    ('base',     0,  0, 1.00, 0),
    ('dx -1',   -1,  0, 1.00, 0),
    ('dx +1',   +1,  0, 1.00, 0),
    ('dy -1',    0, -1, 1.00, 0),
    ('dy +1',    0, +1, 1.00, 0),
    ('size -2%', 0,  0, 0.98, 0),
    ('size +2%', 0,  0, 1.02, 0),
    ('noise a',  0,  0, 1.00, 1),
    ('noise b',  0,  0, 1.00, 2),
    ('noise c',  0,  0, 1.00, 3),
]


def run_variant(job):
    label, dx, dy, sz, seed, clipdir, names = job
    sys.path.insert(0, HERE)
    import eval_tracker as et
    import simtrack as st
    cues = st.CUESETS['FUSE3']
    out = []
    for nm in names:
        frames = et.read_video(os.path.join(clipdir, nm + '.mp4'))
        gt, dz = et.read_labels(os.path.join(clipdir, nm + '.csv'), len(frames))
        yuv = [et.bgr_to_yuvdict(x) for x in frames]
        if seed:
            # Sub-quantisation noise: smaller than the 8-bit step, so it cannot
            # carry information the tracker could legitimately use.
            rng = np.random.RandomState(seed)
            for f in yuv:
                f['y'] = f['y'] + rng.normal(0, 0.5, f['y'].shape).astype(np.float32)
        fi0, cx0, cy0, sz0 = dz
        dz2 = (fi0, cx0 + dx, cy0 + dy, sz0 * sz)
        out.append(et.run(yuv, gt, dz2, cues, 25.0).get('on_pct', 0.0))
    return label, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    a = ap.parse_args()
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))
    jobs = [(l, dx, dy, s, sd, a.clips, names) for l, dx, dy, s, sd in VARIANTS]
    with mp.Pool(min(len(jobs), max(1, os.cpu_count() or 2))) as p:
        res = dict(p.map(run_variant, jobs))

    cols = [v[0] for v in VARIANTS]
    print(f"{'clip':<21}" + "".join(f"{c:>9}" for c in cols) + f"{'spread':>9}")
    for i, nm in enumerate(names):
        vals = [res[c][i] for c in cols]
        print(f"{nm:<21}" + "".join(f"{v:>8.0f}%" for v in vals)
              + f"{max(vals)-min(vals):>8.0f}")
    means = [float(np.mean(res[c])) for c in cols]
    print('-' * (21 + 9 * (len(cols) + 1)))
    print(f"{'MEAN':<21}" + "".join(f"{m:>8.0f}%" for m in means)
          + f"{max(means)-min(means):>8.1f}")
    print()
    print(f"  mean of means      {np.mean(means):.1f}%")
    print(f"  std of means       {np.std(means, ddof=1):.2f} points")
    print(f"  full range         {max(means)-min(means):.1f} points")
    print(f"  worst single clip  {max(max(res[c][i] for c in cols) - min(res[c][i] for c in cols) for i in range(len(names))):.0f} points")
    print()
    print("  Every column above is the SAME tracker on the SAME footage. Any")
    print("  measured effect smaller than the full range is not resolvable here.")


if __name__ == '__main__':
    main()
