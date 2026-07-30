#!/usr/bin/env python3
"""
Does the coasting search have to open to the WHOLE crop?

Instrumenting the tracker over 307 real frames showed it evaluates 3641 NCC
search positions per frame -- against the ~225 the base configuration implies.
The gap is almost entirely coasting frames: `wideOpen` sets the search half-width
to `maxhalf`, so the searched AREA quadruples at exactly the moment the tracker
is already working hardest, and since most frames on the hard clips are coasting,
that case sets the average cost of the whole tracker.

At 3641 positions x 784 template pixels x 2 passes, this one decision accounts
for 5.71M of the ~10M inner-loop pixel visits per frame -- the single largest
line item in the budget, larger than the full-frame optical flow pass.

That does not make it wrong. A coasting tracker has lost the target and a wide
search is how it gets it back; the previous FOV work in this file measured that
widening EARLY hurts but widening after FOV_DELAY helps. This script asks the
narrower question the earlier work did not: given that it widens, does it have to
widen ALL THE WAY, or does most of the recovery come from the first part of the
opening?

Reports lock quality and search cost together, because either alone is
meaningless: a cheaper search that drops the target is not an optimization.

ANSWERED: it has to widen all the way. The full opening is earned.

    cap of maxhalf          1.00    0.85    0.70    0.55    0.40
    d_pan_shake              69%     71%     71%     74%    100%
    f_maneuver               68%      8%      7%      7%     10%
    g_occlusion              90%     50%     30%     58%     82%
    MEAN lock                77%     66%     62%     68%     72%
    NCC positions/frame     3235    2896    2540    2111    1965
    cost vs full            100%     90%     79%     65%     61%

Trimming 15% off the width costs 11 points of mean lock and collapses
f_maneuver from 68% to 8%. Every cap loses. The largest single line item in the
tracker's budget -- 5.71M of ~10M inner-loop pixel visits per frame -- is not
recoverable by narrowing it, and a graded ramp is if anything worse, because the
damage is concentrated in the FIRST coasting frames where a ramp is narrowest.

Note the shape of the failure as well as its direction: 1.00 -> 0.85 is a 15%
change in width that moves f_maneuver by 60 points, and 0.40 scores better than
0.70. That is not a smooth cost/quality tradeoff being sampled, it is the same
trajectory-divergence sensitivity seen in the LK point-count sweep -- a small
change decides which frame re-locks and everything after diverges. So the right
reading is not "0.40 is the second-best cap"; it is "this parameter cannot be
tuned on a battery this size, and the default is the only setting with evidence
behind it."

    python3 eval_searchcap.py
"""
import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAPS = [1.0, 0.85, 0.7, 0.55, 0.4]


def run_config(job):
    cap, clipdir, names = job
    sys.path.insert(0, HERE)
    import eval_tracker as et
    import simtrack as st
    st.COAST_HALF = cap
    # Count search positions actually evaluated, so the saving is measured in the
    # same units the cost analysis used rather than inferred from the setting.
    counter = {'pos': 0, 'frames': 0}
    _ncc = st.ncc_map

    def ncc(chan, tmpl, tn, g0, g1, stride=st.STRIDE):
        r, extra = _ncc(chan, tmpl, tn, g0, g1, stride)
        counter['pos'] += r.size
        return r, extra
    st.ncc_map = ncc

    cues = st.CUESETS['FUSE3']
    out = []
    for nm in names:
        frames = et.read_video(os.path.join(clipdir, nm + '.mp4'))
        gt, dz = et.read_labels(os.path.join(clipdir, nm + '.csv'), len(frames))
        yuv = [et.bgr_to_yuvdict(x) for x in frames]
        counter['frames'] += len(frames) - 1
        out.append(et.run(yuv, gt, dz, cues, 25.0).get('on_pct', 0.0))
    return cap, out, counter['pos'] / max(1, counter['frames'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    a = ap.parse_args()
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))
    jobs = [(c, a.clips, names) for c in CAPS]
    with mp.Pool(min(len(jobs), max(1, os.cpu_count() or 2))) as p:
        got = p.map(run_config, jobs)
    res = {c: v for c, v, _ in got}
    pos = {c: n for c, _, n in got}

    print(f"{'clip':<21}" + "".join(f"{c:>8.2f}" for c in CAPS))
    for i, nm in enumerate(names):
        print(f"{nm:<21}" + "".join(f"{res[c][i]:>7.0f}%" for c in CAPS))
    print('-' * (21 + 8 * len(CAPS)))
    print(f"{'MEAN lock':<21}" + "".join(f"{np.mean(res[c]):>7.0f}%" for c in CAPS))
    print(f"{'NCC positions/frame':<21}" + "".join(f"{pos[c]:>8.0f}" for c in CAPS))
    base = pos[CAPS[0]]
    print(f"{'cost vs full':<21}" + "".join(f"{100*pos[c]/base:>7.0f}%" for c in CAPS))


if __name__ == '__main__':
    main()
