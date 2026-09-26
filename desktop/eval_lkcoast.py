#!/usr/bin/env python3
"""
Sweep the LK coast assist's design choices, because the obvious integration of
it measured WORSE than the throwaway prototype that motivated it.

The idea is simple and the prototype confirmed it is worth having: when the
appearance match fails, stop extrapolating a constant velocity and go look at
the image instead -- corners inside the last good box, tracked by Lucas-Kanade,
median displacement. It is appearance-independent, so it survives exactly the
pose change / partial occlusion / motion blur that broke the template.

    clip                    base   prototype
    f_maneuver               19%       62%
    g_occlusion              79%       91%
    e_recede                100%       82%
    MEAN                     70%       73%

Turning that into a shipped feature took three rounds, and the first two were
spent measuring the wrong thing.

ROUND 0 -- the integration was not equivalent to what it was copying. The first
port scored 65% with its gates switched off, against the prototype's 73%, and no
amount of gate tuning on top of a broken baseline means anything. Three defects,
each found by diffing the two implementations frame by frame rather than by
reasoning about them:

  apply point   the port fed flow into the PREDICTION; the prototype corrected
                the OUTPUT after the search had already decided. Any frame where
                the NCC then accepts a peak discards a prediction correction, and
                on a clip alternating LOCKED/COASTING that is most frames.
                14% vs 62% on f_maneuver, from this alone.
  flow window   15 px at 2 pyramid levels cannot track a target moving 18 px per
                frame. The prototype used OpenCV's defaults (21 px, 3 levels).
  seed at t=0   the port seeded points at designation, which sounds obviously
                right and moves the trajectory from frame 1 onward. It costs.

Fixing those reproduced the prototype exactly -- same states, same displacements,
62% = 62% -- and only then was there something to sweep against.

ROUND 1 -- every quality gate, added one at a time to the verified baseline:

    base  proto  +qual  +box   +fb  +sprd  +inner  +sub  +rfr4  +vfb  +seed0
     70%    73%    72%   73%   69%    63%     68%   75%    69%   70%     70%

All four gates lose. They were invented to fix e_recede, which the ungated
prototype dropped from 100% to 82% -- but e_recede's loss was never a missing
gate. It was the SEED SCOPE, and the fix runs the opposite way to the guess:
goodFeaturesToTrack's qualityLevel is relative to the strongest corner in the
image it is handed, so a full-frame mask keeps only corners strong against the
whole SCENE, and on a shrinking target those are the background edges clipped by
the box rather than the target itself. Flow then measures the background and
glues the box to the scene. Box-relative selection keeps the corners on the
target -- and it is also the ~60x cheaper option. The cheap fix and the correct
fix are the same fix, which is not how it usually goes.

ROUND 3 -- sweeping the surviving free parameter, LK_MAX_PTS, from 6 to 30:

    p6   p8  p10  p12  p14  p16  p20  p24  p30
    72%  72%  69%  77%  74%  67%  74%  68%  75%

Jagged, no plateau: changing the point set changes which frame re-locks and the
trajectory diverges from there. So the headline +7 is the best draw, not the
expected value -- the average over that sweep, about +2, is the honest number for
the mean. What survives the jaggedness is the SIGN per clip, and that is
consistent: f_maneuver improves at 8 of 9 point counts, g_occlusion at 7 of 9,
d_pan_shake loses 2-7 at all 9. Flow helps when the appearance model fails on a
moving or occluded target and mildly hurts when the camera shakes, which is
exactly what the mechanism predicts, and is why it ships enabled.

Each config is a full battery run in its own process, so they are independent and
a round costs one battery's wall-clock.

    python3 eval_lkcoast.py [--round 1|2|3] [--clips clips] [--only proto,+sub]
"""
import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Gate settings shared by every gated config. PROTO switches them all off.
GATED = dict(LK_QUALITY=0.05, LK_INNER=0.7, LK_MIN_BOX=28.0,
             LK_FB_MAX=1.5, LK_SPREAD=2.5, LK_REFRESH=4)
OPEN = dict(LK_QUALITY=0.01, LK_INNER=1.0, LK_MIN_BOX=0.0,
            LK_FB_MAX=1e9, LK_SPREAD=1e9, LK_REFRESH=1)

# Verified frame-for-frame identical to the throwaway prototype on f_maneuver
# (62% = 62%, same states, same displacements) before any gate was added. Every
# column below is one deviation from THIS, so a column that loses is that one
# change losing, not an unaccounted-for porting difference.
PROTO = dict(LK_ASSIST=1, LK_MODE=2, LK_VFB=0, LK_DROP=0, LK_FULLSEED=1,
             LK_SEED0=0, LK_WIN=21, LK_LEVELS=3, LK_MAX_PTS=30, LK_MIN_PTS=4,
             **OPEN)

SUB = {**PROTO, 'LK_FULLSEED': 0}       # the round-1 winner; round 2 probes it

ROUND1 = [
    ('base',   dict(LK_ASSIST=0)),
    ('proto',  dict(PROTO)),
    # quality gates, one at a time
    ('+qual',  {**PROTO, 'LK_QUALITY': 0.05}),
    ('+box',   {**PROTO, 'LK_MIN_BOX': 28.0}),
    ('+fb',    {**PROTO, 'LK_FB_MAX': 1.5}),
    ('+sprd',  {**PROTO, 'LK_SPREAD': 2.5}),
    ('+inner', {**PROTO, 'LK_INNER': 0.7}),
    # the cheap variants, one at a time: is the ~60x cheaper seed good enough?
    ('+sub',   dict(SUB)),
    ('+rfr4',  {**PROTO, 'LK_REFRESH': 4}),
    # remaining design questions
    ('+vfb',   {**PROTO, 'LK_VFB': 1}),
    ('+seed0', {**PROTO, 'LK_SEED0': 1}),
]

ROUND2 = [
    ('base',   dict(LK_ASSIST=0)),
    ('sub',    dict(SUB)),
    ('s+qual', {**SUB, 'LK_QUALITY': 0.05}),
    ('s+box',  {**SUB, 'LK_MIN_BOX': 28.0}),
    ('s+fb',   {**SUB, 'LK_FB_MAX': 1.5}),
    ('s+sprd', {**SUB, 'LK_SPREAD': 2.5}),
    ('s+innr', {**SUB, 'LK_INNER': 0.7}),
    ('s+rfr2', {**SUB, 'LK_REFRESH': 2}),
    ('s+w15',  {**SUB, 'LK_WIN': 15, 'LK_LEVELS': 2}),
    ('s+p12',  {**SUB, 'LK_MAX_PTS': 12}),
    ('s+min8', {**SUB, 'LK_MIN_PTS': 8}),
]

# Round 2 put LK_MAX_PTS=12 two points above the 30 the prototype used, which is
# exactly the kind of single-value win a 10-clip battery produces by accident.
# Sweep the neighbourhood and take a PLATEAU rather than a peak.
ROUND3 = [('base', dict(LK_ASSIST=0))] + [
    (f'p{k}', {**SUB, 'LK_MAX_PTS': k}) for k in (6, 8, 10, 12, 14, 16, 20, 24, 30)
]

ROUNDS = {'1': ROUND1, '2': ROUND2, '3': ROUND3}


def run_config(job):
    name, cfg, clipdir, names = job
    sys.path.insert(0, HERE)
    import time
    import eval_tracker as et
    import simtrack as st
    for k, v in cfg.items():
        setattr(st, k, v)
    cues = st.CUESETS['FUSE3']
    out = []
    t = 0.0
    nf = 0
    for nm in names:
        frames = et.read_video(os.path.join(clipdir, nm + '.mp4'))
        gt, dz = et.read_labels(os.path.join(clipdir, nm + '.csv'), len(frames))
        yuv = [et.bgr_to_yuvdict(x) for x in frames]
        t0 = time.perf_counter()
        out.append(et.run(yuv, gt, dz, cues, 25.0).get('on_pct', 0.0))
        t += time.perf_counter() - t0
        nf += len(frames)
    # wall-clock per frame. Only comparable WITHIN a run (every config shares the
    # same oversubscribed pool), which is all that is being asked of it.
    return name, out, 1000.0 * t / max(1, nf)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    ap.add_argument('--only', default='', help='comma-separated config names')
    ap.add_argument('--round', default='1', choices=sorted(ROUNDS))
    a = ap.parse_args()

    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))
    want = set(a.only.split(',')) if a.only else None
    cfgs = [c for c in ROUNDS[a.round] if want is None or c[0] in want]

    jobs = [(n, c, a.clips, names) for n, c in cfgs]
    with mp.Pool(min(len(jobs), max(1, (os.cpu_count() or 2)))) as p:
        got = p.map(run_config, jobs)
    res = {n: v for n, v, _ in got}
    tms = {n: t for n, _, t in got}

    cols = [c[0] for c in cfgs]
    print(f"{'clip':<21}" + "".join(f"{c:>8}" for c in cols))
    for i, nm in enumerate(names):
        print(f"{nm:<21}" + "".join(f"{res[c][i]:>7.0f}%" for c in cols))
    print('-' * (21 + 8 * len(cols)))
    print(f"{'MEAN':<21}" + "".join(f"{np.mean(res[c]):>7.0f}%" for c in cols))
    print(f"{'ms/frame':<21}" + "".join(f"{tms[c]:>8.1f}" for c in cols))


if __name__ == '__main__':
    main()
