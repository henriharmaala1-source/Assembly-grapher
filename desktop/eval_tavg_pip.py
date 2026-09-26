#!/usr/bin/env python3
"""
Temporal averaging of the TRACKING CROP -- a re-run of an experiment that
already failed once at the frame level, on the argument that the crop is not the
same experiment.

What failed. Averaging N ego-ALIGNED FULL FRAMES, to buy sqrt(N) SNR against
analog noise, lost decisively:

    clip                    base   frame-level averaging
    f_maneuver               19%          9%
    g_occlusion              79%         35%
    MEAN                     70%         65%

and the reason is structural, not a tuning failure: frame alignment cancels
CAMERA motion only. A target moving relative to the scene is smeared by exactly
its own displacement, so the averaging destroys the one region being matched,
and destroys it in proportion to how hard the target is manoeuvring.

Why the crop might not share the defect. The crop is resampled about the
tracker's own predicted centre every frame. A tracked target therefore sits at
the SAME crop coordinate in consecutive crops with no warping at all -- the
alignment is a side effect of the sampling, and it is alignment on the TARGET
rather than on the scene. Residual misalignment is the tracker's own prediction
error, a couple of pixels, not the target's full displacement. Noise is
temporally uncorrelated and still averages down; what smears instead is the
BACKGROUND, which for a template matcher trying to separate target from
background is a second benefit rather than a cost.

So the two experiments differ in what they blur, and the frame-level result says
nothing about this one.

RESULT: it loses too, and by almost as much.

    clip                    off     N=2     N=3     N=4
    c_lowcontrast          100%     74%     93%    100%
    d_pan_shake             71%     65%     72%     48%
    e_recede               100%     83%     93%     87%
    f_maneuver              19%      3%     14%      1%
    g_occlusion             79%     94%     42%     41%
    h_clutter_distractor    99%     94%     97%     78%
    MEAN                    70%     64%     63%     58%

The premise was right and insufficient. Crop alignment IS target alignment, but
only as accurate as the tracker's own prediction -- and the prediction is least
accurate exactly when the target is moving, which is when the averaging then
smears it. Worse, the crop MAGNIFIES the error it does have: a 36 px box is
sampled over a 79 px region into a 128 px crop, so every pixel of frame-space
prediction error becomes ~1.6 px of crop-space misalignment against a 28 px
template. The frame-level version smeared by the target's full displacement and
this one smears by ~1.6x the prediction error, which is smaller but not small.

f_maneuver, 19% -> 3%, is the same failure the frame-level version had, for the
same reason, just reached by a different route.

The one place it works is worth recording: g_occlusion 79% -> 94% at N=2. An
occluder is transient, so averaging two frames dilutes it while the target --
still roughly aligned across a short window -- survives. That is a real effect,
but N=2 costs 6 points of mean elsewhere to buy it, and the LK coast assist
already takes g_occlusion to 90% without costing anything.

Kept switchable (TAVG_PIP in simtrack.py, default off) so the result stays
reproducible.

    python3 eval_tavg_pip.py
"""
import argparse
import multiprocessing as mp
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WINDOWS = [0, 2, 3, 4]


def run_config(job):
    n, clipdir, names = job
    sys.path.insert(0, HERE)
    import eval_tracker as et
    import simtrack as st
    st.TAVG_PIP = n
    st.LK_ASSIST = 0          # isolate: measure averaging alone, not the pair
    cues = st.CUESETS['FUSE3']
    out = []
    for nm in names:
        frames = et.read_video(os.path.join(clipdir, nm + '.mp4'))
        gt, dz = et.read_labels(os.path.join(clipdir, nm + '.csv'), len(frames))
        yuv = [et.bgr_to_yuvdict(x) for x in frames]
        out.append(et.run(yuv, gt, dz, cues, 25.0).get('on_pct', 0.0))
    return n, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    a = ap.parse_args()
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))
    jobs = [(n, a.clips, names) for n in WINDOWS]
    with mp.Pool(min(len(jobs), max(1, os.cpu_count() or 2))) as p:
        res = dict(p.map(run_config, jobs))
    print(f"{'clip':<21}" + "".join(f"{('off' if n<2 else 'N=%d'%n):>8}" for n in WINDOWS))
    for i, nm in enumerate(names):
        print(f"{nm:<21}" + "".join(f"{res[n][i]:>7.0f}%" for n in WINDOWS))
    print('-' * (21 + 8 * len(WINDOWS)))
    print(f"{'MEAN':<21}" + "".join(f"{np.mean(res[n]):>7.0f}%" for n in WINDOWS))


if __name__ == '__main__':
    main()
