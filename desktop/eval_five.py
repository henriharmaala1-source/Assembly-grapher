#!/usr/bin/env python3
"""
Five-way battery: is there a READY-MADE tracker already better than ours?

Answer, measured: yes — OpenCV's CSRT (CSR-DCF), and it is not close.

    clip                  |   NCC   KCF  CSRT   VIT  GATED
    a_baseline            |   100%  100%  100%  100%   100%
    b_analog              |   100%   99%  100%  100%   100%
    c_lowcontrast         |   100%   12%  100%   54%   100%
    d_pan_shake           |    71%   51%  100%  100%   100%
    e_recede              |   100%   50%  100%  100%   100%
    f_maneuver            |    19%    6%  100%  100%   100%
    g_occlusion           |    79%   39%   31%   39%    79%
    h_clutter_distractor  |    99%   25%  100%   92%    99%
    i_worst               |    18%    9%   64%   74%    73%
    z_below_floor         |    11%    8%   80%   75%    75%
    MEAN                  |    70%   40%   88%   83%    93%
    ms/frame              |  57.4   8.2  138.6   5.0

Reading it:

  * CSRT beats this project's tracker on EIGHT of ten clips and beats the
    neural network on the mean. It needs no training, no model file and no
    NPU. f_maneuver 19% -> 100% is the same failure the learned tracker fixed,
    fixed a second way.
  * KCF at 40% shows the family is not automatically good — the difference is
    that CSRT learns a discriminative filter WITH channel and spatial
    reliability, while KCF is a plain kernelised filter.
  * The one place this project's tracker is the best of all five is
    g_occlusion: 79%, against 31% for CSRT and 39% for the network. The
    occlusion hysteresis, coasting and velocity clamping built here are doing
    something none of the off-the-shelf trackers do.
  * z_below_floor: CSRT 80% where this tracker gets 11%. The "detectability
    cliff" documented in eval_battery.py as expected physics is not physics
    and is not specific to pixel matching either — it is specific to THIS
    matcher.
  * Cost is the counterweight: CSRT is 28x the network's per-frame time here.

So the honest split is matcher vs machinery. The matcher (zero-mean NCC over
luma/edge/chroma) is the weakest part and a solved problem elsewhere. The
machinery around it — occlusion handling, coasting, ego-motion feed-forward,
latency-compensated aim — is the part worth keeping, and it is what the
off-the-shelf trackers lack.

Needs opencv-contrib-python for CSRT/KCF (the plain opencv-python wheel does
not ship the tracking module):

    pip install --target ./ocv opencv-contrib-python==4.12.0.88
    PYTHONPATH=./ocv python3 eval_five.py --model vittrack.onnx
"""
import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import eval_tracker as et      # noqa: E402
import eval_vs_learned as ev   # noqa: E402
import simtrack as st          # noqa: E402

KEYS = ('NCC', 'KCF', 'CSRT', 'VIT', 'GATED')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--clips', default=os.path.join(HERE, 'clips'))
    a = ap.parse_args()
    for n in ('TrackerCSRT_create', 'TrackerKCF_create'):
        if not hasattr(cv2, n):
            sys.exit("need opencv-contrib-python for CSRT/KCF — see the module docstring")

    def vit():
        p = cv2.TrackerVit_Params(); p.net = a.model
        return cv2.TrackerVit_create(p)

    cues = st.CUESETS.get('FUSE3', ['edge', 'chroma', 'none'])
    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith('.mp4'))
    print(f"\n{'clip':<21} | " + "".join(f"{k:>6}" for k in KEYS))
    print('-' * 60)
    acc = {k: [] for k in KEYS}
    ms = {k: [] for k in KEYS[:4]}
    for nm in names:
        fr = et.read_video(os.path.join(a.clips, f'{nm}.mp4'))
        gt, dz = et.read_labels(os.path.join(a.clips, f'{nm}.csv'), len(fr))
        r = {
            'NCC': ev.run_classical(fr, gt, dz, cues, 25.0),
            'KCF': ev.run_learned(fr, gt, dz, 25.0, cv2.TrackerKCF_create),
            'CSRT': ev.run_learned(fr, gt, dz, 25.0, cv2.TrackerCSRT_create),
            'VIT': ev.run_learned(fr, gt, dz, 25.0, vit),
            'GATED': ev.run_conf_gated(fr, gt, dz, cues, 25.0, vit),
        }
        for k in KEYS:
            acc[k].append(r[k].get('on_pct', 0))
        for k in KEYS[:4]:
            ms[k].append(r[k]['ms'])
        print(f"{nm:<21} | " + "".join(f"{r[k].get('on_pct',0):>5.0f}%" for k in KEYS), flush=True)
    print('-' * 60)
    print(f"{'MEAN':<21} | " + "".join(f"{np.mean(acc[k]):>5.0f}%" for k in KEYS))
    print(f"{'ms/frame':<21} | " + "".join(f"{np.mean(ms[k]):>6.1f}" for k in KEYS[:4]))


if __name__ == '__main__':
    main()
