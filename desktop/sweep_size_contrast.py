#!/usr/bin/env python3
"""
Isolate WHAT breaks the learned tracker: target size, or target contrast?

The 10-clip battery left this ambiguous. `c_lowcontrast` (14 px target) was the
learned tracker's worst result at SNR 1.80, while `z_below_floor` succeeded at
SNR 1.09 — an ordering that looks physically backwards and which I first
attributed to out-of-distribution input. It is not. These two sweeps hold
everything constant but one variable and show the real cause.

Result 1 — SIZE IS NOT THE LIMIT.
At fixed contrast 60, the learned tracker holds 100% on-target from 32 px down
to 6 px, and at 6 px it BEATS the classical tracker (100% vs 87%). Small
bounding boxes are not the problem; the fixed-size template crop is upsampled
and the network copes.

Result 2 — CONTRAST IS, and the cliff is sharp.
At a fixed 14 px target, sweeping contrast:

    contrast   SNR    classical    learned  (learned reports)
        18    1.57        91%         28%        14%
        22    1.72        96%         53%        43%
        26    1.83        95%         58%        48%
        34    1.99       100%        100%        91%
        45    2.10       100%        100%        99%
        60    2.19       100%        100%       100%

The learned tracker falls off a cliff between contrast 26 and 34 (SNR 1.83 ->
1.99). The classical tracker holds 91-100% across the whole range. So the pixel
tracker's contrast floor is LOWER than the network's — the opposite of the usual
assumption, and the reason it stays in the design.

This also resolves the apparent contradiction with z_below_floor. SNR as
measured conflates two different physical causes:
  * low SNR from MOTION BLUR      -> learned tracker wins (trained for it)
  * low SNR from LOW CONTRAST     -> classical tracker wins
z_below_floor is blur-limited (blur 6.0, contrast 42); c_lowcontrast is
contrast-limited. Same SNR number, opposite outcome, no mystery.

Operationally that maps to: a hard-manoeuvring target in good light is the
network's case; a dim target at long range against low-contrast background is
the pixel tracker's. Both occur in one engagement.

The confidence gate handles the split without being told any of this — it scores
91/96/95% in the contrast-limited rows, i.e. exactly the classical tracker's
numbers, because the network honestly reports low confidence there (14/43/48%).

  python3 sweep_size_contrast.py --model /path/to/vittrack.onnx
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

import eval_tracker as et
import eval_vs_learned as ev
import simtrack as st
import synth_footage as sf

HERE = os.path.dirname(os.path.abspath(__file__))


def base_spec(rad, contrast):
    """Everything held constant but rad/contrast. Motion is deliberately MILD so
    the learned tracker's fast-motion advantage cannot masquerade as a size or
    contrast effect."""
    return dict(frames=80, pan=(0.5, 0.0), shake=1.2, blur=3.0, noise=7.0, agc=0.08,
                interlace=True,
                rad=lambda t, r=rad: r,
                contrast=lambda t, c=contrast: c,
                path=lambda i, t: (70 + i * 2.2, 120.0))


def score(nm, spec, outdir, cues, mk):
    sf.write_clip(nm, spec, outdir, 1234)
    fr = et.read_video(os.path.join(outdir, f"{nm}.mp4"))
    gt, dz = et.read_labels(os.path.join(outdir, f"{nm}.csv"), len(fr))
    return (ev._snr(fr, gt),
            ev.run_classical(fr, gt, dz, cues, 25.0),
            ev.run_learned(fr, gt, dz, 25.0, mk),
            ev.run_conf_gated(fr, gt, dz, cues, 25.0, mk))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    a = ap.parse_args()

    def mk():
        p = cv2.TrackerVit_Params(); p.net = a.model
        return cv2.TrackerVit_create(p)

    cues = st.CUESETS.get('FUSE3', ['edge', 'chroma', 'none'])
    d1 = os.path.join(HERE, 'clips_size'); os.makedirs(d1, exist_ok=True)
    d2 = os.path.join(HERE, 'clips_contrast'); os.makedirs(d2, exist_ok=True)

    print("\nSIZE sweep (contrast fixed at 60)")
    print(f"{'target px':>10}{'SNR':>7} | {'classical':>10}{'learned':>9}{'gated':>7}")
    print('-' * 46)
    for rad in (3, 4, 5, 6, 8, 11, 16):
        snr, c, v, g = score(f"size_{2*rad:02d}", base_spec(rad, 60.0), d1, cues, mk)
        print(f"{2*rad:>10}{snr:>7.2f} | {c.get('on_pct',0):>9.0f}%"
              f"{v.get('on_pct',0):>8.0f}%{g.get('on_pct',0):>6.0f}%")

    print("\nCONTRAST sweep (target fixed at 14 px)")
    print(f"{'contrast':>10}{'SNR':>7} | {'classical':>10}{'learned':>9}{'reports':>9}{'gated':>7}")
    print('-' * 55)
    for con in (18, 22, 26, 34, 45, 60):
        snr, c, v, g = score(f"con_{con}", base_spec(7, float(con)), d2, cues, mk)
        print(f"{con:>10}{snr:>7.2f} | {c.get('on_pct',0):>9.0f}%"
              f"{v.get('on_pct',0):>8.0f}%{v.get('rep_pct',0):>8.0f}%{g.get('on_pct',0):>6.0f}%")

    # Cost is independent of the box: the network resamples its crops to a fixed
    # input, and the classical tracker resamples to a fixed CROP=128 as well. So
    # frame rate does not depend on how big the target is on screen.
    print("\nper-frame cost vs box size")
    f = (np.random.rand(240, 400, 3) * 255).astype(np.uint8)
    for box in (8, 16, 32, 64, 128):
        t = mk(); t.init(f, (150, 100, box, box))
        for _ in range(5):
            t.update(f)
        n = 40; s = time.perf_counter()
        for _ in range(n):
            t.update(f)
        print(f"   box {box:>4} px -> {1000*(time.perf_counter()-s)/n:5.2f} ms")


if __name__ == '__main__':
    main()
