#!/usr/bin/env python3
"""
Turn footage you HAVE into footage at the range you cannot get to.

The problem this solves: the engagement envelope is 50-800 m, so the tracker has
to hold targets a handful of pixels across -- and long-range clips are the
hardest kind to find. The synthetic battery covers small targets, but it does
not contain the thing that actually breaks trackers on this rig: analog
interference, moire and the refresh beat of a phone filming a monitor. Those are
in the real footage and cannot be simulated convincingly.

So rather than synthesise the range, take the real clip and move the camera
back. For a fixed sensor and lens, a target at k times the range subtends 1/k
the pixels. Cropping a k-times-larger region and resampling it to the SAME
output size reproduces exactly that: the target shrinks by k, and every pixel
that remains is genuine -- real noise, real interference, real compression.

    k=1   crop 640x360, output 640x360   target 48 px
    k=2   crop 1280x720 -> 640x360       target 24 px
    k=3   crop 1920x1080 -> 640x360      target 16 px

Which is why this reads the source at NATIVE resolution rather than the
viewer's downscaled frame: the headroom between the source and the output IS the
range extension available. A 1080p phone recording gives about 3x against a
640-wide output. Beyond that the crop would exceed the frame and would have to
be padded, which invents pixels -- so it refuses instead.

The crop follows the target so it stays framed, using the tracker log if one
exists (the LOG button in track_viewer writes it), otherwise a fixed centre.

HONEST LIMITS. This reproduces the geometry of range and nothing else. Real
distance also brings atmospheric scattering, contrast loss, and a target that
falls toward the sensor noise floor rather than staying a clean downscale of a
near one. Downscaling AVERAGES noise, so the result is if anything cleaner per
pixel than the real thing at that range. Treat it as a lower bound on
difficulty, not a substitute for a genuinely distant clip.

    python3 range_extend.py --video clip.mp4 --scales 1.5 2 3
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_track(path):
    """Centre per frame from a track_viewer log, if one sits next to the clip.
    Prefers whichever tracker held the tightest median box -- that is the one
    most likely to have actually been on the target."""
    if not os.path.exists(path):
        return None
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return None
    by = {}
    for r in rows:
        by.setdefault(r['tracker'], []).append(r)
    best = min(by, key=lambda t: np.median([float(r['size']) for r in by[t]]))
    out = {}
    for r in by[best]:
        out[int(r['frame'])] = (float(r['cx']), float(r['cy']))
    print(f"  using track from '{best}' ({len(out)} frames)")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--scales', nargs='+', type=float, default=[1.5, 2.0, 3.0])
    ap.add_argument('--out-w', type=int, default=640,
                    help='output width; the source must be this * scale or wider')
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        sys.exit(f"cannot open {a.video}")
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_h = int(round(a.out_w * H / W / 2) * 2)
    print(f"source {W}x{H} @ {fps:.0f} fps, {n} frames -> output {a.out_w}x{out_h}")
    print(f"max usable scale = {min(W / a.out_w, H / out_h):.2f}x "
          f"(beyond that the crop leaves the frame)")

    track = load_track(os.path.splitext(a.video)[0] + '_trackerlog.csv')
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
    cap.release()

    for k in a.scales:
        cw, ch = int(a.out_w * k), int(out_h * k)
        if cw > W or ch > H:
            print(f"  x{k:<4} SKIPPED: needs a {cw}x{ch} crop from {W}x{H}. "
                  f"Padding would invent pixels, so it is refused.")
            continue
        dst = os.path.splitext(a.video)[0] + f'_range{k:g}x.mp4'
        vw = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*'mp4v'), fps, (a.out_w, out_h))
        for i, f in enumerate(frames):
            if track and i in track:
                # log coords are in the VIEWER's downscaled space; rescale to source
                sx = W / (a.out_w if track else W)
                cx, cy = track[i][0] * (W / a.out_w), track[i][1] * (H / out_h)
            else:
                cx, cy = W / 2.0, H / 2.0
            x0 = int(round(min(max(cx - cw / 2, 0), W - cw)))
            y0 = int(round(min(max(cy - ch / 2, 0), H - ch)))
            # INTER_AREA: proper averaging on the way down. INTER_LINEAR would
            # alias a small target into flicker, which is a defect this tool
            # would be introducing rather than measuring.
            vw.write(cv2.resize(f[y0:y0 + ch, x0:x0 + cw], (a.out_w, out_h),
                                interpolation=cv2.INTER_AREA))
        vw.release()
        print(f"  x{k:<4} target shrinks {k:g}x -> {os.path.basename(dst)}")

    print("\nOpen these in track_viewer and designate the same target in each.")
    print("The scale at which a tracker stops holding is its range limit, measured")
    print("on real imagery rather than on synthetic 1/f noise.")


if __name__ == '__main__':
    main()
