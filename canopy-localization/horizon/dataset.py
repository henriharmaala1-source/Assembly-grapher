"""Mass-generate auto-labeled horizon curves from the DSM.

Every panorama is tagged with the exact (x, y, z) it was cast from -> "horizon
pictures with coordinates" for free, no flying, no labeling. The panorama trick
(all headings are slices of one 360 deg cast) and multi-altitude-per-march keep
it cheap. Output is a 1-D curve per sample (~1 KB quantized).
"""
from __future__ import annotations
import time
import numpy as np


def generate_grid(rc, bounds_m, spacing_m, view_heights, n_az=720,
                  max_range_m=1500.0, quantize=True, progress=None):
    """Cast panoramas on a regular grid. Returns (curves, coords, stats).

    bounds_m     : (xmin, xmax, ymin, ymax)
    view_heights : list of absolute z to emit per position
    curves       : (N, n_az) int16 (deci-degrees) if quantize else float32 rad
    coords       : (N, 4) -> x, y, z, (alt index)
    """
    xmin, xmax, ymin, ymax = bounds_m
    xs = np.arange(xmin, xmax, spacing_m)
    ys = np.arange(ymin, ymax, spacing_m)
    zs = np.atleast_1d(view_heights)
    pano_az = np.linspace(0.0, 2 * np.pi, n_az, endpoint=False)

    curves, coords = [], []
    t0 = time.time()
    total = xs.size * ys.size
    done = 0
    for y in ys:
        for x in xs:
            h = rc.raycast(x, y, zs, pano_az, max_range_m=max_range_m)  # (A,n_az)
            for ai, z in enumerate(zs):
                curves.append(h[ai]); coords.append((x, y, float(z), ai))
            done += 1
            if progress and done % progress == 0:
                print(f"  {done}/{total} positions  "
                      f"({done / (time.time() - t0):.0f} pos/s)")
    dt = time.time() - t0

    curves = np.array(curves, dtype=np.float32)
    if quantize:
        curves = np.round(np.rad2deg(curves) * 10).astype(np.int16)  # deci-deg
    coords = np.array(coords, dtype=np.float32)

    bytes_per = curves.shape[1] * (2 if quantize else 4)
    stats = dict(n_positions=total, n_curves=len(coords),
                 seconds=dt, pos_per_s=total / dt,
                 bytes_per_curve=bytes_per)
    return curves, coords, stats
