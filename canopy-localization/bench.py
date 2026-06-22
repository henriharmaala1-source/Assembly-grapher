"""Benchmark: how fast can we generate horizon curves (with coordinates)?

One ray-march per position yields ALL altitude curves at once (the numerator
trick), so 'curves' = positions x n_altitudes. We time the per-position loop and
a vectorized multi-position batch to show the CPU ceiling, then project to millions.
"""
import time
import numpy as np
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm

dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])
ALTS = [float(np.nanmedian(dsm) + a) for a in (5, 10, 15, 20)]   # 4 curves/position


def time_loop(n_az, max_range, dr, P=300):
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    xs = np.random.default_rng(0).uniform(700, 2300, (P, 2))
    t0 = time.time()
    for x, y in xs:
        rc.raycast(x, y, ALTS, az, max_range_m=max_range, dr_m=dr)
    dt = time.time() - t0
    return dt / P                                   # seconds per position


def time_batch(n_az, max_range, dr, P=256, chunk=16):
    """Vectorized over positions in chunks (CPU ceiling)."""
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    cos, sin = np.cos(az), np.sin(az)
    ranges = np.arange(4.0, max_range + dr, dr, dtype=np.float32)
    zs = np.array(ALTS, np.float32)
    pts = np.random.default_rng(1).uniform(700, 2300, (P, 2)).astype(np.float32)
    curve = r2 = ranges[None, :, None] ** 2 / (2 * rc.R_eff)
    t0 = time.time()
    for c0 in range(0, P, chunk):
        cx = pts[c0:c0 + chunk, 0][:, None, None]   # (p,1,1)
        cy = pts[c0:c0 + chunk, 1][:, None, None]
        X = cx + ranges[None, :, None] * cos[None, None, :]   # (p,R,N)
        Y = cy + ranges[None, :, None] * sin[None, None, :]
        h = rc.sample(X.ravel(), Y.ravel()).reshape(X.shape)  # (p,R,N)
        h_eff = h - r2                                          # broadcast (p,R,N)
        num = h_eff[None] - zs[:, None, None, None]            # (A,p,R,N)
        elev = np.arctan2(num, ranges[None, None, :, None])
        np.nanmax(np.where(np.isnan(elev), -np.pi / 2, elev), axis=2)  # (A,p,N)
    dt = time.time() - t0
    return dt / P


def report(label, spp):
    cps = 4 / spp                                   # curves/sec (4 alts/position)
    print(f"  {label:32s} {spp*1000:6.2f} ms/position  "
          f"{cps:7.0f} curves/s/core")
    for n, tag in ((1e6, "1M"), (10e6, "10M"), (27e6, "27M (100km disk)")):
        h1 = n / cps / 3600
        print(f"       {tag:18s}: {h1:6.1f} h (1 core)   "
              f"{h1/8:5.1f} h (8 cores)")


print("QUALITY  (n_az=720, range=1200 m, dr=2 m):")
report("loop", time_loop(720, 1200, 2))
report("vectorized batch", time_batch(720, 1200, 2))
print("\nFAST     (n_az=360, range=900 m, dr=4 m):")
report("loop", time_loop(360, 900, 4))
report("vectorized batch", time_batch(360, 900, 4))
