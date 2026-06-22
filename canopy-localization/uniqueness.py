"""Is one horizon curve unique enough to localize over a wide area?

Builds a grid of reference panoramas over the real Lapland DEM (~40x33 km),
then for many query positions measures how many OTHER cells match the query
about as well (the 'confusion set'). Compares:
  - single 90 deg look (known heading from IMU)
  - single 360 deg panorama
  - a SEQUENCE of 3 looks along a short path
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rasterio
from rasterio.warp import calculate_default_transform, reproject, Resampling
from rasterio.windows import from_bounds
from rasterio.transform import array_bounds
from horizon.raycaster import HorizonRaycaster

AGL, MAXR, DR, NAZ = 40.0, 20000.0, 60.0, 360
FOV_DEG, GRID_M, NOISE_DEG = 65.0, 500.0, 0.4
rng = np.random.default_rng(0)

# --- real DEM -> metric grid ---
src = rasterio.open("cop_n67.tif")
win = from_bounds(24.00, 67.45, 24.75, 67.80, src.transform)
data = src.read(1, window=win).astype(np.float32)
wt = src.window_transform(win)
b = array_bounds(data.shape[0], data.shape[1], wt)
dt, w, h = calculate_default_transform(src.crs, "EPSG:32635", data.shape[1],
                                       data.shape[0], *b, resolution=30)
dsm = np.empty((h, w), np.float32)
reproject(data, dsm, src_transform=wt, src_crs=src.crs, dst_transform=dt,
          dst_crs="EPSG:32635", resampling=Resampling.bilinear,
          src_nodata=src.nodata, dst_nodata=np.nan)
res = abs(dt.a)
rc = HorizonRaycaster(dsm, res)
H, Wd = dsm.shape
print(f"area {Wd*res/1000:.0f} x {H*res/1000:.0f} km @ {res:.0f} m")

# --- grid of reference panoramas ---
az = np.linspace(0, 2*np.pi, NAZ, endpoint=False)
step = int(round(GRID_M / res))
ys = np.arange(step, H - step, step)
xs = np.arange(step, Wd - step, step)
ny, nx = ys.size, xs.size
panos = np.full((ny, nx, NAZ), np.nan, np.float32)
for iy, r in enumerate(ys):
    for ix, c in enumerate(xs):
        z = dsm[r, c]
        if np.isfinite(z):
            gx, gy = c * res, (H - 1 - r) * res
            panos[iy, ix] = rc.raycast(gx, gy, [float(z) + AGL], az,
                                       max_range_m=MAXR, dr_m=DR)[0]
valid = np.isfinite(panos).all(2)
print(f"grid {ny}x{nx} = {ny*nx} cells ({valid.sum()} valid), {GRID_M:.0f} m spacing")

# zero-mean panoramas (pitch-invariant); saliency = std
pz = panos - np.nanmean(panos, axis=2, keepdims=True)
sal = np.nanstd(panos, axis=2)
fov_n = int(round(FOV_DEG / 360 * NAZ))
noise = np.deg2rad(NOISE_DEG)


def resid_map(qwin, idx, wq):
    """weighted SSD of query window vs every cell's same-azimuth window."""
    refw = pz[:, :, idx]                       # (ny,nx,L)
    refw = refw - refw.mean(2, keepdims=True)
    q = qwin - qwin.mean()
    d = (refw - q) ** 2
    return np.tensordot(d, wq, axes=([2], [0]))  # (ny,nx)


def confusion(rmap, ti, tj):
    rt = rmap[ti, tj]
    thr = rt + noise ** 2                       # indistinguishable given noise
    mask = valid & (rmap <= thr)
    n = int(mask.sum())
    yy, xx = np.where(mask)
    if n > 1:
        d = np.hypot((yy - ti) * step * res, (xx - tj) * step * res).max() / 1000
    else:
        d = 0.0
    rank = int((valid & (rmap < rt)).sum()) + 1
    return rank, n, d


# --- queries on salient cells (where the method is meant to work) ---
cand = np.argwhere(valid & (sal > np.nanpercentile(sal[valid], 50)))
sel = cand[rng.choice(len(cand), size=40, replace=False)]
results = {"90": [], "360": [], "seq3": []}
hi = NAZ // 4                                   # heading index (+x); aligned to grid cols
fov_idx = (np.arange(-fov_n // 2, fov_n // 2) + hi) % NAZ
k = 2                                           # sequence step = 2 cells (~1 km)

for (ti, tj) in sel:
    # single 90 deg
    q = pz[ti, tj, fov_idx] + rng.normal(0, noise, fov_n)
    wq = np.abs(np.gradient(q)); wq /= wq.sum() + 1e-9
    results["90"].append(confusion(resid_map(q, fov_idx, wq), ti, tj))
    # single 360 deg
    q3 = pz[ti, tj] + rng.normal(0, noise, NAZ)
    w3 = np.abs(np.gradient(q3)); w3 /= w3.sum() + 1e-9
    results["360"].append(confusion(resid_map(q3, np.arange(NAZ), w3), ti, tj))
    # sequence of 3 along +x (cols): hypothesis (i,j) predicts cells j,j+k,j+2k
    if tj + 2 * k < nx:
        r0 = resid_map(q, fov_idx, wq)
        q1 = pz[ti, tj + k, fov_idx] + rng.normal(0, noise, fov_n)
        q2 = pz[ti, tj + 2 * k, fov_idx] + rng.normal(0, noise, fov_n)
        w1 = np.abs(np.gradient(q1)); w1 /= w1.sum() + 1e-9
        w2 = np.abs(np.gradient(q2)); w2 /= w2.sum() + 1e-9
        r1 = resid_map(q1, fov_idx, w1); r2 = resid_map(q2, fov_idx, w2)
        seq = np.full_like(r0, np.inf)
        seq[:, :nx - 2 * k] = r0[:, :nx - 2 * k] + r1[:, k:nx - k] + r2[:, 2 * k:]
        results["seq3"].append(confusion(seq, ti, tj))

print(f"\nUniqueness over {Wd*res/1000:.0f}x{H*res/1000:.0f} km, "
      f"{NOISE_DEG} deg curve noise, heading known:")
print(f"{'mode':>6} | {'%true=rank1':>11} | {'median confusion cells':>22} | {'median spread km':>16}")
for m in ("90", "360", "seq3"):
    R = np.array(results[m])
    rank1 = 100 * np.mean(R[:, 0] == 1)
    print(f"{m:>6} | {rank1:10.0f}% | {np.median(R[:,1]):22.0f} | {np.median(R[:,2]):16.1f}")

# figure: one query's residual map, single vs sequence
ti, tj = sel[int(np.argmax([r[1] for r in results['90']]))]   # an ambiguous one
q = pz[ti, tj, fov_idx] + rng.normal(0, noise, fov_n)
wq = np.abs(np.gradient(q)); wq /= wq.sum() + 1e-9
r0 = resid_map(q, fov_idx, wq)
q1 = pz[ti, tj + k, fov_idx]; q2 = pz[ti, tj + 2 * k, fov_idx]
w1 = np.abs(np.gradient(q1)); w1 /= w1.sum()+1e-9
w2 = np.abs(np.gradient(q2)); w2 /= w2.sum()+1e-9
seq = np.full_like(r0, np.inf)
seq[:, :nx-2*k] = r0[:, :nx-2*k] + resid_map(q1, fov_idx, w1)[:, k:nx-k] + resid_map(q2, fov_idx, w2)[:, 2*k:]
fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
ax[0].imshow(sal, cmap="viridis"); ax[0].set_title("skyline saliency (std, deg-ish)")
for a, R, t in ((ax[1], r0, "single 90° look — likelihood"),
                (ax[2], seq, "sequence of 3 looks — likelihood")):
    a.imshow(np.exp(-(R - np.nanmin(R)) / (3 * noise ** 2)), cmap="magma")
    a.plot(tj, ti, "c+", ms=14, mew=3); a.set_title(t)
fig.suptitle("Single curve vs sequence — global uniqueness (real Lapland DEM)")
fig.tight_layout(); fig.savefig("out/uniqueness.png", dpi=120)
print("saved out/uniqueness.png")
