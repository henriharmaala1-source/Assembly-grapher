"""Run the horizon pipeline on REAL Finnish terrain (Copernicus DEM, Lapland).

Caveat: this is a 30 m radar DEM (TanDEM-X), not 2 m MML LiDAR, so it resolves
terrain relief (fells) but NOT canopy. It validates the ray-caster + matcher on
real geography; canopy-scale needs the 2 m MML tiles (blocked by this sandbox's
network policy). Same code path as the synthetic demo.
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
from horizon import camera, matcher

HFOV, IMG_W, IMG_H = 65.0, 640, 480
MAXR, DR, AGL = 22000.0, 30.0, 40.0          # see distant fells

# --- read a fell area and reproject to metric (UTM 35N) ---
W, S, E, N = 24.00, 67.45, 24.75, 67.80      # Yllas / Aakenustunturi area
src = rasterio.open("cop_n67.tif")
win = from_bounds(W, S, E, N, src.transform)
data = src.read(1, window=win).astype(np.float32)
wt = src.window_transform(win)
b = array_bounds(data.shape[0], data.shape[1], wt)
dst_crs = "EPSG:32635"
dt, w, h = calculate_default_transform(src.crs, dst_crs, data.shape[1],
                                       data.shape[0], *b, resolution=30)
dsm = np.empty((h, w), np.float32)
reproject(data, dsm, src_transform=wt, src_crs=src.crs,
          dst_transform=dt, dst_crs=dst_crs, resampling=Resampling.bilinear,
          src_nodata=src.nodata, dst_nodata=np.nan)
res = abs(dt.a)
rc = HorizonRaycaster(dsm, res)
print(f"real DEM {dsm.shape} @ {res:.0f} m/px, "
      f"elev {np.nanmin(dsm):.0f}-{np.nanmax(dsm):.0f} m")

f = camera.focal_px(IMG_W, HFOV)
cx, cy = IMG_W / 2, IMG_H / 2
dcol = np.arctan((np.arange(IMG_W) - cx) / f)

# --- auto-pick a viewpoint with fells in view (in-frame structure) ---
rng = np.random.default_rng(0)
H, Wd = dsm.shape
best = None
for _ in range(160):
    r = rng.integers(int(0.2 * H), int(0.8 * H))
    c = rng.integers(int(0.2 * Wd), int(0.8 * Wd))
    gx, gy = c * res, (H - 1 - r) * res
    z = dsm[r, c]
    if not np.isfinite(z):
        continue
    z0 = float(z) + AGL
    for hd in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        Hc = rc.raycast(gx, gy, [z0], hd + dcol, max_range_m=MAXR, dr_m=DR)[0]
        ptp = float(np.rad2deg(np.nanmax(Hc) - np.nanmin(Hc)))
        row = cy - f * np.tan(Hc)
        if 2.0 < ptp < 8.0 and np.nanmin(row) > 40 and np.nanmax(row) < 440:
            if best is None or ptp > best[0]:
                best = (ptp, gx, gy, z0, hd)
_, tx, ty, z0, heading = best
print(f"viewpoint E={tx:.0f} N={ty:.0f} z={z0:.0f} m heading={np.rad2deg(heading):.0f} deg "
      f"(skyline ptp {best[0]:.1f} deg)")

# --- generated curve + simulated camera + segmentation + localization ---
H_lidar = rc.raycast(tx, ty, [z0], heading + dcol, max_range_m=MAXR, dr_m=DR)[0]
img, H_true, hr = camera.render_camera_frame(
    rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, False, 3, MAXR)
H_cam = camera.extract_horizon(img, hfov_deg=HFOV)
seg_rms = np.rad2deg(np.sqrt(np.nanmean((H_cam - H_lidar) ** 2)))
prior = (tx + 60.0, ty - 45.0)
loc = matcher.localize(rc, H_cam, dcol, z0, prior,
                       heading_prior_rad=heading + np.deg2rad(6),
                       heading_search_deg=8.0, search_radius_m=120, step_m=20,
                       max_range_m=MAXR, dr_m=DR)
perr = np.hypot(loc["est_xy"][0] - tx, loc["est_xy"][1] - ty)
print(f"segmentation RMS {seg_rms:.2f} deg | prior off "
      f"{np.hypot(prior[0]-tx, prior[1]-ty):.0f} m -> fix error {perr:.0f} m")

# --- figure ---
rows = np.arange(IMG_H)[:, None]


def synth_view(Hc):
    hrow = cy - f * np.tan(Hc)
    sky = rows < hrow[None, :]
    rgb = np.empty((IMG_H, IMG_W, 3))
    rgb[..., 0] = np.where(sky, 0.72, 0.40)
    rgb[..., 1] = np.where(sky, 0.84, 0.36)
    rgb[..., 2] = np.where(sky, 0.97, 0.34)
    return np.clip(rgb, 0, 1)


hs = np.hypot(*np.gradient(np.nan_to_num(dsm, nan=np.nanmin(dsm))))   # quick hillshade
fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
ax[0].imshow(hs, origin="upper", cmap="gray",
             extent=[0, Wd * res, 0, H * res], vmax=np.percentile(hs, 99))
ax[0].plot(tx, ty, "r*", ms=15)
ax[0].annotate("", xy=(tx + 6000 * np.cos(heading), ty + 6000 * np.sin(heading)),
               xytext=(tx, ty), arrowprops=dict(arrowstyle="->", color="red", lw=2))
ax[0].set_title("Real Lapland DEM (Copernicus 30 m) + viewpoint")
ax[0].set_xlabel("E (m)"); ax[0].set_ylabel("N (m)")

ax[1].imshow(synth_view(H_lidar)); ax[1].set_xticks([]); ax[1].set_yticks([])
ax[1].set_title("Horizon view generated from REAL terrain")

ax[2].imshow(camera.render_camera_frame(rc, tx, ty, z0, heading, IMG_W, IMG_H,
             HFOV, False, 3, MAXR)[0], cmap="gray", aspect="auto")
ax[2].plot(np.arange(IMG_W), cy - f * np.tan(H_lidar), "c-", lw=2, label="DEM horizon")
ax[2].plot(np.arange(IMG_W), cy - f * np.tan(H_cam), "r--", lw=1.3, label="segmented")
ax[2].legend(loc="lower left", fontsize=9); ax[2].set_xticks([]); ax[2].set_yticks([])
ax[2].set_title(f"Match on real terrain — fix error {perr:.0f} m")

fig.suptitle("Horizon localization on REAL Finnish terrain (Copernicus DEM, Lapland fells)",
             fontsize=13)
fig.tight_layout()
fig.savefig("out/real_demo.png", dpi=120)
print("saved out/real_demo.png")
