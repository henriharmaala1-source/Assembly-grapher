"""Visualize the synthetic horizon curve and the gap to the 'camera' curve.

Panels:
  (a) the synthetic LiDAR-raycast curve (what we generate)
  (b) synthetic vs camera-segmented curve + the gap (segmentation component)
  (c) the curve at 3 nearby positions (why it localizes)
  (d) synthetic vs 'canopy changed since LiDAR' (the real-world staleness gap)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera

HFOV, IMG_W, IMG_H, MAXR = 65.0, 640, 480, 1200.0
dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])

tx, ty = 1000.0, 1500.0
z0 = float(rc.sample(tx, ty)) + 15.0
heading = np.pi / 2
hfov = np.deg2rad(HFOV)

# synthetic curve (FOV)
H_lidar, daz = rc.fov_view(tx, ty, z0, heading, hfov, n=IMG_W, max_range_m=MAXR)
dazd = np.rad2deg(daz)

# camera-segmented curve
img, H_true, _ = camera.render_camera_frame(
    rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, False, 3, MAXR)
cols = np.arange(IMG_W)
dcol = np.arctan((cols - IMG_W / 2) / camera.focal_px(IMG_W, HFOV))
H_cam = camera.extract_horizon(img, hfov_deg=HFOV)
H_cam_i = np.interp(daz, dcol, H_cam)
gap_seg = np.rad2deg(H_cam_i - H_lidar)
rms_seg = np.sqrt(np.mean(gap_seg ** 2))

# 'canopy changed since LiDAR': differential growth + a clear-cut on the
# far-shore treeline that actually forms the visible skyline
dsm2 = dsm.copy()
terr = meta["terrain"]
forest = (dsm - terr) > 3
yy, xx = np.mgrid[0:dsm.shape[0], 0:dsm.shape[1]]
xm, ym = xx * 2.0, yy * 2.0
growth = 2.0 + 1.5 * np.sin(xm / 700.0) * np.cos(ym / 650.0)   # non-uniform
dsm2[forest] += growth[forest]
# worst case: heavy thinning (-8 m) at the horizon-FORMING distance ring
rr = np.hypot(xm - tx, ym - ty)
ring = forest & (rr > 950) & (rr < 1200) & (ym > ty)          # far ring, north
dsm2[ring] -= 8.0
rc2 = HorizonRaycaster(dsm2, meta["res_m"])
H_stale, _ = rc2.fov_view(tx, ty, z0, heading, hfov, n=IMG_W, max_range_m=MAXR)
# zero-mean both: uniform growth cancels; only the real differences remain
zl = H_lidar - H_lidar.mean(); zs = H_stale - H_stale.mean()
gap_stale = np.rad2deg(zs - zl)
rms_stale = np.sqrt(np.mean(gap_stale ** 2))

fig, ax = plt.subplots(2, 2, figsize=(13, 9))

ax[0, 0].plot(dazd, np.rad2deg(H_lidar), "b-", lw=1.8)
ax[0, 0].set_title("(a) synthetic LiDAR horizon curve  H_lidar")
ax[0, 0].set_xlabel("relative azimuth (deg)"); ax[0, 0].set_ylabel("elevation (deg)")
ax[0, 0].grid(alpha=.3)
ax[0, 0].annotate("lake notch\n(see across to far shore)",
                  xy=(0, np.rad2deg(H_lidar.min())), xytext=(-25, np.rad2deg(H_lidar.min()) + 1.2),
                  arrowprops=dict(arrowstyle="->"), fontsize=9)

ax[0, 1].plot(dazd, np.rad2deg(H_lidar), "b-", lw=1.6, label="synthetic (LiDAR)")
ax[0, 1].plot(dazd, np.rad2deg(H_cam_i), "r--", lw=1.2, label="camera (segmented)")
ax[0, 1].fill_between(dazd, np.rad2deg(H_lidar), np.rad2deg(H_cam_i),
                      color="orange", alpha=.4, label=f"gap (RMS {rms_seg:.2f}°)")
ax[0, 1].set_title("(b) synthetic vs camera — segmentation gap")
ax[0, 1].set_xlabel("relative azimuth (deg)"); ax[0, 1].set_ylabel("elevation (deg)")
ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=.3)

for dy, c in ((0, "b"), (40, "g"), (80, "m")):
    h, _ = rc.fov_view(tx, ty + dy, z0, heading, hfov, n=IMG_W, max_range_m=MAXR)
    ax[1, 0].plot(dazd, np.rad2deg(h), c, lw=1.4, label=f"+{dy} m toward lake")
ax[1, 0].set_title("(c) curve vs position — what makes it localize")
ax[1, 0].set_xlabel("relative azimuth (deg)"); ax[1, 0].set_ylabel("elevation (deg)")
ax[1, 0].legend(fontsize=8); ax[1, 0].grid(alpha=.3)

ax[1, 1].plot(dazd, np.rad2deg(zl), "b-", lw=1.6, label="synthetic (zero-mean)")
ax[1, 1].plot(dazd, np.rad2deg(zs), "r--", lw=1.2, label="canopy changed (zero-mean)")
ax[1, 1].fill_between(dazd, np.rad2deg(zl), np.rad2deg(zs),
                      color="crimson", alpha=.3, label=f"gap (RMS {rms_stale:.2f}°)")
ax[1, 1].set_title("(d) staleness: growth + heavy thinning at horizon ring (worst case)")
ax[1, 1].set_xlabel("relative azimuth (deg)"); ax[1, 1].set_ylabel("elevation (deg)")
ax[1, 1].legend(fontsize=8); ax[1, 1].grid(alpha=.3)

fig.suptitle("Synthetic horizon curve and the gap to the camera feed", fontsize=13)
fig.tight_layout()
fig.savefig("out/curves.png", dpi=120)
print(f"segmentation gap RMS = {rms_seg:.3f} deg")
print(f"staleness gap RMS    = {rms_stale:.3f} deg (after zero-mean; uniform growth cancels)")
print("saved out/curves.png")
