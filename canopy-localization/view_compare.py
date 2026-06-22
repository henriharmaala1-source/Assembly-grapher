"""Image-domain comparison: synthetic view vs camera feed vs overlay."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera

HFOV, IMG_W, IMG_H, MAXR, ALT = 65.0, 640, 480, 1200.0, 9.0
dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])
f = camera.focal_px(IMG_W, HFOV)
cx, cy = IMG_W / 2, IMG_H / 2
cols = np.arange(IMG_W)
dcol = np.arctan((cols - cx) / f)

# auto-select a view with VISIBLE but in-frame structure (terrain relief / notch):
# peak-to-peak between 2.5 and 7 deg, horizon stays inside the frame.
rng = np.random.default_rng(0)
best = None
for _ in range(120):
    x = rng.uniform(500, 2500); y = rng.uniform(500, 2500)
    z = float(rc.sample(x, y)) + ALT
    if not np.isfinite(z):
        continue
    for hd in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        H = rc.raycast(x, y, [z], hd + dcol, max_range_m=MAXR)[0]
        ptp = float(np.rad2deg(H.max() - H.min()))
        row = cy - f * np.tan(H)
        if 2.5 < ptp < 7.0 and row.min() > 50 and row.max() < 430:
            if best is None or ptp > best[0]:
                best = (ptp, x, y, z, hd)
_, tx, ty, z0, heading = best
print(f"selected pose x={tx:.0f} y={ty:.0f} heading={np.rad2deg(heading):.0f} deg "
      f"(skyline peak-to-peak {best[0]:.1f} deg)")

# generated curve from the LiDAR map + camera feed (+ its segmentation)
H_lidar = rc.raycast(tx, ty, [z0], heading + dcol, max_range_m=MAXR)[0]
img_gray, H_true, hr = camera.render_camera_frame(
    rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, False, 3, MAXR)
H_cam = camera.extract_horizon(img_gray, hfov_deg=HFOV)
row_lidar = cy - f * np.tan(H_lidar)
row_cam = cy - f * np.tan(H_cam)
rows = np.arange(IMG_H)[:, None]


def synth_view(H):
    hrow = cy - f * np.tan(H)
    sky = rows < hrow[None, :]
    rgb = np.empty((IMG_H, IMG_W, 3))
    rgb[..., 0] = np.where(sky, 0.72, 0.16)
    rgb[..., 1] = np.where(sky, 0.84, 0.36)
    rgb[..., 2] = np.where(sky, 0.97, 0.16)
    rng = np.random.default_rng(0)
    tex = 0.06 * rng.normal(0, 1, (IMG_H, IMG_W)) * (~sky)
    return np.clip(rgb + tex[..., None], 0, 1)


def colorize(gray, hrow):
    g = gray.astype(float) / 255
    sky = rows < hrow[None, :]
    rgb = np.empty((IMG_H, IMG_W, 3))
    rgb[..., 0] = np.where(sky, 0.78 * g, 0.20 * g + 0.04)
    rgb[..., 1] = np.where(sky, 0.86 * g, 0.62 * g + 0.04)
    rgb[..., 2] = np.where(sky, 1.00 * g, 0.20 * g + 0.04)
    return np.clip(rgb, 0, 1)


cam_rgb = colorize(img_gray, hr)

fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
ax[0].imshow(synth_view(H_lidar)); ax[0].set_title("Synthetic view — generated from LiDAR map")
ax[1].imshow(cam_rgb); ax[1].set_title("Camera feed (simulated)")
ax[2].imshow(cam_rgb)
ax[2].plot(cols, row_lidar, "c-", lw=2.0, label="LiDAR horizon (expected)")
ax[2].plot(cols, row_cam, "r--", lw=1.4, label="camera horizon (segmented)")
ax[2].set_title("Overlay — generated vs detected")
ax[2].legend(loc="lower left", fontsize=9)
for a in ax:
    a.set_xticks([]); a.set_yticks([])
fig.suptitle("Horizon match: synthetic (LiDAR) vs camera — no real data, no training",
             fontsize=13)
fig.tight_layout()
fig.savefig("out/view_compare.png", dpi=120)
print(f"horizon offset (LiDAR vs segmented): mean {np.rad2deg(np.mean(H_cam-H_lidar)):+.2f} deg, "
      f"RMS {np.rad2deg(np.sqrt(np.mean((H_cam-H_lidar)**2))):.2f} deg")
print("saved out/view_compare.png")
