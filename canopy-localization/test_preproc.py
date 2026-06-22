"""Show that preprocessing makes a corrupted (real-like) clip comparable again."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera
import video_preproc as vp

FOV = 65.0
dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])
K = np.array([[camera.focal_px(640, FOV), 0, 320],
              [0, camera.focal_px(640, FOV), 240], [0, 0, 1]], float)
dist = np.array([-0.30, 0.10, 0.0, 0.0, 0.0])          # barrel
rolls = [6, -5, 7, -4, 5, -6, 4, -7]

bad, good, roll_err = [], [], []
ex = None
for i, rl in enumerate(rolls):
    x = 900 + i * 25; y = 1450.0
    z = float(rc.sample(x, y)) + 12.0
    clean, _, _ = camera.render_camera_frame(rc, x, y, z, np.pi/2, 640, 480, FOV, False, i, 1200)
    truth = camera.extract_horizon(clean, hfov_deg=FOV)
    cor = vp.corrupt(clean, K, dist, rl)
    # naive (no preprocessing)
    b = camera.extract_horizon(cor, hfov_deg=FOV)
    bad.append(np.rad2deg(np.sqrt(np.nanmean((b - truth) ** 2))))
    # with preprocessing
    pre, fov_p, est = vp.preprocess(cor, FOV, calib=(K, dist), auto_level=True)
    g = camera.extract_horizon(pre, hfov_deg=fov_p)
    good.append(np.rad2deg(np.sqrt(np.nanmean((g - truth) ** 2))))
    roll_err.append(abs(est - rl))
    if ex is None:
        ex = (clean, cor, pre, truth, b, g, rl, est, fov_p)

print(f"camera roll recovery: mean |error| {np.mean(roll_err):.1f} deg")
print(f"skyline RMS vs truth: naive {np.mean(bad):.2f} deg  ->  "
      f"preprocessed {np.mean(good):.2f} deg")

clean, cor, pre, truth, b, g, rl, est, fov_p = ex
f = camera.focal_px(640, FOV)
fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
ax[0].imshow(cor, cmap="gray"); ax[0].set_title(f"corrupted frame (roll {rl}°, distorted, dim)")
ax[0].set_xticks([]); ax[0].set_yticks([])
ax[1].imshow(pre, cmap="gray")
ax[1].plot(np.arange(640), 240 - f*np.tan(g), "r-", lw=1.5)
ax[1].set_title(f"preprocessed (leveled est {est:.1f}°, undistorted, CLAHE)")
ax[1].set_xticks([]); ax[1].set_yticks([])
dcol = np.rad2deg(np.arctan((np.arange(640)-320)/f))
ax[2].plot(dcol, np.rad2deg(truth), "g-", label="truth (clean)")
ax[2].plot(dcol, np.rad2deg(b), "r:", label="naive on corrupted")
ax[2].plot(dcol, np.rad2deg(g), "b--", label="after preprocessing")
ax[2].set_xlabel("relative azimuth (deg)"); ax[2].set_ylabel("elevation (deg)")
ax[2].legend(); ax[2].grid(alpha=.3); ax[2].set_title("skyline curve")
fig.suptitle("Video preprocessing: making a real-like clip comparable", fontsize=13)
fig.tight_layout(); fig.savefig("out/preproc.png", dpi=120)
print("saved out/preproc.png")
