"""Monte-Carlo accuracy + confidence calibration (no real data, no training).

Many random poses across the tile. Each: render -> classical segment ->
localize under a wrong GPS prior + noisy IMU heading. We report the error
distribution and, crucially, whether CONFIDENCE predicts error (so the system
can abstain where the skyline is uninformative).
"""
import numpy as np
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera, matcher

HFOV, IMG_W, IMG_H, MAXR = 90.0, 640, 480, 1200.0
dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])
cols = np.arange(IMG_W)
DAZ = np.arctan((cols - IMG_W / 2) / camera.focal_px(IMG_W, HFOV))


def fix_for(tx, ty, heading, seed, exact=False, prior_err=70.0, head_err_deg=6.0):
    z0 = float(rc.sample(tx, ty)) + 15.0
    img, elev_true, _ = camera.render_camera_frame(
        rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, False, seed, MAXR)
    q = elev_true if exact else camera.extract_horizon(img, hfov_deg=HFOV)
    rng = np.random.default_rng(seed + 1)
    ang = rng.uniform(0, 2 * np.pi)
    prior = (tx + prior_err * np.cos(ang), ty + prior_err * np.sin(ang))
    hp = heading + np.deg2rad(rng.uniform(-head_err_deg, head_err_deg))
    r = matcher.localize(rc, q, DAZ, z0, prior, heading_prior_rad=hp,
                         heading_search_deg=8.0, search_radius_m=96, step_m=14,
                         max_range_m=MAXR)
    err = np.hypot(r["est_xy"][0] - tx, r["est_xy"][1] - ty)
    return err, r


# sanity: exact curve must give ~0
e, _ = fix_for(1000, 1500, np.pi / 2, 1, exact=True)
print(f"[sanity] exact curve, lake head-on: error {e:.1f} m (expect ~grid step)")

# monte carlo
rng = np.random.default_rng(7)
N = 45
err = np.zeros(N); conf = np.zeros(N); sal = np.zeros(N); resid = np.zeros(N)
for i in range(N):
    tx = rng.uniform(700, 2300); ty = rng.uniform(700, 2300)
    heading = rng.uniform(0, 2 * np.pi)
    e, r = fix_for(tx, ty, heading, 100 + i)
    err[i] = e; conf[i] = r["confidence"]; sal[i] = r["saliency"]; resid[i] = r["resid_best"]

print(f"\nMonte Carlo: {N} random poses, GPS prior ~70 m off, IMU heading ~6 deg off")
print(f"  ALL poses        : median {np.median(err):5.1f} m   p90 {np.percentile(err,90):5.1f} m")

# which signal predicts error? (Spearman-ish via rank correlation)
def rankcorr(a, b):
    ra = np.argsort(np.argsort(a)); rb = np.argsort(np.argsort(b))
    return float(np.corrcoef(ra, rb)[0, 1])
print(f"\n  predictor vs error (rank corr; want NEGATIVE for confidence):")
print(f"    saliency (skyline structure) : {rankcorr(sal, err):+.2f}")
print(f"    resid_best (fit quality)     : {rankcorr(resid, err):+.2f}")

print(f"\n  NEW confidence (from saliency) calibration:")
for thr in (0.2, 0.4, 0.6):
    m = conf >= thr
    if m.sum():
        print(f"    conf>={thr:.1f}: {m.sum():2d}/{N} poses, median {np.median(err[m]):5.1f} m"
              f"   p90 {np.percentile(err[m],90):5.1f} m")
print("    (poses below conf threshold -> system ABSTAINS, SLAM coasts)")
