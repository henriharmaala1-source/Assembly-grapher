"""End-to-end proof — no real-world data, no training.

  1. build a Finnish-style DSM (swap for MML GeoTIFF later)
  2. mass-generate auto-labeled horizon curves; report throughput / storage
  3. render camera frames from the DSM (clear + overcast)
  4. extract the horizon with the classical, training-free segmenter
  5. localize under a wrong GPS prior: single-frame + temporal-averaged
  6. save a 4-panel figure

Run:  python3 demo.py
"""
import os
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera, dataset, matcher

OUT = os.path.join(os.path.dirname(__file__), "out")
os.makedirs(OUT, exist_ok=True)
HFOV = 65.0          # focused FOV: notch-dominated, better conditioned
ALT = 15.0           # m above canopy
MAXR = 1200.0        # ray-cast range (m)
IMG_W, IMG_H = 640, 480


def main():
    print("=" * 66)
    print("CANOPY HORIZON LOCALIZATION — end-to-end proof (no real data)")
    print("=" * 66)

    # 1. DSM -----------------------------------------------------------------
    dsm, meta = make_synthetic_dsm(size_px=1500, res_m=2.0, seed=0)
    res = meta["res_m"]
    rc = HorizonRaycaster(dsm, res)
    print(f"\n[1] DSM {dsm.shape} @ {res} m/px "
          f"({dsm.shape[0]*res/1000:.1f} x {dsm.shape[1]*res/1000:.1f} km)")

    # 2. mass dataset generation (the 'massive amounts' answer) ---------------
    print("\n[2] Mass-generating auto-labeled horizon curves ...")
    curves, coords, st = dataset.generate_grid(
        rc, bounds_m=(900, 2100, 900, 2100), spacing_m=70.0,
        view_heights=[float(np.nanmedian(dsm) + a) for a in (5, 10, 15, 20)],
        n_az=720, max_range_m=MAXR, progress=150)
    disk = np.pi * 100 ** 2 / ((dsm.shape[0] * res / 1000) ** 2) * st["n_curves"]
    print(f"    {st['n_curves']:,} curves / {st['n_positions']:,} positions "
          f"in {st['seconds']:.0f}s ({st['pos_per_s']:.0f} pos/s, "
          f"{st['bytes_per_curve']} B/curve)")
    print(f"    -> 100 km disk @50 m,4 alts: ~{disk/1e6:.0f}M curves, "
          f"~{disk*st['bytes_per_curve']/1e9:.0f} GB, "
          f"~{disk/st['pos_per_s']/4/3600:.0f} h CPU (minutes on GPU)")

    # 3. true pose: lake head-on (a well-conditioned, high-saliency view) -----
    tx, ty = 1000.0, 1500.0
    z0 = float(rc.sample(tx, ty)) + ALT
    heading = np.pi / 2
    print(f"\n[3] True pose x={tx:.0f} y={ty:.0f} z={z0:.1f} "
          f"heading={np.rad2deg(heading):.0f} deg")

    # 4. classical, training-free segmentation (clear & overcast) ------------
    print("\n[4] Classical horizon segmentation (no training):")
    seg = {}
    for mode, oc in (("clear", False), ("overcast", True)):
        img, elev_true, hr = camera.render_camera_frame(
            rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, oc, 2, MAXR)
        elev_seg = camera.extract_horizon(img, hfov_deg=HFOV)
        rms = np.rad2deg(np.sqrt(np.mean((elev_seg - elev_true) ** 2)))
        seg[mode] = dict(img=img, elev_true=elev_true, elev_seg=elev_seg, hr=hr)
        print(f"    {mode:9s}: segmentation RMS vs truth = {rms:.3f} deg")

    # 5. localize: single-frame + temporal averaging -------------------------
    cols = np.arange(IMG_W)
    daz = np.arctan((cols - IMG_W / 2) / camera.focal_px(IMG_W, HFOV))
    prior = (tx + 58.0, ty - 44.0)
    heading_prior = heading + np.deg2rad(6.0)
    print(f"\n[5] Localizing. GPS prior off "
          f"{np.hypot(prior[0]-tx, prior[1]-ty):.0f} m, IMU heading off 6 deg")

    def one_fix(seed):
        img, _, _ = camera.render_camera_frame(
            rc, tx, ty, z0, heading, IMG_W, IMG_H, HFOV, False, seed, MAXR)
        e = camera.extract_horizon(img, hfov_deg=HFOV)
        return matcher.localize(rc, e, daz, z0, prior,
                                heading_prior_rad=heading_prior,
                                heading_search_deg=8.0,
                                search_radius_m=96, step_m=12, max_range_m=MAXR)

    N = 6
    t0 = time.time()
    fixes = [one_fix(s) for s in range(10, 10 + N)]
    dt = time.time() - t0
    errs = [np.hypot(f["est_xy"][0] - tx, f["est_xy"][1] - ty) for f in fixes]
    res_loc = fixes[int(np.argsort(errs)[len(errs) // 2])]   # representative (median)
    w = np.array([f["confidence"] for f in fixes]) + 1e-9
    ax_ = np.average([f["est_xy"][0] for f in fixes], weights=w)
    ay_ = np.average([f["est_xy"][1] for f in fixes], weights=w)
    avg_err = np.hypot(ax_ - tx, ay_ - ty)
    herr = np.rad2deg(abs(np.angle(np.exp(1j * (res_loc["heading_rad"] - heading)))))
    print(f"    prior error            : {np.hypot(prior[0]-tx, prior[1]-ty):.1f} m")
    print(f"    single-frame fix error : median {np.median(errs):.1f} m  "
          f"(range {min(errs):.1f}-{max(errs):.1f} m over {N})")
    print(f"    {N}-frame averaged fix  : {avg_err:.1f} m  <-- what the EKF does")
    print(f"    heading error          : {herr:.1f} deg")
    print(f"    peak corr {res_loc['corr']:.3f}  confidence {res_loc['confidence']:.2f}"
          f"  ({dt/N:.1f}s/fix)")
    ex, ey = res_loc["est_xy"]

    # 6. figure --------------------------------------------------------------
    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    im = ax[0, 0].imshow(dsm, origin="lower", cmap="terrain",
                         extent=[0, dsm.shape[1]*res, 0, dsm.shape[0]*res])
    ax[0, 0].plot(tx, ty, "r*", ms=17, label="true")
    ax[0, 0].plot(*prior, "yo", ms=8, label="GPS prior")
    ax[0, 0].plot(ex, ey, "c+", ms=13, mew=3, label="single fix")
    ax[0, 0].plot(ax_, ay_, "mx", ms=12, mew=3, label="avg fix")
    ax[0, 0].set_title("DSM + localization"); ax[0, 0].legend(loc="upper left")
    fig.colorbar(im, ax=ax[0, 0], shrink=0.7, label="surface elev (m)")

    s = seg["overcast"]
    ax[0, 1].imshow(s["img"], cmap="gray", aspect="auto")
    ax[0, 1].plot(np.arange(IMG_W), s["hr"], "g-", lw=1.2, label="true")
    f = camera.focal_px(IMG_W, HFOV)
    ax[0, 1].plot(np.arange(IMG_W), IMG_H/2 - f*np.tan(s["elev_seg"]),
                  "r--", lw=1, label="segmented")
    ax[0, 1].set_title("overcast frame + classical horizon"); ax[0, 1].legend()

    ax[1, 0].plot(np.rad2deg(daz), np.rad2deg(seg["clear"]["elev_true"]),
                  "g-", label="true skyline")
    ax[1, 0].plot(np.rad2deg(daz), np.rad2deg(seg["clear"]["elev_seg"]),
                  "r--", label="segmented")
    ax[1, 0].set_xlabel("relative azimuth (deg)"); ax[1, 0].set_ylabel("elevation (deg)")
    ax[1, 0].set_title("camera horizon curve"); ax[1, 0].legend(); ax[1, 0].grid(alpha=.3)

    off = res_loc["offsets"]
    rim = ax[1, 1].imshow(res_loc["response"], origin="lower", cmap="viridis",
                          extent=[prior[0]+off[0], prior[0]+off[-1],
                                  prior[1]+off[0], prior[1]+off[-1]])
    ax[1, 1].plot(tx, ty, "r*", ms=15); ax[1, 1].plot(ex, ey, "c+", ms=13, mew=3)
    ax[1, 1].set_title("match score surface (-residual)")
    fig.colorbar(rim, ax=ax[1, 1], shrink=0.7)

    fig.suptitle("Canopy horizon localization — synthetic DSM, no real data, no training",
                 fontsize=13)
    fig.tight_layout()
    path = os.path.join(OUT, "demo.png")
    fig.savefig(path, dpi=110)
    print(f"\n[6] Figure saved -> {path}")
    print("=" * 66)


if __name__ == "__main__":
    main()
