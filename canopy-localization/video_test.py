"""Test the pipeline on a VIDEO.

Reads a forward-looking video, extracts the skyline from every frame with the
classical (training-free) segmenter, and on a cadence matches it against the
DSM-generated reference to produce an absolute fix. Outputs an annotated video
(detected horizon in red, DSM-expected horizon at the fix in cyan) and a summary
report (truth vs fixes, GPS-prior error vs horizon-fix error).

Usage:
  python3 video_test.py --demo                       # make a synthetic flythrough and test it
  python3 video_test.py --video clip.mp4 [--dsm t.tif --fov 65]
"""
from __future__ import annotations
import argparse
import os
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm, load_geotiff
from horizon import camera, matcher

IMG_W, IMG_H, MAXR = 640, 480, 1200.0


def _rows(elev, hfov):
    f = camera.focal_px(IMG_W, hfov)
    r = IMG_H / 2 - f * np.tan(elev)
    return np.clip(np.nan_to_num(r, nan=IMG_H), 0, IMG_H - 1)


def _polyline(vis, elev, hfov, color, th=2):
    rows = _rows(elev, hfov)
    pts = np.stack([np.arange(IMG_W), rows], 1).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(vis, [pts], False, color, th)


def make_demo_video(path, hfov=65.0, nframes=48, fps=10):
    """Synthetic flythrough over the synthetic DSM; returns (rc, track)."""
    dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
    rc = HorizonRaycaster(dsm, meta["res_m"])
    xs = np.linspace(840, 1180, nframes)
    y, heading = 1450.0, np.pi / 2
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                         (IMG_W, IMG_H), True)
    track = []
    for i, x in enumerate(xs):
        z = float(rc.sample(x, y)) + 12.0
        img, _, _ = camera.render_camera_frame(rc, x, y, z, heading, IMG_W,
                                               IMG_H, hfov, False, i, MAXR)
        vw.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
        track.append((x, y, z, heading))
    vw.release()
    return rc, np.array(track)


def run_test(video, out_video, out_png, hfov=65.0, rc=None, gt_track=None,
             match_every=4, prior_sigma=110.0, seed=0):   # denied/degraded GPS
    cap = cv2.VideoCapture(video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10
    f = camera.focal_px(IMG_W, hfov)
    dcol = np.arctan((np.arange(IMG_W) - IMG_W / 2) / f)
    rng = np.random.default_rng(seed)
    vw = cv2.VideoWriter(out_video, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                         (IMG_W, IMG_H), True)
    recs, example = [], None
    fi = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray.shape != (IMG_H, IMG_W):
            gray = cv2.resize(gray, (IMG_W, IMG_H))
        elev = camera.extract_horizon(gray, hfov_deg=hfov)
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        _polyline(vis, elev, hfov, (0, 0, 255))                 # detected = red

        if rc is not None and gt_track is not None and fi % match_every == 0:
            gx, gy, gz, gh = gt_track[fi]
            prior = (gx + rng.normal(0, prior_sigma), gy + rng.normal(0, prior_sigma))
            hp = gh + np.deg2rad(rng.normal(0, 5))
            r = matcher.localize(rc, elev, dcol, gz, prior, heading_prior_rad=hp,
                                 heading_search_deg=8.0, search_radius_m=170,
                                 step_m=18, max_range_m=MAXR)
            ex, ey = r["est_xy"]
            perr = float(np.hypot(ex - gx, ey - gy))
            prerr = float(np.hypot(prior[0] - gx, prior[1] - gy))
            He = rc.raycast(ex, ey, [gz], r["heading_rad"] + dcol,
                            max_range_m=MAXR)[0]
            _polyline(vis, He, hfov, (255, 200, 0))             # expected = cyan
            cv2.putText(vis, f"frame {fi}  fix {perr:.0f} m  (prior {prerr:.0f} m)",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            recs.append((fi, gx, gy, ex, ey, prerr, perr, r["confidence"]))
            if example is None and fi >= len(gt_track) // 3:
                example = vis.copy()
        else:
            cv2.putText(vis, f"frame {fi}  (segmentation only)", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        vw.write(vis)
        fi += 1
    cap.release(); vw.release()

    _summary(out_png, recs, rc, gt_track, example)
    if recs:
        R = np.array([(x[5], x[6]) for x in recs])
        print(f"frames {fi} | matched {len(recs)} | "
              f"prior err median {np.median(R[:,0]):.0f} m -> "
              f"horizon fix median {np.median(R[:,1]):.0f} m")
    else:
        print(f"frames {fi} | horizon OVERLAY only — this tool does NOT localize a "
              f"real clip.\n  For a LOCATION run:  python3 locate_video.py --dsm "
              f"your_dsm.tif --video your_clip.mp4 --fov 90\n  (or in the app use "
              f"'Test on video...' and SELECT your DSM when asked).")
    print(f"annotated video -> {out_video}\nsummary -> {out_png}")
    return recs


def _summary(out_png, recs, rc, gt_track, example):
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
    if recs and rc is not None:
        R = np.array(recs)
        ax[0].imshow(rc.dsm, origin="lower", cmap="terrain",
                     extent=[0, rc.W * rc.res, 0, rc.H * rc.res])
        ax[0].plot(gt_track[:, 0], gt_track[:, 1], "g-", lw=2, label="truth")
        ax[0].plot(R[:, 3], R[:, 4], "c+", ms=10, mew=2, label="horizon fix")
        ax[0].set_title("trajectory: truth vs horizon fixes")
        ax[0].legend(); ax[0].set_xlabel("E (m)"); ax[0].set_ylabel("N (m)")
        ax[1].plot(R[:, 0], R[:, 5], "o-", color="orange", label="GPS prior error")
        ax[1].plot(R[:, 0], R[:, 6], "o-", color="b", label="horizon fix error")
        ax[1].set_xlabel("frame"); ax[1].set_ylabel("position error (m)")
        ax[1].set_title("error per matched frame"); ax[1].legend(); ax[1].grid(alpha=.3)
    else:
        ax[0].text(.5, .5, "no DSM/track:\nsegmentation overlay only",
                   ha="center", va="center"); ax[0].axis("off")
        ax[1].axis("off")
    if example is not None:
        ax[2].imshow(cv2.cvtColor(example, cv2.COLOR_BGR2RGB))
        ax[2].set_title("example frame: detected (red) vs DSM (cyan)")
    ax[2].set_xticks([]); ax[2].set_yticks([])
    fig.suptitle("Video test — skyline extraction + DSM comparison", fontsize=13)
    fig.tight_layout(); fig.savefig(out_png, dpi=120)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--video")
    ap.add_argument("--dsm", help="DSM GeoTIFF for matching (optional)")
    ap.add_argument("--fov", type=float, default=65.0)
    ap.add_argument("--out", default="out")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if a.demo:
        vid = os.path.join(a.out, "flythrough.mp4")
        rc, track = make_demo_video(vid, a.fov)
        run_test(vid, os.path.join(a.out, "annotated.mp4"),
                 os.path.join(a.out, "video_summary.png"), a.fov, rc, track)
    elif a.video:
        rc = None
        if a.dsm:
            arr, m = load_geotiff(a.dsm)
            rc = HorizonRaycaster(arr, m["res_m"], nodata=m.get("nodata"))
        run_test(a.video, os.path.join(a.out, "annotated.mp4"),
                 os.path.join(a.out, "video_summary.png"), a.fov, rc, None)
    else:
        ap.error("use --demo or --video")


if __name__ == "__main__":
    main()
