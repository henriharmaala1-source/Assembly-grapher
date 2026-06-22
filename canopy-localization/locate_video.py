"""End-to-end: a DSM GeoTIFF + a video -> a global location (cold start).

Real-data glue. Loads a DSM tile (MML EPSG:3067 metres, or any projected/
geographic GeoTIFF), reads a forward-looking video, extracts the skyline from
each frame, and runs blind cold start (no GPS/IMU/VO) against a reference grid
built from the DSM. Reports the recovered position in the DSM's CRS (+ lat/lon
if pyproj is available) and saves a figure.

You must provide: a DSM tile covering the flight area, the camera horizontal FOV,
and the approximate altitude above ground (AGL). The footage must be from above
the canopy looking forward, in an area with skyline structure (lakes/relief).

  # make a test video from the DSM itself (to check the plumbing):
  python3 locate_video.py --dsm tile.tif --make-test-video --alt 15 --fov 65
  # localize a real video:
  python3 locate_video.py --dsm tile.tif --video clip.mp4 --alt 15 --fov 65 \
      [--en-bounds Emin Emax Nmin Nmax | --lonlat-bounds W S E N] [--grid 80]
"""
from __future__ import annotations
import argparse
import os
import numpy as np
import cv2
import rasterio
from rasterio.warp import calculate_default_transform, reproject, Resampling
from rasterio.windows import from_bounds
from rasterio.transform import array_bounds, xy as transform_xy

from horizon.raycaster import HorizonRaycaster
from horizon import camera
import coldstart

try:
    from pyproj import Transformer
except Exception:                                       # noqa: BLE001
    Transformer = None


def load_metric_dsm(path, en_bounds=None, lonlat_bounds=None, target_res=30.0):
    """Return (dsm, res_m, transform, crs) in a metric, square-pixel frame."""
    with rasterio.open(path) as src:
        crs = src.crs
        if crs is not None and crs.is_geographic:
            if lonlat_bounds:
                w, s, e, n = lonlat_bounds
                win = from_bounds(w, s, e, n, src.transform)
                data = src.read(1, window=win).astype(np.float32)
                wt = src.window_transform(win)
            else:
                data = src.read(1).astype(np.float32); wt = src.transform
            b = array_bounds(data.shape[0], data.shape[1], wt)
            lon0 = (b[0] + b[2]) / 2
            dst = f"EPSG:{32600 + int((lon0 + 180) / 6) + 1}"   # auto UTM N
            dt, w2, h2 = calculate_default_transform(
                crs, dst, data.shape[1], data.shape[0], *b, resolution=target_res)
            out = np.empty((h2, w2), np.float32)
            reproject(data, out, src_transform=wt, src_crs=crs, dst_transform=dt,
                      dst_crs=dst, resampling=Resampling.bilinear,
                      src_nodata=src.nodata, dst_nodata=np.nan)
            return out, abs(dt.a), dt, dst
        # projected (e.g. MML EPSG:3067) — use directly
        if en_bounds:
            e0, e1, n0, n1 = en_bounds
            win = from_bounds(e0, n0, e1, n1, src.transform)
            data = src.read(1, window=win).astype(np.float32)
            wt = src.window_transform(win)
        else:
            data = src.read(1).astype(np.float32); wt = src.transform
        nod = src.nodata
        if nod is not None:
            data[data == nod] = np.nan
        return data, abs(wt.a), wt, str(crs)


def local_to_world(x, y, res, transform):
    """local raycaster coords (x=col*res, y=row*res) -> world (E, N)."""
    e, n = transform_xy(transform, y / res, x / res, offset="center")
    return e, n


def make_test_video(dsm, res, out, alt, fov, nframes=12, fps=8):
    rc = HorizonRaycaster(dsm, res)
    H, W = dsm.shape
    # start near center, fly +x; pick the most-structured heading
    cx0, cy0 = W * res * 0.4, H * res * 0.5
    dcol = np.arctan((np.arange(640) - 320) / camera.focal_px(640, fov))
    best = (-1, 0.0)
    for hd in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        z = float(rc.sample(cx0, cy0)) + alt
        Hc = rc.raycast(cx0, cy0, [z], hd + dcol, max_range_m=20000, dr_m=res)[0]
        s = float(np.nanstd(Hc))
        if s > best[0]:
            best = (s, hd)
    heading = best[1]
    step = 100.0                                  # m/frame (within blind search range)
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (640, 480), True)
    track = []
    for i in range(nframes):
        x = cx0 + i * step * np.cos(heading); y = cy0 + i * step * np.sin(heading)
        z = float(rc.sample(x, y)) + alt
        img, _, _ = camera.render_camera_frame(rc, x, y, z, heading, 640, 480,
                                               fov, False, i, 20000)
        vw.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)); track.append((x, y))
    vw.release()
    return np.array(track), heading


def locate(video, dsm, res, transform, crs, fov, alt, grid_m=80.0, max_n=12,
           speeds=None):
    rc = HorizonRaycaster(dsm, res)
    H, W = dsm.shape
    f = camera.focal_px(640, fov)
    dcol = np.arctan((np.arange(640) - 320) / f)

    cap = cv2.VideoCapture(video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    take = max(1, total // max_n) if total else 1
    frames, i = [], 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i % take == 0:
            g = fr if fr.ndim == 2 else cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            if g.shape != (480, 640):
                g = cv2.resize(g, (640, 480))
            frames.append(camera.extract_horizon(g, hfov_deg=fov))
        i += 1
    cap.release()
    print(f"  {len(frames)} frames used (of {total})")

    ref = coldstart.build_reference(
        rc, (res * 2, W * res - res * 2, res * 2, H * res - res * 2),
        grid_m, agl=alt, max_range_m=min(20000, max(W, H) * res), dr_m=res)
    r = coldstart.cold_start_blind(ref, frames, dcol, fov,
                                   speeds_m=np.array(speeds) if speeds else None)
    E, N = local_to_world(*r["start_xy"], res, transform)
    out = dict(result=r, E=E, N=N, crs=crs)
    if Transformer is not None and crs:
        try:
            lon, lat = Transformer.from_crs(crs, "EPSG:4326",
                                            always_xy=True).transform(E, N)
            out["lat"], out["lon"] = lat, lon
        except Exception:                               # noqa: BLE001
            pass
    return out, ref


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsm", required=True)
    ap.add_argument("--video")
    ap.add_argument("--make-test-video", action="store_true")
    ap.add_argument("--fov", type=float, default=65.0)
    ap.add_argument("--alt", type=float, default=15.0)
    ap.add_argument("--grid", type=float, default=80.0)
    ap.add_argument("--speeds", type=float, nargs="+",
                    help="candidate per-sampled-frame motion (m); tune for your video")
    ap.add_argument("--en-bounds", type=float, nargs=4)
    ap.add_argument("--lonlat-bounds", type=float, nargs=4)
    ap.add_argument("--out", default="out")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    dsm, res, tr, crs = load_metric_dsm(a.dsm, a.en_bounds, a.lonlat_bounds)
    print(f"DSM {dsm.shape} @ {res:.0f} m, CRS {crs}, "
          f"elev {np.nanmin(dsm):.0f}-{np.nanmax(dsm):.0f} m")

    vid = a.video
    truth = None
    if a.make_test_video:
        vid = os.path.join(a.out, "test_clip.mp4")
        truth, hd = make_test_video(dsm, res, vid, a.alt, a.fov)
        print(f"test video -> {vid} (truth start local {truth[0].round()} m)")
    if not vid:
        print("Provide --video, or --make-test-video. Done (DSM loaded OK).")
        return

    out, ref = locate(vid, dsm, res, tr, crs, a.fov, a.alt, a.grid, speeds=a.speeds)
    r = out["result"]
    print(f"\nLOCATION (cold start, no GPS/IMU/VO):")
    print(f"  E={out['E']:.0f}  N={out['N']:.0f}  ({crs})")
    if "lat" in out:
        print(f"  lat={out['lat']:.5f}  lon={out['lon']:.5f}")
    print(f"  heading {r['heading_deg']:.0f} deg, speed {r['speed']:.0f} m/frame, "
          f"single-look ambiguity {r['single_frame_confusion']} cells")
    if truth is not None:
        eE, eN = local_to_world(*r["start_xy"], res, tr)
        tE, tN = local_to_world(truth[0][0], truth[0][1], res, tr)
        print(f"  (test) start error: {np.hypot(eE-tE, eN-tN):.0f} m")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    H, W = dsm.shape
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.4))
    ax[0].imshow(dsm, origin="lower", cmap="terrain", extent=[0, W*res, 0, H*res])
    if truth is not None:
        ax[0].plot(truth[:, 0], truth[:, 1], "g-", lw=2, label="true path")
        ax[0].plot(truth[0, 0], truth[0, 1], "g*", ms=14)
    ax[0].plot(r["trajectory"][:, 0], r["trajectory"][:, 1], "c--", lw=2, label="recovered")
    ax[0].plot(*r["start_xy"], "c+", ms=14, mew=3)
    ax[0].legend(); ax[0].set_title("DSM + recovered trajectory (local m)")
    sc = r["seq_cost"].copy(); sc[~np.isfinite(sc)] = np.nan
    ax[1].imshow(np.exp(-(sc - np.nanmin(sc)) / (3 * np.deg2rad(0.4) ** 2)),
                 origin="lower", cmap="magma",
                 extent=[ref["xs"][0], ref["xs"][-1], ref["ys"][0], ref["ys"][-1]])
    if truth is not None:
        ax[1].plot(truth[0, 0], truth[0, 1], "g*", ms=14)
    ax[1].plot(*r["start_xy"], "c+", ms=14, mew=3)
    ax[1].set_title("cold-start likelihood")
    fig.suptitle("locate_video: DSM GeoTIFF + video -> location (cold start)", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "locate.png"), dpi=120)
    print(f"  figure -> {os.path.join(a.out, 'locate.png')}")


if __name__ == "__main__":
    main()
