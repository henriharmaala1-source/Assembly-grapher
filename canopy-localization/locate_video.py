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
import video_preproc as vp

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
    """local raycaster coords (x=col*res, y=row*res) -> world (E, N).
    Rasters are north-up (row 0 = north), so local y points SOUTH."""
    e, n = transform_xy(transform, y / res, x / res, offset="center")
    return e, n


def world_to_local(E, N, res, transform):
    """inverse of local_to_world."""
    col, row = ~transform * (E, N)
    return (col - 0.5) * res, (row - 0.5) * res


def bearing_deg(heading_deg, transform):
    """internal heading (deg from +col toward +row) -> compass bearing (0 = N, CW)."""
    p = np.deg2rad(heading_deg)
    dE = np.cos(p) * np.sign(transform.a); dN = np.sin(p) * np.sign(transform.e)
    return float(np.degrees(np.arctan2(dE, dN)) % 360)


def grid_extent(xs, ys, res, transform):
    """world (left, right, bottom, top) of a local grid, for imshow(origin='upper')."""
    e0, n0 = local_to_world(xs[0], ys[0], res, transform)
    e1, n1 = local_to_world(xs[-1], ys[-1], res, transform)
    return [e0, e1, n1, n0]


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
           speeds=None, calib=None, auto_level=False, use_clahe=True, roll=None,
           range_step=None, spin=False, spin_n=240):
    rc = HorizonRaycaster(dsm, res)
    H, W = dsm.shape

    cap = cv2.VideoCapture(video)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    # spin needs DENSE frames: rotation is tracked from overlap between
    # consecutive frames, which vanishes if they are a FOV or more apart
    want = spin_n if spin else max_n
    take = max(1, total // want) if total else 1
    frames, i, fov_eff, dcol, cal = [], 0, None, None, None
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i % take == 0:
            if calib is not None and cal is None:
                cal, crop = vp.fit_calib(calib, fr.shape[1], fr.shape[0])
                if crop:
                    print("  NOTE: video aspect differs from the calibration images "
                          "(video mode crops the sensor?) — calib may not apply")
            pre, fov_used, _ = vp.preprocess(fr, fov, cal, auto_level, use_clahe,
                                             roll_override=roll)
            if pre.shape[1] != 640:                     # keep aspect: square pixels
                h640 = max(2, int(round(640 * pre.shape[0] / pre.shape[1])))
                pre = cv2.resize(pre, (640, h640), interpolation=cv2.INTER_AREA)
            if fov_eff is None:
                fov_eff = fov_used
                dcol = np.arctan((np.arange(640) - 320) / camera.focal_px(640, fov_eff))
            frames.append(camera.extract_horizon(pre, hfov_deg=fov_eff))
        i += 1
    cap.release()
    if not frames:
        raise RuntimeError(f"no frames decoded from '{video}' — unreadable, empty, "
                           f"or unsupported codec (try re-encoding to H.264 mp4)")
    if len(frames) > want:                              # frame count was unknown
        keep = np.linspace(0, len(frames) - 1, want).round().astype(int)
        frames = [frames[j] for j in keep]
    print(f"  {len(frames)} frames used (of {total}); preprocessed, fov {fov_eff:.0f} deg")

    bnds = (res * 2, W * res - res * 2, res * 2, H * res - res * 2)
    xspan, yspan = bnds[1] - bnds[0], bnds[3] - bnds[2]
    n_cells = (xspan / grid_m) * (yspan / grid_m)
    if n_cells > 6000:                                  # keep build time bounded
        grid_m = float(np.sqrt(xspan * yspan / 6000))
        print(f"  NOTE: DSM is large ({xspan/1000:.0f}x{yspan/1000:.0f} km) -> "
              f"coarsening grid to {grid_m:.0f} m so the build stays fast. For a "
              f"finer fix, crop the DSM (--en-bounds) or set --grid explicitly.")
    mr = min(8000.0, max(W, H) * res)                   # far horizon: little gain, big cost
    print(f"  building reference: ~{(xspan/grid_m)*(yspan/grid_m):.0f} cells, "
          f"range {mr/1000:.0f} km …", flush=True)
    ref = coldstart.build_reference(
        rc, bnds, grid_m, agl=alt, max_range_m=mr,
        dr_m=range_step or max(res, 5.0))              # march step: speed/accuracy knob
    if spin:                                            # camera spinning in place
        r = coldstart.cold_start_spin(ref, frames, dcol, fov_eff)
    else:
        r = coldstart.cold_start_blind(ref, frames, dcol, fov_eff,
                                       speeds_m=np.array(speeds) if speeds else None)
    E, N = local_to_world(*r["start_xy"], res, transform)
    out = dict(result=r, E=E, N=N, crs=crs)
    tf = None
    if Transformer is not None and crs:
        try:
            tf = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
            lon, lat = tf.transform(E, N)
            out["lat"], out["lon"] = lat, lon
        except Exception:                               # noqa: BLE001
            tf = None
    # the top-K best-guess positions (spatially-distinct minima of the profile
    # cost, i.e. each place under its OWN best heading/speed/altitude), in world
    # coords + lat/lon, so a human can check if the true spot is among them
    cands = coldstart.top_k_candidates(r["profile_cost"], ref, k=8)
    for cd in cands:
        iy, ix = cd["iy"], cd["ix"]
        cd["bearing"] = bearing_deg(r["profile_heading"][iy, ix], transform)
        cd["speed"] = float(r["profile_speed"][iy, ix])
        cd["altitude"] = float(r["profile_alt"][iy, ix])
        cd["E"], cd["N"] = local_to_world(cd["x"], cd["y"], res, transform)
        if tf is not None:
            try:
                clon, clat = tf.transform(cd["E"], cd["N"])
                cd["lat"], cd["lon"] = clat, clon
            except Exception:                           # noqa: BLE001
                pass
    out["candidates"] = cands
    return out, ref


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsm", required=True)
    ap.add_argument("--video")
    ap.add_argument("--make-test-video", action="store_true")
    ap.add_argument("--fov", type=float, default=65.0)
    ap.add_argument("--alt", type=float, nargs="+",
                    help="height(s) above the treetops (m); give one exact value, a "
                         "few to search, or omit to search a default 5-18 m set")
    ap.add_argument("--grid", type=float, default=80.0)
    ap.add_argument("--range-step", type=float,
                    help="ray-march step (m) for the reference; larger = faster build")
    ap.add_argument("--speeds", type=float, nargs="+",
                    help="candidate per-sampled-frame motion (m); tune for your video")
    ap.add_argument("--calib", help="camera calibration .json/.npz (fx,fy,cx,cy,k1,k2,...)")
    ap.add_argument("--roll", type=float, help="constant camera roll (deg) from IMU to correct")
    ap.add_argument("--level", action="store_true",
                    help="skyline-based auto roll-level — ONLY for ~flat horizons; "
                         "conflates with real terrain slope, prefer --roll from IMU")
    ap.add_argument("--no-clahe", action="store_true", help="disable contrast normalization")
    ap.add_argument("--spin", action="store_true",
                    help="camera spinning in place (hover+yaw): treat as a panorama "
                         "from one spot instead of forward flight")
    ap.add_argument("--en-bounds", type=float, nargs=4)
    ap.add_argument("--lonlat-bounds", type=float, nargs=4)
    ap.add_argument("--truth-en", type=float, nargs=2, metavar=("E", "N"),
                    help="known true start position (DSM CRS) — marked on the maps, "
                         "with each guess's distance and the true spot's rank")
    ap.add_argument("--out", default="out")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    alts = a.alt if a.alt else [5., 8., 11., 14., 18.]   # search if not given

    dsm, res, tr, crs = load_metric_dsm(a.dsm, a.en_bounds, a.lonlat_bounds)
    print(f"DSM {dsm.shape} @ {res:.0f} m, CRS {crs}, "
          f"elev {np.nanmin(dsm):.0f}-{np.nanmax(dsm):.0f} m")

    vid = a.video
    truth = None
    if a.make_test_video:
        vid = os.path.join(a.out, "test_clip.mp4")
        truth, hd = make_test_video(dsm, res, vid, alts[len(alts) // 2], a.fov)
        print(f"test video -> {vid} (truth start local {truth[0].round()} m)")
    if not vid:
        print("Provide --video, or --make-test-video. Done (DSM loaded OK).")
        return

    calib = vp.load_calib(a.calib) if a.calib else None
    out, ref = locate(vid, dsm, res, tr, crs, a.fov, alts, a.grid, speeds=a.speeds,
                      calib=calib, auto_level=a.level, use_clahe=not a.no_clahe,
                      roll=a.roll, range_step=a.range_step, spin=a.spin)
    r = out["result"]
    brg = bearing_deg(r["heading_deg"], tr)
    print(f"\nLOCATION (cold start, no GPS/IMU/VO{', SPIN' if a.spin else ''}):")
    print(f"  E={out['E']:.0f}  N={out['N']:.0f}  ({crs})")
    if "lat" in out:
        print(f"  lat={out['lat']:.5f}  lon={out['lon']:.5f}")
    if a.spin:
        print(f"  first-frame bearing {brg:.0f} deg (0=N, clockwise), altitude "
              f"{r['altitude']:.0f} m, swept ~{r['spin_span_deg']:.0f} deg, "
              f"single-look ambiguity {r['single_frame_confusion']} cells")
    else:
        print(f"  bearing {brg:.0f} deg (0=N, clockwise), speed {r['speed']:.0f} m/frame, "
              f"altitude {r['altitude']:.0f} m, "
              f"single-look ambiguity {r['single_frame_confusion']} cells")

    tEN = tuple(a.truth_en) if a.truth_en else None
    if truth is not None:
        tEN = local_to_world(truth[0][0], truth[0][1], res, tr)
        print(f"  (test) start error: {np.hypot(out['E']-tEN[0], out['N']-tEN[1]):.0f} m")

    cands = out["candidates"]
    c0 = cands[0]["cost"] if cands else 1.0
    print(f"\nBEST GUESSES (top {len(cands)} — check if one is your true spot; "
          f"cost x1.0 = best, near-equal costs = ambiguous):")
    for cd in cands:
        ll = f"  ({cd['lat']:.5f}, {cd['lon']:.5f})" if "lat" in cd else ""
        rel = cd["cost"] / c0 if c0 else 1.0
        how = (f"brg {cd['bearing']:.0f}" + ("" if a.spin else f", {cd['speed']:.0f} m/fr")
               + f", alt {cd['altitude']:.0f}")
        far = (f"  [{np.hypot(cd['E']-tEN[0], cd['N']-tEN[1]):.0f} m from truth]"
               if tEN else "")
        print(f"  #{cd['rank']}  E={cd['E']:.0f}  N={cd['N']:.0f}{ll}   "
              f"cost x{rel:.3f}  ({how}){far}")
    if tEN:
        tx, ty = world_to_local(tEN[0], tEN[1], res, tr)
        ix = int(np.argmin(np.abs(ref["xs"] - tx))); iy = int(np.argmin(np.abs(ref["ys"] - ty)))
        P = r["profile_cost"]; fin = np.isfinite(P)
        inside = (ref["xs"][0] - ref["spacing"] <= tx <= ref["xs"][-1] + ref["spacing"]
                  and ref["ys"][0] - ref["spacing"] <= ty <= ref["ys"][-1] + ref["spacing"])
        if inside and np.isfinite(P[iy, ix]):
            rank = int((P[fin] < P[iy, ix]).sum()) + 1
            print(f"  TRUE SPOT: its grid cell ranks {rank} of {int(fin.sum())} "
                  f"(top {100 * rank / fin.sum():.1f}%), cost x{P[iy, ix] / c0:.3f}")
        else:
            print("  TRUE SPOT: outside the searched area (DSM/crop doesn't cover it)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from rasterio.plot import plotting_extent
    H, W = dsm.shape
    dext = plotting_extent(dsm, tr)                     # north-up, real E/N axes
    traj = np.array([local_to_world(px, py, res, tr) for px, py in r["trajectory"]])
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.4))
    ax[0].imshow(dsm, cmap="terrain", extent=dext)
    if truth is not None:
        tw = np.array([local_to_world(px, py, res, tr) for px, py in truth])
        ax[0].plot(tw[:, 0], tw[:, 1], "g-", lw=2, label="true path")
    if tEN:
        ax[0].plot(*tEN, "g*", ms=14, label="true start")
    ax[0].plot(traj[:, 0], traj[:, 1], "c--", lw=2, label="recovered")
    ax[0].plot(out["E"], out["N"], "c+", ms=14, mew=3)
    ax[0].legend(); ax[0].set_title("DSM + recovered trajectory (north up)")
    sc = r["profile_cost"].copy(); sc[~np.isfinite(sc)] = np.nan
    ax[1].imshow(np.exp(-(sc - np.nanmin(sc)) / (3 * np.deg2rad(0.4) ** 2)),
                 cmap="magma", extent=grid_extent(ref["xs"], ref["ys"], res, tr))
    if tEN:
        ax[1].plot(*tEN, "g*", ms=14)
    ax[1].plot(out["E"], out["N"], "c+", ms=14, mew=3)
    ax[1].set_title("cold-start likelihood (best over heading/speed/alt)")
    for x_ in ax:
        x_.ticklabel_format(useOffset=False, style="plain")
        x_.set_xlabel("E (m)"); x_.set_ylabel("N (m)")
    fig.suptitle(f"locate_video: DSM + video -> location (cold start), {crs}", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "locate.png"), dpi=120)
    print(f"  figure -> {os.path.join(a.out, 'locate.png')}")

    # best-guesses map: numbered top-K candidates on the DSM, north up in real
    # E/N, so the user can see at a glance whether their true spot is among them
    fig2, bx = plt.subplots(figsize=(8.6, 8.0))
    bx.imshow(dsm, cmap="terrain", extent=dext)
    for cd in cands:
        first = cd["rank"] == 1
        col = "lime" if first else "yellow"
        bx.plot(cd["E"], cd["N"], "o", mfc="none", mec=col,
                ms=18 if first else 13, mew=2.5 if first else 1.8)
        lab = f"#{cd['rank']}"
        if "lat" in cd:
            lab += f"\n{cd['lat']:.4f},{cd['lon']:.4f}"
        bx.annotate(lab, (cd["E"], cd["N"]), color=col, fontsize=8,
                    fontweight="bold", xytext=(7, 7), textcoords="offset points")
    if tEN:
        bx.plot(*tEN, "r*", ms=18, label="true spot")
        bx.legend(loc="upper right")
    bx.ticklabel_format(useOffset=False, style="plain")
    bx.set_title(f"Best guesses (#1 = best, lime) — north up\n"
                 f"DSM {W*res/1000:.1f}x{H*res/1000:.1f} km @ {res:.0f} m, {crs}")
    bx.set_xlabel("E (m)"); bx.set_ylabel("N (m)")
    fig2.tight_layout(); fig2.savefig(os.path.join(a.out, "candidates.png"), dpi=120)
    print(f"  best-guesses map -> {os.path.join(a.out, 'candidates.png')}")


if __name__ == "__main__":
    main()
