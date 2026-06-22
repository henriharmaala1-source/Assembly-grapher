"""Cold-start (prior-free) global relocalization.

A single 90 deg look is highly ambiguous over a wide area (~188 look-alikes in
the uniqueness experiment). This fuses a SHORT SEQUENCE of looks with known
relative motion (from IMU heading + VO/baro displacement) against a reference
grid of horizon curves, collapsing the ambiguity to a unique global fix — with
no GPS and no position prior.

  build_reference(rc, bounds, spacing) -> grid of (smoothed) panoramas
  residual_map(ref, curve, daz, heading) -> per-cell match cost for one look
  cold_start(ref, frames, headings, rel_xy) -> global position + trajectory

Run:  python3 coldstart.py --demo
"""
from __future__ import annotations
import argparse
import os
import time
import numpy as np

from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm, load_geotiff
from horizon import camera
from horizon.matcher import _smooth

MAXR, DR = 1200.0, 2.0


def build_reference(rc, bounds, spacing_m, n_az=360, agl=12.0,
                    max_range_m=MAXR, dr_m=DR, progress=True):
    xmin, xmax, ymin, ymax = bounds
    xs = np.arange(xmin, xmax, spacing_m)
    ys = np.arange(ymin, ymax, spacing_m)
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    panos = np.full((ys.size, xs.size, n_az), np.nan, np.float32)
    t0 = time.time()
    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            z = rc.sample(x, y)
            if np.isfinite(z):
                p = rc.raycast(x, y, [float(z) + agl], az,
                               max_range_m=max_range_m, dr_m=dr_m)[0]
                panos[iy, ix] = _smooth(p, 5)            # pre-smooth once
    if progress:
        print(f"  reference {ys.size}x{xs.size} = {xs.size*ys.size} cells "
              f"in {time.time()-t0:.0f}s")
    return dict(panos=panos, xs=xs, ys=ys, az=az, n_az=n_az, spacing=spacing_m,
                valid=np.isfinite(panos).all(2))


def residual_map(ref, query_elev, query_daz, heading_rad, fov_deg=65.0):
    """Weighted SSD of one look (at known heading) vs every reference cell."""
    n_az = ref["n_az"]; step = 2 * np.pi / n_az
    nfov = int(np.deg2rad(fov_deg) / step)
    k = np.arange(-nfov // 2, nfov - nfov // 2)
    q = np.interp(k * step, query_daz, query_elev)
    q = _smooth(q, 5); q = q - q.mean()
    w = _smooth(np.abs(np.gradient(q)), 15); w /= w.sum() + 1e-9
    base = int(round((heading_rad % (2 * np.pi)) / (2 * np.pi) * n_az))
    idx = (base + k) % n_az
    refw = ref["panos"][:, :, idx]
    refw = refw - refw.mean(2, keepdims=True)
    return np.tensordot((refw - q) ** 2, w, axes=([2], [0]))   # (ny,nx)


def _lookup_shift(R, oy, ox):
    ny, nx = R.shape
    out = np.full((ny, nx), np.nan, R.dtype)
    y0, y1 = max(0, -oy), min(ny, ny - oy)
    x0, x1 = max(0, -ox), min(nx, nx - ox)
    out[y0:y1, x0:x1] = R[y0 + oy:y1 + oy, x0 + ox:x1 + ox]
    return out


def cold_start(ref, frame_curves, frame_daz, headings, rel_xy, fov_deg=65.0):
    """Fuse a sequence (no prior). rel_xy: cumulative (dE,dN) metres from frame 0.

    Returns dict: start_xy, trajectory, confidence, single_frame_confusion.
    """
    sp = ref["spacing"]; ny, nx = ref["panos"].shape[:2]
    seq = np.zeros((ny, nx)); cnt = np.zeros((ny, nx))
    first_R = None
    for i in range(len(frame_curves)):
        R = residual_map(ref, frame_curves[i], frame_daz, headings[i], fov_deg)
        if first_R is None:
            first_R = R
        ox = int(round(rel_xy[i][0] / sp)); oy = int(round(rel_xy[i][1] / sp))
        m = _lookup_shift(R, oy, ox)
        good = np.isfinite(m) & ref["valid"]
        seq[good] += m[good]; cnt[good] += 1
    seq[cnt < len(frame_curves)] = np.inf

    iy, ix = np.unravel_index(np.argmin(seq), seq.shape)
    sx, sy = ref["xs"][ix], ref["ys"][iy]
    traj = np.array([(sx + d[0], sy + d[1]) for d in rel_xy])

    flat = np.sort(seq[np.isfinite(seq)])
    distinct = float((flat[1] - flat[0]) / (np.median(flat) - flat[0] + 1e-9)) \
        if flat.size > 2 else 0.0
    conf = float(np.clip(distinct, 0, 1))

    # how ambiguous a SINGLE look was (cells within noise of its best)
    f = first_R[ref["valid"]]
    single_conf = int((f <= f.min() + np.deg2rad(0.4) ** 2).sum())
    return dict(start_xy=(sx, sy), trajectory=traj, confidence=conf,
                seq_cost=seq, first_cost=first_R,
                single_frame_confusion=single_conf)


def cold_start_blind(ref, frame_curves, frame_daz, fov_deg=65.0,
                     headings_deg=None, speeds_m=None, progress=True):
    """Cold start with NO IMU and NO VO: brute-force heading + speed too.

    Assumes roughly straight forward flight (camera along course) during the
    bootstrap window. Searches start position x heading x constant speed.
    """
    if headings_deg is None:
        headings_deg = np.arange(0, 360, 5)
    if speeds_m is None:
        speeds_m = np.array([20., 40., 60., 80., 100., 120., 150.])
    ny, nx = ref["panos"].shape[:2]; sp = ref["spacing"]; Nf = len(frame_curves)

    t0 = time.time()
    cube = {}                                          # heading -> [per-frame cost maps]
    for th in headings_deg:
        thr = np.deg2rad(th)
        cube[th] = [residual_map(ref, frame_curves[i], frame_daz, thr, fov_deg)
                    for i in range(Nf)]
    if progress:
        print(f"  precomputed {len(headings_deg)*Nf} cost maps in {time.time()-t0:.0f}s")

    best = None
    for th in headings_deg:
        c, s = np.cos(np.deg2rad(th)), np.sin(np.deg2rad(th))
        maps = cube[th]
        for v in speeds_m:
            seq = np.zeros((ny, nx)); cnt = np.zeros((ny, nx))
            for i in range(Nf):
                ox = int(round(i * v * c / sp)); oy = int(round(i * v * s / sp))
                m = _lookup_shift(maps[i], oy, ox)
                good = np.isfinite(m) & ref["valid"]
                seq[good] += m[good]; cnt[good] += 1
            seq[cnt < Nf] = np.inf
            mn = float(seq.min())
            if best is None or mn < best["cost"]:
                iy, ix = np.unravel_index(np.argmin(seq), seq.shape)
                best = dict(cost=mn, theta=th, speed=float(v), iy=iy, ix=ix,
                            seq=seq.copy())

    sx, sy = ref["xs"][best["ix"]], ref["ys"][best["iy"]]
    c, s = np.cos(np.deg2rad(best["theta"])), np.sin(np.deg2rad(best["theta"]))
    traj = np.array([(sx + i * best["speed"] * c, sy + i * best["speed"] * s)
                     for i in range(Nf)])
    first = np.min([cube[th][0] for th in headings_deg], axis=0)   # best-over-heading
    f0 = first[ref["valid"]]
    single = int((f0 <= f0.min() + np.deg2rad(0.4) ** 2).sum())
    return dict(start_xy=(sx, sy), heading_deg=best["theta"], speed=best["speed"],
                trajectory=traj, seq_cost=best["seq"], first_cost=first,
                single_frame_confusion=single, seconds=time.time() - t0)


# ----------------------------------------------------------------- demo
def _demo():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
    rc = HorizonRaycaster(dsm, meta["res_m"])
    fov = 65.0
    f = camera.focal_px(640, fov)
    dcol = np.arctan((np.arange(640) - 320) / f)

    # bootstrap flythrough (truth) — no GPS, IMU heading + VO displacement only
    n = 12
    xs = np.linspace(720, 1270, n); y, heading = 1450.0, np.pi / 2
    rng = np.random.default_rng(3)
    frames, headings, rel = [], [], []
    for i, x in enumerate(xs):
        z = float(rc.sample(x, y)) + 12.0
        img, _, _ = camera.render_camera_frame(rc, x, y, z, heading, 640, 480,
                                               fov, False, i, MAXR)
        frames.append(camera.extract_horizon(img, hfov_deg=fov))
        headings.append(heading + np.deg2rad(rng.normal(0, 3)))      # noisy IMU
        rel.append((xs[i] - xs[0] + rng.normal(0, 4), rng.normal(0, 4)))  # noisy VO

    print("Building reference grid over the whole area (no prior search)…")
    ref = build_reference(rc, (400, 2600, 400, 2600), 60.0, agl=12.0)

    t0 = time.time()
    r = cold_start(ref, frames, dcol, headings, rel, fov)
    dt = time.time() - t0
    tx, ty = xs[0], y
    err = np.hypot(r["start_xy"][0] - tx, r["start_xy"][1] - ty)
    print(f"\nCold start (NO prior) over "
          f"{(ref['xs'][-1]-ref['xs'][0])/1000:.1f}x"
          f"{(ref['ys'][-1]-ref['ys'][0])/1000:.1f} km, {n}-frame sequence:")
    print(f"  single look alone   : {r['single_frame_confusion']} look-alike cells")
    print(f"  sequence start fix  : {err:.0f} m error  "
          f"(conf {r['confidence']:.2f}, {dt:.1f}s)")

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.7))
    ext = [ref["xs"][0], ref["xs"][-1], ref["ys"][0], ref["ys"][-1]]
    ax[0].imshow(np.exp(-(r["first_cost"] - np.nanmin(r["first_cost"]))
                        / (3 * np.deg2rad(0.4) ** 2)), origin="lower",
                 cmap="magma", extent=ext)
    ax[0].plot(tx, ty, "c+", ms=14, mew=3)
    ax[0].set_title(f"one look — likelihood ({r['single_frame_confusion']} matches)")
    sc = r["seq_cost"].copy(); sc[~np.isfinite(sc)] = np.nan
    ax[1].imshow(np.exp(-(sc - np.nanmin(sc)) / (3 * np.deg2rad(0.4) ** 2)),
                 origin="lower", cmap="magma", extent=ext)
    ax[1].plot(tx, ty, "c+", ms=14, mew=3)
    ax[1].set_title("sequence — likelihood (unique)")
    ax[2].imshow(dsm, origin="lower", cmap="terrain",
                 extent=[0, dsm.shape[1]*2, 0, dsm.shape[0]*2])
    ax[2].plot(xs, [y]*n, "g-", lw=2, label="true path")
    ax[2].plot(r["trajectory"][:, 0], r["trajectory"][:, 1], "c--", lw=2, label="recovered")
    ax[2].plot(tx, ty, "g*", ms=14); ax[2].plot(*r["start_xy"], "c+", ms=14, mew=3)
    ax[2].legend(); ax[2].set_title(f"trajectory (start error {err:.0f} m)")
    fig.suptitle("Cold-start relocalization — no GPS, no prior (sequence of looks)",
                 fontsize=13)
    fig.tight_layout(); fig.savefig("out/coldstart.png", dpi=120)
    print("saved out/coldstart.png")


def _demo_blind():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
    rc = HorizonRaycaster(dsm, meta["res_m"])
    fov = 65.0
    f = camera.focal_px(640, fov)
    dcol = np.arctan((np.arange(640) - 320) / f)

    # forward flight (camera along course), NO IMU heading, NO VO motion logged
    n, step, heading = 12, 40.0, np.pi / 2          # truth: north, 40 m/frame
    sx0, sy0 = 1000.0, 1430.0
    frames = []
    for i in range(n):
        x = sx0 + i * step * np.cos(heading); y = sy0 + i * step * np.sin(heading)
        z = float(rc.sample(x, y)) + 12.0
        img, _, _ = camera.render_camera_frame(rc, x, y, z, heading, 640, 480,
                                               fov, False, i, MAXR)
        frames.append(camera.extract_horizon(img, hfov_deg=fov))

    print("Blind cold start (NO IMU, NO VO) — brute-forcing heading + speed too…")
    ref = build_reference(rc, (400, 2600, 400, 2600), 60.0, agl=12.0)
    r = cold_start_blind(ref, frames, dcol, fov,
                         headings_deg=np.arange(0, 360, 5),
                         speeds_m=np.array([20., 30., 40., 50., 60.]))
    err = np.hypot(r["start_xy"][0] - sx0, r["start_xy"][1] - sy0)
    print(f"\nResult over {(ref['xs'][-1]-ref['xs'][0])/1000:.1f}x"
          f"{(ref['ys'][-1]-ref['ys'][0])/1000:.1f} km, {n}-frame sequence, "
          f"{r['seconds']:.0f}s:")
    print(f"  single look alone  : {r['single_frame_confusion']} look-alike cells "
          f"(heading also unknown)")
    print(f"  recovered start    : {err:.0f} m error")
    print(f"  recovered heading  : {r['heading_deg']:.0f} deg (truth 90)")
    print(f"  recovered speed    : {r['speed']:.0f} m/frame (truth 40)")

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.7))
    ext = [ref["xs"][0], ref["xs"][-1], ref["ys"][0], ref["ys"][-1]]
    sc = lambda R: np.exp(-(R - np.nanmin(R)) / (3 * np.deg2rad(0.4) ** 2))
    ax[0].imshow(sc(r["first_cost"]), origin="lower", cmap="magma", extent=ext)
    ax[0].plot(sx0, sy0, "c+", ms=14, mew=3)
    ax[0].set_title(f"one look, heading unknown ({r['single_frame_confusion']} matches)")
    seqm = r["seq_cost"].copy(); seqm[~np.isfinite(seqm)] = np.nan
    ax[1].imshow(sc(seqm), origin="lower", cmap="magma", extent=ext)
    ax[1].plot(sx0, sy0, "c+", ms=14, mew=3)
    ax[1].set_title("blind sequence — likelihood")
    ax[2].imshow(dsm, origin="lower", cmap="terrain",
                 extent=[0, dsm.shape[1]*2, 0, dsm.shape[0]*2])
    tru = np.array([(sx0 + i*step*np.cos(heading), sy0 + i*step*np.sin(heading))
                    for i in range(n)])
    ax[2].plot(tru[:, 0], tru[:, 1], "g-", lw=2, label="true path")
    ax[2].plot(r["trajectory"][:, 0], r["trajectory"][:, 1], "c--", lw=2, label="recovered")
    ax[2].plot(sx0, sy0, "g*", ms=14); ax[2].plot(*r["start_xy"], "c+", ms=14, mew=3)
    ax[2].legend(); ax[2].set_title(
        f"start err {err:.0f} m | hdg {r['heading_deg']:.0f}° spd {r['speed']:.0f}")
    fig.suptitle("Blind cold start — NO IMU, NO VO (search position+heading+speed)",
                 fontsize=13)
    fig.tight_layout(); fig.savefig("out/coldstart_blind.png", dpi=120)
    print("saved out/coldstart_blind.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--blind", action="store_true",
                    help="cold start with no IMU and no VO (search heading+speed)")
    a = ap.parse_args()
    os.makedirs("out", exist_ok=True)
    if a.blind:
        _demo_blind()
    elif a.demo:
        _demo()
    else:
        ap.error("use --demo or --blind")


if __name__ == "__main__":
    main()
