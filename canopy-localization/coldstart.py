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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    os.makedirs("out", exist_ok=True)
    if a.demo:
        _demo()
    else:
        ap.error("use --demo (library functions: build_reference, cold_start)")


if __name__ == "__main__":
    main()
