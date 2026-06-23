"""Rasterize an MML LiDAR point cloud (Laserkeilausaineisto, .laz/.las) to a DSM.

MML's 'Korkeusmalli 2 m' is the DTM (bare earth — trees removed), so it does NOT
carry the canopy. The canopy surface comes from the laser point cloud: this grids
the HIGHEST return per cell (canopy/building top) into a DSM GeoTIFF (EPSG:3067)
that locate_video.py / the planner use directly.

For the older 0.5 pts/m^2 product, use a COARSER cell (~5 m) so each cell has
enough points; the skyline method works fine at 5-10 m (it worked at 30 m).

  pip install "laspy[lazrs]" rasterio numpy
  python3 laz_to_dsm.py tile1.laz tile2.laz --res 5 --out dsm.tif   # 0.5 p -> 5 m
  python3 laz_to_dsm.py --selftest
"""
from __future__ import annotations
import argparse
import glob
import sys
import numpy as np
import laspy
import rasterio
from rasterio.transform import from_origin

NOISE_CLASSES = {7, 18}            # low/high noise — drop so they don't spike the DSM


def _bounds(paths):
    xmin = ymin = np.inf
    xmax = ymax = -np.inf
    crs = None
    for p in paths:
        with laspy.open(p) as f:
            h = f.header
            xmin = min(xmin, h.mins[0]); ymin = min(ymin, h.mins[1])
            xmax = max(xmax, h.maxs[0]); ymax = max(ymax, h.maxs[1])
            if crs is None:
                try:
                    crs = h.parse_crs()
                except Exception:               # noqa: BLE001
                    crs = None
    return (xmin, ymin, xmax, ymax), crs


def fill_gaps(dsm, nod, iters=6):
    a = dsm.copy()
    mask = a == nod
    for _ in range(iters):
        if not mask.any():
            break
        valid = ~mask
        b = np.where(mask, 0.0, a)
        c = valid.astype(np.float32)
        s = np.zeros_like(a); cnt = np.zeros_like(a)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == dx == 0:
                    continue
                s += np.roll(np.roll(b, dy, 0), dx, 1)
                cnt += np.roll(np.roll(c, dy, 0), dx, 1)
        fillable = mask & (cnt > 0)
        a[fillable] = s[fillable] / cnt[fillable]
        mask[fillable] = False
    return a


def rasterize(paths, res, out, crs_override=None, drop_noise=True, fill=True):
    (xmin, ymin, xmax, ymax), crs = _bounds(paths)
    W = int(np.ceil((xmax - xmin) / res))
    H = int(np.ceil((ymax - ymin) / res))
    dsm = np.full((H, W), -np.inf, np.float32)
    npts = 0
    for p in paths:
        las = laspy.read(p)
        x = np.asarray(las.x); y = np.asarray(las.y)
        z = np.asarray(las.z).astype(np.float32)
        if drop_noise and hasattr(las, "classification"):
            keep = ~np.isin(np.asarray(las.classification), list(NOISE_CLASSES))
            x, y, z = x[keep], y[keep], z[keep]
        col = np.clip(((x - xmin) / res).astype(np.int64), 0, W - 1)
        row = np.clip(((ymax - y) / res).astype(np.int64), 0, H - 1)   # north-up
        np.maximum.at(dsm.reshape(-1), row * W + col, z)               # highest return
        npts += z.size
    nod = -9999.0
    dsm[~np.isfinite(dsm)] = nod
    filled = float((dsm != nod).mean()) * 100
    if fill:
        dsm = fill_gaps(dsm, nod)
    transform = from_origin(xmin, ymax, res, res)
    out_crs = crs_override or (crs.to_string() if crs else "EPSG:3067")
    with rasterio.open(out, "w", driver="GTiff", height=H, width=W, count=1,
                       dtype="float32", crs=out_crs, transform=transform,
                       nodata=nod, compress="deflate") as ds:
        ds.write(dsm, 1)
    vals = dsm[dsm != nod]
    return dict(W=W, H=H, res=res, npts=npts, filled_pct=filled,
                pts_per_m2=npts / ((xmax - xmin) * (ymax - ymin) + 1e-9),
                elev=(float(vals.min()), float(vals.max())) if vals.size else (0, 0),
                crs=out_crs, out=out)


def _selftest():
    from horizon.synthetic_dsm import make_synthetic_dsm
    d, m = make_synthetic_dsm(500, 2.0, 0)              # 1x1 km truth @ 2 m
    H, W = d.shape; E0, Ntop = 380000.0, 6951000.0
    rng = np.random.default_rng(0)
    N = int(0.5 * 1000 * 1000)                          # 0.5 pts/m^2
    px = rng.uniform(0, 1000, N); py = rng.uniform(0, 1000, N)
    z = d[np.clip((py / 2).astype(int), 0, H - 1),
          np.clip((px / 2).astype(int), 0, W - 1)] + rng.normal(0, 0.3, N)
    hdr = laspy.LasHeader(point_format=3, version="1.4")
    hdr.offsets = [E0, Ntop - 1000, float(z.min())]; hdr.scales = [0.01, 0.01, 0.01]
    las = laspy.LasData(hdr)
    las.x = E0 + px; las.y = Ntop - py; las.z = z
    las.classification = np.full(N, 3, np.uint8)
    las.write("out/_selftest.las")
    st = rasterize(["out/_selftest.las"], 5.0, "out/dsm_from_laz.tif", "EPSG:3067")
    print(f"selftest: {st['npts']:,} pts ({st['pts_per_m2']:.2f}/m^2) -> "
          f"DSM {st['W']}x{st['H']} @ {st['res']:.0f} m, filled {st['filled_pct']:.0f}% "
          f"before fill, elev {st['elev'][0]:.0f}-{st['elev'][1]:.0f} m -> {st['out']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*", help=".laz/.las files or globs")
    ap.add_argument("--res", type=float, default=5.0, help="cell size (m); ~5 for 0.5p")
    ap.add_argument("--out", default="dsm.tif")
    ap.add_argument("--crs", help="override output CRS (default: from LAS / EPSG:3067)")
    ap.add_argument("--no-fill", action="store_true", help="don't fill empty cells")
    ap.add_argument("--keep-noise", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    import os; os.makedirs("out", exist_ok=True)
    if a.selftest:
        _selftest(); return
    paths = [p for g in a.files for p in glob.glob(g)]
    if not paths:
        ap.error("provide .laz/.las files (or --selftest)")
    st = rasterize(paths, a.res, a.out, a.crs, not a.keep_noise, not a.no_fill)
    print(f"{st['npts']:,} pts ({st['pts_per_m2']:.2f}/m^2) -> DSM {st['W']}x{st['H']} "
          f"@ {st['res']:.0f} m, {st['filled_pct']:.0f}% filled, "
          f"elev {st['elev'][0]:.0f}-{st['elev'][1]:.0f} m, CRS {st['crs']} -> {st['out']}")
    print(f"-> python3 locate_video.py --dsm {st['out']} --video clip.mp4 --alt 12 ...")


if __name__ == "__main__":
    main()
