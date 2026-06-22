"""Synthetic but realistic Finnish-style DSM for end-to-end testing.

Lets the whole pipeline run with zero downloads. Produces a 2 m/px surface
height-field with gentle terrain relief, a near-uniform ~20 m canopy with
treetop roughness, and the features that actually create skyline signal:
a lake, a road/clearing corridor, and a clear-cut block.

Swap-in for real data: load an MML DSM GeoTIFF with rasterio (windowed) and
feed the array + res to HorizonRaycaster identically. See load_geotiff().
"""
from __future__ import annotations
import numpy as np


def make_synthetic_dsm(size_px=1500, res_m=2.0, seed=0):
    rng = np.random.default_rng(seed)
    H = W = size_px
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    xm = xx * res_m; ym = yy * res_m

    # --- terrain (DTM): gentle hills + one esker-like rise ---
    terr = (18 * np.sin(xm / 1300.0) * np.cos(ym / 1700.0)
            + 11 * np.sin((xm + ym) / 900.0)
            + 26 * np.exp(-(((xm - 1800) ** 2 + (ym - 1200) ** 2) / (2 * 700.0 ** 2))))
    terr -= terr.min()
    water_level = float(np.percentile(terr, 18))

    # --- canopy height: near-uniform w/ low-freq patches + treetop roughness ---
    canopy = 20.0 + 4.0 * np.sin(xm / 600.0 + 1.3) * np.cos(ym / 500.0)
    canopy += rng.normal(0, 2.0, size=(H, W)).astype(np.float32)
    canopy = np.clip(canopy, 0, None)

    dsm = terr + canopy

    # --- open features (where the skyline carries information) ---
    lake = ((xm - 1000) ** 2 / 560.0 ** 2 + (ym - 1900) ** 2 / 380.0 ** 2) < 1.0
    road = np.abs(ym - 0.6 * xm - 200.0) < 16.0
    cut = (np.abs(xm - 2200) < 260) & (np.abs(ym - 2400) < 200)

    dsm[lake] = water_level
    bare = road | cut
    dsm[bare] = (terr + rng.normal(0, 0.3, size=(H, W)).astype(np.float32))[bare]

    meta = dict(res_m=res_m, water_level=water_level,
                terrain=terr.astype(np.float32),
                open_mask=(lake | road | cut))
    return dsm.astype(np.float32), meta


def load_geotiff(path, window=None):
    """Real-data path: load an MML DSM GeoTIFF (windowed). Requires rasterio."""
    import rasterio
    from rasterio.windows import Window
    with rasterio.open(path) as ds:
        if window is not None:
            r0, c0, h, w = window
            arr = ds.read(1, window=Window(c0, r0, w, h))
        else:
            arr = ds.read(1)
        res_m = float(abs(ds.transform.a))
        nodata = ds.nodata
    return arr.astype(np.float32), dict(res_m=res_m, nodata=nodata)
