"""Download an MML DSM (or DTM) tile for an area, as a GeoTIFF for locate_video.

Uses MML's open WCS (Web Coverage Service). You need a free MML API key
(maanmittauslaitos.fi -> "API key instructions"); pass --api-key or set
MML_API_KEY. Coverage IDs differ per service, so run --capabilities first to list
them, then request your area with --bbox in EPSG:3067 (ETRS-TM35FIN) metres.

  export MML_API_KEY=xxxx
  python3 download_mml.py --capabilities
  python3 download_mml.py --coverage <id> --bbox 380000 388000 7500000 7508000 \
      --out dsm.tif
  python3 download_mml.py --check dsm.tif        # validate any GeoTIFF

This sandbox cannot reach MML (network policy) — run it on your own machine.
"""
from __future__ import annotations
import argparse
import os
import sys
import xml.etree.ElementTree as ET
import requests

# Defaults point at MML's elevation WCS. VERIFY the current base + coverage id
# against MML's API docs / --capabilities; they can change.
DEFAULT_BASE = "https://avoin-paikkatieto.maanmittauslaitos.fi/korkeusmalli/wcs/v2"


def _params(api_key):
    return {"api-key": api_key} if api_key else {}


def capabilities(base, api_key):
    p = {"service": "WCS", "version": "2.0.1", "request": "GetCapabilities"}
    p.update(_params(api_key))
    r = requests.get(base, params=p, timeout=60)
    r.raise_for_status()
    ids = [e.text for e in ET.fromstring(r.content).iter()
           if e.tag.endswith("CoverageId")]
    return ids


def build_getcoverage_url(base, coverage, bbox_en, api_key, axes=("E", "N"),
                          fmt="image/tiff", res=None):
    e0, e1, n0, n1 = bbox_en
    p = [("service", "WCS"), ("version", "2.0.1"), ("request", "GetCoverage"),
         ("CoverageId", coverage),
         ("subset", f"{axes[0]}({e0},{e1})"), ("subset", f"{axes[1]}({n0},{n1})"),
         ("format", fmt)]
    if res:
        p += [("scaleaxes", f"{axes[0]}({res}),{axes[1]}({res})")]
    if api_key:
        p += [("api-key", api_key)]
    req = requests.Request("GET", base, params=p).prepare()
    return req.url


def fetch(url, out):
    r = requests.get(url, timeout=300, stream=True)
    if r.status_code in (401, 403):
        raise RuntimeError("auth failed (check API key / that it's enabled for this service)")
    if r.status_code == 404:
        raise RuntimeError("404 (check --base and --coverage; run --capabilities)")
    if not r.ok:
        raise RuntimeError(f"HTTP {r.status_code}: {r.text[:300]}")
    ct = r.headers.get("Content-Type", "")
    if "xml" in ct or "html" in ct:                 # WCS errors come back as XML
        raise RuntimeError(f"server returned {ct}, not a raster:\n{r.text[:400]}")
    with open(out, "wb") as fh:
        for chunk in r.iter_content(1 << 16):
            fh.write(chunk)
    return out


def check(path):
    import rasterio
    with rasterio.open(path) as ds:
        arr = ds.read(1)
        print(f"  OK: {ds.width}x{ds.height} @ {abs(ds.transform.a):.1f} m, "
              f"CRS {ds.crs}, elev {float(arr.min()):.0f}-{float(arr.max()):.0f} m")
        print(f"  -> use: python3 locate_video.py --dsm {path} --video clip.mp4 ...")


MANUAL = """
Manual download (if the API/network is unavailable):
  1. MML MapSite file service:
       https://asiointi.maanmittauslaitos.fi/karttapaikka/tiedostopalvelu
     pick product 'Pintamalli 2 m' (DSM), draw your area, download GeoTIFF.
  2. or Paituli:  https://paituli.csc.fi/download.html
  3. or funet:    https://www.nic.funet.fi/index/geodata/mml/
Then validate with:  python3 download_mml.py --check yourtile.tif
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=DEFAULT_BASE)
    ap.add_argument("--coverage")
    ap.add_argument("--bbox", type=float, nargs=4,
                    metavar=("Emin", "Emax", "Nmin", "Nmax"))
    ap.add_argument("--axes", nargs=2, default=["E", "N"])
    ap.add_argument("--res", type=float, help="optional output resolution (m)")
    ap.add_argument("--api-key", default=os.environ.get("MML_API_KEY"))
    ap.add_argument("--out", default="dsm.tif")
    ap.add_argument("--capabilities", action="store_true")
    ap.add_argument("--print-url", action="store_true")
    ap.add_argument("--check")
    a = ap.parse_args()

    if a.check:
        check(a.check); return
    try:
        if a.capabilities:
            print("Coverage IDs:")
            for c in capabilities(a.base, a.api_key):
                print("  ", c)
            return
        if not (a.coverage and a.bbox):
            ap.error("need --coverage and --bbox (or --capabilities / --check)")
        url = build_getcoverage_url(a.base, a.coverage, a.bbox, a.api_key,
                                    tuple(a.axes), res=a.res)
        if a.print_url:
            print(url.replace(a.api_key or "", "***") if a.api_key else url); return
        print(f"Requesting {a.coverage} for {a.bbox} …")
        fetch(url, a.out)
        check(a.out)
    except Exception as ex:                          # noqa: BLE001
        print(f"ERROR: {ex}", file=sys.stderr)
        print(MANUAL)
        sys.exit(1)


if __name__ == "__main__":
    main()
