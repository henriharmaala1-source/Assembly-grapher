"""Write the synthetic DSM as a real EPSG:3067 GeoTIFF to exercise the MML path."""
import numpy as np
import rasterio
from rasterio.transform import from_origin
from horizon.synthetic_dsm import make_synthetic_dsm

dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
res = meta["res_m"]
# place it at a plausible Finnish ETRS-TM35FIN location (E~380 km, N~6950 km)
west, north = 380000.0, 6950000.0 + dsm.shape[0] * res
transform = from_origin(west, north, res, res)
with rasterio.open("mml_test.tif", "w", driver="GTiff", height=dsm.shape[0],
                   width=dsm.shape[1], count=1, dtype="float32",
                   crs="EPSG:3067", transform=transform, nodata=-9999.0) as ds:
    ds.write(dsm.astype("float32"), 1)
print("wrote mml_test.tif  EPSG:3067  "
      f"{dsm.shape}  origin E={west:.0f} N={north:.0f}  res {res} m")
