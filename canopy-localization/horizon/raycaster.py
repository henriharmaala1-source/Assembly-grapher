"""DSM horizon ray-caster.

The MML DSM (a 2.5-D surface height-field) IS the 3-D model. We do NOT build a
mesh or a photoreal render. For each azimuth we march outward over the
height-field and track the running-max elevation angle -> the skyline. This
inherently handles occlusion (near treeline hides far terrain) and gives a
1-D curve that matches what a camera's sky/canopy boundary produces.

Coordinate convention (planar, metric):
    x = column * res_m   (East),   y = row * res_m   (North)
    azimuth phi measured from +x axis, CCW positive; dir = (cos phi, sin phi)
"""
from __future__ import annotations
import numpy as np


class HorizonRaycaster:
    def __init__(self, dsm, res_m, nodata=None,
                 earth_radius_m=6.371e6, refraction_k=0.13):
        dsm = np.asarray(dsm, dtype=np.float32).copy()
        if nodata is not None:
            dsm[dsm == nodata] = np.nan
        self.dsm = dsm
        self.H, self.W = dsm.shape
        self.res = float(res_m)
        # effective earth radius incl. standard atmospheric refraction
        self.R_eff = earth_radius_m / (1.0 - refraction_k)

    # ---- bilinear height-field sampler (NaN outside grid / nodata) ----------
    def sample(self, x, y):
        fx = np.asarray(x, dtype=np.float64) / self.res
        fy = np.asarray(y, dtype=np.float64) / self.res
        x0 = np.floor(fx).astype(np.int64); x1 = x0 + 1
        y0 = np.floor(fy).astype(np.int64); y1 = y0 + 1
        inb = (x0 >= 0) & (y0 >= 0) & (x1 < self.W) & (y1 < self.H)
        x0c = np.clip(x0, 0, self.W - 1); x1c = np.clip(x1, 0, self.W - 1)
        y0c = np.clip(y0, 0, self.H - 1); y1c = np.clip(y1, 0, self.H - 1)
        wx = fx - x0; wy = fy - y0
        d = self.dsm
        top = d[y0c, x0c] * (1 - wx) + d[y0c, x1c] * wx
        bot = d[y1c, x0c] * (1 - wx) + d[y1c, x1c] * wx
        val = top * (1 - wy) + bot * wy
        return np.where(inb, val, np.nan)

    # ---- core: skyline elevation angle per azimuth --------------------------
    def raycast(self, x0, y0, view_heights, az_rad,
                max_range_m=1500.0, dr_m=None, r_min_m=4.0):
        """Return skyline elevation angles (radians), shape (n_alt, n_az).

        view_heights : absolute viewpoint height(s) z (m). One ray-march serves
                       all of them (the trick: only the atan numerator changes).
        az_rad       : azimuths to evaluate (rad).
        """
        zs = np.atleast_1d(np.asarray(view_heights, dtype=np.float32))   # (A,)
        az = np.asarray(az_rad, dtype=np.float32)                        # (N,)
        if dr_m is None:
            dr_m = self.res
        ranges = np.arange(r_min_m, max_range_m + dr_m, dr_m, dtype=np.float32)  # (R,)
        cos = np.cos(az)[None, :]; sin = np.sin(az)[None, :]             # (1,N)
        R = ranges[:, None]                                             # (R,1)
        h = self.sample(x0 + R * cos, y0 + R * sin)                      # (R,N)
        # Earth-curvature + refraction: distant surface appears lower.
        h_eff = h - (ranges[:, None] ** 2) / (2.0 * self.R_eff)         # (R,N)
        num = h_eff[None, :, :] - zs[:, None, None]                     # (A,R,N)
        elev = np.arctan2(num, ranges[None, :, None])                   # (A,R,N)
        elev = np.where(np.isnan(elev), -np.pi / 2, elev)
        return elev.max(axis=1)                                          # (A,N)

    def panorama(self, x0, y0, view_height, n_az=720, **kw):
        """Full 360 deg skyline; any heading 'picture' is a slice of this."""
        az = np.linspace(0.0, 2 * np.pi, n_az, endpoint=False)
        return self.raycast(x0, y0, view_height, az, **kw)[0], az

    def fov_view(self, x0, y0, view_height, heading_rad, hfov_rad,
                 n=180, **kw):
        """Skyline over a forward FOV (relative-azimuth axis is intrinsic)."""
        daz = np.linspace(-hfov_rad / 2, hfov_rad / 2, n)
        az = heading_rad + daz
        return self.raycast(x0, y0, view_height, az, **kw)[0], daz
