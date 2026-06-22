"""Camera front-end: classical, training-free sky/canopy segmentation.

This is the half that runs on the live camera. It uses NO learned model and NO
real-world training data. The cue is geometric/structural, not appearance:
  sky    -> bright, low local texture (variance)
  canopy -> darker, high local texture
Per column we find the row that best separates "sky above" from "canopy below"
(a 1-D step), then convert boundary rows to elevation angles via intrinsics.

render_camera_frame() synthesizes a test frame from the DSM so the segmenter
can be exercised without real footage (incl. an overcast / low-contrast mode).
"""
from __future__ import annotations
import numpy as np


# ---------------------------------------------------------------- intrinsics
def focal_px(img_w, hfov_deg):
    return (img_w / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)


# ---------------------------------------------------------------- test render
def render_camera_frame(rc, x0, y0, z0, heading_rad,
                        img_w=640, img_h=480, hfov_deg=65.0,
                        overcast=False, seed=0, max_range_m=1500.0):
    """Synthesize a grayscale frame from the DSM skyline (for testing only)."""
    rng = np.random.default_rng(seed)
    f = focal_px(img_w, hfov_deg)
    cx, cy = img_w / 2.0, img_h / 2.0
    cols = np.arange(img_w)
    daz = np.arctan((cols - cx) / f)
    az = heading_rad + daz
    elev = rc.raycast(x0, y0, [z0], az, max_range_m=max_range_m)[0]   # (img_w,)
    horizon_row = cy - f * np.tan(elev)                              # float per col

    rows = np.arange(img_h)[:, None]
    sky_mask = rows < horizon_row[None, :]

    if overcast:                      # bright white sky, low colour contrast
        sky = 232 + rng.normal(0, 3, (img_h, img_w))
        canopy = 96 + rng.normal(0, 26, (img_h, img_w))             # texture survives
    else:
        sky = (188 + 46 * (rows / img_h)) + rng.normal(0, 3, (img_h, img_w))
        canopy = 70 + rng.normal(0, 30, (img_h, img_w))
    # add coarse canopy structure so texture is realistic
    canopy += 14 * np.sin(np.arange(img_w)[None, :] / 7.0) * (rng.random((img_h, img_w)) > 0.5)

    img = np.where(sky_mask, sky, canopy)
    return np.clip(img, 0, 255).astype(np.uint8), elev, horizon_row


# ---------------------------------------------------------------- box stats
def _boxmean(I, r):
    H, W = I.shape
    P = np.zeros((H + 1, W + 1), dtype=np.float64)
    P[1:, 1:] = np.cumsum(np.cumsum(I, 0), 1)
    i0 = np.clip(np.arange(H) - r, 0, H); i1 = np.clip(np.arange(H) + r + 1, 0, H)
    j0 = np.clip(np.arange(W) - r, 0, W); j1 = np.clip(np.arange(W) + r + 1, 0, W)
    A = (P[np.ix_(i1, j1)] - P[np.ix_(i0, j1)]
         - P[np.ix_(i1, j0)] + P[np.ix_(i0, j0)])
    cnt = (i1 - i0)[:, None] * (j1 - j0)[None, :]
    return A / cnt


# ---------------------------------------------------------------- segmentation
def extract_horizon(img, img_w=None, hfov_deg=65.0, win=4, return_conf=False):
    """Classical sky/canopy boundary -> elevation angle per column (radians)."""
    I = img.astype(np.float64)
    H, W = I.shape
    f = focal_px(W, hfov_deg)
    cy = H / 2.0

    mean = _boxmean(I, win)
    var = np.clip(_boxmean(I * I, win) - mean ** 2, 0, None)
    std = np.sqrt(var)

    tn = (std - std.mean()) / (std.std() + 1e-9)        # texture: canopy high
    bn = (mean - mean.mean()) / (mean.std() + 1e-9)     # brightness: sky high
    score = tn - bn                                     # canopy-likeness

    # per-column 1-D step: maximize mean(below) - mean(above)
    cs = np.concatenate([np.zeros((1, W)), np.cumsum(score, 0)], 0)      # (H+1,W)
    r = np.arange(1, H)[:, None]                                         # (H-1,1)
    above = cs[1:H] / r
    below = (cs[H] - cs[1:H]) / (H - r)
    gap = below - above                                                 # (H-1,W)
    bi = np.argmax(gap, 0)                                              # idx in gap rows
    conf = gap.max(0)                                                   # separation strength

    # sub-pixel boundary: parabolic vertex of gap around the max
    cols = np.arange(W)
    bic = np.clip(bi, 1, gap.shape[0] - 2)
    gm = gap[bic - 1, cols]; g0 = gap[bic, cols]; gp = gap[bic + 1, cols]
    denom = gm - 2 * g0 + gp
    off = np.where(denom != 0, 0.5 * (gm - gp) / denom, 0.0)
    off = np.clip(off, -1, 1)
    boundary = bic + off + 1                                            # row (float) per col

    elev = np.arctan((cy - boundary) / f)
    if return_conf:
        return elev, conf
    return elev
