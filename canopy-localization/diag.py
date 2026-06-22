"""Diagnose the residual surface: is the TRUE pose the minimum?

Separates three causes:
  - geometry/observability (true is a flat valley / not a sharp min)
  - segmentation noise (sharp w/ truth curve, blurred w/ segmented curve)
  - bug (true is never the min even with the exact truth curve)
"""
import numpy as np
from horizon.raycaster import HorizonRaycaster
from horizon.synthetic_dsm import make_synthetic_dsm
from horizon import camera, matcher

dsm, meta = make_synthetic_dsm(1500, 2.0, 0)
rc = HorizonRaycaster(dsm, meta["res_m"])


def residual_surface(tx, ty, hfov, use_seg, heading=np.pi/2, radius=96, step=8,
                     hsearch=8.0):
    z0 = float(rc.sample(tx, ty)) + 15.0
    img, elev_true, _ = camera.render_camera_frame(
        rc, tx, ty, z0, heading, hfov_deg=hfov, overcast=False, seed=7)
    if use_seg:
        q = camera.extract_horizon(img, hfov_deg=hfov)
    else:
        q = elev_true
    cols = np.arange(img.shape[1])
    daz = np.arctan((cols - img.shape[1] / 2) / camera.focal_px(img.shape[1], hfov))

    r = matcher.localize(rc, q, daz, z0, (tx, ty),
                         heading_prior_rad=heading + np.deg2rad(6),
                         heading_search_deg=hsearch,
                         search_radius_m=radius, step_m=step, max_range_m=1500)
    off = r["offsets"]
    resp = r["response"]                       # score = -residual, centered on true
    ij = np.unravel_index(np.argmax(resp), resp.shape)
    bx, by = off[ij[1]], off[ij[0]]            # offset of best from true
    derr = np.hypot(bx, by)
    # value at true (center cell)
    c = len(off) // 2
    val_true = resp[c, c]; val_best = resp[ij]
    return derr, val_true, val_best, r["corr"], r["confidence"]


print("pose A (1000,1500) — lake only in view")
for hfov in (65, 90):
    for seg in (False, True):
        d, vt, vb, cc, conf = residual_surface(1000, 1500, hfov, seg)
        print(f"  hfov={hfov:3d} {'seg' if seg else 'truth'}: "
              f"argmin {d:5.1f} m from true | resid@true={-vt:.2e} "
              f"resid@best={-vb:.2e} | corr {cc:.3f} conf {conf:.2f}")

print("\npose B (1500,1450) — lake NW + clear-cut NE both in a 90deg view")
for hfov in (65, 90):
    for seg in (False, True):
        d, vt, vb, cc, conf = residual_surface(1500, 1450, hfov, seg,
                                               heading=np.deg2rad(110))
        print(f"  hfov={hfov:3d} {'seg' if seg else 'truth'}: "
              f"argmin {d:5.1f} m from true | resid@true={-vt:.2e} "
              f"resid@best={-vb:.2e} | corr {cc:.3f} conf {conf:.2f}")
