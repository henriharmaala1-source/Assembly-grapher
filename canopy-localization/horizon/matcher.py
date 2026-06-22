"""Curve matcher — localize by matching a horizon curve to DSM ray-casts.

The camera curve (query) and the DSM panorama (reference) are both
elevation-vs-azimuth. We smooth out treetop roughness, zero-mean (removes
pitch/attitude bias), and match on the resulting low-frequency SHAPE — the part
created by lakes/clearings/relief, which survives canopy change.

Heading is NOT free-searched over 360 deg (a single notch makes position and
heading ambiguous). The drone's IMU gives a heading prior; we search a small
window around it, which makes position observable.

  best_heading():  windowed 1-D circular cross-correlation -> heading + score
  localize():      grid search over position -> (x, y, heading, confidence)
"""
from __future__ import annotations
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view


def _smooth(a, win):
    if win <= 1:
        return a
    pad = win // 2
    ap = np.pad(a, pad, mode="edge")
    k = np.ones(win) / win
    return np.convolve(ap, k, mode="valid")[:a.size]


def zmean_smooth(elev, smooth_win=5):
    """Smoothed, zero-mean elevation curve in radians (amplitude preserved).

    Zero-mean removes constant pitch/attitude bias; amplitude (notch depth ->
    range cue) is kept, unlike the unit-normalized shape_feature.
    """
    e = np.asarray(elev, dtype=np.float64).copy()
    n = e.size
    bad = ~np.isfinite(e)
    if bad.any() and (~bad).sum() >= 2:
        idx = np.arange(n)
        e[bad] = np.interp(idx[bad], idx[~bad], e[~bad])
    e = _smooth(e, smooth_win)
    return e - e.mean()


def shape_feature(elev, smooth_win=5, mode="curve"):
    """elevation curve -> shape descriptor (smoothed, zero-mean, unit-norm).

    NaNs (occluded / abstained segments) are linearly interpolated first.
    mode='curve' matches the skyline shape; mode='deriv' uses its slope.
    """
    e = np.asarray(elev, dtype=np.float64).copy()
    n = e.size
    bad = ~np.isfinite(e)
    if bad.any() and (~bad).sum() >= 2:
        idx = np.arange(n)
        e[bad] = np.interp(idx[bad], idx[~bad], e[~bad])
    e = _smooth(e, smooth_win)
    f = np.gradient(e) if mode == "deriv" else e
    f = f - f.mean()
    f /= (np.linalg.norm(f) + 1e-9)
    return f


def best_heading(query_feat, pano_feat, center_k=None, half_window=None):
    """Slide the FOV query around the panorama (circular). Optionally restrict
    shifts to +/- half_window samples around center_k (an IMU heading prior).

    Returns (shift_index, correlation, full_corr_array).
    """
    Nq = query_feat.size
    Np = pano_feat.size
    ext = np.concatenate([pano_feat, pano_feat[:Nq]])
    W = sliding_window_view(ext, Nq)[:Np]                # (Np, Nq)
    Wz = W - W.mean(1, keepdims=True)
    Wn = np.linalg.norm(Wz, axis=1) + 1e-9
    qz = query_feat - query_feat.mean()
    corr = (Wz @ qz) / (Wn * (np.linalg.norm(qz) + 1e-9))
    if center_k is not None and half_window is not None:
        d = np.arange(Np) - center_k
        dist = np.minimum(d % Np, (-d) % Np)
        corr = np.where(dist <= half_window, corr, -2.0)
    k = int(np.argmax(corr))
    return k, float(corr[k]), corr


def localize(rc, query_elev, query_daz, view_height, prior_xy,
             heading_prior_rad, heading_search_deg=8.0,
             search_radius_m=96.0, step_m=12.0,
             max_range_m=1200.0, dr_m=2.0, smooth_win=5, refine=True,
             daz_step_deg=0.5):
    """Coarse->fine grid search. Ray-casts only the heading-prior arc (fast).

    heading_prior_rad : from IMU/magnetometer; restricts the heading search.
    Returns dict with est_xy, heading_rad, corr, confidence, response surface.
    """
    daz_step = np.deg2rad(daz_step_deg)
    dazq = np.arange(query_daz.min(), query_daz.max() + 1e-9, daz_step)
    qe = np.interp(dazq, query_daz, query_elev)
    q_amp = zmean_smooth(qe, smooth_win)               # radians, zero-mean
    qn = np.linalg.norm(q_amp) + 1e-9
    Nq = qe.size
    wq = _smooth(np.abs(np.gradient(q_amp)), 3 * smooth_win)
    wq = wq / (wq.sum() + 1e-12)

    hs = np.deg2rad(heading_search_deg)
    dazc = 0.5 * (dazq[0] + dazq[-1])
    margin = 0.5 * (dazq[-1] - dazq[0]) + hs + np.deg2rad(2)
    arc = np.arange(-margin, margin + 1e-9, daz_step)
    arc_az = heading_prior_rad + dazc + arc            # absolute azimuths
    head_of_shift = arc_az[:arc.size - Nq + 1] - dazq[0]
    ok = np.abs(np.angle(np.exp(1j * (head_of_shift - heading_prior_rad)))) <= hs

    def score_at(xc, yc):
        ae = rc.raycast(xc, yc, [view_height], arc_az,
                        max_range_m=max_range_m, dr_m=dr_m)[0]
        ae = _smooth(ae, smooth_win)
        Wn = sliding_window_view(ae, Nq)               # (S, Nq)
        Wz = Wn - Wn.mean(1, keepdims=True)
        corr = (Wz @ q_amp) / ((np.linalg.norm(Wz, axis=1) + 1e-9) * qn)
        corr = np.where(ok, corr, -2.0)
        k = int(np.argmax(corr))
        resid = float(np.sum(wq * (q_amp - Wz[k]) ** 2))
        return -resid, float(corr[k]), float(np.mod(head_of_shift[k], 2 * np.pi))

    def grid(cx, cy, radius, step):
        offs = np.arange(-radius, radius + step, step)
        resp = np.full((offs.size, offs.size), -1e9)
        best = dict(score=-1e18, xy=(cx, cy), heading=heading_prior_rad, corr=0.0)
        for iy, dy in enumerate(offs):
            for ix, dx in enumerate(offs):
                s, corr, hd = score_at(cx + dx, cy + dy)
                resp[iy, ix] = s
                if s > best["score"]:
                    best = dict(score=s, corr=corr, xy=(cx + dx, cy + dy), heading=hd)
        return best, resp, offs

    best, resp, offs = grid(prior_xy[0], prior_xy[1], search_radius_m, step_m)
    if refine:
        best, _, _ = grid(best["xy"][0], best["xy"][1], step_m, max(step_m / 4, 3))

    flat = np.sort(resp.ravel())[::-1]
    med = float(np.median(-resp))
    distinct = float(flat[0] - flat[1]) if flat.size > 1 else 0.0
    saliency = float(np.std(q_amp))                 # how much skyline structure
    resid_best = float(-best["score"])              # fit quality at optimum
    # confidence: structure in view is the physical predictor of localizability.
    confidence = float(np.clip((saliency - 0.010) / (0.045 - 0.010), 0, 1))
    return dict(est_xy=best["xy"], heading_rad=best["heading"],
                corr=best["corr"], confidence=confidence,
                saliency=saliency, resid_best=resid_best, distinct=distinct,
                response=resp, offsets=offs)
