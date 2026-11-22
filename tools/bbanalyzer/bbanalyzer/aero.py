"""Air resistance and the power-vs-speed curve.

Two independent drag estimates:

1. Coast-down (needs mass, no efficiency assumption). During unpowered deceleration
   the energy balance gives  D = -(m v dv/dt + m g dh/dt) / v,  and CdA = 2D / (rho v^2).
   If throttle was above idle, residual thrust was still pushing, so CdA is a LOWER bound.

2. Power-curve fit (needs mass and prop size for the induced term). Steady level flight is
   binned by speed and fitted with
        P_el(v) = a * W * v_i(v) + b + c * v^3
   where v_i is the momentum-theory induced velocity in forward flight, b collects
   profile/fixed losses and c = rho*CdA / (2*eta). c gives an "electrical drag area"
   CdA/eta; multiplying by an assumed eta gives CdA. If coast-down also worked, the ratio
   of the two estimates is a measured eta.

Airspeed: iNav's wind estimate is used when present, otherwise ground speed stands in for
airspeed and the report says so. Out-and-back legs at each speed cancel wind.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import nnls
from scipy.signal import savgol_filter

from .config import Aircraft
from .loader import Log


def induced_velocity(v: np.ndarray, vh: float) -> np.ndarray:
    """Momentum theory, horizontal disc in edgewise flow: v_i^4 + v^2 v_i^2 - vh^4 = 0."""
    v2 = v * v
    return np.sqrt((-v2 + np.sqrt(v2 * v2 + 4 * vh ** 4)) / 2.0)


def _airspeed(log: Log) -> tuple[np.ndarray | None, bool]:
    df = log.df
    if not log.has("speed"):
        return None, False
    if log.has("vn", "ve", "wind_x", "wind_y") and np.nanmax(np.abs(df[["wind_x", "wind_y"]].to_numpy())) > 0.05:
        ax = df["vn"].to_numpy() - df["wind_x"].to_numpy()
        ay = df["ve"].to_numpy() - df["wind_y"].to_numpy()
        return np.hypot(ax, ay), True
    return df["speed"].to_numpy(), False


def coast_down(log: Log, ac: Aircraft, phases: dict, speed: np.ndarray) -> dict:
    out = {"segments": [], "available": False}
    if ac.mass_kg is None:
        out["reason"] = "needs --mass"
        return out
    segs = phases["segments"].get("coast", [])
    if not segs:
        out["reason"] = (f"no unpowered deceleration found (throttle <= {ac.coast_throttle:.0%} while slowing "
                         "from > 8 m/s). Fly a level leg, chop throttle to zero for 2-3 s.")
        return out
    df, fs, m, g, rho = log.df, log.fs, ac.mass_kg, 9.81, ac.rho
    win = max(5, int(0.4 * fs) | 1)
    alt = df["alt"].to_numpy() if log.has("alt") else np.zeros(len(df))
    thr = df["throttle"].to_numpy() if "throttle" in df else np.zeros(len(df))
    vs, ds = [], []
    for a, b in segs:
        if b - a < win + 2:
            continue
        v = savgol_filter(speed[a:b], win, 2)
        dvdt = savgol_filter(speed[a:b], win, 2, deriv=1, delta=1.0 / fs)
        dhdt = savgol_filter(alt[a:b], win, 2, deriv=1, delta=1.0 / fs)
        D = -(m * v * dvdt + m * g * dhdt) / np.maximum(v, 1.0)
        ok = (v > 5) & np.isfinite(D)
        if ok.sum() < 5:
            continue
        cda = 2 * D[ok] / (rho * v[ok] ** 2)
        out["segments"].append({"t": float(log.df["t"].iloc[a]), "v_mean": float(v[ok].mean()),
                                "drag_n": float(np.median(D[ok])), "cda_m2": float(np.median(cda)),
                                "throttle": float(np.nanmean(thr[a:b]))})
        vs.append(v[ok])
        ds.append(D[ok])
    if not vs:
        out["reason"] = "coast segments too short or too slow"
        return out
    v = np.concatenate(vs)
    D = np.concatenate(ds)
    q = 0.5 * rho * v ** 2
    cda = float(np.sum(q * D) / np.sum(q * q))       # least squares D = q * CdA
    out.update({"available": True, "cda_m2": cda, "v_range": [float(v.min()), float(v.max())],
                "lower_bound": bool(np.mean([s["throttle"] for s in out["segments"]]) > 0.05),
                "points": (v.tolist()[::max(1, len(v) // 400)], D.tolist()[::max(1, len(v) // 400)])})
    return out


def power_curve(log: Log, ac: Aircraft, phases: dict, speed: np.ndarray) -> dict:
    df, fs = log.df, log.fs
    if not log.has("P"):
        return {"available": False, "reason": "no power data"}
    steady = (phases["label"] == "cruise") | (phases["label"] == "hover")
    v = speed[steady]
    P = df["P"].to_numpy()[steady]
    ok = np.isfinite(v) & np.isfinite(P)
    v, P = v[ok], P[ok]
    if v.size < 3 * fs:
        return {"available": False, "reason": "not enough steady flight (hover or constant-speed level legs)"}

    edges = np.arange(0, np.nanmax(v) + 2.0, 2.0)
    centers, med, cnt = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = (v >= lo) & (v < hi)
        if sel.sum() >= 1.0 * fs:
            centers.append(float(np.median(v[sel])))
            med.append(float(np.median(P[sel])))
            cnt.append(int(sel.sum()))
    res: dict = {"available": True, "bins": {"v": centers, "p": med, "n": cnt}}
    if len(centers) < 2:
        res["reason"] = "steady flight covers only one speed band - fly level legs at several speeds"
        return res

    c = np.array(centers)
    pm = np.array(med)
    mv = c >= 3.0
    if mv.any():
        k = int(np.argmin(pm[mv] / c[mv]))
        res["best_range_speed_bins"] = float(c[mv][k])
        res["wh_per_km_at_best_bins"] = float(pm[mv][k] / c[mv][k] / 3.6)

    if ac.mass_kg and ac.disc_area_m2 and len(centers) >= 3:
        W, A, rho = ac.weight_n, ac.disc_area_m2, ac.rho
        vh = np.sqrt(W / (2 * rho * A))
        X = np.column_stack([W * induced_velocity(c, vh), np.ones_like(c), c ** 3])
        w = np.sqrt(np.array(cnt, float))
        coef, _ = nnls(X * w[:, None], pm * w)
        a, b, cc = coef
        grid = np.linspace(max(0.5, c.min() * 0.5), c.max() * 1.15, 300)
        model = a * W * induced_velocity(grid, vh) + b + cc * grid ** 3
        fit = a * W * induced_velocity(c, vh) + b + cc * c ** 3
        r2 = 1 - np.sum((pm - fit) ** 2) / max(1e-9, np.sum((pm - pm.mean()) ** 2))
        cda_el = 2 * cc / rho
        res["model"] = {
            "vh_m_s": float(vh), "induced_factor_over_eta": float(a), "fixed_w": float(b),
            "cda_electrical_m2": float(cda_el), "cda_m2_assumed_eta": float(cda_el * ac.eta_drag),
            "eta_assumed": ac.eta_drag, "r2": float(r2),
            "grid_v": grid.tolist(), "grid_p": model.tolist(),
        }
        moving = grid >= 2.0
        res["best_endurance_speed"] = float(grid[int(np.argmin(model))])
        res["best_range_speed"] = float(grid[moving][int(np.argmin(model[moving] / grid[moving]))])
        res["min_power_w"] = float(model.min())
        res["wh_per_km_at_best"] = float((model[moving] / grid[moving]).min() / 3.6)
        if cc <= 0:
            res["model"]["note"] = "parasitic term fitted as zero: speed range too narrow to separate drag"
    return res


def analyze(log: Log, ac: Aircraft, phases: dict) -> dict:
    speed, true_air = _airspeed(log)
    if speed is None:
        return {"available": False, "reason": "no speed data (needs GPS / navVel in the log)"}
    res = {"available": True, "airspeed_source": "wind-corrected (iNav wind estimate)" if true_air
           else "ground speed (no wind estimate in log; fly out-and-back legs to cancel wind)",
           "rho": ac.rho}
    res["coast"] = coast_down(log, ac, phases, speed)
    res["curve"] = power_curve(log, ac, phases, speed)
    cd, pc = res["coast"], res["curve"]
    if cd.get("available") and not cd.get("lower_bound") and pc.get("model", {}).get("cda_electrical_m2", 0) > 0:
        res["eta_measured"] = cd["cda_m2"] / pc["model"]["cda_electrical_m2"]
    return res
