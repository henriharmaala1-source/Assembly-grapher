"""Power, energy, battery health and hover efficiency."""

from __future__ import annotations

import numpy as np
import pandas as pd

from .config import Aircraft
from .loader import Log, infer_cells
from .segments import runs


def _trapz(y: np.ndarray, x: np.ndarray) -> float:
    ok = np.isfinite(y) & np.isfinite(x)
    return float(np.trapezoid(y[ok], x[ok])) if ok.sum() > 1 else 0.0


def internal_resistance(V: np.ndarray, I: np.ndarray, fs: float) -> dict | None:
    """Pack + wiring resistance from fast voltage dips against fast current changes.

    The slow trend (state of charge) is removed with a 2 s rolling median on both
    signals, leaving V_fast = -R * I_fast. Fit through the origin with outlier rejection.
    """
    n = max(3, int(2.0 * fs))
    v_fast = V - pd.Series(V).rolling(n, center=True, min_periods=n // 4).median().to_numpy()
    i_fast = I - pd.Series(I).rolling(n, center=True, min_periods=n // 4).median().to_numpy()
    ok = np.isfinite(v_fast) & np.isfinite(i_fast)
    v_fast, i_fast = v_fast[ok], i_fast[ok]
    if v_fast.size < 100:
        return None
    sel = np.abs(i_fast) > max(2.0, np.percentile(np.abs(i_fast), 80))
    if sel.sum() < 50:
        return None
    x, y = i_fast[sel], v_fast[sel]
    for _ in range(3):
        r = -np.sum(x * y) / np.sum(x * x)
        res = y + r * x
        keep = np.abs(res) < 3 * np.std(res) + 1e-9
        x, y = x[keep], y[keep]
    r = -np.sum(x * y) / np.sum(x * x)
    resid = y + r * x
    r2 = 1 - np.sum(resid ** 2) / np.sum((y - y.mean()) ** 2)
    return {"r_mohm": float(r * 1000), "r2": float(r2), "points": int(x.size)}


def detect_clipping(I: np.ndarray, fs: float) -> dict:
    """Current sensor saturating: repeated flat tops at the same maximum value."""
    if not np.isfinite(I).any():
        return {"suspect": False}
    imax = float(np.nanmax(I))
    near = I >= imax - max(0.005 * imax, 0.05)
    flat = [(a, b) for a, b in runs(near) if (b - a) >= max(5, int(0.01 * fs))]
    total = sum(b - a for a, b in flat) / fs
    return {"suspect": bool(len(flat) >= 2 or total >= 0.05),
            "max_a": imax, "flat_tops": len(flat), "flat_time_s": round(total, 3)}


def analyze(log: Log, ac: Aircraft, phases: dict) -> dict:
    df = log.df
    if not log.has("V", "I"):
        return {"available": False, "reason": "no voltage/current fields in log"}

    t = df["t"].to_numpy()
    V = df["V"].to_numpy()
    I = df["I"].to_numpy()
    V = np.where(V > 1.0, V, np.nan)
    P = V * I
    fs = log.fs

    cells = ac.cells or infer_cells(float(np.nanpercentile(V, 99.5)))
    res: dict = {"available": True, "cells": cells}
    res["energy_wh"] = _trapz(P, t) / 3600.0
    res["charge_mah"] = _trapz(I, t) / 3.6
    res["duration_s"] = float(t[-1] - t[0])
    res["v_start"] = float(np.nanmedian(V[: int(2 * fs)]))
    res["v_end"] = float(np.nanmedian(V[-int(2 * fs):]))
    res["v_min"] = float(np.nanmin(V))
    res["v_min_cell"] = res["v_min"] / cells
    res["i_mean"] = float(np.nanmean(I))
    res["i_median"] = float(np.nanmedian(I))
    res["i_peak"] = float(np.nanmax(I))
    res["p_mean"] = float(np.nanmean(P))
    res["p_peak"] = float(np.nanmax(P))
    res["p_peak_at"] = {"V": float(V[np.nanargmax(P)]), "I": float(I[np.nanargmax(P)])}
    dt = np.gradient(t)
    res["time_above_a"] = {str(a): float(np.sum(dt[I > a])) for a in (25, 50, 100)}

    if ac.capacity_mah:
        cap_a = ac.capacity_mah / 1000.0
        res["c_mean"] = res["i_mean"] / cap_a
        res["c_peak"] = res["i_peak"] / cap_a
        res["capacity_used_pct"] = 100.0 * res["charge_mah"] / ac.capacity_mah

    res["clipping"] = detect_clipping(I, fs)
    I_fit = I
    if res["clipping"]["suspect"]:
        # clipped samples report too little current for the voltage dip they cause
        I_fit = np.where(I >= res["clipping"]["max_a"] - max(0.005 * res["clipping"]["max_a"], 0.05), np.nan, I)
    ir = internal_resistance(V, I_fit, fs)
    res["internal_resistance"] = ir
    if "impedance_mohm" in df and df["impedance_mohm"].notna().any():
        z = df["impedance_mohm"].to_numpy()
        z = z[np.isfinite(z) & (z > 0)]
        if z.size:
            res["fc_impedance_mohm"] = float(np.median(z))

    # distance and energy per km
    if log.has("speed"):
        dist_m = _trapz(df["speed"].to_numpy(), t)
        res["distance_km"] = dist_m / 1000.0
        if dist_m > 100:
            res["wh_per_km"] = res["energy_wh"] / (dist_m / 1000.0)
        cruise = phases["label"] == "cruise"
        if cruise.sum() > 2 * fs:
            e = sum(_trapz(P[a:b], t[a:b]) for a, b in phases["segments"]["cruise"]) / 3600.0
            d = sum(_trapz(df["speed"].to_numpy()[a:b], t[a:b]) for a, b in phases["segments"]["cruise"])
            if d > 50:
                res["cruise_wh_per_km"] = e / (d / 1000.0)

    # hover efficiency
    hover = phases["label"] == "hover"
    if hover.sum() > 1.0 * fs:
        h = {"seconds": float(hover.sum() / fs),
             "power_w": float(np.nanmean(P[hover])),
             "current_a": float(np.nanmean(I[hover]))}
        if "mavg" in df:
            u = float(np.nanmean(df["mavg"].to_numpy()[hover]))
            h["motor_output"] = u
            # thrust ~ (motor output)^2 for a voltage-mode ESC: crude TWR estimate
            if 0.05 < u < 0.95:
                h["twr_estimate"] = 1.0 / (u * u)
        if ac.mass_kg:
            h["g_per_w"] = ac.mass_kg * 1000.0 / h["power_w"]
        res["hover"] = h
    return res
