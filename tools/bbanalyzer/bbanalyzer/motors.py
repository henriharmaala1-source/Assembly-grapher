"""Motor balance, thrust splits, saturation and motor-output noise.

Splits are measured only while the sticks are centred (all rate setpoints small), so a
persistent split means the aircraft needs uneven thrust just to fly straight:
  front-rear  -> centre of gravity fore/aft offset or front/rear thrust difference
  left-right  -> lateral CG offset or side-to-side thrust difference
  diagonal    -> yaw torque bias (twisted motor/arm, prop mismatch, motor efficiency)
"""

from __future__ import annotations

import numpy as np
from scipy.signal import welch

from .config import Aircraft, QUADX_POSITIONS
from .loader import Log


def _groups() -> dict[str, tuple[list[int], list[int]]]:
    pos = QUADX_POSITIONS
    front = [k for k, p in pos.items() if p.startswith("front")]
    rear = [k for k, p in pos.items() if p.startswith("rear")]
    left = [k for k, p in pos.items() if p.endswith("left")]
    right = [k for k, p in pos.items() if p.endswith("right")]
    diag_a = [k for k, p in pos.items() if p in ("rear-right", "front-left")]   # M1 + M4
    diag_b = [k for k, p in pos.items() if p in ("front-right", "rear-left")]   # M2 + M3
    return {"front_minus_rear": (front, rear), "left_minus_right": (left, right),
            "diagM1M4_minus_diagM2M3": (diag_a, diag_b)}


def analyze(log: Log, ac: Aircraft, phases: dict) -> dict:
    df, fs = log.df, log.fs
    motors = [f"m{k}" for k in range(4) if f"m{k}" in df]
    if len(motors) < 4:
        return {"available": False, "reason": "fewer than 4 motor fields"}
    M = df[motors].to_numpy()
    flying = df["mavg"].to_numpy() > 0.05
    res: dict = {"available": True, "positions": {f"m{k}": QUADX_POSITIONS[k] for k in range(4)}}
    res["mean_pct"] = {m: float(100 * np.nanmean(M[flying, k])) for k, m in enumerate(motors)}

    centred = flying.copy()
    for i in range(3):
        if f"sp{i}" in df:
            centred &= np.abs(df[f"sp{i}"].to_numpy()) < 20
    res["centred_seconds"] = float(centred.sum() / fs)
    splits = {}
    if centred.sum() > fs:
        for name, (a, b) in _groups().items():
            s = (M[centred][:, a].mean(axis=1) - M[centred][:, b].mean(axis=1)) * 100
            splits[name] = {"mean_pct": float(np.mean(s)), "std_pct": float(np.std(s))}
    res["splits_centred"] = splits

    hi = (M >= 0.98).any(axis=1) & flying
    lo = (M <= 0.02).any(axis=1) & flying
    res["saturation_pct"] = float(100 * hi.sum() / max(1, flying.sum()))
    res["min_clip_pct"] = float(100 * lo.sum() / max(1, flying.sum()))

    # how much high-frequency content reaches the motors (D-term noise -> hot motors)
    hf = []
    for k in range(4):
        x = M[flying, k]
        if x.size > 4 * fs:
            f, p = welch(x - x.mean(), fs=fs, nperseg=int(fs))
            band = f > 0
            hf.append(float(p[f > 100].sum() / max(1e-12, p[band].sum())))
    if hf:
        res["motor_hf_fraction"] = float(np.mean(hf))
    return res
