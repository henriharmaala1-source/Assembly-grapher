"""Split a flight into phases: hover, cruise (steady level), coast, punch.

Phases are detected on smoothed signals and kept only when they last long enough,
so the analyses downstream work on stretches where the physics assumptions hold
(steady flight for power curves, unpowered deceleration for coast-down drag).
"""

from __future__ import annotations

import numpy as np
import pandas as pd

MIN_DURATION = {"hover": 1.0, "cruise": 2.0, "coast": 0.5, "punch": 0.3}


def smooth(x: np.ndarray, n: int) -> np.ndarray:
    n = max(1, int(n))
    return pd.Series(x).rolling(n, center=True, min_periods=1).mean().to_numpy()


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) index pairs of True runs."""
    m = np.concatenate([[False], mask.astype(bool), [False]])
    d = np.diff(m.astype(int))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def detect(df: pd.DataFrame, fs: float, coast_throttle: float = 0.2) -> dict:
    n_half = int(0.5 * fs)
    n_one = int(1.0 * fs)
    N = len(df)
    nan = np.full(N, np.nan)

    thr = smooth(df["throttle"].to_numpy(), n_half) if "throttle" in df else nan
    mavg = smooth(df["mavg"].to_numpy(), n_half) if "mavg" in df else nan
    speed = smooth(df["speed"].to_numpy(), n_half) if "speed" in df else nan
    vz = smooth(df["vz"].to_numpy(), n_half) if "vz" in df else nan
    if "speed" in df:
        accel = np.gradient(smooth(df["speed"].to_numpy(), n_one), df["t"].to_numpy())
    else:
        accel = nan
    if all(f"gy{i}" in df for i in range(3)):
        rate = smooth(np.sqrt(sum(df[f"gy{i}"].to_numpy() ** 2 for i in range(3))), n_half)
    else:
        rate = nan

    flying = (mavg > 0.03) if "mavg" in df else np.ones(N, bool)
    have_speed = "speed" in df and np.isfinite(speed).any()

    masks = {}
    with np.errstate(invalid="ignore"):
        masks["hover"] = flying & (np.abs(vz) < 0.5) & (rate < 40) & \
            ((speed < 2.0) if have_speed else True)
        if have_speed:
            masks["cruise"] = flying & (speed > 4.0) & (np.abs(accel) < 0.6) & \
                (np.abs(vz) < 0.8) & (rate < 60) & (thr < 0.9)
            masks["coast"] = (thr <= coast_throttle) & (accel < -0.5) & (speed > 8.0)
        else:
            masks["cruise"] = np.zeros(N, bool)
            masks["coast"] = np.zeros(N, bool)
        masks["punch"] = thr > 0.8

    label = np.full(N, "", dtype=object)
    segs: dict[str, list[tuple[int, int]]] = {}
    for name in ("punch", "coast", "cruise", "hover"):          # priority order
        keep = []
        for a, b in runs(np.nan_to_num(masks[name]).astype(bool)):
            if (b - a) >= MIN_DURATION[name] * fs:
                free = label[a:b] == ""
                if free.mean() > 0.9:
                    label[a:b][free] = name
                    keep.append((int(a), int(b)))
        segs[name] = keep

    summary = {name: round(sum(b - a for a, b in s) / fs, 1) for name, s in segs.items()}
    return {"label": label, "segments": segs, "seconds": summary, "have_speed": bool(have_speed)}
