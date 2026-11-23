"""Compare the JSON metrics of two flights (A/B tests: props, fairings, tune changes)."""

from __future__ import annotations

import json
from pathlib import Path

KEYS = [
    ("power", "energy_wh", "energy Wh"), ("power", "wh_per_km", "Wh/km"), ("power", "i_peak", "peak current A"),
    ("power", "v_min_cell", "min V/cell"), ("aero", "curve.best_range_speed", "best range speed m/s"),
    ("aero", "curve.wh_per_km_at_best", "Wh/km at best range"), ("aero", "curve.model.cda_electrical_m2", "CdA/eta m2"),
    ("aero", "coast.cda_m2", "CdA coast m2"), ("motors", "saturation_pct", "motor saturation %"),
    ("pid", "axes.roll.step.overshoot_pct", "roll overshoot %"), ("pid", "axes.pitch.step.overshoot_pct", "pitch overshoot %"),
    ("pid", "axes.roll.err_mean", "roll mean err"), ("pid", "axes.pitch.err_mean", "pitch mean err"),
    ("vibration", "axes.roll.rms_raw_hf", "roll raw noise rms"), ("vibration", "axes.pitch.rms_raw_hf", "pitch raw noise rms"),
]


def _get(d, path):
    for p in path.split("."):
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


def rows(a: dict, b: dict) -> list[tuple[str, float | None, float | None, str]]:
    """(label, value A, value B, change) for every metric present in either flight."""
    out = []
    for sec, path, label in KEYS:
        x, y = _get(a.get(sec, {}), path), _get(b.get(sec, {}), path)
        if x is None and y is None:
            continue
        num = isinstance(x, (int, float)) and isinstance(y, (int, float))
        change = f"{100 * (y - x) / abs(x):+.1f} %" if num and x else "–"
        out.append((label, x, y, change))
    return out


def load(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))
