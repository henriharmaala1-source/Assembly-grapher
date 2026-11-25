"""Run every analysis on one log and collect the results."""

from __future__ import annotations

import json
import math

import numpy as np

from . import aero, motors, pid, power, segments, vibration
from .config import Aircraft
from .loader import Log, load


def run(log_or_path, ac: Aircraft | None = None, log_index: int | None = None,
        motor_range: tuple[float, float] | None = None) -> tuple[Log, dict, dict]:
    """Returns (log, results, phases)."""
    ac = ac or Aircraft()
    log = log_or_path if isinstance(log_or_path, Log) else load(log_or_path, log_index, motor_range)
    phases = segments.detect(log.df, log.fs, ac.coast_throttle)

    res: dict = {
        "source": log.source,
        "firmware": log.firmware,
        "firmware_family": log.family,
        "aircraft": ac.to_dict(),
        "integrity": {
            "duration_s": log.duration,
            "sample_rate_hz": log.fs,
            "trimmed_artifact_s": log.trimmed_s,
            "gaps": log.gaps.to_dict("records") if log.gaps is not None else [],
            "notes": list(log.notes),
            "fields_present": sorted(c for c in log.df.columns if not c.endswith("_raw")),
        },
        "phases": {"seconds": phases["seconds"], "have_speed": phases["have_speed"]},
    }
    for name, mod in (("power", power), ("aero", aero), ("motors", motors), ("vibration", vibration), ("pid", pid)):
        try:
            res[name] = mod.analyze(log, ac, phases)
        except Exception as exc:          # one failing analysis must not kill the report
            res[name] = {"available": False, "reason": f"analysis failed: {exc!r}"}
    res["scorecard"] = pid.scorecard(res.get("pid", {}), res.get("vibration"), res.get("motors"))
    return log, res, phases


def _clean(o, max_list: int = 64):
    if isinstance(o, dict):
        return {k: _clean(v, max_list) for k, v in o.items()
                if not (isinstance(v, (list, tuple)) and len(v) > max_list)}
    if isinstance(o, (list, tuple)):
        return [_clean(v, max_list) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not math.isfinite(float(o)) else round(float(o), 5)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return _clean(o.tolist(), max_list) if o.size <= max_list else None
    return o


def to_json(res: dict) -> str:
    """Compact metrics (long arrays such as spectra are dropped)."""
    return json.dumps(_clean(res), indent=2, default=str)
