"""Load a blackbox log into a DataFrame with physical units and standard column names.

Supported inputs:
  * raw blackbox files (.TXT / .BBL / .BFL) from iNav or Betaflight, decoded with orangebox
  * CSV exports (blackbox_decode or similar). Columns may carry a unit suffix such as
    "vbat (V)"; a unit suffix means the column is already in physical units.

Standard columns produced (missing ones are simply absent):
  t                 time since start of valid data [s]
  throttle          stick throttle 0..1
  m0..m3            motor outputs normalised 0..1 (layout order, see config.QUADX_POSITIONS)
  mavg              mean motor output 0..1
  V, I, P           pack voltage [V], current [A], electrical power [W]
  sp0..sp2          rate setpoint roll/pitch/yaw [deg/s]
  gy0..gy2          filtered gyro [deg/s];  gr0..gr2 raw gyro [deg/s]
  p0..p2, i0..i2, d0..d2, f0..f2   PID term outputs
  speed, vz, alt    horizontal speed [m/s], vertical speed [m/s, up +], altitude [m]
  vn, ve            north/east velocity [m/s]
  att_roll/att_pitch/att_yaw [deg], accvib, impedance_mohm, rpm (mean motor RPM)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

AXES = ("roll", "pitch", "yaw")


@dataclass
class Log:
    df: pd.DataFrame
    headers: dict
    fs: float
    source: str
    firmware: str
    notes: list[str] = field(default_factory=list)
    trimmed_s: float = 0.0
    gaps: pd.DataFrame | None = None
    raw_columns: list[str] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return float(self.df["t"].iloc[-1] - self.df["t"].iloc[0]) if len(self.df) else 0.0

    def has(self, *cols: str) -> bool:
        return all(c in self.df.columns and self.df[c].notna().any() for c in cols)


# --------------------------------------------------------------------------- reading

def list_logs(path: str | Path) -> list[dict]:
    """Describe every log stored in one blackbox file."""
    from orangebox import Parser

    p = Parser.load(str(path))
    out = []
    for idx in range(1, p.reader.log_count + 1):
        p.set_log_index(idx)
        h = p.headers
        out.append({"index": idx,
                    "firmware": str(h.get("Firmware revision", "?")),
                    "start": str(h.get("Log start datetime", "?"))})
    return out


def _read_raw(path: Path, log_index: int | None) -> tuple[pd.DataFrame, dict, list]:
    if path.suffix.lower() == ".csv":
        df = pd.read_csv(path)
        df.columns = [c.strip() for c in df.columns]
        return df, {}, []

    from orangebox import Parser

    p = Parser.load(str(path))
    count = p.reader.log_count
    if log_index is None:
        # default: the largest log in the file
        sizes = []
        for idx in range(1, count + 1):
            p.set_log_index(idx)
            sizes.append(len(p.reader._frame_data))
        log_index = int(np.argmax(sizes)) + 1
    p.set_log_index(log_index)
    names = list(p.field_names)
    rows = [f.data for f in p.frames()]
    df = pd.DataFrame(rows, columns=names)
    events = [(getattr(e, "type", None), getattr(e, "data", None)) for e in p.events]
    return df, dict(p.headers), events


def _unit_split(col: str) -> tuple[str, str | None]:
    if col.endswith(")") and " (" in col:
        name, unit = col[:-1].split(" (", 1)
        return name.strip(), unit.strip()
    return col.strip(), None


def _num(series: pd.Series) -> np.ndarray:
    return pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)


def _header_pair(h: dict, key: str) -> tuple[float, float] | None:
    v = h.get(key)
    if v is None:
        return None
    if isinstance(v, (list, tuple)) and len(v) >= 2:
        return float(v[0]), float(v[1])
    if isinstance(v, str) and "," in v:
        a, b = v.split(",")[:2]
        return float(a), float(b)
    return None


# --------------------------------------------------------------------------- conversion

def _convert(raw: pd.DataFrame, headers: dict) -> tuple[pd.DataFrame, list[str]]:
    notes: list[str] = []
    cols: dict[str, str] = {}
    units: dict[str, str | None] = {}
    for c in raw.columns:
        name, unit = _unit_split(c)
        cols[name] = c
        units[name] = unit

    def get(name: str) -> np.ndarray | None:
        return _num(raw[cols[name]]) if name in cols else None

    out: dict[str, np.ndarray] = {}

    t = get("time")
    if t is None:
        raise ValueError("log has no 'time' field")
    out["t"] = t / 1e6 if (units.get("time") in (None, "us")) else t

    # throttle 1000..2000 -> 0..1; reject decode garbage (e.g. unsigned wrap-around)
    thr = get("rcCommand[3]")
    if thr is not None:
        thr = np.where((thr < 900) | (thr > 2100), np.nan, thr)
        out["throttle"] = np.clip((thr - 1000.0) / 1000.0, 0, 1)

    # motors, normalised with the motorOutput header range when available
    mrange = _header_pair(headers, "motorOutput")
    if mrange is None:
        lo, hi = headers.get("minthrottle"), headers.get("maxthrottle")
        mrange = (float(lo), float(hi)) if lo is not None and hi is not None else (1000.0, 2000.0)
    lo, hi = mrange
    motors = []
    for k in range(8):
        m = get(f"motor[{k}]")
        if m is None:
            break
        out[f"m{k}_raw"] = m
        out[f"m{k}"] = np.clip((m - lo) / (hi - lo), 0, 1.05)
        motors.append(f"m{k}")
    if motors:
        out["mavg"] = np.mean([out[m] for m in motors], axis=0)

    v = get("vbat")
    if v is not None:
        out["V"] = v if units.get("vbat") == "V" else v * 0.01
    a = get("amperage")
    if a is not None:
        out["I"] = a if units.get("amperage") == "A" else a * 0.01

    for i in range(3):
        sp = get(f"axisRate[{i}]")
        if sp is None:
            sp = get(f"setpoint[{i}]")          # Betaflight
        if sp is not None:
            out[f"sp{i}"] = sp
        g = get(f"gyroADC[{i}]")
        if g is not None:
            out[f"gy{i}"] = g
        g = get(f"gyroRaw[{i}]")
        if g is None:
            g = get(f"gyroUnfilt[{i}]")         # Betaflight name
        if g is not None:
            out[f"gr{i}"] = g
        for term, key in (("p", "axisP"), ("i", "axisI"), ("d", "axisD"), ("f", "axisF")):
            x = get(f"{key}[{i}]")
            if x is not None:
                out[f"{term}{i}"] = x

    # velocity / position: iNav navVel/navPos in cm(/s), NED-ish with z up; Betaflight falls back to GPS_speed
    vn, ve, vz = get("navVel[0]"), get("navVel[1]"), get("navVel[2]")
    if vn is not None and ve is not None:
        out["vn"], out["ve"] = vn / 100.0, ve / 100.0
        out["speed"] = np.hypot(out["vn"], out["ve"])
    else:
        gs = get("GPS_speed")
        if gs is not None:
            out["speed"] = gs / 100.0
    if vz is not None:
        out["vz"] = vz / 100.0
    pz = get("navPos[2]")
    baro = get("BaroAlt")
    if pz is not None:
        out["alt"] = pz / 100.0
    elif baro is not None:
        out["alt"] = baro / 100.0

    for i, ax in enumerate(AXES):
        x = get(f"attitude[{i}]")
        if x is not None:
            out[f"att_{ax}"] = x / 10.0

    for name, key, scale in (("accvib", "accVib", 1.0),
                             ("impedance_mohm", "powerSupplyImpedance", 1.0),
                             ("wind_x", "wind[0]", 0.01), ("wind_y", "wind[1]", 0.01)):
        x = get(key)
        if x is not None:
            out[name] = x * scale

    # motor RPM: Betaflight eRPM[k] (eRPM/100) when bidirectional DShot is used
    erpm = [get(f"eRPM[{k}]") for k in range(4)]
    if all(e is not None for e in erpm):
        out["erpm_mean"] = np.mean(erpm, axis=0)

    df = pd.DataFrame(out)
    if "V" in df and df["V"].notna().any():
        vmax = np.nanmax(df["V"])
        if not (3.0 < vmax < 60.0):
            notes.append(f"Voltage scale looks wrong (max {vmax:.1f} V) - check vbat units.")
    return df, notes


# --------------------------------------------------------------------------- validity

def _leading_artifact(df: pd.DataFrame, fs: float) -> int:
    """Index of the first sample after a decoder artifact.

    Some decoders reconstruct the pre-arm part of iNav logs as a perfectly linear ramp
    on every motor (all motors identical, zero second difference). Real flight data
    never looks like that, so drop that leading run.
    """
    motors = [c for c in ("m0_raw", "m1_raw", "m2_raw", "m3_raw") if c in df]
    if len(motors) < 2:
        return 0
    m = df[motors].to_numpy()
    identical = (np.nanmax(m, axis=1) - np.nanmin(m, axis=1)) < 1e-9
    d2 = np.zeros(len(m), dtype=bool)
    d2[1:-1] = np.abs(np.diff(m[:, 0], 2)) < 1e-9
    artifact = identical & d2
    # first sample where motors differ (real flight) after any artifact run
    real = np.flatnonzero(~identical)
    if real.size == 0:
        return 0
    first_real = int(real[0])
    if artifact[:first_real].sum() > 0.5 * fs:
        return first_real
    return 0


def find_gaps(t: np.ndarray, fs: float) -> pd.DataFrame:
    dt = np.diff(t)
    thresh = max(5.0 / fs, 0.05)
    idx = np.flatnonzero(dt > thresh)
    return pd.DataFrame({"t": t[idx], "gap_ms": dt[idx] * 1000.0})


# --------------------------------------------------------------------------- public API

def load(path: str | Path, log_index: int | None = None,
         motor_range: tuple[float, float] | None = None) -> Log:
    """motor_range overrides the motorOutput header (needed for CSV files, which have no header)."""
    path = Path(path)
    raw, headers, events = _read_raw(path, log_index)
    if motor_range is not None:
        headers["motorOutput"] = list(motor_range)
    df, notes = _convert(raw, headers)

    t = df["t"].to_numpy()
    dt = np.diff(t)
    dt = dt[(dt > 0) & np.isfinite(dt)]
    fs = 1.0 / float(np.median(dt)) if dt.size else float("nan")

    start = _leading_artifact(df, fs)
    trimmed = 0.0
    if start > 0:
        trimmed = float(t[start] - t[0])
        notes.append(f"Dropped {trimmed:.1f} s of decoder artifact at the start "
                     "(all motors identical and ramping linearly).")
        df = df.iloc[start:].reset_index(drop=True)

    df = df[np.isfinite(df["t"])].reset_index(drop=True)
    df["t"] = df["t"] - df["t"].iloc[0]
    if "V" in df and "I" in df:
        df["P"] = df["V"] * df["I"]

    gaps = find_gaps(df["t"].to_numpy(), fs)
    unknown_events = sum(1 for e in events if e[0] is None)
    if unknown_events:
        notes.append(f"{unknown_events} log event(s) of unknown type were skipped by the decoder.")

    fw = str(headers.get("Firmware revision", headers.get("Firmware type", "unknown")))
    return Log(df=df, headers=headers, fs=fs, source=str(path), firmware=fw,
               notes=notes, trimmed_s=trimmed, gaps=gaps, raw_columns=list(raw.columns))


def from_dataframe(raw: pd.DataFrame, headers: dict | None = None, source: str = "<dataframe>") -> Log:
    """Build a Log from raw (log-unit) fields already in memory. Used by tests."""
    headers = headers or {}
    df, notes = _convert(raw, headers)
    t = df["t"].to_numpy()
    fs = 1.0 / float(np.median(np.diff(t)))
    start = _leading_artifact(df, fs)
    trimmed = 0.0
    if start > 0:
        trimmed = float(t[start] - t[0])
        notes.append(f"Dropped {trimmed:.1f} s of decoder artifact at the start.")
        df = df.iloc[start:].reset_index(drop=True)
    df["t"] = df["t"] - df["t"].iloc[0]
    if "V" in df and "I" in df:
        df["P"] = df["V"] * df["I"]
    return Log(df=df, headers=headers, fs=fs, source=source,
               firmware=str(headers.get("Firmware revision", "synthetic")),
               notes=notes, trimmed_s=trimmed, gaps=find_gaps(df["t"].to_numpy(), fs),
               raw_columns=list(raw.columns))


def contiguous_blocks(t: np.ndarray, fs: float, min_len_s: float = 1.0) -> list[tuple[int, int]]:
    """Index ranges [a, b) without logging gaps, at least min_len_s long."""
    dt = np.diff(t)
    breaks = np.flatnonzero(dt > max(5.0 / fs, 0.05)) + 1
    edges = np.concatenate([[0], breaks, [len(t)]])
    out = []
    for a, b in zip(edges[:-1], edges[1:]):
        if (b - a) >= min_len_s * fs:
            out.append((int(a), int(b)))
    return out


def infer_cells(v_max: float) -> int:
    return max(1, math.ceil(v_max / 4.25))
