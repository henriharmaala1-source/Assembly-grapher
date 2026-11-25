"""ArduPilot DataFlash logs (.BIN binary or .log text) -> the analyser's standard columns.

The binary format is self-describing: FMT messages (id 128) give each message type's id,
length, name, field format characters and column names. Every message starts with the
two header bytes 0xA3 0x95 and its type id. Fields are little-endian.

Messages used (ArduCopter 4.x names; older equivalents in brackets):
  RATE   RDes/R, PDes/P, YDes/Y      rate setpoint and filtered rate [deg/s]  (step response)
  GYR    GyrX/Y/Z                    raw gyro [rad/s] when IMU_RAW logging is on (spectra)
  IMU    GyrX/Y/Z                    gyro [rad/s], fallback when RATE/GYR are missing
  PIDR/PIDP/PIDY  P, I, D, FF        PID term outputs
  RCOU   C1..C14                     output PWM, mapped to motors with SERVOn_FUNCTION
  CTUN   ThO, Alt, CRt               throttle out 0..1, altitude [m], climb rate [cm/s]
  BAT (CURR)  Volt, Curr, Res        pack voltage, current, ArduPilot's resistance estimate
  XKF1 (NKF1) VN, VE, VD, PD         EKF velocity/position [m/s, m] NED
  XKF2 (NKF2) VWN, VWE               EKF wind estimate [m/s]
  GPS    Spd, VZ                     fallback speed
  ATT    Roll, Pitch, Yaw            attitude [deg]
  VIBE   VibeX/Y/Z                   accelerometer vibration [m/s^2]
  ESC    RPM                         motor RPM from ESC telemetry
  PARM, VER, MSG, MODE               parameters, firmware version, text messages, flight modes
"""

from __future__ import annotations

import struct
import warnings
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd

HEAD = b"\xa3\x95"
FMT_ID = 128

# format char -> (numpy dtype, scale applied when reading the binary log)
_TYPES = {
    "a": ("<i2", None, 32), "b": ("i1", None, 1), "B": ("u1", None, 1), "g": ("<f2", None, 1),
    "h": ("<i2", None, 1), "H": ("<u2", None, 1), "i": ("<i4", None, 1), "I": ("<u4", None, 1),
    "f": ("<f4", None, 1), "d": ("<f8", None, 1), "n": ("S4", None, 1), "N": ("S16", None, 1),
    "Z": ("S64", None, 1), "c": ("<i2", 0.01, 1), "C": ("<u2", 0.01, 1), "e": ("<i4", 0.01, 1),
    "E": ("<u4", 0.01, 1), "L": ("<i4", 1e-7, 1), "M": ("u1", None, 1), "q": ("<i8", None, 1),
    "Q": ("<u8", None, 1),
}

# ArduPilot motor number (1-based) -> position, per FRAME_TYPE, for quad frames
_QUAD_POSITIONS = {
    1: {1: "front-right", 2: "rear-left", 3: "front-left", 4: "rear-right"},     # X
    2: {1: "front-right", 2: "rear-left", 3: "front-left", 4: "rear-right"},     # V
    3: {1: "front-right", 2: "rear-left", 3: "front-left", 4: "rear-right"},     # H
    12: {1: "rear-right", 2: "front-right", 3: "rear-left", 4: "front-left"},    # Betaflight X
    13: {1: "front-right", 2: "front-left", 3: "rear-left", 4: "rear-right"},    # DJI X
    14: {1: "front-right", 2: "rear-right", 3: "rear-left", 4: "front-left"},    # clockwise X
    18: {1: "rear-right", 2: "front-right", 3: "rear-left", 4: "front-left"},    # Betaflight X reversed
}
_COPTER_MODES = {0: "Stabilize", 1: "Acro", 2: "AltHold", 3: "Auto", 4: "Guided", 5: "Loiter", 6: "RTL",
                 7: "Circle", 9: "Land", 11: "Drift", 13: "Sport", 14: "Flip", 15: "AutoTune", 16: "PosHold",
                 17: "Brake", 18: "Throw", 19: "Avoid_ADSB", 20: "Guided_NoGPS", 21: "Smart_RTL",
                 22: "FlowHold", 23: "Follow", 24: "ZigZag", 25: "SystemID", 26: "Heli_Autorotate",
                 27: "Auto RTL", 28: "Turtle"}
_FRAME_TYPE_NAMES = {0: "plus", 1: "X", 2: "V", 3: "H", 4: "V-tail", 5: "A-tail", 10: "Y6B",
                     11: "Y6F", 12: "Betaflight X", 13: "DJI X", 14: "clockwise X", 15: "I",
                     18: "Betaflight X reversed"}


# --------------------------------------------------------------------------- sniffing

def sniff(path: str | Path) -> str | None:
    """'bin' or 'text' if the file is an ArduPilot DataFlash log, else None."""
    with open(path, "rb") as fh:
        head = fh.read(512)
    if head[:2] == HEAD and len(head) > 2 and head[2] == FMT_ID:
        return "bin"
    if head.lstrip().startswith(b"FMT,") or head.lstrip().startswith(b"FMT ,"):
        return "text"
    return None


# --------------------------------------------------------------------------- binary

class _Fmt:
    def __init__(self, mid: int, length: int, name: str, fmt: str, cols: str):
        self.mid, self.length, self.name, self.fmt = mid, length, name, fmt
        self.cols = [c for c in cols.split(",") if c] if cols else []
        fields, self.scales, self.ok = [], {}, True
        for ch, col in zip(fmt, self.cols):
            if ch not in _TYPES:
                self.ok = False
                break
            dt, scale, count = _TYPES[ch]
            fields.append((col, dt, (count,)) if count > 1 else (col, dt))
            if scale is not None:
                self.scales[col] = scale
        if self.ok and len(fmt) == len(self.cols):
            try:
                self.dtype = np.dtype(fields)
                self.ok = self.dtype.itemsize == length - 3
            except (TypeError, ValueError):
                self.ok = False
        else:
            self.ok = False


def _read_bin(data: bytes) -> tuple[dict[str, pd.DataFrame], list[str]]:
    notes: list[str] = []
    fmts: dict[int, _Fmt] = {FMT_ID: _Fmt(FMT_ID, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")}
    offsets: dict[int, list[int]] = defaultdict(list)
    n, pos, skipped = len(data), 0, 0
    fmt_struct = struct.Struct("<BB4s16s64s")
    while pos + 3 <= n:
        if data[pos] != 0xA3 or data[pos + 1] != 0x95 or data[pos + 2] not in fmts:
            nxt = data.find(HEAD, pos + 1)
            if nxt < 0:
                skipped += n - pos
                break
            skipped += nxt - pos
            pos = nxt
            continue
        mid = data[pos + 2]
        length = fmts[mid].length
        if pos + length > n:
            break
        if mid == FMT_ID:
            t, ln, name, fmt, cols = fmt_struct.unpack_from(data, pos + 3)
            dec = lambda b: b.split(b"\0")[0].decode("ascii", "replace").strip()
            fmts[t] = _Fmt(t, ln, dec(name), dec(fmt), dec(cols))
        else:
            offsets[mid].append(pos)
        pos += length
    if skipped:
        notes.append(f"Skipped {skipped} corrupt or unknown bytes while reading the log.")

    buf = np.frombuffer(data, dtype=np.uint8)
    out: dict[str, pd.DataFrame] = {}
    for mid, offs in offsets.items():
        f = fmts[mid]
        if not f.ok:
            notes.append(f"Message {f.name} has an unsupported format ({f.fmt}); skipped.")
            continue
        offs = np.asarray(offs, dtype=np.int64) + 3
        span = np.arange(f.length - 3, dtype=np.int64)
        parts = []
        for k in range(0, offs.size, 50_000):                    # chunked to bound memory
            rows = buf[offs[k:k + 50_000, None] + span]
            parts.append(np.ascontiguousarray(rows).view(f.dtype).reshape(-1))
        rec = np.concatenate(parts)
        cols = {}
        for col in f.cols:
            x = rec[col]
            if x.dtype.kind == "S":
                cols[col] = [s.split(b"\0")[0].decode("ascii", "replace") for s in x]
            elif x.ndim > 1:
                continue                                          # int16[32] arrays: not needed
            else:
                x = x.astype(np.float64) if x.dtype.kind in "iuf" else x
                cols[col] = x * f.scales[col] if col in f.scales else x
        out[f.name] = pd.DataFrame(cols)
    return out, notes


# --------------------------------------------------------------------------- text

def _read_text(text: str) -> tuple[dict[str, pd.DataFrame], list[str]]:
    cols: dict[str, list[str]] = {"FMT": ["Type", "Length", "Name", "Format", "Columns"]}
    rows: dict[str, list[list[str]]] = defaultdict(list)
    bad = 0
    for line in text.splitlines():
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 2 or not parts[0]:
            continue
        name = parts[0]
        if name == "FMT" and len(parts) >= 6:
            cols[parts[3]] = parts[5:]
            continue
        if name not in cols:
            bad += 1
            continue
        vals = parts[1:]
        c = cols[name]
        if len(vals) < len(c):
            bad += 1
            continue
        rows[name].append(vals[:len(c)])
    out = {}
    for name, r in rows.items():
        df = pd.DataFrame(r, columns=cols[name])
        for c in df.columns:
            num = pd.to_numeric(df[c], errors="coerce")
            if num.notna().mean() > 0.9:
                df[c] = num.astype(float)
        out[name] = df
    return out, ([f"Skipped {bad} malformed lines."] if bad else [])


def read(path: str | Path) -> tuple[dict[str, pd.DataFrame], list[str]]:
    """Every message type in the log as a DataFrame (binary values already scaled)."""
    kind = sniff(path)
    if kind == "bin":
        return _read_bin(Path(path).read_bytes())
    if kind == "text":
        return _read_text(Path(path).read_text(encoding="ascii", errors="replace"))
    raise ValueError(f"{path} is not an ArduPilot DataFlash log")


# --------------------------------------------------------------------------- mapping

def _first_instance(df: pd.DataFrame) -> pd.DataFrame:
    """Keep the primary sensor / EKF core / battery when a message has an instance column."""
    # the instance is the field right after TimeUS (PIDR also has an "I" column: the I-term)
    if len(df.columns) > 1 and df.columns[0] == "TimeUS":
        c = df.columns[1]
        if c in ("I", "C", "Inst", "Instance", "IMU", "Core") and df[c].dtype.kind in "iuf":
            vals = df[c].dropna()
            if 1 < vals.nunique() <= 16 and np.all(vals == np.round(vals)):
                return df[df[c] == vals.min()]
    return df


def _clean_time(df: pd.DataFrame) -> pd.DataFrame:
    if "TimeUS" not in df:
        return df.iloc[0:0]
    df = df[np.isfinite(df["TimeUS"])].sort_values("TimeUS", kind="stable")
    return df.drop_duplicates("TimeUS", keep="last")


class _Resampler:
    def __init__(self, base_us: np.ndarray):
        self.base = base_us

    def __call__(self, src_us: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Linear interpolation onto the base time line; NaN where the source has no data nearby."""
        ok = np.isfinite(y)
        src, y = src_us[ok], np.asarray(y, dtype=float)[ok]
        if src.size < 2:
            return np.full(self.base.size, np.nan)
        out = np.interp(self.base, src, y, left=np.nan, right=np.nan)
        dt = np.diff(src)
        max_gap = max(3.0 * float(np.median(dt)), 50_000.0)       # us
        j = np.clip(np.searchsorted(src, self.base), 1, src.size - 1)
        span = src[j] - src[j - 1]
        out[span > max_gap] = np.nan
        return out


def _params(msgs: dict) -> dict[str, float]:
    p = msgs.get("PARM")
    if p is None or "Name" not in p:
        return {}
    # float32 in the log: 0.135 reads back as 0.13500000536, so round to 7 significant digits
    return {str(n): float(f"{float(v):.7g}") for n, v in zip(p["Name"], p["Value"]) if pd.notna(v)}


def _firmware(msgs: dict) -> str:
    ver = msgs.get("VER")
    if ver is not None and "FWS" in ver and len(ver):
        s = str(ver["FWS"].iloc[0]).strip()
        if s:
            return s
    m = msgs.get("MSG")
    if m is not None and "Message" in m:
        for s in m["Message"]:
            s = str(s)
            if s.startswith("Ardu") or "Rover" in s or "Sub" in s.split(" ")[0]:
                return s.strip()
    return "ArduPilot"


def _motor_outputs(params: dict) -> dict[int, int]:
    """ArduPilot motor number -> output channel, from SERVOn_FUNCTION (33..40 = motor 1..8)."""
    mapping = {}
    for n in range(1, 33):
        f = params.get(f"SERVO{n}_FUNCTION")
        if f is None:
            continue
        f = int(f)
        if 33 <= f <= 40:
            mapping[f - 32] = n
        elif 82 <= f <= 85:
            mapping[f - 73] = n
    return mapping


def convert(msgs: dict[str, pd.DataFrame]) -> tuple[pd.DataFrame, dict, list[str]]:
    """Standard-column DataFrame (see loader.py), headers dict and notes."""
    notes: list[str] = []
    params = _params(msgs)
    fw = _firmware(msgs)
    get = lambda name: _clean_time(_first_instance(msgs[name])) if name in msgs and len(msgs[name]) else None

    # base time line: raw gyro if logged (highest rate), else RATE (loop rate), else IMU
    base_msg = None
    for name, need in (("GYR", "GyrX"), ("RATE", "R"), ("IMU", "GyrX"), ("ATT", "Roll")):
        m = get(name)
        if m is not None and need in m and len(m) > 100:
            if name == "GYR":
                rate = 1e6 / float(np.median(np.diff(m["TimeUS"].to_numpy())))
                if rate > 4000:
                    notes.append(f"Raw gyro (GYR) logged at {rate:.0f} Hz; using RATE as the time base.")
                    continue
            base_msg = name
            break
    if base_msg is None:
        raise ValueError("ArduPilot log has none of GYR / RATE / IMU / ATT - nothing to analyse")
    base = get(base_msg)["TimeUS"].to_numpy(dtype=float)
    rs = _Resampler(base)
    out: dict[str, np.ndarray] = {"t": (base - base[0]) / 1e6}

    def put(col: str, name: str, field: str, scale: float = 1.0, offset: float = 0.0):
        m = get(name)
        if m is None or field not in m:
            return False
        out[col] = rs(m["TimeUS"].to_numpy(dtype=float), m[field].to_numpy(dtype=float)) * scale + offset
        return True

    # rates, setpoints, gyro
    for i, (sp, ga) in enumerate((("RDes", "R"), ("PDes", "P"), ("YDes", "Y"))):
        put(f"sp{i}", "RATE", sp)
        if not put(f"gy{i}", "RATE", ga):
            put(f"gy{i}", "IMU", ("GyrX", "GyrY", "GyrZ")[i], 180 / np.pi)
        put(f"gr{i}", "GYR", ("GyrX", "GyrY", "GyrZ")[i], 180 / np.pi)
    if "gy0" not in out:
        notes.append("No RATE message: step response needs LOG_BITMASK 'ATTITUDE_FAST' (or PID logging).")
    if "gr0" not in out:
        notes.append("No raw gyro (GYR): spectra use the filtered rate. Enable INS_LOG_BAT_OPT / IMU_RAW "
                     "logging to see unfiltered noise.")
    for i, msg in enumerate(("PIDR", "PIDP", "PIDY")):
        for term, field in (("p", "P"), ("i", "I"), ("d", "D"), ("f", "FF")):
            put(f"{term}{i}", msg, field)

    # throttle (output, like iNav's rcCommand in nav modes); ArduPlane logs it in percent
    if put("throttle", "CTUN", "ThO"):
        if np.nanmax(out["throttle"]) > 1.5:
            out["throttle"] = out["throttle"] / 100.0
        out["throttle"] = np.clip(out["throttle"], 0, 1)

    # motors
    headers: dict = {}
    rcou = get("RCOU")
    if rcou is not None:
        lo = params.get("MOT_PWM_MIN") or params.get("RC3_MIN") or 1000.0
        hi = params.get("MOT_PWM_MAX") or params.get("RC3_MAX") or 2000.0
        headers["motorOutput"] = [lo, hi]
        mot = _motor_outputs(params) or {k: k for k in range(1, 5)}
        if not _motor_outputs(params):
            notes.append("No SERVOn_FUNCTION parameters in the log: assuming outputs 1-4 are motors 1-4.")
        series = {}
        for k, ch in sorted(mot.items()):
            c = f"C{ch}"
            if c in rcou:
                pwm = rs(rcou["TimeUS"].to_numpy(dtype=float), rcou[c].to_numpy(dtype=float))
                series[k] = np.clip((pwm - lo) / (hi - lo), 0, 1.05)
        if series:
            with warnings.catch_warnings():               # rows before the first RCOU sample are all NaN
                warnings.simplefilter("ignore", RuntimeWarning)
                out["mavg"] = np.nanmean(list(series.values()), axis=0)
        fclass = int(params.get("FRAME_CLASS", 1))
        ftype = int(params.get("FRAME_TYPE", 1))
        headers["frame"] = f"class {fclass}, type {_FRAME_TYPE_NAMES.get(ftype, ftype)}"
        if "FRAME_TYPE" not in params:
            notes.append("No FRAME_TYPE parameter: assuming a Quad X motor layout.")
        pos = _QUAD_POSITIONS.get(ftype) if fclass == 1 else None
        if pos and all(k in series for k in range(1, 5)):
            want = {"rear-right": 0, "front-right": 1, "rear-left": 2, "front-left": 3}   # analyser order
            for k in range(1, 5):
                out[f"m{want[pos[k]]}"] = series[k]
        elif series:
            notes.append(f"Frame {headers['frame']} is not a supported quad layout: per-motor balance skipped, "
                         "average motor output still used.")

    # battery
    if not put("V", "BAT", "Volt"):
        put("V", "CURR", "Volt")
    if not put("I", "BAT", "Curr"):
        put("I", "CURR", "Curr")
    bat = get("BAT")
    if bat is not None and "Res" in bat and np.nanmax(bat["Res"].to_numpy(dtype=float)) > 0:
        put("impedance_mohm", "BAT", "Res", 1000.0)

    # velocity, position, wind: EKF3, then EKF2, then GPS / CTUN
    for xk in ("XKF1", "NKF1"):
        if put("vn", xk, "VN"):
            put("ve", xk, "VE")
            put("vz", xk, "VD", -1.0)
            put("alt", xk, "PD", -1.0)
            break
    if "vn" in out and "ve" in out:
        out["speed"] = np.hypot(out["vn"], out["ve"])
    elif put("speed", "GPS", "Spd"):
        put("vz", "GPS", "VZ", -1.0)
    if "alt" not in out:
        put("alt", "CTUN", "Alt")
    if "vz" not in out:
        put("vz", "CTUN", "CRt", 0.01)
    for xk in ("XKF2", "NKF2"):
        if put("wind_x", xk, "VWN"):
            put("wind_y", xk, "VWE")
            break

    for ax, field in (("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")):
        put(f"att_{ax}", "ATT", field)

    vibe = get("VIBE")
    if vibe is not None and {"VibeX", "VibeY", "VibeZ"} <= set(vibe.columns):
        mag = np.sqrt(vibe["VibeX"] ** 2 + vibe["VibeY"] ** 2 + vibe["VibeZ"] ** 2).to_numpy(dtype=float)
        out["accvib"] = rs(vibe["TimeUS"].to_numpy(dtype=float), mag)

    esc = msgs.get("ESC")
    if esc is not None and "RPM" in esc and len(esc):
        e = esc[np.isfinite(esc["TimeUS"])]
        inst = next((c for c in ("Instance", "I", "Inst") if c in e), None)
        groups = [g for _, g in e.groupby(inst)] if inst else [e]
        rpms = [rs(*(_clean_time(g)[["TimeUS", "RPM"]].to_numpy(dtype=float).T)) for g in groups]
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            if rpms and np.nanmax(rpms) > 0:
                out["rpm_mean"] = np.nanmean(rpms, axis=0)

    df = pd.DataFrame(out)

    # headers: firmware, tuning parameters, flight modes
    headers["Firmware revision"] = fw
    headers.update({k: v for k, v in params.items()
                    if k.startswith(("ATC_", "INS_GYRO", "INS_ACCEL_F", "INS_HNTCH", "INS_HNTC2", "PSC_",
                                     "MOT_", "FRAME_", "SCHED_LOOP", "LOG_BITMASK", "SERVO_BLH_POLES",
                                     "BATT_CAPACITY"))})
    modes = msgs.get("MODE")
    if modes is not None and len(modes) and "TimeUS" in modes:
        mcol = "Mode" if "Mode" in modes else modes.columns[1]
        names = _COPTER_MODES if "Copter" in fw else {}
        fm = []
        for r in modes.itertuples():
            v = getattr(r, mcol)
            try:
                v = names.get(int(v), str(int(v)))
            except (TypeError, ValueError):
                v = str(v)
            fm.append((round(max(0.0, (float(r.TimeUS) - base[0]) / 1e6), 1), v))
        headers["flight_modes"] = fm
    if "Plane" in fw or "Rover" in fw or "Sub" in fw:
        notes.append(f"{fw}: hover, thrust-split and coast-down analyses assume a multirotor.")
    return df, headers, notes
