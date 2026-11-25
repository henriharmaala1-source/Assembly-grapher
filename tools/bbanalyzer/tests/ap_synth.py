"""Write the synthetic flight (synth.py) as an ArduPilot DataFlash log, binary and text.

Message layouts and logging rates follow ArduCopter 4.x: RATE/GYR/PID at loop rate,
RCOU at 50 Hz, EKF and attitude at 25 Hz, battery and CTUN at 10 Hz. An extra EKF core
with wrong data checks that only the primary core is used.
"""

from __future__ import annotations

import numpy as np

from synth import TRUTH, make_flight

HEAD = (0xA3, 0x95)

# name: (id, format, columns)
MSGS = {
    "PARM": (64, "QNff", "TimeUS,Name,Value,Default"),
    "VER": (65, "QBHBBBBIZ", "TimeUS,BT,BST,Maj,Min,Pat,FWT,GH,FWS"),
    "MSG": (66, "QZ", "TimeUS,Message"),
    "MODE": (67, "QMBB", "TimeUS,Mode,ModeNum,Rsn"),
    "RATE": (70, "Qffffffffffff", "TimeUS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut"),
    "GYR": (71, "QBQfff", "TimeUS,I,SampleUS,GyrX,GyrY,GyrZ"),
    "PIDR": (72, "QfffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod,SRate,Flags"),
    "PIDP": (73, "QfffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod,SRate,Flags"),
    "PIDY": (74, "QfffffffffB", "TimeUS,Tar,Act,Err,P,I,D,FF,Dmod,SRate,Flags"),
    "RCOU": (75, "QHHHHHHHHHHHHHH", "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14"),
    "CTUN": (76, "Qffffffffh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DCRt,CRt"),
    "BAT": (77, "QBfffffcfBB", "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H"),
    "XKF1": (78, "QBccCfffffffccce", "TimeUS,C,Roll,Pitch,Yaw,VN,VE,VD,dPD,PN,PE,PD,GX,GY,GZ,OH"),
    "XKF2": (79, "QBfffff", "TimeUS,C,AX,AY,AZ,VWN,VWE"),
    "ATT": (80, "QccccCCff", "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw"),
    "VIBE": (81, "QBfffI", "TimeUS,IMU,VibeX,VibeY,VibeZ,Clip"),
}
_NP = {"Q": "<u8", "q": "<i8", "B": "u1", "b": "i1", "M": "u1", "H": "<u2", "h": "<i2", "I": "<u4",
       "i": "<i4", "f": "<f4", "d": "<f8", "n": "S4", "N": "S16", "Z": "S64",
       "c": "<i2", "C": "<u2", "e": "<i4", "E": "<u4", "L": "<i4"}
_SCALE = {"c": 100.0, "C": 100.0, "e": 100.0, "E": 100.0, "L": 1e7}

PARAMS = {"FRAME_CLASS": 1, "FRAME_TYPE": 1, "MOT_PWM_MIN": 1000, "MOT_PWM_MAX": 2000,
          "SERVO1_FUNCTION": 33, "SERVO2_FUNCTION": 34, "SERVO3_FUNCTION": 35, "SERVO4_FUNCTION": 36,
          "ATC_RAT_RLL_P": 0.135, "ATC_RAT_RLL_I": 0.135, "ATC_RAT_RLL_D": 0.0036,
          "ATC_RAT_PIT_P": 0.135, "INS_GYRO_FILTER": 40, "SCHED_LOOP_RATE": 1000, "LOG_BITMASK": 180222}
FIRMWARE = "ArduCopter V4.5.7 (2a3dc4b7)"


def _dtype(fmt, cols):
    return np.dtype([(c, _NP[ch]) for ch, c in zip(fmt, cols.split(","))])


def _sample(t, rate):
    """Indices of the base (1 kHz) samples at which a message logged at `rate` Hz is written."""
    step = max(1, int(round(1000.0 / rate)))
    return np.arange(0, t.size, step)


def build_messages(seed: int = 3):
    """{name: list of row dicts as arrays} in physical units, plus the truth for checks."""
    raw, _ = make_flight(seed=seed)
    raw = raw[raw["time"] >= 0].reset_index(drop=True)          # no decoder artifact in DataFlash
    t = raw["time"].to_numpy() / 1e6
    us = (t + 5.0) * 1e6                                          # log starts 5 s after boot
    g = lambda c: raw[c].to_numpy(dtype=float)
    N = t.size
    zeros = np.zeros(N)
    out = {}

    full = np.arange(N)
    out["RATE"] = (full, {"RDes": g("axisRate[0]"), "R": g("gyroADC[0]"), "ROut": zeros,
                          "PDes": g("axisRate[1]"), "P": g("gyroADC[1]"), "POut": zeros,
                          "YDes": g("axisRate[2]"), "Y": g("gyroADC[2]"), "YOut": zeros,
                          "ADes": zeros, "A": zeros, "AOut": zeros})
    out["GYR"] = (full, {"I": zeros, "SampleUS": us, "GyrX": np.deg2rad(g("gyroRaw[0]")),
                         "GyrY": np.deg2rad(g("gyroRaw[1]")), "GyrZ": np.deg2rad(g("gyroRaw[2]"))})
    for i, name in enumerate(("PIDR", "PIDP", "PIDY")):
        d = g(f"axisD[{i}]") if f"axisD[{i}]" in raw else zeros
        out[name] = (full, {"Tar": g(f"axisRate[{i}]"), "Act": g(f"gyroADC[{i}]"), "Err": zeros,
                            "P": g(f"axisP[{i}]"), "I": g(f"axisI[{i}]"), "D": d, "FF": g(f"axisF[{i}]"),
                            "Dmod": zeros + 1, "SRate": zeros, "Flags": zeros})

    # motors: synth order is Betaflight's (m0 RR, m1 FR, m2 RL, m3 FL);
    # ArduCopter Quad X is motor1 FR, motor2 RL, motor3 FL, motor4 RR on outputs 1..4
    mot = [(g(f"motor[{k}]") - 1080.0) / (2000.0 - 1080.0) for k in range(4)]
    pwm = {"C1": mot[1], "C2": mot[2], "C3": mot[3], "C4": mot[0]}
    idx = _sample(t, 50)
    out["RCOU"] = (idx, {f"C{k}": (1000 + 1000 * pwm[f"C{k}"]) if f"C{k}" in pwm else zeros + 1500
                         for k in range(1, 15)})

    thr = (g("rcCommand[3]") - 1000) / 1000
    alt = g("navPos[2]") / 100
    vz = g("navVel[2]") / 100
    out["CTUN"] = (_sample(t, 10), {"ThI": thr, "ABst": zeros, "ThO": thr, "ThH": zeros + 0.35,
                                    "DAlt": alt, "Alt": alt, "BAlt": alt, "DCRt": zeros, "CRt": vz * 100})
    V, I = g("vbat") * 0.01, g("amperage") * 0.01
    out["BAT"] = (_sample(t, 10), {"Inst": zeros, "Volt": V, "VoltR": V, "Curr": I, "CurrTot": zeros,
                                   "EnrgTot": zeros, "Temp": zeros + 30, "Res": zeros + TRUTH["r_ohm"],
                                   "RemPct": zeros + 50, "H": zeros + 1})
    vn, ve = g("navVel[0]") / 100, g("navVel[1]") / 100
    idx = _sample(t, 25)
    k1 = {"Roll": zeros, "Pitch": zeros, "Yaw": zeros, "VN": vn, "VE": ve, "VD": -vz, "dPD": zeros,
          "PN": zeros, "PE": zeros, "PD": -alt, "GX": zeros, "GY": zeros, "GZ": zeros, "OH": zeros}
    out["XKF1"] = (idx, dict(k1, C=zeros))
    out["XKF1#core1"] = (idx, dict(k1, C=zeros + 1, VN=vn + 7.0, VE=ve - 3.0))     # wrong on purpose
    out["XKF2"] = (idx, {"C": zeros, "AX": zeros, "AY": zeros, "AZ": zeros, "VWN": zeros, "VWE": zeros})
    out["ATT"] = (idx, {"DesRoll": zeros, "Roll": zeros + 1.25, "DesPitch": zeros, "Pitch": zeros - 3.5,
                        "DesYaw": zeros, "Yaw": zeros + 123.4, "ErrRP": zeros, "ErrYaw": zeros})
    out["VIBE"] = (_sample(t, 10), {"IMU": zeros, "VibeX": zeros + 3, "VibeY": zeros + 4,
                                    "VibeZ": zeros + 12, "Clip": zeros})
    return t, us, out


def write_bin(path, seed: int = 3, garbage_at: float | None = 30.0):
    t, us, msgs = build_messages(seed)
    chunks = []          # (time_us, order, bytes)

    def fmt_record(mid, fmt, cols, name):
        rec = np.zeros(1, np.dtype([("h1", "u1"), ("h2", "u1"), ("id", "u1"), ("Type", "u1"), ("Length", "u1"),
                                    ("Name", "S4"), ("Format", "S16"), ("Columns", "S64")]))
        rec["h1"], rec["h2"], rec["id"] = HEAD[0], HEAD[1], 128
        rec["Type"], rec["Length"] = mid, 3 + _dtype(fmt, cols).itemsize
        rec["Name"], rec["Format"], rec["Columns"] = name.encode(), fmt.encode(), cols.encode()
        return rec.tobytes()

    head = [fmt_record(128, "BBnNZ", "Type,Length,Name,Format,Columns", "FMT")]
    for name, (mid, fmt, cols) in MSGS.items():
        head.append(fmt_record(mid, fmt, cols, name))

    def records(name, rows: dict, times):
        mid, fmt, cols = MSGS[name]
        dt = np.dtype([("h1", "u1"), ("h2", "u1"), ("id", "u1")] + _dtype(fmt, cols).descr)
        rec = np.zeros(len(times), dt)
        rec["h1"], rec["h2"], rec["id"] = HEAD[0], HEAD[1], mid
        rec["TimeUS"] = times
        for ch, c in zip(fmt[1:], cols.split(",")[1:]):
            v = np.asarray(rows[c])
            if ch in _SCALE:
                v = np.round(v * _SCALE[ch])
            elif ch in "BbHhIiQqM":
                v = np.round(v)
            rec[c] = v
        return rec

    def static(name, rows):
        rec = records(name, rows, np.full(len(next(iter(rows.values()))), int(us[0]) - 1000))
        return [r.tobytes() for r in rec]

    pre = []
    pre += static("VER", {"BT": [1], "BST": [0], "Maj": [4], "Min": [5], "Pat": [7], "FWT": [0], "GH": [0],
                          "FWS": [FIRMWARE.encode()]})
    pre += static("MSG", {"Message": [FIRMWARE.encode(), b"Frame: QUAD/X"]})
    pre += static("PARM", {"Name": [k.encode() for k in PARAMS], "Value": list(PARAMS.values()),
                           "Default": [0.0] * len(PARAMS)})
    pre += static("MODE", {"Mode": [5], "ModeNum": [5], "Rsn": [1]})

    allrec, alltime = [], []
    for key, (idx, rows) in msgs.items():
        name = key.split("#")[0]
        rows = {c: np.asarray(v)[idx] for c, v in rows.items()}
        rec = records(name, rows, us[idx].astype(np.uint64))
        allrec.append(rec.view(np.dtype((np.void, rec.dtype.itemsize))))
        alltime.append(us[idx])
    times = np.concatenate(alltime)
    order = np.argsort(times, kind="stable")
    flat = [r for arr in allrec for r in arr]
    body = [flat[i].tobytes() for i in order]
    if garbage_at is not None:                    # a few corrupt bytes, as after a brown-out write error
        k = int(np.searchsorted(np.sort(times), (garbage_at + 5.0) * 1e6))
        body.insert(k, b"\xa3\x95\xfe\x00\x13\x37garbage")
    data = b"".join(head + pre + body)
    with open(path, "wb") as fh:
        fh.write(data)
    return t


def write_text(path, seed: int = 3, seconds: float = 3.0):
    """mavlogdump-style text log of the first seconds (values in physical units)."""
    t, us, msgs = build_messages(seed)
    lines = ["FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns"]
    for name, (mid, fmt, cols) in MSGS.items():
        lines.append(f"FMT, {mid}, {3 + _dtype(fmt, cols).itemsize}, {name}, {fmt}, {cols}")
    lines.append(f"VER, {int(us[0]) - 1000}, 1, 0, 4, 5, 7, 0, 0, {FIRMWARE}")
    for k, v in PARAMS.items():
        lines.append(f"PARM, {int(us[0]) - 1000}, {k}, {v}, 0")
    rows_t = []
    for key, (idx, rows) in msgs.items():
        name = key.split("#")[0]
        fmt, cols = MSGS[name][1][1:], MSGS[name][2].split(",")[1:]
        idx = idx[t[idx] < seconds]

        def as_logged(ch, v):                     # same quantisation as the binary file
            if ch in _SCALE:
                return round(v * _SCALE[ch]) / _SCALE[ch]
            return float(round(v)) if ch in "BbHhIiQqM" else float(np.float32(v))

        for i in idx:
            vals = ", ".join(f"{as_logged(ch, float(np.asarray(rows[c])[i])):.9g}" for ch, c in zip(fmt, cols))
            rows_t.append((us[i], f"{name}, {int(us[i])}, {vals}"))
    rows_t.sort(key=lambda r: r[0])
    lines += [r[1] for r in rows_t]
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
