"""Synthetic iNav-style flight with known ground truth, in raw log units.

Every quantity the analyser estimates is planted here with a known value so the tests can
check the methods end to end (unit conversion included).
"""

from __future__ import annotations

import numpy as np
import pandas as pd

FS = 1000.0
RHO = 101325.0 / (287.05 * 288.15)

TRUTH = {
    "mass_kg": 1.1, "prop_in": 7.0, "rotors": 4,
    "cda_m2": 0.012, "eta": 0.55, "induced_k": 1.15, "fixed_w": 30.0,
    "r_ohm": 0.025, "clip_a": 50.0,
    "resonance_hz": 92.0,
    "front_minus_rear_pct": -3.0,
    "pitch_i_offset": 22.0,
    "gap_t": 60.0, "gap_ms": 340.0,
    "axes": {"roll": (8.0, 0.5), "pitch": (8.0, 0.7), "yaw": (4.0, 0.9)},   # (fn Hz, zeta)
    "delay_s": 0.008,
}


def _vi(v, vh):
    v2 = v * v
    return np.sqrt((-v2 + np.sqrt(v2 * v2 + 4 * vh ** 4)) / 2.0)


def power_model(v, accel=0.0):
    T = TRUTH
    W = T["mass_kg"] * 9.81
    A = T["rotors"] * np.pi * (T["prop_in"] * 0.0254 / 2) ** 2
    vh = np.sqrt(W / (2 * RHO * A))
    mech = T["induced_k"] * W * _vi(v, vh) + T["fixed_w"] + 0.5 * RHO * T["cda_m2"] * v ** 3 \
        + np.maximum(0, T["mass_kg"] * accel * v)
    return mech / T["eta"]


def _lowpass(x, fc, fs=FS):
    a = np.exp(-2 * np.pi * fc / fs)
    y = np.empty_like(x)
    acc = 0.0
    for k, v in enumerate(x):
        acc = a * acc + (1 - a) * v
        y[k] = acc
    return y


def _second_order(u, fn, zeta, delay, fs=FS):
    wn = 2 * np.pi * fn
    d = int(round(delay * fs))
    u = np.concatenate([np.full(d, u[0]), u[:len(u) - d]])
    x = v = 0.0
    y = np.empty_like(u)
    h = 1.0 / fs
    for k in range(len(u)):
        a = wn * wn * (u[k] - x) - 2 * zeta * wn * v
        v += a * h
        x += v * h
        y[k] = x
    return y


def make_flight(seed: int = 0):
    rng = np.random.default_rng(seed)
    T = TRUTH
    pieces = []                       # (duration, kind, v_start, v_end)
    pieces.append((12, "hover", 0, 0))
    v_prev = 0.0
    for v in (5, 8, 11, 14, 17, 20):
        pieces.append((2, "ramp", v_prev, v))
        pieces.append((8, "cruise", v, v))
        v_prev = v
    pieces.append((4, "coast", 20, None))
    pieces.append((3, "ramp", None, 0))
    pieces.append((4, "hover", 0, 0))
    pieces.append((30, "sticks", 0, 0))
    pieces.append((12, "sweep", 0, 0))                 # climb with rising throttle
    for name in ("punch", "punch", "punch_hard"):
        pieces.append((1, name, 0, 0))
        pieces.append((2, "hover", 0, 0))
    pieces.append((5, "hover", 0, 0))

    speed, kind = [], []
    k_drag = 0.5 * RHO * T["cda_m2"] / T["mass_kg"]
    for dur, kd, v0, v1 in pieces:
        n = int(dur * FS)
        tt = np.arange(n) / FS
        if kd == "coast":
            v = v0 / (1 + k_drag * v0 * tt)
        elif kd == "ramp":
            v0 = speed[-1][-1] if v0 is None else v0
            v = v0 + (v1 - v0) * tt / dur
        else:
            v = np.full(n, float(v0))
        speed.append(v)
        kind += [kd] * n
    speed = np.concatenate(speed)
    kind = np.array(kind)
    N = speed.size
    t = np.arange(N) / FS
    accel = np.gradient(speed, t)

    # ---- power, throttle, motors
    P = power_model(speed, accel)
    P = np.where(kind == "coast", 15.0, P)
    sw = kind == "sweep"
    P = np.where(sw, np.interp(np.cumsum(sw) / max(1, sw.sum()), [0, 1], [90.0, 900.0]), P)
    P = np.where(kind == "punch", 950.0, P)            # below the current-sensor limit
    P = np.where(kind == "punch_hard", 1500.0, P)      # saturates the sensor
    P = P * (1 + 0.08 * _lowpass(rng.standard_normal(N), 3.0) * 3)        # flight turbulence
    p_hover = power_model(np.array([0.0]))[0]
    mavg = np.clip(0.35 * np.sqrt(np.maximum(P, 0) / p_hover), 0.03, 1.0)
    thr = np.where(kind == "coast", 0.0, np.clip(mavg * 1.05, 0, 1))

    # ---- battery: V = Voc(charge) - R I, solved for I
    I = np.zeros(N)
    V = np.zeros(N)
    used_ah = 0.0
    for k in range(N):
        voc = 25.0 - 2.4 * used_ah / 1.6
        disc = max(voc * voc - 4 * T["r_ohm"] * P[k], 0.0)
        I[k] = (voc - np.sqrt(disc)) / (2 * T["r_ohm"])
        V[k] = voc - T["r_ohm"] * I[k]
        used_ah += I[k] / FS / 3600
    I_meas = np.minimum(I + 0.15 * rng.standard_normal(N), T["clip_a"])       # sensor saturates
    V_meas = V + 0.01 * rng.standard_normal(N)

    # ---- rates: setpoint -> closed loop -> gyro
    sp = np.zeros((3, N))
    for ax in range(3):
        base = _lowpass(rng.standard_normal(N), 1.0) * 8                     # small corrections
        steps = np.zeros(N)
        idx = np.flatnonzero(kind == "sticks")
        k = idx[0]
        while k < idx[-1]:
            dur = int(rng.uniform(0.15, 0.5) * FS)
            steps[k:min(k + dur, idx[-1])] = rng.uniform(-450, 450) if ax < 2 else rng.uniform(-250, 250)
            k += dur
        sp[ax] = base + _lowpass(steps, 60.0)
    gy = np.zeros((3, N))
    for ax, name in enumerate(("roll", "pitch", "yaw")):
        fn, z = T["axes"][name]
        gy[ax] = _second_order(sp[ax], fn, z, T["delay_s"])

    phase_m = np.cumsum(2 * np.pi * (60 + 250 * mavg) / FS)
    res = 10.0 * np.sin(2 * np.pi * T["resonance_hz"] * t)
    gr = np.zeros((3, N))
    for ax in range(3):
        gr[ax] = gy[ax] + res + 6.0 * np.sin(phase_m + ax) + 1.5 * rng.standard_normal(N)
    gyf = gy + 0.1 * (gr - gy)                                                # filtered gyro

    # ---- PID terms
    err = sp - gyf
    pterm = 0.4 * err
    iterm = np.zeros((3, N))
    for ax in range(3):
        iterm[ax] = _lowpass(rng.standard_normal(N), 0.3) * 15
    iterm[1] += T["pitch_i_offset"]
    dterm = -0.002 * np.gradient(gr, axis=1) * FS
    fterm = 0.02 * np.gradient(sp, axis=1) * FS / 10

    # ---- motors: base output plus a planted front/rear split (rear works harder)
    split = T["front_minus_rear_pct"] / 100.0                  # front - rear
    pos_front = np.array([0, 1, 0, 1])                          # m0 RR, m1 FR, m2 RL, m3 FL
    motors = np.zeros((4, N))
    for k in range(4):
        motors[k] = mavg + (split / 2 if pos_front[k] else -split / 2) + 0.004 * rng.standard_normal(N)
    motors = np.clip(motors, 0, 1)

    lo, hi = 1080.0, 2000.0
    raw = {
        "loopIteration": np.arange(N), "time": t * 1e6,
        "rcCommand[3]": 1000 + 1000 * thr,
        "vbat": V_meas / 0.01, "amperage": I_meas / 0.01,
        "navVel[0]": speed * 100, "navVel[1]": np.zeros(N),
        "navVel[2]": np.where(sw, 300.0, 0.0) + np.where(np.char.startswith(kind.astype(str), "punch"), 800.0, 0.0)
                     + 2 * rng.standard_normal(N),
        "navPos[2]": 1000 + 2 * rng.standard_normal(N),
        "accVib": np.abs(rng.standard_normal(N)) * 500,
    }
    for k in range(4):
        raw[f"motor[{k}]"] = lo + motors[k] * (hi - lo)
    for ax in range(3):
        raw[f"axisRate[{ax}]"] = sp[ax]
        raw[f"gyroADC[{ax}]"] = gyf[ax]
        raw[f"gyroRaw[{ax}]"] = gr[ax]
        raw[f"axisP[{ax}]"] = pterm[ax]
        raw[f"axisI[{ax}]"] = iterm[ax]
        raw[f"axisF[{ax}]"] = fterm[ax]
        if ax < 2:
            raw[f"axisD[{ax}]"] = dterm[ax]
    df = pd.DataFrame(raw)

    # leading decoder artifact: 2 s of identical, linearly ramping motors
    n_art = int(2 * FS)
    art = pd.DataFrame({c: np.zeros(n_art) for c in df.columns})
    art["time"] = (np.arange(n_art) - n_art) / FS * 1e6
    ramp = np.linspace(0, lo, n_art)
    for k in range(4):
        art[f"motor[{k}]"] = ramp
    art["rcCommand[3]"] = 1000.0
    df = pd.concat([art, df], ignore_index=True)

    # logging gap
    tt = df["time"].to_numpy() / 1e6
    gap = (tt >= T["gap_t"]) & (tt < T["gap_t"] + T["gap_ms"] / 1000)
    df = df[~gap].reset_index(drop=True)

    headers = {"Firmware revision": "synthetic iNav", "motorOutput": [1080, 2000],
               "rollPID": [40, 75, 26, 100], "pitchPID": [44, 82, 29, 110], "yawPID": [45, 80, 0, 100]}
    return df, headers
