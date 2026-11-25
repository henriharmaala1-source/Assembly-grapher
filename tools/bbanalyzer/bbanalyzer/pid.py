"""PID / tune review.

Step response: the closed-loop response from rate setpoint to measured gyro is estimated by
Wiener deconvolution over overlapping windows (the approach popularised by PID-Analyzer),
integrated to a step response and averaged over every window with enough stick input.
That turns a whole flight of ordinary flying into the response to a clean step, which is
far easier to judge than raw traces.

Plus: tracking error, response delay, P/I/D/F contributions, I-term offset (mechanical
bias), D-term noise, and a rule-based scorecard with plain-language suggestions.
The scorecard thresholds are rules of thumb, not physics; they are listed in the report.
"""

from __future__ import annotations

import numpy as np
from scipy.signal.windows import tukey

from .config import Aircraft
from .loader import AXES, Log, contiguous_blocks

RESP_S = 0.5      # length of step response shown
WIN_S = 2.0       # analysis window


def step_response(sp: np.ndarray, gy: np.ndarray, fs: float, blocks, min_input: float = 20.0,
                  min_input_max: float | None = None) -> dict | None:
    n = int(WIN_S * fs)
    hop = n // 2
    m = int(RESP_S * fs)
    w = tukey(n, 0.5)
    nfft = 2 * n
    steps, weights = [], []
    for a, b in blocks:
        for s in range(a, b - n, hop):
            x = sp[s:s + n]
            y = gy[s:s + n]
            if not (np.isfinite(x).all() and np.isfinite(y).all()):
                continue
            amp = np.max(np.abs(x))
            if amp < min_input or (min_input_max is not None and amp > min_input_max):
                continue
            x0 = (x - x.mean()) * w
            y0 = (y - y.mean()) * w
            X = np.fft.rfft(x0, nfft)
            Y = np.fft.rfft(y0, nfft)
            pxx = np.abs(X) ** 2
            H = Y * np.conj(X) / (pxx + 1e-3 * pxx.mean())
            h = np.fft.irfft(H, nfft)[:m]
            st = np.cumsum(h)
            ss = st[int(0.6 * m):].mean()
            if not (0.3 < ss < 2.0):          # reject windows the deconvolution could not resolve
                continue
            steps.append(st)
            weights.append(np.std(x))
    if len(steps) < 3:
        return None
    S = np.array(steps)
    wts = np.array(weights)
    mean = np.average(S, axis=0, weights=wts)
    spread = np.sqrt(np.average((S - mean) ** 2, axis=0, weights=wts))
    t = np.arange(m) / fs
    return {"t": t.tolist(), "step": mean.tolist(), "spread": spread.tolist(), "windows": len(steps),
            **step_metrics(t, mean)}


def step_metrics(t: np.ndarray, s: np.ndarray) -> dict:
    ss = float(s[t >= 0.6 * t[-1]].mean())
    norm = s / ss if ss > 0.2 else s
    try:
        t10 = t[np.flatnonzero(norm >= 0.1)[0]]
        t90 = t[np.flatnonzero(norm >= 0.9)[0]]
        rise = float(t90 - t10)
    except IndexError:
        rise = float("nan")
    peak = float(norm.max())
    i50 = np.flatnonzero(norm >= 0.5)
    t50 = float(t[i50[0]]) if i50.size else float("nan")
    outside = np.flatnonzero(np.abs(norm - 1) > 0.05)
    settle = float(t[outside[-1]]) if outside.size else 0.0
    after = norm[np.argmax(norm):]
    outside_band = after[np.abs(after - 1) > 0.02]      # ignore noise around the setpoint
    crossings = int(np.sum(np.diff(np.sign(outside_band - 1)) != 0))
    return {"steady_state": ss, "rise_ms": rise * 1000, "overshoot_pct": (peak - 1) * 100,
            "settle_ms": settle * 1000, "ringing_crossings": crossings, "t50_ms": t50 * 1000,
            "peak_time_ms": float(t[np.argmax(norm)] * 1000)}


def response_delay_ms(sp: np.ndarray, gy: np.ndarray, fs: float, max_ms: float = 60) -> float | None:
    ok = np.isfinite(sp) & np.isfinite(gy)
    x, y = sp[ok] - np.mean(sp[ok]), gy[ok] - np.mean(gy[ok])
    if x.size < fs or np.std(x) < 5:
        return None
    lags = np.arange(0, int(max_ms / 1000 * fs) + 1)
    c = [np.dot(x[: len(x) - L], y[L:]) for L in lags]
    return float(lags[int(np.argmax(c))] / fs * 1000)


def _worst_events(err: np.ndarray, t: np.ndarray, fs: float, k: int = 2, half_s: float = 0.6):
    e = np.abs(np.nan_to_num(err))
    out = []
    taken = np.zeros_like(e, bool)
    for _ in range(k):
        cand = np.where(taken, 0, e)
        i = int(np.argmax(cand))
        if cand[i] <= 0:
            break
        a, b = max(0, i - int(half_s * fs)), min(len(e), i + int(half_s * fs))
        taken[a:b] = True
        out.append({"t": float(t[i]), "a": a, "b": b, "err": float(err[i])})
    return out


def analyze(log: Log, ac: Aircraft, phases: dict) -> dict:
    df, fs = log.df, log.fs
    t = df["t"].to_numpy()
    flying = df["mavg"].to_numpy() > 0.05 if "mavg" in df else np.ones(len(df), bool)
    blocks = [(a, b) for a, b in contiguous_blocks(t, fs, WIN_S + 0.5) if flying[a:b].mean() > 0.8]
    res: dict = {"available": False, "axes": {}}
    for i, ax in enumerate(AXES):
        if f"sp{i}" not in df or f"gy{i}" not in df:
            continue
        sp, gy = df[f"sp{i}"].to_numpy(), df[f"gy{i}"].to_numpy()
        e = sp - gy
        A: dict = {}
        A["step"] = step_response(sp, gy, fs, blocks)
        A["step_large"] = step_response(sp, gy, fs, blocks, min_input=300.0)
        A["delay_ms"] = response_delay_ms(sp[flying], gy[flying], fs)
        A["err_mean"] = float(np.nanmean(np.abs(e[flying])))
        A["err_p95"] = float(np.nanpercentile(np.abs(e[flying]), 95))
        if np.nanstd(sp[flying]) > 1:
            A["corr"] = float(np.corrcoef(np.nan_to_num(sp[flying]), np.nan_to_num(gy[flying]))[0, 1])
        A["worst"] = _worst_events(np.where(flying, e, 0), t, fs)
        terms = {}
        for term in "pidf":
            c = f"{term}{i}"
            if c in df and df[c].notna().any():
                x = df[c].to_numpy()[flying]
                terms[term.upper()] = {"mean": float(np.nanmean(x)), "std": float(np.nanstd(x)),
                                       "p95_abs": float(np.nanpercentile(np.abs(x), 95))}
        A["terms"] = terms
        if "I" in terms and terms["I"]["p95_abs"] > 0:
            A["i_offset_ratio"] = abs(terms["I"]["mean"]) / terms["I"]["p95_abs"]
        res["axes"][ax] = A
        res["available"] = True
    res["header"] = {k: v for k, v in log.headers.items()
                     if any(s in k.lower() for s in ("pid", "lpf", "rpm", "notch", "dyn", "tpa", "rate", "rc_", "ff",
                                                     "d_min", "antigravity", "looptime", "pid_process",
                                                     "atc_", "ins_gyro", "ins_hntch", "ins_hntc2", "sched_loop"))}
    res["scorecard"] = scorecard(res)
    return res


# --------------------------------------------------------------------------- scorecard

def _item(area, axis, value, status, note):
    return {"area": area, "axis": axis, "value": value, "status": status, "note": note}


def scorecard(res: dict, vib: dict | None = None, motors: dict | None = None) -> list[dict]:
    """Rule-of-thumb review. status: good / check / bad."""
    out = []
    axes = res.get("axes", {})
    for ax, A in axes.items():
        st = A.get("step")
        yaw = ax == "yaw"
        if st:
            ov = st["overshoot_pct"]
            lim = (15, 35) if yaw else (10, 25)
            status = "good" if ov < lim[0] else ("check" if ov < lim[1] else "bad")
            out.append(_item("Overshoot", ax, f"{ov:.0f} %", status,
                             "Overshoots the setpoint: more D (or less P / feedforward) usually tames it."
                             if status != "good" else "Little overshoot."))
            ss = st["steady_state"]
            status = "good" if 0.95 <= ss <= 1.05 else ("check" if 0.85 <= ss <= 1.15 else "bad")
            note = ("Response settles below the setpoint: I-term or feedforward low, or heavy filtering."
                    if ss < 0.95 else ("Settles above setpoint: feedforward or I too high." if ss > 1.05
                                       else "Settles on the setpoint."))
            out.append(_item("Steady state", ax, f"{ss:.2f}", status, note))
            if st["ringing_crossings"] > 3:
                out.append(_item("Ringing", ax, f"{st['ringing_crossings']} crossings", "check",
                                 "Oscillates before settling: P too high or D too low for this airframe."))
        if st and np.isfinite(st.get("t50_ms", np.nan)):
            d = st["t50_ms"]
            lim = (60, 100) if yaw else (40, 70)
            status = "good" if d <= lim[0] else ("check" if d <= lim[1] else "bad")
            out.append(_item("Response time (50 %)", ax, f"{d:.0f} ms", status,
                             "Slow to reach half the commanded rate: heavy filtering, low P/feedforward, or a large, "
                             "heavy airframe. Compare between tunes of the same craft rather than to a fixed number."
                             if status != "good" else "Responsive."))
        r = A.get("i_offset_ratio")
        if r is not None:
            status = "good" if r < 0.3 else ("check" if r < 0.5 else "bad")
            out.append(_item("I-term offset", ax, f"{r:.2f}", status,
                             "I-term holds a constant correction: mechanical bias (CG offset, bent arm, "
                             "motor/prop mismatch). Fix mechanically, not with PIDs."
                             if status != "good" else "No standing correction."))
    rp = [axes[a]["step"]["rise_ms"] for a in ("roll", "pitch") if a in axes and axes[a].get("step")]
    if len(rp) == 2 and min(rp) > 0:
        diff = abs(rp[0] - rp[1]) / min(rp)
        if diff > 0.4:
            out.append(_item("Roll/pitch match", "roll+pitch", f"{diff * 100:.0f} % apart", "check",
                             "Roll and pitch respond at clearly different speeds: tune them separately "
                             "(different inertia), or check for a stiffness problem."))
    if vib:
        for ax, V in vib.get("axes", {}).items():
            fr = V.get("dterm_hf_fraction")
            if fr is not None:
                status = "good" if fr < 0.5 else ("check" if fr < 0.7 else "bad")
                out.append(_item("D-term noise", ax, f"{fr * 100:.0f} % above 100 Hz", status,
                                 "D-term dominated by high-frequency noise: hot motors. RPM filter "
                                 "(bidirectional DShot) or more D filtering." if status != "good"
                                 else "D-term mostly real motion."))
        for pk in vib.get("peak_classes", []):
            if pk["class"].startswith("structural"):
                out.append(_item("Frame resonance", "all", f"{pk['freq_hz']:.0f} Hz", "check",
                                 "Fixed-frequency peak across throttle: arm/frame mode. Stiffen, damp, or notch it; "
                                 "keep hover RPM lines away from it."))
    if motors and motors.get("available"):
        s = motors.get("saturation_pct", 0)
        status = "good" if s < 2 else ("check" if s < 5 else "bad")
        out.append(_item("Motor saturation", "all", f"{s:.1f} % of flight", status,
                         "Motors often at max: little control margin (low thrust-to-weight or aggressive inputs)."
                         if status != "good" else "Plenty of control margin."))
        for name, v in motors.get("splits_centred", {}).items():
            mag = abs(v["mean_pct"])
            status = "good" if mag < 2 else ("check" if mag < 5 else "bad")
            label = {"front_minus_rear": "Front/rear split", "left_minus_right": "Left/right split",
                     "diagM1M4_minus_diagM2M3": "Diagonal (yaw) split"}[name]
            out.append(_item(label, "motors", f"{v['mean_pct']:+.1f} %", status,
                             "Persistent thrust difference with sticks centred: CG offset or mechanical "
                             "asymmetry on that axis." if status != "good" else "Balanced."))
    return out
