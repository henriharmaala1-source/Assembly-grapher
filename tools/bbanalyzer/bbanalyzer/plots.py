"""Matplotlib figures, returned as base64 PNG strings for the HTML report."""

from __future__ import annotations

import base64
import io

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .loader import AXES  # noqa: E402

PHASE_COLORS = {"hover": "#4c9be8", "cruise": "#3fb27f", "coast": "#e8a33d", "punch": "#d9534f"}
AX_COLORS = {"roll": "#d9534f", "pitch": "#3fb27f", "yaw": "#4c9be8"}
plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})


def _png(fig) -> str:
    buf = io.BytesIO()
    fig.tight_layout()
    fig.savefig(buf, format="png")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def _decimate(n: int, target: int = 6000) -> slice:
    return slice(None, None, max(1, n // target))


def _shade(ax, t, phases):
    for name, segs in phases["segments"].items():
        for a, b in segs:
            ax.axvspan(t[a], t[min(b, len(t) - 1)], color=PHASE_COLORS[name], alpha=0.12, lw=0)


def overview(log, phases) -> str:
    df = log.df
    t = df["t"].to_numpy()
    d = _decimate(len(t))
    rows = [r for r in (("throttle", "mavg"), ("speed", "alt"), ("V", "I")) if any(c in df for c in r)]
    fig, axes = plt.subplots(len(rows), 1, figsize=(10, 2.1 * len(rows)), sharex=True)
    axes = np.atleast_1d(axes)
    labels = {"throttle": "throttle", "mavg": "motor avg", "speed": "speed [m/s]", "alt": "altitude [m]",
              "V": "voltage [V]", "I": "current [A]"}
    for ax, (a, b) in zip(axes, rows):
        _shade(ax, t, phases)
        if a in df:
            ax.plot(t[d], df[a].to_numpy()[d], lw=0.7, color="#333", label=labels[a])
            ax.set_ylabel(labels[a])
        if b in df:
            ax2 = ax.twinx()
            ax2.plot(t[d], df[b].to_numpy()[d], lw=0.7, color="#e8743d", label=labels[b])
            ax2.set_ylabel(labels[b], color="#e8743d")
            ax2.grid(False)
    for name, c in PHASE_COLORS.items():
        axes[0].plot([], [], color=c, lw=6, alpha=0.4, label=name)
    axes[0].legend(loc="upper right", ncol=6, fontsize=7)
    axes[-1].set_xlabel("time [s]")
    if log.gaps is not None:
        for g in log.gaps.itertuples():
            for ax in axes:
                ax.axvline(g.t, color="red", lw=1, ls="--")
    return _png(fig)


def power_curve(aero_res) -> str | None:
    pc = aero_res.get("curve", {})
    if not pc.get("bins") or len(pc["bins"]["v"]) < 2:
        return None
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4))
    v, p = np.array(pc["bins"]["v"]), np.array(pc["bins"]["p"])
    a1.scatter(v, p, s=10 + 30 * np.array(pc["bins"]["n"]) / max(pc["bins"]["n"]), color="#333", label="steady-flight bins")
    a2.scatter(v[v >= 3], p[v >= 3] / v[v >= 3] / 3.6, color="#333", s=18)
    if "model" in pc:
        gv, gp = np.array(pc["model"]["grid_v"]), np.array(pc["model"]["grid_p"])
        a1.plot(gv, gp, color="#3fb27f", label="fitted model")
        m = gv >= 2
        a2.plot(gv[m], gp[m] / gv[m] / 3.6, color="#3fb27f")
        a1.axvline(pc["best_endurance_speed"], color="#4c9be8", ls="--", label=f"best endurance {pc['best_endurance_speed']:.1f} m/s")
        a2.axvline(pc["best_range_speed"], color="#e8a33d", ls="--", label=f"best range {pc['best_range_speed']:.1f} m/s")
        a2.legend(fontsize=7)
    a1.set_xlabel("airspeed [m/s]"); a1.set_ylabel("electrical power [W]"); a1.legend(fontsize=7)
    a2.set_xlabel("airspeed [m/s]"); a2.set_ylabel("energy [Wh/km]")
    return _png(fig)


def coast(aero_res) -> str | None:
    cd = aero_res.get("coast", {})
    if not cd.get("available"):
        return None
    v, D = map(np.array, cd["points"])
    fig, ax = plt.subplots(figsize=(5, 3.2))
    ax.scatter(v, D, s=4, alpha=0.4, color="#333")
    g = np.linspace(0, v.max() * 1.05, 50)
    ax.plot(g, 0.5 * aero_res["rho"] * cd["cda_m2"] * g ** 2, color="#e8a33d", label=f"CdA = {cd['cda_m2']:.4f} m²")
    ax.set_xlabel("airspeed [m/s]"); ax.set_ylabel("drag [N]"); ax.legend()
    ax.set_title("Coast-down drag" + (" (lower bound: throttle above idle)" if cd["lower_bound"] else ""), fontsize=9)
    return _png(fig)


def battery(log) -> str | None:
    df = log.df
    if "V" not in df or "I" not in df:
        return None
    d = _decimate(len(df), 8000)
    fig, ax = plt.subplots(figsize=(5, 3.2))
    sc = ax.scatter(df["I"].to_numpy()[d], df["V"].to_numpy()[d], c=df["t"].to_numpy()[d], s=2, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="time [s]")
    ax.set_xlabel("current [A]"); ax.set_ylabel("voltage [V]"); ax.set_title("Voltage sag vs load", fontsize=9)
    return _png(fig)


def steps(pid_res) -> str | None:
    axes_res = pid_res.get("axes", {})
    if not any(a.get("step") for a in axes_res.values()):
        return None
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.3), sharey=True)
    for ax, name in zip(axs, AXES):
        A = axes_res.get(name, {})
        st = A.get("step")
        ax.axhline(1, color="#999", lw=0.8)
        ax.axhspan(0.95, 1.05, color="#3fb27f", alpha=0.08)
        if st:
            t = np.array(st["t"]) * 1000
            s, sp = np.array(st["step"]), np.array(st["spread"])
            ax.fill_between(t, s - sp, s + sp, color=AX_COLORS[name], alpha=0.15)
            ax.plot(t, s, color=AX_COLORS[name], lw=1.6, label=f"all inputs ({st['windows']} windows)")
            if A.get("step_large"):
                sl = A["step_large"]
                ax.plot(np.array(sl["t"]) * 1000, sl["step"], color=AX_COLORS[name], lw=1, ls="--",
                        label="large inputs (>300 °/s)")
            ax.text(0.97, 0.05, f"rise {st['rise_ms']:.0f} ms\novershoot {st['overshoot_pct']:.0f} %\n"
                                f"settle {st['settle_ms']:.0f} ms\nsteady {st['steady_state']:.2f}",
                    transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
                    bbox=dict(boxstyle="round", fc="white", ec="#ccc"))
            ax.legend(fontsize=7, loc="upper right")
        else:
            ax.text(0.5, 0.5, "not enough stick input", ha="center", transform=ax.transAxes)
        ax.set_title(name.capitalize()); ax.set_xlabel("ms after step")
    axs[0].set_ylabel("response / setpoint")
    axs[0].set_ylim(0, 1.6)
    return _png(fig)


def worst_events(log, pid_res) -> str | None:
    df = log.df
    t = df["t"].to_numpy()
    cols = [(n, A["worst"][0]) for n, A in pid_res.get("axes", {}).items() if A.get("worst")]
    if not cols:
        return None
    fig, axs = plt.subplots(2, len(cols), figsize=(3.8 * len(cols), 4.6), sharex="col")
    axs = np.atleast_2d(axs)
    if axs.shape[0] != 2:
        axs = axs.T
    for j, (name, ev) in enumerate(cols):
        i = AXES.index(name)
        a, b = ev["a"], ev["b"]
        tt = t[a:b]
        top, bot = axs[0, j], axs[1, j]
        top.plot(tt, df[f"sp{i}"].to_numpy()[a:b], color="#333", lw=1.2, label="setpoint")
        top.plot(tt, df[f"gy{i}"].to_numpy()[a:b], color=AX_COLORS[name], lw=1, label="gyro")
        top.set_title(f"{name}: worst tracking error at t={ev['t']:.1f} s", fontsize=8)
        top.legend(fontsize=7); top.set_ylabel("°/s")
        for term, c in (("p", "#d9534f"), ("i", "#e8a33d"), ("d", "#4c9be8"), ("f", "#8e6bbf")):
            col = f"{term}{i}"
            if col in df:
                bot.plot(tt, df[col].to_numpy()[a:b], color=c, lw=0.9, label=term.upper())
        bot.legend(fontsize=7, ncol=4); bot.set_xlabel("time [s]"); bot.set_ylabel("PID term output")
    return _png(fig)


def term_bars(pid_res) -> str | None:
    axes_res = pid_res.get("axes", {})
    if not axes_res:
        return None
    fig, axs = plt.subplots(1, 3, figsize=(10, 2.8), sharey=True)
    for ax, name in zip(axs, AXES):
        terms = axes_res.get(name, {}).get("terms", {})
        keys = list(terms)
        if not keys:
            continue
        mean = [terms[k]["mean"] for k in keys]
        p95 = [terms[k]["p95_abs"] for k in keys]
        x = np.arange(len(keys))
        ax.bar(x - 0.2, p95, 0.4, color="#ccc", label="typical size (p95 |x|)")
        ax.bar(x + 0.2, mean, 0.4, color=AX_COLORS[name], label="mean (standing offset)")
        ax.set_xticks(x, keys); ax.set_title(name.capitalize()); ax.axhline(0, color="#333", lw=0.6)
    axs[0].legend(fontsize=7)
    return _png(fig)


def spectra(vib) -> str | None:
    axes_res = vib.get("axes", {})
    if not axes_res:
        return None
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.1), sharey=True)
    for ax, name in zip(axs, AXES):
        V = axes_res.get(name)
        if not V:
            continue
        f = np.array(V["f"])
        ax.semilogy(f, V["raw"], color="#999", lw=0.8, label="raw gyro")
        if "filt" in V:
            ax.semilogy(f, V["filt"], color=AX_COLORS[name], lw=1, label="filtered gyro")
        for p in V.get("peaks_hz", [])[:3]:
            ax.axvline(p, color="#e8a33d", lw=0.7, ls=":")
        ax.set_title(name.capitalize()); ax.set_xlabel("Hz"); ax.legend(fontsize=7)
    axs[0].set_ylabel("PSD [(°/s)²/Hz]")
    return _png(fig)


def noise_map(vib, ac) -> str | None:
    m = vib.get("map")
    if not m:
        return None
    f = np.array(m["f"]); x = np.array(m["x"]); db = np.array(m["db"])
    fig, ax = plt.subplots(figsize=(10, 3.8))
    sel = f <= f.max()
    pc = ax.pcolormesh(x, f[sel], db[sel], shading="nearest", cmap="magma")
    fig.colorbar(pc, ax=ax, label="dB")
    for pk in vib.get("peak_classes", []):
        style = "-" if pk["class"].startswith("structural") else "--"
        tr = np.array(pk["track_hz"], dtype=float)
        ok = np.isfinite(tr)
        if not ok.any():
            continue
        ax.plot(x[ok], tr[ok], style, color="cyan", lw=1.2, marker="o", ms=3)
        last = np.flatnonzero(ok)[-1]
        ax.annotate(f"{pk['freq_hz']:.0f} Hz {pk['class'].split(' ')[0]}", (x[last], tr[last]),
                    xytext=(-4, 6), textcoords="offset points", ha="right", color="cyan", fontsize=7)
    if m["xlabel"] == "motor RPM":
        for k, lbl in ((1, "1×"), (2, "2×"), (ac.blades, f"{ac.blades}× blade pass")):
            ax.plot(x, k * x / 60.0, color="white", lw=0.8, ls=":")
            ax.text(x[0], k * x[0] / 60.0, lbl, color="white", fontsize=7)
    ax.set_xlabel(m["xlabel"]); ax.set_ylabel("frequency [Hz]")
    ax.set_title("Raw gyro noise vs " + m["xlabel"] + " (solid = fixed-frequency/structural, dashed = moves with load)", fontsize=9)
    return _png(fig)


def motor_bars(mot) -> str | None:
    if not mot.get("available"):
        return None
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 2.8))
    names = list(mot["mean_pct"])
    a1.bar([f"{n}\n{mot['positions'][n]}" for n in names], [mot["mean_pct"][n] for n in names], color="#4c9be8")
    a1.set_ylabel("mean output [%]"); a1.set_title("Motor outputs in flight", fontsize=9)
    s = mot.get("splits_centred", {})
    if s:
        lab = {"front_minus_rear": "front − rear", "left_minus_right": "left − right",
               "diagM1M4_minus_diagM2M3": "diag M1M4 − M2M3"}
        vals = [s[k]["mean_pct"] for k in s]
        a2.bar([lab[k] for k in s], vals, color=["#d9534f" if abs(v) >= 2 else "#3fb27f" for v in vals])
        a2.axhline(0, color="#333", lw=0.6)
        a2.set_ylabel("split [% of motor range]"); a2.set_title("Thrust splits with sticks centred", fontsize=9)
    return _png(fig)
