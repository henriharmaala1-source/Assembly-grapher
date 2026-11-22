"""Single-file HTML report (figures embedded) plus a JSON metrics file."""

from __future__ import annotations

import html
from datetime import datetime

from . import plots

CSS = """
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#f6f7f9;color:#1d2330}
main{max-width:1150px;margin:auto;padding:24px 20px 60px}
h1{font-size:22px;margin:0 0 4px} h2{font-size:17px;margin:34px 0 8px;border-bottom:2px solid #dde1e7;padding-bottom:4px}
.sub{color:#5b6475;font-size:13px}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px;margin:16px 0}
.card{background:#fff;border:1px solid #e1e5eb;border-radius:8px;padding:10px 12px}
.card .k{font-size:11px;color:#5b6475;text-transform:uppercase;letter-spacing:.03em}
.card .v{font-size:19px;font-weight:600;margin-top:2px}
table{border-collapse:collapse;width:100%;background:#fff;font-size:13px;margin:8px 0}
th,td{border:1px solid #e1e5eb;padding:5px 8px;text-align:left;vertical-align:top}
th{background:#eef1f5}
img{max-width:100%;background:#fff;border:1px solid #e1e5eb;border-radius:6px;margin:6px 0}
.badge{display:inline-block;padding:1px 8px;border-radius:10px;font-size:11px;font-weight:600;color:#fff}
.good{background:#3fb27f}.check{background:#e8a33d}.bad{background:#d9534f}
.note{background:#fff8e6;border:1px solid #f1d9a3;border-radius:6px;padding:8px 12px;margin:8px 0;font-size:13px}
.muted{color:#5b6475;font-size:12px}
"""


def _f(x, fmt="{:.1f}", none="–"):
    try:
        return none if x is None else fmt.format(x)
    except (TypeError, ValueError):
        return none


def _img(b64):
    return f'<img src="data:image/png;base64,{b64}">' if b64 else ""


def _card(k, v):
    return f'<div class="card"><div class="k">{html.escape(k)}</div><div class="v">{v}</div></div>'


def _table(rows, head):
    h = "".join(f"<th>{html.escape(c)}</th>" for c in head)
    body = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows)
    return f"<table><tr>{h}</tr>{body}</table>"


def _unavailable(sec):
    return f'<p class="muted">Not available: {html.escape(str(sec.get("reason", "missing data")))}</p>'


def build(log, res, phases, ac) -> str:
    pw, ae, mo, vi, pi = (res.get(k, {}) for k in ("power", "aero", "motors", "vibration", "pid"))
    integ = res["integrity"]
    parts = [f"<!doctype html><html><head><meta charset='utf-8'><title>Blackbox analysis – {html.escape(log.source)}</title>"
             f"<style>{CSS}</style></head><body><main>"]
    parts.append(f"<h1>Blackbox analysis</h1><div class='sub'>{html.escape(log.source)} · {html.escape(log.firmware)} · "
                 f"{integ['duration_s']:.0f} s analysed at {integ['sample_rate_hz']:.0f} Hz · generated "
                 f"{datetime.now():%Y-%m-%d %H:%M}</div>")

    # ---- headline cards
    cards = []
    if pw.get("available"):
        cards += [_card("Energy used", f"{pw['energy_wh']:.1f} Wh"), _card("Charge", f"{pw['charge_mah']:.0f} mAh"),
                  _card("Peak power", f"{pw['p_peak']:.0f} W"), _card("Peak current", f"{pw['i_peak']:.0f} A"),
                  _card("Min cell voltage", f"{pw['v_min_cell']:.2f} V")]
        if pw.get("wh_per_km"):
            cards.append(_card("Energy per km", f"{pw['wh_per_km']:.2f} Wh/km"))
        ir = pw.get("internal_resistance")
        if ir:
            cards.append(_card("Pack + wiring R", f"{ir['r_mohm']:.0f} mΩ"))
        if pw.get("hover", {}).get("g_per_w"):
            cards.append(_card("Hover efficiency", f"{pw['hover']['g_per_w']:.1f} g/W"))
    pc = ae.get("curve", {}) if ae.get("available") else {}
    if pc.get("best_range_speed"):
        cards.append(_card("Best range speed", f"{pc['best_range_speed']:.1f} m/s"))
    cd = ae.get("coast", {}) if ae.get("available") else {}
    if cd.get("available"):
        cards.append(_card("Drag area (coast)", f"{cd['cda_m2']:.4f} m²" + (" ≥" if cd["lower_bound"] else "")))
    parts.append("<div class='cards'>" + "".join(cards) + "</div>")

    # ---- integrity
    parts.append("<h2>Log integrity</h2>")
    for n in integ["notes"]:
        parts.append(f"<div class='note'>{html.escape(n)}</div>")
    gaps = integ["gaps"]
    if gaps:
        parts.append(f"<div class='note'><b>{len(gaps)} logging gap(s)</b> – the flight controller stopped logging "
                     "briefly. Gaps at ordinary load (not during punches) point at a supply/connection interruption "
                     "rather than a current brownout.</div>")
        parts.append(_table([[_f(g['t']), _f(g['gap_ms'], '{:.0f}')] for g in gaps], ["time [s]", "gap [ms]"]))
    else:
        parts.append("<p>No logging gaps.</p>")
    parts.append(f"<p class='muted'>Phases found (s): " + ", ".join(f"{k} {v}" for k, v in res['phases']['seconds'].items()) + "</p>")
    parts.append(_img(plots.overview(log, phases)))

    # ---- scorecard
    parts.append("<h2>Tune &amp; airframe scorecard</h2><p class='muted'>Rule-of-thumb review – thresholds are typical "
                 "values for multirotors, not hard limits. Start with the red items.</p>")
    sc = sorted(res.get("scorecard", []), key=lambda r: {"bad": 0, "check": 1, "good": 2}[r["status"]])
    parts.append(_table([[html.escape(r["area"]), html.escape(r["axis"]), html.escape(str(r["value"])),
                          f"<span class='badge {r['status']}'>{r['status']}</span>", html.escape(r["note"])]
                         for r in sc], ["area", "axis", "value", "status", "meaning / suggestion"]))

    # ---- PID
    parts.append("<h2>PID response</h2>")
    if pi.get("available"):
        parts.append("<p>Step response reconstructed from the whole flight (Wiener deconvolution of setpoint → gyro). "
                     "Ideal: fast rise, small overshoot, settles inside the green band at 1.0. Shaded = spread between windows; "
                     "a wide spread means the response depends on conditions (throttle, battery, input size).</p>")
        parts.append(_img(plots.steps(pi)))
        rows = []
        for ax, A in pi["axes"].items():
            st = A.get("step") or {}
            rows.append([ax, _f(st.get("rise_ms"), "{:.0f}"), _f(st.get("overshoot_pct"), "{:.0f}"),
                         _f(st.get("settle_ms"), "{:.0f}"), _f(st.get("steady_state"), "{:.2f}"),
                         _f(A.get("delay_ms"), "{:.0f}"), _f(A.get("err_mean")), _f(A.get("err_p95")),
                         _f(A.get("corr"), "{:.3f}"), _f(A.get("i_offset_ratio"), "{:.2f}")])
        parts.append(_table(rows, ["axis", "rise ms", "overshoot %", "settle ms", "steady", "delay ms",
                                   "mean |err| °/s", "p95 |err| °/s", "corr", "I offset ratio"]))
        parts.append("<p>PID term contributions. A mean far from zero on the <b>I</b> term is a standing correction – "
                     "the aircraft needs constant help on that axis, which is a mechanical issue.</p>")
        parts.append(_img(plots.term_bars(pi)))
        parts.append("<p>Worst tracking moments, with what each term was doing:</p>")
        parts.append(_img(plots.worst_events(log, pi)))
        hdr = pi.get("header", {})
        if hdr:
            parts.append("<details><summary>Tune settings from the log header</summary>" +
                         _table([[html.escape(k), html.escape(str(v))] for k, v in sorted(hdr.items())], ["setting", "value"]) +
                         "</details>")
    else:
        parts.append(_unavailable(pi))

    # ---- vibration
    parts.append("<h2>Vibration &amp; filtering</h2>")
    if vi.get("available"):
        parts.append(_img(plots.spectra(vi)))
        rows = [[ax, _f(V.get("rms_raw_hf")), _f(V.get("rms_filt_hf")), _f(V.get("filter_attenuation_db")),
                 ", ".join(f"{p:.0f}" for p in V.get("peaks_hz", [])), _f(100 * V["dterm_hf_fraction"] if V.get("dterm_hf_fraction") is not None else None, "{:.0f}")]
                for ax, V in vi["axes"].items()]
        parts.append(_table(rows, ["axis", "raw noise >20 Hz (°/s rms)", "filtered", "attenuation dB",
                                   "peaks Hz", "D-term energy >100 Hz %"]))
        parts.append(_img(plots.noise_map(vi, ac)))
        pk = vi.get("peak_classes", [])
        if pk:
            parts.append(_table([[_f(p["freq_hz"], "{:.0f}"), html.escape(p["class"]), _f(p["variation_pct"]),
                                  _f(p["corr_with_x"], "{:.2f}"), _f(p.get("harmonic"), "{:.2f}")] for p in pk],
                                ["peak Hz", "classification", "freq variation %", "corr with load", "× rotation (RPM logs)"]))
    else:
        parts.append(_unavailable(vi))

    # ---- motors
    parts.append("<h2>Motors</h2>")
    if mo.get("available"):
        parts.append(_img(plots.motor_bars(mo)))
        parts.append(f"<p class='muted'>Motor order assumed Betaflight/iNav Quad X: "
                     + ", ".join(f"{k} = {v}" for k, v in mo["positions"].items())
                     + f". Splits measured over {mo['centred_seconds']:.0f} s with sticks centred. "
                     f"Saturation {mo['saturation_pct']:.1f} % of flight; motor-output energy above 100 Hz "
                     f"{_f(100 * mo.get('motor_hf_fraction', float('nan')), '{:.0f}')} %.</p>")
    else:
        parts.append(_unavailable(mo))

    # ---- power & battery
    parts.append("<h2>Power &amp; battery</h2>")
    if pw.get("available"):
        rows = [["Voltage start → end", f"{pw['v_start']:.2f} → {pw['v_end']:.2f} V ({pw['cells']}S)"],
                ["Minimum under load", f"{pw['v_min']:.2f} V ({pw['v_min_cell']:.2f} V/cell)"],
                ["Current mean / median / peak", f"{pw['i_mean']:.1f} / {pw['i_median']:.1f} / {pw['i_peak']:.1f} A"],
                ["Power mean / peak", f"{pw['p_mean']:.0f} / {pw['p_peak']:.0f} W (peak at {pw['p_peak_at']['V']:.1f} V × {pw['p_peak_at']['I']:.0f} A)"],
                ["Time above 25 / 50 / 100 A", " / ".join(f"{pw['time_above_a'][k]:.1f} s" for k in ("25", "50", "100"))]]
        if "c_peak" in pw:
            rows.append(["C-rate mean / peak", f"{pw['c_mean']:.1f} C / {pw['c_peak']:.0f} C ({pw['capacity_used_pct']:.0f} % of pack used)"])
        ir = pw.get("internal_resistance")
        if ir:
            extra = f"; FC estimate {pw['fc_impedance_mohm']:.0f} mΩ" if pw.get("fc_impedance_mohm") else ""
            rows.append(["Pack + wiring resistance", f"{ir['r_mohm']:.1f} mΩ (fit R² {ir['r2']:.2f}, {ir['points']} points{extra}). "
                         f"Includes connector and solder joints – compare packs on the same aircraft."])
        cl = pw["clipping"]
        if cl.get("suspect"):
            rows.append(["Current sensor", f"<span class='badge check'>check</span> {cl['flat_tops']} flat tops at "
                         f"{cl['max_a']:.1f} A – likely sensor saturation; real peaks may be higher."])
        h = pw.get("hover")
        if h:
            rows.append(["Hover", f"{h['power_w']:.0f} W, {h['current_a']:.1f} A, motor output "
                         f"{_f(100 * h.get('motor_output', float('nan')), '{:.0f}')} %"
                         + (f", {h['g_per_w']:.1f} g/W" if h.get("g_per_w") else "")
                         + (f", thrust-to-weight ≈ {h['twr_estimate']:.1f} (rough, from hover output)" if h.get("twr_estimate") else "")])
        if pw.get("wh_per_km"):
            rows.append(["Energy per km (whole flight / steady cruise)", f"{pw['wh_per_km']:.2f} / {_f(pw.get('cruise_wh_per_km'), '{:.2f}')} Wh/km"])
        parts.append(_table(rows, ["metric", "value"]))
        parts.append(_img(plots.battery(log)))
    else:
        parts.append(_unavailable(pw))

    # ---- aero
    parts.append("<h2>Efficiency &amp; air resistance</h2>")
    if ae.get("available"):
        parts.append(f"<p class='muted'>Airspeed source: {html.escape(ae['airspeed_source'])}. Air density {ae['rho']:.3f} kg/m³.</p>")
        if pc.get("available"):
            parts.append(_img(plots.power_curve(ae)))
            m = pc.get("model")
            rows = []
            if m:
                rows += [["Best endurance speed (minimum power)", f"{pc['best_endurance_speed']:.1f} m/s · {pc['min_power_w']:.0f} W"],
                         ["Best range speed (minimum energy per km)", f"{pc['best_range_speed']:.1f} m/s · {pc['wh_per_km_at_best']:.2f} Wh/km"],
                         ["Electrical drag area CdA/η", f"{m['cda_electrical_m2']:.4f} m²"],
                         [f"Drag area CdA (assuming η = {m['eta_assumed']:.2f})", f"{m['cda_m2_assumed_eta']:.4f} m²"],
                         ["Model fit R²", f"{m['r2']:.3f}"]]
                if m.get("note"):
                    rows.append(["Note", html.escape(m["note"])])
            elif pc.get("best_range_speed_bins"):
                rows.append(["Best range speed (from data bins)", f"{pc['best_range_speed_bins']:.1f} m/s · {pc['wh_per_km_at_best_bins']:.2f} Wh/km"])
                rows.append(["Model", "give --mass and --prop to fit the physics model and get drag area"])
            if pc.get("reason"):
                rows.append(["Note", html.escape(pc["reason"])])
            parts.append(_table(rows, ["metric", "value"]))
        else:
            parts.append(_unavailable(pc))
        if cd.get("available"):
            parts.append(_img(plots.coast(ae)))
            parts.append(_table([[_f(s["t"]), _f(s["v_mean"]), _f(s["drag_n"], "{:.2f}"), _f(s["cda_m2"], "{:.4f}"),
                                  _f(100 * s["throttle"], "{:.0f}")] for s in cd["segments"]],
                                ["time s", "mean speed m/s", "drag N", "CdA m²", "throttle %"]))
        else:
            parts.append(f"<p class='muted'>Coast-down drag: {html.escape(str(cd.get('reason', 'not available')))}</p>")
        if ae.get("eta_measured"):
            parts.append(f"<div class='note'>Coast-down and power-curve drag combined give a measured drag-work "
                         f"efficiency η ≈ {ae['eta_measured']:.2f}.</div>")
    else:
        parts.append(_unavailable(ae))

    # ---- method notes
    parts.append("<h2>Methods &amp; assumptions</h2><ul class='muted'>"
                 "<li>Units: vbat ×0.01 V, amperage ×0.01 A, navVel/BaroAlt in cm, attitude in 0.1°, throttle 1000–2000.</li>"
                 "<li>Drag, coast-down: D = −(m·v·dv/dt + m·g·dh/dt)/v during throttle ≤ coast threshold; a lower bound if throttle is above idle.</li>"
                 "<li>Power curve: P = a·W·v<sub>i</sub>(v) + b + c·v³ fitted to steady level flight; v<sub>i</sub> from momentum theory; CdA = 2cη/ρ.</li>"
                 "<li>Step response: Wiener deconvolution over 2 s windows with 50 % overlap, averaged over windows with ≥ 20 °/s input.</li>"
                 "<li>Battery resistance: regression of fast voltage changes on fast current changes after removing the 2 s trend; includes wiring and connectors.</li>"
                 "<li>Peak classification: dominant peaks tracked across load bins; < 8 % frequency variation = structural.</li>"
                 "</ul></main></body></html>")
    return "".join(parts)
