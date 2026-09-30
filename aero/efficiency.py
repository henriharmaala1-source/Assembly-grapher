#!/usr/bin/env python3
"""Drag and efficiency study for the Kipinä airframe: baseline against optimized.

    pip install -r aero/requirements.txt
    python3 aero/efficiency.py              # writes aero/study/study.json and REPORT.md
    python3 aero/efficiency.py --n-crit 5   # rougher surface / more turbulent air

Two levels:

  1. The wing section (aero/airfoil.py): NeuralFoil optimizes the airfoil for
     the lift and Reynolds numbers the plane flies at, within what the printed
     wing needs (spars, servo, trailing edge, hinge) and what the plane needs
     (maximum lift, pitching moment). It starts from four known airfoils and
     keeps the best result.
  2. The whole plane: the wing's profile drag from NeuralFoil, the induced
     drag, and every other part from the drag build-up in aero/models/airframe.py,
     across the speed range. It compares three versions: as built, with the
     optimized airfoil, and with the airfoil plus the detail changes the
     drag optimizer (aero/optimize.py) finds.

aero/study/index.html shows the comparison; aero/study/kipina-opt.dat has the
optimized airfoil's coordinates (Selig format, for XFLR5 or the CAD).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import airfoil as A  # noqa: E402
import optimize as O  # noqa: E402
from models import airframe as AF  # noqa: E402

D = AF.D
OUT = HERE / "study"
STARTS = ["naca4412", "sd7062", "sd7037", "e387"]     # where the optimizer starts from
REFERENCES = ["naca4412", "sd7062", "sd7037", "e387", "s7075"]
PANEL_SHELL_G = 2 * 10.8 + 2 * 2.8   # wing panels and ailerons: one-wall shells, mass follows the perimeter
G = 9.81


def conditions():
    """Wing size, weight and operating points of the plane as built."""
    p, k = AF.BASE, AF.KIT
    L, auw, _ = D.evaluate(p, k)
    mass = auw * AF.MASS_FACTOR
    w = mass / 1000 * G
    s = L.area * 1e-6
    c = L.chord / 1000
    stall = math.sqrt(2 * w / (1.225 * s * p.cl_max))
    re = lambda v: v * c / 1.46e-5  # noqa: E731
    cl = lambda v: w / (0.5 * 1.225 * v * v * s)  # noqa: E731
    v_loiter, v_cruise, v_fast = round(1.2 * stall, 1), AF.SPEED, 18.0
    points = [A.Point("loiter", v_loiter, cl(v_loiter), re(v_loiter), 0.3),
              A.Point("cruise", v_cruise, cl(v_cruise), re(v_cruise), 0.5),
              A.Point("fast", v_fast, cl(v_fast), re(v_fast), 0.2)]
    sv = k.servo
    xa = p.main_spar_pos * L.chord + p.main_spar_d / 2 + 0.4
    xb = p.rear_spar_pos * L.chord - p.rear_spar_d / 2 - 0.4
    xm = (xa + xb) / 2
    wall = 2 * p.skin + 0.2 + 0.5                        # both skins, hole clearance, a margin
    lim = A.Limits(chord=L.chord, te=p.te_min,
                   main_spar=(p.main_spar_pos, p.main_spar_d + wall), rear_spar=(p.rear_spar_pos, p.rear_spar_d + wall),
                   hinge=(p.hinge_pos, 3.0), servo_zone=(xm - sv.tab_span / 2, xm + sv.tab_span / 2),
                   servo_width=sv.width, servo_top_gap=1.2, re_stall=re(9.0))
    return {"L": L, "mass": mass, "w": w, "s": s, "chord": c, "stall": stall, "points": points, "lim": lim}


def section_summary(name, kulfan, alphas, points, lim, n_crit) -> dict:
    r = A.evaluate(kulfan, alphas, points, lim, n_crit)
    af = A.Section(name, kulfan).airfoil
    return {"name": name, "alphas": np.round(alphas, 3).tolist(),
            "cd": np.round(r["cd"], 5).tolist(), "cl": np.round(r["cl"], 4).tolist(),
            "cm": np.round(r["cm"], 4).tolist(), "confidence": np.round(r["conf"], 3).tolist(),
            "clmax": round(r["clmax"], 3), "cl_post": round(r["cl_post"], 3), "max_tc": round(r["max_tc"], 4),
            "depth_main": round(r["main"], 2), "depth_rear": round(r["rear"], 2), "depth_hinge": round(r["hinge"], 2),
            "servo_bump": round(r["bump"], 2), "perimeter": round(float(af.perimeter()), 4),
            "problems": A.feasible(r, points, lim)}


def optimize_section(points, lim, n_crit, starts, log):
    te = lim.te / lim.chord
    base = A.Section("NACA 4412", A.fit("naca4412", te))
    base.alphas = [A.alpha_for(base.kulfan, p.cl, p.re, n_crit) for p in points]
    rb = A.evaluate(base.kulfan, base.alphas, points, lim, n_crit)
    lim.servo_bump, lim.clmax, lim.cl_post, lim.cm = rb["bump"], rb["clmax"], rb["cl_post"], float(rb["cm"][1])
    best, best_r, runs = None, None, []
    for name in starts:
        t0 = time.time()
        sec, r = A.optimize(A.Section(name.upper(), A.fit(name, te)), points, lim, n_crit, rb["cd"])
        problems = A.feasible(r, points, lim)
        score = float(np.sum(np.array([p.weight for p in points]) * r["cd"]))
        runs.append({"start": name.upper(), "score": round(score, 6), "problems": problems,
                     "iterations": r["nit"], "seconds": round(time.time() - t0, 1)})
        log(f"  from {name.upper():9s} weighted cd {score:.5f} "
            f"{'ok' if not problems else 'breaks: ' + ', '.join(problems)} ({r['nit']} iterations)")
        if not problems and (best_r is None or score < best_r["score"]):
            best, best_r = sec, {**r, "score": score}
    if best is None:
        raise SystemExit("no start reached a shape that meets every limit")
    best.name = "Optimized"
    return base, best, runs


def aircraft(kulfan, details: dict, cond, n_crit, speeds, shell_ratio=1.0) -> dict:
    """Drag and power of the whole plane over the speed range."""
    x = {p.name: p.value for p in AF.params()}
    x.update(details)
    mass = cond["mass"] + PANEL_SHELL_G * (shell_ratio - 1)
    w, s, c = mass / 1000 * G, cond["s"], cond["chord"]
    L = cond["L"]
    s_wet = s - 0.5 * 2 * L.half_w * L.chord * 1e-6          # the pod covers part of the centre's underside
    ar = AF.BASE.aspect_ratio
    v_stall = math.sqrt(2 * w / (1.225 * s * AF.BASE.cl_max))
    rows = []
    for v in speeds:
        if v < v_stall:
            continue
        q = 0.5 * 1.225 * v * v
        cl = w / (q * s)
        re = v * c / 1.46e-5
        alpha = A.alpha_for(kulfan, cl, re, n_crit)
        cd = float(np.squeeze(A.aero(kulfan, alpha, re, n_crit)["CD"]))
        other = [i for i in AF.evaluate(x, v).items if not i.name.startswith(("wing", "junctions"))]
        wing = cd * s_wet
        fp = sum(i.cda for i in other if i.group in ("friction", "pressure")) + wing
        items = [("wing profile (NeuralFoil)", wing)] + [(i.name, i.cda) for i in other] + [("junctions", 0.10 * fp)]
        induced = q * s * cl * cl / (math.pi * AF.E_OSWALD * ar)
        drag = q * sum(a for _, a in items) + induced
        power = drag * v / AF.ETA_PROP + AF.AVIONICS_W
        rows.append({"v": round(v, 2), "cl": round(cl, 3), "cd_wing": round(cd, 5), "drag": round(drag, 4),
                     "induced": round(induced, 4), "power": round(power, 2), "ld": round(w / drag, 2),
                     "items": [(n, round(q * a, 4)) for n, a in items] + [("induced", round(induced, 4))]})
    # level top speed on the stock thrust line (as design.py)
    k = AF.KIT
    v_pitch = k.motor_kv * k.v_loaded * k.rpm_frac / 60 * k.prop_pitch_in * 0.0254
    top = next((r["v"] for r in reversed(rows) if w * (1 - r["v"] / v_pitch) >= r["drag"]), None)
    return {"mass": round(mass, 1), "stall": round(v_stall, 2), "rows": rows, "top": top}


def kpis(a: dict, v_cruise: float) -> dict:
    rows = a["rows"]
    cruise = min(rows, key=lambda r: abs(r["v"] - v_cruise))
    pmin = min(rows, key=lambda r: r["power"])
    ld = max(rows, key=lambda r: r["ld"])
    wh = 0.8 * AF.KIT.batt_wh
    return {"drag_cruise": cruise["drag"], "power_cruise": cruise["power"],
            "endurance_cruise": round(wh / cruise["power"] * 60, 1),
            "power_min": pmin["power"], "v_power_min": pmin["v"], "endurance_max": round(wh / pmin["power"] * 60, 1),
            "ld_max": ld["ld"], "v_ld_max": ld["v"], "stall": a["stall"], "top": a["top"], "mass": a["mass"],
            "cd_wing_cruise": cruise["cd_wing"]}


def detail_package(v: float) -> dict:
    """The what-if details the drag optimizer picks at this speed (sizing held)."""
    params = AF.params()
    ev = O.Evaluator(AF, v)
    x0 = {p.name: p.value for p in params}
    free = [p.name for p in params if p.kind.startswith("what-if")]
    x1, _ = O.search(ev, params, x0, free, ev(x0))
    return {n: x1[n] for n in free if x1[n] != x0[n]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--n-crit", type=float, default=7.0,
                    help="transition sensitivity: 9 smooth wing in calm air, 5-7 printed skin or gusty air")
    ap.add_argument("--quick", action="store_true", help="start only from the NACA 4412")
    a = ap.parse_args(argv)
    log = lambda s: print(s, flush=True)  # noqa: E731
    t0 = time.time()
    cond = conditions()
    points, lim = cond["points"], cond["lim"]
    log(f"Kipinä wing: chord {lim.chord:.0f} mm, {cond['mass']:.0f} g, n_crit {a.n_crit:g}")
    for p in points:
        log(f"  {p.name:7s} {p.v:5.1f} m/s  cl {p.cl:.3f}  Re {p.re / 1e3:.0f}k  weight {p.weight}")
    log("Optimizing the section:")
    base, opt, runs = optimize_section(points, lim, a.n_crit, STARTS[:1] if a.quick else STARTS, log)

    te = lim.te / lim.chord
    sections = {"baseline": section_summary("NACA 4412", base.kulfan, base.alphas, points, lim, a.n_crit),
                "optimized": section_summary("Optimized", opt.kulfan, opt.alphas, points, lim, a.n_crit)}
    refs = []
    for name in REFERENCES:
        k = A.fit(name, te)
        al = [A.alpha_for(k, p.cl, p.re, a.n_crit) for p in points]
        refs.append(section_summary(name.upper(), k, al, points, lim, a.n_crit))
    refs.append(sections["optimized"])
    shapes = {key: {"name": s.name, **A.coordinates(s.kulfan), "kulfan": {
        "upper_weights": np.round(s.kulfan["upper_weights"], 5).tolist(),
        "lower_weights": np.round(s.kulfan["lower_weights"], 5).tolist(),
        "leading_edge_weight": round(float(s.kulfan["leading_edge_weight"]), 5),
        "TE_thickness": round(float(s.kulfan["TE_thickness"]), 5)}} for key, s in (("baseline", base), ("optimized", opt))}
    af_base, af_opt = base.airfoil, opt.airfoil
    cruise = points[1]
    polars = {key: {"cruise": A.polar(s.kulfan, cruise.re, a.n_crit), "stall": A.polar(s.kulfan, lim.re_stall, a.n_crit)}
              for key, s in (("baseline", base), ("optimized", opt))}
    pressure = {key: A.pressure(s.kulfan, s.alphas[1], cruise.re, a.n_crit) for key, s in (("baseline", base), ("optimized", opt))}
    robust = {key: {str(n): round(float(np.squeeze(A.aero(s.kulfan, A.alpha_for(s.kulfan, cruise.cl, cruise.re, n),
                                                          cruise.re, n)["CD"])), 5) for n in (5, 7, 9)}
              for key, s in (("baseline", base), ("optimized", opt))}

    log("Whole plane:")
    details = detail_package(cruise.v)
    speeds = np.round(np.arange(8.5, 22.01, 0.25), 2)
    ratio = float(af_opt.perimeter() / af_base.perimeter())
    variants = {
        "baseline": aircraft(base.kulfan, {}, cond, a.n_crit, speeds),
        "airfoil": aircraft(opt.kulfan, {}, cond, a.n_crit, speeds, ratio),
        "full": aircraft(opt.kulfan, details, cond, a.n_crit, speeds, ratio),
    }
    labels = {"baseline": "As built", "airfoil": "Optimized airfoil", "full": "Optimized airfoil + details"}
    summary = {k: kpis(v, cruise.v) for k, v in variants.items()}
    for k, s in summary.items():
        log(f"  {labels[k]:28s} drag {s['drag_cruise']:.3f} N, {s['power_cruise']:.1f} W at {cruise.v:g} m/s, "
            f"{s['endurance_cruise']} min; best L/D {s['ld_max']} at {s['v_ld_max']} m/s")

    L = cond["L"]
    wing_limits = {
        "chord_mm": lim.chord, "main_spar": {"x": AF.BASE.main_spar_pos, "d_mm": AF.BASE.main_spar_d},
        "rear_spar": {"x": AF.BASE.rear_spar_pos, "d_mm": AF.BASE.rear_spar_d}, "hinge_x": AF.BASE.hinge_pos,
        "servo_zone_mm": [round(z, 1) for z in lim.servo_zone], "servo_width_mm": lim.servo_width,
        "servo_top_gap_mm": lim.servo_top_gap, "te_mm": lim.te,
        "needs": {"main_spar_depth_mm": lim.main_spar[1], "rear_spar_depth_mm": lim.rear_spar[1],
                  "hinge_depth_mm": lim.hinge[1], "servo_bump_mm": round(lim.servo_bump + 0.25, 2),
                  "clmax": round(lim.clmax - 0.01, 3), "cl_post": round(lim.cl_post - 0.03, 3),
                  "cm": round(lim.cm - 0.02, 3), "max_tc": lim.max_tc,
                  "confidence": lim.min_confidence}}
    study = {
        "generated": time.strftime("%Y-%m-%d"), "seconds": round(time.time() - t0),
        "n_crit": a.n_crit, "model": f"NeuralFoil {A.nf.__version__} ({A.MODEL})",
        "plane": {"span_mm": AF.BASE.span, "chord_mm": round(L.chord, 1), "mass_g": round(cond["mass"], 1),
                  "aspect_ratio": AF.BASE.aspect_ratio, "stall": round(cond["stall"], 2)},
        "points": [{"name": p.name, "v": p.v, "cl": round(p.cl, 4), "re": round(p.re), "weight": p.weight} for p in points],
        "limits": wing_limits, "runs": runs, "sections": sections, "references": refs, "shapes": shapes,
        "polars": polars, "pressure": pressure, "robustness": robust, "details": details,
        "labels": labels, "variants": variants, "summary": summary, "cruise_v": cruise.v,
    }
    OUT.mkdir(exist_ok=True)
    (OUT / "study.json").write_text(json.dumps(study, separators=(",", ":")))
    xy = zip(shapes["optimized"]["x"], shapes["optimized"]["y"])
    (OUT / "kipina-opt.dat").write_text("Kipina optimized (aero/efficiency.py)\n"
                                        + "\n".join(f"{x:.5f} {y:.5f}" for x, y in xy) + "\n")
    (OUT / "REPORT.md").write_text(report(study))
    log(f"wrote {OUT.relative_to(Path.cwd()) if OUT.is_relative_to(Path.cwd()) else OUT}/study.json and REPORT.md "
        f"in {study['seconds']} s")


def report(s: dict) -> str:
    b, o = s["sections"]["baseline"], s["sections"]["optimized"]
    L = ["# Drag and efficiency study: Kipinä, baseline against optimized", "",
         f"Generated by `aero/efficiency.py` ({s['model']}, n_crit {s['n_crit']:g}). Do not edit by hand. "
         "The charts are in [index.html](index.html).", "",
         "## The wing section", "",
         "| | " + " | ".join(f"{p['name']} {p['v']:g} m/s (cl {p['cl']:.2f}, Re {p['re'] / 1e3:.0f}k)" for p in s["points"])
         + " |", "|---|" + "---|" * len(s["points"])]
    for sec in (b, o):
        L.append(f"| {sec['name']} cd | " + " | ".join(f"{c:.4f}" for c in sec["cd"]) + " |")
    L.append("| change | " + " | ".join(f"{(y - x) / x * 100:+.0f} %" for x, y in zip(b["cd"], o["cd"])) + " |")
    L += ["", "| limit | needs | NACA 4412 | optimized |", "|---|---|---|---|"]
    n = s["limits"]["needs"]
    rows = [("main spar depth, mm", f"≥ {n['main_spar_depth_mm']:.1f}", "depth_main"),
            ("rear spar depth, mm", f"≥ {n['rear_spar_depth_mm']:.1f}", "depth_rear"),
            ("aileron hinge depth, mm", f"≥ {n['hinge_depth_mm']:.1f}", "depth_hinge"),
            ("servo bump below the wing, mm", f"≤ {n['servo_bump_mm']:.2f}", "servo_bump"),
            ("maximum lift (Re at 9 m/s)", f"≥ {n['clmax']:.3f}", "clmax"),
            ("lowest lift past the stall, 9-16°", f"≥ {n['cl_post']:.3f}", "cl_post"),
            ("maximum thickness", f"≤ {n['max_tc']:.0%}", "max_tc")]
    for label, need, key in rows:
        fmt = (lambda v: f"{v:.1%}") if key == "max_tc" else (lambda v: f"{v:.2f}")
        L.append(f"| {label} | {need} | {fmt(b[key])} | {fmt(o[key])} |")
    L.append(f"| pitching moment at cruise | ≥ {n['cm']:.3f} | {b['cm'][1]:.3f} | {o['cm'][1]:.3f} |")
    L += ["", "Optimizer starts:", ""]
    for r in s["runs"]:
        L.append(f"- from {r['start']}: weighted cd {r['score']:.5f}, "
                 f"{'meets every limit' if not r['problems'] else 'breaks ' + ', '.join(r['problems'])}")
    L += ["", "Known airfoils against the same limits (cruise cd):", "", "| airfoil | cruise cd | max lift | thickness | servo bump, mm | limits |",
          "|---|---|---|---|---|---|"]
    for r in s["references"]:
        L.append(f"| {r['name']} | {r['cd'][1]:.4f} | {r['clmax']:.2f} | {r['max_tc']:.1%} | {r['servo_bump']:.1f} | "
                 f"{'all met' if not r['problems'] else ', '.join(r['problems'])} |")
    rb = s["robustness"]
    L += ["", "Cruise cd against transition sensitivity (n_crit 5 rough, 9 smooth): "
          + "; ".join(f"{k}: " + ", ".join(f"{n} → {v:.4f}" for n, v in rb[k].items()) for k in rb) + ".", "",
          "## The whole plane", "",
          "| | " + " | ".join(s["labels"][k] for k in s["summary"]) + " |", "|---|" + "---|" * len(s["summary"])]
    rows = [("Drag at cruise, N", "drag_cruise", "{:.3f}"), ("Power at cruise, W", "power_cruise", "{:.1f}"),
            ("Flight time at cruise, min", "endurance_cruise", "{:.1f}"), ("Best L/D", "ld_max", "{:.1f}"),
            ("... at, m/s", "v_ld_max", "{:.1f}"), ("Least power, W", "power_min", "{:.1f}"),
            ("... at, m/s", "v_power_min", "{:.1f}"), ("Longest flight time, min", "endurance_max", "{:.1f}"),
            ("Stall, m/s", "stall", "{:.2f}"), ("Top speed, m/s", "top", "{:.1f}"), ("Weight, g", "mass", "{:.0f}")]
    for label, key, f in rows:
        L.append(f"| {label} | " + " | ".join(f.format(s["summary"][k][key]) if s["summary"][k][key] is not None else "-"
                                          for k in s["summary"]) + " |")
    L += ["", "The details (from the drag optimizer, `aero/optimize.py airframe`): "
          + ", ".join(f"`{k}` = {v:g}" for k, v in s["details"].items()) + ".", ""]
    return "\n".join(L)


if __name__ == "__main__":
    main()
