#!/usr/bin/env python3
"""As built against optimized: the two CAD variants' weights, and what the
weight difference does to the way Kipinä flies.

    python3 airframe/build.py                       # the as-built CAD (airframe/viewer/spec.json)
    python3 airframe/build.py --variant optimized   # the optimized CAD (airframe/variants/optimized/)
    python3 aero/variants.py                        # airframe/variants/compare.json and COMPARE.md

Each variant flies at its own CAD weight with its own aerodynamics: the wing's
profile drag from NeuralFoil with its airfoil at the lift it needs, every other
part from the drag build-up (aero/models/airframe.py) with its details, and the
induced drag as in the efficiency study. The maximum lift follows each
airfoil's own section maximum. Both have the same motor and prop, so the
thrust is the as-built plane's (1:1 static, falling to zero at the pitch
speed) for both.

A third case, the optimized aerodynamics at the as-built weight, splits the
difference: what the aerodynamics give, and what the weight takes back. A
weight sweep shows how each design's flight changes with every gram.
"""
from __future__ import annotations

import json
import math
import sys
import time
from functools import lru_cache
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import airfoil as A  # noqa: E402
from models import airframe as AF  # noqa: E402

D = AF.D
OUT = ROOT / "airframe" / "variants"
RHO, G = 1.225, 9.81
V_CRUISE = AF.SPEED
SPEEDS = np.round(np.arange(7.5, 24.01, 0.25), 2)
MASSES = np.arange(195.0, 265.01, 5.0)
USABLE = 0.8                                  # of the battery's energy
VOLTS = 7.4


def spec(variant: str) -> dict:
    path = ROOT / "airframe" / ("viewer" if variant == "baseline" else f"variants/{variant}/viewer") / "spec.json"
    return json.loads(path.read_text())


def kulfan(study: dict, key: str) -> dict:
    k = study["shapes"][key]["kulfan"]
    return {"upper_weights": np.array(k["upper_weights"]), "lower_weights": np.array(k["lower_weights"]),
            "leading_edge_weight": k["leading_edge_weight"], "TE_thickness": k["TE_thickness"]}


class Plane:
    """One configuration: aerodynamics (airfoil and details) and a weight."""

    def __init__(self, label: str, section: str, details: dict, mass_g: float, study: dict):
        self.label, self.section, self.details, self.mass = label, section, dict(details), mass_g
        self.kulfan = kulfan(study, section)
        self.n_crit = study["n_crit"]
        L, _, _ = D.evaluate(AF.BASE, AF.KIT)
        self.L = L
        self.s = L.area * 1e-6
        self.c = L.chord / 1000
        self.s_wet = self.s - 0.5 * 2 * L.half_w * L.chord * 1e-6     # the pod covers part of the centre
        self.ar = AF.BASE.aspect_ratio
        # 3D maximum lift: the design's 0.95, scaled by the section's own maximum
        self.clmax = AF.BASE.cl_max * study["sections"][section]["clmax"] / study["sections"]["baseline"]["clmax"]
        self.w = mass_g / 1000 * G
        k = AF.KIT
        self.v_pitch = k.motor_kv * k.v_loaded * k.rpm_frac / 60 * k.prop_pitch_in * 0.0254

    def at_mass(self, mass_g: float) -> "Plane":
        p = object.__new__(Plane)
        p.__dict__.update(self.__dict__)
        p.mass, p.w = mass_g, mass_g / 1000 * G
        return p

    def parasite(self, v: float) -> tuple[float, list]:
        """Drag area of everything but the wing and the junctions (m^2), and the items."""
        return _parasite(tuple(sorted(self.details.items())), round(v, 3))

    def drag(self, v: float, n: float = 1.0) -> dict:
        """Level (n = 1) or turning (load factor n) drag at speed v."""
        q = 0.5 * RHO * v * v
        cl = n * self.w / (q * self.s)
        re = v * self.c / 1.46e-5
        cd = _section_cd(self.section, round(min(cl, 1.25), 4), round(re, -2), self.n_crit, self)
        other, items = self.parasite(v)
        wing = cd * self.s_wet
        fp = other["fp"] + wing
        induced = q * self.s * cl * cl / (math.pi * AF.E_OSWALD * self.ar)
        total = q * (wing + other["all"] + 0.10 * fp) + induced
        return {"drag": total, "induced": induced, "wing": q * wing, "cl": cl, "cd_wing": cd,
                "items": [("wing profile", q * wing)] + [(nm, q * a) for nm, a in items]
                + [("junctions", q * 0.10 * fp), ("induced", induced)]}

    def thrust(self, v: float, t0: float) -> float:
        return max(0.0, t0 * (1 - v / self.v_pitch))


@lru_cache(maxsize=None)
def _parasite(details: tuple, v: float):
    x = {p.name: p.value for p in AF.params()}
    x.update(dict(details))
    items = [i for i in AF.evaluate(x, v).items if not i.name.startswith(("wing", "junctions"))]
    fp = sum(i.cda for i in items if i.group in ("friction", "pressure"))
    return {"all": sum(i.cda for i in items), "fp": fp}, [(i.name, i.cda) for i in items]


_CD = {}


def _section_cd(section: str, cl: float, re: float, n_crit: float, plane: Plane) -> float:
    key = (section, cl, re, n_crit)
    if key not in _CD:
        alpha = A.alpha_for(plane.kulfan, cl, re, n_crit)
        _CD[key] = float(np.squeeze(A.aero(plane.kulfan, alpha, re, n_crit)["CD"]))
    return _CD[key]


def fly(p: Plane, t0: float, curves: bool = True) -> dict:
    """Everything about how one configuration flies."""
    w, s = p.w, p.s
    stall = math.sqrt(2 * w / (RHO * s * p.clmax))
    rows = []
    for v in SPEEDS:
        if v < stall * 1.02:
            continue
        d = p.drag(v)
        power = d["drag"] * v / AF.ETA_PROP + AF.AVIONICS_W
        t = p.thrust(v, t0)
        rows.append({"v": float(v), "drag": d["drag"], "power": power, "ld": w / d["drag"],
                     "sink": d["drag"] * v / w, "climb": (t - d["drag"]) * v / w, "thrust": t,
                     "endurance": USABLE * AF.KIT.batt_wh / power * 60,
                     "range": USABLE * AF.KIT.batt_wh / power * 3600 * v / 1000})
    cruise = min(rows, key=lambda r: abs(r["v"] - V_CRUISE))
    dc = p.drag(cruise["v"])
    pmin = min(rows, key=lambda r: r["power"])
    ld = max(rows, key=lambda r: r["ld"])
    sink = min(rows, key=lambda r: r["sink"])
    rng = max(rows, key=lambda r: r["range"])
    climb = max(rows, key=lambda r: r["climb"])
    top = None                                        # where the climb rate crosses zero, between grid speeds
    for r0, r1 in zip(rows, rows[1:]):
        if r0["climb"] >= 0 > r1["climb"]:
            top = round(r0["v"] + (r1["v"] - r0["v"]) * r0["climb"] / (r0["climb"] - r1["climb"]), 2)
    if top is None and rows and rows[-1]["climb"] >= 0:
        top = rows[-1]["v"]
    # turns at the cruise speed: stall-limited, and the tightest the thrust can hold
    q = 0.5 * RHO * V_CRUISE ** 2
    n_stall = q * s * p.clmax / w
    n_sus, t_c = 1.0, p.thrust(V_CRUISE, t0)
    for n in np.arange(1.0, n_stall, 0.01):
        if p.drag(V_CRUISE, n)["drag"] > t_c:
            break
        n_sus = float(n)
    radius = lambda n: V_CRUISE ** 2 / (G * math.sqrt(n * n - 1)) if n > 1.0005 else None  # noqa: E731
    out = {
        "label": p.label, "mass": round(p.mass, 1), "loading": round(p.mass / (p.L.area / 1e4), 1),
        "clmax": round(p.clmax, 3), "stall": round(stall, 2),
        "cruise_v": cruise["v"], "drag_cruise": round(cruise["drag"], 4), "power_cruise": round(cruise["power"], 2),
        "current_cruise": round(cruise["power"] / VOLTS, 2), "endurance_cruise": round(cruise["endurance"], 1),
        "range_cruise": round(cruise["range"], 2), "cl_cruise": round(dc["cl"], 3),
        "induced_cruise": round(dc["induced"], 4),
        "v_power_min": pmin["v"], "power_min": round(pmin["power"], 2), "endurance_max": round(pmin["endurance"], 1),
        "v_ld_max": ld["v"], "ld_max": round(ld["ld"], 2),
        "v_range_max": rng["v"], "range_max": round(rng["range"], 2),
        "v_sink_min": sink["v"], "sink_min": round(sink["sink"], 3), "glide_max": round(ld["ld"], 2),
        "v_climb_max": climb["v"], "climb_max": round(climb["climb"], 2),
        "climb_angle": round(math.degrees(math.asin(max(-1.0, min(1.0, climb["climb"] / climb["v"])))), 1),
        "top": top,
        "turn_n_stall": round(n_stall, 2), "turn_bank_stall": round(math.degrees(math.acos(1 / n_stall)), 1),
        "turn_radius_stall": round(radius(n_stall), 1) if radius(n_stall) else None,
        "turn_n_sustained": round(n_sus, 2), "turn_bank_sustained": round(math.degrees(math.acos(1 / n_sus)), 1),
        "turn_radius_sustained": round(radius(n_sus), 1) if radius(n_sus) else None,
        "items_cruise": [(nm, round(val, 4)) for nm, val in dc["items"]],
    }
    if curves:
        out["curves"] = {k: [round(r[k], 4) for r in rows] for k in ("drag", "power", "ld", "sink", "climb", "endurance")}
        out["curves"]["v"] = [r["v"] for r in rows]
    return out


def main():
    t0_clock = time.time()
    study = json.loads((ROOT / "aero" / "study" / "study.json").read_text())
    specs = {k: spec(k) for k in ("baseline", "optimized")}
    m_a, m_c = (specs[k].get("auw_g", specs[k]["auw"]) for k in ("baseline", "optimized"))
    details = {k: float(v) for k, v in study["details"].items()}
    none = {k: 0.0 for k in details}
    A_ = Plane("As built", "baseline", none, m_a, study)
    C_ = Plane("Optimized", "optimized", details, m_c, study)
    B_ = C_.at_mass(m_a)
    B_.label = "Optimized aerodynamics, as-built weight"
    t0 = m_a / 1000 * G                                   # the same motor and prop: 1:1 static for the as-built plane
    print(f"As built {m_a} g, optimized {m_c} g ({m_c - m_a:+.1f} g)", flush=True)
    cases = {"A": fly(A_, t0), "B": fly(B_, t0), "C": fly(C_, t0)}
    for k, c in cases.items():
        print(f"  {k} {c['label']:42s} stall {c['stall']:.2f}  cruise {c['power_cruise']:.2f} W "
              f"{c['endurance_cruise']:.1f} min  climb {c['climb_max']:.2f} m/s  L/D {c['ld_max']:.2f}", flush=True)

    sweep = {}
    for key, plane in (("baseline", A_), ("optimized", C_)):
        rows = []
        for m in MASSES:
            r = fly(plane.at_mass(float(m)), t0, curves=False)
            rows.append({k: r[k] for k in ("mass", "stall", "power_cruise", "endurance_cruise", "endurance_max",
                                           "climb_max", "ld_max", "sink_min", "turn_radius_sustained", "top",
                                           "range_max")})
        sweep[key] = rows
    per10 = {}
    for key, rows in sweep.items():
        fit = lambda k: np.polyfit([r["mass"] for r in rows], [r[k] for r in rows], 1)[0] * 10  # noqa: E731
        per10[key] = {k: round(float(fit(k)), 3) for k in ("stall", "power_cruise", "endurance_cruise",
                                                            "endurance_max", "climb_max", "ld_max", "sink_min")}
    # how much heavier the optimized plane could be before its cruise power is back to the as-built one's
    lo, hi = m_c, 600.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if fly(C_.at_mass(mid), t0, curves=False)["power_cruise"] < cases["A"]["power_cruise"]:
            lo = mid
        else:
            hi = mid
    break_even = (lo + hi) / 2
    stall_limit = AF.BASE.stall_limit
    max_mass = {k: round(RHO * A_.s * p.clmax * stall_limit ** 2 / (2 * G) * 1000, 1)
                for k, p in (("baseline", A_), ("optimized", C_))}

    ma = {m["name"]: m for m in specs["baseline"]["mass"]}
    mc = {m["name"]: m for m in specs["optimized"]["mass"]}
    mass_rows = []
    for name in sorted(set(ma) | set(mc), key=lambda n: -max(ma.get(n, {}).get("g", 0), mc.get(n, {}).get("g", 0))):
        mass_rows.append({"name": name, "baseline": ma.get(name, {}).get("g", 0.0), "optimized": mc.get(name, {}).get("g", 0.0),
                          "x_baseline": ma.get(name, {}).get("x"), "x_optimized": mc.get(name, {}).get("x")})
    keys = ("x_le", "cg_x", "battery_x", "np_x", "pod_len", "length", "x_stab", "printed", "stall", "loading", "airfoil", "details")
    out = {
        "generated": time.strftime("%Y-%m-%d"), "n_crit": study["n_crit"], "cruise_v": V_CRUISE,
        "thrust_static_n": round(t0, 3), "v_pitch": round(A_.v_pitch, 2), "stall_limit": stall_limit,
        "variants": {k: {**{kk: specs[k].get(kk) for kk in keys}, "auw": m} for k, m in (("baseline", m_a), ("optimized", m_c))},
        "mass": mass_rows, "cases": cases, "sweep": sweep, "per10g": per10,
        "break_even_mass": round(break_even, 1), "max_mass_for_stall": max_mass,
        "battery_wh": AF.KIT.batt_wh, "usable": USABLE,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "compare.json").write_text(json.dumps(out, separators=(",", ":")))
    (OUT / "COMPARE.md").write_text(report(out))
    print(f"wrote airframe/variants/compare.json and COMPARE.md in {time.time() - t0_clock:.0f} s")


def report(o: dict) -> str:
    A_, B_, C_ = o["cases"]["A"], o["cases"]["B"], o["cases"]["C"]
    va, vc = o["variants"]["baseline"], o["variants"]["optimized"]
    d = lambda k, f="{:+.2f}": f.format(C_[k] - A_[k])  # noqa: E731
    rows = [("Weight, g", "mass", "{:.1f}"), ("Wing loading, g/dm²", "loading", "{:.1f}"),
            ("Maximum lift (3D)", "clmax", "{:.3f}"), ("Stall, m/s", "stall", "{:.2f}"),
            ("Drag at 13 m/s, N", "drag_cruise", "{:.3f}"), ("Power at 13 m/s, W", "power_cruise", "{:.2f}"),
            ("Current at 13 m/s, A", "current_cruise", "{:.2f}"), ("Flight time at 13 m/s, min", "endurance_cruise", "{:.1f}"),
            ("Distance at 13 m/s, km", "range_cruise", "{:.1f}"), ("Longest flight, min", "endurance_max", "{:.1f}"),
            ("... at, m/s", "v_power_min", "{:.1f}"), ("Furthest flight, km", "range_max", "{:.1f}"),
            ("... at, m/s", "v_range_max", "{:.1f}"), ("Best glide ratio", "ld_max", "{:.2f}"),
            ("... at, m/s", "v_ld_max", "{:.1f}"), ("Least sink, motor off, m/s", "sink_min", "{:.3f}"),
            ("Best climb, m/s", "climb_max", "{:.2f}"), ("Top speed, m/s", "top", "{:.1f}"),
            ("Tightest level turn at 13 m/s, bank °", "turn_bank_stall", "{:.1f}"),
            ("... radius, m", "turn_radius_stall", "{:.2f}")]
    L = ["# Kipinä: as built against optimized", "",
         "Generated by `aero/variants.py` from the two CAD builds (`airframe/viewer/spec.json` and "
         "`airframe/variants/optimized/viewer/spec.json`). Do not edit by hand.", "",
         "## The two designs", "",
         "| | As built | Optimized |", "|---|---|---|",
         f"| Airfoil | {va['airfoil']} | {vc['airfoil']} |",
         f"| Details | none | " + ", ".join(f"{k} {v:g}" if not isinstance(v, bool) else k for k, v in vc["details"].items() if v) + " |",
         f"| Weight (CAD) | {va['auw']:.1f} g | {vc['auw']:.1f} g ({vc['auw'] - va['auw']:+.1f} g) |",
         f"| Printed parts | {va['printed']} g | {vc['printed']} g |",
         f"| Wing leading edge from the nose (balanced) | {va['x_le']} mm | {vc['x_le']} mm |",
         f"| CG | {va['cg_x']} mm | {vc['cg_x']} mm |", "",
         "## Where the weight changed", "", "| part | as built, g | optimized, g | change |", "|---|---|---|---|"]
    for m in o["mass"]:
        if abs(m["optimized"] - m["baseline"]) >= 0.05:
            L.append(f"| {m['name']} | {m['baseline']:.1f} | {m['optimized']:.1f} | {m['optimized'] - m['baseline']:+.1f} |")
    L += ["", "## How they fly", "",
          f"Thrust: the same motor and prop, {o['thrust_static_n']:.2f} N static (1:1 for the as-built plane), "
          f"falling to zero at the {o['v_pitch']:.1f} m/s pitch speed.", "",
          "| | As built | Optimized aero, as-built weight | Optimized | change | of which aerodynamics | of which weight |",
          "|---|---|---|---|---|---|---|"]
    for label, k, f in rows:
        if A_.get(k) is None or C_.get(k) is None:
            continue
        ch = C_[k] - A_[k]
        aero = B_[k] - A_[k]
        wt = C_[k] - B_[k]
        L.append(f"| {label} | {f.format(A_[k])} | {f.format(B_[k])} | {f.format(C_[k])} | {ch:+.3g} | {aero:+.3g} | {wt:+.3g} |")
    p10 = o["per10g"]
    L += ["", "## Every 10 g", "", "| per +10 g | as built | optimized |", "|---|---|---|"]
    for k, lab in (("stall", "stall, m/s"), ("power_cruise", "power at 13 m/s, W"), ("endurance_cruise", "flight time at 13 m/s, min"),
                   ("endurance_max", "longest flight, min"), ("climb_max", "best climb, m/s"), ("ld_max", "best glide ratio"),
                   ("sink_min", "least sink, m/s")):
        L.append(f"| {lab} | {p10['baseline'][k]:+.3f} | {p10['optimized'][k]:+.3f} |")
    L += ["", f"The optimized plane could weigh {o['break_even_mass']:.0f} g "
          f"({o['break_even_mass'] - vc['auw']:+.0f} g on its CAD weight) before it needs as much power at 13 m/s as the "
          f"as-built one: that is what the drag saving is worth in weight. The {o['stall_limit']} m/s stall limit "
          f"allows {o['max_mass_for_stall']['baseline']:.0f} g as built and {o['max_mass_for_stall']['optimized']:.0f} g optimized.",
          "", "The tightest level turn at 13 m/s is set by the stall (the maximum lift), not by the motor, "
          "which has thrust to spare at that bank.", ""]
    return "\n".join(L)


if __name__ == "__main__":
    main()
