#!/usr/bin/env python3
"""Kipinä, extreme aero: how far the single boom's aerodynamics can be pushed
with the same electronics, the same motor and prop and the same printer.

    python3 aero/extreme.py          # aero/study/extreme.json and EXTREME.md
    python3 airframe/build.py --variant extreme   # then builds what it picks

The single boom's drag at 13 m/s is about a third induced drag and a third the
wing's own profile drag; the fuselage, tail and fittings share the rest. The
wing's span is what sets the induced drag (D_i = W^2 / (q pi e b^2)), so the
search is over the wing, and over the tail it needs:

  * span, root chord and taper. The wing stays flat (no dihedral, the ailerons
    and rudder turn it). A centre section on the fuselage, then an inner panel
    of constant chord and an outer panel that tapers to the tip, both standing
    on their root ribs on the 180 mm bed, so each is at most 175 mm long. The
    aileron servo sits at the outer panel's root; its tabs must still fit
    between the spars where the taper has begun. The quarter-chord line is
    straight, so the main spar runs straight through every panel.
  * the tail arm. The tail volumes stay those of the design (V_H 0.50,
    V_V 0.045), so the tail shrinks as its arm grows and the boom gets longer.
  * the main spar: the lightest carbon tube that keeps the 1 g bending under
    1.5 % of the half span and fits the tip's depth; the boom: the lightest
    roll-wrapped tube as stiff as the single boom's (first bending mode).
  * the balance. design.py places the wing where the battery balances the
    plane (moved by as much as the single boom's CAD moved it). A heavier tail
    puts the wing further aft, so the fuselage stretches between the servos
    and the ESC until the wing again sits down on its full section as far
    ahead of the tail cone as the single boom's does.

Each design is flown at its own weight:

  * the span loading from a lifting-line solution (Glauert, 40 terms), with
    the section's own lift slope and the twist; its span efficiency scales
    the 0.8 the design uses for the rectangular AR 5 wing (fuselage and
    viscous losses stay the same share). An AeroSandbox vortex lattice checks
    the induced drag of both planforms;
  * the wing's profile drag strip by strip, at each strip's own lift
    coefficient and Reynolds number, from NeuralFoil on the optimized section
    (aero/study/kipina-opt.dat);
  * everything else from the single boom's drag build-up (aero/models/airframe.py)
    on this plane's layout;
  * the maximum lift: the single boom's 0.967, scaled by how much lift the new
    planform carries when its first section reaches its own maximum lift,
    against the rectangular wing;
  * the weight: the single boom's CAD weight with the wing (by wall area, as
    the CAD weighs a printed shell), spars, tail, boom, pushrods, motor wires
    and fuselage re-sized.

It keeps the best glide ratio among the designs that stall under 9 m/s and
need no more power at 13 m/s than the single boom; of the designs within 1 %
of that glide, the lightest. Then it picks the outer panels' twist: the one
that glides best while the stall still starts inboard of the ailerons.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, replace
from functools import lru_cache
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import airfoil as A  # noqa: E402
import dragkit as K  # noqa: E402
from models import airframe as AF  # noqa: E402

D = AF.D
RHO, NU, G = 1.225, 1.46e-5, 9.81
OUT = HERE / "study"
STUDY = json.loads((OUT / "study.json").read_text())
N_CRIT = STUDY["n_crit"]
KULFAN = {k: (np.array(v) if isinstance(v, list) else v)
          for k, v in STUDY["shapes"]["optimized"]["kulfan"].items()}
SINGLE = json.loads((ROOT / "airframe" / "variants" / "single-boom" / "viewer" / "spec.json").read_text())
V_CRUISE = AF.SPEED
STALL_LIMIT = 9.0
SPEEDS = np.round(np.arange(7.0, 22.01, 0.25), 2)

PANEL_MAX = 175.0                  # each panel stands on its root rib: the A1 mini's height less a margin
CENTRE_MIN = 56.0                  # the centre section across the fuselage
DEPTH_MAIN = 10.69 / 97.0          # the section's depth at the main spar, per chord (study.json)
SKINS = 2 * 0.45 + 0.2 + 0.5       # both skins, hole clearance and a margin round a spar (efficiency.py)
E_CARBON = 70e9                    # roll-wrapped tube, as design.boom_options
DEFLECTION = 0.015                 # bending at 1 g, of the half span
# carbon spars: (outer, inner) mm; rods have inner 0
SPARS = ((3.0, 0.0), (4.0, 3.0), (5.0, 4.0), (6.0, 5.0), (8.0, 7.0))
# roll-wrapped boom tubes, and the single boom's first bending mode they must keep
BOOMS = ((10.0, 9.0), (10.0, 8.0), (12.0, 11.0), (12.0, 10.0), (14.0, 13.0), (14.0, 12.0))

# the wing panels weigh what build.print_mass gives a shell: one 0.45 mm wall of LW-PLA, 0 % infill, over every
# surface, the spar holes and lead channels included (g per mm² of wall)
G_WALL = 0.45 * D.LW_PLA / 1000
PERIMETER = STUDY["sections"]["optimized"]["perimeter"]   # skin round the section, per chord
SECTION_AREA = 0.08                # the section's area, per chord²: the end ribs
# grams per mm² of planform (or per mm) from the single boom's CAD (spec.json)
G_CENTRE = 15.9 / (135.0 * 97.0)   # centre section: 0.5 mm walls, 5 % infill
G_AILERON = 2.9 / (158.0 * 24.0)
G_STAB = (4.1 + 2.1) / (175.0 * 48.0)
G_FIN = (1.9 + 1.7) / (79.0 * 48.0)
G_PUSHROD = D.rod_mass(1.0, 1.0) + D.rod_mass(1.5, 1.0, 0.8, D.PTFE)   # carbon rod in its PTFE sleeve, per mm
WASHOUT = 2.0                      # degrees, root of the outer panel to its tip


def mass_of(name: str) -> float:
    return next(m["g"] for m in SINGLE["mass"] if m["name"] == name)


# --------------------------------------------------------------------------
# Section data: profile drag and lift slope from NeuralFoil


RE_GRID = np.geomspace(25e3, 250e3, 14)
ALPHA_GRID = np.arange(-6.0, 14.01, 0.25)


@lru_cache(maxsize=1)
def polars():
    """CL and CD over alpha at each Reynolds number of the grid, the pre-stall
    branch only, plus each Re's lift slope and maximum lift."""
    out = []
    for re in RE_GRID:
        a = A.aero(KULFAN, ALPHA_GRID, re, N_CRIT)
        cl, cd = np.asarray(a["CL"], float), np.asarray(a["CD"], float)
        k = int(np.argmax(cl))
        slope = float(np.polyfit(ALPHA_GRID[(ALPHA_GRID >= -2) & (ALPHA_GRID <= 4)],
                                 cl[(ALPHA_GRID >= -2) & (ALPHA_GRID <= 4)], 1)[0]) * 180 / math.pi
        a0 = float(np.interp(0.0, cl[:k + 1], ALPHA_GRID[:k + 1]))
        out.append({"cl": cl[:k + 1], "cd": cd[:k + 1], "clmax": float(cl[k]), "slope": slope, "alpha0": a0})
    return out


def section_cd(cl: np.ndarray, re: np.ndarray) -> np.ndarray:
    """Profile drag at each (cl, Re), interpolated in log Re between grid polars."""
    P = polars()
    lr = np.log(RE_GRID)
    out = np.empty(len(cl))
    for i, (c, r) in enumerate(zip(cl, re)):
        x = np.clip(math.log(r), lr[0], lr[-1])
        j = min(int(np.searchsorted(lr, x)), len(lr) - 1)
        j0 = max(j - 1, 0)
        t = 0.0 if j == j0 else (x - lr[j0]) / (lr[j] - lr[j0])
        v = [float(np.interp(min(c, P[k]["cl"][-1]), P[k]["cl"], P[k]["cd"])) for k in (j0, j)]
        out[i] = v[0] + t * (v[1] - v[0])
    return out


def section_value(key: str, re: float) -> float:
    P = polars()
    return float(np.interp(math.log(re), np.log(RE_GRID), [p[key] for p in P]))


# --------------------------------------------------------------------------
# Planform and lifting line


@dataclass(frozen=True)
class Design:
    span: float = 485.0            # mm
    c_root: float = 97.0           # mm, centre and inner panel
    taper: float = 1.0             # tip chord / root chord, on the outer panel
    washout: float = 0.0           # deg, outer panel root to tip
    tail_arm: float = 271.6        # mm, wing c/4 to stab c/4 (the single boom's 2.8 chords)
    spar: tuple = (3.0, 0.0)       # main spar (outer, inner) mm
    boom: tuple = (10.0, 9.0)      # tail boom (outer, inner) mm
    centre: float = 135.0          # centre section width, mm
    rect_panels: int = 1           # the single boom: one panel each side
    stretch: float = 0.0           # fuselage stretch between the servo bay and the ESC, mm

    @property
    def c_tip(self) -> float:
        return self.c_root * self.taper

    @property
    def panel(self) -> float:
        """Each panel's length (two each side, the same length)."""
        return (self.span - self.centre) / (2 * self.rect_panels) if self.rect_panels == 1 else \
            (self.span - self.centre) / 4

    @property
    def y_break(self) -> float:
        """Where the taper starts (mm from the centre-line)."""
        return self.span / 2 if self.rect_panels == 1 else self.centre / 2 + self.panel

    def chord(self, y):
        y = np.abs(np.asarray(y, float))
        yb, h = self.y_break, self.span / 2
        if yb >= h - 1e-6:
            return np.full_like(y, self.c_root)
        return np.where(y <= yb, self.c_root, self.c_root + (self.c_tip - self.c_root) * (y - yb) / (h - yb))

    def twist(self, y):
        """Geometric twist, deg (negative = nose down)."""
        y = np.abs(np.asarray(y, float))
        yb, h = self.y_break, self.span / 2
        if yb >= h - 1e-6:
            return np.zeros_like(y)
        return np.where(y <= yb, 0.0, -self.washout * (y - yb) / (h - yb))

    @property
    def area(self) -> float:
        yb, h = self.y_break, self.span / 2
        return 2 * (self.c_root * yb + (self.c_root + self.c_tip) / 2 * (h - yb))

    @property
    def mac(self) -> float:
        y = np.linspace(0, self.span / 2, 2001)
        c = self.chord(y)
        return float(2 * np.trapezoid(c * c, y) / self.area)

    @property
    def ar(self) -> float:
        return self.span ** 2 / self.area


N_LL = 40


@lru_cache(maxsize=4096)
def lifting_line(d: Design, re_ref: float):
    """Glauert's lifting line: the spanwise lift per unit wing CL (the additional
    load) and the twist's basic load, with the induced drag factors. Returns the
    stations, local cl = cl_a * CL + cl_b, and CDi = k2 * CL^2 + k1 * CL + k0."""
    b = d.span / 1000
    n = np.arange(1, 2 * N_LL, 2)                        # odd terms: a symmetric load
    th = np.linspace(0.5 * math.pi / N_LL, 0.5 * math.pi, N_LL)   # half span, root at pi/2
    y = -b / 2 * np.cos(th)
    c = d.chord(y * 1000) / 1000
    a0 = min(section_value("slope", re_ref), 2 * math.pi)   # per radian; NeuralFoil reads steeper at low Re
    mu = a0 * c / (4 * b)
    M = np.sin(np.outer(th, n)) * (np.sin(th)[:, None] + n[None, :] * mu[:, None])
    tw = np.radians(d.twist(y * 1000))

    def solve(alpha):                                     # alpha from zero lift, per station
        A_ = np.linalg.solve(M, mu * alpha * np.sin(th))
        cl_loc = 4 * b * (np.sin(np.outer(th, n)) @ A_) / c
        CL = math.pi * d.ar * A_[0]
        CDi = math.pi * d.ar * float(np.sum(n * A_ ** 2))
        return A_, cl_loc, CL, CDi

    A1, cl1, CL1, _ = solve(np.ones_like(th))            # 1 rad everywhere: the additional load
    Ab, clb, CLb, _ = solve(tw)                          # the twist alone
    # local cl = cl_a * CL + cl_b, where the twist's own load is held at zero net lift
    cl_a = cl1 / CL1
    cl_b = clb - cl_a * CLb
    # CDi(CL) from the Fourier coefficients: A = A1/CL1 * CL + (Ab - A1 * CLb/CL1)
    ua = A1 / CL1
    ub = Ab - A1 * CLb / CL1
    k2 = math.pi * d.ar * float(np.sum(n * ua * ua))
    k1 = 2 * math.pi * d.ar * float(np.sum(n * ua * ub))
    k0 = math.pi * d.ar * float(np.sum(n * ub * ub))
    return {"y": np.abs(y) * 1000, "c": c * 1000, "cl_a": cl_a, "cl_b": cl_b, "k2": k2, "k1": k1, "k0": k0,
            "e_ll": 1 / (math.pi * d.ar * k2)}


RECT = Design()


def e_scale() -> float:
    """The design's 0.8 at the rectangular AR 5 wing, over its lifting-line value."""
    return AF.E_OSWALD * math.pi * RECT.ar * lifting_line(RECT, 70e3)["k2"]


def first_stall(d: Design, v: float) -> tuple[float, float]:
    """Wing CL when the first section reaches its own maximum lift, and where
    that section is (mm from the centre-line)."""
    ll = lifting_line(d, 70e3)
    re = v * ll["c"] / 1000 / NU
    clmax = np.array([section_value("clmax", r) for r in re])
    ok = ll["cl_a"] > 1e-6
    cl = (clmax[ok] - ll["cl_b"][ok]) / ll["cl_a"][ok]
    i = int(np.argmin(cl))
    return float(cl[i]), float(ll["y"][ok][i])


def first_stall_cl(d: Design, v: float) -> float:
    return first_stall(d, v)[0]


# --------------------------------------------------------------------------
# The plane round the wing: design.Layout, balanced, and its weight


def params(d: Design) -> "D.Params":
    """The single boom's parameters with this wing, tail arm, spar, boom and
    fuselage stretch (what build.py --variant extreme builds)."""
    p = D.single_boom(D.Params())
    tapered = d.rect_panels > 1
    mac = d.mac
    return replace(p, span=d.span, root_chord=d.c_root, taper=d.taper if tapered else 1.0,
                   washout=d.washout if tapered else 0.0, panels=d.rect_panels,
                   main_spar_d=d.spar[0], main_spar_id=d.spar[1], boom_od=d.boom[0], boom_id=d.boom[1],
                   tail_arm=d.tail_arm / mac, bay_stretch=d.stretch)


@lru_cache(maxsize=None)
def layout(d: Design) -> "D.Layout":
    """The layout with the wing where the plane balances: design.py's analytic
    balance, moved by as much as the single boom's CAD build moved it."""
    p = params(d)
    return D.Layout(p, D.solve_x_le(p, KIT) + X_OFFSET, KIT)


def seat_margin(L) -> float:
    """How far ahead of the tail cone the wing centre sits down on the fuselage's
    full section (mm): build.make_fuselage opens the top from there to the cone."""
    return L.cone_x0 - (L.x_le + 0.08 * L.chord + 2.0)


def tail(d: Design) -> dict:
    L = layout(d)
    return {"s_h": L.s_h, "b_h": L.b_h, "c_h": L.c_h, "fin_area": L.fin_area, "fin_h": L.fin_h}


def spar_for(d: Design, mass_g: float):
    """Lightest spar that keeps the 1 g bending under DEFLECTION of the half span
    and fits the tip's depth; None if none does."""
    h = (d.span / 2) / 1000
    w = mass_g / 1000 * G / 2 / h                       # half the weight, spread evenly
    for od, di in SPARS:
        if od + SKINS > DEPTH_MAIN * d.c_tip:
            continue
        ei = E_CARBON * math.pi / 64 * (od ** 4 - di ** 4) * 1e-12
        if w * h ** 4 / (8 * ei) <= DEFLECTION * h:
            return (od, di)
    return None


TAIL_PARTS = ("stab", "elevator", "fin", "rudder")


def tip_mass(d: Design) -> float:
    """What hangs on the boom's end: the tail surfaces (re-sized), the tail mount,
    the motor mount, the motor and the prop."""
    t, t0 = tail(d), tail(RECT)
    surfaces = sum(mass_of(n) for n in TAIL_PARTS) + G_STAB * (t["s_h"] - t0["s_h"]) \
        + G_FIN * (t["fin_area"] - t0["fin_area"])
    return surfaces + sum(mass_of(n) for n in ("tail_mount", "motor_mount", "motor", "prop"))


def boom_mode(d: Design, boom: tuple) -> float:
    """First bending mode of the boom with the tail, motor and prop on its end (Hz),
    as design.boom_options: a cantilever from the fuselage socket's end."""
    od, di = boom
    L = layout(d)
    free = (L.x_mp - L.pod_len) / 1000
    ei = E_CARBON * math.pi / 64 * (od ** 4 - di ** 4) * 1e-12
    g_mm = D.rod_mass(od, 1.0, di)
    m = tip_mass(d) / 1000 + 0.24 * g_mm * free     # g_mm (g/mm) x free (m) x 1000 / 1000 = kg
    return math.sqrt(3 * ei / free ** 3 / m) / (2 * math.pi)


def servo_y(d: Design) -> float:
    """The aileron servo's base: beside the centre section, or at the outer panel's root."""
    return (d.y_break if d.rect_panels > 1 else d.centre / 2) + 3.0


def fuselage_estimate(L) -> float:
    est = dict((n, g) for n, g, _ in D.structure_estimate(L.p, L))
    return est["fuselage"] + est["lid"]


def holes(d: Design) -> float:
    """Wall round both spar holes, per mm of panel."""
    return math.pi * (d.spar[0] + 0.2 + 2.0 + 0.2)


def panel_walls(d: Design) -> float:
    """Wall area of the wing panels, both sides (mm²). A panel with an aileron
    has its skin cut away behind the hinge and a servo pocket: its skin per
    chord (P_AIL) comes from the single boom's CAD panel. An inner panel keeps
    the whole skin and carries the aileron lead's channel."""
    c, ct = d.c_root, d.c_tip
    if d.rect_panels == 1:
        L = d.panel
        return 2 * (P_AIL * c * L + holes(d) * L + 2 * SECTION_AREA * c * c)
    L = d.panel
    inner = PERIMETER * c * L + (holes(d) + math.pi * 4.0) * L + 2 * SECTION_AREA * c * c
    outer = P_AIL * (c + ct) / 2 * L + holes(d) * (L - 5) + SECTION_AREA * (c * c + ct * ct)
    return 2 * (inner + outer)


def weight(d: Design) -> dict:
    """The single boom's CAD weight with the wing, spars, tail, boom, pushrods,
    motor wires, fuselage and the aileron leads re-sized."""
    t, t0 = tail(d), tail(RECT)
    L, L0 = layout(d), layout(RECT)
    b = d.span
    y_ail0 = d.y_break + 10.0 if d.rect_panels > 1 else d.centre / 2 + 10.0
    ail_len = b / 2 - 6.0 - y_ail0
    ail_c = 0.25 * (d.chord(y_ail0) + d.chord(b / 2 - 6)) / 2
    rods = lambda L_: (2 * D.pushrod_mass(L_.x_hinge - L_.servo_elev[0])   # noqa: E731
                       + D.motor_wire_mass(L_.x_mp - L_.esc_x))
    g = {
        "wing panels": G_WALL * panel_walls(d) - 2 * mass_of("wing_R"),
        "wing centre": G_CENTRE * d.centre * d.c_root * (d.c_root / 97.0) - mass_of("wing_centre"),
        "ailerons": 2 * G_AILERON * ail_len * ail_c - 2 * mass_of("aileron_R"),
        "main spar": D.rod_mass(d.spar[0], b - 10, d.spar[1]) - mass_of("spar_main"),
        "rear spar": D.rod_mass(2.0, b - 10) - mass_of("spar_rear"),
        "tail plates": G_STAB * (t["s_h"] - t0["s_h"]) + G_FIN * (t["fin_area"] - t0["fin_area"]),
        "boom": D.rod_mass(d.boom[0], L.boom_len, d.boom[1]) - mass_of("boom"),
        "pushrods, motor wires": rods(L) - rods(L0),
        "fuselage": (fuselage_estimate(L) - fuselage_estimate(L0)) * FUS_RATIO,
        "aileron lead extensions": 2 * D.motor_wire_mass(1.0) * (servo_y(d) - servo_y(RECT)),   # 3 wires each
    }
    total = SINGLE["auw_g"] + sum(g.values())
    return {"total": total, "delta": g}


DETAILS = {p.name: p.value for p in AF.params()}
DETAILS.update({k: float(val) for k, val in STUDY["details"].items()})
DETAILS.update(single_boom=1.0, boattail=0.0, joiners_hidden=0.0)


@lru_cache(maxsize=None)
def _other_items(d: Design, v: float):
    """The single boom's drag build-up (aero/models/airframe.py) for everything
    but the wing and the junctions, on this plane's layout."""
    L = layout(d)
    p = L.p
    ch = L.c_h / 1000
    tail_wet = 2 * (L.b_h * L.c_h + L.fin_plan) * 1e-6
    k_tail = K.cf(K.reynolds(v, ch), AF.LAMINAR_WING) * K.ff_wing(p.plate / L.c_h, 0.3)
    items = [K.Item("tail plates, skin friction", k_tail * tail_wet, "friction",
                    f"stabiliser {L.b_h:.0f} x {L.c_h:.0f} mm, one fin {L.fin_h:.0f} mm tall")]
    return items + AF._single_boom_items(p, L, DETAILS, v)


class Plane:
    def __init__(self, d: Design, mass_g: float = None):
        self.d = d
        self.mass = mass_g if mass_g is not None else weight(d)["total"]
        self.w = self.mass / 1000 * G
        self.s = d.area * 1e-6
        L = layout(d)
        self.fus_cover = 2 * L.half_w                       # the fuselage covers this much of the centre's underside
        # maximum lift: the single boom's, scaled by the planform's first-stall lift
        self.clmax = SINGLE_CLMAX * first_stall_cl(d, 9.0) / first_stall_cl(RECT, 9.0)
        self.e_scale = e_scale()

    def drag(self, v: float, n: float = 1.0) -> dict:
        d, q = self.d, 0.5 * RHO * v * v
        CL = n * self.w / (q * self.s)
        ll = lifting_line(d, 70e3)
        cl = ll["cl_a"] * CL + ll["cl_b"]
        re = v * ll["c"] / 1000 / NU
        cd = section_cd(cl, re)
        y = ll["y"]
        # profile drag: integrate cd c over the half span (stations run tip to root), twice
        order = np.argsort(y)
        ys, cs, cds = y[order], ll["c"][order], cd[order]
        ys = np.concatenate([[0.0], ys, [d.span / 2]])
        f = np.concatenate([[cds[0] * cs[0]], cds * cs, [cds[-1] * d.c_tip]])
        cda_wing = 2 * np.trapezoid(f, ys) * 1e-6
        cda_wing -= 0.5 * self.fus_cover * d.c_root * 1e-6 * float(cds[0])   # under the fuselage's top
        cdi = (ll["k2"] * CL * CL + ll["k1"] * CL + ll["k0"]) / self.e_scale
        induced = q * self.s * cdi
        items = _other_items(d, round(v, 3))
        other = sum(i.cda for i in items)
        fp = cda_wing + sum(i.cda for i in items if i.group in ("friction", "pressure"))
        total = q * (cda_wing + other + 0.10 * fp) + induced
        return {"drag": total, "induced": induced, "wing": q * cda_wing, "CL": CL, "cl_max_local": float(np.max(cl)),
                "items": [("wing profile", q * cda_wing)] + [(i.name, q * i.cda) for i in items]
                + [("junctions", q * 0.10 * fp), ("induced", induced)]}


KIT = AF.KIT
# the single boom's panel: what is left of its skin per chord once the holes and end ribs are taken off
P_AIL = (mass_of("wing_R") / G_WALL - holes(RECT) * RECT.panel - 2 * SECTION_AREA * 97.0 ** 2) / (97.0 * RECT.panel)
X_OFFSET = 0.0
X_OFFSET = SINGLE["x_le"] - layout(RECT).x_le                       # CAD against design.py, the single boom
layout.cache_clear()
SINGLE_MARGIN = seat_margin(layout(RECT))
FUS_RATIO = sum(mass_of(n) for n in ("fuselage_front", "fuselage_rear", "lid")) / fuselage_estimate(layout(RECT))
SINGLE_MODE = boom_mode(RECT, (10.0, 9.0))

# the single boom's 3D maximum lift (aero/variants.py: the design's 0.95 x the section's ratio)
SINGLE_CLMAX = AF.BASE.cl_max * STUDY["sections"]["optimized"]["clmax"] / STUDY["sections"]["baseline"]["clmax"]


def fly(p: Plane, t0: float) -> dict:
    stall = math.sqrt(2 * p.w / (RHO * p.s * p.clmax))
    v_pitch = AF.KIT.motor_kv * AF.KIT.v_loaded * AF.KIT.rpm_frac / 60 * AF.KIT.prop_pitch_in * 0.0254
    rows = []
    for v in SPEEDS:
        if v < stall * 1.05:
            continue
        dr = p.drag(float(v))
        power = dr["drag"] * v / AF.ETA_PROP + AF.AVIONICS_W
        thrust = max(0.0, t0 * (1 - v / v_pitch))
        rows.append({"v": float(v), "drag": dr["drag"], "power": power, "ld": p.w / dr["drag"],
                     "sink": dr["drag"] * v / p.w, "climb": (thrust - dr["drag"]) * v / p.w,
                     "endurance": 0.8 * AF.KIT.batt_wh / power * 60})
    cruise = min(rows, key=lambda r: abs(r["v"] - V_CRUISE))
    ld = max(rows, key=lambda r: r["ld"])
    pmin = min(rows, key=lambda r: r["power"])
    sink = min(rows, key=lambda r: r["sink"])
    top = next((r0["v"] + (r1["v"] - r0["v"]) * r0["climb"] / (r0["climb"] - r1["climb"])
                for r0, r1 in zip(rows, rows[1:]) if r0["climb"] >= 0 > r1["climb"]), rows[-1]["v"])
    return {"mass": p.mass, "stall": stall, "power_cruise": cruise["power"], "drag_cruise": cruise["drag"],
            "endurance_cruise": cruise["endurance"], "ld_max": ld["ld"], "v_ld_max": ld["v"],
            "power_min": pmin["power"], "v_power_min": pmin["v"], "endurance_max": pmin["endurance"],
            "sink_min": sink["sink"], "v_sink_min": sink["v"], "top": top, "clmax": p.clmax,
            "curves": {k: [round(r[k], 4) for r in rows] for k in ("v", "drag", "power", "ld", "sink")}}


# --------------------------------------------------------------------------
# The search


SV = D.Servo()
SERVO_MARGIN = 1.0                 # mm left over between the spars beside the SG90's tabs


def servo_room(d: Design) -> float:
    """Room between the spars for the aileron SG90's tabs, less the tabs and the
    margin (mm). The servo sits at the outer panel's root, shaft outboard; its
    tabs reach about 22 mm out, where the taper has already narrowed the chord
    and the rear spar (at 70 % of the local chord) has moved forward."""
    y_tab = d.y_break + 3.0 + SV.tab_z + SV.tab_t + 0.2
    c = float(d.chord(y_tab))
    room = (0.70 - 0.25) * c - (d.spar[0] + 2.0) / 2 - 0.8
    return room - SV.tab_span - SERVO_MARGIN


def candidate(span, c_root, taper, tail_arm, washout=WASHOUT) -> tuple[Design | None, str]:
    centre = max(CENTRE_MIN, span - 4 * PANEL_MAX)
    d = Design(span=span, c_root=c_root, taper=taper, washout=washout, tail_arm=tail_arm,
               centre=centre, rect_panels=2)
    if d.panel > PANEL_MAX + 1e-6:
        return None, "panel longer than the bed"
    if tail(d)["fin_h"] > PANEL_MAX - 6:
        return None, "fin taller than the bed"
    for _ in range(3):              # the stretch, the boom and the spar each change the balance and the weight
        for _ in range(4):          # stretch the fuselage until the wing sits down as far ahead of the cone as now
            miss = SINGLE_MARGIN - seat_margin(layout(d))
            if miss <= 0.5 and (miss >= -2.0 or d.stretch == 0):
                break
            d = replace(d, stretch=max(0.0, float(math.ceil(d.stretch + miss / 0.9))))
        boom = next((b for b in BOOMS if boom_mode(d, b) >= SINGLE_MODE), None)
        if boom is None:
            return None, "no boom keeps the tail as stiff as the single boom's"
        d = replace(d, boom=boom)
        sp = spar_for(d, weight(d)["total"])
        if sp is None:
            return None, "no spar fits the tip"
        d = replace(d, spar=sp)
    if SINGLE_MARGIN - seat_margin(layout(d)) > 1.0:
        return None, "the wing does not balance over the fuselage"
    L = layout(d)
    if L.pod_len - (L.boom_x0 + 2.0 if d.stretch > 0 else L.batt_max - 10.0) > PANEL_MAX:
        return None, "the fuselage's rear half is taller than the bed"
    if servo_room(d) < 0:
        return None, "the SG90's tabs do not fit between the spars"
    return d, ""


GRID = {"span": [float(b) for b in range(560, 841, 20)], "c_root": [82.0, 86.0, 90.0, 94.0, 98.0, 104.0],
        "taper": [0.5, 0.6, 0.7, 0.8, 1.0], "tail_arm": [240.0, 270.0, 300.0, 340.0, 380.0, 420.0]}
FINE = {"span": [float(b) for b in range(680, 761, 10)], "c_root": [84.0, 86.0, 88.0, 90.0, 92.0],
        "taper": [0.7, 0.75, 0.8, 0.85, 0.9], "tail_arm": [220.0, 240.0, 260.0, 280.0, 300.0]}
LD_BAND = 0.01                     # designs within 1 % of the best glide count as equal; the lightest wins
T0 = 223.2 / 1000 * G              # aero/variants.py: 1:1 static thrust for the as-built plane


def sweep(grid: dict, limit: dict) -> tuple[list, dict]:
    rows, why = [], {}
    for span in grid["span"]:
        for c_root in grid["c_root"]:
            for taper in grid["taper"]:
                for tail_arm in grid["tail_arm"]:
                    d, reason = candidate(span, c_root, taper, tail_arm)
                    if d is None:
                        why[reason] = why.get(reason, 0) + 1
                        continue
                    f = fly(Plane(d), T0)
                    ok = bool(f["stall"] <= STALL_LIMIT and f["power_cruise"] <= limit["power_cruise"])
                    if not ok:
                        why["more power at 13 m/s than the single boom"] = why.get(
                            "more power at 13 m/s than the single boom", 0) + 1
                    rows.append({"design": asdict(d), "fly": {k: float(v) for k, v in f.items() if k != "curves"},
                                 "ok": ok})
    return rows, why


def vlm_check(d: Design, cl: float) -> dict:
    """The planform in AeroSandbox's vortex lattice (a symmetric section, with the
    twist): its induced drag at the lift coefficient cl, against this lifting
    line's. Both are inviscid, so they compare like for like."""
    import aerosandbox as asb
    mm, b = 1e-3, d.span
    ys = [0.0, d.y_break, b / 2] if d.rect_panels > 1 else [0.0, b / 2]
    af = asb.Airfoil("naca0010")
    xs = [asb.WingXSec(xyz_le=[-0.25 * float(d.chord(y)) * mm, y * mm, 0], chord=float(d.chord(y)) * mm,
                       twist=float(d.twist(y)), airfoil=af) for y in ys]
    plane = asb.Airplane(wings=[asb.Wing(symmetric=True, xsecs=xs)], s_ref=d.area * mm * mm, b_ref=b * mm,
                         c_ref=d.mac * mm)

    def run(alpha):
        """CL and CDi in the Trefftz plane: the spanwise circulation fitted with an
        odd sine series, as aero/crosscheck.py does (the lattice's own near-field
        drag is less reliable)."""
        vel = 13.0
        vlm = asb.VortexLatticeMethod(plane, asb.OperatingPoint(velocity=vel, alpha=alpha),
                                      spanwise_resolution=24, chordwise_resolution=8)
        vlm.run()
        y = np.round(vlm.vortex_centers[:, 1], 7)
        yu = np.unique(y)
        gam = np.array([vlm.vortex_strengths[y == s_].sum() for s_ in yu])
        n = np.arange(1, 30, 2)
        bm = b * mm
        an = np.linalg.lstsq(2 * bm * vel * np.sin(np.outer(np.arccos(np.clip(-2 * yu / bm, -1, 1)), n)), gam,
                             rcond=None)[0]
        return math.pi * d.ar * an[0], math.pi * d.ar * float(np.sum(n * an * an))
    a, (c0, _) = 4.0, run(4.0)
    for _ in range(3):                                   # the angle for this CL
        c1, _ = run(a + 0.5)
        a += (cl - c0) / ((c1 - c0) / 0.5)
        c0, cdi = run(a)
    ll = lifting_line(d, 70e3)
    cdi_ll = ll["k2"] * c0 * c0 + ll["k1"] * c0 + ll["k0"]
    return {"cl": round(c0, 4), "cdi_vlm": round(cdi, 6), "cdi_ll": round(cdi_ll, 6),
            "e_vlm": round(c0 * c0 / (math.pi * d.ar * cdi), 4), "e_ll": round(c0 * c0 / (math.pi * d.ar * cdi_ll), 4)}


def summary(f: dict) -> dict:
    keys = ("mass", "stall", "clmax", "power_cruise", "drag_cruise", "endurance_cruise", "ld_max", "v_ld_max",
            "power_min", "v_power_min", "endurance_max", "sink_min", "v_sink_min", "top")
    return {k: round(float(f[k]), 4) for k in keys}


def describe(d: Design) -> dict:
    t = tail(d)
    return {**asdict(d), "c_tip": round(d.c_tip, 1), "area_dm2": round(d.area / 1e4, 3), "ar": round(d.ar, 2),
            "mac": round(d.mac, 1), "panel": round(d.panel, 1), "y_break": round(d.y_break, 1),
            "tail": {k: round(v, 1) for k, v in t.items()}, "boom_len": round(layout(d).boom_len, 1),
            "x_le": round(layout(d).x_le, 1), "pod_len": round(layout(d).pod_len, 1),
            "seat_margin": round(seat_margin(layout(d)), 1),
            "boom_mode_hz": round(boom_mode(d, d.boom), 1), "servo_room": round(servo_room(d), 2),
            "e_ll": round(lifting_line(d, 70e3)["e_ll"], 4),
            "weight": {k: round(float(v), 2) for k, v in weight(d)["delta"].items()}}


def main():
    t_clock = time.time()
    polars()
    single = Plane(RECT, SINGLE["auw_g"])
    base = fly(single, T0)
    print(f"single boom: {base['mass']:.1f} g, stall {base['stall']:.2f}, {base['power_cruise']:.2f} W at 13, "
          f"L/D {base['ld_max']:.2f} at {base['v_ld_max']}", flush=True)
    coarse, why_c = sweep(GRID, base)
    print(f"  coarse: {len(coarse)} designs, {time.time() - t_clock:.0f} s", flush=True)
    fine, why_f = sweep(FINE, base)
    print(f"  fine: {len(fine)} designs, {time.time() - t_clock:.0f} s", flush=True)
    good = [r for r in coarse + fine if r["ok"]]
    top = max(r["fly"]["ld_max"] for r in good)
    best = min((r for r in good if r["fly"]["ld_max"] >= (1 - LD_BAND) * top), key=lambda r: r["fly"]["mass"])
    d = Design(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in best["design"].items()})
    # the twist for this planform: the best glide, as long as the stall starts inboard of the ailerons
    mass = weight(d)["total"]
    wash = []
    for w in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
        dw = replace(d, washout=w)
        f = fly(Plane(dw, mass), T0)
        cl_s, y_s = first_stall(dw, 9.0)
        wash.append({"washout": w, **summary(f), "e_ll": round(lifting_line(dw, 70e3)["e_ll"], 4),
                     "stall_y": round(y_s, 1), "inboard": bool(y_s < d.y_break + 10.0)})
    pick = max((r for r in wash if r["inboard"]), key=lambda r: r["ld_max"])
    d = replace(d, washout=pick["washout"])
    chosen = fly(Plane(d), T0)
    # the best design at each span, to show the trend
    per_span = []
    for span in sorted({r["design"]["span"] for r in good}):
        r = max((r for r in good if r["design"]["span"] == span), key=lambda r: r["fly"]["ld_max"])
        per_span.append({"span": span, "c_root": r["design"]["c_root"], "taper": r["design"]["taper"],
                         "tail_arm": r["design"]["tail_arm"], **summary(r["fly"])})
    # the drag at 13 m/s and at the best glide, item by item
    items = {name: {"single": [(n, round(v, 5)) for n, v in p.drag(v_)["items"]]}
             for name, v_, p in (("cruise", V_CRUISE, single), ("glide", chosen["v_ld_max"], single))}
    ext = Plane(d)
    items["cruise"]["extreme"] = [(n, round(v, 5)) for n, v in ext.drag(V_CRUISE)["items"]]
    items["glide"]["extreme"] = [(n, round(v, 5)) for n, v in ext.drag(chosen["v_ld_max"])["items"]]
    # the lifting line against a vortex lattice, at each plane's lift coefficient for the cruise and the best glide
    check = {}
    for key, dd, m, f in (("single", RECT, SINGLE["auw_g"], base), ("extreme", d, chosen["mass"], chosen)):
        w = m / 1000 * G
        check[key] = [vlm_check(dd, w / (0.5 * RHO * v * v * dd.area * 1e-6)) for v in (V_CRUISE, f["v_ld_max"])]
    print("vortex lattice:", check, flush=True)
    out = {"generated": time.strftime("%Y-%m-%d"), "n_crit": N_CRIT, "cruise_v": V_CRUISE, "vlm": check,
           "ld_top": round(top, 3), "ld_band": LD_BAND, "x_offset": round(X_OFFSET, 1),
           "single_margin": round(SINGLE_MARGIN, 1),
           "stall_limit": STALL_LIMIT, "washout_default": WASHOUT, "deflection": DEFLECTION,
           "single_mode_hz": round(SINGLE_MODE, 1), "e_scale": round(e_scale(), 4),
           "single": {"design": describe(RECT), **summary(base), "curves": base["curves"],
                      "stall_y": round(first_stall(RECT, 9.0)[1], 1)},
           "chosen": {"design": describe(d), **summary(chosen), "curves": chosen["curves"]},
           "counts": {"coarse": len(GRID["span"]) * len(GRID["c_root"]) * len(GRID["taper"]) * len(GRID["tail_arm"]),
                      "fine": len(FINE["span"]) * len(FINE["c_root"]) * len(FINE["taper"]) * len(FINE["tail_arm"]),
                      "flown": len(coarse) + len(fine), "passed": len(good),
                      "dropped": {k: why_c.get(k, 0) + why_f.get(k, 0) for k in set(why_c) | set(why_f)}},
           "per_span": per_span, "washout": wash, "items": items, "grid": GRID, "fine_grid": FINE}
    (OUT / "extreme.json").write_text(json.dumps(out, indent=1))
    (OUT / "EXTREME.md").write_text(report(out))
    print(json.dumps(out["chosen"]["design"], indent=1))
    print({k: v for k, v in out["chosen"].items() if k not in ("design", "curves")})
    print(f"wrote aero/study/extreme.json and EXTREME.md in {time.time() - t_clock:.0f} s")


def report(o: dict) -> str:
    s, c = o["single"], o["chosen"]
    d, d0 = c["design"], s["design"]
    rows = [("Weight, g", "mass", "{:.1f}"), ("Maximum lift (3D)", "clmax", "{:.3f}"), ("Stall, m/s", "stall", "{:.2f}"),
            ("Drag at 13 m/s, N", "drag_cruise", "{:.3f}"), ("Power at 13 m/s, W", "power_cruise", "{:.2f}"),
            ("Flight time at 13 m/s, min", "endurance_cruise", "{:.1f}"),
            ("Best glide ratio", "ld_max", "{:.2f}"), ("... at, m/s", "v_ld_max", "{:.2f}"),
            ("Least power, W", "power_min", "{:.2f}"), ("... at, m/s", "v_power_min", "{:.2f}"),
            ("Longest flight, min", "endurance_max", "{:.1f}"), ("Least sink, motor off, m/s", "sink_min", "{:.3f}"),
            ("Top speed, m/s", "top", "{:.1f}")]
    L = ["# Kipinä, extreme aero: the design study", "",
         "Generated by `aero/extreme.py`. Do not edit by hand. Same electronics, motor, prop and battery as the "
         "single boom, the same optimized section (kipina-opt), every part on a 180 mm bed.", "",
         "## The planform it picks", "",
         "| | Single boom | Extreme aero |", "|---|---|---|",
         f"| Span | {d0['span']:.0f} mm | {d['span']:.0f} mm |",
         f"| Chord | {d0['c_root']:.0f} mm, straight | {d['c_root']:.0f} mm root, {d['c_tip']:.0f} mm tip "
         f"(taper {d['taper']:.2f} on the outer panels) |",
         f"| Wing area | {d0['area_dm2']:.2f} dm² | {d['area_dm2']:.2f} dm² |",
         f"| Aspect ratio | {d0['ar']:.1f} | {d['ar']:.1f} |",
         f"| Panels | centre {d0['centre']:.0f} mm, one {d0['panel']:.0f} mm panel a side | centre {d['centre']:.0f} mm, "
         f"two {d['panel']:.0f} mm panels a side, the taper from {d['y_break']:.0f} mm out |",
         f"| Washout | none | {d['washout']:.0f}° on the outer panels |",
         f"| Span efficiency (lifting line) | {d0['e_ll']:.3f} | {d['e_ll']:.3f} |",
         f"| Main spar | {d0['spar'][0]:.0f} mm carbon rod | {d['spar'][0]:.0f}x{d['spar'][1]:.0f} mm carbon tube |",
         f"| Tail arm | {d0['tail_arm']:.0f} mm | {d['tail_arm']:.0f} mm |",
         f"| Stabiliser | {d0['tail']['b_h']:.0f} x {d0['tail']['c_h']:.0f} mm | {d['tail']['b_h']:.0f} x {d['tail']['c_h']:.0f} mm |",
         f"| Fin | {d0['tail']['fin_h']:.0f} x {d0['tail']['c_h']:.0f} mm | {d['tail']['fin_h']:.0f} x {d['tail']['c_h']:.0f} mm |",
         f"| Boom | {d0['boom'][0]:.0f}x{d0['boom'][1]:.0f} mm, {d0['boom_len']:.0f} mm, {d0['boom_mode_hz']:.0f} Hz | "
         f"{d['boom'][0]:.0f}x{d['boom'][1]:.0f} mm, {d['boom_len']:.0f} mm, {d['boom_mode_hz']:.0f} Hz |", "",
         "## How it flies (study model)", "",
         "| | Single boom | Extreme aero | change |", "|---|---|---|---|"]
    for label, k, f in rows:
        L.append(f"| {label} | {f.format(s[k])} | {f.format(c[k])} | {c[k] - s[k]:+.3g} |")
    L += ["", "Where the weight goes, grams against the single boom:", "",
          "| | g |", "|---|---|"]
    for k, v in d["weight"].items():
        L.append(f"| {k} | {v:+.1f} |")
    L += ["", "## Drag, item by item", "",
          f"At 13 m/s and at the extreme plane's best glide ({c['v_ld_max']:.2f} m/s), newtons.", "",
          "| | single, 13 m/s | extreme, 13 m/s | single, best glide speed | extreme, best glide speed |",
          "|---|---|---|---|---|"]
    it = o["items"]
    names = [n for n, _ in it["cruise"]["single"]]
    for n in names:
        vals = [dict(it[a][b])[n] for a, b in (("cruise", "single"), ("cruise", "extreme"), ("glide", "single"),
                                                ("glide", "extreme"))]
        L.append(f"| {n} | " + " | ".join(f"{v:.4f}" for v in vals) + " |")
    tot = [sum(v for _, v in it[a][b]) for a, b in (("cruise", "single"), ("cruise", "extreme"), ("glide", "single"),
                                                      ("glide", "extreme"))]
    L.append("| **total** | " + " | ".join(f"**{v:.4f}**" for v in tot) + " |")
    L += ["", "## The search", "",
          f"{o['counts']['coarse']} designs on a coarse grid (span {min(o['grid']['span']):.0f}-"
          f"{max(o['grid']['span']):.0f} mm, root chord {min(o['grid']['c_root']):.0f}-{max(o['grid']['c_root']):.0f} mm, "
          f"taper {min(o['grid']['taper'])}-{max(o['grid']['taper'])}, tail arm {min(o['grid']['tail_arm']):.0f}-"
          f"{max(o['grid']['tail_arm']):.0f} mm) and {o['counts']['fine']} on a fine one round the best. "
          f"{o['counts']['flown']} could be built and were flown; {o['counts']['passed']} passed. Dropped:", ""]
    for k, v in sorted(o["counts"]["dropped"].items(), key=lambda kv: -kv[1]):
        L.append(f"- {k}: {v}")
    L += ["", "The best design at each span:", "",
          "| span | root chord | taper | tail arm | weight g | stall | power at 13 m/s | best glide | least power W | least sink |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in o["per_span"]:
        L.append(f"| {r['span']:.0f} | {r['c_root']:.0f} | {r['taper']:.2f} | {r['tail_arm']:.0f} | {r['mass']:.1f} | "
                 f"{r['stall']:.2f} | {r['power_cruise']:.2f} | {r['ld_max']:.2f} | {r['power_min']:.2f} | {r['sink_min']:.3f} |")
    wide = max(r["span"] for r in o["per_span"])
    L += ["", f"Nothing wider than {wide:.0f} mm passed. A wider wing needs a stiffer spar, and the stiffer tube no "
          "longer fits the depth of a tip that is narrow enough to keep the area down; with a broader tip the "
          "area, the weight and the tail grow, and the plane needs more power at 13 m/s than the single boom.", "",
          "## Washout", "",
          f"The search flies every planform with {o['washout_default']:.0f}° of twist on the outer panels. The "
          "chosen one, at its weight, with the twist from 0 to 4° (tip nose-down), and where along the half "
          f"span the first section reaches its maximum lift (the outer panels start at {d['y_break']:.0f} mm):", "",
          "| washout | maximum lift | stall | stall starts at | best glide | power at 13 m/s |",
          "|---|---|---|---|---|---|"]
    for r in o["washout"]:
        mark = "**" if r["washout"] == d["washout"] else ""
        L.append(f"| {mark}{r['washout']:g}°{mark} | {r['clmax']:.3f} | {r['stall']:.2f} | {r['stall_y']:.0f} mm "
                 f"({'inboard' if r['inboard'] else 'on the aileron'}) | {r['ld_max']:.2f} | {r['power_cruise']:.2f} |")
    root_first = all(r["stall_y"] < 0.1 * d["span"] / 2 for r in o["washout"])
    L += ["", ("Even untwisted, this planform reaches its maximum lift first at the root (the inner panels keep "
               "the full chord, and the tips fly at a lower Reynolds number, where the section's maximum lift is "
               "lower), as the single boom's straight wing does. So the twist is not needed for a root-first "
               "stall here; it unloads the tips, which costs a little maximum lift and gains a little glide. "
               if root_first else
               "Twist unloads the tips, so the stall starts further in, and costs a little maximum lift. ")
          + "The design keeps the twist that glides best while the stall still starts inboard of the ailerons, "
          f"so they keep working: {d['washout']:g}°.", "",
          "## The lifting line against a vortex lattice", "",
          "Both wings alone in AeroSandbox's vortex lattice (24 panels across each half, 8 along the chord, "
          "the twist included), at each plane's lift coefficient at 13 m/s and at its best glide. Both "
          "methods are inviscid, so the induced drag compares like for like:", "",
          "| | lift coefficient | induced drag, lifting line | vortex lattice | span efficiency, lifting line | vortex lattice |",
          "|---|---|---|---|---|---|"]
    for key, name in (("single", "Single boom"), ("extreme", "Extreme aero")):
        for r, where in zip(o["vlm"][key], ("13 m/s", "best glide")):
            L.append(f"| {name}, {where} | {r['cl']:.3f} | {r['cdi_ll']:.5f} | {r['cdi_vlm']:.5f} | {r['e_ll']:.3f} | "
                     f"{r['e_vlm']:.3f} |")
    sv, ev = o["vlm"]["single"][0], o["vlm"]["extreme"][0]
    ind = dict(o["items"]["cruise"]["extreme"])["induced"]
    tot = sum(v for _, v in o["items"]["cruise"]["extreme"])
    r_ll, r_vlm = ev["cdi_ll"] / sv["cdi_ll"], ev["cdi_vlm"] / sv["cdi_vlm"]
    L += ["", "The design's induced drag scales these by the same factor for both planes (the 0.8 span "
          "efficiency it takes for the straight wing with the fuselage), so what matters is the ratio between "
          f"them. At 13 m/s the extreme wing has {r_ll:.3f} of the single boom's induced drag on the lifting line "
          f"and {r_vlm:.3f} on the vortex lattice: the lifting line is {abs(r_vlm / r_ll - 1) * 100:.0f} % "
          f"{'kinder' if r_vlm > r_ll else 'harder'} on the extreme wing, which is "
          f"{abs(r_vlm / r_ll - 1) * ind:.4f} N, {abs(r_vlm / r_ll - 1) * ind / tot * 100:.1f} % of its drag.", "",
          "## Method and limits", "",
          "- Span loading: Glauert's lifting line, 40 terms, the section's own lift slope (capped at 2π), with the "
          "twist. Its span efficiency scales the design's 0.8 at the rectangular AR 5 wing "
          f"(factor {o['e_scale']:.3f}), so the fuselage's and the viscous share stay as before.",
          "- Profile drag: NeuralFoil on kipina-opt at each of 40 stations' own lift coefficient and Reynolds "
          f"number (n_crit {o['n_crit']}).",
          "- Maximum lift: the single boom's 0.967, scaled by the lift the planform carries when its first section "
          "reaches its maximum, against the rectangular wing's.",
          "- Everything else: the single boom's drag build-up (aero/models/airframe.py), tail plates and boom re-sized.",
          f"- Spar: the lightest carbon tube that keeps the 1 g bending under {o['deflection'] * 100:.1f} % of the "
          "half span (E 70 GPa, even load) and fits the tip's depth with both skins.",
          f"- Boom: the lightest roll-wrapped tube whose first bending mode, with the tail, motor and prop on its "
          f"end, is no lower than the single boom's {o['single_mode_hz']:.0f} Hz.",
          "- Weight: the single boom's CAD weight, with each re-sized part at the CAD's own grams per mm² or per mm. "
          "`build.py --variant extreme` then builds it and weighs the real parts.",
          "- Handbook uncertainty is the same as the drag optimizer's (perhaps ±30 % on absolute drag); the "
          "differences between the two planes are more trustworthy than either number.", ""]
    return "\n".join(L)


if __name__ == "__main__":
    main()
