#!/usr/bin/env python3
"""Fuselage cross-section: which outline holds the single-boom Kipinä's
contents with the least skin?

    python3 aero/section.py          # airframe/variants/single-boom/SECTION.md and section.png

The contents are drawn as rectangles in the y-z plane, z up from the floor:
the battery with its strap slots, the flight controller on its standoffs, the
two SG90s lying on their sides with the horns swinging up to the boom, the
carbon boom's socket under the wing, and the ESC lying flat. For each family
of outlines (circle, ellipse, superellipses, a flat-topped box with rounded
corners) it finds the smallest one that holds them all, then adds the wall.

Skin friction scales with the perimeter (the wetted area per unit length), so
that is what it minimises. The frontal area and the width of flat top the wing
can sit on are reported alongside.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "airframe"))

import numpy as np  # noqa: E402

import dragkit as K  # noqa: E402
from design import Kit  # noqa: E402

OUT = ROOT / "airframe" / "variants" / "single-boom"
WALL, GAP = 0.8, 0.3                     # printed wall; clearance round the contents
BOOM_AXIS = 17.2                         # boom centre-line above the floor (the pod's depth less the floor)
BOOM_BOSS = 6.4                          # socket radius round the 10 mm boom
TOP = BOOM_AXIS + BOOM_BOSS              # the wing seat, flush with the socket top
FUSELAGE_LEN = 300.0                     # nose to the end of the tail cone, for the drag numbers


def contents(k: Kit = Kit()):
    """(name, y0, y1, z0, z1) of everything the section has to hold, z from the floor's top."""
    sv = k.servo
    across = sv.height + sv.boss_h + sv.spline_h + 0.9      # base to the horn screw
    return [
        ("battery", -k.batt_w / 2, k.batt_w / 2, 0.0, k.batt_h),
        ("strap slots", -17.0, 17.0, 0.0, 0.8),
        ("flight controller", -k.fc_wid / 2, k.fc_wid / 2, 0.0, 3.0 + k.fc_hgt),
        ("servo on its side", -across / 2, across / 2, 0.0, sv.width),
        ("servo horn swing", across / 2 - 4.5, across / 2 - 3.0, 0.0, BOOM_AXIS + 2.0),
        ("boom socket", -BOOM_BOSS, BOOM_BOSS, BOOM_AXIS - BOOM_BOSS, TOP),
        ("ESC, flat", -k.esc_wid / 2, k.esc_wid / 2, 0.0, k.esc_thk),
    ]


def corners(items):
    pts = []
    for _, y0, y1, z0, z1 in items:
        pts += [(y0 - GAP, z0 - GAP), (y1 + GAP, z0 - GAP), (y0 - GAP, z1 + GAP), (y1 + GAP, z1 + GAP)]
    pts = np.array(pts)
    pts[:, 1] = np.minimum(pts[:, 1], TOP)               # the top is the wing seat, no clearance above it
    return pts


def superellipse(a, b, zc, n, m=720):
    t = np.linspace(0, 2 * math.pi, m, endpoint=False)
    c, s = np.cos(t), np.sin(t)
    return np.column_stack([a * np.sign(c) * np.abs(c) ** (2 / n), zc + b * np.sign(s) * np.abs(s) ** (2 / n)])


def rounded_box(w, z0, z1, r_top, r_bot, m=90):
    """Flat-topped box, width w, from z0 to z1, corner radii r_top and r_bot."""
    pts = []
    for (cy, cz, r, a0) in ((w / 2 - r_top, z1 - r_top, r_top, 0), (-w / 2 + r_top, z1 - r_top, r_top, 90),
                            (-w / 2 + r_bot, z0 + r_bot, r_bot, 180), (w / 2 - r_bot, z0 + r_bot, r_bot, 270)):
        for a in np.linspace(math.radians(a0), math.radians(a0 + 90), m if r > 0 else 1):
            p = (cy + r * math.cos(a), cz + r * math.sin(a))
            if not pts or math.dist(p, pts[-1]) > 1e-9:          # a sharp corner is one point, not 90
                pts.append(p)
    return np.array(pts)


def perimeter(poly):
    d = np.diff(np.vstack([poly, poly[:1]]), axis=0)
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def area(poly):
    y, z = poly[:, 0], poly[:, 1]
    return float(0.5 * abs(np.dot(y, np.roll(z, -1)) - np.dot(z, np.roll(y, -1))))


def flat_top(poly, tol=0.3):
    """Width of the outline's top that lies within tol of its highest point."""
    top = poly[:, 1].max()
    y = poly[poly[:, 1] >= top - tol, 0]
    return float(y.max() - y.min()) if len(y) else 0.0


def offset(poly, t):
    """Outer outline a wall t outside a convex, counter-clockwise polygon."""
    n = len(poly)
    out = []
    for i in range(n):
        p0, p1, p2 = poly[i - 1], poly[i], poly[(i + 1) % n]
        e0, e1 = p1 - p0, p2 - p1
        n0 = np.array([e0[1], -e0[0]]) / (np.hypot(*e0) or 1)
        n1 = np.array([e1[1], -e1[0]]) / (np.hypot(*e1) or 1)
        nm = n0 + n1
        nm /= np.hypot(*nm) or 1
        out.append(p1 + nm * t / max(float(np.dot(nm, n0)), 0.3))      # mitred, so sharp corners stay sharp
    return np.array(out)


def fit_superellipse(pts, n):
    """Smallest-perimeter |y/a|^n + |(z-zc)/b|^n <= 1 holding every point."""
    best = None
    y, z = np.abs(pts[:, 0]), pts[:, 1]
    for zc in np.linspace(z.min() + 1, z.max() - 1, 160):
        dz = np.abs(z - zc)
        for b in np.linspace(dz.max() + 0.05, dz.max() + 25, 160):
            rest = 1 - (dz / b) ** n
            if (rest <= 0).any():
                continue
            a = float((y / rest ** (1 / n)).max())
            poly = superellipse(a, b, zc, n, 240)
            p = perimeter(poly)
            if best is None or p < best[0]:
                best = (p, a, b, zc)
    _, a, b, zc = best
    return superellipse(a, b, zc, n)


def fit_box(pts, r_bot_max=None):
    """Flat-topped box round the points with the biggest corner radii that still hold them."""
    w = 2 * np.abs(pts[:, 0]).max()
    z0, z1 = pts[:, 1].min(), pts[:, 1].max()

    def holds(r_top, r_bot):
        for py, pz in pts:
            ay = abs(py)
            for cy, cz, r, above in ((w / 2 - r_top, z1 - r_top, r_top, True), (w / 2 - r_bot, z0 + r_bot, r_bot, False)):
                if ay > cy and ((pz > cz) if above else (pz < cz)) and math.hypot(ay - cy, pz - cz) > r + 1e-9:
                    return False
        return True
    r_top = max(r for r in np.arange(0, min(w, z1 - z0) / 2, 0.25) if holds(r, 0))
    r_bot = max(r for r in np.arange(0, (r_bot_max or min(w, z1 - z0) / 2), 0.25) if holds(r_top, r))
    return rounded_box(w, z0, z1, r_top, r_bot), r_top, r_bot


def main():
    k = Kit()
    items = contents(k)
    pts = corners(items)
    shapes = {}
    shapes["circle"] = fit_superellipse(pts, 2.0)
    # a circle is the superellipse with a = b: refit with that constraint
    best = None
    for zc in np.linspace(pts[:, 1].min(), pts[:, 1].max(), 400):
        r = float(np.hypot(pts[:, 0], pts[:, 1] - zc).max())
        if best is None or r < best[0]:
            best = (r, zc)
    shapes["circle"] = superellipse(best[0], best[0], best[1], 2.0)
    shapes["ellipse"] = fit_superellipse(pts, 2.0)
    shapes["superellipse, n 3"] = fit_superellipse(pts, 3.0)
    shapes["superellipse, n 4"] = fit_superellipse(pts, 4.0)
    box_poly, r_top, r_bot = fit_box(pts)
    shapes[f"flat-topped box, r {r_top:.0f} top, {r_bot:.1f} bottom"] = box_poly

    # skin friction per mm of perimeter, for the fuselage drag at the 13 m/s cruise
    v = 13.0
    rows = []
    for name, inner in shapes.items():
        outer = offset(inner, WALL)
        p, a = perimeter(outer), area(outer)
        d_eq = 2 * math.sqrt(a / math.pi)
        k_f = K.cf(K.reynolds(v, FUSELAGE_LEN / 1000), 0.1) * K.ff_body(FUSELAGE_LEN / d_eq)
        drag = K.q(v) * k_f * p * FUSELAGE_LEN * 1e-6
        rows.append({"name": name, "inner": inner, "outer": outer, "perimeter": p, "area": a,
                     "width": float(np.ptp(outer[:, 0])), "height": float(np.ptp(outer[:, 1])),
                     "flat": flat_top(outer), "drag": drag})
    ref = next(r for r in rows if r["name"].startswith("flat-topped"))
    OUT.mkdir(parents=True, exist_ok=True)
    lines = ["# Fuselage cross-section for the single-boom Kipinä", "",
             "Generated by `aero/section.py`. Do not edit by hand.", "",
             "The smallest outline of each family that holds the battery with its strap slots, the flight "
             f"controller on its standoffs, both SG90s lying on their sides, the boom socket and the ESC, "
             f"with {GAP} mm clearance and a {WALL} mm wall. Skin friction follows the perimeter; the drag "
             f"column is the skin friction and form drag of a {FUSELAGE_LEN:.0f} mm fuselage with that "
             "section at 13 m/s (the whole plane has about 0.23 N).", "",
             "| Outline | Outside, mm | Perimeter | Frontal area | Flat top for the wing | Fuselage drag at 13 m/s |",
             "|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda r: r["perimeter"]):
        lines.append(f"| {r['name']} | {r['width']:.1f} x {r['height']:.1f} | {r['perimeter']:.0f} mm "
                     f"({(r['perimeter'] / ref['perimeter'] - 1) * 100:+.0f} %) | {r['area'] / 100:.1f} cm² | "
                     f"{r['flat']:.0f} mm | {r['drag'] * 1000:.1f} mN ({(r['drag'] - ref['drag']) * 1000:+.1f}) |")
    lines += ["", "What goes inside, y across and z up from the floor (mm):", "", "| item | y | z |", "|---|---|---|"]
    for name, y0, y1, z0, z1 in items:
        lines.append(f"| {name} | {y0:.1f} to {y1:.1f} | {z0:.1f} to {z1:.1f} |")
    (OUT / "SECTION.md").write_text("\n".join(lines) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    order = sorted(rows, key=lambda r: r["perimeter"])
    fig, axes = plt.subplots(1, len(order), figsize=(3.0 * len(order), 3.9))
    ink, muted, accent = "#1B2226", "#6E7B80", "#d95926"
    for ax, r in zip(axes, order):
        for name, y0, y1, z0, z1 in items:
            ax.add_patch(Rectangle((y0, z0), y1 - y0, z1 - z0, fc="#C9D1D4", ec=muted, lw=0.6, alpha=0.7))
        o = np.vstack([r["outer"], r["outer"][:1]])
        ax.fill(o[:, 0], o[:, 1], fc="none", ec=accent if r is order[0] else ink, lw=1.6)
        title = r["name"].replace("flat-topped box, ", "flat-topped box\n").replace("superellipse, ", "superellipse ")
        ax.set_title(f"{title}\n{r['perimeter']:.0f} mm round", fontsize=9, color=ink)
        ax.set_xlim(-30, 30)
        ax.set_ylim(-16, 36)
        ax.set_aspect("equal", adjustable="box")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(OUT / "section.png", dpi=160)
    for r in order:
        print(f"{r['name']:40s} {r['width']:5.1f} x {r['height']:5.1f}  perimeter {r['perimeter']:6.1f}  "
              f"area {r['area']:6.0f}  flat top {r['flat']:4.0f}  drag {r['drag'] * 1000:5.2f} mN")


if __name__ == "__main__":
    main()
