#!/usr/bin/env python3
"""Dimensioned sketch of a modified Mark4 10" arm, from measured distances.

    python3 frames/mark4-v2-10/sketch/modified_arm.py      # needs ezdxf, matplotlib, shapely

Writes modified_arm.dxf (mm, layers PROFILE / HOLES / SLOTS), modified_arm.png and
modified_arm.svg next to this file.

Coordinates: u runs along the arm's long axis from the tip (u = 0) towards the frame's
centre, v runs across it. The spar-hole side is +v. The arm is the stock Mark4 V2 10"
front-right arm turned so its tip points left (not mirrored; the other three arms are
mirror images).

What is measured, and what is borrowed from the stock arm (frames/mark4-v2-10/dxf/arm.dxf):

  measured     tip to the motor hole's near edge 22.5; motor hole 8 mm; straight edge ends
               200 (side without spar holes) and 198 (spar-hole side); tip to the spar
               holes' near edge 65; tip to the arm holes' near edges 204 and 201.
  measured     also: the motor bolt slots are 5 mm end to end (width 3 mm, from stock)
  from stock   the tip outline, the lightening cut-out, the slot width, the arm width
               (about 10-11 mm), the spar holes (2 x 4 mm, 8.6 mm apart, on the bump), the
               arm holes (2 x 3.2 mm, 11.14 mm apart across the arm), the bump's shape.
  assumed      the inner end's outline: two rounded bosses round the arm holes.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import ezdxf
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon as MplPolygon  # noqa: E402
from shapely.geometry import LineString, Point, Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
FRAME = HERE.parent

# ---- measured (mm) --------------------------------------------------------------------
MOTOR_EDGE = 22.5          # tip to the motor hole's near edge
MOTOR_D = 8.0              # motor centre hole
END_NONSPAR = 200.0        # tip to the end of the straight edge on the side without spar holes
END_SPAR = 198.0           # ... and on the spar-hole side
SPAR_EDGE = 65.0           # tip to the spar holes' (bump's) near edge
ARMHOLE_EDGE_NONSPAR = 204.0   # tip to the near edge of the arm hole on the side without spar holes
ARMHOLE_EDGE_SPAR = 201.0      # ... and on the spar-hole side
SLOT_LEN = 5.0             # the motor bolt slots, end to end
# ---- borrowed from the stock arm ------------------------------------------------------
ARMHOLE_D = 3.2
SPARHOLE_D = 4.0
SLOT_W = 3.0               # slot width (M3 clearance), as on the stock arm


def load_stock():
    """The stock arm in arm-local coordinates (u from the tip, v across; v < 0 on the spar-hole side)."""
    rows = {}
    with open(FRAME / "holes.csv") as f:
        for r in csv.DictReader(f):
            if r["part"] == "arm":
                rows[r["label"]] = r
    pt = lambda k: np.array([float(rows[k]["x_mm"]), float(rows[k]["y_mm"])])  # noqa: E731
    h1, s3, h4 = pt("H1"), pt("S3"), pt("H4")
    ax = (h1 - (s3 + h4) / 2)
    ax /= np.linalg.norm(ax)                                   # towards the tip
    normal = np.array([-ax[1], ax[0]])

    doc = ezdxf.readfile(FRAME / "dxf" / "arm.dxf")
    segs, circles = [], []
    for e in doc.modelspace():
        t = e.dxftype()
        if t == "LINE":
            segs.append([tuple(e.dxf.start)[:2], tuple(e.dxf.end)[:2]])
        elif t == "ARC":
            c, r = e.dxf.center, e.dxf.radius
            a0, a1 = math.radians(e.dxf.start_angle), math.radians(e.dxf.end_angle)
            if a1 < a0:
                a1 += 2 * math.pi
            n = max(4, int((a1 - a0) / 0.03))
            segs.append([(c.x + r * math.cos(a0 + (a1 - a0) * i / n), c.y + r * math.sin(a0 + (a1 - a0) * i / n))
                         for i in range(n + 1)])
        elif t == "CIRCLE":
            circles.append((e.dxf.center.x, e.dxf.center.y, e.dxf.radius))
    tip = max(float((np.array(p) - h1) @ ax) for s in segs for p in s)

    def loc(p):
        d = np.array(p, float) - h1
        return (tip - float(d @ ax), float(d @ normal))

    loops = chain([[loc(p) for p in s] for s in segs])
    polys = sorted((Polygon(lp) for lp in loops if len(lp) > 3), key=lambda g: -g.area)
    outer, inner = polys[0], polys[1:]
    slots = []
    for k in ("L1", "L2", "L3", "L4"):
        # slot rows are in holes.csv with end-arc centres (x, y) - (x2, y2) and a width
        pass
    with open(FRAME / "holes.csv") as f:
        for r in csv.DictReader(f):
            if r["part"] == "arm" and r["kind"] == "slot":
                a, b = loc((float(r["x_mm"]), float(r["y_mm"]))), loc((float(r["x2_mm"]), float(r["y2_mm"])))
                slots.append((a, b, float(r["dia_or_width_mm"])))
    return {"outer": outer, "inner": inner, "slots": slots, "loc": loc, "h1": loc(h1),
            "spar_holes": [loc(pt("H2")), loc(pt("H3"))], "arm_holes": [loc(s3), loc(h4)]}


def chain(segs, tol=1e-3):
    """Join line and arc pieces into closed loops by their end points."""
    d = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])  # noqa: E731
    remaining, loops = [list(s) for s in segs], []
    while remaining:
        pts = remaining.pop(0)
        grew = True
        while grew:
            grew = False
            for i, s in enumerate(remaining):
                if d(s[0], pts[-1]) < tol:
                    pts += s[1:]
                elif d(s[-1], pts[-1]) < tol:
                    pts += s[::-1][1:]
                elif d(s[-1], pts[0]) < tol:
                    pts = s[:-1] + pts
                elif d(s[0], pts[0]) < tol:
                    pts = s[::-1][:-1] + pts
                else:
                    continue
                remaining.pop(i)
                grew = True
                break
        loops.append(pts)
    return loops


def build(stock):
    """The modified arm in arm-local coordinates with v < 0 on the spar-hole side."""
    outer = stock["outer"]
    sh = stock["spar_holes"]
    v_top = 4.98                                      # the stock arm's straight edge on the non-spar side
    arm_holes_nonspar = (ARMHOLE_EDGE_NONSPAR + ARMHOLE_D / 2, +5.57)
    arm_holes_spar = (ARMHOLE_EDGE_SPAR + ARMHOLE_D / 2, -5.57)

    tip = outer.intersection(box(-1, -40, 45.75, 40))            # stock tip and motor mount, as is

    # bump with the spar holes: the stock one, moved so its near edge is SPAR_EDGE from the tip
    stock_bump_start = 71.35
    bump = outer.intersection(box(stock_bump_start - 0.01, -20, 101.0, -5.2))
    bump = unary_union([bump]).buffer(0)
    from shapely.affinity import translate
    shift = SPAR_EDGE - stock_bump_start
    bump = translate(bump, xoff=shift)

    v_bot_end = -6.70
    strip = Polygon([(45.0, v_top), (END_NONSPAR, v_top), (END_SPAR, v_bot_end),
                     (94.09 + 0.0, -6.24), (SPAR_EDGE, -5.40), (45.0, -5.10)])

    # inner end: bosses round the arm holes, radii taken so the straight edges end where measured
    r_top = math.hypot(arm_holes_nonspar[0] - END_NONSPAR, arm_holes_nonspar[1] - v_top)
    r_bot = math.hypot(arm_holes_spar[0] - END_SPAR, arm_holes_spar[1] - v_bot_end)
    boss_top = Point(*arm_holes_nonspar).buffer(r_top, 64)
    boss_bot = Point(*arm_holes_spar).buffer(r_bot, 64)
    foot = unary_union([boss_top, boss_bot, Polygon([(END_NONSPAR, v_top), (END_SPAR, v_bot_end),
                                                    arm_holes_spar, arm_holes_nonspar])])
    foot = foot.convex_hull
    body = unary_union([tip, strip, bump, foot]).buffer(0.25, 32).buffer(-0.25, 32)
    body = body.buffer(0)
    # the stock lightening cut-out(s) and the motor hole/slots stay
    cut = unary_union([g for g in stock["inner"]])
    body = body.difference(cut)

    motor = (MOTOR_EDGE + MOTOR_D / 2, 0.0)
    dx = motor[0] - stock["h1"][0]                    # the stock slots stay concentric with the moved hole
    slots = []
    for a, b, _ in stock["slots"]:
        mid = np.array([(a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2])
        d = np.array([b[0] - a[0], b[1] - a[1]])
        d /= np.linalg.norm(d)                        # radial: the 16 and 19 mm bolt circles run along it
        half = (SLOT_LEN - SLOT_W) / 2                # end-arc centres; the slot is SLOT_LEN long over its ends
        slots.append((tuple(mid - d * half), tuple(mid + d * half), SLOT_W))
    holes = [("motor", motor, MOTOR_D),
             ("spar", (sh[0][0] + shift, sh[0][1]), SPARHOLE_D), ("spar", (sh[1][0] + shift, sh[1][1]), SPARHOLE_D),
             ("arm", arm_holes_nonspar, ARMHOLE_D), ("arm", arm_holes_spar, ARMHOLE_D)]
    return {"body": body, "holes": holes, "slots": slots, "bosses": (r_top, r_bot)}


# ---------------------------------------------------------------------------------------
# Outputs. The spar-hole side is drawn at +v (the stock arm, turned, not mirrored).

def fv(p):
    return (p[0], -p[1])


def poly_rings(g):
    geoms = list(g.geoms) if hasattr(g, "geoms") else [g]
    for gg in geoms:
        yield [fv(p) for p in gg.exterior.coords], [[fv(p) for p in r.coords] for r in gg.interiors]


def write_dxf(m, path):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    for name, colour in (("PROFILE", 7), ("HOLES", 1), ("SLOTS", 5), ("CENTRE", 8)):
        doc.layers.add(name, color=colour)
    msp = doc.modelspace()
    for ext, ints in poly_rings(m["body"]):
        msp.add_lwpolyline(ext, close=True, dxfattribs={"layer": "PROFILE"})
        for r in ints:
            msp.add_lwpolyline(r, close=True, dxfattribs={"layer": "PROFILE"})
    for _, (u, v), d in m["holes"]:
        msp.add_circle((u, -v), d / 2, dxfattribs={"layer": "HOLES"})
        msp.add_line((u - d, -v), (u + d, -v), dxfattribs={"layer": "CENTRE"})
        msp.add_line((u, -v - d), (u, -v + d), dxfattribs={"layer": "CENTRE"})
    for a, b, w in m["slots"]:
        s = LineString([a, b]).buffer(w / 2, 24)
        msp.add_lwpolyline([fv(p) for p in s.exterior.coords], close=True, dxfattribs={"layer": "SLOTS"})
    doc.saveas(path)


INK, DIM, GREY = "#1B2226", "#B4470C", "#8C979C"


def dim_h(ax, u0, u1, y, label, y_from0, y_from1):
    """Horizontal dimension from u0 to u1 on row y, with extension lines from the features."""
    for u, yf in ((u0, y_from0), (u1, y_from1)):
        ax.plot([u, u], [yf, y + math.copysign(2.0, y)], color=DIM, lw=0.6, zorder=3)
    ax.annotate("", xy=(u0, y), xytext=(u1, y), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9,
                                                              shrinkA=0, shrinkB=0), zorder=4)
    ax.text((u0 + u1) / 2, y + math.copysign(1.3, y), label, color=DIM, ha="center",
            va="bottom" if y > 0 else "top", fontsize=10, fontweight="bold", zorder=5)


def dim_slot(ax, m, text):
    """Length of the slot nearest the upper right of the motor hole, end to end, on its own axis."""
    cands = [(a, b, w) for a, b, w in m["slots"] if (a[0] + b[0]) / 2 > MOTOR_EDGE + MOTOR_D / 2 and (a[1] + b[1]) / 2 < 0]
    a, b, w = cands[0]                                  # arm-local v < 0 is the spar-hole side, drawn at +v
    a, b = np.array(fv(a)), np.array(fv(b))
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]])
    if n @ ((a + b) / 2 - np.array([MOTOR_EDGE + MOTOR_D / 2, 0.0])) < 0:
        n = -n                                          # the dimension sits outside the slot, away from the hole
    p0, p1 = a - d * w / 2, b + d * w / 2               # slot ends
    off = n * 3.4
    for p in (p0, p1):
        ax.plot([p[0], p[0] + off[0] * 1.15], [p[1], p[1] + off[1] * 1.15], color=DIM, lw=0.6, zorder=5)
    ax.annotate("", xy=tuple(p0 + off), xytext=tuple(p1 + off),
                arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9, shrinkA=0, shrinkB=0), zorder=6)
    mid = (p0 + p1) / 2 + off + n * 1.9
    ax.text(mid[0], mid[1], text, color=DIM, fontsize=10, fontweight="bold", ha="center", va="center", zorder=7)


def draw_part(ax, m, stock_outline=None, lw=1.4):
    if stock_outline is not None:
        for ext, _ in poly_rings(stock_outline):
            ax.plot(*zip(*ext), color=GREY, lw=0.8, ls=(0, (4, 3)), zorder=1)
    for ext, ints in poly_rings(m["body"]):
        ax.add_patch(MplPolygon(ext, closed=True, fc="#E6ECEE", ec=INK, lw=lw, zorder=2))
        for r in ints:
            ax.add_patch(MplPolygon(r, closed=True, fc="white", ec=INK, lw=lw, zorder=3))
    for a, b, w in m["slots"]:
        s = LineString([a, b]).buffer(w / 2, 24)
        ax.add_patch(MplPolygon([fv(p) for p in s.exterior.coords], closed=True, fc="white", ec=INK, lw=lw * 0.8, zorder=3))
    for kind, (u, v), d in m["holes"]:
        ax.add_patch(Circle((u, -v), d / 2, fc="white", ec=INK, lw=lw, zorder=4))
        ax.plot([u - d, u + d], [-v, -v], color=GREY, lw=0.5, zorder=4)
        ax.plot([u, u], [-v - d, -v + d], color=GREY, lw=0.5, zorder=4)


def draw(m, stock, path_png, path_svg):
    fig = plt.figure(figsize=(13.2, 10.4), dpi=170)
    fig.patch.set_facecolor("white")
    top = fig.add_axes([0.02, 0.40, 0.96, 0.52])
    body = m["body"]
    u_max = body.bounds[2]
    draw_part(top, m, stock_outline=stock["outer"])
    top.set_xlim(-24, 232)
    top.set_ylim(-42, 44)
    top.set_aspect("equal")
    top.axis("off")

    holes = {(k, round(u, 1)): (u, v, d) for k, (u, v), d in m["holes"]}
    motor_u = MOTOR_EDGE + MOTOR_D / 2
    spar_near = SPAR_EDGE
    # dimension rows: spar-hole side (+v) above, the other side below
    y_top = [21, 29, 37]
    y_bot = [-21, -29, -37]
    dim_h(top, 0, MOTOR_EDGE, y_bot[0], "22.5", 0, 0)
    dim_h(top, 0, END_NONSPAR, y_bot[1], "200", -4.98, -4.98)
    dim_h(top, 0, ARMHOLE_EDGE_NONSPAR, y_bot[2], "204", -5.57, -5.57)
    dim_h(top, 0, SPAR_EDGE, y_top[0], "65", 5.45, 5.45)
    dim_h(top, 0, END_SPAR, y_top[1], "198", 6.7, 6.7)
    dim_h(top, 0, ARMHOLE_EDGE_SPAR, y_top[2], "201", 5.57, 5.57)
    top.text(-3, 29, "spar-hole\nside", color=DIM, fontsize=9, ha="right", va="center", style="italic")
    top.text(-3, -29, "other\nside", color=DIM, fontsize=9, ha="right", va="center", style="italic")
    top.text(150, -12.5, f"dashed: stock Mark4 V2 10\" arm ({stock['outer'].bounds[2]:.0f} long)\n"
             f"inner end of this arm: {u_max:.1f} (derived)", fontsize=8, color=GREY, ha="center", va="top")
    fig.text(0.02, 0.975, "Modified Mark4 arm: measured dimensions, mm", fontsize=14, fontweight="bold",
             color=INK, va="top")
    fig.text(0.02, 0.948, "Distances run along the arm from its tip. Spar-hole side up; the stock front-right arm, "
             "viewed from the top face, not mirrored.", fontsize=9, color=GREY, va="top")

    # details
    tip = fig.add_axes([0.03, 0.045, 0.45, 0.33])
    draw_part(tip, m, lw=1.6)
    tip.set_xlim(-3, 70)
    tip.set_ylim(-17, 16)
    tip.set_aspect("equal")
    tip.axis("off")
    mu = motor_u
    tip.annotate("Ø8 motor hole", xy=(mu, 4.0), xytext=(mu, 12.8), fontsize=9, color=INK, ha="center",
                 arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))
    tip.text(69.5, -8.5, "4 bolt slots, 5 long end to end\nand 3 wide, on the 16 and 19 mm\nbolt circles (width from stock)", fontsize=8,
             color=GREY, ha="right", va="top")
    tip.text(-3, 15.5, "Tip", fontsize=10, fontweight="bold", color=INK, va="top")
    dim_h(tip, 0, MOTOR_EDGE, -12.5, "22.5", 0.0, 0.0)
    dim_slot(tip, m, "5")

    end = fig.add_axes([0.52, 0.045, 0.45, 0.33])
    draw_part(end, m, lw=1.6)
    end.set_xlim(184, 217)
    end.set_ylim(-17, 16)
    end.set_aspect("equal")
    end.axis("off")
    end.text(184.3, 15.5, "Inner end (outline assumed)", fontsize=10, fontweight="bold", color=INK, va="top")
    for kind, (u, v), d in m["holes"]:
        if kind == "arm":
            side = 1 if -v > 0 else -1
            end.annotate(f"Ø{d:g}", xy=(u + d / 2 * 0.7, -v + side * d / 2 * 0.7), xytext=(u + 6.5, -v + side * 6.5),
                         fontsize=9, color=INK, arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))
    end.text(184.3, -16.2, "straight edges end at 200 and 198; the holes' near edges are at\n"
                           "204 and 201; the holes are 11.14 mm apart across the arm (stock)", fontsize=8, color=GREY,
             va="bottom")
    fig.savefig(path_png)
    fig.savefig(path_svg)
    plt.close(fig)


def verify(m):
    """Read the dimensions back off the finished outline (arm-local, v < 0 on the spar-hole side)."""
    g = m["body"]

    def edge(u):
        c = g.intersection(LineString([(u, -40), (u, 40)])).bounds
        return c[3], c[1]                                       # top (non-spar) and bottom (spar-hole) edge at u

    us = np.arange(50.0, 215.0, 0.05)
    top = np.array([edge(u)[0] for u in us])
    bot = np.array([edge(u)[1] for u in us])
    end_top = us[np.argmax((top > top[40] + 0.08) & (us > 120))]    # first place the edge leaves its straight line
    line = -6.24 + (-6.70 + 6.24) * (us - 94.09) / (198.0 - 94.09)     # the spar-hole side's gentle taper
    end_bot = us[np.argmax((bot < line - 0.08) & (us > 150))]
    bump = us[np.argmax(bot < bot[0] - 0.3)]
    print(f"straight edge ends: non-spar side {end_top:.2f} (asked 200), spar-hole side {end_bot:.2f} (asked 198); "
          f"bump starts {bump:.2f} (asked 65)")
    for a, b, w in m["slots"]:
        print(f"  slot end to end {math.dist(a, b) + w:.2f}, radii of its end-arc centres "
              f"{math.dist(a, (26.5, 0)):.2f} / {math.dist(b, (26.5, 0)):.2f}")
    for kind, (u, v), d in m["holes"]:
        print(f"  {kind:5s} hole: centre u {u:7.2f} v {-v:6.2f}  d {d:g}  near edge {u - d / 2:7.2f}")


def main():
    stock = load_stock()
    m = build(stock)
    write_dxf(m, HERE / "modified_arm.dxf")
    draw(m, stock, HERE / "modified_arm.png", HERE / "modified_arm.svg")
    b = m["body"].bounds
    print(f"outline u {b[0]:.2f}..{b[2]:.2f}, v {-b[3]:.2f}..{-b[1]:.2f}; boss radii {m['bosses'][0]:.2f} / {m['bosses'][1]:.2f}")
    verify(m)
    print("wrote", HERE / "modified_arm.dxf")


if __name__ == "__main__":
    main()
