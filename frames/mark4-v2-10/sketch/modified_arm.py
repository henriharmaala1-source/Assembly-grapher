#!/usr/bin/env python3
"""Dimensioned sketch of a modified Mark4 10" arm, from measured distances.

    python3 frames/mark4-v2-10/sketch/modified_arm.py      # needs ezdxf, matplotlib, shapely

Writes modified_arm.dxf (mm, layers PROFILE / HOLES / SLOTS / CENTRE), modified_arm.png and
modified_arm.svg next to this file.

The arm is the stock Mark4 V2 10" arm (frames/mark4-v2-10/dxf/arm.dxf), kept as it is at the
motor end (tip, motor mount, cut-out), at the bump with the two small holes, and at the inner end
(the stock outline with its notch). What changes:

  * the straight section is shortened, so everything past it moves toward the tip: the shift is
    set by putting the notch tip NOTCH_PAST mm beyond the end of the bump-side straight edge;
  * the motor hole is 8 mm and the four motor bolt slots are 5 mm end to end (3 mm wide);
  * the arm is 11.0 mm wide at the motor end and 11.5 mm at the frame end;
  * the notch's tip edge is 7.5 mm wide (stock 7.0);
  * the two end holes sit where they were measured, not where the stock arm has them.

Coordinates: u runs along the arm from the tip (u = 0) toward the frame, v across it. Drawn with
the bump side up and the notch side down (the stock front-right arm, turned, not mirrored).
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
from shapely.affinity import translate  # noqa: E402
from shapely.geometry import LineString, Polygon, box  # noqa: E402
from shapely.ops import unary_union  # noqa: E402

HERE = Path(__file__).resolve().parent
FRAME = HERE.parent

# ---- measured (mm) --------------------------------------------------------------------
MOTOR_EDGE = 22.5          # tip to the motor hole's near edge
MOTOR_D = 8.0              # motor centre hole
SLOT_LEN, SLOT_W = 5.0, 3.0    # motor bolt slots, end to end, and their width (stock)
WIDTH_MOTOR_END = 11.0     # arm width just behind the motor mount
WIDTH_FRAME_END = 11.5     # ... and at the frame end
END_BUMP_SIDE = 200.0      # tip to the end of the straight edge on the side with the bump
END_NOTCH_SIDE = 198.0     # ... and on the side with the notch
NOTCH_PAST = 18.0          # notch tip beyond the bump-side straight edge's end
TAB_W = 7.5                # width of the notch's flat tip (stock 7.0)
BUMP_NEAR = 65.0           # tip to the bump's near sloped edge (mid-height)
HOLE_S3_EDGE = 204.0       # tip to the near edge of the end hole beside the notch
HOLE_H4_EDGE = 201.0       # ... and of the end hole on the bump side
END_HOLE_D = 3.2           # from the stock arm
BUMP_HOLE_D = 4.0          # from the stock arm
V_NOTCH_EDGE = 4.97        # the stock arm's straight edge on the notch side
STOCK_END_BUMP = 211.02    # where the stock arm's straight edges end, read off its outline:
STOCK_END_NOTCH = 207.93   # bump side and notch side


def load_stock():
    """The stock arm in arm-local coordinates (u from the tip, v across; v > 0 on the notch side)."""
    rows = {}
    with open(FRAME / "holes.csv") as f:
        for r in csv.DictReader(f):
            if r["part"] == "arm":
                rows.setdefault(r["label"], r)
    pt = lambda k: np.array([float(rows[k]["x_mm"]), float(rows[k]["y_mm"])])  # noqa: E731
    h1, s3, h4 = pt("H1"), pt("S3"), pt("H4")
    ax = h1 - (s3 + h4) / 2
    ax /= np.linalg.norm(ax)                                   # towards the tip
    normal = np.array([-ax[1], ax[0]])

    doc = ezdxf.readfile(FRAME / "dxf" / "arm.dxf")
    segs = []
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
    tip = max(float((np.array(p) - h1) @ ax) for s in segs for p in s)

    def loc(p):
        d = np.array(p, float) - h1
        return (tip - float(d @ ax), float(d @ normal))

    loops = chain([[loc(p) for p in s] for s in segs])
    polys = sorted((Polygon(lp) for lp in loops if len(lp) > 3), key=lambda g: -g.area)
    slots = []
    with open(FRAME / "holes.csv") as f:
        for r in csv.DictReader(f):
            if r["part"] == "arm" and r["kind"] == "slot":
                a, b = loc((float(r["x_mm"]), float(r["y_mm"]))), loc((float(r["x2_mm"]), float(r["y2_mm"])))
                slots.append((a, b, float(r["dia_or_width_mm"])))
    return {"outer": polys[0], "inner": polys[1:], "slots": slots, "h1": loc(h1),
            "bump_holes": [loc(pt("H2")), loc(pt("H3"))], "end_holes": [loc(s3), loc(h4)]}


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
    """The modified arm in arm-local coordinates (v > 0 on the notch side)."""
    outer = stock["outer"]
    # the notch's tip edge: the stock one runs from A to B (7.0 wide); stretch it to TAB_W about B
    A, B = (225.13, 15.65), (229.06, 9.86)
    k = TAB_W / math.dist(A, B)
    A2 = (B[0] + (A[0] - B[0]) * k, B[1] + (A[1] - B[1]) * k)
    ring = list(outer.exterior.coords)
    i = min(range(len(ring)), key=lambda j: math.dist(ring[j], A))
    assert math.dist(ring[i], A) < 0.05, "stock notch corner not found"
    ring[i] = A2
    if i == 0:
        ring[-1] = A2
    outer = Polygon(ring)
    for u0, v0 in ((STOCK_END_BUMP, -6.71), (STOCK_END_NOTCH, 4.95)):
        assert min(math.dist(p, (u0, v0)) for p in ring) < 0.05, "stock straight edge end not found"
    tip_u = max(p[0] for p in ring)                              # the notch tip, stock: 229.06
    shift = (END_BUMP_SIDE + NOTCH_PAST) - tip_u                 # how far the stock arm's rear part moves toward the tip

    tip = outer.intersection(box(-1, -40, 45.75, 40))            # motor end, as on the stock arm
    # straight section: the notch-side edge straight, the bump-side edge tapering so the width goes from
    # WIDTH_MOTOR_END at u = 50 to WIDTH_FRAME_END at u = 190, then eases into the stock inner end (11.67 wide)
    foot_u = 205.0 + shift                                       # where the stock inner end starts, after the shift
    v_foot = -6.70                                               # the stock inner end's bump-side edge

    def vb(u):
        v_motor, v190 = -(WIDTH_MOTOR_END - V_NOTCH_EDGE), -(WIDTH_FRAME_END - V_NOTCH_EDGE)
        if u < 50.0:                                             # from the stock edge at the end of the motor mount
            return -5.10 + (v_motor + 5.10) * (u - 45.75) / (50.0 - 45.75)
        if u <= 190.0:
            return v_motor + (v190 - v_motor) * (u - 50.0) / (190.0 - 50.0)
        return min(v_foot, v190 + (v_foot - v190) * (u - 190.0) / (foot_u - 190.0)) if u < foot_u else v_foot

    pts = [(45.0, V_NOTCH_EDGE), (foot_u + 1.0, V_NOTCH_EDGE)] + [(u, vb(u)) for u in (foot_u + 1.0, foot_u, 190.0, 50.0, 45.75, 45.0)]
    strip = Polygon(pts)
    bump = translate(outer.intersection(box(71.30, -20, 101.0, -5.2)), xoff=shift)    # stock bump, moved with the rest
    foot = translate(outer.intersection(box(205.0, -40, 240, 40)), xoff=shift)         # the stock inner end, with its notch
    body = unary_union([tip, strip, bump, foot]).buffer(0.3, 32).buffer(-0.3, 32).buffer(0)
    body = body.difference(unary_union(stock["inner"]))

    motor = (MOTOR_EDGE + MOTOR_D / 2, 0.0)
    dx = motor[0] - stock["h1"][0]                                 # slots stay concentric with the moved hole
    slots = []
    for a, b, _ in stock["slots"]:
        mid = np.array([(a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2])
        d = np.array([b[0] - a[0], b[1] - a[1]])
        d /= np.linalg.norm(d)
        half = (SLOT_LEN - SLOT_W) / 2
        slots.append((tuple(mid - d * half), tuple(mid + d * half), SLOT_W))
    s3_old, h4_old = stock["end_holes"]
    holes = [("motor", motor, MOTOR_D),
             ("bump", (stock["bump_holes"][0][0] + shift, stock["bump_holes"][0][1]), BUMP_HOLE_D),
             ("bump", (stock["bump_holes"][1][0] + shift, stock["bump_holes"][1][1]), BUMP_HOLE_D),
             ("end", (HOLE_S3_EDGE + END_HOLE_D / 2, s3_old[1]), END_HOLE_D),      # beside the notch
             ("end", (HOLE_H4_EDGE + END_HOLE_D / 2, h4_old[1]), END_HOLE_D)]      # bump side
    ghosts = [(s3_old[0] + shift, s3_old[1]), (h4_old[0] + shift, h4_old[1])]      # where the stock holes would be
    return {"body": body, "holes": holes, "slots": slots, "ghosts": ghosts, "shift": shift, "vb": vb,
            "notch": (A2, B), "tip_u": tip_u + shift,
            "end_bump": STOCK_END_BUMP + shift, "end_notch": STOCK_END_NOTCH + shift}


# ---------------------------------------------------------------------------------------
# Outputs. The bump side is drawn at +v (v_out = -v).

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


def num(x):
    return f"{x:.0f}" if abs(x - round(x)) < 0.06 else f"{x:.1f}"


def lab(measured, said=None):
    """The drawn value, and what you said if the drawing differs by more than half a millimetre."""
    t = num(measured)
    return t if said is None or abs(measured - said) <= 0.5 else f"{t}  (you said {num(said)})"


def dim_h(ax, u0, u1, y, label, y_from0, y_from1):
    for u, yf in ((u0, y_from0), (u1, y_from1)):
        ax.plot([u, u], [yf, y + math.copysign(2.0, y)], color=DIM, lw=0.6, zorder=3)
    ax.annotate("", xy=(u0, y), xytext=(u1, y), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9,
                                                              shrinkA=0, shrinkB=0), zorder=4)
    ax.text((u0 + u1) / 2, y + math.copysign(1.3, y), label, color=DIM, ha="center",
            va="bottom" if y > 0 else "top", fontsize=10, fontweight="bold", zorder=5)


def dim_v(ax, u, y0, y1, label):
    ax.annotate("", xy=(u, y0), xytext=(u, y1), arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9,
                                                              shrinkA=0, shrinkB=0), zorder=6)
    ax.text(u + 1.6, (y0 + y1) / 2, label, color=DIM, ha="left", va="center", fontsize=9, fontweight="bold",
            zorder=7, bbox=dict(fc="white", ec="none", pad=0.6))


def dim_between(ax, p0, p1, outward, dist, text, gap=2.7):
    """Dimension of the distance p0-p1, drawn `dist` away on the `outward` side."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    n = np.array([-d[1], d[0]])
    if n @ np.array(outward) < 0:
        n = -n
    off = n * dist
    for p in (p0, p1):
        ax.plot([p[0], p[0] + off[0] * 1.15], [p[1], p[1] + off[1] * 1.15], color=DIM, lw=0.6, zorder=5)
    ax.annotate("", xy=tuple(p0 + off), xytext=tuple(p1 + off),
                arrowprops=dict(arrowstyle="<->", color=DIM, lw=0.9, shrinkA=0, shrinkB=0), zorder=6)
    mid = (p0 + p1) / 2 + off + n * gap
    ax.text(mid[0], mid[1], text, color=DIM, fontsize=10, fontweight="bold", ha="center", va="center", zorder=7)


def dim_slot(ax, m, text):
    """Length of the slot nearest the upper right of the motor hole, end to end, on its own axis."""
    cands = [(a, b, w) for a, b, w in m["slots"] if (a[0] + b[0]) / 2 > MOTOR_EDGE + MOTOR_D / 2 and (a[1] + b[1]) / 2 < 0]
    a, b, w = cands[0]
    a, b = np.array(fv(a)), np.array(fv(b))
    d = (b - a) / np.linalg.norm(b - a)
    p0, p1 = a - d * w / 2, b + d * w / 2
    dim_between(ax, p0, p1, outward=(((a + b) / 2) - np.array([MOTOR_EDGE + MOTOR_D / 2, 0.0])), dist=3.4, text=text)


def draw_part(ax, m, stock_outline=None, lw=1.4, ghosts=False):
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
    if ghosts:
        for u, v in m["ghosts"]:
            ax.add_patch(Circle((u, -v), END_HOLE_D / 2, fc="none", ec=GREY, lw=0.9, ls=(0, (2, 2)), zorder=5))


def measure(m):
    """Read the dimensions back off the finished outline."""
    g, vb = m["body"], m["vb"]

    def edge(u):
        c = g.intersection(LineString([(u, -40), (u, 40)])).bounds
        return c[3], c[1]                                       # notch-side (+v) and bump-side (-v) edge at u

    us = np.arange(50.0, 221.0, 0.05)
    e = np.array([edge(u) for u in us])
    taper = np.array([vb(u) for u in us])
    start = us[np.argmax((e[:, 1] < taper - 0.3) & (us > 50))]
    flat = us[np.argmax((e[:, 1] < -12.5) & (us > 50))]
    ta, tb = m["notch"]
    w = lambda u: e[np.argmin(abs(us - u)), 0] - e[np.argmin(abs(us - u)), 1]  # noqa: E731
    return {"end_notch": m["end_notch"], "end_bump": m["end_bump"], "bump_start": start, "bump_flat": flat,
            "bump_mid": (start + flat) / 2, "tip": g.bounds[2], "tab_w": math.dist(ta, tb),
            "width60": w(60.0), "width190": w(190.0)}


def walls(m):
    """Material left between each hole and the outline (mm); the stock arm has 2.8 to 3.4 round its end holes."""
    from shapely.geometry import Point
    out = []
    for kind, (u, v), d in m["holes"]:
        c = Point(u, v)
        w = min([m["body"].exterior.distance(c)] + [r.distance(c) for r in m["body"].interiors]) - d / 2
        out.append((kind, (u, v), d, w))
    return out


def draw(m, stock, ms, path_png, path_svg):
    fig = plt.figure(figsize=(13.2, 10.4), dpi=170)
    fig.patch.set_facecolor("white")
    top = fig.add_axes([0.02, 0.40, 0.96, 0.52])
    draw_part(top, m, stock_outline=stock["outer"])
    top.set_xlim(-24, 232)
    top.set_ylim(-42, 44)
    top.set_aspect("equal")
    top.axis("off")

    ta, tb = m["notch"]
    ta_u, tb_u, tb_v = ta[0] + m["shift"], tb[0] + m["shift"], tb[1]
    h_s3 = next(h for k, h, d in m["holes"] if k == "end" and h[1] > 0)         # beside the notch
    h_h4 = next(h for k, h, d in m["holes"] if k == "end" and h[1] < 0)         # bump side
    y_top, y_bot = [21, 29, 37], [-21, -29, -37]
    vb = m["vb"]
    dim_h(top, 0, MOTOR_EDGE, y_bot[0], "22.5", 0, 0)
    dim_h(top, 0, ms["end_notch"], y_bot[1], lab(ms["end_notch"], END_NOTCH_SIDE), -V_NOTCH_EDGE, -V_NOTCH_EDGE)
    dim_h(top, 0, HOLE_S3_EDGE, y_bot[2], num(HOLE_S3_EDGE), -h_s3[1], -h_s3[1])
    dim_h(top, 0, ms["bump_mid"], y_top[0], lab(ms["bump_mid"], BUMP_NEAR), 9.5, 9.5)
    dim_h(top, 0, ms["end_bump"], y_top[1], lab(ms["end_bump"], END_BUMP_SIDE), -vb(ms["end_bump"]), -vb(ms["end_bump"]))
    dim_h(top, 0, HOLE_H4_EDGE, y_top[2], num(HOLE_H4_EDGE), -h_h4[1], -h_h4[1])
    dim_h(top, ms["end_bump"], ms["tip"], -17.5, lab(ms["tip"] - ms["end_bump"], NOTCH_PAST), -vb(ms["end_bump"]), -tb_v)
    dim_v(top, 54, -V_NOTCH_EDGE, -vb(54), num(ms["width60"]))
    dim_v(top, 176, -V_NOTCH_EDGE, -vb(176), num(ms["width190"]))
    top.text(-3, 29, "bump side", color=DIM, fontsize=9, ha="right", va="center", style="italic")
    top.text(-3, -29, "notch side", color=DIM, fontsize=9, ha="right", va="center", style="italic")
    top.text(150, -12.5, f"dashed: stock Mark4 V2 10\" arm ({stock['outer'].bounds[2]:.0f} long)\n"
             f"this arm: {ms['tip']:.1f} long, the stock arm shortened {-m['shift']:.1f} in the straight section",
             fontsize=8, color=GREY, ha="center", va="top")
    fig.text(0.02, 0.975, "Modified Mark4 arm: measured dimensions, mm", fontsize=14, fontweight="bold", color=INK, va="top")
    fig.text(0.02, 0.948, "Distances run along the arm from its tip. The stock arm's outline is kept at the tip, the bump and the "
             "inner end; bump side up, notch side down.", fontsize=9, color=GREY, va="top")

    # details
    tipax = fig.add_axes([0.03, 0.045, 0.45, 0.33])
    draw_part(tipax, m, lw=1.6)
    tipax.set_xlim(-3, 70)
    tipax.set_ylim(-17, 16)
    tipax.set_aspect("equal")
    tipax.axis("off")
    mu = MOTOR_EDGE + MOTOR_D / 2
    tipax.annotate("Ø8 motor hole", xy=(mu, 4.0), xytext=(mu, 12.8), fontsize=9, color=INK, ha="center",
                   arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))
    tipax.text(69.5, -8.5, "4 bolt slots, 5 long end to end\nand 3 wide, on the 16 and 19 mm\nbolt circles (width from stock)",
               fontsize=8, color=GREY, ha="right", va="top")
    tipax.text(-3, 15.5, "Motor end", fontsize=10, fontweight="bold", color=INK, va="top")
    dim_h(tipax, 0, MOTOR_EDGE, -12.5, "22.5", 0.0, 0.0)
    dim_slot(tipax, m, "5")

    end = fig.add_axes([0.52, 0.045, 0.45, 0.33])
    draw_part(end, m, lw=1.6, ghosts=True)
    end.set_xlim(184, 224)
    end.set_ylim(-32, 19)
    end.set_aspect("equal")
    end.axis("off")
    end.text(184.3, 18.5, "Inner end: the stock outline, holes where you measured", fontsize=10, fontweight="bold",
             color=INK, va="top")
    end.annotate("Ø3.2", xy=(h_h4[0] + 1.1, -h_h4[1] + 1.1), xytext=(h_h4[0] + 12.5, -h_h4[1] + 4.0), fontsize=9, color=INK,
                 arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))
    end.annotate("Ø3.2", xy=(h_s3[0] - 1.1, -h_s3[1] - 1.1), xytext=(h_s3[0] - 11, -h_s3[1] - 9.5), fontsize=9, color=INK,
                 arrowprops=dict(arrowstyle="-", color=INK, lw=0.7))
    dim_h(end, ms["end_bump"], ms["tip"], -19.5, lab(ms["tip"] - ms["end_bump"], NOTCH_PAST), -vb(ms["end_bump"]), -tb_v)
    dim_between(end, (ta_u, -ta[1]), (tb_u, -tb_v), outward=(1, 0), dist=3.0, text="7.5", gap=2.7)
    thin = [(h, w) for k, h, d, w in walls(m) if k == "end" and w < 1.0]
    if thin:                                                    # a hole that leaves almost no material on this outline
        (u, v), w = thin[0]
        end.annotate(f"only {w:.2f} mm of material\nbetween this hole and the edge", xy=(u - 0.9, -v + 1.4),
                     xytext=(u - 17.5, -v + 6.0), fontsize=8.5, color=DIM, ha="left",
                     arrowprops=dict(arrowstyle="-", color=DIM, lw=0.7), zorder=9)
    end.text(184.3, -31.8, "dashed circles: where the stock arm's holes would be after the shortening.\n"
                           "Your hole beside the notch is 0.8 mm from there; the one on the bump side\n"
                           "is 3.4 mm from it (the stock outline leaves 2.8 mm of material round that hole).",
             fontsize=8, color=GREY, va="bottom")
    fig.savefig(path_png)
    fig.savefig(path_svg)
    plt.close(fig)


def main():
    stock = load_stock()
    m = build(stock)
    ms = measure(m)
    write_dxf(m, HERE / "modified_arm.dxf")
    draw(m, stock, ms, HERE / "modified_arm.png", HERE / "modified_arm.svg")
    print(f"shift {m['shift']:.2f} (stock arm shortened); arm {ms['tip']:.2f} long")
    print(f"straight edges end: bump side {ms['end_bump']:.2f} (you said {END_BUMP_SIDE:g}), "
          f"notch side {ms['end_notch']:.2f} (you said {END_NOTCH_SIDE:g})")
    print(f"notch tip {ms['tip']:.2f} = {ms['tip'] - ms['end_bump']:.2f} past the bump-side end (you said {NOTCH_PAST:g}); "
          f"tip edge {ms['tab_w']:.2f} wide (you said {TAB_W:g})")
    print(f"bump: near sloped edge from {ms['bump_start']:.1f} to {ms['bump_flat']:.1f}, mid {ms['bump_mid']:.1f} "
          f"(you said {BUMP_NEAR:g})")
    print(f"width: {ms['width60']:.2f} at u 60 (you said {WIDTH_MOTOR_END:g}), {ms['width190']:.2f} at u 190 "
          f"(you said {WIDTH_FRAME_END:g})")
    for kind, (u, v), d in m["holes"]:
        print(f"  {kind:5s} hole: centre u {u:7.2f} v {-v:6.2f}  d {d:g}  near edge {u - d / 2:7.2f}")
    for k, (u, v), d, w in walls(m):
        print(f"  {k:5s} hole at u {u:6.2f}: {w:.2f} mm of material to the outline")
    for a, b, w in m["slots"]:
        assert abs(math.dist(a, b) + w - SLOT_LEN) < 1e-6
    print("wrote", HERE / "modified_arm.dxf")


if __name__ == "__main__":
    main()
