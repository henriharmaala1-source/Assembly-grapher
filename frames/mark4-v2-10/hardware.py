#!/usr/bin/env python3
"""Printable stand-ins for the Mark4 V2 10" frame's standoffs, screws and nuts.

    python3 frames/mark4-v2-10/hardware.py [--pin 2.7] [--bore 2.7]

No threads. A plain pin with a screw's head shape replaces each screw and
presses into a plain bore: the standoffs are tubes, the press nuts are flanged
rings. The pin is printed lying flat across the bed, the tubes and rings
standing, so the bores print slightly undersize and the fit comes out tight.
Print fit_gauge first and set --bore to the hole that grips the pin as you want.
For test assembly only: a 2.7 mm plastic pin is no substitute for an M3 screw in flight.

Writes, next to this file, in hardware/:
    <part>.stl, <part>.step   one of each part, in its print orientation
    hardware.pdf, hardware.png  a dimensioned sheet: every part, quantity and where it goes
    assembly.step              the frame with every stand-in in place, to check fit and lengths
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
from cadquery import Vector as V

import build as b

OUT = b.HERE / "hardware"


@dataclass(frozen=True)
class Fit:
    pin: float = 2.7      # pin shank diameter: clears the plates' 3.0-3.2 mm holes
    bore: float = 2.7     # standoff and nut bore; prints a little undersize standing up, so it grips


# --------------------------------------------------------------------------
# Parts, modelled along +z


def standoff(f: Fit, d=6.0, length=35.0):
    """The M3 x 35 Ø6 standoff as a plain tube; a pin presses into each end."""
    return cq.Workplane().circle(d / 2).circle(f.bore / 2).extrude(length).edges().chamfer(0.3).val()


def cap_pin(f: Fit, length: float, head_d=5.5, head_h=3.0):
    """Socket-head screw stand-in: an ISO 4762 M3 head on a plain shank of the screw's length.
    Head from z = -head_h to 0, shank from 0 to `length` (length is under the head, as for the screw)."""
    head = cq.Workplane().workplane(offset=-head_h).circle(head_d / 2).extrude(head_h).edges("<Z").fillet(0.4)
    head = head.faces("<Z").workplane().polygon(6, 2.5 / math.cos(math.pi / 6)).cutBlind(-1.3)
    shank = cq.Workplane().circle(f.pin / 2).extrude(length).faces(">Z").edges().chamfer(0.3)
    return head.union(shank).val()


def csk_pin(f: Fit, length: float, head_d=6.0):
    """Countersunk screw stand-in (ISO 10642 M3, 90 degree head). The head's top face is at z = 0,
    the tip at z = length (length is overall, as for a countersunk screw)."""
    head_h = (head_d - f.pin) / 2                                   # 90 degree cone
    head = cq.Solid.makeCone(head_d / 2, f.pin / 2, head_h)
    head = cq.Workplane().add(head).faces("<Z").workplane().polygon(6, 2.0 / math.cos(math.pi / 6)).cutBlind(-1.0)
    shank = cq.Workplane().circle(f.pin / 2).extrude(length).faces(">Z").edges().chamfer(0.3)
    return head.union(shank).val()


def nut(f: Fit, body_d=5.9, body_h=3.0, flange_d=8.0, flange_h=0.8):
    """Press-nut stand-in: a body that fits the frame's Ø6 press-nut holes, with a flange.
    Flange from z = 0 to flange_h, body from -body_h to 0."""
    body = cq.Workplane().workplane(offset=-body_h).circle(body_d / 2).extrude(body_h)
    flange = cq.Workplane().circle(flange_d / 2).extrude(flange_h)
    return body.union(flange).faces(">Z").workplane().hole(f.bore).edges("<Z").chamfer(0.3).val()


def fit_gauge(f: Fit):
    """A tab with standing bores from 2.4 to 3.0 mm and one notch per 0.1 mm above 2.4, plus a test pin."""
    sizes = [2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0]
    w = 9.0 * len(sizes) + 3
    tab = cq.Workplane().box(w, 14, 8, centered=(False, True, False))
    for i, s in enumerate(sizes):
        x = 6 + 9 * i
        tab = tab.cut(cq.Workplane().center(x, 1.5).circle(s / 2).extrude(8))
        for k in range(i):
            tab = tab.cut(cq.Workplane().box(0.8, 2.0, 1.2).translate((x - 3 + 1.4 * k, -6.0, 7.6)))
    return tab.val()


# --------------------------------------------------------------------------
# Where everything goes


def placements(shapes, f: Fit, countersunk=False):
    """(part key, location (x, y, z), flip) for every stand-in in the assembly.
    flip=True turns the part over so its head is at the bottom, pointing up."""
    P = b.BY_KEY
    low, br = P["lower_plate"], P["arm_brace"]
    S = {h.label: h for h in low.holes if h.label.startswith("S")}
    out = []
    z_top, z_low = P["top_plate"].z[1], P["lower_plate"].z[1]
    z_belly, z_chin, z_arm_top, z_arm_bot = P["belly_plate"].z[0], P["chin_plate"].z[0], P["arm"].z[1], P["arm"].z[0]
    for k, h in S.items():
        out.append(("standoff", (h.x, h.y, z_low), False))
        out.append(("pin_cap_m3x6", (h.x, h.y, z_top), True))           # head on the top plate, pointing down
    # chin (S1, S2) and tail (S7, S8): up through the 2.25 plate and the lower plate into the standoff.
    # The STEP countersinks these holes from below; plain holes take cap-head pins, their heads under the plate.
    for keys, L in ((("S1", "S2"), 10), (("S7", "S8"), 8)):
        for k in keys:
            if countersunk:
                out.append((f"pin_csk_m3x{L}", (S[k].x, S[k].y, z_chin), False))
            else:
                out.append((f"pin_cap_m3x{L}", (S[k].x, S[k].y, z_chin), False))
    for k in ("S3", "S4", "S5", "S6"):
        out.append(("pin_cap_m3x18", (S[k].x, S[k].y, z_belly), False))  # up through belly, arm, lower plate
    # arm bolts: up through the belly and arm into a press nut in the lower plate's Ø6 hole
    for h in low.holes:
        if abs(abs(h.x) - 18.57) < 0.05:
            out.append(("pin_cap_m3x12", (h.x, h.y, z_belly), False))
            out.append(("nut", (h.x, h.y, z_low), False))
    # arm braces: down through brace and arm into a nut under the arm (the RJX manual's M3 x 12 and press nuts)
    for h in br.holes:
        for sx in (1, -1):
            x = h.x if sx > 0 else -h.x - 0.035                         # the left brace is mirrored about x = -0.017
            out.append(("pin_cap_m3x12", (x, h.y, P["arm_brace"].z[1]), True))
            out.append(("nut", (x, h.y, z_arm_bot - 0.8), False))       # flange against the arm's underside
    return out


def located(shape, at, flip):
    s = shape.rotate(V(0, 0, 0), V(1, 0, 0), 180) if flip else shape
    return s.translate(V(*at))


PARTS = {   # key: (maker, title, print note)
    "standoff": (lambda f: standoff(f), "Standoff Ø6 x 35", "stand on end"),
    "pin_cap_m3x6": (lambda f: cap_pin(f, 6), "Pin, cap head, 6", "lying flat"),
    "pin_cap_m3x8": (lambda f: cap_pin(f, 8), "Pin, cap head, 8", "lying flat"),
    "pin_cap_m3x10": (lambda f: cap_pin(f, 10), "Pin, cap head, 10", "lying flat"),
    "pin_cap_m3x12": (lambda f: cap_pin(f, 12), "Pin, cap head, 12", "lying flat"),
    "pin_cap_m3x18": (lambda f: cap_pin(f, 18), "Pin, cap head, 18", "lying flat"),
    "pin_csk_m3x8": (lambda f: csk_pin(f, 8), "Pin, countersunk, 8", "lying flat"),
    "pin_csk_m3x10": (lambda f: csk_pin(f, 10), "Pin, countersunk, 10", "lying flat"),
    "nut": (lambda f: nut(f), "Press-nut ring", "flange down"),
}
WHERE = {
    "standoff": "S1-S8, lower plate to top plate",
    "pin_cap_m3x6": "top plate into the standoffs",
    "pin_cap_m3x8": "S7, S8: tail plate, from below",
    "pin_cap_m3x10": "S1, S2: chin plate, from below",
    "pin_cap_m3x12": "arm bolts (4) from below; braces (8) from above",
    "pin_cap_m3x18": "S3-S6 from below: belly, arm, lower plate",
    "pin_csk_m3x8": "S7, S8: tail plate, from below",
    "pin_csk_m3x10": "S1, S2: chin plate, from below",
    "nut": "lower plate's arm-bolt holes (4); under the arms at the braces (8)",
}


def print_pose(key, s):
    """Lay pins on their side, stand tubes up, rings flange-down; sit everything on z = 0."""
    if key.startswith("pin"):
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    elif key == "nut":
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    bb = s.BoundingBox()
    return s.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))


# --------------------------------------------------------------------------
# Drawing


def sheet(out: Path, f: Fit, counts: dict, made: dict):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.backends.backend_pdf import PdfPages

    sh = b.Sheet("Hardware stand-ins (printed)", "2:1",
                 f"Pins Ø{f.pin:g} in Ø{f.bore:g} bores, no threads.  For test assembly only", 1, 1)
    ax = sh.ax
    INK, DIM = b.INK, b.DIM

    def outline(prof, at, sc=2.0, fill="#eceef1"):
        """Draw a turned part's half-profile mirrored: prof = [(r, z), ...] with z up."""
        pts = [(r, z) for r, z in prof] + [(-r, z) for r, z in reversed(prof)]
        q = np.array(pts) * sc + at
        ax.fill(q[:, 0], q[:, 1], color=fill, lw=0)
        ax.plot(np.r_[q[:, 0], q[0, 0]], np.r_[q[:, 1], q[0, 1]], c=INK, lw=0.6)

    def vdim(x, z0, z1, at, sc, text):
        y0, y1 = at[1] + z0 * sc, at[1] + z1 * sc
        ax.annotate("", xy=(at[0] + x, y0), xytext=(at[0] + x, y1),
                    arrowprops=dict(arrowstyle="<|-|>", lw=0.4, color=DIM, mutation_scale=4.5, shrinkA=0, shrinkB=0))
        ax.text(at[0] + x - 1.2, (y0 + y1) / 2, text, fontsize=5.4, color=DIM, rotation=90, ha="center",
                va="bottom", rotation_mode="anchor")

    def hdim(r, z, at, sc, text, dy):
        y = at[1] + z * sc + dy
        ax.annotate("", xy=(at[0] - r * sc, y), xytext=(at[0] + r * sc, y),
                    arrowprops=dict(arrowstyle="<|-|>", lw=0.4, color=DIM, mutation_scale=4.5, shrinkA=0, shrinkB=0))
        ax.text(at[0], y + 0.8, text, fontsize=5.4, color=DIM, ha="center", va="bottom")

    sc = 2.0
    r, br = f.pin / 2, f.bore / 2
    # standoff (drawn at 2:1 would be 70 mm tall; fine)
    at = np.array([30.0, 121.0])
    outline([(3.0, 0), (3.0, 35)], at, sc)
    ax.plot([at[0] - br * sc] * 2, [at[1], at[1] + 35 * sc], c=INK, lw=0.4, ls=(0, (3, 1.5)))
    ax.plot([at[0] + br * sc] * 2, [at[1], at[1] + 35 * sc], c=INK, lw=0.4, ls=(0, (3, 1.5)))
    vdim(-9, 0, 35, at, sc, "35")
    hdim(3.0, 35, at, sc, "Ø6", 2.5)
    hdim(br, 0, at, sc, f"Ø{f.bore:g} through", -6)
    ax.text(at[0], at[1] - 15, "Standoff", fontsize=6.5, weight="bold", ha="center")

    def cap(at, L):
        outline([(2.75, -3), (2.75, 0), (r, 0), (r, L)], at, sc)
        vdim(-8, 0, L, at, sc, f"{L:g}")
        vdim(8.5, -3, 0, at, sc, "3")
        hdim(2.75, -3, at, sc, "Ø5.5", -5)
        hdim(r, L, at, sc, f"Ø{f.pin:g}", 2.5)
        ax.text(at[0], at[1] - 20, f"Pin, cap head, {L:g}", fontsize=6.5, weight="bold", ha="center")

    def csk(at, L):
        hh = (6.0 - f.pin) / 2
        outline([(3.0, 0), (r, hh), (r, L)], at, sc)
        vdim(-8, 0, L, at, sc, f"{L:g} overall")
        hdim(3.0, 0, at, sc, "Ø6, 90°", -5)
        hdim(r, L, at, sc, f"Ø{f.pin:g}", 2.5)
        ax.text(at[0], at[1] - 20, f"Pin, countersunk, {L:g}", fontsize=6.5, weight="bold", ha="center")

    pins = [k for k in PARTS if k.startswith("pin")]
    for i, k in enumerate(pins):
        L = float(k.split("x")[-1])
        at = np.array([68.0 + 38.0 * i, 178.0 - 2 * L])        # shank tips level along the top
        (cap if "cap" in k else csk)(at, L)
    # nut: flange at the top as drawn
    at = np.array([268.0, 158.0])
    outline([(2.95, -3), (2.95, 0), (4.0, 0), (4.0, 0.8)], at, sc)
    ax.plot([at[0] - br * sc] * 2, [at[1] - 6, at[1] + 1.6], c=INK, lw=0.4, ls=(0, (3, 1.5)))
    ax.plot([at[0] + br * sc] * 2, [at[1] - 6, at[1] + 1.6], c=INK, lw=0.4, ls=(0, (3, 1.5)))
    hdim(4.0, 0.8, at, sc, "Ø8 x 0.8", 2.5)
    hdim(2.95, -3, at, sc, "Ø5.9", -5)
    vdim(-11, -3, 0, at, sc, "3")
    ax.text(at[0], at[1] - 20, "Press-nut ring", fontsize=6.5, weight="bold", ha="center")
    ax.text(at[0], at[1] - 24, f"bore Ø{f.bore:g}", fontsize=5.4, color=DIM, ha="center")

    rows = [[str(counts[k]), PARTS[k][1], WHERE[k], PARTS[k][2]] for k in PARTS]
    rows.append(["1", "Fit gauge", "print first: bores 2.4-3.0, notches count 0.1 mm steps", "flat"])
    sh.text(14, 96, "Parts list", fs=7, weight="bold", va="top")
    sh.table(14, 91, ["Qty", "Part", "Goes", "Print"], rows, [9, 36, 112, 20], fs=5.4,
             aligns=["right", "left", "left", "left"])
    notes = [
        f"Fit: pins Ø{f.pin:g} clear the plates' 3.0-3.2 holes and press into Ø{f.bore:g} bores.  Bores printed standing come",
        "out about 0.1-0.15 undersize, so this grips.  Too tight or loose: print the gauge, then rerun hardware.py --bore.",
        "Print pins lying flat (stronger across the layers), in PETG or PA, 100 % infill.  Lengths match the real screws,",
        "measured under the head.  Chin and tail plate holes are Ø2.8 in the model: open them to 3.2 for the pins to pass.",
        "The STEP countersinks those holes; with plain holes, cap-head pins go there (hardware.py --countersunk for the other).",
        "The brace fasteners are as the RJX manual shows them (M3 x 12, press nuts); pins sit loose in the Ø5 and Ø4 holes.",
    ]
    for i, t in enumerate(notes):
        sh.text(14, 50 - i * 4.3, t, fs=5.4, color="#444")
    with PdfPages(out / "hardware.pdf") as pdf:
        pdf.savefig(sh.fig)
    sh.fig.savefig(out / "hardware.png", dpi=160)
    plt.close(sh.fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pin", type=float, default=2.7)
    ap.add_argument("--bore", type=float, default=2.7)
    ap.add_argument("--countersunk", action="store_true",
                    help="countersunk pins at S1, S2, S7, S8, for chin and tail plates countersunk as in the STEP")
    args = ap.parse_args(argv)
    f = Fit(args.pin, args.bore)
    OUT.mkdir(exist_ok=True)
    shapes, so = b.extract(b.STEP)
    made = {k: mk(f) for k, (mk, *_) in PARTS.items()}
    made["fit_gauge"] = fit_gauge(f)
    for k, s in made.items():
        assert s.isValid(), k
        pose = print_pose(k, s)
        cq.exporters.export(pose, str(OUT / f"{k}.stl"), tolerance=0.01, angularTolerance=0.1)
        cq.exporters.export(pose, str(OUT / f"{k}.step"))
    place = placements(shapes, f, args.countersunk)
    counts = {k: sum(1 for p in place if p[0] == k) for k in PARTS}
    for k in [k for k in PARTS if counts[k] == 0]:          # only the pins this build uses
        del PARTS[k]
        for ext in ("stl", "step"):
            (OUT / f"{k}.{ext}").unlink(missing_ok=True)

    # the assembly, and a check that no stand-in cuts into a frame part
    assy = cq.Assembly(name="mark4_v2_10_test_build")
    frame = []
    for n, s in shapes.items():
        if "Standoff" in n:
            continue                      # the model's own standoffs are replaced by the printed ones
        assy.add(s, name=n.replace(" ", "_"), color=cq.Color(0.15, 0.15, 0.16))
        frame.append((n, s))
    worst = []
    for i, (k, at, flip) in enumerate(place):
        s = located(made[k], at, flip)
        assy.add(s, name=f"{k}_{i + 1}", color=cq.Color(0.85, 0.45, 0.1))
        bb = s.BoundingBox()
        for n, fs in frame:
            fb = fs.BoundingBox()
            if bb.xmax < fb.xmin or bb.xmin > fb.xmax or bb.ymax < fb.ymin or bb.ymin > fb.ymax \
                    or bb.zmax < fb.zmin or bb.zmin > fb.zmax:
                continue
            v = s.intersect(fs).Volume()
            if v > 0.01:
                worst.append((round(v, 3), k, at, n))
    assy.export(str(OUT / "assembly.step"))
    sheet(OUT, f, counts, made)
    for k in PARTS:
        print(f"{k:15s} x{counts[k]}")
    print("overlaps with frame parts:", worst if worst else "none")


if __name__ == "__main__":
    main()
