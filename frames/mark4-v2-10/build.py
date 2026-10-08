#!/usr/bin/env python3
"""Mark4 V2 10-inch frame: exact 2D profiles and drawings from its STEP model.

    pip install cadquery matplotlib
    python3 frames/mark4-v2-10/build.py [--step path/to/model.step]

Every part in the frame is a flat plate, so each one is a 2D profile plus a
thickness. This cuts each part through the middle of its thickness and writes
that profile exactly as modelled: nothing is redrawn or rounded.

Coordinates are the STEP model's own, in mm: x to the right, y forward, z up,
origin on the centreline. Horizontal parts keep their x, y positions in their
DXFs, so the standoff holes have the same coordinates in every plate and the
parts stack straight back into the assembly. The camera plate is drawn in its
own plane: y forward, z up.

Writes, next to this file:
    dxf/<part>.dxf        one profile per part, 1:1 mm, to extrude by its thickness
    dxf/assembly_top.dxf  every horizontal part and standoff overlaid, a layer each
    holes.csv             every hole and slot of every part, with coordinates
    drawing.pdf           dimensioned A4 sheets; print at 100 % (actual size)
    sheets/*.png          the same sheets as images
    CHECK.md              standoff alignment through the stack, and model quirks
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import Vector as V

HERE = Path(__file__).resolve().parent
STEP = HERE / "source" / "mark4_v2_10in.step"


# --------------------------------------------------------------------------
# Reading the assembly


def load_step(path: Path) -> dict:
    """Leaf parts of the STEP assembly by name, placed as in the assembly."""
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != 1:
        raise SystemExit(f"cannot read {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    parts = {}

    def name_of(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "?"

    def walk(label, loc):
        if tool.IsReference_s(label):
            ref = TDF_Label()
            tool.GetReferredShape_s(label, ref)
            walk(ref, loc * tool.GetLocation_s(label))
        elif tool.IsAssembly_s(label):
            kids = TDF_LabelSequence()
            tool.GetComponents_s(label, kids)
            for i in range(1, kids.Length() + 1):
                walk(kids.Value(i), loc)
        else:
            parts[name_of(label)] = cq.Shape.cast(tool.GetShape_s(label).Moved(loc))

    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    return parts


@dataclass
class Part:
    key: str
    title: str
    manual: str            # what the RJX assembly manual calls it
    sources: tuple         # names in the STEP; the first is the one drawn
    plane: str = "xy"      # "xy" lies flat; "yz" stands on edge (the camera plates)
    note: str = ""
    # filled in by extract()
    face: cq.Face = None
    t: float = 0.0
    z: tuple = (0.0, 0.0)  # bottom and top in the assembly (x for a "yz" part)
    holes: list = field(default_factory=list)
    slots: list = field(default_factory=list)
    cutouts: list = field(default_factory=list)
    sinks: list = field(default_factory=list)
    copies: list = field(default_factory=list)   # (STEP name, holes) of every copy in the assembly
    z2: tuple = (0.0, 0.0)                       # height range of a part that stands on edge

    @property
    def qty(self):
        return len(self.sources)


PARTS = [
    Part("top_plate", "Top plate", "Top plate", ("Top Plate",)),
    Part("lower_plate", "Lower plate", "Mid plate", ("Lower Plate",),
         note="carries the standoffs and the 30.5 / 20 mm stacks"),
    Part("belly_plate", "Belly plate", "Bottom plate", ("Belly Plate",), note="under the arms"),
    Part("chin_plate", "Chin plate", "Front plate", ("Chin Plate",), note="under the lower plate, ahead of the arms"),
    Part("tail_plate", "Tail plate", "Back plate", ("Tail Plate",), note="under the lower plate, behind the arms"),
    Part("arm", "Arm", "Arm", ("10in Arm", "10in Arm001", "10in Arm002", "10in Arm003"),
         note="drawn: front right. The other three are its mirror images (flip it over)"),
    Part("arm_brace", "Arm brace", "Arm bracer", ("Arm Brace001", "Arm Brace002"),
         note="drawn: right. The left one is its mirror image (flip it over)"),
    Part("cam_plate", "Camera plate", "Camera plate", ("Cam plate", "Cam plate001"), plane="yz",
         note="drawn: right, seen from the right; y forward, z up. The left one is its mirror image"),
]
BY_KEY = {p.key: p for p in PARTS}


# --------------------------------------------------------------------------
# Profiles and features


def profile(shape: cq.Shape, plane: str):
    """The part's through profile, as a face in the drawing plane, and its
    thickness and extent across the plane. Cut through the middle of the
    thickness, or for a countersunk part next to the face the countersinks
    don't reach, so the holes come out at their through size."""
    bb = shape.BoundingBox()
    if plane == "xy":
        lo, hi = bb.zmin, bb.zmax
        sinks = countersinks(shape)
        if not sinks:
            mid = (lo + hi) / 2
        elif all(zl < zs for *_, zl, zs in sinks):      # wide ends on the underside
            mid = hi - 0.02
        else:
            mid = lo + 0.02
        slab = cq.Solid.makeBox(bb.xlen + 20, bb.ylen + 20, 0.02, V(bb.xmin - 10, bb.ymin - 10, mid - 0.01))
        cut = shape.intersect(slab)
        face = max((f for f in cut.Faces() if f.normalAt().z > 0.999), key=lambda f: f.Area())
        face = face.translate(V(0, 0, -face.Center().z))
    else:   # stands on edge, thickness along x: map (y, z) to the drawing's (X, Y)
        lo, hi = bb.xmin, bb.xmax
        mid = (lo + hi) / 2
        slab = cq.Solid.makeBox(0.02, bb.ylen + 20, bb.zlen + 20, V(mid - 0.01, bb.ymin - 10, bb.zmin - 10))
        cut = shape.intersect(slab)
        face = max((f for f in cut.Faces() if f.normalAt().x > 0.999), key=lambda f: f.Area())
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.gp import gp_Trsf
        tr = gp_Trsf()
        tr.SetValues(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, -face.Center().x)    # (x, y, z) -> (y, z, x - mid)
        face = cq.Face(BRepBuilderAPI_Transform(face.wrapped, tr, True).Shape())
    return face, hi - lo, (lo, hi)


@dataclass
class Hole:
    x: float
    y: float
    d: float
    label: str = ""


@dataclass
class Slot:
    x1: float
    y1: float
    x2: float
    y2: float
    w: float
    label: str = ""

    @property
    def length(self):
        return math.dist((self.x1, self.y1), (self.x2, self.y2)) + self.w


def features(face: cq.Face):
    """Sort the face's inner wires into round holes, straight slots and other cut-outs."""
    holes, slots, cutouts = [], [], []
    for w in face.innerWires():
        es = w.Edges()
        arcs = [e for e in es if e.geomType() == "CIRCLE"]
        lines = [e for e in es if e.geomType() == "LINE"]
        circ = {(round(e.arcCenter().x, 4), round(e.arcCenter().y, 4), round(e.radius(), 4)) for e in arcs}
        if arcs and not lines and len(circ) == 1:
            x, y, r = circ.pop()
            holes.append(Hole(x, y, 2 * r))
        elif len(arcs) == 2 and len(lines) == 2 and abs(arcs[0].radius() - arcs[1].radius()) < 1e-4:
            a, b = (e.arcCenter() for e in arcs)
            slots.append(Slot(a.x, a.y, b.x, b.y, 2 * arcs[0].radius()))
        else:
            bb = w.BoundingBox()
            cutouts.append((bb.xmin, bb.ymin, bb.xmax, bb.ymax))
    holes.sort(key=lambda h: (-round(h.y, 1), h.x))
    slots.sort(key=lambda s: (-round((s.y1 + s.y2) / 2, 1), (s.x1 + s.x2) / 2))
    return holes, slots, cutouts


def countersinks(shape: cq.Shape):
    """Conical faces: (x, y, small dia, large dia, z of the large end)."""
    out = []
    for f in shape.Faces():
        if f.geomType() != "CONE":
            continue
        circles = [e for e in f.Edges() if e.geomType() == "CIRCLE"]
        small, large = min(circles, key=lambda e: e.radius()), max(circles, key=lambda e: e.radius())
        c = large.arcCenter()
        out.append((c.x, c.y, 2 * small.radius(), 2 * large.radius(), c.z, small.arcCenter().z))
    return out


def sink_angle(sink):
    x, y, ds, dl, zl, zs = sink
    return 2 * math.degrees(math.atan2((dl - ds) / 2, abs(zl - zs)))


def extract(path: Path):
    shapes = load_step(path)
    for p in PARTS:
        p.face, p.t, p.z = profile(shapes[p.sources[0]], p.plane)
        p.holes, p.slots, p.cutouts = features(p.face)
        p.sinks = countersinks(shapes[p.sources[0]])
        p.copies = [(n, features(profile(shapes[n], p.plane)[0])[0]) for n in p.sources] if p.plane == "xy" else []
        if p.plane == "yz":
            bb = shapes[p.sources[0]].BoundingBox()
            p.z2 = (bb.zmin, bb.zmax)
    standoffs = sorted(((s.BoundingBox(), n) for n, s in shapes.items() if "Standoff" in n),
                       key=lambda bn: (-round(bn[0].center.y, 1), -bn[0].center.x))
    so = []
    for i, (bb, n) in enumerate(standoffs):
        so.append(dict(label=f"S{i + 1}", x=bb.center.x, y=bb.center.y, d=bb.xlen, z0=bb.zmin, z1=bb.zmax, name=n))
    label_holes(so)
    return shapes, so


def label_holes(so):
    """S1..S8 for the holes on a standoff's axis (in every plate), H.. and L.. for the rest."""
    for p in PARTS:
        n = 0
        for h in p.holes:
            if p.plane == "xy":
                s = next((s for s in so if math.dist((s["x"], s["y"]), (h.x, h.y)) < 0.5), None)
                if s:
                    h.label = s["label"]
                    continue
            n += 1
            h.label = f"H{n}"
        for i, s in enumerate(p.slots):
            s.label = f"L{i + 1}"


# --------------------------------------------------------------------------
# Checks: do the standoffs line up through the stack?


def check(so):
    """For each standoff, the hole on its axis in every horizontal part, and how
    far it sits from the standoff's own axis."""
    rows = []
    for s in so:
        hits = []
        for p in PARTS:
            for name, holes in p.copies:
                h = next((h for h in holes if math.dist((s["x"], s["y"]), (h.x, h.y)) < 0.5), None)
                if h:
                    hits.append((p, h, math.dist((s["x"], s["y"]), (h.x, h.y))))
        rows.append((s, hits))
    return rows


# --------------------------------------------------------------------------
# Exports


def export_dxf(out: Path, so):
    from cadquery.occ_impl.exporters.dxf import DxfDocument
    d = out / "dxf"
    d.mkdir(parents=True, exist_ok=True)
    for p in PARTS:
        doc = DxfDocument(setup=True, metadata={"part": p.title, "thickness_mm": f"{p.t:g}"})
        doc.add_shape(p.face, layer="PROFILE")
        if p.sinks:
            doc.document.layers.add("COUNTERSINK", color=3)
            for x, y, ds, dl, *_ in p.sinks:
                doc.document.modelspace().add_circle((x, y), dl / 2, dxfattribs={"layer": "COUNTERSINK"})
        doc.document.saveas(str(d / f"{p.key}.dxf"))
    # every horizontal part overlaid as in the assembly, plus the standoffs
    doc = DxfDocument(setup=True)
    for i, p in enumerate(PARTS):
        if p.plane != "xy":
            continue
        doc.document.layers.add(p.key.upper(), color=i + 1)
        doc.add_shape(p.face, layer=p.key.upper())
    doc.document.layers.add("STANDOFFS", color=1)
    msp = doc.document.modelspace()
    for s in so:
        msp.add_circle((s["x"], s["y"]), s["d"] / 2, dxfattribs={"layer": "STANDOFFS"})
        msp.add_text(s["label"], height=2.5, dxfattribs={"layer": "STANDOFFS"}).set_placement((s["x"] + 3.5, s["y"] + 1))
    doc.document.saveas(str(d / "assembly_top.dxf"))


def export_csv(out: Path):
    with open(out / "holes.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["part", "label", "kind", "x_mm", "y_mm", "x2_mm", "y2_mm", "dia_or_width_mm", "note"])
        for p in PARTS:
            ax = "y,z" if p.plane == "yz" else "x,y"
            for h in p.holes:
                sink = next((s for s in p.sinks if math.dist(s[:2], (h.x, h.y)) < 0.05), None)
                note = f"countersink to {sink[3]:.2f} on the {'bottom' if sink[4] < sink[5] else 'top'} face" if sink else ""
                w.writerow([p.key, h.label, "hole", f"{h.x:.3f}", f"{h.y:.3f}", "", "", f"{h.d:.3f}",
                            (note + f" (coordinates are {ax})" if p.plane == "yz" else note)])
            for s in p.slots:
                w.writerow([p.key, s.label, "slot", f"{s.x1:.3f}", f"{s.y1:.3f}", f"{s.x2:.3f}", f"{s.y2:.3f}",
                            f"{s.w:.3f}", "end-arc centres"])


def bed_fit(face):
    """Longest extent of a part and the smallest square it fits in, over 0.5 degree turns."""
    pts = np.vstack([edge_points(e) for e in face.outerWire().Edges()])
    best, longest = (1e9, 0.0), 0.0
    for a in np.radians(np.arange(0, 180, 0.5)):
        ext = np.ptp(pts @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]), axis=0)
        longest = max(longest, ext[0])
        best = min(best, (ext.max(), math.degrees(a)))
    return longest, best[0], best[1]


def export_check(out: Path, so, rows):
    lo = BY_KEY["lower_plate"]
    lines = [
        "# Standoff alignment check",
        "",
        "Generated by `build.py` from `source/mark4_v2_10in.step`. Coordinates in mm, x to the right,",
        "y forward, from the model's origin.",
        "",
        "Each standoff's axis and the hole on that axis in every horizontal part it",
        "passes through or bolts to. *Off* is how far the hole's centre sits from the",
        "standoff's axis.",
        "",
        "| Standoff | Axis x | Axis y | Part | Hole x | Hole y | Hole Ø | Off |",
        "|---|---:|---:|---|---:|---:|---:|---:|",
    ]
    worst = 0.0
    for s, hits in rows:
        first = True
        for p, h, off in hits:
            worst = max(worst, off)
            head = f"| **{s['label']}** | {s['x']:.2f} | {s['y']:.2f} " if first else "| | | "
            lines.append(head + f"| {p.title} | {h.x:.2f} | {h.y:.2f} | {h.d:.2f} | {off:.2f} |")
            first = False
    top, low = BY_KEY["top_plate"], BY_KEY["lower_plate"]
    tl = max(math.dist((a.x, a.y), (b.x, b.y)) for a in top.holes for b in low.holes if a.label == b.label and a.label.startswith("S"))
    lines += [
        "",
        f"Largest offset of any hole from its standoff's axis: **{worst:.2f} mm**. Between the",
        f"top plate and the lower plate, the two plates the standoffs join, it is **{tl:.2f} mm**.",
        "That is far inside what a CNC cutter (about ±0.1 mm) or a printer (about ±0.2 mm)",
        "holds, so the plates line up as modelled. The offsets come from how the parts were",
        "placed in the model's assembly, a few hundredths of a millimetre apart.",
        "",
        "## Model quirks worth knowing",
        "",
    ]
    q = []
    asym = [(h, next((g for g in lo.holes if abs(g.x + h.x) < 0.05 and abs(g.y - h.y) < 0.05 and abs(g.d - h.d) < 0.01), None))
            for h in lo.holes if h.x > 0.1]
    odd = [h for h, m in asym if m is None]
    if odd:
        q.append("- **Lower plate, rear 30.5 mm pattern is not square or centred.** These holes have no mirror "
                 "partner across the centreline: " + ", ".join(f"{h.label} ({h.x:.2f}, {h.y:.2f})" for h in odd)
                 + ". The other side's holes are at " + ", ".join(
                     f"({g.x:.2f}, {g.y:.2f})" for g in lo.holes if g.x < -0.1 and not any(
                         abs(g.x + m.x) < 0.05 and abs(g.y - m.y) < 0.05 for m, mm in asym if mm is not None))
                 + ". A true 30.5 × 30.5 pattern on the centreline would put them at x ±15.25. They are drawn "
                 "as modelled; check them against your VTX or plate before you rely on them.")
    arm = BY_KEY["arm"]
    motor = max(arm.holes, key=lambda h: h.d)
    mx, my = motor.x, motor.y
    ry = 2 * (-19.084) - my
    wb = math.dist((mx, my), (-mx, ry))
    q.append(f"- **Wheelbase.** The motor centres are at x ±{mx:.2f}, y {my:.2f} (front) and y {ry:.2f} "
             f"(rear): {2 * mx:.1f} mm apart side to side, {my - ry:.1f} mm front to back, **{wb:.1f} mm** "
             "diagonally. RJX lists its Mark4 V2 10\" at 427 mm and GEPRC its Mark4 10\" at 428-429 mm, so "
             "the arms in this model are longer than either. The centre plates and standoffs don't depend on it.")
    q.append(f"- **Thicknesses.** The model has arms {arm.t:g} mm, lower and belly plates "
             f"{BY_KEY['lower_plate'].t:g} mm, top plate {BY_KEY['top_plate'].t:g} mm, braces "
             f"{BY_KEY['arm_brace'].t:g} mm, camera plates {BY_KEY['cam_plate'].t:g} mm. RJX lists 7.5 mm arms; "
             "measure yours.")
    lines += q
    lines += ["", "## Print bed fit", "",
              "The smallest square bed each part fits, turned to the best angle (outline only, no brim).", "",
              "| Part | Longest | Smallest square bed | Turned | Fits 180 x 180 |", "|---|---:|---:|---:|---|"]
    for p in PARTS:
        longest, side, ang = bed_fit(p.face)
        lines.append(f"| {p.title} | {longest:.1f} | {side:.1f} | {ang:.0f}° | {'yes' if side <= 180 else 'no'} |")
    (out / "CHECK.md").write_text("\n".join(lines) + "\n")
    return worst, tl, wb, (mx, my, ry)


# --------------------------------------------------------------------------
# Drawing sheets (matplotlib, A4 landscape, true scale)

A4 = (297.0, 210.0)
INK, DIM, FAINT, ACC = "#111111", "#1f4e9c", "#9aa0a6", "#b3261e"
PT = 25.4 / 72      # mm per point


def edge_points(e, n=40):
    k = 2 if e.geomType() == "LINE" else max(8, int(n * e.Length() / 20) + 8)
    return np.array([(q.x, q.y) for q in (e.positionAt(t) for t in np.linspace(0, 1, k))])


class Sheet:
    def __init__(self, title, scale, sub, number, total):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig = plt.figure(figsize=(A4[0] / 25.4, A4[1] / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        a = self.ax
        a.set_xlim(0, A4[0])
        a.set_ylim(0, A4[1])
        a.set_aspect("equal")
        a.axis("off")
        a.add_patch(plt.Rectangle((8, 8), A4[0] - 16, A4[1] - 16, fill=False, lw=0.8, ec=INK))
        x0, y0, w = A4[0] - 8 - 104, 8, 104
        a.add_patch(plt.Rectangle((x0, y0), w, 22, fill=False, lw=0.6, ec=INK))
        a.plot([x0, x0 + w], [y0 + 11, y0 + 11], lw=0.4, c=INK)
        a.plot([x0 + 74, x0 + 74], [y0, y0 + 22], lw=0.4, c=INK)
        a.text(x0 + 2.5, y0 + 16.5, title, fontsize=8.5, weight="bold", va="center")
        a.text(x0 + 2.5, y0 + 5.5, sub, fontsize=5.8, va="center")
        a.text(x0 + 76.5, y0 + 16.5, f"Scale {scale}", fontsize=6.5, va="center")
        a.text(x0 + 76.5, y0 + 5.5, f"Sheet {number} of {total}", fontsize=6.5, va="center")
        a.text(12, 11.5, "Mark4 V2 10\" frame, from its STEP model.  mm.  Print at 100 % (actual size) and check the bar.",
               fontsize=5.5, color="#444")
        a.plot([12, 62], [16, 16], lw=1.1, c=INK)
        for xx in (12, 62):
            a.plot([xx, xx], [14.6, 17.4], lw=1.1, c=INK)
        a.text(37, 18, "50 mm on paper", fontsize=5.5, ha="center")

    def view(self, at, scale=1.0, rot=0.0, **kw):
        return View(self, at, scale, rot, **kw)

    def text(self, x, y, s, fs=6.5, **kw):
        self.ax.text(x, y, s, fontsize=fs, **kw)

    def table(self, x, y, header, rows, widths, fs=5.6, row_h=3.15, aligns=None):
        """A table with its top-left corner at (x, y) on the paper. Returns its bottom y."""
        a = self.ax
        aligns = aligns or ["left"] + ["right"] * (len(header) - 1)
        tw = sum(widths)
        a.plot([x, x + tw], [y, y], lw=0.5, c=INK)
        cx = x
        for h, w, al in zip(header, widths, aligns):
            a.text(cx + (w - 1 if al == "right" else 0.8), y - row_h * 0.55, h, fontsize=fs, weight="bold",
                   ha=al, va="center")
            cx += w
        yy = y - row_h
        a.plot([x, x + tw], [yy, yy], lw=0.35, c=INK)
        for r in rows:
            cx = x
            for v, w, al in zip(r, widths, aligns):
                a.text(cx + (w - 1 if al == "right" else 0.8), yy - row_h * 0.55, v, fontsize=fs, ha=al,
                       va="center", family="DejaVu Sans Mono" if al == "right" else None)
                cx += w
            yy -= row_h
        a.plot([x, x + tw], [yy, yy], lw=0.5, c=INK)
        return yy

    def axes_glyph(self, x, y, right, up, size=9):
        """Small arrows naming the model axes that run right and up on this view."""
        a = self.ax
        for (dx, dy), lab in (((size, 0), right), ((0, size), up)):
            a.annotate("", xy=(x + dx, y + dy), xytext=(x, y),
                       arrowprops=dict(arrowstyle="-|>", lw=0.6, color=INK, mutation_scale=6))
            a.text(x + dx * 1.15 + (1 if dy == 0 else 0), y + dy * 1.12 + (0 if dy else 0), lab, fontsize=6,
                   ha="left" if dy == 0 else "center", va="center" if dy == 0 else "bottom")

    def save(self, pdf, png):
        pdf.savefig(self.fig)
        self.fig.savefig(png, dpi=160)
        self.plt.close(self.fig)


class View:
    """A part placed on a sheet: model mm -> paper mm, at a scale, turned by rot
    degrees. Lines are clipped to `clip` (paper x0, y0, x1, y1), by default the border."""

    def __init__(self, sheet, at, scale, rot, clip=(8, 8, A4[0] - 8, A4[1] - 8)):
        self.s, self.ax, self.at, self.sc = sheet, sheet.ax, np.asarray(at, float), scale
        c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        self.R = np.array([[c, s], [-s, c]])
        x0, y0, x1, y1 = clip
        self.clip = sheet.plt.Rectangle((x0, y0), x1 - x0, y1 - y0, transform=sheet.ax.transData)

    def p(self, pts):
        return np.asarray(pts, float).reshape(-1, 2) @ self.R * self.sc + self.at

    def _plot(self, q, **kw):
        for ln in self.ax.plot(q[:, 0], q[:, 1], **kw):
            ln.set_clip_path(self.clip)

    def face(self, face, lw=0.6, color=INK, ls="-", z=2):
        for w in [face.outerWire()] + face.innerWires():
            for e in w.Edges():
                self._plot(self.p(edge_points(e)), lw=lw, c=color, ls=ls, solid_capstyle="round", zorder=z)

    def poly(self, pts, fill=None, **kw):
        q = self.p(pts)
        if fill:
            self.ax.fill(q[:, 0], q[:, 1], color=fill, lw=0, zorder=kw.get("zorder", 1) - 0.5)
        self._plot(q, **kw)

    def circle(self, x, y, r, **kw):
        t = np.linspace(0, 2 * math.pi, 96)
        self.poly(np.c_[x + r * np.cos(t), y + r * np.sin(t)], **kw)

    def centre(self, x, y, size=2.5):
        (px, py), = self.p([(x, y)])
        kw = dict(lw=0.25, c=DIM, zorder=1)
        self.ax.plot([px - size, px + size], [py, py], **kw)
        self.ax.plot([px, px], [py - size, py + size], **kw)

    def origin(self, size=3.0):
        (ox, oy), = self.p([(0, 0)])
        self.ax.plot([ox - size, ox + size], [oy, oy], lw=0.45, c=INK)
        self.ax.plot([ox, ox], [oy - size, oy + size], lw=0.45, c=INK)
        self.ax.add_patch(self.s.plt.Circle((ox, oy), size * 0.37, fill=False, lw=0.45, ec=INK))

    def label(self, x, y, text, dx=1.2, dy=1.2, fs=4.6, color=DIM, ha="left", **kw):
        (px, py), = self.p([(x, y)])
        self.ax.text(px + dx, py + dy, text, fontsize=fs, color=color, ha=ha, va="center", zorder=5,
                     bbox=dict(fc="white", ec="none", pad=0.15, alpha=0.85), **kw)

    def dim(self, p1, p2, axis, at, text, fs=5.6, ext=True):
        """A dimension between two model points, its line horizontal ('h') or vertical
        ('v') on the paper, placed at paper coordinate `at` (y for 'h', x for 'v')."""
        (ax1, ay1), (ax2, ay2) = self.p([p1, p2])
        if axis == "h":
            d1, d2 = np.array([ax1, at]), np.array([ax2, at])
        else:
            d1, d2 = np.array([at, ay1]), np.array([at, ay2])
        if ext:
            for (sx, sy), d in zip(((ax1, ay1), (ax2, ay2)), (d1, d2)):
                v = d - (sx, sy)
                ln = np.linalg.norm(v)
                if ln > 0.8:
                    u = v / ln
                    a0, a1 = np.array((sx, sy)) + u * 0.8, d + u * 1.2
                    self.ax.plot([a0[0], a1[0]], [a0[1], a1[1]], lw=0.25, c=DIM, zorder=1)
        self.ax.annotate("", xy=d1, xytext=d2, arrowprops=dict(arrowstyle="<|-|>", lw=0.4, color=DIM,
                                                               mutation_scale=4.5, shrinkA=0, shrinkB=0))
        mid = (d1 + d2) / 2
        if axis == "h":
            self.ax.text(mid[0], mid[1] + 1.3, text, fontsize=fs, color=DIM, ha="center", va="bottom",
                         bbox=dict(fc="white", ec="none", pad=0.1))
        else:
            self.ax.text(mid[0] - 1.3, mid[1], text, fontsize=fs, color=DIM, ha="center", va="bottom",
                         rotation=90, rotation_mode="anchor", bbox=dict(fc="white", ec="none", pad=0.1))


def f2(v):
    return f"{0.0 if abs(v) < 0.005 else v:.2f}"


def fg(v):
    return f"{round(v, 3) + 0.0:g}"


def hole_rows(p):
    rows = []
    for h in p.holes:
        sink = next((s for s in p.sinks if math.dist(s[:2], (h.x, h.y)) < 0.05), None)
        note = (f"csk Ø{sink[3]:.1f} x {sink_angle(sink):.0f}°, {'under' if sink[4] < sink[5] else 'top'}"
                if sink else "")
        rows.append([h.label, f2(h.x), f2(h.y), f2(h.d), note])
    return rows


def slot_rows(p):
    return [[s.label, f2(s.x1), f2(s.y1), f2(s.x2), f2(s.y2), f2(s.w)] for s in p.slots]


def draw_labels(v, p, fs=4.4):
    for h in p.holes:
        r = h.d / 2 * v.sc
        std = h.label.startswith("S")
        v.label(h.x, h.y, h.label, dx=r * 0.75 + 0.5, dy=r * 0.75 + 0.9, fs=fs + (0.5 if std else 0),
                color=ACC if std else DIM, weight="bold" if std else None)
    for s in p.slots:
        v.label((s.x1 + s.x2) / 2, (s.y1 + s.y2) / 2, s.label, dx=s.w * v.sc / 2 + 0.6, dy=0.9, fs=fs)


def nose_right(sheet, face, left, mid, scale=1.0, **kw):
    """A top view with the nose to the right: model y runs right, -x runs up.
    The part's rear edge lands at paper x `left`, its centreline-of-extent at paper y `mid`."""
    bb = face.BoundingBox()
    return sheet.view((left - bb.ymin * scale, mid + (bb.xmin + bb.xmax) / 2 * scale), scale, -90.0, **kw)


def as_drawn(sheet, face, left, mid, scale=1.0, **kw):
    """The part in its own drawing axes (x right, y up), its left edge at paper x
    `left` and its middle at paper y `mid`."""
    bb = face.BoundingBox()
    return sheet.view((left - bb.xmin * scale, mid - (bb.ymin + bb.ymax) / 2 * scale), scale, 0.0, **kw)


def overall_dims(v, face, off=6.0, nose=True, labels=True, names=("x", "y")):
    """Length and width of a part, with the model coordinates of its extent."""
    bb = face.BoundingBox()
    if nose:
        top = v.p([(bb.xmin, 0)])[0][1]
        front = v.p([(0, bb.ymax)])[0][0]
        v.dim((bb.xmin, bb.ymin), (bb.xmin, bb.ymax), "h", top + off,
              f"{bb.ylen:.2f}" + (f"   (y {f2(bb.ymin)} to {f2(bb.ymax)})" if labels else ""))
        v.dim((bb.xmin, bb.ymax), (bb.xmax, bb.ymax), "v", front + off,
              f"{bb.xlen:.2f}" + (f"   (x {f2(bb.xmin)} to {f2(bb.xmax)})" if labels else ""))
        return top + off
    top = v.p([(0, bb.ymax)])[0][1]
    right = v.p([(bb.xmax, 0)])[0][0]
    v.dim((bb.xmin, bb.ymax), (bb.xmax, bb.ymax), "h", top + off,
          f"{bb.xlen:.2f}" + (f"   ({names[0]} {f2(bb.xmin)} to {f2(bb.xmax)})" if labels else ""))
    v.dim((bb.xmax, bb.ymin), (bb.xmax, bb.ymax), "v", right + off,
          f"{bb.ylen:.2f}" + (f"   ({names[1]} {f2(bb.ymin)} to {f2(bb.ymax)})" if labels else ""))
    return top + off


def part_view(v, p, labels=True):
    v.face(p.face, lw=0.6)
    for h in p.holes:
        v.centre(h.x, h.y, h.d / 2 * v.sc + 1.2)
    for x, y, ds, dl, *_ in p.sinks:
        v.circle(x, y, dl / 2, lw=0.35, c=DIM, ls=(0, (2, 1)))
    if labels:
        draw_labels(v, p)


HOLE_HEAD = ["", "x", "y", "Ø", ""]
HOLE_W = [9, 14, 15, 11, 30]


def hole_tables(sh, p, x, y, cols=None, gap=6, row_h=3.0, fs=5.2, head=HOLE_HEAD, widths=HOLE_W):
    rws = hole_rows(p)
    cols = cols or (1 if len(rws) <= 10 else 2 if len(rws) <= 22 else 3)
    per = math.ceil(len(rws) / cols)
    yb = y
    for c in range(cols):
        chunk = rws[c * per:(c + 1) * per]
        if chunk:
            yb = min(yb, sh.table(x + c * (sum(widths) + gap), y, head, chunk, widths, fs=fs, row_h=row_h))
    return yb


def sheets(out: Path, so, rows, wb_info):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.backends.backend_pdf import PdfPages
    (out / "sheets").mkdir(exist_ok=True)
    total = 7
    worst, tl, wb, (mx, my, ry) = wb_info
    P = BY_KEY
    low, top = P["lower_plate"], P["top_plate"]
    S_low = {h.label: h for h in low.holes if h.label.startswith("S")}
    S_top = {h.label: h for h in top.holes if h.label.startswith("S")}
    grey = "#5b5f66"
    pages = []

    # ---- 1. overview, 1:2, nose up -------------------------------------------
    sh = Sheet("Frame overview", "1:2", f"Motor centres {2 * mx:.1f} x {my - ry:.1f} mm, {wb:.1f} mm diagonal",
               1, total)
    yc = -19.084                                   # the arms' fore-and-aft mirror line
    v = sh.view((110, 110 - 0.5 * yc), 0.5, 0.0, clip=(8, 31, 211, A4[1] - 8))
    arm = P["arm"]
    for sx, sy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        f = arm.face
        if sx < 0:
            f = f.mirror("YZ", V(-0.017, 0, 0))
        if sy < 0:
            f = f.mirror("XZ", V(0, yc, 0))
        v.face(f, lw=0.45)
    v.face(P["arm_brace"].face, lw=0.45)
    v.face(P["arm_brace"].face.mirror("YZ", V(-0.017, 0, 0)), lw=0.45)
    for key in ("belly_plate", "chin_plate", "tail_plate"):
        v.face(P[key].face, lw=0.35, color=FAINT)
    v.face(low.face, lw=0.5)
    v.face(top.face, lw=0.45, color="#4a6fa5")
    pr = 5 * 25.4
    for sx, yy in ((1, my), (-1, my), (1, ry), (-1, ry)):
        v.circle(sx * mx, yy, pr, lw=0.35, c=FAINT, ls=(0, (4, 3)))
        v.centre(sx * mx, yy, 3)
    for s in so:
        v.circle(s["x"], s["y"], s["d"] / 2, lw=0.6, c=ACC)
    v.dim((-mx, ry), (mx, ry), "h", v.p([(0, ry)])[0][1], f"{2 * mx:.2f}", ext=False)
    v.dim((mx, ry), (mx, my), "v", v.p([(mx, 0)])[0][0], f"{my - ry:.2f}", ext=False)
    (ax1, ay1), (ax2, ay2) = v.p([(-mx, my), (mx, ry)])
    sh.ax.annotate("", xy=(ax1, ay1), xytext=(ax2, ay2),
                   arrowprops=dict(arrowstyle="<|-|>", lw=0.35, color=DIM, mutation_scale=4.5, shrinkA=0, shrinkB=0))
    ang = math.degrees(math.atan2(ay2 - ay1, ax2 - ax1))
    tx, ty = ax1 + (ax2 - ax1) * 0.17, ay1 + (ay2 - ay1) * 0.17
    sh.text(tx + 1.5, ty + 1.5, f"{wb:.1f}", fs=5.6, color=DIM, rotation=ang, ha="center", va="bottom",
            rotation_mode="anchor")
    v.label(mx, my, f"motor  x ±{mx:.2f}, y {my:.2f}", dx=-4, dy=7, fs=5, color=INK, ha="right")
    v.label(mx, ry, f"motor  x ±{mx:.2f}, y {ry:.2f}", dx=-4, dy=-7, fs=5, color=INK, ha="right")
    sh.axes_glyph(216, 70, "+x  right", "+y  front")
    sh.text(216, 60, "Seen from above, nose up.\nDashed: 10\" prop discs.\nRed: standoffs.  Blue: top plate.",
            fs=5.3, color="#444", va="top")
    tx = 214
    sh.text(tx, 196, "Parts", fs=7.5, weight="bold", va="top")
    prow = [[str(p.qty), p.title, fg(p.t), p.manual] for p in PARTS]
    prow.append([str(len(so)), "Standoff M3x35 Ø6", "", "Standoff spacer"])
    yb = sh.table(tx, 190, ["Qty", "Part (STEP)", "t mm", "RJX manual"], prow, [7, 26, 9, 25], fs=5.2,
                  aligns=["right", "left", "right", "left"])
    sh.text(tx, yb - 4, "Stack-up: z in mm, 0 = underside\nof the lower plate", fs=6.0, weight="bold", va="top")
    stack = [("top plate", top.z), ("standoffs", (so[0]["z0"], so[0]["z1"])), ("camera plates", P["cam_plate"].z2),
             ("lower plate", low.z), ("arm braces", P["arm_brace"].z), ("chin, tail plates", P["chin_plate"].z),
             ("arms", arm.z), ("belly plate", P["belly_plate"].z)]
    yb = sh.table(tx, yb - 12, ["Part", "z from", "z to"], [[n, fg(a), fg(b)] for n, (a, b) in stack],
                  [37, 15, 15], fs=5.2)
    sh.text(tx, yb - 4, "Every part is a flat plate: extrude its\nDXF profile by its thickness, at its z.",
            fs=5.3, va="top", color="#444")
    pages.append(("1_overview", sh))

    # ---- 2. standoffs and stack-up, 1:1 ------------------------------------
    sh = Sheet("Standoffs (struts) and stack-up", "1:1", "8 x M3 x 35 mm standoffs, Ø6, lower plate to top plate",
               2, total)
    v = nose_right(sh, low.face, 18, 150)
    v.face(low.face, lw=0.55)
    v.face(top.face, lw=0.5, color="#4a6fa5", ls=(0, (3, 1.5)))
    for s in so:
        v.circle(s["x"], s["y"], s["d"] / 2, lw=0.7, c=ACC)
    for h in S_low.values():
        v.centre(h.x, h.y, 4.5)
    v.origin(4.0)
    pairs = sorted({round(h.y, 2) for h in S_low.values()}, reverse=True)
    xr = {y: max(h.x for h in S_low.values() if abs(h.y - y) < 0.01) for y in pairs}
    ord_y = 181.0
    for y in [0.0] + pairs:
        (px, py), = v.p([(0.0 if y == 0.0 else -xr[y], y)])
        sh.ax.plot([px, px], [py + (1.0 if y else 2.2), ord_y], lw=0.25, c=DIM, zorder=1)
        sh.text(px, ord_y + 0.8, f2(y), fs=5.6, color=DIM, rotation=90, ha="center", va="bottom")
    sh.text(v.p([(0, pairs[0])])[0][0] + 8, ord_y + 4.5, "y of each standoff pair,\nfrom the origin ⊕", fs=5,
            color=DIM, va="center")
    for y in pairs:
        (px, _), = v.p([(0, y)])
        v.dim((xr[y], y), (-xr[y], y), "v", px + 6.5, f2(2 * xr[y]))
    for h in S_low.values():
        v.label(h.x, h.y, h.label, dx=4.0, dy=-3.6 if h.x > 0 else 3.6, fs=5.4, color=ACC, weight="bold")
    sh.axes_glyph(243, 160, "+y  front", "−x  left")
    sh.text(243, 150, "Top view, nose right.\nSolid: lower plate.\nDashed: top plate.\nRed: standoffs, Ø6.",
            fs=5.3, color="#444", va="top")

    # side view from the right, nose right: paper x = y, paper y = z
    sv = sh.view((18 - low.face.BoundingBox().ymin, 63), 1.0, 0.0)
    ylo, yhi = -126.0, 92.0

    def band(lo_hi_y, lo_hi_z, edge=INK, fill="#eceef1", lw=0.45):
        (y0, y1), (z0, z1) = lo_hi_y, lo_hi_z
        y0, y1 = max(y0, ylo), min(y1, yhi)
        sv.poly([(y0, z0), (y1, z0), (y1, z1), (y0, z1), (y0, z0)], fill=fill, lw=lw, c=edge, zorder=2)

    def yext(p):
        bb = p.face.BoundingBox()
        return bb.ymin, bb.ymax

    band((-173.47, 135.30), arm.z, edge=grey, fill="#f4f5f7", lw=0.35)
    for key in ("belly_plate", "chin_plate", "tail_plate", "lower_plate", "top_plate"):
        band(yext(P[key]), P[key].z)
    for y in pairs:
        band((y - 3, y + 3), (so[0]["z0"], so[0]["z1"]), edge=ACC, fill="#f8e3e1", lw=0.5)
    sv.face(P["cam_plate"].face, lw=0.45, color="#4a6fa5")
    right = sv.p([(yhi, 0)])[0][0]
    for z in sorted({top.z[1], top.z[0], low.z[1], 0.0, P["chin_plate"].z[0], arm.z[0], P["belly_plate"].z[0]}):
        (_, py), = sv.p([(0, z)])
        sh.ax.plot([right + 1, right + 3.5], [py, py], lw=0.25, c=DIM)
        sh.text(right + 4, py, fg(z), fs=4.8, color=DIM, va="center")
    sv.dim((yhi, low.z[1]), (yhi, top.z[0]), "v", right + 16, f"{fg(so[0]['z1'] - so[0]['z0'])} standoff")
    for name, (y, z) in (("top plate 2", (-90, top.z[1] + 0.8)), ("lower plate 3", (-90, low.z[1] + 0.8)),
                         ("standoffs", (-55.06 + 4, 20)), ("camera plate", (62, 41)),
                         ("arms 7 (cut short)", (-90, arm.z[0] - 0.8)), ("belly 3", (-19, P["belly_plate"].z[0] - 0.8)),
                         ("chin", (70, P["chin_plate"].z[0] - 0.8)), ("tail", (-117, P["tail_plate"].z[0] - 0.8))):
        (px, py), = sv.p([(y, z)])
        below = z < 0
        sh.text(px, py, name, fs=4.7, color="#444", ha="left" if name == "standoffs" else "center",
                va="top" if below else "bottom")
    sh.text(right + 4, 116, "Side view from the right,\nnose right; z in mm.", fs=5.3, color="#444", va="top")

    srows = []
    for s, hits in rows:
        h = S_low[s["label"]]
        t = S_top[s["label"]]
        names = ", ".join(sorted({p.title.split()[0] for p, hh, off in hits if p.key not in ("top_plate",
                                                                                            "lower_plate")}))
        srows.append([s["label"], f2(h.x), f2(h.y), "same" if (f2(h.x), f2(h.y)) == (f2(t.x), f2(t.y))
                      else f"y {f2(t.y)}", names])
    sh.text(14, 46, "Standoff holes (lower plate; use these in every plate)", fs=6.2, weight="bold", va="top")
    w = [7, 12, 14, 15, 35]
    sh.table(14, 42, ["", "x", "y", "Top plate", "Bolts from below via"], srows[:4], w, fs=5.0, row_h=2.9,
             aligns=["left", "right", "right", "right", "left"])
    sh.table(100, 42, ["", "x", "y", "Top plate", "Bolts from below via"], srows[4:], w, fs=5.0, row_h=2.9,
             aligns=["left", "right", "right", "right", "left"])
    sh.text(14, 25.5, f"In the model every hole on a standoff axis is within {worst:.2f} mm of it, and the top and lower "
            f"plates within {tl:.2f} mm of each other (CHECK.md).", fs=5.0, color="#444", va="top")
    pages.append(("2_standoffs", sh))

    # ---- 3, 4: top and lower plates, 1:1 -------------------------------------
    def plate_sheet(num, p, title, sub):
        sh = Sheet(title, "1:1", sub, num, total)
        v = nose_right(sh, p.face, 18, 158)
        part_view(v, p)
        v.origin()
        overall_dims(v, p.face)
        sh.axes_glyph(248, 168, "+y  front", "−x  left")
        sh.text(14, 124, f"Hole centres from the model's origin ⊕: x right, y forward.  S1-S8: standoff holes, "
                f"on the same axes in every plate.  Thickness {fg(p.t)} mm.", fs=5.3, color="#444")
        if p.cutouts:
            sh.text(14, 119.5, f"{len(p.cutouts)} cut-out{'s' if len(p.cutouts) > 1 else ''} and the outline: "
                    f"take them from dxf/{p.key}.dxf.", fs=5.3, color="#444")
        yb = hole_tables(sh, p, 14, 114)
        if p.slots:
            sh.table(14 + 2 * (sum(HOLE_W) + 6) if len(p.holes) > 10 else 14 + sum(HOLE_W) + 6, 114,
                     ["Slot", "x1", "y1", "x2", "y2", "w"], slot_rows(p), [9, 13, 15, 13, 15, 10], fs=5.0, row_h=3.0)
        return sh

    pages.append(("3_top_plate", plate_sheet(3, top, "Top plate", f"Qty 1.  Carbon {fg(top.t)} mm.  Battery on top")))
    pages.append(("4_lower_plate", plate_sheet(4, low, "Lower plate (manual: mid plate)",
                                               f"Qty 1.  Carbon {fg(low.t)} mm.  Standoffs and the stacks")))

    # ---- 5. belly, chin and tail plates -------------------------------------
    sh = Sheet("Belly, chin and tail plates", "1:1",
               f"1 each.  Belly carbon {fg(P['belly_plate'].t)} mm; chin and tail {fg(P['chin_plate'].t)} mm",
               5, total)
    for key, left in (("belly_plate", 22), ("chin_plate", 150), ("tail_plate", 214)):
        p = P[key]
        v = nose_right(sh, p.face, left, 150)
        part_view(v, p)
        ytop = overall_dims(v, p.face, off=5, labels=False)
        sh.text(left, ytop + 8, p.title, fs=6.5, weight="bold")
    sh.axes_glyph(250, 112, "+y  front", "−x  left")
    sh.text(14, 112, "Belly plate", fs=6, weight="bold")
    hole_tables(sh, P["belly_plate"], 14, 109, cols=2, gap=4, fs=5.0, row_h=2.9, widths=[9, 14, 15, 11, 2])
    sh.text(128, 112, "Chin plate", fs=6, weight="bold")
    yb = sh.table(128, 109, HOLE_HEAD, hole_rows(P["chin_plate"]), [9, 14, 15, 11, 34], fs=5.0, row_h=2.9)
    sh.text(128, yb - 4, "Tail plate", fs=6, weight="bold")
    sh.table(128, yb - 7, HOLE_HEAD, hole_rows(P["tail_plate"]), [9, 14, 15, 11, 34], fs=5.0, row_h=2.9)
    sh.text(14, 50, "Chin and tail: Ø2.8 through, countersunk (dashed) on the underside, under standoffs S1, S2 and "
            "S7, S8.", fs=5.3, color="#444")
    sh.text(14, 45.5, "Belly: under the arms; S3-S6 bolt up through it, the arms and the lower plate into the "
            "standoffs.  Hole centres from the model's origin.", fs=5.3, color="#444")
    pages.append(("5_belly_chin_tail", sh))

    # ---- 6. arm, 1:1, as it sits (front right), x right, y up -------------
    p = arm
    sh = Sheet("Arm", "1:1", f"Qty 4.  Carbon {fg(p.t)} mm.  Front right drawn; the others are mirror images", 6, total)
    v = as_drawn(sh, p.face, 16, 112)
    part_view(v, p, labels=False)
    draw_labels(v, p, fs=4.2)
    overall_dims(v, p.face, off=5, nose=False)
    sh.axes_glyph(232, 186, "+x  right", "+y  front")
    yb = sh.table(213, 172, ["", "x", "y", "Ø"], [r[:4] for r in hole_rows(p)], [8, 14, 14, 10], fs=5.0, row_h=2.9)
    sh.table(213, yb - 5, ["Slot", "x1", "y1", "x2", "y2", "w"], slot_rows(p), [8, 12, 12, 12, 12, 8], fs=4.6,
             row_h=2.7)
    motor = max(p.holes, key=lambda h: h.d)
    rr = [math.dist((motor.x, motor.y), (s.x1, s.y1)) for s in p.slots] + \
         [math.dist((motor.x, motor.y), (s.x2, s.y2)) for s in p.slots]
    sh.text(14, 40, f"Motor centre {motor.label}: x {f2(motor.x)}, y {f2(motor.y)}, Ø{f2(motor.d)}.  The motor slots "
            f"run from radius {min(rr):.2f} to {max(rr):.2f} about it: bolt circles Ø{2 * min(rr):.1f} to "
            f"Ø{2 * max(rr):.1f} (16x16 and 19x19 motors).", fs=5.3, color="#444")
    sh.text(14, 35.5, f"S3 (and S4-S6 on the mirrored arms) carries a standoff bolt.  The outline and the "
            f"{len(p.cutouts)} lightening cut-out{'s' if len(p.cutouts) != 1 else ''} come from dxf/arm.dxf.",
            fs=5.3, color="#444")
    pages.append(("6_arm", sh))

    # ---- 7. arm brace and camera plate -------------------------------------
    br, cam = P["arm_brace"], P["cam_plate"]
    sh = Sheet("Arm brace, camera plate", "1:1", f"Brace qty 2, {fg(br.t)} mm.  Camera plate qty 2, {fg(cam.t)} mm",
               7, total)
    v = nose_right(sh, br.face, 30, 168)
    part_view(v, br)
    ytop = overall_dims(v, br.face, off=5)
    sh.text(30, ytop + 8, "Arm brace (right), from above", fs=6.5, weight="bold")
    sh.table(14, 150, ["", "x", "y", "Ø"], [r[:4] for r in hole_rows(br)], [8, 14, 15, 10], fs=5.0, row_h=2.9)
    sh.axes_glyph(260, 150, "+y  front", "−x  left")
    v = as_drawn(sh, cam.face, 120, 78)
    part_view(v, cam)
    ytop = overall_dims(v, cam.face, off=5, nose=False, names=("y", "z"))
    sh.text(120, ytop + 8, "Camera plate (right), from the right", fs=6.5, weight="bold")
    yb = 108
    if cam.holes:
        yb = sh.table(190, yb, ["", "y", "z", "Ø"], [r[:4] for r in hole_rows(cam)], [8, 14, 14, 10], fs=5.0,
                      row_h=2.9) - 5
    if cam.slots:
        sh.table(190, yb, ["Slot", "y1", "z1", "y2", "z2", "w"], slot_rows(cam), [8, 12, 12, 12, 12, 9], fs=4.8,
                 row_h=2.8)
    sh.axes_glyph(14, 100, "+y  front", "+z  up")
    sh.text(14, 47, f"Camera plates: z {fg(cam.z2[0])} to {fg(cam.z2[1])}.  Their tabs pass through the slots in the "
            f"lower and top plates, flush with the outside faces.  The right one's faces are at x {cam.z[0]:.2f} and "
            f"{cam.z[1]:.2f}.", fs=5.3, color="#444")
    sh.text(14, 42.5, f"Brace: on top of the arms, z {fg(br.z[0])} to {fg(br.z[1])}; its holes line up with the arm's "
            f"H2 and H3.", fs=5.3, color="#444")
    pages.append(("7_brace_camera", sh))

    with PdfPages(out / "drawing.pdf") as pdf:
        for name, s in pages:
            s.save(pdf, out / "sheets" / f"{name}.png")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--step", type=Path, default=STEP)
    ap.add_argument("--out", type=Path, default=HERE)
    args = ap.parse_args(argv)
    shapes, so = extract(args.step)
    rows = check(so)
    export_dxf(args.out, so)
    export_csv(args.out)
    info = export_check(args.out, so, rows)
    sheets(args.out, so, rows, info)
    for p in PARTS:
        bb = p.face.BoundingBox()
        print(f"{p.key:12s} x{p.qty}  {bb.xlen:6.1f} x {bb.ylen:6.1f} x {p.t:4.2f} mm  "
              f"{len(p.holes):2d} holes, {len(p.slots):2d} slots, {len(p.cutouts):2d} cut-outs, {len(p.sinks)} csk faces")
    print(f"standoff alignment: worst {info[0]:.3f} mm, top-to-lower {info[1]:.3f} mm; wheelbase {info[2]:.1f} mm")


if __name__ == "__main__":
    main()
