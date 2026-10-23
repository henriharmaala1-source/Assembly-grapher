#!/usr/bin/env python3
"""A desk-size scale model of the single boom, to print: to check how the parts
go together, and to show the plane.

    python3 airframe/mini.py                      # 1:2
    python3 airframe/mini.py --scale 0.4 --fit 0.3
    python3 airframe/mini.py --no-slice           # skip the PrusaSlicer times

It builds the single boom (build.py --variant single-boom) at full size, then
again with its walls and trailing edges thickened so they are still two
perimeters after scaling. The wing, the battery and every part stay where the
full-size plane has them. Then it scales the result, and:

- drills the boom and spar holes again for stock rods (stock diameter + --fit):
  scaled, the holes would be the scaled rod plus half the full-size clearance,
  too tight to print;
- turns the bought parts (servos, motor, prop, flight controller, ESC, battery,
  camera, VTX, receiver, antenna) into printed dummies, one solid each, a
  little smaller than the full-size parts so they drop into their scaled
  pockets;
- adds a stand that holds the plane under its balance point.

Outputs (variants/single-boom-mini/):
    stl/            the printed parts, dummy_*.stl and stand.stl, each on the bed
    cad/mini.step   the model assembled on its stand
    viewer/mini.glb the same, for the viewer
    REPORT.md       rods to cut, the print list with PrusaSlicer times, the checks
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import replace
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import Vector as V
from OCP.BRepBuilderAPI import BRepBuilderAPI_GTransform
from OCP.gp import gp_GTrsf, gp_Mat, gp_XYZ

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import build as B  # noqa: E402
from geom import box, cut, cyl_x, cyl_y, cyl_z, fuse  # noqa: E402

OUT = HERE / "variants" / "single-boom-mini"
MIN_WALL = 0.8                    # two 0.4 mm lines
STOCK = (0.5, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)   # common rod and tube diameters

# The bought parts that become dummies, and the sub-bodies left off them: too thin to print.
DUMMIES = ("servo_ail_R", "servo_ail_L", "servo_elev", "servo_rud", "motor", "prop", "camera", "vtx",
           "antenna", "fc", "receiver", "esc", "battery")
DROP = ("wires", "leads", "pads")
# Dummies that would be too thin to print at scale, thickened about a point of their bounding box:
# {name: ((fx, fy, fz), anchor)}, anchor "min" keeps the face nearest the motor, "centre" the axis.
THICKEN = {"prop": ((1.6, 1.0, 1.0), "min"), "antenna": ((1.6, 1.6, 1.0), "centre")}
ROD_KIND = {"boom": "carbon tube (or an aluminium tube or wooden dowel)",
            "spar_main": "carbon rod (or steel wire)", "spar_rear": "carbon rod (or steel wire, a paper clip)"}
COLOURS = {"printed": None, "dummy": None, "stock": (0.08, 0.08, 0.09), "stand": (0.45, 0.47, 0.50)}


# --------------------------------------------------------------------------
# Geometry helpers


def gscale(shape, f, c):
    """Scale by (fx, fy, fz) about the point c."""
    fx, fy, fz = f
    g = gp_GTrsf()
    g.SetVectorialPart(gp_Mat(fx, 0, 0, 0, fy, 0, 0, 0, fz))
    g.SetTranslationPart(gp_XYZ(c[0] * (1 - fx), c[1] * (1 - fy), c[2] * (1 - fz)))
    return cq.Shape.cast(BRepBuilderAPI_GTransform(shape.wrapped, g, True).Shape())


def overlap(a, b) -> bool:
    """Do two bounding boxes overlap?"""
    return not (a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin
                or a.zmax < b.zmin or b.zmax < a.zmin)


def nearest_stock(d: float) -> float:
    return min(STOCK, key=lambda s: abs(s - d))


def rod_axis(shape):
    """A straight rod's axis (0 x, 1 y, 2 z), its centre line and its extent along the axis."""
    bb = shape.BoundingBox()
    lo, hi = np.array([bb.xmin, bb.ymin, bb.zmin]), np.array([bb.xmax, bb.ymax, bb.zmax])
    ax = int(np.argmax(hi - lo))
    return ax, (lo + hi) / 2, lo[ax], hi[ax]


def cylinder(ax, r, a0, a1, c):
    return (cyl_x(r, a0, a1, c[1], c[2]), cyl_y(r, a0, a1, c[0], c[2]), cyl_z(r, a0, a1, c[0], c[1]))[ax]


def lay_flat(s):
    """Turn a dummy so its thinnest dimension is vertical, then put it on the bed."""
    bb = s.BoundingBox()
    d = (bb.xlen, bb.ylen, bb.zlen)
    if d.index(min(d)) == 0:
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), 90)
    elif d.index(min(d)) == 1:
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    bb = s.BoundingBox()
    return s.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))


def largest_solid(s):
    solids = s.Solids()
    return max(solids, key=lambda x: x.Volume()) if len(solids) > 1 else s


# --------------------------------------------------------------------------
# The model


def full_size(scale: float):
    """The single boom as designed, then rebuilt with the thin features thickened for the scale,
    at the same wing position and battery station."""
    p0 = B.variant_params("single-boom")
    p0, k, L0, *_, batt_x = B.build(p0)
    t = MIN_WALL / scale
    p = replace(p0, wall=max(p0.wall, t), te_min=max(p0.te_min, t), boss_wall=max(p0.boss_wall, t),
                plate=max(p0.plate, 1.0 / scale))
    fairing_wall = B.FAIRING_WALL
    B.FAIRING_WALL = max(fairing_wall, t)               # the servo fairings' shell
    try:
        p, k, L, sec, parts, refs, items, batt_x = B.build_at(p, k, L0.x_le, batt_x)
    finally:
        B.FAIRING_WALL = fairing_wall
    return p0, p, k, L, parts, refs, batt_x


DIRS = [V(i, j, k).normalized() for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]


def fitted(sh, parts, gap):
    """Trim a dummy wherever it comes within `gap` of a printed part: cut it with the part's
    material near it, moved `gap` in each of 26 directions (the faces, edges and corners of a cube),
    which is close to the part grown by `gap` all round."""
    bb = sh.BoundingBox()
    zone = box(bb.xmin - 1, bb.xmax + 1, bb.ymin - 1, bb.ymax + 1, bb.zmin - 1, bb.zmax + 1)
    for p in parts.values():
        if not overlap(p.BoundingBox().enlarge(gap + 0.5), bb) or sh.distance(p) >= gap:
            continue
        local = p.intersect(zone)
        if local.Volume() < 1e-6:
            continue
        for v in DIRS:
            sh = largest_solid(cut(sh, local.translate(v * gap)))
    return sh


def nearest(sh, parts) -> float:
    """The smallest distance from sh to a printed part near it (mm)."""
    bb = sh.BoundingBox()
    near = [p for p in parts.values() if overlap(p.BoundingBox().enlarge(2.0), bb)]
    return min((sh.distance(p) for p in near), default=float("inf"))


def make_model(scale: float, fit: float, gap: float, rods: dict):
    p0, p, k, L, parts_full, refs, batt_x = full_size(scale)
    s = scale
    parts = {n: sh.scale(s) for n, sh in parts_full.items()}

    # stock rods, and their holes drilled again for them
    stock, removed = {}, {}
    for r in refs:
        if r.name not in ROD_KIND:
            continue
        sh = r.shape.scale(s)
        ax, c, a0, a1 = rod_axis(sh)
        scaled_d = {"boom": p.boom_od, "spar_main": p.main_spar_d, "spar_rear": p.rear_spar_d}[r.name] * s
        d = rods.get(r.name) or nearest_stock(scaled_d)
        drill = cylinder(ax, (d + fit) / 2, a0, a1, c)
        bd = drill.BoundingBox()
        for n, part in parts.items():
            if not overlap(part.BoundingBox(), bd):
                continue
            v0 = part.Volume()
            parts[n] = largest_solid(cut(part, drill))
            if v0 - parts[n].Volume() > 1e-3:
                removed.setdefault(r.name, []).append(n)
        body = cylinder(ax, d / 2, a0, a1, c)
        if r.name == "boom" and d >= 3.0:                 # a tube: 0.5 mm wall for the look
            body = cut(body, cylinder(ax, d / 2 - 0.5, a0 - 1, a1 + 1, c))
        stock[r.name] = {"shape": body, "d": d, "scaled_d": round(scaled_d, 2), "length": round(a1 - a0, 1),
                         "hole": round(d + fit, 2), "in": removed.get(r.name, [])}

    # dummies of the bought parts
    by_comp = {}
    for r in refs:
        comp = B.component(r.name)
        if comp in DUMMIES and r.name.rpartition("_")[2] not in DROP:
            by_comp.setdefault(comp, []).append(r)
    dummies = {}
    for name in DUMMIES:
        rs = by_comp[name]
        sh = largest_solid(fuse(*[r.shape for r in rs]).clean()).scale(s)
        bb = sh.BoundingBox()
        full = [round(d / s, 1) for d in (bb.xlen, bb.ylen, bb.zlen)]
        if name in THICKEN:
            f, anchor = THICKEN[name]
            c = (bb.xmin, bb.center.y, bb.center.z) if anchor == "min" else (bb.center.x, bb.center.y, bb.center.z)
            sh = gscale(sh, f, c)
        main = next(r for r in rs if r.name == name)
        v0 = sh.Volume()
        d = sh if name in THICKEN else fitted(sh, parts, gap)
        dummies[name] = {"shape": d, "colour": main.colour, "full": full,
                         "trimmed": round(100 * (1 - d.Volume() / v0), 1)}

    clear_thickened(parts, dummies)
    stand = make_stand(p, L, s, parts, dummies, stock)
    return p0, p, k, L, batt_x, parts, dummies, stock, stand


def clear_thickened(parts, dummies, by=0.2):
    """The thickened dummies no longer fit the holes the full-size parts have for them: the antenna
    in the lid, and the prop, thicker aft of its hub, runs into the motor's prop nut. Cut a recess
    round each overlap (its bounding box and `by` more): into the lid, and into the prop's hub."""
    for _ in range(3):
        refs = [B.Ref(f"dummy_{n}", d["shape"], d["colour"], 0, "electronics") for n, d in dummies.items()]
        hits = [h for h in B.interference(parts, refs, tol=0.0)
                if h[1].removeprefix("dummy_") in THICKEN or h[0].removeprefix("dummy_") in THICKEN]
        if not hits:
            return
        for a, b, _ in hits:
            sa = parts[a] if a in parts else dummies[a.removeprefix("dummy_")]["shape"]
            sb = dummies[b.removeprefix("dummy_")]["shape"]
            r = sa.intersect(sb).BoundingBox()
            tool = box(r.xmin - by, r.xmax + by, r.ymin - by, r.ymax + by, r.zmin - by, r.zmax + by)
            if a in parts:                                  # the part makes room for the thicker dummy
                parts[a] = largest_solid(cut(parts[a], tool))
            else:                                           # the thicker dummy makes room for the other
                n = b if b.removeprefix("dummy_") in THICKEN else a
                d = dummies[n.removeprefix("dummy_")]
                d["shape"] = largest_solid(cut(d["shape"], tool))


def make_stand(p, L, s, parts, dummies, stock):
    """A base, a post and a cradle under the fuselage at the balance point. The cradle is cut
    by the fuselage's outer section, so the belly and its chamfers sit in it."""
    x_c, hw, zb = L.cg_x * s, L.half_w * s, L.z_bottom * s
    gap = 0.15
    env = B.section_solid(L.half_w + gap / s, L.z_bottom - gap / s, L.z_top, p.top_r, p.belly_ch,
                          L.cg_x - 40, L.cg_x + 40).scale(s)
    cradle = cut(box(x_c - 9, x_c + 9, -(hw + 2.4), hw + 2.4, zb - 3.0, zb + 5.0), env)
    lowest = min(sh.BoundingBox().zmin for sh in [*parts.values(), *(d["shape"] for d in dummies.values())])
    z_base = min(lowest, zb - 3.0) - 12.0            # the base's top, under the prop's lowest tip
    post = cyl_z(5.0, z_base - 1.0, zb - 2.5, x_c, 0.0)
    base = (cq.Workplane().box(110, 84, 4).edges("|Z").fillet(20).edges(">Z").fillet(1.2)
            .translate((x_c, 0, z_base - 2.0)).val())
    try:                                              # the name on the base, engraved
        label = (cq.Workplane().text(f"Kipinä  1:{1 / s:g}", 7.5, -0.6, halign="center", valign="center",
                                     kind="bold").translate((x_c, -28.0, z_base)).val())
        base = cut(base, label)
    except Exception as e:                            # no font: a plain base
        print("stand: no label:", e)
    return fuse(base, post, cradle).clean()


# --------------------------------------------------------------------------
# Checks


def check_fits(parts, dummies, stock, stand):
    """Overlaps between the printed parts and the dummies, stock rods and stand (mm^3)."""
    refs = [B.Ref(f"dummy_{n}", d["shape"], d["colour"], 0, "electronics") for n, d in dummies.items()]
    refs += [B.Ref(f"rod_{n}", r["shape"], COLOURS["stock"], 0, "hardware") for n, r in stock.items()]
    refs.append(B.Ref("stand", stand, COLOURS["stand"], 0, "hardware"))
    return B.interference(parts, refs, tol=0.01)   # dummies against each other too


def clearances(parts, dummies):
    """For each dummy, its smallest distance to a printed part (mm)."""
    return {n: round(nearest(d["shape"], parts), 2) for n, d in dummies.items()}


# --------------------------------------------------------------------------
# Printing


SLICE_BASE = ["--layer-height", "0.12", "--first-layer-height", "0.2", "--perimeters", "2",
              "--top-solid-layers", "5", "--bottom-solid-layers", "4", "--fill-density", "15%",
              "--filament-max-volumetric-speed", "12", "--filament-density", "1.24"]


def slicer_options(name: str) -> list[str]:
    opts = list(SLICE_BASE)
    if name.startswith(("wing_R", "wing_L", "fairing")):   # standing on the root rib; a thin rim
        opts += ["--brim-width", "5"]
    if name.startswith("dummy_"):
        opts += ["--support-material", "--support-material-auto", "--support-material-threshold", "40"]
    return opts


def slice_one(exe, ini, stl: Path, opts, tmp: Path):
    out = tmp / (stl.stem + ".gcode")
    subprocess.run([exe, "--export-gcode", "--load", str(ini), *opts, "--output", str(out), str(stl)],
                   check=True, capture_output=True, timeout=900)
    g = out.read_text(errors="ignore")
    t = re.search(r"estimated printing time \(normal mode\) = (.+)", g).group(1)
    return {"seconds": sum(int(v) * {"d": 86400, "h": 3600, "m": 60, "s": 1}[u] for v, u in re.findall(r"(\d+)([dhms])", t)),
            "grams": float(re.search(r"filament used \[g\] = ([\d.]+)", g).group(1)),
            "cm3": float(re.search(r"filament used \[cm3\] = ([\d.]+)", g).group(1))}


def slice_all(stls: dict, volumes: dict):
    """Times and grams with the print settings, and how much of each printed part a solid
    print fills (thin features the slicer drops show up as a fill well under 100 %)."""
    exe = shutil.which("prusa-slicer")
    if not exe:
        print("no prusa-slicer: no print times")
        return {}
    ini = HERE / "tools" / "a1mini.ini"
    res = {}
    with tempfile.TemporaryDirectory() as tmp:
        for name, stl in stls.items():
            try:
                r = slice_one(exe, ini, stl, slicer_options(name), Path(tmp))
            except subprocess.CalledProcessError as e:
                print(f"slicing {name} failed:", e.stdout.decode(errors="ignore").strip().splitlines()[-2:])
                continue
            if not name.startswith("dummy_") and name != "stand":
                solid = [o if o != "15%" else "100%" for o in SLICE_BASE] + ["--fill-pattern", "rectilinear"]
                r["fill"] = round(slice_one(exe, ini, stl, solid, Path(tmp))["cm3"] * 1000 / volumes[name], 3)
            res[name] = r
            print(f"sliced {name:22s} {r['seconds'] / 60:6.1f} min {r['grams']:6.1f} g", r.get("fill", ""))
    return res


# --------------------------------------------------------------------------
# Export and report


ORIENT_NOTES = {
    "wing_centre": "On its side, spar holes vertical.",
    "wing_R": "Standing on the root rib, brim.", "wing_L": "Standing on the root rib, brim.",
    "aileron_R": "Flat on the lower surface.", "aileron_L": "Flat on the lower surface.",
    "fuselage_front": "Upright, open top up.", "fuselage_rear": "Standing on its front face.",
    "lid": "Flat.", "tail_mount": "Front face down.", "stab": "Flat.", "elevator": "Flat.",
    "fin": "Flat.", "rudder": "Flat, horn up.", "motor_mount": "Plate down.",
    "fairing_ail_R": "Rim down.", "fairing_ail_L": "Rim down.",
}
COLOUR_NAMES = {"fuselage_front": "dark grey", "fuselage_rear": "dark grey", "lid": "grey",
                "tail_mount": "dark grey", "motor_mount": "dark grey"}


def colour_name(name):
    if name in COLOUR_NAMES:
        return COLOUR_NAMES[name]
    return "orange" if name.startswith(("aileron", "elevator", "rudder")) else "white"


def bodies(parts, dummies, stock, stand):
    out = [(n, sh, B.PRINT[n][3], "printed") for n, sh in parts.items()]
    out += [(f"dummy_{n}", d["shape"], d["colour"], "dummies") for n, d in dummies.items()]
    out += [(f"rod_{n}", r["shape"], COLOURS["stock"], "stock") for n, r in stock.items()]
    out.append(("stand", stand, COLOURS["stand"], "display"))
    return out


def export(p, parts, dummies, stock, stand, out: Path):
    (out / "stl").mkdir(parents=True, exist_ok=True)
    for f in (out / "stl").glob("*.stl"):
        f.unlink()
    stls, volumes, beds = {}, {}, {}
    for n, sh in parts.items():
        stls[n], volumes[n] = out / "stl" / f"{n}.stl", sh.Volume()
        bed = B.to_bed(sh, n, p)
        beds[n] = bed.BoundingBox()
        cq.exporters.export(bed, str(stls[n]), tolerance=0.01, angularTolerance=0.1)
    for n, d in dummies.items():
        k = f"dummy_{n}"
        stls[k], volumes[k] = out / "stl" / f"{k}.stl", d["shape"].Volume()
        bed = lay_flat(d["shape"])
        beds[k] = bed.BoundingBox()
        cq.exporters.export(bed, str(stls[k]), tolerance=0.01, angularTolerance=0.1)
    stls["stand"], volumes["stand"] = out / "stl" / "stand.stl", stand.Volume()
    bb = stand.BoundingBox()
    stand_bed = stand.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))
    beds["stand"] = stand_bed.BoundingBox()
    cq.exporters.export(stand_bed, str(stls["stand"]), tolerance=0.01, angularTolerance=0.1)

    everything = bodies(parts, dummies, stock, stand)
    assy = cq.Assembly(name="kipina_mini")
    groups = {g: cq.Assembly(name=g) for g in ("printed", "dummies", "stock", "display")}
    for name, sh, colour, group in everything:
        groups[group].add(sh, name=name, color=cq.Color(*colour))
    for g in groups.values():
        assy.add(g)
    (out / "cad").mkdir(exist_ok=True)
    assy.export(str(out / "cad" / "mini.step"))

    import trimesh
    from trimesh.visual.material import PBRMaterial
    scene = trimesh.Scene()
    for name, sh, colour, group in everything:
        v, t = B.tessellate(sh, 0.02, 0.2)
        v = np.column_stack([v[:, 0], v[:, 2], -v[:, 1]]) / 1000.0     # z-up mm -> y-up m
        mesh = trimesh.Trimesh(v, t, process=False)
        mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=name, baseColorFactor=[*B.srgb_to_linear(colour), 1.0], metallicFactor=0.0, roughnessFactor=0.6))
        scene.add_geometry(mesh, node_name=name, geom_name=name, metadata={"group": group})
    (out / "viewer").mkdir(exist_ok=True)
    scene.export(str(out / "viewer" / "mini.glb"))
    return stls, volumes, beds


def fmt_min(sec):
    h, m = divmod(round(sec / 60), 60)
    return f"{h} h {m:02d} min" if h else f"{m} min"


def write_report(args, p0, p, L, batt_x, parts, dummies, stock, stand, beds, sliced, hits, gaps, out: Path):
    s = args.scale
    every = bodies(parts, dummies, stock, stand)
    bb = cq.Compound.makeCompound([sh for n, sh, *_ in every if n != "stand"]).BoundingBox()
    sb = stand.BoundingBox()
    fits = {n: all(d <= m for d, m in zip(sorted((b.xlen, b.ylen, b.zlen)), sorted(p.bed))) for n, b in beds.items()}
    total_t = sum(r["seconds"] for r in sliced.values())
    total_g = sum(r["grams"] for r in sliced.values())
    thin = {n: r["fill"] for n, r in sliced.items() if "fill" in r and r["fill"] < 0.9}
    L_ = []
    a = L_.append
    a(f"# Kipinä single boom, 1:{1 / s:g} desk model: build report\n")
    a(f"Generated by `mini.py --scale {s:g} --fit {args.fit:g}` from the single boom's CAD. Do not edit by hand.\n")
    a("A scale model to print: to check how the parts go together before printing the full-size plane, "
      "and to show it. It doesn't fly. The parts are the full-size plane's, split the same way and scaled; "
      "the bought parts are printed dummies and the rods are stock rods cut to length.\n")
    a("## Key numbers\n")
    a("| | full size | model |")
    a("|---|---|---|")
    a(f"| Scale | 1:1 | **1:{1 / s:g}** |")
    a(f"| Wingspan | {p0.span:.0f} mm | {p0.span * s:.1f} mm |")
    a(f"| Length (nose to prop) | {bb.xlen / s:.0f} mm | {bb.xlen:.1f} mm |")
    a(f"| Height | {bb.zlen / s:.0f} mm | {bb.zlen:.1f} mm (on the stand {sb.zmax - sb.zmin + (bb.zmax - sb.zmax):.0f} mm) |")
    a(f"| Fuselage walls | {p0.wall:.1f} mm | {p.wall * s:.2f} mm (thickened inwards before scaling) |")
    a(f"| Servo fairing shells | {B.FAIRING_WALL:.2f} mm | {max(B.FAIRING_WALL, MIN_WALL / s) * s:.2f} mm |")
    a(f"| Trailing edges | {p0.te_min:.1f} mm | {p.te_min * s:.2f} mm |")
    a(f"| Tail plates | {p0.plate:.1f} mm | {p.plate * s:.2f} mm |")
    a(f"| Printed parts | {len(parts)} | {len(parts)} + {len(dummies)} dummies + the stand |")
    if sliced:
        a(f"| Print time, all of it | | **{fmt_min(total_t)}**, {total_g:.0f} g of PLA (PrusaSlicer, A1 mini profile) |")
    a("")
    a(f"The wing sits where it does on the full-size plane (leading edge {L.x_le:.1f} mm from the nose, "
      f"{L.x_le * s:.1f} mm on the model) and the battery at its full-size station ({batt_x:.1f} mm, "
      f"{batt_x * s:.1f} mm). Only the walls, the trailing edges and the boom socket's sleeve are thicker, "
      "inwards, so the outside is the full-size plane's shape.\n")

    a("## Rods to cut\n")
    a("The holes are drilled for these stock sizes plus the fit clearance, not scaled.\n")
    a("| rod | full size | scaled | use | length | hole | through |")
    a("|---|---|---|---|---|---|---|")
    full_d = {"boom": f"{p0.boom_od:g} x {p0.boom_id:g} mm tube", "spar_main": f"{p0.main_spar_d:g} mm rod",
              "spar_rear": f"{p0.rear_spar_d:g} mm rod"}
    for n, r in stock.items():
        a(f"| {n} | {full_d[n]} | {r['scaled_d']:g} mm | **{r['d']:g} mm** {ROD_KIND[n]} | **{r['length']:.0f} mm** | "
          f"{r['hole']:g} mm | {', '.join(r['in'])} |")
    a("")
    a("The pushrods, the control horns' screws, the hinge tape and the wiring are left off; glue the control "
      "surfaces at neutral, or hinge them with tape.\n")

    a("## Print list\n")
    a(f"Build volume checked: {p.bed[0]:.0f} x {p.bed[1]:.0f} x {p.bed[2]:.0f} mm (Bambu Lab A1 mini). "
      "PLA, 0.12 mm layers, 2 walls, 15 % infill, 5 top and 4 bottom layers; dummies with supports where the "
      "slicer wants them.\n")
    a("| file | colour | on the bed, mm | fits | time | g | orientation |")
    a("|---|---|---|---|---|---|---|")
    for n in [*parts, *(f"dummy_{d}" for d in dummies), "stand"]:
        b = beds[n]
        r = sliced.get(n, {})
        col = colour_name(n) if n in parts else ("as you like" if n.startswith("dummy_") else "grey")
        note = ORIENT_NOTES.get(n, "Thinnest side down." if n.startswith("dummy_") else "Base down.")
        t = fmt_min(r["seconds"]) if r else "-"
        g = f"{r['grams']:.1f}" if r else "-"
        a(f"| stl/{n}.stl | {col} | {b.xlen:.0f} x {b.ylen:.0f} x {b.zlen:.0f} | {'yes' if fits[n] else '**no**'} | "
          f"{t} | {g} | {note} |")
    a("")

    a("## Dummies\n")
    a("One solid each, from the full-size models without their wires and solder pads, scaled, then trimmed "
      f"wherever it comes within {args.gap:g} mm of a printed part (cut by the part's material near it, moved "
      f"{args.gap:g} mm in 26 directions), so it drops into its pocket without forcing. The prop is 1.6 times "
      "thicker along the shaft (aft of its hub, with a recess for the prop nut) and the antenna 1.6 times "
      "thicker (the lid's hole is opened up for it), so they print.\n")
    a("| dummy | full-size part, mm | trimmed off | gap to the nearest printed part |")
    a("|---|---|---|---|")
    for n, d in dummies.items():
        g = "-" if gaps[n] == float("inf") else f"{gaps[n]:.2f} mm"
        a(f"| dummy_{n} | {' x '.join(f'{v:g}' for v in d['full'])} | {d['trimmed']:g} % | {g} |")
    a("")

    a("## Checks\n")
    if hits:
        a("Overlaps (printed parts against the dummies, the stock rods and the stand):\n")
        for x, y, v in hits:
            a(f"- {x} and {y}: {v} mm^3")
    else:
        a("Interference: the dummies, the stock rods and the stand overlap no printed part.")
    a("")
    if sliced:
        if thin:
            a("Thin features: printed solid, these parts fill less than 90 % of their volume, so the slicer "
              "drops some of their detail:\n")
            for n, f in thin.items():
                a(f"- {n}: {f * 100:.0f} %")
        else:
            a("Thin features: printed solid, every part fills at least 90 % of its volume in PrusaSlicer, so "
              "nothing is too thin to print.")
        a("")
        a("| part | solid fill |")
        a("|---|---|")
        for n, r in sliced.items():
            if "fill" in r:
                a(f"| {n} | {r['fill'] * 100:.0f} % |")
        a("")
    a(f"Every file fits the build volume: {'yes' if all(fits.values()) else 'no'}.\n")

    a("## Assembly\n")
    a("In the full-size plane's order ([BOP.md](../single-boom/BOP.md)), with glue in place of the screws "
      "and velcro:\n")
    a("1. Glue the two fuselage halves. Slide the boom into its socket and glue it; glue the motor mount on "
      "its end and the motor dummy on the mount, then the prop.")
    a("2. Slide the tail mount on the boom and glue the stabiliser under it and the fin into its socket. "
      "Glue the elevator and the rudder at neutral.")
    a("3. Drop the servo, ESC, FC, receiver, VTX and camera dummies into their places in the fuselage; the "
      "battery sits in its bay; put on the lid and the antenna.")
    a("4. Push the main and rear spars through the wing centre and both panels. Put the aileron servo "
      "dummies in their pockets, glue the fairings over them and the ailerons on.")
    a("5. Seat the wing on the fuselage and stand the plane in the stand's cradle.")
    a("")
    a("What it shows: every printed part's fit against its neighbours, the spar and boom paths, the pockets "
      "and bays for the bought parts, and the plane's look. What it can't show: the wiring (left off), the "
      "pushrods' travel, the full-size print tolerances (the clearances here are re-cut for the model), "
      "and the strength.\n")
    (out / "REPORT.md").write_text("\n".join(L_))
    return bb


def hex_colour(rgb):
    return "#" + "".join(f"{round(c * 255):02x}" for c in rgb)


def write_spec(args, p0, parts, dummies, stock, stand, beds, sliced, hits, gaps, bb, out: Path):
    """viewer/spec.json: the numbers the viewer page shows."""
    s = args.scale
    sb = stand.BoundingBox()
    prints = []
    for n in [*parts, *(f"dummy_{d}" for d in dummies), "stand"]:
        group = "printed" if n in parts else "dummy" if n.startswith("dummy_") else "stand"
        rgb = B.PRINT[n][3] if n in parts else dummies[n[6:]]["colour"] if group == "dummy" else COLOURS["stand"]
        r = sliced.get(n, {})
        b = beds[n]
        prints.append({"file": f"{n}.stl", "group": group, "swatch": hex_colour(rgb),
                       "colour": colour_name(n) if n in parts else "any" if group == "dummy" else "grey",
                       "bed": [round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)],
                       "minutes": round(r["seconds"] / 60, 1) if r else None, "grams": r.get("grams")})
    full = {"boom": f"{p0.boom_od:g} x {p0.boom_id:g} mm carbon tube", "spar_main": f"{p0.main_spar_d:g} mm carbon rod",
            "spar_rear": f"{p0.rear_spar_d:g} mm carbon rod"}
    names = {"boom": "Boom", "spar_main": "Main spar", "spar_rear": "Rear spar"}
    fills = {n: r["fill"] for n, r in sliced.items() if "fill" in r}
    spec = {
        "ratio": f"{1 / s:g}", "scale": s, "gap": args.gap, "fit": args.fit,
        "span": p0.span * s, "length": bb.xlen, "height_on_stand": bb.zmax - sb.zmin,
        "minutes": round(sum(r["seconds"] for r in sliced.values()) / 60, 1) if sliced else None,
        "grams": round(sum(r["grams"] for r in sliced.values()), 1) if sliced else None,
        "prints": prints,
        "rods": [{"name": n, "label": names[n], "full": full[n], "d": r["d"], "kind": ROD_KIND[n], "length": r["length"],
                  "hole": r["hole"], "through": r["in"]} for n, r in stock.items()],
        "dummies": [{"name": n, "full": d["full"], "trimmed": d["trimmed"],
                     "gap": None if gaps[n] == float("inf") else gaps[n]} for n, d in dummies.items()],
        "checks": {"hits": hits, "thin": {n: f for n, f in fills.items() if f < 0.9},
                   "thin_ok": (all(f >= 0.9 for f in fills.values()) if fills else None),
                   "thin_min": list(min(fills.items(), key=lambda kv: kv[1])) if fills else None,
                   "fits": all(all(d <= m for d, m in zip(sorted((b.xlen, b.ylen, b.zlen)), sorted(p0.bed)))
                               for b in beds.values())},
    }
    (out / "viewer" / "spec.json").write_text(json.dumps(spec, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--scale", type=float, default=0.5, help="model size / full size (default 0.5, 1:2)")
    ap.add_argument("--fit", type=float, default=0.25, help="hole oversize for the stock rods, mm")
    ap.add_argument("--gap", type=float, default=0.15, help="the dummies' smallest gap to a printed part, mm")
    ap.add_argument("--boom", type=float, help="boom tube diameter, mm (default: the nearest stock size)")
    ap.add_argument("--spar", type=float, help="main spar diameter, mm")
    ap.add_argument("--rear-spar", type=float, help="rear spar diameter, mm")
    ap.add_argument("--no-slice", action="store_true", help="skip PrusaSlicer")
    args = ap.parse_args(argv)
    rods = {"boom": args.boom, "spar_main": args.spar, "spar_rear": args.rear_spar}
    p0, p, k, L, batt_x, parts, dummies, stock, stand = make_model(args.scale, args.fit, args.gap, rods)
    for n, sh in [*parts.items(), *((f"dummy_{n}", d["shape"]) for n, d in dummies.items()), ("stand", stand)]:
        assert sh.isValid(), f"{n} is not a valid solid"
    hits = check_fits(parts, dummies, stock, stand)
    for a, b, v in hits:
        print(f"WARNING: {a} overlaps {b} by {v} mm^3")
    gaps = clearances(parts, dummies)
    stls, volumes, beds = export(p, parts, dummies, stock, stand, OUT)
    sliced = {} if args.no_slice else slice_all(stls, volumes)
    (OUT / "checks.json").write_text(json.dumps({"hits": hits, "gaps": gaps, "sliced": sliced}, indent=1))
    bb = write_report(args, p0, p, L, batt_x, parts, dummies, stock, stand, beds, sliced, hits, gaps, OUT)
    write_spec(args, p0, parts, dummies, stock, stand, beds, sliced, hits, gaps, bb, OUT)
    print(f"1:{1 / args.scale:g}: span {p0.span * args.scale:.1f} mm, length {bb.xlen:.1f} mm; wrote {OUT}")


if __name__ == "__main__":
    main()
