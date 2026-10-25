#!/usr/bin/env python3
"""A desk-size scale model of the single boom, to print: to check how the parts
go together, and to show the plane.

    python3 airframe/mini.py                      # 1:2
    python3 airframe/mini.py --fit 0.4 --gap 0.3     # after the fit test, if the rods were tight
    python3 airframe/mini.py --stock-rods            # bought carbon rods instead of printed ones
    python3 airframe/mini.py --no-slice           # skip the PrusaSlicer times

It builds the single boom (build.py --variant single-boom) at full size, then
again with its walls and trailing edges thickened so they are still two
perimeters after scaling. The wing, the battery and every part stay where the
full-size plane has them. Then it scales the result, and:

- replaces the carbon boom and spars with printed rods (or, with --stock-rods,
  bought rods of the nearest stock size) and drills their holes again for them
  (rod + --fit): scaled, the holes would be the scaled rod plus half the
  full-size clearance, too tight to print. Each spar is printed in two halves
  that meet in the wing centre, so they fit the bed;
- opens the slip fits between printed parts (the lid in the fuselage, the fin in
  the tail mount, the fuselage halves' joint) to --gap, since scaling halves
  the full-size clearances;
- turns the bought parts (servos, motor, prop, flight controller, ESC, battery,
  camera, VTX, receiver, antenna) into printed dummies, one solid each, a
  little smaller than the full-size parts so they drop into their scaled
  pockets;
- adds a stand that holds the plane under its balance point, and a fit test:
  a small plate with holes for each rod at three clearances and slots for the
  slip fits, to print first.

Outputs (variants/single-boom-mini/):
    stl/            the printed parts, rod_*.stl, dummy_*.stl, stand.stl and fit_test.stl, on the bed
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
# Printed rods: as thick as the wing allows with 0.8 mm of wall round the hole (the main spar's axis
# is 2.7 mm from the skin, the rear spar's 1.9 mm, at 1:2). The boom fills its socket: the socket's
# scaled bore less the fit (a smaller drill can't make a hole smaller).
PRINTED_RODS = {"spar_main": 3.0, "spar_rear": 1.8}
ROD_KIND = {"boom": "carbon tube (or an aluminium tube or wooden dowel)",
            "spar_main": "carbon rod (or steel wire)", "spar_rear": "carbon rod (or steel wire, a paper clip)"}
ROD_NAMES = {"boom": "Boom", "spar_main": "Main spar", "spar_rear": "Rear spar"}
# Slip fits between printed parts: {part that is trimmed: the parts it goes into}
SLIP = {"lid": ("fuselage_front", "fuselage_rear"), "fin": ("tail_mount",), "fuselage_front": ("fuselage_rear",)}
COLOURS = {"stock": (0.08, 0.08, 0.09), "rod": (0.12, 0.12, 0.13), "stand": (0.45, 0.47, 0.50),
           "fit_test": (0.45, 0.47, 0.50)}


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
                collar_wall=max(p0.collar_wall, t), plate=max(p0.plate, 1.0 / scale))
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


def printed_rod(ax, d, a0, a1, c):
    """A printed rod along axis ax from a0 to a1: round, with a flat 10 % of its diameter deep along
    its underside to print on (it lies flat on the bed)."""
    rod = cylinder(ax, d / 2, a0, a1, c)
    f = max(0.2, 0.1 * d)
    return cut(rod, box(-1e3, 1e3, -1e3, 1e3, c[2] - d / 2 - 1, c[2] - d / 2 + f))


def make_model(scale: float, fit: float, gap: float, rods: dict, stock_rods: bool = False):
    p0, p, k, L, parts_full, refs, batt_x = full_size(scale)
    s = scale
    parts = {n: sh.scale(s) for n, sh in parts_full.items()}

    # the rods, and their holes drilled again for them
    stock = {}
    for r in refs:
        if r.name not in ROD_KIND:
            continue
        sh = r.shape.scale(s)
        ax, c, a0, a1 = rod_axis(sh)
        scaled_d = {"boom": p.boom_od, "spar_main": p.main_spar_d, "spar_rear": p.rear_spar_d}[r.name] * s
        bore = 2 * L.r_hole * s if r.name == "boom" else 0.0       # the boom socket's scaled bore
        d = rods.get(r.name) or (nearest_stock(scaled_d) if stock_rods else
                                 round(bore - fit, 2) if r.name == "boom" else PRINTED_RODS[r.name])
        drill = cylinder(ax, (d + fit) / 2, a0, a1, c)
        bd, through = drill.BoundingBox(), []
        sleeve = cylinder(ax, (d + fit) / 2 + 0.5, a0, a1, c)         # what surrounds the hole
        for n, part in parts.items():
            if not overlap(part.BoundingBox(), bd):
                continue
            parts[n] = largest_solid(cut(part, drill))
            if parts[n].intersect(sleeve).Volume() > 1e-3:
                through.append(n)
        if stock_rods:
            body = cylinder(ax, d / 2, a0, a1, c)
            if r.name == "boom" and d >= 3.0:             # a tube: 0.5 mm wall for the look
                body = cut(body, cylinder(ax, d / 2 - 0.5, a0 - 1, a1 + 1, c))
            pieces = {f"rod_{r.name}": body}
        elif ax == 1:                                     # a spar: two halves that meet at the centre line
            pieces = {f"rod_{r.name}_R": printed_rod(ax, d, 0.1, a1, c), f"rod_{r.name}_L": printed_rod(ax, d, a0, -0.1, c)}
        else:
            pieces = {f"rod_{r.name}": printed_rod(ax, d, a0, a1, c)}
        stock[r.name] = {"pieces": pieces, "d": d, "scaled_d": round(scaled_d, 2), "length": round(a1 - a0, 1),
                         "hole": round(max(d + fit, bore), 2), "in": through, "printed": not stock_rods,
                         "axis": (ax, tuple(c), a0, a1)}

    # slip fits between printed parts, opened up to the gap
    slip = {}
    for n, into in SLIP.items():
        v0 = parts[n].Volume()
        parts[n] = fitted(parts[n], {m: parts[m] for m in into}, gap)
        slip[n] = {"into": list(into), "trimmed": round(100 * (1 - parts[n].Volume() / v0), 2)}

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
    stand = make_stand(p, L, s, parts, dummies, stock, gap)
    return p0, p, k, L, batt_x, parts, dummies, stock, stand, slip


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


def make_stand(p, L, s, parts, dummies, stock, gap):
    """A base, a post and a cradle under the fuselage at the balance point. The cradle is cut
    by the fuselage's outer section, so the belly and its chamfers sit in it."""
    x_c, hw, zb = L.cg_x * s, L.half_w * s, L.z_bottom * s
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


def make_fit_test(stock, fit, gap, plate_t=1.0):
    """A plate with vertical holes for each rod at fit - 0.1, fit and fit + 0.1 (one, two and three
    dots beside them), slots for a 1 mm tail plate at gap - 0.1, gap and gap + 0.1 a side, and a
    20 mm stub of each printed rod and a tail-plate tab to try in them."""
    steps = (-0.1, 0.0, 0.1)
    bodies, x = [], 4.0
    holes, dots = [], []
    for r in stock.values():
        for i, dv in enumerate(steps):
            dh = r["d"] + fit + dv
            x += dh / 2
            holes.append(cyl_z(dh / 2, -1, 6, x, 0))
            for j in range(i + 1):
                dots.append(cyl_z(0.45, 4.6, 6, x - 1.2 + 1.2 * j, 6.0))
            x += dh / 2 + 3.0
    for i, dg in enumerate(steps):
        w = plate_t + 2 * (gap + dg)
        x += w / 2 + 1.0
        holes.append(box(x - w / 2, x + w / 2, -4.0, 4.0, 1.5, 6))
        for j in range(i + 1):
            dots.append(cyl_z(0.45, 4.6, 6, x - 1.2 + 1.2 * j, 6.0))
        x += w / 2 + 3.0
    plate = cut(box(0, x + 1.0, -8.0, 8.0, 0, 5.0), *holes, *dots)
    bodies.append(plate)
    y = 12.0
    for r in stock.values():                         # stubs of the rods, lying flat as they print
        if r["printed"]:
            bodies.append(printed_rod(0, r["d"], 2.0, 22.0, (0, y, r["d"] / 2 - max(0.2, 0.1 * r["d"]))))
            y += r["d"] + 3.0
    bodies.append(box(2.0, 12.0, y, y + 8.0, 0, plate_t))   # a tail-plate tab
    out = bodies[0]
    for b in bodies[1:]:
        out = out.fuse(b)
    return out


# --------------------------------------------------------------------------
# Checks


def check_fits(parts, dummies, stock, stand):
    """Overlaps between the printed parts and the dummies, stock rods and stand (mm^3)."""
    refs = [B.Ref(f"dummy_{n}", d["shape"], d["colour"], 0, "electronics") for n, d in dummies.items()]
    refs += [B.Ref(k, sh, COLOURS["rod"], 0, "hardware") for r in stock.values() for k, sh in r["pieces"].items()]
    refs.append(B.Ref("stand", stand, COLOURS["stand"], 0, "hardware"))
    return B.interference(parts, refs, tol=0.01)   # printed parts against each other, and everything else


def ray_hits(v, t, origin, dirs):
    """Distances along each ray from origin to every triangle it crosses (Moller-Trumbore)."""
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    e1, e2 = b - a, c - a
    out = []
    for d in dirs:
        pv = np.cross(d, e2)
        det = np.einsum("ij,ij->i", e1, pv)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tv = origin - a
        u = np.einsum("ij,ij->i", tv, pv) * inv
        qv = np.cross(tv, e1)
        w = (qv @ d) * inv
        dist = np.einsum("ij,ij->i", e2, qv) * inv
        hit = ok & (u >= 0) & (w >= 0) & (u + w <= 1) & (dist > 1e-6)
        out.append(np.unique(np.round(dist[hit], 4)))
    return out


def rod_walls(parts, stock, stations=9, angles=24):
    """The thinnest wall round each rod's hole, in each part it goes through: rays from the rod's
    axis out across the hole's wall, at `stations` points along the part and `angles` directions.
    A ray whose first hit is not the hole's wall (a slot, an opening) is skipped."""
    out = {}
    for name, r in stock.items():
        ax, c, a0, a1 = r["axis"]
        rh = r["hole"] / 2
        perp = [i for i in range(3) if i != ax]
        dirs = []
        for k in range(angles):
            d = np.zeros(3)
            d[perp[0]], d[perp[1]] = np.cos(2 * np.pi * k / angles), np.sin(2 * np.pi * k / angles)
            dirs.append(d)
        for n in r["in"]:
            v, t = B.tessellate(parts[n], 0.01, 0.1)
            v, t = np.asarray(v, float), np.asarray(t, int)
            lo, hi = max(a0, v[:, ax].min()), min(a1, v[:, ax].max())
            walls = []
            for x in np.linspace(lo, hi, stations + 2)[1:-1]:
                o = np.array(c, float)
                o[ax] = x
                for hits in ray_hits(v, t, o, dirs):
                    if len(hits) >= 2 and abs(hits[0] - rh) < 0.08:
                        walls.append(hits[1] - hits[0])
            if walls:
                out.setdefault(name, {})[n] = round(float(min(walls)), 2)
    return out


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
    if name.startswith("rod_"):                           # solid, so it is as stiff as it can be
        opts[opts.index("15%")] = "100%"
        opts += ["--brim-width", "3"]
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
            if not name.startswith("dummy_") and name not in ("stand", "fit_test"):
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
    out += [(k, sh, COLOURS["rod"] if r["printed"] else COLOURS["stock"], "rods")
            for r in stock.values() for k, sh in r["pieces"].items()]
    out.append(("stand", stand, COLOURS["stand"], "display"))
    return out


def rod_to_bed(sh):
    """A printed rod lying along x, its flat on the bed."""
    bb = sh.BoundingBox()
    if bb.ylen > bb.xlen:
        sh = sh.rotate(V(0, 0, 0), V(0, 0, 1), 90)
    bb = sh.BoundingBox()
    return sh.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))


def print_files(parts, dummies, stock):
    """Every file to print, in print order: the airframe, the rods, the dummies, the stand, the fit test."""
    rods = [k for r in stock.values() if r["printed"] for k in r["pieces"]]
    return [*parts, *rods, *(f"dummy_{d}" for d in dummies), "stand", "fit_test"]


def export(p, parts, dummies, stock, stand, fit_test, out: Path):
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
    for r in stock.values():
        if r["printed"]:
            for k, sh in r["pieces"].items():
                stls[k], volumes[k] = out / "stl" / f"{k}.stl", sh.Volume()
                bed = rod_to_bed(sh)
                beds[k] = bed.BoundingBox()
                cq.exporters.export(bed, str(stls[k]), tolerance=0.005, angularTolerance=0.1)
    for k, sh in (("stand", stand), ("fit_test", fit_test)):
        stls[k], volumes[k] = out / "stl" / f"{k}.stl", sh.Volume()
        bb = sh.BoundingBox()
        bed = sh.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))
        beds[k] = bed.BoundingBox()
        cq.exporters.export(bed, str(stls[k]), tolerance=0.005, angularTolerance=0.1)

    everything = bodies(parts, dummies, stock, stand)
    assy = cq.Assembly(name="kipina_mini")
    groups = {g: cq.Assembly(name=g) for g in ("printed", "dummies", "rods", "display")}
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


def bed_fits(b, bed) -> bool:
    return all(d <= m for d, m in zip(sorted((b.xlen, b.ylen, b.zlen)), sorted(bed)))


def file_info(n, parts, dummies, stock):
    """(group, colour name, sRGB, print note) of a file."""
    if n in parts:
        return "printed", colour_name(n), B.PRINT[n][3], ORIENT_NOTES.get(n, "")
    if n.startswith("rod_"):
        return "rod", "black", COLOURS["rod"], "Lying flat on its flat, brim. 100 % infill."
    if n.startswith("dummy_"):
        return "dummy", "as you like", dummies[n[6:]]["colour"], "Thinnest side down, supports where needed."
    if n == "stand":
        return "stand", "grey", COLOURS["stand"], "Base down."
    return "test", "any", COLOURS["fit_test"], "Flat. Print it first."


def write_report(args, p0, p, L, batt_x, parts, dummies, stock, stand, slip, walls, beds, sliced, hits, gaps,
                 out: Path):
    s = args.scale
    every = bodies(parts, dummies, stock, stand)
    bb = cq.Compound.makeCompound([sh for n, sh, *_ in every if n != "stand"]).BoundingBox()
    sb = stand.BoundingBox()
    files = print_files(parts, dummies, stock)
    fits = {n: bed_fits(beds[n], p.bed) for n in files}
    total_t = sum(r["seconds"] for r in sliced.values())
    total_g = sum(r["grams"] for r in sliced.values())
    fills = {n: r["fill"] for n, r in sliced.items() if "fill" in r}
    printed_rods = any(r["printed"] for r in stock.values())
    min_wall = min((w for ws in walls.values() for w in ws.values()), default=None)
    dummy_gaps = [g for g in gaps.values() if 0 < g < float("inf")]
    L_ = []
    a = L_.append
    a(f"# Kipinä single boom, 1:{1 / s:g} desk model: build report\n")
    flags = f"--scale {s:g} --fit {args.fit:g} --gap {args.gap:g}" + (" --stock-rods" if args.stock_rods else "")
    a(f"Generated by `mini.py {flags}` from the single boom's CAD. Do not edit by hand.\n")
    a("A scale model to print: to check how the parts go together before printing the full-size plane, "
      "and to show it. It doesn't fly. The parts are the full-size plane's, split the same way and scaled; "
      "the bought parts are printed dummies, and the carbon boom and spars are "
      + ("printed rods.\n" if printed_rods else "stock rods cut to length.\n"))
    a("## Key numbers\n")
    a("| | full size | model |")
    a("|---|---|---|")
    a(f"| Scale | 1:1 | **1:{1 / s:g}** |")
    a(f"| Wingspan | {p0.span:.0f} mm | {p0.span * s:.1f} mm |")
    a(f"| Length (nose to prop) | {bb.xlen / s:.0f} mm | {bb.xlen:.1f} mm |")
    a(f"| Height | {bb.zlen / s:.0f} mm | {bb.zlen:.1f} mm (on the stand {bb.zmax - sb.zmin:.0f} mm) |")
    a(f"| Fuselage walls | {p0.wall:.1f} mm | {p.wall * s:.2f} mm (thickened inwards before scaling) |")
    a(f"| Servo fairing shells | {B.FAIRING_WALL:.2f} mm | {max(B.FAIRING_WALL, MIN_WALL / s) * s:.2f} mm |")
    a(f"| Trailing edges | {p0.te_min:.1f} mm | {p.te_min * s:.2f} mm |")
    a(f"| Tail plates | {p0.plate:.1f} mm | {p.plate * s:.2f} mm |")
    a(f"| Tail mount collar and fin socket cheeks | {p0.collar_wall:.1f} mm | {p.collar_wall * s:.2f} mm |")
    a(f"| Files to print | | {len(files)}: {len(parts)} airframe parts, "
      + (f"{sum(len(r['pieces']) for r in stock.values())} rods, " if printed_rods else "")
      + f"{len(dummies)} dummies, the stand and the fit test |")
    if sliced:
        a(f"| Print time, all of it | | **{fmt_min(total_t)}**, {total_g:.0f} g of PLA (PrusaSlicer, A1 mini profile) |")
    a("")
    a(f"The wing sits where it does on the full-size plane (leading edge {L.x_le:.1f} mm from the nose, "
      f"{L.x_le * s:.1f} mm on the model) and the battery at its full-size station ({batt_x:.1f} mm, "
      f"{batt_x * s:.1f} mm). The walls, the trailing edges and the servo fairings' shells are thicker "
      "inwards, so the outside is the full-size plane's shape; the tail mount's collar and fin socket and the "
      "boom sleeves are thicker outwards "
      f"({p.boss_wall * s:.2f} mm instead of {p0.boss_wall * s:.2f} mm), which lifts the wing seat "
      f"{(p.boss_wall - p0.boss_wall) * s:.2f} mm.\n")

    a("## Fits and tolerances\n")
    a("Scaling halves every clearance the full-size plane has, and a printed part fits a printed part "
      "less well than a carbon rod fits a printed hole. So every fit is cut again for the model:\n")
    a("| fit | where | clearance | how |")
    a("|---|---|---|---|")
    a(f"| Rod in its hole | boom in the fuselage, tail mount and motor cap; spars in the wing | "
      f"**{args.fit:g} mm** on the diameter | holes drilled for the rod + {args.fit:g} mm; every hole is "
      "vertical on the bed, so it prints round |")
    a(f"| Slip fit | lid in the fuselage, fin in the tail mount, the fuselage halves' joint | "
      f"**{args.gap:g} mm** a side (at least {0.7 * args.gap:.2f} mm at edges and corners) | the inner part "
      "trimmed where it came closer to the outer one |")
    if dummy_gaps:
        a(f"| Dummy in its pocket | servos, FC, ESC, receiver, VTX, camera, battery | **{args.gap:g} mm** a side "
          f"(measured: at least {min(dummy_gaps):.2f} mm) | each dummy trimmed where it came closer to a printed part |")
    a(f"| Fuselage in the stand's cradle | under the balance point | {args.gap:g} mm | the cradle cut by the "
      "fuselage's section grown by the gap |")
    a("| Glued joints | wing panels on the centre, stabiliser under the tail mount, wing on the fuselage, "
      "fairings under the wing | 0 (faces touch) | as designed; nothing overlaps |")
    a("")
    a("**Print `stl/fit_test.stl` first** (about 15 minutes). It has a hole for each rod at "
      f"{args.fit - 0.1:g}, {args.fit:g} and {args.fit + 0.1:g} mm (one, two and three dots), slots for the "
      f"{p.plate * s:g} mm tail plates at {args.gap - 0.1:g}, {args.gap:g} and {args.gap + 0.1:g} mm a side, "
      "and a stub of each rod and a tail-plate tab to try in them:\n")
    a("- The stubs and the tab slide into the two-dot holes and slots by hand, without play: print the model "
      "as it is.")
    a(f"- They only go into the three-dot ones: run `python3 airframe/mini.py --fit {args.fit + 0.1:g} "
      f"--gap {args.gap + 0.1:g}`.")
    a(f"- They are loose even in the one-dot ones: `--fit {args.fit - 0.1:g} --gap {args.gap - 0.1:g}`.")
    a("")

    a("## Rods\n")
    if printed_rods:
        a("Printed in black PLA, solid, lying flat on a flat 10 % of their diameter deep. The spars are as "
          "thick as the wing allows with 0.8 mm of wall round the hole, thicker than the scaled carbon so they "
          "hold their shape; each is two halves that meet at the centre line, inside the wing centre, so each "
          "half fits the bed. The boom fills its socket's scaled bore less the fit clearance.\n")
    else:
        a("The holes are drilled for these stock sizes plus the fit clearance, not scaled.\n")
    a("| rod | full size | model | files | length | hole | through (thinnest wall round the hole) |")
    a("|---|---|---|---|---|---|---|")
    full_d = {"boom": f"{p0.boom_od:g} x {p0.boom_id:g} mm carbon tube", "spar_main": f"{p0.main_spar_d:g} mm carbon rod",
              "spar_rear": f"{p0.rear_spar_d:g} mm carbon rod"}
    for n, r in stock.items():
        what = f"**{r['d']:g} mm** printed" if r["printed"] else f"**{r['d']:g} mm** {ROD_KIND[n]}"
        files_ = ", ".join(f"`{k}.stl`" for k in r["pieces"]) if r["printed"] else "bought"
        ws = walls.get(n, {})
        thr = ", ".join(f"{m} ({ws[m]:.2f} mm)" if m in ws else m for m in r["in"])
        a(f"| {ROD_NAMES[n]} | {full_d[n]} | {what} | {files_} | {r['length']:.0f} mm | {r['hole']:g} mm | {thr} |")
    a("")
    if min_wall is not None:
        a(f"Thinnest wall round any rod hole: **{min_wall:.2f} mm** (rays from each rod's axis across its hole's "
          "wall, at nine stations along each part and every 15 degrees).\n")
    a("The pushrods, the control horns' screws, the hinge tape and the wiring are left off; glue the control "
      "surfaces at neutral, or hinge them with tape.\n")

    a("## Print list\n")
    a(f"Build volume checked: {p.bed[0]:.0f} x {p.bed[1]:.0f} x {p.bed[2]:.0f} mm (Bambu Lab A1 mini). "
      "PLA, 0.4 mm nozzle, 0.12 mm layers, 2 walls, 15 % infill, 5 top and 4 bottom layers; the rods solid; "
      "the dummies with supports where the slicer wants them. Leave the slicer's X-Y hole and contour "
      "compensation at 0: the fit test sets the clearances.\n")
    a("| file | colour | on the bed, mm | fits | time | g | orientation |")
    a("|---|---|---|---|---|---|---|")
    for n in files:
        b = beds[n]
        r = sliced.get(n, {})
        _, col, _, note = file_info(n, parts, dummies, stock)
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
      "thicker (the lid's hole is opened up for it), so they print. A gap of 0 is a face the dummy rests or "
      "is glued on, like the motor on its mount.\n")
    a("| dummy | full-size part, mm | trimmed off | gap to the nearest printed part |")
    a("|---|---|---|---|")
    for n, d in dummies.items():
        g = "-" if gaps[n] == float("inf") else f"{gaps[n]:.2f} mm"
        a(f"| dummy_{n} | {' x '.join(f'{v:g}' for v in d['full'])} | {d['trimmed']:g} % | {g} |")
    a("")

    a("## Checks\n")
    if hits:
        a("Overlaps (printed parts against each other, and against the dummies, the rods and the stand):\n")
        for x, y, v in hits:
            a(f"- {x} and {y}: {v} mm^3")
    else:
        a("Interference: no two printed parts overlap, and the dummies, the rods and the stand overlap none of them.")
    a("")
    if fills:
        low = {n: f for n, f in fills.items() if f < 0.9}
        if low:
            a("Thin features: printed solid, these parts fill less than 90 % of their volume, so the slicer "
              "drops some of their detail:\n")
            for n, f in low.items():
                a(f"- {n}: {f * 100:.0f} %")
        else:
            lo = min(fills.items(), key=lambda kv: kv[1])
            a(f"Thin features: printed solid, every part fills at least {lo[1] * 100:.0f} % of its volume in "
              f"PrusaSlicer (the lowest is {lo[0]}), so nothing is too thin to print. Over 100 % is the slicer's "
              "lines overlapping a little.")
        a("")
        a("| part | solid fill |")
        a("|---|---|")
        for n, f in fills.items():
            a(f"| {n} | {f * 100:.0f} % |")
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
    a("4. Push the main and rear spar halves in from each wing tip, through the panel into the wing centre, "
      "until they meet in the middle. Put the aileron servo dummies in their pockets, glue the fairings over "
      "them and the ailerons on.")
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


def write_spec(args, p0, parts, dummies, stock, stand, slip, walls, beds, sliced, hits, gaps, bb, out: Path):
    """viewer/spec.json: the numbers the viewer page shows."""
    s = args.scale
    sb = stand.BoundingBox()
    prints = []
    for n in print_files(parts, dummies, stock):
        group, col, rgb, _ = file_info(n, parts, dummies, stock)
        r = sliced.get(n, {})
        b = beds[n]
        prints.append({"file": f"{n}.stl", "group": group, "swatch": hex_colour(rgb), "colour": col,
                       "bed": [round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)],
                       "minutes": round(r["seconds"] / 60, 1) if r else None, "grams": r.get("grams")})
    full = {"boom": f"{p0.boom_od:g} x {p0.boom_id:g} mm carbon tube", "spar_main": f"{p0.main_spar_d:g} mm carbon rod",
            "spar_rear": f"{p0.rear_spar_d:g} mm carbon rod"}
    fills = {n: r["fill"] for n, r in sliced.items() if "fill" in r}
    spec = {
        "ratio": f"{1 / s:g}", "scale": s, "gap": args.gap, "fit": args.fit,
        "span": p0.span * s, "length": bb.xlen, "height_on_stand": bb.zmax - sb.zmin,
        "minutes": round(sum(r["seconds"] for r in sliced.values()) / 60, 1) if sliced else None,
        "grams": round(sum(r["grams"] for r in sliced.values()), 1) if sliced else None,
        "prints": prints,
        "rods": [{"name": n, "label": ROD_NAMES[n], "full": full[n], "d": r["d"], "printed": r["printed"],
                  "kind": "printed PLA" if r["printed"] else ROD_KIND[n], "files": list(r["pieces"]) if r["printed"] else [],
                  "length": r["length"], "hole": r["hole"], "through": r["in"], "walls": walls.get(n, {})}
                 for n, r in stock.items()],
        "slip": slip,
        "dummies": [{"name": n, "full": d["full"], "trimmed": d["trimmed"],
                     "gap": None if gaps[n] == float("inf") else gaps[n]} for n, d in dummies.items()],
        "checks": {"hits": hits, "thin": {n: f for n, f in fills.items() if f < 0.9},
                   "thin_ok": (all(f >= 0.9 for f in fills.values()) if fills else None),
                   "thin_min": list(min(fills.items(), key=lambda kv: kv[1])) if fills else None,
                   "min_wall": min((w for ws in walls.values() for w in ws.values()), default=None),
                   "fits": all(bed_fits(b, p0.bed) for b in beds.values())},
    }
    (out / "viewer" / "spec.json").write_text(json.dumps(spec, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--scale", type=float, default=0.5, help="model size / full size (default 0.5, 1:2)")
    ap.add_argument("--fit", type=float, default=0.3, help="rod hole oversize on the diameter, mm")
    ap.add_argument("--gap", type=float, default=0.25, help="slip fits and dummies: clearance a side, mm")
    ap.add_argument("--stock-rods", action="store_true", help="bought rods of the nearest stock size, not printed ones")
    ap.add_argument("--boom", type=float, help="boom diameter, mm")
    ap.add_argument("--spar", type=float, help="main spar diameter, mm")
    ap.add_argument("--rear-spar", type=float, help="rear spar diameter, mm")
    ap.add_argument("--no-slice", action="store_true", help="skip PrusaSlicer")
    args = ap.parse_args(argv)
    rods = {"boom": args.boom, "spar_main": args.spar, "spar_rear": args.rear_spar}
    p0, p, k, L, batt_x, parts, dummies, stock, stand, slip = make_model(args.scale, args.fit, args.gap, rods,
                                                                         args.stock_rods)
    fit_test = make_fit_test(stock, args.fit, args.gap, p.plate * args.scale)
    for n, sh in [*parts.items(), *((f"dummy_{n}", d["shape"]) for n, d in dummies.items()), ("stand", stand),
                  *((k_, sh_) for r in stock.values() for k_, sh_ in r["pieces"].items()), ("fit_test", fit_test)]:
        assert sh.isValid(), f"{n} is not a valid solid"
    hits = check_fits(parts, dummies, stock, stand)
    for a, b, v in hits:
        print(f"WARNING: {a} overlaps {b} by {v} mm^3")
    walls = rod_walls(parts, stock)
    for n, ws in walls.items():
        for m, w in ws.items():
            if w < MIN_WALL - 0.05:
                print(f"WARNING: {n} leaves {w} mm of wall in {m}")
    gaps = clearances(parts, dummies)
    stls, volumes, beds = export(p, parts, dummies, stock, stand, fit_test, OUT)
    sliced = {} if args.no_slice else slice_all(stls, volumes)
    (OUT / "checks.json").write_text(json.dumps({"hits": hits, "gaps": gaps, "walls": walls, "slip": slip,
                                                 "sliced": sliced}, indent=1))
    bb = write_report(args, p0, p, L, batt_x, parts, dummies, stock, stand, slip, walls, beds, sliced, hits, gaps, OUT)
    write_spec(args, p0, parts, dummies, stock, stand, slip, walls, beds, sliced, hits, gaps, bb, OUT)
    print(f"1:{1 / args.scale:g}: span {p0.span * args.scale:.1f} mm, length {bb.xlen:.1f} mm; wrote {OUT}")


if __name__ == "__main__":
    main()
