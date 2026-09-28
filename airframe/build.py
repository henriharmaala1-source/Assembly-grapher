#!/usr/bin/env python3
"""Build the twin-tube micro FPV plane in CadQuery and export everything.

    pip install cadquery trimesh
    python3 airframe/build.py

Outputs (next to this file):
    stl/        one print-oriented STL per printed part
    cad/        airframe.step (flight position, coloured assembly)
    viewer/     airframe.glb for the three.js viewer (tools/render.mjs makes the PNGs)
    REPORT.md   masses from the CAD volumes, CG, battery station, sizing sweep
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import Vector as V

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from design import (CARBON, LW_PLA, PLA, Kit, Layout, Params, camber,  # noqa: E402
                    naca4, neutral_point, performance, report, rod_mass,
                    solve_x_le, structure_estimate)

# --------------------------------------------------------------------------
# Small solid helpers (all coordinates in the flight frame)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_x(r, x0, x1, y, z):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_y(r, y0, y1, x, z):
    return cq.Solid.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_z(r, z0, z1, x, y):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def prism_y(pts_xz, y0, y1):
    """Extrude a closed polygon drawn in x-z along y."""
    wire = cq.Wire.makePolygon([V(x, y0, z) for x, z in pts_xz], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, y1 - y0, 0))


def prism_z(pts_xy, z0, z1):
    """Extrude a closed polygon drawn in x-y along z."""
    wire = cq.Wire.makePolygon([V(x, y, z0) for x, y in pts_xy], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, 0, z1 - z0))


def fuse(*shapes):
    return shapes[0].fuse(*shapes[1:]).clean()


def cut(shape, *tools):
    return shape.cut(*tools).clean()


# --------------------------------------------------------------------------
# Wing section


class Section:
    """NACA section at the wing's incidence, placed on the pod top."""

    def __init__(self, p: Params, L: Layout):
        self.p, self.L = p, L
        c = L.chord
        self.th = math.radians(p.incidence)
        up, lo = naca4(p.naca, 70, p.te_min / c)
        up = [self._rot(*q) for q in up]
        lo = [self._rot(*q) for q in lo]
        self.dz = L.z_top + p.wing_gap - min(z for _, z in lo)
        self.up = [(x, z + self.dz) for x, z in up]
        self.lo = [(x, z + self.dz) for x, z in lo]

    def _rot(self, xf, zf):
        c = self.L.chord
        dx, dz = (xf - 0.25) * c, zf * c
        return (self.L.x_qc + dx * math.cos(self.th) + dz * math.sin(self.th),
                -dx * math.sin(self.th) + dz * math.cos(self.th))

    def camber_point(self, xf):
        x, z = self._rot(xf, camber(self.p.naca, xf)[0])
        return x, z + self.dz

    def x_at(self, xf):
        return self.camber_point(xf)[0]

    def upper_z(self, x):
        pts = sorted(q for q in self.up[5:])
        return float(np.interp(x, [q[0] for q in pts], [q[1] for q in pts]))

    def lower_z(self, x):
        pts = sorted(q for q in self.lo[5:])
        return float(np.interp(x, [q[0] for q in pts], [q[1] for q in pts]))

    def outline(self):
        return list(reversed(self.up)) + self.lo[1:]

    def solid(self, y0, y1):
        vs = [V(x, y0, z) for x, z in self.outline()]
        wire = cq.Wire.assembleEdges([cq.Edge.makeSpline(vs), cq.Edge.makeLine(vs[-1], vs[0])])
        return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, y1 - y0, 0))

    def centre_solid(self, y0, y1, z_floor, fill_from=0.08):
        """Section with the gap under it filled down to z_floor (sits on the pod)."""
        k = next(i for i, (x, _) in enumerate(self.lo) if x >= self.L.x_le + fill_from * self.L.chord)
        head = [V(x, y0, z) for x, z in list(reversed(self.up)) + self.lo[1:k + 1]]
        x_f, x_t = self.lo[k][0], self.lo[-1][0]
        z_t = self.lo[-1][1]
        tail = [head[-1], V(x_f, y0, z_floor), V(x_t, y0, z_floor), V(x_t, y0, z_t), head[0]]
        edges = [cq.Edge.makeSpline(head)]
        edges += [cq.Edge.makeLine(a, b) for a, b in zip(tail[:-1], tail[1:])]
        face = cq.Face.makeFromWires(cq.Wire.assembleEdges(edges))
        return cq.Solid.extrudeLinear(face, V(0, y1 - y0, 0))


# --------------------------------------------------------------------------
# Printed parts


def make_pod(p: Params, L: Layout, batt_x: float):
    hw, zb, Lp = L.half_w, L.z_bottom, L.pod_len
    ty, rh, rb = L.tube_y, L.r_hole, L.r_boss
    w, fw, ch = p.wall, p.front_wall, 4.0
    zf = zb + w                                             # floor top

    outer = (cq.Workplane().box(Lp, 2 * hw, -zb, centered=(False, True, False))
             .translate((0, 0, zb)).edges("|X and <Z").chamfer(ch)
             .faces("<X").edges("<Z").chamfer(ch)).val()
    ch_in = ch + w * math.sqrt(2) - 2 * w                   # keeps the wall even
    inner = (cq.Workplane().box(Lp - fw - w, 2 * (hw - w), 20 - zf, centered=(False, True, False))
             .translate((fw, 0, zf)).edges("|X and <Z").chamfer(ch_in)
             .faces("<X").edges("<Z").chamfer(ch + w * math.sqrt(2) - fw - w)).val()
    pod = cut(outer, inner)

    sleeves = [cyl_x(rb, x0, x0 + p.sleeve_len, s * ty, 0)
               for x0 in (0.0, Lp - p.sleeve_len) for s in (-1, 1)]
    nose = [box(0, fw, -ty, ty, 0, rb),                     # between the front sleeves
            box(0, 3.2, -13, 13, 0, p.motor_z),              # motor plate
            cyl_x(13, 0, 3.2, 0, p.motor_z)]
    fc = [cyl_z(2.3, zf - 0.1, zf + 3, 39 + dx, dy) for dx in (-10, 10) for dy in (-10, 10)]
    # elevator servo stands upright beside the camera, horn up at the pushrod height
    sx0, sx1, sy0, sy1 = fw + 0.6, fw + 0.6 + 20.8, 9.6, 18.6
    cradle = cut(box(sx0 - 1, sx1 + 1, sy0 - 1, sy1 + 0.5, zf - 0.1, zf + 5),
                 box(sx0, sx1, sy0, sy1, zf, zf + 6))
    pod = fuse(pod, *sleeves, *nose, *fc, cradle)

    holes = [cyl_x(rh, L.tube_x0, Lp + 1, s * ty, 0) for s in (-1, 1)]
    holes += [cyl_z(0.85, zf, zf + 4, 39 + dx, dy) for dx in (-10, 10) for dy in (-10, 10)]
    holes.append(cyl_x(3.2, -1, 4, 0, p.motor_z))           # motor shaft / circlip
    for a in (45, 135, 225, 315):                           # 9x9, 12 mm and 16 mm patterns
        r = (5.9 + 8.6) / 2
        slot = (cq.Workplane("YZ").center(r * math.cos(math.radians(a)),
                                          p.motor_z + r * math.sin(math.radians(a)))
                .slot2D(8.6 - 5.9 + 2.2, 2.2, a).extrude(6).translate((-1, 0, 0))).val()
        holes.append(slot)
    zc = -9.0                                               # nano camera (14 mm) window
    holes.append(box(-1, fw + 1, -7.2, 7.2, zc - 7.2, zc + 7.2))
    holes.append(box(-1, fw + 1, -14.0, -8.0, -2.0, 1.2))   # motor wires
    for s in (-1, 1):                                       # battery strap slots
        y = s * (Kit().batt_w / 2 + 1.5)
        holes.append(box(batt_x - 12, batt_x + 12, y - 1.0, y + 1.0, zb - 1, zf + 1))
    return cut(pod, *holes)


def make_lid(p: Params, L: Layout):
    x0, x1 = p.sleeve_len + 0.3, L.x_le - 0.5
    z0 = p.tube_od / 2                                      # rests on the tubes
    plate = box(x0, x1, -L.tube_y - 1.5, L.tube_y + 1.5, z0, z0 + 0.8)
    y_lip = L.tube_y - p.tube_od / 2 - 0.3
    lips = [box(x0 + 2, x1 - 2, s * y_lip - (0.8 if s > 0 else 0), s * y_lip + (0.8 if s < 0 else 0),
                z0 - 2.5, z0 + 0.1) for s in (-1, 1)]
    lid = fuse(plate, *lips)
    return cut(lid, cyl_z(1.8, z0 - 3, z0 + 2, x0 + 8, -9))   # VTX antenna


def make_wing_centre(p: Params, L: Layout, sec: Section):
    cw = p.center_width
    body = sec.centre_solid(-cw / 2, cw / 2, L.z_top)
    x0, x1 = L.pod_len + 0.3, L.x_te - 1.5                  # saddle behind the pod
    saddle = [cyl_x(L.r_boss, x0, x1, s * L.tube_y, 0) for s in (-1, 1)]
    saddle.append(box(x0, x1, -L.tube_y, L.tube_y, 2.0, L.z_top + 0.4))
    body = fuse(body, *saddle)
    holes = [cyl_x(L.r_hole, x0 - 1, x1 + 1, s * L.tube_y, 0) for s in (-1, 1)]
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, -cw, cw, x, z))
    xw, zw = sec.camber_point(0.45)                          # servo leads into the pod
    holes.append(cyl_y(2.0, -cw, cw, xw, zw))
    holes.append(cyl_z(3.0, L.z_top - 1, zw, xw, 0))
    return cut(body, *holes)


def servo_pocket(p: Params, L: Layout, sec: Section, y0: float):
    """Flat-mounted aileron servo, output shaft pointing at the tip."""
    xa = sec.x_at(0.30)
    xb = xa + 21.0
    zt = min(sec.upper_z(x) for x in np.linspace(xa, xb, 12)) - 1.2
    zb = min(sec.lower_z(x) for x in np.linspace(xa, xb, 12)) - 3
    ya, yb = y0 + 4.0, y0 + 4.0 + 18.8
    xs = xb - 5.5                                            # shaft station
    return [box(xa, xb, ya, yb, zb, zt), box(xs - 6, xs + 6, yb - 1, yb + 4.5, zb, zt)], xs, yb


def make_panel(p: Params, L: Layout, sec: Section):
    """Right wing panel (the left one is its mirror image)."""
    y0, y1 = p.center_width / 2, p.span / 2
    panel = sec.solid(y0, y1)
    holes = []
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, y0 - 1, y1 - 5, x, z))
    xw, zw = sec.camber_point(0.45)
    holes.append(cyl_y(2.0, y0 - 1, y0 + 8, xw, zw))
    pocket, _, _ = servo_pocket(p, L, sec, y0)
    holes += pocket
    xh = sec.x_at(p.hinge_pos)
    holes.append(box(xh - 0.3, L.x_te + 5, y0 + p.aileron_root_gap, y1 - p.aileron_tip_gap, -20, 40))
    return cut(panel, *holes)


def make_aileron(p: Params, L: Layout, sec: Section):
    y0, y1 = p.center_width / 2, p.span / 2
    xh = sec.x_at(p.hinge_pos) + 0.3
    ail = sec.solid(y0 + p.aileron_root_gap + 0.5, y1 - p.aileron_tip_gap - 0.5)
    ail = ail.intersect(box(xh, L.x_te + 5, -y1, y1, -20, 40))
    zl, zu = sec.lower_z(xh), sec.upper_z(xh)
    bevel = prism_y([(xh - 0.1, zl - 1), (xh - 0.1, zu - 1.0), (xh + 2.5, zl - 1)], -y1, y1)
    return cut(ail, bevel)


def make_tail_mount(p: Params, L: Layout):
    x0, x1 = L.x_stab + 0.5, L.x_stab + L.c_fix - 0.5
    r_out = L.r_hole + 1.2
    body = fuse(box(x0, x1, -L.tube_y - r_out, L.tube_y + r_out, L.tail_z - 1.4, L.tail_z),
                *[cyl_x(r_out, x0, x1, s * L.tube_y, 0) for s in (-1, 1)],
                box(x0 + 2.5, x0 + 10.5, -1.5, 1.5, L.pushrod_z - 2.5, L.tail_z - 1.2))
    holes = [cyl_x(L.r_hole, x0 - 1, L.tube_x1 + 0.2, s * L.tube_y, 0) for s in (-1, 1)]
    holes.append(cyl_x(0.8, x0, x0 + 12, 0, L.pushrod_z))   # 1 mm pushrod guide
    return cut(body, *holes)


def make_stab(p: Params, L: Layout):
    s = (cq.Workplane().box(L.c_fix, L.b_h - 1.0, p.plate, centered=(False, True, False))
         .translate((L.x_stab, 0, L.tail_z)).faces("<X").edges("|Y").fillet(p.plate * 0.45)).val()
    return s


def make_elevator(p: Params, L: Layout):
    x0, x1 = L.x_stab + L.c_fix + 0.6, L.x_stab + L.c_h
    y = L.b_h / 2 - 0.6
    z0, z1 = L.tail_z, L.tail_z + p.plate
    el = box(x0, x1, -y, y, z0, z1)
    horn = prism_y([(x0 + 0.5, z0 + 0.1), (x0 + 8, z0 + 0.1), (x0 + 4, L.pushrod_z - 2.2),
                    (x0 + 0.5, L.pushrod_z - 2.2)], -0.8, 0.8)
    el = fuse(el, horn)
    cuts = [prism_y([(x0 - 0.1, z0 - 0.1), (x0 - 0.1, z1 - 0.6), (x0 + 2.0, z0 - 0.1)], -y - 1, -0.8),
            prism_y([(x0 - 0.1, z0 - 0.1), (x0 - 0.1, z1 - 0.6), (x0 + 2.0, z0 - 0.1)], 0.8, y + 1),
            prism_y([(x1 - 7, z0 - 0.1), (x1 + 0.1, z0 - 0.1), (x1 + 0.1, z0 + 1.3)], -y - 1, y + 1),
            cyl_y(0.6, -2, 2, x0 + 3.0, L.pushrod_z)]
    return cut(el, *cuts)


def make_fin(p: Params, L: Layout):
    """Right fin; the left one is its mirror image."""
    y0 = L.b_h / 2
    z0, z1 = L.tail_z - p.fin_below, L.tail_z + p.plate + L.fin_above
    fin = (cq.Workplane().box(L.c_h, p.plate, z1 - z0, centered=False)
           .translate((L.x_stab, y0, z0)).faces("<X").edges("|Z").fillet(p.plate * 0.45)).val()
    g = 0.15
    jaws = [box(L.x_stab + 1, L.x_stab + L.c_fix - 1, y0 - 7, y0 + 0.1, zz0, zz1)
            for zz0, zz1 in ((L.tail_z - g - 1.2, L.tail_z - g),
                             (L.tail_z + p.plate + g, L.tail_z + p.plate + g + 1.2))]
    x_te = L.x_stab + L.c_h
    te = prism_z([(x_te - 7, y0 - 0.1), (x_te + 0.1, y0 - 0.1), (x_te + 0.1, y0 + 1.3)], z0 - 1, z1 + 1)
    return cut(fuse(fin, *jaws), te)


# --------------------------------------------------------------------------
# Non-printed reference bodies (for the preview, the STEP and the CG)


def reference_bodies(p: Params, k: Kit, L: Layout, sec: Section, batt_x: float):
    zf = L.z_bottom + p.wall
    ref = {}
    ref["tube_L"] = cut(cyl_x(p.tube_od / 2, L.tube_x0, L.tube_x1, -L.tube_y, 0),
                        cyl_x(p.tube_id / 2, L.tube_x0 - 1, L.tube_x1 + 1, -L.tube_y, 0))
    ref["tube_R"] = ref["tube_L"].mirror("XZ")
    for pos, d, name in ((p.main_spar_pos, p.main_spar_d, "spar_main"),
                         (p.rear_spar_pos, p.rear_spar_d, "spar_rear")):
        x, z = sec.camber_point(pos)
        ref[name] = cyl_y(d / 2, -p.span / 2 + 5, p.span / 2 - 5, x, z)
    ref["motor"] = fuse(cyl_x(9.0, -12.5, 0, 0, p.motor_z), cyl_x(2.5, -17, -12.5, 0, p.motor_z))
    blade = box(-15.5, -14.5, -50.8, 50.8, p.motor_z - 4, p.motor_z + 4)
    ref["prop"] = fuse(blade, cyl_x(4, -16.5, -13.5, 0, p.motor_z))
    ref["battery"] = box(batt_x - k.batt_len / 2, batt_x + k.batt_len / 2,
                         -k.batt_w / 2, k.batt_w / 2, zf, zf + k.batt_h)
    ref["camera"] = box(p.front_wall, p.front_wall + 14, -7, 7, -16, -2)
    ref["fc"] = box(39 - 13.5, 39 + 13.5, -13.5, 13.5, zf + 3, zf + 8)
    ref["servo_elev"] = box(p.front_wall + 0.6, p.front_wall + 21.4, 9.6, 18.6, zf, zf + 16)
    _, xs, yb = servo_pocket(p, L, sec, p.center_width / 2)
    zt = sec.upper_z(xs) - 1.2
    servo = box(xs - 15.5, xs + 5.5, p.center_width / 2 + 4, yb, zt - 8.5, zt)
    ref["servo_ail_R"] = servo
    ref["servo_ail_L"] = servo.mirror("XZ")
    x_h = L.x_stab + L.c_fix + 3.6
    ref["pushrod"] = cyl_x(0.5, p.front_wall + 16, x_h, 0, L.pushrod_z)
    return ref


# --------------------------------------------------------------------------
# Mass properties


# name: (density g/cm^3, shell thickness mm, infill fraction, colour)
PRINT = {
    "pod":         (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "lid":         (PLA, 0.8, 0.15, (0.28, 0.30, 0.34)),
    "wing_centre": (LW_PLA, 0.6, 0.08, (0.86, 0.87, 0.84)),
    "wing_R":      (LW_PLA, 0.45, 0.0, (0.93, 0.93, 0.90)),
    "wing_L":      (LW_PLA, 0.45, 0.0, (0.93, 0.93, 0.90)),
    "aileron_R":   (LW_PLA, 0.45, 0.0, (0.96, 0.45, 0.10)),
    "aileron_L":   (LW_PLA, 0.45, 0.0, (0.96, 0.45, 0.10)),
    "tail_mount":  (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "stab":        (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "elevator":    (LW_PLA, 0.4, 0.15, (0.96, 0.45, 0.10)),
    "fin_R":       (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "fin_L":       (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
}
PRINT_NOTES = {
    "pod": "Upright, open top up. 2 walls, 15 % infill.",
    "lid": "Flat, lips up.",
    "wing_centre": "On its side, spar holes vertical. 0.6 mm walls, 8 % infill.",
    "wing_R": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "wing_L": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "aileron_R": "Flat on the lower surface. 1 wall, 0 % infill.",
    "aileron_L": "Flat on the lower surface. 1 wall, 0 % infill.",
    "tail_mount": "Plate face down.",
    "stab": "Flat. 2 top / 2 bottom layers, 15 % infill.",
    "elevator": "Top face down, horn up.",
    "fin_R": "Outer face down, jaws up.",
    "fin_L": "Outer face down, jaws up.",
}
REF_COLOUR = {
    "tube": (0.08, 0.08, 0.09), "spar": (0.08, 0.08, 0.09), "motor": (0.55, 0.57, 0.60),
    "prop": (0.10, 0.45, 0.85), "battery": (0.85, 0.75, 0.15), "camera": (0.10, 0.10, 0.10),
    "fc": (0.10, 0.55, 0.30), "servo": (0.15, 0.30, 0.70), "pushrod": (0.08, 0.08, 0.09),
}


def print_mass(name, shape):
    rho, shell, infill, _ = PRINT[name]
    vol, area = shape.Volume(), shape.Area()
    solid = min(vol, area * shell)
    return rho * (solid + infill * max(0.0, vol - solid)) / 1000


def ref_mass(name, p: Params, k: Kit, L: Layout):
    if name.startswith("tube"):
        return rod_mass(p.tube_od, L.tube_len, p.tube_id, p.tube_density)
    table = {
        "spar_main": rod_mass(p.main_spar_d, p.span - 10),
        "spar_rear": rod_mass(p.rear_spar_d, p.span - 10),
        "motor": k.motor, "prop": k.prop, "battery": k.battery, "camera": k.cam_vtx,
        "fc": k.fc + k.rx, "servo_elev": k.servo, "servo_ail_R": k.servo,
        "servo_ail_L": k.servo, "pushrod": rod_mass(1.0, L.tube_len, rho=CARBON),
    }
    return table[name]


# --------------------------------------------------------------------------
# Print orientation


def to_bed(shape, name):
    s = shape
    if name == "lid" or name == "tail_mount" or name == "elevator":
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    elif name in ("wing_centre", "wing_R", "fin_L"):
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    elif name in ("wing_L", "fin_R"):
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), -90)
    elif name.startswith("aileron"):
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), -Params().incidence)
    bb = s.BoundingBox()
    return s.translate(V(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


# --------------------------------------------------------------------------
# Build


def build(p: Params = Params(), k: Kit = Kit()):
    x_le = solve_x_le(p, k)
    L = Layout(p, x_le)
    sec = Section(p, L)
    batt_nominal = (L.batt_min + L.batt_max) / 2

    parts = {}
    parts["wing_centre"] = make_wing_centre(p, L, sec)
    parts["wing_R"] = make_panel(p, L, sec)
    parts["wing_L"] = parts["wing_R"].mirror("XZ")
    parts["aileron_R"] = make_aileron(p, L, sec)
    parts["aileron_L"] = parts["aileron_R"].mirror("XZ")
    parts["tail_mount"] = make_tail_mount(p, L)
    parts["stab"] = make_stab(p, L)
    parts["elevator"] = make_elevator(p, L)
    parts["fin_R"] = make_fin(p, L)
    parts["fin_L"] = parts["fin_R"].mirror("XZ")
    parts["lid"] = make_lid(p, L)

    # CG with CAD masses: solve the battery station, then build the pod around it
    parts["pod"] = make_pod(p, L, batt_nominal)
    ref = reference_bodies(p, k, L, sec, batt_nominal)
    items = []
    for name, shape in parts.items():
        c = shape.Center()
        items.append((name, print_mass(name, shape), c.x, c.z))
    for name, shape in ref.items():
        if name == "battery":
            continue
        c = shape.Center()
        items.append((name, ref_mass(name, p, k, L), c.x, c.z))
    items += [("wiring", k.wiring, 0.5 * L.pod_len, -8.0), ("esc", k.esc, 30.0, -10.0),
              ("antenna", k.antenna, 20.0, 5.0), ("hardware", k.hardware, 0.55 * L.x_stab, 0.0)]
    target = L.x_le + p.cg_target * L.chord
    m = sum(i[1] for i in items)
    batt_x = (target * (m + k.battery) - sum(i[1] * i[2] for i in items)) / k.battery
    batt_x = round(batt_x, 1)
    parts["pod"] = make_pod(p, L, batt_x)
    ref = reference_bodies(p, k, L, sec, batt_x)
    items = [i for i in items if i[0] != "pod"]
    c = parts["pod"].Center()
    items.append(("pod", print_mass("pod", parts["pod"]), c.x, c.z))
    items.append(("battery", k.battery, batt_x, ref["battery"].Center().z))
    return p, k, L, sec, parts, ref, items, batt_x


# --------------------------------------------------------------------------
# Export


def tessellate(shape, tol=0.08, ang=0.25):
    vs, tris = shape.tessellate(tol, ang)
    return np.array([(v.x, v.y, v.z) for v in vs]), np.array(tris, dtype=np.int64)


def colour_of(name):
    if name in PRINT:
        return PRINT[name][3]
    return REF_COLOUR[name.split("_")[0]]


def srgb_to_linear(rgb):
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb]


def export_all(p, k, L, parts, ref, out: Path):
    (out / "stl").mkdir(parents=True, exist_ok=True)
    (out / "cad").mkdir(parents=True, exist_ok=True)
    for name, shape in parts.items():
        cq.exporters.export(to_bed(shape, name), str(out / "stl" / f"{name}.stl"),
                            tolerance=0.03, angularTolerance=0.15)

    assy = cq.Assembly(name="twin_tube_micro")
    for name, shape in {**parts, **ref}.items():
        assy.add(shape, name=name, color=cq.Color(*colour_of(name)))
    assy.export(str(out / "cad" / "airframe.step"))

    import trimesh
    from trimesh.visual.material import PBRMaterial
    scene = trimesh.Scene()
    for name, shape in {**parts, **ref}.items():
        v, t = tessellate(shape, 0.05, 0.2)
        v = np.column_stack([v[:, 0], v[:, 2], -v[:, 1]]) / 1000.0  # z-up mm -> y-up m
        mesh = trimesh.Trimesh(v, t, process=False)
        mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=name, baseColorFactor=[*srgb_to_linear(colour_of(name)), 1.0],
            metallicFactor=0.0, roughnessFactor=0.6))
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    (out / "viewer").mkdir(exist_ok=True)
    scene.export(str(out / "viewer" / "airframe.glb"), include_normals=True)


def write_report(p, k, L, parts, items, batt_x, out: Path):
    auw = sum(i[1] for i in items)
    cg = sum(i[1] * i[2] for i in items) / auw
    cgz = sum(i[1] * i[3] for i in items) / auw
    perf = performance(p, auw, L, k)
    np_ = neutral_point(p, L)
    est = sum(g for _, g, _ in structure_estimate(p, L))
    printed = sum(i[1] for i in items if i[0] in PRINT)
    lines = [
        "# Twin-tube micro FPV plane: build report",
        "",
        "Generated by `build.py` from the CAD volumes. Do not edit by hand.",
        "",
        "## Key numbers",
        "",
        "| | |",
        "|---|---|",
        f"| Wingspan | {p.span:.0f} mm |",
        f"| Chord | {L.chord:.0f} mm (NACA {p.naca}, {p.incidence:.0f} deg incidence) |",
        f"| Wing area | {L.area / 1e4:.2f} dm^2 |",
        f"| Length (prop to elevator TE) | {L.length:.0f} mm |",
        f"| Tubes | 2 x {p.tube_od:.0f}x{p.tube_id:.0f} mm, {L.tube_len:.0f} mm long, {p.tube_spacing:.0f} mm apart |",
        f"| Stabiliser | {L.b_h:.0f} x {L.c_h:.1f} mm (elevator {L.c_e:.1f} mm) |",
        f"| Fins | 2 x {L.c_h:.1f} x {L.fin_h:.0f} mm |",
        f"| All-up weight | **{auw:.0f} g** (printed parts {printed:.0f} g) |",
        f"| Wing loading | {perf['loading']:.1f} g/dm^2 |",
        f"| Stall / cruise | {perf['stall']:.1f} / {perf['cruise']:.1f} m/s |",
        f"| Endurance (rough) | ~{perf['endurance_min']:.0f} min on 2S 450 mAh |",
        f"| Tail volumes | Vh {L.vh_actual:.2f}, Vv {L.vv_actual:.3f} |",
        f"| CG target | {p.cg_target * 100:.0f} % chord = **{L.x_le + p.cg_target * L.chord:.1f} mm** "
        f"from the motor face ({p.cg_target * L.chord:.1f} mm behind the wing LE) |",
        f"| CG (computed) | x {cg:.1f} mm, z {cgz:.1f} mm |",
        f"| Neutral point | {np_ * 100:.0f} % chord -> static margin {(np_ - p.cg_target) * 100:.0f} % |",
        f"| Battery centre | **{batt_x:.0f} mm** from the motor face "
        f"(travel {L.batt_min + k.batt_len / 2:.0f}-{L.batt_max - k.batt_len / 2:.0f} mm) |",
        "",
        f"Analytic structure estimate for comparison: {est:.0f} g "
        f"(CAD printed + rods/tubes: {sum(i[1] for i in items if i[0] in PRINT or i[0].startswith(('tube', 'spar'))):.0f} g).",
        "",
        "## Mass breakdown",
        "",
        "| item | g | x mm |",
        "|---|---|---|",
    ]
    for name, g, x, _ in sorted(items, key=lambda i: i[2]):
        lines.append(f"| {name} | {g:.1f} | {x:.0f} |")
    lines += ["", "## Print list", "",
              "| file | material | est. g | on the bed, mm | orientation |", "|---|---|---|---|---|"]
    print_list = []
    for name, shape in parts.items():
        bb = to_bed(shape, name).BoundingBox()
        mat = "LW-PLA" if PRINT[name][0] == LW_PLA else "PLA/PETG"
        bed = f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}"
        grams = print_mass(name, shape)
        lines.append(f"| stl/{name}.stl | {mat} | {grams:.1f} | {bed} | {PRINT_NOTES[name]} |")
        print_list.append({"name": name, "material": mat, "grams": round(grams, 1),
                           "bed": bed, "note": PRINT_NOTES[name]})
    lines += ["", report(p, k), ""]
    (out / "REPORT.md").write_text("\n".join(lines))

    from dataclasses import replace
    from design import evaluate, smallest_span
    sweep = []
    for b in range(300, 561, 20):
        _, a, pf = evaluate(replace(p, span=float(b)), k)
        sweep.append({"span": b, "auw": round(a), "stall": round(pf["stall"], 2)})
    spec = {
        "span": p.span, "chord": L.chord, "area_dm2": round(L.area / 1e4, 2),
        "length": round(L.length), "naca": p.naca, "incidence": p.incidence,
        "auw": round(auw), "printed": round(printed), "loading": round(perf["loading"], 1),
        "stall": round(perf["stall"], 1), "cruise": round(perf["cruise"], 1),
        "endurance": round(perf["endurance_min"]), "vh": round(L.vh_actual, 2),
        "vv": round(L.vv_actual, 3), "x_le": L.x_le, "x_te": round(L.x_te, 1),
        "pod_len": round(L.pod_len, 1), "x_stab": round(L.x_stab, 1), "c_h": L.c_h,
        "cg_x": round(cg, 1), "cg_pct": round(p.cg_target * 100), "np_pct": round(np_ * 100),
        "np_x": round(L.x_le + np_ * L.chord, 1), "battery_x": batt_x,
        "battery_len": k.batt_len, "batt_range": [round(L.batt_min + k.batt_len / 2),
                                                  round(L.batt_max - k.batt_len / 2)],
        "tube": f"{p.tube_od:.0f}x{p.tube_id:.0f}", "tube_len": round(L.tube_len),
        "tube_spacing": p.tube_spacing, "b_h": L.b_h, "fin_h": L.fin_h,
        "stall_limit": p.stall_limit, "min_span": smallest_span(p, k), "sweep": sweep,
        "parts": print_list,
        "mass": [{"name": n, "g": round(g, 1), "x": round(x)} for n, g, x, _ in items],
    }
    import json
    (out / "viewer").mkdir(exist_ok=True)
    (out / "viewer" / "spec.json").write_text(json.dumps(spec, indent=1))
    return auw, cg


def main():
    out = HERE
    p, k, L, sec, parts, ref, items, batt_x = build()
    for name, s in parts.items():
        assert s.isValid(), f"{name} is not a valid solid"
    auw, cg = write_report(p, k, L, parts, items, batt_x, out)
    export_all(p, k, L, parts, ref, out)
    print(f"AUW {auw:.1f} g, CG {cg:.1f} mm, battery centre {batt_x:.1f} mm, x_le {L.x_le:.1f}")


if __name__ == "__main__":
    main()
