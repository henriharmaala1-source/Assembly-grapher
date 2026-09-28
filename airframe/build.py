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

import components as parts_lib  # noqa: E402
from design import (LW_PLA, PLA, Kit, Layout, Params, camber,  # noqa: E402
                    naca4, neutral_point, performance, report, rod_mass,
                    solve_x_le, structure_estimate)
from geom import (box, cut, cyl_x, cyl_y, cyl_z, fuse, prism_y, prism_z,  # noqa: E402
                  rod, shaft_along_y)

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


def nose_cuts(p: Params, L: Layout, off: float, ch: float):
    """Tools that round the pod's front edges, offset inward by `off` (0 for the
    outside, the wall for the cavity). The two vertical edges get radius
    nose_r; along the bottom the 45 deg belly chamfer (size ch) blends into the
    front face with the same radius, so printed upright nothing overhangs more
    than 45 deg. The blend stays above the bed for nose_r < ch / (1 - 1/sqrt 2)."""
    r, ro, hw, zb = p.nose_r, p.nose_r - off, L.half_w, L.z_bottom
    tools = []
    if ro > 0:
        for s in (-1, 1):
            yc = s * (hw - r)
            corner = box(-1, r, min(yc, s * (hw + 1)), max(yc, s * (hw + 1)), zb - 1, 30)
            tools.append(cut(corner, cyl_z(ro, zb - 2, 31, r, yc)))
    zl = zb + ch + off * math.sqrt(2)                       # chamfer line x + z = zl
    wp = cq.Workplane("XZ")
    if ro > 0:
        zc, a = zb + ch + r * (math.sqrt(2) - 1), math.radians(202.5)
        wp = (wp.moveTo(-5, zc).lineTo(off, zc)
              .threePointArc((r + ro * math.cos(a), zc + ro * math.sin(a)),
                             (r - ro / math.sqrt(2), zc - ro / math.sqrt(2))))
    else:
        wp = wp.moveTo(-5, zl - off).lineTo(off, zl - off)
    tools.append(wp.lineTo(zl - zb + 5, zb - 5).lineTo(-5, zb - 5).close()
                 .extrude(hw + 10, both=True).val())
    return tools


def make_pod(p: Params, L: Layout, batt_x: float):
    hw, zb, Lp = L.half_w, L.z_bottom, L.pod_len
    ty, rh, rb = L.tube_y, L.r_hole, L.r_boss
    w, fw, ch = p.wall, p.front_wall, 4.0
    zf = zb + w                                             # floor top

    outer = (cq.Workplane().box(Lp, 2 * hw, -zb, centered=(False, True, False))
             .translate((0, 0, zb)).edges("|X and <Z").chamfer(ch)).val()
    ch_in = ch + w * math.sqrt(2) - 2 * w                   # keeps the wall even
    inner = (cq.Workplane().box(Lp - fw - w, 2 * (hw - w), 20 - zf, centered=(False, True, False))
             .translate((fw, 0, zf)).edges("|X and <Z").chamfer(ch_in)).val()
    round_front = nose_cuts(p, L, 0.0, ch)
    pod = cut(cut(outer, *round_front), cut(inner, *nose_cuts(p, L, w, ch)))

    rounded = p.nose_r > 0

    def sleeve(x0, y, nose=0.0):
        s = cq.Workplane("YZ").circle(rb).extrude(p.sleeve_len)
        return (s.faces("<X").edges().fillet(nose) if nose else s).val().translate(V(x0, y, 0))
    sleeves = ([sleeve(0.0, s * ty, rb - 0.9 if rounded else 0.0) for s in (-1, 1)]  # domed noses
               + [sleeve(Lp - p.sleeve_len, s * ty) for s in (-1, 1)])
    strip = cq.Workplane().box(fw, 2 * ty, rb, centered=(False, True, False))  # between the sleeves
    plate = (cq.Workplane("YZ").moveTo(-13, 0).lineTo(13, 0).lineTo(13, p.motor_z)
             .threePointArc((0, p.motor_z + 13), (-13, p.motor_z)).close().extrude(3.2))  # motor plate
    if rounded:
        strip = strip.edges("|Y and >Z and <X").fillet(2.0)
        plate = plate.faces("<X").edges("not <Z").fillet(1.5)
    strip, plate = strip.val(), plate.val()
    nose = [cut(strip, *round_front), plate,
            cut(cyl_x(13, 0, 3.2, 0, p.motor_z), box(-1, 4, -14, 14, 0, 30))]   # motor boss
    fc = [cyl_z(2.3, zf - 0.1, zf + 3, FC_X + dx, dy) for dx in (-10, 10) for dy in (-10, 10)]
    # elevator SG90 lies on its side behind the battery: two ribs locate the body,
    # stopping short of the mounting tabs
    sv = Kit().servo
    ribs = [box(L.elev_servo_x + sx * (sv.length / 2 + 0.2) - (1.2 if sx < 0 else 0),
                L.elev_servo_x + sx * (sv.length / 2 + 0.2) + (1.2 if sx > 0 else 0),
                -(hw - w) - 0.1, L.elev_servo_base_y + sv.tab_z - 0.5, zf - 0.1, zf + 4)
            for sx in (-1, 1)]
    pod = fuse(pod, *sleeves, *nose, *fc, *ribs)

    holes = [cyl_x(rh, L.tube_x0, Lp + 1, s * ty, 0) for s in (-1, 1)]
    holes += [cyl_z(0.85, zf, zf + 4, FC_X + dx, dy) for dx in (-10, 10) for dy in (-10, 10)]
    holes.append(box(Lp - 3, Lp + 1, L.pushrod_y - 2, L.pushrod_y + 2,
                     L.pushrod_z - 2.5, 1))                    # pushrod over the rear wall
    if p.rudders:
        holes.append(box(Lp - 3, Lp + 1, L.rudder_pushrod_y - 2, L.rudder_pushrod_y + 2,
                         -4.5, 1))                             # rudder pushrod
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
    cw = p.centre_w
    body = sec.centre_solid(-cw / 2, cw / 2, L.z_top)
    ty, rb = L.tube_y, L.r_boss
    # caps that sit over the pod's rear tube sleeves
    c0, c1 = max(L.x_le, L.pod_len - p.sleeve_len), min(L.x_te - 1.0, L.pod_len)
    caps = [cut(box(c0, c1, s * ty - rb - 1.2, s * ty + rb + 1.2, 0, L.z_top + 0.4),
                cyl_x(rb + 0.15, c0 - 1, c1 + 1, s * ty, 0)) for s in (-1, 1)]
    body = fuse(body, *caps)
    holes = []
    x0, x1 = L.pod_len + 0.3, L.x_te - 1.5                  # saddle behind the pod
    if x1 - x0 >= 10:
        saddle = [cyl_x(rb, x0, x1, s * ty, 0) for s in (-1, 1)]
        saddle.append(box(x0, x1, -ty, ty, 2.0, L.z_top + 0.4))
        body = fuse(body, *saddle)
        holes += [cyl_x(L.r_hole, x0 - 1, x1 + 1, s * ty, 0) for s in (-1, 1)]
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, -cw, cw, x, z))
    xw, zw = sec.camber_point(0.45)                          # servo leads into the pod
    holes.append(cyl_y(2.0, -cw, cw, xw, zw))
    holes.append(cyl_z(3.0, L.z_top - 1, zw, xw, 0))
    if p.rudders:
        rs = rudder_servo(p, L, sec, Kit())
        sv = Kit().servo
        ya = -L.rudder_pushrod_y - sv.horn_z                 # as in rudder_servo, then mirrored
        y0f, y1f = -(ya + sv.height + sv.boss_h + sv.spline_h + 3.5), -(ya - 2.5)
        hump = (cq.Workplane().box(sv.tab_span + 8, y1f - y0f, rs["top"] + 1.6 - L.z_top,
                                   centered=(True, False, False))
                .translate((rs["xm"], y0f, L.z_top)).edges(">Z").fillet(2.0)).val()
        body = fuse(body, hump)
        holes += [h.mirror("XZ") for h in rs["pocket"]]
        holes.append(box(x0 - 0.2, x1 + 1, L.rudder_pushrod_y - 1.3, L.rudder_pushrod_y + 1.3,
                         -2, 3.8))                           # rudder pushrod through the saddle
    return cut(body, *holes)


def side_servo(p: Params, sec: Section, k: Kit, ya: float, horn_deg: float = 90.0,
               horn_len: float = None, hole: float = None, z_base: float = None):
    """SG90 lying on its side in a wing pocket: tabs between the spars, shaft
    pointing +y from its base at y = ya, horn hanging below the wing."""
    sv = k.servo
    hole = hole or sv.horn_hole
    xa = sec.camber_point(p.main_spar_pos)[0] + p.main_spar_d / 2 + 0.4
    xb = sec.camber_point(p.rear_spar_pos)[0] - p.rear_spar_d / 2 - 0.4
    if xb - xa < sv.tab_span:
        raise ValueError(f"{sv.name} tabs need {sv.tab_span} mm between the spars, "
                         f"only {xb - xa:.1f} mm at this chord")
    xm = (xa + xb) / 2                                       # body centre
    xs = xm + sv.shaft_x                                     # output shaft
    span = np.linspace(xm - sv.tab_span / 2, xm + sv.tab_span / 2, 16)
    zt = min(sec.upper_z(x) for x in span) - 1.2            # top of the servo
    if z_base is not None:                                   # raised into a fairing
        zt = z_base + sv.width
    zc = zt - sv.width / 2                                   # shaft axis height
    zl = min(min(sec.lower_z(x) for x in span), zt - sv.width) - 3
    top = ya + sv.height
    pocket = [
        box(xm - sv.length / 2 - 0.2, xm + sv.length / 2 + 0.2, ya - 0.2, top + 0.2, zl, zt),
        box(xm - sv.tab_span / 2 - 0.3, xm + sv.tab_span / 2 + 0.3,
            ya + sv.tab_z - 0.2, ya + sv.tab_z + sv.tab_t + 0.2, zl, zt),
        box(xm - 5.5, xs + 7.5, top, top + sv.boss_h + sv.spline_h + 1.2, zl, zt),
        box(xm - sv.length / 2 - 6, xm - sv.length / 2, ya + 2.0, ya + 6.0, zl, zt),   # lead
    ]
    return {"pocket": pocket, "origin": (xm, ya, zc), "horn_deg": horn_deg, "horn_len": horn_len,
            "link": (xs, ya + sv.horn_z, zc - hole), "top": zt, "xm": xm}


def aileron_servo(p: Params, L: Layout, sec: Section, k: Kit):
    """Right aileron SG90 under the panel, shaft pointing at the tip."""
    return side_servo(p, sec, k, p.centre_w / 2 + 3.0)


def rudder_servo(p: Params, L: Layout, sec: Section, k: Kit):
    """Rudder SG90 in the wing centre, between the tubes. Built with the shaft
    pointing +y and mirrored, so it ends up pointing left with its horn at
    y = L.rudder_pushrod_y. It sits high, in a fairing on top of the centre
    section, so its horn clears the elevator servo and pushrod in the pod."""
    ya = -L.rudder_pushrod_y - k.servo.horn_z               # base, before mirroring
    return side_servo(p, sec, k, ya, 90.0, RUDDER_HORN_LEN, p.rudder_servo_hole,
                      z_base=L.z_top + 1.8)


def make_panel(p: Params, L: Layout, sec: Section):
    """Right wing panel (the left one is its mirror image)."""
    y0, y1 = p.centre_w / 2, p.span / 2
    panel = sec.solid(y0, y1)
    holes = []
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, y0 - 1, y1 - 5, x, z))
    xw, zw = sec.camber_point(0.45)
    holes.append(cyl_y(2.0, y0 - 1, y0 + 8, xw, zw))
    holes += aileron_servo(p, L, sec, Kit())["pocket"]
    xh = sec.x_at(p.hinge_pos)
    holes.append(box(xh - 0.3, L.x_te + 5, y0 + p.aileron_root_gap, y1 - p.aileron_tip_gap, -20, 40))
    return cut(panel, *holes)


def make_aileron(p: Params, L: Layout, sec: Section):
    y0, y1 = p.centre_w / 2, p.span / 2
    xh = sec.x_at(p.hinge_pos) + 0.3
    ail = sec.solid(y0 + p.aileron_root_gap + 0.5, y1 - p.aileron_tip_gap - 0.5)
    ail = ail.intersect(box(xh, L.x_te + 5, -y1, y1, -20, 40))
    zl, zu = sec.lower_z(xh), sec.upper_z(xh)
    bevel = prism_y([(xh - 0.1, zl - 1), (xh - 0.1, zu - 1.0), (xh + 2.5, zl - 1)], -y1, y1)
    ly = aileron_servo(p, L, sec, Kit())["link"][1]
    slot = box(xh + 0.4, xh + 9.8, ly - 0.75, ly + 0.75, zl - 2, sec.lower_z(xh + 5) + 1.6)
    return cut(ail, bevel, slot)


def make_tail_mount(p: Params, L: Layout):
    x0, x1 = L.x_stab + 0.5, L.x_stab + L.c_fix - 0.5
    r_out = L.r_hole + 1.2
    body = fuse(box(x0, x1, -L.tube_y - r_out, L.tube_y + r_out, L.tail_z - 1.4, L.tail_z),
                *[cyl_x(r_out, x0, x1, s * L.tube_y, 0) for s in (-1, 1)],
                box(x0 + 2.5, x0 + 10.5, L.pushrod_y - 1.5, L.pushrod_y + 1.5,
                    L.pushrod_z - 2.5, L.tail_z - 1.2))
    holes = [cyl_x(L.r_hole, x0 - 1, L.tube_x1 + 0.2, s * L.tube_y, 0) for s in (-1, 1)]
    holes.append(cyl_x(0.8, x0, x0 + 12, L.pushrod_y, L.pushrod_z))   # 1 mm pushrod guide
    if p.rudders:                                            # bellcrank pivot, M2 from below
        body = fuse(body, cyl_z(2.8, L.joiner_z + 1.0, L.tail_z - 1.3, L.bellcrank_x, 0))
        holes.append(cyl_z(0.8, L.joiner_z, L.joiner_z + 7, L.bellcrank_x, 0))
    return cut(body, *holes)


def make_bellcrank(p: Params, L: Layout):
    """90 degree bellcrank under the tail mount: the rudder pushrod pulls the
    left arm, the forward arm drives the joiner wires to both rudders."""
    bx, z0, z1 = L.bellcrank_x, L.joiner_z - 0.8, L.joiner_z + 0.8
    iy, ox = L.rudder_pushrod_y, L.x_hinge + p.rudder_horn
    body = fuse(cyl_z(3.4, z0, z1, bx, 0), cyl_z(2.4, z0, z1, bx, iy), cyl_z(2.4, z0, z1, ox, 0),
                box(bx - 2.2, bx + 2.2, iy, 0, z0, z1), box(bx, ox, -2.2, 2.2, z0, z1))
    return cut(body, cyl_z(1.1, z0 - 1, z1 + 1, bx, 0),
               cyl_z(0.55, z0 - 1, z1 + 1, bx, iy), cyl_z(0.55, z0 - 1, z1 + 1, ox, 0))


def make_stab(p: Params, L: Layout):
    s = (cq.Workplane().box(L.c_fix, L.b_h - 1.0, p.plate, centered=(False, True, False))
         .translate((L.x_stab, 0, L.tail_z)).faces("<X").edges("|Y").fillet(p.plate * 0.45)).val()
    return s


def make_elevator(p: Params, L: Layout):
    x0, x1 = L.x_stab + L.c_fix + 0.6, L.x_stab + L.c_h
    y = L.b_h / 2 - L.elevator_inset
    z0, z1 = L.tail_z, L.tail_z + p.plate
    el = box(x0, x1, -y, y, z0, z1)
    hy = L.pushrod_y
    horn = prism_y([(x0 + 0.5, z0 + 0.1), (x0 + 8, z0 + 0.1), (x0 + 4, L.pushrod_z - 2.2),
                    (x0 + 0.5, L.pushrod_z - 2.2)], hy - 0.8, hy + 0.8)
    el = fuse(el, horn)
    bevel = [(x0 - 0.1, z0 - 0.1), (x0 - 0.1, z1 - 0.6), (x0 + 2.0, z0 - 0.1)]
    cuts = [prism_y(bevel, -y - 1, hy - 0.8), prism_y(bevel, hy + 0.8, y + 1),
            prism_y([(x1 - 7, z0 - 0.1), (x1 + 0.1, z0 - 0.1), (x1 + 0.1, z0 + 1.3)], -y - 1, y + 1),
            cyl_y(0.6, hy - 2, hy + 2, x0 + 3.0, L.pushrod_z)]
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
    if p.rudders:                                            # rudder takes everything aft of the hinge
        return cut(fuse(fin, *jaws), box(L.x_hinge - 0.3, x_te + 1, y0 - 1, y0 + p.plate + 1, z0 - 1, z1 + 1))
    return cut(fuse(fin, *jaws), te)


def make_rudder(p: Params, L: Layout):
    """Right rudder: the fin aft of the hinge line, with a horn tab below it for the
    joiner wire. Tape hinge on the outboard face."""
    y0, t = L.b_h / 2, p.plate
    z0, z1 = L.tail_z - p.fin_below, L.tail_z + p.plate + L.fin_above
    xh, x_te, zj = L.x_hinge, L.x_stab + L.c_h, L.joiner_z
    xr = xh + 0.3
    xo = xh + p.rudder_horn                                  # joiner hole
    rudder = fuse(
        box(xr, x_te, y0, y0 + t, z0, z1),
        prism_y([(xr, z0 + 0.5), (xo + 5, z0 + 0.5), (xo + 2.5, zj - 1.6), (xr, zj - 1.6)],
                y0, y0 + t),
        box(xo - 2.5, xo + 2.5, y0 - 4.0, y0 + 0.1, zj - 1.6, zj + 1.6))   # flange for the wire
    cuts = [prism_z([(xr - 0.1, y0 - 0.1), (xr + 2.0, y0 - 0.1), (xr - 0.1, y0 + t - 0.6)], z0 - 1, z1 + 1),
            prism_z([(x_te - 7, y0 - 0.1), (x_te + 0.1, y0 - 0.1), (x_te + 0.1, y0 + 1.3)], z0 - 1, z1 + 1),
            cyl_z(0.55, zj - 3, zj + 3, xo, y0 - 2.0)]
    return cut(rudder, *cuts)


# --------------------------------------------------------------------------
# Bought parts in flight position (for the STEP, the viewer and the CG)

FC_X = 33.0                     # flight controller centre
ELEV_HORN_LEN = 13.5            # elevator servo horn trimmed to clear the wing centre
RUDDER_HORN_LEN = 11.0          # rudder servo horn trimmed to clear the elevator servo
BLACK, WHITE = (0.08, 0.08, 0.09), (0.93, 0.93, 0.92)


class Ref:
    """A non-printed item: shape, colour, mass and which assembly group it joins."""

    def __init__(self, name, shape, colour, mass, group):
        self.name, self.shape, self.colour, self.mass, self.group = name, shape, colour, mass, group


def bought_parts(p: Params, k: Kit, L: Layout, sec: Section, batt_x: float):
    zf, sv = L.z_floor, k.servo
    out = []

    def add(name, shapes, colours, mass, group):
        """shapes: {suffix: shape} from components.py; mass goes on the main body."""
        for suffix, shape in shapes.items():
            out.append(Ref(name + ("_" + suffix if suffix else ""), shape,
                           colours.get(suffix, colours[""]), mass if not suffix else 0.0, group))

    # structure
    tube = cut(cyl_x(p.tube_od / 2, L.tube_x0, L.tube_x1, -L.tube_y, 0),
               cyl_x(p.tube_id / 2, L.tube_x0 - 1, L.tube_x1 + 1, -L.tube_y, 0))
    tube_g = rod_mass(p.tube_od, L.tube_len, p.tube_id, p.tube_density)
    out += [Ref("tube_L", tube, BLACK, tube_g, "hardware"),
            Ref("tube_R", tube.mirror("XZ"), BLACK, tube_g, "hardware")]
    for pos, d, name in ((p.main_spar_pos, p.main_spar_d, "spar_main"),
                         (p.rear_spar_pos, p.rear_spar_d, "spar_rear")):
        x, z = sec.camber_point(pos)
        out.append(Ref(name, cyl_y(d / 2, -p.span / 2 + 5, p.span / 2 - 5, x, z), BLACK,
                       rod_mass(d, p.span - 10), "hardware"))

    # propulsion
    add("motor", {n: s.translate(V(0, 0, p.motor_z)) for n, s in parts_lib.motor_1404().items()},
        {"": (0.62, 0.64, 0.68), "shaft": (0.25, 0.25, 0.28)}, k.motor, "electronics")
    add("prop", {n: s.translate(V(-14.8, 0, p.motor_z)) for n, s in parts_lib.prop_4x25().items()},
        {"": (0.10, 0.42, 0.85)}, k.prop, "electronics")

    # pod contents
    add("camera", {n: s.translate(V(3.0, 0, -9.0)) for n, s in parts_lib.nano_camera().items()},
        {"": (0.10, 0.10, 0.11), "lens": (0.20, 0.35, 0.55)}, k.cam_vtx * 0.55, "electronics")
    vtx = box(15.6, 18.6, -10, 10, zf + 0.4, zf + 19.4)
    out.append(Ref("vtx", vtx, (0.12, 0.40, 0.25), k.cam_vtx * 0.45, "electronics"))
    ant_x, ant_y = p.sleeve_len + 0.3 + 8, -9
    add("antenna", {n: s.translate(V(ant_x, ant_y, -4.0)) for n, s in parts_lib.whip_antenna(38).items()},
        {"": BLACK}, k.antenna, "electronics")
    add("fc", {n: s.translate(V(FC_X, 0, zf + 3)) for n, s in parts_lib.flight_controller().items()},
        {"": (0.10, 0.12, 0.14), "chips": (0.55, 0.58, 0.62)}, k.fc, "electronics")
    out.append(Ref("receiver", box(37, 47, 16.2, 19.2, zf + 4, zf + 14), (0.22, 0.22, 0.25),
                   k.rx, "electronics"))
    out.append(Ref("esc", box(22, 42, -18.8, -14.8, zf + 3, zf + 13), (0.45, 0.25, 0.65),
                   k.esc, "electronics"))
    add("battery", {n: s.translate(V(batt_x, 0, zf + k.batt_h / 2 + 0.1))
                    for n, s in parts_lib.battery_2s(k.batt_len, k.batt_w, k.batt_h).items()},
        {"": (0.88, 0.74, 0.16), "leads": (0.75, 0.20, 0.15)}, k.battery, "electronics")

    # servos
    servo_colours = {"": (0.16, 0.38, 0.85), "horn": WHITE}
    el_origin = (L.elev_servo_x, L.elev_servo_base_y, zf + sv.width / 2)
    trimmed = parts_lib.sg90(sv, -90, horn_len=ELEV_HORN_LEN)
    add("servo_elev", {n: shaft_along_y(s, el_origin) for n, s in trimmed.items()},
        servo_colours, sv.mass, "electronics")
    hx, _, _ = parts_lib.horn_point(sv, -90)
    el_link = (L.elev_servo_x + hx, L.elev_servo_base_y + sv.horn_z, zf + sv.width / 2 + sv.horn_hole)
    x_eh = L.x_stab + L.c_fix + 0.6 + 3.0                    # elevator horn hole
    out.append(Ref("pushrod_elev", rod(0.5, el_link, (x_eh - 1.5, L.pushrod_y, L.pushrod_z)), BLACK,
                   rod_mass(1.0, x_eh - el_link[0]), "hardware"))

    if p.rudders:
        rs = rudder_servo(p, L, sec, k)
        servo = parts_lib.sg90(sv, rs["horn_deg"], horn_len=rs["horn_len"])
        add("servo_rud", {n: shaft_along_y(s, rs["origin"]).mirror("XZ") for n, s in servo.items()},
            {"": (0.16, 0.38, 0.85), "horn": WHITE}, sv.mass, "electronics")
        lx, ly, lz = rs["link"]
        start = (lx, -ly, lz)
        crank_in = (L.bellcrank_x, L.rudder_pushrod_y, L.joiner_z)
        out.append(Ref("pushrod_rud", rod(0.5, start, crank_in), BLACK,
                       rod_mass(1.0, crank_in[0] - start[0]), "hardware"))
        crank_out = (L.x_hinge + p.rudder_horn, 0, L.joiner_z)
        for side, sgn in (("R", 1), ("L", -1)):
            horn = (L.x_hinge + p.rudder_horn, sgn * (L.b_h / 2 - 2.0), L.joiner_z)
            out.append(Ref(f"joiner_{side}", rod(0.4, crank_out, horn), BLACK,
                           rod_mass(0.8, L.b_h / 2, rho=7.8), "hardware"))

    ail = aileron_servo(p, L, sec, k)
    servo = {n: shaft_along_y(s, ail["origin"]) for n, s in parts_lib.sg90(sv, ail["horn_deg"]).items()}
    xh = sec.x_at(p.hinge_pos) + 0.3
    zl = sec.lower_z(xh + 1.8)
    ly = ail["link"][1]
    horn = parts_lib.micro_horn()[""].translate(V(xh + 0.3, ly, zl + 0.2))
    rod_r = rod(0.4, ail["link"], (xh + 2.1, ly, zl - 7.8))
    for side, m in (("R", lambda s: s), ("L", lambda s: s.mirror("XZ"))):
        add(f"servo_ail_{side}", {n: m(s) for n, s in servo.items()}, servo_colours, sv.mass,
            "electronics")
        out.append(Ref(f"horn_ail_{side}", m(horn), WHITE, 0.3, "hardware"))
        out.append(Ref(f"pushrod_ail_{side}", m(rod_r), BLACK, 0.2, "hardware"))
    return out


# --------------------------------------------------------------------------
# Mass properties


# name: (density g/cm^3, shell thickness mm, infill fraction, colour)
PRINT = {
    "pod":         (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "lid":         (PLA, 0.8, 0.15, (0.28, 0.30, 0.34)),
    "wing_centre": (LW_PLA, 0.5, 0.05, (0.86, 0.87, 0.84)),
    "wing_R":      (LW_PLA, 0.45, 0.0, (0.93, 0.93, 0.90)),
    "wing_L":      (LW_PLA, 0.45, 0.0, (0.93, 0.93, 0.90)),
    "aileron_R":   (LW_PLA, 0.45, 0.0, (0.96, 0.45, 0.10)),
    "aileron_L":   (LW_PLA, 0.45, 0.0, (0.96, 0.45, 0.10)),
    "tail_mount":  (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "stab":        (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "elevator":    (LW_PLA, 0.4, 0.15, (0.96, 0.45, 0.10)),
    "fin_R":       (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "fin_L":       (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "rudder_R":    (PLA, 0.4, 0.15, (0.96, 0.45, 0.10)),
    "rudder_L":    (PLA, 0.4, 0.15, (0.96, 0.45, 0.10)),
    "bellcrank":   (PLA, 0.8, 0.5, (0.20, 0.22, 0.25)),
}
PRINT_NOTES = {
    "pod": "Upright, open top up. 2 walls, 15 % infill.",
    "lid": "Flat, lips up.",
    "wing_centre": "On its side, spar holes vertical. 0.5 mm walls, 5 % infill.",
    "wing_R": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "wing_L": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "aileron_R": "Flat on the lower surface. 1 wall, 0 % infill.",
    "aileron_L": "Flat on the lower surface. 1 wall, 0 % infill.",
    "tail_mount": "Plate face down.",
    "stab": "Flat. 2 top / 2 bottom layers, 15 % infill.",
    "elevator": "Top face down, horn up.",
    "fin_R": "Outer face down, jaws up.",
    "fin_L": "Outer face down, jaws up.",
    "rudder_R": "Outer face down, wire flange up. 2 top / 2 bottom layers, 15 % infill.",
    "rudder_L": "Outer face down, wire flange up. 2 top / 2 bottom layers, 15 % infill.",
    "bellcrank": "Flat. Solid.",
}
def print_mass(name, shape):
    rho, shell, infill, _ = PRINT[name]
    vol, area = shape.Volume(), shape.Area()
    solid = min(vol, area * shell)
    return rho * (solid + infill * max(0.0, vol - solid)) / 1000


# --------------------------------------------------------------------------
# Print orientation


def to_bed(shape, name):
    s = shape
    if name == "lid" or name == "tail_mount" or name == "elevator":
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    elif name in ("wing_centre", "wing_R", "fin_L", "rudder_L"):
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    elif name in ("wing_L", "fin_R", "rudder_R"):
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), -90)
    elif name.startswith("aileron"):
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), -Params().incidence)
    bb = s.BoundingBox()
    return s.translate(V(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


# --------------------------------------------------------------------------
# Build


def build(p: Params = Params(), k: Kit = Kit()):
    """Place the wing so the battery balances the plane mid-way in its travel,
    using CAD masses (secant steps from the analytic first guess)."""
    def miss(x_le):
        r = build_at(p, k, x_le)
        L, batt_x = r[2], r[-1]
        return batt_x - (L.batt_min + L.batt_max) / 2, r

    x0 = solve_x_le(p, k)
    f0, r = miss(x0)
    x1 = x0 + 3.0
    for _ in range(4):
        if abs(f0) < 0.5:
            break
        f1, r1 = miss(x1)
        x0, x1, f0, r = x1, x1 - f1 * (x1 - x0) / (f1 - f0), f1, r1
    return r


def build_at(p: Params, k: Kit, x_le: float):
    x_le = round(x_le, 1)
    L = Layout(p, x_le, k)
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
    if p.rudders:
        parts["rudder_R"] = make_rudder(p, L)
        parts["rudder_L"] = parts["rudder_R"].mirror("XZ")
        parts["bellcrank"] = make_bellcrank(p, L)
    parts["lid"] = make_lid(p, L)

    # CG with CAD masses: solve the battery station, then build the pod around it
    parts["pod"] = make_pod(p, L, batt_nominal)
    refs = bought_parts(p, k, L, sec, batt_nominal)

    def mass_items():
        items = []
        for name, shape in parts.items():
            c = shape.Center()
            items.append((name, print_mass(name, shape), c.x, c.z))
        for r in refs:
            if r.mass > 0:
                c = r.shape.Center()
                items.append((r.name, r.mass, c.x, c.z))
        items += [("wiring", k.wiring, 0.5 * L.pod_len, -8.0),
                  ("hardware", k.hardware, 0.55 * L.x_stab, 0.0)]
        return items

    others = [i for i in mass_items() if i[0] != "battery"]
    target = L.x_le + p.cg_target * L.chord
    m = sum(i[1] for i in others)
    batt_x = round((target * (m + k.battery) - sum(i[1] * i[2] for i in others)) / k.battery, 1)
    parts["pod"] = make_pod(p, L, batt_x)
    refs = bought_parts(p, k, L, sec, batt_x)
    return p, k, L, sec, parts, refs, mass_items(), batt_x


# --------------------------------------------------------------------------
# Export


def tessellate(shape, tol=0.08, ang=0.25):
    vs, tris = shape.tessellate(tol, ang)
    return np.array([(v.x, v.y, v.z) for v in vs]), np.array(tris, dtype=np.int64)


def srgb_to_linear(rgb):
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb]


def export_all(p, k, L, parts, refs, out: Path):
    (out / "stl").mkdir(parents=True, exist_ok=True)
    (out / "cad").mkdir(parents=True, exist_ok=True)
    for name, shape in parts.items():
        cq.exporters.export(to_bed(shape, name), str(out / "stl" / f"{name}.stl"),
                            tolerance=0.03, angularTolerance=0.15)

    bodies = [(n, s, PRINT[n][3], "printed") for n, s in parts.items()]
    bodies += [(r.name, r.shape, r.colour, r.group) for r in refs]
    assy = cq.Assembly(name=f"kipina_{p.span:.0f}")
    groups = {g: cq.Assembly(name=g) for g in ("printed", "hardware", "electronics")}
    for name, shape, colour, group in bodies:
        groups[group].add(shape, name=name, color=cq.Color(*colour))
    for g in groups.values():
        assy.add(g)
    assy.export(str(out / "cad" / "airframe.step"))

    import trimesh
    from trimesh.visual.material import PBRMaterial
    scene = trimesh.Scene()
    for name, shape, colour, group in bodies:
        v, t = tessellate(shape, 0.05, 0.2)
        v = np.column_stack([v[:, 0], v[:, 2], -v[:, 1]]) / 1000.0  # z-up mm -> y-up m
        mesh = trimesh.Trimesh(v, t, process=False)
        mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=name, baseColorFactor=[*srgb_to_linear(colour), 1.0],
            metallicFactor=0.0, roughnessFactor=0.6))
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    (out / "viewer").mkdir(exist_ok=True)
    scene.export(str(out / "viewer" / "airframe.glb"), include_normals=True)


def fits_bed(p: Params, shape, name) -> bool:
    """Does the print-oriented part fit the build volume (footprint may turn 90 deg)?"""
    bb = to_bed(shape, name).BoundingBox()
    bx, by, bz = p.bed
    flat = (bb.xlen <= bx and bb.ylen <= by) or (bb.xlen <= by and bb.ylen <= bx)
    return flat and bb.zlen <= bz


def write_report(p, k, L, parts, items, batt_x, out: Path, hits=()):
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
        f"| Stall / cruise | {perf['stall']:.2f} / {perf['cruise']:.1f} m/s |",
        f"| Top speed (level, estimate) | {perf['top']:.1f} m/s = {perf['top'] * 3.6:.0f} km/h "
        f"(pitch speed {perf['pitch_speed'] * 3.6:.0f} km/h, CdA {perf['cda'] * 1e4:.1f} cm^2) |",
        f"| Endurance (rough) | ~{perf['endurance_min']:.0f} min on 2S 450 mAh |",
        f"| Current (rough) | cruise ~{perf['cruise_a']:.1f} A ({perf['cruise_w']:.0f} W), "
        f"full throttle at 1:1 thrust ~{perf['full_a']:.0f} A |",
        f"| Thrust target | >= {auw:.0f} g static (1:1) |",
        f"| Tail volumes | Vh {L.vh_actual:.2f}, Vv {L.vv_actual:.3f} |",
        f"| CG target | {p.cg_target * 100:.0f} % chord = **{L.x_le + p.cg_target * L.chord:.1f} mm** "
        f"from the motor face ({p.cg_target * L.chord:.1f} mm behind the wing LE) |",
        f"| CG (computed) | x {cg:.1f} mm, z {cgz:.1f} mm |",
        f"| Neutral point | {np_ * 100:.0f} % chord -> static margin {(np_ - p.cg_target) * 100:.0f} % |",
        f"| Battery centre | **{batt_x:.0f} mm** from the motor face "
        f"(travel {L.batt_min + k.batt_len / 2:.0f}-{L.batt_max - k.batt_len / 2:.0f} mm) |",
        "",
        f"Sizing check with CAD weights: stall {perf['stall']:.2f} m/s against the "
        f"{p.stall_limit} m/s limit: **{'passes' if perf['stall'] <= p.stall_limit else 'FAILS'}**.",
        "",
        "Interference check (bought parts against everything, pushrod-in-horn links "
        "excluded): " + ("**" + "; ".join(f"{a} / {b} {v} mm^3" for a, b, v in hits) + "**"
                         if hits else "no overlaps."),
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
    bed = " x ".join(f"{v:.0f}" for v in p.bed)
    lines += ["", "## Print list", "",
              f"Build volume checked: {bed} mm (Bambu Lab A1 mini).", "",
              "| file | material | est. g | on the bed, mm | fits | orientation |",
              "|---|---|---|---|---|---|"]
    print_list = []
    for name, shape in parts.items():
        bb = to_bed(shape, name).BoundingBox()
        mat = "LW-PLA" if PRINT[name][0] == LW_PLA else "PLA/PETG"
        dims = f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}"
        grams = print_mass(name, shape)
        fits = fits_bed(p, shape, name)
        lines.append(f"| stl/{name}.stl | {mat} | {grams:.1f} | {dims} | {'yes' if fits else '**no**'} | "
                     f"{PRINT_NOTES[name]} |")
        print_list.append({"name": name, "material": mat, "grams": round(grams, 1),
                           "bed": dims, "fits": fits, "note": PRINT_NOTES[name]})
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
        "top": round(perf["top"], 1), "pitch_speed": round(perf["pitch_speed"], 1),
        "endurance": round(perf["endurance_min"]), "vh": round(L.vh_actual, 2),
        "cruise_a": round(perf["cruise_a"], 1), "full_a": round(perf["full_a"], 1),
        "vv": round(L.vv_actual, 3), "x_le": L.x_le, "x_te": round(L.x_te, 1),
        "pod_len": round(L.pod_len, 1), "x_stab": round(L.x_stab, 1), "c_h": L.c_h,
        "cg_x": round(cg, 1), "cg_pct": round(p.cg_target * 100), "np_pct": round(np_ * 100),
        "np_x": round(L.x_le + np_ * L.chord, 1), "battery_x": batt_x,
        "battery_len": k.batt_len, "batt_range": [round(L.batt_min + k.batt_len / 2),
                                                  round(L.batt_max - k.batt_len / 2)],
        "tube": f"{p.tube_od:.0f}x{p.tube_id:.0f}", "tube_len": round(L.tube_len),
        "tube_spacing": p.tube_spacing, "b_h": L.b_h, "fin_h": L.fin_h,
        "stall_limit": p.stall_limit, "min_span": smallest_span(p, k), "sweep": sweep,
        "bed": list(p.bed), "centre_w": p.centre_w,
        "parts": print_list,
        "mass": [{"name": n, "g": round(g, 1), "x": round(x)} for n, g, x, _ in items],
    }
    import json
    (out / "viewer").mkdir(exist_ok=True)
    (out / "viewer" / "spec.json").write_text(json.dumps(spec, indent=1))
    return auw, cg


# pushrods are meant to pass through their horns
LINKS = {frozenset(pair) for pair in (
    ("pushrod_elev", "elevator"), ("pushrod_elev", "servo_elev_horn"),
    ("pushrod_ail_R", "servo_ail_R_horn"), ("pushrod_ail_R", "horn_ail_R"),
    ("pushrod_ail_L", "servo_ail_L_horn"), ("pushrod_ail_L", "horn_ail_L"),
    ("pushrod_rud", "servo_rud_horn"), ("pushrod_rud", "bellcrank"),
    ("joiner_R", "bellcrank"), ("joiner_L", "bellcrank"),
    ("joiner_R", "rudder_R"), ("joiner_L", "rudder_L"), ("joiner_R", "joiner_L"))}


def component(name):
    """servo_ail_R_horn -> servo_ail_R: sub-bodies of one bought part."""
    for suffix in ("_horn", "_shaft", "_lens", "_chips", "_leads"):
        if name.endswith(suffix) and not name.startswith("horn_"):
            return name[: -len(suffix)]
    return name


def interference(parts, refs, tol=0.05):
    """Pairs of bodies that overlap by more than tol mm^3."""
    bodies = [(n, s) for n, s in parts.items()] + [(r.name, r.shape) for r in refs]
    boxes = {n: s.BoundingBox() for n, s in bodies}
    hits = []
    for i, (a, sa) in enumerate(bodies):
        for b, sb in bodies[i + 1:]:
            if a in parts and b in parts:
                continue                                   # printed parts are glued, not checked
            if component(a) == component(b) or frozenset((a, b)) in LINKS:
                continue
            ba, bb = boxes[a], boxes[b]
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                    or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            v = sa.intersect(sb).Volume()
            if v > tol:
                hits.append((a, b, round(v, 2)))
    return hits


def main():
    out = HERE
    p, k, L, sec, parts, refs, items, batt_x = build()
    for name, s in parts.items():
        assert s.isValid(), f"{name} is not a valid solid"
    for name, shape in parts.items():
        if not fits_bed(p, shape, name):
            print(f"WARNING: {name} does not fit the {p.bed} mm build volume")
    hits = interference(parts, refs)
    for a, b, v in hits:
        print(f"WARNING: {a} overlaps {b} by {v} mm^3")
    auw, cg = write_report(p, k, L, parts, items, batt_x, out, hits)
    export_all(p, k, L, parts, refs, out)
    print(f"AUW {auw:.1f} g, CG {cg:.1f} mm, battery centre {batt_x:.1f} mm, x_le {L.x_le:.1f}")


if __name__ == "__main__":
    main()
