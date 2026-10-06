#!/usr/bin/env python3
"""Build the twin-boom micro FPV pusher in CadQuery and export everything.

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
from design import (LW_PLA, PLA, Kit, Layout, Params, airfoil_file,  # noqa: E402
                    boattail_taper, boom_options, camber, motor_wire_mass, naca4, neutral_point,
                    performance, pushrod_mass, report, rod_mass, single_boom, solve_x_le,
                    structure_estimate)
from geom import (ball, box, cut, cyl_x, cyl_y, cyl_z, fuse, prism_x, prism_y, prism_z,  # noqa: E402
                  rod, shaft_along_y, wire_path)

# --------------------------------------------------------------------------
# Wing section


def airfoil_label(p: Params) -> str:
    return Path(p.airfoil).stem if p.airfoil else f"NACA {p.naca}"


class Section:
    """Wing section (NACA 4-digit, or a .dat file) at the wing's incidence,
    placed on the pod top."""

    def __init__(self, p: Params, L: Layout):
        self.p, self.L = p, L
        c = L.chord
        self.th = math.radians(p.incidence)
        up, lo = airfoil_file(p.airfoil) if p.airfoil else naca4(p.naca, 70, p.te_min / c)
        self._cu = sorted(up)
        self._cl = sorted(lo)
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

    def camber(self, xf):
        """Mean line at xf (chord fraction): the NACA formula, or from a .dat
        section's own surfaces."""
        if not self.p.airfoil:
            return camber(self.p.naca, xf)[0]
        u = np.interp(xf, [q[0] for q in self._cu], [q[1] for q in self._cu])
        lo = np.interp(xf, [q[0] for q in self._cl], [q[1] for q in self._cl])
        return float((u + lo) / 2)

    def camber_point(self, xf):
        x, z = self._rot(xf, self.camber(xf))
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
    rt = p.nose_top_r                                       # top front edge: a short hood over the camera
    if rt > 0 and rt - off > 0:
        zt = L.z_top
        corner = box(-1, rt, -(hw + 1), hw + 1, zt - rt, zt + 6)   # up past the open top for the cavity
        tools.append(cut(corner, cyl_y(rt - off, -(hw + 2), hw + 2, rt, zt - rt)))
    return tools


def taper_cuts(p: Params, L: Layout, off: float):
    """Tools for the boattail: the sides step in by boattail_deg over the last
    `boattail` mm, the belly as far as the motor boss allows; `off` as in nose_cuts."""
    side, belly = boattail_taper(p, L)
    if side <= 0:
        return []
    hw, zb, zt, Lp = L.half_w, L.z_bottom, L.z_top, L.pod_len
    x0, x1 = Lp - p.boattail, Lp + 5
    tools = [prism_z([(x0, s * (hw - off)), (x1, s * (hw - off - (x1 - x0) * side / p.boattail)),
                      (x1, s * (hw + 10)), (x0, s * (hw + 10))], zb - 10, zt + 10) for s in (1, -1)]
    if belly > 0:
        tools.append(prism_y([(x0, zb + off), (x1, zb + off + (x1 - x0) * belly / p.boattail),
                              (x1, zb - 10), (x0, zb - 10)], -hw - 10, hw + 10))
    return tools


def make_pod(p: Params, L: Layout, batt_x: float):
    """Nacelle under the wing centre: camera in the nose, FC and battery ahead of
    the wing, ESC at the back and the pusher motor on the back wall. The top is
    open: the lid covers it up to the wing, the wing centre from there on."""
    hw, zb, zt, Lp = L.half_w, L.z_bottom, L.z_top, L.pod_len
    w, fw, rw, ch = p.wall, p.front_wall, p.rear_wall, 4.0
    zf = zb + w                                             # floor top

    outer = (cq.Workplane().box(Lp, 2 * hw, zt - zb, centered=(False, True, False))
             .translate((0, 0, zb)).edges("|X and <Z").chamfer(ch)).val()
    ch_in = ch + w * math.sqrt(2) - 2 * w                   # keeps the wall even
    inner = (cq.Workplane().box(Lp - fw - rw, 2 * (hw - w), zt + 5 - zf, centered=(False, True, False))
             .translate((fw, 0, zf)).edges("|X and <Z").chamfer(ch_in)).val()
    pod = cut(cut(outer, *nose_cuts(p, L, 0.0, ch), *taper_cuts(p, L, 0.0)),
              cut(inner, *nose_cuts(p, L, w, ch), *taper_cuts(p, L, w)))

    mz = p.motor_z
    plate = cyl_x(13, Lp - rw, Lp, 0, mz)                   # motor boss; its top stands above the pod
    hp = Kit().fc_holes / 2
    fc = [cyl_z(2.0, zf - 0.1, zf + 3, L.fc_x + dx, dy) for dx in (-hp, hp) for dy in (-hp, hp)]
    ek = Kit()                                              # ESC card between two ribs
    esc_ribs = [box(x0, x0 + 1.0, -9.0, 9.0, zf - 0.1, zf + 6.0)
                for x0 in (L.esc_bay_x, L.esc_x + ek.esc_thk + 0.3)]
    ledges = []                                             # the lid sits flush on these
    for s in (1, -1):
        ya, yb = s * (hw - w - 1.2), s * (hw - w + 0.05)
        ledges.append(box(lid_start(p) - 0.5, lid_end(L), min(ya, yb), max(ya, yb), zt - 2.0, zt - 0.8))
    pod = fuse(pod, plate, *fc, *esc_ribs, *ledges)

    holes = [cyl_z(0.85, zf, zf + 4, L.fc_x + dx, dy) for dx in (-hp, hp) for dy in (-hp, hp)]
    holes.append(cyl_x(3.2, Lp - rw - 1, Lp + 1, 0, mz))    # motor shaft / circlip
    for a in (45, 135, 225, 315):                           # 9x9, 12 mm and 16 mm patterns
        r = (5.9 + 8.6) / 2
        slot = (cq.Workplane("YZ").center(r * math.cos(math.radians(a)), mz + r * math.sin(math.radians(a)))
                .slot2D(8.6 - 5.9 + 2.2, 2.2, a).extrude(6).translate((Lp - rw - 2, 0, 0))).val()
        holes.append(slot)
    if p.boattail > 0:                                      # motor wires, inside the narrower back wall
        holes.append(box(Lp - rw - 1, Lp + 1, -12.5, -7.5, mz - 12, mz - 8))
    else:
        holes.append(box(Lp - rw - 1, Lp + 1, -14.0, -9.0, mz - 13, mz - 9))
    zc = -9.0                                               # nano camera (14 mm) window
    holes.append(box(-1, fw + 1, -7.2, 7.2, zc - 7.2, zc + 7.2))
    for s in (-1, 1):                                       # battery strap slots
        y = s * (Kit().batt_w / 2 + 1.5)
        holes.append(box(batt_x - 12, batt_x + 12, y - 1.0, y + 1.0, zb - 1, zf + 1))
    return cut(pod, *holes)


def pod_split_x(L: Layout) -> float:
    return round(L.batt_max + 10.0)


def split_pod(p: Params, L: Layout, pod):
    """Front and rear halves, each short enough for the bed. A U-shaped tongue on
    the rear half slides 8 mm into the front half for a glued joint."""
    xs, big = pod_split_x(L), 500.0
    front = pod.intersect(box(-10, xs, -big, big, -big, big))
    rear = pod.intersect(box(xs, L.pod_len + 50, -big, big, -big, big))
    hw, w, zf, zt, g = L.half_w, p.wall, L.z_floor, L.z_top, 0.15
    tongue = cut(box(xs - 8, xs + 1, -(hw - w - g), hw - w - g, zf + g, zt - 2.5),
                 box(xs - 9, xs + 2, -(hw - w - g - 0.9), hw - w - g - 0.9, zf + g + 0.9, zt))
    ch_in = 4.0 + w * math.sqrt(2) - 2 * w                  # stay clear of the belly chamfers inside
    corners = [prism_x([(s * (hw - w + 1), zf - 1), (s * (hw - w - ch_in - 1.35), zf - 1),
                        (s * (hw - w + 1), zf + ch_in + 1.35)], xs - 10, xs + 3) for s in (1, -1)]
    return front, fuse(rear, cut(tongue, *corners))


def lid_start(p: Params) -> float:
    """The lid's front edge: behind the front wall, or behind the nose hood."""
    return p.nose_top_r + 0.3 if p.nose_top_r > 0 else p.front_wall + 0.5


def lid_end(L: Layout) -> float:
    """The lid's back edge tucks under the wing's leading edge, just ahead of
    where the wing centre sits down on the pod."""
    return L.x_le + 0.08 * L.chord - 0.5


def make_lid(p: Params, L: Layout):
    """Hatch over the pod ahead of the wing, flush with the pod top, on two
    ledges. Hook the back edge under the wing, then press the front down."""
    x0, x1 = lid_start(p), lid_end(L)
    zt, y = L.z_top, L.half_w - p.wall - 0.2
    lid = box(x0, x1, -y, y, zt - 0.8, zt)
    return cut(lid, cyl_z(1.0, zt - 3, zt + 2, L.ant_x, L.ant_y))   # VTX antenna coax


def make_wing_centre(p: Params, L: Layout, sec: Section):
    """Centre section: the pod is glued under it, the two tail booms plug into
    sockets under it, and the elevator and rudder servos sit in it."""
    cw = p.centre_w
    body = sec.centre_solid(-cw / 2, cw / 2, L.z_top)
    ty, rb, zt = L.tube_y, L.r_boss, L.z_top
    x0, x1 = L.socket_x0, L.socket_x1
    socket = fuse(cq.Workplane("YZ").circle(rb).extrude(x1 - x0).faces("<X").edges().fillet(rb - 1.0).val()
                  .translate(V(x0, ty, 0)), box(x0 + rb, x1, ty - 2.5, ty + 2.5, 0, zt + 0.5))
    rail_end = min(L.x_te - 5, L.pod_len - p.boattail - 1) if p.boattail > 0 else L.x_te - 5
    rails = [box(L.x_le + 10, rail_end, s * L.half_w + (0.2 if s > 0 else -1.4),
                 s * L.half_w + (1.4 if s > 0 else -0.2), zt - 3.0, zt + 0.3) for s in (1, -1)]
    body = fuse(body, socket, socket.mirror("XZ"), *rails)
    holes = [cyl_x(L.r_hole, L.tube_x0, x1 + 1, s * ty, 0) for s in (-1, 1)]
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, -cw, cw, x, z))
    xw, zw = sec.camber_point(0.45)                          # servo leads into the pod
    holes.append(cyl_y(2.0, -cw, cw, xw, zw))
    holes.append(cyl_z(3.0, zt - 1, zw, xw, 0))
    holes += elevator_servo(p, L, sec, Kit())["pocket"]
    if p.rudders:
        holes += [h.mirror("XZ") for h in rudder_servo(p, L, sec, Kit())["pocket"]]
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


def elevator_servo(p: Params, L: Layout, sec: Section, k: Kit):
    """Elevator SG90 in the wing centre, right of the pod: shaft pointing right,
    horn hanging below the wing on the line of the right boom's pushrod."""
    s = side_servo(p, sec, k, L.pushrod_y - k.servo.horn_z)
    return side_servo(p, sec, k, L.pushrod_y - k.servo.horn_z, hole=s["origin"][2] - L.pushrod_z)


def rudder_servo(p: Params, L: Layout, sec: Section, k: Kit):
    """Rudder SG90, the mirror image on the left. Built with the shaft pointing +y
    and mirrored, so its horn ends up on the left pushrod line."""
    s = side_servo(p, sec, k, -L.rudder_pushrod_y - k.servo.horn_z)
    return side_servo(p, sec, k, -L.rudder_pushrod_y - k.servo.horn_z, hole=s["origin"][2] - L.pushrod_z)


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
    """Two collars on the boom ends with pads the stabiliser is glued to, a thin
    strip under the stabiliser's leading edge that ties them together, the
    elevator pushrod guide on the right and the bellcrank pivot on the left."""
    x0, x1 = L.x_stab + 0.5, L.x_stab + L.c_fix - 0.5
    ty, r_out, zt = L.tube_y, L.r_hole + 1.2, L.tail_z
    pad_in = L.pushrod_y - 3.0                               # right pad reaches the pushrod guide
    body = [cyl_x(r_out, x0, x1, s * ty, 0) for s in (-1, 1)]
    body += [box(x0, x1, pad_in, ty + 8, zt - 1.4, zt), box(x0, x1, -ty - 8, -ty + 8, zt - 1.4, zt),
             box(x0, x0 + 6, -ty, ty, zt - 1.0, zt),
             box(x0 + 2.5, x0 + 10.5, L.pushrod_y - 1.5, L.pushrod_y + 1.5, L.pushrod_z - 2.5, zt - 1.2)]
    holes = [cyl_x(L.r_hole, x0 - 1, L.tube_x1 + 0.2, s * ty, 0) for s in (-1, 1)]
    holes.append(cyl_x(0.8, x0, x0 + 12, L.pushrod_y, L.pushrod_z))   # 1 mm pushrod guide
    if p.rudders:                                            # bellcrank pivot, M2 from below
        bx, by = L.bellcrank_x, L.bellcrank_y
        body += [box(bx - 3.5, x1, -ty, by + 3.5, zt - 1.4, zt),
                 cyl_z(2.8, L.joiner_z + 1.0, zt - 1.3, bx, by)]
        holes.append(cyl_z(0.8, L.joiner_z, L.joiner_z + 7, bx, by))
    return cut(fuse(*body), *holes)


def make_bellcrank(p: Params, L: Layout):
    """90 degree bellcrank under the tail mount, near the left boom: the rudder
    pushrod pulls the outer arm, the rear arm drives the joiner wires to both rudders."""
    bx, by, z0, z1 = L.bellcrank_x, L.bellcrank_y, L.joiner_z - 0.8, L.joiner_z + 0.8
    iy, ox = L.rudder_pushrod_y, L.x_hinge + p.rudder_horn
    body = fuse(cyl_z(3.4, z0, z1, bx, by), cyl_z(2.4, z0, z1, bx, iy), cyl_z(2.4, z0, z1, ox, by),
                box(bx - 2.2, bx + 2.2, min(iy, by), max(iy, by), z0, z1), box(bx, ox, by - 2.2, by + 2.2, z0, z1))
    return cut(body, cyl_z(1.1, z0 - 1, z1 + 1, bx, by),
               cyl_z(0.55, z0 - 1, z1 + 1, bx, iy), cyl_z(0.55, z0 - 1, z1 + 1, ox, by))


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
# Drag details (the optimized variant): fairings and joiner sleeves


def streamline(t: float) -> float:
    """Streamlined thickness along a body, nose (0) to tail (1), 1 at its thickest
    (NACA 00xx distribution, closed off a little at both ends)."""
    t = min(max(t, 0.0), 1.0)
    return max(0.08, (0.2969 * math.sqrt(t) - 0.126 * t - 0.3516 * t * t + 0.2843 * t ** 3
                      - 0.1036 * t ** 4) / 0.1002)


def blister(x0, x1, yc, half_w, depth, surface, grow=0.0, n=4.0, seam=0.0, body=None):
    """Blister under a surface: squircle sections (exponent n) of half-width half_w
    and half-depth `depth`, each centred on the surface z = surface(x) it hangs
    from, lofted from nose to tail. With body=(xa, xb) it is full size from xa to
    xb (what it covers), with an elliptic nose ahead and a tail that tapers to x1
    (cut off blunt if there is no room to close it); otherwise it follows a
    streamlined thickness. `seam` turns the start of each section's spline, to
    keep two blisters' seams apart where they are fused."""
    if body:
        xa, xb = body
        blunt = x1 - xb < 18
        xs = [xa - (xa - x0) * math.cos(math.pi / 2 * i / 8) for i in range(9)]
        n_mid = max(1, math.ceil((xb - xa) / 4))
        xs += [xa + (xb - xa) * i / n_mid for i in range(1, n_mid + 1)]
        xs += [xb + (x1 - xb) * i / 6 for i in range(1, 7)] if x1 > xb + 0.5 else []

        def f(x):
            if x <= xa:
                return max(0.12, math.sqrt(max(0.0, 1 - ((xa - x) / (xa - x0)) ** 2)))
            if x <= xb:
                return 1.0
            end = 0.35 if blunt else 0.08
            return end + (1 - end) * 0.5 * (1 + math.cos(math.pi * (x - xb) / (x1 - xb)))
    else:
        xs = [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * i / 16)) for i in range(17)]

        def f(x):
            return streamline((x - x0) / (x1 - x0))
    wires = []
    for x in xs:
        a, b, zc = max(half_w * f(x) + grow, 0.2), max(depth * f(x) + grow, 0.2), surface(x)
        pts = []
        for k in range(48):
            th = seam + 2 * math.pi * k / 48
            c, sn = math.cos(th), math.sin(th)
            pts.append(V(x, yc + a * math.copysign(abs(c) ** (2 / n), c), zc + b * math.copysign(abs(sn) ** (2 / n), sn)))
        wires.append(cq.Wire.assembleEdges([cq.Edge.makeSpline(pts, periodic=True)]))
    return cq.Solid.makeLoft(wires, True)                  # ruled: no overshoot between the sections


FAIRING_WALL = 0.45


def horn_trim(s: dict) -> float:
    """With fairings, the horn arm is cut off 2.5 mm past its pushrod hole."""
    return s["origin"][2] - s["link"][2] + 2.5


def servo_fairing(p: Params, L: Layout, s: dict, k: Kit, surface, above, x_limit, rod_to, clear=()):
    """A single-wall fairing under a side-mounted SG90: a shallow blister over the
    case and its tabs where they stand out below the wing, and a narrow deep one
    round the (trimmed) horn and its +-30 deg swing. It ends before x_limit (the
    aileron hinge, or the centre section's trailing edge) and leaves a hole where
    the pushrod runs out. `above` is everything it is glued under; `clear` are
    solids it must stay off (the boom sockets)."""
    sv, w = k.servo, FAIRING_WALL
    xm, ya, zc = s["origin"]
    xs, yh = xm + sv.shaft_x, ya + sv.horn_z
    reach = horn_trim(s) + 0.8                              # horn tip below the shaft axis
    outer, inner = [], []
    z_case = min(s["top"] - sv.width, zc - sv.boss_d / 2)     # lowest point of the case and boss
    xa, xb = xm - sv.tab_span / 2 - 0.8, xm + sv.tab_span / 2 + 0.8
    bump = max(surface(x) for x in (xa, xm, xb)) - z_case
    if bump > 0.3:
        y0, y1 = ya - 1.0, ya + sv.height + sv.boss_h + sv.spline_h + 0.8
        bx0, bx1 = xa - 12, min(xb + 22, x_limit)
        a, b = 1.12 * (y1 - y0) / 2 + w, 1.35 * (bump + 1.0) + w
        outer.append(blister(bx0, bx1, (y0 + y1) / 2, a, b, surface, body=(xa, xb)))
        inner.append(blister(bx0, bx1, (y0 + y1) / 2, a, b, surface, grow=-w, body=(xa, xb)))
    swing = reach * math.sin(math.radians(30))
    ha, hb = xs - swing - 1.0, xs + swing + 1.0
    depth = max(surface(x) for x in (ha, xs, hb)) - (zc - reach) + 1.0
    hx0, hx1 = ha - 7, min(hb + 20, x_limit)
    outer.append(blister(hx0, hx1, yh, 3.3 + w, 1.2 * depth + w, surface, n=2.6, seam=0.37, body=(ha, hb)))
    inner.append(blister(hx0, hx1, yh, 3.3, 1.2 * depth, surface, n=2.6, seam=0.37, body=(ha, hb)))
    lx, ly, lz = s["link"]
    d = V(*rod_to) - V(lx, ly, lz)
    exit_hole = rod(1.5, (lx, ly, lz), tuple(V(lx, ly, lz) + d.normalized() * 70))
    return cut(fuse(*outer), *inner, above, exit_hole, *clear)


def fairings(p: Params, L: Layout, sec: Section, k: Kit) -> dict:
    """Fairings for the two aileron servos and the elevator and rudder servos."""
    out = {}
    ail = aileron_servo(p, L, sec, k)
    xh = sec.x_at(p.hinge_pos) + 0.3
    y0, y1 = p.centre_w / 2, p.span / 2
    lower = sorted(sec.lo)                                  # everything above the panel's lower surface
    above = prism_y([(lower[0][0] - 60, lower[0][1]), *lower, (lower[-1][0] + 60, lower[-1][1]),
                     (lower[-1][0] + 60, 80), (lower[0][0] - 60, 80)], y0 - 5, y1)
    out["fairing_ail_R"] = servo_fairing(
        p, L, ail, k, lambda x: sec.lower_z(x) + 0.2, above, sec.x_at(p.hinge_pos) - 1.5,
        (xh + 2.1, ail["link"][1], sec.lower_z(xh + 1.8) - 7.8))
    out["fairing_ail_L"] = out["fairing_ail_R"].mirror("XZ")
    if L.single:
        return out
    flat = lambda x: L.z_top + 0.2  # noqa: E731  the centre section's flat underside
    lid = box(L.x_le - 20, L.x_te + 30, -p.centre_w, p.centre_w, L.z_top, L.z_top + 60)
    es = elevator_servo(p, L, sec, k)
    x_eh = L.x_stab + L.c_fix + 0.6 + 3.0
    booms = [cyl_x(L.r_boss + 0.4, L.x_le - 20, L.x_te + 40, L.tube_y, 0)]
    out["fairing_elev"] = servo_fairing(p, L, es, k, flat, lid, L.x_te - 1.0, (x_eh - 1.5, L.pushrod_y, L.pushrod_z),
                                        booms)
    if p.rudders:
        rs = rudder_servo(p, L, sec, k)
        out["fairing_rud"] = servo_fairing(p, L, rs, k, flat, lid, L.x_te - 1.0,
                                           (L.bellcrank_x, -L.rudder_pushrod_y, L.joiner_z), booms).mirror("XZ")
    return out


def make_joiner_sleeves(p: Params, L: Layout) -> dict:
    """Streamlined sleeves glued on the two rudder joiner wires: a teardrop
    5.5 mm long and 2 mm thick round each wire. They slide with the wire, so
    each end stays 5 mm clear of the bellcrank arm and the rudder's flange."""
    xo, zj, by = L.x_hinge + p.rudder_horn, L.joiner_z, L.bellcrank_y
    prof = []
    for k in range(25):                                     # upper then lower surface, nose at xo - 1.5
        t = k / 24
        prof.append((xo - 1.5 + 5.5 * t, zj + 1.0 * streamline(t)))
    prof += [(x, 2 * zj - z) for x, z in reversed(prof[1:-1])]
    y_tip = L.b_h / 2 - 4.0 - 5.0
    out = {}
    for name, ya, yb in (("sleeve_R", by + 6.0, y_tip), ("sleeve_L", -y_tip, by - 6.0)):
        out[name] = cut(prism_y(prof, ya, yb), cyl_y(0.55, ya - 1, yb + 1, xo, zj))
    return out


# --------------------------------------------------------------------------
# Single-boom layout: fuselage, lid, wing centre, tail and motor mount


def section_solid(hw, zb, zt, r, ch, x0, x1):
    """The fuselage section extruded from x0 to x1: flat top at zt with its long
    edges rounded to r, flat sides, 45 deg chamfers of size ch on the bottom edges."""
    a = math.radians(45)
    wp = cq.Workplane("YZ", origin=(x0, 0, 0)).moveTo(0, zb)
    wp = wp.lineTo(hw - ch, zb).lineTo(hw, zb + ch) if ch > 0.05 else wp.lineTo(hw, zb)
    wp = (wp.lineTo(hw, zt - r).threePointArc((hw - r + r * math.cos(a), zt - r + r * math.sin(a)), (hw - r, zt))
          .lineTo(-(hw - r), zt).threePointArc((-(hw - r) - r * math.cos(a), zt - r + r * math.sin(a)), (-hw, zt - r)))
    wp = wp.lineTo(-hw, zb + ch).lineTo(-(hw - ch), zb) if ch > 0.05 else wp.lineTo(-hw, zb)
    return wp.close().extrude(x1 - x0).val()


def section_points(hw, zb, zt, r, ch, zc, n=144):
    """The same section as n points on rays from (0, zc) at even angles, from the
    bottom centre round, so it lofts onto a circle without twisting."""
    pts = []
    for i in range(n):
        th = -math.pi / 2 + 2 * math.pi * i / n
        dy, dz = math.cos(th), math.sin(th)
        sy, ay = (1 if dy >= 0 else -1), abs(dy)
        lim = []
        if dz < 0:
            lim.append((zb - zc) / dz)
        if dz > 0:
            lim.append((zt - zc) / dz)
        if ay > 1e-9:
            lim.append(hw / ay)
        if ch > 0.05 and ay - dz > 1e-9:                    # chamfer line y - z = hw - ch - zb
            lim.append((hw - ch - zb + zc) / (ay - dz))
        t = min(lim)
        y, z = t * ay, zc + t * dz
        if y > hw - r and z > zt - r:                       # in the rounded corner: hit the arc instead
            cy, cz = hw - r, zt - r
            b = ay * (0 - cy) + dz * (zc - cz)
            cc = cy ** 2 + (zc - cz) ** 2 - r * r
            t = -b + math.sqrt(max(b * b - cc, 0.0))
            y, z = t * ay, zc + t * dz
        pts.append((sy * y, z))
    return pts


def loft_x(pts0, x0, pts1, x1):
    w0 = cq.Wire.makePolygon([V(x0, y, z) for y, z in pts0], close=True)
    w1 = cq.Wire.makePolygon([V(x1, y, z) for y, z in pts1], close=True)
    return cq.Solid.makeLoft([w0, w1], True)


def circle_points(r, zc, n=144):
    return [(r * math.cos(-math.pi / 2 + 2 * math.pi * i / n), zc + r * math.sin(-math.pi / 2 + 2 * math.pi * i / n))
            for i in range(n)]


def fuselage_split_x(L: Layout) -> float:
    """Front and rear halves meet in the battery bay, clear of the servo bay."""
    return round(L.batt_max - 10.0)


def single_lid_end(p: Params, L: Layout) -> float:
    """The lid stops at the wing, or where it would outgrow the bed."""
    return min(lid_end(L), lid_start(p) + p.bed[0] - p.bed_margin)


def make_fuselage(p: Params, L: Layout, batt_x: float):
    """Fuselage for the single boom, and the lid cut from its top. The section is
    the flat-topped box aero/section.py picks: rounded top edges, small belly
    chamfers. Camera in the nose, FC and battery ahead of the servo bay; the
    boom's socket starts behind the servos and the tail cone narrows onto it.
    The lid is the top ahead of the wing, round edges and all. Under the wing
    the top is open; the wing centre closes it and glues to the boom socket's web."""
    hw, zb, zt, w = L.half_w, L.z_bottom, L.z_top, p.wall
    fw, ch, r = p.front_wall, p.belly_ch, p.top_r
    zf, zs = L.z_floor, zt - r                              # floor top; where the sides meet the round edges
    x0c, x1c = L.cone_x0, L.pod_len
    zc = (zb + zt) / 2
    outer = fuse(section_solid(hw, zb, zt, r, ch, 0, x0c),
                 loft_x(section_points(hw, zb, zt, r, ch, zc), x0c, circle_points(L.r_boss, 0.0), x1c))
    ch_in = ch + w * math.sqrt(2) - 2 * w
    inner = fuse(section_solid(hw - w, zb + w, zt - w, r - w, ch_in, fw, x0c),
                 loft_x(section_points(hw - w, zb + w, zt - w, r - w, ch_in, zc), x0c,
                        circle_points(L.r_boss - w - 0.6, 0.0), x1c - 6.0))
    body = cut(cut(outer, *nose_cuts(p, L, 0.0, ch)), cut(inner, *nose_cuts(p, L, w, ch)))

    big, xl0, xl1 = 200.0, lid_start(p), single_lid_end(p, L)
    lid = body.intersect(box(xl0 + 0.15, xl1 - 0.15, -big, big, zs, zt + 1))
    lid = cut(lid, cyl_z(1.5, zs - 1, zt + 2, L.ant_x, L.ant_y))          # the whip passes the round edge
    seat = L.x_le + 0.08 * L.chord + 2.0                                  # the wing centre closes the top from here
    body = cut(body, box(xl0, xl1, -big, big, zs, zt + 1),
               box(seat, x0c, -(hw - r), hw - r, zt - w - 0.2, zt + 1))

    k, sv = Kit(), Kit().servo
    hp = k.fc_holes / 2
    adds = [cyl_x(L.r_hole + w, L.boom_x0, x1c, 0, 0),                  # boom socket, and a web up to the
            box(L.boom_x0, x1c, -1.5, 1.5, 0, zt)]                       # wing centre's underside
    adds += [cyl_z(2.0, zf - 0.1, zf + 3, L.fc_x + dx, dy) for dx in (-hp, hp) for dy in (-hp, hp)]
    for s in (1, -1):                                                    # the lid's edges sit on these
        ya, yb = s * (hw - w - 1.2), s * (hw - w + 0.05)
        adds.append(box(xl0 - 0.5, xl1 + 0.5, min(ya, yb), max(ya, yb), zs - 1.2, zs))
    holes = []
    for (xm, yb), sgn in ((L.servo_elev, 1), (L.servo_rud, -1)):           # posts under each servo's tabs
        tab = yb + sgn * sv.tab_z                                        # the tabs' face towards the base
        for dx in (-1, 1):
            xh = xm + dx * (sv.tab_span / 2 - 2.3)
            ya, yb2 = yb + sgn * 5.5, tab - sgn * 0.1
            adds.append(box(xh - 1.6, xh + 1.6, min(ya, yb2), max(ya, yb2), zf - 0.1, L.servo_zc + sv.width / 2))
            holes.append(cyl_y(0.8, min(ya, yb2) - 1, max(ya, yb2) + 1, xh, L.servo_zc))
    body = fuse(body, *adds)

    holes.append(cyl_x(L.r_hole, L.boom_x0 - 1, x1c + 1, 0, 0))
    holes += [cyl_z(0.85, zf, zf + 4, L.fc_x + dx, dy) for dx in (-hp, hp) for dy in (-hp, hp)]
    zcam = -9.0                                                          # nano camera (14 mm) window
    holes.append(box(-1, fw + 1, -7.2, 7.2, zcam - 7.2, zcam + 7.2))
    for s in (-1, 1):                                                    # battery strap slots
        y = s * (k.batt_w / 2 + 1.5)
        holes.append(box(batt_x - 12, batt_x + 12, y - 1.0, y + 1.0, zb - 1, zf + 1))
    return cut(body, *holes), lid


def split_fuselage(p: Params, L: Layout, body):
    """Front and rear halves, each short enough for the bed. Two strips on the
    front half's walls slide 8 mm into the rear half for a glued joint, so the
    rear half can print standing on its flat front face."""
    xs, big = fuselage_split_x(L), 500.0
    front = body.intersect(box(-10, xs, -big, big, -big, big))
    rear = body.intersect(box(xs, L.pod_len + 50, -big, big, -big, big))
    hw, w, zf, zs, g = L.half_w, p.wall, L.z_floor, L.z_top - p.top_r, 0.15
    ch_in = p.belly_ch + w * math.sqrt(2) - 2 * w
    strips = [box(xs - 1, xs + 8, min(s * (hw - w - g), s * (hw - w - g - 0.9)), max(s * (hw - w - g), s * (hw - w - g - 0.9)),
                  zf + ch_in + 0.5, zs - 2.0) for s in (1, -1)]
    return fuse(front, *strips), rear


def make_wing_centre_single(p: Params, L: Layout, sec: Section):
    """Centre section for the single boom: it sits on the fuselage top and closes
    it under the wing. No boom sockets and no servos; the aileron leads drop
    into the fuselage through the middle."""
    cw = p.centre_w
    body = sec.centre_solid(-cw / 2, cw / 2, L.z_top)
    holes = []
    for pos, d in ((p.main_spar_pos, p.main_spar_d), (p.rear_spar_pos, p.rear_spar_d)):
        x, z = sec.camber_point(pos)
        holes.append(cyl_y(d / 2 + 0.1, -cw, cw, x, z))
    xw, zw = sec.camber_point(0.45)                          # aileron servo leads into the fuselage
    holes.append(cyl_y(2.0, -cw, cw, xw, zw))
    holes.append(cyl_z(3.0, L.z_top - 1, zw, xw, 0))
    return cut(body, *holes)


def make_tail_mount_single(p: Params, L: Layout):
    """Collar on the boom: a socket for the fin on top, and two pylons under it
    that hold the stabiliser low enough for the elevator to swing up past the
    boom. The elevator pushrod runs between the pylons."""
    x0, x1 = L.x_stab + 0.5, L.x_hinge - 0.5
    r_out, g = L.r_hole + 1.2, 0.15
    body = [cyl_x(r_out, x0, x1, 0, 0),
            box(x0, x1, -(1.0 + g), 1.0 + g, L.r_hole, L.fin_z0)]          # flat seat for the fin's root
    for s in (1, -1):
        body.append(box(x0, x1, min(s * (1.0 + g), s * (2.2 + g)), max(s * (1.0 + g), s * (2.2 + g)),
                        L.r_hole, L.fin_z0 + 6.0))                        # socket cheeks
        body.append(prism_z([(x0, s * 4.2), (x0 + 5.0, s * 2.2), (x1, s * 2.2), (x1, s * 6.2), (x0 + 5.0, s * 6.2)],
                            L.tail_z, -2.5))                               # pylons, wedge-nosed
    return cut(fuse(*body), cyl_x(L.r_hole, x0 - 1, x1 + 1, 0, 0))


def make_stab_single(p: Params, L: Layout):
    return (cq.Workplane().box(L.c_fix, L.b_h - 1.0, p.plate, centered=(False, True, False))
            .translate((L.x_stab, 0, L.tail_z - p.plate)).faces("<X").edges("|Y").fillet(p.plate * 0.45)).val()


def elevator_horn_single(L: Layout):
    """The elevator horn's pushrod hole, on top of the elevator in the middle."""
    return (L.x_hinge + 3.6, 0.0, L.tail_z + 3.5)


def make_elevator_single(p: Params, L: Layout):
    """One-piece elevator under the boom, tape hinge on top, horn standing up in the middle."""
    x0, x1 = L.x_hinge + 0.6, L.x_stab + L.c_h
    y = L.b_h / 2 - 0.6
    z0, z1 = L.tail_z - p.plate, L.tail_z
    hx, _, hz = elevator_horn_single(L)
    horn = prism_y([(x0 + 0.5, z1 - 0.1), (x0 + 8, z1 - 0.1), (x0 + 4, z1 + 4.7), (x0 + 0.5, z1 + 4.7)], -0.8, 0.8)
    el = fuse(box(x0, x1, -y, y, z0, z1), horn)
    cuts = [prism_y([(x0 - 0.1, z0 - 0.1), (x0 - 0.1, z1 - 0.6), (x0 + 2.0, z0 - 0.1)], -y - 1, y + 1),
            prism_y([(x1 - 7, z0 - 0.1), (x1 + 0.1, z0 - 0.1), (x1 + 0.1, z0 + 1.3)], -y - 1, y + 1),
            cyl_y(0.6, -2, 2, hx, hz)]
    return cut(el, *cuts)


def make_fin_single(p: Params, L: Layout):
    """One fin on the boom's centre-line, its root in the tail mount's socket."""
    return (cq.Workplane().box(L.x_hinge - 0.3 - L.x_stab, p.plate, L.fin_z1 - L.fin_z0, centered=False)
            .translate((L.x_stab, -p.plate / 2, L.fin_z0)).faces("<X").edges("|Z").fillet(p.plate * 0.45)).val()


def rudder_horn_single(L: Layout):
    return (L.x_hinge + 3.0, -7.0, L.rudder_z0 + 0.8)


def make_rudder_single(p: Params, L: Layout):
    """The rudder, from just above the boom to the fin's top, with a horn on its
    left side at the bottom. Tape hinge on the right face."""
    t, xr, x_te = p.plate, L.x_hinge + 0.3, L.x_stab + L.c_h
    z0, z1, y0 = L.rudder_z0, L.fin_z1, -p.plate / 2
    hx, hy, _ = rudder_horn_single(L)
    rudder = fuse(box(xr, x_te, y0, y0 + t, z0, z1),
                  box(xr, xr + 6.0, hy - 1.6, y0 + 0.1, z0, z0 + 1.6))
    cuts = [prism_z([(xr - 0.1, y0 - 0.1), (xr + 2.0, y0 - 0.1), (xr - 0.1, y0 + t - 0.6)], z0 - 1, z1 + 1),
            prism_z([(x_te - 7, y0 + t + 0.1), (x_te + 0.1, y0 + t + 0.1), (x_te + 0.1, y0 + t - 1.3)], z0 - 1, z1 + 1),
            cyl_z(0.55, z0 - 1, z0 + 3, hx, hy)]
    return cut(rudder, *cuts)


MOTOR_PLATE_R = 10.0


def make_motor_mount(p: Params, L: Layout):
    """Cap glued on the boom's end, behind the tail: a 16 deg cone from the boom
    up to the plate the motor screws to (9 x 9, 12 and 16 mm patterns), facing
    aft. The motor wires come out of the boom and through the slot at the top."""
    xm = L.x_mp
    cone = cq.Solid.makeCone(L.r_boss, MOTOR_PLATE_R, 11.6, V(xm - 14.0, 0, 0), V(1, 0, 0))
    body = fuse(cone, cyl_x(MOTOR_PLATE_R, xm - 2.4, xm, 0, 0))
    holes = [cyl_x(L.r_hole, xm - 15, xm - 2.4, 0, 0), cyl_x(3.2, xm - 3, xm + 1, 0, 0),
             box(xm - 3, xm + 1, -2.0, 2.0, 3.0, 9.6)]
    for a in (45, 135, 225, 315):
        rr = (5.9 + 8.6) / 2
        holes.append(cq.Workplane("YZ").center(rr * math.cos(math.radians(a)), rr * math.sin(math.radians(a)))
                     .slot2D(8.6 - 5.9 + 2.2, 2.2, a).extrude(6).translate((xm - 4, 0, 0)).val())
    return cut(body, *holes)


def pushrod_paths(L: Layout):
    """Polylines of the elevator and rudder pushrods: from the servo horns in the
    fuselage into the boom's mouth, along inside it, out through a hole ahead of
    the tail mount, and to the horns."""
    sv = Kit().servo
    a = math.radians(L.horn_deg)
    out = {}
    for name, (xm, _), sgn in (("elev", L.servo_elev, 1), ("rud", L.servo_rud, -1)):
        link = (xm + sv.shaft_x + sv.horn_hole * math.cos(a), sgn * L.horn_y, L.servo_zc + sv.horn_hole * math.sin(a))
        mouth = (L.boom_x0 - 2.0, sgn * 1.8, L.link_z)
        inside = (L.pushrod_exit_x - 10.0, sgn * 1.8, L.link_z)
        if name == "elev":
            out[name] = [link, mouth, inside, (L.pushrod_exit_x, 0.0, -6.2), (L.x_stab - 1.0, 0.0, -7.6),
                         elevator_horn_single(L)]
        else:
            out[name] = [link, mouth, inside, (L.pushrod_exit_x, -4.9, 4.9), (L.x_stab - 1.0, -6.5, 6.0),
                         rudder_horn_single(L)]
    return out


def bought_parts_single(p: Params, k: Kit, L: Layout, sec: Section, batt_x: float):
    """The bought parts for the single boom, in flight position."""
    zf, sv = L.z_floor, k.servo
    out = []

    def add(name, shapes, colours, mass, group):
        for suffix, shape in shapes.items():
            out.append(Ref(name + ("_" + suffix if suffix else ""), shape,
                           colours.get(suffix, colours[""]), mass if not suffix else 0.0, group))

    paths = pushrod_paths(L)
    boom = cut(cyl_x(p.boom_od / 2, L.boom_x0, L.boom_x1, 0, 0), cyl_x(p.boom_id / 2, L.boom_x0 - 1, L.boom_x1 + 1, 0, 0),
               *[rod(0.9, pts[2], pts[3]) for pts in paths.values()])        # the pushrods' exit holes
    out.append(Ref("boom", boom, BLACK, rod_mass(p.boom_od, L.boom_len, p.boom_id, p.tube_density), "hardware"))
    for pos, d, name in ((p.main_spar_pos, p.main_spar_d, "spar_main"),
                         (p.rear_spar_pos, p.rear_spar_d, "spar_rear")):
        x, z = sec.camber_point(pos)
        out.append(Ref(name, cyl_y(d / 2, -p.span / 2 + 5, p.span / 2 - 5, x, z), BLACK,
                       rod_mass(d, p.span - 10), "hardware"))

    turn = lambda s: s.rotate(V(0, 0, 0), V(0, 0, 1), 180)   # noqa: E731  pusher: turned to face aft
    add("motor", {n: turn(s).translate(V(L.x_mp, 0, 0)) for n, s in parts_lib.motor_1404().items()},
        {"": (0.62, 0.64, 0.68), "base": (0.14, 0.14, 0.16), "coil": (0.72, 0.42, 0.18),
         "shaft": (0.78, 0.79, 0.81), "nut": (0.18, 0.18, 0.20)}, k.motor, "electronics")
    add("prop", {n: s.mirror("YZ").translate(V(L.x_prop, 0, 0)) for n, s in parts_lib.prop_4x25().items()},
        {"": (0.10, 0.42, 0.85)}, k.prop, "electronics")

    add("camera", {n: s.translate(V(3.0, 0, -9.0)) for n, s in parts_lib.nano_camera().items()},
        {"": (0.10, 0.10, 0.11), "lens": (0.20, 0.35, 0.55), "pcb": (0.05, 0.28, 0.18), "pads": GOLD},
        k.cam_vtx * 0.55, "electronics")
    add("vtx", {n: stand(s).translate(V(L.vtx_x, 0, zf + 0.4 + 9.5)) for n, s in parts_lib.vtx_card().items()},
        {"": (0.05, 0.30, 0.16), "chips": SILVER, "pads": GOLD, "conn": GOLD}, k.cam_vtx * 0.45, "electronics")
    ant = {n: s.translate(V(L.ant_x, L.ant_y, L.ant_z)) for n, s in parts_lib.whip_antenna(38).items()}
    if p.antenna_lean:
        piv = V(L.ant_x, L.ant_y, L.z_top + 1.5)
        whip = ant[""]
        low = whip.intersect(box(piv.x - 5, piv.x + 5, piv.y - 5, piv.y + 5, piv.z - 60, piv.z))
        high = whip.intersect(box(piv.x - 5, piv.x + 5, piv.y - 5, piv.y + 5, piv.z, piv.z + 60))
        ant[""] = fuse(low, high.rotate(piv, piv + V(0, 1, 0), p.antenna_lean), ball(1.3, piv.x, piv.y, piv.z))
    add("antenna", ant, {"": BLACK, "conn": GOLD}, k.antenna, "electronics")
    fc_at = V(L.fc_x, 0, zf + 3)
    add("fc", {n: s.translate(fc_at) for n, s in parts_lib.flight_controller(k).items()},
        {"": (0.08, 0.09, 0.11), "chips": (0.18, 0.19, 0.22), "metal": SILVER, "jst": WHITE, "pads": GOLD},
        k.fc, "electronics")
    out.append(Ref("fc_keepout", parts_lib.fc_keepout(k).translate(fc_at), BLACK, 0.0, "keepout"))
    add("receiver", {n: s.rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(42.0, L.half_w - p.wall - 0.4, zf + 9.0))
                     for n, s in parts_lib.receiver_nano().items()},
        {"": (0.07, 0.10, 0.20), "chips": (0.18, 0.19, 0.22), "pads": GOLD}, k.rx, "electronics")
    add("esc", {n: s.translate(V(L.esc_x + k.esc_len / 2, 0, zf + 0.2)) for n, s in parts_lib.esc_xrotor30(k).items()},
        {"": (0.09, 0.09, 0.10), "chips": (0.18, 0.19, 0.22), "conn": (0.10, 0.14, 0.32), "pads": GOLD,
         "wires": (0.75, 0.15, 0.12)}, k.esc, "electronics")
    add("battery", {n: s.translate(V(batt_x, 0, zf + k.batt_h / 2 + 0.1))
                    for n, s in parts_lib.battery_2s(k.batt_len, k.batt_w, k.batt_h).items()},
        {"": (0.17, 0.28, 0.60), "leads": (0.75, 0.16, 0.12), "xt30": (0.96, 0.80, 0.10),
         "jst": (0.92, 0.92, 0.90)}, k.battery, "electronics")

    servo_colours = {"": (0.16, 0.38, 0.85), "horn": WHITE, "wires": (0.55, 0.25, 0.10)}
    model = parts_lib.sg90(sv, -L.horn_deg, L.horn_len)
    xm, yb = L.servo_elev
    add("servo_elev", {n: shaft_along_y(s, (xm, yb, L.servo_zc)) for n, s in model.items()},
        servo_colours, sv.mass, "electronics")
    xm, yb = L.servo_rud
    add("servo_rud", {n: shaft_along_y(s, (xm, -yb, L.servo_zc)).mirror("XZ") for n, s in model.items()},
        servo_colours, sv.mass, "electronics")
    for name, pts in paths.items():
        length = sum((V(*b) - V(*a)).Length for a, b in zip(pts, pts[1:]))
        out.append(Ref(f"pushrod_{name}", wire_path(0.5, pts), BLACK, pushrod_mass(length), "hardware"))

    ail = aileron_servo(p, L, sec, k)
    trim = (lambda s: horn_trim(s)) if p.fairings else (lambda s: None)   # noqa: E731
    servo = {n: shaft_along_y(s, ail["origin"]) for n, s in parts_lib.sg90(sv, ail["horn_deg"], trim(ail)).items()}
    xh = sec.x_at(p.hinge_pos) + 0.3
    zl = sec.lower_z(xh + 1.8)
    ly = ail["link"][1]
    horn = parts_lib.micro_horn()[""].translate(V(xh + 0.3, ly, zl + 0.2))
    rod_r = rod(0.4, ail["link"], (xh + 2.1, ly, zl - 7.8))
    for side, m in (("R", lambda s: s), ("L", lambda s: s.mirror("XZ"))):
        add(f"servo_ail_{side}", {n: m(s) for n, s in servo.items()}, servo_colours, sv.mass, "electronics")
        out.append(Ref(f"horn_ail_{side}", m(horn), WHITE, 0.3, "hardware"))
        out.append(Ref(f"pushrod_ail_{side}", m(rod_r), BLACK, 0.2, "hardware"))
    return out


# --------------------------------------------------------------------------
# Bought parts in flight position (for the STEP, the viewer and the CG)

BLACK, WHITE = (0.08, 0.08, 0.09), (0.93, 0.93, 0.92)
GOLD, SILVER = (0.85, 0.68, 0.25), (0.66, 0.68, 0.72)


def stand(shape):
    """Turn a card modelled flat (x long, y wide, z thick) so it stands across
    the pod: x -> y, y -> z, z -> x (a 120 degree turn about the (1, 1, 1) axis)."""
    return shape.rotate(V(0, 0, 0), V(1, 1, 1), 120)


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
    # pusher: the motor is turned round on the pod's back wall; the prop is its mirror image
    turn = lambda s: s.rotate(V(0, 0, 0), V(0, 0, 1), 180)
    add("motor", {n: turn(s).translate(V(L.pod_len, 0, p.motor_z)) for n, s in parts_lib.motor_1404().items()},
        {"": (0.62, 0.64, 0.68), "base": (0.14, 0.14, 0.16), "coil": (0.72, 0.42, 0.18),
         "shaft": (0.78, 0.79, 0.81), "nut": (0.18, 0.18, 0.20)}, k.motor, "electronics")
    add("prop", {n: s.mirror("YZ").translate(V(L.pod_len + 14.8, 0, p.motor_z))
                 for n, s in parts_lib.prop_4x25().items()},
        {"": (0.10, 0.42, 0.85)}, k.prop, "electronics")

    # pod contents. Cards that stand across the pod are modelled flat and turned
    # with stand(): local x -> y, y -> z, thickness z -> x.
    add("camera", {n: s.translate(V(3.0, 0, -9.0)) for n, s in parts_lib.nano_camera().items()},
        {"": (0.10, 0.10, 0.11), "lens": (0.20, 0.35, 0.55), "pcb": (0.05, 0.28, 0.18), "pads": GOLD},
        k.cam_vtx * 0.55, "electronics")
    add("vtx", {n: stand(s).translate(V(L.vtx_x, 0, zf + 0.4 + 9.5)) for n, s in parts_lib.vtx_card().items()},
        {"": (0.05, 0.30, 0.16), "chips": SILVER, "pads": GOLD, "conn": GOLD}, k.cam_vtx * 0.45, "electronics")
    ant = {n: s.translate(V(L.ant_x, L.ant_y, L.ant_z)) for n, s in parts_lib.whip_antenna(38).items()}
    if p.antenna_lean:                                      # bent back where it leaves the lid
        piv = V(L.ant_x, L.ant_y, L.z_top + 1.5)
        whip = ant[""]
        low = whip.intersect(box(piv.x - 5, piv.x + 5, piv.y - 5, piv.y + 5, piv.z - 60, piv.z))
        high = whip.intersect(box(piv.x - 5, piv.x + 5, piv.y - 5, piv.y + 5, piv.z, piv.z + 60))
        ant[""] = fuse(low, high.rotate(piv, piv + V(0, 1, 0), p.antenna_lean), ball(1.3, piv.x, piv.y, piv.z))
    add("antenna", ant, {"": BLACK, "conn": GOLD}, k.antenna, "electronics")
    fc_at = V(L.fc_x, 0, zf + 3)
    add("fc", {n: s.translate(fc_at) for n, s in parts_lib.flight_controller(k).items()},
        {"": (0.08, 0.09, 0.11), "chips": (0.18, 0.19, 0.22), "metal": SILVER, "jst": WHITE, "pads": GOLD},
        k.fc, "electronics")
    out.append(Ref("fc_keepout", parts_lib.fc_keepout(k).translate(fc_at), BLACK, 0.0, "keepout"))
    add("receiver", {n: s.rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(42.0, 19.2, zf + 9.0))
                     for n, s in parts_lib.receiver_nano().items()},
        {"": (0.07, 0.10, 0.20), "chips": (0.18, 0.19, 0.22), "pads": GOLD}, k.rx, "electronics")
    # motor tabs face the left wall, where the motor wires arrive; battery wires face right
    add("esc", {n: stand(s.rotate(V(0, 0, 0), V(0, 0, 1), 180)).translate(V(L.esc_x, 0, zf + 0.2 + k.esc_wid / 2))
                for n, s in parts_lib.esc_xrotor30(k).items()},
        {"": (0.09, 0.09, 0.10), "chips": (0.18, 0.19, 0.22), "conn": (0.10, 0.14, 0.32), "pads": GOLD,
         "wires": (0.75, 0.15, 0.12)}, k.esc, "electronics")
    add("battery", {n: s.translate(V(batt_x, 0, zf + k.batt_h / 2 + 0.1))
                    for n, s in parts_lib.battery_2s(k.batt_len, k.batt_w, k.batt_h).items()},
        {"": (0.17, 0.28, 0.60), "leads": (0.75, 0.16, 0.12), "xt30": (0.96, 0.80, 0.10),
         "jst": (0.92, 0.92, 0.90)}, k.battery, "electronics")

    # servos
    servo_colours = {"": (0.16, 0.38, 0.85), "horn": WHITE, "wires": (0.55, 0.25, 0.10)}
    es = elevator_servo(p, L, sec, k)
    trim = (lambda s: horn_trim(s)) if p.fairings else (lambda s: None)   # faired horns are cut short
    add("servo_elev", {n: shaft_along_y(s, es["origin"]) for n, s in parts_lib.sg90(sv, es["horn_deg"], trim(es)).items()},
        servo_colours, sv.mass, "electronics")
    el_link = es["link"]
    x_eh = L.x_stab + L.c_fix + 0.6 + 3.0                    # elevator horn hole
    out.append(Ref("pushrod_elev", rod(0.5, el_link, (x_eh - 1.5, L.pushrod_y, L.pushrod_z)), BLACK,
                   rod_mass(1.0, x_eh - el_link[0]), "hardware"))

    if p.rudders:
        rs = rudder_servo(p, L, sec, k)
        servo = parts_lib.sg90(sv, rs["horn_deg"], trim(rs))
        add("servo_rud", {n: shaft_along_y(s, rs["origin"]).mirror("XZ") for n, s in servo.items()},
            servo_colours, sv.mass, "electronics")
        lx, ly, lz = rs["link"]
        start = (lx, -ly, lz)
        crank_in = (L.bellcrank_x, L.rudder_pushrod_y, L.joiner_z)
        out.append(Ref("pushrod_rud", rod(0.5, start, crank_in), BLACK,
                       rod_mass(1.0, crank_in[0] - start[0]), "hardware"))
        crank_out = (L.x_hinge + p.rudder_horn, L.bellcrank_y, L.joiner_z)
        for side, sgn in (("R", 1), ("L", -1)):
            horn = (L.x_hinge + p.rudder_horn, sgn * (L.b_h / 2 - 2.0), L.joiner_z)
            out.append(Ref(f"joiner_{side}", rod(0.4, crank_out, horn), BLACK,
                           rod_mass(0.8, L.b_h / 2, rho=7.8), "hardware"))

    ail = aileron_servo(p, L, sec, k)
    servo = {n: shaft_along_y(s, ail["origin"]) for n, s in parts_lib.sg90(sv, ail["horn_deg"], trim(ail)).items()}
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
    "pod_front":   (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "pod_rear":    (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
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
    "fairing_ail_R": (LW_PLA, FAIRING_WALL, 0.0, (0.93, 0.93, 0.90)),
    "fairing_ail_L": (LW_PLA, FAIRING_WALL, 0.0, (0.93, 0.93, 0.90)),
    "fairing_elev":  (LW_PLA, FAIRING_WALL, 0.0, (0.86, 0.87, 0.84)),
    "fairing_rud":   (LW_PLA, FAIRING_WALL, 0.0, (0.86, 0.87, 0.84)),
    "sleeve_R":    (PLA, 0.4, 0.3, (0.20, 0.22, 0.25)),
    "sleeve_L":    (PLA, 0.4, 0.3, (0.20, 0.22, 0.25)),
    "fuselage_front": (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "fuselage_rear":  (PLA, 0.8, 0.15, (0.20, 0.22, 0.25)),
    "fin":         (LW_PLA, 0.4, 0.15, (0.93, 0.93, 0.90)),
    "rudder":      (PLA, 0.4, 0.15, (0.96, 0.45, 0.10)),
    "motor_mount": (PLA, 1.6, 0.4, (0.20, 0.22, 0.25)),
}
PRINT_NOTES = {
    "pod_front": "Upright, open top up. 2 walls, 15 % infill.",
    "pod_rear": "Upright, open top up, tongue forward. 2 walls, 15 % infill.",
    "lid": "Flat.",
    "wing_centre": "On its side, spar holes vertical. 0.5 mm walls, 5 % infill.",
    "wing_R": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "wing_L": "Standing on the root rib, brim. 1 wall, 0 % infill.",
    "aileron_R": "Flat on the lower surface. 1 wall, 0 % infill.",
    "aileron_L": "Flat on the lower surface. 1 wall, 0 % infill.",
    "tail_mount": "Pads face down.",
    "stab": "Flat. 2 top / 2 bottom layers, 15 % infill.",
    "elevator": "Top face down, horn up.",
    "fin_R": "Outer face down, jaws up.",
    "fin_L": "Outer face down, jaws up.",
    "rudder_R": "Outer face down, wire flange up. 2 top / 2 bottom layers, 15 % infill.",
    "rudder_L": "Outer face down, wire flange up. 2 top / 2 bottom layers, 15 % infill.",
    "bellcrank": "Flat. Solid.",
    "fairing_ail_R": "Rim down on the bed, brim. 1 wall, 0 % infill. Glue under the wing over the servo.",
    "fairing_ail_L": "Rim down on the bed, brim. 1 wall, 0 % infill. Glue under the wing over the servo.",
    "fairing_elev": "Rim down on the bed. 1 wall, 0 % infill. Glue under the wing centre.",
    "fairing_rud": "Rim down on the bed. 1 wall, 0 % infill. Glue under the wing centre.",
    "sleeve_R": "Flat. Slide on the joiner wire before the Z-bends and glue.",
    "sleeve_L": "Flat. Slide on the joiner wire before the Z-bends and glue.",
    "fuselage_front": "Upright, open top up, wall strips aft. 2 walls, 15 % infill.",
    "fuselage_rear": "Standing on its front face. 2 walls, 15 % infill.",
    "fin": "Flat. 2 top / 2 bottom layers, 15 % infill. Root into the tail mount's socket.",
    "rudder": "Flat, horn up. 2 top / 2 bottom layers, 15 % infill.",
    "motor_mount": "Plate down. 4 walls, 40 % infill. Glue on the boom's end.",
}
# printed parts whose print orientation differs between the layouts
SINGLE_ORIENT = {"lid": "as is", "tail_mount": "front down", "elevator": "as is"}
def print_mass(name, shape):
    rho, shell, infill, _ = PRINT[name]
    vol, area = shape.Volume(), shape.Area()
    solid = min(vol, area * shell)
    return rho * (solid + infill * max(0.0, vol - solid)) / 1000


# --------------------------------------------------------------------------
# Print orientation


def to_bed(shape, name, p: Params = None):
    s = shape
    how = SINGLE_ORIENT.get(name) if p is not None and p.layout == "single" else None
    if how == "as is":
        pass
    elif how == "front down" or name == "fuselage_rear":
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), -90)
    elif name == "motor_mount":
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), 90)
    elif name == "fin":
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    elif name == "rudder":                                  # horn up
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), -90)
    elif name == "lid" or name == "tail_mount" or name == "elevator" or name.startswith("fairing"):
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
    parts["wing_centre"] = (make_wing_centre_single if L.single else make_wing_centre)(p, L, sec)
    parts["wing_R"] = make_panel(p, L, sec)
    parts["wing_L"] = parts["wing_R"].mirror("XZ")
    parts["aileron_R"] = make_aileron(p, L, sec)
    parts["aileron_L"] = parts["aileron_R"].mirror("XZ")
    if L.single:
        parts["tail_mount"] = make_tail_mount_single(p, L)
        parts["stab"] = make_stab_single(p, L)
        parts["elevator"] = make_elevator_single(p, L)
        parts["fin"] = make_fin_single(p, L)
        parts["rudder"] = make_rudder_single(p, L)
        parts["motor_mount"] = make_motor_mount(p, L)
    else:
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
    if p.fairings:
        parts.update(fairings(p, L, sec, k))
    if p.joiner_sleeves and p.rudders and not L.single:
        parts.update(make_joiner_sleeves(p, L))

    def body(batt_x):
        """The pod (or fuselage and its lid) round the battery's strap slots."""
        if L.single:
            fus, lid = make_fuselage(p, L, batt_x)
            front, rear = split_fuselage(p, L, fus)
            return {"fuselage_front": front, "fuselage_rear": rear, "lid": lid}
        front, rear = split_pod(p, L, make_pod(p, L, batt_x))
        return {"pod_front": front, "pod_rear": rear}
    bought = bought_parts_single if L.single else bought_parts

    # CG with CAD masses: solve the battery station, then build the pod around it
    parts.update(body(batt_nominal))
    refs = bought(p, k, L, sec, batt_nominal)

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
        if L.single:                                         # ESC to motor, inside the boom
            items.append(("motor_wires", motor_wire_mass(L.x_mp - L.esc_x), (L.esc_x + L.x_mp) / 2, 0.0))
        return items

    others = [i for i in mass_items() if i[0] != "battery"]
    target = L.x_le + p.cg_target * L.chord
    m = sum(i[1] for i in others)
    batt_x = round((target * (m + k.battery) - sum(i[1] * i[2] for i in others)) / k.battery, 1)
    parts.update(body(batt_x))
    refs = bought(p, k, L, sec, batt_x)
    return p, k, L, sec, parts, refs, mass_items(), batt_x


# --------------------------------------------------------------------------
# Export


def tessellate(shape, tol=0.08, ang=0.25):
    vs, tris = shape.tessellate(tol, ang)
    return np.array([(v.x, v.y, v.z) for v in vs]), np.array(tris, dtype=np.int64)


def srgb_to_linear(rgb):
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb]


def export_all(p, k, L, parts, refs, out: Path, step: bool = True):
    (out / "stl").mkdir(parents=True, exist_ok=True)
    for name, shape in parts.items():
        cq.exporters.export(to_bed(shape, name, p), str(out / "stl" / f"{name}.stl"),
                            tolerance=0.03, angularTolerance=0.15)

    bodies = [(n, s, PRINT[n][3], "printed") for n, s in parts.items()]
    bodies += [(r.name, r.shape, r.colour, r.group) for r in refs if r.group != "keepout"]
    assy = cq.Assembly(name=f"kipina_{p.span:.0f}")
    groups = {g: cq.Assembly(name=g) for g in ("printed", "hardware", "electronics")}
    for name, shape, colour, group in bodies:
        groups[group].add(shape, name=name, color=cq.Color(*colour))
    for g in groups.values():
        assy.add(g)
    if step:
        (out / "cad").mkdir(parents=True, exist_ok=True)
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
    bb = to_bed(shape, name, p).BoundingBox()
    bx, by, bz = p.bed
    flat = (bb.xlen <= bx and bb.ylen <= by) or (bb.xlen <= by and bb.ylen <= bx)
    return flat and bb.zlen <= bz


def write_report(p, k, L, parts, items, batt_x, out: Path, hits=(), variant="baseline"):
    auw = sum(i[1] for i in items)
    cg = sum(i[1] * i[2] for i in items) / auw
    cgz = sum(i[1] * i[3] for i in items) / auw
    perf = performance(p, auw, L, k)
    np_ = neutral_point(p, L)
    est = sum(g for _, g, _ in structure_estimate(p, L))
    printed = sum(i[1] for i in items if i[0] in PRINT)
    if L.single:
        tip = sum(i[1] for i in items if i[0] in ("stab", "elevator", "fin", "rudder", "tail_mount", "motor_mount",
                                                   "motor", "prop"))
        booms = boom_options(p, L, tip, L.x_mp - L.pod_len)
        layout_rows = [
            f"| Boom | 1 x {p.boom_od:.0f}x{p.boom_id:.0f} mm roll-wrapped carbon, {L.boom_len:.0f} mm long, "
            f"{L.pod_len - L.boom_x0:.0f} mm of it in the fuselage's socket |",
            f"| Prop clearance | prop {L.x_prop - L.x_stab - L.c_h:.1f} mm behind the tail's trailing edge |",
            f"| Stabiliser | {L.b_h:.0f} x {L.c_h:.1f} mm (elevator {L.c_e:.1f} mm), "
            f"{-L.tail_z:.1f} mm under the boom's axis |",
            f"| Fin | 1 x {L.c_h:.1f} x {L.fin_h:.0f} mm on the boom |"]
    else:
        layout_rows = [
            f"| Tail booms | 2 x {p.tube_od:.0f}x{p.tube_id:.0f} mm, {L.tube_len:.0f} mm long, {p.tube_spacing:.0f} mm apart |",
            f"| Prop clearance | {L.tube_y - k.prop_d_in * 12.7 - p.tube_od / 2:.1f} mm to each boom, "
            f"{L.pushrod_y - k.prop_d_in * 12.7 - 0.5:.1f} mm to each pushrod |",
            f"| Stabiliser | {L.b_h:.0f} x {L.c_h:.1f} mm (elevator {L.c_e:.1f} mm) |",
            f"| Fins | 2 x {L.c_h:.1f} x {L.fin_h:.0f} mm |"]
    title = "Single-boom micro FPV pusher" if L.single else "Twin-boom micro FPV pusher"
    lines = [
        f"# {title}: build report" + ("" if variant == "baseline" else f" ({variant} variant)"),
        "",
        "Generated by `build.py" + ("" if variant == "baseline" else f" --variant {variant}") +
        "` from the CAD volumes. Do not edit by hand.",
        "",
        "## Key numbers",
        "",
        "| | |",
        "|---|---|",
        f"| Wingspan | {p.span:.0f} mm |",
        f"| Chord | {L.chord:.0f} mm ({airfoil_label(p)}, {p.incidence:.0f} deg incidence) |",
        f"| Wing area | {L.area / 1e4:.2f} dm^2 |",
        f"| Length ({'nose to prop' if L.single else 'nose to elevator TE'}) | {L.length:.0f} mm |",
        *layout_rows,
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
        f"from the nose ({p.cg_target * L.chord:.1f} mm behind the wing LE) |",
        f"| CG (computed) | x {cg:.1f} mm, z {cgz:.1f} mm |",
        f"| Neutral point | {np_ * 100:.0f} % chord -> static margin {(np_ - p.cg_target) * 100:.0f} % |",
        f"| Battery centre | **{batt_x:.0f} mm** from the nose "
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
    sv = k.servo
    lines += ["", "## Bought electronics", "",
              "Outer sizes are from datasheets or retailer listings; the detail inside them "
              "(chips, connectors, wire lengths) is representative.", "",
              "| part | model | size, mm | g |", "|---|---|---|---|",
              f"| Servos, 4 | Tower Pro SG90 | {sv.length} x {sv.width} x {sv.height} body, "
              f"{sv.tab_span} over the tabs | {4 * sv.mass:.0f} |",
              f"| Motor | 1404, {k.motor_kv:.0f} KV | 18 dia x 12.2 long (bell) | {k.motor:.0f} |",
              f"| Prop | {k.prop_d_in:.0f} x {k.prop_pitch_in} two-blade | 101.6 dia | {k.prop:.0f} |",
              f"| ESC | {k.esc_name} (BLHeli_S, 2-4S, 30 A, no BEC) | "
              f"{k.esc_len} x {k.esc_wid} x {k.esc_thk} | {k.esc:.1f} |",
              f"| Flight controller | {k.fc_name} (INAV / ArduPilot, 2-6S, 12 outputs, 5 A servo BEC) | "
              f"{k.fc_len:.0f} x {k.fc_wid:.0f} x {k.fc_hgt}, {k.fc_holes:.0f} mm hole pattern | {k.fc:.0f} |",
              f"| Receiver | ELRS 2.4 GHz nano | 10 x 10 x 3 | {k.rx} |",
              f"| Camera + VTX | 14 mm nano camera, 5.8 GHz AIO VTX | 14 x 14 x 12 ; 20 x 19 x 3 | {k.cam_vtx:.0f} |",
              f"| Antenna | 5.8 GHz whip | 38 long | {k.antenna} |",
              f"| Battery | 2S {k.batt_wh / 7.4 * 1000:.0f} mAh LiPo, XT30 | "
              f"{k.batt_len:.0f} x {k.batt_w:.0f} x {k.batt_h:.0f} | {k.battery:.0f} |"]
    bed = " x ".join(f"{v:.0f}" for v in p.bed)
    lines += ["", "## Print list", "",
              f"Build volume checked: {bed} mm (Bambu Lab A1 mini).", "",
              "| file | material | est. g | on the bed, mm | fits | orientation |",
              "|---|---|---|---|---|---|"]
    print_list = []
    for name, shape in parts.items():
        bb = to_bed(shape, name, p).BoundingBox()
        mat = "LW-PLA" if PRINT[name][0] == LW_PLA else "PLA/PETG"
        dims = f"{bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f}"
        grams = print_mass(name, shape)
        fits = fits_bed(p, shape, name)
        lines.append(f"| stl/{name}.stl | {mat} | {grams:.1f} | {dims} | {'yes' if fits else '**no**'} | "
                     f"{PRINT_NOTES[name]} |")
        print_list.append({"name": name, "material": mat, "grams": round(grams, 1),
                           "bed": dims, "fits": fits, "note": PRINT_NOTES[name]})
    if L.single:
        lines += ["", "## The boom", "",
                  f"One carbon tube carries the tail, the motor and the prop ({tip:.0f} g on its end), "
                  f"{L.x_mp - L.pod_len:.0f} mm out from the fuselage's socket. The first bending mode should "
                  "stay near the twin booms' (about 49 Hz with the tail alone) and far below the motor "
                  f"({k.motor_kv * k.v_loaded * k.rpm_frac / 60:.0f} rev/s at full throttle). Roll-wrapped "
                  "carbon (E 70 GPa); a pultruded tube is stiffer in bending but weak in twist and splits "
                  "in a crash.", "",
                  "| tube, mm | g | EI, N m^2 | first bending mode | end bends in a 30 g landing |",
                  "|---|---|---|---|---|"]
        for b in booms:
            mark = "**" if b["chosen"] else ""
            lines.append(f"| {mark}{b['tube']}{mark} | {b['g']} | {b['ei']} | {b['f1']} Hz | {b['landing_mm']} mm |")
    lines += ["", report(p, k), ""]
    (out / "REPORT.md").write_text("\n".join(lines))

    from dataclasses import replace
    from design import evaluate, smallest_span
    sweep = []
    for b in sorted(set(range(300, 561, 20)) | {round(p.span)}):
        _, a, pf = evaluate(replace(p, span=float(b)), k)
        sweep.append({"span": b, "auw": round(a), "stall": round(pf["stall"], 2)})
    spec = {
        "span": p.span, "chord": L.chord, "area_dm2": round(L.area / 1e4, 2),
        "length": round(L.length), "naca": p.naca, "airfoil": airfoil_label(p), "incidence": p.incidence,
        "layout": p.layout,
        "details": {"nose_top_r": p.nose_top_r, "boattail": p.boattail, "antenna_lean": p.antenna_lean,
                    "fairings": p.fairings, "joiner_sleeves": p.joiner_sleeves},
        "auw": round(auw), "auw_g": round(auw, 2), "printed": round(printed), "printed_g": round(printed, 2),
        "loading": round(perf["loading"], 1),
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
        "tube_x0": round(L.tube_x0, 1), "tube_x1": round(L.tube_x1, 1),
        "x_prop": round(L.x_prop if L.single else L.pod_len + 14.8, 1),
        "prop_r": k.prop_d_in * 12.7, "split_x": fuselage_split_x(L) if L.single else pod_split_x(L),
        "stall_limit": p.stall_limit, "min_span": smallest_span(p, k),
        "no_rudder_span": smallest_span(replace(p, rudders=False), k), "sweep": sweep,
        "fc": k.fc_name, "esc": k.esc_name, "variant": variant,
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
    ("joiner_R", "rudder_R"), ("joiner_L", "rudder_L"), ("joiner_R", "joiner_L"),
    ("pushrod_rud", "rudder"))}


def component(name):
    """servo_ail_R_horn -> servo_ail_R: sub-bodies of one bought part."""
    head, _, tail = name.rpartition("_")
    if head and tail in parts_lib.SUFFIXES and not name.startswith("horn_"):
        return head
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


VARIANTS = ("baseline", "optimized", "single-boom")


def variant_params(name: str) -> Params:
    """The as-built design, or the one the efficiency study recommends: its
    optimized airfoil and the detail changes the drag optimizer picked
    (aero/study/study.json)."""
    from dataclasses import replace
    p = Params()
    if name == "baseline":
        return p
    import json
    study = json.loads((HERE.parent / "aero" / "study" / "study.json").read_text())
    d = study["details"]
    # the 3D maximum lift scales with the section's own maximum, as in aero/variants.py
    secs = study["sections"]
    p = replace(p, cl_max=round(p.cl_max * secs["optimized"]["clmax"] / secs["baseline"]["clmax"], 3))
    p = replace(p, airfoil="aero/study/kipina-opt.dat", nose_top_r=float(d.get("nose_top_r", 0)),
                boattail=float(d.get("boattail", 0)), antenna_lean=float(d.get("antenna_lean", 0)),
                fairings=bool(d.get("fairings")), joiner_sleeves=bool(d.get("joiners_hidden")))
    if name == "single-boom":
        p = single_boom(p)                          # the optimized plane on one carbon boom
    return p


def variant_dir(name: str) -> Path:
    return HERE if name == "baseline" else HERE / "variants" / name


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--variant", choices=VARIANTS, default="baseline",
                    help="baseline: as built (writes here); optimized: the efficiency study's airfoil "
                         "and detail changes (writes variants/optimized/); single-boom: the optimized plane "
                         "on one carbon boom with the motor behind the tail (writes variants/single-boom/)")
    ap.add_argument("--step", action="store_true", help="also write the STEP for a variant (always for the baseline)")
    args = ap.parse_args(argv)
    out = variant_dir(args.variant)
    out.mkdir(parents=True, exist_ok=True)
    p, k, L, sec, parts, refs, items, batt_x = build(variant_params(args.variant))
    for name, s in parts.items():
        assert s.isValid(), f"{name} is not a valid solid"
    for name, shape in parts.items():
        if not fits_bed(p, shape, name):
            print(f"WARNING: {name} does not fit the {p.bed} mm build volume")
    hits = interference(parts, refs)
    for a, b, v in hits:
        print(f"WARNING: {a} overlaps {b} by {v} mm^3")
    auw, cg = write_report(p, k, L, parts, items, batt_x, out, hits, args.variant)
    export_all(p, k, L, parts, refs, out, step=args.variant == "baseline" or args.step)
    print(f"AUW {auw:.1f} g, CG {cg:.1f} mm, battery centre {batt_x:.1f} mm, x_le {L.x_le:.1f}")


if __name__ == "__main__":
    main()
