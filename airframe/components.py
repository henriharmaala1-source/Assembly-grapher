"""Bought parts as recognisable solids for the assembly (not for printing).

Each builder works in its own local frame, described in its docstring, and
returns a dict of {suffix: shape} so a part can carry more than one colour
(for example a servo body and its horn).
"""
import math

import cadquery as cq
from cadquery import Vector as V

from design import Servo
from geom import box, cut, cyl_x, cyl_y, cyl_z, fuse, prism_z


def sg90(s: Servo = Servo(), horn_deg: float = 90.0, horn_len: float = None):
    """SG90 micro servo. Base on z = 0, length along x, width along y, output
    shaft on +z at x = s.shaft_x. horn_deg turns the single-arm horn in the
    x-y plane (0 = along +x); horn_len trims the arm."""
    L, W, H = s.length, s.width, s.height
    body = fuse(
        box(-L / 2, L / 2, -W / 2, W / 2, 0, H),
        box(-s.tab_span / 2, s.tab_span / 2, -W / 2, W / 2, s.tab_z, s.tab_z + s.tab_t),
        cyl_z(s.boss_d / 2, H, H + s.boss_h, s.shaft_x, 0),
        cyl_z(2.6, H, H + 2.4, s.shaft_x - 7.8, 0),
        cyl_z(s.spline_d / 2, H + s.boss_h, H + s.boss_h + s.spline_h, s.shaft_x, 0),
        box(-L / 2 - 5, -L / 2, -1.6, 1.6, 3.2, 4.4),                  # lead exit
    )
    for sx in (-1, 1):                                                 # tab holes + slots
        xh = sx * (s.tab_span / 2 - 2.3)
        body = cut(body, cyl_z(1.0, s.tab_z - 1, s.tab_z + s.tab_t + 1, xh, 0),
                   box(min(xh, sx * s.tab_span / 2), max(xh, sx * s.tab_span / 2), -0.5, 0.5,
                       s.tab_z - 1, s.tab_z + s.tab_t + 1))
    z1 = H + s.boss_h + s.spline_h
    z0 = z1 - s.horn_t
    a = math.radians(horn_deg)
    ux, uy, vx, vy = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    cx = s.shaft_x
    outline = [(cx + ux * r + vx * w, uy * r + vy * w)
               for r, w in ((0, 3.2), (horn_len or s.horn_len, 2.0),
                            (horn_len or s.horn_len, -2.0), (0, -3.2))]
    horn = fuse(prism_z(outline, z0, z1), cyl_z(3.4, H + s.boss_h + 0.4, z1, cx, 0))
    for r in (s.horn_hole - 2.5, s.horn_hole, s.horn_hole + 2.5):
        if r > (horn_len or s.horn_len) - 1.5:
            continue
        horn = cut(horn, cyl_z(0.5, z0 - 1, z1 + 1, cx + ux * r, uy * r))
    return {"": body, "horn": horn}


def horn_point(s: Servo, horn_deg: float):
    """Local position of the pushrod hole on the horn."""
    a = math.radians(horn_deg)
    return (s.shaft_x + s.horn_hole * math.cos(a), s.horn_hole * math.sin(a), s.horn_z)


def motor_1404():
    """1404 outrunner. Mounting face on x = 0, shaft pointing to -x."""
    body = fuse(
        cyl_x(9.0, -1.6, 0, 0, 0),                     # base
        cyl_x(8.0, -2.4, -1.6, 0, 0),                  # air gap
        cyl_x(9.0, -11.2, -2.4, 0, 0),                 # bell
        cyl_x(7.8, -12.2, -11.2, 0, 0),                # bell cap
    )
    for a in range(0, 360, 60):                        # vents in the cap
        body = cut(body, cyl_x(1.6, -12.5, -11.0, 5.2 * math.cos(math.radians(a)),
                                5.2 * math.sin(math.radians(a))))
    shaft = fuse(cyl_x(2.5, -17.6, -12.2, 0, 0), cyl_x(3.5, -19.6, -17.6, 0, 0))
    return {"": body, "shaft": shaft}


def prop_4x25(pitch_deg: float = 16.0):
    """4x2.5 two-blade prop, hub centred on the origin, disc in the y-z plane."""
    r = 50.8
    blade_yz = [(4.0, -3.4), (14, -4.8), (34, -4.0), (r - 1.5, -1.8), (r, 0),
                (r - 1.5, 1.8), (34, 4.0), (14, 4.8), (4.0, 3.4)]
    wire = cq.Wire.makePolygon([V(-0.5, y, z) for y, z in blade_yz], close=True)
    blade = cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(1.0, 0, 0))
    blade = blade.rotate(V(0, 0, 0), V(0, 1, 0), pitch_deg)
    blades = fuse(blade, blade.rotate(V(0, 0, 0), V(1, 0, 0), 180))
    return {"": cut(fuse(blades, cyl_x(4.5, -2.5, 2.5, 0, 0)), cyl_x(2.55, -3, 3, 0, 0))}


def nano_camera():
    """14 mm nano FPV camera. Lens on -x, body centred on the origin."""
    body = box(0, 12, -7, 7, -7, 7)
    lens = fuse(cyl_x(4.0, -4.0, 0, 0, 0), cyl_x(4.6, -1.2, 0, 0, 0))
    return {"": fuse(body, lens), "lens": cyl_x(3.0, -4.2, -3.9, 0, 0)}


def board(lx, ly, t=1.6, holes=None, parts=()):
    """Generic PCB lying in x-y on z = 0..t. parts: (x0, x1, y0, y1, h) boxes on top."""
    pcb = box(-lx / 2, lx / 2, -ly / 2, ly / 2, 0, t)
    if holes:
        pcb = cut(pcb, *[cyl_z(1.05, -1, t + 1, hx, hy) for hx in (-holes / 2, holes / 2)
                         for hy in (-holes / 2, holes / 2)])
    chips = [box(x0, x1, y0, y1, t, t + h) for x0, x1, y0, y1, h in parts]
    return {"": pcb, "chips": fuse(*chips)} if chips else {"": pcb}


def flight_controller():
    """20x20 wing FC: 27 x 27 mm board, USB on +y."""
    return board(27, 27, 1.6, holes=20, parts=[(-3.5, 3.5, -3.5, 3.5, 1.0),
                                                (-4, 4, 10.5, 14.5, 2.6),
                                                (7, 11, -9, -5, 1.2)])


def battery_2s(length, width, height):
    """2S LiPo pack centred on the origin. The leads leave the -x end and fold
    back over the top, where the XT30 plug rests."""
    pack = (cq.Workplane().box(length, width, height).edges("|X").fillet(2.0)).val()
    x0, top = -length / 2, height / 2
    leads = fuse(box(x0 - 3, x0, -3.5, 3.5, top - 5, top + 1.4),
                 box(x0 - 3, x0 + 6, -3.5, 3.5, top, top + 1.4),
                 box(x0 + 6, x0 + 14, -5, 5, top, top + 5))                    # XT30
    return {"": pack, "leads": leads}


def whip_antenna(length=32.0):
    """5.8 GHz whip on +z from the origin."""
    return {"": fuse(cyl_z(0.9, 0, length, 0, 0), cyl_z(1.8, 4, 13, 0, 0))}


def micro_horn():
    """Glue-in control horn, base on z = 0, arm on -z, hole row along x."""
    arm = cq.Workplane("XZ").polyline([(0, 0.8), (9, 0.8), (1.2, -10), (0, -10)]).close() \
        .extrude(0.6, both=True).val()
    return {"": cut(arm, *[cyl_y(0.5, -2, 2, 1.8, z) for z in (-5.5, -8.0)])}
