"""Bought parts as recognisable solids for the assembly (not for printing).

Each builder works in its own local frame, described in its docstring, and
returns a dict of {suffix: shape}, so a part can carry several colours (a servo
body, its horn and its wires). The suffixes are listed in SUFFIXES; build.py
colours them and the viewer groups them back under the main part.

Outer dimensions come from datasheets or retailer listings. The detail inside
those envelopes (chip positions, connector sizes, wire lengths) is
representative, not a copy of the manufacturer's drawing.
"""
import math

import cadquery as cq
from cadquery import Vector as V

from design import Kit, Servo
from geom import ball, box, cut, cyl_x, cyl_y, cyl_z, fuse, prism_z, rbox, rod, wire_path

# Sub-bodies of one bought part are named <part>_<suffix>.
SUFFIXES = ("horn", "shaft", "lens", "chips", "leads", "wires", "pcb", "pads", "base", "coil",
            "nut", "metal", "xt30", "jst", "conn", "keepout")


def sg90(s: Servo = Servo(), horn_deg: float = 90.0, horn_len: float = None):
    """SG90 micro servo. Base on z = 0, length along x, width along y, output
    shaft on +z at x = s.shaft_x. horn_deg turns the single-arm horn in the
    x-y plane (0 = along +x); horn_len trims the arm. Case seams, label
    recess, rounded tabs, lead grommet and three wires, and a horn with a
    rounded tip and a centre screw."""
    L, W, H = s.length, s.width, s.height
    hl = horn_len or s.horn_len
    body = fuse(
        rbox(-L / 2, L / 2, -W / 2, W / 2, 0, H, 0.7, "|Z"),
        rbox(-s.tab_span / 2, s.tab_span / 2, -W / 2, W / 2, s.tab_z, s.tab_z + s.tab_t, 1.5, "|Z"),
        cyl_z(s.boss_d / 2, H, H + s.boss_h, s.shaft_x, 0),
        cyl_z(2.6, H, H + 2.4, s.shaft_x - 7.8, 0),
        cyl_z(s.spline_d / 2, H + s.boss_h, H + s.boss_h + s.spline_h, s.shaft_x, 0),
        rbox(-L / 2 - 1.6, -L / 2 + 0.2, -2.0, 2.0, 2.8, 4.8, 0.6, "|X"),      # lead grommet
    )

    def seam(z, d=0.25, h=0.4):
        return cut(box(-L / 2 - 1, L / 2 + 1, -W / 2 - 1, W / 2 + 1, z, z + h),
                   box(-L / 2 + d, L / 2 - d, -W / 2 + d, W / 2 - d, z - 1, z + h + 1))
    body = cut(body, seam(4.0), seam(12.6),
               box(-6, 5, -W / 2 - 0.1, -W / 2 + 0.2, 6.2, 11.6))                 # label recess
    for sx in (-1, 1):                                                 # tab holes + slots
        xh = sx * (s.tab_span / 2 - 2.3)
        body = cut(body, cyl_z(1.0, s.tab_z - 1, s.tab_z + s.tab_t + 1, xh, 0),
                   box(min(xh, sx * s.tab_span / 2), max(xh, sx * s.tab_span / 2), -0.5, 0.5,
                       s.tab_z - 1, s.tab_z + s.tab_t + 1))

    wires = fuse(*[rod(0.45, (-L / 2 - 1.0, y, 3.8), (-L / 2 - 5.0, y, 3.8)) for y in (-1.0, 0.0, 1.0)])

    z1 = H + s.boss_h + s.spline_h
    z0 = z1 - s.horn_t
    a = math.radians(horn_deg)
    ux, uy, vx, vy = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    cx = s.shaft_x
    end = hl - 2.0                                                     # arm ends in a 2 mm radius
    outline = [(cx + ux * r + vx * w, uy * r + vy * w)
               for r, w in ((0, 3.2), (end, 2.0), (end, -2.0), (0, -3.2))]
    horn = fuse(prism_z(outline, z0, z1),
                cyl_z(3.6, H + s.boss_h + 0.4, z1, cx, 0),             # hub
                cyl_z(2.0, z0, z1, cx + ux * end, uy * end),           # rounded tip
                cyl_z(1.5, z1, z1 + 0.9, cx, 0))                       # centre screw
    horn = cut(horn, box(cx - 1.2, cx + 1.2, -0.25, 0.25, z1 + 0.35, z1 + 1.0),
               box(cx - 0.25, cx + 0.25, -1.2, 1.2, z1 + 0.35, z1 + 1.0))        # screw slot
    for r in (s.horn_hole - 2.5, s.horn_hole, s.horn_hole + 2.5):
        if r > hl - 1.5:
            continue
        horn = cut(horn, cyl_z(0.5, z0 - 1, z1 + 1, cx + ux * r, uy * r))
    return {"": body, "horn": horn, "wires": wires}


def horn_point(s: Servo, horn_deg: float):
    """Local position of the pushrod hole on the horn."""
    a = math.radians(horn_deg)
    return (s.shaft_x + s.horn_hole * math.cos(a), s.horn_hole * math.sin(a), s.horn_z)


def motor_1404():
    """1404 outrunner. Mounting face on x = 0, shaft pointing to -x. Black
    base with the 9 x 9 mm screw pattern, a copper 12-tooth stator visible in
    the gap, a chamfered bell with six vent slots and two grooves, the shaft
    and a hex prop nut."""
    base = cut(cyl_x(9.0, -1.6, 0, 0, 0),
               *[cyl_x(1.1, -2, 1, sy * 4.5, sz * 4.5) for sy in (-1, 1) for sz in (-1, 1)])
    teeth = [box(-2.4, -1.6, -0.7, 0.7, 4.0, 7.8).rotate(V(0, 0, 0), V(1, 0, 0), a) for a in range(0, 360, 30)]
    coil = fuse(cyl_x(4.6, -2.4, -1.6, 0, 0), *teeth)
    bell = (cq.Workplane("XY")
            .polyline([(-2.4, 0), (-2.4, 9.0), (-11.0, 9.0), (-11.8, 8.3), (-12.2, 7.4), (-12.2, 0)])
            .close().revolve(360, (0, 0, 0), (1, 0, 0)).val())
    vents = [cq.Workplane("YZ").workplane(offset=-12.6)
             .center(4.8 * math.cos(math.radians(a)), 4.8 * math.sin(math.radians(a)))
             .slot2D(4.6, 2.2, a + 90).extrude(1.8).val() for a in range(0, 360, 60)]
    grooves = [cut(cyl_x(9.5, x, x + 0.35, 0, 0), cyl_x(8.75, x - 1, x + 2, 0, 0)) for x in (-4.6, -9.0)]
    bell = cut(bell, *vents, *grooves)
    shaft = cyl_x(2.5, -17.6, -12.2, 0, 0)
    nut = cq.Workplane("YZ").workplane(offset=-19.8).polygon(6, 9.2).extrude(2.2).val()
    return {"": bell, "base": base, "coil": coil, "shaft": shaft, "nut": nut}


PROP_R, PROP_PITCH = 50.8, 63.5                                       # 4 in diameter, 2.5 in pitch
# radius, chord, thickness (mm). The root stays narrow so the twisted blade
# clears the motor bell behind it.
PROP_STATIONS = ((3.5, 3.6, 0.9), (6.0, 4.6, 1.0), (7.8, 5.5, 1.0), (9.5, 6.6, 1.0), (14.0, 8.8, 0.95),
                 (22.0, 9.6, 0.9), (32.0, 8.8, 0.75), (42.0, 6.8, 0.6), (50.0, 3.4, 0.4), (PROP_R, 1.2, 0.3))


def prop_section(r, chord, thick):
    """Cambered blade section at radius r, twisted to the local pitch angle.
    The disc is in the y-z plane and the suction face looks toward -x."""
    beta = math.atan2(PROP_PITCH, 2 * math.pi * r)
    sb, cb = math.sin(beta), math.cos(beta)

    def pt(s, side):                                                  # s: 0 at the leading edge
        env = (4 * s * (1 - s)) ** 0.7 if 0 < s < 1 else 0.0
        w = 0.05 * chord * 4 * s * (1 - s) + side * thick * env / 2
        u = (0.4 - s) * chord
        return V(-sb * u - cb * w, r, cb * u - sb * w)
    mid = (0.15, 0.35, 0.55, 0.75, 0.92)
    pts = [pt(0.0, 0)] + [pt(s, 1) for s in mid] + [pt(1.0, 0)] + [pt(s, -1) for s in reversed(mid)]
    return cq.Wire.makePolygon(pts, close=True)


def prop_4x25():
    """4x2.5 two-blade prop, hub centred on the origin, disc in the y-z plane.
    Twisted, cambered blades lofted through ten stations."""
    wires = [prop_section(*st) for st in PROP_STATIONS]
    blade = cq.Solid.makeLoft(wires, False)
    if not blade.isValid():
        blade = cq.Solid.makeLoft(wires, True)
    blades = fuse(blade, blade.rotate(V(0, 0, 0), V(1, 0, 0), 180))
    hub = cq.Workplane("YZ").workplane(offset=-2.5).circle(4.5).extrude(5.0).edges().fillet(1.0).val()
    return {"": cut(fuse(blades, hub), cyl_x(2.55, -3, 3, 0, 0))}


def nano_camera():
    """14 mm nano FPV camera. Lens on -x, body centred on the origin. Rounded
    housing, threaded lens barrel with a glass disc, and a PCB on the back with
    three solder pads."""
    housing = rbox(0, 10.8, -7, 7, -7, 7, 1.6, "|X")
    barrel = cut(cyl_x(4.2, -4.0, -1.6, 0, 0),
                 *[cut(cyl_x(4.5, x, x + 0.3, 0, 0), cyl_x(3.85, x - 1, x + 2, 0, 0))
                   for x in (-3.4, -2.8, -2.2)])                       # thread grooves
    body = fuse(housing, cyl_x(5.4, -1.6, 0.4, 0, 0), barrel)
    pcb = rbox(10.8, 12.0, -6, 6, -6, 6, 1.0, "|X")
    pads = fuse(*[box(12.0, 12.08, y - 1.0, y + 1.0, -4.0, -2.0) for y in (-3.0, 0.0, 3.0)])
    return {"": body, "lens": cyl_x(3.0, -4.2, -3.9, 0, 0), "pcb": pcb, "pads": pads}


def vtx_card(w=20.0, h=19.0):
    """5.8 GHz AIO video transmitter, a 20 x 19 x 3 mm card lying in the x-y
    plane on z = 0..3, with an RF shield each side, a row of solder pads on
    the +y edge and an MMCX socket on the -x edge at y = 2.7."""
    pcb = rbox(-w / 2, w / 2, -h / 2, h / 2, 1.0, 2.0, 1.2, "|Z")
    chips = fuse(box(-2, 8, -8, 1, 2.0, 3.0),                          # RF shield, top
                 box(-8, -4, 3, 7, 2.0, 2.8), box(-8, -5, -6, -2, 2.0, 2.7),
                 box(-3, 7, -7, 0, 0.0, 1.0), box(-8, -4, 2, 6, 0.3, 1.0))
    pads = fuse(*[box(-7 + 2.8 * k, -7 + 2.8 * k + 1.8, h / 2 - 1.3, h / 2, 2.0, 2.06) for k in range(6)])
    socket = fuse(cyl_x(1.5, -w / 2 - 1.6, -w / 2 + 0.5, 2.7, 1.5), cyl_x(1.0, -w / 2 - 1.6, -w / 2 - 1.0, 2.7, 1.5))
    return {"": pcb, "chips": chips, "pads": pads, "conn": socket}


def receiver_nano():
    """ELRS 2.4 GHz nano receiver, a 10 x 10 x 3 mm card lying in the x-y
    plane: shield can, ceramic antenna chip and four pads."""
    pcb = rbox(-5, 5, -5, 5, 1.0, 1.8, 0.8, "|Z")
    chips = fuse(box(-4, 1, -4, 1, 1.8, 2.9), box(-1.6, 1.6, 3.0, 4.6, 1.8, 2.7), box(-3, 3, -3, 3, 0.3, 1.0))
    pads = fuse(*[box(-3.6 + 2.4 * k, -3.6 + 2.4 * k + 1.2, -5.0, -3.5, 1.8, 1.86) for k in range(4)])
    return {"": pcb, "chips": chips, "pads": pads}


def whip_antenna(length=38.0):
    """5.8 GHz whip on +z from the origin: gold right-angle plug (3.4 mm),
    thin coax to z = 11, then the black radiator tube with a rounded tip."""
    tube = fuse(cyl_z(1.3, 11.0, length - 1.3, 0, 0), ball(1.3, 0, 0, length - 1.3))
    return {"": fuse(cyl_z(0.6, 3.0, 11.2, 0, 0), tube), "conn": cyl_z(1.7, 0, 3.4, 0, 0)}


def flight_controller(k: Kit = Kit()):
    """Matek F405-WMN class wing FC: a 31 x 26 mm board on 22 x 22 mm holes,
    lying in the x-y plane on z = 0..1.6 (long edge along x). Pads (battery,
    12 outputs) are on the underside. On top: the F405, IMU, barometer, OSD
    and flash, two BEC inductors, two capacitors, a JST-SH port, the DFU
    button and a USB-C socket on the +x edge."""
    lx, ly, hh = k.fc_len, k.fc_wid, k.fc_holes / 2
    pcb = rbox(-lx / 2, lx / 2, -ly / 2, ly / 2, 0, 1.6, 1.5, "|Z")
    pcb = cut(pcb, *[cyl_z(1.0, -1, 2.6, sx * hh, sy * hh) for sx in (-1, 1) for sy in (-1, 1)])
    t = 1.6
    chips = fuse(
        box(-6, 4, -4.5, 5.5, t, t + 1.4),                             # STM32F405
        box(5.6, 8.6, 4.9, 7.4, t, t + 0.9),                           # IMU
        box(-2, 0, 7.0, 9.0, t, t + 0.9),                              # barometer
        box(-13.5, -7.5, -5.5, 0.5, t, t + 1.0),                       # AT7456E OSD
        box(4.5, 9.5, -8.0, -4.0, t, t + 1.6),                         # blackbox flash
        rbox(-14.5, -6.5, 4.5, 12.0, t, t + 4.5, 0.6, "|Z"),           # 5 A servo BEC inductor
        rbox(-14.0, -9.0, -12.5, -7.5, t, t + 3.0, 0.5, "|Z"),         # 5 V BEC inductor
        cyl_z(3.15, t, t + 5.4, 12.0, 9.0), cyl_z(3.15, t, t + 5.4, 12.0, -9.0))     # capacitors
    usb = cut(rbox(lx / 2 - 7.0, lx / 2 + 1.0, -4.5, 4.5, t, t + 3.3, 0.5, "|X"),
              box(lx / 2 - 0.1, lx / 2 + 1.5, -3.6, 3.6, t + 0.7, t + 2.6))
    metal = fuse(usb, rbox(2.0, 5.6, 7.8, 10.2, t, t + 1.5, 0.3, "|Z"))          # USB-C, DFU button
    jst = box(-3.5, 5.5, -12.9, -8.7, t, t + 3.0)                       # JST-SH 6-pin
    pads = fuse(*[box(-7.5 + 3.0 * n - 1.0, -7.5 + 3.0 * n + 1.0, sy * 11.4 - 1.1, sy * 11.4 + 1.1, -0.08, 0)
                  for n in range(6) for sy in (-1, 1)],                        # clear of the four posts
                *[box(-lx / 2 + 1.0, -lx / 2 + 5.0, sy * 3.3 - 1.8, sy * 3.3 + 1.8, -0.08, 0) for sy in (-1, 1)])
    return {"": pcb, "chips": chips, "metal": metal, "jst": jst, "pads": pads}


def fc_keepout(k: Kit = Kit()):
    """The clearance envelope vendors quote for the FC (31 x 26 x 16.5 mm).
    Used only by the interference check; it is not exported."""
    return box(-k.fc_len / 2, k.fc_len / 2, -k.fc_wid / 2, k.fc_wid / 2, 0, k.fc_hgt)


def esc_xrotor30(k: Kit = Kit()):
    """Hobbywing XRotor Micro 30A: a 23.8 x 14.5 mm card, 5.8 mm thick, lying
    in the x-y plane (long edge along x, z = 0..5.8). Battery pads with two
    short 20 AWG wires at -x, three motor tabs at +x. MOSFETs on both sides
    and a low-profile capacitor lying across the top."""
    L, W, T = k.esc_len, k.esc_wid, k.esc_thk
    zb, pt = 0.9, 1.1
    top = zb + pt
    pcb = rbox(-L / 2, L / 2, -W / 2, W / 2, zb, top, 1.0, "|Z")
    fets = [(x, y) for x in (2.5,) for y in (-5.0, 0.0, 5.0)]
    chips = fuse(*[box(x - 1.65, x + 1.65, y - 1.65, y + 1.65, top, top + 0.9) for x, y in fets],   # top FETs
                 *[box(x - 1.65, x + 1.65, y - 1.65, y + 1.65, 0, zb) for x, y in fets],           # bottom FETs
                 box(-4.5, -1.5, 2.5, 5.5, top, top + 0.9),                                        # MCU
                 box(-5.0, -1.0, -5.0, -1.5, top, top + 0.9),                                      # gate drivers
                 *[box(6.0, 7.2, y - 1.0, y + 1.0, top, top + 0.8) for y in (-4.0, 0.0, 4.0)])     # ceramics
    cap = cyl_y((T - top) / 2, -4.5, 4.5, -7.0, top + (T - top) / 2)
    pads = fuse(*[box(-L / 2 + 0.5, -L / 2 + 3.3, y - 1.8, y + 1.8, top, top + 0.15) for y in (-3.2, 3.2)],
                *[box(L / 2 - 3.5, L / 2, y - 1.6, y + 1.6, top, top + 0.15) for y in (-4.6, 0.0, 4.6)])
    wires = fuse(*[rod(1.0, (-L / 2 + 1.5, y, top + 1.15), (-L / 2 - 6.0, y, top + 1.15)) for y in (-3.2, 3.2)])
    return {"": pcb, "chips": chips, "conn": cap, "pads": pads, "wires": wires}


def battery_2s(length, width, height):
    """2S LiPo pack centred on the origin, with a label recess on top. The
    red and black leads leave the -x end and fold back over the top to a
    yellow XT30; a white JST-XH balance plug sits at the front corner."""
    pack = cq.Workplane().box(length, width, height).edges("|X").fillet(2.0)
    try:
        pack = pack.faces(">X or <X").edges().fillet(0.6)
    except Exception:
        pass
    x0, top = -length / 2, height / 2
    pack = cut(pack.val(), box(-14, 22, -9, 9, top - 0.2, top + 0.1))
    leads = fuse(*[wire_path(1.0, [(x0 + 0.5, y, top - 4.0), (x0 - 3.0, y, top - 4.0),
                                   (x0 - 3.0, y, top + 1.2), (x0 + 6.5, y, top + 1.2)]) for y in (-2.5, 2.5)])
    xt30 = cut(rbox(x0 + 6, x0 + 14, -5, 5, top, top + 5, 0.4, "|X"),
               *[cyl_x(1.05, x0 + 10, x0 + 15, y, top + 2.5) for y in (-2.5, 2.5)])
    xt30 = xt30.cut(box(x0 + 5, x0 + 15, 3.5, 6, top + 3.5, top + 6)).clean()          # keying chamfer
    jst = box(x0 - 6, x0 - 1, 7.0, 14.5, top - 9.0, top - 3.3)
    return {"": pack, "leads": leads, "xt30": xt30, "jst": jst}


def micro_horn():
    """Glue-in control horn, base on z = 0, arm on -z, hole row along x."""
    arm = cq.Workplane("XZ").polyline([(0, 0.8), (9, 0.8), (1.2, -10), (0, -10)]).close() \
        .extrude(0.6, both=True).val()
    return {"": cut(arm, *[cyl_y(0.5, -2, 2, 1.8, z) for z in (-5.5, -8.0)])}
