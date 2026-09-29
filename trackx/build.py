#!/usr/bin/env python3
"""Patria TRACKX, exterior only: a parametric CadQuery model.

    python3 trackx/build.py [--scale 43]   # STEP, GLB and STL into trackx/out/

Coordinates are millimetres, full size: x forward, y left, z up, ground at z = 0.

Sources: published numbers (length just over 7 m, width just under 3 m, 2.0 m to
the roof plate, 56 cm tracks, 55 cm ground clearance, front sprocket, rear idler,
six dual road wheels and two return rollers a side) and three reference photos:
the front on snow, the rear at DSEI, the front in a forest. From them:
  * plan: a narrow central hull (bonnet and cab) between the two tracks, and a much
    wider troop box behind it, chamfered at the corners, with the crew doors on the
    forward chamfers and lights on the rear ones;
  * front: black bumper with wing plates over the tracks, headlights at the bonnet
    corners, tow shackles on the lower plate, louvred bonnet with a centre hatch, a
    very wide, low windscreen with three wipers, mirrors on outrigger frames;
  * side: sloped upper armour with a rail, bolt rows, long guards over the tracks;
  * wheels: dual road wheels with dished rims and no bolts, rear mud flaps.
Sizes are read off the photos against the published width and clearance, so they
are estimates. The roof weapon station, drone jammer, antennas and interior are
left off.
"""
import math
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import Vector as V

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

# ---------------------------------------------------------------- dimensions
X_REAR = -3400.0                             # rear face of the troop box
X_BOX_FRONT = 2200.0                         # front face of the troop box
X_NOSE = 3450.0                              # lower front plate
X_BUMPER = 3670.0
BOX_HW = 1450.0                              # troop box half-width
CAB_HW = 900.0                               # central hull half-width (between the tracks)
Z_ROOF = 2000.0
Z_CAB_ROOF = 1975.0
Z_SPONSON = 1100.0                           # underside of the troop box and the track guards
Z_BELLY = 550.0                              # ground clearance
TRACK_W, TRACK_T = 560.0, 45.0
TRACK_Y0, TRACK_Y1 = 915.0, 1475.0           # outer track edge sets the 2.95 m width
TRACK_YC = (TRACK_Y0 + TRACK_Y1) / 2

WHEEL_R = 300.0
WHEEL_X = [2250.0 - 940.0 * i for i in range(6)]
IDLER = (-3150.0, TRACK_T + 340.0, 340.0)    # x, z, radius (inner path radius)
SPROCKET = (3000.0, 650.0, 350.0)
ROLLER_R = 95.0
ROLLER_X = [-1000.0, 1300.0]
CLEAT_PITCH = 190.0

# colours
OLIVE, DARK_OLIVE = (0.17, 0.21, 0.13), (0.12, 0.15, 0.10)
RUBBER, STEEL, GLASS = (0.05, 0.05, 0.055), (0.42, 0.44, 0.44), (0.07, 0.11, 0.14)
LIGHT, RED = (0.92, 0.92, 0.86), (0.75, 0.06, 0.05)
BLACK, RIM_GREEN = (0.06, 0.06, 0.065), (0.20, 0.26, 0.16)


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    x0, x1, y0, y1, z0, z1 = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1)
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_x(r, x0, x1, y, z):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def prism_x(pts_yz, x0, x1):
    wire = cq.Wire.makePolygon([V(x0, y, z) for y, z in pts_yz], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(x1 - x0, 0, 0))


def prism_z(pts_xy, z0, z1):
    wire = cq.Wire.makePolygon([V(x, y, z0) for x, y in pts_xy], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, 0, z1 - z0))


def spin_z(shape, deg):
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), deg)


def cyl_y(r, y0, y1, x, z):
    return cq.Solid.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_z(r, z0, z1, x, y):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def rod(r, p0, p1):
    a, b = V(*p0), V(*p1)
    return cq.Solid.makeCylinder(r, (b - a).Length, a, b - a)


def prism_y(pts_xz, y0, y1):
    wire = cq.Wire.makePolygon([V(x, y0, z) for x, z in pts_xz], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(0, y1 - y0, 0))


def fuse(*shapes):
    if len(shapes) == 1:
        return shapes[0]
    return shapes[0].fuse(*shapes[1:]).clean()


def cut(shape, *tools):
    return shape.cut(*tools).clean()


def spin_y(shape, deg, origin=(0, 0, 0)):
    """Rotate about a line parallel to y through `origin`. Positive turns z toward x."""
    o = V(*origin)
    return shape.rotate(o, o + V(0, 1, 0), deg)


def mirror_y(shape):
    return shape.mirror("XZ")


def chamfered(solid, selector, size):
    try:
        return cq.Workplane().add(solid).edges(selector).chamfer(size).val()
    except Exception as e:
        print(f"WARNING: chamfer {size} skipped ({e})")
        return solid


def revolve_y(profile_ry, cx, cz):
    """Solid of revolution about a y-parallel axis through (cx, cz). Profile points are (r, y)."""
    s = cq.Workplane("XY").polyline(profile_ry).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()
    return s.translate(V(cx, 0, cz))


def disc(cx, cz, r, y0, y1, ch=0.0):
    if ch <= 0:
        return cyl_y(r, y0, y1, cx, cz)
    return revolve_y([(0, y0), (r - ch, y0), (r, y0 + ch), (r, y1 - ch), (r - ch, y1), (0, y1)], cx, cz)


def convex_hull(pts):
    pts = sorted(set((round(x, 4), round(z, 4)) for x, z in pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-9:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-9:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]            # counter-clockwise in (x, z)


def circle_pts(cx, cz, r, n=72):
    return [(cx + r * math.cos(2 * math.pi * i / n), cz + r * math.sin(2 * math.pi * i / n)) for i in range(n)]


# ---------------------------------------------------------------- running gear
def wheel_circles():
    zc = TRACK_T + WHEEL_R
    return [(x, zc, WHEEL_R) for x in WHEEL_X]


def track_loop(offset):
    """Convex belt path round the sprocket, idler and road wheels, offset outward by `offset`."""
    circles = wheel_circles() + [(IDLER[0], IDLER[1], IDLER[2]), (SPROCKET[0], SPROCKET[1], SPROCKET[2])]
    pts = []
    for cx, cz, r in circles:
        pts += circle_pts(cx, cz, r + offset)
    return convex_hull(pts)


def hull_z_at(poly, x, top=True):
    """Height of the polygon's upper (or lower) edge at x."""
    zs = []
    n = len(poly)
    for i in range(n):
        (x0, z0), (x1, z1) = poly[i], poly[(i + 1) % n]
        if (x0 - x) * (x1 - x) <= 0 and x0 != x1:
            zs.append(z0 + (z1 - z0) * (x - x0) / (x1 - x0))
    return max(zs) if top else min(zs)


def make_track(side):
    """One track: rubber band, centre guide ridge and cross cleats."""
    inner, outer = track_loop(0.0), track_loop(TRACK_T)
    y0, y1 = TRACK_Y0, TRACK_Y1
    band = cut(prism_y(outer, y0, y1), prism_y(inner, y0 - 1, y1 + 1))
    ridge_in = track_loop(-32.0)
    ridge = cut(prism_y(inner, TRACK_YC - 32, TRACK_YC + 32), prism_y(ridge_in, TRACK_YC - 33, TRACK_YC + 33))
    # cleats along the outer surface
    n = len(outer)
    seg = [(outer[i], outer[(i + 1) % n]) for i in range(n)]
    lens = [math.dist(a, b) for a, b in seg]
    total = sum(lens)
    count = int(total // CLEAT_PITCH)
    cleats = []
    for k in range(count):
        s = (k + 0.5) * total / count
        i = 0
        while s > lens[i]:
            s -= lens[i]
            i += 1
        (xa, za), (xb, zb) = seg[i]
        t = s / lens[i]
        px, pz = xa + (xb - xa) * t, za + (zb - za) * t
        tx, tz = (xb - xa) / lens[i], (zb - za) / lens[i]
        ang = math.degrees(math.atan2(-tz, tx))
        c = box(-30, 30, y0 + 20, y1 - 20, -32, 5)             # local z < 0 points outward
        cleats.append(spin_y(c, ang).translate(V(px, 0, pz)))
    lugs = cq.Compound.makeCompound(cleats)
    track = fuse(band, lugs)
    ridge = ridge
    if side < 0:
        track, ridge = mirror_y(track), mirror_y(ridge)
    return track, ridge


def toothed_disc(cx, cz, y0, y1, teeth=17, r_tip=352.0, r_root=318.0):
    pts = []
    for i in range(teeth):
        a0 = 2 * math.pi * i / teeth
        step = 2 * math.pi / teeth
        for f, r in ((0.0, r_root), (0.22, r_root), (0.34, r_tip), (0.66, r_tip), (0.78, r_root)):
            a = a0 + f * step
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    return prism_y(pts, y0, y1)


def ring_y(cx, cz, r_out, r_in, y0, y1, ch=0.0):
    """Tyre: an annulus with rounded shoulders."""
    return revolve_y([(r_in, y0), (r_out - ch, y0), (r_out, y0 + ch), (r_out, y1 - ch), (r_out - ch, y1), (r_in, y1)], cx, cz)


def wheel(cx, cz, r, yc, tyre_w=170.0):
    """Dual wheel: two tyres with dished rims that sit inside the tyre's outer face."""
    tyres, rims, caps = [], [], []
    for dy in (-120.0, 120.0):
        ya, yb = yc + dy - tyre_w / 2, yc + dy + tyre_w / 2
        tyres.append(ring_y(cx, cz, r, r - 62, ya, yb, 18))
        rims.append(disc(cx, cz, r - 60, ya + 28, yb - 36, 6))                     # dished rim
        rims.append(ring_y(cx, cz, r - 52, r - 78, yb - 36, yb - 12, 4))               # rim lip
    caps.append(disc(cx, cz, 70, yc + 120 + tyre_w / 2 - 36, yc + 120 + tyre_w / 2 - 14, 6))
    return tyres, rims, caps


def make_running_gear(side):
    """Wheels, rims, sprocket, idler, return rollers and swing arms for one side (left, y > 0)."""
    yc = TRACK_YC
    tyres, rims, hubs, arms = [], [], [], []
    for x, z, r in wheel_circles():
        t, rm, c = wheel(x, z, r, yc)
        tyres += t; rims += rm; hubs += c
        # inboard axle and swing arm
        arms.append(cyl_y(48, TRACK_Y0 - 70, yc - 205, x, z))
        arms.append(rod(46, (x, TRACK_Y0 - 70, z), (x + 430, TRACK_Y0 - 70, z + 300)))
        arms.append(cyl_y(60, TRACK_Y0 - 120, TRACK_Y0 - 30, x + 430, z + 300))
    ix, iz, ir = IDLER
    t, rm, c = wheel(ix, iz, ir - 4, yc)
    tyres += t; rims += rm; hubs += c
    arms.append(cyl_y(55, TRACK_Y0 - 70, yc - 205, ix, iz))
    arms.append(rod(50, (ix, TRACK_Y0 - 70, iz), (ix + 450, TRACK_Y0 - 70, iz + 420)))
    # return rollers under the top run
    inner = track_loop(0.0)
    rollers = []
    for x in ROLLER_X:
        z = hull_z_at(inner, x) - ROLLER_R
        rollers.append(disc(x, z, ROLLER_R, yc - 60, yc + 60, 6))
        arms.append(rod(24, (x, yc - 60, z), (x, TRACK_Y0 - 60, z + 130)))
    # sprocket
    sx, sz, _ = SPROCKET
    sprocket = fuse(toothed_disc(sx, sz, yc - 205, yc - 120), toothed_disc(sx, sz, yc + 120, yc + 205),
                    disc(sx, sz, 240, yc - 205, yc + 205, 8), disc(sx, sz, 95, yc + 205, yc + 232, 8))
    arms.append(cyl_y(60, TRACK_Y0 - 90, yc - 205, sx, sz))
    groups = {
        "tyres": fuse(*tyres), "rims": fuse(*rims), "hubs": fuse(*hubs),
        "rollers": fuse(*rollers), "sprocket": sprocket, "arms": fuse(*arms),
    }
    if side < 0:
        groups = {k: mirror_y(v) for k, v in groups.items()}
    return groups


# ---------------------------------------------------------------- hull
WS_T, WS_B = (2870.0, 1960.0), (2960.0, 1650.0)             # windscreen plane, top and bottom, x-z
WS_DEG = math.degrees(math.atan2(WS_T[1] - WS_B[1], WS_B[0] - WS_T[0]))
WS_LEN = math.dist(WS_T, WS_B)
DECK_F, DECK_B = (X_NOSE, 1380.0), (WS_B[0], WS_B[1])       # bonnet deck, front and back edge
DECK_LEN = math.dist(DECK_F, DECK_B)
DECK_DEG = math.degrees(math.atan2(-(DECK_B[1] - DECK_F[1]), DECK_B[0] - DECK_F[0]))       # -151 deg
CF = 320.0                                                   # forward chamfer of the troop box (plan)
CR = 220.0                                                   # rear chamfer
CF_LEN = math.hypot(CF, CF)


def ws_box(u0, u1, y0, y1, n0, n1):
    """Box on the windscreen plane: u from the top edge down, y across, n along the outward normal."""
    return spin_y(box(u0, u1, y0, y1, n0, n1), WS_DEG).translate(V(WS_T[0], 0, WS_T[1]))


def deck_box(u0, u1, y0, y1, h0, h1):
    """Box on the bonnet deck: u from the front edge backwards, h = height out of the deck."""
    return spin_y(box(u0, u1, y0, y1, -h1, -h0), DECK_DEG).translate(V(DECK_F[0], 0, DECK_F[1]))


def cface_box(u0, u1, h0, h1, z0, z1):
    """Box on the left forward chamfer of the troop box: u along the face, h out of it."""
    return spin_z(box(u0, u1, -h1, -h0, z0, z1), 135).translate(V(X_BOX_FRONT, BOX_HW - CF, 0))


def rface_box(u0, u1, h0, h1, z0, z1):
    """Box on the left rear chamfer: u along the face, h out of it."""
    return spin_z(box(u0, u1, h0, h1, z0, z1), 45).translate(V(X_REAR, BOX_HW - CR, 0))


def corner_cutter(x, y, sx, sy, leg):
    """Triangle prism that removes a plan-view corner (sx, sy point into the hull)."""
    p1, p2, c = (x + sx * leg, y), (x, y + sy * leg), (x, y)
    def ext(a, b, t=0.25):
        return (a[0] + (a[0] - b[0]) * t, a[1] + (a[1] - b[1]) * t)
    out = (2 * c[0] - (p1[0] + p2[0]) / 2, 2 * c[1] - (p1[1] + p2[1]) / 2)
    return prism_z([ext(p1, p2), ext(p2, p1), out], 1000, 2100)


def make_hull():
    # troop box: sloped upper sides, chamfered corners
    box_hull = prism_x([(-BOX_HW, Z_SPONSON), (BOX_HW, Z_SPONSON), (BOX_HW, 1720.0), (1250.0, Z_ROOF),
                        (-1250.0, Z_ROOF), (-BOX_HW, 1720.0)], X_REAR, X_BOX_FRONT)
    cutters = []
    for sy in (1, -1):
        cutters.append(corner_cutter(X_REAR, sy * BOX_HW, 1, -sy, CR))
        cutters.append(corner_cutter(X_BOX_FRONT, sy * BOX_HW, -1, -sy, CF))
    box_hull = cut(box_hull, *cutters)

    # central hull: tub, lower front plate, bonnet, cab
    spine_pts = [(-3330.0, Z_BELLY), (3330.0, Z_BELLY), (X_NOSE, 700.0), (X_NOSE, DECK_F[1]), DECK_B,
                 WS_T, (WS_T[0] - 60, Z_CAB_ROOF), (X_BOX_FRONT - 100, Z_CAB_ROOF), (X_BOX_FRONT - 100, Z_SPONSON),
                 (-3330.0, Z_SPONSON)]
    spine = prism_y(spine_pts, -CAB_HW, CAB_HW)
    spine = chamfered(spine, cq.selectors.BoxSelector((-3400, -1000, 540), (3400, 1000, 560)), 45)
    spine = chamfered(spine, cq.selectors.BoxSelector((2000, CAB_HW - 10, Z_CAB_ROOF - 10), (3000, CAB_HW + 10, Z_CAB_ROOF + 10)), 90)
    spine = chamfered(spine, cq.selectors.BoxSelector((2000, -CAB_HW - 10, Z_CAB_ROOF - 10), (3000, -CAB_HW + 10, Z_CAB_ROOF + 10)), 90)
    hull = fuse(box_hull, spine)

    cuts = []
    # windscreen: one wide, low pane
    cuts.append(ws_box(28, WS_LEN - 26, -745, 745, -26, 6))
    # bonnet louvres: two panels of six slots
    for yc in (420.0, -420.0):
        for k in range(6):
            u = 110 + 62 * k
            cuts.append(deck_box(u, u + 30, yc - 250, yc + 250, -10, 4))
    # crew doors on the forward chamfers (left; mirrored below): outline grooves and window recess
    doors = []
    u0, u1, z0, z1 = 22.0, CF_LEN - 22, 1160.0, 1940.0
    for (a, b, c, d) in ((u0, u1, z0, z0 + 14), (u0, u1, z1 - 14, z1), (u0, u0 + 14, z0, z1), (u1 - 14, u1, z0, z1)):
        doors.append(cface_box(a, b, -9, 2, c, d))
    doors.append(cface_box(100, 350, -20, 2, 1500, 1860))
    for dcut in doors:
        cuts += [dcut, mirror_y(dcut)]
    # rear door, hinged on the left: outline grooves
    for (a, b, c, d) in ((-700, 700, 1160, 1174), (-700, 700, 1900, 1914), (-700, -686, 1160, 1914), (686, 700, 1160, 1914)):
        cuts.append(box(X_REAR - 2, X_REAR + 9, a, b, c, d))
    return cut(hull, *cuts)


def make_glass():
    panes = [ws_box(36, WS_LEN - 34, -738, 738, -19, -7)]
    win = cface_box(100, 350, -16, -4, 1500, 1860)
    panes += [win, mirror_y(win)]
    return fuse(*panes)


def make_fittings():
    """Frames, hatches, handles, wipers, bolts and rails: the small olive and dark parts."""
    parts = []
    # windscreen frame and eyebrow, three wipers
    frame = cut(ws_box(14, WS_LEN + 6, -830, 830, 0, 30), ws_box(30, WS_LEN - 30, -750, 750, -1, 32))
    parts.append(frame)
    for y in (-480.0, 20.0, 520.0):
        parts.append(spin_y(ws_box(WS_LEN - 70, WS_LEN - 50, y, y + 12, 24, 34), 0)
                     if False else ws_box(60, 300, y - 6, y + 6, 20, 30))
    # bonnet: raised centre hatch with two handles, hinge lugs at the back
    hatch = deck_box(140, 440, -430, 430, 0, 55)
    parts.append(hatch)
    parts += [deck_box(205, 240, y0, y0 + 90, 55, 85) for y0 in (-320, 230)]
    # headlight housings at the bonnet corners
    for s in (1, -1):
        parts.append(box(3380, 3535, s * 800 - 100, s * 800 + 100, 1385, 1540))
    # roof: three raised blocks at the front of the roof, a round rear hatch, lifting eyes
    for y in (-560.0, 0.0, 560.0):
        parts.append(box(2380, 2780, y - 190, y + 190, Z_CAB_ROOF - 5, Z_CAB_ROOF + 42))
    ring = cut(cyl_z(390, Z_ROOF - 10, Z_ROOF + 50, -2500, 650), cyl_z(340, Z_ROOF + 15, Z_ROOF + 60, -2500, 650))
    parts.append(fuse(ring, cyl_z(345, Z_ROOF + 30, Z_ROOF + 58, -2500, 650)))
    for x, y in ((-2900, 1000), (-2900, -1000), (1300, 1000), (1300, -1000)):
        parts.append(cyl_z(45, Z_ROOF, Z_ROOF + 22, x, y))
    # door handles on the chamfers (mirrored)
    hd = cface_box(CF_LEN - 90, CF_LEN - 40, 0, 20, 1470, 1600)
    parts += [hd, mirror_y(hd)]
    # rear door handle and hinges (hinged on the left)
    parts.append(box(X_REAR - 22, X_REAR - 2, 500, 620, 1520, 1560))
    for z in (1230, 1850):
        parts.append(box(X_REAR - 16, X_REAR - 2, 640, 700, z - 25, z + 25))
    # side rails where the sloped armour starts
    for s in (1, -1):
        parts.append(box(-3200, 1750, s * (BOX_HW + 2), s * (BOX_HW + 48), 1690, 1750))
    return fuse(*parts)


def make_bolts():
    """Rows of armour bolt heads on the vertical side walls of the troop box."""
    bolts = []
    for x in np.arange(-2900.0, 1600.0, 400.0):
        for z in (1250.0, 1500.0):
            bolts.append(cyl_y(17, BOX_HW - 1, BOX_HW + 11, float(x), z))
    left = fuse(*bolts)
    return fuse(left, mirror_y(left))


def make_bumper():
    """Black front bumper beam, wing plates over the tracks, and the rear mud flaps."""
    beam = prism_z([(X_NOSE - 10, -1000.0), (X_NOSE - 10, 1000.0), (X_BUMPER, 900.0), (X_BUMPER, -900.0)], Z_SPONSON, 1400.0)
    wing = prism_z([(1900.0, TRACK_Y0), (X_NOSE, TRACK_Y0), (3580.0, 1100.0), (3580.0, TRACK_Y1), (1900.0, TRACK_Y1)],
                   Z_SPONSON, Z_SPONSON + 45)
    wings = fuse(wing, mirror_y(wing))
    flap = box(X_REAR - 30, X_REAR - 6, 960, 1440, 790, Z_SPONSON)
    guard = box(-3300.0, 1900.0, BOX_HW, TRACK_Y1, Z_SPONSON - 40, Z_SPONSON)         # guard along the track top
    return fuse(beam, wings, flap, mirror_y(flap), guard, mirror_y(guard))


def make_mirrors():
    """Outrigger mirror frames: two arms, an upright and the mirror plate, each side."""
    x, y0, y1 = 2650.0, CAB_HW - 10, 1400.0
    arms = [rod(15, (x, y0, 1900.0), (x, y1, 1900.0)), rod(15, (x, y0, 1560.0), (x, y1, 1560.0)),
            rod(15, (x, y1, 1540.0), (x, y1, 1920.0)), box(x - 90, x + 90, y1 - 12, y1 + 22, 1590, 1880)]
    left = fuse(*arms)
    return fuse(left, mirror_y(left))


def make_lights():
    """Headlamp lenses, tow shackles front and rear, rear light clusters."""
    lenses = []
    for s in (1, -1):
        lenses.append(cyl_x(62, 3535, 3552, s * 800, 1462))
    shackles = []
    for x0, sgn, ys in ((X_NOSE, 1, (570.0, -570.0)), (X_REAR, -1, (450.0, -450.0))):
        for y in ys:
            xa, xb = sorted((x0, x0 + sgn * 38))
            shackles.append(cut(cyl_x(75, xa, xb, y, 830), cyl_x(42, xa - 5, xb + 5, y, 830)))
            xl, xm = sorted((x0, x0 + sgn * 38))
            shackles.append(box(xl, xm, y - 42, y + 42, 890, 960))
    lamp = fuse(rface_box(50, 105, 0, 16, 1180, 1300), rface_box(115, 170, 0, 16, 1180, 1300))
    lamps = fuse(lamp, mirror_y(lamp))
    return fuse(*lenses), fuse(*shackles), lamps


def build():
    parts = []                 # (name, shape, colour)
    parts.append(("hull", make_hull(), OLIVE))
    parts.append(("glass", make_glass(), GLASS))
    parts.append(("fittings", make_fittings(), DARK_OLIVE))
    parts.append(("bolts", make_bolts(), (0.22, 0.26, 0.17)))
    parts.append(("bumper", make_bumper(), BLACK))
    parts.append(("mirrors", make_mirrors(), BLACK))
    lenses, shackles, lamps = make_lights()
    parts.append(("lights", lenses, LIGHT))
    parts.append(("tow_eyes", shackles, STEEL))
    parts.append(("rear_lights", lamps, RED))
    for side, tag in ((1, "L"), (-1, "R")):
        track, ridge = make_track(side)
        parts.append((f"track_{tag}", track, RUBBER))
        parts.append((f"track_guide_{tag}", ridge, (0.10, 0.10, 0.11)))
        g = make_running_gear(side)
        parts.append((f"tyres_{tag}", g["tyres"], RUBBER))
        parts.append((f"rims_{tag}", g["rims"], RIM_GREEN))
        parts.append((f"hubs_{tag}", g["hubs"], STEEL))
        parts.append((f"rollers_{tag}", g["rollers"], (0.12, 0.12, 0.13)))
        parts.append((f"sprocket_{tag}", g["sprocket"], (0.36, 0.37, 0.35)))
        parts.append((f"arms_{tag}", g["arms"], (0.16, 0.17, 0.15)))
    return parts


# ---------------------------------------------------------------- export
def tessellate(shape, tol=0.5, ang=0.25):
    verts, tris = shape.tessellate(tol, ang)
    return np.array([(v.x, v.y, v.z) for v in verts]), np.array(tris)


def srgb_to_linear(c):
    return [((v / 12.92) if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4) for v in c]


def export(parts, scale=None):
    import trimesh
    from trimesh.visual.material import PBRMaterial
    OUT.mkdir(exist_ok=True)
    assy = cq.Assembly(name="patria_trackx")
    for name, shape, colour in parts:
        assy.add(shape, name=name, color=cq.Color(*colour))
    assy.export(str(OUT / "trackx.step"))

    scene = trimesh.Scene()
    meshes = []
    for name, shape, colour in parts:
        v, t = tessellate(shape)
        m = trimesh.Trimesh(np.column_stack([v[:, 0], v[:, 2], -v[:, 1]]) / 1000.0, t, process=False)   # y-up, metres
        m.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            name=name, baseColorFactor=[*srgb_to_linear(colour), 1.0], metallicFactor=0.0, roughnessFactor=0.7))
        scene.add_geometry(m, node_name=name, geom_name=name)
        # STL keeps the z-up frame, in millimetres
        meshes.append(trimesh.Trimesh(v, t, process=False))
    scene.export(str(OUT / "trackx.glb"), include_normals=True)
    (HERE / "viewer").mkdir(exist_ok=True)
    scene.export(str(HERE / "viewer" / "trackx.glb"), include_normals=True)
    whole = trimesh.util.concatenate(meshes)
    whole.export(str(OUT / "trackx_1to1.stl"))
    if scale:
        s = whole.copy()
        s.apply_scale(1.0 / scale)
        s.export(str(OUT / f"trackx_1to{scale}.stl"))
        print(f"scaled STL 1:{scale}: {s.bounds[1] - s.bounds[0]} mm")


def main():
    scale = None
    if "--scale" in sys.argv:
        scale = int(sys.argv[sys.argv.index("--scale") + 1])
    parts = build()
    bad = [n for n, s, _ in parts if not s.isValid()]
    if bad:
        print("WARNING: invalid solids:", bad)
    x0, x1 = min(s.BoundingBox().xmin for _, s, _ in parts), max(s.BoundingBox().xmax for _, s, _ in parts)
    y1 = max(s.BoundingBox().ymax for _, s, _ in parts)
    z1 = max(s.BoundingBox().zmax for _, s, _ in parts)
    print(f"length {x1 - x0:.0f} mm, width {2 * y1:.0f} mm, height {z1:.0f} mm, {len(parts)} bodies")
    export(parts, scale)


if __name__ == "__main__":
    main()
