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
# plan (x): bumper front 3650, lower front plate 3470, glacis front edge 3480,
# windscreen base 2990, rear corner blocks -3380, recessed rear door -3080
X_FRONT = 3480.0                             # front face of the upper hull, under the bumper
X_BUMPER = 3650.0
X_REAR = -3380.0                             # rear faces of the two corner blocks
X_DOOR = -2900.0                             # recessed rear door between them
HULL_HW = 1420.0                             # upper hull half-width (side walls)
ROOF_HW = 1330.0                             # roof half-width after the upper sides lean in
TUB_HW = 800.0                               # lower hull between the tracks
DOOR_HW = 650.0                              # half-width of the rear recess
Z_ROOF = 2000.0
Z_SPONSON = 1080.0                           # underside of the upper hull, just above the tracks
Z_BELLY = 550.0                              # ground clearance
Z_RAIL = 1760.0                              # side rail; the side leans in above it
TRACK_W, TRACK_T = 560.0, 45.0
TRACK_Y0, TRACK_Y1 = 850.0, 1410.0           # tracks sit just inboard of the hull sides
TRACK_YC = (TRACK_Y0 + TRACK_Y1) / 2

TOP_IN = 985.0                               # inside of the level top run
# the sprocket and idler are raised, so the lower run climbs to them at each end while the top run stays level
SPROCKET = (3080.0, TOP_IN - 420.0, 420.0)   # x, z, radius (inner path radius)
IDLER = (-2900.0, TOP_IN - 380.0, 380.0)
WHEEL_R = 370.0
WHEEL_X = [2250.0 - 874.0 * i for i in range(6)]
ROLLER_R = 95.0
ROLLER_X = [-850.0, 1150.0]
CLEAT_PITCH = 170.0

# colours (sRGB)
OLIVE, DARK_OLIVE = (0.20, 0.235, 0.175), (0.13, 0.15, 0.115)
RUBBER, STEEL, GLASS = (0.05, 0.05, 0.055), (0.42, 0.44, 0.44), (0.07, 0.11, 0.14)
LIGHT, RED = (0.92, 0.92, 0.86), (0.75, 0.06, 0.05)
BLACK, RIM_GREEN = (0.08, 0.085, 0.09), (0.30, 0.33, 0.24)
GUARD, END_DARK = (0.25, 0.25, 0.24), (0.12, 0.13, 0.11)


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


def loft(sections):
    """Ruled loft through closed polygons given as lists of (x, y, z) points."""
    wires = [cq.Wire.makePolygon([V(*p) for p in sec], close=True) for sec in sections]
    return cq.Solid.makeLoft(wires, True)


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
    t, rm, c = wheel(ix, iz, ir - 8, yc)
    tyres += t; ends = rm + c                                # end wheels are dark, like in the photos
    arms.append(cyl_y(55, TRACK_Y0 - 70, yc - 205, ix, iz))
    arms.append(rod(50, (ix, TRACK_Y0 - 70, iz), (ix + 450, TRACK_Y0 - 70, iz + 420)))
    # return rollers under the top run
    inner = track_loop(0.0)
    rollers = []
    for x in ROLLER_X:
        z = hull_z_at(inner, x) - ROLLER_R
        rollers.append(disc(x, z, ROLLER_R, yc - 60, yc + 60, 6))
        arms.append(rod(24, (x, yc - 60, z), (x, TRACK_Y0 - 60, z + 130)))
    # sprocket: dual wheel, plus the toothed ring that runs in the track's centre gap
    sx, sz, sr = SPROCKET
    t, rm, c = wheel(sx, sz, sr - 8, yc)
    tyres += t; ends += rm + c
    sprocket = fuse(toothed_disc(sx, sz, yc - 34, yc + 34, teeth=24, r_tip=sr - 34, r_root=sr - 76), *ends)
    arms.append(cyl_y(60, TRACK_Y0 - 90, yc - 205, sx, sz))
    groups = {
        "tyres": fuse(*tyres), "rims": fuse(*rims), "hubs": fuse(*hubs),
        "rollers": fuse(*rollers), "sprocket": sprocket, "arms": fuse(*arms),
    }
    if side < 0:
        groups = {k: mirror_y(v) for k, v in groups.items()}
    return groups


# ---------------------------------------------------------------- hull
WS_T, WS_B = (2860.0, 1900.0), (2990.0, 1530.0)             # windscreen top and base, x-z
WS_DEG = math.degrees(math.atan2(WS_T[1] - WS_B[1], WS_B[0] - WS_T[0]))
WS_LEN = math.dist(WS_T, WS_B)
WS_HW = 930.0                                                # front face half-width (glass 850)
DECK_F, DECK_B = (X_FRONT, 1370.0), WS_B                     # glacis front and back edge
DECK_LEN = math.dist(DECK_F, DECK_B)
DECK_DEG = math.degrees(math.atan2(-(DECK_B[1] - DECK_F[1]), DECK_B[0] - DECK_F[0]))
CH_F = (WS_B[0], WS_HW)                                      # angled cab corner, front end
CH_R = (2150.0, HULL_HW)                                     # ... and rear end, on the side wall
CH_LEN = math.dist(CH_F, CH_R)
CH_D = ((CH_R[0] - CH_F[0]) / CH_LEN, (CH_R[1] - CH_F[1]) / CH_LEN)   # along the face, backwards
CH_N = (CH_D[1], -CH_D[0])                                   # outward normal (forward and out)
CR = 200.0                                                   # plan chamfer on the rear corners


def ws_point(u, y, n):
    """World point on the windscreen plane: u down from the top edge, n out of the glass."""
    dx, dz = (WS_B[0] - WS_T[0]) / WS_LEN, (WS_B[1] - WS_T[1]) / WS_LEN
    return (WS_T[0] + u * dx - n * dz, y, WS_T[1] + u * dz + n * dx)


def ws_box(u0, u1, y0, y1, n0, n1):
    """Box on the windscreen plane: u from the top edge down, y across, n along the outward normal."""
    return spin_y(box(u0, u1, y0, y1, n0, n1), WS_DEG).translate(V(WS_T[0], 0, WS_T[1]))


def deck_box(u0, u1, y0, y1, h0, h1):
    """Box on the glacis: u from the front edge backwards, h = height out of the surface."""
    return spin_y(box(u0, u1, y0, y1, -h1, -h0), DECK_DEG).translate(V(DECK_F[0], 0, DECK_F[1]))


def ch_box(u0, u1, h0, h1, z0, z1):
    """Box on the left angled cab corner: u along the face from its front end, h out of it."""
    ang = math.degrees(math.atan2(CH_D[1], CH_D[0]))
    return spin_z(box(u0, u1, -h1, -h0, z0, z1), ang).translate(V(CH_F[0], CH_F[1], 0))


def ch_point(u, h, z):
    return (CH_F[0] + u * CH_D[0] + h * CH_N[0], CH_F[1] + u * CH_D[1] + h * CH_N[1], z)


def both(shape):
    return fuse(shape, mirror_y(shape))


def make_hull():
    """Upper hull (full width over the tracks) and the lower tub between the tracks."""
    side = [(X_REAR, Z_SPONSON), (X_FRONT, Z_SPONSON), DECK_F, WS_B, WS_T, (2800.0, Z_ROOF),
            (-2650.0, Z_ROOF), (X_REAR, 1400.0)]
    section = [(-HULL_HW, Z_SPONSON), (HULL_HW, Z_SPONSON), (HULL_HW, Z_RAIL), (ROOF_HW, Z_ROOF),
               (-ROOF_HW, Z_ROOF), (-HULL_HW, Z_RAIL)]
    upper = prism_y(side, -HULL_HW, HULL_HW).intersect(prism_x(section, X_REAR - 10, X_BUMPER))
    # angled cab corners behind the shoulders; a 50 mm step is left over the tracks
    ext = (CH_R[0] + CH_D[0] * 80, CH_R[1] + CH_D[1] * 80)
    corner = prism_z([CH_F, ext, (ext[0], HULL_HW + 60), (CH_F[0], HULL_HW + 60)], Z_SPONSON + 50, Z_ROOF + 100)
    # recessed rear door between the two corner blocks; chamfered outer rear corners
    recess = box(X_REAR - 20, X_DOOR, -DOOR_HW, DOOR_HW, Z_SPONSON - 10, Z_ROOF + 50)
    rc = prism_z([(X_REAR - 30, HULL_HW - CR - 30 * 1.0), (X_REAR + CR + 30, HULL_HW + 30), (X_REAR - 30, HULL_HW + 30)],
                 Z_SPONSON - 10, Z_ROOF + 50)
    fc = prism_z([(X_FRONT + 20, HULL_HW - 170), (X_FRONT + 20, HULL_HW + 30), (X_FRONT - 260, HULL_HW + 30)],
                 Z_SPONSON - 10, Z_ROOF + 50)
    upper = cut(upper, corner, mirror_y(corner), recess, rc, mirror_y(rc), fc, mirror_y(fc))
    # lower tub, rounded at its front corners
    tub = prism_y([(X_DOOR, 610.0), (X_DOOR + 60, Z_BELLY), (3330.0, Z_BELLY), (3470.0, 700.0), (3470.0, Z_SPONSON + 20),
                   (X_DOOR, Z_SPONSON + 20)], -TUB_HW, TUB_HW)
    tub = cut(tub, *[box(3300, 3500, s * (TUB_HW - 110), s * (TUB_HW + 10), 500, 1200) for s in (1, -1)])
    corners = [cyl_z(110, 600, Z_SPONSON, 3360, s * (TUB_HW - 110)) for s in (1, -1)]
    hull = fuse(upper, tub, *corners)

    cuts = []
    cuts.append(ws_box(40, WS_LEN - 36, -850, 850, -30, 6))                        # windscreen opening
    # crew doors on the angled corners: outline grooves and the window opening
    door = [ch_box(a, b, -9, 2, c, d) for a, b, c, d in
            ((60, CH_LEN - 70, 1150, 1164), (60, CH_LEN - 70, 1946, 1960), (60, 74, 1150, 1960),
             (CH_LEN - 84, CH_LEN - 70, 1150, 1960))]
    door.append(ch_box(330, 730, -24, 2, 1560, 1860))
    cuts += door + [mirror_y(d) for d in door]
    # rear door on the recessed face, hinged on the left (+y)
    for a, b, c, d in ((-590, 590, 640, 654), (-590, 590, 1730, 1744), (-590, -576, 640, 1744), (576, 590, 640, 1744)):
        cuts.append(box(X_DOOR - 2, X_DOOR + 9, a, b, c, d))
    return cut(hull, *cuts)


def make_glass():
    panes = [ws_box(48, WS_LEN - 44, -842, 842, -22, -8)]
    win = ch_box(338, 722, -18, -6, 1568, 1852)
    return fuse(*panes, win, mirror_y(win))


def make_fittings():
    """Windscreen frame, wipers, louvres, hatches, door frames and handles, rails, roof items."""
    parts = []
    # windscreen frame and three wipers parked diagonally from pivots at the top
    parts.append(cut(ws_box(10, WS_LEN + 4, -WS_HW + 2, WS_HW - 2, 0, 26), ws_box(34, WS_LEN - 30, -860, 860, -1, 28)))
    for y in (-570.0, 0.0, 570.0):
        top, tip = ws_point(52, y + 60, 18), ws_point(300, y - 110, 18)
        parts.append(rod(8, top, tip))
        parts.append(cyl_x(16, top[0] - 4, top[0] + 30, top[1], top[2]))            # pivot cap
    # glacis louvres: two panels of raised slats
    for yc in (400.0, -400.0):
        for k in range(6):
            u = 120 + 50 * k
            parts.append(deck_box(u, u + 24, yc - 230, yc + 230, 0, 16))
    # door window frames and handles on the angled corners, grab rails beside them
    frame = cut(ch_box(300, 760, 0, 30, 1530, 1890), ch_box(335, 725, -2, 34, 1565, 1855))
    handle = ch_box(CH_LEN - 150, CH_LEN - 110, 0, 22, 1420, 1540)
    a, b = ch_point(40, 60, 1230), ch_point(40, 60, 1700)
    rail = fuse(rod(15, a, b), rod(12, ch_point(40, 0, 1260), ch_point(40, 60, 1260)),
                rod(12, ch_point(40, 0, 1670), ch_point(40, 60, 1670)))
    parts += [both(frame), both(handle), both(rail)]
    # rear door handle and hinges (hinged on the left)
    parts.append(box(X_DOOR - 22, X_DOOR - 2, -470, -350, 1180, 1220))
    for z in (760, 1180, 1600):
        parts.append(box(X_DOOR - 18, X_DOOR - 2, 540, 600, z - 30, z + 30))
    # side rails where the sides start to slope in, with rectangular slots
    rail = box(-3150.0, 2080.0, HULL_HW - 5, HULL_HW + 45, Z_RAIL - 40, Z_RAIL + 20)
    slots = [box(float(x) - 70, float(x) + 70, HULL_HW + 22, HULL_HW + 50, Z_RAIL - 28, Z_RAIL + 8)
             for x in np.arange(-3000.0, 2000.0, 250.0)]
    parts.append(both(cut(rail, *slots)))
    # roof: commander's hatch, rear round hatch, camera and lamps at the front edge, lifting eyes
    ring = cut(box(1950, 2450, -900, -350, Z_ROOF - 10, Z_ROOF + 55), box(2000, 2400, -850, -400, Z_ROOF + 10, Z_ROOF + 60))
    parts.append(fuse(ring, box(1995, 2405, -855, -395, Z_ROOF + 30, Z_ROOF + 62)))
    ring2 = cut(cyl_z(360, Z_ROOF - 10, Z_ROOF + 50, -2300, 600), cyl_z(310, Z_ROOF + 15, Z_ROOF + 60, -2300, 600))
    parts.append(fuse(ring2, cyl_z(315, Z_ROOF + 30, Z_ROOF + 58, -2300, 600)))
    parts.append(fuse(box(2640, 2760, -60, 60, Z_ROOF, Z_ROOF + 90), cyl_x(28, 2760, 2785, 0, Z_ROOF + 50),
                      cyl_z(18, Z_ROOF + 90, Z_ROOF + 150, 2700, 0)))
    parts.append(both(box(2680, 2780, 640, 760, Z_ROOF, Z_ROOF + 55)))
    for x, y in ((-2750, 1050), (-2750, -1050), (1500, 1050), (1500, -1050)):
        parts.append(cyl_z(45, Z_ROOF, Z_ROOF + 22, x, y))
    return fuse(*parts)


def make_bolts():
    """Armour bolt pads in vertical pairs along the side walls."""
    pads = [box(float(x) - 22, float(x) + 22, HULL_HW - 2, HULL_HW + 12, z - 22, z + 22)
            for x in np.arange(-2900.0, 2000.0, 380.0) for z in (1330.0, 1450.0)]
    return both(fuse(*pads))


def make_bumper():
    """Black front bumper: a straight centre beam and end pieces that drop and sweep back over
    the tracks; plus the two rubber bonnet latches."""
    prof = [(X_FRONT - 20, 1090.0), (X_BUMPER, 1090.0), (X_BUMPER, 1290.0), (X_BUMPER - 45, 1345.0), (X_FRONT - 20, 1345.0)]
    centre = prism_y(prof, -650, 650)
    end = loft([[(x, 650.0, z) for x, z in prof],
                [(X_FRONT - 60, 1450.0, 980.0), (3590.0, 1450.0, 980.0), (3590.0, 1450.0, 1160.0),
                 (3550.0, 1450.0, 1210.0), (X_FRONT - 60, 1450.0, 1210.0)]])
    latch = both(box(X_FRONT - 60, X_FRONT + 8, 380, 460, 1340, 1405))
    return fuse(centre, end, mirror_y(end), latch)


def make_guards():
    """Box-section guards along the track tops, from the rear blocks to the cab corners."""
    guard = box(X_REAR + 20, CH_F[0] - 60, HULL_HW - 40, HULL_HW + 75, 1060.0, 1300.0)
    guard = chamfered(guard, cq.selectors.BoxSelector((X_REAR, HULL_HW + 65, 1290), (3400, HULL_HW + 85, 1310)), 18)
    nose = prism_z([(CH_F[0] - 61, HULL_HW - 40), (CH_F[0] + 150, HULL_HW - 40), (CH_F[0] - 61, HULL_HW + 75)], 1060.0, 1300.0)
    return both(fuse(guard, nose))


def make_flaps():
    """Curved rubber flaps hanging from the rear corner blocks, behind the tracks."""
    cx, cy, r = X_REAR + 290, TRACK_YC + 40, 310.0          # vertical shell, curved in plan
    arc = [(cx + rr * math.cos(math.radians(t)), cy + rr * math.sin(math.radians(t)))
           for rr, ts in ((r, range(122, 239, 6)), (r - 24, range(236, 121, -6))) for t in ts]
    return both(prism_z(arc, 690.0, Z_SPONSON + 5))


def make_mirrors():
    """Mirrors on tubular outrigger frames from the windscreen corners."""
    y0, y1 = WS_HW + 10, 1560.0
    zt, zb = 1860.0, 1580.0
    xt, xb = ws_point((WS_T[1] - zt) / (WS_T[1] - WS_B[1]) * WS_LEN, 0, 0)[0] - 30, \
        ws_point((WS_T[1] - zb) / (WS_T[1] - WS_B[1]) * WS_LEN, 0, 0)[0] - 30
    frame = fuse(rod(16, (xt, y0, zt), (xt + 60, y1, zt)), rod(16, (xb, y0, zb), (xb + 60 - (xb - xt), y1, zb)),
                 rod(16, (xt + 60, y1, zt), (xt + 60, y1, zb)))
    head = box(xt + 35, xt + 85, y1 - 60, y1 + 90, 1400, 1640)
    return both(fuse(frame, head))


def shackle(x_face, y, z_pin, sgn):
    """Lug with a pin and a U-shackle hanging from it; sgn = +1 on the front face, -1 on the rear."""
    xc = x_face + sgn * 45
    xa, xb = sorted((x_face, x_face + sgn * 75))
    lug = cut(box(xa, xb, y - 15, y + 15, z_pin - 35, z_pin + 45), cyl_y(13, y - 20, y + 20, xc, z_pin))
    pin = cyl_y(11, y - 58, y + 58, xc, z_pin)
    legs = [cyl_z(11, z_pin - 70, z_pin, xc, y + s * 42) for s in (1, -1)]
    bow = cut(cq.Solid.makeTorus(42, 11, V(xc, y, z_pin - 70), V(1, 0, 0)), box(xc - 20, xc + 20, y - 60, y + 60, z_pin - 70, z_pin + 10))
    return fuse(lug, pin, *legs, bow)


def make_lights():
    """Headlamp lenses, headlamp pods, tow shackles, rear light clusters."""
    lens = both(cyl_x(72, 3555, 3570, 1110, 1350))
    pod = both(chamfered(box(3350, 3555, 975, 1250, 1255, 1445), cq.selectors.BoxSelector((3540, 960, 1435), (3570, 1260, 1455)), 45))
    shackles = fuse(*[shackle(3470.0, s * 450, 900, 1) for s in (1, -1)],
                    *[shackle(X_DOOR, s * 470, 820, -1) for s in (1, -1)])
    housing = box(X_REAR - 18, X_REAR + 2, 880, 1140, 1290, 1385)
    # seen from behind, both clusters have the red lamp on the left (+y) end, as in the DSEI photo
    red = fuse(*[cyl_x(34, X_REAR - 26, X_REAR - 16, y, 1338) for y in (1095, -925)])
    white = fuse(*[cyl_x(22, X_REAR - 26, X_REAR - 16, y, 1338) for y in (910, 960, 1010, -1110, -1060, -1010)])
    return lens, pod, shackles, both(housing), red, white


def build():
    parts = []                 # (name, shape, colour)
    parts.append(("hull", make_hull(), OLIVE))
    parts.append(("glass", make_glass(), GLASS))
    parts.append(("fittings", make_fittings(), DARK_OLIVE))
    parts.append(("bolts", make_bolts(), (0.25, 0.30, 0.20)))
    parts.append(("bumper", make_bumper(), BLACK))
    parts.append(("guards", make_guards(), GUARD))
    parts.append(("flaps", make_flaps(), BLACK))
    parts.append(("mirrors", make_mirrors(), BLACK))
    lens, pod, shackles, housing, red, white = make_lights()
    parts.append(("lights", lens, LIGHT))
    parts.append(("light_pods", pod, DARK_OLIVE))
    parts.append(("tow_eyes", shackles, STEEL))
    parts.append(("rear_light_housings", housing, BLACK))
    parts.append(("rear_lights", red, RED))
    parts.append(("rear_lamps", white, LIGHT))
    for side, tag in ((1, "L"), (-1, "R")):
        track, ridge = make_track(side)
        parts.append((f"track_{tag}", track, RUBBER))
        parts.append((f"track_guide_{tag}", ridge, (0.10, 0.10, 0.11)))
        g = make_running_gear(side)
        parts.append((f"tyres_{tag}", g["tyres"], RUBBER))
        parts.append((f"rims_{tag}", g["rims"], RIM_GREEN))
        parts.append((f"hubs_{tag}", g["hubs"], STEEL))
        parts.append((f"rollers_{tag}", g["rollers"], (0.12, 0.12, 0.13)))
        parts.append((f"sprocket_{tag}", g["sprocket"], END_DARK))
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
    y1 = max(s.BoundingBox().ymax for n, s, _ in parts if n != "mirrors")
    z1 = max(s.BoundingBox().zmax for _, s, _ in parts)
    print(f"length {x1 - x0:.0f} mm, width {2 * y1:.0f} mm, height {z1:.0f} mm, {len(parts)} bodies")
    export(parts, scale)


if __name__ == "__main__":
    main()
