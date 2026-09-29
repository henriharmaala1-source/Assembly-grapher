#!/usr/bin/env python3
"""Patria TRACKX, exterior only: a parametric CadQuery model.

    python3 trackx/build.py [--scale 43]   # STEP, GLB and STL into trackx/out/

Coordinates are millimetres, full size: x forward, y left, z up, ground at z = 0.

Sources (search summaries; the makers' pages and photos could not be opened):
  length just over 7 m, width just under 3 m, 2.0 m to the roof plate;
  56 cm rubber tracks; 55 cm ground clearance; front drive sprocket, rear idler,
  six dual rubber-tyred road wheels and two return rollers a side; driver and
  commander at the front behind a large armoured windscreen, two side crew
  doors, a forward-opening commander's roof hatch, a rear troop door hinged on
  the left; nearly flat underside.
Everything else (bonnet height, windscreen rake, door sizes, wheel diameters,
hatch and light details) is an estimate. Weapon stations are left off.
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
X_REAR, X_FRONT = -3450.0, 3650.0            # hull ends
HULL_HW = 1450.0                             # hull half-width
Z_ROOF = 2000.0
Z_SPONSON = 1100.0                           # underside of the full-width upper hull
Z_BELLY = 550.0                              # ground clearance
TUB_HW = 910.0                               # belly tub between the tracks
TRACK_W, TRACK_T = 560.0, 45.0
TRACK_Y0, TRACK_Y1 = 915.0, 1475.0           # outer track edge sets the 2.95 m width
TRACK_YC = (TRACK_Y0 + TRACK_Y1) / 2

WHEEL_R = 300.0
WHEEL_X = [2250.0 - 940.0 * i for i in range(6)]
IDLER = (-3150.0, TRACK_T + 340.0, 340.0)    # x, z, radius (inner path radius)
SPROCKET = (3000.0, 650.0, 350.0)
ROLLER_R = 95.0
ROLLER_X = [-1000.0, 1300.0]
CLEAT_PITCH = 150.0

# colours
OLIVE, DARK_OLIVE = (0.27, 0.31, 0.21), (0.20, 0.23, 0.15)
RUBBER, STEEL, GLASS = (0.06, 0.06, 0.065), (0.42, 0.44, 0.44), (0.09, 0.14, 0.18)
LIGHT, AMBER = (0.9, 0.9, 0.85), (0.9, 0.5, 0.1)


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    x0, x1, y0, y1, z0, z1 = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1)
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


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
        c = box(-24, 24, y0 + 25, y1 - 25, -20, 5)             # local z < 0 points outward
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


def make_running_gear(side):
    """Wheels, rims, hubs, sprocket, idler, return rollers and swing arms for one side (left, y > 0)."""
    yc = TRACK_YC
    tyre_w = 170.0
    tyres, rims, hubs, bolts, arms = [], [], [], [], []
    y_in_face = yc - 205.0
    for x, z, r in wheel_circles():
        for dy in (-120.0, 120.0):
            ya, yb = yc + dy - tyre_w / 2, yc + dy + tyre_w / 2
            tyres.append(disc(x, z, r, ya, yb, 16))
            rims.append(disc(x, z, 205, ya - 6, yb + 6, 8))
        # outer hub cap and bolts
        hubs.append(disc(x, z, 85, yc + 205, yc + 232, 8))
        for k in range(8):
            a = 2 * math.pi * k / 8
            bolts.append(cyl_y(13, yc + 205, yc + 218, x + 145 * math.cos(a), z + 145 * math.sin(a)))
        # inboard axle and swing arm
        arms.append(cyl_y(48, TRACK_Y0 - 70, yc - 205, x, z))
        arms.append(rod(46, (x, TRACK_Y0 - 70, z), (x + 430, TRACK_Y0 - 70, z + 300)))
        arms.append(cyl_y(60, TRACK_Y0 - 120, TRACK_Y0 - 30, x + 430, z + 300))
    # idler: dual tyres like the road wheels but larger
    ix, iz, ir = IDLER
    for dy in (-120.0, 120.0):
        tyres.append(disc(ix, iz, ir - 4, yc + dy - tyre_w / 2, yc + dy + tyre_w / 2, 16))
        rims.append(disc(ix, iz, 240, yc + dy - tyre_w / 2 - 6, yc + dy + tyre_w / 2 + 6, 8))
    hubs.append(disc(ix, iz, 95, yc + 205, yc + 232, 8))
    arms.append(cyl_y(55, TRACK_Y0 - 70, yc - 205, ix, iz))
    arms.append(rod(50, (ix, TRACK_Y0 - 70, iz), (ix + 450, TRACK_Y0 - 70, iz + 420)))
    # return rollers under the top run
    outer_in = track_loop(0.0)
    rollers = []
    for x in ROLLER_X:
        z = hull_z_at(outer_in, x) - ROLLER_R
        rollers.append(disc(x, z, ROLLER_R, yc - 60, yc + 60, 6))
        arms.append(rod(24, (x, yc - 60, z), (x, TRACK_Y0 - 60, z + 130)))
    # sprocket
    sx, sz, _ = SPROCKET
    sprocket = fuse(toothed_disc(sx, sz, yc - 205, yc - 120), toothed_disc(sx, sz, yc + 120, yc + 205),
                    disc(sx, sz, 240, yc - 205, yc + 205, 8), disc(sx, sz, 95, yc + 205, yc + 232, 8))
    arms.append(cyl_y(60, TRACK_Y0 - 90, yc - 205, sx, sz))
    groups = {
        "tyres": fuse(*tyres), "rims": fuse(*rims), "hubs": fuse(*hubs, *bolts),
        "rollers": fuse(*rollers), "sprocket": sprocket, "arms": fuse(*arms),
    }
    if side < 0:
        groups = {k: mirror_y(v) for k, v in groups.items()}
    return groups


# ---------------------------------------------------------------- hull
WS_TOP, WS_BOT = (2500.0, 2000.0), (2770.0, 1480.0)         # windscreen plane, x-z
WS_DEG = math.degrees(math.atan2(WS_TOP[1] - WS_BOT[1], WS_BOT[0] - WS_TOP[0]))   # 62.5 deg from horizontal
WS_LEN = math.dist(WS_TOP, WS_BOT)


def plane_box(u0, u1, y0, y1, n0, n1):
    """Box on the windscreen plane: u along the plane from the top edge downward, y across,
    n along the outward normal. Returns a solid in world coordinates."""
    b = box(u0, u1, y0, y1, n0, n1)
    return spin_y(b, WS_DEG).translate(V(*(WS_TOP[0], 0, WS_TOP[1])))


def make_hull():
    upper = [(X_REAR, Z_SPONSON), (X_REAR, Z_ROOF), (WS_TOP[0], Z_ROOF), WS_BOT,
             (3400.0, 1300.0), (X_FRONT, 1120.0), (X_FRONT, Z_SPONSON)]
    upper_s = prism_y(upper, -HULL_HW, HULL_HW)
    # sloping side armour: a wide bevel where the roof meets each side, and a small one at the rear edge
    for y in (-1450.0, 1450.0):
        upper_s = chamfered(upper_s, cq.selectors.BoxSelector((-3400, y - 10, 1990), (2400, y + 10, 2010)), 130)
    upper_s = chamfered(upper_s, cq.selectors.BoxSelector((-3460, -100, 1990), (-3440, 100, 2010)), 55)
    for y in (-1450.0, 1450.0):                                        # lower edge of the sides
        upper_s = chamfered(upper_s, cq.selectors.BoxSelector((-3400, y - 10, 1090), (3600, y + 10, 1110)), 70)
    upper_s = chamfered(upper_s, cq.selectors.BoxSelector((-3460, -1460, 1300), (-3440, 1460, 1800)), 60)
    tub = [(X_REAR + 50, Z_BELLY), (3380.0, Z_BELLY), (3560.0, 800.0), (X_FRONT - 20, Z_SPONSON + 20),
           (X_REAR, Z_SPONSON + 20)]
    tub_s = prism_y(tub, -TUB_HW, TUB_HW)
    tub_s = chamfered(tub_s, cq.selectors.BoxSelector((-3500, -1000, 540), (3400, 1000, 560)), 45)
    hull = fuse(upper_s, tub_s)

    cuts = []
    # windscreen: two recessed panes
    for y0, y1 in ((60, 1030), (-1030, -60)):
        cuts.append(plane_box(50, WS_LEN - 50, y0, y1, -28, 6))
    # crew doors, one each side: outline grooves, window recess
    for s in (1, -1):
        ys = s * HULL_HW
        y_in, y_out = sorted((ys - s * 9, ys + s * 2))
        x0, x1, z0, z1 = 1700.0, 2440.0, 1180.0, 1930.0
        for (a, b, c, d) in ((x0, x1, z0, z0 + 14), (x0, x1, z1 - 14, z1), (x0, x0 + 14, z0, z1), (x1 - 14, x1, z0, z1)):
            cuts.append(box(a, b, y_in, y_out, c, d))
        cuts.append(box(1900, 2360, y_in - 6, y_out, 1560, 1850))       # armoured window recess
    # rear door, hinged on the left: outline + a hinge strip
    for (a, b, c, d) in ((-700, 700, 1160, 1174), (-700, 700, 1900, 1914), (-700, -686, 1160, 1914), (686, 700, 1160, 1914)):
        cuts.append(box(X_REAR - 2, X_REAR + 9, a, b, c, d))
    # roof hatch recesses
    cuts.append(box(1950, 2450, -900, -300, Z_ROOF - 12, Z_ROOF + 2))          # commander hatch seat
    hull = cut(hull, *cuts)
    return hull


def make_glass():
    panes = []
    for y0, y1 in ((60, 1030), (-1030, -60)):
        panes.append(plane_box(50, WS_LEN - 50, y0, y1, -20, -6))
    for s in (1, -1):
        ys = s * HULL_HW
        y_in, y_out = sorted((ys - s * 15, ys - s * 5))
        panes.append(box(1900, 2360, y_in, y_out, 1560, 1850))
    return fuse(*panes)


def make_fittings():
    """Hatches, door handles, lights, tow eyes, wipers, roof lifting eyes."""
    parts = []
    # commander's hatch (right side): raised ring, lid hinged at the rear, three vision blocks in front
    ring = cut(box(1940, 2460, -910, -290, Z_ROOF - 10, Z_ROOF + 55), box(1990, 2410, -860, -340, Z_ROOF + 10, Z_ROOF + 60))
    lid = box(1985, 2415, -865, -335, Z_ROOF + 30, Z_ROOF + 62)
    vision = [box(2380, 2440, -800 + 190 * i, -650 + 190 * i, Z_ROOF + 55, Z_ROOF + 100) for i in range(3)]
    parts.append(fuse(ring, lid, *vision))
    # rear-left round hatch (for the rear crew position): ring and lid
    ring2 = cut(cyl_z(390, Z_ROOF - 10, Z_ROOF + 50, -2500, 650), cyl_z(340, Z_ROOF + 15, Z_ROOF + 60, -2500, 650))
    lid2 = cyl_z(345, Z_ROOF + 30, Z_ROOF + 58, -2500, 650)
    parts.append(fuse(ring2, lid2, box(-2560, -2440, 60 + 650 - 40, 60 + 650 + 40, Z_ROOF + 58, Z_ROOF + 80)))
    # door handles
    for s in (1, -1):
        parts.append(box(1760, 1850, s * (HULL_HW + 2), s * (HULL_HW + 22), 1480, 1520))
    parts.append(box(X_REAR - 22, X_REAR - 2, 500, 620, 1520, 1560))              # rear door handle
    for z in (1230, 1850):
        parts.append(box(X_REAR - 16, X_REAR - 2, 640, 700, z - 25, z + 25))      # rear door hinges (left)
    # roof lifting eyes
    for x, y in ((-3000, 1100), (-3000, -1100), (1400, 1100), (1400, -1100)):
        parts.append(fuse(cyl_z(45, Z_ROOF, Z_ROOF + 22, x, y)))
    # wipers, parked along the bottom edge of the glass
    for y in (545, -545):
        parts.append(plane_box(WS_LEN - 95, WS_LEN - 80, y - 240, y + 240, 8, 20))
    return fuse(*parts)


def make_lights():
    """Headlights and indicators on the glacis, tow eyes on the lower front plate."""
    lamps = []
    for s in (1, -1):
        lamps.append(box(3470, 3540, s * 1130 - 90, s * 1130 + 90, 1170, 1250))
    eyes = []
    for s in (1, -1):
        e = cut(cyl_y(75, s * 700 - 25, s * 700 + 25, 3560, 830), cyl_y(38, s * 700 - 30, s * 700 + 30, 3560, 830))
        eyes.append(e)
        eyes.append(box(3500, 3560, s * 700 - 25, s * 700 + 25, 760, 830))
    return fuse(*lamps), fuse(*eyes)


def build():
    parts = []                 # (name, shape, colour)
    hull = make_hull()
    parts.append(("hull", hull, OLIVE))
    parts.append(("glass", make_glass(), GLASS))
    parts.append(("fittings", make_fittings(), DARK_OLIVE))
    lamps, eyes = make_lights()
    parts.append(("lights", lamps, LIGHT))
    parts.append(("tow_eyes", eyes, STEEL))
    for side, tag in ((1, "L"), (-1, "R")):
        track, ridge = make_track(side)
        parts.append((f"track_{tag}", track, RUBBER))
        parts.append((f"track_guide_{tag}", ridge, (0.10, 0.10, 0.11)))
        g = make_running_gear(side)
        parts.append((f"tyres_{tag}", g["tyres"], RUBBER))
        parts.append((f"rims_{tag}", g["rims"], (0.30, 0.32, 0.28)))
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
