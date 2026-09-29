"""Small CadQuery solid helpers shared by build.py and components.py (mm)."""
import cadquery as cq
from cadquery import Vector as V


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_x(r, x0, x1, y, z):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_y(r, y0, y1, x, z):
    return cq.Solid.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_z(r, z0, z1, x, y):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def rbox(x0, x1, y0, y1, z0, z1, r, edges="|Z"):
    """Box with the selected edges rounded (falls back to a sharp box if the fillet fails)."""
    w = cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
    try:
        return w.edges(edges).fillet(r).val()
    except Exception:
        return w.val()


def ball(r, x, y, z):
    return cq.Solid.makeSphere(r, V(x, y, z), V(0, 0, 1), -90, 90, 360)


def wire_path(r, pts):
    """Round wire of radius r along a polyline, with rounded corners."""
    segs = [rod(r, a, b) for a, b in zip(pts, pts[1:])]
    return fuse(*segs, *[ball(r, *p) for p in pts[1:-1]])


def rod(r, p0, p1):
    """Cylinder of radius r from point p0 to point p1."""
    a, b = V(*p0), V(*p1)
    return cq.Solid.makeCylinder(r, (b - a).Length, a, b - a)


def prism_x(pts_yz, x0, x1):
    """Extrude a closed polygon drawn in y-z along x."""
    wire = cq.Wire.makePolygon([V(x0, y, z) for y, z in pts_yz], close=True)
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), V(x1 - x0, 0, 0))


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


def shaft_along_y(shape, origin):
    """Turn a part modelled with its shaft on +z so the shaft points along +y.

    Local x stays x, local z becomes +y and local y becomes -z; then the local
    origin moves to `origin`.
    """
    return shape.rotate(V(0, 0, 0), V(1, 0, 0), -90).translate(V(*origin))
