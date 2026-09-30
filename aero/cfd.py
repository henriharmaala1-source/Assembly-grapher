"""2D CFD cross-check of a wing section with OpenFOAM (aero/crosscheck.py --cfd).

This is an independent check on NeuralFoil and XFOIL: a finite-volume solution
of the Reynolds-averaged Navier-Stokes equations (simpleFoam) with the k-omega
SST turbulence model and the Langtry-Menter transition model (kOmegaSSTLM), so
laminar runs, separation bubbles and transition come from a different model of
the physics than XFOIL's e^n method.

    apt install openfoam       # Ubuntu's package (ESI v1912) has everything used here

The mesh is a structured C-grid written straight to OpenFOAM's polyMesh format:
wall-normal lines at the surface (first cell 5e-5 chords, y+ under 1), blended
into straight lines to a far field 20 chords away, a wake block behind the
0.8 mm blunt trailing edge. The flow is non-dimensional (chord 1, speed 1,
viscosity 1/Re), and the angle of attack is set by the far-field velocity, so
one mesh serves every angle and Reynolds number.

The free-stream turbulence is matched to XFOIL's n_crit with Mack's relation
(n_crit 7 is about 0.16 %), allowing for its decay on the way from the far
field to the wing.
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

R_FAR = 20.0            # far field and outlet distance, chords
NS = 180                # cells along each surface
NW = 90                 # cells along the wake
NJ = 120                # cells from the wall to the far field
NB = 24                 # cells across the blunt trailing edge
H_WALL = 5e-5           # first cell height at the wall, chords
DS_LE, DS_TE = 1.0e-3, 2.5e-3   # spacing along the surface at the leading and trailing edges
DZ = 0.1                # span of the one-cell-thick 2D mesh
BLEND = (0.004, 0.4)    # grid lines turn from wall-normal to straight between these distances, chords
VISC_RATIO = 5.0        # free-stream eddy viscosity / viscosity
ITERATIONS = 8000        # most iterations per run
CHUNK = 500             # iterations between force checks


# --------------------------------------------------------------------------- grid

def _one_sided(n: int, first: float) -> np.ndarray:
    """n intervals on [0, 1], the first `first` long, tanh-stretched."""
    if first >= 1.0 / n:
        return np.linspace(0, 1, n + 1)
    xi = np.linspace(0, 1, n + 1)
    f = lambda d: 1 + math.tanh(d * (1 / n - 1)) / math.tanh(d) - first  # noqa: E731
    d = brentq(f, 1e-6, 60)
    return 1 + np.tanh(d * (xi - 1)) / math.tanh(d)


def _two_sided(n: int, ds0: float, ds1: float) -> np.ndarray:
    """n intervals on [0, 1] with end spacings ds0 and ds1 (Vinokur's stretching)."""
    xi = np.linspace(0, 1, n + 1)
    s0, s1 = 1 / (n * ds0), 1 / (n * ds1)
    a, b = math.sqrt(s0 / s1), math.sqrt(s0 * s1)
    if b > 1.001:
        dy = brentq(lambda y: math.sinh(y) / y - b, 1e-6, 100)
        u = 0.5 * (1 + np.tanh(dy * (xi - 0.5)) / math.tanh(dy / 2))
    elif b < 0.999:
        dy = brentq(lambda y: math.sin(y) / y - b, 1e-6, math.pi - 1e-6)
        u = 0.5 * (1 + np.tan(dy * (xi - 0.5)) / math.tan(dy / 2))
    else:
        u = xi * (1 + 2 * (b - 1) * (xi - 0.5) * (1 - xi))
    return u / (a + (1 - a) * u)


def _surface(fn, n: int) -> tuple[np.ndarray, np.ndarray]:
    """Points along one surface from the leading edge to the trailing edge, spaced by arc length."""
    t = np.linspace(0, 1, 6001)
    x = (1 - np.cos(np.pi * t)) / 2
    xy = np.asarray(fn(x))
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0], np.cumsum(seg)])
    u = _two_sided(n, DS_LE / s[-1], DS_TE / s[-1]) * s[-1]
    return np.interp(u, s, xy[:, 0]), np.interp(u, s, xy[:, 1])


def c_grid(upper, lower) -> dict:
    """Structured C-grid around a section. `upper(x)` and `lower(x)` give (N, 2)
    surface points for chord positions x in [0, 1] (leading edge at 0)."""
    xu, yu = _surface(upper, NS)
    xl, yl = _surface(lower, NS)
    wx = 1 + R_FAR * _one_sided(NW, DS_TE / R_FAR)
    gap = yu[-1] - yl[-1]
    # the path along j = 0: lower wake (outlet -> TE), lower surface (TE -> LE), upper surface, upper wake
    px = np.concatenate([wx[::-1], xl[::-1][1:], xu[1:], wx[1:]])
    py = np.concatenate([np.full(NW + 1, yl[-1]), yl[::-1][1:], yu[1:], np.full(NW, yu[-1])])
    ni = len(px) - 1
    i_te_lo, i_le, i_te_up = NW, NW + NS, NW + 2 * NS
    kind = np.array(["wake"] * (NW + 1) + ["surf"] * (2 * NS - 1) + ["wake"] * (NW + 1))
    # arc-length fraction from the leading edge, per surface station
    frac = np.zeros(ni + 1)
    for a, b in ((i_te_lo, i_le), (i_le, i_te_up)):
        s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(px[a:b + 1]), np.diff(py[a:b + 1])))])
        f = s / s[-1]
        frac[a:b + 1] = 1 - f if a == i_te_lo else f
    # wall normals, smoothed across the trailing-edge corners
    tx, ty = np.gradient(px), np.gradient(py)
    norm = np.hypot(tx, ty)
    nx, ny = -ty / norm, tx / norm
    for c in (i_te_lo, i_te_up):
        lo, hi = c - 12, c + 13
        for _ in range(30):
            nx[lo + 1:hi - 1] = (nx[lo:hi - 2] + 2 * nx[lo + 1:hi - 1] + nx[lo + 2:hi]) / 4
            ny[lo + 1:hi - 1] = (ny[lo:hi - 2] + 2 * ny[lo + 1:hi - 1] + ny[lo + 2:hi]) / 4
            m = np.hypot(nx, ny)
            nx, ny = nx / m, ny / m
    # far-field points: straight above/below the wake, a semicircle round the front
    ox, oy = np.empty(ni + 1), np.empty(ni + 1)
    for i in range(ni + 1):
        if kind[i] == "wake" or i in (i_te_lo, i_te_up):
            ox[i], oy[i] = px[i], (-R_FAR if i <= i_te_lo else R_FAR)
        else:
            psi = math.pi / 2 + math.pi / 2 * (1 - math.sqrt(frac[i])) ** 1.5
            ox[i] = 1 + R_FAR * math.cos(psi)
            oy[i] = R_FAR * math.sin(psi) * (1 if i >= i_le else -1)
    # the same first cell height on every line (wall and wake cut), so the grid lines out in the
    # far field stay parallel to the boundary instead of stepping where the spacing changes
    h = np.full(ni + 1, H_WALL)
    X, Y = np.empty((ni + 1, NJ + 1)), np.empty((ni + 1, NJ + 1))
    la, lb = math.log(BLEND[0]), math.log(BLEND[1])
    for i in range(ni + 1):
        L = math.hypot(ox[i] - px[i], oy[i] - py[i])
        ux, uy = (ox[i] - px[i]) / L, (oy[i] - py[i]) / L
        d = _one_sided(NJ, h[i] / L) * L
        t = np.clip((np.log(np.maximum(d, 1e-12)) - la) / (lb - la), 0, 1)
        b = t * t * (3 - 2 * t)
        X[i] = px[i] + d * ((1 - b) * nx[i] + b * ux)
        Y[i] = py[i] + d * ((1 - b) * ny[i] + b * uy)
    # blunt trailing edge block: across the gap, along the wake
    tb = np.array([_two_sided(NB, h[i_te_lo - ib] / gap, h[i_te_lo - ib] / gap) for ib in range(NW + 1)])
    return {"X": X, "Y": Y, "ni": ni, "i_te": (i_te_lo, i_te_up), "wx": wx, "y_te": (yl[-1], yu[-1]),
            "tb": tb, "surface": (np.concatenate([xl[::-1], xu[1:]]), np.concatenate([yl[::-1], yu[1:]]))}


def assemble(g: dict):
    """Points, quads (counter-clockwise) and the wall edges of the grid."""
    ni, (i_lo, i_up) = g["ni"], g["i_te"]
    nj1 = NJ + 1
    pts = list(zip(g["X"].ravel(), g["Y"].ravel()))
    cid = lambda i, j: i * nj1 + j  # noqa: E731
    base = {}
    y_lo, y_up = g["y_te"]
    for ib in range(NW + 1):
        base[(ib, 0)] = cid(i_lo - ib, 0)
        base[(ib, NB)] = cid(i_up + ib, 0)
        for k in range(1, NB):
            base[(ib, k)] = len(pts)
            pts.append((g["wx"][ib], y_lo + (y_up - y_lo) * g["tb"][ib][k]))
    quads = [(cid(i, j), cid(i + 1, j), cid(i + 1, j + 1), cid(i, j + 1)) for i in range(ni) for j in range(NJ)]
    quads += [(base[(ib, k)], base[(ib + 1, k)], base[(ib + 1, k + 1)], base[(ib, k + 1)])
              for ib in range(NW) for k in range(NB)]
    wall = {frozenset((cid(i, 0), cid(i + 1, 0))) for i in range(i_lo, i_up)}
    wall |= {frozenset((base[(0, k)], base[(0, k + 1)])) for k in range(NB)}
    return np.array(pts), np.array(quads), wall


def quality(pts: np.ndarray, quads: np.ndarray) -> dict:
    p = pts[quads]
    x, y = p[..., 0], p[..., 1]
    area = 0.5 * np.sum(x * np.roll(y, -1, 1) - np.roll(x, -1, 1) * y, axis=1)
    return {"cells": int(len(quads)), "min_area": float(area.min()), "negative": int(np.sum(area <= 0))}


def _header(cls: str, obj: str, loc: str = "constant/polyMesh") -> str:
    return ("FoamFile\n{\n    version     2.0;\n    format      ascii;\n"
            f"    class       {cls};\n    location    \"{loc}\";\n    object      {obj};\n}}\n\n")


def write_polymesh(case: Path, pts: np.ndarray, quads: np.ndarray, wall: set) -> dict:
    n2 = len(pts)
    edges = {}
    for c, q in enumerate(quads):
        for a, b in ((q[0], q[1]), (q[1], q[2]), (q[2], q[3]), (q[3], q[0])):
            edges.setdefault(frozenset((a, b)), []).append((c, a, b))
    internal, bwall, bfar = [], [], []
    for key, lst in edges.items():
        if len(lst) == 2:
            (c1, a1, b1), (c2, a2, b2) = sorted(lst)
            internal.append((c1, c2, a1, b1))
        else:
            c, a, b = lst[0]
            (bwall if key in wall else bfar).append((c, a, b))
    internal.sort()
    bwall.sort()
    bfar.sort()
    faces, owner, neigh = [], [], []
    for c1, c2, a, b in internal:
        faces.append((a, b, b + n2, a + n2))
        owner.append(c1)
        neigh.append(c2)
    starts = {}
    for name, lst in (("airfoil", bwall), ("farfield", bfar)):
        starts[name] = (len(faces), len(lst))
        for c, a, b in lst:
            faces.append((a, b, b + n2, a + n2))
            owner.append(c)
    starts["frontAndBack"] = (len(faces), 2 * len(quads))
    for c, q in enumerate(quads):
        faces.append((q[0], q[3], q[2], q[1]))
        owner.append(c)
    for c, q in enumerate(quads):
        faces.append(tuple(int(v) + n2 for v in q))
        owner.append(c)
    d = case / "constant" / "polyMesh"
    d.mkdir(parents=True, exist_ok=True)
    with open(d / "points", "w") as f:
        f.write(_header("vectorField", "points") + f"{2 * n2}\n(\n")
        for z in (0.0, DZ):
            f.write("".join(f"({x:.10g} {y:.10g} {z:g})\n" for x, y in pts))
        f.write(")\n")
    with open(d / "faces", "w") as f:
        f.write(_header("faceList", "faces") + f"{len(faces)}\n(\n")
        f.write("".join(f"4({a} {b} {c} {e})\n" for a, b, c, e in faces))
        f.write(")\n")
    note = f"nPoints:{2 * n2}  nCells:{len(quads)}  nFaces:{len(faces)}  nInternalFaces:{len(internal)}"
    for name, data in (("owner", owner), ("neighbour", neigh)):
        with open(d / name, "w") as f:
            f.write(_header("labelList", name).replace("    object", f"    note        \"{note}\";\n    object")
                    + f"{len(data)}\n(\n" + "\n".join(map(str, data)) + "\n)\n")
    types = {"airfoil": "wall", "farfield": "patch", "frontAndBack": "empty"}
    with open(d / "boundary", "w") as f:
        f.write(_header("polyBoundaryMesh", "boundary") + "3\n(\n")
        for name, (start, n) in starts.items():
            f.write(f"    {name}\n    {{\n        type            {types[name]};\n"
                    f"        nFaces          {n};\n        startFace       {start};\n    }}\n")
        f.write(")\n")
    return {"cells": len(quads), "faces": len(faces), "wall_faces": len(bwall)}


# --------------------------------------------------------------------------- case

def turbulence(re: float, n_crit: float) -> dict:
    """Far-field k, omega and transition onset Re_theta for XFOIL's n_crit (Mack),
    allowing for the decay over the 20 chords to the wing (SST: beta 0.0828, beta* 0.09)."""
    tu = math.exp(-(n_crit + 8.43) / 2.4)
    nu = 1.0 / re
    k_wing = 1.5 * tu * tu
    k0 = k_wing
    for _ in range(20):
        w0 = k0 / (VISC_RATIO * nu)
        k0 = k_wing * (1 + 0.0828 * w0 * R_FAR) ** (0.09 / 0.0828)
    tp = 100 * tu
    re_theta = 1173.51 - 589.428 * tp + 0.2196 / (tp * tp) if tp <= 1.3 else 331.5 * (tp - 0.5658) ** -0.671
    return {"tu": tu, "k": k0, "omega": k0 / (VISC_RATIO * nu), "re_theta": max(re_theta, 20.0), "nu": nu}


def _field(name: str, cls: str, dims: str, internal: str, patches: dict) -> str:
    body = "".join(f"    {p}\n    {{\n" + "".join(f"        {k:15s} {v};\n" for k, v in d.items()) + "    }\n"
                   for p, d in patches.items())
    return (_header(cls, name, "0") + f"dimensions      {dims};\n\ninternalField   {internal};\n\n"
            f"boundaryField\n{{\n{body}    frontAndBack\n    {{\n        type            empty;\n    }}\n}}\n")


def write_case(case: Path, re: float, alpha: float, n_crit: float) -> dict:
    """Fields, physical properties and solver settings (the mesh must already be there)."""
    t = turbulence(re, n_crit)
    a = math.radians(alpha)
    u = f"uniform ({math.cos(a):.8f} {math.sin(a):.8f} 0)"
    (case / "0").mkdir(parents=True, exist_ok=True)
    (case / "system").mkdir(exist_ok=True)
    far = lambda v: {"type": "inletOutlet", "inletValue": v, "value": v}  # noqa: E731
    fields = {
        "U": ("volVectorField", "[0 1 -1 0 0 0 0]", u, {"airfoil": {"type": "noSlip"},
              "farfield": {"type": "freestreamVelocity", "freestreamValue": u, "value": u}}),
        "p": ("volScalarField", "[0 2 -2 0 0 0 0]", "uniform 0", {"airfoil": {"type": "zeroGradient"},
              "farfield": {"type": "freestreamPressure", "freestreamValue": "uniform 0", "value": "uniform 0"}}),
        "k": ("volScalarField", "[0 2 -2 0 0 0 0]", f"uniform {t['k']:.6g}", {
              "airfoil": {"type": "fixedValue", "value": "uniform 0"}, "farfield": far(f"uniform {t['k']:.6g}")}),
        "omega": ("volScalarField", "[0 0 -1 0 0 0 0]", f"uniform {t['omega']:.6g}", {
              "airfoil": {"type": "omegaWallFunction", "value": f"uniform {t['omega']:.6g}"},
              "farfield": far(f"uniform {t['omega']:.6g}")}),
        "nut": ("volScalarField", "[0 2 -1 0 0 0 0]", "uniform 0", {
              "airfoil": {"type": "nutLowReWallFunction", "value": "uniform 0"},
              "farfield": {"type": "calculated", "value": "uniform 0"}}),
        "ReThetat": ("volScalarField", "[0 0 0 0 0 0 0]", f"uniform {t['re_theta']:.6g}", {
              "airfoil": {"type": "zeroGradient"}, "farfield": far(f"uniform {t['re_theta']:.6g}")}),
        "gammaInt": ("volScalarField", "[0 0 0 0 0 0 0]", "uniform 1", {
              "airfoil": {"type": "zeroGradient"}, "farfield": far("uniform 1")}),
    }
    for name, (cls, dims, internal, patches) in fields.items():
        (case / "0" / name).write_text(_field(name, cls, dims, internal, patches))
    (case / "constant" / "transportProperties").write_text(
        _header("dictionary", "transportProperties", "constant")
        + f"transportModel  Newtonian;\n\nnu              {t['nu']:.8g};\n")
    (case / "constant" / "turbulenceProperties").write_text(
        _header("dictionary", "turbulenceProperties", "constant")
        + "simulationType  RAS;\n\nRAS\n{\n    RASModel        kOmegaSSTLM;\n    turbulence      on;\n"
          "    printCoeffs     off;\n}\n")
    _control(case, CHUNK)
    (case / "system" / "fvSchemes").write_text(_header("dictionary", "fvSchemes", "system") + """\
ddtSchemes      { default steadyState; }
gradSchemes
{
    default         Gauss linear;
    grad(U)         cellLimited Gauss linear 1;
    grad(k)         cellLimited Gauss linear 1;
    grad(omega)     cellLimited Gauss linear 1;
}
divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
    div(phi,k)      bounded Gauss limitedLinear 1;
    div(phi,omega)  bounded Gauss limitedLinear 1;
    div(phi,ReThetat) bounded Gauss limitedLinear 1;
    div(phi,gammaInt) bounded Gauss limitedLinear 1;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear limited corrected 0.5; }
interpolationSchemes { default linear; }
snGradSchemes   { default limited corrected 0.5; }
wallDist        { method meshWave; }
""")
    (case / "system" / "fvSolution").write_text(_header("dictionary", "fvSolution", "system") + """\
solvers
{
    p
    {
        solver          GAMG;
        smoother        GaussSeidel;
        tolerance       1e-9;
        relTol          0.01;
    }
    "(U|k|omega|ReThetat|gammaInt)"
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-10;
        relTol          0.1;
    }
}
SIMPLE
{
    nNonOrthogonalCorrectors 0;
    consistent      no;
    residualControl
    {
        p               1e-6;
        U               1e-7;
        "(k|omega|ReThetat|gammaInt)" 1e-6;
    }
}
relaxationFactors
{
    fields          { p 0.3; }
    equations       { U 0.7; ".*" 0.7; }
}
""")
    (case / "case.json").write_text(json.dumps({"re": re, "alpha": alpha, "n_crit": n_crit, **t}))
    return t


def _control(case: Path, end: int) -> None:
    # No function objects: Ubuntu's v1912 build dies on any of them ("error in IOstream sha1"),
    # so forces, Cp and friction are worked out here from the written fields instead.
    (case / "system" / "controlDict").write_text(_header("dictionary", "controlDict", "system") + f"""\
application     simpleFoam;
startFrom       latestTime;
startTime       0;
stopAt          endTime;
endTime         {end};
deltaT          1;
writeControl    timeStep;
writeInterval   {CHUNK};
purgeWrite      1;
writeFormat     ascii;
writePrecision  10;
writeCompression off;
timeFormat      general;
timePrecision   8;
runTimeModifiable false;
""")


def restart_at(src: Path, case: Path, alpha: float) -> None:
    """A new case at another angle, starting from `src`'s solution (same mesh and flow)."""
    case.mkdir(parents=True, exist_ok=True)
    for d in ("constant", "system"):
        shutil.copytree(src / d, case / d, dirs_exist_ok=True, symlinks=True)
    last = _latest(src)
    shutil.copytree(src / str(last), case / "0", dirs_exist_ok=True)
    shutil.rmtree(case / "0" / "uniform", ignore_errors=True)
    a = math.radians(alpha)
    vec = f"({math.cos(a):.8f} {math.sin(a):.8f} 0)"
    text = (case / "0" / "U").read_text()
    text = re.sub(r"freestreamValue\s+uniform\s+\([^)]*\);", f"freestreamValue uniform {vec};", text)
    (case / "0" / "U").write_text(text)
    info = json.loads((src / "case.json").read_text())
    (case / "case.json").write_text(json.dumps({**info, "alpha": alpha, "restart_from": src.name}))
    shutil.copy(src / "wall.npz", case / "wall.npz")
    _control(case, CHUNK)


def _latest(case: Path) -> int:
    times = [int(d.name) for d in case.iterdir() if d.is_dir() and d.name.isdigit()]
    return max(times) if times else 0


def run(case: Path, max_iter: int | None = None, min_iter: int = 2000, log=None) -> list[dict]:
    """simpleFoam in chunks, working out the coefficients after each, until they settle
    (drag within 0.3 % and lift within 0.002 over two chunks), the residuals meet their
    targets, or the iteration limit."""
    max_iter = max_iter or ITERATIONS
    info = json.loads((case / "case.json").read_text())
    history = []
    start = _latest(case)
    it = start
    while it < start + max_iter:
        _control(case, it + CHUNK)
        with open(case / "log.simpleFoam", "a") as f:
            rc = subprocess.run(["simpleFoam", "-case", str(case)], stdout=f, stderr=subprocess.STDOUT,
                                env=_env()).returncode
        new = _latest(case)
        if rc != 0 or new <= it:
            history.append({"iteration": new, "error": f"simpleFoam stopped (exit {rc})"})
            break
        it = new
        c = coefficients(case, it, info["alpha"], info["nu"])
        history.append({"iteration": it, **{k: c[k] for k in ("cl", "cd", "cm")}})
        if log:
            log(f"    {case.name}: {it} iterations, cl {c['cl']:.4f} cd {c['cd']:.5f}")
        tail = (case / "log.simpleFoam").read_text()[-4000:]
        if "SIMPLE solution converged" in tail:
            break
        if it - start >= min_iter and len(history) >= 3:
            h0, h2 = history[-3], history[-1]
            if abs(h2["cd"] - h0["cd"]) < 0.003 * abs(h2["cd"]) and abs(h2["cl"] - h0["cl"]) < 0.002:
                break
    return history


def _read_field(path: Path) -> np.ndarray | None:
    text = path.read_text()
    i = text.index("internalField")
    head = text[i:i + 200]
    if "nonuniform" not in head:
        return None
    m = re.search(r"nonuniform\s+List<(\w+)>\s*(\d+)\s*\(", text[i:])
    n = int(m.group(2))
    body_start = i + m.end()
    body_end = text.index("\n)\n", body_start)
    vals = np.array(text[body_start:body_end].replace("(", " ").replace(")", " ").split(), float)
    return vals.reshape(n, -1) if m.group(1) == "vector" else vals


def coefficients(case: Path, time: int, alpha: float, nu: float, surface: bool = False) -> dict:
    """Lift, drag and moment from the pressure and the wall shear in the wall cells.
    The first cell is under y+ 1, so the shear is the laminar viscosity times the
    velocity over the distance to the wall."""
    w = np.load(case / "wall.npz")
    p = _read_field(case / str(time) / "p")
    U = _read_field(case / str(time) / "U")
    cell, side = w["cell"], w["side"]
    a, b, c = w["a"], w["b"], w["c"]
    e = b - a
    ln = np.hypot(e[:, 0], e[:, 1])
    s = np.stack([e[:, 1], -e[:, 0]], 1)                    # out of the flow, into the section
    n = s / ln[:, None]
    uc = U[cell, :2]
    ut = uc - np.sum(uc * n, 1)[:, None] * n
    yc = np.abs(np.sum((c - a) * n, 1))
    tau = nu * ut / yc[:, None]
    fp = p[cell][:, None] * s
    fv = tau * ln[:, None]
    ar = math.radians(alpha)
    dd, ld = np.array([math.cos(ar), math.sin(ar)]), np.array([-math.sin(ar), math.cos(ar)])
    Fp, Fv = fp.sum(0), fv.sum(0)
    mid = (a + b) / 2 - np.array([0.25, 0.0])
    f = fp + fv
    mz = np.sum(mid[:, 0] * f[:, 1] - mid[:, 1] * f[:, 0])
    out = {"cl": float((Fp + Fv) @ ld / 0.5), "cd": float((Fp + Fv) @ dd / 0.5),
           "cd_pressure": float(Fp @ dd / 0.5), "cd_friction": float(Fv @ dd / 0.5), "cm": float(-mz / 0.5)}
    ut_mag = np.hypot(tau[:, 0], tau[:, 1])
    out["yplus_max"] = float(np.max(yc * np.sqrt(ut_mag) / nu))
    if surface:
        t_hat = e / ln[:, None] * np.where(side < 0, -1, 1)[:, None]    # leading edge towards trailing edge
        cf = 2 * ut_mag * np.sign(np.sum(tau * t_hat, 1))
        cp = 2 * p[cell]
        dist = {}
        for name, sd in (("upper", 1), ("lower", -1)):
            k = np.where(side == sd)[0]
            o = k[np.argsort(mid[k, 0])]
            dist[name] = {"x": np.round(mid[o, 0] + 0.25, 5).tolist(), "cp": np.round(cp[o], 4).tolist(),
                          "cf": np.round(cf[o], 6).tolist()}
        out["surface"] = dist
    return out


def results(case: Path, history: list[dict]) -> dict:
    info = json.loads((case / "case.json").read_text())
    last = _latest(case)
    out = {"alpha": info["alpha"], "re": info["re"], "iterations": last,
           "converged": "SIMPLE solution converged" in (case / "log.simpleFoam").read_text()[-4000:]}
    if not history or "error" in history[-1]:
        return {**out, "error": history[-1]["error"] if history else "no run"}
    out.update(coefficients(case, last, info["alpha"], info["nu"], surface=True))
    tail = [h for h in history[-3:] if "cd" in h]
    out["cd_swing"] = float(max(h["cd"] for h in tail) - min(h["cd"] for h in tail))
    out["cl_swing"] = float(max(h["cl"] for h in tail) - min(h["cl"] for h in tail))
    out["history"] = [{k: (round(v, 6) if isinstance(v, float) else v) for k, v in h.items()} for h in history]
    up = out["surface"]["upper"]
    xs, cf = np.array(up["x"]), np.array(up["cf"])
    sep = np.where((cf < 0) & (xs > 0.02))[0]
    if sep.size:
        out["bubble_upper"] = [round(float(xs[sep[0]]), 3), round(float(xs[sep[-1]]), 3)]
    return out


def available() -> bool:
    return shutil.which("simpleFoam") is not None


def _env() -> dict:
    """OpenFOAM's environment; Ubuntu's package only needs to know where its etc files are."""
    env = dict(os.environ)
    if "WM_PROJECT_DIR" not in env and Path("/usr/share/openfoam/etc").is_dir():
        env["WM_PROJECT_DIR"] = "/usr/share/openfoam"
    return env


def version() -> str:
    try:
        out = subprocess.run(["simpleFoam", "-help"], capture_output=True, text=True, timeout=60, env=_env()).stdout
    except (OSError, subprocess.TimeoutExpired):
        return "OpenFOAM"
    m = re.search(r"OpenFOAM-(v?\d[\w.]*)", out)
    return f"OpenFOAM {m.group(1)}" if m else "OpenFOAM"


def mesh(case: Path, upper, lower, scale: float = 1.0) -> dict:
    """Write the grid for a section (scale < 1 for a coarser grid-dependence check)."""
    global NS, NW, NJ, NB
    keep = NS, NW, NJ, NB
    try:
        NS, NW, NJ, NB = (max(8, round(v * scale)) for v in keep)
        g = c_grid(upper, lower)
        pts, quads, wall = assemble(g)
        q = quality(pts, quads)
        if q["negative"]:
            raise RuntimeError(f"mesh has {q['negative']} inverted cells")
        info = write_polymesh(case, pts, quads, wall)
        np.savez(case / "wall.npz", **wall_faces(g, pts, quads))
    finally:
        NS, NW, NJ, NB = keep
    return {**info, **q}


def wall_faces(g: dict, pts: np.ndarray, quads: np.ndarray) -> dict:
    """The wall cells, their wall edges (in the cell's own order) and which surface each is on."""
    ni, (i_lo, i_up) = g["ni"], g["i_te"]
    i_le = (i_lo + i_up) // 2
    cells, a, b, side = [], [], [], []
    for i in range(i_lo, i_up):
        q = quads[i * NJ]
        cells.append(i * NJ)
        a.append(pts[q[0]])
        b.append(pts[q[1]])
        side.append(-1 if i < i_le else 1)
    for k in range(NB):
        c = ni * NJ + k
        q = quads[c]
        cells.append(c)
        a.append(pts[q[3]])
        b.append(pts[q[0]])
        side.append(0)
    cells = np.array(cells)
    return {"cell": cells, "a": np.array(a), "b": np.array(b), "c": pts[quads[cells]].mean(1), "side": np.array(side)}


def check_mesh(case: Path) -> dict:
    out = subprocess.run(["checkMesh", "-case", str(case)], capture_output=True, text=True, timeout=600,
                         env=_env()).stdout
    info = {"ok": "Mesh OK." in out}
    for key, pat in (("non_orthogonality_max", r"Mesh non-orthogonality Max: ([0-9.]+)"),
                     ("skewness_max", r"Max skewness = ([0-9.]+)"),
                     ("aspect_max", r"Max aspect ratio = ([0-9.eE+]+)")):
        m = re.search(pat, out)
        if m:
            info[key] = float(m.group(1))
    return info


def cpus() -> int:
    return max(1, os.cpu_count() or 1)
