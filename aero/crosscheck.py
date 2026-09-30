#!/usr/bin/env python3
"""Cross-checks for the efficiency study: the same two wing sections, and the
same plane, through independent tools.

    python3 aero/efficiency.py               # first: the study itself
    python3 aero/crosscheck.py               # NeuralFoil, XFOIL, AeroSandbox (a few minutes)
    python3 aero/crosscheck.py --cfd         # and OpenFOAM 2D CFD (an hour or two on 4 cores)

The wing section, NACA 4412 against the optimized section, at the loiter,
cruise and fast points (the same lift coefficients and Reynolds numbers):

  1. NeuralFoil "large": the network the optimizer used.
  2. NeuralFoil "xxxlarge": the biggest network, a check on the network's own fit.
  3. XFOIL 6.99 (aero/xfoil.py): the panel code with a coupled boundary layer that
     NeuralFoil was trained to imitate, run for real on the exact shape.
  4. OpenFOAM (aero/cfd.py): 2D RANS, k-omega SST with the Langtry-Menter
     transition model. Different physics from XFOIL's e^n transition.

The whole plane, as built and with the optimized airfoil:

  5. The drag build-up the study uses (aero/efficiency.py).
  6. AeroSandbox AeroBuildup: its own component models (fully turbulent friction
     with its form factors on the pod and booms, base drag, its span efficiency).
  7. A vortex lattice (AeroSandbox VLM) of the wing and tail: the induced drag the
     study gets from an assumed span efficiency of 0.8.

Writes aero/study/crosscheck.json and CROSSCHECK.md; the study page shows both.
A run without --cfd keeps earlier CFD results if the two sections have not changed.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import hashlib
import json
import math
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import aerosandbox as asb  # noqa: E402

import airfoil as A  # noqa: E402
import cfd as C  # noqa: E402
import efficiency as E  # noqa: E402
import xfoil as X  # noqa: E402
from models import airframe as AF  # noqa: E402

OUT = HERE / "study"
KEYS = ("baseline", "optimized")
STALL_ALPHAS = np.arange(0.0, 18.01, 0.25)
VERIFY = {"re": 1.0e6, "alpha": 4.0, "n_crit": 9.0}   # where XFOIL and RANS are both dependable
G = 9.81


def kulfan_of(shape: dict) -> dict:
    k = shape["kulfan"]
    return {"upper_weights": np.array(k["upper_weights"]), "lower_weights": np.array(k["lower_weights"]),
            "leading_edge_weight": k["leading_edge_weight"], "TE_thickness": k["TE_thickness"]}


def fingerprint(shapes: dict) -> str:
    return hashlib.sha1(json.dumps([shapes[k]["kulfan"] for k in KEYS], sort_keys=True).encode()).hexdigest()[:12]


def r(v, n=5):
    return None if v is None else round(float(v), n)


# --------------------------------------------------------------------------- NeuralFoil

def neuralfoil(kulfan: dict, points, re_stall: float, n_crit: float, size: str) -> dict:
    keep = A.MODEL
    A.MODEL = size
    try:
        out = []
        for p in points:
            al = A.alpha_for(kulfan, p.cl, p.re, n_crit)
            a = A.aero(kulfan, al, p.re, n_crit)
            out.append({"alpha": r(al, 3), "cl": r(np.squeeze(a["CL"]), 4), "cd": r(np.squeeze(a["CD"])),
                        "cm": r(np.squeeze(a["CM"]), 4), "xtr_top": r(np.squeeze(a["Top_Xtr"]), 3),
                        "xtr_bot": r(np.squeeze(a["Bot_Xtr"]), 3)})
        st = A.aero(kulfan, STALL_ALPHAS, re_stall, n_crit)
        return {"points": out, "stall": {"alpha": STALL_ALPHAS.tolist(), "cl": np.round(st["CL"], 4).tolist()}}
    finally:
        A.MODEL = keep


# --------------------------------------------------------------------------- XFOIL

def xfoil(dat: Path, points, alphas, re_stall: float, n_crit: float) -> dict:
    out = []
    for p, a0 in zip(points, alphas):
        res = X.at_cl(dat, p.re, n_crit, p.cl, a0, cp=(p.name == "cruise"))
        if res is None:
            out.append(None)
            continue
        cp = res.pop("cp", None)
        row = {"alpha": r(res["alpha"], 3), "cl": r(res["cl"], 4), "cd": r(res["cd"]), "cd_pressure": r(res["cdp"]),
               "cm": r(res["cm"], 4), "xtr_top": r(res["xtr_top"], 3), "xtr_bot": r(res["xtr_bot"], 3)}
        if cp:
            xs, cps = np.array(cp["x"]), np.array(cp["cp"])
            le = int(np.argmin(xs))
            row["surface"] = {"upper": {"x": np.round(xs[:le + 1][::-1], 4).tolist(), "cp": np.round(cps[:le + 1][::-1], 4).tolist()},
                              "lower": {"x": np.round(xs[le:], 4).tolist(), "cp": np.round(cps[le:], 4).tolist()}}
        out.append(row)
    sw = X.sweep(dat, re_stall, n_crit, STALL_ALPHAS)
    cruise = points[1]
    rough = {}
    for n in (5.0, 9.0):
        res = X.at_cl(dat, cruise.re, n, cruise.cl, alphas[1])
        rough[str(int(n))] = r(res["cd"]) if res else None
    rough["7"] = out[1]["cd"] if out[1] else None
    return {"points": out, "stall": {"alpha": [p["alpha"] for p in sw], "cl": [r(p["cl"], 4) for p in sw],
                                     "cd": [r(p["cd"]) for p in sw]},
            "n_crit_cruise": dict(sorted(rough.items(), key=lambda kv: float(kv[0])))}


# --------------------------------------------------------------------------- OpenFOAM

def cfd_all(kulfans: dict, points, xfoil_alphas: dict, n_crit: float, work: Path, log, resume: bool = False) -> dict:
    """Two angles per point and section (the second restarted from the first), the
    drag at the design lift coefficient by interpolation; and one coarser grid.
    With `resume`, cases already in `work` carry on from where they stopped."""
    meshes = {}
    for key, k in kulfans.items():
        af = asb.KulfanAirfoil(**k)
        m = work / key / "mesh"
        shutil.rmtree(m, ignore_errors=True)
        m.mkdir(parents=True)
        info = C.mesh(m, af.upper_coordinates, af.lower_coordinates)
        C.write_case(m, points[1].re, 0.0, n_crit)
        info["check"] = C.check_mesh(m)
        meshes[key] = info
        log(f"  mesh {key}: {info['cells']} cells, non-orthogonality {info['check'].get('non_orthogonality_max')}°")

    def case_dir(key, name, mesh_dir):
        d = work / key / name
        if resume and (d / "case.json").exists():
            return d
        shutil.rmtree(d, ignore_errors=True)
        (d / "constant").mkdir(parents=True)
        (d / "constant" / "polyMesh").symlink_to((mesh_dir / "constant" / "polyMesh").resolve())
        shutil.copy(mesh_dir / "wall.npz", d / "wall.npz")
        return d

    def solved(d, **run_kw):
        """A case's results, from its result.json when an earlier run finished it."""
        f = d / "result.json"
        if resume and f.exists():
            return json.loads(f.read_text())
        res = C.results(d, C.run(d, log=log, **run_kw))
        if "error" not in res:
            f.write_text(json.dumps(res))
        return res

    def task(key, i):
        p = points[i]
        a1 = xfoil_alphas[key][i]
        d1 = case_dir(key, f"{p.name}_a", work / key / "mesh")
        if not (d1 / "case.json").exists():
            C.write_case(d1, p.re, a1, n_crit)
        r1 = solved(d1)
        if "error" in r1:
            return key, i, {"runs": [r1], "error": r1["error"]}
        slope = 0.105                                      # per degree, a first guess
        a2 = a1 + (p.cl - r1["cl"]) / slope
        if abs(a2 - a1) < 0.5:
            a2 = a1 + (0.75 if p.cl >= r1["cl"] else -0.75)
        d2 = work / key / f"{p.name}_b"
        if not (resume and (d2 / "case.json").exists()):
            shutil.rmtree(d2, ignore_errors=True)
            C.restart_at(d1, d2, a2)
        r2 = solved(d2, max_iter=4000, min_iter=1000)
        if "error" in r2:
            return key, i, {"runs": [r1, r2], "error": r2["error"]}
        return key, i, interpolate(r1, r2, p.cl)

    def grid_task():
        key, i = "baseline", 1
        p = points[i]
        m = work / key / "mesh_coarse"
        shutil.rmtree(m, ignore_errors=True)
        m.mkdir(parents=True)
        af = asb.KulfanAirfoil(**kulfans[key])
        info = C.mesh(m, af.upper_coordinates, af.lower_coordinates, scale=0.67)
        d = case_dir(key, "cruise_coarse", m)
        if not (d / "case.json").exists():
            C.write_case(d, p.re, xfoil_alphas[key][i], n_crit)
        res = dict(solved(d))
        res.pop("surface", None)
        return {"cells": info["cells"], **res}

    def verify_task():
        """The set-up against XFOIL where both are dependable: the NACA 4412 at Re 1e6."""
        d = case_dir("baseline", "verify_re1e6", work / "baseline" / "mesh")
        if not (d / "case.json").exists():
            C.write_case(d, VERIFY["re"], VERIFY["alpha"], VERIFY["n_crit"])
        res = dict(solved(d, max_iter=6500, min_iter=4000))
        for k in ("surface", "history"):
            res.pop(k, None)
        return res

    out = {"meshes": meshes, "points": {k: [None] * len(points) for k in kulfans}}
    with cf.ThreadPoolExecutor(max_workers=C.cpus() + 1) as ex:
        verify = ex.submit(verify_task)
        futs = [ex.submit(task, k, i) for k in kulfans for i in range(len(points))]
        grid = ex.submit(grid_task)
        for f in cf.as_completed(futs):
            key, i, res = f.result()
            out["points"][key][i] = res
            log(f"  CFD {key} {points[i].name}: " + (res.get("error") or f"cd {res['cd']:.5f} at cl {res['cl']:.3f}"))
        out["grid"] = grid.result()
        out["verify"] = verify.result()
        log(f"  CFD check at Re {VERIFY['re']:.0e}: cl {out['verify'].get('cl', 0):.3f}, cd {out['verify'].get('cd', 0):.5f}")
    fine = out["points"]["baseline"][1]["runs"][0] if "runs" in out["points"]["baseline"][1] else None
    if fine and "cd" in out["grid"]:
        out["grid"]["fine_cells"] = meshes["baseline"]["cells"]
        out["grid"]["fine_cd"] = fine["cd"]
        out["grid"]["fine_cl"] = fine["cl"]
    return out


def interpolate(r1: dict, r2: dict, cl: float) -> dict:
    """Values at the design lift coefficient from the two angles (linear in cl)."""
    t = (cl - r1["cl"]) / (r2["cl"] - r1["cl"])
    mix = lambda a, b: a + t * (b - a)  # noqa: E731
    out = {"alpha": r(mix(r1["alpha"], r2["alpha"]), 3), "cl": r(cl, 4), "t": r(t, 3)}
    for k in ("cd", "cd_pressure", "cd_friction", "cm"):
        out[k] = r(mix(r1[k], r2[k]), 6 if k.startswith("cd") else 4)
    out["converged_runs"] = [bool(x["converged"]) for x in (r1, r2)]
    out["swing_cd"] = r(max(r1["cd_swing"], r2["cd_swing"]), 6)
    out["swing_cl"] = r(max(r1["cl_swing"], r2["cl_swing"]), 4)
    out["iterations"] = [r1["iterations"], r2["iterations"]]
    out["yplus_max"] = r(max(r1["yplus_max"], r2["yplus_max"]), 2)
    s1, s2 = r1.get("surface"), r2.get("surface")
    if s1 and s2:
        out["surface"] = {side: {"x": s1[side]["x"],
                                 "cp": np.round(np.array(s1[side]["cp"]) + t * (np.array(s2[side]["cp"]) - np.array(s1[side]["cp"])), 4).tolist(),
                                 "cf": np.round(np.array(s1[side]["cf"]) + t * (np.array(s2[side]["cf"]) - np.array(s1[side]["cf"])), 6).tolist()}
                          for side in ("upper", "lower")}
    bub = [x.get("bubble_upper") for x in (r1, r2)]
    out["bubble_upper"] = bub[0] if abs(t) < 0.5 else bub[1]
    out["runs"] = [{k: x[k] for k in ("alpha", "cl", "cd", "cd_pressure", "cd_friction", "cm", "iterations", "converged",
                                      "cd_swing", "cl_swing", "snapshots", "history")} for x in (r1, r2)]
    return out


# --------------------------------------------------------------------------- the whole plane

class Section(asb.KulfanAirfoil):
    """A Kulfan airfoil that NeuralFoil analyses at the study's n_crit inside
    AeroBuildup (which otherwise uses NeuralFoil's default, 9: a smooth wing)."""
    n_crit = 7.0

    def get_aero_from_neuralfoil(self, *args, **kwargs):
        kwargs.setdefault("n_crit", self.n_crit)
        return super().get_aero_from_neuralfoil(*args, **kwargs)


def section(kulfan: dict, name: str, n_crit: float) -> Section:
    s = Section(name=name, **kulfan)
    s.n_crit = n_crit
    return s


def airplane(kulfan: dict, cond: dict, n_crit: float) -> asb.Airplane:
    """Kipinä in AeroSandbox: the rectangular wing on the pod, the twin booms and
    fins, the stabiliser between them (metres, from airframe/design.py)."""
    L, p, mm = cond["L"], AF.BASE, 1e-3
    wing_af = section(kulfan, "wing", n_crit)
    t = min(max(round(100 * p.plate / L.c_h), 3), 12)
    k = asb.Airfoil(f"naca00{t:02d}").to_kulfan_airfoil()
    plate = section({"upper_weights": k.upper_weights, "lower_weights": k.lower_weights,
                     "leading_edge_weight": k.leading_edge_weight, "TE_thickness": k.TE_thickness}, "plate", n_crit)
    wing = asb.Wing(name="wing", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[L.x_le * mm, 0, L.z_top * mm], chord=L.chord * mm, airfoil=wing_af),
        asb.WingXSec(xyz_le=[L.x_le * mm, p.span / 2 * mm, L.z_top * mm], chord=L.chord * mm, airfoil=wing_af)])
    stab = asb.Wing(name="stabiliser", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[L.x_stab * mm, 0, L.tail_z * mm], chord=L.c_h * mm, airfoil=plate),
        asb.WingXSec(xyz_le=[L.x_stab * mm, L.b_h / 2 * mm, L.tail_z * mm], chord=L.c_h * mm, airfoil=plate)])
    z0 = L.tail_z - p.fin_below
    fins = asb.Wing(name="fins", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[L.x_stab * mm, L.tube_y * mm, z0 * mm], chord=L.c_h * mm, airfoil=plate),
        asb.WingXSec(xyz_le=[L.x_stab * mm, L.tube_y * mm, (z0 + L.fin_h) * mm], chord=L.c_h * mm, airfoil=plate)])
    w, h, zc = 2 * L.half_w, L.z_top - L.z_bottom, (L.z_top + L.z_bottom) / 2
    pod = asb.Fuselage(name="pod", xsecs=[
        asb.FuselageXSec(xyz_c=[x * mm, 0, zc * mm], width=w * mm, height=h * mm, shape=5)
        for x in np.linspace(0, L.pod_len, 8)])
    booms = [asb.Fuselage(name=f"boom {s:+d}", xsecs=[
        asb.FuselageXSec(xyz_c=[x * mm, s * L.tube_y * mm, 0], radius=p.tube_od / 2 * mm)
        for x in np.linspace(L.socket_x1, L.tube_x1, 4)]) for s in (-1, 1)]
    return asb.Airplane(name="Kipina", xyz_ref=[L.x_qc * mm, 0, 0], wings=[wing, stab, fins],
                        fuselages=[pod, *booms], s_ref=L.area * mm * mm, c_ref=L.chord * mm, b_ref=p.span * mm)


def _buildup(plane: asb.Airplane, v: float, alpha: float) -> asb.AeroBuildup:
    return asb.AeroBuildup(plane, asb.OperatingPoint(velocity=v, alpha=alpha), model_size="large")


def trimmed_buildup(plane: asb.Airplane, v: float, weight: float) -> dict:
    """AeroBuildup at the angle that carries the weight."""
    f = lambda a: {k: float(np.squeeze(x)) for k, x in _buildup(plane, v, a).run().items()  # noqa: E731
                   if k in ("L", "D", "CL", "CD")}
    alpha = 2.0
    for _ in range(6):
        r0, r1 = f(alpha), f(alpha + 0.5)
        alpha += (weight - r0["L"]) / ((r1["L"] - r0["L"]) / 0.5)
    return {"alpha": alpha, **f(alpha)}


def buildup_parts(plane: asb.Airplane, v: float, alpha: float) -> dict:
    """AeroBuildup's profile drag per component (no induced drag), newtons."""
    ab = _buildup(plane, v, alpha)
    drag = lambda comp: -float(np.squeeze(ab.op_point.convert_axes(*comp.F_g, from_axes="geometry", to_axes="wind")[0]))  # noqa: E731
    out = {}
    for wing in plane.wings:
        out[wing.name] = drag(ab.wing_aerodynamics(wing=wing, include_induced_drag=False))
    for fuse in plane.fuselages:
        out[fuse.name] = drag(ab.fuselage_aerodynamics(fuselage=fuse, include_induced_drag=False))
    return out


CATEGORIES = [("wing", "Wing profile"), ("tail", "Tail"), ("pod", "Pod"), ("booms", "Booms, sockets, tail mount"),
              ("small", "Horns, wires, antenna"), ("junctions", "Junctions"), ("induced", "Induced")]


def handbook_category(name: str) -> str:
    if name.startswith("wing"):
        return "wing"
    if name.startswith("tail plates"):
        return "tail"
    if name.startswith("pod"):
        return "pod"
    if name.startswith(("booms", "boom sockets", "tail mount")):
        return "booms"
    if name.startswith("junctions"):
        return "junctions"
    if name == "induced":
        return "induced"
    return "small"


def vlm_efficiency(plane: asb.Airplane) -> dict:
    """Span efficiency of the wing from a vortex lattice, in the Trefftz plane: the
    spanwise circulation fitted with a sine series, e = 1 / (1 + sum n (An / A1)^2).
    The study's wing drag already follows the lift coefficient (NeuralFoil at each
    speed), so this inviscid number is the like-for-like one for its induced drag."""
    wing = asb.Airplane(name="wing", xyz_ref=plane.xyz_ref, wings=plane.wings[:1], s_ref=plane.s_ref,
                        c_ref=plane.c_ref, b_ref=plane.b_ref)
    b, v = plane.b_ref, AF.SPEED
    rows = []
    for res_span in (12, 24, 48):
        vlm = asb.VortexLatticeMethod(wing, asb.OperatingPoint(velocity=v, alpha=4.0),
                                      spanwise_resolution=res_span, chordwise_resolution=10)
        vlm.run()
        y = np.round(vlm.vortex_centers[:, 1], 7)
        ys = np.unique(y)
        gam = np.array([vlm.vortex_strengths[y == s].sum() for s in ys])
        n = np.arange(1, 20, 2)
        an = np.linalg.lstsq(2 * b * v * np.sin(np.outer(np.arccos(-2 * ys / b), n)), gam, rcond=None)[0]
        rows.append({"panels_across": int(len(ys)), "e": r(1 / (1 + np.sum(n[1:] * (an[1:] / an[0]) ** 2)), 4)})
    return {"rows": rows, "e": rows[-1]["e"], "assumed": AF.E_OSWALD}


def plane_check(kulfans: dict, study: dict, cond: dict, n_crit: float, log) -> dict:
    w = cond["mass"] / 1000 * G
    s = cond["s"]
    speeds = [9.5, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0]
    out = {"weight_n": r(w, 3), "speeds": speeds, "variants": {}, "categories": CATEGORIES}
    vlm = None
    for key, variant in (("baseline", "baseline"), ("optimized", "airfoil")):
        plane = airplane(kulfans[key], cond, n_crit)
        if vlm is None:
            vlm = vlm_efficiency(plane)
            log(f"  vortex lattice: span efficiency {vlm['e']:.3f} (the study assumes {vlm['assumed']})")
        rows = {row["v"]: row for row in study["variants"][variant]["rows"]}
        ab, hb, hb_vlm = [], [], []
        for v in speeds:
            t = trimmed_buildup(plane, v, w)
            ab.append(r(t["D"], 4))
            row = rows[min(rows, key=lambda x: abs(x - v))]
            hb.append(row["drag"])
            q = 0.5 * 1.225 * v * v
            cl = w / (q * s)
            hb_vlm.append(r(row["drag"] - row["induced"] + q * s * cl * cl / (math.pi * vlm["e"] * AF.BASE.aspect_ratio), 4))
        v = AF.SPEED
        t = trimmed_buildup(plane, v, w)
        parts = buildup_parts(plane, v, t["alpha"])
        cats_ab = {c: 0.0 for c, _ in CATEGORIES}
        for name, d in parts.items():
            cats_ab["tail" if name in ("stabiliser", "fins") else "booms" if name.startswith("boom") else name] += d
        cats_ab["induced"] = t["D"] - sum(parts.values())
        cruise = rows[min(rows, key=lambda x: abs(x - v))]
        cats_hb = {c: 0.0 for c, _ in CATEGORIES}
        for name, d in cruise["items"]:
            cats_hb[handbook_category(name)] += d
        out["variants"][key] = {"buildup": ab, "handbook": hb, "handbook_vlm": hb_vlm, "alpha_cruise": r(t["alpha"], 2),
                                "cruise": {"buildup": r(t["D"], 4), "handbook": cruise["drag"],
                                           "handbook_vlm": hb_vlm[speeds.index(13.0)],
                                           "buildup_parts": {c: r(d, 4) for c, d in cats_ab.items()},
                                           "handbook_parts": {c: r(d, 4) for c, d in cats_hb.items()}}}
        log(f"  {key}: drag at {v:g} m/s, handbook {cruise['drag']:.3f} N, AeroBuildup {t['D']:.3f} N")
    out["vlm"] = vlm
    return out


# --------------------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cfd", action="store_true", help="also run the OpenFOAM 2D CFD (an hour or two)")
    ap.add_argument("--work", type=Path, help="where to keep the CFD cases (default: a temporary folder)")
    ap.add_argument("--resume", action="store_true", help="carry on with the CFD cases already in --work")
    a = ap.parse_args(argv)
    log = lambda s: print(s, flush=True)  # noqa: E731
    t0 = time.time()
    study = json.loads((OUT / "study.json").read_text())
    n_crit = study["n_crit"]
    cond = E.conditions()
    points, lim = cond["points"], cond["lim"]
    kulfans = {k: kulfan_of(study["shapes"][k]) for k in KEYS}
    fp = fingerprint(study["shapes"])
    old = {}
    if (OUT / "crosscheck.json").exists():
        old = json.loads((OUT / "crosscheck.json").read_text())
    out = {"generated": time.strftime("%Y-%m-%d"), "fingerprint": fp, "n_crit": n_crit,
           "points": study["points"], "re_stall": round(lim.re_stall), "tools": {}}

    log("NeuralFoil:")
    nf = {size: {k: neuralfoil(kulfans[k], points, lim.re_stall, n_crit, size) for k in KEYS}
          for size in ("large", "xxxlarge")}
    out["tools"]["neuralfoil"] = {"label": f"NeuralFoil {A.nf.__version__} (large)", "sections": nf["large"]}
    out["tools"]["neuralfoil_xxxl"] = {"label": f"NeuralFoil {A.nf.__version__} (xxxlarge)", "sections": nf["xxxlarge"]}
    for size in nf:
        log(f"  {size:8s} cruise cd: " + ", ".join(f"{k} {nf[size][k]['points'][1]['cd']:.5f}" for k in KEYS))

    log("XFOIL:")
    tmp = Path(tempfile.mkdtemp(prefix="kipina-xfoil-"))
    xf = {}
    for k in KEYS:
        xy = np.asarray(asb.KulfanAirfoil(**kulfans[k]).to_airfoil(n_coordinates_per_side=200).coordinates)
        dat = tmp / f"{k}.dat"
        X.write_dat(dat, xy[:, 0], xy[:, 1], k)
        xf[k] = xfoil(dat, points, [p["alpha"] for p in nf["large"][k]["points"]], lim.re_stall, n_crit)
        log(f"  {k:9s} cd " + ", ".join(f"{p.name} {x['cd']:.5f}" if x else f"{p.name} -" for p, x in zip(points, xf[k]["points"])))
    pts = X.run(tmp / "baseline.dat", VERIFY["re"], VERIFY["n_crit"],
                ["ALFA 0", "PACC", "polar.txt", "", f"ASEQ 0.5 {VERIFY['alpha']:g} 0.5"])["points"]
    ver = next((p for p in pts if abs(p["alpha"] - VERIFY["alpha"]) < 1e-6), None)
    shutil.rmtree(tmp, ignore_errors=True)
    out["tools"]["xfoil"] = {"label": X.version(), "sections": xf, "verify": ver}

    if a.cfd:
        if not C.available():
            raise SystemExit("OpenFOAM's simpleFoam is not on the PATH (apt install openfoam)")
        log(f"OpenFOAM ({C.cpus()} at a time):")
        work = a.work or Path(tempfile.mkdtemp(prefix="kipina-cfd-"))
        work.mkdir(parents=True, exist_ok=True)
        alphas = {k: [p["alpha"] if p else n["alpha"] for p, n in zip(xf[k]["points"], nf["large"][k]["points"])]
                  for k in KEYS}
        t1 = time.time()
        res = cfd_all(kulfans, points, alphas, n_crit, work, log, resume=a.resume and a.work is not None)
        out["tools"]["openfoam"] = {"label": f"{C.version()} simpleFoam, k-omega SST + Langtry-Menter",
                                    "sections": {k: {"points": res["points"][k]} for k in KEYS},
                                    "meshes": res["meshes"], "grid": res["grid"],
                                    "verify": {**VERIFY, "cfd": res.get("verify"), "xfoil": out["tools"]["xfoil"]["verify"]},
                                    "turbulence": {"tu_percent": r(100 * C.turbulence(points[1].re, n_crit)["tu"], 3)},
                                    "minutes": round((time.time() - t1) / 60)}
        if a.work is None:
            shutil.rmtree(work, ignore_errors=True)
    elif old.get("fingerprint") == fp and "openfoam" in old.get("tools", {}):
        out["tools"]["openfoam"] = old["tools"]["openfoam"]
        log("OpenFOAM: kept the earlier results (same sections)")

    log("Whole plane:")
    out["plane"] = plane_check(kulfans, study, cond, n_crit, log)
    out["summary"] = summary(out)
    out["seconds"] = round(time.time() - t0)
    (OUT / "crosscheck.json").write_text(json.dumps(out, separators=(",", ":")))
    (OUT / "CROSSCHECK.md").write_text(report(out, study))
    log(f"wrote aero/study/crosscheck.json and CROSSCHECK.md in {out['seconds']} s")


def summary(out: dict) -> dict:
    """Per tool: cd of both sections at each point and the change."""
    rows = {}
    for tool, data in out["tools"].items():
        secs = data["sections"]
        cds = {k: [p["cd"] if p and "cd" in p else None for p in secs[k]["points"]] for k in KEYS}
        change = [None if b is None or o is None else round((o - b) / b * 100, 1) for b, o in zip(cds["baseline"], cds["optimized"])]
        weights = [p["weight"] for p in out["points"]]
        ok = all(c is not None for c in cds["baseline"] + cds["optimized"])
        wsum = (lambda k: sum(w * c for w, c in zip(weights, cds[k]))) if ok else None
        rows[tool] = {"label": data["label"], "baseline": cds["baseline"], "optimized": cds["optimized"],
                      "change": change, "weighted_change": round((wsum("optimized") - wsum("baseline")) / wsum("baseline") * 100, 1) if ok else None}
    return rows


def report(out: dict, study: dict) -> str:
    pts = out["points"]
    L = ["# Cross-checks of the efficiency study", "",
         f"Generated by `aero/crosscheck.py` from the sections in `study.json` (n_crit {out['n_crit']:g}). "
         "Do not edit by hand. The charts are in [index.html](index.html#crosscheck).", "",
         "## The wing section: drag at the design points", "",
         "| tool | section | " + " | ".join(f"{p['name']} (cl {p['cl']:.2f}, Re {p['re'] / 1e3:.0f}k)" for p in pts) + " | weighted change |",
         "|---|---|" + "---|" * len(pts) + "---|"]
    for tool, s in out["summary"].items():
        for k in KEYS:
            cells = " | ".join("-" if c is None else f"{c:.4f}" for c in s[k])
            L.append(f"| {s['label'] if k == 'baseline' else ''} | {'NACA 4412' if k == 'baseline' else 'optimized'} | {cells} | "
                     + (("" if s["weighted_change"] is None else f"{s['weighted_change']:+.1f} %") if k == "optimized" else "") + " |")
        L.append("| | change | " + " | ".join("-" if c is None else f"{c:+.0f} %" for c in s["change"]) + " | |")
    xf = out["tools"]["xfoil"]["sections"]
    nf = out["tools"]["neuralfoil"]["sections"]
    L += ["", "## Maximum lift at 9 m/s (Re " + f"{out['re_stall'] / 1e3:.0f}k)", "", "| tool | NACA 4412 | optimized |", "|---|---|---|"]
    for tool, secs in (("NeuralFoil (large)", nf), ("NeuralFoil (xxxlarge)", out["tools"]["neuralfoil_xxxl"]["sections"]), ("XFOIL", xf)):
        cells = []
        for k in KEYS:
            st = secs[k]["stall"]
            i = int(np.argmax(st["cl"]))
            cells.append(f"{st['cl'][i]:.2f} at {st['alpha'][i]:g}°")
        L.append(f"| {tool} | " + " | ".join(cells) + " |")
    L += ["", "XFOIL cruise cd against n_crit (5 rough, 9 smooth): "
          + "; ".join(f"{'NACA 4412' if k == 'baseline' else 'optimized'}: " + ", ".join(f"{n} → {v:.4f}" for n, v in xf[k]["n_crit_cruise"].items() if v)
                      for k in KEYS) + "."]
    of = out["tools"].get("openfoam")
    if of:
        L += ["", "## The CFD runs", "", f"{of['label']}, free-stream turbulence {of['turbulence']['tu_percent']} % "
              f"(n_crit {out['n_crit']:g} by Mack's relation), {of['meshes']['baseline']['cells']} cells, y+ under 1.", "",
              "| section | point | angle, ° | cd | pressure | friction | cm | upper-surface bubble, x/c |", "|---|---|---|---|---|---|---|---|"]
        for k in KEYS:
            for p, x in zip(pts, of["sections"][k]["points"]):
                if not x or "cd" not in x:
                    L.append(f"| {k} | {p['name']} | failed | | | | | |")
                    continue
                b = x.get("bubble_upper")
                L.append(f"| {'NACA 4412' if k == 'baseline' else 'optimized'} | {p['name']} | {x['alpha']:.2f} | {x['cd']:.4f} | "
                         f"{x['cd_pressure']:.4f} | {x['cd_friction']:.4f} | {x['cm']:.3f} | {'-' if not b else f'{b[0]:.2f}-{b[1]:.2f}'} |")
        v = of.get("verify", {})
        if v.get("cfd") and v.get("xfoil") and "cd" in v["cfd"]:
            c, x = v["cfd"], v["xfoil"]
            L += ["", f"Set-up check (NACA 4412, Re {v['re']:.0e}, {v['alpha']:g}°, n_crit {v['n_crit']:g}): CFD cl {c['cl']:.3f}, "
                  f"cd {c['cd']:.5f}; XFOIL cl {x['cl']:.3f}, cd {x['cd']:.5f} "
                  f"({(c['cl'] - x['cl']) / x['cl'] * 100:+.1f} % lift, {(c['cd'] - x['cd']) / x['cd'] * 100:+.1f} % drag)."]
        g = of.get("grid", {})
        if "fine_cd" in g and "cd" in g:
            L += ["", f"Grid check (NACA 4412 at cruise, same angle): {g['cells']} cells cd {g['cd']:.5f}, cl {g['cl']:.4f}; "
                  f"{g['fine_cells']} cells cd {g['fine_cd']:.5f}, cl {g['fine_cl']:.4f} "
                  f"({(g['cd'] - g['fine_cd']) / g['fine_cd'] * 100:+.1f} % drag on the coarser grid)."]
    pl = out["plane"]
    L += ["", "## The whole plane", "",
          f"Span efficiency from the vortex lattice: {pl['vlm']['e']:.2f} (the study assumes {pl['vlm']['assumed']}).", "",
          "| drag, N | " + " | ".join(f"{v:g} m/s" for v in pl["speeds"]) + " |", "|---|" + "---|" * len(pl["speeds"])]
    for k in KEYS:
        v = pl["variants"][k]
        name = "as built" if k == "baseline" else "optimized airfoil"
        L.append(f"| handbook, {name} | " + " | ".join(f"{d:.3f}" for d in v["handbook"]) + " |")
        L.append(f"| handbook with the lattice's induced drag, {name} | " + " | ".join(f"{d:.3f}" for d in v["handbook_vlm"]) + " |")
        L.append(f"| AeroBuildup, {name} | " + " | ".join(f"{d:.3f}" for d in v["buildup"]) + " |")
    v = pl["variants"]["baseline"]["cruise"]
    L += ["", f"At {AF.SPEED:g} m/s, as built, part by part (N):", "", "| part | handbook | AeroBuildup |", "|---|---|---|"]
    L += [f"| {label} | {v['handbook_parts'][c]:.4f} | {v['buildup_parts'][c]:.4f} |" for c, label in pl["categories"]]
    L.append(f"| total | {v['handbook']:.4f} | {v['buildup']:.4f} |")
    L.append("")
    return "\n".join(L)


if __name__ == "__main__":
    main()
