"""Wing-section optimization for the Kipinä airframe, with NeuralFoil.

NeuralFoil (MIT, Sharpe & Hansman) is a neural network trained on tens of
millions of XFOIL runs. It gives XFOIL-like lift, drag, moment, transition and
boundary-layer data for any shape in about a millisecond, which is fast enough
to optimize with. Shapes are Kulfan (CST) airfoils: 8 upper and 8 lower
weights, a leading-edge weight and a trailing-edge thickness.

What the optimizer does:

  * minimize the wing's profile drag, weighted over three operating points
    (loiter, cruise and fast), each at the lift coefficient and Reynolds number
    the plane actually flies at;
  * keep what the printed wing needs: room for the 3 mm and 2 mm spars, the
    SG90 aileron servo no deeper than now, a 0.8 mm printable trailing edge,
    enough depth at the aileron hinge;
  * keep what the plane needs: maximum lift at least as high as the NACA 4412's
    (so the stall speed holds), lift past the stall falling off no faster than
    the NACA 4412's (a gentle stall for hand launches), and a pitching moment
    no more nose-down (so the tail still trims);
  * stay where NeuralFoil is confident, so the optimizer cannot exploit gaps in
    its training data.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

import aerosandbox as asb
import neuralfoil as nf

N_W = 8                         # Kulfan weights per surface
MODEL = "large"                 # NeuralFoil model size
X_THICK = np.linspace(0.02, 0.98, 49)


@dataclass
class Point:
    """An operating point of the section."""
    name: str
    v: float                    # m/s
    cl: float
    re: float
    weight: float               # share of the objective


@dataclass
class Limits:
    """What the printed wing and the plane need (mm, from airframe/design.py)."""
    chord: float
    te: float                   # printable trailing edge
    main_spar: tuple            # (x/c, needed depth mm)
    rear_spar: tuple
    hinge: tuple                # aileron hinge (x/c, depth mm)
    servo_zone: tuple           # (x0, x1) mm along the chord where the servo tabs sit
    servo_width: float          # SG90 lying on its side
    servo_top_gap: float        # clearance under the upper skin
    servo_bump: float = 0.0     # how far the servo stands out below the wing now (set from the baseline)
    max_tc: float = 0.15
    re_stall: float = 60e3
    clmax: float = 0.0          # the baseline's, set later
    cl_post: float = 0.0        # the baseline's lowest lift past the stall, set later
    cm: float = 0.0
    min_confidence: float = 0.85


@dataclass
class Section:
    name: str
    kulfan: dict
    alphas: list = field(default_factory=list)   # at the operating points

    @property
    def airfoil(self) -> asb.KulfanAirfoil:
        return asb.KulfanAirfoil(name=self.name, **self.kulfan)


def fit(name: str, te: float) -> dict:
    """Kulfan parameters of a named airfoil (NACA 4-digit or the UIUC database), with our TE."""
    k = asb.Airfoil(name).to_kulfan_airfoil()
    return {"upper_weights": np.array(k.upper_weights, float), "lower_weights": np.array(k.lower_weights, float),
            "leading_edge_weight": float(k.leading_edge_weight), "TE_thickness": te}


def aero(kulfan: dict, alpha, re, n_crit: float) -> dict:
    return nf.get_aero_from_kulfan_parameters(kulfan, alpha=np.atleast_1d(alpha).astype(float),
                                              Re=np.broadcast_to(np.atleast_1d(re), np.shape(np.atleast_1d(alpha))).astype(float),
                                              n_crit=n_crit, model_size=MODEL)


def alpha_for(kulfan: dict, cl: float, re: float, n_crit: float) -> float:
    """Angle of attack for a lift coefficient (below stall), by interpolation then two Newton steps."""
    grid = np.linspace(-6, 12, 37)
    a = aero(kulfan, grid, re, n_crit)["CL"]
    k = int(np.argmax(a))
    alpha = float(np.interp(cl, a[:k + 1], grid[:k + 1]))
    for _ in range(2):
        c0, c1 = aero(kulfan, [alpha, alpha + 0.1], re, n_crit)["CL"]
        alpha += (cl - c0) / max((c1 - c0) / 0.1, 1e-3)
    return alpha


STALL_SWEEP = np.arange(4.0, 18.01, 0.5)
POST_STALL = (STALL_SWEEP >= 9.0) & (STALL_SWEEP <= 16.0)   # where the lift may fall away after its peak


def stall(kulfan: dict, re: float, n_crit: float, k: float = 40.0) -> tuple[float, float]:
    """Smooth maximum lift over an angle sweep, and the smooth minimum lift
    between 9 and 16 degrees (how far it falls past the peak), by log-sum-exp."""
    cl = np.asarray(aero(kulfan, STALL_SWEEP, re, n_crit)["CL"])
    top = float(np.log(np.sum(np.exp(k * (cl - cl.max())))) / k + cl.max())
    post = cl[POST_STALL]
    low = float(-np.log(np.sum(np.exp(-k * (post - post.min())))) / k + post.min())
    return top, low


def servo_bump(kulfan: dict, lim: Limits) -> float:
    """How far (mm) the servo on its side stands out below the lower skin: its top
    sits under the lowest point of the upper skin over the tab zone."""
    af = asb.KulfanAirfoil(**kulfan)
    xs = np.linspace(lim.servo_zone[0], lim.servo_zone[1], 16) / lim.chord
    cam, t = af.local_camber(x_over_c=xs), af.local_thickness(x_over_c=xs)
    upper, lower = (cam + t / 2) * lim.chord, (cam - t / 2) * lim.chord
    bottom = upper.min() - lim.servo_top_gap - lim.servo_width
    return float(lower.max() - bottom)


def geometry(kulfan: dict, lim: Limits) -> dict:
    af = asb.KulfanAirfoil(**kulfan)
    depth = lambda x: float(af.local_thickness(x_over_c=np.array([x]))[0]) * lim.chord  # noqa: E731
    t = af.local_thickness(x_over_c=X_THICK)
    return {"main": depth(lim.main_spar[0]), "rear": depth(lim.rear_spar[0]), "hinge": depth(lim.hinge[0]),
            "min_t": float(t.min()) * lim.chord, "max_tc": float(t.max()), "bump": servo_bump(kulfan, lim)}


def evaluate(kulfan: dict, alphas, points: list[Point], lim: Limits, n_crit: float) -> dict:
    """Everything the optimizer and the report need for one shape."""
    a = aero(kulfan, alphas, [p.re for p in points], n_crit)
    clmax, cl_post = stall(kulfan, lim.re_stall, n_crit)
    return {"cl": np.asarray(a["CL"]), "cd": np.asarray(a["CD"]), "cm": np.asarray(a["CM"]),
            "conf": np.asarray(a["analysis_confidence"]), "clmax": clmax, "cl_post": cl_post,
            **geometry(kulfan, lim)}


def pack(k: dict, alphas) -> np.ndarray:
    return np.concatenate([k["upper_weights"], k["lower_weights"], [k["leading_edge_weight"]], np.asarray(alphas) / 10])


def unpack(x: np.ndarray, te: float):
    k = {"upper_weights": x[:N_W], "lower_weights": x[N_W:2 * N_W], "leading_edge_weight": float(x[2 * N_W]),
         "TE_thickness": te}
    return k, x[2 * N_W + 1:] * 10


def optimize(start: Section, points: list[Point], lim: Limits, n_crit: float, base_cd: np.ndarray,
             maxiter: int = 150) -> tuple[Section, dict]:
    """SLSQP from one starting shape. Returns the best shape and its evaluation."""
    te = lim.te / lim.chord
    alphas0 = [alpha_for(start.kulfan, p.cl, p.re, n_crit) for p in points]
    x0 = pack(start.kulfan, alphas0)
    w = np.array([p.weight for p in points])
    cache = {}

    def ev(x):
        key = x.tobytes()
        if key not in cache:
            k, al = unpack(x, te)
            cache[key] = evaluate(k, al, points, lim, n_crit)
        return cache[key]

    def objective(x):
        r = ev(x)
        smooth = sum(np.sum(np.diff(x[s], 2) ** 2) for s in (slice(0, N_W), slice(N_W, 2 * N_W)))
        return float(np.sum(w * r["cd"] / base_cd)) + 2e-3 * smooth

    cons = [
        {"type": "eq", "fun": lambda x: ev(x)["cl"] - np.array([p.cl for p in points])},
        {"type": "ineq", "fun": lambda x: np.array([
            ev(x)["main"] - lim.main_spar[1], ev(x)["rear"] - lim.rear_spar[1], ev(x)["hinge"] - lim.hinge[1],
            ev(x)["min_t"] - lim.te, lim.max_tc - ev(x)["max_tc"],
            lim.servo_bump + 0.25 - ev(x)["bump"],
            ev(x)["clmax"] - (lim.clmax - 0.01), ev(x)["cl_post"] - (lim.cl_post - 0.03),
            ev(x)["cm"][1] - (lim.cm - 0.02),
            ev(x)["conf"].min() - lim.min_confidence])},
    ]
    bounds = ([(-0.3, 0.8)] * N_W + [(-0.8, 0.4)] * N_W + [(-0.3, 0.8)]
              + [(-0.6, 1.2)] * len(points))
    res = minimize(objective, x0, method="SLSQP", bounds=bounds, constraints=cons,
                   options={"maxiter": maxiter, "ftol": 1e-7, "eps": 1e-4})
    k, al = unpack(res.x, te)
    return Section(f"optimized from {start.name}", k, list(al)), {**ev(res.x), "success": bool(res.success),
                                                                   "message": str(res.message), "nit": int(res.nit)}


def feasible(r: dict, points: list[Point], lim: Limits, tol: float = 2e-3) -> list[str]:
    """Broken limits, as text (empty when the shape is acceptable)."""
    bad = []
    if np.max(np.abs(r["cl"] - np.array([p.cl for p in points]))) > 0.01:
        bad.append("misses a design lift coefficient")
    checks = [("main spar depth", r["main"] - lim.main_spar[1]), ("rear spar depth", r["rear"] - lim.rear_spar[1]),
              ("hinge depth", r["hinge"] - lim.hinge[1]), ("thickness", r["min_t"] - lim.te),
              ("maximum thickness", lim.max_tc - r["max_tc"]), ("servo bump", lim.servo_bump + 0.25 - r["bump"]),
              ("maximum lift", r["clmax"] - (lim.clmax - 0.01)), ("gentle stall", r["cl_post"] - (lim.cl_post - 0.03)),
              ("pitching moment", r["cm"][1] - (lim.cm - 0.02)),
              ("NeuralFoil confidence", r["conf"].min() - lim.min_confidence)]
    return bad + [name for name, g in checks if g < -tol]


def polar(kulfan: dict, re: float, n_crit: float, alphas=np.arange(-4.0, 16.01, 0.5)) -> dict:
    a = aero(kulfan, alphas, re, n_crit)
    return {"alpha": alphas.tolist(), "cl": np.round(a["CL"], 4).tolist(), "cd": np.round(a["CD"], 5).tolist(),
            "cm": np.round(a["CM"], 4).tolist(), "conf": np.round(a["analysis_confidence"], 3).tolist()}


def pressure(kulfan: dict, alpha: float, re: float, n_crit: float) -> dict:
    """Surface pressure from NeuralFoil's boundary-layer edge velocity: Cp = 1 - (ue / V)^2."""
    a = aero(kulfan, alpha, re, n_crit)
    x = nf.bl_x_points
    up = np.array([float(np.squeeze(a[f"upper_bl_ue/vinf_{i}"])) for i in range(len(x))])
    lo = np.array([float(np.squeeze(a[f"lower_bl_ue/vinf_{i}"])) for i in range(len(x))])
    return {"x": np.round(x, 4).tolist(), "upper": np.round(1 - up ** 2, 4).tolist(),
            "lower": np.round(1 - lo ** 2, 4).tolist(),
            "xtr_upper": round(float(np.squeeze(a["Top_Xtr"])), 3), "xtr_lower": round(float(np.squeeze(a["Bot_Xtr"])), 3)}


def coordinates(kulfan: dict, n: int = 121) -> dict:
    af = asb.KulfanAirfoil(**kulfan).to_airfoil(n_coordinates_per_side=n) if hasattr(
        asb.KulfanAirfoil, "to_airfoil") else asb.KulfanAirfoil(**kulfan)
    xy = np.asarray(af.coordinates)
    return {"x": np.round(xy[:, 0], 5).tolist(), "y": np.round(xy[:, 1], 5).tolist()}
