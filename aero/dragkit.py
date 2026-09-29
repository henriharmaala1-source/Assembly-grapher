"""Handbook drag estimates for a quick component build-up.

Every item is a drag area, CdA = Cd x its reference area (m^2). Their sum times
the dynamic pressure q = rho v^2 / 2 is the parasite drag. The methods are the
usual first-pass ones:

  * skin friction: flat-plate Cf (Blasius laminar, Prandtl-Schlichting
    turbulent, Schlichting's mixed-flow correction) times a form factor
    (Raymer, Aircraft Design: A Conceptual Approach, ch. 12);
  * blunt fronts: forebody pressure drag against the edge radius, fitted to
    Hoerner (Fluid-Dynamic Drag, ch. 3, flat-faced bodies with rounded edges)
    and Hucho (Aerodynamics of Road Vehicles, boxes and buses near the ground);
  * blunt bases, cylinders and plates across the flow: Hoerner.

Expect +-30 % on the absolute numbers. The difference between two versions of
the same shape is more trustworthy than either total.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

RHO = 1.225          # kg/m^3, sea level, 15 C
NU = 1.46e-5         # m^2/s

CD_BASE = 0.20       # square-cut base, on the base area (Hoerner, Hucho: 0.15-0.25)
CD_CYLINDER = 1.1    # round wire or rod across the flow, Re 10^2..10^5
CD_PLATE = 1.2       # thin flat strip edge-on across the flow (a servo horn, a lug)
CD_BUMP = 0.5        # small bump with a sloped front on a surface


def q(v: float) -> float:
    """Dynamic pressure, Pa."""
    return 0.5 * RHO * v * v


def reynolds(v: float, length_m: float) -> float:
    return v * length_m / NU


def cf_turbulent(re: float) -> float:
    return 0.455 / math.log10(max(re, 1e4)) ** 2.58


def cf_laminar(re: float) -> float:
    return 1.328 / math.sqrt(max(re, 1e3))


def cf(re: float, laminar: float = 0.0) -> float:
    """Mean skin-friction coefficient over a length at Reynolds number `re`,
    with the first `laminar` fraction of the length laminar."""
    if laminar <= 0:
        return cf_turbulent(re)
    re_t = re * laminar
    return cf_turbulent(re) - laminar * cf_turbulent(re_t) + laminar * cf_laminar(re_t)


def ff_wing(tc: float, x_tmax: float = 0.3) -> float:
    """Raymer's low-speed form factor for a wing or tail section of thickness
    ratio `tc` with its thickest point at `x_tmax` of the chord."""
    return 1 + 0.6 / x_tmax * tc + 100 * tc ** 4


def ff_body(fineness: float) -> float:
    """Raymer's form factor for a body of length / diameter `fineness`."""
    f = max(fineness, 1.0)
    return 1 + 60 / f ** 3 + f / 400


def hydraulic_d(w: float, h: float) -> float:
    """Hydraulic diameter of a w x h rectangle (same units)."""
    return 2 * w * h / (w + h)


def cd_forebody(r_over_d: float, sharp: float, scale: float, rounded: float = 0.03) -> float:
    """Pressure drag of a blunt front face, on its frontal area.

    `sharp` is the value with square edges; rounding the edges to a radius r
    lets the flow turn the corner instead of separating, and the drag falls
    roughly exponentially with r/d towards `rounded`. Two calibrations are used:

      * a free body (Hoerner, flat-faced cylinders): sharp 0.75, d = hydraulic
        diameter of the face, most of the drag gone by r/d ~ 0.1 (scale 0.04);
      * a box near the ground (Hucho, bus and box tests): sharp 0.5, d = sqrt of
        the frontal area, most of the drag gone by r/d ~ 0.045 (scale 0.015).
    """
    return rounded + (sharp - rounded) * math.exp(-max(r_over_d, 0.0) / scale)


def chamfer_radius(c: float) -> float:
    """A 45 deg chamfer of size c works about like an edge radius of 0.7 c."""
    return 0.7 * c


def cylinder_cda(d: float, length: float, lean_deg: float = 0.0) -> float:
    """Round rod or wire across the flow (m in, m^2 out). Leaning it back by
    `lean_deg` cuts the drag roughly as cos^3 (the cross-flow principle)."""
    return CD_CYLINDER * d * length * math.cos(math.radians(lean_deg)) ** 3


@dataclass
class Item:
    """One line of a drag build-up."""
    name: str
    cda: float                 # m^2
    group: str = ""            # for summing, e.g. "friction", "pressure", "protuberances"
    note: str = ""


def total(items: list[Item]) -> float:
    return sum(i.cda for i in items)
