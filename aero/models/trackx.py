"""Patria TRACKX exterior (trackx/): drag model for aero/optimize.py.

The dimensions are read from the constants at the top of trackx/build.py (no
CadQuery needed). The body is treated the way road-vehicle aerodynamics treats
a truck: a front face whose drag depends on how round its edges are, a blunt
base, skin friction, the running gear, the underbody and the add-on parts.
"""
from __future__ import annotations

import ast
import math
from pathlib import Path

import dragkit as K
from model import Param, Result

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "trackx" / "build.py"

NAME = "trackx"
TITLE = "Patria TRACKX (exterior model)"
AREA_UNIT = ("m²", 1.0)
SPEED = 60 / 3.6
NOTE = ("The drag is the aerodynamic drag only; for a tracked vehicle the track and rolling "
        "losses are larger at any road speed, so the fuel figures are the air's share alone.")

BOATTAIL_DEG = 12.0
CD_RUNNING_GEAR = 0.5          # open tracks, sprocket and idler, on their frontal area
CD_SHIELDED = 0.2              # the part of the track front behind a faired shield
CD_UNDERBODY = 0.04            # belly, suspension arms, on the whole frontal area
CD_FITTINGS = 0.03             # lamps, wipers, rails, guard ends, louvres, shackles, bolts
CD_BUMPER = 0.3                # a flat-faced bar standing proud of the front face
BSFC = 0.22                    # kg of diesel per kWh at the crankshaft
DRIVE_EFF = 0.8                # crankshaft to sprocket
DIESEL = 0.835                 # kg/L


def _constants() -> dict:
    """Module-level constants of trackx/build.py, evaluated in order."""
    env: dict = {}
    for node in ast.parse(SRC.read_text()).body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        try:
            val = eval(compile(ast.Expression(node.value), str(SRC), "eval"),
                       {"__builtins__": {}, "math": math}, dict(env))
        except Exception:
            continue
        t = node.targets[0]
        if isinstance(t, ast.Name):
            env[t.id] = val
        elif isinstance(t, ast.Tuple) and all(isinstance(e, ast.Name) for e in t.elts):
            for e, v in zip(t.elts, val):
                env[e.id] = v
    return env


C = _constants()
MM = 1e-3
W_UP = 2 * C["HULL_HW"] * MM                               # upper hull, across the tracks
H_UP = (C["Z_ROOF"] - C["Z_SPONSON"]) * MM
LEAN = 2 * 0.5 * (C["HULL_HW"] - C["ROOF_HW"]) * (C["Z_ROOF"] - C["Z_RAIL"]) * MM * MM
A_UP = W_UP * H_UP - LEAN
A_TRACKS = 2 * C["TRACK_W"] * C["Z_SPONSON"] * MM * MM
A_TUB = 2 * C["TUB_HW"] * (C["Z_SPONSON"] - C["Z_BELLY"]) * MM * MM
A_FRONT = A_UP + A_TRACKS + A_TUB
SQRT_A = math.sqrt(A_FRONT)
LENGTH = (C["X_BUMPER"] - C["X_REAR"]) * MM
# roof edge: the slope from the windscreen top to the roof, worked out as a chamfer
_ws_top, _roof = C["WS_T"], (2800.0, C["Z_ROOF"])
ROOF_EDGE_R = K.chamfer_radius(math.sqrt(abs(_ws_top[0] - _roof[0]) * abs(_roof[1] - _ws_top[1])))
# cab corners: the angled crew-door faces between the windscreen and the side walls
CAB_CORNER_R = K.chamfer_radius(C["HULL_HW"] - C["WS_HW"])
H_CAB = (C["Z_ROOF"] - C["DECK_F"][1]) * MM                 # the angled corners' height
H_BONNET = H_UP - H_CAB                                     # below them: the bonnet corners
BONNET_CORNER = 200.0                                       # the 45 deg plan chamfer (build.py: fc)
SHIELD_BASE = 980.0                                         # the bumper's end plates already reach down here


def params() -> list[Param]:
    return [
        Param("roof_edge_r", round(ROOF_EDGE_R / 10) * 10, 0, 300, 10, "mm",
              "radius of the edge from the windscreen top to the roof",
              "300 mm keeps the windscreen's top and the roof hatches"),
        Param("bonnet_corner", BONNET_CORNER, 0, 400, 20, "mm", "45° chamfer on the bonnet's outer corners",
              "400 mm reaches the headlamp pods"),
        Param("boattail", 0, 0, 800, 50, "mm",
              f"{BOATTAIL_DEG:.0f}° panels on the rear edges of the sides and roof",
              "800 mm folding panels, as on truck trailers; the rear door must still open", in_cad=False),
        Param("mirrors", 0, 0, 2, 1, "", "mirrors",
              "a camera-monitor system has to meet the road rules for mirrors (UN R46)", in_cad=False,
              labels={0: "outrigger frames", 1: "compact faired heads", 2: "camera pods"}),
        Param("track_shield", SHIELD_BASE, 600, SHIELD_BASE, 20, "mm",
              "lower edge of a faired shield in front of each track (height above ground)",
              "600 mm keeps the approach angle and clears the sprocket", in_cad=False),
    ]


def _mirrors(kind: int) -> float:
    if kind == 0:   # heads 150 x 240 mm on frames of 32 mm tube (build.py make_mirrors)
        return 2 * (K.CD_PLATE * 0.15 * 0.24 + K.cylinder_cda(0.032, 0.63 + 0.63 + 0.28))
    if kind == 1:   # 120 x 200 mm heads in a rounded housing on one faired arm
        return 2 * (0.5 * 0.12 * 0.20 + 0.3 * 0.03 * 0.30)
    return 2 * 0.4 * 0.06 * 0.08


def evaluate(x: dict, v: float) -> Result:
    items = []
    # front face of the upper hull: edge radii against sqrt(frontal area), Hucho's box data
    f = lambda r: K.cd_forebody(r * MM / SQRT_A, 0.5, 0.015)           # noqa: E731
    top_len, side_len = 2 * C["WS_HW"] * MM, 2 * H_UP
    side = (H_CAB * f(CAB_CORNER_R) + H_BONNET * f(K.chamfer_radius(x["bonnet_corner"]))) / H_UP
    cd_front = (top_len * f(x["roof_edge_r"]) + side_len * side) / (top_len + side_len)
    items.append(K.Item("front face (pressure)", cd_front * A_UP, "pressure",
                        f"Cd {cd_front:.3f} on {A_UP:.2f} m²; roof edge r {x['roof_edge_r']:.0f} mm"))
    items.append(K.Item("bumper", CD_BUMPER * 1.3 * 0.255, "pressure", "centre beam 1.3 x 0.26 m"))
    # running gear: the track fronts, partly behind the bumper's end plates or a shield
    shielded = (C["Z_SPONSON"] - x["track_shield"]) / C["Z_SPONSON"]
    cd_rg = CD_RUNNING_GEAR * (1 - shielded) + CD_SHIELDED * shielded
    items.append(K.Item("tracks and running gear", cd_rg * A_TRACKS, "pressure",
                        f"{A_TRACKS:.2f} m² of track front, {shielded * 100:.0f} % shielded"))
    # base: the rear of the upper hull, reduced by boat-tail panels
    t = x["boattail"] * MM * math.tan(math.radians(BOATTAIL_DEG))
    a_base = (W_UP - 2 * t) * (H_UP - t) - LEAN
    items.append(K.Item("rear of the upper hull (base)", K.CD_BASE * a_base, "pressure",
                        f"{a_base:.2f} m² of blunt base" + (", boat-tailed" if t else "")))
    items.append(K.Item("rear of the lower hull (base)", K.CD_BASE * A_TUB, "pressure", ""))
    # skin friction on the sides, roof and belly; pressure drag is counted above
    wet = LENGTH * (2 * H_UP + 2 * C["ROOF_HW"] * MM + 2 * C["TUB_HW"] * MM)
    items.append(K.Item("skin friction", K.cf(K.reynolds(v, LENGTH)) * wet, "friction",
                        f"{wet:.0f} m² wetted"))
    items.append(K.Item("underbody", CD_UNDERBODY * A_FRONT, "pressure", "belly and suspension arms"))
    items.append(K.Item("mirrors", _mirrors(round(x["mirrors"])), "protuberances",
                        {0: "heads and tube frames", 1: "compact faired heads", 2: "camera pods"}[round(x["mirrors"])]))
    items.append(K.Item("fittings", CD_FITTINGS * A_FRONT, "protuberances",
                        "lamps, wipers, rails, guard ends, louvres, shackles"))

    cda = K.total(items)
    drag = K.q(v) * cda
    kw = drag * v / 1000
    fuel = kw / DRIVE_EFF * BSFC / DIESEL * 100 / (v * 3.6)
    metrics = [("Drag coefficient on the frontal area", f"{cda / A_FRONT:.2f} on {A_FRONT:.2f} m²"),
               ("Power to push the air", f"{kw:.1f} kW"),
               ("Diesel for the air alone", f"{fuel:.1f} L/100 km")]
    return Result(items, drag, [], metrics, [])


def speed_text(v: float) -> str:
    return f"{v * 3.6:.0f} km/h"


def force_text(n: float) -> str:
    return f"{n:.0f} N"


METHOD = """
* Dimensions are read from the constants in `trackx/build.py`: the frontal
  area is the upper hull across both tracks, the two track fronts and the
  lower hull between them.
* The front face uses Hucho's result for boxes and buses near the ground:
  a square-edged front costs about Cd 0.5, and rounding the edges to about
  4.5 % of the square root of the frontal area removes most of that. A 45°
  chamfer counts as a radius of 0.7 times its size. The roof edge and the
  side edges are weighted by their length; the angled cab corners already
  act as large radii.
* The base is a square-cut rear (Cd 0.2); boat-tail panels at 12° shrink it.
  Open tracks count Cd 0.5 on their frontal area, 0.2 behind a faired shield.
  Underbody, fittings and the bumper are handbook increments; the mirrors
  are plates and tubes across the flow.
* Fuel: drag x speed, 80 % from crankshaft to sprocket, 220 g/kWh, diesel
  at 0.835 kg/L. This is the air's share only; the tracks' own losses are
  not modelled.
* This is a first-pass build-up for comparing changes, perhaps +-30 % on the
  totals. The published data do not include a drag figure to check it against.
"""
