"""A typical 5-inch FPV quad: aerodynamic model for aero/optimize.py.

The quad is an ordinary 6S freestyle build, and its propulsion stays stock:
2306 1750 KV motors, 5.1 x 4.3 tri-blades, a 55 A 4-in-1 ESC and a 6S 1100 mAh
LiPo. What the optimizer changes is the airframe's aerodynamics:

  * the body: open frame, a canopy over the stack and camera, or a full fairing
    over the stack and battery;
  * where the battery sits, on top or underneath;
  * the arms' width and thickness, held to the stock arms' stiffness, and
    printed streamlined sleeves over them;
  * forward-tilted motor mounts, so the body flies level at cruise;
  * the VTX antenna.

Every one of these shows up in the power needed for a steady cruise, which is
the objective: the body's drag at the attitude that drag sets, the prop wash
pushing down on the arms, and the weight of any printed fairing. The model
has no speed or acceleration objective and no top-speed estimate, and the
propulsion is not touched, so the quad stays an ordinary FPV quad.
"""
from __future__ import annotations

import math

import dragkit as K
from model import Param, Result

NAME = "quad"
TITLE = "Typical 5-inch FPV quad, aerodynamics"
OBJECTIVE = "Power at cruise"
MAXIMIZE = False
BASE_LABEL = "typical build"
AREA_UNIT = ("cm²", 1e4)
SPEED = 60 / 3.6
NOTE = ("Every aerodynamic change ends up in this power: the body's drag, the prop wash "
        "on the arms and the weight of any printed fairing. The motors, props and "
        "battery stay stock. 60 km/h is a brisk but ordinary cruise; the model does not "
        "look at top speed.")
ITEMS_NOTE = ("Drag of the body, arms, motors and antennas at the cruise attitude. The props' "
              "own drag is part of the rotor power.")
MIN_GAIN = 0.002               # changes worth less than 0.2 % are below this model's resolution

G = 9.81
TW_MIN, TW_MAX = 5.0, 12.0     # the band ordinary freestyle quads fly with
AUW_MAX = 700.0                # g; what a 5-inch frame and props are built for
AVIONICS_W = 7.0               # FC, receiver, camera, 400 mW analog VTX
ETA_ESC = 0.95
KAPPA = 1.15                   # induced power over ideal

# stock propulsion: 2306 1750 KV, 5.1 x 4.3 x 3, 6S 1100 mAh LiPo
PROP_D_IN, PROP_PITCH_IN, BLADES = 5.1, 4.3, 3
KV, KM, IRON_10K = 1750.0, 0.0193, 4.0        # motor constant N m / sqrt W; iron loss W at 10 000 rpm
CELLS, V_CELL, PACK_AH, PACK_R = 6, 3.7, 1.1, 0.0285
USABLE = 0.80

# weights, g
BASE_G = 504.0                 # the typical build as a whole
ARM_LEN, ARM_EXPOSED, ARM_SPAN = 100.0, 75.0, 53.0   # mm: arm, beyond the body, across the flow (X at 45 deg)
CARBON = 1.6e-3                # g/mm^3
EXTRA_G = {"body": (0.0, 10.0, 35.0), "arm_sleeves": 12.0, "motor_tilt": 4.0}

# body, mm
STACK_W, STACK_H = 30.0, 30.0
PLATE = 40.0 * 90.0            # body plates, plan area
CANOPY = 40.0 * 40.0           # plan area a canopy covers
BATT = (75.0, 35.0, 31.0)      # 6S 1100 mAh, length, width, height
BELL, BELL_H = 28.0, 18.0
FAIRING = (110.0, 42.0, STACK_H + BATT[2] + 4.0)  # full fairing: length, width, height
DOWNLOAD_K = 0.545             # prop wash on the arms: 4.5 % of thrust for 14 mm flat arms in a hover
STOCK_ARM = (14.0, 5.0)        # width, thickness mm


def params() -> list[Param]:
    return [
        Param("body", 0, 0, 2, 1, "", "body",
              "canopy +10 g; full fairing +35 g, and it has to open for battery changes", "printed part",
              {0: "open frame", 1: "canopy over stack and camera", 2: "full fairing over stack and battery"}),
        Param("battery_mount", 0, 0, 1, 1, "", "battery position", "", "build choice",
              {0: "on top", 1: "underneath"}),
        Param("arm_width", STOCK_ARM[0], 10, 18, 1, "mm", "arm width",
              "arm stiffness up and down and sideways at least 80 % of the stock 14 x 5 mm arms", "frame choice"),
        Param("arm_thickness", STOCK_ARM[1], 4, 6, 0.5, "mm", "arm thickness", "as for the width", "frame choice"),
        Param("arm_sleeves", 0, 0, 1, 1, "", "streamlined printed sleeves on the arms", "+12 g", "printed part",
              {0: "none", 1: "fitted"}),
        Param("motor_tilt", 0, 0, 15, 1, "°", "motors tilted forward on printed wedges",
              "the camera's uptilt drops by the same angle; the quad hovers nose-up by it", "printed part"),
        Param("antenna", 0, 0, 2, 1, "", "VTX antenna", "", "part choice",
              {0: "upright lollipop", 1: "lollipop laid back 45°", 2: "stubby"}),
    ]


def prop_coeffs():
    """Figure of merit and static coefficients of the stock 5.1 x 4.3 tri-blade
    (T = CT rho n^2 D^4, P = CP rho n^3 D^5, n in rev/s)."""
    ratio = PROP_PITCH_IN / PROP_D_IN
    fm = 0.45 - 0.04 * (BLADES - 2) - 0.15 * (ratio - 0.6)
    ct = 0.105 * (ratio / 0.8) ** 0.8 * (BLADES / 2) ** 0.75
    return fm, ct, ct ** 1.5 / (math.sqrt(math.pi / 2) * fm)


def body_items(x: dict, theta: float) -> list[K.Item]:
    """Drag areas (m^2) with the body pitched nose-down by theta (negative: nose-up)."""
    c, s = math.cos(theta), abs(math.sin(theta))
    nose_down = theta >= 0
    body, under = round(x["body"]), round(x["battery_mount"]) == 1
    sleeves = round(x["arm_sleeves"]) == 1
    l, w, h = BATT
    items = []
    if body == 2:
        fl, fw, fh = FAIRING
        items.append(K.Item("body fairing (stack and battery)", (0.25 * fw * fh * c + 0.3 * fl * fw * s) * 1e-6,
                            "body", f"rounded pod {fl:.0f} x {fw:.0f} x {fh:.0f} mm"))
    else:
        canopy = body == 1
        items.append(K.Item("stack and camera, front", (0.5 if canopy else 1.1) * STACK_W * STACK_H * c * 1e-6,
                            "body", "under a canopy" if canopy else "open standoffs and camera cage"))
        items.append(K.Item("battery, front", 0.9 * w * h * c * 1e-6, "body",
                            f"{w:.0f} x {h:.0f} mm, {'underneath' if under else 'on top'}"))
        # the face the flow reaches from above (nose-down) or below (nose-up)
        cover = CANOPY if canopy else 0.0
        if nose_down == (not under):             # battery on the windward side
            windward = 0.9 * l * w + 1.1 * max(PLATE - l * w - cover, 0.0) + 0.4 * cover
        else:                                    # battery on the lee side, the plate faces the flow
            windward = 1.1 * max(PLATE - cover, 0.0) + 0.4 * cover
        items.append(K.Item("body, top or bottom face", (windward + 0.2 * PLATE) * s * 1e-6, "body",
                            "flow reaches it because the body is tilted"))
    aw, at = x["arm_width"], x["arm_thickness"]
    items.append(K.Item("arms", ((0.6 if sleeves else 1.2) * 4 * ARM_SPAN * at * c
                                 + (0.9 if sleeves else 1.2) * 4 * ARM_EXPOSED * aw * s) * 1e-6, "body",
                        f"{aw:.0f} x {at:.1f} mm" + (", sleeved" if sleeves else "")))
    items.append(K.Item("motors", 0.8 * 4 * (BELL * BELL_H * c + math.pi * (BELL / 2) ** 2 * s) * 1e-6, "body",
                        f"{BELL:.0f} mm bells"))
    ant = [1.1 * 5 * 60 + 0.5 * 254, 1.1 * 5 * 60 * math.cos(math.radians(45)) ** 3 + 0.5 * 254,
           1.1 * 8 * 25][round(x["antenna"])]
    items.append(K.Item("antennas", (ant + 1.1 * 2 * 1.5 * 50) * 1e-6, "protuberances",
                        ["upright lollipop", "lollipop laid back 45°", "stubby"][round(x["antenna"])] + " and receiver"))
    items.append(K.Item("junctions (10 %)", 0.10 * K.total(items), "interference", ""))
    return items


def evaluate(x: dict, v: float) -> Result:
    aw, at = x["arm_width"], x["arm_thickness"]
    sleeves = round(x["arm_sleeves"]) == 1
    arm_g = 4 * ARM_LEN * CARBON
    mass_g = (BASE_G + arm_g * (aw * at - STOCK_ARM[0] * STOCK_ARM[1])
              + EXTRA_G["body"][round(x["body"])] + (EXTRA_G["arm_sleeves"] if sleeves else 0)
              + (EXTRA_G["motor_tilt"] if x["motor_tilt"] else 0))
    weight = mass_g / 1000 * G
    bad = []
    sw, st = STOCK_ARM
    if aw * at ** 3 < 0.8 * sw * st ** 3 or at * aw ** 3 < 0.8 * st * sw ** 3:
        bad.append(f"{aw:.0f} x {at:.1f} mm arms are under 80 % of the stock stiffness")
    if mass_g > AUW_MAX:
        bad.append(f"{mass_g:.0f} g is over {AUW_MAX:.0f} g")

    fm, ct, cp = prop_coeffs()
    d = PROP_D_IN * 0.0254
    area = 4 * math.pi * (d / 2) ** 2
    beta = math.radians(x["motor_tilt"])
    cd_v = 0.8 if sleeves else 1.2
    arm_under = 4 * aw * (d / 2 * 1000)                  # mm^2 of arm under the discs
    dl_hover = DOWNLOAD_K * cd_v * arm_under / (area * 1e6)

    # cruise: the rotor discs tilt by alpha so thrust balances weight and drag;
    # the body sits at alpha - beta
    alpha, items, dl, vi = 0.0, [], dl_hover, 0.0
    for _ in range(12):
        items = body_items(x, alpha - beta)
        drag = K.q(v) * K.total(items)
        alpha = math.atan2(drag, weight)
        t = math.hypot(weight, drag) * (1 + dl)
        vh = math.sqrt(t / (2 * K.RHO * area))
        vi = vh
        for _ in range(60):
            vi = 0.5 * vi + 0.5 * t / (2 * K.RHO * area * math.hypot(v * math.cos(alpha), v * math.sin(alpha) + vi))
        skew = math.atan2(v * math.cos(alpha), vi + v * math.sin(alpha))   # 0 in a hover
        dl = dl_hover * math.cos(skew)                   # a swept-back wake misses most of the arms
    t = math.hypot(weight, drag) * (1 + dl)

    def power(t_total: float, v_air: float, a: float, v_i: float) -> float:
        """Electrical power at the battery for total prop thrust t_total."""
        n = math.sqrt(t_total / 4 / (ct * K.RHO * d ** 4))
        vh = math.sqrt(t_total / (2 * K.RHO * area))
        mu = v_air * math.cos(a) / (math.pi * d * n)
        p_rotor = (KAPPA * t_total * v_i + t_total * v_air * math.sin(a)
                   + t_total * vh * max(1 / fm - KAPPA, 0.0) * (1 + 4.65 * mu * mu))
        q = p_rotor / 4 / (2 * math.pi * n)
        p_motors = p_rotor + 4 * ((q / KM) ** 2 + IRON_10K * (n * 60 / 10000) ** 1.6)
        p_bus = p_motors / ETA_ESC + AVIONICS_W
        v_oc = CELLS * V_CELL
        i = (v_oc - math.sqrt(v_oc ** 2 - 4 * PACK_R * p_bus)) / (2 * PACK_R)
        return v_oc * i

    p_cruise = power(t, v, alpha, vi)
    t_hover = weight * (1 + dl_hover)
    p_hover = power(t_hover, 0.0, 0.0, math.sqrt(t_hover / (2 * K.RHO * area)))
    wh = CELLS * V_CELL * PACK_AH * USABLE

    # full-throttle thrust of the stock motors on the sagging pack (unchanged by the aero)
    v_b, i_b, n = CELLS * V_CELL, 0.0, 0.0
    kt = 60 / (2 * math.pi * KV)
    rm = kt * kt / (KM * KM)
    for _ in range(40):
        v_m = 0.97 * v_b
        lo, hi = 0.0, KV * v_m / 60
        for _ in range(50):
            n = (lo + hi) / 2
            i_m = cp * K.RHO * n * n * d ** 5 / (2 * math.pi) / kt + IRON_10K * (n * 60 / 10000) ** 1.6 / v_m
            lo, hi = (n, hi) if n * 60 / KV + i_m * rm < v_m else (lo, n)
        i_b = 0.5 * i_b + 0.5 * 4 * v_m * i_m / ETA_ESC / v_b
        v_b = CELLS * V_CELL - i_b * PACK_R
    tw = 4 * ct * K.RHO * n * n * d ** 4 / (1 + dl_hover) / weight
    if not TW_MIN <= tw <= TW_MAX:
        bad.append(f"thrust-to-weight {tw:.1f} is outside {TW_MIN:.0f} to {TW_MAX:.0f}")

    metrics = [
        ("Body drag at cruise", f"{drag:.2f} N"),
        ("Flight time at cruise", f"{wh / p_cruise * 60:.1f} min"),
        ("Cruise attitude", f"discs {math.degrees(alpha):.0f}° nose down, body {math.degrees(alpha - beta):+.0f}°"),
        ("Prop wash on the arms", f"{dl_hover * 100:.1f} % of thrust in a hover, {dl * 100:.1f} % at cruise"),
        ("All-up weight", f"{mass_g:.0f} g"),
        ("Hover power / flight time", f"{p_hover:.0f} W / {wh / p_hover * 60:.1f} min"),
        ("Thrust-to-weight", f"{tw:.1f}"),
    ]
    return Result(p_cruise, items, metrics, bad)


def speed_text(v: float) -> str:
    return f"{v * 3.6:.0f} km/h cruise"


def value_text(w: float) -> str:
    return f"{w:.1f} W"


METHOD = f"""
* **Scope.** The airframe's aerodynamics only. The motors, props and battery stay
  stock (2306 1750 KV, 5.1 x 4.3 x 3, 6S 1100 mAh), so the quad keeps its
  ordinary thrust-to-weight, which must stay between {TW_MIN:.0f} and {TW_MAX:.0f}.
  The objective is the power at a steady cruise. There is no speed or
  acceleration objective and no top-speed estimate.
* **Body drag**: each part's frontal area (times the cosine of the body's
  pitch) and its top or bottom face (times the sine), with handbook drag
  coefficients: open stack and camera cage 1.1, canopy 0.5, battery 0.9,
  flat arms 1.2 (0.6 edge-on with sleeves), motor bells 0.8, a rounded full
  fairing 0.25, antennas as rods. The nose-down pitch comes from the drag
  itself: the rotor discs tilt until thrust balances weight and drag, and
  forward-tilted motors take that much off the body's pitch. 10 % more for
  the junctions.
* **Prop wash on the arms**: the part of each arm under the prop pushes down
  against the wash, about {DOWNLOAD_K * 1.2 * 4 * 14 * 64.8 / (4 * math.pi * 64.8 ** 2) * 100:.1f} % of the thrust for
  14 mm flat arms in a hover, less with sleeves. In forward flight the wake
  sweeps back and misses most of it.
* **Power**: the stock props' rotor power in forward flight (momentum theory
  with Glauert's inflow, the work against drag, profile power growing with the
  advance ratio), motor winding and iron losses, a {ETA_ESC:.0%} ESC,
  {AVIONICS_W:.0f} W of avionics and the pack's sag.
* **Arms** must keep at least 80 % of the stock 14 x 5 mm arms' stiffness
  both up and down (width x thickness³) and sideways (thickness x width³).
  Printed parts add their weight: canopy 10 g, full fairing 35 g, arm sleeves
  12 g, motor wedges 4 g.
* These are handbook numbers, not a wind-tunnel test: perhaps ±30 % on the
  drag of each part. The ranking of changes is more reliable than the totals.
"""
