"""A typical 5-inch FPV quad: flight-time model for aero/optimize.py.

The starting point is an ordinary 6S freestyle build: a 5-inch frame,
2306 1750 KV motors, 5.1 x 4.3 tri-blades, a 55 A 4-in-1 ESC, analog video and
a 6S 1100 mAh LiPo. The model works out, for any mix of props, motors and
battery:

  * the weight, from the parts;
  * the body drag at the cruise attitude (battery, stack, arms, motors,
    antennas), with the nose-down tilt that drag itself sets;
  * the rotor power in forward flight (momentum theory with Glauert's inflow,
    plus the props' profile power), then the motor, ESC and battery losses;
  * full-throttle thrust and currents, from a motor model (KV, winding
    resistance, iron loss) and battery sag.

What it rewards is flight time at a relaxed cruise and nothing else. There is
no speed, acceleration or payload objective and no top-speed estimate, and
thrust-to-weight is held inside the band ordinary freestyle quads fly with
(5 to 12), so the optimum stays a normal FPV quad.
"""
from __future__ import annotations

import math

import dragkit as K
from model import Param, Result

NAME = "quad"
TITLE = "Typical 5-inch FPV quad"
OBJECTIVE = "Flight time"
MAXIMIZE = True
BASE_LABEL = "typical build"
AREA_UNIT = ("cm²", 1e4)
SPEED = 40 / 3.6
NOTE = ("Flight time is at a steady cruise, from a full pack down to the usable limit "
        "(80 % for LiPo, 85 % for Li-ion); real freestyle flying, with bursts of full "
        "throttle, gets less.")
ITEMS_NOTE = ("Drag of the body, arms, motors and antennas at the cruise attitude. The props' "
              "own drag is part of the rotor power.")

G = 9.81
TW_MIN, TW_MAX = 5.0, 12.0     # the band ordinary freestyle quads fly with
AUW_MAX = 700.0                # g; a 5-inch frame and props are built for about this much
ESC_A = 55.0                   # per motor, 4-in-1 rating
W_PER_G = 28.0                 # peak motor power per gram of motor
AVIONICS_W = 7.0               # FC, receiver, camera, 400 mW analog VTX
ETA_ESC = 0.95
KAPPA = 1.15                   # induced power over ideal
DOWNWASH = 1.05                # extra thrust to make up for the arms and body under the props
WIRING_OHM = 0.004             # XT60, leads, ESC pads

FRAME_G = 115.0                # 5-inch freestyle frame with 5 mm arms and hardware
FIXED_G = {"FC and 4-in-1 ESC": 20.0, "VTX and antenna": 10.0, "camera": 6.0,
           "receiver": 1.5, "wires, straps, TPU parts": 20.0}
FAIRING_G = 10.0

# name, g, stator volume mm^3 (d^2 x h), motor constant Km (N m / sqrt W), bell diameter, height mm
MOTORS = [
    ("1804", 17.0, 18 * 18 * 4, 0.0098, 23.0, 15.0),
    ("2004", 21.0, 20 * 20 * 4, 0.0113, 25.0, 16.0),
    ("2207", 33.0, 22 * 22 * 7, 0.0200, 27.5, 19.0),
    ("2306", 32.0, 23 * 23 * 6, 0.0193, 28.0, 18.0),
    ("2506", 37.0, 25 * 25 * 6, 0.0220, 30.0, 18.0),
]
IRON_2306 = 4.0                # W at 10 000 rpm, from ~1 A no-load at 10 V for a 1750 KV 2306

# name, pack Wh/kg, fixed g, cell V, usable fraction, cell mOhm x Ah, burst C, pack Wh/cm^3,
# and the length : width : height of the pack (scaled to its volume)
CHEMS = [
    ("LiPo", 140.0, 12.0, 3.7, 0.80, 4.5, 150.0, 0.30, (75.0, 35.0, 31.0)),
    ("Li-ion 21700", 230.0, 20.0, 3.6, 0.85, 67.0, 12.0, 0.42, (72.0, 63.0, 42.0)),
]


def params() -> list[Param]:
    part = "part choice"
    return [
        Param("prop_d", 5.1, 4.5, 5.1, 0.1, "in", "prop diameter",
              "a 5-inch frame takes props up to 5.1 inch", part),
        Param("prop_pitch", 4.3, 3.0, 5.0, 0.1, "in", "prop pitch",
              "5-inch FPV props are sold from about 3 to 5 inch pitch", part),
        Param("blades", 3, 2, 3, 1, "", "blades per prop", "", part, {2: "two", 3: "three"}),
        Param("motor", 3, 0, len(MOTORS) - 1, 1, "", "motor size",
              f"peak power {W_PER_G:.0f} W per gram of motor", part, {i: m[0] for i, m in enumerate(MOTORS)}),
        Param("rpm0", 1750 * 6 * 3.7, 20000, 45000, 500, "rpm", "motor KV x nominal pack voltage",
              "sets the KV for the cell count", part),
        Param("cells", 6, 4, 6, 1, "S", "cells in series", "", part),
        Param("battery_wh", 6 * 3.7 * 1.1, 10, 100, 1, "Wh", "battery energy",
              f"full-throttle current within the pack's C rating; all-up weight at most {AUW_MAX:.0f} g", part),
        Param("chemistry", 0, 0, 1, 1, "", "battery chemistry", "", part,
              {i: c[0] for i, c in enumerate(CHEMS)}),
        Param("fairing", 0, 0, 1, 1, "", "printed canopy over the stack and camera", f"+{FAIRING_G:.0f} g",
              "printed part", {0: "none", 1: "fitted"}),
    ]


def prop_coeffs(d_in: float, pitch_in: float, blades: int):
    """Figure of merit and static thrust / power coefficients (T = CT rho n^2 D^4,
    P = CP rho n^3 D^5, n in rev/s), fitted to typical 5-inch FPV props: lower
    pitch and fewer blades hover more efficiently; more pitch and blades give
    more thrust per rpm. CP follows from FM = CT^1.5 / (CP sqrt(pi / 2))."""
    ratio = pitch_in / d_in
    fm = 0.45 - 0.04 * (blades - 2) - 0.15 * (ratio - 0.6)
    ct = 0.105 * (ratio / 0.8) ** 0.8 * (blades / 2) ** 0.75
    cp = ct ** 1.5 / (math.sqrt(math.pi / 2) * fm)
    return fm, ct, cp


def battery(x: dict):
    chem = CHEMS[round(x["chemistry"])]
    _, wh_kg, fixed, v_cell, usable, mohm_ah, c_max, wh_cm3, shape = chem
    cells = round(x["cells"])
    wh = x["battery_wh"]
    ah = wh / (cells * v_cell)
    return {"name": chem[0], "cells": cells, "ah": ah, "wh": wh, "v_oc": cells * v_cell,
            "r": cells * mohm_ah / 1000 / ah + WIRING_OHM, "g": wh / wh_kg * 1000 + fixed,
            "usable": usable, "i_max": c_max * ah, "c_max": c_max,
            "dims": _scaled(shape, wh / wh_cm3 * 1000)}


def _scaled(shape, volume_mm3: float):
    """Pack dimensions with the given proportions and volume."""
    k = (volume_mm3 / (shape[0] * shape[1] * shape[2])) ** (1 / 3)
    return tuple(k * a for a in shape)


def body_items(x: dict, bat: dict, motor, alpha: float) -> list[K.Item]:
    """Body drag areas (m^2) with the quad pitched nose-down by alpha."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    l, w, h = bat["dims"]
    mm2 = 1e-6
    cd_front = 0.55 if x["fairing"] else 1.1
    _, _, _, _, bell, mh = motor
    items = [
        K.Item("battery", 0.9 * (w * h * ca + l * w * sa) * mm2, "body", f"{l:.0f} x {w:.0f} x {h:.0f} mm on top"),
        K.Item("stack, camera, body plates", (cd_front * 30 * 30 * ca + 1.1 * max(40 * 90 - l * w, 0) * sa) * mm2,
               "body", "canopy fitted" if x["fairing"] else "open stack"),
        K.Item("arms", 1.2 * (4 * 53 * 5 * ca + 4 * 75 * 14 * sa) * mm2, "body", "5 mm carbon, X layout"),
        K.Item("motors", 0.8 * 4 * (bell * mh * ca + math.pi * (bell / 2) ** 2 * sa) * mm2, "body",
               f"{bell:.0f} mm bells"),
        K.Item("antennas", 1.1 * (5 * 60 + 2 * 1.5 * 50) * mm2, "protuberances", "VTX and receiver"),
    ]
    items.append(K.Item("junctions (10 %)", 0.10 * K.total(items), "interference", ""))
    return items


def rotor_power(t: float, v: float, alpha: float, area: float, fm: float, tip: float) -> float:
    """Shaft power (W) for total thrust t at speed v with the rotor discs tilted
    by alpha: induced (Glauert), the work against drag, and profile power grown
    with the advance ratio."""
    vh = math.sqrt(t / (2 * K.RHO * area))
    vi = vh
    for _ in range(60):
        new = t / (2 * K.RHO * area * math.hypot(v * math.cos(alpha), v * math.sin(alpha) + vi))
        vi = 0.5 * vi + 0.5 * new
    profile0 = t * vh * max(1 / fm - KAPPA, 0.0)
    mu = v * math.cos(alpha) / tip
    return KAPPA * t * vi + t * v * math.sin(alpha) + profile0 * (1 + 4.65 * mu * mu)


def evaluate(x: dict, v: float) -> Result:
    motor = MOTORS[round(x["motor"])]
    m_name, m_g, vol, km, _, _ = motor
    blades = round(x["blades"])
    d = x["prop_d"] * 0.0254
    fm, ct, cp = prop_coeffs(x["prop_d"], x["prop_pitch"], blades)
    bat = battery(x)
    prop_g = 4.3 * (blades / 3) ** 0.65 * (x["prop_d"] / 5.1) ** 2.5
    mass_g = (FRAME_G + sum(FIXED_G.values()) + 4 * m_g + 4 * prop_g + bat["g"]
              + (FAIRING_G if x["fairing"] else 0))
    weight = mass_g / 1000 * G
    area = 4 * math.pi * (d / 2) ** 2
    iron10k = IRON_2306 * (vol / (23 * 23 * 6)) ** 0.9
    kv = x["rpm0"] / bat["v_oc"]
    kt = 60 / (2 * math.pi * kv)                  # N m / A
    rm = kt * kt / (km * km)                      # winding resistance, ohm

    def motors_power(thrust_total: float, v_air: float, alpha: float) -> float:
        """Electrical power (W) into the four motors for a total prop thrust."""
        t_m = thrust_total / 4
        n = math.sqrt(t_m / (ct * K.RHO * d ** 4))        # rev/s
        tip = math.pi * d * n
        p_shaft = rotor_power(thrust_total, v_air, alpha, area, fm, tip) / 4
        q = p_shaft / (2 * math.pi * n)
        return 4 * (p_shaft + (q / km) ** 2 + iron10k * (n * 60 / 10000) ** 1.6)

    def battery_draw(p_bus: float):
        """Current and chemical power for a bus power, with the pack's sag."""
        disc = bat["v_oc"] ** 2 - 4 * bat["r"] * p_bus
        if disc <= 0:
            return math.inf, math.inf
        i = (bat["v_oc"] - math.sqrt(disc)) / (2 * bat["r"])
        return i, bat["v_oc"] * i

    # cruise: the tilt balances drag against weight
    alpha, items = 0.0, []
    for _ in range(8):
        items = body_items(x, bat, motor, alpha)
        drag = K.q(v) * K.total(items)
        alpha = math.atan2(drag, weight)
    thrust = math.hypot(weight, drag) * DOWNWASH
    p_cruise = motors_power(thrust, v, alpha) / ETA_ESC + AVIONICS_W
    _, p_chem = battery_draw(p_cruise)
    minutes = bat["wh"] * bat["usable"] / p_chem * 60
    p_hover = motors_power(weight * DOWNWASH, 0.0, 0.0) / ETA_ESC + AVIONICS_W
    hover_min = bat["wh"] * bat["usable"] / battery_draw(p_hover)[1] * 60

    # full throttle: motor rpm limited by back-EMF and winding drop, pack sag
    v_b = bat["v_oc"]
    i_m = i_b = 0.0
    n = 0.0
    for _ in range(40):
        v_m = 0.97 * v_b
        lo, hi = 0.0, kv * v_m / 60                   # rev/s
        for _ in range(50):
            n = (lo + hi) / 2
            q = cp * K.RHO * n * n * d ** 5 / (2 * math.pi)
            i_m = q / kt + iron10k * (n * 60 / 10000) ** 1.6 / max(v_m, 1e-6)
            if n * 60 / kv + i_m * rm < v_m:
                lo = n
            else:
                hi = n
        i_b_new = 4 * v_m * i_m / ETA_ESC / v_b
        i_b = 0.5 * i_b + 0.5 * i_b_new
        v_b = max(bat["v_oc"] - i_b * bat["r"], 0.3 * bat["v_oc"])
    thrust_max = 4 * ct * K.RHO * n * n * d ** 4 / DOWNWASH
    tw = thrust_max / weight
    p_motor_max = v_m * i_m

    bad = []
    if mass_g > AUW_MAX:
        bad.append(f"{mass_g:.0f} g is over the {AUW_MAX:.0f} g a 5-inch frame is built for")
    if tw < TW_MIN:
        bad.append(f"thrust-to-weight {tw:.1f} is under {TW_MIN:.0f}")
    if tw > TW_MAX:
        bad.append(f"thrust-to-weight {tw:.1f} is over {TW_MAX:.0f}, past ordinary freestyle quads")
    if i_m > ESC_A:
        bad.append(f"{i_m:.0f} A per motor is over the ESC's {ESC_A:.0f} A")
    if i_b > bat["i_max"]:
        bad.append(f"{i_b:.0f} A is over the pack's {bat['i_max']:.0f} A ({bat['c_max']:.0f}C)")
    if p_motor_max > W_PER_G * m_g:
        bad.append(f"{p_motor_max:.0f} W per motor is over the {m_name}'s {W_PER_G * m_g:.0f} W")

    metrics = [
        ("All-up weight", f"{mass_g:.0f} g"),
        ("Thrust-to-weight", f"{tw:.1f}"),
        ("Battery", f"{bat['cells']}S {bat['ah'] * 1000:.0f} mAh {bat['name']}, {bat['g']:.0f} g"),
        ("Motors", f"{m_name} {kv:.0f} KV"),
        ("Props", f"{x['prop_d']:.1f} x {x['prop_pitch']:.1f} x {blades}"),
        ("Power in cruise", f"{p_cruise:.0f} W ({mass_g / p_cruise:.1f} g/W)"),
        ("Cruise attitude", f"{math.degrees(alpha):.0f}° nose down"),
        ("Body drag in cruise", f"{drag:.2f} N"),
        ("Hover flight time", f"{hover_min:.1f} min"),
        ("Full-throttle current", f"{i_b:.0f} A ({i_b / bat['ah']:.0f}C), {i_m:.0f} A per motor"),
    ]
    return Result(minutes, items, metrics, bad)


def speed_text(v: float) -> str:
    return f"{v * 3.6:.0f} km/h cruise"


def value_text(minutes: float) -> str:
    return f"{minutes:.1f} min"


METHOD = f"""
* **Scope.** A typical 5-inch FPV quad, optimised for flight time at a relaxed
  cruise. The model has no speed, acceleration or payload objective and does
  not estimate top speed. Thrust-to-weight must stay between {TW_MIN:.0f} and
  {TW_MAX:.0f}, the band ordinary freestyle quads fly with: enough to fly and
  recover normally, not a racer's or an interceptor's. The all-up weight stays
  at or under {AUW_MAX:.0f} g, what a 5-inch frame and props are built for;
  heavier long-range builds belong on 6- or 7-inch frames. Thrust-to-weight
  here is at mid-pack voltage with the pack sagging, which reads about a
  quarter under thrust-stand figures.
* **Weight** is summed from the parts: frame {FRAME_G:.0f} g, electronics and
  hardware {sum(FIXED_G.values()):.0f} g, four motors, four props and the
  battery (LiPo at 140 Wh/kg, Li-ion 21700 packs at 230 Wh/kg, plus leads).
* **Props**: a figure of merit and thrust and power coefficients fitted to
  typical 5-inch FPV props. Lower pitch and two blades hover more efficiently;
  more pitch and three blades give more thrust per rpm.
* **Cruise**: body drag from the parts' frontal and plan areas at the tilt the
  drag itself sets; rotor power from momentum theory with Glauert's inflow,
  the work against drag, and profile power that grows with the advance ratio.
  {DOWNWASH - 1:.0%} extra thrust covers the arms and body under the props.
* **Motors**: winding loss from the motor constant Km (the same torque costs
  the same copper loss at any KV), iron loss rising with rpm (about
  {IRON_2306:.0f} W at 10 000 rpm for a 2306), a {ETA_ESC:.0%} ESC and
  {AVIONICS_W:.0f} W for the FC, receiver, camera and VTX.
* **Full throttle**: rpm where back-EMF plus the winding drop meets the ESC's
  output voltage, with the pack sagging under the current. The limits: {ESC_A:.0f} A
  per motor, the pack's burst C rating (LiPo 150C, Li-ion 12C) and about
  {W_PER_G:.0f} W per gram of motor.
* These are first-pass fits, not measurements. Expect ±15 % on flight times;
  the ranking of changes is more reliable. Check a real combination on a
  thrust stand or in eCalc before buying.
"""
