"""Typical FPV quads: aerodynamic model for aero/optimize.py.

A quad is an ordinary build (a `Build` below), and its propulsion stays stock:
motors, props, ESC and battery. What the optimizer changes is the airframe's
aerodynamics:

  * the body: open frame, a canopy over the stack and camera, or a full fairing
    over the stack and battery;
  * where the battery sits, on top or underneath;
  * the arms' width and thickness, held to the stock arms' stiffness, and
    printed streamlined sleeves over them;
  * forward-tilted motor mounts, so the body flies closer to level at cruise;
  * the VTX antenna.

Every one of these shows up in the power needed for a steady cruise, which is
the objective: the body's drag at the attitude that drag sets, the prop wash
pushing down on the arms, and the weight of any printed part. The model has no
speed or acceleration objective and no top-speed estimate, and the propulsion
is not touched, so the quad stays an ordinary FPV quad of its class.

This module is the 5-inch freestyle quad; quad7.py is a 7-inch long-range one.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import dragkit as K
from model import Param, Result

G = 9.81
ETA_ESC = 0.95
KAPPA = 1.15                   # induced power over ideal
CARBON = 1.6e-3                # g/mm^3
DOWNLOAD_K = 0.545             # prop wash on the arms: 4.5 % of thrust for 14 mm flat arms under 5-inch props
HEADROOM = 0.70                # cruise may use at most this share of full thrust
ANTENNAS = ("upright lollipop", "lollipop laid back 45°", "stubby")


@dataclass(frozen=True)
class Build:
    """An ordinary quad as its owner would build it. Lengths mm, masses g."""
    name: str
    title: str
    speed_kmh: float
    parts: str                             # one line for the report
    mass: float                            # all-up, as built
    tw: tuple                              # thrust-to-weight band of the class
    auw_max: float
    # propulsion (stock, not optimized)
    prop: tuple                            # diameter, pitch (inch), blades
    kv: float
    km: float                              # motor constant, N m / sqrt W
    iron_10k: float                        # motor iron loss at 10 000 rpm, W
    bell: tuple                            # bell diameter, height
    cells: int
    pack_ah: float
    pack_r: float                          # pack plus wiring, ohm
    avionics_w: float
    # airframe
    battery: tuple                         # length, width, height
    stack: tuple                           # front of the stack and camera cage: width, height
    plate: float                           # body plates, plan area mm^2
    canopy: float                          # plan area a canopy covers
    arm: tuple                             # stock width, thickness
    arm_len: float                         # centre to motor
    arm_exposed: float                     # beyond the body
    arm_widths: tuple                      # range the optimizer may try
    extras_g: dict                         # printed parts: canopy, fairing, sleeves, wedges
    other_cda: float = 0.0                 # fixed extras (GPS mast), mm^2
    other_note: str = ""
    v_cell: float = 3.7
    usable: float = 0.80

    @property
    def fairing(self):
        l, w, h = self.battery
        return (l + 35.0, max(w, self.stack[0]) + 7.0, self.stack[1] + h + 4.0)


FIVE_INCH = Build(
    name="quad", title="Typical 5-inch FPV quad, aerodynamics", speed_kmh=60,
    parts="5-inch freestyle frame, 2306 1750 KV, 5.1 x 4.3 x 3 props, 55 A 4-in-1, 6S 1100 mAh LiPo, analog video",
    mass=504.0, tw=(5.0, 12.0), auw_max=700.0,
    prop=(5.1, 4.3, 3), kv=1750.0, km=0.0193, iron_10k=4.0, bell=(28.0, 18.0),
    cells=6, pack_ah=1.1, pack_r=0.0285, avionics_w=7.0,
    battery=(75.0, 35.0, 31.0), stack=(30.0, 30.0), plate=40.0 * 90.0, canopy=40.0 * 40.0,
    arm=(14.0, 5.0), arm_len=100.0, arm_exposed=75.0, arm_widths=(10, 18),
    extras_g={"canopy": 10.0, "fairing": 35.0, "sleeves": 12.0, "wedges": 4.0},
)


class QuadModel:
    OBJECTIVE = "Power at cruise"
    MAXIMIZE = False
    BASE_LABEL = "typical build"
    AREA_UNIT = ("cm²", 1e4)
    MIN_GAIN = 0.002           # changes worth less than 0.2 % are below the model's resolution
    ITEMS_NOTE = ("Drag of the body, arms, motors and antennas at the cruise attitude. The props' "
                  "own drag is part of the rotor power.")

    def __init__(self, b: Build):
        self.b = b
        self.NAME, self.TITLE = b.name, b.title
        self.SPEED = b.speed_kmh / 3.6
        self.NOTE = (f"The stock quad: {b.parts}, {b.mass:.0f} g. Every aerodynamic change ends up in "
                     "this power: the body's drag, the prop wash on the arms and the weight of any "
                     "printed part. The propulsion stays stock, and the model does not look at top speed.")
        d, pitch, blades = b.prop
        ratio = pitch / d
        self.fm = 0.45 - 0.04 * (blades - 2) - 0.15 * (ratio - 0.6)
        self.ct = 0.105 * (ratio / 0.8) ** 0.8 * (blades / 2) ** 0.75
        self.cp = self.ct ** 1.5 / (math.sqrt(math.pi / 2) * self.fm)
        self.d = d * 0.0254
        self.area = 4 * math.pi * (self.d / 2) ** 2
        self.arm_span = b.arm_exposed * math.sin(math.radians(45))
        self.METHOD = self._method()

    # ------------------------------------------------------------------ parameters
    def params(self) -> list[Param]:
        b, e = self.b, self.b.extras_g
        return [
            Param("body", 0, 0, 2, 1, "", "body",
                  f"canopy +{e['canopy']:.0f} g; full fairing +{e['fairing']:.0f} g, and it has to open for "
                  "battery changes", "printed part",
                  {0: "open frame", 1: "canopy over stack and camera", 2: "full fairing over stack and battery"}),
            Param("battery_mount", 0, 0, 1, 1, "", "battery position", "", "build choice",
                  {0: "on top", 1: "underneath"}),
            Param("arm_width", b.arm[0], b.arm_widths[0], b.arm_widths[1], 1, "mm", "arm width",
                  f"arm stiffness up and down and sideways at least 80 % of the stock "
                  f"{b.arm[0]:.0f} x {b.arm[1]:.0f} mm arms", "frame choice"),
            Param("arm_thickness", b.arm[1], b.arm[1] - 1, b.arm[1] + 1, 0.5, "mm", "arm thickness",
                  "as for the width", "frame choice"),
            Param("arm_sleeves", 0, 0, 1, 1, "", "streamlined printed sleeves on the arms",
                  f"+{e['sleeves']:.0f} g", "printed part", {0: "none", 1: "fitted"}),
            Param("motor_tilt", 0, 0, 15, 1, "°", "motors tilted forward on printed wedges",
                  "the camera's uptilt drops by the same angle; the quad hovers nose-up by it, "
                  "and 15° keeps that sane", "printed part"),
            Param("antenna", 0, 0, 2, 1, "", "VTX antenna", "", "part choice", dict(enumerate(ANTENNAS))),
        ]

    # ------------------------------------------------------------------ drag
    def body_items(self, x: dict, theta: float) -> list[K.Item]:
        """Drag areas (m^2) with the body pitched nose-down by theta (negative: nose-up)."""
        b = self.b
        c, s = math.cos(theta), abs(math.sin(theta))
        nose_down = theta >= 0
        body, under = round(x["body"]), round(x["battery_mount"]) == 1
        sleeves = round(x["arm_sleeves"]) == 1
        l, w, h = b.battery
        mm2 = 1e-6
        items = []
        if body == 2:
            fl, fw, fh = b.fairing
            items.append(K.Item("body fairing (stack and battery)", (0.25 * fw * fh * c + 0.3 * fl * fw * s) * mm2,
                                "body", f"rounded pod {fl:.0f} x {fw:.0f} x {fh:.0f} mm"))
        else:
            canopy = body == 1
            sw, sh = b.stack
            items.append(K.Item("stack and camera, front", (0.5 if canopy else 1.1) * sw * sh * c * mm2, "body",
                                "under a canopy" if canopy else "open standoffs and camera cage"))
            items.append(K.Item("battery, front", 0.9 * w * h * c * mm2, "body",
                                f"{w:.0f} x {h:.0f} mm, {'underneath' if under else 'on top'}"))
            cover = b.canopy if canopy else 0.0
            if nose_down == (not under):             # the battery is on the side the flow reaches
                windward = 0.9 * l * w + 1.1 * max(b.plate - l * w - cover, 0.0) + 0.4 * cover
            else:                                    # the plate faces the flow, the battery is in its lee
                windward = 1.1 * max(b.plate - cover, 0.0) + 0.4 * cover
            items.append(K.Item("body, top or bottom face", (windward + 0.2 * b.plate) * s * mm2, "body",
                                "flow reaches it because the body is tilted"))
        aw, at = x["arm_width"], x["arm_thickness"]
        items.append(K.Item("arms", ((0.6 if sleeves else 1.2) * 4 * self.arm_span * at * c
                                     + (0.9 if sleeves else 1.2) * 4 * b.arm_exposed * aw * s) * mm2, "body",
                            f"{aw:.0f} x {at:.1f} mm" + (", sleeved" if sleeves else "")))
        bell, bh = b.bell
        items.append(K.Item("motors", 0.8 * 4 * (bell * bh * c + math.pi * (bell / 2) ** 2 * s) * mm2, "body",
                            f"{bell:.0f} mm bells"))
        ant = [1.1 * 5 * 60 + 0.5 * 254, 1.1 * 5 * 60 * math.cos(math.radians(45)) ** 3 + 0.5 * 254,
               1.1 * 8 * 25][round(x["antenna"])]
        items.append(K.Item("antennas", (ant + 1.1 * 2 * 1.5 * 50) * mm2, "protuberances",
                            ANTENNAS[round(x["antenna"])] + " and receiver"))
        if b.other_cda:
            items.append(K.Item(b.other_note, b.other_cda * mm2, "protuberances", ""))
        items.append(K.Item("junctions (10 %)", 0.10 * K.total(items), "interference", ""))
        return items

    # ------------------------------------------------------------------ power
    def power(self, t_total: float, v_air: float, a: float, v_i: float) -> float:
        """Chemical power drawn from the pack for total prop thrust t_total."""
        b, d = self.b, self.d
        n = math.sqrt(t_total / 4 / (self.ct * K.RHO * d ** 4))
        vh = math.sqrt(t_total / (2 * K.RHO * self.area))
        mu = v_air * math.cos(a) / (math.pi * d * n)
        p_rotor = (KAPPA * t_total * v_i + t_total * v_air * math.sin(a)
                   + t_total * vh * max(1 / self.fm - KAPPA, 0.0) * (1 + 4.65 * mu * mu))
        q = p_rotor / 4 / (2 * math.pi * n)
        p_motors = p_rotor + 4 * ((q / b.km) ** 2 + b.iron_10k * (n * 60 / 10000) ** 1.6)
        p_bus = p_motors / ETA_ESC + b.avionics_w
        v_oc = b.cells * b.v_cell
        disc = v_oc ** 2 - 4 * b.pack_r * p_bus
        if disc <= 0:
            return math.inf
        return v_oc * (v_oc - math.sqrt(disc)) / (2 * b.pack_r)

    def full_thrust(self) -> float:
        """Full-throttle thrust of the stock motors on the sagging pack at mid-charge, N."""
        b, d = self.b, self.d
        kt = 60 / (2 * math.pi * b.kv)
        rm = kt * kt / (b.km * b.km)
        v_b, i_b, n = b.cells * b.v_cell, 0.0, 0.0
        for _ in range(40):
            v_m = 0.97 * v_b
            lo, hi = 0.0, b.kv * v_m / 60
            for _ in range(50):
                n = (lo + hi) / 2
                i_m = (self.cp * K.RHO * n * n * d ** 5 / (2 * math.pi) / kt
                       + b.iron_10k * (n * 60 / 10000) ** 1.6 / v_m)
                lo, hi = (n, hi) if n * 60 / b.kv + i_m * rm < v_m else (lo, n)
            i_b = 0.5 * i_b + 0.5 * 4 * v_m * i_m / ETA_ESC / v_b
            v_b = b.cells * b.v_cell - i_b * b.pack_r
        return 4 * self.ct * K.RHO * n * n * d ** 4

    def evaluate(self, x: dict, v: float) -> Result:
        b, e = self.b, self.b.extras_g
        aw, at = x["arm_width"], x["arm_thickness"]
        sleeves = round(x["arm_sleeves"]) == 1
        mass_g = (b.mass + 4 * b.arm_len * CARBON * (aw * at - b.arm[0] * b.arm[1])
                  + (0.0, e["canopy"], e["fairing"])[round(x["body"])]
                  + (e["sleeves"] if sleeves else 0.0) + (e["wedges"] if x["motor_tilt"] else 0.0))
        weight = mass_g / 1000 * G
        bad = []
        sw, st = b.arm
        if aw * at ** 3 < 0.8 * sw * st ** 3 or at * aw ** 3 < 0.8 * st * sw ** 3:
            bad.append(f"{aw:.0f} x {at:.1f} mm arms are under 80 % of the stock stiffness")
        if mass_g > b.auw_max:
            bad.append(f"{mass_g:.0f} g is over {b.auw_max:.0f} g")

        beta = math.radians(x["motor_tilt"])
        cd_v = 0.8 if sleeves else 1.2
        r_mm = self.d / 2 * 1000
        dl_hover = DOWNLOAD_K * cd_v * 4 * aw * r_mm / (self.area * 1e6)

        # cruise: the discs tilt by alpha so thrust balances weight and drag; the body sits at alpha - beta
        alpha, items, dl, vi, drag = 0.0, [], dl_hover, 0.0, 0.0
        for _ in range(12):
            items = self.body_items(x, alpha - beta)
            drag = K.q(v) * K.total(items)
            alpha = math.atan2(drag, weight)
            t = math.hypot(weight, drag) * (1 + dl)
            vi = math.sqrt(t / (2 * K.RHO * self.area))
            for _ in range(60):
                vi = 0.5 * vi + 0.5 * t / (2 * K.RHO * self.area
                                           * math.hypot(v * math.cos(alpha), v * math.sin(alpha) + vi))
            skew = math.atan2(v * math.cos(alpha), vi + v * math.sin(alpha))   # 0 in a hover
            dl = dl_hover * math.cos(skew)                                     # a swept-back wake misses the arms
        t = math.hypot(weight, drag) * (1 + dl)
        p_cruise = self.power(t, v, alpha, vi)
        t_hover = weight * (1 + dl_hover)
        p_hover = self.power(t_hover, 0.0, 0.0, math.sqrt(t_hover / (2 * K.RHO * self.area)))
        wh = b.cells * b.v_cell * b.pack_ah * b.usable

        t_full = self.full_thrust()
        tw = t_full / (1 + dl_hover) / weight
        if not b.tw[0] <= tw <= b.tw[1]:
            bad.append(f"thrust-to-weight {tw:.1f} is outside {b.tw[0]:.0f} to {b.tw[1]:.0f}")
        if t > HEADROOM * t_full:
            bad.append(f"cruise needs {t / t_full:.0%} of full thrust, over {HEADROOM:.0%}")

        metrics = [
            ("Body drag at cruise", f"{drag:.2f} N"),
            ("Flight time at cruise", f"{wh / p_cruise * 60:.1f} min"),
            ("Cruise attitude", f"discs {math.degrees(alpha):.0f}° nose down, body {math.degrees(alpha - beta):+.0f}°"),
            ("Thrust used at cruise", f"{t / t_full:.0%} of full thrust"),
            ("Prop wash on the arms", f"{dl_hover * 100:.1f} % of thrust in a hover, {dl * 100:.1f} % at cruise"),
            ("All-up weight", f"{mass_g:.0f} g"),
            ("Hover power / flight time", f"{p_hover:.0f} W / {wh / p_hover * 60:.1f} min"),
            ("Thrust-to-weight", f"{tw:.1f}"),
        ]
        return Result(p_cruise, items, metrics, bad)

    def speed_text(self, v: float) -> str:
        return f"{v * 3.6:.0f} km/h cruise"

    def value_text(self, w: float) -> str:
        return f"{w:.1f} W"

    def _method(self) -> str:
        b, e = self.b, self.b.extras_g
        dl = DOWNLOAD_K * 1.2 * 4 * b.arm[0] * (self.d / 2 * 1000) / (self.area * 1e6)
        return f"""
* **Scope.** The airframe's aerodynamics only. The motors, props and battery stay
  stock ({b.parts}), so the quad keeps its thrust-to-weight, which must stay
  between {b.tw[0]:.0f} and {b.tw[1]:.0f} (the band this class flies with), and
  a cruise may use at most {HEADROOM:.0%} of full thrust. The objective is the
  power at a steady cruise. There is no speed or acceleration objective and no
  top-speed estimate.
* **Body drag**: each part's frontal area (times the cosine of the body's
  pitch) and its top or bottom face (times the sine), with handbook drag
  coefficients: open stack and camera cage 1.1, canopy 0.5, battery 0.9,
  flat arms 1.2 (0.6 edge-on with sleeves), motor bells 0.8, a rounded full
  fairing 0.25, antennas as rods. The nose-down pitch comes from the drag
  itself: the rotor discs tilt until thrust balances weight and drag, and
  forward-tilted motors take that much off the body's pitch. 10 % more for
  the junctions.
* **Prop wash on the arms**: the part of each arm under the prop pushes down
  against the wash, about {dl * 100:.1f} % of the thrust for the stock
  {b.arm[0]:.0f} mm flat arms in a hover, less with sleeves. In forward flight
  the wake sweeps back and misses most of it.
* **Power**: the stock props' rotor power in forward flight (momentum theory
  with Glauert's inflow, the work against drag, profile power growing with the
  advance ratio), motor winding and iron losses, a {ETA_ESC:.0%} ESC,
  {b.avionics_w:.0f} W of avionics and the pack's sag.
* **Arms** must keep at least 80 % of the stock {b.arm[0]:.0f} x {b.arm[1]:.0f} mm
  arms' stiffness both up and down (width x thickness³) and sideways
  (thickness x width³). Printed parts add their weight: canopy
  {e['canopy']:.0f} g, full fairing {e['fairing']:.0f} g, arm sleeves
  {e['sleeves']:.0f} g, motor wedges {e['wedges']:.0f} g.
* These are handbook numbers, not a wind-tunnel test: perhaps ±30 % on the
  drag of each part. The ranking of changes is more reliable than the totals.
"""


MODEL = QuadModel(FIVE_INCH)
