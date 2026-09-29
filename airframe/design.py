#!/usr/bin/env python3
"""Twin-tube micro FPV plane: parameters, layout and first-order sizing.

Pure Python (no CAD dependency) so the numbers can be checked quickly:

    python3 airframe/design.py

prints the span sweep that picks the smallest workable wing, and the layout
of the chosen design. build.py turns the same parameters into printable parts.

Coordinates are millimetres: x runs aft from the motor mounting face, y towards
the right wing tip, z up. The tube centre-line is z = 0.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

RHO_AIR = 1.225   # kg/m^3, sea level
NU_AIR = 1.46e-5  # m^2/s, 15 C
G = 9.81

# Material densities, g/cm^3 (= mg/mm^3)
PLA = 1.24
LW_PLA = 0.75     # ColorFabb LW-PLA, foamed (~40 % lighter than PLA)
CARBON = 1.55


@dataclass(frozen=True)
class Params:
    # Wing: straight, rectangular, like the Molniya's
    span: float = 465.0
    aspect_ratio: float = 5.0
    naca: str = "4412"             # Clark-Y-like, forgiving at Re 50-80k
    incidence: float = 2.0         # deg, wing chord vs tube line
    te_min: float = 0.8            # printable trailing-edge thickness
    center_width: float = 56.0     # minimum printed centre section across the pod
    wing_gap: float = 0.6          # wing lower surface above the pod top
    main_spar_d: float = 3.0       # carbon rod
    main_spar_pos: float = 0.25    # fraction of chord
    rear_spar_d: float = 2.0       # carbon rod
    rear_spar_pos: float = 0.70    # aft enough for SG90 tabs between the spars
    hinge_pos: float = 0.75        # aileron hinge, fraction of chord
    aileron_root_gap: float = 10.0
    aileron_tip_gap: float = 6.0
    skin: float = 0.45             # printed wing shell (1 perimeter, 0 % infill)
    cl_max: float = 0.95           # 3D wing, Re 50-70k

    # Tail: flat plates, H-tail with a fin on each stabiliser tip
    tail_arm: float = 2.8          # wing c/4 to stab c/4, in wing chords
    vh: float = 0.50               # horizontal tail volume coefficient
    vv: float = 0.045              # vertical tail volume (both fins)
    stab_chord_ratio: float = 0.45
    elevator_frac: float = 0.36
    plate: float = 2.0             # tail plate thickness
    fin_below: float = 6.0         # fin depth below the stabiliser
    rudders: bool = True           # rudder on each fin, hinged on the elevator hinge line
    rudder_horn: float = 8.0       # rudder horn hole behind the hinge (below the rudder)
    rudder_servo_hole: float = 9.0 # inner hole on the rudder servo's trimmed horn

    # The two main struts: plain round tubes
    tube_od: float = 6.0
    tube_id: float = 5.0
    tube_density: float = CARBON   # 6x0.5 aluminium tube: 2.7
    tube_spacing: float = 40.0     # centre to centre = pod width
    clearance: float = 0.2         # hole oversize for glued fits

    # Pod (between the tubes, like the Molniya's payload position)
    wall: float = 0.8
    boss_wall: float = 1.3         # printed sleeve round each tube
    sleeve_len: float = 14.0       # sleeves at the pod front and rear
    front_wall: float = 2.4        # carries the motor
    nose_r: float = 8.0            # radius on the pod's front edges (0 = sharp box)
    pod_depth: float = 18.0        # tube centre-line to pod bottom
    pod_overlap: float = 0.70      # pod runs this far under the wing (chords)
    front_bay: float = 53.5        # camera, VTX, FC; battery behind
    esc_bay: float = 8.4           # ESC card in its two ribs, between the battery and the servo bay
    batt_trim: float = 16.0        # battery travel for balancing
    servo_bay: float = 36.0        # elevator servo lies behind the battery
    motor_z: float = 9.0           # thrust line above the tube centre-line

    cg_target: float = 0.28        # fraction of chord, first flights
    bed: tuple = (180.0, 180.0, 180.0)   # printer build volume: Bambu Lab A1 mini
    bed_margin: float = 5.0

    @property
    def centre_w(self) -> float:
        """Centre section width: wide enough that a wing panel standing on its
        root rib fits the printer's build height."""
        return max(self.center_width, self.span - 2 * (self.bed[2] - self.bed_margin))
    stall_limit: float = 9.0       # m/s, "as small as possible" criterion


@dataclass(frozen=True)
class Servo:
    """Tower Pro SG90 micro servo, datasheet dimensions (mm, g)."""
    name: str = "SG90"
    mass: float = 9.0
    length: float = 22.8           # body, along the mounting tabs
    width: float = 12.2
    height: float = 22.7           # base to top of the case
    tab_span: float = 32.3
    tab_t: float = 2.5
    tab_z: float = 15.9            # base to the underside of the tabs
    shaft_from_end: float = 5.9
    boss_d: float = 11.8
    boss_h: float = 4.0
    spline_d: float = 4.8
    spline_h: float = 3.2
    horn_len: float = 16.0         # single-arm horn
    horn_t: float = 1.5
    horn_hole: float = 11.5        # pushrod hole radius on the horn

    @property
    def shaft_x(self):             # shaft offset from the body centre, along length
        return self.length / 2 - self.shaft_from_end

    @property
    def horn_z(self):              # horn arm mid-plane above the base
        return self.height + self.boss_h + self.spline_h - self.horn_t / 2


@dataclass(frozen=True)
class Kit:
    """Reference electronics (analog FPV, INAV-capable). Grams / mm."""
    motor: float = 9.0             # 1404 3800KV class, 2S
    prop: float = 2.0              # 4x2.5 two-blade
    esc: float = 6.0               # Hobbywing XRotor Micro 30A (BLHeli_S, 2-4S, no BEC)
    fc: float = 10.0               # Matek F405-WMN wing FC (12 outputs, 5 A servo BEC)
    rx: float = 1.5                # ELRS nano
    cam_vtx: float = 6.0           # nano camera + AIO 25-200 mW VTX
    antenna: float = 1.5
    servo: Servo = Servo()         # x4: two ailerons, elevator, rudder
    battery: float = 27.0          # 2S 450 mAh LiPo
    batt_len: float = 58.0
    batt_w: float = 31.0
    batt_h: float = 13.0
    batt_wh: float = 3.33          # 450 mAh * 7.4 V
    wiring: float = 6.0            # the ESC sits behind the battery: long motor and signal runs
    # outer dimensions from retailer listings (the maker's drawings are not reachable)
    fc_name: str = "Matek F405-WMN"
    fc_len: float = 31.0
    fc_wid: float = 26.0
    fc_hgt: float = 16.5           # vendor-quoted clearance height
    fc_holes: float = 22.0         # square hole pattern, 2 mm holes
    esc_name: str = "Hobbywing XRotor Micro 30A"
    esc_len: float = 23.8
    esc_wid: float = 14.5
    esc_thk: float = 5.8
    hardware: float = 2.0          # screws, hinge tape, velcro (horns, rods modelled)
    # propulsion, for the top-speed estimate
    motor_kv: float = 3800.0
    prop_d_in: float = 4.0
    prop_pitch_in: float = 2.5
    v_loaded: float = 7.0          # 2S under load
    rpm_frac: float = 0.85         # motor rpm at top speed / (kv * v_loaded)


def round_to(v: float, step: float) -> float:
    return step * round(v / step)


# --------------------------------------------------------------------------
# Airfoil


def naca4(code: str, n: int = 60, te_frac: float = 0.0):
    """Upper and lower surfaces of a NACA 4-digit airfoil, LE -> TE, chord 1.

    te_frac thickens the section linearly so the trailing edge is te_frac
    thick instead of a knife edge that can't be printed.
    """
    m, p, t = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    upper, lower = [], []
    for i in range(n):
        x = 0.5 * (1 - math.cos(math.pi * i / (n - 1)))
        yt = 5 * t * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2
                      + 0.2843 * x ** 3 - 0.1036 * x ** 4) + 0.5 * te_frac * x
        yc, dyc = camber(code, x)
        th = math.atan(dyc)
        upper.append((x - yt * math.sin(th), yc + yt * math.cos(th)))
        lower.append((x + yt * math.sin(th), yc - yt * math.cos(th)))
    return upper, lower


def camber(code: str, x: float):
    m, p = int(code[0]) / 100, int(code[1]) / 10
    if p == 0:
        return 0.0, 0.0
    if x < p:
        return m / p ** 2 * (2 * p * x - x ** 2), 2 * m / p ** 2 * (p - x)
    return (m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2),
            2 * m / (1 - p) ** 2 * (p - x))


def lift_slope(ar: float) -> float:
    """Helmbold finite-wing lift slope, per radian."""
    return 2 * math.pi * ar / (2 + math.sqrt(ar ** 2 + 4))


# --------------------------------------------------------------------------
# Layout


class Layout:
    """Every derived dimension for a given wing leading-edge station x_le."""

    def __init__(self, p: Params, x_le: float, k: Kit = Kit()):
        self.p, self.k = p, k
        c = self.chord = p.span / p.aspect_ratio
        self.area = p.span * c                       # mm^2
        self.x_le = x_le
        self.x_te = x_le + c
        self.x_qc = x_le + 0.25 * c
        self.l_h = p.tail_arm * c
        self.c_h = p.stab_chord_ratio * c
        s_h = p.vh * self.area * c / self.l_h
        # keep the stabiliser short enough to print flat; widen its chord instead
        self.b_h = min(round_to(s_h / self.c_h, 2), p.bed[0] - p.bed_margin)
        self.c_h = s_h / self.b_h
        self.c_e = p.elevator_frac * self.c_h
        self.c_fix = self.c_h - self.c_e
        self.x_stab = self.x_qc + self.l_h - 0.25 * self.c_h
        fin_area = p.vv * self.area * p.span / self.l_h / 2   # per fin
        self.fin_h = round_to(fin_area / self.c_h, 1)         # total height
        self.fin_above = self.fin_h - p.fin_below - p.plate

        self.tube_y = p.tube_spacing / 2
        self.r_hole = p.tube_od / 2 + p.clearance / 2
        self.r_boss = self.r_hole + p.boss_wall
        self.z_top = self.r_boss                     # pod top = wing seat
        self.half_w = self.tube_y + p.wall / 2       # side walls under the tubes
        self.z_bottom = -p.pod_depth
        self.pod_len = max(x_le + p.pod_overlap * c,
                           p.front_bay + k.batt_len + p.batt_trim + p.esc_bay + p.servo_bay + p.wall)
        self.tail_z = self.r_hole + 1.2              # stab seat on the tail mount
        self.tube_x0 = p.front_wall + 1.6
        self.tube_x1 = self.x_stab + self.c_fix - 2.0
        self.tube_len = self.tube_x1 - self.tube_x0
        self.z_floor = self.z_bottom + p.wall
        # elevator servo lies on its side on the floor, base against the left
        # wall, shaft pointing right, horn arm up
        sv = k.servo
        self.elev_servo_x = self.pod_len - p.wall - p.servo_bay / 2      # body centre
        self.elev_servo_base_y = -(self.half_w - p.wall) + 3.9   # clear of the belly chamfer
        self.pushrod_y = self.elev_servo_base_y + sv.horn_z
        self.pushrod_z = self.z_floor + sv.width / 2 + sv.horn_hole
        # yaw: rudder servo under the wing centre, bellcrank under the tail mount,
        # joiner wires to both rudders below the fins
        self.x_hinge = self.x_stab + self.c_fix
        self.joiner_z = -(self.r_hole + 1.2) - 1.2           # below the tail mount sleeves
        self.bellcrank_x = self.x_hinge - 2.0
        self.rudder_pushrod_y = -13.0
        self.elevator_inset = 6.0 if p.rudders else 0.6      # room for the rudders to swing
        # front bay, x from the motor face: camera 3..15, VTX card 15.6..18.6, then the FC
        self.vtx_x = 15.6
        self.fc_x = self.vtx_x + 3.0 + 0.4 + k.fc_len / 2                # FC centre
        if self.fc_x + k.fc_len / 2 + 3.0 > p.front_bay:
            raise ValueError(f"front bay {p.front_bay} mm is too short for the {k.fc_name}: "
                             f"needs {self.fc_x + k.fc_len / 2 + 3.0:.1f} mm")
        # VTX antenna: MMCX socket on the card's -y edge, plug and whip above it
        self.ant_x, self.ant_y, self.ant_z = self.vtx_x + 1.5, -(10.0 + 3.6), -6.5
        # ESC card stands crosswise between two ribs, behind the battery
        self.esc_bay_x = self.pod_len - p.wall - p.servo_bay - p.esc_bay      # bay front
        self.esc_x = self.esc_bay_x + 1.0 + 0.3                              # card front face
        self.batt_min = p.front_bay + 0.5            # battery front limit
        self.batt_max = self.esc_bay_x - 0.5
        self.length = self.x_stab + self.c_h + 20.0  # prop to elevator TE

    def servo_slack(self) -> float:
        """Room left between the spars for the aileron servo's tabs (mm)."""
        p, c = self.p, self.chord
        room = ((p.rear_spar_pos - p.main_spar_pos) * c
                - (p.main_spar_d + p.rear_spar_d) / 2 - 0.8)
        return room - self.k.servo.tab_span

    @property
    def s_h(self):
        return self.b_h * self.c_h

    @property
    def vh_actual(self):
        return self.s_h * self.l_h / (self.area * self.chord)

    @property
    def vv_actual(self):
        return 2 * self.fin_h * self.c_h * self.l_h / (self.area * self.p.span)


# --------------------------------------------------------------------------
# Mass model (analytic; build.py replaces the structure with CAD volumes)


def rod_mass(d: float, length: float, di: float = 0.0, rho: float = CARBON):
    return rho * math.pi / 4 * (d ** 2 - di ** 2) * length / 1000


def structure_estimate(p: Params, L: Layout):
    """(name, grams, x) for the printed parts, tubes and rods."""
    c, b = L.chord, p.span
    panels = b - p.centre_w                              # the centre section is counted separately
    wing_area = (2.06 * c + math.pi * (p.main_spar_d + p.rear_spar_d + 4)) * panels
    tail_plan = L.b_h * L.c_h + 2 * L.fin_h * L.c_h
    plate_equiv = 2 * 0.4 + 0.15 * (p.plate - 0.8)     # 2+2 solid layers, 15 % infill
    pod_vol = (L.pod_len * (2 * p.wall * (p.pod_depth - L.r_hole) + p.wall * 2 * L.half_w)
               + 4 * math.pi * (L.r_boss ** 2 - L.r_hole ** 2) * p.sleeve_len
               + p.front_wall * 2 * L.half_w * (p.pod_depth + L.z_top))
    lid_len = L.x_le - p.sleeve_len
    lid_vol = lid_len * (2 * L.tube_y * 0.8 + 2 * 0.8 * 3.0)
    return [
        ("wing panels", LW_PLA * p.skin * wing_area / 1000, L.x_le + 0.42 * c),
        # calibrated on the CAD: 0.5 mm walls, 5 % infill, saddle, caps, servo fairing
        ("wing centre + saddle", 0.128 * p.centre_w * c / 90 - (0 if p.rudders else 1.0),
         L.x_le + 0.6 * c),
        ("spar rods", rod_mass(p.main_spar_d, b - 10) + rod_mass(p.rear_spar_d, b - 10),
         L.x_le + 0.43 * c),
        ("tail plates", LW_PLA * plate_equiv * tail_plan / 1000, L.x_stab + 0.45 * L.c_h),
        ("tail mount", 3.5, L.x_stab + L.c_fix / 2),
        ("tubes", 2 * rod_mass(p.tube_od, L.tube_len, p.tube_id, p.tube_density),
         (L.tube_x0 + L.tube_x1) / 2),
        ("pod", PLA * pod_vol / 1000, 0.45 * L.pod_len),
        ("lid", PLA * lid_vol / 1000, (p.sleeve_len + L.x_le) / 2),
    ]


def components(p: Params, k: Kit, L: Layout):
    """(name, grams, x, z) for everything that isn't printed, battery excluded."""
    servo_x = L.x_le + (p.main_spar_pos + p.rear_spar_pos) / 2 * L.chord
    return [
        ("motor", k.motor, -7.0, p.motor_z),
        ("prop", k.prop, -15.0, p.motor_z),
        ("camera + VTX", k.cam_vtx, 11.0, -8.0),
        ("antenna", k.antenna, L.ant_x, 10.0),
        ("FC", k.fc, L.fc_x, -13.0),
        ("receiver", k.rx, 42.0, -8.0),
        ("ESC", k.esc, L.esc_x + k.esc_thk / 2, -10.0),
        ("elevator servo", k.servo.mass, L.elev_servo_x, L.z_floor + 6.0),
        ("aileron servos", 2 * k.servo.mass, servo_x, 4.0),
        *([("rudder servo", k.servo.mass, servo_x, 8.0),
           ("rudder linkage", 2.0, L.x_stab + L.c_fix, -4.0)] if p.rudders else []),
        ("wiring", k.wiring, 0.5 * L.pod_len, -8.0),
        ("hardware (fwd)", k.hardware / 2, L.x_le, 0.0),
        ("hardware (tail)", k.hardware / 2, L.x_stab + L.c_h / 2, 0.0),
    ]


def battery_for_cg(items, k: Kit, target_x: float) -> float:
    """Battery centre station that puts the CG on target_x."""
    m = sum(i[1] for i in items)
    mx = sum(i[1] * i[2] for i in items)
    return (target_x * (m + k.battery) - mx) / k.battery


def solve_x_le(p: Params, k: Kit, structure=structure_estimate) -> float:
    """Wing station that balances with the battery mid-way in its travel."""

    def f(x_le):
        L = Layout(p, x_le, k)
        items = [(n, g, x) for n, g, x in structure(p, L)]
        items += [(n, g, x) for n, g, x, _ in components(p, k, L)]
        need = battery_for_cg(items, k, x_le + p.cg_target * L.chord)
        mid = (L.batt_min + L.batt_max) / 2
        return need - mid

    lo, hi = 20.0, 400.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return round((lo + hi) / 2, 1)


# --------------------------------------------------------------------------
# Aerodynamics


def neutral_point(p: Params, L: Layout) -> float:
    """Neutral point as a fraction of chord (wing + tail - pod)."""
    ar = p.aspect_ratio
    a_w = lift_slope(ar)
    a_h = lift_slope(L.b_h / L.c_h)
    downwash = 2 * a_w / (math.pi * ar)
    h_n = 0.25 + 0.9 * L.vh_actual * (a_h / a_w) * (1 - downwash)
    # Pod: Gilruth/Raymer K_f estimate, per degree
    k_f = 0.005 + 0.02 * min(L.x_le / L.pod_len, 1.0) ** 2
    cm_pod = k_f * (2 * L.half_w) ** 2 * L.pod_len / (L.chord * L.area)
    return h_n - cm_pod / (a_w / 57.3)


def drag_area(p: Params, L: Layout, k: Kit):
    """Parasite drag area CdA (m^2) built up from the geometry, per item."""
    mm2 = 1e-6
    tail = (L.b_h * L.c_h + 2 * L.fin_h * L.c_h) * mm2
    w, h = 2 * L.half_w, p.pod_depth + L.z_top
    pod_front = w * h * mm2
    plate = 26 * 13 * mm2                                   # motor plate above the pod
    pod_wet = 2 * L.pod_len * (w + h) * mm2
    tubes_wet = 2 * math.pi * p.tube_od * L.tube_len * mm2
    # Front + base drag of the pod (Hoerner's forebody trend): a sharp-edged box
    # face ~0.5; rounding the edges to r/d ~ 0.2 of the hydraulic diameter
    # removes most of the forebody part, leaving ~0.25 (mostly the blunt base).
    d_h = 2 * w * h / (w + h)                               # hydraulic diameter, mm
    rounded = min(1.0, p.nose_r / (0.2 * d_h))
    cd_pod = 0.5 - 0.25 * rounded
    cd_plate = 0.5 - 0.1 * rounded                          # small fillets only
    items = {
        "wing (Cd0 0.014, printed surface)": L.area * mm2 * 0.014,
        "tail plates (Cd 0.02, flat 2 mm)": tail * 0.02,
        f"pod front + base (Cd {cd_pod:.2f})": pod_front * cd_pod,
        f"motor plate (Cd {cd_plate:.2f})": plate * cd_plate,
        "pod skin friction": pod_wet * 0.006,
        "tubes skin friction": tubes_wet * 0.006,
        "servo bumps, fairing, horns, antenna": 1.5e-4,
    }
    return {n: v * 1.15 for n, v in items.items()}     # +15 % interference


def top_speed(p: Params, auw_g: float, L: Layout, k: Kit):
    """Level top speed where prop thrust meets drag.

    Thrust falls linearly from the static value (the 1:1 thrust target) to zero
    at the loaded pitch speed; drag is parasite CdA plus induced drag."""
    s = L.area / 1e6
    w = auw_g / 1000 * G
    rpm = k.motor_kv * k.v_loaded * k.rpm_frac
    v_pitch = rpm / 60 * k.prop_pitch_in * 0.0254
    cda = sum(drag_area(p, L, k).values())
    t0 = w                                                 # static thrust = weight (1:1)
    lo, hi = 1.0, v_pitch
    for _ in range(60):
        v = (lo + hi) / 2
        q = 0.5 * RHO_AIR * v * v
        cl = w / (q * s)
        drag = q * (cda + s * cl * cl / (math.pi * 0.8 * p.aspect_ratio))
        if t0 * (1 - v / v_pitch) > drag:
            lo = v
        else:
            hi = v
    return {"v_max": lo, "v_pitch": v_pitch, "cda": cda}


def performance(p: Params, auw_g: float, L: Layout, k: Kit):
    s = L.area / 1e6
    w = auw_g / 1000 * G
    v_s = math.sqrt(2 * w / (RHO_AIR * s * p.cl_max))
    v_c = 1.45 * v_s
    # Rough endurance: L/D 5, 40 % prop-motor-ESC efficiency, 1.5 W avionics, 80 % usable
    p_elec = w * v_c / 5 / 0.40 + 1.5
    # Static thrust for a 1:1 thrust-to-weight: ideal induced power of the 4" disc,
    # figure of merit 0.5, motor + ESC 70 %, 7.0 V under load
    disc = math.pi * 0.0508 ** 2
    p_static = (w ** 1.5 / math.sqrt(2 * RHO_AIR * disc)) / 0.5 / 0.70
    top = top_speed(p, auw_g, L, k)
    return {
        "top": top["v_max"],
        "pitch_speed": top["v_pitch"],
        "cda": top["cda"],
        "cruise_w": p_elec,
        "cruise_a": p_elec / 7.4,
        "full_a": p_static / 7.0,
        "stall": v_s,
        "cruise": v_c,
        "re_stall": v_s * L.chord / 1000 / NU_AIR,
        "loading": auw_g / (L.area / 1e4),                               # g/dm^2
        "wcl": (auw_g / 28.35) / ((L.area / 92903) ** 1.5),              # oz/ft^1.5
        "endurance_min": 0.8 * k.batt_wh / p_elec * 60,
    }


def evaluate(p: Params, k: Kit):
    x_le = solve_x_le(p, k)
    L = Layout(p, x_le, k)
    items = [(n, g, x) for n, g, x in structure_estimate(p, L)]
    items += [(n, g, x) for n, g, x, _ in components(p, k, L)]
    auw = sum(i[1] for i in items) + k.battery
    return L, auw, performance(p, auw, L, k)


def sweep(p: Params, k: Kit, spans=range(300, 561, 20)):
    rows = []
    for b in spans:
        q = replace(p, span=float(b))
        L, auw, perf = evaluate(q, k)
        rows.append((b, L, auw, perf))
    return rows


def smallest_span(p: Params, k: Kit, step: int = 10) -> int:
    for b in range(260, 800, step):
        L, _, perf = evaluate(replace(p, span=float(b)), k)
        if perf["stall"] <= p.stall_limit and L.servo_slack() >= 0:
            return b
    raise ValueError("no span meets the stall limit")


def report(p: Params = Params(), k: Kit = Kit()) -> str:
    out = ["## Span sweep (analytic mass model)", "",
           f"Criteria: stall speed <= {p.stall_limit} m/s at CLmax {p.cl_max} "
           f"(comfortable hand launch, FPV-flyable), and the {k.servo.name} aileron servo's "
           "mounting tabs fit between the spars.", "",
           "| span mm | chord mm | AUW g | loading g/dm^2 | WCL | stall m/s | servo room mm | Re at stall |",
           "|---|---|---|---|---|---|---|---|"]
    for b, L, auw, perf in sweep(p, k):
        ok = "**" if perf["stall"] <= p.stall_limit and L.servo_slack() >= 0 else ""
        out.append(f"| {ok}{b}{ok} | {L.chord:.0f} | {auw:.0f} | {perf['loading']:.1f} | "
                   f"{perf['wcl']:.1f} | {ok}{perf['stall']:.2f}{ok} | {L.servo_slack():+.1f} | "
                   f"{perf['re_stall']/1000:.0f}k |")
    b_min = smallest_span(p, k)
    out += ["", f"Smallest span meeting the criterion: **{b_min} mm** "
            f"(design uses {p.span:.0f} mm)."]
    return "\n".join(out)


if __name__ == "__main__":
    p, k = Params(), Kit()
    print(report(p, k))
    L, auw, perf = evaluate(p, k)
    print()
    print(f"x_le {L.x_le:.1f}  chord {L.chord:.1f}  pod {L.pod_len:.1f}  servo room {L.servo_slack():+.1f}  "
          f"stab {L.b_h:.0f}x{L.c_h:.1f} @ {L.x_stab:.1f}  fin h {L.fin_h:.1f}  "
          f"tube {L.tube_len:.0f}  length {L.length:.0f}")
    print(f"AUW {auw:.1f} g  stall {perf['stall']:.2f} m/s  cruise {perf['cruise']:.1f} m/s  "
          f"endurance ~{perf['endurance_min']:.0f} min")
    print(f"Vh {L.vh_actual:.3f}  Vv {L.vv_actual:.3f}  NP {neutral_point(p, L):.3f} c  "
          f"SM {neutral_point(p, L) - p.cg_target:.3f}")
