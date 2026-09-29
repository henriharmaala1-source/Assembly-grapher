"""Kipinä (airframe/): drag model for aero/optimize.py.

The layout comes from airframe/design.py, so a change to the span, aspect ratio
or tail arm re-sizes the tail, re-balances the plane and re-estimates the weight
exactly as the sizing loop does. The drag is then built up item by item from
that layout. Weights are scaled by the CAD/analytic ratio of the design as
built (the CAD adds printed detail and wiring the quick estimate leaves out).
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "airframe"))

import design as D  # noqa: E402
import dragkit as K  # noqa: E402
from model import Param, Result  # noqa: E402

NAME = "airframe"
TITLE = "Kipinä twin-boom pusher"
AREA_UNIT = ("cm²", 1e4)
OBJECTIVE = "Drag"
MAXIMIZE = False
BASE_LABEL = "as built"
DEFAULT_FIX = ["span"]         # the brief: the smallest plane that passes (free it with --free span)
ITEMS_NOTE = "Parasite drag at the speed; the induced drag is on top of this."
BASE, KIT = D.Params(), D.Kit()

_, _auw_quick, _perf0 = D.evaluate(BASE, KIT)
try:
    _auw_cad = json.loads((ROOT / "airframe" / "viewer" / "spec.json").read_text())["auw"]
except (OSError, KeyError, ValueError):
    _auw_cad = _auw_quick
MASS_FACTOR = _auw_cad / _auw_quick
SPEED = round(1.45 * _perf0["stall"] * math.sqrt(MASS_FACTOR), 1)   # the design's cruise speed

NOTE = ("The drag is the total at that speed: parasite (the build-up below) plus the induced "
        "drag of carrying the weight. Weights are the sizing estimate scaled to the CAD "
        f"({MASS_FACTOR:.3f}). Power and endurance here come from this drag (a lift-to-drag "
        "ratio near 7), so they read better than design.py's rougher figure, which assumes 5.")

E_OSWALD = 0.8                 # span efficiency, as design.py
STALL_MARGIN = 0.05            # m/s under the limit, the margin the CAD build keeps
LAMINAR_WING = 0.5             # laminar fraction of the chord at Re 60-90k on a printed skin
LAMINAR_BODY = 0.1
PROP_SUCTION = 0.7             # the pusher prop right behind the pod eases its base drag
ETA_PROP = 0.40                # prop x motor x ESC at cruise, as design.py
AVIONICS_W = 1.5
BOATTAIL_DEG = 12.0            # steepest taper the flow follows without separating
BELLY_CH = 4.0                 # 45 deg chamfers along the pod's belly (build.py)
MOTOR_BOSS_R = 13.0            # motor boss on the pod's back wall (build.py)
HORN_T, HORN_LEN = 1.5, 16.0   # SG90 single-arm horn, edge-on to the flow
ANTENNA_D, ANTENNA_UP = 2.6, 27.0   # VTX whip above the lid


def params() -> list[Param]:
    p = BASE
    out = [
        Param("span", p.span, 460, 600, 5, "mm", "wingspan",
              "held by the brief (the smallest plane that passes) unless --free span; "
              f"stall at most {p.stall_limit - STALL_MARGIN:.2f} m/s with the CAD weight "
              f"({p.stall_limit} m/s less the build's margin)"),
        Param("aspect_ratio", p.aspect_ratio, 4.0, 7.0, 0.1, "", "aspect ratio (span / chord)",
              "the SG90's tabs must fit between the spars (chord of about 80 mm or more); stall"),
        Param("tail_arm", p.tail_arm, 2.0, 4.0, 0.1, "chords", "wing-to-tail distance",
              "tail volumes are held, so stability stays the same; a longer arm moves the wing aft, "
              "and the lid and the rear pod half must still fit the 180 mm bed"),
        Param("nose_r", p.nose_r, 0.0, 12.0, 0.5, "mm", "radius on the pod's front side and belly edges",
              "the 14 mm camera window needs a flat face at least 16 mm wide"),
        Param("nose_top_r", 0.0, 0.0, 10.0, 0.5, "mm", "radius on the pod's top front edge (front of the lid)",
              "a rounded nose block on the lid; the lid itself is only 0.8 mm", kind="what-if, not in the CAD"),
        Param("boattail", 0.0, 0.0, 30.0, 2.0, "mm",
              f"{BOATTAIL_DEG:.0f}° taper on the pod's rear sides and belly, down to the motor mount",
              "the motor mount needs 28 mm of width; the ESC moves 30 mm forward", kind="what-if, not in the CAD"),
        Param("antenna_lean", 0.0, 0.0, 60.0, 5.0, "°", "VTX whip laid back from vertical",
              "60° still clears the wing", kind="what-if, not in the CAD"),
        Param("fairings", 0.0, 0.0, 1.0, 1.0, "", "printed fairings over the four servo horns and "
              "the aileron servo bumps", "about 1 g", kind="what-if, not in the CAD", labels={0: "none", 1: "fitted"}),
        Param("joiners_hidden", 0.0, 0.0, 1.0, 1.0, "", "rudder joiner wires in a groove under the "
              "stabiliser instead of across the open flow", "", kind="what-if, not in the CAD",
              labels={0: "in the open", 1: "in a groove"}),
    ]
    for q in out:
        q.kind = q.kind or "CAD parameter"
    return out


def _segment(r: float, h: float) -> float:
    """Area of a circle's segment of height h (0 <= h <= 2r)."""
    h = min(max(h, 0.0), 2 * r)
    return r * r * math.acos((r - h) / r) - (r - h) * math.sqrt(max(2 * r * h - h * h, 0.0))


def _induced(w: float, s: float, ar: float, v: float) -> float:
    cl = w / (K.q(v) * s)
    return K.q(v) * s * cl * cl / (math.pi * E_OSWALD * ar)


def evaluate(x: dict, v: float) -> Result:
    p = replace(BASE, span=x["span"], aspect_ratio=x["aspect_ratio"], tail_arm=x["tail_arm"],
                nose_r=x["nose_r"])
    try:
        L, auw, _ = D.evaluate(p, KIT)
    except ValueError as e:
        return Result(math.inf, violations=[str(e)])
    auw *= MASS_FACTOR
    w, s = auw / 1000 * D.G, L.area * 1e-6
    stall = math.sqrt(2 * w / (K.RHO * s * p.cl_max))
    bad = []
    if stall > p.stall_limit - STALL_MARGIN + 1e-9:
        bad.append(f"stall {stall:.2f} m/s is over {p.stall_limit - STALL_MARGIN:.2f}")
    if L.servo_slack() < 0:
        bad.append(f"chord {L.chord:.0f} mm is too short for the SG90 tabs")
    # printed parts that grow with the layout (airframe/build.py: lid_end, pod_split_x)
    lid = L.x_le + 0.08 * L.chord - 0.5 - (p.front_wall + 0.5)
    rear_half = L.pod_len - round(L.batt_max + 10.0) + 8.0
    for part, size in (("lid", lid), ("rear pod half", rear_half)):
        if size > p.bed[0]:
            bad.append(f"{part} is {size:.0f} mm, longer than the {p.bed[0]:.0f} mm bed")

    mm, mm2 = 1e-3, 1e-6
    items = []
    # wing: printed NACA 4412 shell; the pod covers part of the centre's underside
    c = L.chord * mm
    wet = (2.04 * L.area - 1.02 * 2 * L.half_w * L.chord) * mm2
    k_wing = K.cf(K.reynolds(v, c), LAMINAR_WING) * K.ff_wing(0.12, 0.3)
    items.append(K.Item("wing, skin friction and form", k_wing * wet, "friction",
                        f"Re {K.reynolds(v, c) / 1e3:.0f}k, laminar over half the chord"))
    # tail: 2 mm plates with round leading edges
    ch = L.c_h * mm
    tail_wet = 2 * (L.b_h * L.c_h + 2 * L.fin_h * L.c_h) * mm2
    k_tail = K.cf(K.reynolds(v, ch), LAMINAR_WING) * K.ff_wing(p.plate / L.c_h, 0.3)
    items.append(K.Item("tail plates, skin friction", k_tail * tail_wet, "friction",
                        f"stabiliser {L.b_h:.0f} x {L.c_h:.0f} mm, fins {L.fin_h:.0f} mm tall"))
    # pod front: pressure drag depends on how round each front edge is
    pw, ph = 2 * L.half_w, L.z_top - L.z_bottom
    a_front = pw * ph - BELLY_CH ** 2
    dh = K.hydraulic_d(pw, ph)
    edges = [(pw, x["nose_top_r"]), (pw, max(x["nose_r"], K.chamfer_radius(BELLY_CH))), (2 * ph, x["nose_r"])]
    cd_front = sum(n * K.cd_forebody(r / dh, 0.75, 0.04) for n, r in edges) / sum(n for n, _ in edges)
    items.append(K.Item("pod front (pressure)", cd_front * a_front * mm2, "pressure",
                        f"Cd {cd_front:.2f} on {a_front / 100:.1f} cm², top edge "
                        f"{'sharp' if x['nose_top_r'] == 0 else 'r ' + str(x['nose_top_r'])}"))
    # pod base: the back wall and the motor boss above the pod top
    taper = x["boattail"] * math.tan(math.radians(BOATTAIL_DEG))
    a_base = (pw - 2 * taper) * (ph - taper) + _segment(MOTOR_BOSS_R, MOTOR_BOSS_R - L.z_top)
    items.append(K.Item("pod base (back wall)", K.CD_BASE * PROP_SUCTION * a_base * mm2, "pressure",
                        f"{a_base / 100:.1f} cm² of blunt base" + (", boat-tailed" if taper else "")))
    # pod skin: the top under the wing is not wetted
    pod_wet = (2 * (pw + ph) * L.pod_len - pw * L.chord) * mm2
    k_pod = K.cf(K.reynolds(v, L.pod_len * mm), LAMINAR_BODY) * K.ff_body(L.pod_len / dh)
    items.append(K.Item("pod, skin friction", k_pod * pod_wet, "friction", f"{L.pod_len:.0f} mm long"))
    # booms: exposed between the sockets and the tail mount
    od = p.tube_od
    exposed = L.tube_x1 - L.socket_x1 - L.c_fix
    k_boom = K.cf(K.reynolds(v, exposed * mm), LAMINAR_BODY) * K.ff_body(exposed / od)
    items.append(K.Item("booms, skin friction", k_boom * 2 * math.pi * od * exposed * mm2, "friction",
                        f"2 x {exposed:.0f} mm exposed"))
    # boom sockets: domed noses, a step down to the tube at their ends, half their skin
    rb = L.r_boss
    sock = (2 * math.pi * rb * rb * K.cd_forebody((rb - 1) / (2 * rb), 0.75, 0.04)
            + 2 * math.pi * (rb * rb - (od / 2) ** 2) * K.CD_BASE
            + k_boom * math.pi * rb * (L.socket_x1 - L.socket_x0) * 2)
    items.append(K.Item("boom sockets", sock * mm2, "pressure", "domed noses, step at the tube"))
    # tail mount: forward-facing steps of the collars, pushrod guide
    r_out = L.r_hole + 1.2
    mount = 2 * math.pi * (r_out ** 2 - (od / 2) ** 2) * 0.6 + 3 * 5.5 * 0.6
    items.append(K.Item("tail mount collars and guide", mount * mm2, "pressure", ""))
    # protuberances
    horns = 4 * K.CD_PLATE * HORN_T * HORN_LEN * (0.25 if x["fairings"] else 1)
    bumps = 2 * K.CD_BUMP * 3.0 * D.Servo().width * (0.5 if x["fairings"] else 1)
    items.append(K.Item("servo horns and aileron servo bumps", (horns + bumps) * mm2, "protuberances",
                        "faired" if x["fairings"] else "4 horns edge-on below the wing, 3 mm servo bumps"))
    links = 2 * (K.CD_PLATE * 1.0 * 9.0 + K.CD_CYLINDER * 0.8 * 8.0)
    items.append(K.Item("aileron horns and links", links * mm2, "protuberances", ""))
    joiners = K.cylinder_cda(0.8 * mm, (L.b_h - 4) * mm) * (0.2 if x["joiners_hidden"] else 1)
    items.append(K.Item("rudder joiner wires", joiners, "protuberances",
                        f"0.8 mm wire, {L.b_h - 4:.0f} mm across the flow" +
                        (", in a groove" if x["joiners_hidden"] else "")))
    tail_bits = (3 * K.CD_PLATE * 1.6 * 7.0 + K.CD_PLATE * 1.6 * 20.0) * mm2
    items.append(K.Item("elevator and rudder horns, bellcrank", tail_bits, "protuberances", ""))
    items.append(K.Item("VTX antenna", K.cylinder_cda(ANTENNA_D * mm, ANTENNA_UP * mm, x["antenna_lean"]),
                        "protuberances", f"whip {ANTENNA_UP:.0f} mm above the lid" +
                        (f", laid back {x['antenna_lean']:.0f}°" if x["antenna_lean"] else "")))
    junction = 0.10 * sum(i.cda for i in items if i.group in ("friction", "pressure"))
    items.append(K.Item("junctions (10 % of the above)", junction, "interference",
                        "wing-pod, boom-wing and tail joints"))

    cda = K.total(items)
    d_ind = _induced(w, s, p.aspect_ratio, v)
    drag = K.q(v) * cda + d_ind

    # level top speed: thrust falls from the weight (1:1 static) to zero at the pitch speed
    v_pitch = KIT.motor_kv * KIT.v_loaded * KIT.rpm_frac / 60 * KIT.prop_pitch_in * 0.0254
    lo, hi = 1.0, v_pitch
    for _ in range(50):
        m = (lo + hi) / 2
        if w * (1 - m / v_pitch) > K.q(m) * cda + _induced(w, s, p.aspect_ratio, m):
            lo = m
        else:
            hi = m
    p_elec = drag * v / ETA_PROP + AVIONICS_W
    metrics = [("Induced drag", force_text(d_ind)),
               ("Weight", f"{auw:.0f} g"), ("Stall", f"{stall:.2f} m/s"),
               ("Chord / pod length", f"{L.chord:.0f} / {L.pod_len:.0f} mm"),
               ("Top speed (level)", f"{lo * 3.6:.0f} km/h"),
               ("Power at this speed", f"{p_elec:.1f} W"),
               ("Endurance at this speed", f"{0.8 * KIT.batt_wh / p_elec * 60:.1f} min")]
    return Result(drag, items, metrics, bad)


def speed_text(v: float) -> str:
    return f"{v:.1f} m/s ({v * 3.6:.0f} km/h)"


def value_text(n: float) -> str:
    return force_text(n)


def force_text(n: float) -> str:
    return f"{n:.3f} N ({n / D.G * 1000:.1f} gf)"


METHOD = """
* The layout, tail size, balance and weight at each point come from
  `airframe/design.py`, the same model the CAD build starts from.
* Drag build-up (`aero/dragkit.py`): Raymer-style skin friction and form
  factors for the wing, tail, pod and booms (laminar over half the chord on
  the flying surfaces at these low Reynolds numbers); Hoerner's rounded-edge
  forebody curve for the pod front, weighted over its four edges; a square-cut
  base (Cd 0.2, eased 30 % by the pusher prop's suction) for the back wall;
  wires, horns and the antenna as cylinders and strips across the flow;
  10 % for the junctions. Induced drag with a span efficiency of 0.8.
* The stall limit uses the CAD weight; the chord must leave room for the SG90
  tabs between the spars, as in the sizing loop.
* What-ifs (not built in the CAD yet) are marked in the tables. They would
  need CAD changes before they count.
* At Reynolds numbers of 50 000 to 300 000 the handbook coefficients are
  uncertain, perhaps +-30 % on the totals and more on the small items. Use the
  ranking and the differences, not the absolute numbers, and check anything
  important with a glide or power test.
"""
