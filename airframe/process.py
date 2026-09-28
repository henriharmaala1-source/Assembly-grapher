#!/usr/bin/env python3
"""Bill of process (BOP), assembly sequence and DFMA analysis for Kipinä.

Run after build.py (it reads viewer/spec.json and stl/):

    python3 airframe/process.py            # uses PrusaSlicer for print times if installed
    python3 airframe/process.py --no-slice # reuse the cached slicer results

Writes:
    BOP.md               operations, precedence graph, schedule
    DFMA.md              Boothroyd-Dewhurst style assembly analysis + DFM of the prints
    viewer/process.json  the same data for the viewer and its assembly animation
    tools/slice_cache.json

Time standards
--------------
Handling and insertion times follow the Boothroyd-Dewhurst manual assembly
charts (rounded values for the classes used here). Separate processes (gluing,
soldering, taping, bending) use shop estimates listed in PROC. They are
estimates for one experienced builder, not measurements.
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent

# --------------------------------------------------------------------------
# Time standards (seconds)

HANDLING = {  # class: (seconds, description)
    "H00": (1.13, "one hand, easy to grasp, symmetric (alpha+beta < 360 deg)"),
    "H10": (1.50, "one hand, alpha+beta 360-540 deg"),
    "H20": (1.80, "one hand, alpha+beta 540-720 deg"),
    "Hs":  (2.50, "small part, fingertips or tweezers (M2 screw, horn)"),
    "Hf":  (4.00, "flexible or tangles (wire, lead, pushrod)"),
    "H2h": (3.00, "long or large, two hands (tube, spar, wing panel)"),
}
INSERTION = {  # class: (seconds, description)
    "I00": (1.5, "placed, no holding, easy to align"),
    "I01": (2.5, "placed, not easy to align"),
    "I03": (3.5, "placed, hard to align, resistance"),
    "I06": (5.5, "held during the next step, easy to align"),
    "I08": (6.5, "held during the next step, hard to align"),
    "I30": (2.0, "snap or press fit, easy"),
    "I31": (5.0, "press fit with resistance"),
    "I38": (6.0, "screwed in immediately, easy"),
    "I39": (8.0, "screwed in immediately, awkward access"),
}
PROC = {  # separate processes: (seconds each, description)
    "ca":         (15, "CA glue joint: apply and hold"),
    "epoxy_mix":  (60, "mix a batch of 5-minute epoxy"),
    "epoxy":      (20, "epoxy joint"),
    "hotglue":    (20, "hot-glue a servo in its pocket"),
    "solder":     (25, "solder joint, tinned"),
    "shrink":     (15, "heat-shrink sleeve"),
    "tape":       (10, "double-sided tape pad"),
    "tape_hinge": (40, "tape hinge, top and bottom"),
    "strap":      (20, "velcro strap"),
    "cut_tube":   (60, "mark, cut and deburr a carbon tube"),
    "cut_rod":    (30, "cut a carbon rod or wire"),
    "z_bend":     (30, "Z-bend in 0.8 mm wire"),
    "l_bend":     (20, "L-bend in 0.8 mm wire"),
    "rod_end":    (40, "glue a wire end into a carbon rod, shrink"),
    "trim_horn":  (30, "trim a servo horn"),
    "ream":       (15, "ream a hole with a hand drill"),
    "unplate":    (20, "take a part off the plate, clean edges"),
    "brim":       (60, "remove a brim"),
    "route":      (30, "route a servo lead"),
    "hook":       (15, "hook a pushrod end into a horn"),
    "square":     (60, "sight and square the tail to the wing"),
    "plug":       (8, "plug a lead into the FC"),
    "plate_swap": (120, "clear the plate, start the next job"),
    "bind":       (120, "bind the receiver"),
    "config":     (1800, "INAV setup: mixer, servo directions, endpoints, OSD"),
    "balance":    (180, "set the battery position for the CG"),
    "checks":     (300, "control directions, throws, failsafe, range check"),
}
EPOXY_CURE = 15 * 60          # 5-minute epoxy to handling strength
BRIDGE_MAX = 30.0             # mm a flat ceiling can bridge without support
FILAMENT_CHANGE = 300         # swap spools between materials
EUR_PER_KG = {"PLA/PETG": 20.0, "LW-PLA": 55.0}   # rough street prices

# Rough street prices for the bought parts, EUR, for relative comparison only
BOUGHT = [
    ("SG90 servo", 4, 3.0), ("1404 motor, 3000-3800 KV", 1, 15.0), ("4x2.5 prop (pair)", 1, 1.0),
    ("12 A ESC", 1, 8.0), ("20x20 wing FC (INAV)", 1, 40.0), ("ELRS nano receiver", 1, 10.0),
    ("nano camera 14 mm", 1, 15.0), ("VTX 25-200 mW", 1, 15.0), ("5.8 GHz whip antenna", 1, 3.0),
    ("2S 450 mAh LiPo", 1, 8.0), ("carbon tube 6x5 x 500 mm", 2, 3.0), ("carbon rod 3 / 2 mm", 2, 2.0),
    ("pushrod rod, wire, horns", 1, 3.0), ("screws, tape, velcro, glue", 1, 4.0),
]

# --------------------------------------------------------------------------
# Bill of process
#
# items:   (name, qty, handling, insertion, n_min, reason) -- DFMA rows
# proc:    (activity, count)
# anim:    (parts, from_offset_mm, to_offset_mm); offsets in the flight frame
#          (x aft, y right, z up). Parts appear at `from` and move to `to`.
# focus:   parts the camera frames; view: camera direction (viewer frame)

TAIL = ["tail_mount", "stab", "fin_R", "fin_L", "elevator", "rudder_R", "rudder_L", "bellcrank"]
PARK = (90, 0, 0)             # tail sub-assembly is built behind the plane, then slid on
V_POD, V_WHOLE, V_TAIL = [-0.8, 0.8, 0.9], [-0.95, 0.62, 0.9], [0.9, 0.55, 0.85]
V_UNDER, V_UNDER_TAIL = [-0.45, -0.8, -0.6], [0.75, -0.5, 0.6]

OPS = [
    # ---- printing (A1 mini) ----
    dict(id="P1", title="Print plate 1: pod, lid, tail mount, rudders, bellcrank", station="Print",
         resource="printer", needs=[], plate=["pod", "lid", "tail_mount", "rudder_R", "rudder_L", "bellcrank"]),
    dict(id="P2", title="Print plate 2: wing centre, fins", station="Print", resource="printer",
         needs=[], plate=["wing_centre", "fin_R", "fin_L"]),
    dict(id="P3", title="Print plate 3: stabiliser, elevator, ailerons", station="Print",
         resource="printer", needs=[], plate=["stab", "elevator", "aileron_R", "aileron_L"]),
    dict(id="P4", title="Print plate 4: right wing panel", station="Print", resource="printer",
         needs=[], plate=["wing_R"]),
    dict(id="P5", title="Print plate 5: left wing panel", station="Print", resource="printer",
         needs=[], plate=["wing_L"]),
    # ---- preparation ----
    dict(id="F1", title="Cut the carbon tubes and spars", station="Prep", resource="builder", needs=[],
         proc=[("cut_tube", 2), ("cut_rod", 2)], tools=["rotary tool or fine saw", "ruler", "file"]),
    dict(id="F2", title="Make the pushrods, aileron links and rudder joiners", station="Prep",
         resource="builder", needs=[],
         proc=[("cut_rod", 6), ("z_bend", 8), ("rod_end", 4), ("l_bend", 4)],
         tools=["wire bender or pliers", "cutter", "heat gun"], consumables=["CA", "heat-shrink"]),
    dict(id="F3", title="Trim the elevator (13.5 mm) and rudder (11 mm) servo horns", station="Prep",
         resource="builder", needs=[], proc=[("trim_horn", 2)], tools=["side cutter"]),
    dict(id="Q1", title="Finish plate 1 parts, ream tube and pivot holes", station="Prep",
         resource="builder", needs=["P1"], proc=[("unplate", 6), ("ream", 5)],
         tools=["6.2 / 1.6 mm drill bits in a pin vice"]),
    dict(id="Q2", title="Finish the wing centre and fins, ream holes", station="Prep",
         resource="builder", needs=["P2"], proc=[("unplate", 3), ("ream", 4)],
         tools=["6.2 / 3.2 / 2.2 mm drill bits"]),
    dict(id="Q3", title="Finish the tail surfaces and ailerons", station="Prep", resource="builder",
         needs=["P3"], proc=[("unplate", 4)]),
    dict(id="Q4", title="Remove the brim from the right panel, ream spar holes", station="Prep",
         resource="builder", needs=["P4"], proc=[("unplate", 1), ("brim", 1), ("ream", 2)]),
    dict(id="Q5", title="Remove the brim from the left panel, ream spar holes", station="Prep",
         resource="builder", needs=["P5"], proc=[("unplate", 1), ("brim", 1), ("ream", 2)]),
    # ---- pod ----
    dict(id="A1", title="Screw the motor to the pod", station="Pod", resource="builder", needs=["Q1"],
         items=[("pod", 1, "H10", "I00", 1, "base part"),
                ("motor", 1, "H10", "I06", 1, "rotates"),
                ("M2 screw (motor)", 4, "Hs", "I38", 0, "fastener")],
         tools=["1.5 mm hex driver"],
         anim=[(["pod"], (0, 0, -30), None), (["motor"], (-45, 0, 0), None)], focus=["pod"], view=V_POD),
    dict(id="A2", title="Fit the camera and VTX", station="Pod", resource="builder", needs=["A1"],
         items=[("camera", 1, "H00", "I08", 1, "separate function, optics"),
                ("vtx", 1, "H00", "I06", 0, "could be one camera+VTX module")],
         proc=[("ca", 2), ("tape", 1)], consumables=["CA", "foam tape"],
         anim=[(["camera"], (0, 0, 45), None), (["vtx"], (0, 0, 45), None)], focus=["pod"], view=V_POD),
    dict(id="A3", title="Mount the FC, ESC and receiver", station="Pod", resource="builder", needs=["A2"],
         items=[("fc", 1, "H00", "I01", 1, "separate electronics"),
                ("M2 screw (FC)", 4, "Hs", "I38", 0, "fastener"),
                ("esc", 1, "H10", "I06", 0, "could be integrated in the FC"),
                ("receiver", 1, "H10", "I06", 0, "could be integrated in the FC")],
         proc=[("tape", 2)], tools=["1.5 mm hex driver"], consumables=["foam tape"],
         anim=[(["fc"], (0, 0, 45), None), (["esc", "receiver"], (0, 0, 45), None)],
         focus=["pod"], view=V_POD),
    dict(id="A4", title="Solder the power, motor, video and receiver wiring", station="Pod",
         resource="builder", needs=["A3"], proc=[("solder", 16), ("shrink", 8)],
         tools=["soldering iron"], consumables=["solder", "heat-shrink", "silicone wire"]),
    dict(id="A5", title="Glue the elevator servo into the rear bay", station="Pod", resource="builder",
         needs=["A4", "F3"],
         items=[("servo_elev", 1, "H10", "I08", 1, "moves"),
                ("servo horn (elevator)", 1, "Hs", "I01", 0, "supplied with the servo"),
                ("horn screw", 1, "Hs", "I38", 0, "fastener")],
         proc=[("hotglue", 1)], consumables=["hot glue"],
         anim=[(["servo_elev"], (0, 0, 45), None)], focus=["pod"], view=V_POD),
    # ---- wing centre ----
    dict(id="A6", title="Fit the rudder servo and its pushrod in the wing centre", station="Wing",
         resource="builder", needs=["Q2", "F2", "F3"],
         items=[("servo_rud", 1, "H10", "I08", 1, "moves"),
                ("servo horn (rudder)", 1, "Hs", "I01", 0, "supplied with the servo"),
                ("horn screw", 1, "Hs", "I38", 0, "fastener"),
                ("pushrod_rud", 1, "Hf", "I03", 1, "moves")],
         proc=[("hotglue", 1), ("hook", 1)], consumables=["hot glue"],
         anim=[(["wing_centre"], (0, 0, 60), None), (["servo_rud"], (0, 0, -40), None),
               (["pushrod_rud"], (120, 0, 0), None)],
         focus=["pod", "wing_centre"], view=V_POD),
    # ---- frame ----
    dict(id="A7", title="Join the tubes, pod and wing centre", station="Frame", resource="builder",
         needs=["A5", "A6", "F1"], cure=EPOXY_CURE,
         items=[("wing_centre", 1, "H10", "I08", 1, "different material (LW-PLA)"),
                ("tube_L", 1, "H2h", "I03", 1, "different material (carbon)"),
                ("tube_R", 1, "H2h", "I03", 0, "could be one U-frame with tube_L")],
         proc=[("epoxy_mix", 1), ("epoxy", 6)], consumables=["5-minute epoxy"],
         anim=[(["tube_L", "tube_R"], (300, 0, 0), None)], focus=["pod", "wing_centre", "tube_L"],
         view=V_WHOLE),
    # ---- wing ----
    dict(id="A8", title="Slide in the spars and glue on both wing panels", station="Wing",
         resource="builder", needs=["A7", "Q4", "Q5", "F1"], cure=EPOXY_CURE,
         items=[("spar_main", 1, "H2h", "I03", 1, "different material (carbon)"),
                ("spar_rear", 1, "H2h", "I03", 0, "could merge with the main spar"),
                ("wing_R", 1, "H2h", "I08", 0, "separate only for the 180 mm bed"),
                ("wing_L", 1, "H2h", "I08", 0, "separate only for the 180 mm bed")],
         proc=[("epoxy_mix", 1), ("epoxy", 2)], consumables=["5-minute epoxy"],
         anim=[(["spar_main", "spar_rear"], (0, 320, 0), None), (["wing_R"], (0, 140, 0), None),
               (["wing_L"], (0, -140, 0), None)],
         focus=["wing_R", "wing_L", "pod"], view=V_WHOLE),
    dict(id="A9", title="Fit the aileron servos and route their leads", station="Wing",
         resource="builder", needs=["A8"],
         items=[("servo_ail_R", 1, "H10", "I08", 1, "moves"),
                ("servo_ail_L", 1, "H10", "I08", 1, "moves")],
         proc=[("hotglue", 2), ("route", 2)], consumables=["hot glue"],
         anim=[(["servo_ail_R", "servo_ail_L"], (0, 0, -45), None)],
         focus=["wing_R", "wing_L"], view=V_UNDER),
    dict(id="A10", title="Hinge the ailerons, glue the horns, fit the links", station="Wing",
         resource="builder", needs=["A9", "Q3", "F2"],
         items=[("aileron_R", 1, "H10", "I08", 1, "moves"),
                ("aileron_L", 1, "H10", "I08", 1, "moves"),
                ("horn_ail_R", 1, "Hs", "I08", 0, "could be printed into the aileron"),
                ("horn_ail_L", 1, "Hs", "I08", 0, "could be printed into the aileron"),
                ("pushrod_ail_R", 1, "Hf", "I03", 1, "moves"),
                ("pushrod_ail_L", 1, "Hf", "I03", 1, "moves")],
         proc=[("tape_hinge", 2), ("ca", 2)], consumables=["hinge tape", "CA"],
         anim=[(["aileron_R", "aileron_L"], (45, 0, 0), None),
               (["horn_ail_R", "horn_ail_L", "pushrod_ail_R", "pushrod_ail_L"], (0, 0, -30), None)],
         focus=["wing_R", "wing_L"], view=V_UNDER),
    # ---- tail ----
    dict(id="A11", title="Glue the stabiliser to the tail mount and the fins to the tips",
         station="Tail", resource="builder", needs=["Q1", "Q2", "Q3"],
         items=[("tail_mount", 1, "H10", "I00", 0, "could be printed with the stabiliser"),
                ("stab", 1, "H10", "I08", 1, "different material (LW-PLA)"),
                ("fin_R", 1, "H00", "I31", 0, "could be printed with the stabiliser"),
                ("fin_L", 1, "H00", "I31", 0, "could be printed with the stabiliser")],
         proc=[("ca", 4)], consumables=["CA"],
         anim=[(["tail_mount"], (PARK[0], 0, -40), PARK), (["stab"], (PARK[0], 0, 40), PARK),
               (["fin_R"], (PARK[0], 60, 0), PARK), (["fin_L"], (PARK[0], -60, 0), PARK)],
         focus=["stab", "fin_R", "fin_L", "tail_mount"], focus_offset=PARK, view=V_TAIL),
    dict(id="A12", title="Hinge the elevator and rudders, screw on the bellcrank", station="Tail",
         resource="builder", needs=["A11", "Q1"],
         items=[("elevator", 1, "H10", "I08", 1, "moves"),
                ("rudder_R", 1, "H00", "I08", 1, "moves"),
                ("rudder_L", 1, "H00", "I08", 1, "moves"),
                ("bellcrank", 1, "Hs", "I01", 1, "moves"),
                ("M2 screw (bellcrank)", 1, "Hs", "I39", 0, "fastener")],
         proc=[("tape_hinge", 3)], consumables=["hinge tape"], tools=["1.5 mm hex driver"],
         anim=[(["elevator", "rudder_R", "rudder_L"], (PARK[0] + 35, 0, 0), PARK),
               (["bellcrank"], (PARK[0], 0, -30), PARK)],
         focus=["stab", "fin_R", "fin_L", "tail_mount"], focus_offset=PARK, view=V_UNDER_TAIL),
    dict(id="A13", title="Slide the tail onto the tubes, square it and glue", station="Frame",
         resource="builder", needs=["A12", "A8"], cure=EPOXY_CURE,
         items=[("tail sub-assembly", 1, "H2h", "I08", 0, "sub-assembly, already counted")],
         proc=[("square", 1), ("epoxy_mix", 1), ("epoxy", 2)], consumables=["5-minute epoxy"],
         anim=[(TAIL, PARK, None)], focus=None, view=V_WHOLE),
    dict(id="A14", title="Connect the elevator pushrod, rudder pushrod and joiners", station="Final",
         resource="builder", needs=["A13", "F2"],
         items=[("pushrod_elev", 1, "Hf", "I03", 1, "moves"),
                ("joiner_R", 1, "Hf", "I08", 1, "moves"),
                ("joiner_L", 1, "Hf", "I08", 0, "could be one wire with joiner_R")],
         proc=[("hook", 3)],
         anim=[(["pushrod_elev"], (140, 0, 0), None), (["joiner_R"], (0, 60, 0), None),
               (["joiner_L"], (0, -60, 0), None)],
         focus=TAIL, view=V_UNDER_TAIL),
    dict(id="A15", title="Plug in the servo leads and tidy the wiring", station="Final",
         resource="builder", needs=["A14", "A10", "A9", "A6"], proc=[("plug", 4), ("route", 1)]),
    dict(id="A16", title="Fit the prop, battery, antenna and lid", station="Final", resource="builder",
         needs=["A15"],
         items=[("prop", 1, "H10", "I38", 1, "rotates, replaced after crashes"),
                ("battery", 1, "H10", "I06", 1, "removed for charging"),
                ("antenna", 1, "Hf", "I01", 0, "part of the VTX"),
                ("lid", 1, "H00", "I30", 1, "opened for battery changes")],
         proc=[("strap", 1)], tools=["prop wrench"],
         anim=[(["battery"], (0, 0, 50), None), (["antenna"], (0, 0, 40), None),
               (["lid"], (0, 0, 30), None), (["prop"], (-45, 0, 0), None)],
         focus=None, view=V_WHOLE),
    # ---- setup ----
    dict(id="S1", title="Bind the receiver and set up INAV", station="Setup", resource="builder",
         needs=["A16"], proc=[("bind", 1), ("config", 1)], tools=["laptop with INAV Configurator"]),
    dict(id="S2", title="Balance and run the pre-flight checks", station="Setup", resource="builder",
         needs=["S1"], proc=[("balance", 1), ("checks", 1)], tools=["CG balancer or fingertips"]),
]


# --------------------------------------------------------------------------
# Printing: PrusaSlicer with an A1-mini-like profile


def slicer_options(name: str, material: str) -> list[str]:
    if material == "PLA/PETG":
        opts = ["--perimeters", "2", "--fill-density", "50%" if name == "bellcrank" else "15%",
                "--top-solid-layers", "3", "--bottom-solid-layers", "3",
                "--filament-max-volumetric-speed", "12", "--filament-density", "1.27"]
        if name.startswith("rudder"):
            opts[4:8] = ["--top-solid-layers", "2", "--bottom-solid-layers", "2"]
        return opts
    # LW-PLA: slow, foamed (flow ~55 %), one wall
    slow = ["--perimeter-speed", "60", "--external-perimeter-speed", "50", "--infill-speed", "70",
            "--solid-infill-speed", "60", "--top-solid-infill-speed", "50", "--first-layer-speed", "25",
            "--filament-max-volumetric-speed", "6", "--extrusion-multiplier", "0.55",
            "--filament-density", "1.24", "--perimeters", "1"]
    if name.startswith(("wing_R", "wing_L", "aileron")):
        return slow + ["--fill-density", "0%", "--top-solid-layers", "2", "--bottom-solid-layers", "2"] + \
            (["--brim-width", "8"] if name.startswith("wing_") else [])
    if name == "wing_centre":
        return slow + ["--fill-density", "5%", "--top-solid-layers", "2", "--bottom-solid-layers", "2",
                       "--extrusion-width", "0.5"]
    return slow + ["--fill-density", "15%", "--top-solid-layers", "2", "--bottom-solid-layers", "2"]


def parse_time(text: str) -> float:
    secs = 0.0
    for value, unit in re.findall(r"(\d+)([dhms])", text):
        secs += int(value) * {"d": 86400, "h": 3600, "m": 60, "s": 1}[unit]
    return secs


def slice_parts(parts, cache_path: Path, use_slicer: bool):
    cache = json.loads(cache_path.read_text()) if cache_path.exists() else {}
    exe = shutil.which("prusa-slicer")
    if not (use_slicer and exe):
        missing = [p["name"] for p in parts if p["name"] not in cache]
        if missing:
            print("no slicer results for", missing, "- install prusa-slicer or run without --no-slice")
        return cache
    ini = HERE / "tools" / "a1mini.ini"
    with tempfile.TemporaryDirectory() as tmp:
        for p in parts:
            stl = HERE / "stl" / f"{p['name']}.stl"
            out = Path(tmp) / f"{p['name']}.gcode"
            cmd = [exe, "--export-gcode", "--load", str(ini), *slicer_options(p["name"], p["material"]),
                   "--output", str(out), str(stl)]
            subprocess.run(cmd, check=True, capture_output=True, timeout=900)
            g = out.read_text(errors="ignore")
            t = re.search(r"estimated printing time \(normal mode\) = (.+)", g).group(1)
            grams = float(re.search(r"filament used \[g\] = ([\d.]+)", g).group(1))
            cache[p["name"]] = {"seconds": parse_time(t), "grams": grams}
            print(f"sliced {p['name']:12s} {t:>12s} {grams:6.1f} g")
    cache_path.write_text(json.dumps(cache, indent=1))
    return cache


# --------------------------------------------------------------------------
# DFM: printability from the print-oriented STLs


def overhangs(stl: Path, angle: float = 45.0, small: float = 10.0):
    """Down-facing regions steeper than `angle` from vertical, above the first layer.
    Returns (total area mm^2, [(area, span mm, flat)] for regions wider than `small`, mesh).
    A flat region is a horizontal ceiling the printer can bridge."""
    import numpy as np
    import trimesh
    m = trimesh.load(stl)
    n = m.face_normals
    tri_z = m.triangles[:, :, 2]
    down = (n[:, 2] < -np.cos(np.radians(angle))) & (tri_z.min(axis=1) > 0.3)
    faces = np.nonzero(down)[0]
    if len(faces) == 0:
        return 0.0, [], m
    adj = m.face_adjacency
    mask = down[adj[:, 0]] & down[adj[:, 1]]
    groups = trimesh.graph.connected_components(adj[mask], nodes=faces)
    wide = []
    for g in groups:
        pts = m.triangles[g].reshape(-1, 3)
        span = float(min(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        area = float(m.area_faces[g].sum())
        flat = bool(np.abs(n[g, 2]).min() > 0.98)
        if span > small and area > 5:
            wide.append((area, span, flat))
    return float(m.area_faces[faces].sum()), sorted(wide, reverse=True), m


def dfm(spec, sliced):
    import numpy as np
    rows = []
    for p in spec["parts"]:
        stl = HERE / "stl" / f"{p['name']}.stl"
        total, wide, m = overhangs(stl)
        ext = m.extents
        slender = float(ext[2] / max(min(ext[0], ext[1]), 1e-6))
        s = sliced.get(p["name"], {})
        notes = []
        needs_support = [w for w in wide if not (w[2] and w[1] <= BRIDGE_MAX)]
        bridges = [w for w in wide if w[2] and w[1] <= BRIDGE_MAX]
        if needs_support:
            support = "support"
            notes.append(f"{len(needs_support)} overhang(s) wider than 10 mm; largest {needs_support[0][1]:.0f} mm")
        elif bridges:
            support = f"bridge {max(b[1] for b in bridges):.0f} mm"
        else:
            support = "none"
        if slender > 6:
            notes.append(f"tall and thin (height/width {slender:.0f}); brim and slow Y moves on a bed-slinger")
        if not p.get("fits", True):
            notes.append("does not fit the build volume")
        bed_area = float(m.area_faces[(m.face_normals[:, 2] < -0.99) &
                                      (m.triangles[:, :, 2].max(axis=1) < 0.05)].sum())
        if bed_area < 150:
            notes.append(f"small bed contact ({bed_area:.0f} mm^2); use a brim or glue stick")
        rows.append({
            "name": p["name"], "material": p["material"], "bed": p["bed"],
            "minutes": round(s.get("seconds", 0) / 60, 1), "grams": s.get("grams"),
            "cost": round((s.get("grams") or p["grams"]) / 1000 * EUR_PER_KG[p["material"]], 2),
            "overhang_mm2": round(total), "support": support,
            "bed_contact_mm2": round(bed_area), "slender": round(slender, 1), "notes": notes,
        })
    return rows


# --------------------------------------------------------------------------
# Times, schedule, DFMA metrics


def op_times(op, dfm_rows):
    items = [(n, q, h, i, HANDLING[h][0] + INSERTION[i][0]) for n, q, h, i, *_ in op.get("items", [])]
    t_items = sum(q * t for _, q, _, _, t in items)
    t_proc = sum(PROC[a][0] * c for a, c in op.get("proc", []))
    if op["resource"] == "printer":
        minutes = {r["name"]: r["minutes"] for r in dfm_rows}
        return {"items": 0.0, "process": sum(minutes[n] * 60 for n in op["plate"]) + PROC["plate_swap"][0],
                "cure": 0.0}
    return {"items": t_items, "process": t_proc, "cure": op.get("cure", 0)}


def schedule(ops, times):
    """List scheduling: one printer, one builder; cure time blocks successors only."""
    free = {"printer": 0.0, "builder": 0.0}
    done, start, finish, ready = {}, {}, {}, {}
    order, pending = [], list(ops)
    last_material = None
    while pending:
        avail = [o for o in pending if all(n in done for n in o["needs"])]
        # prefer the op that can start earliest; printer jobs keep their listed order
        def est(o):
            t = max([ready.get(n, 0.0) for n in o["needs"]] + [free[o["resource"]]])
            return (t, o["resource"] != "printer", ops.index(o))
        o = min(avail, key=est)
        t0 = est(o)[0]
        dur = times[o["id"]]["items"] + times[o["id"]]["process"]
        if o["resource"] == "printer":
            mat = "PETG" if o["id"] == "P1" else "LW-PLA"
            if last_material and mat != last_material:
                dur += FILAMENT_CHANGE
            last_material = mat
        start[o["id"]], finish[o["id"]] = t0, t0 + dur
        ready[o["id"]] = t0 + dur + times[o["id"]]["cure"]
        free[o["resource"]] = t0 + dur
        done[o["id"]] = True
        pending.remove(o)
        order.append(o["id"])
    return start, finish, ready, order


def dfma_rows():
    rows = []
    for op in OPS:
        for name, qty, h, i, nmin, why in op.get("items", []):
            t = HANDLING[h][0] + INSERTION[i][0]
            rows.append({"op": op["id"], "item": name, "qty": qty, "handling": h, "insertion": i,
                         "t_each": round(t, 2), "t_total": round(qty * t, 2),
                         "n_min": nmin * (1 if qty == 1 else 1), "why": why})
    return rows


SUGGESTIONS = [
    # (title, parts removed, seconds saved, trade-off)
    ("Print the aileron control horns into the ailerons", 2,
     2 * (HANDLING["Hs"][0] + INSERTION["I08"][0] + PROC["ca"][0]),
     "PETG ailerons, or a thicker LW-PLA horn; +0.3 g"),
    ("Use an FC with a built-in ELRS receiver", 1,
     HANDLING["H10"][0] + INSERTION["I06"][0] + PROC["tape"][0] + 4 * PROC["solder"][0] + 2 * PROC["shrink"][0],
     "fewer board choices in 20x20 wing FCs"),
    ("Use a nano camera with an integrated VTX", 1,
     HANDLING["H00"][0] + INSERTION["I06"][0] + PROC["tape"][0] + 3 * PROC["solder"][0] + PROC["shrink"][0],
     "fixed VTX power, harder to replace"),
    ("Print the tail mount and fins in one piece with the stabiliser (PETG)", 3,
     HANDLING["H10"][0] + INSERTION["I00"][0] + 2 * (HANDLING["H00"][0] + INSERTION["I31"][0]) + 4 * PROC["ca"][0],
     "about +3 g in the tail (PETG instead of LW-PLA), needs supports under the fins"),
    ("Replace tape hinges with printed pin hinges", 0, 5 * PROC["tape_hinge"][0] - 5 * 20,
     "stiffer surfaces, 0.8 mm pins; slightly more drag at the gap"),
    ("Key the tube ends (a flat) so the tail squares itself", 0, PROC["square"][0],
     "a file stroke per tube; no alignment jig needed"),
    ("One U-shaped joiner wire instead of two joiners", 1, HANDLING["Hf"][0] + INSERTION["I08"][0] + PROC["hook"][0],
     "must be threaded past the bellcrank"),
    ("Snap-fit tabs for the servos instead of hot glue", 0, 4 * PROC["hotglue"][0] - 4 * INSERTION["I30"][0],
     "SG90 clones vary; needs a test print of the pocket"),
]


# --------------------------------------------------------------------------
# Reports


def fmt_min(sec):
    h, m = divmod(round(sec / 60), 60)
    return f"{h} h {m:02d} min" if h else f"{m} min"


SHORT = {
    "P1": "Plate 1 PETG",
    "P2": "Plate 2",
    "P3": "Plate 3",
    "P4": "Plate 4",
    "P5": "Plate 5",
    "F1": "Cut carbon",
    "F2": "Make linkages",
    "F3": "Trim horns",
    "Q1": "Finish plate 1",
    "Q2": "Finish plate 2",
    "Q3": "Finish plate 3",
    "Q4": "Finish plate 4",
    "Q5": "Finish plate 5",
    "A1": "Motor",
    "A2": "Camera, VTX",
    "A3": "FC, ESC, RX",
    "A4": "Solder",
    "A5": "Elevator servo",
    "A6": "Rudder servo",
    "A7": "Join frame",
    "A8": "Wing panels",
    "A9": "Aileron servos",
    "A10": "Ailerons",
    "A11": "Tail glue-up",
    "A12": "Tail hinges",
    "A13": "Tail on tubes",
    "A14": "Linkages",
    "A15": "Servo leads",
    "A16": "Prop, battery",
    "S1": "INAV setup",
    "S2": "Balance, checks"
}


def critical_path(ops, times, finish, ready):
    """Walk back from the last op through whichever predecessor (or the previous
    printer job) released it."""
    by_id = {o["id"]: o for o in ops}
    printer = [o["id"] for o in ops if o["resource"] == "printer"]
    last = max(finish, key=lambda k: ready[k])
    path = [last]
    while True:
        o = by_id[path[-1]]
        cands = list(o["needs"])
        if o["id"] in printer and printer.index(o["id"]) > 0:
            cands.append(printer[printer.index(o["id"]) - 1])
        start = finish[o["id"]] - times[o["id"]]["items"] - times[o["id"]]["process"]
        cands = [c for c in cands if min(abs(ready[c] - start), abs(finish[c] - start)) < 400]
        if not cands:
            break
        path.append(max(cands, key=lambda c: ready[c]))
    return list(reversed(path))


def mermaid(ops, crit):
    lines = ["flowchart TB"]
    for o in ops:
        lines.append(f'  {o["id"]}["{o["id"]} {SHORT[o["id"]]}"]')
    for o in ops:
        for n in o["needs"]:
            lines.append(f"  {n} --> {o['id']}")
    printer = [o["id"] for o in ops if o["resource"] == "printer"]
    for a, b in zip(printer, printer[1:]):
        lines.append(f"  {a} -. printer queue .-> {b}")
    lines.append("  classDef crit stroke:#E4601A,stroke-width:2px")
    lines.append(f"  class {','.join(crit)} crit")
    return "\n".join(lines)


def main():
    use_slicer = "--no-slice" not in sys.argv
    spec = json.loads((HERE / "viewer" / "spec.json").read_text())
    sliced = slice_parts(spec["parts"], HERE / "tools" / "slice_cache.json", use_slicer)
    dfm_list = dfm(spec, sliced)
    times = {o["id"]: op_times(o, dfm_list) for o in OPS}
    start, finish, ready, order = schedule(OPS, times)
    rows = dfma_rows()
    crit = critical_path(OPS, times, finish, ready)

    n_actual = sum(r["qty"] for r in rows if r["item"] != "tail sub-assembly")
    n_min = sum(r["n_min"] for r in rows)
    t_handle_insert = sum(r["t_total"] for r in rows)
    t_assembly_proc = sum(times[o["id"]]["process"] for o in OPS if o["id"].startswith("A"))
    t_ma = t_handle_insert + t_assembly_proc
    e_ma = 3 * n_min / t_ma
    e_hi = 3 * n_min / t_handle_insert
    t_print = sum(times[o["id"]]["process"] for o in OPS if o["resource"] == "printer")
    t_hands = sum(times[o["id"]]["items"] + times[o["id"]]["process"] for o in OPS
                  if o["resource"] == "builder")
    t_cure = sum(times[o["id"]]["cure"] for o in OPS)
    lead = max(ready.values())
    filament = sum((r["grams"] or 0) for r in dfm_list)
    print_cost = sum(r["cost"] for r in dfm_list)
    bought_cost = sum(q * c for _, q, c in BOUGHT)

    suggestions = []
    n_after, t_after = n_actual, t_ma
    for title, dn, dt, trade in SUGGESTIONS:
        suggestions.append({"title": title, "parts": dn, "seconds": round(dt), "trade": trade})
        n_after -= dn
        t_after -= dt
    e_after = 3 * n_min / t_after

    summary = {
        "n_actual": n_actual, "n_min": n_min, "t_ma": round(t_ma), "t_handle_insert": round(t_handle_insert),
        "e_ma": round(e_ma, 3), "e_hi": round(e_hi, 3), "n_after": n_after, "t_after": round(t_after),
        "e_after": round(e_after, 3), "t_print": round(t_print), "t_hands": round(t_hands),
        "t_cure": round(t_cure), "lead": round(lead), "filament_g": round(filament, 1),
        "print_cost": round(print_cost, 2), "bought_cost": round(bought_cost),
        "slicer": "PrusaSlicer 2.7, A1-mini-like profile (tools/a1mini.ini)" if sliced else "not sliced",
    }

    ops_out = []
    for o in OPS:
        t = times[o["id"]]
        ops_out.append({
            "id": o["id"], "title": o["title"], "short": SHORT[o["id"]], "station": o["station"],
            "resource": o["resource"], "critical": o["id"] in crit,
            "needs": o["needs"], "parts": o.get("plate") or [n for n, *_ in o.get("items", [])],
            "tools": o.get("tools", []), "consumables": o.get("consumables", []),
            "seconds": round(t["items"] + t["process"]), "cure": t["cure"],
            "start": round(start[o["id"]]), "finish": round(finish[o["id"]]),
            "anim": [{"parts": p, "from": list(f), "to": list(to or (0, 0, 0))} for p, f, to in o.get("anim", [])],
            "focus": o.get("focus"), "focus_offset": list(o.get("focus_offset", (0, 0, 0))),
            "view": o.get("view"),
        })
    data = {"summary": summary, "critical": crit, "ops": ops_out, "dfma": rows, "dfm": dfm_list, "suggestions": suggestions,
            "handling": {k: v[0] for k, v in HANDLING.items()}, "insertion": {k: v[0] for k, v in INSERTION.items()}}
    (HERE / "viewer" / "process.json").write_text(json.dumps(data, indent=1))

    # ---------------- BOP.md
    L = ["# Kipinä: bill of process", "",
         "Generated by `process.py` from the CAD model and the slicer. Do not edit by hand.", "",
         "One builder and one Bambu Lab A1 mini. Print times come from "
         f"{summary['slicer']}; hand times are Boothroyd-Dewhurst handling and insertion estimates "
         "plus shop times for gluing, soldering and bending (see `process.py`).", "",
         "| | |", "|---|---|",
         f"| Printer time | {fmt_min(t_print)} over 5 plates, {filament:.0f} g of filament |",
         f"| Hands-on time | {fmt_min(t_hands)} (prep, assembly, setup) |",
         f"| Epoxy cure waits | {fmt_min(t_cure)} |",
         f"| Lead time | **{fmt_min(lead)}** with the builder working while the printer runs |",
         f"| Critical path | {' > '.join(crit)} |", "",
         "## Precedence graph", "",
         "Solid arrows: must finish first. Dashed: the printer queue. Highlighted: critical path.", "",
         "```mermaid", mermaid(OPS, crit), "```", "",
         "## Operations", "",
         "| op | operation | station | resource | time | start | end | needs | tools / consumables |",
         "|---|---|---|---|---|---|---|---|---|"]
    for oid in order:
        o = next(x for x in ops_out if x["id"] == oid)
        tc = ", ".join(o["tools"] + o["consumables"])
        cure = f" + {o['cure'] // 60} min cure" if o["cure"] else ""
        L.append(f"| {oid} | {o['title']} | {o['station']} | {o['resource']} | {fmt_min(o['seconds'])}{cure} | "
                 f"{fmt_min(o['start'])} | {fmt_min(o['finish'])} | {', '.join(o['needs']) or '-'} | {tc} |")
    L += ["", "## Parts per operation", ""]
    for o in ops_out:
        if o["parts"]:
            L.append(f"- **{o['id']}** {', '.join(o['parts'])}")
    (HERE / "BOP.md").write_text("\n".join(L) + "\n")

    # ---------------- DFMA.md
    D = ["# Kipinä: DFMA analysis", "",
         "Generated by `process.py`. Do not edit by hand.", "",
         "## Design for assembly (Boothroyd-Dewhurst, manual)", "",
         "| | |", "|---|---|",
         f"| Parts and fasteners assembled | {n_actual} |",
         f"| Theoretical minimum part count | {n_min} |",
         f"| Handling + insertion time | {t_handle_insert:.0f} s |",
         f"| Plus separate processes (glue, solder, tape, bend) | {t_assembly_proc:.0f} s |",
         f"| Total manual assembly time | {t_ma:.0f} s ({fmt_min(t_ma)}) |",
         f"| Design efficiency, handling + insertion only | {e_hi:.0%} |",
         f"| Design efficiency, including processes | **{e_ma:.0%}** |",
         f"| After the suggestions below | {n_after} parts, {t_after:.0f} s, efficiency {e_after:.0%} |", "",
         "Design efficiency = 3 s x theoretical minimum parts / actual assembly time. "
         "Handling and inserting parts is only a fifth of the assembly time; gluing, soldering, "
         "taping and bending take the rest, so the biggest wins remove joints, not just parts.", "",
         "A part counts toward the theoretical minimum when it moves relative to the parts "
         "already assembled, must be a different material, or must be separate for "
         "assembly or service.", "",
         "### Redesign suggestions", "",
         "| suggestion | parts removed | time saved | trade-off |", "|---|---|---|---|"]
    for s in suggestions:
        D.append(f"| {s['title']} | {s['parts']} | {s['seconds']} s | {s['trade']} |")
    D += ["", "### Assembly worksheet", "",
          "| op | item | qty | handling | insertion | s each | s total | min parts | reason |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        D.append(f"| {r['op']} | {r['item']} | {r['qty']} | {r['handling']} | {r['insertion']} | "
                 f"{r['t_each']:.2f} | {r['t_total']:.2f} | {r['n_min']} | {r['why']} |")
    D += ["", "Handling classes: " + "; ".join(f"{k} {v[0]} s ({v[1]})" for k, v in HANDLING.items()) + ".",
          "", "Insertion classes: " + "; ".join(f"{k} {v[0]} s ({v[1]})" for k, v in INSERTION.items()) + ".",
          "", "Process times: " + "; ".join(f"{k} {v[0]} s ({v[1]})" for k, v in PROC.items()) + ".",
          "", "## Design for manufacture (3D printing)", "",
          f"Sliced with {summary['slicer']}. Overhangs: down-facing surfaces steeper than 45 deg "
          "from vertical, above the first layer. Regions narrower than 10 mm (hole tops, chamfers) "
          f"print without support; flat ceilings up to {BRIDGE_MAX:.0f} mm are bridges.", "",
          "| part | material | print time | filament | cost | overhang area | support | notes |",
          "|---|---|---|---|---|---|---|---|"]
    for r in dfm_list:
        D.append(f"| {r['name']} | {r['material']} | {r['minutes']:.0f} min | {r['grams'] or 0:.1f} g | "
                 f"{r['cost']:.2f} EUR | {r['overhang_mm2']} mm^2 | {r['support']} | {'; '.join(r['notes']) or '-'} |")
    D += ["", f"Total: {fmt_min(t_print)} of printing, {filament:.0f} g of filament, "
          f"about {print_cost:.2f} EUR of material (PETG {EUR_PER_KG['PLA/PETG']:.0f} EUR/kg, "
          f"LW-PLA {EUR_PER_KG['LW-PLA']:.0f} EUR/kg).", "",
          "## Cost", "", "| item | qty | EUR each | EUR |", "|---|---|---|---|"]
    for name, q, c in BOUGHT:
        D.append(f"| {name} | {q} | {c:.2f} | {q * c:.2f} |")
    D += [f"| printed parts (filament) | 1 | {print_cost:.2f} | {print_cost:.2f} |",
          f"| **total** | | | **{bought_cost + print_cost:.0f}** |", "",
          "Prices are rough street prices for comparison only; the flight controller is most of it."]
    (HERE / "DFMA.md").write_text("\n".join(D) + "\n")

    print(f"parts {n_actual}, min {n_min}, t_ma {t_ma:.0f} s, E {e_ma:.1%} (h+i {e_hi:.1%}); "
          f"print {fmt_min(t_print)}, hands-on {fmt_min(t_hands)}, lead {fmt_min(lead)}")


if __name__ == "__main__":
    main()
