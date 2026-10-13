"""
Real Assembly Test — two standard industrial assemblies run through the full
DFMA + resource calculator + assembly sequence pipeline.

Assembly 1: PN16 Flange Joint (DN50)
  Source: DIN EN 1092-1, standard weld-neck flange kit
  Parts : weld-neck flange, EPDM ring gasket, M16 stud bolts × 4,
          M16 hex nuts × 4, flat washers × 8
  Expected: clean FVS (all M16 hex), gasket handling flag, 1 assembly direction

Assembly 2: UCP205 Pillow-Block Bearing Unit
  Source: SKF / FYH UCP205 datasheet, INA UC205 insert bearing
  Parts : cast-iron housing, UC205 insert bearing, M8 set-screws × 2,
          rubber end-seal, Zerk grease fitting, M12 mounting bolts × 2
  Expected: clean FVS per zone, bearing must_move flag, two distinct tool zones

Run:
    python real_assemblies_demo.py
    python real_assemblies_demo.py --verbose
    python real_assemblies_demo.py --no-plan   (skip assembly planning; faster)
"""

from __future__ import annotations
import argparse

from dfma.models.part     import Part, Assembly, Geometry, ManufacturingProcess, Material
from dfma.models.fastener import FastenerSpec, DriveType, HeadStyle
from dfma.models.warning  import Severity
from dfma.analyzer        import analyze
from dfma.rules.resource_calculator import calculate_resources
from assembly_graph import AssemblyPlanner


# ═══════════════════════════════════════════════════════════════════════════════
# ASSEMBLY 1 — PN16 Weld-Neck Flange Joint, DN50
# DIN EN 1092-1 / ISO 7005-1
# ═══════════════════════════════════════════════════════════════════════════════

def build_flange_assembly() -> Assembly:
    """
    PN16 DN50 bolted flange joint (one side).
    Dimensions from DIN EN 1092-1 Table 7 (DN50 PN16):
      Flange OD=165mm, raised-face OD=85mm, PCD=125mm, thickness=18mm, 4 holes M16
    """
    flange = Part(
        id="FL01", name="Weld-Neck Flange DN50",
        process=ManufacturingProcess.CNC_MACHINING,   # forged + machined
        material=Material.METAL,
        must_be_separate=True,                         # structural pressure vessel part
        geometry=Geometry(
            length=165, width=165, height=60,          # OD × OD × (flange+neck height)
            min_wall_thickness=18.0,                   # flange face thickness
            fillet_radius=3.0,                         # neck-to-flange blend
            draft_angle=2.0,                           # machined — no draft needed but benign
            tolerance=0.1,
            alpha=360, beta=360,                       # rotationally symmetric
            mass_grams=2500,
        ),
    )

    gasket = Part(
        id="FL02", name="EPDM Ring Gasket DN50",
        process=ManufacturingProcess.INJECTION_MOLDING,
        material=Material.RUBBER,
        must_differ_material=True,                     # sealing — must be rubber/elastomer
        geometry=Geometry(
            length=80, width=80, height=3,             # OD × OD × thickness
            min_wall_thickness=3.0,
            tolerance=0.2,
            alpha=360, beta=360,
            mass_grams=18,
            is_flexible=True,                          # rubber ring deforms during handling
        ),
    )

    stud_bolt = Part(
        id="FL03", name="M16 Stud Bolt × 60 mm",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        quantity=4,                                    # DIN EN 1092-1 DN50 PN16: 4 bolts
        geometry=Geometry(
            length=60, width=16, height=16,
            min_wall_thickness=4.0,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=85,                             # M16×60 ≈ 85 g per bolt
            is_fastener=True,
            fastener_clearance_mm=12.0,                # ample clearance at flange face
        ),
    )

    hex_nut = Part(
        id="FL04", name="M16 Hex Nut (DIN 934)",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        quantity=4,
        geometry=Geometry(
            length=24, width=24, height=13,            # 24 mm AF, 13 mm height
            min_wall_thickness=4.0,
            tolerance=0.05,
            alpha=60, beta=360,                        # hex cross-section: α=60° period
            mass_grams=45,
            is_fastener=True,
            fastener_clearance_mm=12.0,
        ),
    )

    washer = Part(
        id="FL05", name="M16 Flat Washer (DIN 125)",
        process=ManufacturingProcess.SHEET_METAL,
        material=Material.METAL,
        quantity=8,                                    # 2 per bolt (under head and nut)
        geometry=Geometry(
            length=30, width=30, height=2,             # OD=30mm, thickness=2mm
            min_wall_thickness=2.0,
            fillet_radius=0.1,
            tolerance=0.1,
            alpha=360, beta=360,
            mass_grams=8,
        ),
    )

    return Assembly(
        id="ASM_FL",
        name="PN16 Flange Joint DN50",
        parts=[flange, gasket, stud_bolt, hex_nut, washer],
        assembly_directions=1,                         # all bolts from same +Z direction
    )


def build_flange_fasteners() -> list[FastenerSpec]:
    """
    4 × M16 hex stud bolts in the flange bolt circle.
    All the same size and drive type → FVS should be clean (zero issues).
    Torque: 150–180 Nm per DIN EN 13480 / VDI 2230.
    """
    bolts = []
    for i in range(1, 5):
        bolts.append(FastenerSpec(
            id=f"FB0{i}", name=f"M16 Flange Stud Bolt #{i}",
            nominal_diameter_mm=16.0,
            total_length_mm=60.0,
            head_diameter_mm=24.0,     # 24 mm AF — drives with 24 mm socket
            head_height_mm=13.0,       # nut height
            thread_pitch_mm=2.0,       # M16 coarse
            drive_type=DriveType.HEX_BOLT,
            head_style=HeadStyle.HEX,
            hole_diameter_mm=18.0,     # M16 clearance hole per DIN EN 1092-1
            grip_thickness_mm=36.0,    # 2× flange thickness 18 mm
            thread_engagement_mm=20.0, # nut engagement ≈ 1.25 × d = 20 mm
            axial_clearance_above_head_mm=80.0,   # open flange environment
            radial_clearance_mm=30.0,
            insertion_path_length_mm=90.0,
            subassembly_id="SA_FLANGE_BOLTS",
        ))
    return bolts


def build_flange_planner() -> AssemblyPlanner:
    """
    Liaison contacts for the flange joint (directed: base-outward).
    Assembly base = FL01 (flange body).
    """
    ids   = ["FL01", "FL02", "FL03", "FL04", "FL05"]
    names = dict(zip(ids, ["Weld-Neck Flange", "EPDM Gasket", "Stud Bolt M16",
                           "Hex Nut M16", "Flat Washer M16"]))
    times = {"FL01": 15.0, "FL02": 8.0, "FL03": 6.0, "FL04": 12.0, "FL05": 4.0}
    tools = {
        "FL01": {"fixture"},
        "FL02": {"hands"},
        "FL03": {"hands"},
        "FL04": {"torque_wrench_24mm"},
        "FL05": {"hands"},
    }
    dirs  = {"FL01": "base", "FL02": "+Z", "FL03": "+Z", "FL04": "+Z", "FL05": "+Z"}

    planner = AssemblyPlanner(
        part_ids=ids, part_names=names, part_times=times,
        part_tools=tools, part_dirs=dirs, base_part_id="FL01",
    )
    planner.liaison.add_contact("FL01", "FL02", "face",   strength="rigid",    direction="+Z", is_structural=True)
    planner.liaison.add_contact("FL01", "FL03", "thread", strength="rigid",    direction="+Z", is_structural=True)
    planner.liaison.add_contact("FL03", "FL04", "thread", strength="rigid",    direction="+Z", is_structural=True)
    planner.liaison.add_contact("FL03", "FL05", "face",   strength="sliding",  direction="+Z", is_structural=False)
    planner.liaison.add_contact("FL04", "FL05", "face",   strength="sliding",  direction="+Z", is_structural=False)
    return planner


# ═══════════════════════════════════════════════════════════════════════════════
# ASSEMBLY 2 — UCP205 Pillow-Block Bearing Unit (SKF / FYH)
# INA / SKF UC205 insert bearing + P205 cast iron housing
# ═══════════════════════════════════════════════════════════════════════════════

def build_bearing_assembly() -> Assembly:
    """
    UCP205 pillow-block bearing unit: 25 mm shaft bore.
    Housing dimensions from SKF catalogue: L=143mm H=103mm W=51mm, 4.8 kg.
    UC205 bearing: 52mm OD, 36.5mm width, 25mm bore, ~410g.
    """
    housing = Part(
        id="BR01", name="Cast Iron Pillow Block Housing P205",
        process=ManufacturingProcess.DIE_CASTING,      # sand-cast gray iron
        material=Material.METAL,
        must_be_separate=True,
        geometry=Geometry(
            length=143, width=51, height=103,
            min_wall_thickness=8.0,                    # typical cast iron wall
            fillet_radius=3.0,
            draft_angle=1.5,                           # sand cast: ≥ 1°
            tolerance=0.1,
            alpha=180, beta=180,                       # symmetric about shaft axis
            mass_grams=4800,
        ),
    )

    insert_bearing = Part(
        id="BR02", name="UC205 Insert Ball Bearing (25mm bore)",
        process=ManufacturingProcess.CNC_MACHINING,    # ground bearing races
        material=Material.METAL,
        must_move=True,                                # inner ring rotates
        must_be_separate=True,                         # separate precision component
        geometry=Geometry(
            length=52, width=52, height=36,            # OD=52mm, W=36.5mm
            min_wall_thickness=5.0,                    # race wall
            fillet_radius=0.5,
            tolerance=0.005,                           # precision ground ±5 µm
            alpha=360, beta=360,
            mass_grams=410,
            requires_alignment=True,                   # bore must align with shaft
        ),
    )

    set_screw = Part(
        id="BR03", name="M8 Set Screw (bearing lock)",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        quantity=2,
        geometry=Geometry(
            length=12, width=8, height=8,
            min_wall_thickness=2.0,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=4,
            is_fastener=True,
            fastener_clearance_mm=8.0,
        ),
    )

    end_seal = Part(
        id="BR04", name="Rubber End Seal / Cover",
        process=ManufacturingProcess.INJECTION_MOLDING,
        material=Material.RUBBER,
        must_differ_material=True,                     # elastomer seal vs steel/iron
        geometry=Geometry(
            length=52, width=52, height=5,
            min_wall_thickness=3.0,
            tolerance=0.3,
            alpha=360, beta=360,
            mass_grams=55,
            is_flexible=True,
        ),
    )

    grease_fitting = Part(
        id="BR05", name="Zerk Grease Fitting M10×1.0",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        geometry=Geometry(
            length=12, width=10, height=10,
            min_wall_thickness=1.5,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=5,
            is_fastener=True,
            fastener_clearance_mm=15.0,
        ),
    )

    mount_bolt = Part(
        id="BR06", name="M12 Socket Cap Mounting Bolt × 40 mm",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        quantity=2,
        geometry=Geometry(
            length=40, width=12, height=12,
            min_wall_thickness=3.0,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=28,
            is_fastener=True,
            fastener_clearance_mm=10.0,
        ),
    )

    return Assembly(
        id="ASM_BR",
        name="UCP205 Pillow-Block Bearing Unit",
        parts=[housing, insert_bearing, set_screw, end_seal, grease_fitting, mount_bolt],
        assembly_directions=2,                         # bearing +Y into housing; bolts +Z down
    )


def build_bearing_fasteners() -> list[FastenerSpec]:
    """
    Two separate fastener zones:
      SA_MOUNT        : 2 × M12 socket-cap mounting bolts (clamp unit to machine)
      SA_BEARING_LOCK : 2 × M8 socket set-screws  (lock bearing on shaft)
    Both zones are standardized (one diameter per zone) → FVS clean.
    """
    return [
        # ── Mounting bolts M12 ─────────────────────────────────────────────
        FastenerSpec(
            id="BB01", name="M12 Socket Cap Mounting Bolt #1",
            nominal_diameter_mm=12.0, total_length_mm=40.0,
            head_diameter_mm=18.0,   head_height_mm=12.0,   # ISO 4762 M12
            thread_pitch_mm=1.75,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.SOCKET_CAP,
            hole_diameter_mm=13.0,           # M12 clearance
            grip_thickness_mm=16.0,
            thread_engagement_mm=18.0,       # 1.5 × d = 18 mm in cast iron
            axial_clearance_above_head_mm=35.0,
            radial_clearance_mm=12.0,
            insertion_path_length_mm=60.0,
            subassembly_id="SA_MOUNT",
        ),
        FastenerSpec(
            id="BB02", name="M12 Socket Cap Mounting Bolt #2",
            nominal_diameter_mm=12.0, total_length_mm=40.0,
            head_diameter_mm=18.0,   head_height_mm=12.0,
            thread_pitch_mm=1.75,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.SOCKET_CAP,
            hole_diameter_mm=13.0,
            grip_thickness_mm=16.0,
            thread_engagement_mm=18.0,
            axial_clearance_above_head_mm=35.0,
            radial_clearance_mm=12.0,
            insertion_path_length_mm=60.0,
            subassembly_id="SA_MOUNT",
        ),

        # ── Set screws M8 (bearing lock) ────────────────────────────────────
        FastenerSpec(
            id="BS01", name="M8 Set Screw Bearing Lock #1",
            nominal_diameter_mm=8.0,  total_length_mm=12.0,
            head_diameter_mm=6.0,    head_height_mm=0.0,    # headless set screw
            thread_pitch_mm=1.25,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.STUD,
            hole_diameter_mm=8.2,
            grip_thickness_mm=0.0,
            thread_engagement_mm=12.0,       # full engagement into bearing inner ring
            axial_clearance_above_head_mm=10.0,
            radial_clearance_mm=8.0,
            insertion_path_length_mm=20.0,
            subassembly_id="SA_BEARING_LOCK",
        ),
        FastenerSpec(
            id="BS02", name="M8 Set Screw Bearing Lock #2",
            nominal_diameter_mm=8.0,  total_length_mm=12.0,
            head_diameter_mm=6.0,    head_height_mm=0.0,
            thread_pitch_mm=1.25,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.STUD,
            hole_diameter_mm=8.2,
            grip_thickness_mm=0.0,
            thread_engagement_mm=12.0,
            axial_clearance_above_head_mm=10.0,
            radial_clearance_mm=8.0,
            insertion_path_length_mm=20.0,
            subassembly_id="SA_BEARING_LOCK",
        ),
    ]


def build_bearing_planner() -> AssemblyPlanner:
    """
    Liaison contacts for the pillow-block assembly.
    Assembly base = BR01 (housing body).
    """
    ids   = ["BR01", "BR02", "BR03", "BR04", "BR05", "BR06"]
    names = dict(zip(ids, ["Cast Iron Housing", "UC205 Insert Bearing", "M8 Set Screw",
                           "Rubber End Seal", "Zerk Grease Fitting", "M12 Mounting Bolt"]))
    times = {"BR01": 20.0, "BR02": 25.0, "BR03": 10.0,
             "BR04": 8.0,  "BR05": 5.0,  "BR06": 12.0}
    tools = {
        "BR01": {"fixture"},
        "BR02": {"rubber_mallet", "bearing_press"},
        "BR03": {"hex_key_M4"},
        "BR04": {"hands"},
        "BR05": {"spanner_M10"},
        "BR06": {"hex_key_M10"},
    }
    dirs  = {"BR01": "base", "BR02": "+Y", "BR03": "+R",
             "BR04": "+Z",   "BR05": "+Z", "BR06": "+Z"}

    planner = AssemblyPlanner(
        part_ids=ids, part_names=names, part_times=times,
        part_tools=tools, part_dirs=dirs, base_part_id="BR01",
    )
    planner.liaison.add_contact("BR01", "BR02", "press_fit", strength="rigid",   direction="+Y", is_structural=True)
    planner.liaison.add_contact("BR02", "BR03", "thread",    strength="rigid",   direction="+R", is_structural=True)
    planner.liaison.add_contact("BR01", "BR04", "press_fit", strength="sliding", direction="+Z", is_structural=False)
    planner.liaison.add_contact("BR01", "BR05", "thread",    strength="rigid",   direction="+Z", is_structural=False)
    planner.liaison.add_contact("BR01", "BR06", "thread",    strength="rigid",   direction="+Z", is_structural=True)
    return planner


# ═══════════════════════════════════════════════════════════════════════════════
# Runner
# ═══════════════════════════════════════════════════════════════════════════════

RESET = "\033[0m"
BOLD  = "\033[1m"
CYAN  = "\033[96m"
GREEN = "\033[92m"
YELLOW= "\033[93m"
RED   = "\033[91m"
GREY  = "\033[90m"

SEV_CLR = {
    Severity.ERROR:   RED,
    Severity.WARNING: YELLOW,
    Severity.INFO:    CYAN,
}


def _bar(score: float, width: int = 20) -> str:
    filled = round(score * width)
    return "█" * filled + "░" * (width - filled)


def run_analysis(
    assembly, fasteners, planner,
    verbose: bool = False,
    run_plan: bool = True,
) -> None:
    sep = "─" * 70

    # ── DFMA ──────────────────────────────────────────────────────────────────
    result = analyze(assembly, fasteners=fasteners)
    n_e = len(result.errors())
    n_w = len(result.warnings_only())
    n_i = len(result.infos())

    print(f"\n{'═' * 70}")
    print(f"{BOLD}  {assembly.name}{RESET}")
    print(f"{'═' * 70}")
    print(result.summary())

    if result.dfa_index >= 0.60:
        grade, gc = "GOOD  — well-optimised", GREEN
    elif result.dfa_index >= 0.35:
        grade, gc = "FAIR  — moderate improvement potential", YELLOW
    else:
        grade, gc = "POOR  — significant redesign recommended", RED
    print(f"  DFA Grade: {gc}{grade}{RESET}")

    for sev in (Severity.ERROR, Severity.WARNING, Severity.INFO):
        items = [w for w in result.warnings if w.severity == sev]
        if not items:
            continue
        clr = SEV_CLR[sev]
        print(f"\n{sep}")
        print(f"{clr}  {sev.value}S ({len(items)}){RESET}")
        print(sep)
        for w in items:
            print(f"{clr}  [{w.rule_id}] {w.part_id}: {w.message}{RESET}")
            if verbose and w.suggestion:
                print(f"    → {w.suggestion}")
            if w.metric_value is not None and w.threshold is not None:
                print(f"{GREY}    value={w.metric_value:.4g}  threshold={w.threshold:.4g}{RESET}")

    # ── Resource calculator ────────────────────────────────────────────────────
    resources = calculate_resources(fasteners)
    print(f"\n{sep}")
    print(resources.report(show_zones=True))

    # ── Assembly planning ──────────────────────────────────────────────────────
    if run_plan:
        plan = planner.plan(sa_iterations=1000)

        print(f"  {CYAN}ASSEMBLY PLAN{RESET}")
        print(f"  {sep[:68]}")
        print(f"  Detected subassemblies : {len(plan.subassemblies)}")
        for sub in plan.subassemblies:
            parts_str = ", ".join(sorted(sub.part_ids))
            print(f"    {YELLOW}{sub.id}{RESET}  [{parts_str}]  score={sub.overall_score:.2f}")

        art = plan.articulation_points   # list of dicts from articulation_point_report()
        if art:
            art_str = ", ".join(ap["part_id"] for ap in sorted(art, key=lambda x: x["part_id"]))
            print(f"  Critical junction parts: {art_str}")

        seq = plan.optimized_sequence
        print(f"\n  Optimised sequence: {seq.summary()}")
        print(f"  {'#':<3} {'Part':<32} {'Dir':<6} {'Tools':<22} {'Time(s)':>7}")
        print(f"  {'-'*3} {'-'*32} {'-'*6} {'-'*22} {'-'*7}")
        cumul = 0.0
        for i, step in enumerate(plan.optimized_steps(), start=1):
            t = step.get("time_s", 0.0)
            cumul += t
            print(
                f"  {i:<3} {step['part_name']:<32} "
                f"{step.get('direction','—'):<6} "
                f"{step.get('tools','—'):<22} {t:.1f}s"
            )
        print(f"  {'':64} cumul: {cumul:.1f}s")
        print()


# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(description="Real-assembly DFMA + resource test")
    parser.add_argument("--verbose",  "-v", action="store_true")
    parser.add_argument("--no-plan",        action="store_true",
                        help="Skip assembly planning (faster)")
    args = parser.parse_args()

    run_plan = not args.no_plan

    print(f"\n{BOLD}{'═'*70}")
    print("  REAL ASSEMBLY TESTS")
    print(f"{'═'*70}{RESET}")

    # ── Test 1: PN16 flange joint ──────────────────────────────────────────────
    print(f"\n{BOLD}[1/2] PN16 FLANGE JOINT DN50{RESET}  (DIN EN 1092-1)")
    run_analysis(
        assembly  = build_flange_assembly(),
        fasteners = build_flange_fasteners(),
        planner   = build_flange_planner(),
        verbose   = args.verbose,
        run_plan  = run_plan,
    )

    # ── Test 2: UCP205 pillow-block bearing ────────────────────────────────────
    print(f"\n{BOLD}[2/2] UCP205 PILLOW-BLOCK BEARING UNIT{RESET}  (SKF/FYH INA)")
    run_analysis(
        assembly  = build_bearing_assembly(),
        fasteners = build_bearing_fasteners(),
        planner   = build_bearing_planner(),
        verbose   = args.verbose,
        run_plan  = run_plan,
    )

    print(f"\n{BOLD}{'═'*70}")
    print("  ALL TESTS COMPLETE")
    print(f"{'═'*70}{RESET}\n")


if __name__ == "__main__":
    main()
