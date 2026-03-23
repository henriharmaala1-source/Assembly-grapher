"""
DFMA Warning System — demonstration with a realistic example assembly.

Example: Pneumatic Valve Assembly
  Components:
    1. Valve Body         (die cast aluminium)
    2. Valve Spool        (CNC machined steel, must move)
    3. Return Spring      (steel, must move — flexible + tangling)
    4. End Cap            (die cast aluminium)
    5. O-Ring x2          (rubber, must differ material — flexible)
    6. Mounting Screw x4  (fastener, steel)
    7. Retaining Clip     (sheet metal — could be eliminated)

  Fasteners (DFS checks):
    F001  M4 Socket-Cap (end cap, tight axial clearance)
    F002  M6 Hex Bolt   (body mount, loose clearance hole)
    F003  M5 Countersunk (cover plate, missing countersink)
    F004  M4 Socket-Cap (tight bolt pitch, neighbour too close)
    F005  M3 Socket-Cap (screw too short for grip + engagement)
    F006  M8 Hex Bolt   (post-assembly obstruction by cable bracket)

Run with:
    python dfma_demo.py
    python dfma_demo.py --verbose
"""

import argparse
from dfma.models.part     import Part, Assembly, Geometry, ManufacturingProcess, Material
from dfma.models.fastener import FastenerSpec, DriveType, HeadStyle
from dfma.models.warning  import Severity
from dfma.analyzer        import analyze


# ── Build the example assembly ──────────────────────────────────────────────

def build_pneumatic_valve() -> Assembly:
    valve_body = Part(
        id="P001", name="Valve Body",
        process=ManufacturingProcess.DIE_CASTING,
        material=Material.METAL,
        must_be_separate=True,
        geometry=Geometry(
            length=120, width=60, height=45,
            min_wall_thickness=2.5,
            fillet_radius=0.8,
            draft_angle=0.4,          # ← below the 0.5° minimum → DFM-002 WARNING
            tolerance=0.05,
            alpha=90, beta=90,        # ← low symmetry → DFA-002 WARNING
            mass_grams=480,
        ),
    )

    valve_spool = Part(
        id="P002", name="Valve Spool",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        must_move=True,               # ← functionally necessary: slides
        geometry=Geometry(
            length=80, width=18, height=18,
            min_wall_thickness=3.0,
            fillet_radius=0.5,
            tolerance=0.01,           # ← tight tolerance → DFM-004 WARNING
            alpha=360, beta=180,      # cylindrical — partially symmetric
            mass_grams=95,
            requires_alignment=True,  # ← must align ports → DFA-006 INFO
        ),
    )

    return_spring = Part(
        id="P003", name="Return Spring",
        process=ManufacturingProcess.UNKNOWN,
        material=Material.METAL,
        must_move=True,               # ← compresses/extends
        geometry=Geometry(
            length=60, width=15, height=15,
            min_wall_thickness=1.5,
            alpha=360, beta=360,
            mass_grams=12,
            is_flexible=True,         # ← spring deforms in handling
            can_tangle=True,          # ← coil springs tangle → DFA-003 WARNING
        ),
    )

    end_cap = Part(
        id="P004", name="End Cap",
        process=ManufacturingProcess.DIE_CASTING,
        material=Material.METAL,
        # ← no must_move / must_differ_material / must_be_separate
        # candidate for elimination if body redesigned → DFA-001 WARNING
        geometry=Geometry(
            length=60, width=60, height=12,
            min_wall_thickness=1.8,
            fillet_radius=0.3,
            draft_angle=0.6,
            tolerance=0.1,
            alpha=180, beta=180,
            mass_grams=110,
        ),
    )

    oring = Part(
        id="P005", name="O-Ring",
        process=ManufacturingProcess.INJECTION_MOLDING,
        material=Material.RUBBER,
        must_differ_material=True,    # ← must be rubber for sealing
        quantity=2,
        geometry=Geometry(
            length=20, width=20, height=4,
            min_wall_thickness=2.0,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=3,
            is_flexible=True,         # ← rubber ring deforms
        ),
    )

    mounting_screw = Part(
        id="P006", name="M4 Mounting Screw",
        process=ManufacturingProcess.CNC_MACHINING,
        material=Material.METAL,
        quantity=4,
        geometry=Geometry(
            length=20, width=4, height=4,
            min_wall_thickness=1.0,
            tolerance=0.05,
            alpha=360, beta=360,
            mass_grams=4,
            is_fastener=True,
            fastener_clearance_mm=4.0,  # ← below 6 mm → DFA-005 ERROR
        ),
    )

    retaining_clip = Part(
        id="P007", name="Retaining Clip",
        process=ManufacturingProcess.SHEET_METAL,
        material=Material.METAL,
        # ← no functional necessity flags → DFA-001 WARNING
        geometry=Geometry(
            length=30, width=15, height=2,
            min_wall_thickness=0.4,   # ← below 0.5 mm sheet metal min → DFM-001 ERROR
            fillet_radius=0.2,
            tolerance=0.08,
            alpha=180, beta=0,        # asymmetric in β → DFA-002 WARNING
            mass_grams=5,
            has_undercuts=True,       # ← bent tab creates undercut → DFM-005
        ),
    )

    assembly = Assembly(
        id="ASM001",
        name="Pneumatic Valve Assembly",
        parts=[valve_body, valve_spool, return_spring, end_cap,
               oring, mounting_screw, retaining_clip],
        assembly_directions=3,        # ← body, then cap from side, clip from top
    )
    return assembly


# ── Build fastener specs for DFS checks ────────────────────────────────────

def build_fasteners() -> list[FastenerSpec]:
    return [
        # F001 — End-cap M4 socket cap: axial clearance too tight (housing rib above)
        FastenerSpec(
            id="F001", name="M4 End-Cap Socket Screw",
            nominal_diameter_mm=4.0, total_length_mm=16.0,
            head_diameter_mm=7.0,    head_height_mm=4.0,
            thread_pitch_mm=0.7,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.SOCKET_CAP,
            hole_diameter_mm=4.3,
            grip_thickness_mm=8.0,   thread_engagement_mm=6.0,
            axial_clearance_above_head_mm=5.0,   # ← 5 mm < 8 mm min → DFS-001 ERROR
            radial_clearance_mm=4.0,             # ← fine for socket cap (min 3)
            insertion_path_length_mm=30.0,
        ),

        # F002 — Body-mount M6 hex bolt: clearance hole too loose
        FastenerSpec(
            id="F002", name="M6 Body-Mount Hex Bolt",
            nominal_diameter_mm=6.0, total_length_mm=25.0,
            head_diameter_mm=10.0,   head_height_mm=4.0,
            thread_pitch_mm=1.0,
            drive_type=DriveType.HEX_BOLT, head_style=HeadStyle.HEX,
            hole_diameter_mm=9.0,            # ← 9.0 / 6.0 = 1.5× → too loose → DFS-006
            grip_thickness_mm=15.0,  thread_engagement_mm=9.0,
            axial_clearance_above_head_mm=20.0,
            radial_clearance_mm=8.0,
            insertion_path_length_mm=45.0,
        ),

        # F003 — Cover-plate M5 countersunk: countersink missing in assembly
        FastenerSpec(
            id="F003", name="M5 Cover-Plate CSK Screw",
            nominal_diameter_mm=5.0, total_length_mm=20.0,
            head_diameter_mm=9.5,    head_height_mm=3.0,
            thread_pitch_mm=0.8,
            drive_type=DriveType.TORX, head_style=HeadStyle.COUNTERSUNK,
            hole_diameter_mm=5.2,
            countersink_diameter_mm=0.0,     # ← no countersink defined → DFS-007 ERROR
            grip_thickness_mm=10.0,  thread_engagement_mm=8.0,
            axial_clearance_above_head_mm=12.0,
            radial_clearance_mm=3.0,
            insertion_path_length_mm=35.0,
        ),

        # F004 — Bolt-circle M4 socket cap: adjacent screw too close (tight pitch)
        FastenerSpec(
            id="F004", name="M4 Bolt-Circle Socket Screw",
            nominal_diameter_mm=4.0, total_length_mm=12.0,
            head_diameter_mm=7.0,    head_height_mm=4.0,
            thread_pitch_mm=0.7,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.SOCKET_CAP,
            hole_diameter_mm=4.3,
            grip_thickness_mm=6.0,   thread_engagement_mm=6.0,
            axial_clearance_above_head_mm=15.0,
            radial_clearance_mm=5.0,
            insertion_path_length_mm=25.0,
            lateral_obstruction_mm=2.5,      # ← centre-to-obstruction 6.0 mm < min 10.5 → DFS-008
        ),

        # F005 — M3 screw too short for grip + engagement
        FastenerSpec(
            id="F005", name="M3 Sensor-Bracket Screw",
            nominal_diameter_mm=3.0, total_length_mm=8.0,
            head_diameter_mm=5.5,    head_height_mm=3.0,
            thread_pitch_mm=0.5,
            drive_type=DriveType.SOCKET_CAP, head_style=HeadStyle.SOCKET_CAP,
            hole_diameter_mm=3.2,
            grip_thickness_mm=6.0,   thread_engagement_mm=4.5,  # 6+4.5=10.5 > 8 → DFS-010
            axial_clearance_above_head_mm=10.0,
            radial_clearance_mm=4.0,
            insertion_path_length_mm=20.0,
        ),

        # F006 — M8 hex bolt: cable bracket fitted after blocks removal
        FastenerSpec(
            id="F006", name="M8 Main-Body Hex Bolt",
            nominal_diameter_mm=8.0, total_length_mm=35.0,
            head_diameter_mm=13.0,   head_height_mm=5.5,
            thread_pitch_mm=1.25,
            drive_type=DriveType.HEX_BOLT, head_style=HeadStyle.HEX,
            hole_diameter_mm=8.4,
            grip_thickness_mm=20.0,  thread_engagement_mm=12.0,
            axial_clearance_above_head_mm=18.0,
            radial_clearance_mm=7.0,
            insertion_path_length_mm=50.0,
            removal_path_length_mm=15.0,          # ← cable bracket installed after → DFS-004
            parts_assembled_after=["Cable Bracket", "Conduit Clip"],
        ),
    ]


# ── Report renderer ──────────────────────────────────────────────────────────

SEVERITY_COLOUR = {
    Severity.ERROR:   "\033[91m",   # red
    Severity.WARNING: "\033[93m",   # yellow
    Severity.INFO:    "\033[96m",   # cyan
}
RESET = "\033[0m"


def print_report(result, verbose: bool = False) -> None:
    sep = "─" * 72

    print(f"\n{'═' * 72}")
    print(f"  DFMA ANALYSIS REPORT")
    print(f"{'═' * 72}")
    print(result.summary())

    # ── DFA Index interpretation ─────────────────────────────────────────
    idx = result.dfa_index
    if idx >= 0.60:
        grade = "GOOD  — well-optimised for assembly"
    elif idx >= 0.35:
        grade = "FAIR  — moderate improvement potential"
    else:
        grade = "POOR  — significant redesign recommended"
    print(f"  DFA Grade: {grade}")
    print()

    # ── Warnings grouped by severity ─────────────────────────────────────
    for severity in (Severity.ERROR, Severity.WARNING, Severity.INFO):
        items = [w for w in result.warnings if w.severity == severity]
        if not items:
            continue
        colour = SEVERITY_COLOUR[severity]
        print(f"{sep}")
        print(f"{colour}  {severity.value}S ({len(items)}){RESET}")
        print(sep)
        for w in items:
            print(f"{colour}  [{w.rule_id}] {w.part_id}: {w.message}{RESET}")
            if verbose and w.suggestion:
                print(f"    → {w.suggestion}")
            if w.metric_value is not None and w.threshold is not None:
                print(f"    value={w.metric_value:.4g}  threshold={w.threshold:.4g}")
        print()

    print(f"{'═' * 72}\n")


# ── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="DFMA Warning System — Pneumatic Valve Assembly demo"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Show corrective suggestions for each warning"
    )
    args = parser.parse_args()

    assembly  = build_pneumatic_valve()
    fasteners = build_fasteners()
    result    = analyze(assembly, fasteners=fasteners)
    print_report(result, verbose=args.verbose)


if __name__ == "__main__":
    main()
