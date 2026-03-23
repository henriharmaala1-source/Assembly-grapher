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

Run with:
    python dfma_demo.py
    python dfma_demo.py --verbose
"""

import argparse
from dfma.models.part    import Part, Assembly, Geometry, ManufacturingProcess, Material
from dfma.models.warning import Severity
from dfma.analyzer       import analyze


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

    assembly = build_pneumatic_valve()
    result   = analyze(assembly)
    print_report(result, verbose=args.verbose)


if __name__ == "__main__":
    main()
