"""
DFA (Design for Assembly) Rule Checker.

Rules are based on:
  - Boothroyd & Dewhurst DFA methodology
  - Lucas DFA guidelines
  - General industry best practices

Each check function receives a Part and returns a list of Warning objects.
The top-level check_assembly() function runs all rules and computes the
DFA Index and estimated assembly time.
"""

from ..models.part      import Part, Assembly
from ..models.warning   import Warning, Severity, AnalysisResult
from .geometry_scorer   import (
    estimate_total_time,
    IDEAL_ASSEMBLY_TIME,
)


# ── individual part-level DFA rules ────────────────────────────────────────

def rule_dfa001_functional_necessity(part: Part) -> list[Warning]:
    """
    DFA-001: Unnecessary part detection.
    A part that does not need to move, differ in material, or be separate
    for service is a candidate for elimination or consolidation.
    Reference: Boothroyd & Dewhurst minimum-part criterion (3 questions).
    """
    if not (part.must_move or part.must_differ_material or part.must_be_separate):
        return [Warning(
            rule_id="DFA-001",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' may be unnecessary. "
                "It does not need to move, use a different material, "
                "or be separate for assembly/service access."
            ),
            suggestion=(
                "Consider consolidating with an adjacent part via "
                "redesign, casting-in, or snap-fit integration."
            ),
        )]
    return []


def rule_dfa002_symmetry(part: Part) -> list[Warning]:
    """
    DFA-002: Low rotational symmetry increases handling time and error risk.
    Fully symmetric parts (α=360°, β=360°) have the lowest handling time.
    Parts with no symmetry (α<45°, β<45°) are expensive to orient.
    Reference: B&D symmetry classification table.
    """
    geo = part.geometry
    warnings = []
    if geo.alpha < 45.0 and geo.beta < 45.0:
        warnings.append(Warning(
            rule_id="DFA-002",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' has very low rotational symmetry "
                f"(α={geo.alpha}°, β={geo.beta}°), significantly "
                "increasing handling time and mis-assembly risk."
            ),
            suggestion=(
                "Maximise symmetry where possible (α→360, β→360), "
                "or add prominent asymmetric features (pins, flats, chamfers) "
                "to make the correct orientation obvious."
            ),
            metric_value=min(geo.alpha, geo.beta),
            threshold=45.0,
        ))
    return warnings


def rule_dfa003_handling_difficulty(part: Part) -> list[Warning]:
    """
    DFA-003: Flags parts with multiple handling difficulty attributes.
    Parts that are flexible, tangling, fragile, and need two hands
    compound the handling time penalty severely.
    """
    geo = part.geometry
    flags = []
    if geo.is_flexible:    flags.append("flexible")
    if geo.can_tangle:     flags.append("can tangle")
    if geo.is_fragile:     flags.append("fragile")
    if geo.needs_2_hands:  flags.append("requires two hands")

    warnings = []
    if len(flags) >= 2:
        warnings.append(Warning(
            rule_id="DFA-003",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' has {len(flags)} handling difficulty "
                f"attributes: {', '.join(flags)}. This multiplies handling time."
            ),
            suggestion=(
                "Redesign to reduce handling difficulty: add stiffening ribs "
                "to reduce flexibility, use coated/packaged delivery to avoid "
                "tangling, or add guides to eliminate two-hand requirement."
            ),
        ))
    elif len(flags) == 1:
        warnings.append(Warning(
            rule_id="DFA-003",
            severity=Severity.INFO,
            part_id=part.id,
            message=f"'{part.name}' has a handling difficulty: {flags[0]}.",
            suggestion="Review whether this attribute can be designed out.",
        ))
    return warnings


def rule_dfa004_fastener_count(part: Part) -> list[Warning]:
    """
    DFA-004: Fasteners are one of the highest-cost assembly operations.
    Each screw adds ~8–16 s of insertion time vs ~1.5 s for a snap-fit.
    Reference: B&D insertion time tables; general DFA guideline.
    """
    if part.is_fastener if hasattr(part, 'is_fastener') else part.geometry.is_fastener:
        return [Warning(
            rule_id="DFA-004",
            severity=Severity.INFO,
            part_id=part.id,
            message=(
                f"'{part.name}' is a fastener. Fasteners typically add "
                "8–16 s of insertion time each."
            ),
            suggestion=(
                "Consider replacing with integral snap-fits, clips, or tabs "
                "to reduce assembly time by 5–10× per joint."
            ),
        )]
    return []


def rule_dfa005_fastener_clearance(part: Part) -> list[Warning]:
    """
    DFA-005: Insufficient clearance around fastener head obstructs tooling.
    Minimum 6 mm clearance recommended around standard fastener heads.
    Reference: DFMPro DFA guidelines; general mechanical design practice.
    """
    geo = part.geometry
    MIN_CLEARANCE = 6.0   # mm
    warnings = []
    if geo.is_fastener and 0 < geo.fastener_clearance_mm < MIN_CLEARANCE:
        warnings.append(Warning(
            rule_id="DFA-005",
            severity=Severity.ERROR,
            part_id=part.id,
            message=(
                f"'{part.name}' has only {geo.fastener_clearance_mm:.1f} mm "
                f"tool clearance around the fastener head "
                f"(minimum {MIN_CLEARANCE} mm required)."
            ),
            suggestion=(
                "Increase clearance to at least 6 mm or use a low-profile "
                "fastener with appropriate driver access."
            ),
            metric_value=geo.fastener_clearance_mm,
            threshold=MIN_CLEARANCE,
        ))
    return warnings


def rule_dfa006_obstructed_insertion(part: Part) -> list[Warning]:
    """
    DFA-006: Obstructed insertion point raises insertion time by ~80%.
    Reference: B&D insertion code table — restricted access penalty.
    """
    if part.geometry.obstructed_access:
        return [Warning(
            rule_id="DFA-006",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' has an obstructed insertion point, "
                "increasing insertion time by ~80%."
            ),
            suggestion=(
                "Redesign assembly access: use longer tools, move obstructing "
                "parts to a later assembly step, or redesign housing for "
                "unobstructed straight-in insertion."
            ),
        )]
    return []


def rule_dfa007_assembly_direction(assembly: Assembly) -> list[Warning]:
    """
    DFA-007: Multiple assembly directions increase fixture changes and cycle time.
    Best practice: all parts assembled from one direction (top-down preferred).
    Reference: Boothroyd & Dewhurst DFA guideline #6.
    """
    if assembly.assembly_directions > 1:
        return [Warning(
            rule_id="DFA-007",
            severity=Severity.WARNING,
            part_id=assembly.id,
            message=(
                f"Assembly '{assembly.name}' uses {assembly.assembly_directions} "
                "assembly directions. Multiple directions require re-fixturing."
            ),
            suggestion=(
                "Redesign so all parts are added from a single direction "
                "(preferably top-down, using gravity). "
                "Eliminate horizontal or bottom-up insertions."
            ),
            metric_value=float(assembly.assembly_directions),
            threshold=1.0,
        )]
    return []


def rule_dfa008_large_part_count(assembly: Assembly) -> list[Warning]:
    """
    DFA-008: High part count vs theoretical minimum indicates consolidation potential.
    DFA efficiency target: ratio N_min/N_total > 0.5 (ideally > 0.7).

    Skipped when n_min == 0 (no necessity metadata — e.g. raw STEP import).
    """
    warnings = []
    parts = assembly.all_parts()
    n_total = len(parts)
    n_min   = assembly.theoretical_minimum_parts()

    if n_total == 0 or n_min == 0:
        return []

    ratio = n_min / n_total
    if ratio < 0.3:
        warnings.append(Warning(
            rule_id="DFA-008",
            severity=Severity.WARNING,
            part_id=assembly.id,
            message=(
                f"Assembly '{assembly.name}' has a low part-count efficiency: "
                f"{n_min} functionally necessary parts out of {n_total} total "
                f"({ratio:.0%}). High consolidation potential."
            ),
            suggestion=(
                "Apply the Boothroyd-Dewhurst 3-question test to each part: "
                "Does it move? Is a different material needed? Must it be "
                "separate for service? Combine all parts failing all 3 tests."
            ),
            metric_value=ratio,
            threshold=0.3,
        ))
    elif ratio < 0.5:
        warnings.append(Warning(
            rule_id="DFA-008",
            severity=Severity.INFO,
            part_id=assembly.id,
            message=(
                f"Assembly '{assembly.name}': {n_min}/{n_total} parts are "
                f"functionally necessary ({ratio:.0%}). "
                "Some consolidation opportunity exists."
            ),
            suggestion="Review non-essential parts for snap-fit or over-moulding integration.",
            metric_value=ratio,
            threshold=0.5,
        ))
    return warnings


# ── assembly-level analysis ────────────────────────────────────────────────

def check_assembly(assembly: Assembly) -> AnalysisResult:
    """
    Run all DFA rules against the assembly and return an AnalysisResult.

    Computes:
      - per-part handling + insertion time (Boothroyd-Dewhurst model)
      - total estimated assembly time
      - DFA Index  = (N_min × 2.93) / T_total

    When no functional-necessity metadata is available (e.g. raw STEP import
    without a BOM sidecar), DFA-001 and DFA-008 are suppressed to avoid
    flooding the report with misleading warnings.  A single INFO note is
    emitted instead, and N_min is set equal to N_total so the DFA Index
    reflects handling/insertion difficulty rather than part-count efficiency.
    """
    result = AnalysisResult(assembly_name=assembly.name)
    warnings: list[Warning] = []

    parts = assembly.all_parts()

    # Detect whether any part carries explicit necessity metadata.
    # Raw STEP imports have all flags False (defaulted); JSON BOMs set them.
    has_necessity_metadata = any(
        p.must_move or p.must_differ_material or p.must_be_separate
        for p in parts
    )

    # ── assembly-level rules ─────────────────────────────────────────────
    warnings.extend(rule_dfa007_assembly_direction(assembly))

    if has_necessity_metadata:
        warnings.extend(rule_dfa008_large_part_count(assembly))

    # ── part-level rules ─────────────────────────────────────────────────
    total_time = 0.0
    for part in parts:
        t = estimate_total_time(part.geometry) * part.quantity
        total_time += t

        if has_necessity_metadata:
            warnings.extend(rule_dfa001_functional_necessity(part))
        warnings.extend(rule_dfa002_symmetry(part))
        warnings.extend(rule_dfa003_handling_difficulty(part))
        warnings.extend(rule_dfa004_fastener_count(part))
        warnings.extend(rule_dfa005_fastener_clearance(part))
        warnings.extend(rule_dfa006_obstructed_insertion(part))

    if not has_necessity_metadata and parts:
        warnings.append(Warning(
            rule_id="DFA-001",
            severity=Severity.INFO,
            part_id=assembly.id,
            message=(
                "Functional-necessity metadata not available for this assembly "
                "(imported from STEP without a BOM sidecar file). "
                "Rules DFA-001 (part elimination) and DFA-008 (part-count "
                "efficiency) are suppressed to avoid misleading results."
            ),
            suggestion=(
                "Create a JSON BOM sidecar with must_move, must_differ_material, "
                "and must_be_separate flags per part to unlock full DFA scoring."
            ),
        ))

    # ── DFA Index ─────────────────────────────────────────────────────────
    n_total   = sum(p.quantity for p in parts)
    n_min     = assembly.theoretical_minimum_parts()
    # When no necessity metadata exists, treat all parts as necessary so the
    # DFA Index reflects handling/insertion efficiency rather than showing 0 %.
    if n_min == 0 and n_total > 0:
        n_min = n_total
    dfa_index = (n_min * IDEAL_ASSEMBLY_TIME) / total_time if total_time > 0 else 0.0

    result.warnings              = warnings
    result.total_parts           = n_total
    result.theoretical_minimum   = n_min
    result.total_assembly_time_s = round(total_time, 2)
    result.dfa_index             = round(dfa_index, 4)

    return result
