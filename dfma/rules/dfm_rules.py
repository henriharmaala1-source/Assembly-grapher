"""
DFM (Design for Manufacturability) Rule Checker.

Rules are organised by manufacturing process and cover:
  - Minimum wall thickness
  - Draft angles
  - Fillet / corner radius
  - Dimensional tolerances
  - Undercuts
  - Rib geometry (injection moulding)

Thresholds are sourced from:
  - Boothroyd & Dewhurst (2010) "Product Design for Manufacture and Assembly"
  - DFMPro guidelines (HCL / Geometric)
  - NADCA die casting standards
  - General CNC machining DFM guides
"""

from ..models.part    import Part, Geometry, ManufacturingProcess, Material
from ..models.warning import Warning, Severity


# ══════════════════════════════════════════════════════════════════════════════
# Threshold tables (by process)
# ══════════════════════════════════════════════════════════════════════════════

# Minimum wall thickness (mm) per process × material
MIN_WALL_THICKNESS: dict[ManufacturingProcess, dict[Material, float]] = {
    ManufacturingProcess.INJECTION_MOLDING: {
        Material.PLASTIC: 1.0,
        Material.UNKNOWN: 1.0,
    },
    ManufacturingProcess.CNC_MACHINING: {
        Material.METAL:   0.762,   # 0.030 in
        Material.PLASTIC: 1.524,   # 0.060 in
        Material.UNKNOWN: 1.0,
    },
    ManufacturingProcess.DIE_CASTING: {
        Material.METAL:   1.016,   # 0.040 in  (NADCA min)
        Material.UNKNOWN: 1.016,
    },
    ManufacturingProcess.SAND_CASTING: {
        Material.METAL:   3.0,
        Material.UNKNOWN: 3.0,
    },
    ManufacturingProcess.SHEET_METAL: {
        Material.METAL:   0.5,
        Material.UNKNOWN: 0.5,
    },
}

# Minimum draft angle (degrees) per process
MIN_DRAFT_ANGLE: dict[ManufacturingProcess, float] = {
    ManufacturingProcess.INJECTION_MOLDING: 1.0,
    ManufacturingProcess.DIE_CASTING:       0.5,
    ManufacturingProcess.SAND_CASTING:      1.0,
}

# Minimum internal fillet radius (mm)  — as fraction of wall thickness
# rule: fillet_radius >= FILLET_RATIO × min_wall_thickness
FILLET_WALL_RATIO: dict[ManufacturingProcess, float] = {
    ManufacturingProcess.INJECTION_MOLDING: 0.5,    # inside radius ≥ 50% wall
    ManufacturingProcess.DIE_CASTING:       0.25,
    ManufacturingProcess.SAND_CASTING:      0.3,
    ManufacturingProcess.CNC_MACHINING:     0.1,    # tool radius constraint
}

# Tight-tolerance warning thresholds (mm)
TOLERANCE_WARNING: dict[ManufacturingProcess, float] = {
    ManufacturingProcess.CNC_MACHINING:     0.05,   # tighter than ±0.05 mm is costly
    ManufacturingProcess.INJECTION_MOLDING: 0.1,
    ManufacturingProcess.DIE_CASTING:       0.05,
    ManufacturingProcess.SAND_CASTING:      0.5,
    ManufacturingProcess.SHEET_METAL:       0.1,
}
TOLERANCE_ERROR: dict[ManufacturingProcess, float] = {
    ManufacturingProcess.CNC_MACHINING:     0.005,
    ManufacturingProcess.INJECTION_MOLDING: 0.025,
    ManufacturingProcess.DIE_CASTING:       0.013,
    ManufacturingProcess.SAND_CASTING:      0.2,
    ManufacturingProcess.SHEET_METAL:       0.05,
}


# ══════════════════════════════════════════════════════════════════════════════
# Individual DFM rules
# ══════════════════════════════════════════════════════════════════════════════

def rule_dfm001_wall_thickness(part: Part) -> list[Warning]:
    """
    DFM-001: Minimum wall thickness check.
    Thin walls cause incomplete fill (moulding), chatter (machining),
    or cold shuts (casting).
    """
    geo = part.geometry
    if geo.min_wall_thickness <= 0:
        return []   # not specified — skip

    thresholds = MIN_WALL_THICKNESS.get(part.process, {})
    min_t = thresholds.get(part.material) or thresholds.get(Material.UNKNOWN)
    if min_t is None:
        return []

    if geo.min_wall_thickness < min_t:
        return [Warning(
            rule_id="DFM-001",
            severity=Severity.ERROR,
            part_id=part.id,
            message=(
                f"'{part.name}' wall thickness {geo.min_wall_thickness:.2f} mm "
                f"is below the minimum {min_t:.2f} mm for "
                f"{part.process.value} / {part.material.value}."
            ),
            suggestion=(
                f"Increase minimum wall thickness to at least {min_t:.2f} mm. "
                "Add ribs for stiffness rather than increasing uniform thickness."
            ),
            metric_value=geo.min_wall_thickness,
            threshold=min_t,
        )]
    return []


def rule_dfm002_draft_angle(part: Part) -> list[Warning]:
    """
    DFM-002: Missing or insufficient draft angle.
    Parts without adequate draft on vertical walls cannot be ejected
    from moulds or dies without damage.
    Applies to: injection moulding, die casting, sand casting.
    """
    geo = part.geometry
    min_draft = MIN_DRAFT_ANGLE.get(part.process)
    if min_draft is None:
        return []   # process does not require draft

    if geo.draft_angle <= 0:
        return [Warning(
            rule_id="DFM-002",
            severity=Severity.ERROR,
            part_id=part.id,
            message=(
                f"'{part.name}' has no draft angle specified. "
                f"{part.process.value} requires ≥ {min_draft}° draft on vertical faces."
            ),
            suggestion=(
                f"Add a minimum {min_draft}° draft to all faces parallel to the "
                "mould draw direction. For textured surfaces add 1° per 0.025 mm depth."
            ),
        )]

    if geo.draft_angle < min_draft:
        return [Warning(
            rule_id="DFM-002",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' draft angle {geo.draft_angle:.1f}° is below "
                f"the recommended {min_draft}° for {part.process.value}."
            ),
            suggestion=(
                f"Increase draft angle to ≥ {min_draft}°. "
                "Insufficient draft causes part sticking and surface drag marks."
            ),
            metric_value=geo.draft_angle,
            threshold=min_draft,
        )]
    return []


def rule_dfm003_fillet_radius(part: Part) -> list[Warning]:
    """
    DFM-003: Internal fillet radius too small.
    Sharp internal corners create stress concentrations, impede material
    flow in casting/moulding, and cause premature tool wear in machining.
    """
    geo = part.geometry
    if geo.fillet_radius <= 0 or geo.min_wall_thickness <= 0:
        return []

    ratio = FILLET_WALL_RATIO.get(part.process)
    if ratio is None:
        return []

    min_r = ratio * geo.min_wall_thickness

    if geo.fillet_radius < min_r:
        severity = (
            Severity.ERROR if geo.fillet_radius < min_r * 0.5
            else Severity.WARNING
        )
        return [Warning(
            rule_id="DFM-003",
            severity=severity,
            part_id=part.id,
            message=(
                f"'{part.name}' internal fillet radius {geo.fillet_radius:.2f} mm "
                f"is below the minimum {min_r:.2f} mm "
                f"({ratio:.0%} × wall={geo.min_wall_thickness:.2f} mm) "
                f"for {part.process.value}."
            ),
            suggestion=(
                f"Increase internal corner radii to ≥ {min_r:.2f} mm. "
                "For CNC, use the largest tool that fits the pocket geometry."
            ),
            metric_value=geo.fillet_radius,
            threshold=min_r,
        )]
    return []


def rule_dfm004_tolerance(part: Part) -> list[Warning]:
    """
    DFM-004: Over-tight dimensional tolerances.
    Tolerances below process capability inflate cost by 40–80% per feature
    and increase rejection rates.
    """
    geo = part.geometry
    if geo.tolerance <= 0:
        return []

    warn_t  = TOLERANCE_WARNING.get(part.process)
    error_t = TOLERANCE_ERROR.get(part.process)

    warnings = []
    if error_t and geo.tolerance < error_t:
        warnings.append(Warning(
            rule_id="DFM-004",
            severity=Severity.ERROR,
            part_id=part.id,
            message=(
                f"'{part.name}' tolerance ±{geo.tolerance:.4f} mm is beyond "
                f"normal {part.process.value} capability "
                f"(practical limit ≈ ±{error_t:.4f} mm). "
                "This will require special equipment and inspection."
            ),
            suggestion=(
                "Relax tolerance if functionally acceptable. "
                "If the tight tolerance is required, specify on that feature only "
                "and note the inspection method."
            ),
            metric_value=geo.tolerance,
            threshold=error_t,
        ))
    elif warn_t and geo.tolerance < warn_t:
        warnings.append(Warning(
            rule_id="DFM-004",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' tolerance ±{geo.tolerance:.4f} mm is tighter "
                f"than recommended ±{warn_t:.4f} mm for {part.process.value}. "
                "This increases cost significantly."
            ),
            suggestion=(
                "Use ±0.1 mm or looser wherever function allows. "
                "Tight tolerances should be reserved for critical fit surfaces."
            ),
            metric_value=geo.tolerance,
            threshold=warn_t,
        ))
    return warnings


def rule_dfm005_undercuts(part: Part) -> list[Warning]:
    """
    DFM-005: Undercuts require side-actions in moulds/dies or
    multi-step machining — both add significant cost.
    """
    if part.geometry.has_undercuts:
        return [Warning(
            rule_id="DFM-005",
            severity=Severity.WARNING,
            part_id=part.id,
            message=(
                f"'{part.name}' has undercuts. "
                "Undercuts require side-actions (moulding/casting) or "
                "multi-axis toolpaths (CNC), adding 20–50% to tooling cost."
            ),
            suggestion=(
                "Redesign to eliminate undercuts: split the part, "
                "use a different parting line, or replace with a separate clip."
            ),
        )]
    return []


def rule_dfm006_wall_uniformity(part: Part) -> list[Warning]:
    """
    DFM-006: Non-uniform wall thickness.
    Detected as large ratio between max dimension and min wall thickness.
    Differential cooling/shrinkage causes warpage and sink marks.
    A simple proxy: if min_wall_thickness < 50% of the average dimension,
    flag for review.
    """
    geo = part.geometry
    dims = [d for d in [geo.length, geo.width, geo.height] if d > 0]
    if not dims or geo.min_wall_thickness <= 0:
        return []

    avg_dim = sum(dims) / len(dims)
    if geo.min_wall_thickness < 0.005 * avg_dim:   # < 0.5% of avg dimension
        return [Warning(
            rule_id="DFM-006",
            severity=Severity.INFO,
            part_id=part.id,
            message=(
                f"'{part.name}' minimum wall ({geo.min_wall_thickness:.2f} mm) "
                f"is very thin relative to part size ({avg_dim:.1f} mm avg). "
                "Non-uniform walls cause warpage and sink marks."
            ),
            suggestion=(
                "Aim for uniform wall thickness (±10% variation). "
                "Use gradual tapers (3:1 slope) when thickness must change."
            ),
            metric_value=geo.min_wall_thickness,
            threshold=0.005 * avg_dim,
        )]
    return []


# ══════════════════════════════════════════════════════════════════════════════
# Top-level DFM check
# ══════════════════════════════════════════════════════════════════════════════

def check_part_dfm(part: Part) -> list[Warning]:
    """Run all DFM rules against a single part."""
    warnings: list[Warning] = []
    warnings.extend(rule_dfm001_wall_thickness(part))
    warnings.extend(rule_dfm002_draft_angle(part))
    warnings.extend(rule_dfm003_fillet_radius(part))
    warnings.extend(rule_dfm004_tolerance(part))
    warnings.extend(rule_dfm005_undercuts(part))
    warnings.extend(rule_dfm006_wall_uniformity(part))
    return warnings


def check_assembly_dfm(assembly) -> list[Warning]:
    """Run DFM rules across all parts in an assembly."""
    warnings: list[Warning] = []
    for part in assembly.all_parts():
        warnings.extend(check_part_dfm(part))
    return warnings
