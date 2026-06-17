"""
Screw / Bolt Obstruction Checker (DFS — Design for Fastening / Service rules).

Checks whether a fastener can physically be inserted into and removed from
an assembly given the available clearance in the assembled context.

Rules
─────
  DFS-001  Axial tool clearance          (above head, for driver/socket)
  DFS-002  Radial tool clearance         (around head, for wrench/socket swing)
  DFS-003  Insertion path length         (screw must travel full length to seat)
  DFS-004  Removal path length           (screw must travel full length to exit)
  DFS-005  Thread engagement length      (too short → pull-out; too long → bottoms out)
  DFS-006  Hole diameter vs shaft        (clearance hole too tight or too loose)
  DFS-007  Countersink geometry          (flush fit check for CSK screws)
  DFS-008  Lateral bolt-pitch clearance  (adjacent screws too close for tool swing)
  DFS-009  Post-assembly obstruction     (parts fitted AFTER screw now block removal)
  DFS-010  Total grip vs screw length    (screw too short to clamp + engage threads)

References
──────────
  ISO 4762 (socket cap screws), ISO 4014 (hex bolts), ISO 2768 (tolerances),
  Boothroyd & Dewhurst DFA tool-access guidelines,
  DFMPro fastener clearance rules (HCL Geometric).
"""

from ..models.fastener import FastenerSpec, TOOL_ENVELOPE, HeadStyle
from ..models.warning  import Warning, Severity

# Insertion path safety margin (mm) beyond screw length
INSERTION_MARGIN_MM = 3.0

# Recommended thread engagement: min 1.0× and ideal 1.5× nominal diameter
MIN_THREAD_ENGAGEMENT_RATIO   = 1.0
IDEAL_THREAD_ENGAGEMENT_RATIO = 1.5

# Hole diameter tolerances
MAX_CLEARANCE_RATIO = 1.25   # hole > 1.25× nominal diameter → too loose (risk of shift)
MIN_CLEARANCE_RATIO = 1.01   # hole < 1.01× nominal diameter → interference (can't insert)

# Minimum lateral clearance between adjacent fasteners for tool swing (mm)
# = roughly one head-diameter extra space to swing/position tool
MIN_LATERAL_PITCH_FACTOR = 1.5   # pitch between fastener centres >= 1.5 × head_diameter


def rule_dfs001_axial_tool_clearance(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-001: Is there enough height above the screw head for the driving tool?

    The tool must fit axially above the head before it can engage.
    Minimum = TOOL_ENVELOPE[drive_type][0] (e.g. 8 mm for socket cap, 10 mm for hex).
    """
    min_axial, _ = TOOL_ENVELOPE.get(fs.drive_type, (10.0, 4.0))
    if fs.axial_clearance_above_head_mm < min_axial:
        return [Warning(
            rule_id="DFS-001",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': axial tool clearance above head is "
                f"{fs.axial_clearance_above_head_mm:.1f} mm — "
                f"minimum {min_axial:.0f} mm required for a "
                f"{fs.drive_type.value} driver."
            ),
            suggestion=(
                f"Increase clearance above head to ≥ {min_axial:.0f} mm, "
                "or switch to a lower-profile drive type (e.g. socket cap → Torx) "
                "to reduce the required tool envelope."
            ),
            metric_value=fs.axial_clearance_above_head_mm,
            threshold=min_axial,
        )]
    return []


def rule_dfs002_radial_tool_clearance(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-002: Is there enough radial space around the head for the tool?

    A socket or open-ended spanner needs to slip over the head.
    Required clearance beyond the head edge = TOOL_ENVELOPE[drive_type][1].
    """
    _, min_radial = TOOL_ENVELOPE.get(fs.drive_type, (10.0, 4.0))
    if fs.radial_clearance_mm < min_radial:
        severity = (
            Severity.ERROR if fs.radial_clearance_mm < min_radial * 0.5
            else Severity.WARNING
        )
        return [Warning(
            rule_id="DFS-002",
            severity=severity,
            part_id=fs.id,
            message=(
                f"'{fs.name}': radial clearance around head is "
                f"{fs.radial_clearance_mm:.1f} mm — "
                f"minimum {min_radial:.0f} mm needed beyond the head edge "
                f"for {fs.drive_type.value}."
            ),
            suggestion=(
                f"Provide ≥ {min_radial:.0f} mm of radial space around "
                f"the head (head Ø {fs.head_diameter_mm:.1f} mm + "
                f"{min_radial:.0f} mm each side). "
                "Consider recessing the fastener or using a reduced-head variant."
            ),
            metric_value=fs.radial_clearance_mm,
            threshold=min_radial,
        )]
    return []


def rule_dfs003_insertion_path(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-003: Is the straight-line insertion path long enough?

    The screw tip-to-head-top length = total_length_mm + head_height_mm.
    The insertion path must be at least this long plus a safety margin so
    the screw can be held, aligned, and driven into position.
    """
    required = fs.total_length_mm + fs.head_height_mm + INSERTION_MARGIN_MM
    if fs.insertion_path_length_mm < required:
        return [Warning(
            rule_id="DFS-003",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': insertion path {fs.insertion_path_length_mm:.1f} mm "
                f"is too short. Need ≥ {required:.1f} mm "
                f"(length {fs.total_length_mm:.1f} + head {fs.head_height_mm:.1f} "
                f"+ {INSERTION_MARGIN_MM:.0f} mm margin)."
            ),
            suggestion=(
                "Lengthen the insertion clearance, use a shorter screw, "
                "or redesign the assembly to give straight-in access."
            ),
            metric_value=fs.insertion_path_length_mm,
            threshold=required,
        )]
    return []


def rule_dfs004_removal_path(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-004: Is the withdrawal path clear for removal?

    Same geometry as insertion, but the context may differ if parts were
    assembled after the fastener — those parts may now obstruct withdrawal.
    Uses effective_removal_path which can be set to a shorter value when
    post-assembly parts reduce the available exit space.
    """
    required = fs.total_length_mm + fs.head_height_mm + INSERTION_MARGIN_MM
    removal  = fs.effective_removal_path

    warnings = []
    if removal < required:
        extra = ""
        if fs.parts_assembled_after:
            extra = (
                f" Parts assembled after this fastener "
                f"({', '.join(fs.parts_assembled_after)}) "
                "may be blocking withdrawal."
            )
        warnings.append(Warning(
            rule_id="DFS-004",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': removal path {removal:.1f} mm is insufficient "
                f"(need ≥ {required:.1f} mm).{extra}"
            ),
            suggestion=(
                "Ensure the fastener can be fully withdrawn before adjacent "
                "parts are installed, or redesign the assembly sequence so the "
                "fastener is the last item installed and first removed."
            ),
            metric_value=removal,
            threshold=required,
        ))
    return warnings


def rule_dfs005_thread_engagement(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-005: Is thread engagement length adequate?

    Min  = 1.0 × nominal_diameter  (absolute minimum — high-stress applications)
    Ideal= 1.5 × nominal_diameter  (recommended for standard joints)
    Too long: screw bottoms out in a blind hole before head is seated.
    """
    warnings = []
    min_eng  = MIN_THREAD_ENGAGEMENT_RATIO  * fs.nominal_diameter_mm
    ideal_eng= IDEAL_THREAD_ENGAGEMENT_RATIO * fs.nominal_diameter_mm

    # Check if screw bottoms out: thread_engagement > total_length
    # (more engagement required than screw has thread length)
    if fs.thread_engagement_mm > fs.total_length_mm:
        warnings.append(Warning(
            rule_id="DFS-005",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': required thread engagement "
                f"{fs.thread_engagement_mm:.1f} mm exceeds total screw "
                f"length {fs.total_length_mm:.1f} mm — screw will bottom "
                "out before head is seated."
            ),
            suggestion=(
                "Use a longer screw or reduce the required thread engagement "
                "depth. For blind holes, ensure ≥ 2 thread pitches of clearance "
                "at the bottom."
            ),
            metric_value=fs.thread_engagement_mm,
            threshold=fs.total_length_mm,
        ))
    elif fs.thread_engagement_mm < min_eng:
        warnings.append(Warning(
            rule_id="DFS-005",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': thread engagement {fs.thread_engagement_mm:.1f} mm "
                f"is below the minimum {min_eng:.1f} mm "
                f"(1.0 × M{fs.nominal_diameter_mm:.0f}). "
                "Risk of thread pull-out under load."
            ),
            suggestion=(
                f"Increase thread engagement to ≥ {ideal_eng:.1f} mm "
                f"(1.5 × nominal Ø). Use a longer screw or deeper tapped hole."
            ),
            metric_value=fs.thread_engagement_mm,
            threshold=min_eng,
        ))
    elif fs.thread_engagement_mm < ideal_eng:
        warnings.append(Warning(
            rule_id="DFS-005",
            severity=Severity.WARNING,
            part_id=fs.id,
            message=(
                f"'{fs.name}': thread engagement {fs.thread_engagement_mm:.1f} mm "
                f"is below the recommended {ideal_eng:.1f} mm "
                f"(1.5 × M{fs.nominal_diameter_mm:.0f})."
            ),
            suggestion=(
                f"Aim for ≥ {ideal_eng:.1f} mm thread engagement for "
                "reliable load-bearing joints."
            ),
            metric_value=fs.thread_engagement_mm,
            threshold=ideal_eng,
        ))
    return warnings


def rule_dfs006_hole_diameter(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-006: Is the clearance hole diameter compatible with the shaft?

    Too tight (< 1.01× nominal): shaft cannot be inserted.
    Too loose (> 1.25× nominal): risk of positional shift and reduced clamping.
    """
    warnings = []
    d = fs.nominal_diameter_mm
    min_hole = MIN_CLEARANCE_RATIO * d
    max_hole = MAX_CLEARANCE_RATIO * d

    if fs.hole_diameter_mm < min_hole:
        warnings.append(Warning(
            rule_id="DFS-006",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': clearance hole Ø {fs.hole_diameter_mm:.2f} mm "
                f"is smaller than screw shaft Ø {d:.1f} mm — "
                "screw cannot be inserted."
            ),
            suggestion=(
                f"Use a clearance hole of at least Ø {min_hole:.2f} mm "
                f"(H12/H13 fit for M{d:.0f})."
            ),
            metric_value=fs.hole_diameter_mm,
            threshold=min_hole,
        ))
    elif fs.hole_diameter_mm > max_hole:
        warnings.append(Warning(
            rule_id="DFS-006",
            severity=Severity.WARNING,
            part_id=fs.id,
            message=(
                f"'{fs.name}': clearance hole Ø {fs.hole_diameter_mm:.2f} mm "
                f"is {fs.hole_diameter_mm / d:.2f}× the nominal Ø {d:.1f} mm — "
                "excessive play, risk of positional shift."
            ),
            suggestion=(
                f"Reduce hole to ≤ Ø {max_hole:.2f} mm, "
                "or add a shoulder/locating feature."
            ),
            metric_value=fs.hole_diameter_mm,
            threshold=max_hole,
        ))
    return warnings


def rule_dfs007_countersink_geometry(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-007: Countersunk screw flush-fit check.

    For countersunk (flush) heads:
    - countersink_diameter must be >= head_diameter (head must seat fully)
    - If countersink_diameter == 0 for a CSK head style → ERROR (no seat)
    """
    if fs.head_style not in (HeadStyle.COUNTERSUNK, HeadStyle.RAISED_CSK):
        return []

    warnings = []
    if fs.countersink_diameter_mm <= 0:
        warnings.append(Warning(
            rule_id="DFS-007",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}' has a countersunk head style but no countersink "
                "is defined in the assembly. The head will protrude above the surface."
            ),
            suggestion=(
                "Add a countersink with diameter ≥ head Ø "
                f"({fs.head_diameter_mm:.1f} mm) at the correct angle "
                f"({fs.countersink_angle_deg:.0f}°)."
            ),
        ))
    elif fs.countersink_diameter_mm < fs.head_diameter_mm:
        warnings.append(Warning(
            rule_id="DFS-007",
            severity=Severity.WARNING,
            part_id=fs.id,
            message=(
                f"'{fs.name}': countersink Ø {fs.countersink_diameter_mm:.1f} mm "
                f"< head Ø {fs.head_diameter_mm:.1f} mm — head will not sit flush."
            ),
            suggestion=(
                f"Increase countersink to Ø ≥ {fs.head_diameter_mm + 0.2:.1f} mm."
            ),
            metric_value=fs.countersink_diameter_mm,
            threshold=fs.head_diameter_mm,
        ))
    return warnings


def rule_dfs008_lateral_bolt_pitch(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-008: Bolt-circle / bolt-pitch lateral clearance.

    When multiple fasteners are arranged in a bolt circle or row, adjacent
    fasteners must be spaced far enough apart for the driving tool to engage
    without colliding with a neighbour's head or shaft.

    Minimum centre-to-centre pitch = head_diameter × MIN_LATERAL_PITCH_FACTOR.
    lateral_obstruction_mm is the distance from this fastener's axis to the
    nearest obstruction (adjacent fastener head edge, wall, or feature).
    """
    if fs.lateral_obstruction_mm <= 0:
        return []   # no lateral constraint specified

    min_pitch = fs.head_diameter_mm * MIN_LATERAL_PITCH_FACTOR
    # lateral_obstruction is measured from head edge, so convert to centre dist
    centre_to_obstruction = fs.head_diameter_mm / 2 + fs.lateral_obstruction_mm

    if centre_to_obstruction < min_pitch:
        return [Warning(
            rule_id="DFS-008",
            severity=Severity.WARNING,
            part_id=fs.id,
            message=(
                f"'{fs.name}': bolt pitch too tight. Centre-to-obstruction "
                f"{centre_to_obstruction:.1f} mm < minimum pitch "
                f"{min_pitch:.1f} mm ({MIN_LATERAL_PITCH_FACTOR}× head Ø). "
                "Tool cannot swing/position between adjacent fasteners."
            ),
            suggestion=(
                "Increase bolt-circle radius / fastener spacing so "
                f"centre-to-centre ≥ {min_pitch:.1f} mm, "
                "or use a reduced-profile head (button, socket cap) with a "
                "smaller head diameter."
            ),
            metric_value=centre_to_obstruction,
            threshold=min_pitch,
        )]
    return []


def rule_dfs009_post_assembly_obstruction(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-009: Parts assembled after this fastener that block its removal.

    If parts_assembled_after is non-empty AND removal_path_length_mm is
    explicitly shorter than insertion_path_length_mm, those parts have
    reduced the withdrawal clearance.  Flag even if DFS-004 passes, because
    the designer may not have noticed the sequencing dependency.
    """
    if not fs.parts_assembled_after:
        return []

    warnings = []
    if (fs.removal_path_length_mm > 0 and
            fs.removal_path_length_mm < fs.insertion_path_length_mm):
        warnings.append(Warning(
            rule_id="DFS-009",
            severity=Severity.WARNING,
            part_id=fs.id,
            message=(
                f"'{fs.name}': parts assembled after this fastener "
                f"({', '.join(fs.parts_assembled_after)}) "
                f"reduce the removal path from "
                f"{fs.insertion_path_length_mm:.1f} mm to "
                f"{fs.removal_path_length_mm:.1f} mm. "
                "Service/disassembly may be restricted."
            ),
            suggestion=(
                "Verify the removal path still meets the minimum "
                f"({fs.total_length_mm + fs.head_height_mm + INSERTION_MARGIN_MM:.1f} mm). "
                "If not, redesign the assembly sequence so fasteners can be "
                "accessed before adjacent components are removed."
            ),
        ))
    elif fs.parts_assembled_after:
        warnings.append(Warning(
            rule_id="DFS-009",
            severity=Severity.INFO,
            part_id=fs.id,
            message=(
                f"'{fs.name}' has {len(fs.parts_assembled_after)} part(s) "
                f"assembled after it: {', '.join(fs.parts_assembled_after)}. "
                "Verify these do not obstruct removal during service."
            ),
            suggestion=(
                "Document the disassembly sequence. "
                "Consider whether these parts need to be removed before "
                "this fastener can be accessed."
            ),
        ))
    return warnings


def rule_dfs010_grip_vs_length(fs: FastenerSpec) -> list[Warning]:
    """
    DFS-010: Total screw length vs grip thickness + thread engagement.

    The screw must be long enough to:
      1. Pass through the clamped material (grip_thickness_mm)
      2. Provide adequate thread engagement (ideally 1.5 × nominal_diameter)

    If total_length < grip + thread_engagement → screw is too short.
    """
    required_length = fs.grip_thickness_mm + fs.thread_engagement_mm
    warnings = []
    if fs.total_length_mm < required_length:
        warnings.append(Warning(
            rule_id="DFS-010",
            severity=Severity.ERROR,
            part_id=fs.id,
            message=(
                f"'{fs.name}': screw length {fs.total_length_mm:.1f} mm is too short. "
                f"Grip {fs.grip_thickness_mm:.1f} mm + engagement "
                f"{fs.thread_engagement_mm:.1f} mm = "
                f"{required_length:.1f} mm required."
            ),
            suggestion=(
                f"Use a screw of length ≥ {required_length + 2:.0f} mm "
                "(adding 2 mm margin for thread run-out)."
            ),
            metric_value=fs.total_length_mm,
            threshold=required_length,
        ))
    return warnings


# ── top-level checker ────────────────────────────────────────────────────────

def check_fastener(fs: FastenerSpec) -> list[Warning]:
    """Run all DFS rules against a single FastenerSpec. Returns list of Warnings."""
    warnings: list[Warning] = []
    warnings.extend(rule_dfs001_axial_tool_clearance(fs))
    warnings.extend(rule_dfs002_radial_tool_clearance(fs))
    warnings.extend(rule_dfs003_insertion_path(fs))
    warnings.extend(rule_dfs004_removal_path(fs))
    warnings.extend(rule_dfs005_thread_engagement(fs))
    warnings.extend(rule_dfs006_hole_diameter(fs))
    warnings.extend(rule_dfs007_countersink_geometry(fs))
    warnings.extend(rule_dfs008_lateral_bolt_pitch(fs))
    warnings.extend(rule_dfs009_post_assembly_obstruction(fs))
    warnings.extend(rule_dfs010_grip_vs_length(fs))

    sev = {"ERROR": 0, "WARNING": 1, "INFO": 2}
    warnings.sort(key=lambda w: sev[w.severity.value])
    return warnings


def check_fasteners(fasteners: list[FastenerSpec]) -> list[Warning]:
    """Run all DFS rules across a list of fasteners."""
    all_warnings: list[Warning] = []
    for fs in fasteners:
        all_warnings.extend(check_fastener(fs))
    return all_warnings
