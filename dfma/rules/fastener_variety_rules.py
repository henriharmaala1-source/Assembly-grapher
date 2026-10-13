"""
Fastener Variety / Standardization Rules (FVS).

Checks whether fasteners within the same subassembly zone use consistent
sizes, thread pitches, and drive types.  Poor fastener standardization causes:

  - Tool proliferation  : each unique drive type needs a separate tool
  - Picking errors      : similar-looking bolts of different sizes/pitches
                          can be confused during assembly or service
  - Torque spec spread  : every diameter needs its own torque value
  - Re-work cost        : cross-threaded or under-torqued joints from mix-ups

Fasteners are grouped by their ``subassembly_id`` field.  A group with
fewer than ``min_group_size`` (default 2) fasteners is skipped.

Rules
─────
  FVS-001  Diameter variety       — too many distinct shaft diameters in one zone
  FVS-002  Drive-type variety     — too many distinct wrench/bit types required
  FVS-003  Mixed thread pitch     — same nominal diameter used with different pitches
  FVS-004  Low standardization    — unique-size ratio too high (each bolt different)

Thresholds
──────────
  FVS-001 : > 2 diameters → WARNING ; > 3 diameters → ERROR
  FVS-002 : > 1 drive type → WARNING ; > 2 drive types → ERROR
  FVS-003 : any mixing of pitches for the same diameter → WARNING
  FVS-004 : unique_diameters/total > 0.5 → WARNING (n ≥ 3) ;
            ratio == 1.0 (every bolt a different size, n ≥ 3) → ERROR

References
──────────
  Boothroyd & Dewhurst, "Product Design for Manufacture and Assembly", §fastener
  Lucas DFA method — fastener standardization guideline
  Toyota Production System — single-bolt-size principle per sub-station
"""

from __future__ import annotations
from collections import defaultdict

from ..models.fastener import FastenerSpec
from ..models.warning  import Warning, Severity


# ── thresholds ────────────────────────────────────────────────────────────────

MAX_DIAMETERS_WARNING   = 2      # > 2 distinct diameters → WARNING
MAX_DIAMETERS_ERROR     = 3      # > 3 distinct diameters → ERROR

MAX_DRIVE_TYPES_WARNING = 1      # > 1 distinct drive type → WARNING
MAX_DRIVE_TYPES_ERROR   = 2      # > 2 distinct drive types → ERROR

STD_RATIO_WARNING = 0.5          # unique_dia / n > 0.5 → WARNING  (n ≥ 3)
STD_RATIO_ERROR   = 1.0          # unique_dia / n = 1.0 → ERROR    (n ≥ 3)


# ── public API ────────────────────────────────────────────────────────────────

def check_fastener_variety(
    fasteners:      list[FastenerSpec],
    min_group_size: int = 2,
) -> list[Warning]:
    """
    Analyse fastener standardization within each subassembly zone.

    Only fasteners whose ``subassembly_id`` is non-empty are considered.
    Groups smaller than ``min_group_size`` are skipped (no meaningful variety
    analysis possible for a single fastener).

    Parameters
    ----------
    fasteners :
        All FastenerSpec objects in the assembly.
    min_group_size :
        Minimum group size to trigger analysis (default 2).

    Returns
    -------
    list[Warning]
        FVS-001 … FVS-004 warnings sorted by group ID.
    """
    groups = _group_by_subassembly(fasteners)
    warnings: list[Warning] = []
    for gid in sorted(groups):
        members = groups[gid]
        if len(members) < min_group_size:
            continue
        warnings.extend(_check_group(gid, members))
    return warnings


# ── internals ─────────────────────────────────────────────────────────────────

def _group_by_subassembly(
    fasteners: list[FastenerSpec],
) -> dict[str, list[FastenerSpec]]:
    groups: dict[str, list[FastenerSpec]] = defaultdict(list)
    for fs in fasteners:
        gid = (fs.subassembly_id or "").strip()
        if gid:
            groups[gid].append(fs)
    return dict(groups)


def _check_group(group_id: str, members: list[FastenerSpec]) -> list[Warning]:
    warnings: list[Warning] = []
    n = len(members)

    diameters   = sorted({fs.nominal_diameter_mm for fs in members})
    drive_types = {fs.drive_type for fs in members}
    n_dia       = len(diameters)
    n_drv       = len(drive_types)

    # ── FVS-001: Diameter variety ─────────────────────────────────────────────
    dia_str = ", ".join(f"M{d:g}" for d in diameters)

    if n_dia > MAX_DIAMETERS_ERROR:
        warnings.append(Warning(
            rule_id="FVS-001",
            severity=Severity.ERROR,
            part_id=group_id,
            message=(
                f"Zone '{group_id}': {n_dia} distinct bolt diameters "
                f"({dia_str}) — exceeds the recommended maximum of "
                f"{MAX_DIAMETERS_WARNING}."
            ),
            suggestion=(
                "Redesign to use at most two nominal diameters in this zone. "
                "Different sizes look identical at a glance; mis-installation "
                "causes joint failure, stripped threads, and rework."
            ),
            metric_value=float(n_dia),
            threshold=float(MAX_DIAMETERS_WARNING),
        ))
    elif n_dia > MAX_DIAMETERS_WARNING:
        warnings.append(Warning(
            rule_id="FVS-001",
            severity=Severity.WARNING,
            part_id=group_id,
            message=(
                f"Zone '{group_id}': {n_dia} distinct bolt diameters ({dia_str})."
            ),
            suggestion=(
                "Standardize to one diameter per zone where possible to reduce "
                "tool changes and bin picking errors."
            ),
            metric_value=float(n_dia),
            threshold=float(MAX_DIAMETERS_WARNING),
        ))

    # ── FVS-002: Drive-type variety ───────────────────────────────────────────
    drv_str = ", ".join(sorted(d.value for d in drive_types))

    if n_drv > MAX_DRIVE_TYPES_ERROR:
        warnings.append(Warning(
            rule_id="FVS-002",
            severity=Severity.ERROR,
            part_id=group_id,
            message=(
                f"Zone '{group_id}': {n_drv} different driver types required "
                f"({drv_str}). Operator must change tools {n_drv - 1} time(s) "
                "within this zone."
            ),
            suggestion=(
                "Standardize all fasteners in this zone to the same drive type "
                "(TORX or SOCKET_CAP preferred — best torque, no cam-out). "
                "Multiple driver types increase cycle time and tool-change errors."
            ),
            metric_value=float(n_drv),
            threshold=float(MAX_DRIVE_TYPES_WARNING),
        ))
    elif n_drv > MAX_DRIVE_TYPES_WARNING:
        warnings.append(Warning(
            rule_id="FVS-002",
            severity=Severity.WARNING,
            part_id=group_id,
            message=(
                f"Zone '{group_id}': {n_drv} driver types required ({drv_str}), "
                "causing at least one tool change."
            ),
            suggestion=(
                "Use the same drive type for all fasteners in this zone to "
                "allow continuous assembly without re-gripping a different driver."
            ),
            metric_value=float(n_drv),
            threshold=float(MAX_DRIVE_TYPES_WARNING),
        ))

    # ── FVS-003: Mixed thread pitch for the same nominal diameter ─────────────
    pitch_by_dia: dict[float, set[float]] = defaultdict(set)
    for fs in members:
        pitch_by_dia[fs.nominal_diameter_mm].add(fs.thread_pitch_mm)

    for dia, pitches in sorted(pitch_by_dia.items()):
        if len(pitches) > 1:
            pitch_str = " mm, ".join(f"{p:.2f}" for p in sorted(pitches)) + " mm"
            warnings.append(Warning(
                rule_id="FVS-003",
                severity=Severity.WARNING,
                part_id=group_id,
                message=(
                    f"Zone '{group_id}': M{dia:g} fasteners have {len(pitches)} "
                    f"different thread pitches ({pitch_str}). "
                    "Coarse and fine-pitch bolts are visually identical."
                ),
                suggestion=(
                    "Use only one thread pitch per nominal diameter throughout "
                    "the assembly. Mixed pitches cannot be distinguished by sight "
                    "and cause cross-threading, incorrect torque specs, and "
                    "field-service errors."
                ),
                metric_value=float(len(pitches)),
                threshold=1.0,
            ))

    # ── FVS-004: Low standardization ratio ────────────────────────────────────
    if n >= 3:
        std_ratio = n_dia / n
        if std_ratio >= STD_RATIO_ERROR:
            warnings.append(Warning(
                rule_id="FVS-004",
                severity=Severity.ERROR,
                part_id=group_id,
                message=(
                    f"Zone '{group_id}': zero standardization — "
                    f"all {n} fasteners are different sizes ({dia_str}). "
                    "Each bolt requires its own bin, torque value, and driver."
                ),
                suggestion=(
                    "Target ≤ 2 distinct fastener sizes per zone. "
                    "A zone with n unique sizes for n fasteners has the highest "
                    "possible picking-error rate and longest tool-change overhead."
                ),
                metric_value=std_ratio,
                threshold=STD_RATIO_WARNING,
            ))
        elif std_ratio > STD_RATIO_WARNING:
            warnings.append(Warning(
                rule_id="FVS-004",
                severity=Severity.WARNING,
                part_id=group_id,
                message=(
                    f"Zone '{group_id}': standardization ratio {std_ratio:.0%} "
                    f"({n_dia} unique sizes across {n} fasteners)."
                ),
                suggestion=(
                    "Aim for ≤ 2 distinct fastener sizes per zone to minimise "
                    "tool changes, bin proliferation, and picking errors."
                ),
                metric_value=std_ratio,
                threshold=STD_RATIO_WARNING,
            ))

    return warnings
