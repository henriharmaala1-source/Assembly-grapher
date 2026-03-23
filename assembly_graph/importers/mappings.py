"""
mappings.py — translation tables between CAD system vocabulary and the
dfma/assembly_graph data model enums.

  map_material(cad_name)           → Material enum
  map_process_from_geometry(...)   → ManufacturingProcess enum
  map_sw_mate(int)                 → (contact_type, strength)
  map_nx_constraint(str)           → (contact_type, strength)
  upgrade_to_press_fit(...)        → post-process concentric mates
"""

from __future__ import annotations

from dfma.models.part import Material, ManufacturingProcess


# ── material keyword matching ─────────────────────────────────────────────────

_MATERIAL_KEYWORDS: list[tuple[list[str], Material]] = [
    (["steel", "iron", "cast iron", "alumin", "alum", "aluminum", "copper",
      "brass", "bronze", "titanium", "zinc", "nickel", "alloy", "metal",
      "stainless", "carbon steel", "tool steel", "spring steel"],
     Material.METAL),
    (["abs", "nylon", "pa6", "pa12", "pa66", "pp", "pe", "hdpe", "ldpe",
      "pvc", "polypropylene", "polyethylene", "polycarbonate", "pc",
      "acetal", "delrin", "pom", "pmma", "acrylic", "peek",
      "plastic", "polymer", "resin", "epoxy"],
     Material.PLASTIC),
    (["rubber", "epdm", "silicone", "elastomer", "nbr", "neoprene",
      "viton", "teflon", "ptfe", "tpu", "natural rubber", "latex"],
     Material.RUBBER),
]


def map_material(cad_name: str) -> Material:
    """Map a CAD material name string to a Material enum value."""
    if not cad_name:
        return Material.UNKNOWN
    lower = cad_name.lower()
    for keywords, mat in _MATERIAL_KEYWORDS:
        if any(k in lower for k in keywords):
            return mat
    return Material.UNKNOWN


# ── process estimation from geometry ─────────────────────────────────────────

def map_process_from_geometry(
    material: Material,
    min_wall_mm: float,
    tolerance_mm: float,
    draft_angle_deg: float,
    is_fastener: bool = False,
) -> ManufacturingProcess:
    """
    Heuristic: infer most likely manufacturing process from geometry parameters.
    Used when the CAD file provides no explicit process data (e.g. STEP, JSON
    with no process field).
    """
    if is_fastener:
        return ManufacturingProcess.CNC_MACHINING

    if material == Material.RUBBER:
        return ManufacturingProcess.INJECTION_MOLDING

    if material == Material.PLASTIC:
        return ManufacturingProcess.INJECTION_MOLDING

    # Sheet metal: thin wall, no significant draft
    if 0.5 <= min_wall_mm <= 4.0 and draft_angle_deg < 0.5:
        return ManufacturingProcess.SHEET_METAL

    # Casting indicators: draft angle present
    if draft_angle_deg >= 1.0:
        if min_wall_mm >= 6.0:
            return ManufacturingProcess.SAND_CASTING
        return ManufacturingProcess.DIE_CASTING

    # Tight tolerance → machined
    if tolerance_mm <= 0.05:
        return ManufacturingProcess.CNC_MACHINING

    if tolerance_mm <= 0.2:
        return ManufacturingProcess.CNC_MACHINING

    return ManufacturingProcess.UNKNOWN


# ── SolidWorks mate types → (contact_type, strength) ─────────────────────────
# swMateType_e enum values (SolidWorks API)

_SW_MATE: dict[int, tuple[str, str]] = {
    0:  ("face",      "rigid"),     # swMateCOINCIDENT
    1:  ("face",      "rigid"),     # swMateCONCENTRIC   (may upgrade to press_fit)
    2:  ("face",      "rigid"),     # swMatePERPENDICULAR
    3:  ("face",      "rigid"),     # swMatePARALLEL
    4:  ("face",      "sliding"),   # swMateTANGENT
    5:  ("face",      "rigid"),     # swMateDistanceDim
    6:  ("face",      "rigid"),     # swMateAngleDim
    7:  ("face",      "rigid"),     # swMateCAMFOLLOWER
    8:  ("face",      "sliding"),   # swMateGEARSLIDING
    9:  ("thread",    "rigid"),     # swMateSCREW
    10: ("face",      "rigid"),     # swMateRACK_PINION
    12: ("face",      "rigid"),     # swMateSYMMETRIC
    20: ("press_fit", "rigid"),     # swMateWIDTH (symmetric, treated as press_fit)
    23: ("face",      "rigid"),     # swMateHINGE
    24: ("face",      "sliding"),   # swMateGEAR
    25: ("face",      "rigid"),     # swMateUNIVERSALJOINT
}

def map_sw_mate(mate_type_int: int) -> tuple[str, str]:
    """Map a swMateType_e integer to (contact_type, strength)."""
    return _SW_MATE.get(mate_type_int, ("face", "rigid"))


# ── Siemens NX constraint types → (contact_type, strength) ───────────────────
# NXOpen.Positioning.Constraint.Type string values

_NX_CONSTRAINT: dict[str, tuple[str, str]] = {
    "Touch":            ("face",      "rigid"),
    "Align":            ("face",      "rigid"),
    "Concentric":       ("face",      "rigid"),   # may upgrade to press_fit
    "Distance":         ("face",      "rigid"),
    "Angle":            ("face",      "rigid"),
    "Fix":              ("face",      "fixed"),
    "Tangent":          ("face",      "sliding"),
    "Parallel":         ("face",      "rigid"),
    "Perpendicular":    ("face",      "rigid"),
    "Center":           ("face",      "rigid"),
    "Fit":              ("press_fit", "rigid"),
    "BondedContact":    ("face",      "rigid"),
    "Glued":            ("face",      "rigid"),
    "InferredContact":  ("face",      "rigid"),
}

def map_nx_constraint(constraint_type_str: str) -> tuple[str, str]:
    """Map an NX Constraint.Type string to (contact_type, strength)."""
    return _NX_CONSTRAINT.get(constraint_type_str, ("face", "rigid"))


# ── thread/pitch detection ─────────────────────────────────────────────────────

def looks_like_thread_mate(
    contact_type: str,
    part_a_name:  str,
    part_b_name:  str,
) -> bool:
    """
    Upgrade heuristic: promote a face/rigid contact to thread/rigid if the
    part names suggest a threaded connection (e.g. 'bolt', 'nut', 'screw',
    'stud', 'thread').
    """
    if contact_type == "thread":
        return True
    thread_words = {"bolt", "nut", "screw", "stud", "thread", "fastener",
                    "m3", "m4", "m5", "m6", "m8", "m10", "m12", "m16",
                    "m20", "m24"}
    combined = (part_a_name + " " + part_b_name).lower()
    return any(w in combined for w in thread_words)
