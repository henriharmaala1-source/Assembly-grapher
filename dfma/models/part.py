"""
Part and Assembly data models for DFMA analysis.

Geometry values are based on the Boothroyd-Dewhurst DFA classification system:
  - alpha (α): rotational symmetry about the insertion axis (0–360°)
  - beta  (β): rotational symmetry about an axis perpendicular to insertion (0–360°)

Symmetry rules (Boothroyd & Dewhurst):
  alpha=360, beta=360  → fully symmetric (e.g. sphere, plain cylinder)
  alpha=180, beta=360  → one axis of symmetry
  alpha=0,   beta=0    → no symmetry (requires precise orientation)

Handling time baseline (seconds) — simplified from B&D tables:
  base_handling_time ≈ 1.13 s  (fully symmetric, easy grip, small part)
  penalty added for:  low symmetry, large/heavy, flexible, tangling, fragile
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ManufacturingProcess(str, Enum):
    INJECTION_MOLDING = "injection_molding"
    CNC_MACHINING     = "cnc_machining"
    DIE_CASTING       = "die_casting"
    SAND_CASTING      = "sand_casting"
    SHEET_METAL       = "sheet_metal"
    UNKNOWN           = "unknown"


class Material(str, Enum):
    METAL   = "metal"
    PLASTIC = "plastic"
    RUBBER  = "rubber"
    UNKNOWN = "unknown"


@dataclass
class Geometry:
    """
    Geometric properties of a part used for DFA/DFM scoring.

    Dimensions in millimetres; angles in degrees; mass in grams.
    """
    # ── spatial dimensions ─────────────────────────────────────────────────
    length: float = 0.0          # longest dimension (mm)
    width:  float = 0.0          # second dimension  (mm)
    height: float = 0.0          # third  dimension  (mm)

    # ── wall / feature geometry ────────────────────────────────────────────
    min_wall_thickness: float = 0.0    # thinnest wall (mm)
    fillet_radius:      float = 0.0    # smallest internal corner radius (mm)
    draft_angle:        float = 0.0    # smallest draft angle on vertical faces (°)
    tolerance:          float = 0.1    # tightest dimensional tolerance (mm)

    # ── mass & handling ────────────────────────────────────────────────────
    mass_grams: float = 0.0

    # ── Boothroyd-Dewhurst symmetry angles ────────────────────────────────
    alpha: float = 360.0   # rotational symmetry about insertion axis (0–360)
    beta:  float = 360.0   # rotational symmetry about perpendicular axis (0–360)

    # ── handling difficulty flags ──────────────────────────────────────────
    is_flexible:  bool = False   # part bends/deforms during handling
    can_tangle:   bool = False   # part nests or tangles with identical parts
    is_fragile:   bool = False   # part may break during handling
    needs_2_hands: bool = False  # requires two hands to handle

    # ── insertion difficulty flags ─────────────────────────────────────────
    obstructed_access: bool = False   # difficult to reach insertion point
    requires_alignment: bool = False  # needs precise angular alignment before insert
    has_undercuts:      bool = False  # geometry prevents straight-pull mould/machining

    # ── fastener info (if this part IS a fastener) ─────────────────────────
    is_fastener: bool = False
    fastener_clearance_mm: float = 0.0   # tool clearance around fastener head


@dataclass
class Part:
    """A single component in an assembly."""
    id:   str
    name: str

    process:  ManufacturingProcess = ManufacturingProcess.UNKNOWN
    material: Material             = Material.UNKNOWN
    geometry: Geometry             = field(default_factory=Geometry)

    # ── functional necessity flags (for theoretical minimum part count) ────
    # A part is "necessary" if at least one of these is True:
    #   1. it must move relative to assembled parts
    #   2. it must be a different material from adjacent parts
    #   3. it must be separate for assembly / service access
    must_move:              bool = False
    must_differ_material:   bool = False
    must_be_separate:       bool = False

    # ── assembly context ───────────────────────────────────────────────────
    quantity: int = 1   # number of instances in the assembly


@dataclass
class Assembly:
    """
    A hierarchical assembly node — may contain parts and sub-assemblies.
    Mirrors the structure extracted from SolidWorks / NX.
    """
    id:   str
    name: str
    parts:       list[Part]     = field(default_factory=list)
    subassemblies: "list[Assembly]" = field(default_factory=list)

    # ── assembly-level context ─────────────────────────────────────────────
    # number of distinct assembly directions used in this assembly
    assembly_directions: int = 1

    def all_parts(self) -> list[Part]:
        """Flatten the full part list including all sub-assemblies."""
        result = list(self.parts)
        for sub in self.subassemblies:
            result.extend(sub.all_parts())
        return result

    def theoretical_minimum_parts(self) -> int:
        """
        Count parts that are functionally necessary.
        A part is necessary if it must move, differ in material, or be separate.
        Source: Boothroyd & Dewhurst DFA minimum-part criterion.
        """
        return sum(
            1 for p in self.all_parts()
            if p.must_move or p.must_differ_material or p.must_be_separate
        )
