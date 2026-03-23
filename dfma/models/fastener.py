"""
Fastener geometry models for screw/bolt obstruction checking.

A fastener obstruction check requires knowing:
  1. The fastener's own physical dimensions
  2. The space available in the assembly at its location
  3. The tool required to drive/remove it

Coordinate convention
─────────────────────
  +Z  = insertion direction (screw goes down into the assembly)
  X/Y = radial plane around the screw axis

Clearance spaces measured in the assembled state (all other parts in place):

    ┌───────────────────┐
    │  ← radial_mm →   │  ← axial clearance above head (tool must fit here)
    │    ┌───────┐      │  ─── head top surface
    │    │  HEAD │      │  ← head_height_mm
    │    └───┬───┘      │  ─── mating surface / countersink seat
    │        │ shaft    │
    │        │ ← nominal_diameter_mm
    │        │          │  ← grip_thickness_mm  (clamped material)
    │        │~~~~~~~~~│  ─── thread start
    │        │/////////│  ← thread_engagement_mm  (in threaded hole/nut)
    └───────────────────┘

Axial insertion path
─────────────────────
  The screw must travel (total_length_mm + head_height_mm) in the +Z direction
  before reaching its seated position.  If anything blocks that path the
  screw cannot be inserted.

  insertion_path_length_mm: straight-line clearance in the insertion axis
                            from the start of approach to the mating surface,
                            measured in the assembled context (all prior parts
                            in place, the fastener NOT yet present).
"""

from dataclasses import dataclass, field
from enum import Enum


class DriveType(str, Enum):
    """Screw drive type — determines tool envelope."""
    HEX_BOLT     = "hex_bolt"       # wrench/socket on hex head
    SOCKET_CAP   = "socket_cap"     # hex-key (Allen) into cap head recess
    TORX         = "torx"           # Torx bit into recess
    PHILLIPS     = "phillips"       # Phillips bit into recess
    SLOTTED      = "slotted"        # flat-blade bit
    POZI         = "pozi"
    FLANGE_HEX   = "flange_hex"     # hex + integral washer flange


class HeadStyle(str, Enum):
    PAN          = "pan"
    COUNTERSUNK  = "countersunk"    # flush with surface when seated
    RAISED_CSK   = "raised_csk"
    BUTTON       = "button"
    SOCKET_CAP   = "socket_cap"
    HEX          = "hex"
    FLANGE_HEX   = "flange_hex"
    STUD         = "stud"           # threaded rod, no head


# ── Minimum tool-access envelopes (mm) per drive type ──────────────────────
# (min_axial_above_head, min_radial_around_head)
# Sources: DIN/ISO tool dimensions, general DFA tool-access guidelines
TOOL_ENVELOPE: dict[DriveType, tuple[float, float]] = {
    # axial height needed for tool above head, radial clearance beyond head radius
    DriveType.HEX_BOLT:   (10.0, 6.0),   # socket needs depth + wrench swing
    DriveType.SOCKET_CAP: ( 8.0, 3.0),   # L-key or T-handle
    DriveType.TORX:       ( 8.0, 2.0),
    DriveType.PHILLIPS:   (10.0, 2.0),
    DriveType.SLOTTED:    (10.0, 2.0),
    DriveType.POZI:       (10.0, 2.0),
    DriveType.FLANGE_HEX: (10.0, 6.0),
}


@dataclass
class FastenerSpec:
    """
    Complete geometric and contextual specification of a single fastener instance.

    Physical dimensions come from standard tables (ISO 4014, ISO 4762, etc.).
    Contextual dimensions (clearances, path lengths) are measured in the
    assembled state — i.e. what space is actually available.
    """

    # ── identity ────────────────────────────────────────────────────────────
    id:   str = ""
    name: str = "Screw"

    # ── fastener physical dimensions ─────────────────────────────────────────
    nominal_diameter_mm:  float = 4.0    # M4, M6, etc.  shaft diameter
    total_length_mm:      float = 16.0   # shank + thread length (not head)
    head_diameter_mm:     float = 8.0    # outer head / hex AF dimension
    head_height_mm:       float = 4.0    # head height above mating surface
    thread_pitch_mm:      float = 0.7    # coarse pitch default for M4
    drive_type:           DriveType  = DriveType.SOCKET_CAP
    head_style:           HeadStyle  = HeadStyle.SOCKET_CAP

    # ── hole / seat geometry ─────────────────────────────────────────────────
    hole_diameter_mm:         float = 4.2    # clearance hole or tapped hole diameter
    countersink_diameter_mm:  float = 0.0    # 0 = no countersink
    countersink_angle_deg:    float = 90.0   # standard 90° or 120°

    # ── engagement / clamping ────────────────────────────────────────────────
    grip_thickness_mm:        float = 10.0   # total thickness of clamped parts
    thread_engagement_mm:     float = 6.0    # how deep screw goes into threaded hole/nut

    # ── available clearance in the assembled context ─────────────────────────
    # These are what the assembly provides — not the fastener's own geometry.

    axial_clearance_above_head_mm: float = 20.0
    # Space directly above the head along the insertion axis, from head top
    # to the nearest obstruction (another part, housing wall, cable, etc.).
    # Must be >= TOOL_ENVELOPE[drive_type][0] to be driveable.

    radial_clearance_mm: float = 10.0
    # Free space measured radially outward from the head edge
    # (head_diameter/2 + radial_clearance = obstruction radius).
    # Must be >= TOOL_ENVELOPE[drive_type][1] for tool to engage the head.

    insertion_path_length_mm: float = 40.0
    # Straight-line clearance along the insertion axis from the start of the
    # approach (point where screw tip enters the assembly space) to the mating
    # surface.  Must be >= total_length_mm + head_height_mm + 2 mm margin.

    removal_path_length_mm: float = 0.0
    # Clearance in the REVERSE direction for removal (withdrawal path).
    # 0 = same as insertion_path_length_mm (symmetric case).
    # Set explicitly if a part assembled after this screw now blocks removal.

    # ── adjacent obstruction flags ───────────────────────────────────────────
    parts_assembled_after: list[str] = field(default_factory=list)
    # IDs of parts placed into the assembly AFTER this fastener.
    # If any of these obstruct the withdrawal path, removal is blocked.

    lateral_obstruction_mm: float = 0.0
    # Closest lateral (side) obstruction within the bolt circle / bolt group.
    # Used to detect bolt-pitch too tight for a socket or wrench to swing.
    # 0 = no adjacent fasteners / no lateral constraint known.

    @property
    def effective_removal_path(self) -> float:
        return self.removal_path_length_mm if self.removal_path_length_mm > 0 \
               else self.insertion_path_length_mm
