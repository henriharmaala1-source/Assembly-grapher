"""
base.py — shared types and utilities for all assembly importers.

ImportResult  : output container (assembly, liaison, poses, shapes, warnings)
PartPose      : 3D position + rotation for one part instance
CollisionPair : detected collision between two parts
detect_format : infer importer to use from file extension / content sniff
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# ── 3-D pose ──────────────────────────────────────────────────────────────────

@dataclass
class PartPose:
    """
    Position and orientation of one part instance in assembly space.

    position : (x, y, z) translation in mm
    rotation : 3×3 rotation matrix, row-major
               [[R00, R01, R02],
                [R10, R11, R12],
                [R20, R21, R22]]
               Identity = no rotation.
    """
    part_id:  str
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ] = field(default_factory=lambda: ((1,0,0),(0,1,0),(0,0,1)))

    @staticmethod
    def identity(part_id: str) -> "PartPose":
        return PartPose(part_id=part_id)

    def transform_point(self, p: tuple[float,float,float]) -> tuple[float,float,float]:
        """Apply this pose (rotation then translation) to a point."""
        R = self.rotation
        x = R[0][0]*p[0] + R[0][1]*p[1] + R[0][2]*p[2] + self.position[0]
        y = R[1][0]*p[0] + R[1][1]*p[1] + R[1][2]*p[2] + self.position[1]
        z = R[2][0]*p[0] + R[2][1]*p[1] + R[2][2]*p[2] + self.position[2]
        return (x, y, z)

    @property
    def is_identity(self) -> bool:
        R = self.rotation
        return (
            abs(R[0][0]-1)<1e-9 and abs(R[1][1]-1)<1e-9 and abs(R[2][2]-1)<1e-9 and
            abs(R[0][1])<1e-9   and abs(R[0][2])<1e-9   and
            abs(R[1][0])<1e-9   and abs(R[1][2])<1e-9   and
            abs(R[2][0])<1e-9   and abs(R[2][1])<1e-9   and
            all(abs(t)<1e-9 for t in self.position)
        )


# ── collision result ───────────────────────────────────────────────────────────

@dataclass
class CollisionPair:
    """
    A detected collision or near-miss between two parts.

    overlap_mm     : penetration depth (mm) — meaningful for AABB/OBB;
                     for BRep, use overlap_volume_mm3 instead.
    overlap_volume_mm3: volume of interference solid (BRep only; 0 for AABB/OBB).
    method         : detection method used — "aabb" | "obb" | "brep"
    is_interference: True = hard collision; False = contact/near-miss (gap < tolerance)
    """
    part_a:             str
    part_b:             str
    overlap_mm:         float = 0.0
    overlap_volume_mm3: float = 0.0
    method:             str   = "aabb"
    is_interference:    bool  = True
    description:        str   = ""

    def __str__(self) -> str:
        tag = "COLLISION" if self.is_interference else "NEAR-MISS"
        detail = (
            f"overlap={self.overlap_mm:.3f} mm"
            if self.method in ("aabb", "obb")
            else f"vol={self.overlap_volume_mm3:.3f} mm³"
        )
        return f"[{tag}] {self.part_a} ↔ {self.part_b}  {detail}  ({self.method})"


# ── import result ─────────────────────────────────────────────────────────────

@dataclass
class ImportResult:
    """
    Complete output from any assembly importer.

    assembly : dfma.models.part.Assembly object (hierarchy + parts)
    liaison  : LiaisonMatrix pre-populated from CAD mates / JSON contacts
    poses    : dict  part_id → PartPose  (world-space position/rotation per instance)
    shapes   : dict  part_id → OCC TopoDS_Shape  (only for STEP imports)
    warnings : non-fatal messages (e.g. missing material, suppressed component)
    source   : "json" | "step" | "plmxml" | "solidworks" | "nx"
    """
    assembly: Any                              # dfma.models.part.Assembly
    liaison:  Any                              # assembly_graph.liaison_matrix.LiaisonMatrix
    poses:    dict[str, PartPose]  = field(default_factory=dict)
    shapes:   dict[str, Any]       = field(default_factory=dict)   # OCC shapes
    warnings: list[str]            = field(default_factory=list)
    source:   str                  = ""

    @property
    def part_count(self) -> int:
        return len(self.assembly.all_parts())

    @property
    def has_poses(self) -> bool:
        return len(self.poses) > 0

    @property
    def has_shapes(self) -> bool:
        return len(self.shapes) > 0


class ImportError(Exception):
    """Raised when an importer cannot read the supplied file."""


# ── format detection ──────────────────────────────────────────────────────────

_EXT_MAP: dict[str, str] = {
    ".sldasm": "solidworks",
    ".sldprt": "solidworks",
    ".step":   "step",
    ".stp":    "step",
    ".plmxml": "plmxml",
    ".jt":     "jt",       # future
    ".prt":    "nx",        # NX native (ambiguous with Pro/E — checked below)
    ".json":   "json",
}


def detect_format(path: str) -> str:
    """
    Return a format string from the file extension and, for .xml / .json,
    a lightweight content sniff.

    Returns one of: "solidworks" | "nx" | "step" | "plmxml" | "json" | "unknown"
    """
    p   = Path(path)
    ext = p.suffix.lower()

    if ext == ".xml":
        # Sniff for PLMXML namespace
        try:
            with open(path, "r", errors="replace") as fh:
                head = fh.read(512)
            if "plmxml" in head.lower() or "PLMXMLSchema" in head:
                return "plmxml"
        except OSError:
            pass
        return "unknown"

    return _EXT_MAP.get(ext, "unknown")
