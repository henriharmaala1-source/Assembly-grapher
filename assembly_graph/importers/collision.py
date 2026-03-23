"""
collision.py — assembly collision and near-miss detection.

Three detection methods (in increasing accuracy / cost order):

  1. AABB  (Axis-Aligned Bounding Box)
     ─────────────────────────────────
     O(n²) pairs; no dependencies beyond stdlib + this project.
     Uses part position + Geometry (length, width, height) to build world-space
     boxes and checks axis-overlap.  Fast but over-reports when parts are rotated.

  2. OBB   (Oriented Bounding Box via Separating Axis Theorem)
     ─────────────────────────────────────────────────────────
     O(n²) pairs; no extra dependencies.
     Uses the full 3×3 rotation from PartPose.  More accurate than AABB for
     rotated parts.  Still an approximation (misses concave shapes).

  3. BRep  (exact B-rep interference via pythonOCC)
     ──────────────────────────────────────────────
     Uses BRepAlgoAPI_Common to compute the exact intersection solid.
     Returns overlap volume in mm³.  Requires pythonOCC and shape objects
     (available after a STEP import).  Slow for large assemblies — run on
     AABB-filtered candidate pairs only.

Public API
──────────
    from assembly_graph.importers.collision import (
        check_collisions, CollisionReport,
    )

    # From ImportResult (STEP import — has poses + shapes)
    report = check_collisions(result, method="brep")

    # From manually built data
    report = check_collisions(result, method="aabb")

    print(report.summary())
    for pair in report.collisions:
        print(pair)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from dfma.models.part   import Assembly, Part, Geometry
from .base              import ImportResult, PartPose, CollisionPair

# Optional pythonOCC
_OCC_AVAILABLE = False
try:
    from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCC.Core.BRepGProp   import brepgprop
    from OCC.Core.GProp       import GProp_GProps
    from OCC.Core.BRepCheck   import BRepCheck_Analyzer
    _OCC_AVAILABLE = True
except ImportError:
    pass


# ── report container ──────────────────────────────────────────────────────────

@dataclass
class CollisionReport:
    """
    Complete collision analysis result for one assembly.

    collisions  : hard interferences (parts actually penetrate each other)
    near_misses : pairs within the specified tolerance but not overlapping
    method      : detection method used
    checked_pairs: total number of part pairs checked
    skipped_pairs: pairs skipped (no pose / no shape data)
    """
    collisions:    list[CollisionPair] = field(default_factory=list)
    near_misses:   list[CollisionPair] = field(default_factory=list)
    method:        str   = "aabb"
    checked_pairs: int   = 0
    skipped_pairs: int   = 0

    @property
    def is_clean(self) -> bool:
        return len(self.collisions) == 0

    def summary(self) -> str:
        lines = [
            "─" * 56,
            f"  COLLISION REPORT  ({self.method.upper()})",
            "─" * 56,
            f"  Pairs checked  : {self.checked_pairs}",
            f"  Pairs skipped  : {self.skipped_pairs}",
            f"  Hard collisions: {len(self.collisions)}",
            f"  Near-misses    : {len(self.near_misses)}",
            f"  Status         : {'CLEAN ✓' if self.is_clean else 'VIOLATIONS ✗'}",
        ]
        if self.collisions:
            lines.append("")
            lines.append("  Hard collisions:")
            for cp in self.collisions:
                lines.append(f"    {cp}")
        if self.near_misses:
            lines.append("")
            lines.append("  Near-misses (within tolerance):")
            for cp in self.near_misses:
                lines.append(f"    {cp}")
        lines.append("─" * 56)
        return "\n".join(lines)


# ── public entry point ────────────────────────────────────────────────────────

def check_collisions(
    result:         ImportResult,
    method:         str   = "auto",
    near_miss_mm:   float = 1.0,
    brep_min_vol:   float = 0.01,
    ignore_pairs:   set[tuple[str,str]] | None = None,
) -> CollisionReport:
    """
    Run collision detection on an ImportResult.

    Parameters
    ----------
    result         : output of any importer (must have .poses for OBB/AABB;
                     also needs .shapes for BRep)
    method         : "auto" selects the best available method:
                       has shapes → "brep"
                       has poses  → "obb"
                       otherwise  → "aabb" (uses Geometry dims, position=(0,0,0))
                     Override with "aabb" | "obb" | "brep".
    near_miss_mm   : distance ≤ this is reported as a near-miss (not a collision)
    brep_min_vol   : minimum interference volume (mm³) to count as a collision
                     (filters numerical noise in BRep intersection)
    ignore_pairs   : set of (part_a_id, part_b_id) tuples to skip (e.g. known contacts)

    Returns
    -------
    CollisionReport
    """
    ignore = set()
    if ignore_pairs:
        for a, b in ignore_pairs:
            ignore.add((min(a,b), max(a,b)))

    # Auto-select method
    if method == "auto":
        if result.has_shapes and _OCC_AVAILABLE:
            method = "brep"
        elif result.has_poses:
            method = "obb"
        else:
            method = "aabb"

    parts = result.assembly.all_parts()
    n     = len(parts)

    report = CollisionReport(method=method)

    if method == "brep":
        if not _OCC_AVAILABLE:
            raise RuntimeError(
                "BRep collision requires pythonOCC. "
                "Install with: pip install pythonocc-core"
            )
        _check_brep(parts, result, report, near_miss_mm, brep_min_vol, ignore)

    elif method == "obb":
        _check_obb(parts, result, report, near_miss_mm, ignore)

    else:  # aabb
        _check_aabb(parts, result, report, near_miss_mm, ignore)

    return report


# ── per-pair helpers ──────────────────────────────────────────────────────────

def _get_half_extents(part: Part) -> tuple[float, float, float]:
    """Return (hx, hy, hz) — half-dimensions of the part's bounding box."""
    g = part.geometry
    return (g.length / 2.0, g.width / 2.0, g.height / 2.0)


def _get_pose(part_id: str, result: ImportResult) -> PartPose:
    return result.poses.get(part_id, PartPose(part_id))


# ── AABB detector ─────────────────────────────────────────────────────────────

def _aabb_for_part(part: Part, pose: PartPose) -> tuple[float,...]:
    """
    Compute world-space AABB using pose centre and geometry half-extents.
    Ignores rotation (conservative over-approximation).
    """
    hx, hy, hz = _get_half_extents(part)
    cx, cy, cz = pose.position
    return (cx - hx, cy - hy, cz - hz,
            cx + hx, cy + hy, cz + hz)


def _aabb_penetration(
    a: tuple[float,...],
    b: tuple[float,...],
) -> float:
    """
    Signed penetration depth along the minimum-penetration axis.
    Positive = overlap; negative = separation.
    """
    depths = []
    for i in range(3):
        overlap = min(a[i+3], b[i+3]) - max(a[i], b[i])
        depths.append(overlap)
    return min(depths)   # min-axis penetration (most negative = separated)


def _check_aabb(
    parts:  list[Part],
    result: ImportResult,
    report: CollisionReport,
    near_miss_mm: float,
    ignore: set,
) -> None:
    n = len(parts)
    for i in range(n):
        for j in range(i + 1, n):
            pa, pb = parts[i], parts[j]
            key = (min(pa.id, pb.id), max(pa.id, pb.id))
            if key in ignore:
                report.skipped_pairs += 1
                continue

            pose_a = _get_pose(pa.id, result)
            pose_b = _get_pose(pb.id, result)
            bb_a   = _aabb_for_part(pa, pose_a)
            bb_b   = _aabb_for_part(pb, pose_b)

            pen = _aabb_penetration(bb_a, bb_b)
            report.checked_pairs += 1

            if pen > 0:
                report.collisions.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_mm=round(pen, 4),
                    method="aabb", is_interference=True,
                    description=f"{pa.name} ↔ {pb.name}",
                ))
            elif pen > -near_miss_mm:
                report.near_misses.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_mm=round(-pen, 4),
                    method="aabb", is_interference=False,
                    description=f"{pa.name} ↔ {pb.name}  gap={-pen:.3f} mm",
                ))


# ── OBB detector (Separating Axis Theorem) ────────────────────────────────────

def _obb_collide(
    c_a:  tuple[float,float,float],
    R_a:  tuple[tuple[float,float,float],...],
    h_a:  tuple[float,float,float],
    c_b:  tuple[float,float,float],
    R_b:  tuple[tuple[float,float,float],...],
    h_b:  tuple[float,float,float],
) -> tuple[bool, float]:
    """
    OBB vs OBB Separating Axis Theorem.
    Returns (overlapping: bool, min_penetration_mm: float).
    Positive penetration = overlap; negative = separation.

    References:
        Gottschalk, Lin & Manocha (1996) — OBBTree
        Real-Time Collision Detection, Ericson (2005) Chapter 4.4
    """
    # Translation in world space
    T = (c_b[0]-c_a[0], c_b[1]-c_a[1], c_b[2]-c_a[2])

    # Build rotation of B relative to A:  C[i][j] = dot(A_axis_i, B_axis_j)
    C = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            C[i][j] = R_a[0][i]*R_b[0][j] + R_a[1][i]*R_b[1][j] + R_a[2][i]*R_b[2][j]

    AbsC = [[abs(C[i][j]) + 1e-10 for j in range(3)] for i in range(3)]

    min_sep = -math.inf   # most negative separation on any axis = most penetration

    def _sep_on_axis(axis_proj_a, axis_proj_b) -> float:
        """Negative = separation (no overlap); positive = overlap."""
        return axis_proj_a + axis_proj_b - abs(
            T[0]*0 + T[1]*0 + T[2]*0   # will be filled per-axis
        )

    def check_axis(ra, rb, t) -> float:
        """Signed penetration: positive = overlapping."""
        return ra + rb - abs(t)

    # 15 separating axes: 3 A-axes + 3 B-axes + 9 cross-products
    def dot3(v, w): return v[0]*w[0] + v[1]*w[1] + v[2]*w[2]

    # Project T onto each axis of A (A's local x, y, z)
    t_in_a = [
        dot3(T, (R_a[0][0],R_a[1][0],R_a[2][0])),
        dot3(T, (R_a[0][1],R_a[1][1],R_a[2][1])),
        dot3(T, (R_a[0][2],R_a[1][2],R_a[2][2])),
    ]
    t_in_b = [
        dot3(T, (R_b[0][0],R_b[1][0],R_b[2][0])),
        dot3(T, (R_b[0][1],R_b[1][1],R_b[2][1])),
        dot3(T, (R_b[0][2],R_b[1][2],R_b[2][2])),
    ]

    pens: list[float] = []

    # A's 3 axes
    for i in range(3):
        ra = h_a[i]
        rb = h_b[0]*AbsC[i][0] + h_b[1]*AbsC[i][1] + h_b[2]*AbsC[i][2]
        pens.append(check_axis(ra, rb, t_in_a[i]))

    # B's 3 axes
    for j in range(3):
        ra = h_a[0]*AbsC[0][j] + h_a[1]*AbsC[1][j] + h_a[2]*AbsC[2][j]
        rb = h_b[j]
        pens.append(check_axis(ra, rb, t_in_b[j]))

    # 9 cross-product axes  A_i × B_j
    # When A_i and B_j are (nearly) parallel, the cross product is the zero
    # vector — a degenerate separating axis.  Skip it; it provides no
    # constraint (treat penetration as +∞ on this axis).
    for i in range(3):
        ii  = (i+1) % 3
        iii = (i+2) % 3
        for j in range(3):
            # |A_i × B_j|² = 1 - C[i][j]²  (both are unit vectors)
            if 1.0 - C[i][j]**2 < 1e-6:
                continue   # degenerate: axes are parallel → no constraint
            jj  = (j+1) % 3
            jjj = (j+2) % 3
            ra = h_a[ii]*AbsC[iii][j] + h_a[iii]*AbsC[ii][j]
            rb = h_b[jj]*AbsC[i][jjj] + h_b[jjj]*AbsC[i][jj]
            # t projection onto A_i × B_j
            t_proj = (t_in_a[ii] * C[iii][j] - t_in_a[iii] * C[ii][j])
            pens.append(check_axis(ra, rb, t_proj))

    # All pens positive → overlapping on all axes → collision
    min_pen = min(pens)
    overlapping = all(p > 0 for p in pens)
    return overlapping, min_pen


def _check_obb(
    parts:  list[Part],
    result: ImportResult,
    report: CollisionReport,
    near_miss_mm: float,
    ignore: set,
) -> None:
    n = len(parts)
    for i in range(n):
        for j in range(i + 1, n):
            pa, pb = parts[i], parts[j]
            key = (min(pa.id, pb.id), max(pa.id, pb.id))
            if key in ignore:
                report.skipped_pairs += 1
                continue

            pose_a = _get_pose(pa.id, result)
            pose_b = _get_pose(pb.id, result)
            h_a    = _get_half_extents(pa)
            h_b    = _get_half_extents(pb)

            overlapping, min_pen = _obb_collide(
                pose_a.position, pose_a.rotation, h_a,
                pose_b.position, pose_b.rotation, h_b,
            )
            report.checked_pairs += 1

            if overlapping and min_pen > 0:
                report.collisions.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_mm=round(min_pen, 4),
                    method="obb", is_interference=True,
                    description=f"{pa.name} ↔ {pb.name}",
                ))
            elif not overlapping and min_pen > -near_miss_mm:
                # Separated but close
                report.near_misses.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_mm=round(-min_pen, 4),
                    method="obb", is_interference=False,
                    description=f"{pa.name} ↔ {pb.name}  gap={-min_pen:.3f} mm",
                ))


# ── BRep detector (pythonOCC) ─────────────────────────────────────────────────

def _brep_interference_volume(shape_a, shape_b) -> float:
    """
    Compute the volume of the intersection solid of two BRep shapes.
    Returns 0.0 if the shapes do not intersect or on any OCC error.
    """
    try:
        common = BRepAlgoAPI_Common(shape_a, shape_b)
        common.Build()
        if not common.IsDone():
            return 0.0
        result_shape = common.Shape()
        # Check the result is non-empty
        analyzer = BRepCheck_Analyzer(result_shape)
        if not analyzer.IsValid():
            return 0.0
        props = GProp_GProps()
        brepgprop.VolumeProperties(result_shape, props)
        return abs(props.Mass())   # mm³ (AP214 units)
    except Exception:
        return 0.0


def _check_brep(
    parts:        list[Part],
    result:       ImportResult,
    report:       CollisionReport,
    near_miss_mm: float,
    brep_min_vol: float,
    ignore:       set,
) -> None:
    """
    Phase 1: AABB broad-phase filter.
    Phase 2: Exact BRep interference on surviving candidates.
    """
    n = len(parts)
    for i in range(n):
        for j in range(i + 1, n):
            pa, pb = parts[i], parts[j]
            key = (min(pa.id, pb.id), max(pa.id, pb.id))
            if key in ignore:
                report.skipped_pairs += 1
                continue

            shape_a = result.shapes.get(pa.id)
            shape_b = result.shapes.get(pb.id)

            if shape_a is None or shape_b is None:
                # Fall back to AABB for parts without shapes
                pose_a = _get_pose(pa.id, result)
                pose_b = _get_pose(pb.id, result)
                bb_a   = _aabb_for_part(pa, pose_a)
                bb_b   = _aabb_for_part(pb, pose_b)
                pen    = _aabb_penetration(bb_a, bb_b)
                report.checked_pairs += 1
                if pen > 0:
                    report.collisions.append(CollisionPair(
                        part_a=pa.id, part_b=pb.id,
                        overlap_mm=round(pen, 4),
                        method="aabb_fallback", is_interference=True,
                        description=f"{pa.name} ↔ {pb.name}  (no BRep shape)",
                    ))
                continue

            # Broad-phase AABB filter (avoid expensive BRep on clearly separate parts)
            pose_a = _get_pose(pa.id, result)
            pose_b = _get_pose(pb.id, result)
            bb_a   = _aabb_for_part(pa, pose_a)
            bb_b   = _aabb_for_part(pb, pose_b)
            if _aabb_penetration(bb_a, bb_b) < -(near_miss_mm * 5):
                # Clearly far apart — skip expensive BRep
                continue

            # Exact BRep
            vol = _brep_interference_volume(shape_a, shape_b)
            report.checked_pairs += 1

            if vol >= brep_min_vol:
                report.collisions.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_volume_mm3=round(vol, 4),
                    method="brep", is_interference=True,
                    description=f"{pa.name} ↔ {pb.name}  vol={vol:.3f} mm³",
                ))
            else:
                # Near-miss: passed AABB filter but no meaningful BRep interference
                report.near_misses.append(CollisionPair(
                    part_a=pa.id, part_b=pb.id,
                    overlap_volume_mm3=round(vol, 4),
                    method="brep", is_interference=False,
                    description=f"{pa.name} ↔ {pb.name}  contact/near-miss",
                ))
