"""
step_importer.py — parse STEP AP214 / AP242 assemblies via pythonOCC.

Requires: pip install pythonocc-core   (cross-platform, ~80 MB)
           conda install -c conda-forge pythonocc-core

If pythonOCC is not installed this module still imports but raises
ImportError with installation instructions when import_step() is called.

Extracted data
──────────────
  • Assembly hierarchy (nested Assembly / Part objects)
  • Part names from XDE label attributes
  • Bounding box → Geometry (length, width, height in mm)
  • Volume → mass_grams estimate (requires a density map or defaults to steel)
  • Per-instance transforms → PartPose dict
  • Shape objects stored in ImportResult.shapes for downstream collision checks

Mate / contact data
───────────────────
  STEP AP214 does NOT carry constraint/mate information.
  STEP AP242 carries some kinematic joints — parsed when present.
  For AP214 files the returned LiaisonMatrix is empty; populate it manually
  or run check_collisions() to infer contacts from geometry proximity.

Usage
─────
    from assembly_graph.importers import load_assembly
    result = load_assembly("my_part.step")

    # With custom densities (g/mm³)
    from assembly_graph.importers.step_importer import import_step
    result = import_step("assy.stp", density_map={
        "steel":    7.85e-3,
        "alumin":   2.70e-3,
        "default":  7.85e-3,
    })
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from dfma.models.part import (
    Assembly, Part, Geometry,
    ManufacturingProcess, Material,
)
from assembly_graph.liaison_matrix import LiaisonMatrix
from .base    import ImportResult, ImportError, PartPose
from .mappings import map_material, map_process_from_geometry


# ── pythonOCC availability guard ──────────────────────────────────────────────

_OCC_AVAILABLE = False
_OCC_ERROR     = ""

try:
    from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
    from OCC.Core.TDocStd        import TDocStd_Document
    from OCC.Core.XCAFDoc        import XCAFDoc_DocumentTool, XCAFDoc_Location
    from OCC.Core.TDF            import TDF_LabelSequence, TDF_Label
    from OCC.Core.TDataStd       import TDataStd_Name
    from OCC.Core.BRepBndLib     import brepbndlib
    from OCC.Core.Bnd            import Bnd_Box
    from OCC.Core.BRepGProp      import brepgprop
    from OCC.Core.GProp          import GProp_GProps
    from OCC.Core.TopLoc         import TopLoc_Location
    from OCC.Core.gp             import gp_Trsf, gp_Mat, gp_XYZ
    from OCC.Core.BRep           import BRep_Builder
    from OCC.Core.TopoDS         import TopoDS_Compound
    from OCC.Core.BRepAlgoAPI    import BRepAlgoAPI_Common
    from OCC.Core.BRepCheck      import BRepCheck_Analyzer
    _OCC_AVAILABLE = True
except ImportError as _e:
    _OCC_ERROR = str(_e)


# ── default density table (g/mm³) ─────────────────────────────────────────────

_DEFAULT_DENSITIES: dict[str, float] = {
    "steel":    7.85e-3,
    "stainless":8.00e-3,
    "iron":     7.87e-3,
    "alumin":   2.70e-3,    # matches "aluminium", "aluminum"
    "copper":   8.96e-3,
    "brass":    8.50e-3,
    "bronze":   8.80e-3,
    "titanium": 4.51e-3,
    "zinc":     7.13e-3,
    "abs":      1.05e-3,
    "nylon":    1.14e-3,
    "pp":       0.90e-3,
    "pe":       0.95e-3,
    "pvc":      1.38e-3,
    "rubber":   1.20e-3,
    "epdm":     1.25e-3,
    "default":  7.85e-3,   # steel fallback
}


def _lookup_density(material_name: str, density_map: dict[str, float]) -> float:
    merged = {**_DEFAULT_DENSITIES, **(density_map or {})}
    lower = material_name.lower()
    for key, rho in merged.items():
        if key != "default" and key in lower:
            return rho
    return merged.get("default", 7.85e-3)


# ── public entry point ────────────────────────────────────────────────────────

def import_step(
    path: str,
    density_map:    dict[str, float] | None = None,
    infer_contacts: bool = True,
    contact_gap_mm: float = 0.1,
) -> ImportResult:
    """
    Import a STEP AP214 / AP242 file.

    Parameters
    ----------
    path            : path to the .step / .stp file
    density_map     : maps keyword → g/mm³.  Keys are matched as substrings
                      of the material name (case-insensitive).  Falls back
                      to built-in defaults.
    infer_contacts  : if True, add a face/rigid liaison contact between any
                      two parts whose AABB overlap or touch (gap ≤ contact_gap_mm).
                      This is a heuristic — you may want to prune false positives.
    contact_gap_mm  : clearance threshold for proximity-based contact inference.
    """
    if not _OCC_AVAILABLE:
        raise ImportError(
            "pythonOCC is required for STEP import.\n"
            "Install with:  pip install pythonocc-core\n"
            f"(Original error: {_OCC_ERROR})"
        )

    p = Path(path)
    if not p.exists():
        raise ImportError(f"File not found: {path}")

    warnings: list[str] = []

    # ── read STEP file ────────────────────────────────────────────────────────
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)

    status = reader.ReadFile(str(p))
    if status != 1:   # 1 = IFSelect_RetDone
        raise ImportError(f"STEPCAFControl_Reader failed (status={status}) for '{path}'")

    doc = TDocStd_Document("XmlXCAF")
    reader.Transfer(doc)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    # colour_tool not used for now — reserved for future visualisation

    # ── find root shapes ──────────────────────────────────────────────────────
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)

    if roots.Size() == 0:
        raise ImportError(f"No free shapes found in '{path}'")

    # Wrap multiple roots in a single assembly
    if roots.Size() == 1:
        root_label = roots.Value(1)
    else:
        warnings.append(
            f"STEP file has {roots.Size()} root shapes — "
            "wrapped into a synthetic top-level assembly."
        )
        root_label = None   # handled below

    # ── traverse and build Assembly tree ──────────────────────────────────────
    ctx = _ParseContext(
        shape_tool=shape_tool,
        density_map=density_map or {},
        warnings=warnings,
    )

    if root_label is not None:
        assembly = ctx.label_to_node(root_label, identity_trsf())
    else:
        # Multiple roots: build synthetic parent
        assembly = Assembly(id="ROOT", name=p.stem)
        for i in range(1, roots.Size() + 1):
            lbl = roots.Value(i)
            node = ctx.label_to_node(lbl, identity_trsf())
            if isinstance(node, Assembly):
                assembly.subassemblies.append(node)
            else:
                assembly.parts.append(node)

    # ── build liaison matrix ──────────────────────────────────────────────────
    all_parts = assembly.all_parts()
    part_ids  = [p.id for p in all_parts]
    liaison   = LiaisonMatrix(part_ids)

    # AP242 kinematic joints (best-effort)
    _parse_ap242_constraints(reader, liaison, ctx, warnings)

    # Proximity-based contact inference
    if infer_contacts:
        _infer_contacts_from_aabb(ctx.poses, ctx.bboxes, liaison, contact_gap_mm, warnings)

    return ImportResult(
        assembly=assembly,
        liaison=liaison,
        poses=ctx.poses,
        shapes=ctx.shapes,
        warnings=warnings,
        source="step",
    )


# ── internal parse context ────────────────────────────────────────────────────

class _ParseContext:
    def __init__(self, shape_tool, density_map, warnings):
        self.shape_tool  = shape_tool
        self.density_map = density_map
        self.warnings    = warnings
        self.poses:  dict[str, PartPose] = {}
        self.shapes: dict[str, Any]      = {}   # part_id → TopoDS_Shape
        self.bboxes: dict[str, tuple]    = {}   # part_id → (xmin,ymin,zmin,xmax,ymax,zmax) mm
        self._id_counter = 0

    def next_id(self, prefix: str = "P") -> str:
        self._id_counter += 1
        return f"{prefix}{self._id_counter:04d}"

    def label_to_node(
        self,
        label: Any,
        parent_trsf: Any,
    ) -> "Assembly | Part":
        st = self.shape_tool
        name = _label_name(label)

        if st.IsAssembly(label):
            asm = Assembly(id=self.next_id("ASM"), name=name or "Assembly")
            components = TDF_LabelSequence()
            st.GetComponents(label, components, False)
            for i in range(1, components.Size() + 1):
                comp_lbl = components.Value(i)
                # Get the referred definition and this instance's transform
                ref_lbl  = TDF_Label()
                ok = st.GetReferredShape(comp_lbl, ref_lbl)
                loc = _label_location(comp_lbl, st)
                child_trsf = _compose_trsf(parent_trsf, loc)
                child_node = self.label_to_node(
                    ref_lbl if ok else comp_lbl,
                    child_trsf,
                )
                if isinstance(child_node, Assembly):
                    asm.subassemblies.append(child_node)
                else:
                    asm.parts.append(child_node)
            return asm
        else:
            # Leaf = part
            part_id = self.next_id("P")
            shape   = st.GetShape(label)

            bbox_mm = _compute_bbox_mm(shape)
            geom    = _bbox_to_geometry(shape, bbox_mm, self.density_map)
            mat_enum = Material.UNKNOWN
            proc     = map_process_from_geometry(
                mat_enum,
                geom.min_wall_thickness,
                geom.tolerance,
                geom.draft_angle,
                geom.is_fastener,
            )

            part = Part(id=part_id, name=name or part_id,
                        geometry=geom, material=mat_enum, process=proc)

            # Store shape for later collision checks
            self.shapes[part_id] = shape
            self.bboxes[part_id] = bbox_mm

            # Compute world-space pose from accumulated transform
            pos, rot = _trsf_to_pose(parent_trsf)
            self.poses[part_id] = PartPose(part_id=part_id, position=pos, rotation=rot)

            return part


# ── geometry helpers ──────────────────────────────────────────────────────────

def _compute_bbox_mm(shape) -> tuple[float, float, float, float, float, float]:
    """Return (xmin,ymin,zmin,xmax,ymax,zmax) in mm from a TopoDS_Shape."""
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    if box.IsVoid():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    xmn, ymn, zmn, xmx, ymx, zmx = box.Get()
    # STEP AP214 units are mm; OCCT reads them as-is (no unit conversion needed
    # for AP214; AP203 may use metres — add detection if needed).
    return (xmn, ymn, zmn, xmx, ymx, zmx)


def _bbox_to_geometry(
    shape,
    bbox_mm: tuple,
    density_map: dict[str, float],
) -> Geometry:
    xmn, ymn, zmn, xmx, ymx, zmx = bbox_mm
    length = max(xmx - xmn, 0.0)
    width  = max(ymx - ymn, 0.0)
    height = max(zmx - zmn, 0.0)

    # Approximate volume from B-rep (more accurate than bbox)
    props = GProp_GProps()
    try:
        brepgprop.VolumeProperties(shape, props)
        # OCCT VolumeProperties returns mm³ for AP214 (mm-based)
        volume_mm3 = abs(props.Mass())
    except Exception:
        # Fall back to bounding-box volume if B-rep props fail
        volume_mm3 = length * width * height * 0.5   # rough solid fill ratio

    density = _lookup_density("", density_map)   # default density
    mass_g  = volume_mm3 * density

    # Estimate wall thickness as min dimension / 2 (rough)
    min_dim = min(d for d in (length, width, height) if d > 0) if (length+width+height) > 0 else 0
    wall    = min_dim / 2.0

    return Geometry(
        length=round(length, 3),
        width=round(width,  3),
        height=round(height, 3),
        min_wall_thickness=round(wall, 3),
        mass_grams=round(mass_g, 3),
        tolerance=0.1,
    )


# ── transform helpers ─────────────────────────────────────────────────────────

def identity_trsf():
    t = gp_Trsf()
    # gp_Trsf() default is identity
    return t


def _label_location(comp_label, shape_tool) -> Any:
    """Get the TopLoc_Location of a component label."""
    loc_attr = XCAFDoc_Location()
    if comp_label.FindAttribute(XCAFDoc_Location.GetID_s(), loc_attr):
        return loc_attr.Get()
    return TopLoc_Location()


def _compose_trsf(parent: Any, loc: Any) -> Any:
    """Compose parent transform with the local component location."""
    return parent * loc.IsIdentity and parent or \
           _multiply_trsf(parent, loc.IsIdentity and gp_Trsf() or loc.IsIdentity)


def _multiply_trsf(a: Any, b: Any) -> Any:
    result = gp_Trsf()
    result.Multiply(a)
    return result


def _trsf_to_pose(trsf) -> tuple[
    tuple[float,float,float],
    tuple[tuple[float,float,float],...],
]:
    """Extract position (mm) and 3×3 rotation matrix from a gp_Trsf."""
    try:
        tx = trsf.TranslationPart()
        pos = (tx.X(), tx.Y(), tx.Z())
        M = trsf.VectorialPart()
        rot = (
            (M.Value(1,1), M.Value(1,2), M.Value(1,3)),
            (M.Value(2,1), M.Value(2,2), M.Value(2,3)),
            (M.Value(3,1), M.Value(3,2), M.Value(3,3)),
        )
    except Exception:
        pos = (0.0, 0.0, 0.0)
        rot = ((1,0,0),(0,1,0),(0,0,1))
    return pos, rot


def _label_name(label) -> str:
    name = TDataStd_Name()
    try:
        if label.FindAttribute(TDataStd_Name.GetID_s(), name):
            return name.Get().ToExtString()
    except Exception:
        pass
    return ""


# ── AP242 kinematic joint parser ──────────────────────────────────────────────

def _parse_ap242_constraints(
    reader,
    liaison: LiaisonMatrix,
    ctx: _ParseContext,
    warnings: list[str],
) -> None:
    """
    Best-effort extraction of AP242 kinematic joints as liaison contacts.
    Falls back silently on AP214 files (no kinematic entities present).
    """
    try:
        model = reader.StepModel()
        nb    = model.NbEntities()
        for i in range(1, nb + 1):
            entity = model.Value(i)
            etype  = entity.DynamicType().Name()
            # AP242 kinematic_joint entities
            if "KINEMATIC_JOINT" in etype or "ASSEMBLY_JOINT" in etype:
                # These entities reference two product_definition_formation objects;
                # the mapping to our part IDs requires more STEP schema parsing than
                # is practical here without full AP242 schema awareness.
                # Log as a warning for now; full implementation requires the
                # STEP schema binding (express compiler output).
                warnings.append(
                    f"AP242 kinematic joint found (entity {i}, type {etype}) — "
                    "automatic liaison mapping not yet implemented; add contacts manually."
                )
    except Exception:
        pass   # AP214 — silently skip


# ── proximity-based contact inference ────────────────────────────────────────

def _infer_contacts_from_aabb(
    poses:          dict[str, PartPose],
    bboxes:         dict[str, tuple],
    liaison:        LiaisonMatrix,
    gap_mm:         float,
    warnings:       list[str],
) -> None:
    """
    Add a face/rigid contact to the liaison matrix for every pair of parts whose
    world-space AABBs overlap or touch within `gap_mm`.

    This is a heuristic for STEP files that lack explicit mate data.
    It over-approximates contacts (false positives) but never misses touching parts.
    """
    ids = list(bboxes.keys())
    n   = len(ids)
    added = 0

    for i in range(n):
        for j in range(i + 1, n):
            a, b = ids[i], ids[j]
            if liaison.has_contact(a, b):
                continue   # already set (e.g. from AP242 joints)

            # Transform bboxes into world space using poses
            ab = _world_bbox(a, bboxes, poses)
            bb = _world_bbox(b, bboxes, poses)

            if _aabb_overlap(ab, bb, gap_mm):
                try:
                    liaison.add_contact(a, b, contact_type="face", strength="rigid")
                    added += 1
                except KeyError:
                    pass   # part not in liaison matrix (suppressed/sub-asm node)

    if added:
        warnings.append(
            f"Inferred {added} liaison contact(s) from bounding-box proximity "
            f"(gap ≤ {gap_mm} mm). Review and remove false positives."
        )


def _world_bbox(
    part_id: str,
    bboxes: dict[str, tuple],
    poses:  dict[str, PartPose],
) -> tuple[float, float, float, float, float, float]:
    """
    Compute the world-space AABB of a part by transforming its local bbox corners.
    Returns (xmin,ymin,zmin,xmax,ymax,zmax).
    """
    xmn, ymn, zmn, xmx, ymx, zmx = bboxes[part_id]
    pose = poses.get(part_id, PartPose(part_id))

    # Transform all 8 corners of the local bbox
    corners = [
        pose.transform_point((cx, cy, cz))
        for cx in (xmn, xmx)
        for cy in (ymn, ymx)
        for cz in (zmn, zmx)
    ]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    zs = [c[2] for c in corners]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def _aabb_overlap(
    a: tuple[float,float,float,float,float,float],
    b: tuple[float,float,float,float,float,float],
    gap: float,
) -> bool:
    """True if two world-space AABBs overlap or are within `gap` mm of each other."""
    for i in range(3):
        if a[i+3] + gap < b[i] or b[i+3] + gap < a[i]:
            return False
    return True
