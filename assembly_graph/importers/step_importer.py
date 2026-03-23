"""
step_importer.py — parse STEP AP214 / AP242 assemblies via pythonOCC.

Requires: conda install -c conda-forge pythonocc-core

If pythonOCC is not installed this module still imports but raises
ImportError with installation instructions when import_step() is called.

Extracted data
──────────────
  • Individual solid bodies from the STEP file
  • Bounding box → Geometry (length, width, height in mm)
  • Volume → mass_grams estimate (requires a density map or defaults to steel)
  • Shape objects stored in ImportResult.shapes for downstream collision checks

Mate / contact data
───────────────────
  STEP AP214 does NOT carry constraint/mate information.
  The returned LiaisonMatrix is populated via bounding-box proximity;
  you may need to prune false positives or add contacts manually.

Usage
─────
    from assembly_graph.importers import load_assembly
    result = load_assembly("my_part.step")
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dfma.models.part import (
    Assembly, Part, Geometry,
    ManufacturingProcess, Material,
)
from assembly_graph.liaison_matrix import LiaisonMatrix
from .base    import ImportResult, ImportError, PartPose
from .mappings import map_process_from_geometry


# ── pythonOCC availability guard ──────────────────────────────────────────────

_OCC_AVAILABLE = False
_OCC_ERROR     = ""

try:
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.TopExp       import TopExp_Explorer
    from OCC.Core.TopAbs       import TopAbs_SOLID
    from OCC.Core.BRepBndLib   import brepbndlib
    from OCC.Core.Bnd          import Bnd_Box
    from OCC.Core.BRepGProp    import brepgprop
    from OCC.Core.GProp        import GProp_GProps
    _OCC_AVAILABLE = True
except ImportError as _e:
    _OCC_ERROR = str(_e)


# ── default density table (g/mm³) ─────────────────────────────────────────────

_DEFAULT_DENSITIES: dict[str, float] = {
    "steel":    7.85e-3,
    "stainless":8.00e-3,
    "iron":     7.87e-3,
    "alumin":   2.70e-3,
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
    "default":  7.85e-3,
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

    Uses the simple STEPControl_Reader + TopExp_Explorer approach which is
    stable across all pythonOCC versions (no XDE label traversal).

    Parameters
    ----------
    path            : path to the .step / .stp file
    density_map     : maps keyword → g/mm³ (substring match, case-insensitive)
    infer_contacts  : if True, infer contacts from bounding-box proximity
    contact_gap_mm  : clearance threshold for proximity-based contact inference
    """
    if not _OCC_AVAILABLE:
        raise ImportError(
            "pythonOCC is required for STEP import.\n"
            "Install with:  conda install -c conda-forge pythonocc-core\n"
            f"(Original error: {_OCC_ERROR})"
        )

    p = Path(path)
    if not p.exists():
        raise ImportError(f"File not found: {path}")

    warnings: list[str] = []

    # ── read STEP file ────────────────────────────────────────────────────────
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(p))
    if status != 1:   # 1 = IFSelect_RetDone
        raise ImportError(f"STEPControl_Reader failed (status={status}) for '{path}'")

    reader.TransferRoots()
    compound = reader.OneShape()

    # ── extract part names from STEP model (best-effort) ──────────────────────
    step_names = _extract_step_names(reader)

    # ── iterate over all solid bodies ─────────────────────────────────────────
    assembly = Assembly(id="ROOT", name=p.stem)
    poses:  dict[str, PartPose] = {}
    shapes: dict[str, Any]      = {}
    bboxes: dict[str, tuple]    = {}
    counter = 0

    explorer = TopExp_Explorer(compound, TopAbs_SOLID)
    while explorer.More():
        counter += 1
        part_id = f"P{counter:04d}"
        solid   = explorer.Current()

        # Name from STEP entities or fallback
        name = step_names.get(counter - 1, "") or f"Part_{counter}"

        # Bounding box
        bbox_mm = _compute_bbox_mm(solid)
        geom    = _bbox_to_geometry(solid, bbox_mm, density_map or {})

        proc = map_process_from_geometry(
            Material.UNKNOWN,
            geom.min_wall_thickness,
            geom.tolerance,
            geom.draft_angle,
            geom.is_fastener,
        )

        part = Part(
            id=part_id, name=name,
            geometry=geom, material=Material.UNKNOWN, process=proc,
        )
        assembly.parts.append(part)

        shapes[part_id] = solid
        bboxes[part_id] = bbox_mm
        poses[part_id]  = PartPose(part_id=part_id)

        explorer.Next()

    if counter == 0:
        raise ImportError(f"No solid bodies found in '{path}'")

    warnings.append(f"Loaded {counter} solid body/bodies from STEP file.")

    # ── build liaison matrix ──────────────────────────────────────────────────
    part_ids = [pt.id for pt in assembly.parts]
    liaison  = LiaisonMatrix(part_ids)

    if infer_contacts:
        _infer_contacts_from_aabb(poses, bboxes, liaison, contact_gap_mm, warnings)

    return ImportResult(
        assembly=assembly,
        liaison=liaison,
        poses=poses,
        shapes=shapes,
        warnings=warnings,
        source="step",
    )


# ── name extraction from STEP model entities ──────────────────────────────────

def _extract_step_names(reader: Any) -> dict[int, str]:
    """
    Best-effort extraction of part/product names from STEP model entities.
    Returns {solid_index: name} mapping.
    """
    names: dict[int, str] = {}
    try:
        model = reader.StepModel()
        if model is None:
            return names
        product_idx = 0
        nb = model.NbEntities()
        for i in range(1, nb + 1):
            entity = model.Value(i)
            etype  = entity.DynamicType().Name()
            if "PRODUCT_DEFINITION_SHAPE" in etype or "PRODUCT_DEFINITION" in etype:
                try:
                    # Try to get name from the entity
                    name_str = str(entity)
                    # Extract quoted name if present
                    if "'" in name_str:
                        parts = name_str.split("'")
                        if len(parts) >= 2 and parts[1].strip():
                            names[product_idx] = parts[1].strip()
                            product_idx += 1
                            continue
                    product_idx += 1
                except Exception:
                    product_idx += 1
    except Exception:
        pass
    return names


# ── geometry helpers ──────────────────────────────────────────────────────────

def _compute_bbox_mm(shape: Any) -> tuple[float, float, float, float, float, float]:
    """Return (xmin,ymin,zmin,xmax,ymax,zmax) in mm from a TopoDS_Shape."""
    try:
        box = Bnd_Box()
        brepbndlib.Add(shape, box)
        if box.IsVoid():
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        xmn, ymn, zmn, xmx, ymx, zmx = box.Get()
        return (xmn, ymn, zmn, xmx, ymx, zmx)
    except Exception:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _bbox_to_geometry(
    shape: Any,
    bbox_mm: tuple,
    density_map: dict[str, float],
) -> Geometry:
    xmn, ymn, zmn, xmx, ymx, zmx = bbox_mm
    length = max(xmx - xmn, 0.0)
    width  = max(ymx - ymn, 0.0)
    height = max(zmx - zmn, 0.0)

    volume_mm3 = length * width * height * 0.5
    if shape is not None:
        try:
            props = GProp_GProps()
            brepgprop.VolumeProperties(shape, props)
            volume_mm3 = abs(props.Mass())
        except Exception:
            pass

    density = _lookup_density("", density_map)
    mass_g  = volume_mm3 * density

    min_dim = min(d for d in (length, width, height) if d > 0) if (length + width + height) > 0 else 0
    wall    = min_dim / 2.0

    return Geometry(
        length=round(length, 3),
        width=round(width,  3),
        height=round(height, 3),
        min_wall_thickness=round(wall, 3),
        mass_grams=round(mass_g, 3),
        tolerance=0.1,
    )


# ── proximity-based contact inference ────────────────────────────────────────

def _infer_contacts_from_aabb(
    poses:   dict[str, PartPose],
    bboxes:  dict[str, tuple],
    liaison: LiaisonMatrix,
    gap_mm:  float,
    warnings: list[str],
) -> None:
    ids = list(bboxes.keys())
    n   = len(ids)
    added = 0

    for i in range(n):
        for j in range(i + 1, n):
            a, b = ids[i], ids[j]
            if liaison.has_contact(a, b):
                continue

            ab = bboxes[a]
            bb = bboxes[b]

            if _aabb_overlap(ab, bb, gap_mm):
                try:
                    liaison.add_contact(a, b, contact_type="face", strength="rigid")
                    added += 1
                except KeyError:
                    pass

    if added:
        warnings.append(
            f"Inferred {added} liaison contact(s) from bounding-box proximity "
            f"(gap ≤ {gap_mm} mm). Review and remove false positives."
        )


def _aabb_overlap(
    a: tuple[float, float, float, float, float, float],
    b: tuple[float, float, float, float, float, float],
    gap: float,
) -> bool:
    for i in range(3):
        if a[i + 3] + gap < b[i] or b[i + 3] + gap < a[i]:
            return False
    return True
