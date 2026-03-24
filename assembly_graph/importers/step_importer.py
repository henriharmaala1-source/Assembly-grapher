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

# ── arm faulthandler BEFORE any OCC imports ───────────────────────────────────
# Must be the very first import so that a segfault during OCC initialisation
# still produces a traceback in step_debug.log.fault.
from assembly_graph.importers._debug import dbg  # noqa: E402

dbg("step_importer.py: top-level import starting")

from pathlib import Path
from typing import Any

from dfma.models.part import (
    Assembly, Part, Geometry,
    ManufacturingProcess, Material,
)
from assembly_graph.liaison_matrix import LiaisonMatrix
from .base    import ImportResult, ImportError, PartPose
from .mappings import map_process_from_geometry

dbg("step_importer.py: non-OCC imports OK — importing OCC now...")

# ── pythonOCC availability guard ──────────────────────────────────────────────

_OCC_AVAILABLE = False
_OCC_ERROR     = ""

try:
    dbg("  importing OCC.Core.STEPControl...")
    from OCC.Core.STEPControl import STEPControl_Reader
    dbg("  importing OCC.Core.TopExp...")
    from OCC.Core.TopExp       import TopExp_Explorer
    dbg("  importing OCC.Core.TopAbs...")
    from OCC.Core.TopAbs       import TopAbs_SOLID
    dbg("  importing OCC.Core.BRepBndLib...")
    from OCC.Core.BRepBndLib   import brepbndlib
    dbg("  importing OCC.Core.Bnd...")
    from OCC.Core.Bnd          import Bnd_Box
    dbg("  importing OCC.Core.BRepGProp...")
    from OCC.Core.BRepGProp    import brepgprop
    dbg("  importing OCC.Core.GProp...")
    from OCC.Core.GProp        import GProp_GProps
    dbg("  importing OCC.Core.BRepCheck...")
    from OCC.Core.BRepCheck    import BRepCheck_Analyzer
    dbg("  importing OCC.Core.TopoDS...")
    from OCC.Core.TopoDS       import TopoDS_Shape
    _OCC_AVAILABLE = True
    dbg("step_importer.py: all OCC imports OK")
except ImportError as _e:
    _OCC_ERROR = str(_e)
    dbg(f"step_importer.py: OCC ImportError — {_e}")

# Keep the old _dbg name as an alias so load_tab.py can still do
#   from assembly_graph.importers.step_importer import _dbg
_dbg = dbg


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
    dbg("=" * 60)
    dbg(f"import_step called: path={path}")

    if not _OCC_AVAILABLE:
        raise ImportError(
            "pythonOCC is required for STEP import.\n"
            "Install with:  conda install -c conda-forge pythonocc-core\n"
            f"(Original error: {_OCC_ERROR})"
        )

    p = Path(path)
    if not p.exists():
        raise ImportError(f"File not found: {path}")

    dbg(f"  file size: {p.stat().st_size} bytes")
    warnings: list[str] = []

    # ── read STEP file ────────────────────────────────────────────────────────
    dbg("Creating STEPControl_Reader...")
    reader = STEPControl_Reader()
    dbg("Calling ReadFile...")
    status = reader.ReadFile(str(p))
    dbg(f"ReadFile returned status={status}")
    if status != 1:   # 1 = IFSelect_RetDone
        raise ImportError(f"STEPControl_Reader failed (status={status}) for '{path}'")

    dbg("Calling TransferRoots...")
    reader.TransferRoots()
    dbg("Calling OneShape...")
    compound = reader.OneShape()
    dbg(f"OneShape returned: {type(compound).__name__}  IsNull={compound.IsNull()}")

    if compound.IsNull():
        raise ImportError(f"STEP file produced an empty compound shape: '{path}'")

    # ── extract part names from STEP model (best-effort) ──────────────────────
    dbg("Extracting part names (best-effort)...")
    step_names = _extract_step_names(reader)
    dbg(f"  extracted {len(step_names)} name(s): {list(step_names.values())[:5]}")

    # ── iterate over all solid bodies ─────────────────────────────────────────
    assembly = Assembly(id="ROOT", name=p.stem)
    poses:  dict[str, PartPose] = {}
    shapes: dict[str, Any]      = {}
    bboxes: dict[str, tuple]    = {}
    counter = 0

    dbg("Starting TopExp_Explorer over SOLID shapes...")
    explorer = TopExp_Explorer(compound, TopAbs_SOLID)
    while explorer.More():
        counter += 1
        part_id = f"P{counter:04d}"
        dbg(f"  [{counter}] calling Current()...")
        solid = explorer.Current()
        dbg(f"  [{counter}] Current() OK  type={type(solid).__name__}  IsNull={solid.IsNull()}")

        if solid.IsNull():
            dbg(f"  [{counter}] SKIPPING — null shape")
            explorer.Next()
            continue

        name = step_names.get(counter - 1, "") or f"Part_{counter}"
        dbg(f"  [{counter}] name={name!r}")

        dbg(f"  [{counter}] computing bbox...")
        bbox_mm = _compute_bbox_mm(solid, counter)
        dbg(f"  [{counter}] bbox={bbox_mm}")

        dbg(f"  [{counter}] computing geometry / volume...")
        geom = _bbox_to_geometry(solid, bbox_mm, density_map or {}, counter)
        dbg(f"  [{counter}] L={geom.length} W={geom.width} H={geom.height} mass={geom.mass_grams}g")

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

        dbg(f"  [{counter}] calling Next()...")
        explorer.Next()
        dbg(f"  [{counter}] Next() OK")

    dbg(f"Explorer loop done. counter={counter}")

    if counter == 0:
        raise ImportError(f"No solid bodies found in '{path}'")

    warnings.append(f"Loaded {counter} solid body/bodies from STEP file.")

    # ── build liaison matrix ──────────────────────────────────────────────────
    dbg("Building LiaisonMatrix...")
    part_ids = [pt.id for pt in assembly.parts]
    liaison  = LiaisonMatrix(part_ids)

    if infer_contacts:
        dbg("Inferring contacts from AABB proximity...")
        _infer_contacts_from_aabb(poses, bboxes, liaison, contact_gap_mm, warnings)

    dbg("Building ImportResult...")
    result = ImportResult(
        assembly=assembly,
        liaison=liaison,
        poses=poses,
        shapes=shapes,
        warnings=warnings,
        source="step",
    )
    dbg("import_step COMPLETE — returning successfully")
    return result


# ── name extraction from STEP model entities ──────────────────────────────────

def _extract_step_names(reader: Any) -> dict[int, str]:
    """
    Best-effort extraction of part/product names from STEP model entities.
    Returns {solid_index: name} mapping.

    Known crash vector: entity.DynamicType() can return a null handle in
    pythonOCC.  We guard every access and bail out on any error.
    """
    names: dict[int, str] = {}
    try:
        model = reader.StepModel()
        if model is None:
            dbg("  _extract_step_names: StepModel() returned None")
            return names

        nb = model.NbEntities()
        dbg(f"  _extract_step_names: NbEntities={nb}")

        product_idx = 0
        for i in range(1, nb + 1):
            try:
                entity = model.Value(i)
                if entity is None:
                    continue

                # Guard against null DynamicType handle — this is the most
                # common segfault source in pythonOCC STEP traversal.
                dt = entity.DynamicType()
                if dt is None:
                    continue
                etype = dt.Name()
                if etype is None:
                    continue

                if "PRODUCT_DEFINITION_SHAPE" not in etype and "PRODUCT_DEFINITION" not in etype:
                    continue

                # Try to extract a human-readable name from the entity string
                name_str = str(entity)
                if "'" in name_str:
                    parts_list = name_str.split("'")
                    if len(parts_list) >= 2 and parts_list[1].strip():
                        names[product_idx] = parts_list[1].strip()
                product_idx += 1

            except Exception as exc:
                dbg(f"  _extract_step_names: entity {i} error: {exc}")
                continue

    except Exception as exc:
        dbg(f"  _extract_step_names: outer error: {exc}")

    return names


# ── geometry helpers ──────────────────────────────────────────────────────────

def _is_shape_valid(shape: Any) -> bool:
    """
    Quick validity check via BRepCheck_Analyzer.
    Returns True if the shape passes — False means skip expensive operations.
    Errors are treated as invalid (False).
    """
    try:
        analyzer = BRepCheck_Analyzer(shape)
        return bool(analyzer.IsValid())
    except Exception as exc:
        dbg(f"    BRepCheck_Analyzer error: {exc}")
        return False


def _compute_bbox_mm(shape: Any, idx: int = 0) -> tuple[float, float, float, float, float, float]:
    """Return (xmin,ymin,zmin,xmax,ymax,zmax) in mm from a TopoDS_Shape."""
    try:
        box = Bnd_Box()
        dbg(f"    [{idx}] brepbndlib.Add...")
        brepbndlib.Add(shape, box)
        dbg(f"    [{idx}] brepbndlib.Add OK  IsVoid={box.IsVoid()}")
        if box.IsVoid():
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        xmn, ymn, zmn, xmx, ymx, zmx = box.Get()
        return (xmn, ymn, zmn, xmx, ymx, zmx)
    except Exception as exc:
        dbg(f"    [{idx}] _compute_bbox_mm error: {exc}")
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _bbox_to_geometry(
    shape: Any,
    bbox_mm: tuple,
    density_map: dict[str, float],
    idx: int = 0,
) -> Geometry:
    xmn, ymn, zmn, xmx, ymx, zmx = bbox_mm
    length = max(xmx - xmn, 0.0)
    width  = max(ymx - ymn, 0.0)
    height = max(zmx - zmn, 0.0)

    # Use BRepGProp for accurate volume only when the shape passes validity.
    # VolumeProperties segfaults on open shells / non-manifold solids.
    volume_mm3 = length * width * height * 0.5  # conservative default
    if shape is not None:
        dbg(f"    [{idx}] checking shape validity before VolumeProperties...")
        valid = _is_shape_valid(shape)
        dbg(f"    [{idx}] shape valid={valid}")
        if valid:
            try:
                dbg(f"    [{idx}] brepgprop.VolumeProperties...")
                props = GProp_GProps()
                brepgprop.VolumeProperties(shape, props)
                volume_mm3 = abs(props.Mass())
                dbg(f"    [{idx}] VolumeProperties OK  vol={volume_mm3:.3f} mm³")
            except Exception as exc:
                dbg(f"    [{idx}] VolumeProperties error: {exc}")
        else:
            dbg(f"    [{idx}] skipping VolumeProperties on invalid shape")

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

            if _aabb_overlap(bboxes[a], bboxes[b], gap_mm):
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
