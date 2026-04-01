"""
step_importer.py — parse STEP AP214 / AP242 assemblies via pythonOCC or OCP.

Supports two OCC Python backends (tried in order):
  1. OCP  — cadquery-ocp (pip install cadquery)
  2. OCC  — pythonocc-core (conda install -c conda-forge pythonocc-core)

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

# ── arm faulthandler BEFORE any OCC/OCP imports ───────────────────────────────
from assembly_graph.importers._debug import dbg

dbg("step_importer.py: module loading...")

from pathlib import Path
from typing import Any, Callable

from dfma.models.part import (
    Assembly, Part, Geometry,
    ManufacturingProcess, Material,
)
from assembly_graph.liaison_matrix import LiaisonMatrix
from .base    import ImportResult, ImportError, PartPose
from .mappings import map_process_from_geometry


# ── OCC/OCP backend — try OCP (cadquery-ocp) first, then pythonocc-core ───────

_OCC_AVAILABLE = False
_OCC_ERROR     = ""
_OCC_BACKEND   = ""   # "ocp" or "occ"

# These callables wrap the one API difference between backends:
#   OCP  uses class-level static methods: BRepBndLib.Add_s(shape, box)
#   OCC  uses module-level singletons:    brepbndlib.Add(shape, box)
_brepbndlib_add:    Callable | None = None
_brepgprop_volume:  Callable | None = None

dbg("step_importer.py: trying OCP backend...")
try:
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopExp       import TopExp_Explorer
    from OCP.TopAbs       import TopAbs_SOLID
    from OCP.BRepBndLib   import BRepBndLib  as _BRepBndLib
    from OCP.Bnd          import Bnd_Box
    from OCP.BRepGProp    import BRepGProp   as _BRepGProp
    from OCP.GProp        import GProp_GProps
    from OCP.BRepCheck    import BRepCheck_Analyzer

    def _brepbndlib_add(shape: Any, box: Any) -> None:   # type: ignore[misc]
        _BRepBndLib.Add_s(shape, box)

    def _brepgprop_volume(shape: Any, props: Any) -> None:  # type: ignore[misc]
        _BRepGProp.VolumeProperties_s(shape, props)

    _OCC_AVAILABLE = True
    _OCC_BACKEND   = "ocp"
    dbg("step_importer.py: OCP backend loaded OK (cadquery-ocp)")

except ImportError as _ocp_err:
    dbg(f"step_importer.py: OCP not available ({_ocp_err}), trying OCC...")
    try:
        from OCC.Core.STEPControl import STEPControl_Reader   # type: ignore[no-redef]
        from OCC.Core.TopExp       import TopExp_Explorer     # type: ignore[no-redef]
        from OCC.Core.TopAbs       import TopAbs_SOLID        # type: ignore[no-redef]
        from OCC.Core.BRepBndLib   import brepbndlib  as _brepbndlib
        from OCC.Core.Bnd          import Bnd_Box             # type: ignore[no-redef]
        from OCC.Core.BRepGProp    import brepgprop   as _brepgprop
        from OCC.Core.GProp        import GProp_GProps        # type: ignore[no-redef]
        from OCC.Core.BRepCheck    import BRepCheck_Analyzer  # type: ignore[no-redef]

        def _brepbndlib_add(shape: Any, box: Any) -> None:   # type: ignore[misc]
            _brepbndlib.Add(shape, box)

        def _brepgprop_volume(shape: Any, props: Any) -> None:  # type: ignore[misc]
            _brepgprop.VolumeProperties(shape, props)

        _OCC_AVAILABLE = True
        _OCC_BACKEND   = "occ"
        dbg("step_importer.py: OCC backend loaded OK (pythonocc-core)")

    except ImportError as _occ_err:
        _OCC_ERROR = f"OCP: {_ocp_err} | OCC: {_occ_err}"
        dbg(f"step_importer.py: neither backend available — {_OCC_ERROR}")

# Keep backward-compat alias so load_tab.py can still import _dbg from here.
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
    lower  = material_name.lower()
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

    Supports both the OCP backend (cadquery-ocp, pip install cadquery)
    and the OCC backend (pythonocc-core, conda install pythonocc-core).

    Parameters
    ----------
    path            : path to the .step / .stp file
    density_map     : maps keyword → g/mm³ (substring match, case-insensitive)
    infer_contacts  : if True, infer contacts from bounding-box proximity
    contact_gap_mm  : clearance threshold for proximity-based contact inference
    """
    dbg("=" * 60)
    dbg(f"import_step called: path={path}  backend={_OCC_BACKEND!r}")

    if not _OCC_AVAILABLE:
        raise ImportError(
            "No OCC backend found. Install one of:\n"
            "  pip install cadquery          (recommended — OCP backend)\n"
            "  conda install -c conda-forge pythonocc-core  (OCC backend)\n"
            f"\nDetected error: {_OCC_ERROR}"
        )

    p = Path(path)
    if not p.exists():
        raise ImportError(f"File not found: {path}")

    dbg(f"  file size: {p.stat().st_size} bytes")
    warnings: list[str] = []

    # ── read STEP file ────────────────────────────────────────────────────────
    dbg("STEPControl_Reader().ReadFile()...")
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(p))
    dbg(f"  ReadFile status={status}")

    # Status is an enum in OCP, an int in OCC — check both ways
    ok = (status == 1) or (hasattr(status, 'name') and 'Done' in status.name)
    if not ok:
        raise ImportError(
            f"STEPControl_Reader failed (status={status}) reading '{path}'.\n"
            "The file may be corrupt or use an unsupported STEP dialect."
        )

    dbg("TransferRoots()...")
    reader.TransferRoots()
    dbg("OneShape()...")
    compound = reader.OneShape()
    dbg(f"  compound IsNull={compound.IsNull()}")

    if compound.IsNull():
        raise ImportError(f"STEP file produced no geometry: '{path}'")

    # ── extract part names (best-effort, never crashes the import) ────────────
    dbg("_extract_step_names()...")
    step_names = _extract_step_names(reader)
    dbg(f"  {len(step_names)} name(s) found")

    # ── walk solid bodies ─────────────────────────────────────────────────────
    assembly = Assembly(id="ROOT", name=p.stem)
    poses:  dict[str, PartPose] = {}
    shapes: dict[str, Any]      = {}
    bboxes: dict[str, tuple]    = {}
    counter = 0

    dbg("TopExp_Explorer over TopAbs_SOLID...")
    explorer = TopExp_Explorer(compound, TopAbs_SOLID)
    while explorer.More():
        counter += 1
        part_id = f"P{counter:04d}"

        dbg(f"  [{counter}] Current()...")
        solid = explorer.Current()
        dbg(f"  [{counter}]   IsNull={solid.IsNull()}")

        if solid.IsNull():
            dbg(f"  [{counter}]   skipping null solid")
            explorer.Next()
            continue

        name    = step_names.get(counter - 1, "") or f"Part_{counter}"
        bbox_mm = _compute_bbox_mm(solid, counter)
        geom    = _bbox_to_geometry(solid, bbox_mm, density_map or {}, counter)

        dbg(f"  [{counter}]   name={name!r}  L={geom.length} W={geom.width} H={geom.height}  mass={geom.mass_grams}g")

        proc = map_process_from_geometry(
            Material.UNKNOWN,
            geom.min_wall_thickness,
            geom.tolerance,
            geom.draft_angle,
            geom.is_fastener,
        )

        assembly.parts.append(Part(
            id=part_id, name=name,
            geometry=geom, material=Material.UNKNOWN, process=proc,
        ))
        shapes[part_id] = solid
        bboxes[part_id] = bbox_mm
        poses[part_id]  = PartPose(part_id=part_id)

        dbg(f"  [{counter}] Next()...")
        explorer.Next()

    dbg(f"Explorer done — {counter} solid(s)")

    if counter == 0:
        raise ImportError(
            f"No solid bodies found in '{path}'.\n"
            "The file may contain only surfaces, wireframes, or assembly references."
        )

    warnings.append(
        f"Loaded {counter} solid body/bodies via {_OCC_BACKEND.upper()} backend."
    )

    # ── build liaison matrix ──────────────────────────────────────────────────
    dbg("LiaisonMatrix + contact inference...")
    part_ids = [pt.id for pt in assembly.parts]
    liaison  = LiaisonMatrix(part_ids)

    if infer_contacts:
        _infer_contacts_from_aabb(poses, bboxes, liaison, contact_gap_mm, warnings)

    result = ImportResult(
        assembly=assembly,
        liaison=liaison,
        poses=poses,
        shapes=shapes,
        warnings=warnings,
        source="step",
    )
    dbg("import_step COMPLETE")
    return result


# ── name extraction ────────────────────────────────────────────────────────────

def _extract_step_names(reader: Any) -> dict[int, str]:
    """
    Best-effort part name extraction from STEP model entities.

    Works with both OCP entity names  (e.g. "StepRepr_ProductDefinitionShape")
    and OCC entity names              (e.g. "PRODUCT_DEFINITION_SHAPE").
    Null DynamicType handles are skipped safely.
    """
    names: dict[int, str] = {}
    try:
        model = reader.StepModel()
        if model is None:
            return names

        nb = model.NbEntities()
        dbg(f"  _extract_step_names: {nb} entities")
        product_idx = 0

        for i in range(1, nb + 1):
            try:
                entity = model.Value(i)
                if entity is None:
                    continue

                dt = entity.DynamicType()
                if dt is None:
                    continue
                etype = dt.Name()
                if not etype:
                    continue

                # Match both "StepRepr_ProductDefinitionShape" (OCP)
                #       and "PRODUCT_DEFINITION_SHAPE"          (OCC)
                etype_upper = etype.upper()
                if "PRODUCT_DEFINITION_SHAPE" not in etype_upper and \
                   "PRODUCT_DEFINITION" not in etype_upper:
                    continue

                name_str = str(entity)
                if "'" in name_str:
                    parts_list = name_str.split("'")
                    if len(parts_list) >= 2 and parts_list[1].strip():
                        names[product_idx] = parts_list[1].strip()
                product_idx += 1

            except Exception as exc:
                dbg(f"  _extract_step_names entity {i}: {exc}")

    except Exception as exc:
        dbg(f"  _extract_step_names outer: {exc}")

    return names


# ── geometry helpers ──────────────────────────────────────────────────────────

def _is_shape_valid(shape: Any) -> bool:
    """True if BRepCheck_Analyzer passes. Errors treated as invalid."""
    try:
        return bool(BRepCheck_Analyzer(shape).IsValid())
    except Exception as exc:
        dbg(f"    BRepCheck_Analyzer: {exc}")
        return False


def _compute_bbox_mm(
    shape: Any, idx: int = 0,
) -> tuple[float, float, float, float, float, float]:
    """Return (xmin,ymin,zmin,xmax,ymax,zmax) in mm."""
    try:
        box = Bnd_Box()
        dbg(f"    [{idx}] brepbndlib_add...")
        _brepbndlib_add(shape, box)  # type: ignore[misc]
        dbg(f"    [{idx}] brepbndlib_add OK  IsVoid={box.IsVoid()}")
        if box.IsVoid():
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return tuple(box.Get())  # type: ignore[return-value]
    except Exception as exc:
        dbg(f"    [{idx}] _compute_bbox_mm: {exc}")
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

    # Default: half of bounding-box volume (conservative hollow estimate)
    volume_mm3 = length * width * height * 0.5

    if shape is not None:
        dbg(f"    [{idx}] shape validity check...")
        valid = _is_shape_valid(shape)
        dbg(f"    [{idx}]   valid={valid}")
        if valid:
            try:
                props = GProp_GProps()
                dbg(f"    [{idx}] VolumeProperties_s...")
                _brepgprop_volume(shape, props)  # type: ignore[misc]
                volume_mm3 = abs(props.Mass())
                dbg(f"    [{idx}]   volume={volume_mm3:.3f} mm³")
            except Exception as exc:
                dbg(f"    [{idx}] VolumeProperties: {exc}")

    density = _lookup_density("", density_map)
    mass_g  = volume_mm3 * density
    min_dim = min((d for d in (length, width, height) if d > 0), default=0)

    return Geometry(
        length=round(length, 3),
        width=round(width,   3),
        height=round(height, 3),
        min_wall_thickness=round(min_dim / 2.0, 3),
        mass_grams=round(mass_g, 3),
        tolerance=0.1,
    )


# ── proximity-based contact inference ─────────────────────────────────────────

def _infer_contacts_from_aabb(
    poses:    dict[str, PartPose],
    bboxes:   dict[str, tuple],
    liaison:  LiaisonMatrix,
    gap_mm:   float,
    warnings: list[str],
) -> None:
    ids   = list(bboxes.keys())
    added = 0
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if not liaison.has_contact(a, b) and _aabb_overlap(bboxes[a], bboxes[b], gap_mm):
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
