"""
json_importer.py — read an Assembly + LiaisonMatrix from a JSON BOM file.

JSON Schema
───────────
{
  "id":   "ASM001",
  "name": "My Assembly",
  "assembly_directions": 1,
  "parts": [
    {
      "id": "P001", "name": "Base Plate",
      "process":  "cnc_machining",  // see ManufacturingProcess enum values
      "material": "metal",          // see Material enum values
      "quantity": 1,
      "must_move": false, "must_differ_material": false, "must_be_separate": true,
      "geometry": {
        "length": 200, "width": 150, "height": 20,
        "mass_grams": 1500,
        "alpha": 180, "beta": 360,
        "min_wall_thickness": 5.0,
        "fillet_radius": 1.0,
        "draft_angle": 0.0,
        "tolerance": 0.05,
        "is_flexible": false, "can_tangle": false,
        "is_fragile": false,  "needs_2_hands": false,
        "obstructed_access": false, "requires_alignment": false,
        "has_undercuts": false,
        "is_fastener": false, "fastener_clearance_mm": 0.0
      },
      // Optional 3-D pose for collision detection
      "pose": {
        "position": [0, 0, 0],
        "rotation": [[1,0,0],[0,1,0],[0,0,1]]
      }
    }
  ],
  "subassemblies": [],   // same schema as the root assembly (recursive)
  "contacts": [
    {
      "part_a": "P001", "part_b": "P002",
      "contact_type": "face",   // face | edge | point | thread | press_fit
      "strength":     "rigid",  // rigid | flexible | sliding | fixed
      "direction":    "+Z",
      "is_structural": true
    }
  ]
}

Public API
──────────
    result = import_json("my_assembly.json")
    result = import_json_string(json_text)

    # Round-trip helpers (for testing and export)
    text   = assembly_to_json_string(assembly, liaison, poses)
    with open("out.json", "w") as f:
        f.write(text)
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

from dfma.models.part import (
    Assembly, Part, Geometry,
    ManufacturingProcess, Material,
)
from assembly_graph.liaison_matrix import LiaisonMatrix
from .base    import ImportResult, ImportError, PartPose
from .mappings import map_material, map_process_from_geometry


# ── public entry points ───────────────────────────────────────────────────────

def import_json(path: str) -> ImportResult:
    """Load an assembly from a JSON BOM file."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        raise ImportError(f"Cannot read JSON file '{path}': {exc}") from exc
    return _parse(data, source=Path(path).name)


def import_json_string(text: str) -> ImportResult:
    """Load an assembly from a JSON string (useful for testing)."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ImportError(f"Invalid JSON: {exc}") from exc
    return _parse(data, source="<string>")


# ── round-trip export ─────────────────────────────────────────────────────────

def assembly_to_json_string(
    assembly: Assembly,
    liaison:  LiaisonMatrix | None = None,
    poses:    dict[str, PartPose] | None = None,
    indent:   int = 2,
) -> str:
    """
    Serialise an Assembly (and optionally a LiaisonMatrix + poses) back to the
    JSON BOM schema.  Useful for creating test fixtures from demo assemblies.
    """
    data = _assembly_to_dict(assembly, poses or {})
    if liaison is not None:
        data["contacts"] = _liaison_to_list(liaison)
    return json.dumps(data, indent=indent, ensure_ascii=False)


# ── internal parser ───────────────────────────────────────────────────────────

def _parse(data: dict, source: str) -> ImportResult:
    warnings: list[str] = []
    assembly = _dict_to_assembly(data, warnings)

    # Build liaison matrix over all (flat) parts
    all_parts = assembly.all_parts()
    part_ids  = [p.id for p in all_parts]
    liaison   = LiaisonMatrix(part_ids)

    for c in data.get("contacts", []):
        try:
            liaison.add_contact(
                c["part_a"],
                c["part_b"],
                contact_type=c.get("contact_type", "face"),
                strength=c.get("strength",      "rigid"),
                direction=c.get("direction",     ""),
                is_structural=c.get("is_structural", True),
            )
        except (KeyError, ValueError) as exc:
            warnings.append(f"Skipped contact {c}: {exc}")

    # Parse poses
    poses: dict[str, PartPose] = {}
    _collect_poses(data, poses, warnings)

    return ImportResult(
        assembly=assembly,
        liaison=liaison,
        poses=poses,
        warnings=warnings,
        source="json",
    )


def _dict_to_assembly(d: dict, warnings: list[str]) -> Assembly:
    asm = Assembly(
        id=d.get("id",   "ASM"),
        name=d.get("name", "Assembly"),
        assembly_directions=d.get("assembly_directions", 1),
    )

    for pd in d.get("parts", []):
        try:
            asm.parts.append(_dict_to_part(pd, warnings))
        except Exception as exc:
            warnings.append(f"Skipped part {pd.get('id','?')}: {exc}")

    for sd in d.get("subassemblies", []):
        try:
            asm.subassemblies.append(_dict_to_assembly(sd, warnings))
        except Exception as exc:
            warnings.append(f"Skipped subassembly {sd.get('id','?')}: {exc}")

    return asm


def _dict_to_part(d: dict, warnings: list[str]) -> Part:
    pid  = d["id"]
    name = d.get("name", pid)

    # Process
    proc_str = d.get("process", "")
    try:
        process = ManufacturingProcess(proc_str) if proc_str else ManufacturingProcess.UNKNOWN
    except ValueError:
        warnings.append(f"Part {pid}: unknown process '{proc_str}', using UNKNOWN")
        process = ManufacturingProcess.UNKNOWN

    # Material
    mat_str = d.get("material", "")
    try:
        material = Material(mat_str) if mat_str else Material.UNKNOWN
    except ValueError:
        # Try keyword mapping for free-text material names
        material = map_material(mat_str)
        if material == Material.UNKNOWN:
            warnings.append(f"Part {pid}: unknown material '{mat_str}', using UNKNOWN")

    geom = _dict_to_geometry(d.get("geometry", {}))

    # Infer process from geometry if not supplied
    if process == ManufacturingProcess.UNKNOWN and (
        geom.length > 0 or geom.min_wall_thickness > 0
    ):
        process = map_process_from_geometry(
            material, geom.min_wall_thickness, geom.tolerance,
            geom.draft_angle, geom.is_fastener,
        )

    return Part(
        id=pid,
        name=name,
        process=process,
        material=material,
        geometry=geom,
        must_move=d.get("must_move",            False),
        must_differ_material=d.get("must_differ_material", False),
        must_be_separate=d.get("must_be_separate",    False),
        quantity=d.get("quantity", 1),
    )


def _dict_to_geometry(d: dict) -> Geometry:
    def f(k: str, default: float = 0.0) -> float:
        return float(d.get(k, default))
    def b(k: str, default: bool = False) -> bool:
        return bool(d.get(k, default))
    return Geometry(
        length=f("length"), width=f("width"), height=f("height"),
        min_wall_thickness=f("min_wall_thickness"),
        fillet_radius=f("fillet_radius"),
        draft_angle=f("draft_angle"),
        tolerance=f("tolerance", 0.1),
        mass_grams=f("mass_grams"),
        alpha=f("alpha", 360.0), beta=f("beta", 360.0),
        is_flexible=b("is_flexible"), can_tangle=b("can_tangle"),
        is_fragile=b("is_fragile"), needs_2_hands=b("needs_2_hands"),
        obstructed_access=b("obstructed_access"),
        requires_alignment=b("requires_alignment"),
        has_undercuts=b("has_undercuts"),
        is_fastener=b("is_fastener"),
        fastener_clearance_mm=f("fastener_clearance_mm"),
    )


def _collect_poses(data: dict, poses: dict[str, PartPose], warnings: list[str]) -> None:
    """Recursively collect pose entries from parts/subassemblies."""
    for pd in data.get("parts", []):
        if "pose" in pd:
            try:
                poses[pd["id"]] = _dict_to_pose(pd["id"], pd["pose"])
            except Exception as exc:
                warnings.append(f"Bad pose for {pd.get('id','?')}: {exc}")
    for sd in data.get("subassemblies", []):
        _collect_poses(sd, poses, warnings)


def _dict_to_pose(part_id: str, d: dict) -> PartPose:
    pos = tuple(float(v) for v in d.get("position", [0, 0, 0]))
    rot_raw = d.get("rotation", [[1,0,0],[0,1,0],[0,0,1]])
    rot = tuple(tuple(float(v) for v in row) for row in rot_raw)
    if len(pos) != 3:
        raise ValueError("pose.position must have 3 elements")
    if len(rot) != 3 or any(len(r) != 3 for r in rot):
        raise ValueError("pose.rotation must be a 3×3 matrix")
    return PartPose(part_id=part_id, position=pos, rotation=rot)


# ── serialiser helpers ────────────────────────────────────────────────────────

def _assembly_to_dict(
    asm: Assembly,
    poses: dict[str, PartPose],
) -> dict:
    d: dict[str, Any] = {
        "id":   asm.id,
        "name": asm.name,
        "assembly_directions": asm.assembly_directions,
        "parts": [_part_to_dict(p, poses) for p in asm.parts],
        "subassemblies": [_assembly_to_dict(s, poses) for s in asm.subassemblies],
        "contacts": [],   # filled at top level by assembly_to_json_string
    }
    return d


def _part_to_dict(p: Part, poses: dict[str, PartPose]) -> dict:
    g = p.geometry
    d: dict[str, Any] = {
        "id":   p.id,
        "name": p.name,
        "process":  p.process.value,
        "material": p.material.value,
        "quantity": p.quantity,
        "must_move":            p.must_move,
        "must_differ_material": p.must_differ_material,
        "must_be_separate":     p.must_be_separate,
        "geometry": {
            "length": g.length, "width": g.width, "height": g.height,
            "min_wall_thickness": g.min_wall_thickness,
            "fillet_radius":      g.fillet_radius,
            "draft_angle":        g.draft_angle,
            "tolerance":          g.tolerance,
            "mass_grams":         g.mass_grams,
            "alpha": g.alpha, "beta": g.beta,
            "is_flexible":        g.is_flexible,
            "can_tangle":         g.can_tangle,
            "is_fragile":         g.is_fragile,
            "needs_2_hands":      g.needs_2_hands,
            "obstructed_access":  g.obstructed_access,
            "requires_alignment": g.requires_alignment,
            "has_undercuts":      g.has_undercuts,
            "is_fastener":        g.is_fastener,
            "fastener_clearance_mm": g.fastener_clearance_mm,
        },
    }
    if p.id in poses:
        pose = poses[p.id]
        d["pose"] = {
            "position": list(pose.position),
            "rotation": [list(row) for row in pose.rotation],
        }
    return d


def _liaison_to_list(liaison: LiaisonMatrix) -> list[dict]:
    contacts = []
    for (a, b), contact in liaison._contacts.items():
        contacts.append({
            "part_a":       contact.part_a,
            "part_b":       contact.part_b,
            "contact_type": contact.contact_type,
            "strength":     contact.strength,
            "direction":    contact.direction,
            "is_structural": contact.is_structural,
        })
    return contacts
