"""
test_importers.py — tests for JSON BOM importer, STEP importer, and collision detection.

Tests that don't require pythonOCC (JSON, collision geometry, round-trips):
  run with:  python test_importers.py

STEP tests are skipped automatically when pythonOCC is unavailable.
"""

from __future__ import annotations
import json, sys, math, tempfile, os

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
SKIP = "\033[33mSKIP\033[0m"

_results: list[tuple[str, bool | None, str]] = []

def check(name: str, condition: bool, detail: str = "") -> None:
    _results.append((name, condition, detail))
    status = PASS if condition else FAIL
    print(f"  [{status}] {name}" + (f"  ({detail})" if detail else ""))

def skip(name: str, reason: str = "") -> None:
    _results.append((name, None, reason))
    print(f"  [{SKIP}] {name}" + (f"  — {reason}" if reason else ""))

def section(title: str) -> None:
    print(f"\n{'─'*60}\n  {title}\n{'─'*60}")


# ── imports ────────────────────────────────────────────────────────────────────

from assembly_graph.importers.base     import PartPose, CollisionPair, detect_format
from assembly_graph.importers.mappings import map_material, map_process_from_geometry
from assembly_graph.importers.json_importer import (
    import_json, import_json_string, assembly_to_json_string,
)
from assembly_graph.importers.collision import check_collisions, CollisionReport
from assembly_graph.importers import load_assembly, ImportError

from dfma.models.part import Material, ManufacturingProcess


# ──────────────────────────────────────────────────────────────────────────────
# TEST 1 — detect_format
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 1 — Format detection")

check("detect_format .json",    detect_format("asm.json")    == "json")
check("detect_format .step",    detect_format("part.step")   == "step")
check("detect_format .stp",     detect_format("part.STP")    == "step")
check("detect_format .sldasm",  detect_format("asm.sldasm")  == "solidworks")
check("detect_format .prt",     detect_format("asm.prt")     == "nx")
check("detect_format .plmxml",  detect_format("asm.plmxml")  == "plmxml")
check("detect_format unknown",  detect_format("asm.iges")    == "unknown")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 2 — mappings: material
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 2 — Material mappings")

check("'AISI 1045 Steel' → METAL",   map_material("AISI 1045 Steel")  == Material.METAL)
check("'Aluminium 6061-T6' → METAL", map_material("Aluminium 6061-T6") == Material.METAL)
check("'ABS Plastic' → PLASTIC",     map_material("ABS Plastic")       == Material.PLASTIC)
check("'Nylon PA66' → PLASTIC",      map_material("Nylon PA66")        == Material.PLASTIC)
check("'EPDM Rubber' → RUBBER",      map_material("EPDM Rubber")       == Material.RUBBER)
check("'Silicone' → RUBBER",         map_material("Silicone")          == Material.RUBBER)
check("empty string → UNKNOWN",      map_material("")                   == Material.UNKNOWN)
check("unrecognised → UNKNOWN",      map_material("unobtanium")         == Material.UNKNOWN)


# ──────────────────────────────────────────────────────────────────────────────
# TEST 3 — mappings: process heuristic
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 3 — Process heuristics")

from assembly_graph.importers.mappings import map_process_from_geometry as mpfg

check("rubber → INJECTION_MOLDING",
      mpfg(Material.RUBBER,   2.0, 0.1, 0.0) == ManufacturingProcess.INJECTION_MOLDING)
check("plastic → INJECTION_MOLDING",
      mpfg(Material.PLASTIC,  2.0, 0.1, 0.0) == ManufacturingProcess.INJECTION_MOLDING)
check("thin wall, no draft → SHEET_METAL",
      mpfg(Material.METAL,    2.0, 0.1, 0.0) == ManufacturingProcess.SHEET_METAL)
check("tight tolerance → CNC_MACHINING",
      mpfg(Material.METAL,   10.0, 0.02, 0.0) == ManufacturingProcess.CNC_MACHINING)
check("draft angle present, thin → DIE_CASTING",
      mpfg(Material.METAL,    4.0, 0.1, 2.0) == ManufacturingProcess.DIE_CASTING)
check("draft angle, thick wall → SAND_CASTING",
      mpfg(Material.METAL,   10.0, 0.1, 3.0) == ManufacturingProcess.SAND_CASTING)
check("fastener → CNC_MACHINING",
      mpfg(Material.METAL,    1.5, 0.1, 0.0, is_fastener=True) == ManufacturingProcess.CNC_MACHINING)


# ──────────────────────────────────────────────────────────────────────────────
# TEST 4 — JSON round-trip using flange demo assembly
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 4 — JSON round-trip (PN16 flange)")

try:
    from real_assemblies_demo import build_flange_assembly, build_flange_planner

    orig_asm     = build_flange_assembly()
    orig_planner = build_flange_planner()
    orig_parts   = orig_asm.all_parts()

    # Serialise
    json_text = assembly_to_json_string(orig_asm, orig_planner.liaison)
    data      = json.loads(json_text)

    check("flange JSON has 'parts' key",        "parts"    in data)
    check("flange JSON has 'contacts' key",     "contacts" in data)
    check("flange JSON: 5 parts at top level",  len(data["parts"]) == len(orig_parts),
          f"got {len(data['parts'])}")
    expected_contacts = len(orig_planner.liaison._contacts)
    check("flange JSON: correct contact count",
          len(data["contacts"]) == expected_contacts,
          f"got {len(data['contacts'])}, expected {expected_contacts}")

    # Re-import
    result = import_json_string(json_text)
    check("flange round-trip: no import errors",     True)
    check("flange round-trip: same part count",
          len(result.assembly.all_parts()) == len(orig_parts),
          f"{len(result.assembly.all_parts())} vs {len(orig_parts)}")
    check("flange round-trip: liaison matrix connected",
          result.liaison.is_connected(),
          "")

    # Verify part IDs preserved
    rt_ids  = {p.id for p in result.assembly.all_parts()}
    orig_ids = {p.id for p in orig_parts}
    check("flange round-trip: part IDs preserved", rt_ids == orig_ids,
          f"missing: {orig_ids - rt_ids}")

    # Verify contact endpoints preserved
    rt_contacts  = {(min(c.part_a,c.part_b), max(c.part_a,c.part_b))
                    for c in result.liaison._contacts.values()}
    orig_contacts = {k for k in orig_planner.liaison._contacts.keys()}
    check("flange round-trip: contact pairs preserved",
          rt_contacts == orig_contacts,
          f"delta: {orig_contacts.symmetric_difference(rt_contacts)}")

    # Process + material survived
    rt_fl01 = next(p for p in result.assembly.all_parts() if p.id == "FL01")
    check("flange round-trip: FL01 material == METAL",
          rt_fl01.material == Material.METAL, str(rt_fl01.material))
    check("flange round-trip: FL01 process == CNC_MACHINING",
          rt_fl01.process == ManufacturingProcess.CNC_MACHINING, str(rt_fl01.process))

except Exception as exc:
    check(f"flange JSON round-trip: unexpected exception: {exc}", False)
    import traceback; traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 5 — JSON round-trip with pose data
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 5 — JSON round-trip with pose data")

JSON_WITH_POSES = """{
  "id": "TST",
  "name": "Pose Test Assembly",
  "parts": [
    {
      "id": "BASE", "name": "Base",
      "material": "metal", "process": "cnc_machining",
      "geometry": {"length": 100, "width": 80, "height": 20, "mass_grams": 500},
      "pose": {"position": [0, 0, 0], "rotation": [[1,0,0],[0,1,0],[0,0,1]]}
    },
    {
      "id": "COVER", "name": "Cover",
      "material": "metal", "process": "sheet_metal",
      "geometry": {"length": 100, "width": 80, "height": 5, "mass_grams": 200},
      "pose": {"position": [0, 0, 22.5], "rotation": [[1,0,0],[0,1,0],[0,0,1]]}
    },
    {
      "id": "BRACKET", "name": "Bracket",
      "material": "metal", "process": "cnc_machining",
      "geometry": {"length": 40, "width": 40, "height": 60, "mass_grams": 300},
      "pose": {
        "position": [30, 0, 30],
        "rotation": [[0,-1,0],[1,0,0],[0,0,1]]
      }
    }
  ],
  "subassemblies": [],
  "contacts": [
    {"part_a": "BASE",  "part_b": "COVER",   "contact_type": "face", "strength": "rigid"},
    {"part_a": "BASE",  "part_b": "BRACKET", "contact_type": "face", "strength": "rigid"}
  ]
}"""

try:
    result = import_json_string(JSON_WITH_POSES)
    check("pose JSON: 3 parts imported",
          len(result.assembly.all_parts()) == 3,
          f"got {len(result.assembly.all_parts())}")
    check("pose JSON: 3 poses populated",
          len(result.poses) == 3,
          f"got {len(result.poses)}")
    check("pose JSON: COVER pose at (0,0,22.5)",
          result.poses["COVER"].position == (0.0, 0.0, 22.5),
          str(result.poses.get("COVER")))
    check("pose JSON: BRACKET rotation is non-identity",
          not result.poses["BRACKET"].is_identity,
          "")
    check("pose JSON: 2 contacts in liaison",
          len(result.liaison._contacts) == 2, "")
    check("pose JSON: source == 'json'",
          result.source == "json", "")

except Exception as exc:
    check(f"pose JSON: unexpected exception: {exc}", False)
    import traceback; traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 6 — JSON error handling
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 6 — JSON error handling")

# Bad contact: missing part_a
BAD_CONTACT = '{"id":"A","name":"A","parts":[{"id":"P1","name":"x"},{"id":"P2","name":"y"}],' \
              '"contacts":[{"part_a":"P1","part_b":"NONEXISTENT"}]}'
try:
    r = import_json_string(BAD_CONTACT)
    check("bad contact is skipped (not raised)",   True)
    check("bad contact generates a warning",       len(r.warnings) >= 1,
          f"warnings: {r.warnings}")
except ImportError:
    check("bad contact raises ImportError",        True)
except Exception as exc:
    check(f"bad contact: unexpected exception: {exc}", False)

# Malformed JSON
try:
    import_json_string("{bad json}")
    check("malformed JSON raises ImportError", False)
except ImportError:
    check("malformed JSON raises ImportError", True)

# Unknown process string → UNKNOWN (no crash)
UNKNOWN_PROC = '{"id":"X","name":"X","parts":[{"id":"P1","name":"P1","process":"laser_sintering"}]}'
r2 = import_json_string(UNKNOWN_PROC)
p1 = r2.assembly.all_parts()[0]
check("unknown process string → UNKNOWN (no crash)",
      p1.process == ManufacturingProcess.UNKNOWN, str(p1.process))


# ──────────────────────────────────────────────────────────────────────────────
# TEST 7 — PartPose helpers
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 7 — PartPose geometry helpers")

# Identity pose
pose_id = PartPose.identity("X")
check("identity pose.is_identity",       pose_id.is_identity)
pt = pose_id.transform_point((1.0, 2.0, 3.0))
check("identity transform returns input", pt == (1.0, 2.0, 3.0), str(pt))

# Translation only
pose_t = PartPose("X", position=(10.0, 20.0, 30.0))
pt2 = pose_t.transform_point((1.0, 0.0, 0.0))
check("translation: (1,0,0) → (11,20,30)", pt2 == (11.0, 20.0, 30.0), str(pt2))

# 90° rotation around Z: (1,0,0) → (0,1,0)
R90z = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
pose_r = PartPose("X", rotation=R90z)
pt3    = pose_r.transform_point((1.0, 0.0, 0.0))
check("90° Z-rotation: (1,0,0) → (0,1,0)",
      abs(pt3[0]) < 1e-9 and abs(pt3[1]-1.0) < 1e-9 and abs(pt3[2]) < 1e-9,
      str(pt3))
check("rotated pose is not identity", not pose_r.is_identity)


# ──────────────────────────────────────────────────────────────────────────────
# TEST 8 — AABB collision detection: basic geometry
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 8 — AABB collision detection")

from assembly_graph.importers.collision import _aabb_for_part, _aabb_penetration
from assembly_graph.importers.base import ImportResult as IR
from assembly_graph.liaison_matrix import LiaisonMatrix as LM
from dfma.models.part import Part, Geometry, Assembly

def _make_box_part(pid: str, l: float, w: float, h: float) -> Part:
    return Part(id=pid, name=pid, geometry=Geometry(length=l, width=w, height=h))

def _make_result(parts: list, poses: dict) -> IR:
    asm = Assembly(id="T", name="T", parts=parts)
    lm  = LM([p.id for p in parts])
    r   = IR(assembly=asm, liaison=lm, poses=poses, source="test")
    return r

# Two 10×10×10 cubes — one centred at origin, one at (15,0,0): separated by 10mm
p1 = _make_box_part("P1", 10, 10, 10)
p2 = _make_box_part("P2", 10, 10, 10)
result_sep = _make_result([p1, p2], {
    "P1": PartPose("P1", position=(0,  0, 0)),
    "P2": PartPose("P2", position=(15, 0, 0)),
})
report_sep = check_collisions(result_sep, method="aabb", near_miss_mm=1.0)
check("AABB: two separated cubes → no collision",
      len(report_sep.collisions) == 0, str(report_sep.collisions))
check("AABB: separated but gap=10mm > near_miss=1mm → no near-miss either",
      len(report_sep.near_misses) == 0, "")

# Same cubes but overlapping: P2 at (8,0,0) → 2mm overlap
result_col = _make_result([p1, p2], {
    "P1": PartPose("P1", position=(0, 0, 0)),
    "P2": PartPose("P2", position=(8, 0, 0)),
})
report_col = check_collisions(result_col, method="aabb", near_miss_mm=1.0)
check("AABB: overlapping cubes → 1 collision",
      len(report_col.collisions) == 1,
      str(report_col.collisions))
check("AABB: overlap_mm ≈ 2.0",
      abs(report_col.collisions[0].overlap_mm - 2.0) < 0.01,
      f"got {report_col.collisions[0].overlap_mm}")

# Near-miss: P2 at (10.5, 0, 0) → gap = 0.5 mm (< near_miss_mm=1.0)
result_nm = _make_result([p1, p2], {
    "P1": PartPose("P1", position=(0,    0, 0)),
    "P2": PartPose("P2", position=(10.5, 0, 0)),
})
report_nm = check_collisions(result_nm, method="aabb", near_miss_mm=1.0)
check("AABB: near-miss (gap=0.5 mm) → 0 hard collisions",
      len(report_nm.collisions) == 0, "")
check("AABB: near-miss → 1 near-miss entry",
      len(report_nm.near_misses) == 1,
      str(report_nm.near_misses))

# Three-part assembly: P1 collides P2, P1 touches P3, P2 and P3 are separated
p3 = _make_box_part("P3", 10, 10, 10)
result_3 = _make_result([p1, p2, p3], {
    "P1": PartPose("P1", position=(0,   0, 0)),
    "P2": PartPose("P2", position=(8,   0, 0)),
    "P3": PartPose("P3", position=(0,  20, 0)),
})
report_3 = check_collisions(result_3, method="aabb", near_miss_mm=0.5)
check("3-part: exactly 1 collision (P1–P2)",
      len(report_3.collisions) == 1,
      f"collisions: {[(c.part_a,c.part_b) for c in report_3.collisions]}")
check("3-part: P1-P2 are the colliding pair",
      {report_3.collisions[0].part_a, report_3.collisions[0].part_b} == {"P1","P2"},
      "")

# CollisionReport.is_clean
check("CollisionReport.is_clean False when collision exists", not report_col.is_clean)
check("CollisionReport.is_clean True when no collisions",     report_sep.is_clean)


# ──────────────────────────────────────────────────────────────────────────────
# TEST 9 — OBB collision detection
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 9 — OBB collision detection (SAT)")

from assembly_graph.importers.collision import _obb_collide

# Identity rotations — should match AABB
h = (5.0, 5.0, 5.0)   # half-extents of a 10×10×10 cube
I = ((1.0,0.0,0.0),(0.0,1.0,0.0),(0.0,0.0,1.0))

overlap, pen = _obb_collide((0,0,0),I,h, (15,0,0),I,h)
check("OBB: identity rotation, separated → no overlap",
      not overlap, f"pen={pen:.3f}")

overlap2, pen2 = _obb_collide((0,0,0),I,h, (8,0,0),I,h)
check("OBB: identity rotation, overlapping → overlap",
      overlap2, f"pen={pen2:.3f}")
check("OBB: overlap ≈ 2 mm",
      abs(pen2 - 2.0) < 0.5, f"pen={pen2:.3f}")

# 45° rotation around Z for one box: diagonal should still detect overlap at (7,0,0)
cos45 = math.cos(math.pi/4)
sin45 = math.sin(math.pi/4)
R45z  = ((cos45,-sin45,0.0),(sin45,cos45,0.0),(0.0,0.0,1.0))

overlap3, pen3 = _obb_collide((0,0,0),I,h, (7,0,0),R45z,h)
check("OBB: 45° rotated box overlapping at (7,0,0)",
      overlap3, f"pen={pen3:.3f}")

# Clearly separated (100mm apart) — both identity and rotated
overlap4, pen4 = _obb_collide((0,0,0),I,h, (100,0,0),R45z,h)
check("OBB: 100mm apart → no overlap",
      not overlap4, f"pen={pen4:.3f}")

# Full check_collisions via OBB
result_obb = _make_result([p1, p2], {
    "P1": PartPose("P1", position=(0, 0, 0), rotation=I),
    "P2": PartPose("P2", position=(8, 0, 0), rotation=I),
})
report_obb = check_collisions(result_obb, method="obb")
check("OBB check_collisions: overlapping cubes → 1 collision",
      len(report_obb.collisions) == 1, str(report_obb.collisions))


# ──────────────────────────────────────────────────────────────────────────────
# TEST 10 — Collision with ignore_pairs
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 10 — Collision ignore_pairs")

result_col2 = _make_result([p1, p2], {
    "P1": PartPose("P1", position=(0, 0, 0)),
    "P2": PartPose("P2", position=(8, 0, 0)),
})
report_ignored = check_collisions(
    result_col2, method="aabb",
    ignore_pairs={("P1","P2")},
)
check("ignore_pairs: P1–P2 collision skipped → 0 collisions",
      len(report_ignored.collisions) == 0, "")
check("ignore_pairs: 1 skipped pair counted",
      report_ignored.skipped_pairs == 1, f"got {report_ignored.skipped_pairs}")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 11 — BRep collision (skip if pythonOCC unavailable)
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 11 — BRep collision (OCC/OCP required)")

from assembly_graph.importers.collision import _OCC_AVAILABLE as _OCC_OK

def _make_box_shape(dx, dy, dz):
    """Return a TopoDS_Shape box using whichever OCC backend is available."""
    try:
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
        from OCP.gp          import gp_Pnt
        return BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), gp_Pnt(dx,dy,dz)).Shape()
    except ImportError:
        from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
        from OCC.Core.gp          import gp_Pnt
        return BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), gp_Pnt(dx,dy,dz)).Shape()

def _translate_shape(shape, tx, ty, tz):
    """Translate a TopoDS_Shape using whichever OCC backend is available."""
    try:
        from OCP.gp              import gp_Vec, gp_Trsf
        from OCP.BRepBuilderAPI  import BRepBuilderAPI_Transform
        trsf = gp_Trsf(); trsf.SetTranslation(gp_Vec(tx, ty, tz))
        return BRepBuilderAPI_Transform(shape, trsf, True).Shape()
    except ImportError:
        from OCC.Core.gp              import gp_Vec, gp_Trsf
        from OCC.Core.BRepBuilderAPI  import BRepBuilderAPI_Transform
        trsf = gp_Trsf(); trsf.SetTranslation(gp_Vec(tx, ty, tz))
        return BRepBuilderAPI_Transform(shape, trsf, True).Shape()

try:
    if not _OCC_OK:
        raise ImportError("no OCC/OCP backend installed")

    # Build two 10×10×10 boxes; shift second by (8,0,0) → 2mm overlap
    box_a = _make_box_shape(10, 10, 10)
    box_b = _translate_shape(_make_box_shape(10, 10, 10), 8, 0, 0)

    # Build a result with shapes
    r_brep = _make_result([p1, p2], {
        "P1": PartPose("P1", position=(0, 0, 0)),
        "P2": PartPose("P2", position=(8, 0, 0)),
    })
    r_brep.shapes["P1"] = box_a
    r_brep.shapes["P2"] = box_b

    report_brep = check_collisions(r_brep, method="brep", brep_min_vol=0.01)
    check("BRep: overlapping boxes → ≥1 collision",
          len(report_brep.collisions) >= 1,
          str(report_brep.collisions))
    check("BRep: interference volume > 0",
          report_brep.collisions[0].overlap_volume_mm3 > 0 if report_brep.collisions else False,
          f"{report_brep.collisions[0].overlap_volume_mm3 if report_brep.collisions else 'N/A'}")

    # Clearly separated boxes
    box_c = _translate_shape(_make_box_shape(10, 10, 10), 100, 0, 0)
    r_sep2 = _make_result([p1, p2], {
        "P1": PartPose("P1", position=(0,   0, 0)),
        "P2": PartPose("P2", position=(100, 0, 0)),
    })
    r_sep2.shapes["P1"] = box_a
    r_sep2.shapes["P2"] = box_c
    rep_sep2 = check_collisions(r_sep2, method="brep", brep_min_vol=0.01)
    check("BRep: separated boxes → 0 collisions",
          len(rep_sep2.collisions) == 0, "")

except ImportError:
    skip("BRep collision tests", "no OCC/OCP backend")
except Exception as exc:   # pragma: no cover
    check(f"BRep tests: unexpected exception: {exc}", False)
    import traceback; traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 12 — JSON file I/O (round-trip via tempfile)
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 12 — JSON file I/O")

try:
    from real_assemblies_demo import build_bearing_assembly, build_bearing_planner

    bear_asm     = build_bearing_assembly()
    bear_planner = build_bearing_planner()
    json_text    = assembly_to_json_string(bear_asm, bear_planner.liaison)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False, encoding="utf-8"
    ) as f:
        f.write(json_text)
        tmp_path = f.name

    result = import_json(tmp_path)
    check("bearing JSON file I/O: loads from disk",
          len(result.assembly.all_parts()) == len(bear_asm.all_parts()),
          f"got {len(result.assembly.all_parts())}")
    check("bearing JSON file I/O: source == 'json'", result.source == "json")

    # load_assembly() auto-dispatch
    result2 = load_assembly(tmp_path)
    check("load_assembly() dispatches .json correctly",
          result2.source == "json", result2.source)

    os.unlink(tmp_path)

except Exception as exc:
    check(f"JSON file I/O: unexpected exception: {exc}", False)
    import traceback; traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 13 — STEP import (skip if pythonOCC unavailable)
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 13 — STEP importer (OCC/OCP required)")

try:
    if not _OCC_OK:
        raise ImportError("no OCC/OCP backend installed")

    # Use cadquery (OCP) or OCC to write a 100×50×20 box STEP file
    with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as f:
        step_tmp = f.name
    try:
        import cadquery as cq
        cq.exporters.export(cq.Workplane("XY").box(100, 50, 20), step_tmp)
    except ImportError:
        from OCC.Core.BRepPrimAPI     import BRepPrimAPI_MakeBox
        from OCC.Core.STEPControl     import STEPControl_Writer, STEPControl_AsIs
        from OCC.Core.Interface_Static import Interface_Static
        from OCC.Core.gp              import gp_Pnt
        writer = STEPControl_Writer()
        Interface_Static.SetCVal("write.step.schema", "AP214IS")
        box_shape = BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), gp_Pnt(100,50,20)).Shape()
        writer.Transfer(box_shape, STEPControl_AsIs)
        writer.Write(step_tmp)

    from assembly_graph.importers.step_importer import import_step
    step_result = import_step(step_tmp, infer_contacts=False)
    os.unlink(step_tmp)

    check("STEP import: Assembly created",
          step_result.assembly is not None)
    check("STEP import: at least 1 part",
          len(step_result.assembly.all_parts()) >= 1,
          f"got {len(step_result.assembly.all_parts())}")
    check("STEP import: source == 'step'",
          step_result.source == "step")
    check("STEP import: poses populated",
          step_result.has_poses,
          f"poses: {len(step_result.poses)}")
    check("STEP import: shapes populated (for BRep collision)",
          step_result.has_shapes,
          f"shapes: {len(step_result.shapes)}")

    # Bounding box sanity: box is 100×50×20 mm
    part  = step_result.assembly.all_parts()[0]
    check("STEP: bounding box dimensions ≈ 100×50×20 mm",
          abs(part.geometry.length - 100) < 5.0 and
          abs(part.geometry.width  -  50) < 5.0 and
          abs(part.geometry.height -  20) < 5.0,
          f"got {part.geometry.length:.1f}×{part.geometry.width:.1f}×{part.geometry.height:.1f}")

except ImportError:
    skip("STEP importer tests", "no OCC/OCP backend")
except Exception as exc:   # pragma: no cover
    check(f"STEP tests: unexpected exception: {exc}", False)
    import traceback; traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 14 — load_assembly error on unsupported format
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 14 — load_assembly error handling")

caught = False
try:
    load_assembly("drawing.dxf")
except ImportError:
    caught = True
check("load_assembly unsupported format raises ImportError", caught)


# ──────────────────────────────────────────────────────────────────────────────
# TEST 15 — CollisionReport summary text
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 15 — CollisionReport.summary()")

result_s = _make_result([p1, p2], {
    "P1": PartPose("P1", (0,0,0)),
    "P2": PartPose("P2", (8,0,0)),
})
report_s = check_collisions(result_s, method="aabb")
summary  = report_s.summary()
check("summary contains 'COLLISION REPORT'", "COLLISION REPORT" in summary)
check("summary contains 'VIOLATIONS'",       "VIOLATIONS" in summary)

result_clean = _make_result([p1, p2], {
    "P1": PartPose("P1", (0,  0,0)),
    "P2": PartPose("P2", (50, 0,0)),
})
report_clean = check_collisions(result_clean, method="aabb")
check("clean summary contains 'CLEAN'", "CLEAN" in report_clean.summary())


# ──────────────────────────────────────────────────────────────────────────────
# SUMMARY
# ──────────────────────────────────────────────────────────────────────────────

total  = len(_results)
passed = sum(1 for _, ok, _ in _results if ok is True)
skipped = sum(1 for _, ok, _ in _results if ok is None)
failed = total - passed - skipped

print(f"\n{'═'*60}")
print(f"  RESULTS: {passed}/{total-skipped} passed"
      + (f"  — {skipped} skipped" if skipped else "")
      + (f"  — {failed} FAILED"   if failed  else "  — all passed"))
print(f"{'═'*60}")

if failed:
    print("\n  Failed checks:")
    for name, ok, detail in _results:
        if ok is False:
            print(f"    ✗ {name}" + (f"  ({detail})" if detail else ""))
    sys.exit(1)
else:
    print()
    sys.exit(0)
