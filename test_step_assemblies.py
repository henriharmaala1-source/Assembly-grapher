"""
test_step_assemblies.py — Integration tests simulating STEP exports from
SolidWorks and NX / Siemens NX assemblies.

Each test creates a realistic multi-body STEP file using cadquery (OCP backend),
imports it through the full pipeline, and validates geometry, contacts, and
downstream planning / DFMA outputs.

Requires:  pip install cadquery   (provides cadquery + OCP backend)

Assemblies tested
─────────────────
  SW-1  SolidWorks bracket assembly     — 3 parts: base plate + 2 angle brackets
  SW-2  SolidWorks bolted cover         — 6 parts: housing + cover + 4 M6 bolts
  SW-3  SolidWorks thin-wall sheet      — 2 parts: channel + stiffener rib
  NX-1  NX shaft / bearing assembly     — 4 parts: shaft + housing + 2 bearings
  NX-2  NX gearbox sub-assembly         — 7 parts: case + cover + gear + shaft + 3 fasteners
  NX-3  NX large flat panel assembly    — 9 parts: frame (4 rails) + 5 panels
  EDGE  Single-solid part               — 1 part: sanity baseline
  EDGE  Touching parts get contact      — 2 cubes face-to-face → liaison inferred
  EDGE  Non-touching parts no contact   — 2 cubes 50mm apart   → no liaison
  PLAN  Planner runs on imported result — sequence generated, no crash
  DFMA  DFMA analyzer runs on import    — DFA Index computed, no crash

Run:
    cd Assembly-grapher
    python test_step_assemblies.py
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

PASS  = "\033[32mPASS\033[0m"
FAIL  = "\033[31mFAIL\033[0m"
SKIP  = "\033[33mSKIP\033[0m"

_results: list[tuple[str, bool | None, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    _results.append((name, condition, detail))
    status = PASS if condition else FAIL
    print(f"  [{status}] {name}" + (f"  ({detail})" if detail else ""))


def skip(name: str, reason: str = "") -> None:
    _results.append((name, None, reason))
    print(f"  [{SKIP}] {name}" + (f"  — {reason}" if reason else ""))


def section(title: str) -> None:
    print(f"\n{'─'*64}\n  {title}\n{'─'*64}")


# ── availability checks ────────────────────────────────────────────────────────

_HAS_CQ = False
_HAS_OCP = False
try:
    import cadquery as cq
    _HAS_CQ = True
except ImportError:
    pass

try:
    from OCP.STEPControl import STEPControl_Reader  # noqa: F401
    _HAS_OCP = True
except ImportError:
    pass

_CAN_RUN = _HAS_CQ and _HAS_OCP


# ── helpers ────────────────────────────────────────────────────────────────────

def _export_step(asm_or_wp) -> str:
    """Export a cadquery Assembly or Workplane to a temp STEP file; return path."""
    fd, path = tempfile.mkstemp(suffix=".step")
    os.close(fd)
    if isinstance(asm_or_wp, cq.Assembly):
        cq.exporters.export(asm_or_wp.toCompound(), path)
    else:
        cq.exporters.export(asm_or_wp, path)
    return path


def _import(path: str, gap_mm: float = 0.5):
    """Import a STEP file and return ImportResult."""
    from assembly_graph.importers.step_importer import import_step
    return import_step(path, infer_contacts=True, contact_gap_mm=gap_mm)


def _bbox_close(actual: float, expected: float, tol: float = 3.0) -> bool:
    """True if actual is within ±tol of expected."""
    return abs(actual - expected) <= tol


# ══════════════════════════════════════════════════════════════════════════════
# SW-1 — SolidWorks Bracket Assembly (3 parts)
# Geometry: base plate 200×100×10 + two angle brackets 20×80×60
# Simulates: a typical SolidWorks weldment or fixture assembly
# ══════════════════════════════════════════════════════════════════════════════

section("SW-1 — SolidWorks bracket assembly (3 parts)")

if not _CAN_RUN:
    skip("SW-1 all tests", "cadquery/OCP not available")
else:
    try:
        base        = cq.Workplane("XY").box(200, 100, 10)
        bracket_l   = cq.Workplane("XY").box(20, 80, 60).translate(cq.Vector(-90, 0, 35))
        bracket_r   = cq.Workplane("XY").box(20, 80, 60).translate(cq.Vector( 90, 0, 35))

        asm = (cq.Assembly()
               .add(base,      name="Base_Plate-1")
               .add(bracket_l, name="Bracket_L-1")
               .add(bracket_r, name="Bracket_R-1"))

        path = _export_step(asm)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("SW-1: exactly 3 parts imported",
              len(parts) == 3, f"got {len(parts)}")

        # Sort by volume to find base plate (largest)
        sorted_parts = sorted(parts,
                              key=lambda p: p.geometry.length * p.geometry.width * p.geometry.height,
                              reverse=True)
        base_part = sorted_parts[0]
        brk_parts = sorted_parts[1:]

        check("SW-1: base plate length ≈ 200 mm",
              _bbox_close(base_part.geometry.length, 200), f"got {base_part.geometry.length:.1f}")
        check("SW-1: base plate width  ≈ 100 mm",
              _bbox_close(base_part.geometry.width,  100), f"got {base_part.geometry.width:.1f}")
        check("SW-1: base plate height ≈  10 mm",
              _bbox_close(base_part.geometry.height,  10), f"got {base_part.geometry.height:.1f}")
        check("SW-1: bracket height ≈ 60 mm",
              all(_bbox_close(b.geometry.height, 60) for b in brk_parts),
              f"got {[b.geometry.height for b in brk_parts]}")
        check("SW-1: mass estimates > 0 for all parts",
              all(p.geometry.mass_grams > 0 for p in parts), "")
        check("SW-1: liaison matrix has contacts (brackets touch base)",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")
        check("SW-1: source == 'step'", result.source == "step")
        check("SW-1: poses populated for all parts",
              len(result.poses) == len(parts), f"got {len(result.poses)}")

    except Exception as exc:
        check(f"SW-1 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# SW-2 — SolidWorks bolted cover assembly (6 parts)
# Geometry: rectangular housing box + flat cover + 4 cylindrical stud bolts
# Simulates: enclosure / electronics housing assembly
# ══════════════════════════════════════════════════════════════════════════════

section("SW-2 — SolidWorks bolted cover assembly (6 parts)")

if not _CAN_RUN:
    skip("SW-2 all tests", "cadquery/OCP not available")
else:
    try:
        # Housing box 150×100×60, open top (wall 5mm)
        housing = (cq.Workplane("XY")
                   .box(150, 100, 60)
                   .cut(cq.Workplane("XY").box(140, 90, 55).translate(cq.Vector(0, 0, 2.5))))

        # Flat cover lid 150×100×5, positioned on top
        cover = cq.Workplane("XY").box(150, 100, 5).translate(cq.Vector(0, 0, 32.5))

        # 4 M6 hex bolts (cylinders: d=6mm, h=25mm) at corners
        bolt_positions = [(-60, -40), (-60, 40), (60, -40), (60, 40)]
        bolts = [
            cq.Workplane("XY").cylinder(25, 3).translate(cq.Vector(bx, by, 20))
            for bx, by in bolt_positions
        ]

        asm = cq.Assembly()
        asm.add(housing, name="Housing-1")
        asm.add(cover,   name="Cover-1")
        for i, bolt in enumerate(bolts, 1):
            asm.add(bolt, name=f"M6_Bolt-{i}")

        path = _export_step(asm)
        result = _import(path, gap_mm=1.0)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("SW-2: exactly 6 parts imported",
              len(parts) == 6, f"got {len(parts)}")

        # Find housing by largest bbox
        sorted_p = sorted(parts,
                          key=lambda p: p.geometry.length * p.geometry.width,
                          reverse=True)
        largest = sorted_p[0]
        check("SW-2: housing length ≈ 150 mm",
              _bbox_close(largest.geometry.length, 150, tol=5.0),
              f"got {largest.geometry.length:.1f}")
        check("SW-2: housing width ≈ 100 mm",
              _bbox_close(largest.geometry.width, 100, tol=5.0),
              f"got {largest.geometry.width:.1f}")

        # Bolt geometry: height ≈ 25 mm, diameter ≈ 6 mm
        # Sort remaining by volume ascending → smallest are bolts
        small_parts = sorted(parts[1:],
                             key=lambda p: p.geometry.length * p.geometry.width * p.geometry.height)
        bolts_detected = [p for p in small_parts
                          if p.geometry.height < 30 and p.geometry.length < 10]
        check("SW-2: at least 4 small bolt-like parts detected",
              len(bolts_detected) >= 4,
              f"got {len(bolts_detected)}: {[(p.geometry.length, p.geometry.height) for p in bolts_detected]}")

        check("SW-2: liaison contacts inferred",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

    except Exception as exc:
        check(f"SW-2 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# SW-3 — SolidWorks sheet-metal assembly (2 thin parts)
# Geometry: C-channel 200×50×30 (2mm wall) + stiffener rib 40×30×2
# Simulates: laser-cut / press-brake sheet metal sub-assembly
# ══════════════════════════════════════════════════════════════════════════════

section("SW-3 — SolidWorks sheet-metal assembly (2 thin parts)")

if not _CAN_RUN:
    skip("SW-3 all tests", "cadquery/OCP not available")
else:
    try:
        # C-channel: outer box minus inner cut
        channel = (cq.Workplane("XY")
                   .box(200, 50, 30)
                   .cut(cq.Workplane("XY").box(196, 46, 28).translate(cq.Vector(0, 0, 1))))

        # Stiffener rib: 40×30×2 flat plate, standing inside channel
        rib = (cq.Workplane("XY")
               .box(40, 2, 30)
               .translate(cq.Vector(0, 0, 0)))

        asm = (cq.Assembly()
               .add(channel, name="Channel-1")
               .add(rib,     name="Rib-1"))

        path = _export_step(asm)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("SW-3: exactly 2 parts imported",
              len(parts) == 2, f"got {len(parts)}")

        # Channel should be longer
        longest = max(parts, key=lambda p: p.geometry.length)
        shortest = min(parts, key=lambda p: p.geometry.length)
        check("SW-3: channel length ≈ 200 mm",
              _bbox_close(longest.geometry.length, 200, tol=5.0),
              f"got {longest.geometry.length:.1f}")
        check("SW-3: rib length ≈ 40 mm",
              _bbox_close(shortest.geometry.length, 40, tol=5.0),
              f"got {shortest.geometry.length:.1f}")

        # Mass estimates > 0
        check("SW-3: mass estimates > 0",
              all(p.geometry.mass_grams > 0 for p in parts),
              f"{[p.geometry.mass_grams for p in parts]}")

    except Exception as exc:
        check(f"SW-3 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# NX-1 — NX shaft / bearing assembly (4 parts)
# Geometry: solid shaft + hollow housing + 2 ring bearings
# Simulates: typical Siemens NX bearing unit sub-assembly
# ══════════════════════════════════════════════════════════════════════════════

section("NX-1 — NX shaft / bearing assembly (4 parts)")

if not _CAN_RUN:
    skip("NX-1 all tests", "cadquery/OCP not available")
else:
    try:
        shaft   = cq.Workplane("XY").cylinder(120, 10)
        housing = (cq.Workplane("XY")
                   .cylinder(40, 35)
                   .cut(cq.Workplane("XY").cylinder(40, 11)))
        bearing1 = (cq.Workplane("XY")
                    .cylinder(12, 17)
                    .cut(cq.Workplane("XY").cylinder(12, 10.5))
                    .translate(cq.Vector(0, 0, 15)))
        bearing2 = (cq.Workplane("XY")
                    .cylinder(12, 17)
                    .cut(cq.Workplane("XY").cylinder(12, 10.5))
                    .translate(cq.Vector(0, 0, -15)))

        asm = (cq.Assembly()
               .add(shaft,    name="Shaft_001")
               .add(housing,  name="Housing_001")
               .add(bearing1, name="Bearing_F_001")
               .add(bearing2, name="Bearing_R_001"))

        path = _export_step(asm)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("NX-1: exactly 4 parts imported",
              len(parts) == 4, f"got {len(parts)}")

        # Shaft: longest in height (120mm cylinder)
        tallest = max(parts, key=lambda p: p.geometry.height)
        check("NX-1: shaft height ≈ 120 mm",
              _bbox_close(tallest.geometry.height, 120, tol=5.0),
              f"got {tallest.geometry.height:.1f}")

        # All masses > 0
        check("NX-1: all parts have positive mass",
              all(p.geometry.mass_grams > 0 for p in parts), "")

        # Liaison: shaft touches bearings, bearings touch housing
        check("NX-1: liaison contacts inferred",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

        # Result shape storage for collision checking
        check("NX-1: shape data stored for all parts",
              len(result.shapes) == 4, f"got {len(result.shapes)}")

    except Exception as exc:
        check(f"NX-1 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# NX-2 — NX gearbox sub-assembly (7 parts)
# Geometry: case + cover + spur gear + output shaft + 3 hex fasteners
# Simulates: NX mechatronics sub-assembly with rotating parts
# ══════════════════════════════════════════════════════════════════════════════

section("NX-2 — NX gearbox sub-assembly (7 parts)")

if not _CAN_RUN:
    skip("NX-2 all tests", "cadquery/OCP not available")
else:
    try:
        # Gearbox case: 120×80×50, open face
        case = (cq.Workplane("XY")
                .box(120, 80, 50)
                .cut(cq.Workplane("XY").box(110, 70, 44).translate(cq.Vector(0, 0, 3))))

        # Flat cover 120×80×6
        cover = cq.Workplane("XY").box(120, 80, 6).translate(cq.Vector(0, 0, 28))

        # Spur gear (approximate as flat cylinder): d=60mm, h=15mm
        gear = (cq.Workplane("XY")
                .cylinder(15, 30)
                .translate(cq.Vector(0, 0, 0)))

        # Output shaft: d=20mm, h=100mm
        shaft = cq.Workplane("XY").cylinder(100, 10).translate(cq.Vector(0, 0, 0))

        # 3 M8 hex bolts: cylinder d=8mm, h=20mm
        bolt_pos = [(-45, -30), (45, -30), (0, 35)]
        fasteners = [
            cq.Workplane("XY").cylinder(20, 4).translate(cq.Vector(bx, by, 25))
            for bx, by in bolt_pos
        ]

        asm = cq.Assembly()
        asm.add(case,  name="Gearbox_Case")
        asm.add(cover, name="Gearbox_Cover")
        asm.add(gear,  name="Spur_Gear_Z48")
        asm.add(shaft, name="Output_Shaft")
        for i, f in enumerate(fasteners, 1):
            asm.add(f, name=f"M8_Hex_Bolt_{i:03d}")

        path = _export_step(asm)
        result = _import(path, gap_mm=1.0)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("NX-2: exactly 7 parts imported",
              len(parts) == 7, f"got {len(parts)}")

        # Case should be among the largest
        sorted_by_vol = sorted(
            parts,
            key=lambda p: p.geometry.length * p.geometry.width * p.geometry.height,
            reverse=True
        )
        check("NX-2: largest part (case) length ≈ 120 mm",
              _bbox_close(sorted_by_vol[0].geometry.length, 120, tol=5.0),
              f"got {sorted_by_vol[0].geometry.length:.1f}")

        # Fasteners: smallest parts, height ≈ 20mm
        smallest_3 = sorted_by_vol[-3:]
        check("NX-2: 3 fastener-like parts detected (h ≈ 20 mm)",
              all(_bbox_close(p.geometry.height, 20, tol=5.0) for p in smallest_3),
              f"heights: {[p.geometry.height for p in smallest_3]}")

        check("NX-2: liaison matrix populated",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

    except Exception as exc:
        check(f"NX-2 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# NX-3 — NX large flat panel assembly (9 parts)
# Geometry: 4 aluminium extrusion rails + 5 thin panels
# Simulates: NX frame-and-panel sheet structure (aerospace, HVAC)
# ══════════════════════════════════════════════════════════════════════════════

section("NX-3 — NX large flat panel assembly (9 parts)")

if not _CAN_RUN:
    skip("NX-3 all tests", "cadquery/OCP not available")
else:
    try:
        # 4 frame rails (40×40 cross section, 600mm long) at perimeter
        rail_positions = [
            (cq.Vector(0,  -280, 0), True),   # front rail  (lengthwise)
            (cq.Vector(0,   280, 0), True),   # back  rail
            (cq.Vector(-280, 0,  0), False),  # left  rail  (widthwise)
            (cq.Vector( 280, 0,  0), False),  # right rail
        ]
        rails = []
        for pos, lengthwise in rail_positions:
            if lengthwise:
                r = cq.Workplane("XY").box(600, 40, 40).translate(pos)
            else:
                r = cq.Workplane("XY").box(40, 520, 40).translate(pos)
            rails.append(r)

        # 5 infill panels (110×500×3 mm) arranged side by side
        panels = [
            cq.Workplane("XY").box(110, 500, 3).translate(cq.Vector(-220 + i*110, 0, -18.5))
            for i in range(5)
        ]

        asm = cq.Assembly()
        for i, r in enumerate(rails, 1):
            asm.add(r, name=f"Rail_{i:02d}")
        for i, p in enumerate(panels, 1):
            asm.add(p, name=f"Panel_{i:02d}")

        path = _export_step(asm)
        result = _import(path, gap_mm=1.0)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("NX-3: exactly 9 parts imported",
              len(parts) == 9, f"got {len(parts)}")

        # Panels: thickness ≈ 3mm
        thin_parts = [p for p in parts if p.geometry.height <= 5.0]
        check("NX-3: 5 thin panels detected (h ≤ 5 mm)",
              len(thin_parts) == 5,
              f"got {len(thin_parts)}: {[p.geometry.height for p in thin_parts]}")

        # Rails: width/length ≈ 40mm in at least one dimension
        thick_parts = [p for p in parts if p.geometry.height > 5.0]
        check("NX-3: 4 structural rails detected",
              len(thick_parts) == 4,
              f"got {len(thick_parts)}")

        check("NX-3: liaison matrix populated",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

    except Exception as exc:
        check(f"NX-3 unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# EDGE — Single-solid STEP (baseline sanity)
# ══════════════════════════════════════════════════════════════════════════════

section("EDGE — Single-solid STEP file")

if not _CAN_RUN:
    skip("EDGE single-solid", "cadquery/OCP not available")
else:
    try:
        wp   = cq.Workplane("XY").box(80, 60, 40)
        path = _export_step(wp)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("EDGE: 1 part imported",
              len(parts) == 1, f"got {len(parts)}")
        check("EDGE: dimensions ≈ 80×60×40",
              _bbox_close(parts[0].geometry.length, 80) and
              _bbox_close(parts[0].geometry.width,  60) and
              _bbox_close(parts[0].geometry.height, 40),
              f"{parts[0].geometry.length:.1f}×{parts[0].geometry.width:.1f}×{parts[0].geometry.height:.1f}")
        check("EDGE: mass estimate > 0", parts[0].geometry.mass_grams > 0, "")
        check("EDGE: liaison is connected (trivially — 1 part)",
              result.liaison.is_connected(), "")

    except Exception as exc:
        check(f"EDGE single-solid exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# EDGE — Touching cubes: liaison inferred
# ══════════════════════════════════════════════════════════════════════════════

section("EDGE — Touching parts get liaison contact")

if not _CAN_RUN:
    skip("EDGE touching parts", "cadquery/OCP not available")
else:
    try:
        # Two 50mm cubes face-to-face (second cube starts exactly where first ends)
        cube_a = cq.Workplane("XY").box(50, 50, 50).translate(cq.Vector(-25, 0, 0))
        cube_b = cq.Workplane("XY").box(50, 50, 50).translate(cq.Vector( 25, 0, 0))

        asm  = cq.Assembly().add(cube_a, name="CubeA").add(cube_b, name="CubeB")
        path = _export_step(asm)

        # Use gap_mm=0.5: touching (0 gap) should be detected
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("EDGE touching: 2 parts imported", len(parts) == 2, "")
        check("EDGE touching: contact inferred (gap=0 ≤ 0.5mm threshold)",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

    except Exception as exc:
        check(f"EDGE touching exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# EDGE — Non-touching parts: no liaison
# ══════════════════════════════════════════════════════════════════════════════

section("EDGE — Non-touching parts have no liaison contact")

if not _CAN_RUN:
    skip("EDGE non-touching parts", "cadquery/OCP not available")
else:
    try:
        # Two cubes 100mm apart (gap >> threshold)
        cube_a = cq.Workplane("XY").box(50, 50, 50).translate(cq.Vector(-75, 0, 0))
        cube_b = cq.Workplane("XY").box(50, 50, 50).translate(cq.Vector( 75, 0, 0))

        asm  = cq.Assembly().add(cube_a, name="CubeA").add(cube_b, name="CubeB")
        path = _export_step(asm)

        # gap_mm=0.5: gap is 50mm >> 0.5mm threshold → no contact
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        check("EDGE non-touching: 2 parts imported",
              len(result.assembly.all_parts()) == 2, "")
        check("EDGE non-touching: no contacts inferred",
              len(result.liaison._contacts) == 0,
              f"got {len(result.liaison._contacts)}")

    except Exception as exc:
        check(f"EDGE non-touching exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# EDGE — Very thin part (< 2mm wall): min_wall_thickness computed
# ══════════════════════════════════════════════════════════════════════════════

section("EDGE — Very thin part (1mm wall)")

if not _CAN_RUN:
    skip("EDGE thin wall", "cadquery/OCP not available")
else:
    try:
        thin = cq.Workplane("XY").box(200, 150, 1)
        path = _export_step(thin)
        result = _import(path)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("EDGE thin: 1 part imported", len(parts) == 1, "")
        check("EDGE thin: height ≈ 1 mm",
              _bbox_close(parts[0].geometry.height, 1, tol=1.0),
              f"got {parts[0].geometry.height:.3f}")
        check("EDGE thin: min_wall_thickness ≤ 1.0 mm",
              parts[0].geometry.min_wall_thickness <= 1.0,
              f"got {parts[0].geometry.min_wall_thickness:.3f}")

    except Exception as exc:
        check(f"EDGE thin exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# PLAN — Assembly planner runs on imported STEP result (SW-1 geometry re-used)
# ══════════════════════════════════════════════════════════════════════════════

section("PLAN — AssemblyPlanner runs on imported STEP result")

if not _CAN_RUN:
    skip("PLAN tests", "cadquery/OCP not available")
else:
    try:
        base      = cq.Workplane("XY").box(150, 100, 10)
        bracket_l = cq.Workplane("XY").box(20, 80, 50).translate(cq.Vector(-65, 0, 30))
        bracket_r = cq.Workplane("XY").box(20, 80, 50).translate(cq.Vector( 65, 0, 30))
        brace     = cq.Workplane("XY").box(110, 10, 5).translate(cq.Vector(0, 40, 55))

        asm  = (cq.Assembly()
                .add(base,      name="Base")
                .add(bracket_l, name="BracketL")
                .add(bracket_r, name="BracketR")
                .add(brace,     name="TopBrace"))
        path = _export_step(asm)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        from assembly_graph import AssemblyPlanner

        # from_assembly() extracts part IDs and assembly times; then swap
        # in the liaison matrix inferred from the STEP bounding boxes.
        planner         = AssemblyPlanner.from_assembly(result.assembly)
        planner.liaison = result.liaison
        plan            = planner.plan()

        check("PLAN: plan returned (no crash)", plan is not None)
        check("PLAN: optimized sequence covers all parts",
              len(plan.optimized_sequence.sequence) == len(result.assembly.all_parts()),
              f"seq={len(plan.optimized_sequence.sequence)} parts={len(result.assembly.all_parts())}")
        check("PLAN: total cost >= 0",
              plan.optimized_sequence.total_cost >= 0,
              f"cost={plan.optimized_sequence.total_cost:.3f}")
        check("PLAN: top sequences list not empty",
              len(plan.top_sequences) >= 1, f"got {len(plan.top_sequences)}")
        check("PLAN: subassemblies detected or plan completes without error",
              plan is not None, "")

    except Exception as exc:
        check(f"PLAN unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# DFMA — DFMA analyzer runs on imported STEP result
# ══════════════════════════════════════════════════════════════════════════════

section("DFMA — DfmaAnalyzer runs on imported STEP result")

if not _CAN_RUN:
    skip("DFMA tests", "cadquery/OCP not available")
else:
    try:
        base    = cq.Workplane("XY").box(200, 100, 10)
        bracket = cq.Workplane("XY").box(20, 80, 60).translate(cq.Vector(0, 0, 35))

        asm  = cq.Assembly().add(base, name="Base").add(bracket, name="Bracket")
        path = _export_step(asm)
        result = _import(path)
        os.unlink(path)

        from dfma.analyzer import analyze

        dfma_result = analyze(result.assembly)

        check("DFMA: analysis returned result (no crash)", dfma_result is not None)
        check("DFMA: DFA index is a float in [0, 100]",
              isinstance(dfma_result.dfa_index, (int, float)) and
              0.0 <= dfma_result.dfa_index <= 100.0,
              f"got {dfma_result.dfa_index}")
        check("DFMA: total assembly time > 0",
              dfma_result.total_assembly_time_s > 0,
              f"got {dfma_result.total_assembly_time_s:.2f}s")
        check("DFMA: total parts count matches assembly",
              dfma_result.total_parts == len(result.assembly.all_parts()),
              f"got {dfma_result.total_parts}")
        check("DFMA: warnings list is a list",
              isinstance(dfma_result.warnings, list), "")

    except Exception as exc:
        check(f"DFMA unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# COLL — Collision detection on imported STEP (AABB method)
# ══════════════════════════════════════════════════════════════════════════════

section("COLL — Collision detection on imported STEP assembly")

if not _CAN_RUN:
    skip("COLL tests", "cadquery/OCP not available")
else:
    try:
        # Two cubes separated by 60 mm in X (gap >> AABB threshold).
        # After the STEP importer fix, poses carry the bbox centre, so
        # check_collisions correctly sees them as non-overlapping.
        part_a = cq.Workplane("XY").box(50, 50, 20).translate(cq.Vector(-55, 0, 0))
        part_b = cq.Workplane("XY").box(50, 50, 20).translate(cq.Vector( 55, 0, 0))
        # World extents: A → X[-80,-30]  B → X[30,80]  gap = 60 mm

        asm  = cq.Assembly().add(part_a, name="PartA").add(part_b, name="PartB")
        path = _export_step(asm)
        result = _import(path, gap_mm=0.5)
        os.unlink(path)

        # Verify poses were populated with bbox centres (not identity)
        poses_list = list(result.poses.values())
        check("COLL: STEP importer stores non-identity poses (bbox centres)",
              any(abs(p.position[0]) > 1.0 for p in poses_list),
              f"positions: {[p.position for p in poses_list]}")

        from assembly_graph.importers.collision import check_collisions

        report = check_collisions(result, method="aabb", near_miss_mm=2.0)

        check("COLL: collision report returned (no crash)", report is not None)
        check("COLL: non-overlapping parts → no hard collisions",
              len(report.collisions) == 0, f"got {len(report.collisions)}")
        check("COLL: summary string generated",
              isinstance(report.summary(), str), "")

    except Exception as exc:
        check(f"COLL unexpected exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# SUMMARY
# ══════════════════════════════════════════════════════════════════════════════

total   = len(_results)
passed  = sum(1 for _, ok, _ in _results if ok is True)
skipped = sum(1 for _, ok, _ in _results if ok is None)
failed  = total - passed - skipped

print(f"\n{'═'*64}")
print(f"  RESULTS: {passed}/{total - skipped} passed"
      + (f"  — {skipped} skipped" if skipped else "")
      + (f"  — {failed} FAILED"   if failed  else "  — all passed"))
print(f"{'═'*64}")

if failed:
    print("\n  Failed checks:")
    for name, ok, detail in _results:
        if ok is False:
            print(f"    ✗ {name}" + (f"  ({detail})" if detail else ""))
    sys.exit(1)
else:
    print()
    sys.exit(0)
