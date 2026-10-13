"""
test_pipeline.py — headless (no GUI) integration tests for the full pipeline:
  STEP import → DFMA analysis → Assembly sequencing

These replicate exactly what the GUI does when the user clicks
"Run DFMA Analysis" and "Generate Sequence" after loading a STEP file.

Run:
    python test_pipeline.py
"""

from __future__ import annotations
import os, sys, tempfile

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
SKIP = "\033[33mSKIP\033[0m"
_results: list[tuple[str, bool | None, str]] = []

def check(name, cond, detail=""):
    _results.append((name, cond, detail))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

def skip(name, reason=""):
    _results.append((name, None, reason))
    print(f"  [SKIP] {name}" + (f"  — {reason}" if reason else ""))

def section(t):
    print(f"\n{'─'*64}\n  {t}\n{'─'*64}")


# ── availability ──────────────────────────────────────────────────────────────
try:
    import cadquery as cq
    _HAS_CQ = True
except ImportError:
    _HAS_CQ = False

try:
    from OCP.STEPControl import STEPControl_Reader  # noqa
    _HAS_OCP = True
except ImportError:
    _HAS_OCP = False

_CAN_RUN = _HAS_CQ and _HAS_OCP


def _make_bracket_step() -> str:
    """Create a 4-part bracket assembly STEP file, return path."""
    base      = cq.Workplane("XY").box(150, 100, 10)
    bracket_l = cq.Workplane("XY").box(20, 80, 60).translate(cq.Vector(-65, 0, 35))
    bracket_r = cq.Workplane("XY").box(20, 80, 60).translate(cq.Vector( 65, 0, 35))
    brace     = cq.Workplane("XY").box(110, 10, 5).translate(cq.Vector(0, 40, 65))

    asm = (cq.Assembly()
           .add(base,      name="Base_Plate")
           .add(bracket_l, name="Bracket_L")
           .add(bracket_r, name="Bracket_R")
           .add(brace,     name="Top_Brace"))

    fd, path = tempfile.mkstemp(suffix=".step")
    os.close(fd)
    cq.exporters.export(asm.toCompound(), path)
    return path


def _make_shaft_step() -> str:
    """Create a 5-part shaft/bearing assembly STEP file, return path."""
    shaft    = cq.Workplane("XY").cylinder(100, 10)
    housing  = (cq.Workplane("XY").cylinder(35, 30)
                .cut(cq.Workplane("XY").cylinder(35, 11)))
    bearing1 = (cq.Workplane("XY").cylinder(10, 15)
                .cut(cq.Workplane("XY").cylinder(10, 10.5))
                .translate(cq.Vector(0, 0, 14)))
    bearing2 = (cq.Workplane("XY").cylinder(10, 15)
                .cut(cq.Workplane("XY").cylinder(10, 10.5))
                .translate(cq.Vector(0, 0, -14)))
    nut      = cq.Workplane("XY").cylinder(8, 12).translate(cq.Vector(0, 0, 50))

    asm = (cq.Assembly()
           .add(shaft,    name="Output_Shaft")
           .add(housing,  name="Bearing_Housing")
           .add(bearing1, name="Bearing_Front")
           .add(bearing2, name="Bearing_Rear")
           .add(nut,      name="Lock_Nut"))

    fd, path = tempfile.mkstemp(suffix=".step")
    os.close(fd)
    cq.exporters.export(asm.toCompound(), path)
    return path


# ══════════════════════════════════════════════════════════════════════════════
# 1. STEP IMPORT
# ══════════════════════════════════════════════════════════════════════════════

section("1 — STEP import (bracket assembly, 4 parts)")

if not _CAN_RUN:
    for t in ["parts loaded", "liaison populated", "poses non-identity",
              "shapes stored", "source == step"]:
        skip(t, "cadquery/OCP not available")
else:
    try:
        from assembly_graph.importers.step_importer import import_step

        path   = _make_bracket_step()
        result = import_step(path, infer_contacts=True, contact_gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("parts loaded",
              len(parts) == 4, f"got {len(parts)}")
        check("liaison populated",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)} contacts")
        check("poses non-identity (bbox centres stored)",
              any(abs(p.position[0]) > 1 or abs(p.position[2]) > 1
                  for p in result.poses.values()),
              str({k: v.position for k, v in result.poses.items()}))
        check("shapes stored for BRep collision",
              len(result.shapes) == 4, f"got {len(result.shapes)}")
        check("source == 'step'", result.source == "step")

    except Exception as exc:
        check(f"STEP import exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# 2. DFMA ANALYSIS — same path as GUI _handle_run_dfma
# ══════════════════════════════════════════════════════════════════════════════

section("2 — DFMA analysis on STEP-imported bracket")

if not _CAN_RUN:
    for t in ["DFA index computed", "assembly time > 0",
              "part count matches", "warnings is list"]:
        skip(t, "cadquery/OCP not available")
else:
    try:
        path   = _make_bracket_step()
        result = import_step(path, infer_contacts=True, contact_gap_mm=0.5)
        os.unlink(path)

        # Replicate exactly what _handle_run_dfma does
        from dfma.analyzer import analyze
        dfma = analyze(result.assembly, fasteners=None)

        check("DFA index computed (0–100)",
              isinstance(dfma.dfa_index, float) and 0.0 <= dfma.dfa_index <= 100.0,
              f"dfa_index={dfma.dfa_index:.1f}%")
        check("assembly time > 0",
              dfma.total_assembly_time_s > 0,
              f"{dfma.total_assembly_time_s:.2f}s")
        check("part count matches import",
              dfma.total_parts == len(result.assembly.all_parts()),
              f"dfma={dfma.total_parts}  import={len(result.assembly.all_parts())}")
        check("warnings is list (no crash)",
              isinstance(dfma.warnings, list), "")
        # STEP-imported parts lack DFA-001 metadata, so minimum may be 0
        check("theoretical_minimum is an int >= 0",
              isinstance(dfma.theoretical_minimum, int) and dfma.theoretical_minimum >= 0,
              f"got {dfma.theoretical_minimum}")

    except Exception as exc:
        check(f"DFMA analysis exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# 3. ASSEMBLY SEQUENCING — same path as GUI _handle_gen_sequence (FIXED)
# ══════════════════════════════════════════════════════════════════════════════

section("3 — Sequencing on STEP-imported bracket (with liaison)")

if not _CAN_RUN:
    for t in ["planner built with liaison", "plan returned",
              "sequence length == part count", "cost >= 0",
              "subassemblies detected", "top sequences not empty"]:
        skip(t, "cadquery/OCP not available")
else:
    try:
        path   = _make_bracket_step()
        result = import_step(path, infer_contacts=True, contact_gap_mm=0.5)
        os.unlink(path)

        # Replicate exactly what _handle_gen_sequence now does (FIXED)
        from assembly_graph import AssemblyPlanner
        planner = AssemblyPlanner.from_assembly(
            result.assembly,
            base_part_id=result.assembly.all_parts()[0].id,
        )
        planner.liaison = result.liaison   # ← the fix

        plan = planner.plan(sa_iterations=500)

        check("planner built with liaison",
              len(planner.liaison._contacts) >= 1,
              f"{len(planner.liaison._contacts)} contacts")
        check("plan returned (no crash)", plan is not None)
        check("sequence length == part count",
              len(plan.optimized_sequence.sequence) == len(result.assembly.all_parts()),
              f"seq={len(plan.optimized_sequence.sequence)}"
              f" parts={len(result.assembly.all_parts())}")
        check("optimized cost >= 0",
              plan.optimized_sequence.total_cost >= 0,
              f"cost={plan.optimized_sequence.total_cost:.3f}")
        check("top_sequences not empty",
              len(plan.top_sequences) >= 1,
              f"got {len(plan.top_sequences)}")
        check("greedy sequence length == part count",
              len(plan.greedy_sequence.sequence) == len(result.assembly.all_parts()),
              f"got {len(plan.greedy_sequence.sequence)}")

    except Exception as exc:
        check(f"Sequencing exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# 4. SEQUENCING — shaft assembly (5 parts, more contacts)
# ══════════════════════════════════════════════════════════════════════════════

section("4 — Sequencing on STEP-imported shaft assembly (5 parts)")

if not _CAN_RUN:
    for t in ["5 parts imported", "plan covers all parts", "subassemblies"]:
        skip(t, "cadquery/OCP not available")
else:
    try:
        path   = _make_shaft_step()
        result = import_step(path, infer_contacts=True, contact_gap_mm=0.5)
        os.unlink(path)

        parts = result.assembly.all_parts()
        check("5 parts imported", len(parts) == 5, f"got {len(parts)}")
        check("liaison contacts inferred",
              len(result.liaison._contacts) >= 1,
              f"got {len(result.liaison._contacts)}")

        planner = AssemblyPlanner.from_assembly(
            result.assembly,
            base_part_id=parts[0].id,
        )
        planner.liaison = result.liaison

        plan = planner.plan(sa_iterations=500)
        check("plan covers all 5 parts",
              len(plan.optimized_sequence.sequence) == 5,
              f"seq={len(plan.optimized_sequence.sequence)}")
        check("direction_changes reported",
              isinstance(plan.optimized_sequence.direction_changes, int), "")
        check("tool_changes reported",
              isinstance(plan.optimized_sequence.tool_changes, int), "")

    except Exception as exc:
        check(f"Shaft sequencing exception: {exc}", False)
        import traceback; traceback.print_exc()


# ══════════════════════════════════════════════════════════════════════════════
# 5. FULL PIPELINE — import → DFMA → sequence in one flow
# ══════════════════════════════════════════════════════════════════════════════

section("5 — Full pipeline: import → DFMA → sequence")

if not _CAN_RUN:
    skip("Full pipeline", "cadquery/OCP not available")
else:
    try:
        path   = _make_bracket_step()
        result = import_step(path, infer_contacts=True, contact_gap_mm=0.5)
        os.unlink(path)

        # DFMA
        dfma = analyze(result.assembly)

        # Sequence
        planner = AssemblyPlanner.from_assembly(result.assembly)
        planner.liaison = result.liaison
        plan = planner.plan(sa_iterations=200)

        # Validate combined outputs
        check("pipeline: parts in sequence match DFMA part count",
              len(plan.optimized_sequence.sequence) == dfma.total_parts,
              f"seq={len(plan.optimized_sequence.sequence)} dfma={dfma.total_parts}")
        check("pipeline: plan.optimized_steps() returns list of dicts",
              isinstance(plan.optimized_steps(), list) and
              all(isinstance(s, dict) for s in plan.optimized_steps()),
              "")
        check("pipeline: each step has 'part_id' and 'part_name'",
              all("part_id" in s and "part_name" in s for s in plan.optimized_steps()),
              str(plan.optimized_steps()[:1]))
        check("pipeline: summary() string non-empty",
              len(plan.summary()) > 0, "")

    except Exception as exc:
        check(f"Full pipeline exception: {exc}", False)
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
print(f"{'═'*64}\n")

if failed:
    for name, ok, detail in _results:
        if ok is False:
            print(f"    ✗ {name}" + (f"  ({detail})" if detail else ""))
    sys.exit(1)
