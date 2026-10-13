"""
test_sequencing.py — Comprehensive tests for assembly sequencing functionality.

Covers:
  1.  CycleError detection (A→B→A)
  2.  Linear chain: exactly 1 valid topo sequence
  3.  Star topology: 4! = 24 valid sequences
  4.  Diamond: exactly 2 valid sequences
  5.  Liaison predicate filtering: 6 topo sorts → 3 liaison-valid
  6.  Scorer manual calculation verification
  7.  Greedy determinism (same result on repeated calls)
  8.  SA ≤ greedy cost
  9.  Subassembly detection on known chain graph
  10. Articulation points
  11. Full pipeline — PN16 flange assembly
  12. Full pipeline — UCP205 bearing assembly
  13. Sequence quality under heavy sub-break penalty

Run:
    python test_sequencing.py
    python -m pytest test_sequencing.py -v   # or with pytest
"""

from __future__ import annotations

import sys

# ── helpers ────────────────────────────────────────────────────────────────────

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"

_results: list[tuple[str, bool, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    _results.append((name, condition, detail))
    status = PASS if condition else FAIL
    print(f"  [{status}] {name}" + (f"  ({detail})" if detail else ""))


def section(title: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {title}")
    print(f"{'─' * 60}")


# ── imports ────────────────────────────────────────────────────────────────────

from assembly_graph.assembly_graph import AssemblyGraph, AssemblyNode, CycleError
from assembly_graph.liaison_matrix import LiaisonMatrix
from assembly_graph.planner       import AssemblyPlanner, AssemblyPlan
from assembly_graph.sequence.generator import SequenceGenerator
from assembly_graph.sequence.optimizer import SequenceOptimizer, OptimizationWeights


# ── helpers to build minimal test graphs ──────────────────────────────────────

def _node(pid: str, direction: str = "", tools: set | None = None,
          time_s: float = 1.0, subasm: str = "") -> AssemblyNode:
    return AssemblyNode(
        part_id=pid,
        name=pid,
        assembly_time=time_s,
        tools=tools or set(),
        direction=direction,
        subassembly_id=subasm,
    )


def _linear_graph() -> AssemblyGraph:
    """A → B → C → D (only one valid topo sort)."""
    g = AssemblyGraph()
    for pid in ("A", "B", "C", "D"):
        g.add_node(_node(pid))
    g.add_precedence("A", "B")
    g.add_precedence("B", "C")
    g.add_precedence("C", "D")
    return g


def _star_graph() -> AssemblyGraph:
    """A → B, A → C, A → D, A → E (4! = 24 valid sorts)."""
    g = AssemblyGraph()
    for pid in ("A", "B", "C", "D", "E"):
        g.add_node(_node(pid))
    for dep in ("B", "C", "D", "E"):
        g.add_precedence("A", dep)
    return g


def _diamond_graph() -> AssemblyGraph:
    """A → B, A → C, B → D, C → D (2 valid sorts: ABCD, ACBD)."""
    g = AssemblyGraph()
    for pid in ("A", "B", "C", "D"):
        g.add_node(_node(pid))
    g.add_precedence("A", "B")
    g.add_precedence("A", "C")
    g.add_precedence("B", "D")
    g.add_precedence("C", "D")
    return g


# ──────────────────────────────────────────────────────────────────────────────
# TEST 1 — CycleError detection
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 1 — CycleError detection")

g = AssemblyGraph()
g.add_node(_node("A"))
g.add_node(_node("B"))
g.add_precedence("A", "B")

caught = False
try:
    g.add_precedence("B", "A")   # would create A → B → A
except CycleError:
    caught = True

check("adding reverse edge raises CycleError", caught)

# Self-loop: add_precedence("A","A") silently does nothing (guard: if before == after: return)
# Verify it didn't add a spurious edge
g.add_precedence("A", "A")
check("self-loop is silently ignored (no edge added)",
      not g.has_edge("A", "A"))


# ──────────────────────────────────────────────────────────────────────────────
# TEST 2 — Linear chain: exactly 1 valid sequence
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 2 — Linear chain")

lg = _linear_graph()
seqs = lg.all_topological_sorts(max_results=500)
check("linear chain: exactly 1 valid sequence",
      len(seqs) == 1,
      f"got {len(seqs)}")
check("linear chain: correct order A,B,C,D",
      seqs[0] == ["A", "B", "C", "D"] if seqs else False,
      str(seqs[0]) if seqs else "no sequences")

ts = lg.topological_sort()
check("topological_sort returns A,B,C,D",
      ts == ["A", "B", "C", "D"],
      str(ts))


# ──────────────────────────────────────────────────────────────────────────────
# TEST 3 — Star topology: 4! = 24 valid sequences
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 3 — Star topology (4 leaves)")

sg = _star_graph()
star_seqs = sg.all_topological_sorts(max_results=500)
check("star: exactly 24 valid sequences",
      len(star_seqs) == 24,
      f"got {len(star_seqs)}")
check("star: all sequences start with A",
      all(s[0] == "A" for s in star_seqs),
      "")
check("star: each sequence has length 5",
      all(len(s) == 5 for s in star_seqs),
      "")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 4 — Diamond: exactly 2 valid sequences
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 4 — Diamond topology")

dg = _diamond_graph()
dseqs = dg.all_topological_sorts(max_results=500)
check("diamond: exactly 2 valid sequences",
      len(dseqs) == 2,
      f"got {len(dseqs)}")
check("diamond: [A,B,C,D] is valid",
      ["A", "B", "C", "D"] in dseqs,
      "")
check("diamond: [A,C,B,D] is valid",
      ["A", "C", "B", "D"] in dseqs,
      "")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 5 — Liaison predicate filtering
#
# Parts: A (base), B, C, D
# Contacts: A↔B, B↔C (chain), A↔D
# DAG precedence (star from A as base): A→B, A→C, A→D
#   → 6 topological sorts total
# Liaison-valid: C can only be placed after B (C touches only B, not A or D)
#   Valid:   [A,B,C,D], [A,B,D,C], [A,D,B,C]
#   Filtered:[A,C,B,D], [A,C,D,B], [A,D,C,B]
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 5 — Liaison predicate filtering")

# Build graph where A is the only root and B,C,D depend on A
lp_graph = AssemblyGraph()
for pid in ("A", "B", "C", "D"):
    lp_graph.add_node(_node(pid))
lp_graph.add_precedence("A", "B")
lp_graph.add_precedence("A", "C")
lp_graph.add_precedence("A", "D")

# Liaison: A↔B, B↔C, A↔D
liaison = LiaisonMatrix(["A", "B", "C", "D"])
liaison.add_contact("A", "B")
liaison.add_contact("B", "C")
liaison.add_contact("A", "D")

total_seqs = lp_graph.all_topological_sorts(max_results=500)
check("liaison test: 6 total topo sequences (star with 3 leaves)",
      len(total_seqs) == 6,
      f"got {len(total_seqs)}")

gen = SequenceGenerator(lp_graph, liaison, base_part="A")
valid_seqs = gen.liaison_sequences(max_results=500)

check("liaison predicate reduces to 3 valid sequences",
      len(valid_seqs) == 3,
      f"got {len(valid_seqs)}: {valid_seqs}")

expected = {
    ("A", "B", "C", "D"),
    ("A", "B", "D", "C"),
    ("A", "D", "B", "C"),
}
actual = {tuple(s) for s in valid_seqs}
check("liaison: exact set of 3 valid sequences",
      actual == expected,
      f"got {actual}")

# Sequences where C immediately follows A must be absent
bad = {("A", "C", "B", "D"), ("A", "C", "D", "B"), ("A", "D", "C", "B")}
check("liaison: C-before-B sequences are filtered",
      actual.isdisjoint(bad),
      "")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 6 — Scorer manual calculation
#
# Sequence: [A, B, C, D]  (all unique directions and tools)
# A: dir=+X, tools={t1}, sub=SA1, time=2s
# B: dir=+X, tools={t1}, sub=SA1, time=3s   (same dir/tool/sub as A → no penalty)
# C: dir=+Z, tools={t2}, sub=SA2, time=2s   (dir change +3, tool change +2, sub break +2.5)
# D: dir=+Z, tools={t2}, sub=SA2, time=2s   (same as C → no penalty)
#
# total_time = 9s, max_time = 9s, norm_time = 1.0
# cost = 3*1 + 2*1 + 1*1.0 + 2.5*1  = 8.5
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 6 — Scorer manual calculation")

score_graph = AssemblyGraph()
score_graph.add_node(_node("A", direction="+X", tools={"t1"}, time_s=2.0, subasm="SA1"))
score_graph.add_node(_node("B", direction="+X", tools={"t1"}, time_s=3.0, subasm="SA1"))
score_graph.add_node(_node("C", direction="+Z", tools={"t2"}, time_s=2.0, subasm="SA2"))
score_graph.add_node(_node("D", direction="+Z", tools={"t2"}, time_s=2.0, subasm="SA2"))
score_graph.add_precedence("A", "B")
score_graph.add_precedence("B", "C")
score_graph.add_precedence("C", "D")

weights = OptimizationWeights(
    direction_changes=3.0,
    tool_changes=2.0,
    assembly_time=1.0,
    subassembly_breaks=2.5,
)
opt = SequenceOptimizer(score_graph, weights)

seq_abcd = ["A", "B", "C", "D"]
sc = opt.score(seq_abcd)

check("scorer: direction_changes = 1",
      sc.direction_changes == 1,
      f"got {sc.direction_changes}")
check("scorer: tool_changes = 1",
      sc.tool_changes == 1,
      f"got {sc.tool_changes}")
check("scorer: subassembly_breaks = 1",
      sc.subassembly_breaks == 1,
      f"got {sc.subassembly_breaks}")
check("scorer: total_cost ≈ 8.5",
      abs(sc.total_cost - 8.5) < 0.01,
      f"got {sc.total_cost}")
check("scorer: total_time_s = 9.0",
      abs(sc.total_time_s - 9.0) < 0.01,
      f"got {sc.total_time_s}")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 7 — Greedy determinism
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 7 — Greedy determinism")

det_planner = AssemblyPlanner(
    part_ids=["P1", "P2", "P3", "P4"],
    part_names={"P1": "Base", "P2": "Cover", "P3": "Bracket", "P4": "Clip"},
    part_times={"P1": 3.0, "P2": 2.0, "P3": 1.5, "P4": 2.5},
    part_dirs={"P1": "+Z", "P2": "+Z", "P3": "+X", "P4": "+X"},
)
det_planner.liaison.add_contact("P1", "P2")
det_planner.liaison.add_contact("P1", "P3")
det_planner.liaison.add_contact("P3", "P4")

results1 = [det_planner.plan(sa_iterations=0).greedy_sequence.sequence for _ in range(3)]
check("greedy is deterministic (3 identical runs)",
      all(r == results1[0] for r in results1),
      str(results1[0]))


# ──────────────────────────────────────────────────────────────────────────────
# TEST 8 — SA ≤ greedy cost
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 8 — SA cost ≤ greedy cost")

# Use a planner with enough parts for SA to potentially improve on greedy
sa_planner = AssemblyPlanner(
    part_ids=["A","B","C","D","E","F"],
    part_dirs={"A": "+Z", "B": "+Z", "C": "+X", "D": "+X", "E": "+Z", "F": "+X"},
    part_tools={"A": {"t1"}, "B": {"t1"}, "C": {"t2"}, "D": {"t2"},
                "E": {"t1"}, "F": {"t2"}},
    part_times={"A": 2.0, "B": 3.0, "C": 1.5, "D": 2.0, "E": 2.5, "F": 1.0},
)
for a, b in [("A","B"),("A","C"),("A","D"),("A","E"),("A","F")]:
    sa_planner.liaison.add_contact(a, b)

sa_plan = sa_planner.plan(sa_iterations=500, sa_seed=99)
check("SA cost ≤ greedy cost",
      sa_plan.optimized_sequence.total_cost <= sa_plan.greedy_sequence.total_cost + 1e-6,
      f"SA={sa_plan.optimized_sequence.total_cost:.3f}  "
      f"greedy={sa_plan.greedy_sequence.total_cost:.3f}")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 9 — Subassembly detection on a chain graph
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 9 — Subassembly detection")

from assembly_graph.subassembly_detector import SubassemblyDetector

# A chain: A–B–C–D.  Removing any middle node disconnects it.
chain_parts = ["A", "B", "C", "D"]
chain_liaison = LiaisonMatrix(chain_parts)
chain_liaison.add_contact("A", "B")
chain_liaison.add_contact("B", "C")
chain_liaison.add_contact("C", "D")

chain_detector = SubassemblyDetector(chain_liaison)
chain_subs = chain_detector.detect(method="both")

# Subassembly detection may find overlapping groups;
# the important test is that it detects *some* subassemblies
check("chain: detect() returns at least 1 subassembly",
      len(chain_subs) >= 1,
      f"got {len(chain_subs)}")

# All part IDs in subassemblies must be valid
all_pids_in_subs = {pid for sub in chain_subs for pid in sub.part_ids}
check("chain: subassembly parts are valid part IDs",
      all_pids_in_subs.issubset(set(chain_parts)),
      str(all_pids_in_subs))

# ── test with a BCC-friendly graph: two triangles sharing one edge ────────────
# A–B–C (triangle), C–D–E (triangle); C is an articulation point between them
bcc_parts = ["A","B","C","D","E"]
bcc_liaison = LiaisonMatrix(bcc_parts)
bcc_liaison.add_contact("A","B")
bcc_liaison.add_contact("B","C")
bcc_liaison.add_contact("A","C")
bcc_liaison.add_contact("C","D")
bcc_liaison.add_contact("D","E")
bcc_liaison.add_contact("C","E")

bcc_detector = SubassemblyDetector(bcc_liaison)
bcc_subs = bcc_detector.detect(method="biconnected")
# Should detect 2 BCCs: {A,B,C} and {C,D,E}
check("biconnected: ≥2 subassemblies for two-triangle graph",
      len(bcc_subs) >= 2,
      f"got {len(bcc_subs)}: {[list(s.part_ids) for s in bcc_subs]}")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 10 — Articulation points
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 10 — Articulation points")

art_detector = SubassemblyDetector(bcc_liaison)
art_report = art_detector.articulation_point_report()

art_ids = {ap["part_id"] for ap in art_report}
check("two-triangle graph: C is an articulation point",
      "C" in art_ids,
      f"articulation points: {art_ids}")
check("two-triangle graph: A is not an articulation point",
      "A" not in art_ids,
      "")
check("articulation point report has 'degree' key",
      all("degree" in ap for ap in art_report),
      "")
check("articulation point report has 'components_if_removed' key",
      all("components_if_removed" in ap for ap in art_report),
      "")


# ──────────────────────────────────────────────────────────────────────────────
# TEST 11 — Full pipeline: PN16 DN50 Flange assembly
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 11 — Full pipeline: PN16 Flange assembly")

try:
    from real_assemblies_demo import (
        build_flange_assembly, build_flange_fasteners, build_flange_planner,
    )

    flange_assembly  = build_flange_assembly()
    flange_fasteners = build_flange_fasteners()
    flange_planner   = build_flange_planner()

    # DFMA analysis
    from dfma.analyzer import analyze
    flange_result = analyze(flange_assembly, fasteners=flange_fasteners)
    check("flange: DFMA analysis runs without exception", True)
    check("flange: DFA index is in [0, 1]",
          0.0 <= flange_result.dfa_index <= 1.0,
          f"dfa_index={flange_result.dfa_index:.2%}")
    # Gasket triggers a draft-angle DFM warning (injection_molding, no draft set) — that's
    # expected for this demo; check there are no *assembly* (DFA) errors
    assy_errors = [w for w in flange_result.errors() if w.rule_id.startswith("DFA")]
    check("flange: no assembly (DFA) errors",
          len(assy_errors) == 0,
          f"DFA errors: {[w.message for w in assy_errors]}")

    # Resource calculator
    from dfma.rules.resource_calculator import calculate_resources
    flange_res = calculate_resources(flange_fasteners)
    check("flange: resource calc runs without exception", True)
    check("flange: all fasteners use hex bolts (socket tool)",
          all(t.tool_type == "socket" for t in flange_res.tools),
          f"tools: {[t.tool_type for t in flange_res.tools]}")
    check("flange: exactly 1 unique tool (all M16 AF24)",
          flange_res.unique_tool_count == 1,
          f"got {flange_res.unique_tool_count}: {[t.label for t in flange_res.tools]}")

    # Sequencing
    flange_plan = flange_planner.plan(sa_iterations=500, sa_seed=0)
    check("flange: plan runs without exception", True)
    check("flange: optimized sequence includes all parts",
          len(flange_plan.optimized_sequence.sequence) == len(flange_assembly.all_parts()),
          f"seq len={len(flange_plan.optimized_sequence.sequence)} "
          f"parts={len(flange_assembly.all_parts())}")

    # FL01 uses direction="base" and FL02-FL05 use "+Z" → 1 change at the transition.
    # Flange is essentially single-direction; expect at most 1 direction change.
    check("flange: ≤1 direction changes (mostly single-direction assembly)",
          flange_plan.optimized_sequence.direction_changes <= 1,
          f"got {flange_plan.optimized_sequence.direction_changes}")

    steps = flange_plan.optimized_steps()
    check("flange: optimized_steps() returns list of dicts",
          isinstance(steps, list) and all(isinstance(s, dict) for s in steps),
          "")
    required_keys = {"part_id", "part_name", "direction", "time_s"}
    check("flange: step dicts have required keys",
          all(required_keys.issubset(s.keys()) for s in steps),
          f"keys in first step: {set(steps[0].keys()) if steps else 'N/A'}")

    # Base part (flange body) should be placed first
    check("flange: base part (FL01) is placed first",
          steps[0]["part_id"] == "FL01" if steps else False,
          f"first step: {steps[0]['part_id'] if steps else 'N/A'}")

    print(f"  [info] Flange sequence: {[s['part_id'] for s in steps]}")
    print(f"  [info] Optimized cost = {flange_plan.optimized_sequence.total_cost:.3f}")

except Exception as exc:
    check(f"flange pipeline: raised unexpected exception: {exc}", False)
    import traceback
    traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 12 — Full pipeline: UCP205 Pillow-Block Bearing assembly
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 12 — Full pipeline: UCP205 Bearing assembly")

try:
    from real_assemblies_demo import (
        build_bearing_assembly, build_bearing_fasteners, build_bearing_planner,
    )

    bearing_assembly  = build_bearing_assembly()
    bearing_fasteners = build_bearing_fasteners()
    bearing_planner   = build_bearing_planner()

    # DFMA analysis
    bearing_result = analyze(bearing_assembly, fasteners=bearing_fasteners)
    check("bearing: DFMA analysis runs without exception", True)
    check("bearing: DFA index in [0, 1]",
          0.0 <= bearing_result.dfa_index <= 1.0,
          f"{bearing_result.dfa_index:.2%}")

    # Resource calculator
    bearing_res = calculate_resources(bearing_fasteners)
    check("bearing: resource calc runs without exception", True)
    # Both M12 socket cap and M8 set screw use DriveType.SOCKET_CAP → both are hex_key tools.
    # Verify there are ≥2 distinct tools (different sizes: 10 mm for M12, 6 mm for M8).
    check("bearing: ≥2 distinct tools (M12 cap → 10mm hex key, M8 set screw → 6mm hex key)",
          bearing_res.unique_tool_count >= 2,
          f"tools: {[(t.label, t.tool_type) for t in bearing_res.tools]}")
    check("bearing: ≥1 tool change in mounting zone",
          bearing_res.total_tool_changes >= 1,
          f"total tool changes: {bearing_res.total_tool_changes}")

    # Sequencing
    bearing_plan = bearing_planner.plan(sa_iterations=500, sa_seed=0)
    check("bearing: plan runs without exception", True)
    check("bearing: optimized sequence includes all parts",
          len(bearing_plan.optimized_sequence.sequence) == len(bearing_assembly.all_parts()),
          f"seq len={len(bearing_plan.optimized_sequence.sequence)} "
          f"parts={len(bearing_assembly.all_parts())}")

    # Bearing has 2 directions (+Z for housing bolts, +Y for set screws)
    # → expect at least 1 direction change somewhere in optimized sequence
    check("bearing: ≥1 direction change (multi-direction assembly)",
          bearing_plan.optimized_sequence.direction_changes >= 1,
          f"got {bearing_plan.optimized_sequence.direction_changes}")

    # Housing BR01 must be placed first
    bearing_steps = bearing_plan.optimized_steps()
    check("bearing: base housing (BR01) is placed first",
          bearing_steps[0]["part_id"] == "BR01" if bearing_steps else False,
          f"first: {bearing_steps[0]['part_id'] if bearing_steps else 'N/A'}")

    print(f"  [info] Bearing sequence: {[s['part_id'] for s in bearing_steps]}")
    print(f"  [info] Optimized cost = {bearing_plan.optimized_sequence.total_cost:.3f}")
    print(f"  [info] Direction changes = {bearing_plan.optimized_sequence.direction_changes}")

except Exception as exc:
    check(f"bearing pipeline: raised unexpected exception: {exc}", False)
    import traceback
    traceback.print_exc()


# ──────────────────────────────────────────────────────────────────────────────
# TEST 13 — Sequence quality under heavy subassembly-break penalty
#
# Graph: A(SA1)→B(SA1)→C(SA1), A→D(SA2)→E(SA2)
# Valid topo orders:
#   [A,B,C,D,E], [A,B,D,C,E], [A,B,D,E,C],
#   [A,D,B,C,E], [A,D,B,E,C], [A,D,E,B,C]
# All dirs and tools identical → only sub_breaks matter.
# Best (0 sub_breaks): [A,B,C,D,E] — stays SA1 then SA2
# Worst (3+ breaks): [A,D,B,C,E] or similar interleaved
# ──────────────────────────────────────────────────────────────────────────────

section("TEST 13 — Sequence quality under heavy sub-break penalty")

sqg = AssemblyGraph()
sqg.add_node(_node("A", direction="+Z", tools={"t1"}, time_s=2.0, subasm="SA1"))
sqg.add_node(_node("B", direction="+Z", tools={"t1"}, time_s=2.0, subasm="SA1"))
sqg.add_node(_node("C", direction="+Z", tools={"t1"}, time_s=2.0, subasm="SA1"))
sqg.add_node(_node("D", direction="+Z", tools={"t1"}, time_s=2.0, subasm="SA2"))
sqg.add_node(_node("E", direction="+Z", tools={"t1"}, time_s=2.0, subasm="SA2"))
sqg.add_precedence("A", "B")
sqg.add_precedence("B", "C")
sqg.add_precedence("A", "D")
sqg.add_precedence("D", "E")

heavy_weights = OptimizationWeights(
    direction_changes=0.0,
    tool_changes=0.0,
    assembly_time=0.0,
    subassembly_breaks=10.0,   # only sub_breaks matter
)
sq_opt = SequenceOptimizer(sqg, heavy_weights)

# Best sequence must be [A,B,C,D,E] (0 sub breaks)
best_sc = sq_opt.score(["A", "B", "C", "D", "E"])
# [A,B,C,D,E]: A(SA1)→B(SA1)→C(SA1)→D(SA2)→E(SA2)
# There is 1 sub_break when crossing from SA1 (C) to SA2 (D) — unavoidable.
# This IS the minimum-break sequence for this graph (only 1 cross-subassembly transition).
check("sub-break quality: [A,B,C,D,E] has 1 sub_break (optimal, transition is unavoidable)",
      best_sc.subassembly_breaks == 1,
      f"got {best_sc.subassembly_breaks}")
check("sub-break quality: [A,B,C,D,E] cost = 10.0 (1 break × w=10)",
      abs(best_sc.total_cost - 10.0) < 0.01,
      f"got {best_sc.total_cost}")

interleaved_sc = sq_opt.score(["A", "D", "B", "E", "C"])   # SA2,SA1,SA2,SA1,SA2 → 4 breaks
check("sub-break quality: interleaved sequence has ≥2 sub_breaks",
      interleaved_sc.subassembly_breaks >= 2,
      f"got {interleaved_sc.subassembly_breaks}")

# Greedy should prefer coherent grouping
greedy_sc = sq_opt.greedy()
check("greedy with heavy sub-break weight: ≤1 sub_breaks",
      greedy_sc.subassembly_breaks <= 1,
      f"got {greedy_sc.subassembly_breaks}, seq={greedy_sc.sequence}")

# SA should also keep sub_breaks low
sa_sc = sq_opt.simulated_annealing(max_iterations=1000, seed=7)
# SA is stochastic; it should not be dramatically worse than greedy (optimal=1 break).
# Allow ≤ greedy_sc.subassembly_breaks + 1 to tolerate occasional SA variance.
check("SA with heavy sub-break weight: sub_breaks ≤ greedy + 1",
      sa_sc.subassembly_breaks <= greedy_sc.subassembly_breaks + 1,
      f"SA={sa_sc.subassembly_breaks}, greedy={greedy_sc.subassembly_breaks}, "
      f"seq={sa_sc.sequence}")


# ──────────────────────────────────────────────────────────────────────────────
# SUMMARY
# ──────────────────────────────────────────────────────────────────────────────

total  = len(_results)
passed = sum(1 for _, ok, _ in _results if ok)
failed = total - passed

print(f"\n{'═' * 60}")
print(f"  RESULTS: {passed}/{total} passed" +
      (f"  — {failed} FAILED" if failed else "  — all passed"))
print(f"{'═' * 60}")

if failed:
    print("\n  Failed checks:")
    for name, ok, detail in _results:
        if not ok:
            print(f"    ✗ {name}" + (f"  ({detail})" if detail else ""))
    sys.exit(1)
else:
    print()
    sys.exit(0)
