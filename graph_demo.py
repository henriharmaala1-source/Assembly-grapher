"""
Assembly Graph Demo — subassembly detection, sequence generation, optimization.

Example: Pneumatic Valve Assembly (same 7 parts as dfma_demo.py)

Liaison structure (physical contacts):
  Valve Body ←→ Valve Spool     (sliding bore contact)
  Valve Body ←→ End Cap         (face mating)
  Valve Body ←→ O-Ring (x2)    (groove contact)
  Valve Body ←→ Mounting Screw  (threaded)
  Valve Body ←→ Retaining Clip  (snap seat)
  Valve Spool ←→ Return Spring  (end contact)
  End Cap     ←→ Mounting Screw (clearance hole)

Run:
    python graph_demo.py
    python graph_demo.py --verbose
    python graph_demo.py --method biconnected
    python graph_demo.py --method cut_set
"""

import argparse

from assembly_graph import AssemblyPlanner, OptimizationWeights
from dfma_demo import build_pneumatic_valve


# ── colour codes for terminal output ────────────────────────────────────────
_C = {
    "reset":  "\033[0m",
    "bold":   "\033[1m",
    "green":  "\033[92m",
    "yellow": "\033[93m",
    "cyan":   "\033[96m",
    "blue":   "\033[94m",
    "red":    "\033[91m",
    "grey":   "\033[90m",
}


def _col(text: str, colour: str) -> str:
    return f"{_C.get(colour, '')}{text}{_C['reset']}"


def build_planner() -> AssemblyPlanner:
    """Build the planner for the pneumatic valve assembly."""
    assembly = build_pneumatic_valve()

    # Create planner from the existing Assembly object
    # (populates part names + B&D assembly times automatically)
    planner = AssemblyPlanner.from_assembly(
        assembly,
        base_part_id="P001",          # Valve Body is assembled first
        weights=OptimizationWeights(
            direction_changes=3.0,
            tool_changes=2.0,
            assembly_time=1.0,
            subassembly_breaks=2.5,
        ),
    )

    # ── define assembly directions ───────────────────────────────────────────
    # P001 Valve Body    → base (no direction)
    # P002 Valve Spool   → inserted along +X axis (horizontal bore)
    # P003 Return Spring → inserted along +X (same bore axis as spool)
    # P004 End Cap       → bolted from +X end face
    # P005 O-Ring        → slid over spool, inserted along +X
    # P006 Mounting Screw→ tightened from +Z (top face)
    # P007 Retaining Clip→ snapped from +Z (top face)
    planner.part_dirs = {
        "P001": "",           # base part — no insertion direction
        "P002": "+X",
        "P003": "+X",
        "P004": "+X",
        "P005": "+X",
        "P006": "+Z",
        "P007": "+Z",
    }

    # ── define tooling ───────────────────────────────────────────────────────
    planner.part_tools = {
        "P001": {"fixture"},
        "P002": {"hands"},
        "P003": {"hands"},
        "P004": {"torque_wrench"},
        "P005": {"hands"},
        "P006": {"hex_key_M4"},
        "P007": {"snap_tool"},
    }

    # ── define liaison (physical contacts) ───────────────────────────────────
    L = planner.liaison
    L.add_contact("P001", "P002", contact_type="face",   strength="sliding",    direction="+X")
    L.add_contact("P001", "P004", contact_type="face",   strength="rigid",      direction="+X")
    L.add_contact("P001", "P005", contact_type="face",   strength="flexible",   direction="+X", is_structural=False)
    L.add_contact("P001", "P006", contact_type="thread", strength="rigid",      direction="+Z")
    L.add_contact("P001", "P007", contact_type="edge",   strength="flexible",   direction="+Z", is_structural=False)
    L.add_contact("P002", "P003", contact_type="edge",   strength="flexible",   direction="+X")
    L.add_contact("P004", "P006", contact_type="face",   strength="rigid",      direction="+Z")

    return planner


def print_plan(plan, verbose: bool = False) -> None:
    from assembly_graph import SubassemblyDetector

    sep   = "─" * 68
    thick = "═" * 68

    print(f"\n{thick}")
    print(_col("  ASSEMBLY PLANNING REPORT", "bold"))
    print(thick)

    # ── liaison matrix ───────────────────────────────────────────────────────
    print(f"\n{_col('LIAISON MATRIX', 'cyan')}")
    print(plan.liaison)

    # ── subassemblies ────────────────────────────────────────────────────────
    print(f"\n{_col('DETECTED SUBASSEMBLIES', 'cyan')}  ({len(plan.subassemblies)} found)")
    if plan.subassemblies:
        for sub in plan.subassemblies:
            score_bar = "█" * int(sub.overall_score * 10)
            parts_str = ", ".join(sorted(sub.part_ids))
            print(f"  {_col(sub.id, 'yellow')}  [{parts_str}]")
            print(f"    score={sub.overall_score:.2f} {score_bar}  "
                  f"stability={sub.stability_score:.2f}  "
                  f"independence={sub.independence:.2f}  "
                  f"method={sub.method}  level={sub.level}")
            if sub.base_part_id:
                print(f"    suggested base part: {_col(sub.base_part_id, 'green')}")
    else:
        print("  No subassemblies detected (assembly may be fully sequential).")

    # ── articulation points ───────────────────────────────────────────────────
    if plan.articulation_points:
        print(f"\n{_col('CRITICAL JUNCTION PARTS (articulation points)', 'cyan')}")
        for ap in plan.articulation_points:
            flag = _col("CRITICAL", "red") if ap["is_critical"] else "ok"
            print(f"  {ap['part_id']}  degree={ap['degree']}  "
                  f"splits_into={ap['components_if_removed']} groups  [{flag}]")

    # ── valid sequences ───────────────────────────────────────────────────────
    n_seqs = len(plan.all_sequences)
    print(f"\n{_col('VALID ASSEMBLY SEQUENCES', 'cyan')}  ({n_seqs} found)")
    if verbose and plan.all_sequences:
        for i, seq in enumerate(plan.all_sequences[:10]):
            print(f"  [{i+1:>2}]  " + " → ".join(seq))
        if n_seqs > 10:
            print(f"  ... and {n_seqs - 10} more")

    # ── top sequences ─────────────────────────────────────────────────────────
    print(f"\n{_col('TOP-5 SCORED SEQUENCES', 'cyan')}")
    for i, sc in enumerate(plan.top_sequences):
        tag = _col("★ BEST", "green") if i == 0 else f"  #{i+1} "
        print(f"  {tag}  {sc.summary()}")
        if verbose:
            print("         " + " → ".join(sc.sequence))

    # ── greedy vs SA ──────────────────────────────────────────────────────────
    print(f"\n{_col('GREEDY SEQUENCE', 'cyan')}")
    g = plan.greedy_sequence
    print(f"  {g.summary()}")
    print("  " + " → ".join(g.sequence))

    print(f"\n{_col('OPTIMIZED SEQUENCE (Simulated Annealing)', 'cyan')}")
    o = plan.optimized_sequence
    print(f"  {o.summary()}")
    print("  " + " → ".join(o.sequence))

    # ── step-by-step plan ────────────────────────────────────────────────────
    print(f"\n{_col('STEP-BY-STEP ASSEMBLY PLAN', 'cyan')}")
    print(f"  {'#':>3}  {'Part':20}  {'Dir':6}  {'Tools':20}  {'Time(s)':>8}  {'Cumul(s)':>9}  Subasm")
    print(f"  {sep}")
    cumul = 0.0
    for i, step in enumerate(plan.optimized_steps(), start=1):
        cumul += step["time_s"]
        sub = step["subassembly_id"]
        sub_col = _col(f"[{sub}]", "yellow") if sub != "—" else _col("—", "grey")
        print(
            f"  {i:>3}  {step['part_name']:20}  {step['direction']:6}  "
            f"{step['tools']:20}  {step['time_s']:>8.2f}  {cumul:>9.2f}  {sub_col}"
        )

    # ── improvement summary ───────────────────────────────────────────────────
    if plan.top_sequences:
        best_cost   = plan.optimized_sequence.total_cost
        greedy_cost = plan.greedy_sequence.total_cost
        improvement = (greedy_cost - best_cost) / max(greedy_cost, 1e-9) * 100
        print(f"\n  SA improvement over greedy: {_col(f'{improvement:+.1f}%', 'green')}")

    print(f"\n{thick}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Assembly Graph Demo — subassembly detection + sequence optimization"
    )
    parser.add_argument("--verbose",  "-v", action="store_true",
                        help="Show all valid sequences")
    parser.add_argument("--method",   "-m", default="auto",
                        choices=["auto", "biconnected", "cut_set", "both"],
                        help="Subassembly detection method")
    parser.add_argument("--sa-iter",  type=int, default=2000,
                        help="Simulated annealing iterations (default 2000)")
    args = parser.parse_args()

    planner = build_planner()
    plan    = planner.plan(
        detection_method=args.method,
        max_sequences=200,
        sa_iterations=args.sa_iter,
    )
    print_plan(plan, verbose=args.verbose)


if __name__ == "__main__":
    main()
