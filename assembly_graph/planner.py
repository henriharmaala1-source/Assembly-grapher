"""
AssemblyPlanner — high-level API tying all assembly graph components together.

Typical usage
─────────────
    from assembly_graph.planner import AssemblyPlanner
    from dfma.models.part import Assembly

    planner = AssemblyPlanner.from_assembly(assembly)
    planner.liaison.add_contact("P001", "P002")
    planner.liaison.add_contact("P002", "P003")
    ...

    plan = planner.plan()

    print(plan.summary())
    for step in plan.optimized_sequence.sequence:
        print(step)

The planner:
  1. Builds the AssemblyGraph from the liaison matrix
  2. Runs SubassemblyDetector → labels each part with its subassembly group
  3. Generates all feasible sequences (filtered by liaison predicate)
  4. Runs greedy + simulated annealing optimizers
  5. Returns an AssemblyPlan with all results
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .liaison_matrix        import LiaisonMatrix
from .assembly_graph        import AssemblyGraph, AssemblyNode
from .subassembly_detector  import SubassemblyDetector, DetectedSubassembly
from .sequence.generator    import SequenceGenerator
from .sequence.optimizer    import SequenceOptimizer, ScoredSequence, OptimizationWeights


# ── result container ──────────────────────────────────────────────────────────

@dataclass
class AssemblyPlan:
    """
    Complete result of an assembly planning run.

    Attributes
    ----------
    subassemblies       : detected subassembly groups (ranked by score)
    articulation_points : critical junction parts
    all_sequences       : all valid orderings found (liaison-filtered)
    greedy_sequence     : result of greedy optimizer
    optimized_sequence  : result of simulated annealing (best found)
    top_sequences       : top-5 scored sequences from enumeration
    graph               : the assembled precedence graph
    liaison             : the liaison matrix used
    """
    subassemblies:       list[DetectedSubassembly]
    articulation_points: list[dict]
    all_sequences:       list[list[str]]
    greedy_sequence:     ScoredSequence
    optimized_sequence:  ScoredSequence
    top_sequences:       list[ScoredSequence]
    graph:               AssemblyGraph
    liaison:             LiaisonMatrix

    def summary(self) -> str:
        n_parts = len(self.graph.nodes)
        n_subs  = len(self.subassemblies)
        n_seqs  = len(self.all_sequences)
        opt     = self.optimized_sequence

        lines = [
            "─" * 60,
            "  ASSEMBLY PLANNING SUMMARY",
            "─" * 60,
            f"  Parts in assembly        : {n_parts}",
            f"  Subassemblies detected   : {n_subs}",
            f"  Valid sequences found    : {n_seqs}",
            "",
            "  OPTIMIZED SEQUENCE (Simulated Annealing)",
            f"    {opt.summary()}",
            "",
            "  GREEDY SEQUENCE",
            f"    {self.greedy_sequence.summary()}",
        ]

        if self.subassemblies:
            lines += ["", "  DETECTED SUBASSEMBLIES"]
            for sub in self.subassemblies[:5]:
                lines.append(f"    {sub}")

        if self.articulation_points:
            lines += ["", "  CRITICAL JUNCTION PARTS (articulation points)"]
            for ap in self.articulation_points:
                lines.append(
                    f"    {ap['part_id']}  "
                    f"degree={ap['degree']}  "
                    f"splits_into={ap['components_if_removed']} groups"
                )

        lines.append("─" * 60)
        return "\n".join(lines)

    def optimized_steps(self) -> list[dict]:
        """
        Return the optimized sequence as a list of step dicts suitable for the GUI.
        Each dict: {part_id, part_name, process, direction, time_s}
        """
        seq = self.optimized_sequence.sequence
        steps = []
        for pid in seq:
            node = self.graph.nodes.get(pid)
            if node is None:
                continue
            steps.append({
                "part_id":         pid,
                "part_name":       node.name,
                "process":         "",
                "direction":       node.direction or "—",
                "time_s":          node.assembly_time,
                "subassembly_id":  node.subassembly_id or "—",
                "tools":           ", ".join(sorted(node.tools)) if node.tools else "—",
            })
        return steps


# ── planner ───────────────────────────────────────────────────────────────────

class AssemblyPlanner:
    """
    High-level assembly planner.

    Build it from an Assembly object or a bare list of part IDs, then
    populate the liaison matrix before calling plan().
    """

    def __init__(
        self,
        part_ids:     list[str],
        part_names:   dict[str, str] | None  = None,
        part_times:   dict[str, float] | None = None,
        part_tools:   dict[str, set[str]] | None = None,
        part_dirs:    dict[str, str] | None  = None,
        base_part_id: str | None = None,
        weights:      OptimizationWeights | None = None,
    ) -> None:
        self.part_ids     = list(part_ids)
        self.part_names   = part_names   or {p: p for p in part_ids}
        self.part_times   = part_times   or {p: 2.93 for p in part_ids}
        self.part_tools   = part_tools   or {}
        self.part_dirs    = part_dirs    or {}
        self.base_part_id = base_part_id or (part_ids[0] if part_ids else "")
        self.weights      = weights or OptimizationWeights()

        # The liaison matrix — populate via planner.liaison.add_contact()
        self.liaison = LiaisonMatrix(part_ids)

    # ── factories ─────────────────────────────────────────────────────────────

    @classmethod
    def from_assembly(cls, assembly, **kwargs) -> "AssemblyPlanner":
        """
        Create a planner from a dfma Assembly object.
        Part times are populated from the geometry scorer.
        """
        from dfma.rules.geometry_scorer import estimate_total_time

        parts = assembly.all_parts()
        ids   = [p.id for p in parts]
        names = {p.id: p.name for p in parts}
        times = {p.id: estimate_total_time(p.geometry) for p in parts}
        dirs  = {}  # assembly directions not yet in the model — set manually

        planner = cls(
            part_ids=ids,
            part_names=names,
            part_times=times,
            part_dirs=dirs,
            **kwargs,
        )
        return planner

    # ── planning ──────────────────────────────────────────────────────────────

    def plan(
        self,
        detection_method: str = "auto",
        max_sequences:    int = 200,
        sa_iterations:    int = 2000,
        sa_seed:          int | None = 42,
    ) -> AssemblyPlan:
        """
        Run the full assembly planning pipeline.

        Steps:
          1. Build AssemblyGraph from liaison matrix (liaison-based precedence)
          2. Detect subassemblies
          3. Label each AssemblyNode with its subassembly_id
          4. Generate liaison-filtered sequences
          5. Optimize with greedy + simulated annealing
          6. Return AssemblyPlan

        Parameters
        ----------
        detection_method : "auto" | "biconnected" | "cut_set" | "both"
        max_sequences    : max valid sequences to enumerate
        sa_iterations    : SA iterations
        sa_seed          : random seed for SA
        """

        # ── step 1: build precedence graph ───────────────────────────────────
        graph = AssemblyGraph.from_liaison(self.liaison, self.base_part_id)

        # Enrich nodes with timing, tools, direction data
        for pid in self.part_ids:
            node = graph.nodes.get(pid)
            if node:
                node.name          = self.part_names.get(pid, pid)
                node.assembly_time = self.part_times.get(pid, 2.93)
                node.tools         = self.part_tools.get(pid, set())
                node.direction     = self.part_dirs.get(pid, "")

        # ── step 2: detect subassemblies ─────────────────────────────────────
        detector      = SubassemblyDetector(self.liaison)
        subassemblies = detector.detect(method=detection_method)
        art_points    = detector.articulation_point_report()

        # ── step 3: label nodes with subassembly_id ───────────────────────────
        for sub in subassemblies:
            for pid in sub.part_ids:
                node = graph.nodes.get(pid)
                if node and not node.subassembly_id:
                    node.subassembly_id = sub.id

        # ── step 4: generate sequences ────────────────────────────────────────
        generator  = SequenceGenerator(graph, self.liaison, self.base_part_id)
        all_seqs   = generator.liaison_sequences(max_results=max_sequences)

        if not all_seqs:
            # Fall back to pure topological if liaison predicate eliminates everything
            all_seqs = generator.all_sequences(max_results=max_sequences)

        # ── step 5: optimize ──────────────────────────────────────────────────
        optimizer = SequenceOptimizer(graph, self.weights)

        greedy_result = optimizer.greedy()

        sa_start = greedy_result.sequence  # warm-start SA from greedy result
        sa_result = optimizer.simulated_annealing(
            initial_sequence=sa_start,
            max_iterations=sa_iterations,
            seed=sa_seed,
        )

        # Score and rank all enumerated sequences
        top5 = optimizer.rank_sequences(all_seqs)[:5]

        # Pick the overall best
        best = sa_result
        if top5 and top5[0].total_cost < best.total_cost:
            best = top5[0]

        # ── step 6: return plan ───────────────────────────────────────────────
        return AssemblyPlan(
            subassemblies=subassemblies,
            articulation_points=art_points,
            all_sequences=all_seqs,
            greedy_sequence=greedy_result,
            optimized_sequence=best,
            top_sequences=top5,
            graph=graph,
            liaison=self.liaison,
        )
