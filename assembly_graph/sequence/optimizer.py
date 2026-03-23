"""
SequenceOptimizer — ranks and optimizes assembly sequences.

Optimization objectives (all minimised):
  1. Direction changes    — how many times the assembly must be re-oriented
  2. Tool changes         — how many times the worker must change tools
  3. Total assembly time  — sum of per-part handling + insertion times
  4. Subassembly breaks   — how many times the sequence switches subassembly
                            (penalty for incoherent ordering)

Algorithms
──────────
  greedy()              → local greedy: always pick the lowest-cost available part
  simulated_annealing() → SA: perturb valid sequences, accept worse with
                              decreasing probability
  rank_sequences()      → score and sort a list of pre-generated sequences

Cost function
─────────────
  cost(sequence) = w1 * direction_changes(seq)
                 + w2 * tool_changes(seq)
                 + w3 * normalised_time(seq)
                 + w4 * subassembly_breaks(seq)

Weights are configurable; defaults reflect DFA best practices (direction
changes are the dominant cost in manual assembly lines).

Reference:
  Boothroyd & Dewhurst (2010) — assembly direction cost model.
  Lu & Myres (1994)           — assembly sequence optimisation objective functions.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable

from ..assembly_graph import AssemblyGraph


@dataclass
class OptimizationWeights:
    """Configurable weights for the multi-objective cost function."""
    direction_changes:  float = 3.0   # penalty per re-orientation
    tool_changes:       float = 2.0   # penalty per tool swap
    assembly_time:      float = 1.0   # penalty per second (normalised)
    subassembly_breaks: float = 2.5   # penalty per subassembly switch


@dataclass
class ScoredSequence:
    """A sequence with its computed cost breakdown."""
    sequence:           list[str]
    total_cost:         float
    direction_changes:  int   = 0
    tool_changes:       int   = 0
    total_time_s:       float = 0.0
    subassembly_breaks: int   = 0

    def summary(self) -> str:
        return (
            f"cost={self.total_cost:.2f}  "
            f"dir_changes={self.direction_changes}  "
            f"tool_changes={self.tool_changes}  "
            f"time={self.total_time_s:.1f}s  "
            f"sub_breaks={self.subassembly_breaks}"
        )


class SequenceOptimizer:
    """
    Multi-objective assembly sequence optimizer.

    Parameters
    ----------
    graph   : AssemblyGraph — provides node metadata (direction, tools, time)
    weights : OptimizationWeights — cost function weights
    """

    def __init__(
        self,
        graph:   AssemblyGraph,
        weights: OptimizationWeights | None = None,
    ) -> None:
        self.graph   = graph
        self.weights = weights or OptimizationWeights()

    # ── scoring ───────────────────────────────────────────────────────────────

    def score(self, sequence: list[str]) -> ScoredSequence:
        """Compute the full cost breakdown for one sequence."""
        w = self.weights
        nodes = self.graph.nodes

        dir_changes   = 0
        tool_changes  = 0
        sub_breaks    = 0
        total_time    = 0.0

        prev_dir      = ""
        prev_tools: set[str] = set()
        prev_sub      = ""

        for pid in sequence:
            node = nodes.get(pid)
            if node is None:
                continue

            cur_dir   = node.direction
            cur_tools = node.tools
            cur_sub   = node.subassembly_id
            time_s    = node.assembly_time

            if prev_dir and cur_dir and cur_dir != prev_dir:
                dir_changes += 1
            if prev_tools and cur_tools and not cur_tools.intersection(prev_tools):
                tool_changes += 1
            if prev_sub and cur_sub and cur_sub != prev_sub:
                sub_breaks += 1

            total_time += time_s
            prev_dir    = cur_dir or prev_dir
            prev_tools  = cur_tools or prev_tools
            prev_sub    = cur_sub or prev_sub

        # Normalise time: divide by sum of all node times (0–1 range)
        max_time = sum(n.assembly_time for n in nodes.values()) or 1.0
        norm_time = total_time / max_time

        cost = (
            w.direction_changes  * dir_changes   +
            w.tool_changes       * tool_changes  +
            w.assembly_time      * norm_time      +
            w.subassembly_breaks * sub_breaks
        )

        return ScoredSequence(
            sequence=sequence,
            total_cost=round(cost, 4),
            direction_changes=dir_changes,
            tool_changes=tool_changes,
            total_time_s=round(total_time, 2),
            subassembly_breaks=sub_breaks,
        )

    def rank_sequences(
        self,
        sequences: list[list[str]],
    ) -> list[ScoredSequence]:
        """Score a list of sequences and return them sorted best→worst."""
        scored = [self.score(seq) for seq in sequences]
        scored.sort(key=lambda s: s.total_cost)
        return scored

    # ── greedy optimizer ──────────────────────────────────────────────────────

    def greedy(self) -> ScoredSequence:
        """
        Greedy algorithm: at each step, pick the available part with the
        lowest incremental cost addition.

        "Available" = all predecessors already assembled (in-degree 0 in
        the remaining subgraph).

        Runs in O(n²) time.
        """
        in_deg = {pid: self.graph.in_degree(pid) for pid in self.graph.nodes}
        assembled:  set[str]   = set()
        sequence:   list[str]  = []

        prev_dir      = ""
        prev_tools:   set[str] = set()
        prev_sub      = ""
        w = self.weights

        while len(sequence) < len(self.graph.nodes):
            available = [
                pid for pid, d in in_deg.items()
                if d == 0 and pid not in assembled
            ]
            if not available:
                break  # cycle or done

            # Score each candidate by the marginal cost it would add
            def marginal_cost(pid: str) -> float:
                node     = self.graph.nodes[pid]
                cur_dir  = node.direction
                cur_tools = node.tools
                cur_sub  = node.subassembly_id
                cost     = 0.0
                if prev_dir and cur_dir and cur_dir != prev_dir:
                    cost += w.direction_changes
                if prev_tools and cur_tools and not cur_tools.intersection(prev_tools):
                    cost += w.tool_changes
                if prev_sub and cur_sub and cur_sub != prev_sub:
                    cost += w.subassembly_breaks
                # Prefer shorter assembly-time parts (bias towards faster steps first)
                cost += w.assembly_time * (node.assembly_time / max(
                    max(n.assembly_time for n in self.graph.nodes.values()), 1.0
                ))
                return cost

            best = min(available, key=marginal_cost)
            sequence.append(best)
            assembled.add(best)

            node = self.graph.nodes[best]
            prev_dir   = node.direction or prev_dir
            prev_tools = node.tools    or prev_tools
            prev_sub   = node.subassembly_id or prev_sub

            # Update in-degrees
            for s in self.graph.successors(best):
                in_deg[s] -= 1

        return self.score(sequence)

    # ── simulated annealing ──────────────────────────────────────────────────

    def simulated_annealing(
        self,
        initial_sequence: list[str] | None = None,
        max_iterations:   int   = 2000,
        T_start:          float = 5.0,
        T_end:            float = 0.05,
        seed:             int | None = None,
    ) -> ScoredSequence:
        """
        Simulated annealing over the space of valid topological orderings.

        Neighbourhood move: swap two elements at positions i and j only if
        the resulting sequence is still a valid topological order.

        Acceptance: accept worse solutions with probability exp(-Δcost / T).

        Parameters
        ----------
        initial_sequence : starting point (defaults to Kahn's sort)
        max_iterations   : total SA steps
        T_start          : initial temperature
        T_end            : final temperature
        seed             : random seed for reproducibility
        """
        rng = random.Random(seed)

        # Start from Kahn's sequence if none provided
        if initial_sequence is None:
            try:
                initial_sequence = self.graph.topological_sort()
            except Exception:
                return self.greedy()

        current_seq  = list(initial_sequence)
        current_sc   = self.score(current_seq)
        best_sc      = current_sc

        # Pre-compute valid swap check
        def is_valid_swap(seq: list[str], i: int, j: int) -> bool:
            """
            Swapping positions i and j is valid iff all precedence
            constraints are still satisfied in the resulting sequence.
            """
            new_seq = list(seq)
            new_seq[i], new_seq[j] = new_seq[j], new_seq[i]
            pos = {pid: k for k, pid in enumerate(new_seq)}
            for a in self.graph.nodes:
                for b in self.graph.successors(a):
                    if pos[a] >= pos[b]:
                        return False
            return True

        # Temperature schedule (geometric cooling)
        T   = T_start
        alpha = (T_end / T_start) ** (1.0 / max(max_iterations, 1))
        n   = len(current_seq)

        for _ in range(max_iterations):
            if n < 2:
                break

            # Random swap of two positions
            i, j = sorted(rng.sample(range(n), 2))
            if not is_valid_swap(current_seq, i, j):
                T *= alpha
                continue

            new_seq = list(current_seq)
            new_seq[i], new_seq[j] = new_seq[j], new_seq[i]
            new_sc   = self.score(new_seq)

            delta = new_sc.total_cost - current_sc.total_cost

            # Accept improvement, or worse solution with Boltzmann probability
            if delta < 0 or (T > 0 and rng.random() < math.exp(-delta / T)):
                current_seq  = new_seq
                current_sc   = new_sc
                if current_sc.total_cost < best_sc.total_cost:
                    best_sc = current_sc

            T *= alpha

        return best_sc

    # ── subassembly-aware reorder ────────────────────────────────────────────

    def reorder_by_subassembly(
        self,
        sequence: list[str],
        subassembly_groups: list[frozenset[str]],
    ) -> list[str]:
        """
        Reorder a valid sequence to maximise subassembly coherence
        (all parts of the same subassembly are assembled consecutively).

        Approach: group parts by their subassembly assignment, then
        order groups by earliest required step, maintaining all
        precedence constraints within and between groups.

        Returns the best found ordering (may still split groups if
        precedence constraints require it).
        """
        # Map part → subassembly index
        part_to_sub: dict[str, int] = {}
        for idx, group in enumerate(subassembly_groups):
            for pid in group:
                part_to_sub[pid] = idx

        # Build a priority based on first occurrence in original sequence
        pos = {pid: i for i, pid in enumerate(sequence)}

        # Greedy reassembly: process subassemblies in order of their
        # earliest part's position, respecting precedence
        in_deg = {pid: self.graph.in_degree(pid) for pid in self.graph.nodes}
        result: list[str] = []
        assembled: set[str] = set()

        # Determine subassembly order by earliest part
        sub_first = {}
        for pid in sequence:
            si = part_to_sub.get(pid, -1)
            if si not in sub_first:
                sub_first[si] = pos[pid]
        ordered_subs = sorted(sub_first, key=lambda s: sub_first[s])

        for sub_idx in ordered_subs:
            # Get all parts in this subassembly that are now available
            members = [
                pid for pid in sequence
                if part_to_sub.get(pid, -1) == sub_idx
            ]
            for pid in members:
                if in_deg.get(pid, 0) == 0 and pid not in assembled:
                    result.append(pid)
                    assembled.add(pid)
                    for s in self.graph.successors(pid):
                        in_deg[s] = max(0, in_deg.get(s, 0) - 1)

        # Append any remaining parts not yet placed
        for pid in sequence:
            if pid not in assembled:
                result.append(pid)

        return result
