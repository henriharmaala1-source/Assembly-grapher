"""
SequenceGenerator — produces valid assembly orderings from an AssemblyGraph.

Methods
───────
  topological_sort()      → one valid sequence via Kahn's BFS algorithm
  all_sequences()         → all valid topological orderings (bounded)
  liaison_sequences()     → sequences that also satisfy the liaison predicate
                            (each added part must contact the partial assembly)

Liaison Predicate
─────────────────
A part passes the liaison predicate at step k if it has at least one contact
with a part already in the assembled set.

This eliminates "floating" insertions where a part is added to empty space
with no mechanical connection to what's already assembled.

Reference:
  Bourjault (1984), de Mello & Sanderson (1990) — liaison predicate.
"""

from __future__ import annotations
import time
from collections import deque

from ..assembly_graph  import AssemblyGraph
from ..liaison_matrix  import LiaisonMatrix


class SequenceGenerator:
    """
    Generates feasible assembly sequences from a precedence graph,
    optionally filtered by the liaison predicate.

    Parameters
    ----------
    graph    : AssemblyGraph — precedence DAG
    liaison  : LiaisonMatrix (optional) — for liaison predicate filtering
    base_part: str (optional) — ID of the first part placed on the fixture
    """

    def __init__(
        self,
        graph:     AssemblyGraph,
        liaison:   LiaisonMatrix | None = None,
        base_part: str | None           = None,
    ) -> None:
        self.graph     = graph
        self.liaison   = liaison
        self.base_part = base_part or (graph.roots()[0] if graph.roots() else "")

    # ── single sequence (Kahn's) ─────────────────────────────────────────────

    def topological_sort(self) -> list[str]:
        """
        Kahn's BFS topological sort — returns one deterministic valid sequence.
        When multiple nodes are available, prefers the one with the most successors
        (depth-first greediness).
        """
        return self.graph.topological_sort()

    # ── all valid sequences ──────────────────────────────────────────────────

    def all_sequences(self, max_results: int = 200) -> list[list[str]]:
        """
        Enumerate all topological orderings of the precedence graph.
        Capped at max_results for large assemblies.
        """
        return self.graph.all_topological_sorts(max_results=max_results)

    # ── liaison-filtered sequences ────────────────────────────────────────────

    def liaison_sequences(self, max_results: int = 200, timeout_s: float = 8.0) -> list[list[str]]:
        """
        Enumerate assembly sequences that satisfy both:
          1. The precedence constraints (topological order)
          2. The liaison predicate (each part contacts the partial assembly)

        If no liaison matrix is supplied, falls back to all_sequences().
        """
        if self.liaison is None:
            return self.all_sequences(max_results)

        results: list[list[str]] = []
        current: list[str]       = []
        assembled: set[str]      = set()
        deadline = time.monotonic() + timeout_s

        # Pre-compute precedence in-degrees
        in_deg = {pid: self.graph.in_degree(pid) for pid in self.graph.nodes}

        def _liaison_ok(pid: str, assembled: set[str]) -> bool:
            """Part passes liaison predicate: touches at least one assembled part."""
            if not assembled:
                return True   # first part — no assembly yet
            return any(
                self.liaison.has_contact(pid, q)
                for q in assembled
            )

        def backtrack(in_deg: dict, assembled: set) -> None:
            if len(results) >= max_results or time.monotonic() >= deadline:
                return
            if len(current) == len(self.graph.nodes):
                results.append(list(current))
                return

            # Available = zero in-degree + passes liaison predicate
            available = sorted(
                pid for pid, d in in_deg.items()
                if d == 0 and pid not in assembled and _liaison_ok(pid, assembled)
            )

            for pid in available:
                current.append(pid)
                assembled.add(pid)

                new_in_deg = dict(in_deg)
                del new_in_deg[pid]
                for s in self.graph.successors(pid):
                    if s in new_in_deg:
                        new_in_deg[s] -= 1

                backtrack(new_in_deg, assembled)

                current.pop()
                assembled.discard(pid)

        backtrack(in_deg, assembled)

        # If no liaison-valid sequence found (e.g. disconnected graph),
        # fall back to pure topological sequences
        return results if results else self.all_sequences(max_results)

    # ── sequence statistics ───────────────────────────────────────────────────

    def count_valid_sequences(self) -> int:
        """
        Return the number of valid sequences (up to a ceiling of 10 000).
        Useful for reporting combinatorial complexity.
        """
        return len(self.graph.all_topological_sorts(max_results=10_000))

    def sequence_to_steps(self, sequence: list[str]) -> list[dict]:
        """
        Convert a list of part_ids to a list of step dicts for the GUI:
          [{"part_id", "part_name", "process", "direction", "time_s"}, ...]
        """
        steps = []
        for pid in sequence:
            node = self.graph.nodes.get(pid)
            if node is None:
                continue
            steps.append({
                "part_id":   pid,
                "part_name": node.name,
                "process":   "",
                "direction": node.direction or "—",
                "time_s":    node.assembly_time,
            })
        return steps
