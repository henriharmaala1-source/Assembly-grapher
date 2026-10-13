"""
AssemblyGraph — Directed Acyclic Graph (DAG) of assembly precedence.

Nodes represent parts (or detected subassemblies).
A directed edge  A → B  means "A must be assembled before B".

Precedence constraints come from:
  1. Liaison-based feasibility: a part can only join the assembly if it
     is in contact with at least one already-assembled part.
  2. Geometric blocking: part A physically blocks access to B's insertion.
  3. Process rules: fasteners installed after the parts they join.
  4. Subassembly grouping: a whole subassembly node replaces its members.

The graph is built incrementally and supports:
  - Cycle detection
  - Topological sort (Kahn's BFS)
  - Subassembly node collapsing (replace a group of parts with one node)
  - Export to networkx DiGraph
"""

from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
from typing import Iterator


@dataclass
class AssemblyNode:
    """A node in the assembly precedence graph."""
    part_id:        str
    name:           str
    assembly_time:  float = 0.0    # estimated s (from geometry scorer)
    tools:          set[str] = field(default_factory=set)
    direction:      str = ""       # assembly direction "+Z", "-X", etc.
    subassembly_id: str = ""       # set by SubassemblyDetector if part of a sub
    is_subassembly: bool = False   # True if this node represents a collapsed sub
    member_ids:     list[str] = field(default_factory=list)  # parts in collapsed sub

    def __hash__(self):  return hash(self.part_id)
    def __eq__(self, o): return isinstance(o, AssemblyNode) and self.part_id == o.part_id


class CycleError(Exception):
    """Raised when a cycle is detected in the assembly graph."""


class AssemblyGraph:
    """
    Directed Acyclic Graph for assembly precedence.

    Internal representation uses adjacency sets (predecessors + successors)
    for O(1) edge queries.
    """

    def __init__(self) -> None:
        self.nodes:         dict[str, AssemblyNode] = {}
        self._pred:         dict[str, set[str]] = {}   # node → set of predecessors
        self._succ:         dict[str, set[str]] = {}   # node → set of successors

    # ── node management ──────────────────────────────────────────────────────

    def add_node(self, node: AssemblyNode) -> None:
        pid = node.part_id
        self.nodes[pid] = node
        self._pred.setdefault(pid, set())
        self._succ.setdefault(pid, set())

    def remove_node(self, pid: str) -> None:
        """Remove a node and all its edges."""
        for p in list(self._pred.get(pid, [])):
            self._succ[p].discard(pid)
        for s in list(self._succ.get(pid, [])):
            self._pred[s].discard(pid)
        self._pred.pop(pid, None)
        self._succ.pop(pid, None)
        self.nodes.pop(pid, None)

    # ── edge management ───────────────────────────────────────────────────────

    def add_precedence(self, before: str, after: str) -> None:
        """
        Add directed edge before → after.
        Raises CycleError if this would create a cycle.
        """
        if before == after:
            return
        if before not in self.nodes or after not in self.nodes:
            missing = before if before not in self.nodes else after
            raise KeyError(f"Node '{missing}' not in graph")
        # Check for reverse path (would create cycle)
        if self._can_reach(after, before):
            raise CycleError(
                f"Adding edge {before}→{after} would create a cycle "
                f"(path {after}→…→{before} already exists)"
            )
        self._succ[before].add(after)
        self._pred[after].add(before)

    def has_edge(self, before: str, after: str) -> bool:
        return after in self._succ.get(before, set())

    def predecessors(self, pid: str) -> set[str]:
        return set(self._pred.get(pid, set()))

    def successors(self, pid: str) -> set[str]:
        return set(self._succ.get(pid, set()))

    # ── graph queries ─────────────────────────────────────────────────────────

    def roots(self) -> list[str]:
        """Nodes with no predecessors — can be assembled first."""
        return [pid for pid, preds in self._pred.items() if not preds]

    def leaves(self) -> list[str]:
        """Nodes with no successors — assembled last."""
        return [pid for pid, succs in self._succ.items() if not succs]

    def in_degree(self, pid: str) -> int:
        return len(self._pred.get(pid, set()))

    def _can_reach(self, src: str, dst: str) -> bool:
        """BFS reachability: can we reach dst from src?"""
        visited: set[str] = set()
        queue = deque([src])
        while queue:
            cur = queue.popleft()
            if cur == dst:
                return True
            if cur in visited:
                continue
            visited.add(cur)
            queue.extend(self._succ.get(cur, set()))
        return False

    # ── topological ordering ─────────────────────────────────────────────────

    def topological_sort(self) -> list[str]:
        """
        Kahn's BFS topological sort.
        Returns one valid assembly sequence as a list of part_ids.
        Raises CycleError if a cycle exists (should not happen if add_precedence
        guards are used, but checked here for safety).
        """
        in_deg = {pid: len(preds) for pid, preds in self._pred.items()}
        queue  = deque(pid for pid, d in in_deg.items() if d == 0)
        result: list[str] = []

        while queue:
            pid = queue.popleft()
            result.append(pid)
            for s in sorted(self._succ.get(pid, set())):  # sorted for determinism
                in_deg[s] -= 1
                if in_deg[s] == 0:
                    queue.append(s)

        if len(result) != len(self.nodes):
            cycle_nodes = set(self.nodes) - set(result)
            raise CycleError(f"Cycle detected involving: {cycle_nodes}")
        return result

    def all_topological_sorts(
        self,
        max_results: int = 500,
    ) -> list[list[str]]:
        """
        Enumerate all valid topological orderings (feasible assembly sequences).

        Uses recursive backtracking — exponential worst case.
        Capped at max_results to remain practical.

        Reference: Knuth's algorithm for all topological sorts.
        """
        results:  list[list[str]] = []
        current:  list[str]       = []
        in_deg    = {pid: len(preds) for pid, preds in self._pred.items()}
        available = sorted(pid for pid, d in in_deg.items() if d == 0)

        def backtrack(in_deg: dict, available: list[str]) -> None:
            if len(results) >= max_results:
                return
            if not available:
                if len(current) == len(self.nodes):
                    results.append(list(current))
                return
            for pid in list(available):
                # Choose pid next
                current.append(pid)
                new_avail = [p for p in available if p != pid]
                new_in_deg = dict(in_deg)
                for s in self._succ.get(pid, set()):
                    new_in_deg[s] -= 1
                    if new_in_deg[s] == 0:
                        new_avail.append(s)
                new_avail.sort()
                backtrack(new_in_deg, new_avail)
                current.pop()

        backtrack(in_deg, available)
        return results

    # ── subassembly collapsing ────────────────────────────────────────────────

    def collapse_subassembly(
        self,
        sub_id: str,
        member_ids: list[str],
        sub_name: str = "",
        sub_time: float = 0.0,
    ) -> "AssemblyGraph":
        """
        Return a new AssemblyGraph where all members of a subassembly are
        collapsed into a single node.

        Edges to/from any member are redirected to the new sub node.
        """
        new_graph = AssemblyGraph()
        member_set = set(member_ids)

        # Add collapsed subassembly node
        sub_node = AssemblyNode(
            part_id=sub_id,
            name=sub_name or sub_id,
            assembly_time=sub_time,
            is_subassembly=True,
            member_ids=list(member_ids),
        )
        new_graph.add_node(sub_node)

        # Add all non-member nodes
        for pid, node in self.nodes.items():
            if pid not in member_set:
                new_graph.add_node(node)

        # Redirect edges: member → X  becomes  sub → X  (if X not in member)
        #                 X → member  becomes  X → sub   (if X not in member)
        added_edges: set[tuple[str, str]] = set()

        def mapped(pid: str) -> str:
            return sub_id if pid in member_set else pid

        for pid in list(self.nodes):
            for s in self._succ.get(pid, set()):
                a, b = mapped(pid), mapped(s)
                if a != b and (a, b) not in added_edges:
                    try:
                        new_graph.add_precedence(a, b)
                        added_edges.add((a, b))
                    except (CycleError, KeyError):
                        pass
        return new_graph

    # ── networkx export ───────────────────────────────────────────────────────

    def to_networkx(self):
        """Export as networkx.DiGraph."""
        import networkx as nx
        G = nx.DiGraph()
        for pid, node in self.nodes.items():
            G.add_node(pid, name=node.name, direction=node.direction,
                       subassembly_id=node.subassembly_id,
                       assembly_time=node.assembly_time)
        for pid in self.nodes:
            for s in self._succ.get(pid, set()):
                G.add_edge(pid, s)
        return G

    # ── liaison-based feasibility constraints ─────────────────────────────────

    @classmethod
    def from_liaison(
        cls,
        liaison,          # LiaisonMatrix
        base_part_id: str,
    ) -> "AssemblyGraph":
        """
        Build a precedence graph from a liaison matrix.

        Strategy: BFS from the base part (the first part placed on the fixture).
        Each part can be added once it has a liaison with an already-scheduled part.
        This gives an initial valid ordering; the optimizer refines it.

        Parameters
        ----------
        liaison : LiaisonMatrix
        base_part_id : str
            The first part placed on the fixture (no predecessors).
        """
        graph = cls()
        parts = liaison.part_ids

        # Add all parts as nodes
        for pid in parts:
            graph.add_node(AssemblyNode(part_id=pid, name=pid))

        # BFS from base: each newly-reachable part depends on its liaison partner
        scheduled:   set[str]  = {base_part_id}
        remaining:   set[str]  = set(parts) - scheduled
        frontier:    list[str] = [base_part_id]

        while remaining and frontier:
            next_frontier: list[str] = []
            for current in frontier:
                for neighbor in liaison.neighbors(current):
                    if neighbor in remaining:
                        # neighbor can be assembled after current
                        try:
                            graph.add_precedence(current, neighbor)
                        except CycleError:
                            pass
                        scheduled.add(neighbor)
                        remaining.discard(neighbor)
                        next_frontier.append(neighbor)
            frontier = next_frontier

        return graph

    def __len__(self) -> int:
        return len(self.nodes)

    def __repr__(self) -> str:
        e = sum(len(s) for s in self._succ.values())
        return f"AssemblyGraph({len(self.nodes)} nodes, {e} edges)"
