"""
SubassemblyDetector — identifies subassembly groups from the liaison graph.

Two complementary methods:

1. Biconnected Components (BCC)
   ─────────────────────────────
   A biconnected component is a maximal set of nodes such that the subgraph
   they induce has no cut-vertex (articulation point).  Each BCC represents
   a "rigid" group of parts that are mutually dependent and form a natural
   subassembly unit.

   Articulation points are the "joints" between BCCs — they appear in
   multiple BCCs and connect them.

   Reference: Hopcroft & Tarjan (1973) — linear-time articulation points.
   networkx: nx.biconnected_components(), nx.articulation_points()

2. Cut-Set Method (recursive graph bipartition)
   ─────────────────────────────────────────────
   Recursively finds minimum vertex cuts in the liaison graph.  Removing
   the cut vertices splits the graph into two (or more) groups — each group
   plus the shared cut vertices forms a candidate subassembly.

   Smaller cut-sets = more independent subassemblies = better modularity.

   Reference: Gulivindala et al. (2021) — modified cut-set method.
   networkx: nx.minimum_node_cut(), nx.connected_components()

Scoring
───────
Each detected subassembly is scored on:
  - Stability    : ratio of structural contacts to total contacts
  - Compactness  : 1 / (max_eccentricity of liaison subgraph)
  - Size penalty : penalty for too large (>70% of total) or too small (<2 parts)
  - Independence : fraction of subassembly contacts that are internal
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
try:
    import networkx as nx
except ImportError:
    raise ImportError(
        "networkx is required.\n"
        "Install it with:  pip install networkx"
    ) from None

from .liaison_matrix import LiaisonMatrix


@dataclass
class DetectedSubassembly:
    """One candidate subassembly group."""
    id:               str
    part_ids:         frozenset[str]
    method:           str            # "biconnected" | "cut_set" | "manual"
    stability_score:  float = 0.0   # 0–1: fraction of structural contacts
    compactness:      float = 0.0   # 0–1: graph diameter-based
    independence:     float = 0.0   # 0–1: internal contacts / total contacts
    overall_score:    float = 0.0   # weighted composite

    # assembly context
    base_part_id:     str = ""       # suggested first part in this subassembly
    level:            int = 0        # nesting depth (0 = top, higher = deeper)

    @property
    def size(self) -> int:
        return len(self.part_ids)

    def __repr__(self) -> str:
        parts = ", ".join(sorted(self.part_ids))
        return (f"SubAssembly({self.id}: [{parts}]  "
                f"score={self.overall_score:.2f}  method={self.method})")


class SubassemblyDetector:
    """
    Detects natural subassembly groupings from a LiaisonMatrix.

    Usage
    ─────
        detector = SubassemblyDetector(liaison_matrix)
        subs     = detector.detect()          # recommended: auto method
        subs_bcc = detector.detect_biconnected()
        subs_cut = detector.detect_cut_set()
    """

    def __init__(self, liaison: LiaisonMatrix) -> None:
        self.liaison    = liaison
        self._G         = liaison.to_networkx()
        self._n_total   = len(liaison.part_ids)
        self._n_contacts = len(liaison.all_contacts())

    # ── public API ───────────────────────────────────────────────────────────

    def detect(self, method: str = "auto") -> list[DetectedSubassembly]:
        """
        Detect subassemblies using the best available method.

        method = "auto"        → biconnected for simple graphs, cut_set for complex
               = "biconnected" → biconnected components only
               = "cut_set"     → recursive cut-set only
               = "both"        → merge results from both methods
        """
        if method == "biconnected":
            subs = self.detect_biconnected()
        elif method == "cut_set":
            subs = self.detect_cut_set()
        elif method == "both":
            subs = self._merge(self.detect_biconnected(), self.detect_cut_set())
        else:  # auto
            if self._n_total <= 20:
                subs = self.detect_cut_set()
            else:
                subs = self.detect_biconnected()

        # Score and sort
        for sub in subs:
            sub.overall_score = self._score(sub)
            sub.base_part_id  = self._pick_base(sub)

        subs.sort(key=lambda s: s.overall_score, reverse=True)
        return subs

    # ── method 1: biconnected components ─────────────────────────────────────

    def detect_biconnected(self) -> list[DetectedSubassembly]:
        """
        Find subassemblies as biconnected components of the liaison graph.

        A BCC with ≥ 2 nodes is a candidate subassembly.
        Single-node "BCCs" represent isolated parts that should be checked
        for missing liaison data.
        """
        results: list[DetectedSubassembly] = []
        art_points = set(nx.articulation_points(self._G))

        for idx, comp in enumerate(nx.biconnected_components(self._G)):
            part_ids = frozenset(comp)
            if len(part_ids) < 2:
                continue
            sub = DetectedSubassembly(
                id=f"BCC{idx:03d}",
                part_ids=part_ids,
                method="biconnected",
                stability_score=self._stability(part_ids),
                compactness=self._compactness(part_ids),
                independence=self._independence(part_ids),
            )
            results.append(sub)

        # Also expose articulation points as metadata
        self.articulation_points: set[str] = art_points
        return results

    # ── method 2: recursive cut-set ──────────────────────────────────────────

    def detect_cut_set(
        self,
        min_size: int = 2,
        max_depth: int = 6,
    ) -> list[DetectedSubassembly]:
        """
        Recursively partition the liaison graph using minimum vertex cuts.

        At each level:
          1. Find the minimum vertex cut of the current subgraph.
          2. Remove cut vertices → connected components appear.
          3. Each component + the cut vertices = candidate subassembly.
          4. Recurse into each component.

        If the graph has no cut vertex (biconnected), it is treated as a
        single subassembly.
        """
        results: list[DetectedSubassembly] = []
        counter = [0]
        seen_sets: set[frozenset] = set()

        def recurse(G: nx.Graph, depth: int) -> None:
            if depth > max_depth or G.number_of_nodes() < min_size:
                return

            # Minimum vertex cut (set of nodes whose removal disconnects G)
            try:
                cut_nodes = nx.minimum_node_cut(G)
            except nx.NetworkXError:
                cut_nodes = set()

            if not cut_nodes:
                # Biconnected — this whole subgraph is one unit
                pid_set = frozenset(G.nodes())
                if len(pid_set) >= min_size and pid_set not in seen_sets:
                    seen_sets.add(pid_set)
                    sub = DetectedSubassembly(
                        id=f"CUT{counter[0]:03d}",
                        part_ids=pid_set,
                        method="cut_set",
                        stability_score=self._stability(pid_set),
                        compactness=self._compactness(pid_set),
                        independence=self._independence(pid_set),
                        level=depth,
                    )
                    results.append(sub)
                    counter[0] += 1
                return

            # Remove cut nodes and find remaining components
            H = G.copy()
            H.remove_nodes_from(cut_nodes)

            for comp_nodes in nx.connected_components(H):
                # The candidate subassembly = component + shared cut nodes
                candidate = frozenset(comp_nodes | cut_nodes)
                if len(candidate) >= min_size and candidate not in seen_sets:
                    seen_sets.add(candidate)
                    sub = DetectedSubassembly(
                        id=f"CUT{counter[0]:03d}",
                        part_ids=candidate,
                        method="cut_set",
                        stability_score=self._stability(candidate),
                        compactness=self._compactness(candidate),
                        independence=self._independence(candidate),
                        level=depth,
                    )
                    results.append(sub)
                    counter[0] += 1
                    # Recurse into the component (without cut nodes)
                    sub_G = G.subgraph(comp_nodes | cut_nodes).copy()
                    recurse(sub_G, depth + 1)

        recurse(self._G, depth=0)
        return results

    # ── scoring helpers ──────────────────────────────────────────────────────

    def _stability(self, part_ids: frozenset[str]) -> float:
        """
        Fraction of contacts within the subassembly that are structural.
        A fully-structural subassembly scores 1.0.
        """
        contacts = [
            c for c in self.liaison.all_contacts()
            if c.part_a in part_ids and c.part_b in part_ids
        ]
        if not contacts:
            return 0.0
        structural = sum(1 for c in contacts if c.is_structural)
        return structural / len(contacts)

    def _compactness(self, part_ids: frozenset[str]) -> float:
        """
        1 - (diameter / n): compact subassemblies have small diameter.
        Returns 1.0 for a single connected path, lower for star graphs.
        """
        if len(part_ids) < 2:
            return 1.0
        sub = self._G.subgraph(part_ids)
        if not nx.is_connected(sub):
            return 0.0
        try:
            diam = nx.diameter(sub)
            n    = len(part_ids)
            return max(0.0, 1.0 - (diam - 1) / max(n - 1, 1))
        except nx.NetworkXError:
            return 0.0

    def _independence(self, part_ids: frozenset[str]) -> float:
        """
        Fraction of the subassembly's total contacts that are internal.
        High independence → subassembly can be pre-built independently.
        """
        internal = sum(
            1 for c in self.liaison.all_contacts()
            if c.part_a in part_ids and c.part_b in part_ids
        )
        external = sum(
            1 for c in self.liaison.all_contacts()
            if (c.part_a in part_ids) != (c.part_b in part_ids)
        )
        total = internal + external
        return internal / total if total else 0.0

    def _score(self, sub: DetectedSubassembly) -> float:
        """
        Weighted composite score (0–1).

        Higher = better subassembly candidate.
        Penalise very small or very large subassemblies.
        """
        n_ratio = sub.size / max(self._n_total, 1)

        # Size score: peak around 20–50% of total parts
        if sub.size < 2:
            size_score = 0.0
        elif n_ratio > 0.85:
            size_score = 0.2  # too large — nearly the whole assembly
        else:
            # bell-shaped peak at n_ratio ≈ 0.35
            size_score = 1.0 - abs(n_ratio - 0.35) / 0.65

        return (
            0.35 * sub.stability_score +
            0.25 * sub.compactness     +
            0.25 * sub.independence    +
            0.15 * size_score
        )

    def _pick_base(self, sub: DetectedSubassembly) -> str:
        """
        Suggest the first part to assemble in this subassembly.
        Heuristic: the part with the highest degree (most connections) in the subgraph.
        """
        sub_g = self._G.subgraph(sub.part_ids)
        if not sub_g.nodes:
            return ""
        return max(sub_g.nodes, key=lambda n: sub_g.degree(n))

    # ── merging / deduplication ───────────────────────────────────────────────

    def _merge(
        self,
        list_a: list[DetectedSubassembly],
        list_b: list[DetectedSubassembly],
    ) -> list[DetectedSubassembly]:
        """
        Combine two detection results, removing near-duplicate subassemblies.
        Two subassemblies are considered duplicates if their Jaccard similarity > 0.85.
        """
        merged: list[DetectedSubassembly] = list(list_a)
        for sub_b in list_b:
            if not any(_jaccard(sub_b.part_ids, sub_a.part_ids) > 0.85 for sub_a in merged):
                merged.append(sub_b)
        return merged

    # ── articulation point report ────────────────────────────────────────────

    def articulation_point_report(self) -> list[dict]:
        """
        Return a list of articulation points with metadata.
        Useful for identifying critical junction parts.
        """
        art = list(nx.articulation_points(self._G))
        report = []
        for pid in sorted(art):
            # How many components result from removing this node?
            H = self._G.copy()
            H.remove_node(pid)
            n_comp = nx.number_connected_components(H)
            report.append({
                "part_id":         pid,
                "degree":          self._G.degree(pid),
                "components_if_removed": n_comp,
                "is_critical":     n_comp > 1,
            })
        return report


def _jaccard(a: frozenset, b: frozenset) -> float:
    inter = len(a & b)
    union = len(a | b)
    return inter / union if union else 0.0
