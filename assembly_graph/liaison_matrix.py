"""
LiaisonMatrix — n×n symmetric binary matrix encoding physical contacts.

L[i][j] = 1  if part i and part j are physically in contact (liaison)
L[i][j] = 0  otherwise
L[i][i] = 0  always (no self-contact)

A liaison exists when two parts share a face, edge, or mating surface.
The matrix is the foundation for:
  - Building the assembly liaison graph
  - Subassembly detection (cut-set / biconnected components)
  - Validating assembly sequence feasibility (liaison predicate)

Reference:
  Dini & Santochi (1992) — binary liaison matrix for assembly planning.
  Bourjault (1984)       — liaison graph foundations.
"""

from __future__ import annotations
from dataclasses import dataclass, field


@dataclass
class Contact:
    """Metadata about a contact between two parts."""
    part_a: str
    part_b: str
    contact_type: str = "face"       # face | edge | point | thread | press_fit
    strength:     str = "rigid"      # rigid | flexible | sliding | fixed
    direction:    str = ""           # primary assembly direction of this contact ("+Z", "-X", etc.)
    is_structural: bool = True       # load-bearing vs cosmetic contact


class LiaisonMatrix:
    """
    Symmetric adjacency matrix for part-to-part physical contacts.

    Internally stored as a Python list-of-lists (no numpy required).
    """

    def __init__(self, part_ids: list[str]) -> None:
        self.part_ids: list[str] = list(part_ids)
        self._index:   dict[str, int] = {pid: i for i, pid in enumerate(self.part_ids)}
        n = len(self.part_ids)
        self._matrix: list[list[int]] = [[0] * n for _ in range(n)]
        self._contacts: dict[tuple[str, str], Contact] = {}

    # ── construction ────────────────────────────────────────────────────────

    def add_contact(
        self,
        pid_a: str,
        pid_b: str,
        contact_type: str = "face",
        strength:     str = "rigid",
        direction:    str = "",
        is_structural: bool = True,
    ) -> None:
        """Register a physical contact between two parts."""
        if pid_a not in self._index:
            raise KeyError(f"Part '{pid_a}' not in matrix")
        if pid_b not in self._index:
            raise KeyError(f"Part '{pid_b}' not in matrix")
        if pid_a == pid_b:
            raise ValueError("A part cannot contact itself")

        i, j = self._index[pid_a], self._index[pid_b]
        self._matrix[i][j] = 1
        self._matrix[j][i] = 1

        key = (min(pid_a, pid_b), max(pid_a, pid_b))
        self._contacts[key] = Contact(pid_a, pid_b, contact_type, strength,
                                       direction, is_structural)

    def add_part(self, pid: str) -> None:
        """Add a new part to the matrix (extends by one row/column)."""
        if pid in self._index:
            return
        n = len(self.part_ids)
        self.part_ids.append(pid)
        self._index[pid] = n
        for row in self._matrix:
            row.append(0)
        self._matrix.append([0] * (n + 1))

    # ── queries ──────────────────────────────────────────────────────────────

    def has_contact(self, pid_a: str, pid_b: str) -> bool:
        i, j = self._index[pid_a], self._index[pid_b]
        return bool(self._matrix[i][j])

    def get_contact(self, pid_a: str, pid_b: str) -> Contact | None:
        key = (min(pid_a, pid_b), max(pid_a, pid_b))
        return self._contacts.get(key)

    def neighbors(self, pid: str) -> list[str]:
        """Return all parts that are in contact with pid."""
        i = self._index[pid]
        return [self.part_ids[j] for j, v in enumerate(self._matrix[i]) if v]

    def degree(self, pid: str) -> int:
        """Number of contacts for a part."""
        return len(self.neighbors(pid))

    def all_contacts(self) -> list[Contact]:
        return list(self._contacts.values())

    def is_connected(self) -> bool:
        """True if the liaison graph is connected (all parts reachable from any part)."""
        if not self.part_ids:
            return True
        visited: set[str] = set()
        stack = [self.part_ids[0]]
        while stack:
            pid = stack.pop()
            if pid in visited:
                continue
            visited.add(pid)
            stack.extend(n for n in self.neighbors(pid) if n not in visited)
        return len(visited) == len(self.part_ids)

    # ── networkx conversion ─────────────────────────────────────────────────

    def to_networkx(self):
        """Return a networkx.Graph encoding the liaison relationships."""
        import networkx as nx
        G = nx.Graph()
        G.add_nodes_from(self.part_ids)
        for contact in self._contacts.values():
            G.add_edge(
                contact.part_a, contact.part_b,
                contact_type=contact.contact_type,
                strength=contact.strength,
                direction=contact.direction,
                is_structural=contact.is_structural,
            )
        return G

    # ── matrix display ───────────────────────────────────────────────────────

    def as_rows(self) -> list[list[int]]:
        """Return a copy of the raw n×n matrix."""
        return [row[:] for row in self._matrix]

    def __repr__(self) -> str:
        n = len(self.part_ids)
        header = "      " + "  ".join(f"{pid[:4]:>4}" for pid in self.part_ids)
        lines  = [f"LiaisonMatrix ({n}×{n})", header]
        for pid, row in zip(self.part_ids, self._matrix):
            lines.append(f"  {pid[:4]:>4}  " + "  ".join(str(v) for v in row))
        return "\n".join(lines)

    # ── factory helpers ──────────────────────────────────────────────────────

    @classmethod
    def from_assembly(cls, assembly) -> "LiaisonMatrix":
        """
        Build an empty LiaisonMatrix from an Assembly object.
        Contacts must be added manually with add_contact() afterwards.
        """
        mat = cls([p.id for p in assembly.all_parts()])
        return mat

    @classmethod
    def from_contact_list(
        cls,
        part_ids: list[str],
        contacts: list[tuple[str, str]],
    ) -> "LiaisonMatrix":
        """
        Convenience constructor:
            LiaisonMatrix.from_contact_list(
                ["A","B","C","D"],
                [("A","B"), ("B","C"), ("C","D"), ("A","D")]
            )
        """
        mat = cls(part_ids)
        for (a, b) in contacts:
            mat.add_contact(a, b)
        return mat
