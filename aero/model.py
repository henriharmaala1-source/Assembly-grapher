"""What a project model hands to the optimizer."""
from __future__ import annotations

from dataclasses import dataclass, field

from dragkit import Item


@dataclass
class Param:
    name: str
    value: float                # the design as it is
    lo: float
    hi: float
    step: float
    unit: str
    what: str                   # what the number changes
    limit: str = ""             # what sets the bounds
    kind: str = ""              # e.g. "CAD parameter", "what-if", "part choice"
    labels: dict | None = None  # names for discrete values

    def show(self, v: float) -> str:
        if self.labels:
            return self.labels.get(round(v), str(v))
        digits = 0 if self.step >= 1 else len(f"{self.step:g}".split(".")[1])
        return f"{v:.{digits}f}" + (f" {self.unit}" if self.unit else "")

    def snap(self, v: float) -> float:
        v = min(self.hi, max(self.lo, v))
        return round(round((v - self.lo) / self.step) * self.step + self.lo, 6)


@dataclass
class Result:
    value: float                            # the objective (the model says whether up or down is better)
    items: list[Item] = field(default_factory=list)                   # drag build-up, if any
    metrics: list[tuple[str, str]] = field(default_factory=list)      # numbers to show
    violations: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.violations
