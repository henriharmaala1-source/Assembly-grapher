"""
Warning data model returned by every DFMA rule checker.
"""

from dataclasses import dataclass, field
from enum import Enum


class Severity(str, Enum):
    ERROR   = "ERROR"    # will cause assembly/manufacturing failure
    WARNING = "WARNING"  # likely to cause cost or quality problems
    INFO    = "INFO"     # suggestion for improvement


@dataclass
class Warning:
    rule_id:     str        # unique rule identifier, e.g. "DFA-001"
    severity:    Severity
    part_id:     str        # which part triggered this warning
    message:     str        # human-readable description
    suggestion:  str = ""   # recommended corrective action
    metric_value: float | None = None   # the measured value that failed
    threshold:    float | None = None   # the threshold it was compared against

    def __str__(self) -> str:
        metric = ""
        if self.metric_value is not None and self.threshold is not None:
            metric = f" (value={self.metric_value:.3g}, threshold={self.threshold:.3g})"
        return (
            f"[{self.severity.value}] {self.rule_id} | {self.part_id} | "
            f"{self.message}{metric}"
        )


@dataclass
class AnalysisResult:
    """Aggregated result from a full DFMA analysis run."""
    assembly_name: str
    warnings:      list[Warning]       = field(default_factory=list)

    # ── Boothroyd-Dewhurst metrics ─────────────────────────────────────────
    total_parts:          int   = 0
    theoretical_minimum:  int   = 0
    total_assembly_time_s: float = 0.0   # estimated total handling + insertion time
    dfa_index:             float = 0.0   # efficiency = (N_min * 2.93) / T_total

    def errors(self)   -> list[Warning]:
        return [w for w in self.warnings if w.severity == Severity.ERROR]

    def warnings_only(self) -> list[Warning]:
        return [w for w in self.warnings if w.severity == Severity.WARNING]

    def infos(self) -> list[Warning]:
        return [w for w in self.warnings if w.severity == Severity.INFO]

    def summary(self) -> str:
        e = len(self.errors())
        w = len(self.warnings_only())
        i = len(self.infos())
        return (
            f"Assembly: {self.assembly_name}\n"
            f"  Parts (total / theoretical minimum): {self.total_parts} / {self.theoretical_minimum}\n"
            f"  Estimated assembly time: {self.total_assembly_time_s:.1f} s\n"
            f"  DFA Index (efficiency):  {self.dfa_index:.1%}\n"
            f"  Findings: {e} error(s), {w} warning(s), {i} info(s)\n"
        )
