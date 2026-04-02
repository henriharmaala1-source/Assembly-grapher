"""
DfmaTab — Tab 2: DFMA Analysis.

Layout:
  ┌─────────────────────────────────────────────────────────────────────┐
  │  [Run DFMA]  [Run Screw Check]    DFA: 65%  │  27.3 s  │  7 errors │
  ├──────────────────────────────────┬──────────────────────────────────┤
  │  BOM Table (left ⅗)              │  Warnings + Detail (right ⅖)    │
  │  # │Part│Name│Qty│Process│Mat│↯  │  ● DFS-001  …                   │
  │    │    │    │   │       │   │✓  │  ▲ DFA-007  …                   │
  │    │    │    │   │       │   │   │  ─────────────────────────────  │
  │    │    │    │   │       │   │   │  Detail pane                    │
  └──────────────────────────────────┴──────────────────────────────────┘

EventBus events published:
  "run_dfma"    → via toolbar button
  "run_screw"   → via toolbar button

EventBus events consumed: (delegated to embedded BOMTab and WarningPanel)
  "assembly_loaded"   → BOMTab populates rows
  "warnings_updated"  → BOMTab colours rows; WarningPanel fills list; score updated
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus     import EventBus
from .bom_tab        import BOMTab
from .warning_panel  import WarningPanel


class DfmaTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._build()
        bus.subscribe("warnings_updated", self._on_result)

    def _build(self) -> None:
        # ── toolbar ───────────────────────────────────────────────────────────
        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, padx=8, pady=(8, 4))

        ttk.Button(bar, text="Run DFMA Analysis",
                   command=lambda: self.bus.publish("run_dfma", None)
                   ).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(bar, text="Run Screw Check",
                   command=lambda: self.bus.publish("run_screw", None)
                   ).pack(side=tk.LEFT, padx=4)

        ttk.Separator(bar, orient=tk.VERTICAL).pack(
            side=tk.LEFT, padx=10, fill=tk.Y)

        self._score_var = tk.StringVar(value="Run DFMA to see results.")
        ttk.Label(bar, textvariable=self._score_var, style="Muted.TLabel"
                  ).pack(side=tk.LEFT)

        # ── horizontal pane: BOM left │ Warnings right ────────────────────────
        pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

        self.bom_panel     = BOMTab(pane, self.bus)
        self.warning_panel = WarningPanel(pane, self.bus)

        pane.add(self.bom_panel,     weight=3)
        pane.add(self.warning_panel, weight=2)

    # ── event handler ─────────────────────────────────────────────────────────

    def _on_result(self, result) -> None:
        if result is None:
            return
        n_err  = len(result.errors())
        n_warn = len(result.warnings_only())
        n_info = len(result.infos())
        self._score_var.set(
            f"DFA Index: {result.dfa_index:.1%}   •   "
            f"Assembly time: {result.total_assembly_time_s:.1f} s   •   "
            f"Parts: {result.total_parts} (min {result.theoretical_minimum})   •   "
            f"{n_err} error(s)  {n_warn} warning(s)  {n_info} info(s)"
        )
