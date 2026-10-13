"""
SequencingTab — Tab 3: Assembly Sequencing.

Layout:
  ┌────────────────────────────────────────────────────────────────────────┐
  │  [Generate Sequence]  Algorithm: [Optimized (SA) ▾]                   │
  ├─────────────────────────────────────────┬──────────────────────────────┤
  │  Sequence steps (left ⅗)               │  Tool Resources (right ⅖)   │
  │  # │ Part       │ Dir │ Tools │ T(s)   │  ┌──────────────────────────┐│
  │  1 │ Valve Body │ +Z  │ —     │ 3.2   │  │ Tool Inventory           ││
  │  2 │ Piston     │ +Z  │ —     │ 2.1   │  │  tool │size│qty│torque   ││
  │  …                                     │  └──────────────────────────┘│
  │  12 steps │ Total: 27.3 s │ cost=8.40  │  ┌──────────────────────────┐│
  │                                         │  │ Zones                    ││
  │                                         │  └──────────────────────────┘│
  └─────────────────────────────────────────┴──────────────────────────────┘

Embeds the existing SequenceTab (left) and ResourcesTab (right) as
sub-widgets inside a horizontal PanedWindow.

EventBus events published:
  "gen_sequence"      → via toolbar button (carries algo string)

EventBus events consumed: (delegated to embedded panels)
  "assembly_loaded"   → SequenceTab enables button
  "sequence_ready"    → SequenceTab populates steps
  "resources_updated" → ResourcesTab populates tool tables
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus      import EventBus
from .sequence_tab    import SequenceTab
from .resources_tab   import ResourcesTab


class SequencingTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._build()

    def _build(self) -> None:
        # ── horizontal pane: sequence left │ resources right ─────────────────
        pane = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        pane.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.seq_panel       = SequenceTab(pane, self.bus)
        self.resources_panel = ResourcesTab(pane, self.bus)

        pane.add(self.seq_panel,       weight=3)
        pane.add(self.resources_panel, weight=2)
