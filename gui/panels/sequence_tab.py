"""
SequenceTab — CENTRE tab 2: Assembly Sequence.

Displays the optimized assembly order as a numbered step list.
Each step shows:
  - Step number
  - Part name
  - Insertion direction
  - Tools required
  - Subassembly group
  - Estimated assembly time (from geometry scorer)
  - Cumulative time

A "Generate Sequence" button triggers AssemblyPlanner.plan() via the
"gen_sequence" EventBus event.

EventBus events consumed:
  "assembly_loaded"  → enable Generate button
  "sequence_ready"   → populate the step list
                       (payload: list of step dicts from plan.optimized_steps())
  "part_selected"    → highlight the corresponding row

EventBus events published:
  "gen_sequence"     → when user clicks Generate (carries algo_var string)
  "part_selected"    → when user clicks a row
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus


class SequenceTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._build()
        self._subscribe()

    def _build(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=4, pady=4)

        ttk.Button(
            toolbar, text="Generate Sequence",
            command=lambda: self.bus.publish("gen_sequence", self._algo_var.get()),
        ).pack(side=tk.LEFT)

        ttk.Label(toolbar, text="Algorithm:").pack(side=tk.LEFT, padx=(12, 2))
        self._algo_var = tk.StringVar(value="Optimized (Simulated Annealing)")
        ttk.Combobox(
            toolbar, textvariable=self._algo_var, width=28, state="readonly",
            values=[
                "Optimized (Simulated Annealing)",
                "Greedy (Min Cost)",
                "Topological (Kahn)",
            ],
        ).pack(side=tk.LEFT)

        # ── step list ────────────────────────────────────────────────────────
        cols = ("step", "part", "direction", "tools", "subasm", "time_s", "cumul_s")
        self.table = ttk.Treeview(
            self, columns=cols, show="headings", selectmode="browse"
        )
        self.table.heading("step",      text="#")
        self.table.heading("part",      text="Part")
        self.table.heading("direction", text="Dir")
        self.table.heading("tools",     text="Tools")
        self.table.heading("subasm",    text="Subassembly")
        self.table.heading("time_s",    text="Time (s)")
        self.table.heading("cumul_s",   text="Cumul (s)")

        self.table.column("step",      width=32,  stretch=False, anchor=tk.CENTER)
        self.table.column("part",      width=150, stretch=True)
        self.table.column("direction", width=42,  stretch=False, anchor=tk.CENTER)
        self.table.column("tools",     width=120, stretch=False)
        self.table.column("subasm",    width=70,  stretch=False)
        self.table.column("time_s",    width=60,  stretch=False, anchor=tk.E)
        self.table.column("cumul_s",   width=65,  stretch=False, anchor=tk.E)

        sb = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.table.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))
        self.table.bind("<<TreeviewSelect>>", self._on_row_select)

        # ── summary bar ──────────────────────────────────────────────────────
        self._summary_var = tk.StringVar(value="No sequence generated.")
        ttk.Label(self, textvariable=self._summary_var, anchor=tk.W).pack(
            fill=tk.X, padx=6, pady=(0, 4))

    def _subscribe(self) -> None:
        self.bus.subscribe("sequence_ready", self._on_sequence_ready)
        self.bus.subscribe("part_selected",  self._on_part_selected)

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_sequence_ready(self, payload) -> None:
        """
        Populate the step table.

        payload can be:
          - list of step dicts (keys: part_id, part_name, direction, tools,
                                      subassembly_id, time_s)
          - dict {"steps": [...], "cost_summary": "..."}  (from gen_sequence)
        """
        if isinstance(payload, dict) and "steps" in payload:
            steps   = payload["steps"]
            summary = payload.get("cost_summary", "")
        else:
            steps   = payload or []
            summary = ""

        self.table.delete(*self.table.get_children())
        if not steps:
            self._summary_var.set("Empty sequence.")
            return

        cumul = 0.0
        for i, step in enumerate(steps, start=1):
            t = step.get("time_s", 0.0)
            cumul += t
            self.table.insert(
                "", "end",
                iid=step["part_id"],
                values=(
                    i,
                    step.get("part_name", step["part_id"]),
                    step.get("direction", "—"),
                    step.get("tools",     "—"),
                    step.get("subassembly_id", "—"),
                    f"{t:.1f}",
                    f"{cumul:.1f}",
                ),
            )

        base = f"{len(steps)} steps  |  Total: {cumul:.1f} s"
        self._summary_var.set(f"{base}  |  {summary}" if summary else base)

    def _on_part_selected(self, part_id: str) -> None:
        try:
            self.table.selection_set(part_id)
            self.table.see(part_id)
        except tk.TclError:
            pass

    def _on_row_select(self, _event) -> None:
        sel = self.table.focus()
        if sel:
            self.bus.publish("part_selected", sel)
