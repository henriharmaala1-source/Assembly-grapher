"""
SummaryTab — Tab 4: Summary.

Shows a complete at-a-glance overview of all analysis results:

  ┌─────────────────────────────────────────────────────────────────────┐
  │   ASSEMBLY                DFMA                  SEQUENCING          │
  │  ─────────────────   ─────────────────────   ──────────────────     │
  │  Name: Pneum. Valve  DFA Index:   65.0 %    Steps:      12          │
  │  Total parts:  12    Assembly time: 27.3 s  Total time: 27.3 s      │
  │  Subassemblies: 2    Min. parts:  5         Dir changes:  2         │
  │  Min. necessary: 5   Errors:       7        Tool changes: 3         │
  │                      Warnings:    14        Cost score:   8.40      │
  │                      Infos:        2                                 │
  ├─────────────────────────────────────────────────────────────────────┤
  │  DFMA Issues                                                         │
  │  ● ERROR  DFS-001  F001   tool clearance 5.0 mm < 8.0 mm           │
  │  ▲ WARN   DFA-007  ASM01  high part count …                         │
  │  …                                                                  │
  ├─────────────────────────────────────────────────────────────────────┤
  │  [Export Full Report…]                                              │
  └─────────────────────────────────────────────────────────────────────┘

EventBus events consumed:
  "assembly_loaded"   → update Assembly card
  "warnings_updated"  → update DFMA card + issues list
  "sequence_ready"    → update Sequencing card
  "resources_updated" → update resource note in Sequencing card

EventBus events published:
  "export_report"     → via Export button
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus


_SEV_ICON  = {"ERROR": "●", "WARNING": "▲", "INFO": "ℹ"}
_SEV_COLOR = {"ERROR": "#cc0000", "WARNING": "#cc7700", "INFO": "#0055cc"}


class SummaryTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._warnings: list = []
        self._build()
        self._subscribe()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build(self) -> None:
        # ── three metric cards ────────────────────────────────────────────────
        cards_frame = ttk.Frame(self)
        cards_frame.pack(fill=tk.X, padx=8, pady=(10, 4))

        self._asm_card  = _MetricCard(cards_frame, title="ASSEMBLY")
        self._dfa_card  = _MetricCard(cards_frame, title="DFMA RESULTS")
        self._seq_card  = _MetricCard(cards_frame, title="SEQUENCING")

        for card in (self._asm_card, self._dfa_card, self._seq_card):
            card.pack(side=tk.LEFT, expand=True, fill=tk.BOTH, padx=4)

        # Reset cards to waiting state
        self._asm_card.set_lines(["Load an assembly to begin."])
        self._dfa_card.set_lines(["Run DFMA analysis to see results."])
        self._seq_card.set_lines(["Generate a sequence to see results."])

        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=8, pady=4)

        # ── DFMA issues list ──────────────────────────────────────────────────
        issues_lf = ttk.LabelFrame(self, text="DFMA Issues")
        issues_lf.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))

        filter_bar = ttk.Frame(issues_lf)
        filter_bar.pack(fill=tk.X, padx=4, pady=(4, 0))
        ttk.Label(filter_bar, text="Show:").pack(side=tk.LEFT)
        self._filter_var = tk.StringVar(value="All")
        ttk.Combobox(
            filter_bar, textvariable=self._filter_var, width=10,
            state="readonly",
            values=["All", "ERROR", "WARNING", "INFO"],
        ).pack(side=tk.LEFT, padx=4)
        self._filter_var.trace_add("write", lambda *_: self._repopulate_issues())

        cols = ("sev", "rule", "part", "msg")
        self._issues = ttk.Treeview(issues_lf, columns=cols,
                                    show="headings", selectmode="browse")
        self._issues.heading("sev",  text="")
        self._issues.heading("rule", text="Rule")
        self._issues.heading("part", text="Part")
        self._issues.heading("msg",  text="Message")
        self._issues.column("sev",  width=22,  stretch=False, anchor=tk.CENTER)
        self._issues.column("rule", width=70,  stretch=False)
        self._issues.column("part", width=65,  stretch=False)
        self._issues.column("msg",  width=500, stretch=True)

        self._issues.tag_configure("ERROR",   foreground="#cc0000")
        self._issues.tag_configure("WARNING", foreground="#cc7700")
        self._issues.tag_configure("INFO",    foreground="#0055cc")

        sb = ttk.Scrollbar(issues_lf, orient=tk.VERTICAL,
                           command=self._issues.yview)
        self._issues.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._issues.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ── export button ─────────────────────────────────────────────────────
        ttk.Button(
            self, text="Export Full Report…",
            command=lambda: self.bus.publish("export_report", None),
        ).pack(side=tk.BOTTOM, anchor=tk.E, padx=10, pady=6)

    def _subscribe(self) -> None:
        self.bus.subscribe("assembly_loaded",  self._on_assembly)
        self.bus.subscribe("warnings_updated", self._on_warnings)
        self.bus.subscribe("sequence_ready",   self._on_sequence)
        self.bus.subscribe("resources_updated",self._on_resources)

    # ── event handlers ─────────────────────────────────────────────────────────

    def _on_assembly(self, assembly) -> None:
        if not hasattr(assembly, "all_parts"):
            return
        parts  = assembly.all_parts()
        nsub   = len(assembly.subassemblies)
        try:
            n_min = assembly.theoretical_minimum_parts()
        except Exception:
            n_min = "—"
        self._asm_card.set_lines([
            f"Name:          {assembly.name}",
            f"Total parts:   {len(parts)}",
            f"Subassemblies: {nsub}",
            f"Min. necessary:{n_min}",
        ])

    def _on_warnings(self, result) -> None:
        if result is None:
            return
        self._warnings = result.warnings
        n_err  = len(result.errors())
        n_warn = len(result.warnings_only())
        n_info = len(result.infos())
        color  = ("#cc0000" if n_err else
                  "#cc7700" if n_warn else "#006622")
        self._dfa_card.set_lines([
            f"DFA Index:     {result.dfa_index:.1%}",
            f"Assembly time: {result.total_assembly_time_s:.1f} s",
            f"Total parts:   {result.total_parts}",
            f"Min. necessary:{result.theoretical_minimum}",
            f"Errors:        {n_err}",
            f"Warnings:      {n_warn}",
            f"Infos:         {n_info}",
        ], highlight_color=color)
        self._repopulate_issues()

    def _on_sequence(self, payload) -> None:
        if isinstance(payload, dict) and "steps" in payload:
            steps   = payload["steps"]
            summary = payload.get("cost_summary", "")
        elif isinstance(payload, list):
            steps   = payload
            summary = ""
        else:
            return

        total_t = sum(s.get("time_s", 0.0) for s in steps)
        # Parse cost summary for direction/tool changes if available
        dir_ch = tool_ch = cost = "—"
        for token in summary.split("|"):
            t = token.strip()
            if "dir" in t.lower():
                dir_ch = t.split("=")[-1].strip()
            elif "tool" in t.lower():
                tool_ch = t.split("=")[-1].strip()
            elif "cost" in t.lower():
                cost = t.split("=")[-1].strip()

        self._seq_card.set_lines([
            f"Steps:         {len(steps)}",
            f"Total time:    {total_t:.1f} s",
            f"Dir. changes:  {dir_ch}",
            f"Tool changes:  {tool_ch}",
            f"Cost score:    {cost}",
        ])

    def _on_resources(self, resources) -> None:
        if resources is None:
            return
        # Append resource note to sequence card
        existing = self._seq_card.get_lines()
        tool_line = (f"Unique tools:  {resources.unique_tool_count}"
                     f"  ({resources.total_fasteners} fastener(s))")
        if tool_line not in existing:
            self._seq_card.set_lines(existing + [tool_line])

    # ── helpers ────────────────────────────────────────────────────────────────

    def _repopulate_issues(self) -> None:
        self._issues.delete(*self._issues.get_children())
        filt = self._filter_var.get()
        for i, w in enumerate(self._warnings):
            sev = w.severity.value
            if filt != "All" and sev != filt:
                continue
            short = w.message[:80] + ("…" if len(w.message) > 80 else "")
            self._issues.insert(
                "", "end", iid=str(i),
                values=(_SEV_ICON.get(sev, ""), w.rule_id, w.part_id, short),
                tags=(sev,),
            )


# ── simple metric card widget ─────────────────────────────────────────────────

class _MetricCard(ttk.LabelFrame):
    """A framed box that displays a title and a list of key–value lines."""

    def __init__(self, parent, title: str) -> None:
        super().__init__(parent, text=title)
        self._lines: list[str] = []
        self._highlight: str | None = None
        self._text = tk.Text(
            self, height=8, state=tk.DISABLED, relief=tk.FLAT,
            bg=self.winfo_toplevel().cget("bg") if hasattr(self, "winfo_toplevel") else "#f0f0f0",
            font=("Courier", 9), wrap=tk.NONE,
        )
        self._text.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

    def set_lines(self, lines: list[str],
                  highlight_color: str | None = None) -> None:
        self._lines = list(lines)
        self._highlight = highlight_color
        self._text.configure(state=tk.NORMAL)
        self._text.delete("1.0", tk.END)
        self._text.insert("1.0", "\n".join(lines))
        if highlight_color:
            self._text.configure(foreground=highlight_color)
        else:
            self._text.configure(foreground="")
        self._text.configure(state=tk.DISABLED)

    def get_lines(self) -> list[str]:
        return list(self._lines)
