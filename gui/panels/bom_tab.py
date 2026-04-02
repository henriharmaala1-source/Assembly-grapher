"""
BOMTab — CENTRE tab 3: Bill of Materials.

Flat or indented BOM table with columns:
  Item | Part No | Name | Qty | Process | Material | DFA Status | DFM Status

EventBus events consumed:
  "assembly_loaded"  → populate BOM from Assembly object
  "warnings_updated" → update status columns

EventBus events published:
  "part_selected"    → when row clicked
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus
from ..theme import C


COLS = ("item", "part_no", "name", "qty", "process", "material", "dfa", "dfm")
HEADS = ("#",  "Part No", "Name", "Qty", "Process", "Material", "DFA", "DFM")
WIDTHS = (30,   80,        160,    35,    110,        80,          45,    45)


class BOMTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._build()
        self._subscribe()

    def _build(self) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=4, pady=4)
        ttk.Label(toolbar, text="View:").pack(side=tk.LEFT)
        self._view_var = tk.StringVar(value="Indented")
        ttk.Combobox(
            toolbar, textvariable=self._view_var, width=12, state="readonly",
            values=["Indented", "Flat"],
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="Export CSV",
                   command=lambda: self.bus.publish("export_bom_csv", None)
        ).pack(side=tk.RIGHT)

        self.table = ttk.Treeview(
            self, columns=COLS, show="headings", selectmode="browse"
        )
        for col, head, width in zip(COLS, HEADS, WIDTHS):
            anchor = tk.E if col in ("item", "qty") else tk.W
            self.table.heading(col, text=head)
            self.table.column(col, width=width, stretch=(col == "name"), anchor=anchor)

        sb = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.table.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))
        self.table.bind("<<TreeviewSelect>>", self._on_select)

        # severity row tints
        self.table.tag_configure("error",   background=C.ROW_ERROR)
        self.table.tag_configure("warning", background=C.ROW_WARNING)
        self.table.tag_configure("info",    background=C.ROW_INFO)

    def _subscribe(self) -> None:
        self.bus.subscribe("assembly_loaded",  self._on_assembly_loaded)
        self.bus.subscribe("warnings_updated", self._on_warnings_updated)

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_assembly_loaded(self, assembly) -> None:
        self.table.delete(*self.table.get_children())
        if assembly is None:
            return
        for i, part in enumerate(assembly.all_parts(), start=1):
            self.table.insert(
                "", "end", iid=part.id,
                values=(
                    i,
                    part.id,
                    part.name,
                    part.quantity,
                    part.process.value,
                    part.material.value,
                    "—", "—",
                ),
            )

    def _on_warnings_updated(self, result) -> None:
        if result is None:
            return
        dfa_worst: dict[str, str] = {}
        dfm_worst: dict[str, str] = {}
        for w in result.warnings:
            pid = w.part_id
            if w.rule_id.startswith("DFA"):
                if pid not in dfa_worst or w.severity.value < dfa_worst[pid]:
                    dfa_worst[pid] = w.severity.value
            elif w.rule_id.startswith("DFM"):
                if pid not in dfm_worst or w.severity.value < dfm_worst[pid]:
                    dfm_worst[pid] = w.severity.value

        sev_icon = {"ERROR": "✗", "WARNING": "⚠", "INFO": "ℹ"}
        sev_tag  = {"ERROR": "error", "WARNING": "warning", "INFO": "info"}

        for pid in self.table.get_children():
            vals = list(self.table.item(pid, "values"))
            dfa_sev = dfa_worst.get(pid)
            dfm_sev = dfm_worst.get(pid)
            vals[6] = sev_icon.get(dfa_sev, "✓") if dfa_sev else "✓"
            vals[7] = sev_icon.get(dfm_sev, "✓") if dfm_sev else "✓"
            tag = sev_tag.get(dfa_sev or dfm_sev, "")
            self.table.item(pid, values=vals, tags=(tag,))

    def _on_select(self, _event) -> None:
        sel = self.table.focus()
        if sel:
            self.bus.publish("part_selected", sel)
