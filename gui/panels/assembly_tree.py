"""
AssemblyTreePanel — LEFT panel.

Shows two sections:
  1. Assembly Tree   — hierarchical treeview of Assembly / Part nodes
                       with icons indicating worst-severity warning.
  2. Fastener List   — flat list of FastenerSpec objects with pass/fail status.

EventBus events consumed:
  "assembly_loaded"   → repopulate tree from loaded Assembly object
  "fasteners_loaded"  → repopulate fastener list
  "warnings_updated"  → recolour nodes based on new analysis results

EventBus events published:
  "part_selected"     → payload: part_id  (graph + warning panel highlight)
  "fastener_selected" → payload: fastener_id
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus
from ..theme import C


class AssemblyTreePanel(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._build()
        self._subscribe()

    def _build(self) -> None:
        # ── Assembly section ────────────────────────────────────────────────
        ttk.Label(self, text="Assembly", font=("", 9, "bold")).pack(
            anchor=tk.W, padx=6, pady=(6, 0))

        tree_frame = ttk.Frame(self)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=4)

        self.tree = ttk.Treeview(
            tree_frame,
            columns=("type", "status"),
            show="tree headings",
            selectmode="browse",
        )
        self.tree.heading("#0",       text="Name")
        self.tree.heading("type",     text="Type")
        self.tree.heading("status",   text="Status")
        self.tree.column("#0",        width=130, stretch=True)
        self.tree.column("type",      width=50,  stretch=False)
        self.tree.column("status",    width=50,  stretch=False)

        sb = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._on_part_select)

        # ── Fastener section ─────────────────────────────────────────────────
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
        ttk.Label(self, text="Fasteners", font=("", 9, "bold")).pack(
            anchor=tk.W, padx=6)

        fas_frame = ttk.Frame(self)
        fas_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

        self.fas_tree = ttk.Treeview(
            fas_frame,
            columns=("status",),
            show="tree headings",
            selectmode="browse",
            height=6,
        )
        self.fas_tree.heading("#0",     text="Fastener")
        self.fas_tree.heading("status", text="OK?")
        self.fas_tree.column("#0",      width=160, stretch=True)
        self.fas_tree.column("status",  width=40,  stretch=False)

        sb2 = ttk.Scrollbar(fas_frame, orient=tk.VERTICAL, command=self.fas_tree.yview)
        self.fas_tree.configure(yscrollcommand=sb2.set)
        sb2.pack(side=tk.RIGHT, fill=tk.Y)
        self.fas_tree.pack(fill=tk.BOTH, expand=True)
        self.fas_tree.bind("<<TreeviewSelect>>", self._on_fastener_select)

    def _subscribe(self) -> None:
        self.bus.subscribe("assembly_loaded",  self._on_assembly_loaded)
        self.bus.subscribe("fasteners_loaded", self._on_fasteners_loaded)
        self.bus.subscribe("warnings_updated", self._on_warnings_updated)

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_assembly_loaded(self, assembly) -> None:
        """Populate the assembly treeview from an Assembly object."""
        self.tree.delete(*self.tree.get_children())
        if assembly is None:
            return
        self._insert_assembly(assembly, parent="")

    def _insert_assembly(self, asm, parent: str) -> None:
        node = self.tree.insert(
            parent, "end", iid=asm.id,
            text=asm.name, values=("ASM", "—"),
            open=True,
        )
        for part in asm.parts:
            self.tree.insert(
                node, "end", iid=part.id,
                text=part.name,
                values=(part.process.value[:6], "—"),
            )
        for sub in asm.subassemblies:
            self._insert_assembly(sub, parent=node)

    def _on_fasteners_loaded(self, fasteners) -> None:
        """Populate the fastener list."""
        self.fas_tree.delete(*self.fas_tree.get_children())
        if not fasteners:
            return
        for fs in fasteners:
            self.fas_tree.insert("", "end", iid=fs.id, text=fs.name, values=("?",))

    def _on_warnings_updated(self, result) -> None:
        """Colour-code nodes based on worst warning severity."""
        if result is None:
            return
        worst: dict[str, str] = {}
        for w in result.warnings:
            pid = w.part_id
            sev = w.severity.value
            if pid not in worst or sev < worst[pid]:
                worst[pid] = sev

        severity_tag = {"ERROR": "error", "WARNING": "warning", "INFO": "info"}
        self.tree.tag_configure("error",   foreground=C.SEV_ERROR)
        self.tree.tag_configure("warning", foreground=C.SEV_WARNING)
        self.tree.tag_configure("info",    foreground=C.SEV_INFO)

        for pid, sev in worst.items():
            tag = severity_tag.get(sev, "")
            try:
                self.tree.item(pid, tags=(tag,))
                status = {"ERROR": "✗", "WARNING": "⚠", "INFO": "ℹ"}.get(sev, "✓")
                vals = list(self.tree.item(pid, "values"))
                if vals:
                    vals[-1] = status
                    self.tree.item(pid, values=vals)
            except tk.TclError:
                pass  # node not in tree (fastener or sub-asm)

        # update fastener list status
        fastener_ids = {fs_id for w in result.warnings
                        if (fs_id := w.part_id).startswith("F")}
        for pid, sev in worst.items():
            if pid in fastener_ids:
                status = {"ERROR": "✗", "WARNING": "⚠", "INFO": "ℹ"}.get(sev, "✓")
                try:
                    self.fas_tree.item(pid, values=(status,))
                except tk.TclError:
                    pass

    def _on_part_select(self, _event) -> None:
        sel = self.tree.focus()
        if sel:
            self.bus.publish("part_selected", sel)

    def _on_fastener_select(self, _event) -> None:
        sel = self.fas_tree.focus()
        if sel:
            self.bus.publish("fastener_selected", sel)
