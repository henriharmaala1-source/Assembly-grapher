"""
ResourcesTab — CENTRE tab 4: Assembly Tool & Torque Resources.

Shows which tools are required to assemble all fasteners:
  - Tool inventory treeview: tool type, size, count, torque range, fastener IDs
  - Zone breakdown treeview: per-subassembly kit + tool-change count
  - Export button for CSV

EventBus events consumed:
  "fasteners_loaded"   → recalculate resources
  "resources_updated"  → populate both treeviews
                         (payload: AssemblyResources)

EventBus events published:
  (none)
"""

import tkinter as tk
from tkinter import ttk, filedialog
import csv

from ..event_bus import EventBus
from ..theme import C


class ResourcesTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._resources = None
        self._build()
        self._subscribe()

    def _build(self) -> None:
        # ── toolbar ──────────────────────────────────────────────────────────
        toolbar = ttk.Frame(self)
        toolbar.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(toolbar, text="Export CSV", command=self._export_csv).pack(side=tk.LEFT)
        self._summary_var = tk.StringVar(value="No fasteners loaded.")
        ttk.Label(toolbar, textvariable=self._summary_var, anchor=tk.W).pack(
            side=tk.LEFT, padx=12)

        # ── paned: tool inventory (top) / zone breakdown (bottom) ────────────
        pane = ttk.PanedWindow(self, orient=tk.VERTICAL)
        pane.pack(fill=tk.BOTH, expand=True, padx=4, pady=(0, 4))

        # ── tool inventory table ─────────────────────────────────────────────
        top = ttk.LabelFrame(pane, text="Tool Inventory")
        pane.add(top, weight=3)

        tool_cols = ("tool", "size", "qty", "torque", "fasteners")
        self._tool_tree = ttk.Treeview(top, columns=tool_cols,
                                       show="headings", selectmode="browse")
        self._tool_tree.heading("tool",      text="Tool")
        self._tool_tree.heading("size",      text="Size")
        self._tool_tree.heading("qty",       text="Qty")
        self._tool_tree.heading("torque",    text="Torque (Nm)")
        self._tool_tree.heading("fasteners", text="Fasteners")

        self._tool_tree.column("tool",      width=200, stretch=True)
        self._tool_tree.column("size",      width=80,  stretch=False)
        self._tool_tree.column("qty",       width=40,  stretch=False, anchor=tk.CENTER)
        self._tool_tree.column("torque",    width=130, stretch=False, anchor=tk.E)
        self._tool_tree.column("fasteners", width=140, stretch=False)

        tsb = ttk.Scrollbar(top, orient=tk.VERTICAL, command=self._tool_tree.yview)
        self._tool_tree.configure(yscrollcommand=tsb.set)
        tsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._tool_tree.pack(fill=tk.BOTH, expand=True)

        # colour bands per tool type
        self._tool_tree.tag_configure("hex_key",      background=C.ROW_HEX)
        self._tool_tree.tag_configure("socket",       background=C.ROW_SOCKET)
        self._tool_tree.tag_configure("torx_bit",     background=C.ROW_TORX)
        self._tool_tree.tag_configure("phillips_bit", background=C.ROW_PHILIPS)
        self._tool_tree.tag_configure("pozi_bit",     background=C.ROW_POZI)
        self._tool_tree.tag_configure("slotted_bit",  background=C.ROW_SLOTTED)

        # ── zone breakdown table ─────────────────────────────────────────────
        bot = ttk.LabelFrame(pane, text="Tool Kit per Zone")
        pane.add(bot, weight=2)

        zone_cols = ("zone", "tools", "changes", "fasteners")
        self._zone_tree = ttk.Treeview(bot, columns=zone_cols,
                                       show="headings", selectmode="browse")
        self._zone_tree.heading("zone",      text="Zone")
        self._zone_tree.heading("tools",     text="Tools Required")
        self._zone_tree.heading("changes",   text="Tool Changes")
        self._zone_tree.heading("fasteners", text="Fasteners")

        self._zone_tree.column("zone",      width=160, stretch=False)
        self._zone_tree.column("tools",     width=240, stretch=True)
        self._zone_tree.column("changes",   width=90,  stretch=False, anchor=tk.CENTER)
        self._zone_tree.column("fasteners", width=140, stretch=False)

        zsb = ttk.Scrollbar(bot, orient=tk.VERTICAL, command=self._zone_tree.yview)
        self._zone_tree.configure(yscrollcommand=zsb.set)
        zsb.pack(side=tk.RIGHT, fill=tk.Y)
        self._zone_tree.pack(fill=tk.BOTH, expand=True)

        self._zone_tree.tag_configure("clean",   foreground=C.SEV_OK)
        self._zone_tree.tag_configure("oneswap", foreground=C.SEV_WARNING)
        self._zone_tree.tag_configure("many",    foreground=C.SEV_ERROR)

    def _subscribe(self) -> None:
        self.bus.subscribe("resources_updated", self._on_resources_updated)

    # ── population ────────────────────────────────────────────────────────────

    def _on_resources_updated(self, resources) -> None:
        self._resources = resources
        self._tool_tree.delete(*self._tool_tree.get_children())
        self._zone_tree.delete(*self._zone_tree.get_children())

        if resources is None:
            self._summary_var.set("No fasteners loaded.")
            return

        # Summary bar
        tc = resources.total_tool_changes
        self._summary_var.set(
            f"{resources.total_fasteners} fastener(s)  |  "
            f"{resources.unique_tool_count} unique tool(s)  |  "
            f"{tc} tool change(s) total"
        )

        # Tool inventory rows
        for t in resources.tools:
            self._tool_tree.insert(
                "", "end",
                values=(
                    t.label,
                    t.size_label,
                    f"×{t.count}",
                    t.torque_str(),
                    ", ".join(t.fastener_ids),
                ),
                tags=(t.tool_type,),
            )

        # Zone rows
        for zone in resources.zones:
            tool_str = " + ".join(t.size_label for t in zone.tools)
            ch = zone.tool_changes
            ch_str = f"{ch}" if ch > 0 else "0 (no change)"
            tag = "clean" if ch == 0 else ("oneswap" if ch == 1 else "many")
            self._zone_tree.insert(
                "", "end",
                values=(
                    zone.zone_id,
                    tool_str,
                    ch_str,
                    ", ".join(zone.fastener_ids),
                ),
                tags=(tag,),
            )

    # ── export ────────────────────────────────────────────────────────────────

    def _export_csv(self) -> None:
        if not self._resources:
            return
        path = filedialog.asksaveasfilename(
            title="Export Tool Resources",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["Tool", "Size", "Qty", "Torque_Min_Nm",
                        "Torque_Max_Nm", "Fasteners"])
            for t in self._resources.tools:
                w.writerow([t.label, t.size_label, t.count,
                            t.torque_min_nm, t.torque_max_nm,
                            "; ".join(t.fastener_ids)])
            w.writerow([])
            w.writerow(["Zone", "Tools", "Tool_Changes", "Fasteners"])
            for zone in self._resources.zones:
                w.writerow([
                    zone.zone_id,
                    " + ".join(t.size_label for t in zone.tools),
                    zone.tool_changes,
                    "; ".join(zone.fastener_ids),
                ])
