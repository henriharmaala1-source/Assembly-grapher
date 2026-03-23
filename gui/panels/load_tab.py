"""
LoadTab — Tab 1: Load Parts.

Allows the user to:
  • Open a JSON BOM file  (no dependencies)
  • Open a STEP file      (requires pythonOCC)
  • Load the built-in demo assembly

Shows a lightweight BOM preview table and any import warnings produced by
the importer.  A "Run Full Analysis →" button triggers both DFMA and
sequence generation so the other tabs are populated in one click.

EventBus events published:
  "assembly_loaded"   → Assembly object
  "fasteners_loaded"  → list[FastenerSpec]  (demo only)
  "resources_updated" → AssemblyResources   (demo only)
  "run_dfma"          → None
  "gen_sequence"      → algo string

EventBus events consumed:
  "assembly_loaded"   → refresh BOM preview (for pre-loaded demo from gui_main)
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from ..event_bus import EventBus


class LoadTab(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus  = bus
        self._assembly = None
        self._build()
        self._subscribe()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build(self) -> None:
        # ── toolbar ──────────────────────────────────────────────────────────
        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, padx=8, pady=(10, 4))

        ttk.Label(bar, text="Load assembly:", font=("", 10, "bold")).pack(
            side=tk.LEFT, padx=(0, 8))

        ttk.Button(bar, text="Open JSON BOM…",   command=self._open_json).pack(
            side=tk.LEFT, padx=3)
        ttk.Button(bar, text="Open STEP File…",  command=self._open_step).pack(
            side=tk.LEFT, padx=3)
        ttk.Separator(bar, orient=tk.VERTICAL).pack(
            side=tk.LEFT, padx=10, fill=tk.Y)
        ttk.Button(bar, text="Load Demo Assembly", command=self._load_demo).pack(
            side=tk.LEFT, padx=3)

        ttk.Button(
            bar, text="Run Full Analysis  →",
            command=self._run_all,
        ).pack(side=tk.RIGHT, padx=3)

        # ── assembly info bar ─────────────────────────────────────────────────
        self._info_var = tk.StringVar(value="No assembly loaded — open a file or load the demo.")
        ttk.Label(
            self, textvariable=self._info_var, foreground="#444",
        ).pack(anchor=tk.W, padx=10, pady=(0, 4))

        # ── BOM preview table ─────────────────────────────────────────────────
        preview_frame = ttk.LabelFrame(self, text="Parts preview")
        preview_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))

        cols   = ("item", "part_no", "name", "qty", "process", "material")
        heads  = ("#",    "Part No",  "Name", "Qty", "Process",  "Material")
        widths = (30,      90,        200,     40,    130,         90)

        self.table = ttk.Treeview(preview_frame, columns=cols,
                                  show="headings", selectmode="browse")
        for col, head, w in zip(cols, heads, widths):
            anch = tk.E if col in ("item", "qty") else tk.W
            self.table.heading(col, text=head)
            self.table.column(col, width=w, stretch=(col == "name"), anchor=anch)

        sb = ttk.Scrollbar(preview_frame, orient=tk.VERTICAL,
                           command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.table.pack(fill=tk.BOTH, expand=True)

        # ── import warnings ───────────────────────────────────────────────────
        warn_frame = ttk.LabelFrame(self, text="Import warnings")
        warn_frame.pack(fill=tk.X, padx=8, pady=(0, 8))

        self._warn_text = tk.Text(
            warn_frame, height=4, state=tk.DISABLED,
            bg="#fffbe6", relief=tk.FLAT, font=("Courier", 8),
            wrap=tk.WORD,
        )
        self._warn_text.pack(fill=tk.X, padx=4, pady=4)

    def _subscribe(self) -> None:
        # Pick up assemblies pre-loaded from gui_main (demo mode)
        self.bus.subscribe("assembly_loaded", self._on_external_load)

    # ── button handlers ───────────────────────────────────────────────────────

    def _open_json(self) -> None:
        path = filedialog.askopenfilename(
            title="Open JSON BOM",
            filetypes=[("JSON BOM", "*.json"), ("All files", "*.*")],
        )
        if path:
            self._load_path(path)

    def _open_step(self) -> None:
        path = filedialog.askopenfilename(
            title="Open STEP File",
            filetypes=[("STEP", "*.step *.stp"), ("All files", "*.*")],
        )
        if path:
            self._load_path(path)

    def _load_path(self, path: str) -> None:
        try:
            from assembly_graph.importers import load_assembly
            result = load_assembly(path)
        except Exception as exc:
            messagebox.showerror("Load Error", str(exc))
            return

        self._populate(result.assembly, result.warnings)
        # Don't call bus.publish("assembly_loaded") here; let _populate do it
        # so _on_external_load (below) doesn't double-populate.
        self._assembly = result.assembly
        self.bus.publish("assembly_loaded", result.assembly)

    def _load_demo(self) -> None:
        try:
            from dfma_demo import build_pneumatic_valve, build_fasteners
            assembly  = build_pneumatic_valve()
            fasteners = build_fasteners()
        except Exception as exc:
            messagebox.showerror("Demo Error", str(exc))
            return

        self._populate(assembly, [])
        self._assembly = assembly
        self.bus.publish("assembly_loaded",  assembly)
        self.bus.publish("fasteners_loaded", fasteners)

        try:
            from dfma.rules.resource_calculator import calculate_resources
            self.bus.publish("resources_updated", calculate_resources(fasteners))
        except Exception:
            pass

    def _run_all(self) -> None:
        if self._assembly is None:
            messagebox.showinfo("No assembly", "Load an assembly first.")
            return
        self.bus.publish("run_dfma",     None)
        self.bus.publish("gen_sequence", "Optimized (Simulated Annealing)")

    # ── event handlers ─────────────────────────────────────────────────────────

    def _on_external_load(self, assembly) -> None:
        """Receive assemblies published by gui_main (pre-loaded demo)."""
        if self._assembly is None and hasattr(assembly, "all_parts"):
            self._assembly = assembly
            self._populate(assembly, [])

    # ── helpers ────────────────────────────────────────────────────────────────

    def _populate(self, assembly, warnings: list[str]) -> None:
        parts = assembly.all_parts()
        nsub  = len(assembly.subassemblies)
        self._info_var.set(
            f"Assembly: {assembly.name}   •   "
            f"{len(parts)} part(s)   •   "
            f"{nsub} sub-assembly group(s)"
        )

        self.table.delete(*self.table.get_children())
        for i, p in enumerate(parts, start=1):
            self.table.insert("", "end", values=(
                i, p.id, p.name, p.quantity,
                p.process.value, p.material.value,
            ))

        self._warn_text.configure(state=tk.NORMAL)
        self._warn_text.delete("1.0", tk.END)
        self._warn_text.insert("1.0",
                               "\n".join(warnings) if warnings else "(none)")
        self._warn_text.configure(state=tk.DISABLED)
