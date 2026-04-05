"""
Assembly Grapher GUI — simplified 4-tab application window.

Tabs
────
  1. Load Parts   — open JSON BOM / STEP file, preview BOM, trigger analysis
  2. DFMA         — BOM table with DFA/DFM status + warning list + detail pane
  3. Sequencing   — optimised assembly step list + tool resources side panel
  4. Summary      — metric cards (assembly | DFMA | sequencing) + full issues list

All panels communicate via an EventBus (publish/subscribe); the App class
owns the business logic handlers (run_dfma, run_screw, gen_sequence, export).
"""

import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from .panels.load_tab       import LoadTab
from .panels.dfma_tab       import DfmaTab
from .panels.graph_tab      import GraphTab
from .panels.sequencing_tab import SequencingTab
from .panels.summary_tab    import SummaryTab
from .event_bus             import EventBus
from . import theme


class App(tk.Tk):
    """Root window — wires all panels together via EventBus."""

    def __init__(self) -> None:
        super().__init__()
        self.title("Assembly Grapher — DFMA & Sequence Planner")
        self.geometry("1280x800")
        self.minsize(960, 600)

        # Apply dark theme before building any widgets
        theme.apply(self)

        self.bus = EventBus()
        self._assembly      = None
        self._liaison       = None   # LiaisonMatrix from the last import
        self._import_result = None   # full ImportResult (has .shapes for 3D viewer)
        self._fasteners: list = []
        self._plan      = None
        self._result    = None   # most recent AnalysisResult

        self._build_menu()
        self._build_layout()
        self._build_status_bar()
        self._setup_handlers()

    # ── menu ──────────────────────────────────────────────────────────────────

    def _build_menu(self) -> None:
        menubar = tk.Menu(self)

        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Open JSON BOM…",  command=self._menu_open_json)
        file_menu.add_command(label="Open STEP File…", command=self._menu_open_step)
        file_menu.add_command(label="Load Demo Assembly", command=self._menu_load_demo)
        file_menu.add_separator()
        file_menu.add_command(label="Export Report…",  command=self._export_report)
        file_menu.add_separator()
        file_menu.add_command(label="Exit",            command=self.destroy)
        menubar.add_cascade(label="File", menu=file_menu)

        analysis_menu = tk.Menu(menubar, tearoff=0)
        analysis_menu.add_command(label="Run DFMA Analysis",
                                  command=lambda: self.bus.publish("run_dfma", None))
        analysis_menu.add_command(label="Run Screw Check",
                                  command=lambda: self.bus.publish("run_screw", None))
        analysis_menu.add_command(label="Generate Sequence",
                                  command=lambda: self.bus.publish(
                                      "gen_sequence", "Optimized (Simulated Annealing)"))
        analysis_menu.add_separator()
        analysis_menu.add_command(label="Run All",     command=self._run_all)
        analysis_menu.add_separator()
        analysis_menu.add_command(label="View 3D…",   command=self._open_3d_viewer)
        menubar.add_cascade(label="Analysis", menu=analysis_menu)

        self.config(menu=menubar)

    # ── layout ────────────────────────────────────────────────────────────────

    def _build_layout(self) -> None:
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.load_tab    = LoadTab(self.notebook, self.bus)
        self.dfma_tab    = DfmaTab(self.notebook, self.bus)
        self.graph_tab   = GraphTab(self.notebook, self.bus)
        self.seq_tab     = SequencingTab(self.notebook, self.bus)
        self.summary_tab = SummaryTab(self.notebook, self.bus)

        self.notebook.add(self.load_tab,    text="  Load Parts  ")
        self.notebook.add(self.dfma_tab,    text="  DFMA  ")
        self.notebook.add(self.graph_tab,   text="  Graph  ")
        self.notebook.add(self.seq_tab,     text="  Sequencing  ")
        self.notebook.add(self.summary_tab, text="  Summary  ")

    # ── status bar ────────────────────────────────────────────────────────────

    def _build_status_bar(self) -> None:
        bar = ttk.Frame(self, style="Status.TFrame")
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        self._status_var = tk.StringVar(value="Ready — open an assembly to begin.")
        ttk.Label(bar, textvariable=self._status_var, anchor=tk.W,
                  style="Status.TLabel").pack(side=tk.LEFT)

    def _set_status(self, msg: str) -> None:
        self._status_var.set(msg)

    # ── event wiring ──────────────────────────────────────────────────────────

    def _setup_handlers(self) -> None:
        self.bus.subscribe("assembly_loaded",  self._handle_assembly_loaded)
        self.bus.subscribe("fasteners_loaded", self._handle_fasteners_loaded)
        self.bus.subscribe("run_dfma",         self._handle_run_dfma)
        self.bus.subscribe("run_screw",        self._handle_run_screw)
        self.bus.subscribe("gen_sequence",     self._handle_gen_sequence)
        self.bus.subscribe("run_resources",    self._handle_run_resources)
        self.bus.subscribe("export_report",    lambda _: self._export_report())
        self.bus.subscribe("open_3d_viewer",   lambda _: self._open_3d_viewer())
        self.bus.subscribe("export_bom_csv",   lambda _: self._export_bom_csv())

    # ── business logic handlers ───────────────────────────────────────────────

    def _handle_assembly_loaded(self, payload) -> None:
        # Accepts either a bare Assembly or an ImportResult (has .assembly + .liaison)
        if hasattr(payload, "assembly"):
            self._assembly      = payload.assembly
            self._liaison       = payload.liaison
            self._import_result = payload          # keep full result for 3D viewer
        elif hasattr(payload, "all_parts"):
            self._assembly      = payload
            self._liaison       = None
            self._import_result = None

    def _handle_fasteners_loaded(self, fasteners) -> None:
        if isinstance(fasteners, list):
            self._fasteners = fasteners
            self._publish_resources(fasteners)

    def _handle_run_dfma(self, _data) -> None:
        if self._assembly is None:
            self._set_status("No assembly loaded — open a file first.")
            return
        if getattr(self, "_dfma_running", False):
            return
        self._dfma_running = True
        self._set_status("Running DFMA analysis…  (please wait)")
        self.update_idletasks()

        assembly  = self._assembly
        fasteners = self._fasteners or None

        def _run() -> None:
            try:
                from dfma.analyzer import analyze
                result = analyze(assembly, fasteners=fasteners)

                def _done() -> None:
                    self._result = result
                    self.bus.publish("warnings_updated", result)
                    e = len(result.errors())
                    w = len(result.warnings_only())
                    i = len(result.infos())
                    self._set_status(
                        f"DFMA complete — {e} error(s), {w} warning(s), {i} info(s)  |  "
                        f"DFA Index: {result.dfa_index:.1%}"
                    )
                    self._dfma_running = False
                    # Chain: if _run_all requested, now fire sequencing
                    if getattr(self, "_chain_sequence_after_dfma", False):
                        self._chain_sequence_after_dfma = False
                        self.bus.publish("gen_sequence",
                                         "Optimized (Simulated Annealing)")

                self.after(0, _done)
            except Exception as exc:
                self.after(0, lambda: self._set_status(f"DFMA error: {exc}"))
                self.after(0, lambda: setattr(self, "_dfma_running", False))

        import threading
        threading.Thread(target=_run, daemon=True).start()

    def _handle_run_screw(self, _data) -> None:
        if not self._fasteners:
            self._set_status("No fasteners loaded — use File › Open JSON BOM first.")
            return
        try:
            from dfma.rules.screw_rules import check_fasteners
            issues = check_fasteners(self._fasteners)
            self._set_status(f"Screw check complete — {len(issues)} issue(s).")
        except Exception as exc:
            self._set_status(f"Screw check error: {exc}")

    def _handle_gen_sequence(self, algo: str) -> None:
        if self._assembly is None:
            self._set_status("No assembly loaded — cannot generate sequence.")
            return
        if getattr(self, "_seq_running", False):
            return   # already running

        self._seq_running = True
        self._set_status("Generating assembly sequence…  (please wait)")
        self.update_idletasks()

        assembly = self._assembly   # capture for thread
        liaison  = self._liaison    # may be None for demo assemblies

        def _run() -> None:
            try:
                from assembly_graph import AssemblyPlanner
                planner = AssemblyPlanner.from_assembly(
                    assembly,
                    base_part_id=assembly.all_parts()[0].id,
                )
                # Use the liaison matrix from the import (STEP proximity inference
                # or JSON contacts) rather than the empty one from_assembly creates.
                if liaison is not None:
                    planner.liaison = liaison

                sa_iter = 0 if isinstance(algo, str) and (
                    "greedy" in algo.lower() or "kahn" in algo.lower()
                ) else 2000

                plan = planner.plan(sa_iterations=sa_iter)
                seq  = (plan.greedy_sequence if isinstance(algo, str)
                        and "greedy" in algo.lower() else plan.optimized_sequence)

                payload = {
                    "steps":        plan.optimized_steps(),
                    "cost_summary": seq.summary(),
                }
                status = (
                    f"Sequence generated — {len(plan.optimized_steps())} parts  |  "
                    f"{len(plan.subassemblies)} subassembly group(s)  |  "
                    f"cost = {plan.optimized_sequence.total_cost:.2f}"
                )

                def _done() -> None:
                    self._plan = plan
                    self.bus.publish("plan_ready",     plan)
                    self.bus.publish("sequence_ready", payload)
                    self._set_status(status)
                    self._seq_running = False

                self.after(0, _done)

            except Exception as exc:
                self.after(0, lambda: self._set_status(f"Sequence error: {exc}"))
                self.after(0, lambda: setattr(self, "_seq_running", False))

        threading.Thread(target=_run, daemon=True).start()

    def _open_3d_viewer(self) -> None:
        if self._import_result is None:
            self._set_status("Load a STEP file first — JSON BOM has no 3D geometry.")
            return
        from .panels.viewer_3d import open_viewer
        open_viewer(self._import_result, dfma_result=self._result, title="Assembly 3D View")

    def _handle_run_resources(self, _data) -> None:
        self._publish_resources(self._fasteners)

    def _publish_resources(self, fasteners: list) -> None:
        if not fasteners:
            return
        try:
            from dfma.rules.resource_calculator import calculate_resources
            self.bus.publish("resources_updated", calculate_resources(fasteners))
        except Exception as exc:
            self._set_status(f"Resource calc error: {exc}")

    # ── menu commands ─────────────────────────────────────────────────────────

    def _menu_open_json(self) -> None:
        path = filedialog.askopenfilename(
            title="Open JSON BOM",
            filetypes=[("JSON BOM", "*.json"), ("All files", "*.*")],
        )
        if path:
            self.notebook.select(0)   # jump to Load tab
            self.load_tab._load_path(path)

    def _menu_open_step(self) -> None:
        path = filedialog.askopenfilename(
            title="Open STEP File",
            filetypes=[("STEP", "*.step *.stp"), ("All files", "*.*")],
        )
        if path:
            self.notebook.select(0)
            self.load_tab._load_path(path)

    def _menu_load_demo(self) -> None:
        self.notebook.select(0)
        self.load_tab._load_demo()

    def _run_all(self) -> None:
        """Run DFMA first, then generate sequence once DFMA completes."""
        if self._assembly is None:
            self._set_status("No assembly loaded — open a file first.")
            return
        # Chain: kick off DFMA and set a flag so the DFMA _done callback
        # automatically fires gen_sequence afterwards.
        self._chain_sequence_after_dfma = True
        self.bus.publish("run_dfma", None)

    def _export_report(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Export DFMA Report",
            defaultextension=".txt",
            filetypes=[
                ("Text Report", "*.txt"),
                ("JSON",        "*.json"),
                ("All files",   "*.*"),
            ],
        )
        if not path:
            return
        try:
            if path.lower().endswith(".json"):
                import json as _json
                data: dict = {}
                if self._result:
                    data["dfma"] = {
                        "dfa_index":           self._result.dfa_index,
                        "total_assembly_time": self._result.total_assembly_time_s,
                        "total_parts":         self._result.total_parts,
                        "theoretical_minimum": self._result.theoretical_minimum,
                        "warnings": [
                            {"rule": w.rule_id, "part_id": w.part_id,
                             "severity": w.severity.value, "message": w.message}
                            for w in self._result.warnings
                        ],
                    }
                if self._plan:
                    data["sequence"] = {
                        "steps": self._plan.optimized_steps(),
                        "cost":  self._plan.optimized_sequence.total_cost,
                    }
                with open(path, "w", encoding="utf-8") as f:
                    _json.dump(data, f, indent=2)
            else:
                lines: list[str] = []
                if self._result:
                    lines.append(self._result.summary())
                if self._plan:
                    lines.append("")
                    lines.append(self._plan.summary())
                if not lines:
                    lines = ["No analysis results to export."]
                with open(path, "w", encoding="utf-8") as f:
                    f.write("\n".join(lines))
            self._set_status(f"Report exported to: {path}")
        except Exception as exc:
            messagebox.showerror("Export Error", str(exc))

    def _export_bom_csv(self) -> None:
        if self._assembly is None:
            self._set_status("No assembly loaded — nothing to export.")
            return
        path = filedialog.asksaveasfilename(
            title="Export BOM as CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            import csv
            parts = self._assembly.all_parts()
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "ID", "Name", "Qty", "Material", "Process",
                    "Length_mm", "Width_mm", "Height_mm", "Mass_g",
                ])
                for p in parts:
                    g = p.geometry
                    writer.writerow([
                        p.id, p.name, getattr(p, "quantity", 1),
                        p.material.value, p.process.value,
                        g.length, g.width, g.height, g.mass_grams,
                    ])
            self._set_status(f"BOM exported to: {path}")
        except Exception as exc:
            messagebox.showerror("Export Error", str(exc))


def run() -> None:
    app = App()
    app.mainloop()


if __name__ == "__main__":
    run()
