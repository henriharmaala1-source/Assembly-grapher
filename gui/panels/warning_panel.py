"""
WarningPanel — RIGHT panel.

Upper section: grouped warning list (ERROR / WARNING / INFO).
Lower section: detail pane showing full message, metric, threshold,
               and suggestion for the selected warning.

EventBus events consumed:
  "warnings_updated" → repopulate the list
  "part_selected"    → highlight warnings for that part
  "fastener_selected"→ highlight warnings for that fastener

EventBus events published:
  "part_selected"    → when user clicks a warning row (highlights graph node)
"""

import tkinter as tk
from tkinter import ttk

from ..event_bus import EventBus
from ..theme import C, style_text_widget


SEVERITY_ICON  = {"ERROR": "●", "WARNING": "▲", "INFO": "ℹ"}
SEVERITY_COLOR = {"ERROR": C.SEV_ERROR, "WARNING": C.SEV_WARNING, "INFO": C.SEV_INFO}


class WarningPanel(ttk.Frame):

    def __init__(self, parent, bus: EventBus) -> None:
        super().__init__(parent)
        self.bus = bus
        self._warnings: list = []
        self._build()
        self._subscribe()

    def _build(self) -> None:
        ttk.Label(self, text="DFMA Warnings", font=("", 9, "bold")).pack(
            anchor=tk.W, padx=6, pady=(6, 0))

        # ── filter bar ───────────────────────────────────────────────────────
        filter_frame = ttk.Frame(self)
        filter_frame.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(filter_frame, text="Filter:").pack(side=tk.LEFT)
        self._filter_var = tk.StringVar(value="All")
        ttk.Combobox(
            filter_frame, textvariable=self._filter_var, width=10, state="readonly",
            values=["All", "ERROR", "WARNING", "INFO", "DFA only", "DFM only", "DFS only"],
        ).pack(side=tk.LEFT, padx=4)
        self._filter_var.trace_add("write", lambda *_: self._repopulate())

        # ── warning tree ─────────────────────────────────────────────────────
        cols = ("severity", "rule", "part", "msg")
        self.wtree = ttk.Treeview(
            self, columns=cols, show="headings", selectmode="browse"
        )
        self.wtree.heading("severity", text="")
        self.wtree.heading("rule",     text="Rule")
        self.wtree.heading("part",     text="Part")
        self.wtree.heading("msg",      text="Message")
        self.wtree.column("severity", width=22,  stretch=False, anchor=tk.CENTER)
        self.wtree.column("rule",     width=65,  stretch=False)
        self.wtree.column("part",     width=60,  stretch=False)
        self.wtree.column("msg",      width=200, stretch=True)

        # severity text colours
        self.wtree.tag_configure("ERROR",   foreground=C.SEV_ERROR)
        self.wtree.tag_configure("WARNING", foreground=C.SEV_WARNING)
        self.wtree.tag_configure("INFO",    foreground=C.SEV_INFO)

        sb = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.wtree.yview)
        self.wtree.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.wtree.pack(fill=tk.BOTH, expand=True, padx=4)
        self.wtree.bind("<<TreeviewSelect>>", self._on_warning_select)

        # ── detail pane ───────────────────────────────────────────────────────
        ttk.Separator(self, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=4)
        ttk.Label(self, text="Detail", font=("", 9, "bold")).pack(
            anchor=tk.W, padx=6)

        detail_frame = ttk.Frame(self)
        detail_frame.pack(fill=tk.BOTH, expand=False, padx=4, pady=(0, 4))

        self._detail_text = tk.Text(
            detail_frame, height=8, wrap=tk.WORD, state=tk.DISABLED,
            font=("Courier", 8),
        )
        style_text_widget(self._detail_text)
        self._detail_text.pack(fill=tk.BOTH, expand=True)

        # ── export button ────────────────────────────────────────────────────
        ttk.Button(
            self, text="Export Report…",
            command=lambda: self.bus.publish("export_report", None),
        ).pack(fill=tk.X, padx=4, pady=4)

    def _subscribe(self) -> None:
        self.bus.subscribe("warnings_updated",  self._on_warnings_updated)
        self.bus.subscribe("part_selected",     self._on_part_selected)
        self.bus.subscribe("fastener_selected", self._on_part_selected)

    # ── event handlers ───────────────────────────────────────────────────────

    def _on_warnings_updated(self, result) -> None:
        self._warnings = result.warnings if result else []
        self._repopulate()

    def _on_part_selected(self, part_id: str) -> None:
        """Highlight all warnings for the selected part."""
        for iid in self.wtree.get_children():
            tags = self.wtree.item(iid, "tags")
            pid  = self.wtree.item(iid, "values")[2]  # part column
            if pid == part_id:
                self.wtree.selection_set(iid)
                self.wtree.see(iid)
                break

    # ── internal ──────────────────────────────────────────────────────────────

    def _repopulate(self) -> None:
        self.wtree.delete(*self.wtree.get_children())
        filt = self._filter_var.get()
        for i, w in enumerate(self._warnings):
            sev = w.severity.value
            rule = w.rule_id
            if filt != "All":
                if filt in ("ERROR", "WARNING", "INFO") and sev != filt:
                    continue
                if filt == "DFA only" and not rule.startswith("DFA"):
                    continue
                if filt == "DFM only" and not rule.startswith("DFM"):
                    continue
                if filt == "DFS only" and not rule.startswith("DFS"):
                    continue

            icon = SEVERITY_ICON.get(sev, "")
            # truncate message to 60 chars for the list
            short_msg = w.message[:60] + ("…" if len(w.message) > 60 else "")
            self.wtree.insert(
                "", "end", iid=str(i),
                values=(icon, rule, w.part_id, short_msg),
                tags=(sev,),
            )

    def _on_warning_select(self, _event) -> None:
        sel = self.wtree.focus()
        if not sel:
            return
        try:
            idx = int(sel)
            w   = self._warnings[idx]
        except (ValueError, IndexError):
            return

        # Publish part selected so graph highlights it
        self.bus.publish("part_selected", w.part_id)

        # Show detail
        lines = [
            f"Rule      : {w.rule_id}",
            f"Severity  : {w.severity.value}",
            f"Part      : {w.part_id}",
            "",
            f"Message:\n{w.message}",
        ]
        if w.metric_value is not None:
            lines += ["", f"Measured  : {w.metric_value:.4g}",
                      f"Threshold : {w.threshold:.4g}"]
        if w.suggestion:
            lines += ["", f"Suggestion:\n{w.suggestion}"]

        self._detail_text.configure(state=tk.NORMAL)
        self._detail_text.delete("1.0", tk.END)
        self._detail_text.insert("1.0", "\n".join(lines))
        self._detail_text.configure(state=tk.DISABLED)
