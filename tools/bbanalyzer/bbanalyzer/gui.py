"""Desktop window for bbanalyzer (tkinter, no extra dependencies).

Pick logs, fill in what you know about the aircraft, press Analyse. Results appear in tabs:
headline numbers, the colour-coded checklist, the main plots, and an A/B comparison. The full
HTML report is still written next to each log.
"""

from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import traceback
import webbrowser
from pathlib import Path

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .app import _load_settings, _save_settings
from .config import Aircraft

ROW_COLORS = {"bad": "#f6d3d0", "check": "#fbe7c2", "good": "#dcf1e5"}
BADGE = {"bad": "✖ bad", "check": "▲ check", "good": "✔ good"}
FILETYPES = [("Flight logs (Betaflight, iNav, ArduPilot)",
              "*.TXT *.txt *.BBL *.bbl *.BFL *.bfl *.BIN *.bin *.LOG *.log *.CSV *.csv"),
             ("All files", "*.*")]


def _num(text: str, typ=float):
    text = text.strip().replace(",", ".")
    if not text:
        return None
    return typ(text)


class App:
    def __init__(self, root: tk.Tk, paths: list[str]):
        self.root = root
        self.q: queue.Queue = queue.Queue()
        self.results: dict[str, dict] = {}
        self.order: list[str] = []
        self.busy = False
        self.images: dict[str, tk.PhotoImage] = {}
        self.settings = _load_settings()

        root.title("bbanalyzer – blackbox log analyser")
        root.geometry("1280x820")
        root.minsize(1000, 640)
        self._style()
        self._build()
        for p in paths:
            self._add_path(p)
        root.after(100, self._poll)

    # ------------------------------------------------------------------ layout
    def _style(self):
        st = ttk.Style()
        if "vista" in st.theme_names():
            st.theme_use("vista")
        elif "clam" in st.theme_names():
            st.theme_use("clam")
        st.configure("Head.TLabel", font=("Segoe UI", 13, "bold"))
        st.configure("Sub.TLabel", foreground="#5b6475")
        st.configure("CardKey.TLabel", foreground="#5b6475", font=("Segoe UI", 9))
        st.configure("CardVal.TLabel", font=("Segoe UI", 15, "bold"))
        st.configure("Accent.TButton", font=("Segoe UI", 10, "bold"))
        st.configure("Treeview", rowheight=24)

    def _build(self):
        pane = ttk.Panedwindow(self.root, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=8, pady=8)
        left = ttk.Frame(pane, width=330)
        right = ttk.Frame(pane)
        pane.add(left, weight=0)
        pane.add(right, weight=1)

        # logs
        lf = ttk.LabelFrame(left, text="Logs")
        lf.pack(fill="both", expand=True)
        box = ttk.Frame(lf)
        box.pack(fill="both", expand=True, padx=6, pady=6)
        self.listbox = tk.Listbox(box, selectmode="extended", height=10, activestyle="none", exportselection=False)
        sb = ttk.Scrollbar(box, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=sb.set)
        self.listbox.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.listbox.bind("<<ListboxSelect>>", lambda e: self._show_selected())
        btns = ttk.Frame(lf)
        btns.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(btns, text="Add logs…", command=self._add_dialog).pack(side="left")
        ttk.Button(btns, text="Remove", command=self._remove).pack(side="left", padx=4)
        ttk.Button(btns, text="Clear", command=self._clear).pack(side="left")

        # aircraft
        af = ttk.LabelFrame(left, text="Aircraft (leave blank if unknown)")
        af.pack(fill="x", pady=8)
        self.vars = {}
        fields = [("mass_kg", "All-up weight [kg]"), ("prop_diameter_in", "Prop diameter [inch]"),
                  ("capacity_mah", "Battery capacity [mAh]"), ("blades", "Blades per prop"),
                  ("motor_range", "Motor range LO,HI (CSV only)"), ("log_index", "Log number in file")]
        for r, (key, label) in enumerate(fields):
            ttk.Label(af, text=label).grid(row=r, column=0, sticky="w", padx=6, pady=2)
            v = tk.StringVar(value="" if self.settings.get(key) in (None, "") else str(self.settings.get(key)))
            self.vars[key] = v
            ttk.Entry(af, textvariable=v, width=12).grid(row=r, column=1, sticky="ew", padx=6, pady=2)
        af.columnconfigure(1, weight=1)
        ttk.Label(af, text="Weight + prop enable drag and the power model;\ncapacity enables C-rate. Log number blank = largest.",
                  style="Sub.TLabel").grid(row=len(fields), column=0, columnspan=2, sticky="w", padx=6, pady=(4, 6))

        # actions
        act = ttk.Frame(left)
        act.pack(fill="x")
        self.open_browser = tk.BooleanVar(value=bool(self.settings.get("open_browser", False)))
        ttk.Checkbutton(act, text="Open HTML report after analysis", variable=self.open_browser).pack(anchor="w")
        row = ttk.Frame(act)
        row.pack(fill="x", pady=6)
        self.analyse_btn = ttk.Button(row, text="Analyse", style="Accent.TButton", command=self._analyse)
        self.analyse_btn.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Open report", command=self._open_report).pack(side="left", padx=4)
        ttk.Button(row, text="Open folder", command=self._open_folder).pack(side="left")
        self.progress = ttk.Progressbar(act, mode="indeterminate")
        self.progress.pack(fill="x")
        self.status = tk.StringVar(value="Add a log to start.")
        ttk.Label(act, textvariable=self.status, style="Sub.TLabel", wraplength=310).pack(anchor="w", pady=4)

        # tabs
        self.nb = ttk.Notebook(right)
        self.nb.pack(fill="both", expand=True)
        self._build_summary()
        self._build_checklist()
        self._build_plots()
        self._build_compare()
        self._build_messages()

    def _build_summary(self):
        f = ttk.Frame(self.nb, padding=12)
        self.nb.add(f, text="Summary")
        self.sum_title = ttk.Label(f, text="No log analysed yet", style="Head.TLabel")
        self.sum_title.pack(anchor="w")
        self.sum_sub = ttk.Label(f, text="", style="Sub.TLabel")
        self.sum_sub.pack(anchor="w", pady=(0, 10))
        self.cards = ttk.Frame(f)
        self.cards.pack(fill="x")
        ttk.Label(f, text="Needs attention", style="Head.TLabel").pack(anchor="w", pady=(16, 4))
        self.attn = tk.Text(f, height=14, wrap="word", relief="flat", background=self.root.cget("background"),
                            font=("Segoe UI", 10))
        self.attn.pack(fill="both", expand=True)
        for k, c in ROW_COLORS.items():
            self.attn.tag_configure(k, background=c)
        self.attn.tag_configure("bold", font=("Segoe UI", 10, "bold"))
        self.attn.configure(state="disabled")

    def _build_checklist(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Checklist")
        cols = ("status", "area", "axis", "value", "note")
        self.tree = ttk.Treeview(f, columns=cols, show="headings")
        widths = {"status": 80, "area": 170, "axis": 90, "value": 120, "note": 600}
        for c in cols:
            self.tree.heading(c, text=c.capitalize())
            self.tree.column(c, width=widths[c], stretch=(c == "note"))
        for k, c in ROW_COLORS.items():
            self.tree.tag_configure(k, background=c)
        sb = ttk.Scrollbar(f, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="top", fill="both", expand=True)
        sb.place(relx=1.0, rely=0, relheight=0.82, anchor="ne")
        self.note = tk.StringVar(value="Select a row to read the full explanation.")
        ttk.Label(f, textvariable=self.note, wraplength=900, style="Sub.TLabel").pack(anchor="w", pady=6)
        self.tree.bind("<<TreeviewSelect>>", self._tree_note)

    def _build_plots(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Plots")
        top = ttk.Frame(f)
        top.pack(fill="x")
        ttk.Label(top, text="Plot:").pack(side="left")
        self.plot_choice = tk.StringVar()
        self.plot_combo = ttk.Combobox(top, textvariable=self.plot_choice, state="readonly", width=34)
        self.plot_combo.pack(side="left", padx=6)
        self.plot_combo.bind("<<ComboboxSelected>>", lambda e: self._draw_plot())
        ttk.Button(top, text="◀", width=3, command=lambda: self._step_plot(-1)).pack(side="left")
        ttk.Button(top, text="▶", width=3, command=lambda: self._step_plot(1)).pack(side="left", padx=4)
        holder = ttk.Frame(f)
        holder.pack(fill="both", expand=True, pady=6)
        self.canvas = tk.Canvas(holder, background="white", highlightthickness=0)
        ys = ttk.Scrollbar(holder, orient="vertical", command=self.canvas.yview)
        xs = ttk.Scrollbar(holder, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.plot_keys: list[str] = []

    def _build_compare(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Compare")
        top = ttk.Frame(f)
        top.pack(fill="x")
        self.cmp_a, self.cmp_b = tk.StringVar(), tk.StringVar()
        for label, var in (("Flight A", self.cmp_a), ("Flight B", self.cmp_b)):
            ttk.Label(top, text=label).pack(side="left", padx=(0, 4))
            cb = ttk.Combobox(top, textvariable=var, state="readonly", width=32)
            cb.pack(side="left", padx=(0, 10))
            setattr(self, f"cmp_combo_{label[-1].lower()}", cb)
        ttk.Button(top, text="Compare", command=self._compare).pack(side="left")
        ttk.Button(top, text="Load JSON…", command=self._load_json).pack(side="left", padx=6)
        ttk.Label(f, text="Analyse two flights (e.g. before/after a prop or tune change) and compare the key numbers.",
                  style="Sub.TLabel").pack(anchor="w", pady=6)
        cols = ("metric", "a", "b", "change")
        self.cmp_tree = ttk.Treeview(f, columns=cols, show="headings")
        for c, w in zip(cols, (260, 160, 160, 120)):
            self.cmp_tree.heading(c, text={"a": "Flight A", "b": "Flight B"}.get(c, c.capitalize()))
            self.cmp_tree.column(c, width=w, stretch=(c == "metric"))
        self.cmp_tree.pack(fill="both", expand=True)
        self.extra_json: dict[str, str] = {}

    def _build_messages(self):
        f = ttk.Frame(self.nb, padding=8)
        self.nb.add(f, text="Messages")
        self.msg = tk.Text(f, wrap="word", font=("Consolas", 9))
        self.msg.pack(fill="both", expand=True)

    # ------------------------------------------------------------------ logs list
    def _label(self, path: str) -> str:
        r = self.results.get(path)
        mark = "   " if r is None else ("✔ " if "res" in r else "✖ ")
        return f"{mark}{Path(path).name}"

    def _refresh_list(self):
        sel = self._selected_paths()
        self.listbox.delete(0, "end")
        for p in self.order:
            self.listbox.insert("end", self._label(p))
        for i, p in enumerate(self.order):
            if p in sel:
                self.listbox.selection_set(i)
        done = [p for p in self.order if "res" in self.results.get(p, {})]
        names = [Path(p).name for p in done] + list(self.extra_json)
        for cb in (self.cmp_combo_a, self.cmp_combo_b):
            cb.configure(values=names)

    def _add_path(self, p: str):
        p = str(Path(p.strip().strip('"')).resolve())
        if Path(p).is_file() and p not in self.order:
            self.order.append(p)
        self._refresh_list()
        if self.order:
            self.status.set(f"{len(self.order)} log(s) ready. Press Analyse.")

    def _add_dialog(self):
        for p in filedialog.askopenfilenames(title="Choose blackbox logs", filetypes=FILETYPES):
            self._add_path(p)

    def _selected_paths(self) -> list[str]:
        return [self.order[i] for i in self.listbox.curselection() if i < len(self.order)]

    def _remove(self):
        for p in self._selected_paths():
            self.order.remove(p)
            self.results.pop(p, None)
        self._refresh_list()

    def _clear(self):
        self.order.clear()
        self.results.clear()
        self._refresh_list()

    # ------------------------------------------------------------------ analysis
    def _aircraft(self):
        try:
            s = {"mass_kg": _num(self.vars["mass_kg"].get()),
                 "prop_diameter_in": _num(self.vars["prop_diameter_in"].get()),
                 "capacity_mah": _num(self.vars["capacity_mah"].get()),
                 "blades": _num(self.vars["blades"].get(), int) or 2,
                 "motor_range": self.vars["motor_range"].get().strip(),
                 "log_index": _num(self.vars["log_index"].get(), int)}
        except ValueError:
            messagebox.showerror("bbanalyzer", "Aircraft fields must be numbers (or blank).")
            return None
        mr = None
        if s["motor_range"]:
            try:
                lo, hi = (float(x) for x in s["motor_range"].split(",")[:2])
                mr = (lo, hi)
            except ValueError:
                messagebox.showerror("bbanalyzer", "Motor range must look like 1080,2000")
                return None
        self.settings.update(s)
        self.settings["open_browser"] = bool(self.open_browser.get())
        _save_settings(self.settings)
        ac = Aircraft(mass_kg=s["mass_kg"], prop_diameter_in=s["prop_diameter_in"],
                      capacity_mah=s["capacity_mah"], blades=s["blades"])
        return ac, s["log_index"], mr

    def _analyse(self):
        if self.busy:
            return
        files = self._selected_paths() or list(self.order)
        if not files:
            messagebox.showinfo("bbanalyzer", "Add a blackbox log first.")
            return
        cfg = self._aircraft()
        if cfg is None:
            return
        self.busy = True
        self.analyse_btn.configure(state="disabled")
        self.progress.start(12)
        threading.Thread(target=self._worker, args=(files, *cfg), daemon=True).start()

    def _worker(self, files, ac, log_index, motor_range):
        from . import analysis, report

        for f in files:
            name = Path(f).name
            self.q.put(("status", f"Analysing {name} …"))
            try:
                log, res, phases = analysis.run(f, ac, log_index, motor_range)
                figs = report.figures(log, res, phases, ac)
                out = Path(f).with_name(f"{Path(f).stem}_report.html")
                out.write_text(report.build(log, res, phases, ac, figs), encoding="utf-8")
                js = out.with_suffix(".json")
                js.write_text(analysis.to_json(res), encoding="utf-8")
                self.q.put(("done", f, {"res": res, "report": out, "json": js, "figs": figs,
                                         "headline": report.headline(res), "firmware": log.firmware,
                                         "duration": log.duration}))
            except Exception:
                self.q.put(("error", f, traceback.format_exc()))
        self.q.put(("finished", None, None))

    def _poll(self):
        try:
            while True:
                kind, path, payload = (*self.q.get_nowait(), None, None)[:3]
                if kind == "status":
                    self.status.set(path)
                    self._message(path)
                elif kind == "done":
                    self.results[path] = payload
                    self._message(f"Report written: {payload['report']}")
                    self._refresh_list()
                    self._select(path)
                    if self.open_browser.get():
                        webbrowser.open(Path(payload["report"]).resolve().as_uri())
                elif kind == "error":
                    self.results[path] = {"error": payload}
                    self._message(f"FAILED: {Path(path).name}\n{payload}")
                    self._refresh_list()
                    self.nb.select(4)
                elif kind == "finished":
                    self.busy = False
                    self.progress.stop()
                    self.analyse_btn.configure(state="normal")
                    ok = sum(1 for r in self.results.values() if "res" in r)
                    self.status.set(f"Done. {ok} log(s) analysed.")
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    # ------------------------------------------------------------------ display
    def _message(self, text: str):
        self.msg.insert("end", text + "\n")
        self.msg.see("end")

    def _select(self, path: str):
        if path in self.order:
            i = self.order.index(path)
            self.listbox.selection_clear(0, "end")
            self.listbox.selection_set(i)
            self.listbox.see(i)
        self._show(path)
        self.nb.select(0)

    def _show_selected(self):
        sel = self._selected_paths()
        if sel:
            self._show(sel[0])

    def _show(self, path: str):
        r = self.results.get(path)
        for w in self.cards.winfo_children():
            w.destroy()
        self.tree.delete(*self.tree.get_children())
        if not r or "res" not in r:
            self.sum_title.configure(text=Path(path).name)
            self.sum_sub.configure(text="Not analysed yet." if not r else "Analysis failed – see Messages.")
            self._set_attention([])
            return
        res = r["res"]
        self.sum_title.configure(text=Path(path).name)
        self.sum_sub.configure(text=f"{r['firmware']} · {r['duration']:.0f} s analysed · report: {r['report']}")
        for i, (k, v) in enumerate(r["headline"]):
            card = ttk.Frame(self.cards, padding=(10, 6), relief="groove")
            card.grid(row=i // 4, column=i % 4, sticky="nsew", padx=4, pady=4)
            ttk.Label(card, text=k, style="CardKey.TLabel").pack(anchor="w")
            ttk.Label(card, text=v, style="CardVal.TLabel").pack(anchor="w")
        for c in range(4):
            self.cards.columnconfigure(c, weight=1)
        rows = sorted(res.get("scorecard", []), key=lambda x: {"bad": 0, "check": 1, "good": 2}[x["status"]])
        for x in rows:
            self.tree.insert("", "end", values=(BADGE[x["status"]], x["area"], x["axis"], x["value"], x["note"]),
                             tags=(x["status"],))
        self._set_attention([x for x in rows if x["status"] != "good"])
        self.plot_keys = list(r["figs"])
        from .report import FIGURES

        self.plot_combo.configure(values=[FIGURES[k][0] for k in self.plot_keys])
        if self.plot_keys:
            self.plot_combo.current(0)
        self.current = path
        self._draw_plot()

    def _set_attention(self, rows):
        t = self.attn
        t.configure(state="normal")
        t.delete("1.0", "end")
        if not rows:
            t.insert("end", "Nothing flagged." if self.results else "")
        for x in rows:
            t.insert("end", f" {BADGE[x['status']]}  ", (x["status"], "bold"))
            t.insert("end", f" {x['area']} ({x['axis']}): {x['value']}\n", ("bold",))
            t.insert("end", f"    {x['note']}\n\n")
        t.configure(state="disabled")

    def _tree_note(self, _e=None):
        sel = self.tree.selection()
        if sel:
            v = self.tree.item(sel[0], "values")
            self.note.set(f"{v[1]} – {v[2]} – {v[3]}:  {v[4]}")

    def _draw_plot(self):
        self.canvas.delete("all")
        r = self.results.get(getattr(self, "current", ""), {})
        if not self.plot_keys or "figs" not in r:
            return
        idx = max(0, self.plot_combo.current())
        key = self.plot_keys[idx]
        img = self.images.get((self.current, key))
        if img is None:
            img = tk.PhotoImage(data=r["figs"][key])
            self.images[(self.current, key)] = img
        self.canvas.create_image(0, 0, anchor="nw", image=img)
        self.canvas.configure(scrollregion=(0, 0, img.width(), img.height()))

    def _step_plot(self, d: int):
        if self.plot_keys:
            self.plot_combo.current((max(0, self.plot_combo.current()) + d) % len(self.plot_keys))
            self._draw_plot()

    # ------------------------------------------------------------------ compare
    def _json_for(self, name: str) -> str | None:
        if name in self.extra_json:
            return self.extra_json[name]
        for p, r in self.results.items():
            if Path(p).name == name and "json" in r:
                return str(r["json"])
        return None

    def _load_json(self):
        for p in filedialog.askopenfilenames(title="Choose metrics JSON", filetypes=[("Metrics", "*.json")]):
            self.extra_json[Path(p).name] = p
        self._refresh_list()

    def _compare(self):
        from . import compare

        a, b = self._json_for(self.cmp_a.get()), self._json_for(self.cmp_b.get())
        if not a or not b:
            messagebox.showinfo("bbanalyzer", "Choose two analysed flights (or load their JSON files).")
            return
        self.cmp_tree.delete(*self.cmp_tree.get_children())
        for label, x, y, ch in compare.rows(compare.load(a), compare.load(b)):
            fmt = lambda v: f"{v:.4g}" if isinstance(v, (int, float)) else "–"
            self.cmp_tree.insert("", "end", values=(label, fmt(x), fmt(y), ch))

    # ------------------------------------------------------------------ files
    def _current_report(self) -> Path | None:
        sel = self._selected_paths()
        p = sel[0] if sel else getattr(self, "current", None)
        r = self.results.get(p or "", {})
        return Path(r["report"]) if "report" in r else None

    def _open_report(self):
        rep = self._current_report()
        if rep and rep.exists():
            webbrowser.open(rep.resolve().as_uri())
        else:
            messagebox.showinfo("bbanalyzer", "Analyse the selected log first.")

    def _open_folder(self):
        rep = self._current_report()
        sel = self._selected_paths()
        folder = rep.parent if rep else (Path(sel[0]).parent if sel else None)
        if not folder:
            return
        if sys.platform.startswith("win"):
            os.startfile(folder)  # noqa: S606 - opening Explorer on a local folder
        else:
            subprocess.Popen(["xdg-open", str(folder)])


def main(paths: list[str] | None = None) -> int:
    if sys.platform.startswith("win"):
        try:  # crisp text on high-DPI screens
            import ctypes

            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    root = tk.Tk()
    App(root, paths or [])
    root.mainloop()
    return 0
