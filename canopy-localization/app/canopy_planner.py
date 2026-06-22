"""Canopy-localization dataset planner — desktop app (Tkinter).

Pick a training area on a map of Finland (or type ETRS-TM35FIN / EPSG:3067
coordinates), and see the estimated number of horizon 'pictures' (curves),
storage, generation time, and training time. Optionally calibrate the throughput
on your machine, or generate the dataset from a DSM GeoTIFF.

Run:  python3 app/canopy_planner.py     (Tkinter ships with Python on Windows)
"""
from __future__ import annotations
import os
import sys
import json
import time
import queue
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from app.estimator import (estimate, PRESETS, TRAIN_DEVICES,
                           fmt_count, fmt_dur, fmt_size)

# Finland extent in EPSG:3067 (metres) and a schematic outline for orientation.
FIN = dict(emin=80000, emax=740000, nmin=6610000, nmax=7790000)
FIN_OUTLINE = [  # approximate / schematic only
    (387000, 6672000), (500000, 6710000), (560000, 6720000), (660000, 6900000),
    (730000, 6980000), (700000, 7160000), (620000, 7360000), (560000, 7560000),
    (533000, 7779000), (470000, 7710000), (255000, 7670000), (360000, 7340000),
    (320000, 7160000), (210000, 7000000), (205000, 6840000), (245000, 6710000),
    (300000, 6650000),
]


class Planner(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Canopy-Localization Dataset Planner")
        self.geometry("980x620")
        self._syncing = False
        self.cal_pps = None                      # calibrated positions/s/core
        self.q = queue.Queue()

        self.cw, self.ch = 340, 580
        left = ttk.Frame(self); left.pack(side="left", padx=8, pady=8)
        self.canvas = tk.Canvas(left, width=self.cw, height=self.ch,
                                bg="#eef3f7", highlightthickness=1,
                                highlightbackground="#999")
        self.canvas.pack()
        ttk.Label(left, text="Drag on the map to select the training area",
                  foreground="#555").pack(pady=4)
        self.canvas.bind("<ButtonPress-1>", self._press)
        self.canvas.bind("<B1-Motion>", self._drag)

        self._build_controls()
        self._draw_map()
        # default AOI: a 50x50 km square in central Finland
        self.set_aoi(360000, 410000, 6950000, 7000000)
        self.after(120, self._poll)

    # ---------- coordinate transforms ----------
    def en2c(self, e, n):
        x = (e - FIN["emin"]) / (FIN["emax"] - FIN["emin"]) * self.cw
        y = self.ch - (n - FIN["nmin"]) / (FIN["nmax"] - FIN["nmin"]) * self.ch
        return x, y

    def c2en(self, x, y):
        e = FIN["emin"] + x / self.cw * (FIN["emax"] - FIN["emin"])
        n = FIN["nmin"] + (self.ch - y) / self.ch * (FIN["nmax"] - FIN["nmin"])
        return e, n

    # ---------- UI ----------
    def _build_controls(self):
        r = ttk.Frame(self); r.pack(side="left", fill="both", expand=True, padx=10, pady=8)

        box = ttk.LabelFrame(r, text="Area of interest (EPSG:3067, metres)")
        box.pack(fill="x", pady=4)
        self.v = {k: tk.StringVar() for k in ("emin", "emax", "nmin", "nmax")}
        grid = [("E min", "emin", 0, 0), ("E max", "emax", 0, 2),
                ("N min", "nmin", 1, 0), ("N max", "nmax", 1, 2)]
        for label, key, rr, cc in grid:
            ttk.Label(box, text=label).grid(row=rr, column=cc, sticky="e", padx=3, pady=2)
            ent = ttk.Entry(box, textvariable=self.v[key], width=12)
            ent.grid(row=rr, column=cc + 1, padx=3, pady=2)
            self.v[key].trace_add("write", lambda *_: self._entries_changed())

        par = ttk.LabelFrame(r, text="Parameters")
        par.pack(fill="x", pady=4)
        self.preset = tk.StringVar(value="Fast (dataset build)")
        self.spacing = tk.StringVar(value="50")
        self.nalt = tk.StringVar(value="4")
        self.cores = tk.StringVar(value="8")
        self.device = tk.StringVar(value="GPU")
        self.epochs = tk.StringVar(value="15")
        rows = [("Render preset", ttk.Combobox(par, textvariable=self.preset,
                 values=list(PRESETS), state="readonly", width=20)),
                ("Grid spacing (m)", ttk.Entry(par, textvariable=self.spacing, width=10)),
                ("Altitude bins", ttk.Entry(par, textvariable=self.nalt, width=10)),
                ("CPU cores (gen)", ttk.Entry(par, textvariable=self.cores, width=10)),
                ("Train device", ttk.Combobox(par, textvariable=self.device,
                 values=list(TRAIN_DEVICES), state="readonly", width=18)),
                ("Train epochs", ttk.Entry(par, textvariable=self.epochs, width=10))]
        for i, (lab, w) in enumerate(rows):
            ttk.Label(par, text=lab).grid(row=i, column=0, sticky="e", padx=4, pady=2)
            w.grid(row=i, column=1, sticky="w", padx=4, pady=2)
        for var in (self.preset, self.spacing, self.nalt, self.cores,
                    self.device, self.epochs):
            var.trace_add("write", lambda *_: self.recompute())

        out = ttk.LabelFrame(r, text="Estimate")
        out.pack(fill="x", pady=6)
        self.out = {}
        for key, lab in (("area", "Area"), ("curves", "Pictures (curves)"),
                         ("positions", "Ray-march positions"),
                         ("storage", "Dataset storage"),
                         ("gen", "Generation time"),
                         ("train", "Training time (approx)")):
            row = ttk.Frame(out); row.pack(fill="x", pady=1)
            ttk.Label(row, text=lab + ":", width=20, anchor="e").pack(side="left")
            val = ttk.Label(row, text="-", font=("TkDefaultFont", 10, "bold"))
            val.pack(side="left", padx=6)
            self.out[key] = val
        self.note = ttk.Label(out, text="", foreground="#777", wraplength=400)
        self.note.pack(fill="x", pady=2)

        btns = ttk.Frame(r); btns.pack(fill="x", pady=6)
        ttk.Button(btns, text="Calibrate on this PC", command=self._calibrate).pack(side="left", padx=3)
        ttk.Button(btns, text="Generate dataset…", command=self._generate).pack(side="left", padx=3)
        ttk.Button(btns, text="Save plan…", command=self._save_plan).pack(side="left", padx=3)
        self.status = ttk.Label(r, text="", foreground="#06c")
        self.status.pack(fill="x")
        self.prog = ttk.Progressbar(r, mode="determinate")
        self.prog.pack(fill="x", pady=2)

    def _draw_map(self):
        self.canvas.delete("base")
        pts = []
        for e, n in FIN_OUTLINE:
            pts += list(self.en2c(e, n))
        self.canvas.create_polygon(pts, fill="#dbe7d4", outline="#5a8",
                                   width=2, tags="base")
        self.canvas.create_text(self.cw - 6, 12, text="Finland (schematic)",
                                anchor="e", fill="#789", tags="base")

    # ---------- AOI handling ----------
    def set_aoi(self, emin, emax, nmin, nmax):
        self._syncing = True
        self.v["emin"].set(f"{emin:.0f}"); self.v["emax"].set(f"{emax:.0f}")
        self.v["nmin"].set(f"{nmin:.0f}"); self.v["nmax"].set(f"{nmax:.0f}")
        self._syncing = False
        self._draw_aoi(); self.recompute()

    def get_aoi(self):
        try:
            e0, e1 = float(self.v["emin"].get()), float(self.v["emax"].get())
            n0, n1 = float(self.v["nmin"].get()), float(self.v["nmax"].get())
            return min(e0, e1), max(e0, e1), min(n0, n1), max(n0, n1)
        except ValueError:
            return None

    def _entries_changed(self):
        if not self._syncing:
            self._draw_aoi(); self.recompute()

    def _draw_aoi(self):
        self.canvas.delete("aoi")
        a = self.get_aoi()
        if not a:
            return
        x0, y0 = self.en2c(a[0], a[3]); x1, y1 = self.en2c(a[1], a[2])
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#c00", width=2,
                                     fill="#c00", stipple="gray25", tags="aoi")

    def _press(self, ev):
        self._x0, self._y0 = ev.x, ev.y

    def _drag(self, ev):
        e0, n0 = self.c2en(self._x0, self._y0)
        e1, n1 = self.c2en(ev.x, ev.y)
        self.set_aoi(min(e0, e1), max(e0, e1), min(n0, n1), max(n0, n1))

    # ---------- estimate ----------
    def recompute(self):
        a = self.get_aoi()
        if not a:
            return
        try:
            sp = float(self.spacing.get()); nalt = int(float(self.nalt.get()))
            cores = int(float(self.cores.get())); ep = float(self.epochs.get())
        except ValueError:
            return
        km2 = (a[1] - a[0]) * (a[3] - a[2]) / 1e6
        e = estimate(km2, sp, nalt, cores, self.preset.get(), ep,
                     TRAIN_DEVICES.get(self.device.get(), 40000.0),
                     gen_pos_per_s_core=self.cal_pps)
        self.out["area"].config(text=f"{km2:,.0f} km²  "
                                f"({(a[1]-a[0])/1000:.0f}×{(a[3]-a[2])/1000:.0f} km)")
        self.out["curves"].config(text=fmt_count(e["curves"]))
        self.out["positions"].config(text=fmt_count(e["positions"]))
        self.out["storage"].config(text=fmt_size(e["storage_gb"]))
        self.out["gen"].config(text=f"{fmt_dur(e['gen_seconds'])}  @ {cores} cores")
        self.out["train"].config(text=fmt_dur(e["train_seconds"]))
        src = ("measured on this PC" if self.cal_pps else "reference CPU estimate")
        self.note.config(text=f"Throughput: {src}. One ray-march makes all "
                              f"{nalt} altitude curves; gen time scales with positions.")

    # ---------- calibrate / generate (threaded) ----------
    def _calibrate(self):
        self.status.config(text="Calibrating (synthetic ray-casts)…")
        threading.Thread(target=self._calibrate_worker, daemon=True).start()

    def _calibrate_worker(self):
        try:
            import numpy as np
            from horizon.raycaster import HorizonRaycaster
            from horizon.synthetic_dsm import make_synthetic_dsm
            p = PRESETS[self.preset.get()]
            dsm, meta = make_synthetic_dsm(1200, 2.0, 0)
            rc = HorizonRaycaster(dsm, meta["res_m"])
            az = np.linspace(0, 2 * np.pi, p["n_az"], endpoint=False)
            zs = [float(np.nanmedian(dsm)) + a for a in (5, 10, 15, 20)]
            N = 120
            xs = np.random.default_rng(0).uniform(400, 2000, (N, 2))
            t0 = time.time()
            for x, y in xs:
                rc.raycast(x, y, zs, az, max_range_m=p["max_range_m"], dr_m=p["dr_m"])
            pps = N / (time.time() - t0)
            self.q.put(("cal", pps))
        except Exception as ex:                       # noqa: BLE001
            self.q.put(("err", f"Calibration failed: {ex}"))

    def _generate(self):
        a = self.get_aoi()
        if not a:
            return
        path = filedialog.askopenfilename(
            title="DSM GeoTIFF (Cancel = synthetic test DSM)",
            filetypes=[("GeoTIFF", "*.tif *.tiff"), ("All", "*.*")])
        out = filedialog.asksaveasfilename(
            title="Save curves to…", defaultextension=".npz",
            filetypes=[("NumPy npz", "*.npz")])
        if not out:
            return
        self.prog["value"] = 0
        self.status.config(text="Generating…")
        threading.Thread(target=self._generate_worker,
                         args=(a, path, out), daemon=True).start()

    def _generate_worker(self, aoi, tif, out):
        try:
            import numpy as np
            from horizon.raycaster import HorizonRaycaster
            from horizon import dataset
            sp = float(self.spacing.get()); nalt = int(float(self.nalt.get()))
            p = PRESETS[self.preset.get()]
            if tif:
                import rasterio
                from rasterio.windows import from_bounds
                with rasterio.open(tif) as ds:
                    win = from_bounds(aoi[0], aoi[2], aoi[1], aoi[3], ds.transform)
                    arr = ds.read(1, window=win).astype("float32")
                    res = float(abs(ds.transform.a)); nod = ds.nodata
                rc = HorizonRaycaster(arr, res, nodata=nod)
                bounds = (0, arr.shape[1] * res, 0, arr.shape[0] * res)
            else:                                     # synthetic, sized to AOI
                w = max(int((aoi[1] - aoi[0]) / 2), 300)
                h = max(int((aoi[3] - aoi[2]) / 2), 300)
                from horizon.synthetic_dsm import make_synthetic_dsm
                arr, meta = make_synthetic_dsm(min(max(w, h), 2000), 2.0, 0)
                rc = HorizonRaycaster(arr, meta["res_m"])
                bounds = (0, arr.shape[1] * meta["res_m"], 0, arr.shape[0] * meta["res_m"])
            zs = [float(np.nanmedian(arr[np.isfinite(arr)])) + a for a in
                  (5, 10, 15, 20)[:nalt]]
            self.q.put(("status", "Ray-casting curves…"))
            curves, coords, st = dataset.generate_grid(
                rc, bounds, sp, zs, n_az=p["n_az"], max_range_m=p["max_range_m"])
            np.savez_compressed(out, curves=curves, coords=coords)
            self.q.put(("done", (out, st)))
        except Exception as ex:                       # noqa: BLE001
            self.q.put(("err", f"Generation failed: {ex}"))

    def _poll(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "cal":
                    self.cal_pps = payload
                    self.status.config(text=f"Calibrated: {payload:.0f} positions/s/core")
                    self.recompute()
                elif kind == "status":
                    self.status.config(text=payload)
                elif kind == "done":
                    out, st = payload
                    self.prog["value"] = 100
                    self.status.config(text=f"Saved {st['n_curves']:,} curves → "
                                       f"{os.path.basename(out)} "
                                       f"({st['pos_per_s']:.0f} pos/s)")
                    self.cal_pps = st["pos_per_s"]; self.recompute()
                elif kind == "err":
                    self.status.config(text=payload)
                    messagebox.showerror("Error", payload)
        except queue.Empty:
            pass
        self.after(120, self._poll)

    def _save_plan(self):
        a = self.get_aoi()
        if not a:
            return
        f = filedialog.asksaveasfilename(defaultextension=".json",
                                         filetypes=[("JSON", "*.json")])
        if not f:
            return
        km2 = (a[1] - a[0]) * (a[3] - a[2]) / 1e6
        e = estimate(km2, float(self.spacing.get()), int(float(self.nalt.get())),
                     int(float(self.cores.get())), self.preset.get(),
                     float(self.epochs.get()),
                     TRAIN_DEVICES.get(self.device.get(), 40000.0),
                     gen_pos_per_s_core=self.cal_pps)
        plan = dict(aoi_epsg3067=dict(emin=a[0], emax=a[1], nmin=a[2], nmax=a[3]),
                    spacing_m=float(self.spacing.get()),
                    altitude_bins=int(float(self.nalt.get())),
                    preset=self.preset.get(), estimate=e)
        with open(f, "w") as fh:
            json.dump(plan, fh, indent=2)
        self.status.config(text=f"Plan saved → {os.path.basename(f)}")


def main():
    Planner().mainloop()


if __name__ == "__main__":
    main()
