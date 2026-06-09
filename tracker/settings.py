import threading
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass


@dataclass
class Settings:
    # Tracking engine — switched live (lock box carries over)
    tracking_engine: str = "hybrid"      # "hybrid" | "sam2"

    # Tracker backend (hybrid engine) — applied on next lock (re-draw to switch)
    tracker_backend: str = "CSRT"        # "ViT" | "CSRT" | "KCF" | "Optical Flow"

    # Tracking thresholds
    sim_confirm:    float = 0.55
    sim_warning:    float = 0.42
    streak_limit:   int   = 8
    search_radius:  float = 0.6

    # Performance — how often DINOv2 runs (backend tracker runs every frame)
    check_interval: int   = 6    # verify identity every N frames
    attn_interval:  int   = 10   # recompute attention mask every N frames

    # Motion vector
    show_motion_vector: bool = True
    predict_horizon:    int  = 8   # frames ahead the prediction arrow points

    # Attention mask display
    show_mask:      bool  = True
    attn_threshold: float = 0.50
    mask_opacity:   float = 0.40

    # Click-to-segment
    segment_backend:  str   = "SAM 2"   # "SAM 2" | "MobileSAM" | "FastSAM-s"
    seg_opacity:      float = 0.45
    seg_continuous:   bool  = True      # re-segment every frame
    seg_interval:     int   = 3         # re-segment every N frames (non-SAM2)

    # Motion detection (auto-acquire)
    motion_detect:       bool  = False   # run blob motion detection when idle
    motion_threshold:    float = 24.0    # MOG2 varThreshold — lower = more sensitive
    motion_min_area:     int   = 1500    # ignore blobs smaller than this (px)
    motion_auto_segment: bool  = True    # auto-hand the blob to the segmenter
    show_motion_mask:    bool  = False   # overlay the foreground mask

    # Display toggles
    show_confidence_bar: bool = True
    show_fps:            bool = True

    # Drone mode — lightweight click-to-track (no ML inference)
    drone_mode:     bool  = False
    drone_backend:  str   = "CSRT"    # "CSRT" | "KCF" | "Optical Flow"
    drone_box_size: int   = 80        # fixed box diameter (px)

    # Drone detection (Drone-vs-Bird / MAV-VID)
    drone_detect:          bool  = False
    drone_detect_preset:   str   = "Drone-vs-Bird"   # "Drone-vs-Bird" | "MAV-VID"
    drone_detect_conf:     float = 0.25
    drone_detect_interval: int   = 3


TRACKER_BACKENDS  = ["ViT", "CSRT", "KCF", "Optical Flow"]
TRACKING_ENGINES  = ["hybrid", "sam2"]
SEGMENT_BACKENDS  = ["SAM 2", "MobileSAM", "FastSAM-s"]
DRONE_BACKENDS    = ["CSRT", "KCF", "Optical Flow"]


# ----------------------------------------------------------------- GUI builder

def _run_gui(settings: Settings) -> None:
    root = tk.Tk()
    root.title("Tracker Settings")
    root.resizable(False, False)

    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass

    def section(title):
        f = ttk.LabelFrame(root, text=title, padding=(10, 6))
        f.pack(fill="x", padx=10, pady=(4, 0))
        return f

    def slider_row(parent, label, key, lo, hi, as_int=False):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=22, anchor="w").pack(side="left")
        val_lbl = ttk.Label(row, width=5, anchor="e")
        val_lbl.pack(side="right")

        var = tk.DoubleVar(value=getattr(settings, key))

        def on_slide(v=None):
            raw = var.get()
            coerced = int(round(raw)) if as_int else round(raw, 2)
            setattr(settings, key, coerced)
            val_lbl.config(text=str(coerced))

        ttk.Scale(row, from_=lo, to=hi, variable=var,
                  command=on_slide, length=170).pack(side="left", padx=6)
        on_slide()

    def check_row(parent, label, key):
        var = tk.BooleanVar(value=getattr(settings, key))
        ttk.Checkbutton(parent, text=label, variable=var,
                        command=lambda: setattr(settings, key, var.get())
                        ).pack(anchor="w", pady=2)

    def combo_row(parent, label, key, values):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=3)
        ttk.Label(row, text=label, width=22, anchor="w").pack(side="left")
        var = tk.StringVar(value=getattr(settings, key))
        cb = ttk.Combobox(row, textvariable=var, values=values,
                          state="readonly", width=14)
        cb.pack(side="left", padx=6)
        cb.bind("<<ComboboxSelected>>",
                lambda e: setattr(settings, key, var.get()))

    # ── Engine ────────────────────────────────────────────────────────────────
    f = section("Engine  (switches live; lock carries over)")
    combo_row(f, "Tracking engine", "tracking_engine", TRACKING_ENGINES)

    # ── Tracker ───────────────────────────────────────────────────────────────
    f = section("Tracker backend  (hybrid; re-draw box to apply)")
    combo_row(f, "Backend", "tracker_backend", TRACKER_BACKENDS)

    # ── Tracking ─────────────────────────────────────────────────────────────
    f = section("Tracking")
    slider_row(f, "Lock threshold",    "sim_confirm",    0.30, 0.90)
    slider_row(f, "Warning threshold", "sim_warning",    0.20, 0.80)
    slider_row(f, "Search radius",     "search_radius",  0.20, 3.00)
    slider_row(f, "Streak limit",      "streak_limit",   2,    30,  as_int=True)

    # ── Click-to-Segment ──────────────────────────────────────────────────────
    f = section("Click-to-Segment  (single click; drag to track)")
    combo_row(f, "Backend",      "segment_backend", SEGMENT_BACKENDS)
    check_row(f,  "Continuous update (follow object)", "seg_continuous")
    slider_row(f, "Re-segment every N frames", "seg_interval", 1, 15, as_int=True)
    slider_row(f, "Mask opacity",  "seg_opacity", 0.10, 0.80)

    # ── Motion Detection ──────────────────────────────────────────────────────
    f = section("Motion Detection  (auto-acquire when idle)")
    check_row(f,  "Enable motion detect", "motion_detect")
    slider_row(f, "Sensitivity (lower=more)", "motion_threshold", 8, 80)
    slider_row(f, "Min blob area",  "motion_min_area", 300, 20000, as_int=True)
    check_row(f,  "Auto-segment detected object", "motion_auto_segment")
    check_row(f,  "Show motion mask", "show_motion_mask")

    # ── Motion vector ─────────────────────────────────────────────────────────
    f = section("Motion Vector")
    check_row(f, "Show motion vector",  "show_motion_vector")
    slider_row(f, "Prediction horizon", "predict_horizon", 2, 30, as_int=True)

    # ── Performance ───────────────────────────────────────────────────────────
    f = section("Performance")
    slider_row(f, "DINOv2 check every N frames", "check_interval", 1, 20, as_int=True)
    slider_row(f, "Attn mask every N frames",    "attn_interval",  3, 30, as_int=True)

    # ── Attention mask ────────────────────────────────────────────────────────
    f = section("Attention Mask (DINOv2)")
    check_row(f, "Show mask overlay", "show_mask")
    slider_row(f, "Mask threshold",   "attn_threshold", 0.10, 0.90)
    slider_row(f, "Mask opacity",     "mask_opacity",   0.05, 0.80)

    # ── Display ───────────────────────────────────────────────────────────────
    f = section("Display")
    check_row(f, "Show confidence bar", "show_confidence_bar")
    check_row(f, "Show FPS",            "show_fps")

    # ── Drone Mode ────────────────────────────────────────────────────────────
    f = section("Drone Mode  (D key toggles — click=designate, no ML)")
    check_row(f,  "Enable drone mode",    "drone_mode")
    combo_row(f,  "Tracker backend",      "drone_backend", DRONE_BACKENDS)
    slider_row(f, "Box size (px)",        "drone_box_size", 30, 200, as_int=True)

    # ── Drone Detection ───────────────────────────────────────────────────────
    f = section("Drone Detection  (place .pt in models/ — see drone_detect.py)")
    check_row(f,  "Enable drone detection",  "drone_detect")
    combo_row(f,  "Model preset",            "drone_detect_preset",
              ["Drone-vs-Bird", "MAV-VID"])
    slider_row(f, "Confidence threshold",    "drone_detect_conf",     0.10, 0.90)
    slider_row(f, "Run every N frames",      "drone_detect_interval",    1,  15, as_int=True)

    ttk.Separator(root).pack(fill="x", pady=8)
    ttk.Label(root, text="R  reset    D  drone mode    ESC  quit",
              foreground="gray").pack(pady=(0, 8))

    root.mainloop()


def launch_settings(settings: Settings) -> threading.Thread:
    t = threading.Thread(target=_run_gui, args=(settings,), daemon=True)
    t.start()
    return t
