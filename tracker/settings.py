import threading
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass


@dataclass
class Settings:
    # Tracking thresholds
    sim_confirm:    float = 0.55
    sim_warning:    float = 0.42
    streak_limit:   int   = 8
    search_radius:  float = 0.6

    # Performance — how often DINOv2 runs (CSRT runs every frame)
    check_interval: int   = 6    # verify identity every N frames
    attn_interval:  int   = 10   # recompute attention mask every N frames

    # Attention mask display
    show_mask:      bool  = True
    attn_threshold: float = 0.50
    mask_opacity:   float = 0.40

    # Display toggles
    show_confidence_bar: bool = True
    show_fps:            bool = True


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

    # ── Tracking ─────────────────────────────────────────────────────────────
    f = section("Tracking")
    slider_row(f, "Lock threshold",    "sim_confirm",    0.30, 0.90)
    slider_row(f, "Warning threshold", "sim_warning",    0.20, 0.80)
    slider_row(f, "Search radius",     "search_radius",  0.20, 3.00)
    slider_row(f, "Streak limit",      "streak_limit",   2,    30,  as_int=True)

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

    ttk.Separator(root).pack(fill="x", pady=8)
    ttk.Label(root, text="R  reset lock    ESC  quit",
              foreground="gray").pack(pady=(0, 8))

    root.mainloop()


def launch_settings(settings: Settings) -> threading.Thread:
    t = threading.Thread(target=_run_gui, args=(settings,), daemon=True)
    t.start()
    return t
