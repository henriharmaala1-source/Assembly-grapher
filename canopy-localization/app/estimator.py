"""Dataset/training estimator for the canopy-localization planner.

Pure logic (no GUI) so it can be unit-tested headlessly. All throughput numbers
are calibrated defaults from a reference desktop CPU core (see bench.py); the GUI
'Calibrate' button remeasures them on the user's actual machine.
"""
from __future__ import annotations

# Reference per-core throughput (positions/s) and storage, measured by bench.py.
# One ray-march per position yields ALL altitude curves (the numerator trick),
# so generation time scales with POSITIONS, storage/count with positions x alts.
PRESETS = {
    "Fast (dataset build)":  dict(n_az=360, max_range_m=900,  dr_m=4,
                                  pos_per_s_core=132.0, bytes_per_curve=720),
    "Quality (validation)":  dict(n_az=720, max_range_m=1200, dr_m=2,
                                  pos_per_s_core=21.0,  bytes_per_curve=1440),
}

# Rough training throughput (curves/s) for a small 1-D skyline descriptor.
TRAIN_DEVICES = {"GPU": 40000.0, "CPU (multicore)": 2500.0}


def estimate(area_km2, spacing_m, n_alt, cores, preset,
             train_epochs=15, train_curves_per_s=40000.0,
             gen_pos_per_s_core=None):
    """Return a dict of dataset + time estimates for an AOI and parameters."""
    p = PRESETS[preset]
    pps_core = gen_pos_per_s_core if gen_pos_per_s_core else p["pos_per_s_core"]
    pos_per_s = max(pps_core, 1e-9) * max(cores, 1)

    positions = area_km2 * 1e6 / (spacing_m ** 2)
    curves = positions * n_alt
    storage_gb = curves * p["bytes_per_curve"] / 1e9
    gen_s = positions / pos_per_s
    train_s = (curves * train_epochs / train_curves_per_s
               if train_curves_per_s > 0 else 0.0)

    return dict(area_km2=area_km2, positions=positions, curves=curves,
                storage_gb=storage_gb, gen_seconds=gen_s, train_seconds=train_s,
                bytes_per_curve=p["bytes_per_curve"], pos_per_s=pos_per_s)


def fmt_count(n):
    for div, suf in ((1e9, "B"), (1e6, "M"), (1e3, "k")):
        if n >= div:
            return f"{n/div:.2f} {suf}"
    return f"{n:.0f}"


def fmt_dur(seconds):
    if seconds < 1:
        return "<1 s"
    if seconds < 90:
        return f"{seconds:.0f} s"
    if seconds < 5400:
        return f"{seconds/60:.1f} min"
    if seconds < 172800:
        return f"{seconds/3600:.1f} h"
    return f"{seconds/86400:.1f} days"


def fmt_size(gb):
    if gb < 1:
        return f"{gb*1000:.0f} MB"
    return f"{gb:.1f} GB"


if __name__ == "__main__":  # headless self-test
    print("Self-test of estimator (reference CPU throughput):\n")
    for name, (a, sp, alt, cores, preset) in {
        "10x10 km test patch": (100, 50, 4, 8, "Fast (dataset build)"),
        "50x50 km square":     (2500, 50, 4, 8, "Fast (dataset build)"),
        "100 km radius disk":  (31416, 50, 4, 8, "Fast (dataset build)"),
        "50x50 km (Quality)":  (2500, 50, 4, 8, "Quality (validation)"),
    }.items():
        e = estimate(a, sp, alt, cores, preset)
        print(f"  {name:24s}: {fmt_count(e['curves'])} curves, "
              f"{fmt_size(e['storage_gb'])}, "
              f"gen {fmt_dur(e['gen_seconds'])} @8cores, "
              f"train {fmt_dur(e['train_seconds'])}")
