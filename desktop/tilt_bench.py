"""
tilt_bench — live webcam tool to find the camera tilt angle that breaks the
monocular depth model, by eye AND by number.

Runs the SAME DepthNav (tracker/depth_nav.py) the rest of this app uses —
MiDaS Small / Depth Anything v2 via OpenCV DNN, CPU-only — then reproduces
the onboard C++ pipeline's HORIZON-BAND openness histogram (the exact
mechanism `camUpMaxDeg_` / `camDownMaxDeg_` in onboard/include/perception.hpp
gate on: see depth_nav.cpp's "middle 60% band" VFH+ crop). This is the tool
recommended in the tilt/ToF design discussion — bench-measure the real
threshold on real hardware instead of trusting the current reasoned-guess
defaults (12 deg up / 40 deg down).

The failure mode to watch for is NOT visible noise. A depth model pointed at
featureless sky/ceiling tends to output a smooth, CONFIDENT "everything is
far" reading — which reads as a falsely-open corridor, not an obviously bad
one. That's why this tool doesn't just show you the depth map: it computes
the same per-column openness the real navigation logic consumes and flags
when that signal goes suspiciously flat, which is the actual danger sign.

Usage:
    python tilt_bench.py
    python tilt_bench.py --depth-model ~/depth_models/midas_small.onnx
    python tilt_bench.py --depth-backend dav2 --band 0.25

Controls:
    s        save a labelled snapshot (frame + heatmap + metrics) to
             tilt_bench_out/ -- you'll be prompted in the console for a short
             note (e.g. "level", "+15 up", "-20 down"); use a phone
             inclinometer / spirit level against the housing for real numbers
    [ / ]    narrow / widen the analysed horizon band (mirrors tuning the
             onboard elevation-window logic itself)
    q / ESC  quit

What to do: mount/hold the camera roughly level, then slowly tilt it upward
in a few steps toward the ceiling/sky, then back through level and downward
toward the floor. Watch the flag in the top-left. Note the angle (however you
measure it) where it FIRST goes from USABLE to SUSPECT in each direction --
those are your real camUpMaxDeg_ / camDownMaxDeg_ values, not the guesses
currently in the code.
"""

import argparse
import json
import os
import time

import cv2
import numpy as np

from tracker.depth_nav import DepthNav, draw_depth_overlay

OUT_DIR = "tilt_bench_out"


def resolve_depth_model(path: str) -> str:
    """Same search order as main.py: exact path -> DEPTH_MODELS env ->
    ~/depth_models/ -> ./models/ -- kept in sync deliberately so a model
    already fetched for the tracker app works here with no extra setup."""
    if path and os.path.isfile(path):
        return path
    basename = os.path.basename(path) if path else "midas_small.onnx"
    search = [
        os.path.join(os.environ.get("DEPTH_MODELS", ""), basename),
        os.path.join(os.path.expanduser("~"), "depth_models", basename),
        os.path.join("models", basename),
    ]
    for candidate in search:
        if candidate and os.path.isfile(candidate):
            print(f"[depth] resolved model: {candidate}")
            return candidate
    print(f"[depth] model not found: {path!r}")
    print(f"[depth] searched: {[s for s in search if s]}")
    print("[depth] download it once to ~/depth_models/ to keep it across updates:")
    print("  curl -L -o ~/depth_models/midas_small.onnx "
          "https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx")
    return ""


def horizon_band_openness(depth: np.ndarray, band_frac: float):
    """Mirrors onboard/src/depth_nav.cpp's VFH+ horizon-band crop: average
    clearance over the middle (1 - 2*band_frac) vertical band per column, so
    floor/ceiling clutter doesn't blanket every column as blocked. Returns
    (open_col, r0, r1, mean_open, spread, frac_far).
    depth: (H,W) float32 in [0,1], 0=near/blocked, 1=far/open.
    """
    h, w = depth.shape
    r0, r1 = int(h * band_frac), int(h * (1.0 - band_frac))
    r1 = max(r1, r0 + 1)
    band = depth[r0:r1, :]
    open_col = band.mean(axis=0)
    mean_open = float(open_col.mean())
    spread = float(open_col.std())          # low = flat/uninformative
    frac_far = float((open_col > 0.85).mean())   # fraction reading "confidently far"
    return open_col, r0, r1, mean_open, spread, frac_far


def usability_flag(spread: float, frac_far: float):
    """Heuristic starting point, NOT ground truth -- sanity-check it against
    the heatmap itself (does it look like real room structure, or a flat
    wash of one colour) while calibrating. See module docstring for why a
    confidently-flat reading, not visible noise, is the real danger sign."""
    if frac_far > 0.85 and spread < 0.08:
        return "SUSPECT", (60, 60, 230), "looks blown out / featureless (sky-like)"
    if spread < 0.04:
        return "SUSPECT", (60, 60, 230), "flat -- no real per-column structure"
    if spread < 0.10 or frac_far > 0.6:
        return "MARGINAL", (0, 200, 255), "borderline -- eyeball the heatmap"
    return "USABLE", (128, 255, 0), "real structure across the band"


def draw_bar_strip(frame, open_col, x0, y0, w, h):
    """A live per-column openness bar graph under the video -- the same
    signal the corridor scan actually consumes, so structure (or its
    absence) is visible at a glance without reading the heatmap colour."""
    n = len(open_col)
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (40, 40, 44), -1)
    for i, v in enumerate(open_col):
        bx = x0 + int(i / n * w)
        bw = max(1, int(w / n) + 1)
        bh = int(np.clip(v, 0.0, 1.0) * h)
        g = int(np.clip(2.0 * v, 0.0, 1.0) * 255)
        r = int(np.clip(2.0 * (1.0 - v), 0.0, 1.0) * 255)
        cv2.rectangle(frame, (bx, y0 + h - bh), (bx + bw, y0 + h), (0, g, r), -1)
    cv2.rectangle(frame, (x0, y0), (x0 + w, y0 + h), (90, 90, 96), 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--depth-model", default="",
                     help="path to ONNX depth model (MiDaS Small or DAv2 Small)")
    ap.add_argument("--depth-backend", choices=["midas", "dav2"], default="midas",
                     help="depth model type: midas (default) or dav2")
    ap.add_argument("--band", type=float, default=0.2,
                     help="horizon-band crop fraction per edge (default 0.2 == "
                          "the onboard code's fixed 'middle 60%%' band)")
    ap.add_argument("--cam", type=int, default=0, help="webcam index (default 0)")
    args = ap.parse_args()

    model_path = resolve_depth_model(args.depth_model)
    if not model_path:
        return 1

    depth_nav = DepthNav()
    if not depth_nav.init(model_path, args.depth_backend):
        return 1

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f"Error: cannot open webcam at index {args.cam}")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    os.makedirs(OUT_DIR, exist_ok=True)
    band = args.band
    win = "tilt_bench"
    cv2.namedWindow(win)
    print(__doc__)
    print(f"[tilt_bench] band={band:.2f}  saving snapshots to ./{OUT_DIR}/")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Error: webcam read failed")
            break

        depth_nav.update(frame)
        out = frame.copy()
        snap = depth_nav.snapshot()
        draw_depth_overlay(out, snap)

        depth = depth_nav.depth
        flag_text, flag_col, flag_reason = "NO DATA", (150, 150, 150), ""
        open_col = None
        if depth is not None and depth.size:
            open_col, r0, r1, mean_open, spread, frac_far = horizon_band_openness(depth, band)
            flag_text, flag_col, flag_reason = usability_flag(spread, frac_far)

            fh, fw = out.shape[:2]
            # Horizon-band boundaries -- exactly what the onboard elevation
            # window analyses, drawn on the live feed so you can SEE which
            # part of the room the corridor scan is looking at as you tilt.
            y0, y1 = int(r0 / depth.shape[0] * fh), int(r1 / depth.shape[0] * fh)
            cv2.line(out, (0, y0), (fw, y0), (255, 200, 0), 1, cv2.LINE_AA)
            cv2.line(out, (0, y1), (fw, y1), (255, 200, 0), 1, cv2.LINE_AA)
            cv2.line(out, (0, fh // 2), (fw, fh // 2), (255, 255, 255), 1, cv2.LINE_AA)  # level ref

            draw_bar_strip(out, open_col, 8, fh - 70, fw - 16, 46)

            cv2.rectangle(out, (0, 0), (330, 76), (20, 20, 24), -1)
            cv2.putText(out, flag_text, (10, 26), cv2.FONT_HERSHEY_DUPLEX, 0.8, flag_col, 2)
            cv2.putText(out, flag_reason, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (210, 210, 210), 1, cv2.LINE_AA)
            cv2.putText(out, f"mean {mean_open:.2f}  spread {spread:.2f}  far% {frac_far*100:.0f}",
                        (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 210), 1, cv2.LINE_AA)

        cv2.putText(out, "s: save+label   [ ]: band   q: quit",
                    (8, out.shape[0] - 78), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (160, 170, 180), 1, cv2.LINE_AA)

        cv2.imshow(win, out)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('['):
            band = min(0.45, band + 0.02)
            print(f"[tilt_bench] band={band:.2f}")
        elif k == ord(']'):
            band = max(0.02, band - 0.02)
            print(f"[tilt_bench] band={band:.2f}")
        elif k == ord('s') and open_col is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            note = input(f"[{ts}] angle/note for this snapshot (e.g. 'level', '+15up'): ").strip()
            base = os.path.join(OUT_DIR, f"{ts}_{note or 'unlabeled'}".replace(" ", "_"))
            cv2.imwrite(base + "_frame.png", frame)
            cv2.imwrite(base + "_overlay.png", out)
            with open(base + "_metrics.json", "w") as f:
                json.dump({
                    "note": note, "band": band, "mean_open": mean_open,
                    "spread": spread, "frac_far": frac_far, "flag": flag_text,
                    "backend": args.depth_backend,
                }, f, indent=2)
            print(f"[tilt_bench] saved {base}_*")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
