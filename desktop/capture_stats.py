#!/usr/bin/env python3
"""
capture_stats.py — characterise an analog-capture / webcam path (cadence,
jitter, dropped frames) with ZERO computer vision. All numbers come from
frame-arrival timestamps only; pixel content is never inspected for the stats.

This is the CV-free half of the Stage-2 capture-chain check
(onboard/docs/hardware-bringup-checklist.md): frame-rate stability, jitter,
and dropped frames. Absolute glass->frame latency is a separate test (it needs
a clock in the image or an LED/photodiode reference) and is NOT measured here.

Runs the same on Windows (bench, DirectShow) and on the Pi (V4L2). The Pi run
is the one that counts — the UVC stacks differ — but a Windows run is a fine
first look with the rig already on the desk.

Usage:
    pip install opencv-python
    python capture_stats.py                 # device 0, 60 s
    python capture_stats.py --device 1 --seconds 600   # 10 min soak (thermal)
    python capture_stats.py --width 720 --height 576    # request a resolution

Outputs (next to the script), all safe to send back:
    capture_stats_summary.txt   — the headline numbers
    capture_intervals.csv       — every inter-frame interval (ms)
    capture_sample_*.png        — 3 sample frames (wave a hand mid-run so one
                                  catches motion -> reveals interlacing combing)
"""

import argparse
import platform
import statistics
import sys
import time

import cv2


def open_capture(device: int, width: int, height: int):
    """Open the device with the best backend for the OS, best-effort props."""
    if platform.system() == "Windows":
        cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
    else:  # Linux / Raspberry Pi
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device)  # fall back to whatever backend works
    if not cap.isOpened():
        return None
    # Best-effort: minimise driver buffering so cadence reflects the device,
    # not OpenCV's queue. Not all backends honour these; that's fine.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def fourcc_str(cap) -> str:
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    if v <= 0:
        return "?"
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4)).strip("\x00 ")


def main() -> int:
    ap = argparse.ArgumentParser(description="CV-free capture cadence/jitter stats.")
    ap.add_argument("--device", type=int, default=0, help="camera index (default 0)")
    ap.add_argument("--seconds", type=float, default=60.0, help="run length (default 60)")
    ap.add_argument("--width", type=int, default=0, help="requested frame width")
    ap.add_argument("--height", type=int, default=0, help="requested frame height")
    ap.add_argument("--warmup", type=int, default=30,
                    help="frames to discard before timing (auto-exposure settle)")
    args = ap.parse_args()

    cap = open_capture(args.device, args.width, args.height)
    if cap is None:
        print(f"ERROR: could not open camera device {args.device}.", file=sys.stderr)
        print("Try a different --device index (0,1,2...).", file=sys.stderr)
        return 1

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    reported_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = fourcc_str(cap)
    backend = cap.getBackendName()

    print(f"Opened device {args.device}: {w}x{h}  fourcc={fourcc}  "
          f"reported_fps={reported_fps:.1f}  backend={backend}")
    print(f"Running {args.seconds:.0f}s. WAVE A HAND across the view partway "
          f"through so a sample frame catches motion (interlacing check).")
    print("Ctrl-C to stop early.\n")

    # Warm-up: first frames are slow (allocation, auto-exposure) and would
    # skew the cadence stats. Discard them before we start timing.
    for _ in range(args.warmup):
        cap.read()

    deltas_ms: list[float] = []
    sample_frames = []
    sample_targets = None  # set once we know how many frames to expect
    prev = None
    read_fail = 0
    start = time.perf_counter()

    try:
        while True:
            ok, frame = cap.read()
            t = time.perf_counter()
            if not ok:
                read_fail += 1
                if read_fail > 30:
                    print("Too many read failures — device dropped out?")
                    break
                continue
            read_fail = 0
            if prev is not None:
                deltas_ms.append((t - prev) * 1000.0)
            prev = t

            # Grab 3 sample frames spread across the run (for a human to eyeball
            # focus / colour / interlacing — not used in the numeric stats).
            elapsed = t - start
            if sample_targets is None and elapsed > 1.0:
                sample_targets = [args.seconds * f for f in (0.1, 0.5, 0.9)]
            if sample_targets and elapsed >= sample_targets[0]:
                sample_frames.append(frame.copy())
                sample_targets.pop(0)

            if elapsed >= args.seconds:
                break
            # Live heartbeat once a second so a long run isn't a silent stare.
            if len(deltas_ms) and len(deltas_ms) % 100 == 0:
                recent = deltas_ms[-100:]
                print(f"  t={elapsed:5.0f}s  frames={len(deltas_ms):6d}  "
                      f"recent_fps={1000.0/statistics.mean(recent):5.1f}", end="\r")
    except KeyboardInterrupt:
        print("\nStopped early by user.")

    cap.release()
    print()

    if len(deltas_ms) < 10:
        print("ERROR: captured almost no frames — device not delivering.",
              file=sys.stderr)
        return 1

    # --- statistics (robust: median is the baseline, spikes skew the mean) ---
    n = len(deltas_ms)
    total_s = sum(deltas_ms) / 1000.0
    mean = statistics.mean(deltas_ms)
    median = statistics.median(deltas_ms)
    stdev = statistics.pstdev(deltas_ms)
    lo, hi = min(deltas_ms), max(deltas_ms)
    ordered = sorted(deltas_ms)
    p95 = ordered[int(0.95 * (n - 1))]
    p99 = ordered[int(0.99 * (n - 1))]

    # A "hiccup" is an interval well over the median cadence; a likely dropped
    # frame is ~2x median or more (one frame's worth of gap went missing).
    hiccups = sum(1 for d in deltas_ms if d > 1.5 * median)
    drops = sum(1 for d in deltas_ms if d > 1.9 * median)

    lines = []
    lines.append("=== capture_stats summary ===")
    lines.append(f"device            : {args.device}  ({backend})")
    lines.append(f"resolution        : {w}x{h}   fourcc={fourcc}")
    lines.append(f"reported FPS      : {reported_fps:.2f}  (often wrong on dongles)")
    lines.append(f"run length        : {total_s:.1f} s, {n+1} frames")
    lines.append("")
    lines.append(f"MEASURED FPS      : {1000.0/mean:.2f}   (from mean interval)")
    lines.append(f"interval mean     : {mean:.2f} ms")
    lines.append(f"interval median   : {median:.2f} ms")
    lines.append(f"interval stdev    : {stdev:.2f} ms   <- jitter")
    lines.append(f"interval min/max  : {lo:.2f} / {hi:.2f} ms")
    lines.append(f"interval p95/p99  : {p95:.2f} / {p99:.2f} ms")
    lines.append("")
    lines.append(f"hiccups (>1.5x med): {hiccups}   ({100.0*hiccups/n:.2f}% of frames)")
    lines.append(f"likely drops(>1.9x): {drops}   ({100.0*drops/n:.2f}% of frames)")
    lines.append("")
    lines.append("READING IT:")
    lines.append("  GOOD  = max ~= mean, tiny stdev, ~0 drops, drop rate flat over time.")
    lines.append("  BAD   = max 3-5x mean, or drops climbing later in the run")
    lines.append("          (climbing late = thermal throttling of a hot dongle).")

    # Text histogram of intervals (no matplotlib dependency).
    lines.append("")
    lines.append("interval histogram (ms):")
    buckets = 12
    span = max(hi - lo, 1e-6)
    counts = [0] * buckets
    for d in deltas_ms:
        b = min(buckets - 1, int((d - lo) / span * buckets))
        counts[b] += 1
    peak = max(counts) or 1
    for i, c in enumerate(counts):
        edge = lo + span * i / buckets
        bar = "#" * int(40 * c / peak)
        lines.append(f"  {edge:7.1f} | {bar} {c}")

    summary = "\n".join(lines)
    print("\n" + summary)

    with open("capture_stats_summary.txt", "w") as f:
        f.write(summary + "\n")
    with open("capture_intervals.csv", "w") as f:
        f.write("index,interval_ms\n")
        for i, d in enumerate(deltas_ms):
            f.write(f"{i},{d:.3f}\n")
    for i, fr in enumerate(sample_frames):
        cv2.imwrite(f"capture_sample_{i}.png", fr)

    print("\nWrote: capture_stats_summary.txt, capture_intervals.csv, "
          f"capture_sample_*.png ({len(sample_frames)} frames)")
    print("Send those files back for review.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
