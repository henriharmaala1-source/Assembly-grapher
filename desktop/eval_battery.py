#!/usr/bin/env python3
"""
Run the whole realistic-footage battery and report a calibrated table.

Ties together `synth_footage.py` (clip generation) and `eval_tracker.py`
(the real-footage evaluation path), and adds the piece that makes the numbers
interpretable: per-clip target SNR.

SNR here = |mean(target) - mean(local background)| / std(local background).
It matters because tracker scores are meaningless without it — a 20% score on an
SNR-1.0 clip is physics, not a defect. Measured cliff: this tracker holds 79-100%
above SNR ~1.45 and collapses below ~1.40 on 1/f backgrounds.

  python3 eval_battery.py                  # generate (if needed) + evaluate all
  python3 eval_battery.py --cues L+C       # compare cue sets
  python3 eval_battery.py --clips mine/    # point at real footage instead
"""
import argparse
import csv
import os
import subprocess
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def target_snr(mp4, labels, step=5):
    """Mean target-vs-local-background SNR across the clip."""
    cap = cv2.VideoCapture(mp4)
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float32))
    cap.release()
    rows = list(csv.reader(open(labels)))[1:]
    out = []
    for i in range(0, min(len(frames), len(rows)), step):
        g = frames[i]
        cx, cy, sz = float(rows[i][1]), float(rows[i][2]), float(rows[i][3])
        r = max(3, int(sz / 2.6))
        x0, x1 = max(0, int(cx - r)), min(g.shape[1], int(cx + r))
        y0, y1 = max(0, int(cy - r)), min(g.shape[0], int(cy + r))
        if x1 <= x0 or y1 <= y0:
            continue
        R = 3 * r
        bg = g[max(0, int(cy - R)):int(cy + R), max(0, int(cx - R)):int(cx + R)]
        out.append(abs(g[y0:y1, x0:x1].mean() - bg.mean()) / (bg.std() + 1e-6))
    return float(np.mean(out)) if out else float("nan")


def run_eval(mp4, labels, cues):
    r = subprocess.run(
        [sys.executable, os.path.join(HERE, "eval_tracker.py"),
         "--video", mp4, "--labels", labels, "--cues", cues],
        capture_output=True, text=True)
    got = {}
    for line in r.stdout.splitlines():
        p = line.split()
        if "on-target" in line and ":" in line:
            got["on"] = p[2]; got["hold"] = p[4]; got["reacq"] = p[7]
            got["idsw"] = p[-1]
        if "centre err" in line:
            got["mean"] = p[4]; got["p90"] = p[6]
    return got


def main():
    ap = argparse.ArgumentParser(description="realistic-footage tracker battery")
    ap.add_argument("--clips", default=os.path.join(HERE, "clips"))
    ap.add_argument("--cues", default="FUSE3")
    ap.add_argument("--regen", action="store_true", help="force clip regeneration")
    a = ap.parse_args()

    if a.regen or not os.path.isdir(a.clips) or not os.listdir(a.clips):
        print(f"generating clips into {a.clips} ...")
        subprocess.run([sys.executable, os.path.join(HERE, "synth_footage.py"),
                        "--out", a.clips], check=True)

    names = sorted(n[:-4] for n in os.listdir(a.clips) if n.endswith(".mp4"))
    print(f"\ncues={a.cues}   (SNR = target vs local background; "
          f"<~1.4 is below this tracker's detectability cliff)\n")
    print(f"{'clip':<24}{'SNR':>6}{'on-target':>11}{'hold':>7}{'reacq':>7}{'err':>8}")
    print("-" * 63)
    rows = []
    for n in names:
        mp4 = os.path.join(a.clips, f"{n}.mp4")
        lab = os.path.join(a.clips, f"{n}.csv")
        if not os.path.exists(lab):
            continue
        snr = target_snr(mp4, lab)
        g = run_eval(mp4, lab, a.cues)
        on = g.get("on", "?")
        rows.append((n, snr, on))
        print(f"{n:<24}{snr:>6.2f}{on:>11}{g.get('hold','?'):>7}"
              f"{g.get('reacq','?'):>7}{g.get('mean','?'):>8}")
    # Headline: does performance track detectability, or is something else wrong?
    ok = [r for r in rows if r[1] >= 1.45]
    bad = [r for r in rows if r[1] < 1.40]
    def pct(rs):
        v = [float(r[2].rstrip('%')) for r in rs if r[2].rstrip('%').replace('.','').isdigit()]
        return np.mean(v) if v else float('nan')
    print("-" * 63)
    print(f"above SNR 1.45 ({len(ok)} clips): mean on-target {pct(ok):.0f}%")
    print(f"below SNR 1.40 ({len(bad)} clips): mean on-target {pct(bad):.0f}%")


if __name__ == "__main__":
    main()
