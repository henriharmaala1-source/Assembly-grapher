#!/usr/bin/env python3
"""
capture_cv_live.py — live CV quality overlay on the camera feed. The real-time
sibling of capture_cv_check.py: open the device and draw the perception-stack
metrics on every frame so you can refocus, move things, and watch them respond.

Metrics (same meaning as capture_cv_check.py):
  sharpness (Laplacian variance) · edge density (Canny) · ORB feature count ·
  comb ratio (interlacing) · clipping (blown/crushed) · live FPS

Watch the comb ratio while you wave something fast — interlacing shows up as a
live spike on motion, which is the artifact to catch.

Keys:
  v  cycle view: raw -> edges -> features -> raw ...
  s  save a snapshot (cvlive_snap_N.png)
  q  or ESC  quit

Usage:
    pip install opencv-contrib-python numpy   # superset; also what tracker/ needs
                                              # (NOT a -headless build — needs GUI)
    python capture_cv_live.py                 # device 0
    python capture_cv_live.py --device 1
"""

import argparse
import platform
import time

import cv2
import numpy as np


def open_capture(device: int):
    if platform.system() == "Windows":
        cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)   # DSHOW < MSMF for jitter
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap if cap.isOpened() else None


def metrics(gray: np.ndarray, orb):
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    edges = cv2.Canny(gray, 80, 160)
    edge_density = float(np.count_nonzero(edges)) / gray.size
    kps = orb.detect(gray, None)
    g = gray.astype(np.float32)
    diff1 = np.mean(np.abs(g[1:] - g[:-1]))
    diff2 = np.mean(np.abs(g[2:] - g[:-2])) + 1e-6
    comb_ratio = diff1 / (diff2 / 2.0)
    blown = float(np.mean(gray >= 250))
    crushed = float(np.mean(gray <= 5))
    return lap_var, edge_density, edges, kps, comb_ratio, blown, crushed


def draw_hud(frame, lines):
    """Dark strip + text so the overlay is readable over any scene."""
    pad = 6
    y = pad
    for text, color in lines:
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (pad - 3, y), (pad + tw + 3, y + th + 6), (0, 0, 0), -1)
        cv2.putText(frame, text, (pad, y + th + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        y += th + 10
    return frame


def main() -> int:
    ap = argparse.ArgumentParser(description="Live CV quality overlay.")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--features", type=int, default=500,
                    help="ORB feature cap (lower = faster overlay)")
    args = ap.parse_args()

    cap = open_capture(args.device)
    if cap is None:
        print(f"ERROR: could not open device {args.device}. Try --device 1/2.")
        return 1

    orb = cv2.ORB_create(nfeatures=args.features)
    view = 0                      # 0 raw, 1 edges, 2 features
    views = ["raw", "edges", "features"]
    snap = 0
    prev = time.perf_counter()
    fps = 0.0

    GREEN, AMBER, RED = (0, 255, 0), (0, 200, 255), (0, 0, 255)
    print("keys: v=view  s=snapshot  q/ESC=quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("read failed"); break
        t = time.perf_counter()
        dt = t - prev; prev = t
        fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0.0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lap_var, edge_density, edges, kps, comb_ratio, blown, crushed = metrics(gray, orb)

        if view == 1:
            disp = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        elif view == 2:
            disp = cv2.drawKeypoints(frame, kps, None, color=GREEN)
        else:
            disp = frame.copy()

        c_sharp = GREEN if lap_var > 100 else RED
        c_edge = GREEN if edge_density > 0.02 else AMBER
        c_feat = GREEN if len(kps) > 300 else AMBER
        c_comb = GREEN if comb_ratio < 1.3 else RED
        draw_hud(disp, [
            (f"view:{views[view]}  fps:{fps:4.1f}", (255, 255, 255)),
            (f"sharp(lapvar): {lap_var:7.0f}", c_sharp),
            (f"edge density : {edge_density*100:5.2f}%", c_edge),
            (f"features     : {len(kps):5d}", c_feat),
            (f"comb ratio   : {comb_ratio:5.2f}", c_comb),
            (f"blown/crush  : {blown*100:4.1f}%/{crushed*100:4.1f}%", (255, 255, 255)),
        ])

        cv2.imshow("capture_cv_live  (v=view s=snap q=quit)", disp)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("v"):
            view = (view + 1) % len(views)
        elif k == ord("s"):
            name = f"cvlive_snap_{snap}.png"; cv2.imwrite(name, frame); snap += 1
            print(f"saved {name}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
