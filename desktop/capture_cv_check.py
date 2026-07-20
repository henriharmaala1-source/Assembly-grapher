#!/usr/bin/env python3
"""
capture_cv_check.py — objective CV quality check on a captured frame, from the
perception stack's point of view. Run it on the RAW sample frames the capture
tool saved (capture_sample_*.png) — NOT on a screenshot pasted into chat, which
has been re-compressed and would give meaningless numbers.

Answers "what will the depth model / optical flow actually get from this frame":
  * sharpness / focus      (Laplacian variance)
  * edge density           (Canny) — texture the depth model keys on
  * trackable features     (ORB keypoints) — matters for VIO / optical flow (P5a)
  * interlacing / combing  (vertical comb metric) — the CVBS artifact to catch
  * clipping               (blown / crushed pixels) — analog DR limits, sky-blowout
  * colour cast            (per-channel means) — benign for depth, good to know

Saves annotated images next to the input so you can eyeball what it measured:
  <name>_edges.png, <name>_features.png

Usage:
    pip install opencv-contrib-python numpy   # superset; also what tracker/ needs
    python capture_cv_check.py capture_sample_1.png
    python capture_cv_check.py *.png          # several at once
"""

import sys
import glob

import cv2
import numpy as np


def analyse(path: str) -> None:
    img = cv2.imread(path)
    if img is None:
        print(f"!! could not read {path}")
        return
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # --- sharpness / focus: variance of the Laplacian. Higher = sharper.
    # A soft/out-of-focus or motion-blurred frame collapses this number.
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()

    # --- edge density: Canny, fraction of pixels that are edges. This is the
    # texture the monocular depth model leans on; a washed-out low-edge frame
    # gives it little to work with.
    edges = cv2.Canny(gray, 80, 160)
    edge_density = float(np.count_nonzero(edges)) / (h * w)

    # --- trackable features: ORB keypoints. Directly relevant to P5a VIO /
    # sparse optical flow — few features means little to track between frames.
    orb = cv2.ORB_create(nfeatures=2000)
    kps = orb.detect(gray, None)
    n_feat = len(kps)

    # --- interlacing / combing metric. For a progressive frame, the mean abs
    # difference between vertically ADJACENT rows is ~half that between rows two
    # apart (smooth gradient). Interlacing with motion elevates the adjacent-row
    # difference (consecutive rows come from different time-fields), pushing the
    # ratio above ~1. Also count "comb pixels": vertical local extrema whose
    # value sits outside both vertical neighbours (the teeth of a comb).
    g = gray.astype(np.float32)
    diff1 = np.mean(np.abs(g[1:] - g[:-1]))            # adjacent rows
    diff2 = np.mean(np.abs(g[2:] - g[:-2])) + 1e-6     # two rows apart (same field)
    comb_ratio = diff1 / (diff2 / 2.0)                 # ~1 progressive, >1 combed
    up = g[1:-1] - g[:-2]
    dn = g[1:-1] - g[2:]
    comb_pixels = np.mean((up * dn > 25))              # frac. of vertical extrema

    # --- clipping: analog CVBS has narrow dynamic range. Bright sources (lamp,
    # sky, sun) blow to 255 and read as ambiguous to the depth model; deep
    # shadow crushes to 0. Report both.
    blown = float(np.mean(gray >= 250))
    crushed = float(np.mean(gray <= 5))

    # --- colour cast: per-channel means (BGR). Big spread = strong cast. Benign
    # for luminance-based depth, but a sanity check on white balance.
    b, gch, r = (float(img[:, :, c].mean()) for c in range(3))

    print(f"\n=== {path}  ({w}x{h}) ===")
    print(f"sharpness (lap var)   : {lap_var:8.1f}   (higher=sharper; <100 is soft/blurred)")
    print(f"edge density          : {edge_density*100:7.2f}%  (texture for depth; ~2-8% is rich)")
    print(f"ORB features          : {n_feat:6d}     (trackable pts for VIO/flow; want hundreds+)")
    print(f"comb ratio            : {comb_ratio:8.2f}   (~1 progressive, >1.3 suggests interlacing)")
    print(f"comb-pixel fraction   : {comb_pixels*100:7.2f}%  (teeth of a comb; low=clean)")
    print(f"clipping blown/crushed: {blown*100:6.2f}% / {crushed*100:.2f}%  (analog DR limit)")
    print(f"channel means B/G/R   : {b:5.1f} / {gch:5.1f} / {r:5.1f}   (spread = colour cast)")

    # quick verdict lines
    verdict = []
    verdict.append("sharp" if lap_var > 100 else "SOFT/blurred — refocus?")
    verdict.append("rich texture" if edge_density > 0.02 else "LOW texture")
    verdict.append("plenty of features" if n_feat > 300 else "FEW features")
    verdict.append("clean (progressive-like)" if comb_ratio < 1.3 else "possible INTERLACING — try deinterlace")
    print("verdict: " + " | ".join(verdict))

    stem = path.rsplit(".", 1)[0]
    cv2.imwrite(f"{stem}_edges.png", edges)
    vis = cv2.drawKeypoints(img, kps, None, color=(0, 255, 0))
    cv2.imwrite(f"{stem}_features.png", vis)
    print(f"wrote {stem}_edges.png, {stem}_features.png")


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print("usage: python capture_cv_check.py <image> [more images...]")
        print("  (run it on the raw capture_sample_*.png, not a chat screenshot)")
        return 2
    paths: list[str] = []
    for a in args:
        paths.extend(glob.glob(a) or [a])
    for p in paths:
        analyse(p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
