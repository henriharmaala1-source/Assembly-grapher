"""
spin_map — spin the webcam in place and see whether a real 360° scan finds an
escape bearing, the same way the drone's own SCAN phase does.

This is deliberately NOT general visual SLAM. Walking around needs to
estimate translation, which is scale-ambiguous from one camera and drifts
without correction -- genuinely hard. PURE ROTATION from a fixed spot has
neither problem: no translation means no scale ambiguity, and rotation is
directly recoverable from how far the image shifts sideways between frames.
That also happens to be exactly what MoveStopSense::Phase::SCAN does on the
real aircraft (yaw only, no translation, sweep until an opening appears) --
so this tool tests that exact mechanism against real depth-model data
instead of nav-sim's simulated raycasts. It builds a single-vantage-point
360 deg map (everything radiates from wherever you started spinning), not a
multi-room walkthrough -- that would reopen the translation/drift problem.

Pipeline, each frame:
  1. Sparse Lucas-Kanade optical flow between consecutive frames -> median
     horizontal pixel shift of robustly-tracked features -> a yaw delta
     (median is robust to a few moving/mistracked points; a few stray
     features shouldn't swing the whole estimate).
  2. DepthNav (the same model as tilt_bench.py) -> the same horizon-band
     openness histogram used there.
  3. Each column's openness gets written into a POLAR accumulator at
     yaw + that column's offset (mirrors onboard grid.integrate(), just
     bearing-indexed instead of Cartesian, since position never changes).
  4. Once enough of the circle is covered, find the widest contiguous open
     arc -- the same thing SCAN is looking for (corridorOpen >= threshold)
     -- and report its centre bearing as the escape direction.

Usage:
    python spin_map.py
    python spin_map.py --hfov 78          # set to your webcam's real horizontal FOV
    python spin_map.py --depth-model ~/depth_models/midas_small.onnx

Controls:
    r          reset the map and yaw estimate -- start a fresh spin
    s          save the current polar map + escape-bearing readout
    q / ESC    quit

How to use it: hold the camera as close to a fixed pivot point as you can
(rest your elbow, or turn only your wrist/a tripod head -- walking in a
circle instead of rotating in place reintroduces translation error) and
rotate through a full turn. Watch the coverage percentage and the escape
arrow build up on the radar view.
"""

import argparse
import json
import time

import cv2
import numpy as np

from tracker.depth_nav import DepthNav, draw_depth_overlay
from tilt_bench import horizon_band_openness, resolve_depth_model

N_BINS = 180          # 2 deg per bin
MIN_OPEN = 0.35        # mirrors minOpenToMove-style threshold (openness is [0,1] here)
MIN_ARC_BINS = 5        # an "opening" must span at least this many bins (~10 deg)
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
FEATURE_PARAMS = dict(maxCorners=200, qualityLevel=0.01, minDistance=12, blockSize=7)
MAX_PLAUSIBLE_YAW_DELTA = 25.0   # deg/frame -- beyond this, treat as a tracking glitch


class PolarMap:
    """Bearing-indexed openness accumulator for a single, fixed vantage point.
    No Cartesian grid needed -- position never changes here, only yaw."""

    def __init__(self, n_bins=N_BINS):
        self.n_bins = n_bins
        self.open = np.zeros(n_bins, dtype=np.float32)
        self.seen = np.zeros(n_bins, dtype=bool)

    def bin_of(self, bearing_deg):
        b = int(round((bearing_deg % 360.0) / 360.0 * self.n_bins)) % self.n_bins
        return b

    def integrate(self, yaw_deg, open_col, hfov_deg):
        n = len(open_col)
        for i, v in enumerate(open_col):
            rel = (i / (n - 1) - 0.5) * hfov_deg if n > 1 else 0.0
            b = self.bin_of(yaw_deg + rel)
            # latest reading wins for that bearing -- a fresh look at a
            # direction supersedes a stale one, same spirit as the onboard
            # blackboard's "latch, not heartbeat" staleness discipline.
            self.open[b] = float(v)
            self.seen[b] = True

    def coverage(self):
        return float(self.seen.mean())

    def best_escape(self):
        """Widest contiguous run of seen bins with openness >= MIN_OPEN.
        Returns (centre_bearing_deg, arc_width_deg) or (None, 0)."""
        openness_ok = self.seen & (self.open >= MIN_OPEN)
        if not openness_ok.any():
            return None, 0
        # circular run-length search
        doubled = np.concatenate([openness_ok, openness_ok])
        best_len, best_start = 0, 0
        run_start, run_len = None, 0
        for i, v in enumerate(doubled):
            if v:
                if run_start is None:
                    run_start = i
                run_len += 1
                if run_len > best_len:
                    best_len, best_start = run_len, run_start
            else:
                run_start, run_len = None, 0
            if i - (run_start or i) > self.n_bins:
                break
        if best_len < MIN_ARC_BINS:
            return None, 0
        centre_bin = (best_start + best_len / 2.0) % self.n_bins
        centre_deg = centre_bin / self.n_bins * 360.0
        width_deg = best_len / self.n_bins * 360.0
        return centre_deg, width_deg


def estimate_yaw_delta(prev_gray, gray, prev_pts, hfov_deg, frame_w):
    """Sparse LK optical flow -> median horizontal shift -> yaw delta (deg).
    Returns (yaw_delta_deg, new_pts_to_track_next_frame)."""
    if prev_pts is None or len(prev_pts) < 8:
        pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **FEATURE_PARAMS)
        return 0.0, pts

    nxt, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **LK_PARAMS)
    if nxt is None:
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
        return 0.0, pts

    status = status.reshape(-1).astype(bool)
    good_prev, good_next = prev_pts[status], nxt[status]
    if len(good_prev) < 8:
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
        return 0.0, pts

    dx = (good_next[:, 0, 0] - good_prev[:, 0, 0])
    median_dx = float(np.median(dx))
    yaw_delta = (median_dx / frame_w) * hfov_deg
    if abs(yaw_delta) > MAX_PLAUSIBLE_YAW_DELTA:
        # tracking glitch (big scene change, motion blur) -- ignore this
        # frame's estimate rather than let it corrupt the accumulated yaw.
        yaw_delta = 0.0

    # Re-seed features once too many have been lost or drifted.
    if len(good_next) < FEATURE_PARAMS["maxCorners"] * 0.4:
        pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
    else:
        pts = good_next.reshape(-1, 1, 2)
    return yaw_delta, pts


def draw_radar(size, polar: PolarMap, yaw_deg, escape):
    """Top-down radar view: a wedge per bin, coloured by openness, grey where
    unseen, a heading needle, and the escape arc highlighted once found."""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    c = size // 2
    r_max = c - 12
    cv2.circle(img, (c, c), r_max, (60, 60, 66), 1, cv2.LINE_AA)

    for b in range(polar.n_bins):
        bearing = b / polar.n_bins * 360.0
        a0 = np.deg2rad(bearing - 360 / polar.n_bins / 2 - 90)
        a1 = np.deg2rad(bearing + 360 / polar.n_bins / 2 - 90)
        if not polar.seen[b]:
            col = (45, 45, 50)
        else:
            v = polar.open[b]
            col = (0, int(np.clip(2 * v, 0, 1) * 255), int(np.clip(2 * (1 - v), 0, 1) * 255))
        pts = np.array([
            [c, c],
            [c + r_max * np.cos(a0), c + r_max * np.sin(a0)],
            [c + r_max * np.cos(a1), c + r_max * np.sin(a1)],
        ], dtype=np.int32)
        cv2.fillConvexPoly(img, pts, col, cv2.LINE_AA)

    # heading needle (current yaw)
    a = np.deg2rad(yaw_deg - 90)
    tip = (int(c + r_max * np.cos(a)), int(c + r_max * np.sin(a)))
    cv2.arrowedLine(img, (c, c), tip, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.15)

    centre_deg, width_deg = escape
    if centre_deg is not None:
        a = np.deg2rad(centre_deg - 90)
        tip = (int(c + (r_max + 14) * np.cos(a)), int(c + (r_max + 14) * np.sin(a)))
        cv2.arrowedLine(img, (c, c), tip, (128, 255, 0), 3, cv2.LINE_AA, tipLength=0.2)
        cv2.putText(img, f"ESCAPE {centre_deg:.0f} deg (+-{width_deg/2:.0f})",
                    (8, size - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (128, 255, 0), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--depth-model", default="")
    ap.add_argument("--depth-backend", choices=["midas", "dav2"], default="midas")
    ap.add_argument("--hfov", type=float, default=78.0,
                     help="webcam horizontal FOV in degrees (default 78 -- "
                          "check your actual webcam's spec, this matters a lot)")
    ap.add_argument("--band", type=float, default=0.2)
    ap.add_argument("--cam", type=int, default=0)
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

    print(__doc__)
    print(f"[spin_map] hfov={args.hfov} deg  band={args.band}  "
          f"MIN_OPEN={MIN_OPEN}  bins={N_BINS} ({360/N_BINS:.1f} deg/bin)")

    polar = PolarMap()
    yaw = 0.0
    prev_gray, prev_pts = None, None
    win = "spin_map"
    cv2.namedWindow(win)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Error: webcam read failed")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fw = frame.shape[1]

        if prev_gray is not None:
            dyaw, prev_pts = estimate_yaw_delta(prev_gray, gray, prev_pts, args.hfov, fw)
            yaw = (yaw + dyaw) % 360.0
        else:
            prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
        prev_gray = gray

        depth_nav.update(frame)
        depth = depth_nav.depth
        out = frame.copy()
        draw_depth_overlay(out, depth_nav.snapshot())

        if depth is not None and depth.size:
            open_col, r0, r1, mean_o, spread, frac_far = horizon_band_openness(depth, args.band)
            polar.integrate(yaw, open_col, args.hfov)

        escape = polar.best_escape()
        radar = draw_radar(320, polar, yaw, escape)

        cv2.rectangle(out, (0, 0), (330, 56), (20, 20, 24), -1)
        cv2.putText(out, f"yaw {yaw:5.0f} deg   coverage {polar.coverage()*100:4.0f}%",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(out, "r: reset spin   s: save   q: quit",
                    (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 170, 180), 1, cv2.LINE_AA)

        fh, fw2 = out.shape[:2]
        canvas = np.zeros((max(fh, 320), fw2 + 320, 3), dtype=np.uint8)
        canvas[:fh, :fw2] = out
        canvas[:320, fw2:fw2 + 320] = radar
        cv2.imshow(win, canvas)

        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('r'):
            polar = PolarMap()
            yaw = 0.0
            prev_gray, prev_pts = None, None
            print("[spin_map] reset -- start spinning again")
        elif k == ord('s'):
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"spin_map_{ts}_radar.png", radar)
            cv2.imwrite(f"spin_map_{ts}_view.png", out)
            centre_deg, width_deg = escape
            with open(f"spin_map_{ts}_metrics.json", "w") as f:
                json.dump({
                    "coverage": polar.coverage(), "yaw": yaw,
                    "escape_bearing_deg": centre_deg, "escape_arc_width_deg": width_deg,
                    "hfov": args.hfov, "band": args.band,
                }, f, indent=2)
            print(f"[spin_map] saved spin_map_{ts}_*")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
