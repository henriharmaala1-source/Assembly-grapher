"""Camera calibration -> cam.json (for locate_video.py --calib / video_preproc).

Detects a chessboard in photos or a video and runs OpenCV calibrateCamera to
recover intrinsics (fx, fy, cx, cy) + distortion (k1,k2,p1,p2,k3). Output JSON is
directly consumable by video_preproc.load_calib.

  python3 calibrate_camera.py --images calib/*.jpg --rows 6 --cols 9 --out cam.json
  python3 calibrate_camera.py --video calib.mp4   --rows 6 --cols 9 --out cam.json
  python3 calibrate_camera.py --selftest        # synthetic check (no inputs needed)

--rows/--cols are INNER corners (one less than the squares each way).
"""
from __future__ import annotations
import argparse
import glob
import json
import numpy as np
import cv2

_CRIT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


def _pack(K, dist, size, rms, used):
    d = dist.ravel()
    return dict(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]),
                cy=float(K[1, 2]), k1=float(d[0]), k2=float(d[1]),
                p1=float(d[2]), p2=float(d[3]), k3=float(d[4] if d.size > 4 else 0),
                width=int(size[0]), height=int(size[1]),
                fov_h_deg=float(np.degrees(2 * np.arctan(size[0] / (2 * K[0, 0])))),
                rms_reproj_px=float(rms), n_views=int(used))


def calibrate_from_arrays(grays, pattern, square_size):
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_size
    op, ip, size, used = [], [], None, 0
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    for g in grays:
        if g is None:
            continue
        size = (g.shape[1], g.shape[0])
        ok, corners = cv2.findChessboardCorners(g, pattern, flags)
        if not ok:
            continue
        corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), _CRIT)
        op.append(objp); ip.append(corners); used += 1
    if used < 4:
        raise RuntimeError(f"only {used} boards detected (need >=4). Check that "
                           f"--rows/--cols are INNER corners and the board is fully visible.")
    rms, K, dist, _, _ = cv2.calibrateCamera(op, ip, size, None, None)
    return _pack(K, dist, size, rms, used)


def calibrate_from_images(paths, pattern, square_size):
    grays = [cv2.imread(p, cv2.IMREAD_GRAYSCALE) for p in paths]
    return calibrate_from_arrays(grays, pattern, square_size)


def calibrate_from_video(path, pattern, square_size, every=15):
    cap = cv2.VideoCapture(path)
    grays, i = [], 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i % every == 0:
            grays.append(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr)
        i += 1
    cap.release()
    return calibrate_from_arrays(grays, pattern, square_size)


def _make_board(pattern, sq):
    nx, ny = pattern
    b = np.zeros(((ny + 1) * sq, (nx + 1) * sq), np.uint8)
    for r in range(ny + 1):
        for c in range(nx + 1):
            if (r + c) % 2 == 0:
                b[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255
    return b


def _selftest():
    pattern, sqpx, square = (9, 6), 40, 1.0
    board = _make_board(pattern, sqpx)
    Wb, Hb = board.shape[1], board.shape[0]
    imgW, imgH = 1280, 720
    fx = fy = 1100.0
    K = np.array([[fx, 0, imgW / 2], [0, fy, imgH / 2], [0, 0, 1.0]])
    Ki = np.linalg.inv(K)
    ms = square / sqpx
    Mp = np.diag([ms, ms, 1.0])
    Cx, Cy = Wb * ms / 2, Hb * ms / 2
    Z = fx * (Wb * ms) / (0.6 * imgW)
    rng = np.random.default_rng(0)
    grays = []
    for _ in range(16):
        rvec = rng.normal(0, 0.22, 3); rvec[2] *= 0.3
        R, _ = cv2.Rodrigues(rvec)
        dx, dy = rng.uniform(-.12, .12) * imgW, rng.uniform(-.12, .12) * imgH
        t = Z * (Ki @ np.array([imgW / 2 + dx, imgH / 2 + dy, 1.0])) - R[:, :2] @ [Cx, Cy]
        Hm = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ Mp
        grays.append(cv2.warpPerspective(board, Hm, (imgW, imgH), borderValue=128))
    cam = calibrate_from_arrays(grays, pattern, square)
    print("synthetic calibration (true fx=1100, cx=640, cy=360, dist=0):")
    print(f"  recovered fx={cam['fx']:.0f} fy={cam['fy']:.0f} "
          f"cx={cam['cx']:.0f} cy={cam['cy']:.0f}")
    print(f"  dist k1={cam['k1']:+.3f} k2={cam['k2']:+.3f}  "
          f"FOV {cam['fov_h_deg']:.1f}°  RMS {cam['rms_reproj_px']:.3f}px  "
          f"views {cam['n_views']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", help="image files or globs")
    ap.add_argument("--video")
    ap.add_argument("--rows", type=int, help="inner corners (vertical)")
    ap.add_argument("--cols", type=int, help="inner corners (horizontal)")
    ap.add_argument("--square", type=float, default=1.0, help="square size (any unit)")
    ap.add_argument("--every", type=int, default=15, help="video: use every Nth frame")
    ap.add_argument("--out", default="cam.json")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        _selftest(); return
    if not (a.rows and a.cols):
        ap.error("--rows and --cols (inner corners) are required")
    pattern = (a.cols, a.rows)
    if a.video:
        cam = calibrate_from_video(a.video, pattern, a.square, a.every)
    elif a.images:
        paths = [p for g in a.images for p in glob.glob(g)]
        cam = calibrate_from_images(paths, pattern, a.square)
    else:
        ap.error("provide --images or --video (or --selftest)")
    with open(a.out, "w") as fh:
        json.dump(cam, fh, indent=2)
    print(f"calibrated {cam['n_views']} views | fx={cam['fx']:.0f} "
          f"FOV={cam['fov_h_deg']:.1f}° RMS={cam['rms_reproj_px']:.3f}px -> {a.out}")


if __name__ == "__main__":
    main()
