"""Make real video frames comparable to the DSM-generated reference.

A real camera frame differs from the synthetic pinhole reference by lens
distortion, an unknown FOV, camera roll (tilts the elevation axis), and exposure.
This canonicalizes each frame so the extracted skyline curve is comparable:

  1. undistort + derive FOV from calibration (if given)
  2. CLAHE contrast normalization (robust segmentation across exposure/sky)
  3. auto-roll leveling (rotate so the skyline's dominant tilt = camera roll = 0)

  preprocess(frame, fov_default, calib=None, ...) -> (gray_canonical, fov, roll)
"""
from __future__ import annotations
import json
import numpy as np
import cv2
from horizon import camera


def load_calib(path):
    """Return (K, dist, fov_deg) from a .json/.npz with fx,fy,cx,cy,k1,k2,p1,p2,k3."""
    if path.endswith(".npz"):
        d = dict(np.load(path))
    else:
        with open(path) as fh:
            d = json.load(fh)
    K = np.array([[d["fx"], 0, d["cx"]], [0, d["fy"], d["cy"]], [0, 0, 1]], float)
    dist = np.array([d.get(k, 0.0) for k in ("k1", "k2", "p1", "p2", "k3")], float)
    return K, dist


def fov_from_K(K, width):
    return float(np.degrees(2 * np.arctan(width / (2 * K[0, 0]))))


def undistort(gray, K, dist):
    return cv2.undistort(gray, K, dist)


def clahe(gray):
    return cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)


def estimate_roll(gray, fov_deg, max_roll=20.0):
    """Camera roll (deg) from the dominant tilt of the sky/canopy boundary
    (robust Theil-Sen slope, so local skyline structure doesn't bias it)."""
    elev = camera.extract_horizon(gray, hfov_deg=fov_deg)
    f = camera.focal_px(gray.shape[1], fov_deg)
    rows = gray.shape[0] / 2 - f * np.tan(elev)
    idx = np.linspace(0, gray.shape[1] - 1, 90).astype(int)
    c, r = idx.astype(float), rows[idx]
    s = []
    for i in range(len(c)):
        dc = c[i + 1:] - c[i]
        s.extend(((r[i + 1:] - r[i]) / dc).tolist())
    slope = float(np.median(s)) if s else 0.0
    return float(np.clip(np.degrees(np.arctan(slope)), -max_roll, max_roll))


def level(gray, roll_deg):
    h, w = gray.shape
    M = cv2.getRotationMatrix2D((w / 2, h / 2), roll_deg, 1.0)
    return cv2.warpAffine(gray, M, (w, h), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REPLICATE)


def preprocess(frame, fov_default=65.0, calib=None, auto_level=True,
               use_clahe=True, roll_override=None):
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    fov = fov_default
    if calib is not None:
        K, dist = calib
        gray = undistort(gray, K, dist)
        fov = fov_from_K(K, gray.shape[1])
    if use_clahe:
        gray = clahe(gray)
    roll = roll_override if roll_override is not None else \
        (estimate_roll(gray, fov) if auto_level else 0.0)
    if abs(roll) > 0.2:
        gray = level(gray, roll)
    return gray, fov, roll


# synthesize a distorted/rolled/under-exposed frame (for testing the recovery)
def corrupt(gray, K, dist, roll_deg, gain=0.55, bias=40):
    h, w = gray.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    pts = np.stack([u.ravel(), v.ravel()], 1).astype(np.float32)[:, None, :]
    und = cv2.undistortPoints(pts, K, dist, P=K).reshape(h, w, 2)
    out = cv2.remap(gray, und[..., 0].astype(np.float32), und[..., 1].astype(np.float32),
                    cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), -roll_deg, 1.0)
    out = cv2.warpAffine(out, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
    return np.clip(out.astype(float) * gain + bias, 0, 255).astype(np.uint8)
