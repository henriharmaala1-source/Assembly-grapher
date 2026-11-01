"""
Capture-time frame preprocessing: digital zoom + appearance-normalising filters.

Applied in capture_loop BEFORE anything else sees the frame, so capture,
tracking and display all consume one transformed frame — designation,
DINOv2 anchoring, identity verification and the on-screen view stay in a single
consistent coordinate + colour space, with no remapping anywhere downstream.

Two independent aids for the two lock-on failure modes:

  ZOOM (small / distant target) — a virtual pan-tilt-zoom. Crops a (W/z, H/z)
  window and upscales it to full size. While a target is LOCKED the window eases
  to follow the target centre so a magnified object stays framed; otherwise it
  holds the frame centre (good for designating a small target). Honest caveat:
  digital zoom is interpolation — it does NOT add real sensor resolution, so it
  helps designation/framing and marginally the box tracker, not the raw pixels
  on target. Optical zoom or flying closer is the only way to add real detail.

  FILTERS (shifting illumination breaking lock) — per-frame illumination
  normalisation so the DINOv2 anchor captured under one lighting colour still
  matches the live crop after the colour shifts:
    none      passthrough
    wb        gray-world white balance — removes the global colour cast
    wb+clahe  gray-world WB, then CLAHE on L — also normalises local contrast
"""

import cv2
import numpy as np

from .core import State


class FramePreprocessor:
    def __init__(self, settings):
        self._s = settings
        self._cx = None            # eased follow centre, in RAW-frame coords
        self._cy = None
        self._last_crop = None     # (rx, ry, rw, rh) that produced the last frame
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    # ------------------------------------------------------------------- apply

    def process(self, frame, result):
        frame = self._zoom(frame, result)
        frame = self._filter(frame)
        return frame

    # -------------------------------------------------------------------- zoom

    def _zoom(self, frame, result):
        z = float(getattr(self._s, "zoom", 1.0) or 1.0)
        h, w = frame.shape[:2]
        if z <= 1.001:                       # zoom off — reset follow state
            self._cx = self._cy = None
            self._last_crop = None
            return frame

        # Where is the target in RAW coords? Map the last reported bbox (which is
        # in the PREVIOUS processed frame's coords) back through the crop that
        # produced it. Only follow while genuinely locked.
        target = None
        if (result and result.get("state") == State.LOCKED
                and result.get("bbox") and self._last_crop is not None):
            bx, by, bw, bh = result["bbox"]
            rx, ry, rw, rh = self._last_crop
            target = (rx + (bx + bw / 2.0) * rw / w,
                      ry + (by + bh / 2.0) * rh / h)

        goal = target if target is not None else (w / 2.0, h / 2.0)
        if self._cx is None:
            self._cx, self._cy = goal
        else:                                # ease so the view glides, no jitter
            a = 0.15
            self._cx += a * (goal[0] - self._cx)
            self._cy += a * (goal[1] - self._cy)

        cw, ch = w / z, h / z
        rw, rh = int(round(cw)), int(round(ch))
        rx = int(round(self._cx - cw / 2.0))
        ry = int(round(self._cy - ch / 2.0))
        rx = max(0, min(rx, w - rw))         # keep the crop inside the frame
        ry = max(0, min(ry, h - rh))
        self._last_crop = (rx, ry, rw, rh)

        crop = frame[ry:ry + rh, rx:rx + rw]
        if crop.size == 0:
            return frame
        return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)

    # ----------------------------------------------------------------- filters

    def _filter(self, frame):
        mode = getattr(self._s, "filter_mode", "none")
        if mode == "none":
            return frame
        out = self._gray_world(frame)
        if mode == "wb+clahe":
            out = self._apply_clahe(out)
        return out

    @staticmethod
    def _gray_world(bgr):
        """Scale each channel so its mean equals the global gray mean — removes
        the frame's overall colour cast (the shifting-room-light problem)."""
        f = bgr.astype(np.float32)
        means = f.reshape(-1, 3).mean(axis=0) + 1e-6
        gray = float(means.mean())
        f *= (gray / means)
        return np.clip(f, 0, 255).astype(np.uint8)

    def _apply_clahe(self, bgr):
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self._clahe.apply(l)
        return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
