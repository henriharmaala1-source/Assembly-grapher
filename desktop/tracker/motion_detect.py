"""
Blob motion detection via background subtraction (MOG2).

Finds moving regions in the frame, cleans them into blobs, and reports the
largest ones with a persistence check so a momentary flicker doesn't trigger.
Used to *automatically* hand a detected object to the segmenter / tracker:
motion says WHERE, the better model (SAM 2) says WHAT.
"""

import cv2
import numpy as np


class MotionDetector:
    def __init__(self, history: int = 500, var_threshold: float = 24.0):
        self._bg = cv2.createBackgroundSubtractorMOG2(
            history=history, varThreshold=var_threshold, detectShadows=True)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Persistence tracking for auto-trigger debounce
        self._persist_n   = 0
        self._persist_pos = None

    def set_threshold(self, var_threshold: float):
        """Lower = more sensitive to subtle motion."""
        self._bg.setVarThreshold(float(var_threshold))

    def detect(self, frame, min_area: int):
        """
        Returns (fg_mask uint8, blobs) where blobs is a list of
        (x, y, w, h, area) sorted largest-first.
        """
        fg = self._bg.apply(frame)
        # MOG2 marks shadows as 127; keep only hard foreground (255).
        _, fg = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN,  self._kernel)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, self._kernel, iterations=2)
        fg = cv2.dilate(fg, self._kernel, iterations=1)

        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area >= min_area:
                x, y, w, h = cv2.boundingRect(c)
                blobs.append((x, y, w, h, float(area)))
        blobs.sort(key=lambda b: -b[4])
        return fg, blobs

    def confirm(self, blob, need_frames: int = 4, move_tol: int = 60) -> bool:
        """
        True once the same blob (roughly same centre) has persisted for
        need_frames consecutive calls. Debounces the auto-trigger.
        """
        cx = blob[0] + blob[2] / 2.0
        cy = blob[1] + blob[3] / 2.0
        if self._persist_pos is not None:
            px, py = self._persist_pos
            if (cx - px) ** 2 + (cy - py) ** 2 <= move_tol ** 2:
                self._persist_n += 1
            else:
                self._persist_n = 1
        else:
            self._persist_n = 1
        self._persist_pos = (cx, cy)
        return self._persist_n >= need_frames

    def reset_persist(self):
        self._persist_n   = 0
        self._persist_pos = None
