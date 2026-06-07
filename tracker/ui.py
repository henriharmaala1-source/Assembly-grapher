import cv2
import numpy as np
from .core import State


class MouseHandler:
    def __init__(self):
        self._drawing = False
        self._p0 = None
        self._p1 = None
        self.pending_bbox = None   # (x, y, w, h) — consumed by main loop

    @property
    def drawing(self) -> bool:
        return self._drawing

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drawing = True
            self._p0 = (x, y)
            self._p1 = (x, y)
            self.pending_bbox = None
        elif event == cv2.EVENT_MOUSEMOVE and self._drawing:
            self._p1 = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._drawing:
            self._drawing = False
            self._p1 = (x, y)
            x1, y1 = self._p0
            x2, y2 = self._p1
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)
            if x2 - x1 > 10 and y2 - y1 > 10:
                self.pending_bbox = (x1, y1, x2 - x1, y2 - y1)

    def live_rect(self):
        """Current drag rectangle as (x1, y1, x2, y2) or None."""
        if not self._drawing or not self._p0 or not self._p1:
            return None
        x1, y1 = self._p0
        x2, y2 = self._p1
        return (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


# ----------------------------------------------------------------- rendering

def _sim_color(sim: float):
    if sim >= 0.55:
        return (0, 220, 50)       # green  — solid lock
    if sim >= 0.42:
        return (0, 190, 255)      # amber  — shaky
    return (30, 30, 240)          # red    — losing target


def _draw_brackets(img, x, y, w, h, color, thickness=2, arm=16):
    """Corner bracket decoration around bbox."""
    corners = [
        ((x,     y),     ( 1,  1)),
        ((x + w, y),     (-1,  1)),
        ((x,     y + h), ( 1, -1)),
        ((x + w, y + h), (-1, -1)),
    ]
    for (px, py), (dx, dy) in corners:
        cv2.line(img, (px, py), (px + dx * arm, py),        color, thickness)
        cv2.line(img, (px, py), (px,            py + dy * arm), color, thickness)


def draw_overlay(
    frame: np.ndarray,
    state: State,
    bbox,
    sim: float,
    mouse: MouseHandler,
    fps: float,
) -> np.ndarray:
    out = frame.copy()
    fh, fw = out.shape[:2]

    # ── live selection rectangle ──────────────────────────────────────────────
    rect = mouse.live_rect()
    if rect:
        x1, y1, x2, y2 = rect
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 220), 1)
        cv2.putText(out, "Select target", (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1)

    # ── lock overlay ──────────────────────────────────────────────────────────
    if state == State.LOCKED and bbox:
        x, y, w, h = bbox
        cx, cy = x + w // 2, y + h // 2
        color = _sim_color(sim)

        overlay = out.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)
        cv2.addWeighted(overlay, 0.08, out, 0.92, 0, out)

        cv2.rectangle(out, (x, y), (x + w, y + h), color, 1)
        _draw_brackets(out, x, y, w, h, color)

        cv2.line(out,  (cx - 14, cy), (cx + 14, cy), color, 1)
        cv2.line(out,  (cx, cy - 14), (cx, cy + 14), color, 1)
        cv2.circle(out, (cx, cy), 3, color, -1)

        bar_y = max(y - 18, 4)
        cv2.rectangle(out, (x, bar_y), (x + w, bar_y + 8), (40, 40, 40), -1)
        fill = max(0, int(w * min(sim, 1.0)))
        cv2.rectangle(out, (x, bar_y), (x + fill, bar_y + 8), color, -1)
        cv2.putText(out, f"{sim * 100:.0f}%", (x + w + 4, bar_y + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

    # ── searching overlay (last known position, dashed) ───────────────────────
    if state == State.SEARCHING and bbox:
        x, y, w, h = bbox
        pulse = (int(cv2.getTickCount() / cv2.getTickFrequency() * 4) % 2 == 0)
        color = (200, 160, 0) if pulse else (120, 90, 0)
        _draw_brackets(out, x, y, w, h, color, thickness=1)
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(out, (cx, cy), 6, color, 1)

    # ── status bar ────────────────────────────────────────────────────────────
    if state == State.IDLE:
        status_text  = "IDLE  |  Draw a box around the target"
        status_color = (170, 170, 170)
    elif state == State.SEARCHING:
        status_text  = "SEARCHING  |  Looking for target..."
        status_color = (200, 160, 0)
    else:
        status_text  = f"LOCKED  |  {sim * 100:.0f}% confidence"
        status_color = _sim_color(sim)

    cv2.putText(out, status_text, (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, status_color, 2)

    # ── FPS ───────────────────────────────────────────────────────────────────
    cv2.putText(out, f"{fps:.0f} fps", (fw - 76, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (190, 190, 190), 1)

    # ── help strip ───────────────────────────────────────────────────────────
    cv2.putText(out, "R  reset    ESC  quit", (10, fh - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (110, 110, 110), 1)

    return out
