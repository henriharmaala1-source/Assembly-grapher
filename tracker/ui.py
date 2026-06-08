import cv2
import numpy as np
from .core import State
from .settings import Settings


class MouseHandler:
    def __init__(self):
        self._drawing = False
        self._p0 = None
        self._p1 = None
        self.pending_bbox = None

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
        if not self._drawing or not self._p0 or not self._p1:
            return None
        x1, y1 = self._p0
        x2, y2 = self._p1
        return (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


# ----------------------------------------------------------------- rendering

def _sim_color(sim: float):
    if sim >= 0.55:
        return (0, 220, 50)
    if sim >= 0.42:
        return (0, 190, 255)
    return (30, 30, 240)


def _draw_brackets(img, x, y, w, h, color, thickness=2, arm=16):
    for (px, py), (dx, dy) in [
        ((x,     y),     ( 1,  1)),
        ((x + w, y),     (-1,  1)),
        ((x,     y + h), ( 1, -1)),
        ((x + w, y + h), (-1, -1)),
    ]:
        cv2.line(img, (px, py), (px + dx * arm, py),        color, thickness)
        cv2.line(img, (px, py), (px,            py + dy * arm), color, thickness)


def _draw_attention(out, bbox, soft_map, opacity):
    """Blend the DINOv2 attention heat map over the tracked region."""
    x, y, w, h = bbox
    fh, fw = out.shape[:2]
    x1 = max(0, x);   y1 = max(0, y)
    x2 = min(fw, x + w); y2 = min(fh, y + h)
    if x2 <= x1 or y2 <= y1:
        return

    rw, rh = x2 - x1, y2 - y1
    mask = cv2.resize(soft_map, (rw, rh), interpolation=cv2.INTER_LINEAR)

    # Map to a teal-green colour (BGR: 200, 255, 100)
    coloured = np.zeros((rh, rw, 3), dtype=np.float32)
    coloured[:, :, 0] = mask * 180   # B
    coloured[:, :, 1] = mask * 255   # G
    coloured[:, :, 2] = mask * 80    # R

    roi     = out[y1:y2, x1:x2].astype(np.float32)
    blended = np.clip(roi + coloured * opacity, 0, 255).astype(np.uint8)
    out[y1:y2, x1:x2] = blended

    # Contour of the thresholded mask
    binary   = (mask > 0.5).astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out[y1:y2, x1:x2], contours, -1, (100, 255, 160), 1)


def _draw_motion(out, center, trail, predicted):
    """Draw the path trail and a predictive velocity arrow from the center."""
    cx, cy = int(center[0]), int(center[1])

    # Fading trail of past centers
    if trail and len(trail) >= 2:
        n = len(trail)
        for i in range(1, n):
            a = (int(trail[i - 1][0]), int(trail[i - 1][1]))
            b = (int(trail[i][0]),     int(trail[i][1]))
            t = i / n                         # 0 (old) → 1 (recent)
            col = (int(80 + 100 * t), int(120 + 100 * t), 30)
            cv2.line(out, a, b, col, 1, cv2.LINE_AA)

    # Predictive arrow: where the object is heading
    if predicted is not None:
        px, py = int(predicted[0]), int(predicted[1])
        dist = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
        if dist > 4:                          # only when actually moving
            cv2.arrowedLine(out, (cx, cy), (px, py),
                            (0, 140, 255), 2, cv2.LINE_AA, tipLength=0.3)
            cv2.circle(out, (px, py), 4, (0, 140, 255), 1, cv2.LINE_AA)


def draw_overlay(
    frame: np.ndarray,
    state: State,
    bbox,
    sim: float,
    mouse: MouseHandler,
    fps: float,
    settings: Settings = None,
    attn_map=None,          # soft attention map (numpy float32) or None
    motion_trail=None,      # list of (cx, cy) recent centers
    predicted_center=None,  # (px, py) predicted future center
) -> np.ndarray:
    out = frame.copy()
    fh, fw = out.shape[:2]

    cfg = settings or Settings()    # fall back to defaults if not provided

    # ── live selection rectangle ──────────────────────────────────────────────
    rect = mouse.live_rect()
    if rect:
        x1, y1, x2, y2 = rect
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 220), 1)
        cv2.putText(out, "Select target", (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1)

    # ── attention mask overlay ────────────────────────────────────────────────
    if cfg.show_mask and attn_map is not None and bbox is not None:
        _draw_attention(out, bbox, attn_map, cfg.mask_opacity)

    # ── lock overlay ──────────────────────────────────────────────────────────
    if state == State.LOCKED and bbox:
        x, y, w, h = bbox
        cx, cy = x + w // 2, y + h // 2
        color = _sim_color(sim)

        overlay = out.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)
        cv2.addWeighted(overlay, 0.07, out, 0.93, 0, out)

        cv2.rectangle(out, (x, y), (x + w, y + h), color, 1)
        _draw_brackets(out, x, y, w, h, color)

        cv2.line(out,  (cx - 14, cy), (cx + 14, cy), color, 1)
        cv2.line(out,  (cx, cy - 14), (cx, cy + 14), color, 1)
        cv2.circle(out, (cx, cy), 3, color, -1)

        if cfg.show_confidence_bar:
            bar_y = max(y - 18, 4)
            cv2.rectangle(out, (x, bar_y), (x + w, bar_y + 8), (40, 40, 40), -1)
            cv2.rectangle(out, (x, bar_y),
                          (x + max(0, int(w * min(sim, 1.0))), bar_y + 8), color, -1)
            cv2.putText(out, f"{sim * 100:.0f}%", (x + w + 4, bar_y + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        # ── motion vector: trail + predictive arrow ───────────────────────────
        if cfg.show_motion_vector:
            _draw_motion(out, (cx, cy), motion_trail, predicted_center)

    # ── searching overlay ─────────────────────────────────────────────────────
    if state == State.SEARCHING and bbox:
        x, y, w, h = bbox
        pulse = (int(cv2.getTickCount() / cv2.getTickFrequency() * 4) % 2 == 0)
        color = (200, 160, 0) if pulse else (100, 80, 0)
        _draw_brackets(out, x, y, w, h, color, thickness=1)
        cx, cy = x + w // 2, y + h // 2
        cv2.circle(out, (cx, cy), 7, color, 1)

    # ── status bar ────────────────────────────────────────────────────────────
    if state == State.IDLE:
        status, sc = "IDLE  |  Draw a box around the target", (170, 170, 170)
    elif state == State.SEARCHING:
        status, sc = "SEARCHING  |  Looking for target…",    (200, 160,   0)
    else:
        status, sc = f"LOCKED  |  {sim * 100:.0f}% confidence", _sim_color(sim)

    cv2.putText(out, status, (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, sc, 2)

    # ── FPS ───────────────────────────────────────────────────────────────────
    if cfg.show_fps:
        cv2.putText(out, f"{fps:.0f} fps", (fw - 76, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (190, 190, 190), 1)

    # ── backend label ─────────────────────────────────────────────────────────
    cv2.putText(out, f"backend: {cfg.tracker_backend}", (fw - 200, fh - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1)

    # ── help strip ───────────────────────────────────────────────────────────
    cv2.putText(out, "R  reset    ESC  quit", (10, fh - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (110, 110, 110), 1)

    return out
