import cv2
import numpy as np
from .core import State
from .settings import Settings
from .depth_nav import draw_depth_overlay


class MouseHandler:
    _CLICK_THRESH = 8   # pixels — move less than this → treated as a click

    def __init__(self):
        self._drawing = False
        self._p0 = None
        self._p1 = None
        self.pending_bbox  = None
        self.pending_point = None   # (x, y) set on a short click (no drag)

    @property
    def drawing(self) -> bool:
        return self._drawing

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drawing = True
            self._p0 = (x, y)
            self._p1 = (x, y)
            self.pending_bbox  = None
        elif event == cv2.EVENT_MOUSEMOVE and self._drawing:
            self._p1 = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self._drawing:
            self._drawing = False
            self._p1 = (x, y)
            x1, y1 = self._p0
            x2, y2 = self._p1
            dx, dy = abs(x2 - x1), abs(y2 - y1)
            if dx <= self._CLICK_THRESH and dy <= self._CLICK_THRESH:
                # Short click → segment prompt
                self.pending_point = (x, y)
            else:
                # Drag → tracking box
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


def _draw_segment(out, mask, point, backend_name, opacity=0.45):
    """Filled colour mask + white edge contour + clicked point marker."""
    if mask is None:
        return

    # Coloured fill (vivid cyan-green)
    coloured = np.zeros_like(out, dtype=np.float32)
    coloured[mask, 0] = 30
    coloured[mask, 1] = 230
    coloured[mask, 2] = 160
    out[:] = np.clip(
        out.astype(np.float32) * (1 - opacity * mask[:, :, None]) + coloured * opacity,
        0, 255,
    ).astype(np.uint8)

    # Edge contour
    binary = mask.astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, contours, -1, (255, 255, 255), 2, cv2.LINE_AA)

    # Clicked point
    if point is not None:
        px, py = int(point[0]), int(point[1])
        cv2.circle(out, (px, py), 7, (0, 255, 160), -1, cv2.LINE_AA)
        cv2.circle(out, (px, py), 7, (0, 0, 0),     2,  cv2.LINE_AA)

    # Backend label
    cv2.putText(out, f"seg: {backend_name}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 230, 160), 1)


def _draw_drone_detect(out, detections, preset):
    """Boxes for drone/bird detections — red for drones, green for birds."""
    from .drone_detect import PRESETS
    cfg     = PRESETS.get(preset, {})
    classes = cfg.get("classes", [])
    colors  = cfg.get("colors",  [(40, 80, 255)])
    for x, y, w, h, cls_id, conf in detections:
        label = classes[cls_id] if cls_id < len(classes) else str(cls_id)
        color = colors[cls_id % len(colors)]
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2, cv2.LINE_AA)
        txt = f"{label} {conf * 100:.0f}%"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ty = max(y - 2, th + 3)
        cv2.rectangle(out, (x, ty - th - 3), (x + tw + 5, ty + 3), color, -1)
        cv2.putText(out, txt, (x + 2, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def _draw_drone_hud(out, drone_result, settings):
    """Minimal drone-mode overlay: box on target + reliability counters."""
    fh, fw = out.shape[:2]
    dim_gray = (110, 110, 110)

    # No target yet
    bbox = (drone_result or {}).get("bbox")
    if not drone_result or bbox is None:
        cv2.putText(out, "DRONE  —  click to designate target", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (160, 160, 160), 2, cv2.LINE_AA)
        cv2.putText(out, "click=designate  D=exit drone  R=reset  ESC=quit",
                    (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, dim_gray, 1)
        return

    locked       = drone_result.get("locked", False)
    loss_frames  = drone_result.get("loss_frames", 0)
    age          = drone_result.get("age", 0)
    total_losses = drone_result.get("total_losses", 0)

    x, y, w, h = bbox
    cx, cy     = x + w // 2, y + h // 2

    if locked and loss_frames == 0:
        color, label = (0, 255, 70),  "LOCK"
    elif locked:
        color, label = (0, 200, 255), "COAST"
    else:
        color, label = (40, 60, 255), "LOST"

    # Target box
    _draw_brackets(out, x, y, w, h, color, thickness=2, arm=10)
    cv2.circle(out, (cx, cy), 3, color, -1, cv2.LINE_AA)

    # Reliability stats next to the box
    bk  = getattr(settings, "drone_backend", "CSRT")
    lh  = 18
    ty  = max(y - 6, lh)
    cv2.putText(out, f"{label}  {bk}", (x, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    cv2.putText(out, f"age {age}f  lost {total_losses}x",
                (x, ty + lh), cv2.FONT_HERSHEY_SIMPLEX, 0.40, dim_gray, 1)

    # Status bar
    cv2.putText(out, f"DRONE  {label}  |  {bk}", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)

    # Help strip
    cv2.putText(out, "click=designate  D=exit drone  R=reset  ESC=quit",
                (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, dim_gray, 1)


def _draw_motion_detect(out, blobs, fg_mask):
    """Foreground mask tint + boxes around moving blobs (largest highlighted)."""
    if fg_mask is not None:
        tint = np.zeros_like(out)
        tint[fg_mask > 0] = (60, 0, 120)
        cv2.addWeighted(tint, 0.5, out, 1.0, 0, out)

    if not blobs:
        return
    for i, (x, y, w, h, area) in enumerate(blobs):
        primary = (i == 0)
        color = (60, 220, 255) if primary else (90, 140, 160)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2 if primary else 1)
        if primary:
            cv2.putText(out, f"motion {int(area)}px", (x, max(y - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def draw_overlay(
    frame: np.ndarray,
    state: State,
    bbox,
    sim: float,
    mouse: MouseHandler,
    fps: float,
    settings: Settings = None,
    attn_map=None,
    motion_trail=None,
    predicted_center=None,
    seg_result=None,
    motion_blobs=None,
    motion_mask=None,
    drone_result=None,
    drone_detections=None,   # list of (x,y,w,h,cls,conf) from DroneDetector
    drone_detect_preset=None,
    depth_snap=None,         # snapshot dict from DepthNav.snapshot()
) -> np.ndarray:
    out = frame.copy()
    fh, fw = out.shape[:2]

    cfg = settings or Settings()    # fall back to defaults if not provided

    # ── Depth navigation overlay (drawn first — sits under everything) ────────
    if depth_snap is not None:
        draw_depth_overlay(out, depth_snap)

    # ── Drone detections (drawn first — under all tracking overlays) ─────────
    if drone_detections:
        _draw_drone_detect(out, drone_detections, drone_detect_preset)

    # ── motion detection overlay (drawn under everything) ─────────────────────
    if cfg.motion_detect and (motion_blobs or motion_mask is not None):
        _draw_motion_detect(out, motion_blobs, motion_mask)

    # ── click-to-segment overlay (drawn first, tracking overlays on top) ──────
    if seg_result is not None:
        _draw_segment(out, seg_result.get("mask"), seg_result.get("point"),
                      seg_result.get("backend", ""),
                      opacity=getattr(cfg, "seg_opacity", 0.45))

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

    drone_mode = getattr(cfg, "drone_mode", False)

    if drone_mode:
        # Drone mode: drone HUD owns the target box, status bar, and help strip.
        _draw_drone_hud(out, drone_result, cfg)
    else:
        # ── lock overlay ──────────────────────────────────────────────────────
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

            if cfg.show_motion_vector:
                _draw_motion(out, (cx, cy), motion_trail, predicted_center)

        # ── searching overlay ─────────────────────────────────────────────────
        if state == State.SEARCHING and bbox:
            x, y, w, h = bbox
            pulse = (int(cv2.getTickCount() / cv2.getTickFrequency() * 4) % 2 == 0)
            color = (200, 160, 0) if pulse else (100, 80, 0)
            _draw_brackets(out, x, y, w, h, color, thickness=1)
            cx, cy = x + w // 2, y + h // 2
            cv2.circle(out, (cx, cy), 7, color, 1)

        # ── status bar ────────────────────────────────────────────────────────
        if state == State.IDLE:
            status, sc = "IDLE  |  Draw a box around the target", (170, 170, 170)
        elif state == State.SEARCHING:
            status, sc = "SEARCHING  |  Looking for target…",    (200, 160,   0)
        else:
            status, sc = f"LOCKED  |  {sim * 100:.0f}% confidence", _sim_color(sim)

        cv2.putText(out, status, (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, sc, 2)

        # ── help strip ────────────────────────────────────────────────────────
        cv2.putText(out, "click=segment  drag=track  C=clear  D=drone  R=reset  ESC=quit",
                    (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (110, 110, 110), 1)

    # ── FPS (always shown) ────────────────────────────────────────────────────
    if cfg.show_fps:
        cv2.putText(out, f"{fps:.0f} fps", (fw - 76, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (190, 190, 190), 1)

    # ── backend label ─────────────────────────────────────────────────────────
    if not drone_mode:
        cv2.putText(out, f"backend: {cfg.tracker_backend}", (fw - 200, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 140, 140), 1)

    return out
