"""
Monocular depth navigation — Python port of rpi5_tracker/src/depth_nav.cpp.

Runs MiDaS Small or Depth Anything v2 Small via OpenCV DNN (CPU), then
computes the best traverse direction using navigable-corridor scoring:

  1. Clearance field  — Gaussian blur with kernel ~ vehicle width.
     Isolated far pixels (needle gaps) are blurred away; only a far region
     with open margin survives.
  2. Forward bias     — radial falloff from image centre so straight-ahead
     wins when two corridors tie.
  3. Kalman smoothing — steer point filtered through KalmanCenter so the
     arrow glides and coasts through a single noisy frame.
"""

import math

import cv2
import numpy as np

from .motion import KalmanCenter

_WORK_W    = 96
_WORK_H    = 72
_BIAS_K    = 0.40
_MIN_MARGIN = 0.04
_GRID_ROWS = 3
_GRID_COLS = 3


class DepthNav:
    def __init__(self):
        self._net      = None
        self._backend  = "midas"
        self._invert   = False
        self._in_size  = (256, 256)   # (W, H) for cv2.resize
        self._depth    = None         # (H, W) float32, normalised [0,1]
        self._sectors  = None         # (_GRID_ROWS, _GRID_COLS) float32
        self._traverse = None         # dict — see _compute_traverse
        self._bias     = None         # (_WORK_H, _WORK_W) float32, cached
        self._kalman   = KalmanCenter()

    # ----------------------------------------------------------------- init

    def init(self, model_path: str, backend: str = "midas") -> bool:
        try:
            net = cv2.dnn.readNetFromONNX(model_path)
        except cv2.error as e:
            print(f"[depth] failed to load model: {e}")
            return False
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._net     = net
        self._backend = backend
        if backend == "midas":
            self._in_size = (256, 256)
            self._invert  = True    # MiDaS: higher = closer → invert
        else:
            self._in_size = (518, 518)
            self._invert  = False   # DAv2: higher = farther
        print(f"[depth] {backend} loaded ({model_path})")
        return True

    def is_ready(self) -> bool:
        return self._net is not None

    @property
    def traverse(self):
        return self._traverse

    @property
    def sectors(self):
        return self._sectors

    # ---------------------------------------------------------------- update

    def update(self, frame: np.ndarray) -> bool:
        if self._net is None:
            return False

        self._net.setInput(self._preprocess(frame))
        raw = self._net.forward()   # (1,1,H,W) or (1,H,W)

        raw = raw.squeeze()
        if raw.ndim != 2:
            side = int(math.sqrt(raw.size))
            raw = raw.reshape(side, side)

        mn, mx = float(raw.min()), float(raw.max())
        fh, fw = frame.shape[:2]
        if mx - mn < 1e-6:
            self._depth    = np.zeros((fh, fw), dtype=np.float32)
            self._traverse = {"valid": False}
            return True

        norm = ((raw - mn) / (mx - mn)).astype(np.float32)
        if self._invert:
            norm = 1.0 - norm

        self._depth = cv2.resize(norm, (fw, fh), interpolation=cv2.INTER_LINEAR)
        self._compute_sectors(fw, fh)
        self._compute_traverse(fw, fh)
        return True

    # --------------------------------------------------------- preprocessing

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        w, h = self._in_size
        rgb  = cv2.cvtColor(
            cv2.resize(frame, (w, h), interpolation=cv2.INTER_LINEAR),
            cv2.COLOR_BGR2RGB,
        )
        fp   = rgb.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        fp   = (fp - mean) / std
        return fp.transpose(2, 0, 1)[np.newaxis].astype(np.float32)

    # ---------------------------------------------------------- sector grid

    def _compute_sectors(self, fw: int, fh: int):
        cw = fw // _GRID_COLS
        ch = fh // _GRID_ROWS
        self._sectors = np.array(
            [[self._depth[r*ch:(r+1)*ch, c*cw:(c+1)*cw].mean()
              for c in range(_GRID_COLS)]
             for r in range(_GRID_ROWS)],
            dtype=np.float32,
        )

    # ------------------------------------------------------- corridor score

    def _build_bias(self):
        cy, cx = _WORK_H / 2.0, _WORK_W / 2.0
        rmax   = math.sqrt(cx * cx + cy * cy)
        ys, xs = np.mgrid[0:_WORK_H, 0:_WORK_W].astype(np.float32)
        r      = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2) / rmax
        self._bias = (1.0 - _BIAS_K * r).astype(np.float32)

    def _compute_traverse(self, fw: int, fh: int):
        small = cv2.resize(self._depth, (_WORK_W, _WORK_H),
                           interpolation=cv2.INTER_AREA)

        k     = (_WORK_W // 6) | 1     # odd kernel
        clear = cv2.GaussianBlur(small, (k, k), 0)

        if self._bias is None:
            self._build_bias()
        steer = clear * self._bias

        idx       = int(np.argmax(steer))
        py, px    = divmod(idx, _WORK_W)
        mx_val    = float(steer.flat[idx])
        mean_s    = float(steer.mean())

        raw_x = (px + 0.5) / _WORK_W * fw
        raw_y = (py + 0.5) / _WORK_H * fh
        raw   = (raw_x, raw_y)

        if not self._kalman.initialized:
            self._kalman.start(raw)
        self._kalman.predict()
        smooth = self._kalman.correct(raw)

        self._traverse = {
            "valid":    True,
            "point":    smooth,
            "raw":      raw,
            "openness": float(clear[py, px]),
            "margin":   mx_val - mean_s,
        }

    # --------------------------------------------------------------- snapshot

    def snapshot(self) -> dict | None:
        """Serialisable dict for the result pipeline (display thread reads this)."""
        if self._sectors is None or self._traverse is None:
            return None
        return {
            "sectors":  self._sectors.copy(),
            "traverse": dict(self._traverse),
            "backend":  self._backend,
        }


# ----------------------------------------------------------------- rendering

def draw_depth_overlay(frame: np.ndarray, snap: dict) -> None:
    """Draw depth heat grid + corridor arrow onto frame in-place."""
    if snap is None:
        return
    sectors  = snap.get("sectors")
    traverse = snap.get("traverse", {})
    backend  = snap.get("backend", "")

    fh, fw = frame.shape[:2]

    if sectors is not None:
        cw = fw // _GRID_COLS
        ch = fh // _GRID_ROWS
        for r in range(_GRID_ROWS):
            for c in range(_GRID_COLS):
                s    = float(sectors[r, c])
                x0   = c * cw;  y0 = r * ch
                col  = (0, int(s * 220), int((1.0 - s) * 220))
                roi  = frame[y0:y0+ch, x0:x0+cw]
                over = np.full_like(roi, col)
                cv2.addWeighted(roi, 0.75, over, 0.25, 0, roi)
                cv2.rectangle(frame, (x0, y0), (x0+cw, y0+ch), (200, 200, 200), 1)
                cv2.putText(frame, f"{s:.2f}", (x0+4, y0+ch-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
                            cv2.LINE_AA)

    if not traverse.get("valid"):
        return

    decisive = traverse["margin"] >= _MIN_MARGIN
    col      = (128, 255, 0) if decisive else (0, 200, 255)   # green : amber

    pt  = traverse["point"]
    tgt = (int(pt[0]), int(pt[1]))
    ctr = (fw // 2, fh // 2)
    dx, dy = tgt[0] - ctr[0], tgt[1] - ctr[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length > 4:
        tip = max(length - 18, length * 0.6)
        end = (
            int(round(ctr[0] + dx / length * tip)),
            int(round(ctr[1] + dy / length * tip)),
        )
        cv2.arrowedLine(frame, ctr, end, col,
                        3 if decisive else 2, cv2.LINE_AA, tipLength=0.28)
    cv2.circle(frame, tgt, 6, col, 2 if decisive else 1, cv2.LINE_AA)

    label = (f"depth: {backend}   "
             f"{'TRAVERSE' if decisive else 'SCANNING'}  "
             f"open {traverse['openness']*100:.0f}%")
    cv2.putText(frame, label, (8, fh - 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
