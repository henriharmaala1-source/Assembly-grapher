"""
SAM 2 tracking engine — an alternative *philosophy* to the DINOv2 hybrid.

Instead of (cheap box tracker + periodic DINOv2 verification + local search +
full-frame re-acquire scan), SAM 2 is prompted once with the drawn box and then
propagates a pixel-accurate mask through the video, with its own memory bank
that handles occlusion and re-appearance natively.

This class mirrors the public surface the pipeline/overlay use from
LockOnTracker, so it is a drop-in engine:

    state            : State            (IDLE / LOCKED / SEARCHING)
    init(frame, bbox) -> bool
    update(frame)     -> (state, bbox, sim)
    reset()
    center_trail()    -> [(cx, cy), ...]
    predicted_center  : (px, py) | None
    provides_mask     : True            (mask comes from SAM 2, not DINOv2)
    mask_crop()       -> float32 [0,1] map aligned to the current bbox | None

Requires the streaming SAM 2 build:
    pip install "git+https://github.com/Gy920/segment-anything-2-real-time.git"
plus a checkpoint (default: tiny). See README for setup.
"""

import os
from collections import deque

import cv2
import numpy as np

from .core import State
from .motion import KalmanCenter


DEFAULT_CFG  = os.environ.get("SAM2_CFG",  "configs/sam2.1/sam2.1_hiera_t.yaml")
DEFAULT_CKPT = os.environ.get("SAM2_CKPT", "./checkpoints/sam2.1_hiera_tiny.pt")


class SAM2Tracker:
    MOTION_HISTORY  = 14
    MIN_MASK_PIXELS = 60       # below this the target is considered absent
    SEARCH_TIMEOUT  = 300      # frames of empty mask before giving up
    BOX_PAD         = 2        # px padding around the mask bounding box

    def __init__(self, device: str = "cuda",
                 cfg: str = DEFAULT_CFG, ckpt: str = DEFAULT_CKPT):
        self.device = device
        self._predictor = _build_predictor(cfg, ckpt, device)
        self._autocast_dtype = _torch().bfloat16 if device == "cuda" else _torch().float32

        self.state            = State.IDLE
        self.bbox             = None
        self.similarity       = 1.0
        self.provides_mask    = True
        self.predicted_center = None
        self.velocity         = (0.0, 0.0)

        self._centers    = deque(maxlen=self.MOTION_HISTORY)
        self._kalman     = KalmanCenter()
        self._mask_crop  = None     # float32 [0,1], aligned to self.bbox
        self._lost_frame = 0

    # ------------------------------------------------------------------ public

    def center_trail(self):
        return list(self._centers)

    def mask_crop(self):
        return self._mask_crop

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        x, y, w, h = (int(v) for v in bbox)
        if w < 10 or h < 10:
            return False

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        torch = _torch()
        with torch.inference_mode(), torch.autocast(self.device, dtype=self._autocast_dtype):
            self._predictor.load_first_frame(rgb)
            box = [float(x), float(y), float(x + w), float(y + h)]
            _, _, mask_logits = self._predictor.add_new_prompt(
                frame_idx=0, obj_id=1, bbox=box)

        mask = self._logits_to_mask(mask_logits)
        if mask is None:
            return False

        self._reset_motion((x + w / 2.0, y + h / 2.0))
        self._apply_mask(frame, mask)
        self.state = State.LOCKED
        self._lost_frame = 0
        return True

    def update(self, frame: np.ndarray):
        if self.state == State.IDLE:
            return self.state, None, None

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        torch = _torch()
        with torch.inference_mode(), torch.autocast(self.device, dtype=self._autocast_dtype):
            _, mask_logits = self._predictor.track(rgb)

        mask = self._logits_to_mask(mask_logits)

        if mask is not None:
            # Object present — SAM 2 re-acquires on its own after occlusion.
            self._apply_mask(frame, mask)
            self.state       = State.LOCKED
            self._lost_frame = 0
        else:
            # Mask empty: coast the prediction; SAM 2 keeps searching via memory.
            self._mask_crop = None
            self.state = State.SEARCHING
            self._lost_frame += 1
            if self._kalman.initialized:
                self.predicted_center = self._kalman.project(self._lost_frame)
            if self._lost_frame > self.SEARCH_TIMEOUT:
                self.reset()
                return self.state, None, 0.0

        return self.state, self.bbox, self.similarity

    def reset(self):
        # Drop SAM 2's memory so the next prompt starts clean.
        try:
            self._predictor.reset_state()
        except Exception:
            pass
        self.state            = State.IDLE
        self.bbox             = None
        self.similarity       = 1.0
        self.predicted_center = None
        self.velocity         = (0.0, 0.0)
        self._mask_crop       = None
        self._lost_frame      = 0
        self._centers.clear()

    # ----------------------------------------------------------------- helpers

    def _logits_to_mask(self, mask_logits):
        """(N,1,H,W) logits -> bool mask for obj 0, or None if too small."""
        if mask_logits is None or mask_logits.shape[0] == 0:
            return None
        mask = (mask_logits[0, 0] > 0.0).cpu().numpy()
        if int(mask.sum()) < self.MIN_MASK_PIXELS:
            return None
        return mask

    def _apply_mask(self, frame, mask):
        """Derive bbox + confidence from the mask and update motion state."""
        ys, xs = np.where(mask)
        fh, fw = frame.shape[:2]
        x1 = max(0, int(xs.min()) - self.BOX_PAD)
        y1 = max(0, int(ys.min()) - self.BOX_PAD)
        x2 = min(fw, int(xs.max()) + 1 + self.BOX_PAD)
        y2 = min(fh, int(ys.max()) + 1 + self.BOX_PAD)
        w, h = x2 - x1, y2 - y1
        if w < 5 or h < 5:
            self._mask_crop = None
            return
        self.bbox = (x1, y1, w, h)

        # Confidence proxy: how solidly the mask fills its bounding box.
        self.similarity = float(mask[y1:y2, x1:x2].mean())

        # Mask crop aligned to bbox → reuses the existing attention overlay path.
        self._mask_crop = mask[y1:y2, x1:x2].astype(np.float32)

        # Motion: Kalman-filter the mask centroid.
        cx, cy = float(xs.mean()), float(ys.mean())
        if not self._kalman.initialized:
            self._kalman.start((cx, cy))
        self._kalman.predict()
        fx, fy = self._kalman.correct((cx, cy))
        self._centers.append((fx, fy))
        self.velocity         = self._kalman.velocity
        self.predicted_center = self._kalman.project(8)

    def _reset_motion(self, center):
        self._centers.clear()
        self._kalman = KalmanCenter()
        self._kalman.start(center)
        self.velocity         = (0.0, 0.0)
        self.predicted_center = None


# --------------------------------------------------------------- lazy imports
# Keep torch / sam2 out of import time so the app runs without them installed.

def _torch():
    import torch
    return torch


def _build_predictor(cfg: str, ckpt: str, device: str):
    try:
        from sam2.build_sam import build_sam2_camera_predictor
    except Exception as e:
        raise RuntimeError(
            "SAM 2 streaming build not found. Install it with:\n"
            '  pip install "git+https://github.com/Gy920/'
            'segment-anything-2-real-time.git"\n'
            f"(import error: {e})"
        )
    if not os.path.exists(ckpt):
        raise RuntimeError(
            f"SAM 2 checkpoint not found: {ckpt}\n"
            "Download the checkpoints (e.g. tiny) per the SAM 2 repo, or set "
            "SAM2_CKPT / SAM2_CFG to point at your files."
        )
    return build_sam2_camera_predictor(cfg, ckpt, device=device)
