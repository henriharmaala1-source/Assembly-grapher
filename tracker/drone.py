"""
Drone-mode target tracker.

Simulates the tracking workload on drone hardware (Pi 5 / RK3588 / Jetson Nano):
  - Single click designates a fixed-size box — no bounding-box draw needed.
  - CSRT / KCF / Optical Flow backend only — no DINOv2, no SAM 2.
  - Same per-frame inference a sub-10W companion computer would run.

Used to measure tracker reliability (lock duration, loss rate) independent of
the heavy ML stack.
"""

import numpy as np
from .trackers import make_backend


class DroneTracker:
    """Fixed-box lightweight tracker for drone-hardware simulation."""

    _LOSS_TIMEOUT = 15  # frames before declaring target lost

    def __init__(self):
        self._backend_name = "CSRT"
        self._box_size = 80
        self._tracker   = None
        self._bbox       = None   # last known (x, y, w, h)
        self._age        = 0      # frames since last successful update
        self._loss_frames = 0     # consecutive failed updates
        self._total_losses = 0    # re-acquisition counter (for reliability stats)
        self.locked = False

    # ------------------------------------------------------------------ public

    def init(self, frame: np.ndarray, center: tuple,
             backend: str = None, box_size: int = None) -> bool:
        if backend is not None:
            self._backend_name = backend
        if box_size is not None:
            self._box_size = box_size

        fh, fw = frame.shape[:2]
        bs = self._box_size
        cx, cy = int(center[0]), int(center[1])
        x = max(0, min(fw - bs, cx - bs // 2))
        y = max(0, min(fh - bs, cy - bs // 2))
        bbox = (x, y, bs, bs)

        try:
            self._tracker = make_backend(self._backend_name)
            ok = self._tracker.init(frame, bbox)
        except Exception as exc:
            print(f"[drone] {self._backend_name} init failed ({exc}), falling back to CSRT")
            self._tracker = make_backend("CSRT")
            ok = self._tracker.init(frame, bbox)

        if ok:
            self._bbox        = bbox
            self._age         = 0
            self._loss_frames = 0
            self.locked       = True
        return ok

    def update(self, frame: np.ndarray):
        """Returns (ok, bbox). Must be called every frame once init'd."""
        if self._tracker is None or self._bbox is None:
            return False, None

        ok, raw = self._tracker.update(frame)
        if ok:
            self._bbox = tuple(int(v) for v in raw)
            self._age += 1
            if self._loss_frames > 0:
                self._total_losses += 1
            self._loss_frames = 0
            self.locked = True
        else:
            self._loss_frames += 1
            if self._loss_frames >= self._LOSS_TIMEOUT:
                self.locked = False

        return ok, self._bbox

    def reset(self):
        self._tracker      = None
        self._bbox         = None
        self._age          = 0
        self._loss_frames  = 0
        self._total_losses = 0
        self.locked        = False

    # ---------------------------------------------------------------- properties

    @property
    def bbox(self):
        return self._bbox

    @property
    def center(self):
        if self._bbox is None:
            return None
        x, y, w, h = self._bbox
        return (x + w // 2, y + h // 2)

    @property
    def age(self):
        return self._age

    @property
    def loss_frames(self):
        return self._loss_frames

    @property
    def total_losses(self):
        return self._total_losses
