import cv2
import torch
import numpy as np
from collections import deque
from enum import Enum, auto
from .embedding import DINOv2Embedder
from .settings import Settings
from .trackers import make_backend
from .motion import KalmanCenter


class State(Enum):
    IDLE      = auto()
    LOCKED    = auto()
    SEARCHING = auto()


class LockOnTracker:
    """
    Two-tier tracking:
      • CSRT every frame  (~0.5 ms, CPU)  — smooth motion
      • DINOv2 every check_interval frames — identity verify + correction

    SEARCHING spreads the full-frame scan across multiple frames
    (SCAN_BATCH positions per frame) so the video loop never blocks.
    """

    GRID          = 3
    SCALES        = (0.85, 1.0, 1.18)

    BANK_MAX      = 5
    BANK_ADD_SIM  = 0.72
    ANCHOR_WEIGHT = 0.6

    LOST_RADIUS   = 2.0
    SCAN_BATCH    = 64          # positions processed per frame during SEARCHING
    SEARCH_TIMEOUT = 300        # frames before giving up (~10 s at 30 fps)

    MOTION_HISTORY  = 14        # filtered centers kept for the trail

    _SIM_CONFIRM   = 0.55
    _SIM_WARNING   = 0.42
    _STREAK_LIMIT  = 8
    _SEARCH_RADIUS = 0.6
    _CHECK_INTERVAL = 6
    _BACKEND        = "CSRT"
    _PREDICT_HORIZON = 8

    def __init__(self, embedder: DINOv2Embedder, settings: Settings = None):
        self.embedder   = embedder
        self._s         = settings
        self.state      = State.IDLE
        self.bbox       = None
        self.similarity = 1.0

        # Motion vector state (Kalman-filtered)
        self._centers          = deque(maxlen=self.MOTION_HISTORY)
        self._kalman           = KalmanCenter()
        self.velocity          = (0.0, 0.0)
        self.predicted_center  = None

        self._tracker        = None
        self._anchor         = None
        self._bank           = None
        self._bad_streak     = 0
        self._frame_n        = 0
        self._search_frame   = 0
        self._scan_positions = None   # list of (x,y,w,h) for current sweep
        self._scan_idx       = 0
        self._scan_best      = (None, 0.0)
        self._backend_fallback = None  # set when a backend fails to construct

    # ------------------------------------------------------------------ props

    @property
    def _sim_confirm(self):
        return self._s.sim_confirm if self._s else self._SIM_CONFIRM

    @property
    def _sim_warning(self):
        return self._s.sim_warning if self._s else self._SIM_WARNING

    @property
    def _streak_limit(self):
        return self._s.streak_limit if self._s else self._STREAK_LIMIT

    @property
    def _search_radius(self):
        return self._s.search_radius if self._s else self._SEARCH_RADIUS

    @property
    def _check_interval(self):
        return self._s.check_interval if self._s else self._CHECK_INTERVAL

    @property
    def _backend(self):
        return self._s.tracker_backend if self._s else self._BACKEND

    @property
    def _predict_horizon(self):
        return self._s.predict_horizon if self._s else self._PREDICT_HORIZON

    def center_trail(self):
        return list(self._centers)

    def _make_backend(self):
        """Build the configured backend; fall back to CSRT if it can't load
        (e.g. ViT model download failed or opencv lacks TrackerVit)."""
        name = self._backend_fallback or self._backend
        try:
            return make_backend(name)
        except Exception as e:
            if name != "CSRT":
                print(f"[tracker] backend '{name}' unavailable ({e}); "
                      f"falling back to CSRT.")
                self._backend_fallback = "CSRT"
                return make_backend("CSRT")
            raise

    # ------------------------------------------------------------------ public

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        x, y, w, h = self._clamp(frame, bbox)
        if w < 10 or h < 10:
            return False
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return False

        self._anchor  = self.embedder.embed(crop)
        self._bank    = self._anchor.unsqueeze(0).clone()
        self._backend_fallback = None      # re-try the configured backend
        self._tracker = self._make_backend()
        self._tracker.init(frame, (x, y, w, h))
        self.bbox          = (x, y, w, h)
        self._bad_streak   = 0
        self._frame_n      = 0
        self._search_frame = 0
        self._scan_positions = None
        self.similarity    = 1.0
        self._centers.clear()
        self._kalman = KalmanCenter()
        self._kalman.start((x + w / 2.0, y + h / 2.0))
        self.velocity         = (0.0, 0.0)
        self.predicted_center = None
        self._update_motion()
        self.state = State.LOCKED
        return True

    def update(self, frame: np.ndarray):
        if self.state == State.LOCKED:
            return self._update_locked(frame)
        if self.state == State.SEARCHING:
            return self._update_searching(frame)
        return self.state, None, None

    def reset(self):
        self.state           = State.IDLE
        self._tracker        = None
        self.bbox            = None
        self._anchor         = None
        self._bank           = None
        self.similarity      = 1.0
        self._bad_streak     = 0
        self._frame_n        = 0
        self._search_frame   = 0
        self._scan_positions = None
        self._centers.clear()
        self.velocity         = (0.0, 0.0)
        self.predicted_center = None

    # --------------------------------------------------------------- LOCKED

    def _update_locked(self, frame):
        ok, raw = self._tracker.update(frame)
        if not ok:
            self._enter_searching()
            return self.state, self.bbox, self.similarity

        self.bbox = tuple(int(v) for v in raw)
        self._frame_n += 1

        if self._frame_n % self._check_interval != 0:
            self._update_motion()
            return self.state, self.bbox, self.similarity

        sim = self._single_sim(frame, self.bbox)
        self.similarity = sim

        if sim >= self._sim_warning:
            self._bad_streak = 0
            if sim >= self.BANK_ADD_SIM:
                self._maybe_bank(frame, self.bbox)
        else:
            best_box, best_sim = self._local_search(frame, self.bbox, self._search_radius)
            if best_box and best_sim >= self._sim_warning:
                self.bbox = best_box
                self._reseat_csrt(frame, best_box)
                self.similarity = best_sim
                self._bad_streak = 0
            else:
                # Target not found near the box — coast along predicted motion
                # so a brief occlusion or fast move doesn't immediately lose it.
                self._bad_streak += 1
                self._coast(frame)
                if self._bad_streak >= self._streak_limit:
                    self._enter_searching()

        self._update_motion()
        return self.state, self.bbox, self.similarity

    def _update_motion(self):
        """Kalman-filter the box center → smoothed velocity + prediction."""
        if self.bbox is None:
            return
        x, y, w, h = self.bbox
        c = (x + w / 2.0, y + h / 2.0)

        if not self._kalman.initialized:
            self._kalman.start(c)
        self._kalman.predict()
        fx, fy = self._kalman.correct(c)        # filtered center

        self._centers.append((fx, fy))
        self.velocity         = self._kalman.velocity
        self.predicted_center = self._kalman.project(self._predict_horizon)

    def _coast(self, frame):
        """Move the box along the Kalman velocity when a measurement is missed."""
        if self.bbox is None or not self._kalman.initialized:
            return
        px, py = self._kalman.project(1)        # one step ahead
        x, y, w, h = self.bbox
        nx = int(px - w / 2.0)
        ny = int(py - h / 2.0)
        nx, ny, w, h = self._clamp(frame, (nx, ny, w, h))
        if w >= 10 and h >= 10:
            self.bbox = (nx, ny, w, h)
            self._reseat_csrt(frame, self.bbox)

    # ------------------------------------------------------------- SEARCHING
    # The full-frame sweep is spread across frames: SCAN_BATCH positions per
    # frame, so the loop never blocks for more than one GPU batch (~2 ms).

    def _update_searching(self, frame):
        self._search_frame += 1

        if self._search_frame > self.SEARCH_TIMEOUT:
            self.state = State.IDLE
            return self.state, None, 0.0

        # Coast the prediction forward so the motion arrow keeps moving and the
        # scan is steered toward where the object is expected to reappear.
        if self._kalman.initialized:
            self.predicted_center = self._kalman.project(self._search_frame)

        # Start a fresh sweep when the previous one is exhausted.
        if self._scan_positions is None:
            self._scan_positions = self._gen_scan_positions(frame)
            self._scan_idx  = 0
            self._scan_best = (None, 0.0)

        # Process one batch of candidate positions from the current frame.
        end        = min(self._scan_idx + self.SCAN_BATCH, len(self._scan_positions))
        batch_pos  = self._scan_positions[self._scan_idx:end]
        self._scan_idx = end

        fh, fw = frame.shape[:2]
        boxes, crops = [], []
        for (bx, by, bw, bh) in batch_pos:
            crop = frame[by:min(fh, by+bh), bx:min(fw, bx+bw)]
            if crop.size > 0:
                boxes.append((bx, by, bw, bh))
                crops.append(crop)

        if crops:
            embs = self.embedder.embed_batch(crops)
            sims = embs @ self._anchor
            idx  = int(torch.argmax(sims).item())
            if float(sims[idx]) > self._scan_best[1]:
                self._scan_best = (boxes[idx], float(sims[idx]))

        # Sweep complete — check result and start a new sweep next frame.
        if self._scan_idx >= len(self._scan_positions):
            self._scan_positions = None          # triggers new sweep next frame
            best_box, best_sim = self._scan_best
            self.similarity = best_sim

            if best_box and best_sim >= self._sim_confirm:
                self.bbox = best_box
                self._reseat_csrt(frame, best_box)
                self._bad_streak   = 0
                self._frame_n      = 0
                self._search_frame = 0
                self.state = State.LOCKED

        return self.state, self.bbox, self.similarity

    # ----------------------------------------------------------------- search

    def _gen_scan_positions(self, frame):
        """
        All (x, y, w, h) candidate positions for a full-frame sweep, ordered
        nearest-first to the Kalman-predicted reappearance point so recovery
        is fastest where the object is most likely to come back.
        """
        fh, fw = frame.shape[:2]
        _, _, tw, th = self.bbox
        sx = max(tw // 3, 20)
        sy = max(th // 3, 20)
        positions = []
        for s in (0.8, 1.0, 1.25):
            bw, bh = int(tw * s), int(th * s)
            if bw < 10 or bh < 10:
                continue
            for x in range(0, fw - bw + 1, sx):
                for y in range(0, fh - bh + 1, sy):
                    positions.append((x, y, bw, bh))

        # Seed point: predicted reappearance, else last known box center.
        if self.predicted_center is not None:
            pcx, pcy = self.predicted_center
        else:
            bx, by, bw, bh = self.bbox
            pcx, pcy = bx + bw / 2.0, by + bh / 2.0

        positions.sort(key=lambda b:
                       (b[0] + b[2] / 2.0 - pcx) ** 2 +
                       (b[1] + b[3] / 2.0 - pcy) ** 2)
        return positions

    def _single_sim(self, frame, bbox) -> float:
        x, y, w, h = bbox
        fh, fw = frame.shape[:2]
        crop = frame[max(0,y):min(fh,y+h), max(0,x):min(fw,x+w)]
        if crop.size == 0:
            return 0.0
        return self.embedder.similarity(self._anchor, self.embedder.embed(crop))

    def _local_search(self, frame, prior, radius):
        fh, fw = frame.shape[:2]
        px, py, pw, ph = prior
        cx, cy = px + pw / 2.0, py + ph / 2.0

        offs = np.linspace(-radius, radius, self.GRID)
        boxes, crops = [], []
        for s in self.SCALES:
            bw, bh = int(pw * s), int(ph * s)
            if bw < 10 or bh < 10:
                continue
            for ox in offs:
                for oy in offs:
                    x, y, w, h = self._clamp(
                        frame,
                        (int(cx + ox * pw - bw / 2.0),
                         int(cy + oy * ph - bh / 2.0),
                         bw, bh)
                    )
                    if w < 10 or h < 10:
                        continue
                    crop = frame[y:y + h, x:x + w]
                    if crop.size == 0:
                        continue
                    boxes.append((x, y, w, h))
                    crops.append(crop)

        if not crops:
            return None, 0.0

        embs     = self.embedder.embed_batch(crops)
        anchor_s = embs @ self._anchor
        bank_s   = (embs @ self._bank.T).max(dim=1).values
        score    = self.ANCHOR_WEIGHT * anchor_s + (1 - self.ANCHOR_WEIGHT) * bank_s
        best     = int(torch.argmax(score).item())
        return boxes[best], float(anchor_s[best].item())

    # ----------------------------------------------------------------- helpers

    def _maybe_bank(self, frame, box):
        if self._bank.shape[0] >= self.BANK_MAX:
            return
        x, y, w, h = box
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return
        emb = self.embedder.embed(crop)
        if float(emb @ self._anchor) < self.BANK_ADD_SIM:
            return
        if float((self._bank @ emb).max()) > 0.97:
            return
        self._bank = torch.cat([self._bank, emb.unsqueeze(0)], dim=0)

    def _reseat_csrt(self, frame, box):
        self._tracker = self._make_backend()
        self._tracker.init(frame, box)

    def _enter_searching(self):
        self.state           = State.SEARCHING
        self._bad_streak     = 0
        self._search_frame   = 0
        self._scan_positions = None

    def _clamp(self, frame, bbox):
        x, y, w, h = (int(v) for v in bbox)
        fh, fw = frame.shape[:2]
        x = max(0, min(x, fw - 1))
        y = max(0, min(y, fh - 1))
        w = max(0, min(w, fw - x))
        h = max(0, min(h, fh - y))
        return x, y, w, h
