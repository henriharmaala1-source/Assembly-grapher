import cv2
import numpy as np
from enum import Enum, auto
from .embedding import DINOv2Embedder


def _make_csrt():
    # OpenCV 4.5+: trackers live under cv2.legacy; older contrib has them at top level.
    try:
        return cv2.legacy.TrackerCSRT_create()
    except AttributeError:
        return cv2.TrackerCSRT_create()


class State(Enum):
    IDLE = auto()
    LOCKED = auto()


class LockOnTracker:
    # Embedding check cadence
    CHECK_INTERVAL = 5      # every N frames

    # Similarity thresholds
    SIM_CONFIRM = 0.60      # above → solid lock
    SIM_WARNING = 0.45      # above → shaky lock (yellow)
    SIM_LOST    = 0.35      # below this counts as a bad frame

    # Consecutive bad checks before attempting re-acquisition
    STREAK_LIMIT = 3

    # How much to expand the search window for re-acquisition (factor per side)
    EXPAND = 1.5

    def __init__(self, embedder: DINOv2Embedder):
        self.embedder = embedder
        self.state = State.IDLE
        self.bbox = None          # (x, y, w, h) ints, current tracked position
        self.similarity = 1.0    # last computed cosine similarity

        self._csrt = None
        self._target_emb = None
        self._frame_n = 0
        self._bad_streak = 0

    # ------------------------------------------------------------------ public

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        """Lock onto the region defined by bbox (x, y, w, h)."""
        x, y, w, h = (int(v) for v in bbox)
        fh, fw = frame.shape[:2]
        x = max(0, x); y = max(0, y)
        w = min(fw - x, w); h = min(fh - y, h)
        if w < 10 or h < 10:
            return False
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return False

        self._target_emb = self.embedder.embed(crop)
        self._csrt = _make_csrt()
        self._csrt.init(frame, (x, y, w, h))
        self.bbox = (x, y, w, h)
        self._frame_n = 0
        self._bad_streak = 0
        self.similarity = 1.0
        self.state = State.LOCKED
        return True

    def update(self, frame: np.ndarray):
        """
        Advance the tracker by one frame.
        Returns (State, bbox_or_None, similarity_or_None).
        """
        if self.state != State.LOCKED:
            return self.state, None, None

        ok, raw = self._csrt.update(frame)
        if not ok:
            self._lose()
            return self.state, None, 0.0

        self.bbox = tuple(int(v) for v in raw)
        self._frame_n += 1

        # Periodic identity check via DINOv2 similarity
        if self._frame_n % self.CHECK_INTERVAL == 0:
            sim = self._crop_sim(frame, self.bbox)
            self.similarity = sim

            if sim < self.SIM_LOST:
                self._bad_streak += 1
            else:
                self._bad_streak = 0

            if self._bad_streak >= self.STREAK_LIMIT:
                if not self._reacquire(frame):
                    self._lose()
                    return self.state, None, sim

        return self.state, self.bbox, self.similarity

    def reset(self):
        self.state = State.IDLE
        self._csrt = None
        self.bbox = None
        self._target_emb = None
        self.similarity = 1.0
        self._bad_streak = 0

    # ----------------------------------------------------------------- private

    def _crop_sim(self, frame: np.ndarray, bbox: tuple) -> float:
        x, y, w, h = bbox
        fh, fw = frame.shape[:2]
        x1 = max(0, x);  y1 = max(0, y)
        x2 = min(fw, x + w); y2 = min(fh, y + h)
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return 0.0
        return self.embedder.similarity(self._target_emb, self.embedder.embed(crop))

    def _reacquire(self, frame: np.ndarray) -> bool:
        """
        Search an expanded window around the last known position.
        Re-inits CSRT centred on the expanded crop if similarity recovers.
        """
        x, y, w, h = self.bbox
        fh, fw = frame.shape[:2]
        mx = int(w * self.EXPAND / 2)
        my = int(h * self.EXPAND / 2)
        sx = max(0, x - mx);    sy = max(0, y - my)
        ex = min(fw, x + w + mx); ey = min(fh, y + h + my)
        crop = frame[sy:ey, sx:ex]
        if crop.size == 0:
            return False

        sim = self.embedder.similarity(self._target_emb, self.embedder.embed(crop))
        if sim < self.SIM_CONFIRM:
            return False

        # Re-centre tracker inside the found window
        nx = max(0, min(fw - w, sx + (ex - sx) // 2 - w // 2))
        ny = max(0, min(fh - h, sy + (ey - sy) // 2 - h // 2))
        self._csrt = _make_csrt()
        self._csrt.init(frame, (nx, ny, w, h))
        self.bbox = (nx, ny, w, h)
        self._bad_streak = 0
        self.similarity = sim
        return True

    def _lose(self):
        self.state = State.IDLE
        self._bad_streak = 0
