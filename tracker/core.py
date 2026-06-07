import cv2
import torch
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
    """
    Embedding-driven lock-on tracker.

    CSRT provides a cheap motion prior each frame; the DINOv2 anchor embedding
    then *decides* the final box by scoring candidate windows around that prior
    and snapping to the best match. The anchor (the original ROI vector) is
    permanent and never overwritten, so the target identity cannot drift onto a
    neighbouring object over time.
    """

    # --- search geometry (candidate windows scored every frame) ---------------
    SEARCH_RADIUS = 0.6     # search +/- this fraction of the box size per axis
    GRID          = 5       # GRID x GRID spatial offsets
    SCALES        = (0.85, 1.0, 1.18)   # multi-scale candidates

    # --- decision thresholds (cosine similarity vs anchor) --------------------
    SIM_CONFIRM = 0.55      # accept & treat as solid lock
    SIM_WARNING = 0.42      # follow but flag as shaky (no bank update)
    SIM_LOST    = 0.42      # below warning counts as a bad frame

    STREAK_LIMIT = 8        # consecutive bad frames before dropping the lock

    # --- appearance bank (robustness without drift) ---------------------------
    BANK_MAX     = 5        # max stored views (anchor always at index 0)
    BANK_ADD_SIM = 0.72     # only bank a view this similar to the anchor
    ANCHOR_WEIGHT = 0.6     # blend: score = w*anchor_sim + (1-w)*bank_best_sim

    # --- recovery -------------------------------------------------------------
    LOST_SEARCH_RADIUS = 2.0   # widen search this much when struggling

    def __init__(self, embedder: DINOv2Embedder):
        self.embedder = embedder
        self.state = State.IDLE
        self.bbox = None          # (x, y, w, h) ints — current position
        self.similarity = 1.0

        self._csrt = None
        self._anchor = None       # permanent original embedding (384,)
        self._bank = None         # (K, 384) tensor incl. anchor at row 0
        self._bad_streak = 0

    # ------------------------------------------------------------------ public

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        """Lock onto the region defined by bbox (x, y, w, h)."""
        x, y, w, h = self._clamp(frame, bbox)
        if w < 10 or h < 10:
            return False
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return False

        self._anchor = self.embedder.embed(crop)         # never changes
        self._bank = self._anchor.unsqueeze(0).clone()   # (1, 384)
        self._csrt = _make_csrt()
        self._csrt.init(frame, (x, y, w, h))
        self.bbox = (x, y, w, h)
        self._bad_streak = 0
        self.similarity = 1.0
        self.state = State.LOCKED
        return True

    def update(self, frame: np.ndarray):
        """
        Advance one frame. Returns (State, bbox_or_None, similarity_or_None).
        """
        if self.state != State.LOCKED:
            return self.state, None, None

        # 1. CSRT motion prior (cheap). On failure, coast from last box.
        ok, raw = self._csrt.update(frame)
        prior = tuple(int(v) for v in raw) if ok else self.bbox

        # 2. Embedding-driven localisation around the prior.
        radius = self.SEARCH_RADIUS
        if self._bad_streak >= self.STREAK_LIMIT // 2:
            radius = self.LOST_SEARCH_RADIUS    # widen when struggling

        best_box, best_sim = self._search(frame, prior, radius)

        if best_box is None:
            self._bad_streak += 1
            self.similarity = 0.0
            if self._bad_streak >= self.STREAK_LIMIT:
                self._lose()
                return self.state, None, 0.0
            return self.state, self.bbox, self.similarity

        self.similarity = best_sim

        if best_sim >= self.SIM_WARNING:
            # Accept the embedding's choice; re-seat CSRT there.
            self.bbox = best_box
            self._reseat_csrt(frame, best_box)
            self._bad_streak = 0
            # Grow the appearance bank only on very confident, anchor-consistent views.
            if best_sim >= self.BANK_ADD_SIM:
                self._maybe_bank(frame, best_box)
        else:
            # Too weak: hold position, count as bad, keep searching next frame.
            self._bad_streak += 1
            if self._bad_streak >= self.STREAK_LIMIT:
                self._lose()
                return self.state, None, best_sim

        return self.state, self.bbox, self.similarity

    def reset(self):
        self.state = State.IDLE
        self._csrt = None
        self.bbox = None
        self._anchor = None
        self._bank = None
        self.similarity = 1.0
        self._bad_streak = 0

    # ----------------------------------------------------------------- private

    def _search(self, frame, prior, radius):
        """
        Generate candidate windows around `prior`, embed them in one GPU batch,
        score each against the permanent anchor (blended with the appearance
        bank), and return (best_box, best_sim).
        """
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
                    nx = int(cx + ox * pw - bw / 2.0)
                    ny = int(cy + oy * ph - bh / 2.0)
                    x, y, w, h = self._clamp(frame, (nx, ny, bw, bh))
                    if w < 10 or h < 10:
                        continue
                    crop = frame[y:y + h, x:x + w]
                    if crop.size == 0:
                        continue
                    boxes.append((x, y, w, h))
                    crops.append(crop)

        if not crops:
            return None, 0.0

        embs = self.embedder.embed_batch(crops)          # (N, 384)

        # Primary score: similarity to the permanent anchor.
        anchor_sim = embs @ self._anchor                 # (N,)

        # Secondary: best match against the appearance bank (incl. anchor).
        bank_sim = (embs @ self._bank.T).max(dim=1).values   # (N,)

        score = self.ANCHOR_WEIGHT * anchor_sim + (1.0 - self.ANCHOR_WEIGHT) * bank_sim
        best = int(torch.argmax(score).item())
        # Report the anchor similarity as the user-facing confidence.
        return boxes[best], float(anchor_sim[best].item())

    def _maybe_bank(self, frame, box):
        """Add a confident, anchor-consistent view to the appearance bank."""
        if self._bank.shape[0] >= self.BANK_MAX:
            return
        x, y, w, h = box
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return
        emb = self.embedder.embed(crop)
        # Guard: must be close to the anchor and not redundant with existing views.
        if float(emb @ self._anchor) < self.BANK_ADD_SIM:
            return
        if float((self._bank @ emb).max()) > 0.97:
            return
        self._bank = torch.cat([self._bank, emb.unsqueeze(0)], dim=0)

    def _reseat_csrt(self, frame, box):
        self._csrt = _make_csrt()
        self._csrt.init(frame, box)

    def _clamp(self, frame, bbox):
        x, y, w, h = (int(v) for v in bbox)
        fh, fw = frame.shape[:2]
        x = max(0, min(x, fw - 1))
        y = max(0, min(y, fh - 1))
        w = max(0, min(w, fw - x))
        h = max(0, min(h, fh - y))
        return x, y, w, h

    def _lose(self):
        self.state = State.IDLE
        self._bad_streak = 0
