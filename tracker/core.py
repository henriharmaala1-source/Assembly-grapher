import cv2
import torch
import numpy as np
from enum import Enum, auto
from .embedding import DINOv2Embedder


def _make_csrt():
    try:
        return cv2.legacy.TrackerCSRT_create()
    except AttributeError:
        return cv2.TrackerCSRT_create()


class State(Enum):
    IDLE      = auto()
    LOCKED    = auto()
    SEARCHING = auto()   # target lost; scanning full frame to re-acquire


class LockOnTracker:
    # --- search geometry (candidate windows scored every frame) ---------------
    SEARCH_RADIUS = 0.6
    GRID          = 5
    SCALES        = (0.85, 1.0, 1.18)

    # --- decision thresholds --------------------------------------------------
    SIM_CONFIRM = 0.55
    SIM_WARNING = 0.42
    SIM_LOST    = 0.42

    STREAK_LIMIT = 8        # consecutive bad frames before entering SEARCHING

    # --- appearance bank ------------------------------------------------------
    BANK_MAX      = 5
    BANK_ADD_SIM  = 0.72
    ANCHOR_WEIGHT = 0.6

    # --- recovery (full-frame scan) -------------------------------------------
    LOST_SEARCH_RADIUS = 2.0
    SCAN_INTERVAL      = 3       # full-frame scan every N frames in SEARCHING
    SCAN_BATCH         = 64      # crops per GPU forward pass
    SEARCH_TIMEOUT     = 300     # frames (~10 s) before giving up and going IDLE

    def __init__(self, embedder: DINOv2Embedder):
        self.embedder  = embedder
        self.state     = State.IDLE
        self.bbox      = None
        self.similarity = 1.0

        self._csrt         = None
        self._anchor       = None   # permanent — never overwritten
        self._bank         = None
        self._bad_streak   = 0
        self._search_frame = 0      # frame counter in SEARCHING state

    # ------------------------------------------------------------------ public

    def init(self, frame: np.ndarray, bbox: tuple) -> bool:
        x, y, w, h = self._clamp(frame, bbox)
        if w < 10 or h < 10:
            return False
        crop = frame[y:y + h, x:x + w]
        if crop.size == 0:
            return False

        self._anchor = self.embedder.embed(crop)
        self._bank   = self._anchor.unsqueeze(0).clone()
        self._csrt   = _make_csrt()
        self._csrt.init(frame, (x, y, w, h))
        self.bbox      = (x, y, w, h)
        self._bad_streak   = 0
        self._search_frame = 0
        self.similarity    = 1.0
        self.state = State.LOCKED
        return True

    def update(self, frame: np.ndarray):
        """Returns (State, bbox_or_None, similarity_or_None)."""
        if self.state == State.LOCKED:
            return self._update_locked(frame)
        if self.state == State.SEARCHING:
            return self._update_searching(frame)
        return self.state, None, None

    def reset(self):
        self.state     = State.IDLE
        self._csrt     = None
        self.bbox      = None
        self._anchor   = None
        self._bank     = None
        self.similarity    = 1.0
        self._bad_streak   = 0
        self._search_frame = 0

    # ----------------------------------------------------------------- private

    def _update_locked(self, frame):
        ok, raw = self._csrt.update(frame)
        prior = tuple(int(v) for v in raw) if ok else self.bbox

        radius = self.SEARCH_RADIUS
        if self._bad_streak >= self.STREAK_LIMIT // 2:
            radius = self.LOST_SEARCH_RADIUS

        best_box, best_sim = self._local_search(frame, prior, radius)

        if best_box is None or best_sim < self.SIM_WARNING:
            self._bad_streak += 1
            self.similarity = best_sim or 0.0
            if self._bad_streak >= self.STREAK_LIMIT:
                self._enter_searching()
            return self.state, self.bbox, self.similarity

        self.similarity = best_sim
        self.bbox = best_box
        self._reseat_csrt(frame, best_box)
        self._bad_streak = 0
        if best_sim >= self.BANK_ADD_SIM:
            self._maybe_bank(frame, best_box)

        return self.state, self.bbox, self.similarity

    def _update_searching(self, frame):
        self._search_frame += 1

        if self._search_frame > self.SEARCH_TIMEOUT:
            self.state = State.IDLE
            return self.state, None, 0.0

        # Full-frame scan on every SCAN_INTERVAL frame; coast otherwise.
        if self._search_frame % self.SCAN_INTERVAL != 0:
            return self.state, self.bbox, self.similarity

        best_box, best_sim = self._full_frame_search(frame)
        self.similarity = best_sim

        if best_box is not None and best_sim >= self.SIM_CONFIRM:
            # Re-lock — preserve the anchor, just re-seat CSRT.
            self.bbox = best_box
            self._reseat_csrt(frame, best_box)
            self._bad_streak   = 0
            self._search_frame = 0
            self.state = State.LOCKED

        return self.state, self.bbox, self.similarity

    # ----------------------------------------------------------------- search

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

        embs      = self.embedder.embed_batch(crops)
        anchor_s  = embs @ self._anchor
        bank_s    = (embs @ self._bank.T).max(dim=1).values
        score     = self.ANCHOR_WEIGHT * anchor_s + (1.0 - self.ANCHOR_WEIGHT) * bank_s
        best      = int(torch.argmax(score).item())
        return boxes[best], float(anchor_s[best].item())

    def _full_frame_search(self, frame):
        """Slide a window across the entire frame to recover a lost target."""
        fh, fw = frame.shape[:2]
        _, _, tw, th = self.bbox

        stride_x = max(tw // 3, 20)
        stride_y = max(th // 3, 20)

        boxes, crops = [], []
        for s in (0.8, 1.0, 1.25):
            bw, bh = int(tw * s), int(th * s)
            if bw < 10 or bh < 10:
                continue
            for x in range(0, fw - bw + 1, stride_x):
                for y in range(0, fh - bh + 1, stride_y):
                    x2 = min(x + bw, fw); y2 = min(y + bh, fh)
                    crop = frame[y:y2, x:x2]
                    if crop.size == 0:
                        continue
                    boxes.append((x, y, bw, bh))
                    crops.append(crop)

        if not crops:
            return None, 0.0

        # Process in batches to stay within GPU memory.
        best_sim = 0.0
        best_box = None
        for i in range(0, len(crops), self.SCAN_BATCH):
            embs = self.embedder.embed_batch(crops[i:i + self.SCAN_BATCH])
            sims = embs @ self._anchor
            idx  = int(torch.argmax(sims).item())
            if float(sims[idx]) > best_sim:
                best_sim = float(sims[idx])
                best_box = boxes[i + idx]

        return best_box, best_sim

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
        self._csrt = _make_csrt()
        self._csrt.init(frame, box)

    def _enter_searching(self):
        self.state         = State.SEARCHING
        self._bad_streak   = 0
        self._search_frame = 0

    def _clamp(self, frame, bbox):
        x, y, w, h = (int(v) for v in bbox)
        fh, fw = frame.shape[:2]
        x = max(0, min(x, fw - 1))
        y = max(0, min(y, fh - 1))
        w = max(0, min(w, fw - x))
        h = max(0, min(h, fh - y))
        return x, y, w, h
