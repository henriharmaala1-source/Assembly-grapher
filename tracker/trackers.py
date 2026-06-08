import cv2
import numpy as np


def _make_cv_tracker(kind: str):
    """CSRT / KCF from cv2.legacy (4.5+) with a top-level fallback."""
    legacy = getattr(cv2, "legacy", None)
    name = f"Tracker{kind}_create"
    if legacy is not None and hasattr(legacy, name):
        return getattr(legacy, name)()
    return getattr(cv2, name)()


class OpticalFlowTracker:
    """
    Sparse Lucas-Kanade tracker exposing the same interface as the OpenCV
    trackers: init(frame, bbox) and update(frame) -> (ok, (x, y, w, h)).

    Tracks a set of corner points inside the box and shifts the box by the
    median point displacement each frame. Box size is held fixed — scale
    changes are corrected by the DINOv2 anchor search in the main tracker.
    """

    LK_PARAMS = dict(
        winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    FEAT_PARAMS = dict(maxCorners=80, qualityLevel=0.01, minDistance=5, blockSize=7)
    MIN_POINTS = 8

    def __init__(self):
        self._prev_gray = None
        self._pts       = None
        self._box       = None

    def init(self, frame, bbox):
        x, y, w, h = (int(v) for v in bbox)
        self._box = [x, y, w, h]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._prev_gray = gray
        self._pts = self._detect(gray, self._box)
        return True

    def _detect(self, gray, box):
        x, y, w, h = box
        H, W = gray.shape[:2]
        mask = np.zeros((H, W), dtype=np.uint8)
        x0, y0 = max(0, x), max(0, y)
        x1, y1 = min(W, x + w), min(H, y + h)
        if x1 <= x0 or y1 <= y0:
            return None
        mask[y0:y1, x0:x1] = 255
        return cv2.goodFeaturesToTrack(gray, mask=mask, **self.FEAT_PARAMS)

    def update(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Re-seed if we have too few points to be reliable.
        if self._pts is None or len(self._pts) < self.MIN_POINTS:
            self._pts = self._detect(self._prev_gray, self._box)
        if self._pts is None or len(self._pts) < self.MIN_POINTS:
            self._prev_gray = gray
            return False, tuple(self._box)

        nxt, st, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._pts, None, **self.LK_PARAMS
        )
        if nxt is None or st is None:
            self._prev_gray = gray
            self._pts = None
            return False, tuple(self._box)

        st = st.reshape(-1).astype(bool)
        good_old = self._pts.reshape(-1, 2)[st]
        good_new = nxt.reshape(-1, 2)[st]
        if len(good_new) < self.MIN_POINTS:
            self._prev_gray = gray
            self._pts = None
            return False, tuple(self._box)

        disp = good_new - good_old
        dx, dy = float(np.median(disp[:, 0])), float(np.median(disp[:, 1]))
        self._box[0] += int(round(dx))
        self._box[1] += int(round(dy))

        self._prev_gray = gray
        self._pts = good_new.reshape(-1, 1, 2)
        return True, tuple(self._box)


def make_backend(name: str):
    """Factory: 'CSRT' | 'KCF' | 'Optical Flow'."""
    if name == "Optical Flow":
        return OpticalFlowTracker()
    if name == "KCF":
        return _make_cv_tracker("KCF")
    return _make_cv_tracker("CSRT")
