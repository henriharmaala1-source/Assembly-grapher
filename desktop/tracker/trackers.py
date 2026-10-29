import os
import urllib.request

import cv2
import numpy as np


def _make_cv_tracker(kind: str):
    """CSRT / KCF — lives in cv2.legacy on opencv-contrib >= 4.5."""
    name   = f"Tracker{kind}_create"
    legacy = getattr(cv2, "legacy", None)
    if legacy is not None and hasattr(legacy, name):
        return getattr(legacy, name)()
    if hasattr(cv2, name):
        return getattr(cv2, name)()
    raise RuntimeError(
        f"cv2.{name} not found — opencv-contrib-python is required:\n"
        "  pip uninstall opencv-python opencv-contrib-python -y\n"
        "  pip install opencv-contrib-python"
    )


# ----------------------------------------------------- ViT transformer tracker
# OpenCV's TrackerVit is a dedicated single-object transformer tracker (ViT
# backbone, direct box regression). It ships in opencv-contrib but needs a small
# ONNX model, fetched once from the OpenCV model zoo and cached under ./models.

_VIT_MODEL_URL = (
    "https://media.githubusercontent.com/media/opencv/opencv_zoo/main/"
    "models/object_tracking_vittrack/object_tracking_vittrack_2023sep.onnx"
)
_MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
_VIT_MODEL_PATH = os.path.join(_MODELS_DIR, "object_tracking_vittrack_2023sep.onnx")


def _ensure_vit_model() -> str:
    """Download the ViTTrack ONNX once; return the local path."""
    if os.path.exists(_VIT_MODEL_PATH) and os.path.getsize(_VIT_MODEL_PATH) > 0:
        return _VIT_MODEL_PATH
    os.makedirs(_MODELS_DIR, exist_ok=True)
    print("Downloading ViTTrack model (~0.7 MB)…")
    tmp = _VIT_MODEL_PATH + ".part"
    try:
        urllib.request.urlretrieve(_VIT_MODEL_URL, tmp)
        os.replace(tmp, _VIT_MODEL_PATH)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    print(f"Saved ViTTrack model to {_VIT_MODEL_PATH}")
    return _VIT_MODEL_PATH


def _make_vit_tracker():
    """
    cv2.TrackerVit with the zoo model. Its init()/update() signature already
    matches the OpenCV trackers, so it slots straight into the factory.

    The ONNX net runs on OpenCV's DNN backend (CPU) — it's tiny and fast, and
    keeps the GPU free for DINOv2. Build opencv with CUDA DNN to offload it.
    """
    if not hasattr(cv2, "TrackerVit"):
        raise RuntimeError(
            "cv2.TrackerVit is unavailable. Upgrade the tracking module:\n"
            "  pip install -U opencv-contrib-python   (>= 4.8)"
        )
    path = _ensure_vit_model()
    params = cv2.TrackerVit_Params()
    params.net = path
    return cv2.TrackerVit_create(params)


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
    FB_MAX_ERR = 1.5   # px — forward-backward error above this rejects a point

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

        # Forward-backward check: track the results back to the previous frame
        # and reject points that don't land where they started. Points that
        # slid onto the background still "track" — this is what catches them.
        back, st_b, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, nxt, None, **self.LK_PARAMS
        )
        if back is None or st_b is None:
            self._prev_gray = gray
            self._pts = None
            return False, tuple(self._box)

        fb_err = np.linalg.norm(
            self._pts.reshape(-1, 2) - back.reshape(-1, 2), axis=1)
        valid = (st.reshape(-1).astype(bool)
                 & st_b.reshape(-1).astype(bool)
                 & (fb_err < self.FB_MAX_ERR))

        good_old = self._pts.reshape(-1, 2)[valid]
        good_new = nxt.reshape(-1, 2)[valid]
        if len(good_new) < self.MIN_POINTS:
            self._prev_gray = gray
            self._pts = None
            return False, tuple(self._box)

        disp = good_new - good_old
        dx, dy = float(np.median(disp[:, 0])), float(np.median(disp[:, 1]))

        # Scale estimation: compare median point-spread before and after.
        # When the object approaches or recedes the spread ratio gives the
        # zoom factor. Cap to ±18% per frame to prevent runaway box drift.
        if len(good_new) >= 8:
            center_old = good_old.mean(axis=0)
            center_new = good_new.mean(axis=0)
            spread_old = float(np.median(np.linalg.norm(good_old - center_old, axis=1)))
            spread_new = float(np.median(np.linalg.norm(good_new - center_new, axis=1)))
            if spread_old > 2.0:
                scale = float(np.clip(spread_new / (spread_old + 1e-6), 0.82, 1.18))
                cx = self._box[0] + self._box[2] / 2.0 + dx
                cy = self._box[1] + self._box[3] / 2.0 + dy
                w_new = max(20, int(self._box[2] * scale))
                h_new = max(20, int(self._box[3] * scale))
                self._box[0] = int(cx - w_new / 2)
                self._box[1] = int(cy - h_new / 2)
                self._box[2] = w_new
                self._box[3] = h_new
            else:
                self._box[0] += int(round(dx))
                self._box[1] += int(round(dy))
        else:
            self._box[0] += int(round(dx))
            self._box[1] += int(round(dy))

        self._prev_gray = gray
        self._pts = good_new.reshape(-1, 1, 2)
        return True, tuple(self._box)


def make_backend(name: str):
    """Factory: 'ViT' | 'CSRT' | 'KCF' | 'Optical Flow'."""
    if name == "ViT":
        return _make_vit_tracker()
    if name == "Optical Flow":
        return OpticalFlowTracker()
    if name == "KCF":
        return _make_cv_tracker("KCF")
    return _make_cv_tracker("CSRT")
