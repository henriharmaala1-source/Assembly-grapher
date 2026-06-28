"""
Click-to-segment: point-prompt segmentation from a single mouse click.

Three backends:
  "SAM 2"      — SAM 2 image predictor (one-shot) or camera predictor
                 (continuous, uses the tracking engine's predictor)
  "MobileSAM"  — ~40 MB ViT-T, fast CPU or GPU
  "FastSAM-s"  — ~23 MB YOLO-based, fastest

Continuous mode: after a click the ViT tracker propagates the object
position each frame; the segmenter re-segments at the new center every
seg_interval frames.  For the SAM 2 backend, continuous = SAM2Tracker
video propagation (no ViT tracker needed — SAM 2 handles it internally).
"""

import os
import numpy as np
import cv2

DEFAULT_SAM2_CFG  = os.environ.get("SAM2_CFG",  "configs/sam2.1/sam2.1_hiera_t.yaml")
DEFAULT_SAM2_CKPT = os.environ.get("SAM2_CKPT", "./checkpoints/sam2.1_hiera_tiny.pt")
_MODELS_DIR       = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
_MOBILE_SAM_URL   = ("https://github.com/ChaoningZhang/MobileSAM/releases/"
                     "download/v0.0.1/mobile_sam.pt")
_MOBILE_SAM_PATH  = os.path.join(_MODELS_DIR, "mobile_sam.pt")


# ─────────────────────────────── helpers ──────────────────────────────────────

def _best_mask(masks, scores):
    best = int(np.argmax(scores))
    m = masks[best]
    return (m > 0.0).astype(bool) if m.dtype != bool else m.astype(bool)


def _ensure_model(url, dest):
    if os.path.exists(dest) and os.path.getsize(dest) > 1_000_000:
        return dest
    import urllib.request
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    print(f"Downloading {os.path.basename(dest)}…")
    tmp = dest + ".part"
    try:
        urllib.request.urlretrieve(url, tmp)
        os.replace(tmp, dest)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    return dest


def _mask_to_bbox(mask, frame_shape):
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return None
    fh, fw = frame_shape[:2]
    x1, y1 = max(0, int(xs.min()) - 2), max(0, int(ys.min()) - 2)
    x2, y2 = min(fw, int(xs.max()) + 3), min(fh, int(ys.max()) + 3)
    return (x1, y1, x2 - x1, y2 - y1)


# ─────────────────────────── one-shot backends ────────────────────────────────

class _SAM2ImageSegmenter:
    def __init__(self, device):
        if not os.path.exists(DEFAULT_SAM2_CKPT):
            raise RuntimeError(
                f"SAM 2 checkpoint not found: {DEFAULT_SAM2_CKPT}. See README.")
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as e:
            raise RuntimeError(f"SAM 2 not installed ({e}). See README.")
        import torch
        self._dtype  = torch.bfloat16 if device == "cuda" else torch.float32
        self._device = device
        model = build_sam2(DEFAULT_SAM2_CFG, DEFAULT_SAM2_CKPT,
                           device=device, apply_postprocessing=False)
        self._pred = SAM2ImagePredictor(model)

    def segment(self, frame_bgr, point_xy):
        import torch
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        x, y = int(point_xy[0]), int(point_xy[1])
        with torch.inference_mode(), torch.autocast(self._device, dtype=self._dtype):
            self._pred.set_image(rgb)
            masks, scores, _ = self._pred.predict(
                point_coords=np.array([[x, y]]),
                point_labels=np.array([1]),
                multimask_output=True,
            )
        return _best_mask(masks, scores)


class _MobileSAMSegmenter:
    def __init__(self, device):
        try:
            from mobile_sam import sam_model_registry, SamPredictor
        except ImportError as e:
            raise RuntimeError(
                f"MobileSAM not installed ({e}).\n"
                "  pip install git+https://github.com/ChaoningZhang/MobileSAM.git")
        path = _ensure_model(_MOBILE_SAM_URL, _MOBILE_SAM_PATH)
        sam  = sam_model_registry["vit_t"](checkpoint=path)
        sam.to(device).eval()
        self._pred   = SamPredictor(sam)
        self._device = device

    def segment(self, frame_bgr, point_xy):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        x, y = int(point_xy[0]), int(point_xy[1])
        self._pred.set_image(rgb)
        masks, scores, _ = self._pred.predict(
            point_coords=np.array([[x, y]]),
            point_labels=np.array([1]),
            multimask_output=True,
        )
        return _best_mask(masks, scores)


class _FastSAMSegmenter:
    def __init__(self, device):
        try:
            from ultralytics import FastSAM
        except ImportError as e:
            raise RuntimeError(
                f"ultralytics not installed ({e}).\n  pip install ultralytics")
        self._model  = FastSAM("FastSAM-s.pt")
        self._device = device

    def segment(self, frame_bgr, point_xy):
        x, y = int(point_xy[0]), int(point_xy[1])
        results = self._model(
            frame_bgr, points=[[x, y]], labels=[1],
            device=self._device, retina_masks=True,
            imgsz=1024, conf=0.4, iou=0.9, verbose=False,
        )
        if not results or results[0].masks is None:
            return np.zeros(frame_bgr.shape[:2], dtype=bool)
        data = results[0].masks.data.cpu().numpy()
        m = data[0] > 0.5
        fh, fw = frame_bgr.shape[:2]
        if m.shape != (fh, fw):
            m = cv2.resize(m.astype(np.uint8), (fw, fh),
                           interpolation=cv2.INTER_NEAREST).astype(bool)
        return m


# ─────────────────────── continuous segmenter wrapper ─────────────────────────
# Wraps any one-shot backend with a ViT-tracker position propagator.
# SAM 2 continuous takes a different path (SAM2Tracker.init_from_point).

class _ContinuousWrapper:
    """
    Keeps an image-segmenter + a ViT box-tracker alive between frames.
    segment_once()  → first-frame mask + inits tracker
    update()        → tracker step + optional re-segment; returns mask
    stop()          → tear down tracker
    """

    def __init__(self, segmenter, seg_interval: int = 3):
        self._seg        = segmenter
        self._interval   = seg_interval
        self._tracker    = None
        self._last_mask  = None
        self._last_pt    = None
        self._frame_n    = 0

    def start(self, frame, point):
        """Segment first frame, init ViT tracker on the resulting bbox."""
        mask = self._seg.segment(frame, point)
        if mask is None or not mask.any():
            return None
        self._last_mask = mask
        self._last_pt   = point
        bbox = _mask_to_bbox(mask, frame.shape)
        if bbox is None:
            return mask
        self._tracker  = _make_vit_tracker()
        if self._tracker is not None:
            self._tracker.init(frame, bbox)
        self._frame_n = 0
        return mask

    def update(self, frame):
        """Advance tracker one frame; re-segment at tracked centre every N frames."""
        if self._last_mask is None:
            return None

        # Advance tracker
        center = self._last_pt
        if self._tracker is not None:
            ok, box = self._tracker.update(frame)
            if ok and box is not None:
                x, y, w, h = (int(v) for v in box)
                center = (x + w / 2.0, y + h / 2.0)
            else:
                self._tracker = None   # lost — keep last mask until re-segment

        self._frame_n += 1
        if self._frame_n % max(1, self._interval) == 0 and center is not None:
            mask = self._seg.segment(frame, center)
            if mask is not None and mask.any():
                self._last_mask = mask
                self._last_pt   = center
                # Re-seat tracker on the fresh mask bbox
                bbox = _mask_to_bbox(mask, frame.shape)
                if bbox is not None:
                    self._tracker = _make_vit_tracker()
                    if self._tracker is not None:
                        self._tracker.init(frame, bbox)

        return self._last_mask

    def stop(self):
        self._tracker   = None
        self._last_mask = None
        self._last_pt   = None
        self._frame_n   = 0


def _make_vit_tracker():
    """Build a ViT tracker for position propagation; None if unavailable."""
    try:
        from .trackers import make_backend
        return make_backend("ViT")
    except Exception as e:
        print(f"[segmenter] ViT tracker unavailable ({e}); falling back to CSRT")
        try:
            from .trackers import make_backend
            return make_backend("CSRT")
        except Exception:
            return None


# ─────────────────────────── public manager ───────────────────────────────────

SEGMENT_BACKENDS = ["SAM 2", "MobileSAM", "FastSAM-s"]

_BACKEND_CLASSES = {
    "SAM 2":     _SAM2ImageSegmenter,
    "MobileSAM": _MobileSAMSegmenter,
    "FastSAM-s": _FastSAMSegmenter,
}


class ClickSegmenter:
    """
    Manages one-shot and continuous click-to-segment.

    One-shot  (seg_continuous=False):
        segment(frame, point) → mask or None

    Continuous (seg_continuous=True):
        start_continuous(frame, point) → first mask (also activates update loop)
        update(frame)                  → refreshed mask each frame
        stop_continuous()              → clear
    """

    def __init__(self, device: str, settings):
        self._device      = device
        self._settings    = settings
        self._cache: dict = {}
        self._continuous  = None   # _ContinuousWrapper or None when active
        self.last_error   = ""

    # ── one-shot ──────────────────────────────────────────────────────────────

    def segment(self, frame, point):
        backend = self._get_backend()
        if backend is None:
            return None
        try:
            return backend.segment(frame, point)
        except Exception as e:
            self.last_error = str(e)
            return None

    # ── continuous ────────────────────────────────────────────────────────────

    def start_continuous(self, frame, point):
        """Segment once and activate per-frame updates. Returns first mask."""
        self.stop_continuous()
        backend = self._get_backend()
        if backend is None:
            return None
        interval = getattr(self._settings, "seg_interval", 3)
        wrapper = _ContinuousWrapper(backend, seg_interval=interval)
        mask = wrapper.start(frame, point)
        if mask is not None:
            self._continuous = wrapper
        return mask

    def update(self, frame):
        """Called every inference frame when continuous mode is active."""
        if self._continuous is None:
            return None
        # Refresh interval from settings live
        self._continuous._interval = getattr(self._settings, "seg_interval", 3)
        return self._continuous.update(frame)

    def stop_continuous(self):
        if self._continuous is not None:
            self._continuous.stop()
            self._continuous = None

    @property
    def is_continuous(self):
        return self._continuous is not None

    # ── internal ──────────────────────────────────────────────────────────────

    def _get_backend(self):
        name = self._settings.segment_backend
        if name not in self._cache:
            cls = _BACKEND_CLASSES.get(name)
            if cls is None:
                self.last_error = f"Unknown backend: {name}"
                return None
            try:
                print(f"[segmenter] loading {name}…")
                self._cache[name] = cls(self._device)
                print(f"[segmenter] {name} ready")
                self.last_error = ""
            except Exception as e:
                self.last_error = str(e)
                print(f"[segmenter] {name} failed: {e}")
                return None
        return self._cache[name]
