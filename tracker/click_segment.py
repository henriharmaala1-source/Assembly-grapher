"""
Click-to-segment: point-prompt object segmentation from a single mouse click.

Three backends, selectable live in the settings:
  - "SAM 2"      : uses the already-installed SAM 2 image predictor (best quality)
  - "MobileSAM"  : ~40 MB ViT-T model, fast CPU or GPU
  - "FastSAM-s"  : ~23 MB YOLO-based, fastest, coarser masks

All expose the same interface:
    segment(frame_bgr, point_xy) -> bool mask (H x W numpy)
"""

import os
import numpy as np
import cv2

# SAM 2 config defaults — same values as the tracking engine
DEFAULT_SAM2_CFG  = os.environ.get("SAM2_CFG",  "configs/sam2.1/sam2.1_hiera_t.yaml")
DEFAULT_SAM2_CKPT = os.environ.get("SAM2_CKPT", "./checkpoints/sam2.1_hiera_tiny.pt")

_MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")

# MobileSAM checkpoint — auto-downloaded once to ./models/
_MOBILE_SAM_URL  = ("https://github.com/ChaoningZhang/MobileSAM/releases/"
                    "download/v0.0.1/mobile_sam.pt")
_MOBILE_SAM_PATH = os.path.join(_MODELS_DIR, "mobile_sam.pt")


# ───────────────────────────── helpers ────────────────────────────────────────

def _best_mask(masks, scores):
    """Return the highest-scoring mask from a (C, H, W) array."""
    best = int(np.argmax(scores))
    m = masks[best]
    # masks may be float logits or bool — normalise to bool
    if m.dtype != bool:
        m = m > 0.0
    return m.astype(bool)


def _ensure_model(url: str, dest: str) -> str:
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
    print(f"Saved to {dest}")
    return dest


# ───────────────────────────── SAM 2 backend ──────────────────────────────────

class _SAM2Segmenter:
    def __init__(self, device: str):
        cfg  = DEFAULT_SAM2_CFG
        ckpt = DEFAULT_SAM2_CKPT
        if not os.path.exists(ckpt):
            raise RuntimeError(
                f"SAM 2 checkpoint not found: {ckpt}\n"
                "Install it first (see README) or set SAM2_CKPT."
            )
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as e:
            raise RuntimeError(
                f"SAM 2 not installed (import error: {e}). See README."
            )
        import torch
        self._dtype = torch.bfloat16 if device == "cuda" else torch.float32
        self._device = device
        model = build_sam2(cfg, ckpt, device=device, apply_postprocessing=False)
        self._predictor = SAM2ImagePredictor(model)

    def segment(self, frame_bgr: np.ndarray, point_xy: tuple) -> np.ndarray:
        import torch
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        x, y = int(point_xy[0]), int(point_xy[1])
        with torch.inference_mode(), torch.autocast(self._device, dtype=self._dtype):
            self._predictor.set_image(rgb)
            masks, scores, _ = self._predictor.predict(
                point_coords=np.array([[x, y]]),
                point_labels=np.array([1]),
                multimask_output=True,
            )
        return _best_mask(masks, scores)


# ───────────────────────────── MobileSAM backend ──────────────────────────────

class _MobileSAMSegmenter:
    def __init__(self, device: str):
        try:
            from mobile_sam import sam_model_registry, SamPredictor
        except ImportError as e:
            raise RuntimeError(
                f"MobileSAM not installed (import error: {e}).\n"
                "  pip install git+https://github.com/ChaoningZhang/MobileSAM.git"
            )
        import torch
        path = _ensure_model(_MOBILE_SAM_URL, _MOBILE_SAM_PATH)
        sam = sam_model_registry["vit_t"](checkpoint=path)
        sam.to(device)
        sam.eval()
        self._predictor = SamPredictor(sam)
        self._device = device

    def segment(self, frame_bgr: np.ndarray, point_xy: tuple) -> np.ndarray:
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        x, y = int(point_xy[0]), int(point_xy[1])
        self._predictor.set_image(rgb)
        masks, scores, _ = self._predictor.predict(
            point_coords=np.array([[x, y]]),
            point_labels=np.array([1]),
            multimask_output=True,
        )
        return _best_mask(masks, scores)


# ───────────────────────────── FastSAM backend ────────────────────────────────

class _FastSAMSegmenter:
    def __init__(self, device: str):
        try:
            from ultralytics import FastSAM
        except ImportError as e:
            raise RuntimeError(
                f"ultralytics not installed (import error: {e}).\n"
                "  pip install ultralytics"
            )
        # ultralytics auto-downloads FastSAM-s.pt on first use
        self._model  = FastSAM("FastSAM-s.pt")
        self._device = device

    def segment(self, frame_bgr: np.ndarray, point_xy: tuple) -> np.ndarray:
        x, y = int(point_xy[0]), int(point_xy[1])
        results = self._model(
            frame_bgr,
            points=[[x, y]],
            labels=[1],
            device=self._device,
            retina_masks=True,
            imgsz=1024,
            conf=0.4,
            iou=0.9,
            verbose=False,
        )
        if not results or results[0].masks is None:
            return np.zeros(frame_bgr.shape[:2], dtype=bool)
        data = results[0].masks.data.cpu().numpy()   # (N, H, W)
        # FastSAM returns the best match; take the first (highest conf)
        m = data[0] > 0.5
        # resize to original frame size if needed
        fh, fw = frame_bgr.shape[:2]
        if m.shape != (fh, fw):
            m = cv2.resize(m.astype(np.uint8), (fw, fh),
                           interpolation=cv2.INTER_NEAREST).astype(bool)
        return m


# ───────────────────────────── public manager ─────────────────────────────────

SEGMENT_BACKENDS = ["SAM 2", "MobileSAM", "FastSAM-s"]

_BACKEND_MAP = {
    "SAM 2":     _SAM2Segmenter,
    "MobileSAM": _MobileSAMSegmenter,
    "FastSAM-s": _FastSAMSegmenter,
}


class ClickSegmenter:
    """
    Lazy-loading, cache-on-first-use manager for click-to-segment backends.
    Call segment(frame, point) to get a mask; backend is switched live via
    settings.segment_backend.
    """

    def __init__(self, device: str, settings):
        self._device   = device
        self._settings = settings
        self._cache: dict = {}
        self._last_error: str = ""

    @property
    def last_error(self):
        return self._last_error

    def segment(self, frame_bgr: np.ndarray, point_xy: tuple):
        """Returns a bool (H, W) mask, or None on failure."""
        name = self._settings.segment_backend
        if name not in self._cache:
            try:
                cls = _BACKEND_MAP[name]
                print(f"[segmenter] loading {name}…")
                self._cache[name] = cls(self._device)
                print(f"[segmenter] {name} ready")
                self._last_error = ""
            except Exception as e:
                self._last_error = str(e)
                print(f"[segmenter] {name} failed: {e}")
                return None
        try:
            return self._cache[name].segment(frame_bgr, point_xy)
        except Exception as e:
            self._last_error = str(e)
            return None
