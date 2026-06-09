"""
VisDrone object detector — YOLOv8n fine-tuned on the VisDrone-DET dataset.

VisDrone images are captured FROM drones looking down, so the model is tuned
for small, densely-packed ground objects seen at aerial angles.

Classes (10): pedestrian, people, bicycle, car, van, truck, tricycle,
              awning-tricycle, bus, motor.

The model is downloaded automatically from HuggingFace on first use
(requires: pip install huggingface_hub ultralytics).
"""

import os
import shutil

import numpy as np

CLASSES = [
    "pedestrian", "people", "bicycle", "car", "van",
    "truck", "tricycle", "awning-tricycle", "bus", "motor",
]

_MODELS_DIR  = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
_MODEL_PATH  = os.path.join(_MODELS_DIR, "yolov8n-visdrone.pt")
_HF_REPO     = "keremberke/yolov8n-visdrone"
_HF_FILENAME = "best.pt"


def _ensure_model(path: str) -> str:
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        from huggingface_hub import hf_hub_download
        print(f"[visdrone] downloading model from {_HF_REPO}…")
        tmp = hf_hub_download(repo_id=_HF_REPO, filename=_HF_FILENAME)
        shutil.copy(tmp, path)
        print(f"[visdrone] model saved → {path}")
        return path
    except ImportError:
        raise FileNotFoundError(
            f"VisDrone model not found: {path}\n"
            "Run once to auto-download:  pip install huggingface_hub\n"
            "Then restart the app."
        )
    except Exception as exc:
        raise FileNotFoundError(
            f"VisDrone model not found: {path}\n"
            f"Auto-download failed ({exc}).\n"
            f"Download '{_HF_FILENAME}' from HuggingFace repo '{_HF_REPO}' "
            f"and save it as {path}"
        )


class VisDroneDetector:
    """Lazy-loading YOLOv8 detector for aerial-view object detection."""

    def __init__(self, device: str = "cuda", model_path: str = None):
        self._device     = device
        self._model_path = model_path or _MODEL_PATH
        self._model      = None

    def _load(self):
        from ultralytics import YOLO
        path = _ensure_model(self._model_path)
        self._model = YOLO(path)
        self._model.to(self._device)
        print(f"[visdrone] ready on {self._device}")

    def detect(self, frame: np.ndarray,
               conf: float = 0.25, iou: float = 0.45) -> list:
        """
        Returns list of (x, y, w, h, class_id, confidence).
        Loads the model on the first call.
        """
        if self._model is None:
            self._load()
        results = self._model(frame, conf=conf, iou=iou, verbose=False)
        out = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                out.append((
                    int(x1), int(y1), int(x2 - x1), int(y2 - y1),
                    int(box.cls[0]),
                    float(box.conf[0]),
                ))
        return out
