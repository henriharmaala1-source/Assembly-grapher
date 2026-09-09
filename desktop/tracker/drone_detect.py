"""
Drone detector — YOLOv8 models for detecting drones (and birds) in the sky.

Two model presets:
  Drone-vs-Bird  classes: drone, bird
  MAV-VID        classes: mav  (micro aerial vehicles)

Place the trained .pt file in ./models/ before enabling.
Community-trained models are available on HuggingFace and Roboflow Universe;
search "drone detection yolov8" or "drone-vs-bird yolov8".
"""

import os
import numpy as np

_MODELS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")

# Per-preset configuration
PRESETS = {
    "Drone-vs-Bird": {
        "filename": "drone-vs-bird.pt",
        "classes":  ["drone", "bird"],
        "colors":   [(40, 80, 255), (80, 220, 80)],   # red=drone, green=bird
    },
    "MAV-VID": {
        "filename": "mav-vid.pt",
        "classes":  ["mav"],
        "colors":   [(40, 80, 255)],
    },
}


class DroneDetector:
    """Lazy-loading YOLOv8 drone detector; swaps model when preset changes."""

    def __init__(self, device: str = "cuda"):
        self._device      = device
        self._model       = None
        self._loaded_file = None   # path of the currently loaded .pt

    def _model_path(self, preset: str) -> str:
        return os.path.join(_MODELS_DIR, PRESETS[preset]["filename"])

    def _load(self, preset: str):
        from ultralytics import YOLO
        path = self._model_path(preset)
        if not os.path.exists(path):
            cfg = PRESETS[preset]
            raise FileNotFoundError(
                f"Model not found: {path}\n"
                f"Download a YOLOv8 model trained on {preset} and save it there.\n"
                f"Search 'yolov8 {preset.lower()}' on HuggingFace or Roboflow Universe."
            )
        if self._loaded_file != path:
            self._model = YOLO(path)
            self._model.to(self._device)
            self._loaded_file = path
            print(f"[drone-detect] loaded {path} on {self._device}")

    def detect(self, frame: np.ndarray, preset: str,
               conf: float = 0.25, iou: float = 0.45) -> list:
        """
        Returns list of (x, y, w, h, class_id, confidence).
        Loads / swaps model automatically when preset changes.
        """
        self._load(preset)
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
