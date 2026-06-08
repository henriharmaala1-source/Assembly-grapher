# Assembly-grapher — Object Lock-On Tracker

Draw a box around any object in your webcam feed and the app locks on and
tracks it. Two interchangeable tracking **engines** are available:

| Engine | Philosophy | Output | Best for |
|--------|-----------|--------|----------|
| **hybrid** (default) | Fast box tracker (CSRT / KCF / Optical Flow / **ViT** transformer) + DINOv2 identity verification, Kalman smoothing, and full-frame re-acquisition | bounding box | lightweight, no large model |
| **sam2** | [SAM 2](https://github.com/facebookresearch/sam2) promptable mask propagation with built-in memory for occlusion / re-appearance | pixel mask + box | maximum robustness |

Requires an NVIDIA GPU (developed on an RTX 4070) and Python 3.11 (PyTorch has
no 3.13 wheels yet).

## Install

```bash
python -m venv .venv && source .venv/bin/activate   # or py -3.11 -m venv
pip install -r requirements.txt
# CUDA build of PyTorch (the default wheel is CPU-only):
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

`requirements.txt` pulls `opencv-contrib-python>=4.9` (needed for the **ViT**
transformer tracker backend).

## Run

```bash
python main.py                # hybrid engine (default)
python main.py --engine sam2  # SAM 2 engine
```

- **Draw** a box with the mouse to lock on.
- **R** resets the lock, **ESC** quits.
- A settings window lets you switch the box-tracker backend (incl. **ViT**),
  thresholds, motion vector, and mask overlay live. Re-draw the box to apply a
  backend change.

The first time you select the **ViT** backend it downloads a ~0.7 MB ONNX model
from the OpenCV model zoo into `./models/` (gitignored).

## SAM 2 engine setup

The SAM 2 engine needs the streaming (camera) build and a checkpoint:

```bash
# streaming-capable SAM 2 build
pip install "git+https://github.com/Gy920/segment-anything-2-real-time.git"

# a checkpoint — tiny is the fastest; download per the SAM 2 repo, e.g.:
mkdir -p checkpoints && cd checkpoints
# (use the repo's download_ckpts.sh, or fetch sam2.1_hiera_tiny.pt manually)
```

Defaults point at `configs/sam2.1/sam2.1_hiera_t.yaml` and
`./checkpoints/sam2.1_hiera_tiny.pt`. Override with environment variables:

```bash
SAM2_CFG=configs/sam2.1/sam2.1_hiera_s.yaml \
SAM2_CKPT=./checkpoints/sam2.1_hiera_small.pt \
python main.py --engine sam2
```

## Architecture

Three threads (`tracker/pipeline.py`): capture, inference, and display, so GPU
work never stalls the video. The active engine exposes a common interface
(`init / update / reset / center_trail / predicted_center`); the overlay
(`tracker/ui.py`) renders box, confidence, motion vector, and — for SAM 2 or
DINOv2 attention — the object mask.
