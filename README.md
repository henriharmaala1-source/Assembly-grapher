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
python main.py                # starts on the hybrid engine
python main.py --engine sam2  # starts on the SAM 2 engine
```

`--engine` only sets the *initial* engine — you can switch between **hybrid**
and **sam2** live from the settings window; the current lock box carries over
(SAM 2 loads on first switch, which takes a couple of seconds).

- **Draw** a box with the mouse to lock on.
- **R** resets the lock, **ESC** quits.
- The settings window also switches the hybrid box-tracker backend (incl.
  **ViT**), thresholds, motion vector, and mask overlay live. Re-draw the box
  to apply a backend change.

The first time you select the **ViT** backend it downloads a ~0.7 MB ONNX model
from the OpenCV model zoo into `./models/` (gitignored).

## SAM 2 engine setup

The SAM 2 engine needs the streaming (camera) build and a checkpoint. That
fork's `setup.py` tries to compile an *optional* CUDA kernel, which fails
without the full CUDA Toolkit — so install it with that kernel disabled (the app
doesn't use it). **Install into the same Python 3.11 you run the app with.**

```powershell
git clone https://github.com/Gy920/segment-anything-2-real-time.git
cd segment-anything-2-real-time

# Drop the optional CUDA extension so no CUDA Toolkit / nvcc is required
(Get-Content setup.py) -replace 'ext_modules=get_extensions\(\),','ext_modules=[],' | Set-Content setup.py

# Install into your 3.11 env (matches `py -3.11 main.py`)
py -3.11 -m pip install -e . --no-build-isolation
```

Then a checkpoint (tiny is fastest):

```powershell
mkdir checkpoints
curl -L -o checkpoints/sam2.1_hiera_tiny.pt https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
```

Defaults point at `configs/sam2.1/sam2.1_hiera_t.yaml` and
`./checkpoints/sam2.1_hiera_tiny.pt`. Override with environment variables:

```powershell
$env:SAM2_CFG="configs/sam2.1/sam2.1_hiera_s.yaml"
$env:SAM2_CKPT="./checkpoints/sam2.1_hiera_small.pt"
py -3.11 main.py --engine sam2
```

## Architecture

Three threads (`tracker/pipeline.py`): capture, inference, and display, so GPU
work never stalls the video. The active engine exposes a common interface
(`init / update / reset / center_trail / predicted_center`); the overlay
(`tracker/ui.py`) renders box, confidence, motion vector, and — for SAM 2 or
DINOv2 attention — the object mask.
