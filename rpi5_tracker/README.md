# RPi 5 Lock-On Tracker

Standalone C++ app for the Raspberry Pi 5 combining two things:

1. **Click-to-lock tracker** — click to designate a fixed-size box, tracked
   every frame by CSRT / KCF / sparse optical flow. Reports lock age and loss
   count for reliability testing. Pure CPU, no neural networks.

2. **Monocular depth navigation** *(optional)* — a depth estimation model
   (MiDaS Small or Depth Anything v2 Small) runs every N frames via OpenCV DNN,
   divides the frame into a 3×3 grid, and draws a direction arrow toward the
   most open sector. Tells the drone which way has clear airspace.

## Build (Raspberry Pi OS Bookworm)

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev
cd rpi5_tracker
cmake -B build
cmake --build build -j4
```

`libopencv-dev` includes CSRT/KCF tracking and the DNN module — no custom
OpenCV build needed.

## Run — tracker only

```bash
./build/lockon                         # USB cam, 640×480, CSRT
./build/lockon -b=kcf -s=100          # KCF backend, 100 px box
./build/lockon -c=1 --width=1280 --height=720
./build/lockon --deinterlace           # analog capture dongle (EasyCap)
```

## Run — with depth navigation

Download one of the ONNX models first (see below), then:

```bash
# MiDaS Small (~50 MB, ~8-12 fps on Pi 5)
./build/lockon --depth-model=models/midas_small.onnx --depth-backend=midas --depth-on

# Depth Anything v2 Small (~100 MB, ~5-8 fps on Pi 5)
./build/lockon --depth-model=models/depth_anything_v2_small.onnx --depth-backend=dav2 --depth-on

# Run depth every 10 frames instead of default 6 (reduces CPU load)
./build/lockon --depth-model=models/midas_small.onnx --depth-interval=10
```

Press **`d`** at any time to toggle the depth overlay on/off.

## Downloading the ONNX models

**MiDaS Small** (faster, ~8-12 fps):
```bash
mkdir -p models
wget -O models/midas_small.onnx \
  https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx
```

**Depth Anything v2 Small** (better quality, ~5-8 fps):
```bash
mkdir -p models
# Export from PyTorch (requires Python + torch + transformers):
pip install torch transformers
python3 - <<'EOF'
import torch
from transformers import pipeline
pipe = pipeline(task="depth-estimation",
                model="depth-anything/Depth-Anything-V2-Small-hf")
dummy = torch.zeros(1, 3, 518, 518)
torch.onnx.export(pipe.model, dummy, "models/depth_anything_v2_small.onnx",
                  opset_version=17, input_names=["pixel_values"],
                  output_names=["predicted_depth"])
EOF
```

## Keys

| Key | Action |
| --- | ------ |
| left click | lock onto that point |
| `1` / `2` / `3` / `4` | switch backend: CSRT / KCF / optical flow / MOSSE (resets lock) |
| `d` | toggle depth overlay (only if model loaded) |
| `r` | reset tracker |
| `q` / ESC | quit |

## HUD

**Lock box colours:** green = locked, orange = coasting through a momentary
loss, red = lost (15 consecutive failed frames).

**Depth grid** (when enabled): 3×3 coloured overlay — green sector = furthest
(most open), red = closest/blocked. Arrow points from frame centre toward the
most open sector. Score 0.0–1.0 printed per cell.

## Expected performance (640×480, Pi 5)

| Component | Interval | FPS cost |
| --------- | -------- | -------- |
| MOSSE tracker | every frame | 100+ fps (fastest, raw-pixel correlation filter) |
| CSRT tracker | every frame | ~30-60 fps |
| KCF tracker | every frame | 60+ fps |
| Optical flow tracker | every frame | 60+ fps |
| MiDaS Small depth | every 6 frames | ~1-2 fps overhead |
| Depth Anything v2 Small | every 6 frames | ~2-3 fps overhead |

## Choosing a backend

| Backend | Speed | Character |
| ------- | ----- | --------- |
| MOSSE | fastest | Raw-pixel correlation filter (FFT). Lowest CPU — frees headroom for depth on every frame. Drifts on lighting/scale changes; the Kalman + template re-detection layer compensates. |
| KCF | fast | HOG-feature correlation filter. Good balance. |
| CSRT | moderate | Spatial-reliability correlation filter. Most accurate, handles partial occlusion and scale best. |
| Optical Flow | fast | Forward-backward validated Lucas-Kanade. Handles partial occlusion; sensitive to analog noise. |

All four backends sit behind the same Kalman filter and template re-detection,
so even MOSSE's raw-pixel drift is caught and re-acquired. Pick MOSSE when you
need maximum frame rate or CPU headroom for the depth model; CSRT when raw
tracking accuracy matters most.

## Analog camera (FPV drone tap)

Use `--deinterlace` when capturing from an analog FPV camera via a USB capture
dongle (EasyCap UTV007/MS2106). This drops alternating lines and resizes back
up, eliminating comb artifacts on moving objects. Sparse optical flow backend
is most sensitive to analog noise — prefer CSRT/KCF with analog input.

## Pi Camera Module

`cv::VideoCapture(0, CAP_V4L2)` works for USB webcams. The Pi Camera Module
uses libcamera — pipe it through GStreamer or use a USB webcam for testing.
