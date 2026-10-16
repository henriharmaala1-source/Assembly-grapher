# desktop — perception / dev app

Draw a box around any object in a webcam feed and the app locks on and tracks
it. This is the **off-drone** research/visualisation tool: it's where the
perception backends (DINOv2 re-ID, SAM 2, the CV trackers, depth/road analysis)
are developed and eyeballed before the C++ equivalents go onto the drone OS
(`../onboard`). Self-contained — run it from this directory with `python main.py`
(the `tracker/` package is the implementation).

Two interchangeable tracking **engines**:

| Engine | Philosophy | Output | Best for |
|--------|-----------|--------|----------|
| **hybrid** (default) | Fast box tracker (CSRT / KCF / Optical Flow / **ViT**) + DINOv2 identity verification, Kalman smoothing, full-frame re-acquisition | bounding box | lightweight, no large model |
| **sam2** | [SAM 2](https://github.com/facebookresearch/sam2) promptable mask propagation with memory for occlusion / re-appearance | pixel mask + box | maximum robustness |

Requires an NVIDIA GPU (developed on an RTX 4070) and Python 3.11 (PyTorch has
no 3.13 wheels yet). A CPU-only fallback path exists (`--cpu`).

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
python main.py                # hybrid engine
python main.py --engine sam2  # SAM 2 engine
python main.py --cpu          # CPU-only (depth nav + tracking, no GPU engines)
```

`--engine` only sets the *initial* engine — you can switch between **hybrid**
and **sam2** live from the settings window; the current lock box carries over.

- **Draw** a box with the mouse to lock on.
- **R** resets the lock, **ESC** quits.
- The settings window switches the hybrid backend (incl. **ViT**), thresholds,
  motion vector, and mask overlay live. Re-draw the box to apply a backend change.

The first time you select the **ViT** backend it downloads a ~0.7 MB ONNX model
from the OpenCV model zoo into `./models/` (gitignored).

## SAM 2 engine setup

The SAM 2 engine needs the streaming (camera) build and a checkpoint. That
fork's `setup.py` tries to compile an *optional* CUDA kernel that fails without
the full CUDA Toolkit — install it with that kernel disabled (the app doesn't use
it). **Install into the same Python 3.11 you run the app with.**

```powershell
git clone https://github.com/Gy920/segment-anything-2-real-time.git
cd segment-anything-2-real-time
# Drop the optional CUDA extension so no CUDA Toolkit / nvcc is required
(Get-Content setup.py) -replace 'ext_modules=get_extensions\(\),','ext_modules=[],' | Set-Content setup.py
py -3.11 -m pip install -e . --no-build-isolation
```

Then a checkpoint (tiny is fastest):

```powershell
mkdir checkpoints
curl -L -o checkpoints/sam2.1_hiera_tiny.pt https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
```

Defaults point at `configs/sam2.1/sam2.1_hiera_t.yaml` and
`./checkpoints/sam2.1_hiera_tiny.pt`. Override with `SAM2_CFG` / `SAM2_CKPT`.

## Camera-tilt bench (`tilt_bench.py`)

Standalone tool, separate from `main.py`: finds the real camera-tilt angle
that breaks the monocular depth model, on your actual webcam, instead of
trusting the reasoned-guess defaults currently in the onboard C++
(`camUpMaxDeg_` / `camDownMaxDeg_` in `onboard/include/perception.hpp`).

```bash
python tilt_bench.py
python tilt_bench.py --depth-model ~/depth_models/midas_small.onnx
python tilt_bench.py --depth-backend dav2 --band 0.25
```

It runs the same `DepthNav` this app uses, then reproduces the onboard
pipeline's horizon-band openness histogram (the exact mechanism the
elevation-window gate analyses) and flags when that signal goes
suspiciously flat. That flat-and-confident reading — not visible noise — is
the actual danger sign: a depth model pointed at featureless sky tends to
report a smooth "everything is far," which reads as a falsely-open corridor
rather than an obviously bad one.

Hold/mount the camera level, then slowly tilt it up toward the ceiling/sky
in a few steps, then back through level and down toward the floor. Watch the
`USABLE` / `MARGINAL` / `SUSPECT` flag (and the live per-column openness bar
graph, and the horizon-band lines drawn right on the video feed — that's
exactly the region being analysed). Note the angle — a phone inclinometer or
spirit level against the housing works fine — where each direction first
flips to `SUSPECT`. Press **s** at any point to save a labelled snapshot
(frame + heatmap + metrics JSON) to `tilt_bench_out/` for later comparison.
Those two angles are your real `camUpMaxDeg_` / `camDownMaxDeg_` — and,
separately, they tell you how much physical up-tilt the camera mount can
actually afford before hover starts losing scan coverage.

The usability flag is a starting heuristic (see the docstring in
`tilt_bench.py`), not ground truth — cross-check it against the heatmap
itself while calibrating: does it look like real room structure, or a flat
wash of one colour.

## Spin-in-place mapping (`spin_map.py`)

Answers: can spinning the webcam in one spot build a real map the drone's
`SCAN` phase logic could use to find an opening? Deliberately **not** general
SLAM — walking around needs translation, which is scale-ambiguous from one
camera and drifts without correction. Pure rotation from a fixed point has
neither problem, and it's exactly what `MoveStopSense::Phase::SCAN` already
does on the real aircraft (yaw only, no translation, sweep until an opening
appears) — so this tests that mechanism against real depth-model output
instead of nav-sim's simulated raycasts.

```bash
python spin_map.py
python spin_map.py --hfov 78     # set to your webcam's real horizontal FOV — matters a lot
```

Sparse optical flow between frames estimates how far you've rotated (no
translation assumed); the same horizon-band openness histogram from
`tilt_bench.py` gets written into a bearing-indexed polar map as you turn.
Once enough of the circle is covered, it finds the widest open arc — the
same thing `SCAN` is looking for — and draws it as a green arrow on the
radar view, alongside a live heading needle and coverage percentage.

**How to spin:** rest your elbow or use a tripod head — rotate in place,
don't walk in a circle (that reintroduces the translation error this
approach exists to avoid). Press **r** to reset and start a fresh spin,
**s** to save the radar view + a metrics JSON, **q**/ESC to quit.

This builds a single-vantage-point 360° map, not a multi-room one — that
would need walking, i.e. the harder translation/SLAM problem this
deliberately sidesteps.

## Architecture

Three threads (`pipeline.py`): capture, inference, and display, so GPU work
never stalls the video. The active engine exposes a common interface
(`init / update / reset / center_trail / predicted_center`); the overlay
(`ui.py`) renders box, confidence, motion vector, and — for SAM 2 or DINOv2
attention — the object mask.
