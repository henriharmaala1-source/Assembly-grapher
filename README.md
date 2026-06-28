# Assembly-grapher — a small-drone autonomy stack

A cheap, open companion-computer autonomy stack for small drones: vision-based
perception, GPS-denied state estimation, navigation, and flight-controller
integration, built to run on a Raspberry Pi 5 alongside an iNAV (or PX4 /
ArduPilot) flight controller.

The repo is **two clearly separated parts** — the on-drone OS (runs on the Pi)
and the desktop tool the perception is prototyped in (runs on a workstation):

| Directory | What it is | Runs on | Language |
|-----------|-----------|---------|----------|
| **`kestrel/`** | **The drone OS / Pi files.** Capture → perception scheduler → world model → behaviour FSM → controller → flight-controller backend. Self-contained C++. | Raspberry Pi 5 | C++ (OpenCV) |
| **`desktop/`** | **The desktop app.** DINOv2 + SAM 2 + CV lock-on tracker for developing and eyeballing the perception backends off the drone (`main.py` + the `tracker/` package). | workstation (GPU) | Python (PyTorch) |

Nothing is shared at the repo root — each part is fully contained in its own
directory with its own build/deps and its own README:
[`kestrel/README.md`](kestrel/README.md) for the OS,
[`desktop/README.md`](desktop/README.md) for the desktop app.

## kestrel — the drone OS

A modular real-time runtime (CPU-only, no ROS) that orchestrates the perception
modules behind one compute-budgeted scheduler, maintains a shared world model,
fuses a state estimate, and drives the flight controller.

- **Perception** — lock-on tracking (CSRT/KCF/flow/MOSSE + Kalman), monocular
  depth corridor, appearance road-follow (CIELab), YOLO detection.
- **State estimation** — loosely-coupled ENU Kalman filter fusing FC GPS + baro,
  with VIO/SLAM hooks, plus synthetic-GPS feedback into iNAV (`MSP2_SENSOR_GPS`)
  so the FC's own nav works GPS-denied.
- **Behaviour FSM** — hysteretic modes: `MANUAL · IDLE · NAVIGATE · ROAD_FOLLOW ·
  TRACK · SEARCH · EVADE · HOLD · RTL`, with failsafe priority.
- **Flight control** — FC-agnostic `IFlightController`; MSP (iNAV) backend with
  verified AETR channel order + telemetry, MAVLink backend stubbed. Control is
  **dry-run by default**.

See [`kestrel/README.md`](kestrel/README.md) for build, run, the bench-test mode,
and the state-estimator / synthetic-GPS details.

## Scope of the control layer

The OS flies the airframe toward **navigation goals** — an open corridor, a
road/route, a hover, return-to-home — and toward nothing else. Object tracking is
a **sensing** capability: `TRACK` keeps the target in the world model for a
camera/gimbal to point at and for the operator to see, but the airframe holds a
hover and does not translate or steer toward the tracked object. Steering the
aircraft onto a tracked target (target-homing / terminal guidance) is
deliberately **not** part of this codebase.

## Build / run (OS)

```bash
sudo apt install -y build-essential cmake libopencv-dev
cd kestrel && cmake -B build && cmake --build build -j4

# headless, prints world-state telemetry
./build/kestrel
# with a video window for desk testing
./build/kestrel --depth-model=models/midas_small.onnx --display
# talk to an iNAV FC (dry-run until --allow-control)
./build/kestrel --fc=msp --fc-port=/dev/ttyAMA0 --fc-baud=115200 --display
```

## Run (desktop app)

```bash
cd desktop
pip install -r requirements.txt
python main.py            # engines, CUDA, and SAM 2 setup: see desktop/README.md
```
