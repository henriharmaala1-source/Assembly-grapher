# Assembly-grapher

> **New here? Start with [`onboard/docs/introduction.md`](onboard/docs/introduction.md)** —
> a plain-language walkthrough of what this is and why it's built this way,
> assuming only that you know what an FPV drone is.
>
> **Working on this codebase (human or AI)? Start with [`AGENTS.md`](AGENTS.md)** —
> the full architecture, control/data flow, plugin interfaces, invariants, and
> design scope in one place.

Two **independent** tools in one repo — they share no code and build separately:

| Directory | What it is | Runs on | Language |
|-----------|-----------|---------|----------|
| **`onboard/`** | **The on-drone autonomy OS** (named *kestrel*): capture → perception → world model → mode arbiter → controller → flight-controller backend. | Raspberry Pi 5, **on the aircraft** | C++ (OpenCV) |
| **`desktop/`** | **A workstation app** for prototyping the perception backends off the drone: DINOv2 + SAM 2 + CV lock-on tracker (`main.py` + the `tracker/` package). | desktop / laptop, **never flies** | Python (PyTorch) |

`onboard/` is the Pi / on-drone system; `desktop/` is a development tool that
stays on the ground. Each is self-contained with its own build, deps, and README:
[`onboard/README.md`](onboard/README.md) and [`desktop/README.md`](desktop/README.md).

## onboard — the on-drone OS (kestrel)

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

See [`onboard/README.md`](onboard/README.md) for build, run, the bench-test mode,
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
cd onboard && cmake -B build && cmake --build build -j4

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
