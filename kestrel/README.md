# kestrel — Pi 5 drone-vision runtime

A modular real-time runtime that orchestrates the vision modules in this repo
behind a single compute-budgeted scheduler, maintains a shared world model, and
(in later phases) drives a flight controller and an on-device LLM supervisor.

Target hardware: **Raspberry Pi 5, CPU-only**. The on-device LLM runs as a
separate `llama.cpp` sidecar (P3).

## Architecture

```
┌─ DELIBERATIVE (P3, ~0.2–1 Hz, advisory) ────────────────┐
│  on-device LLM (llama.cpp) — mission compile, mode       │
│  decisions, NL commands → VALIDATED goals only           │
└───────────────────────────┬──────────────────────────────┘
                            │ guarded goals / mode / params
┌───────────────────────────▼─ REACTIVE (30 Hz) ──────────┐
│  Capture → PerceptionScheduler → WorldModel (blackboard)  │
│          → Behaviour arbiter → MAVLink offboard (P2)      │
└───────────────────────────────────────────────────────────┘
```

The LLM never sends control commands; it selects among allowed behaviours and
tunes parameters within hard bounds. A guard layer validates every LLM output
and a watchdog keeps the reactive layer flying if the LLM stalls.

## What's implemented (P0–P2)

| Piece | File | Status |
| ----- | ---- | ------ |
| World model (thread-safe blackboard + `brief()` LLM serialiser) | `world_model.*` | ✅ |
| Perception module interface | `perception.hpp` | ✅ |
| Track module (wraps `LockOnTracker`) | `perception.cpp` | ✅ |
| Navigate module (wraps `DepthNav` corridor) | `perception.cpp` | ✅ |
| Detect module (YOLOv8 ONNX via OpenCV DNN) | `perception.cpp` | ✅ |
| Road-follow module (CIELab appearance, no NN) | `road_follow.*` | ✅ |
| Compute-budgeted scheduler | `scheduler.*` | ✅ |
| Behaviour FSM (hysteretic modes) | `fsm.*` | ✅ |
| Controller (behaviour → normalised command) | `controller.*` | ✅ |
| Flight-controller abstraction | `flight_controller.hpp` | ✅ |
| MSP backend (iNAV, verified AETR + telemetry) | `msp_backend.*`, `serial_port.*` | ✅ |
| MAVLink backend (ArduPilot) | `mavlink_backend.hpp` | ⚠️ documented stub |
| LLM sidecar + guard layer | — | ⬜ P3 |
| Ground UI / telemetry viz | — | ⬜ P4 |
| Localization (canopy + SLAM + EKF) | — | ⬜ P5 |

## Modes (behaviour FSM)

`MANUAL · IDLE · NAVIGATE · ROAD_FOLLOW · TRACK · SEARCH · EVADE · HOLD · RTL`

Priority, highest first: **failsafe** (low battery / lost FC link → RTL or HOLD)
→ **operator** latch (`m`/`h`/`g` keys) → **autonomy** ladder (TRACK > EVADE >
ROAD_FOLLOW > NAVIGATE > SEARCH). Debounce counters stop the mode flapping.

## Flight-controller control

The OS is **FC-agnostic**: an `IFlightController` abstraction with an **MSP
(iNAV)** backend now and a **MAVLink (ArduPilot)** stub to fill later. The shared
control primitive is RC override (`MSP_SET_RAW_RC` ↔ `RC_CHANNELS_OVERRIDE`).

> **SAFETY — control is DRY-RUN by default.** The command is computed and shown
> but **not sent**. Pass `--allow-control` (and toggle with `space`) to actually
> send. Arm on the radio, props off, bench-test first. MSP channel order is
> **AETR** (throttle = ch index 2, yaw = ch index 3) — verified against iNAV
> firmware; the classic RPYT assumption silently swaps throttle and yaw.

```bash
# dry-run autonomy with depth + road follow, video window
./build/kestrel --depth-model=models/midas_small.onnx --display

# talk to an iNAV FC over UART, still dry-run until you press space / pass --allow-control
./build/kestrel --fc=msp --fc-port=/dev/ttyAMA0 --fc-baud=115200 --display
```

## Build

```bash
sudo apt install -y build-essential cmake libopencv-dev
cd kestrel
cmake -B build
cmake --build build -j4
```

Reuses `../rpi5_tracker/src` directly — build that project's deps and this
compiles too.

## Run

```bash
# tracker only (headless, prints world-state telemetry)
./build/kestrel

# with monocular corridor navigation
./build/kestrel --depth-model=models/midas_small.onnx

# with drone detection (YOLOv8 ONNX export)
./build/kestrel --detect-model=models/dvb.onnx --detect-labels=drone,bird

# everything, with a video window for desk testing
./build/kestrel --depth-model=models/midas_small.onnx \
                --detect-model=models/dvb.onnx --display
```

### Telemetry line
Headless, the runtime prints one compact world-state line ~2×/sec — this is
exactly the scene state the LLM supervisor will consume in P3:

```
beh=NAVIGATE trk=none nav=TRAVERSE(open72%,hdg318,241) det=[drone:81%] bat=100% alt=0m fps=24
```

## The scheduler

On a CPU-only Pi 5 you cannot run depth + detection every frame. Each heavy
module declares a cost (ms) and a cadence that **tightens when it is "hot"** for
the active behaviour:

| Behaviour | Hot module | Effect |
| --------- | ---------- | ------ |
| TRACK     | (tracker, always-on) | detection runs cold for re-acquire |
| NAVIGATE  | navigate   | depth runs every 3 frames, detection cold |
| SEARCH    | detect     | detection runs every 4 frames, depth cold |

A per-tick millisecond budget guarantees two heavy models never fire on the same
frame, so the tracker keeps its frame rate regardless.

## Roadmap

- **P2** — hysteretic behaviour FSM + MAVLink offboard control (PX4/ArduPilot
  SITL first, no real flight).
- **P3** — `llama.cpp` sidecar: GBNF function-calling scene supervisor + NL
  mission compiler + deterministic guard layer + watchdog.
- **P4** — ground UI / telemetry / corridor + occupancy visualisation.
- **P5** — hardware bring-up (camera, flight controller), bench → flight.
