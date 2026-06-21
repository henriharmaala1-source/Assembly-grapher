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

## What's implemented (P0 + P1)

| Piece | File | Status |
| ----- | ---- | ------ |
| World model (thread-safe blackboard + `brief()` LLM serialiser) | `world_model.*` | ✅ |
| Perception module interface | `perception.hpp` | ✅ |
| Track module (wraps `LockOnTracker`) | `perception.cpp` | ✅ |
| Navigate module (wraps `DepthNav` corridor) | `perception.cpp` | ✅ |
| Detect module (YOLOv8 ONNX via OpenCV DNN) | `perception.cpp` | ✅ |
| Compute-budgeted scheduler | `scheduler.*` | ✅ |
| Behaviour arbiter | `main.cpp` (stub) | ⚠️ stub — full FSM in P2 |
| MAVLink offboard control | — | ⬜ P2 |
| LLM sidecar + guard layer | — | ⬜ P3 |
| Ground UI / telemetry viz | — | ⬜ P4 |

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
