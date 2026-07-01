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
| MSP bench-test mode (live telemetry table) | `main.cpp --bench-test` | ✅ |
| State estimator (loosely-coupled KF, ENU) | `state_estimator.*` | ✅ |
| Synthetic-GPS feedback to iNAV (MSP2_SENSOR_GPS) | `msp_backend.*` | ✅ |
| MAVLink backend (ArduPilot) | `mavlink_backend.hpp` | ⚠️ documented stub |
| LLM sidecar + guard layer | — | ⬜ P3 |
| Ground UI / telemetry viz | — | ⬜ P4 |
| Localization (VIO/SLAM + image-to-map + EKF) | — | ⬜ P5 |

## Modes

Modes are **pluggable control modules** (`IControlMode`), registered with the
`ModeManager` and selected by name. One is active at a time; two **safety layers**
in the manager sit under all of them:

```
1. failsafe : low battery / operator abort  →  iNAV RTH (release + fc->setMode(RTL))
2. reflex   : imminent obstacle in a motion mode  →  HOLD (stop)
3. else     : the active mode module drives control
```

Standard set (keys `f s k o w a h`, `x` = abort→RTH):
`FLY · ASSIST · LOCK_ON · FOLLOW_ROAD · WAYPOINT · AUTONOMY · HOLD · FOLLOW_SUBJECT`.
`WAYPOINT` releases control (iNAV flies the GPS route) while the OS supervises —
the obstacle reflex stops it and detection runs in the deliberator.

## Extending the OS — four plugin interfaces

Adding capability is "implement an interface + register it," never editing a
switch. The four extension points:

| Add a… | Interface | Register / wire | Does |
| ------ | --------- | --------------- | ---- |
| **control mode** | `IControlMode` (`control_mode.hpp`) | `ModeManager::add()` (see `modes.hpp`) | `update(state,ctx)→ControlCmd`; `isMotion()` opts into the obstacle reflex |
| **perception module** | `IPerceptionModule` (`perception.hpp`) | `Deliberator::scheduler().add()` (heavy) or run in the fly loop (cheap) | `run(frame,wm)` → writes findings into the world model |
| **flight controller** | `IFlightController` (`flight_controller.hpp`) | `--fc=<name>` in `main.cpp` | `tick/poll/sendControl` — MSP, sim, MAVLink(stub) |
| **depth sensor** | `ITofSource` (`tof_source.hpp`) | into a `TofNavigateModule` | `read()→metric grid`; VL53L9/L5CX/MCU-hub/OAK/sim |

Two tiers keep it real-time: the **fly loop** (fast, control-critical) and the
**Deliberator** thread (heavy perception / SLAM / planning), sharing the
thread-safe `WorldModel` — so a slow module can never stall control.

A new mode, in full:

```cpp
class MyMode : public IControlMode {
    const char* name() const override { return "MYMODE"; }
    bool isMotion() const override { return true; }        // get the obstacle reflex
    ControlCmd update(WorldState& s, const ControlCtx& c) override {
        return c.controller->compute(s, Behavior::HOLD, c.frameW, c.frameH); // or your own
    }
};
// register:  mgr.add(std::make_unique<MyMode>());  — done, selectable by name.
```

## Flight-controller control

The OS is **FC-agnostic**: an `IFlightController` abstraction with an **MSP
(iNAV)** backend now and a **MAVLink (ArduPilot)** stub to fill later. The shared
control primitive is RC override (`MSP_SET_RAW_RC` ↔ `RC_CHANNELS_OVERRIDE`).

Two control modes (same navigation controller underneath — corridor, road,
hover, RTL — only the stick baseline differs):

| Mode | Flag | Sticks start from | Use |
| ---- | ---- | ----------------- | --- |
| **Total autonomy** (default) | — | neutral (1500 µs) — the OS is the stick source | hands-off autonomous flight |
| **Flight assist** | `--assist` | the operator's **current** sticks, latched at engagement | bumpless takeover; the OS trims from where the pilot's hands are |

Flight assist reads the operator's live channels back via `MSP_RC` and, on the
dry→live engage edge, latches them as the baseline; control output is then
`baseline + clamped-trim` instead of absolute-from-neutral, so there's no jolt at
handoff (throttle especially). It needs a real receiver present — config the FC
for partial override (`receiver_type = SERIAL` + `msp_override_channels`) so
`MSP_RC` reflects actual pilot input and arming/AUX stay on the radio.

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

# bench-test: live MSP telemetry table + dry-run RC channel map, no camera
./build/kestrel --fc=msp --fc-port=/dev/ttyAMA0 --fc-baud=115200 --bench-test

# NO HARDWARE: --fc=sim gives a software-in-the-loop FC that responds to control
# (GPS/attitude/battery evolve like a real link). Works with --bench-test or the
# full loop — test the FSM, controller, estimator and modes on the desktop.
./build/kestrel --fc=sim --bench-test
./build/kestrel --fc=sim --allow-control --display   # full loop, simulated FC
```

## State estimator + synthetic-GPS feedback

`StateEstimator` is a **loosely-coupled Kalman filter** in a local ENU frame
(origin = first 3-D fix). State `[pe,pn,pu, ve,vn,vu]`, constant-velocity model.
It is **not** a re-implementation of the FC's filter — iNAV already fuses IMU +
GPS + baro well. The Pi-side filter fuses the things the FC can't on its own
(chiefly visual odometry / SLAM) into one drift-corrected pose at loop rate:

| Source | Update | Status |
| ------ | ------ | ------ |
| FC GPS (lat/lon + course-speed) | absolute position + velocity, glitch-gated (2.5 m) | ✅ |
| FC baro (`MSP_ALTITUDE`) | vertical position | ✅ |
| Vision odometry velocity | `updateVisionVelocity()` (body→ENU via FC yaw) | ⬜ P5 hook |
| Absolute image-to-map fix | `updateVisionPose()` | ⬜ P5 hook |

Roll/pitch/yaw come straight from FC telemetry (its Mahony AHRS beats anything
derived casually); geodetic projection and body→world rotation are done at the
measurement boundary so the core filter stays linear and well-conditioned.

**Closing the loop — feed the estimate back to iNAV.** iNAV has no native
vision-pose message, so the supported path is a **synthetic GPS** via
`MSP2_SENSOR_GPS` (v2 frame, CRC8/DVB-S2). With `gps_provider = MSP` on the FC,
iNAV's own PosHold/RTH/WP use the injected fix exactly like a real GPS — this is
how a Pi running SLAM keeps the FC navigating GPS-denied. Injection is gated by
`--feed-gps` (off by default) and rate-limited (`--feed-gps-hz`, default 10).

> **SAFETY.** Feeding synthetic GPS requires `set gps_provider = MSP` on the FC.
> Honest accuracy fields are sent (`ephM`/`epvM` from the filter covariance) so
> iNAV's glitch/timeout gating still protects you. Until the P5 VIO front-end
> exists, the estimate is GPS+baro only, so `--feed-gps` is for bench-validating
> the MSP2_SENSOR_GPS framing and the geodetic round-trip, not GPS-denied flight.

```bash
# fuse FC telemetry into the ENU estimate and inject it back as synthetic GPS
./build/kestrel --fc=msp --fc-port=/dev/ttyAMA0 --fc-baud=115200 --feed-gps
```

## Build

```bash
sudo apt install -y build-essential cmake libopencv-dev
cd onboard
cmake -B build
cmake --build build -j4
```

Self-contained — all perception sources (lock-on tracker, depth nav, Kalman)
live under `onboard/` directly; there is no external project dependency.

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
