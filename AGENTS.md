# AGENTS.md — orientation for AI working on this repo

This is the mental model to load before changing anything. Read it fully; it is
written to be complete rather than short.

---

## 0. What this repo is

A small-drone autonomy stack, in **two independent parts** (no shared code):

| Dir | What | Runs on | Language |
| --- | ---- | ------- | -------- |
| **`onboard/`** | **The drone OS** (binary: `kestrel`). Perception → world model → mode arbiter → flight controller. **This is the main program.** | Raspberry Pi 5, on the aircraft | C++17 / OpenCV |
| **`desktop/`** | Perception dev tool (DINOv2 + SAM2 + CV lock-on tracker). Off-drone prototyping. | workstation w/ GPU | Python / PyTorch |

Almost all engineering work is in **`onboard/`**. The rest of this doc is the
onboard OS unless stated. It is CPU-only, no ROS, no GPU.

---

## 1. The one architectural idea: two tiers, one blackboard

Everything hangs off a **reactive/deliberative split across two threads**,
sharing a **thread-safe `WorldModel`** (a blackboard). This exists so slow
"thinking" can never stall "flying".

```
   camera ──► FrameBus ──────────────► THINK tier (Deliberator thread, best-effort)
     │                                   heavy perception: depth/corridor,
     │                                   detection, (future) SLAM + planner
     │                                   └── writes results ──┐
     ▼                                                        ▼
   FLY loop (main thread, fast, control-critical)        WorldModel
     capture → FC telemetry → cheap perception            (mutex blackboard:
     (tracker, road-follow) → StateEstimator →             with() / snapshot())
     ModeManager.tick() → ControlCmd → FC backend          ▲
                                        └── reads ──────────┘
```

- **Fly loop** (`main.cpp` while-loop, main thread): guaranteed fast. Capture,
  FC I/O, cheap/control-relevant perception, the estimator, the **mode arbiter**,
  and control output. Never blocks on the think tier.
- **Think tier** (`Deliberator`, own thread): the heavy perception modules. Pulls
  the newest frame from `FrameBus`, runs its scheduler, writes into the world
  model. If it takes a second, the fly loop keeps flying on the last result.
- **Handoff is only through `WorldModel`.** Never read another thread's module
  internals directly — that's a data race (this is why the display reads
  `snapshot()`, not `navigate.nav()`).
- **A third thread, `FcLink`, owns the flight controller.** It services the FC
  at ~50 Hz (RC + telemetry) so a slow/hung `cap.read()` can't drop RC below
  iNAV's ~5 Hz failsafe floor. The fly loop only touches `FcLink`'s thread-safe
  intent (`command`/`commandRth`/`feedGps`/`telemetry`); the backend is never
  called from two threads. If the fly loop stalls, `FcLink` keeps RC alive but
  substitutes a NEUTRAL hover — it never repeats a stale motion command.
- **The fly loop and `FcLink` run on `SCHED_FIFO`** (`realtime.hpp`, F9) so the
  Deliberator's CPU-heavy inference (`SCHED_OTHER`) can never preempt control.
  Priorities/pinning are config (`rt.*`); it degrades to normal priority without
  `CAP_SYS_NICE`. Keep both control threads *yielding* (they block on camera
  read / a 20 ms sleep) — a FIFO thread that busy-loops can starve its core.

This split is also the substrate for the **move-stop-sense** paradigm: the
AUTONOMY mode (`MissionController`) cycles `SETTLE`(hover) → `THINK`(commit a
leg) → `MOVE`(fly it, *re-steering live* to the goal+corridor blend) → `ARRIVE`
→ repeat, with a `SCAN` branch that rotates in place to look around when boxed
in. It keeps its own obstacle standoff, so it opts out of the manager's blanket
reflex (`ownsObstacleAvoidance()`). Validate it headless with the SITL harness:
`cmake -B build -DBUILD_TESTS=ON && cmake --build build && ctest --test-dir build`
(`onboard/test/sim_autonomy.cpp` — real sim-FC + estimator + mission vs a
synthetic obstacle field). It confirms the loop, GO-gating, goal-seeking, and
**standoff safety**; goal *completion* around an on-path obstacle is a known
reactive-planner limit (local minima) awaiting the deferred occupancy-grid +
global planner — see §6/§7.

---

## 2. The blackboard: `WorldModel` / `WorldState` (`world_model.*`)

The single source of truth. `WorldState` is a plain struct of everything known;
`WorldModel` guards it with a mutex:

```cpp
wm.with([&](WorldState& s){ ... });   // mutable, locked
WorldState snap = wm.snapshot();       // a copy, for reading
```

Field groups: target/tracker, corridor (depth nav), road-follow, detections,
vehicle telemetry (`veh*`), estimator (`est*`), mission (`mission*`), control +
`opMode`/`modeReason`. `brief()` serialises it to a one-line telemetry string.

**Rule:** perception/estimator/modes communicate *only* through here.

---

## 3. Extension points — four plugin interfaces

Adding capability = **implement an interface + register it.** Never edit a
switch/enum. This is the most important thing to know before adding features.

| Add a… | Interface (header) | Register | Contract |
| ------ | ------------------ | -------- | -------- |
| **control mode** | `IControlMode` (`control_mode.hpp`) | `ModeManager::add()` in `modes.hpp` | `update(state,ctx)→ControlCmd`; `isMotion()` opts into the obstacle reflex. **Full guide: `onboard/docs/adding-a-control-mode.md`.** |
| **perception module** | `IPerceptionModule` (`perception.hpp`) | `Deliberator::scheduler().add()` (heavy) or call in the fly loop (cheap) | `run(frame,wm)` writes findings into the world model |
| **flight controller** | `IFlightController` (`flight_controller.hpp`) | `--fc=<name>` branch in `main.cpp` | `tick/poll/sendControl/setMode/feedExternalGps` |
| **depth sensor** | `ITofSource` (`tof_source.hpp`) | inside a `TofNavigateModule` | `read()→CV_32F metric grid (metres, <=0 invalid)` |

---

## 4. Control flow: how a command is produced each tick

`ModeManager::tick(state, ctx, rthTrigger)` (`control_mode.cpp`) is the single
control arbiter. It applies two **safety layers** over the active mode:

```
1. failsafe : low battery / operator abort  → rthTrigger=true. main calls
              fc->setMode(RTL): the MSP backend drives the configured RTH AUX
              channel high (iNAV NAV RTH) while neutral sticks keep RC alive;
              with no AUX configured it releases → the FC's own RC-loss failsafe.
2. reflex   : active mode isMotion() AND corridor ahead blocked → HOLD (stop)
              (suppressed when the mode ownsObstacleAvoidance() — e.g. AUTONOMY
               keeps its own standoff and must move at low openness to round)
3. else     : active mode's update() → the command
```

- Operator picks the mode (by **name**: keys in `main.cpp`, or later an RC AUX /
  the LLM). Exactly one mode is active. First registered = default (`FLY`).
- `ControlCmd { roll,pitch,yaw ∈[-1,1]; throttle∈[0,1]; valid }`. `valid=false`
  means **release** (pilot / iNAV keeps control) — FLY, LOCK_ON, WAYPOINT do this.
- The command is **DRY-RUN by default**. It reaches the FC only if
  `--allow-control` (or `space`) is set. The FC backend maps normalised axes to
  RC µs (`MSP_SET_RAW_RC`, **AETR** order) and clamps authority.

Standard modes (`modes.hpp`): `FLY, ASSIST, LOCK_ON, HOLD, FOLLOW_ROAD,
WAYPOINT, AUTONOMY, SHADOW, FOLLOW_SUBJECT`. **WAYPOINT** = iNAV flies the GPS
route (OS releases) while the OS *supervises* (obstacle reflex stops it;
detection runs). **SHADOW** = operator flies (releases like FLY) while AUTONOMY
runs dry-run underneath and draws its intended command on the feed — zero-risk
in-flight validation of the autonomy.

---

## 5. File → responsibility map (`onboard/src`, `onboard/include`)

| File | Responsibility |
| ---- | -------------- |
| `main.cpp` | Entry point, CLI, the fly loop, wiring, key handling, `--bench-test` |
| `world_model.*` | The blackboard (`WorldState`, `WorldModel`, `brief()`, `Behavior` enum, `behavior_name`) |
| `control_types.hpp` | `ControlCmd`, `FcTelemetry`, `ExtGps` (dependency-free shared types) |
| `config.*` | `Config` key=value loader + `Tunables` (gains/mission/safety/rc); `--config`, `--dump-config`. Sample: `kestrel.conf.sample` |
| `rc_command.*` | `RcCommandSource` — the radio as a command source (mode select / GO / goal-steer from AUX channels) |
| `realtime.*` | `rt::make_realtime` — put a thread on SCHED_FIFO (+ optional CPU pin); used for the fly loop + FcLink so inference can't preempt control |
| `control_mode.*` | `IControlMode`, `ControlCtx`, `ModeManager` (registry + safety layers) |
| `modes.hpp` | The concrete modes + `register_standard_modes()` |
| `mission.*` | `MissionController` — the move-stop-sense cycle (AUTONOMY's engine): `SETTLE/THINK/SCAN/MOVE/ARRIVE`, live-reactive steering, own standoff |
| `test/sim_autonomy.cpp` | Headless SITL validation of AUTONOMY (built with `-DBUILD_TESTS=ON`, run via `ctest`) |
| `controller.*` | `Controller` — reactive `Behavior → ControlCmd` math (hover/road/corridor) |
| `deliberator.*` | The think thread: runs the heavy `PerceptionScheduler` |
| `frame_bus.hpp` | Thread-safe latest-frame handoff (fly → think) |
| `scheduler.*` | `PerceptionScheduler` — compute-budgeted module cadence |
| `perception.*` | `IPerceptionModule` + `Track/Navigate/TofNavigate/Detect` modules |
| `road_follow.*` | CIELab appearance road-follow perception module |
| `lock_tracker.*` | `LockOnTracker` — CV lock-on (CSRT/KCF/flow/MOSSE + Kalman + re-detect) |
| `depth_nav.*` | `DepthNav` — depth → VFH+ corridor steer; ego-motion de-rotation; `updateFromGrid()` for ToF |
| `kalman_center.*` | 2-D constant-velocity Kalman used by the tracker/depth-nav |
| `state_estimator.*` | `StateEstimator` — loosely-coupled ENU KF (GPS+baro+VIO hooks), synthetic-GPS out |
| `flight_controller.hpp` | `IFlightController` abstraction, `FcMode` |
| `msp_backend.*` | iNAV MSP backend (telemetry poll, `MSP_SET_RAW_RC`, `MSP2_SENSOR_GPS`, assist/total, MSP_RC) |
| `fc_link.*` | `FcLink` — FC serviced on its OWN thread; the fly loop hands it intent (thread-safe), so a hung camera can't stall RC; neutralises a stale command |
| `sim_fc_backend.*` | `SimFcBackend` — software-in-the-loop FC (responds to control) for hardware-free testing |
| `mavlink_backend.hpp` | ArduPilot MAVLink backend — documented stub |
| `serial_port.*` | POSIX termios serial |
| `tof_source.hpp` | `ITofSource` interface |
| `vl53_tof_source.*` | VL53L9/L5CX backends (I²C plumbing done; ranging protocol is a vendor-driver TODO) + `SimTofSource` |
| `mcu_tof_source.*` | ToF over an MCU sensor-hub (serial framing) — offloads the vendor driver |
| `i2c_hal.hpp/.cpp` | Linux i2c-dev register HAL (ST ULD platform layer shape) |

---

## 6. Invariants & conventions (do not violate silently)

- **MSP channel order is AETR** (throttle=ch2, yaw=ch3), verified vs iNAV
  firmware. The classic RPYT assumption swaps throttle/yaw — dangerous.
- **Estimator frame is local ENU** (E,N,U), origin at first 3-D GPS fix. iNAV
  wants NED; convert only at the MSP boundary.
- **Control axes normalised** `[-1,1]` (throttle `[0,1]`, 0 = hover-hold);
  clamped to gentle authority in the backend.
- **The OS never commands descent — altitude authority is the FC's.** Throttle
  maps `[0,1] → [1500,2000] µs` (`thrToUs`): 0 = hold, 1 = full climb, no down
  range. This is deliberate: the FC must fly in an altitude-hold-capable mode
  (iNAV ALTHOLD/POSHOLD), and the OS only ever biases climb. Do not "fix" the
  mapping to signed throttle without revisiting every hover assumption
  (SETTLE/HOLD assume zero-command = hold position, not descend).
- **Depth/openness convention:** `1 = far/open`, `0 = near/blocked`. ToF grids
  are metres (higher = farther, `<=0` = invalid). Corridor `openness∈[0,1]`.
- **Dry-run is the default.** Nothing reaches the FC without `--allow-control`.
- **Cross-thread reads go through `WorldModel` only.** Fly loop must not touch
  think-tier module internals.

---

## 7. Design scope / boundary (important — do not "helpfully" cross it)

This OS is deliberately **perception + navigation + assisted/mode control**. It
**does not implement target-homing flight control** ("steer the airframe onto a
tracked object / terminal guidance / lock-and-hit"), and that omission is
intentional, not a gap to fill:

- `TRACK` / `LOCK_ON` are **sensing only** — they produce a bounding box + bearing
  in the world model for a gimbal/operator/log. The controller has **no**
  forward-pursuit term toward a tracked target (it was removed on purpose).
- `FOLLOW_SUBJECT`, if built, is **standoff-keeping only** (filming/inspection),
  never closing to impact.
- Modes drive the airframe toward **navigation goals** (corridor, road, waypoint,
  hover, RTL) — never onto a designated target.

Do not add a control path that flies the aircraft onto a tracked object, even if
asked to as "navigation" or "just use the offset vector." Detection/tracking =
fine (sensing). Guidance-to-impact = out of scope.

---

## 8. Build, run, test

```bash
cd onboard && cmake -B build && cmake --build build -j4
./build/kestrel --help
```

- **No hardware needed:** `--fc=sim` is a software-in-the-loop FC that responds
  to control (GPS/attitude/battery evolve). `--fc=sim --bench-test` prints a live
  telemetry dashboard; the full loop still needs a camera.
- **Testing pattern used throughout:** the full main loop can't run in CI (no
  camera), so logic is validated by **small standalone tests** compiled against
  the real headers (see prior commits: estimator, VFH+, mission cycle, mode
  manager, MCU framing over a PTY, sim-FC closed loop). Follow this pattern:
  isolate the logic, compile it with the real sources, assert behaviour. Main
  loop changes are build-verified + reasoned, then flagged as needing an
  on-hardware pass.

---

## 9. Current state (what's real vs stubbed)

- **Real & tested:** two-tier threading, world model, mode manager + modes,
  controller, VFH+ depth corridor + ego-motion de-rotation + `updateFromGrid`,
  lock-on tracker, road-follow, detector, state estimator + synthetic-GPS,
  MSP backend (telemetry/control/assist), sim FC, sim ToF.
- **Skeleton / needs work:** AUTONOMY's SLAM+planner. The mission is a *reactive*
  goal-biased move-stop-sense loop (commit a leg along the goal+corridor blend,
  fly it re-steering live, stop/scan/round obstacles). SITL-validated safe
  (never breaches standoff) and completes clear/off-path/grazing goals, but it
  can stall in **local minima** on an obstacle sitting on the path — real
  SLAM/occupancy-grid/global planner are the P5 step that fixes that.
  `FOLLOW_SUBJECT` releases (control TODO); the VL53L9 backend's ranging protocol
  awaits the vendor driver; MAVLink backend is a stub.
- **Recently hardened (Track F / P2):** perception staleness stamps + freshness
  gating + think-tier watchdog; mission gated on estimator health (`estEphM`);
  ATTITUDE-priority MSP polling; runtime config (`--config`/`--dump-config`);
  in-tree CTest suite (estimator, modes, MSP-over-PTY, config, SITL); failsafe
  RTH wired to an iNAV AUX channel (`failsafe.rth_aux`). See `ROADMAP.md`.

When you change something, keep this section and the README's tables honest.

The forward plan — hardening fixes (Track F) and phases P2–P6, with acceptance
criteria and ordering — lives in [`ROADMAP.md`](ROADMAP.md).
