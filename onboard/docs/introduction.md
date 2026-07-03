# kestrel — architecture and design rationale

**Summary:** kestrel is a companion-computer runtime for an analog FPV
multirotor. It runs on a Raspberry Pi 5, consumes the aircraft's existing
video signal, and issues normalized control commands to the flight
controller over MSP — functionally equivalent to pilot stick input — to
provide obstacle-avoidance corridor steering, GPS-denied position hold
support, and a small set of semi-autonomous flight modes, without modifying
the flight controller's own attitude-control loop.

This document covers system architecture and the engineering rationale
behind it. It assumes familiarity with FPV multirotor concepts (flight
controller, ESC, VTX, analog video link) and general software engineering,
but no prior context on this specific codebase.

---

## Problem statement

An FPV flight controller (FC) — firmware such as iNAV, running on an
STM32-class MCU — closes the inner attitude-control loop at several hundred
Hz: IMU sampling, sensor fusion, PID computation, motor mixing. This loop is
proven, latency-critical, and out of scope for modification.

What the FC does not provide is exteroceptive sensing or world-frame
reasoning: obstacle detection, semantic understanding of the environment, or
position hold in the absence of GPS. These are the problems a companion
computer is suited to, and the ones kestrel addresses — without touching the
FC's own control loop. The FC receives normalized stick commands from
kestrel exactly as it would from the pilot's receiver; there is no
distinction at the protocol level between an operator input and a
kestrel-generated one. kestrel therefore implements no attitude control,
motor mixing, or gain tuning of its own — it only ever emits the same class
of setpoint a human pilot would, and the FC's existing control loop consumes
it unchanged.

## Design constraint: cost and mass budget

The hardware budget is a fixed, deliberate constraint, not an incidental
limitation. It bounds every downstream design decision in this project and
is worth stating explicitly before anything else.

The alternative architecture — a Jetson-class SoC with GPU-accelerated
inference, LiDAR, stereo depth, a full ROS stack — is what most funded
autonomous-UAV platforms (Skydio, most companion-computer research
platforms) use, and it is a reasonable architecture. It is also
incompatible with the target platform here: an analog FPV airframe, where
added mass and cost scale directly against flight characteristics and
accessibility. The constraint adopted instead is: the companion-computer
subsystem must be addable to an existing, already-flying FPV quad without
airframe changes, at a cost on the order of the aircraft itself, not a
multiple of it. See [`bom.md`](bom.md) for the actual parts list — the
companion-computer subsystem (compute plus the video/power interface) is
approximately €100 on top of the airframe; the full platform, including the
aircraft, is approximately €476.

This constraint is causally upstream of most other design decisions in the
system:

- **No GPU.** The Raspberry Pi 5's GPU is a display/video-decode block with
  no general-purpose ML inference path — no CUDA-equivalent runtime is
  available. Every inference workload runs on the CPU (`DNN_BACKEND_OPENCV`
  / `DNN_TARGET_CPU`), which bounds model size (small variants: MiDaS-small,
  DepthAnything-v2-Small) and inference cadence (not every frame — see
  scheduling, below).
- **No LiDAR, no dedicated stereo rig.** Depth is estimated from a single
  monocular feed by default, with a plugin interface (`ITofSource`) for a
  time-of-flight sensor if one is added later. Obstacle sensing is
  consequently a short-range, forward-looking corridor estimate, not a
  dense 3D reconstruction.
- **No onboard SLAM or continuous replanning, currently.** Maintaining and
  querying an occupancy map at flight rate exceeds the available CPU budget
  under this constraint. The system instead uses a stop/sense/move cycle
  (see Motion Planning, below), trading path efficiency for compute cost.

None of these are permanent architectural ceilings — the plugin interfaces
(`IPerceptionModule`, `ITofSource`, `IFlightController`) exist specifically
so a faster inference backend, an added sensor, or an NPU accelerator can be
integrated without a redesign, and this is tracked explicitly (`ROADMAP.md`,
items F10, P5a–b). But the current system should be read as an
engineering demonstration of what is achievable within a fixed, low
hardware budget — not as the most capable system that could be built on
this airframe class. That trade-off (accessibility and mass budget vs.
maximum capability) is deliberate and informs every section that follows.

## System architecture: threading model

The runtime is partitioned into three threads around one invariant: **a
slow perception computation must never increase the latency of the control
path.**

```
   video ──► frame handoff ──────────► DELIBERATOR  (best-effort thread)
     │                                   heavy perception: monocular depth,
     │                                   object detection
     ▼                                   │
   FLY LOOP  (bounded latency, per-frame) │
     capture → cheap perception →        │
     state estimate → mode arbitration → │
     command                             │
     ▲                                   │
     └──────────── WorldModel ───────────┘

   FC LINK (independent ~50 Hz thread): owns the flight-controller serial
   link; services it on its own schedule regardless of video-frame timing.
```

- **Fly loop** (main thread): executes once per captured frame. Reads FC
  telemetry (via `FcLink`), runs only bounded-cost perception (lock-on
  tracker, appearance-based road classifier — single-digit milliseconds
  each), updates the state estimator, evaluates the active control mode,
  and issues a command. No blocking calls to perception modules with
  unbounded runtime.
- **Deliberator** (separate thread): runs the CPU-bound perception modules
  — the monocular depth model, object detector — via a
  `PerceptionScheduler` that allocates a fixed millisecond budget per tick
  and adjusts cadence per module based on which behavior is active. If a
  depth-model inference takes on the order of 100 ms against a ~33 ms frame
  interval, the fly loop continues operating on the most recent completed
  result rather than blocking.
- **FcLink** (separate thread, added after field testing surfaced the
  requirement): owns the `IFlightController` backend exclusively and
  services it at a fixed ~50 Hz independent of camera timing. This exists
  because iNAV's MSP-RC failsafe triggers below approximately 5 Hz command
  rate; coupling FC I/O to camera read latency risks spurious failsafe
  activation on a transient capture stall. Both the fly loop and `FcLink`
  run at elevated (`SCHED_FIFO`) scheduling priority relative to the
  Deliberator, so CPU-bound inference cannot preempt either control-path
  thread under load.

## Shared state: the world model

Cross-thread communication is through a single mutex-guarded struct
(`WorldState`, held by `WorldModel`) — a blackboard architecture. Each
perception module writes its findings (e.g. corridor heading, openness
score, per-field write timestamp) under lock; the control path reads the
current snapshot under the same lock. No module holds a reference to
another module's internal state, and no data crosses threads by any other
path.

The blackboard also carries per-field monotonic write timestamps, since a
perception result is a *latch*, not a heartbeat — a stalled Deliberator
thread leaves the last-written value in place indefinitely. Consumers on
the control path check field freshness (`corridorFresh()`, et al.) against
a configurable staleness threshold rather than trusting a validity flag
alone; a stale-but-valid corridor is treated identically to no corridor.

## Perception pipeline

kestrel does not use LiDAR or a dedicated stereo pair, and in the current
build it has no dedicated camera at all. The video input is a **USB CVBS
capture device** reading the same analog composite signal already routed
from the FPV camera to the video transmitter — the identical feed the pilot
sees in the goggles, digitized by an inexpensive capture dongle presenting
as a standard V4L2 device. The pilot's signal path (camera → VTX → RF →
goggles) is unmodified and remains fully analog; the capture device is a
passive secondary tap, not an inline component, so the perception pipeline
adds no latency to the pilot's video. This is the literal implementation of
using an existing analog FPV link as an autonomy sensor: no additional
camera hardware, no digital video pipeline on the aircraft side, a signal
tap.

From that captured frame:

- A monocular depth model (MiDaS-small or DepthAnything-v2-Small, via
  OpenCV DNN) produces a per-pixel relative depth estimate. This is reduced
  to a single steering direction using **VFH+** (Vector Field Histogram
  Plus): the depth map is collapsed to a one-dimensional polar openness
  histogram, thresholded into free/blocked sectors with hysteresis, and the
  free sector closest to both the current heading and the previous chosen
  heading is selected. The hysteresis term (weighted toward the prior
  heading) is what prevents oscillation between two similarly-open sectors.
- The `ITofSource` interface allows a metric time-of-flight sensor to
  supply the same VFH+ pipeline with a real distance grid in place of the
  monocular estimate. No such sensor is in the current build (see
  `bom.md`); the interface exists so one can be added without modifying the
  steering algorithm.
- Two additional, independent perception modules run in the fly loop: an
  appearance-based road/line follower (CIELab color-space clustering, no
  neural network, low enough cost to run every frame), and a lock-on
  tracker (CSRT/KCF/optical-flow/MOSSE, Kalman-filtered) that maintains a
  bounding box on an operator- or detector-designated subject.

## State estimation

GPS provides an absolute position fix outdoors; it is unavailable or
degraded indoors, which is a known gap for FC-native position hold on this
class of hardware.

kestrel runs a loosely-coupled Kalman filter (`StateEstimator`) fusing GPS,
barometric altitude, and FC attitude, with hooks for visual-odometry
velocity and pose (`updateVisionVelocity`, `updateVisionPose`) — the
integration points for a future VIO front-end (`ROADMAP.md`, P5a). Roll,
pitch, and yaw are taken directly from the FC's own AHRS rather than
re-estimated, on the basis that the FC's Mahony/Madgwick filter output is
more reliable than an independently derived estimate from the same raw
sensors.

The estimator's output is fed back into the flight controller as a
synthetic GPS fix (`MSP2_SENSOR_GPS`), subject to iNAV's own glitch-radius
and fix-age gating. This means kestrel does not reimplement position hold,
loiter, or waypoint navigation — the FC's existing, already-tuned
navigation firmware consumes the synthetic fix exactly as it would consume
a real one. The estimator's role is limited to producing a fix when the
real GPS cannot.

## Control mode arbitration

The system is in exactly one control mode at a time, selected from a
registered set (`IControlMode` implementations, added via
`ModeManager::add()` — no central switch statement). Each mode implements
`update(WorldState&, ControlCtx&) → ControlCmd`, and may return an invalid
command to explicitly release control to the pilot or to the FC's own
navigation firmware.

Two safety layers are applied by `ModeManager::tick()` above whichever mode
is active, independent of which mode that is:

1. **Failsafe** — low battery or an operator abort releases control and
   triggers RTH (driven via a configured AUX channel on the MSP backend, or
   the FC's own RC-loss failsafe if none is configured).
2. **Obstacle reflex** — a motion-capable mode (`isMotion() == true`) flying
   toward a corridor reading below the openness threshold is overridden to
   HOLD. A mode may opt out (`ownsObstacleAvoidance() == true`) only if it
   implements its own avoidance logic; the default assumption is that it
   does not.

One mode is a specific answer to pre-flight validation of the autonomy
stack: **SHADOW**. The pilot retains control (SHADOW always releases, like
FLY); underneath, the full `MissionController` cycle executes and computes
— but does not transmit — the command it would issue, publishing it to the
world model for HUD overlay. This allows the autonomy's output to be
observed against real flight data with zero risk before it is given
authority.

## Motion planning: move-stop-sense

Maintaining a queryable environment map while also flying exceeds the
compute budget available under the design constraint above. Rather than
attempt continuous mapping and replanning concurrently, `MissionController`
implements a stop/sense/move cycle — structurally the same approach used by
early Mars rover autonomy (Curiosity-era "stop and think" navigation,
prior to Perseverance's move to continuous replanning once onboard compute
allowed it).

The cycle (`SETTLE → THINK → MOVE → ARRIVE`, with a `SCAN` branch):

- **SETTLE** — commands zero (the FC's own position-hold mode maintains a
  hover); waits for velocity to converge and a minimum dwell time to elapse.
- **THINK** — evaluates the current corridor reading. If openness exceeds
  threshold, commits a waypoint one step ahead, blending the operator's goal
  bearing with the vetted-open corridor direction (weighted toward the
  corridor as its deflection from center increases, with a hard clamp
  preventing the goal term from steering back into an occluded sector).
  Otherwise transitions to SCAN.
- **SCAN** — rotates in place (no translation) to bring an occluded escape
  route into the field of view, continuing in the direction of the current
  avoidance maneuver rather than reversing, to avoid oscillating at a
  concave obstacle boundary.
- **MOVE** — flies the committed leg, re-evaluating the corridor and
  estimator health continuously; a stale corridor, a degraded position
  estimate (`estEphM` above threshold), or the leg's fixed distance ends the
  leg and returns to SETTLE.

A purely reactive version of this cycle — steering only on what the camera
currently sees — has a known failure mode: it can enter a local minimum on
an obstacle sitting directly between the drone and the goal, because it
forgets the obstacle the moment it leaves the field of view and re-approaches
it. To close that, the mission accumulates observations into a **local
occupancy grid** (a log-odds map in the local ENU frame) and runs a wavefront
planner over it, which routes around obstacles the grid remembers even after
the live camera has forgotten them. The plan supplies a smarter goal
direction into the reactive blend; the live camera still governs speed and
the stop reflex, so a stale or wrong map can never drive the aircraft into
something the live sensor sees. This is validated in the SITL suite: the
on-path and dead-centre obstacle cases that stall the pure-reactive layer are
now completed, with the obstacle standoff never breached.

One honest limitation remains: an occupancy grid is only geometrically sound
if its input distances are metric, which they are on a ranging sensor
(time-of-flight or stereo) but only *approximately* so on a monocular depth
model, which has no absolute scale. On the current camera-only configuration
the grid runs at a nominal scale — good enough to bias routing, not a true
metric map — and can be disabled in config until a ranging sensor is fitted.
This mirrors the real evolution of compute-limited autonomy (early Mars
rovers moved from stop-and-think to continuous replanning only once better
onboard sensing and compute allowed it); the grid is the first step in that
direction here, bounded by the same sensor-and-compute budget as everything
else.

## Flight-controller interface

kestrel communicates with iNAV over **MSP**, the protocol iNAV's own
configurator uses — not MAVLink, which is the default assumption of most
companion-computer tooling (ROS, MAVROS, the PX4-adjacent ecosystem). This
is a deliberate protocol choice: iNAV is widely deployed on existing analog
FPV airframes, requires no companion-computer-class hardware to operate
standalone, and choosing it means the companion-computer subsystem is
strictly additive — the aircraft is fully flight-capable with the Pi
removed. The cost of this choice is that the MAVLink-oriented tooling
ecosystem (PX4-Avoidance, MAVROS, most published companion-computer
reference architectures) is not directly applicable; the MSP integration
(control framing, telemetry parsing, synthetic-GPS injection, RTH-AUX
triggering) is implemented directly against the protocol.

Every computed `ControlCmd` is **dry-run by default** — computed, logged,
and not transmitted — until control is explicitly armed. This is a default
system state, not a runtime check that can be bypassed by omission.

## Verification: software-in-the-loop testing

The failure modes most worth catching before flight — position-estimate
degradation mid-leg, perception-thread stall, failsafe activation — are
exercised in an automated test suite requiring no flight hardware:

- `SimFcBackend` — a kinematic flight-controller model that responds to
  commanded input (commanded pitch produces simulated GPS displacement,
  battery drains, auto-arms after a fixed delay), used both for interactive
  bench testing and for headless SITL runs.
- `sim_autonomy` — a scenario suite driving the real `MissionController`,
  `StateEstimator`, and `SimFcBackend` against synthetic obstacle fields and
  fault injection (perception dropout, GPS loss mid-leg), asserting minimum
  standoff distance is maintained in every scenario and goal completion in
  the scenarios the reactive layer is designed to handle.
- Seven additional unit-test binaries covering the estimator (glitch
  gating, re-acquisition, uncertainty growth), the mode-arbitration safety
  layers, MSP wire-protocol framing (verified against a PTY-attached
  simulated FC), the config parser, the RC command source, the FC I/O
  thread's stale-command handling, and real-time scheduling.

The full suite (eight CTest targets) runs in under three seconds with no
attached hardware — the same discipline ArduPilot and PX4 apply to their
own SITL-based CI.

## Where "kestrel" comes from

There is no recorded naming rationale in this repository's history — the
name was already in use by the first commit that introduced the runtime.
One observation, offered as a plausible connection rather than a documented
fact: the common kestrel (*Falco tinnunculus* and relatives) is
distinguished among raptors by wind-hovering — holding a fixed position in
the air while visually scanning the ground, then committing to a single
targeted stoop. That behavior — hold position, observe, commit to one
motion, repeat — maps closely onto the SETTLE/THINK/MOVE cycle described
above. Whether that connection motivated the name or is coincidental is not
established; it is a reasonable mnemonic regardless.

## References

For the physical hardware and its cost, see [`bom.md`](bom.md). For
complete file-level architecture, invariants, and the plugin-interface
contracts, see `AGENTS.md` (repository root) — written for implementation
work rather than a first read. For adding a new control mode end to end,
see `onboard/docs/adding-a-control-mode.md`. For planned work and its
sequencing rationale, see `ROADMAP.md`.
