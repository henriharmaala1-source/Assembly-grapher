# kestrel — a companion-computer autonomy stack for analog FPV drones

*Project demonstration. This is a working technical demonstrator built to explore
what's achievable on cheap, CPU-only hardware — not a certified or commercially
flown product. Scope and current validation status are stated explicitly
throughout.*

**Repository:** [`henriharmaala1-source/Assembly-grapher`](https://github.com/henriharmaala1-source/Assembly-grapher)
(`onboard/` — the C++ runtime described below; `desktop/` — a Python
perception-prototyping tool, out of scope for this document)

---

## Summary

A Raspberry Pi 5 companion computer that adds obstacle-avoidance corridor
steering, GPS-denied position-hold support, and a small set of semi-autonomous
flight modes to an existing analog FPV multirotor — without modifying the
flight controller's own attitude-control loop. CPU-only (no GPU, no ROS), built
around a real-time-scheduled two/three-thread architecture, a plugin system for
every extension point, and a headless software-in-the-loop test suite that
validates the full perception→estimation→planning→control pipeline before any
hardware is required.

**Scale:** 24 source files and 29 headers, C++17, 9 CTest targets (in-tree,
hardware-free, runs in under 3 seconds), one custom occupancy-grid planner, one
loosely-coupled Kalman estimator, four plugin interfaces, built iteratively over
roughly 14 weeks (March–July 2026).

---

## Problem

FPV flight controllers (iNAV, Betaflight-derived firmware) close a fast, proven
attitude-control loop but have no exteroceptive sensing: no obstacle detection,
no semantic scene understanding, and no reliable position hold where GPS is
degraded or absent. Adding that capability conventionally means a Jetson-class
board, LiDAR, and a ROS stack — hundreds of dollars and enough added mass to
change the aircraft's flight characteristics entirely.

**The constraint this project was built against:** the companion-computer
subsystem must attach to an airframe that already flies on its own, at a cost on
the order of the airframe itself (not a multiple of it), with no GPU. That
constraint is deliberate and is treated as a first-class design input, not an
obstacle worked around after the fact — it shapes every architectural decision
below.

---

## Architecture

### Real-time threading model

The runtime is split across three threads around one invariant: **a slow
perception computation must never increase control-loop latency.**

- **Fly loop** (main thread) — runs once per captured frame: reads flight-
  controller telemetry, runs only bounded-cost perception (a few ms per
  module), updates the state estimator, evaluates the active control mode, and
  issues a command.
- **Deliberator** (dedicated thread) — runs the CPU-bound perception (monocular
  depth inference, object detection) via a compute-budgeted scheduler that
  adjusts module cadence to the active behavior. The fly loop consumes the
  most recent completed result rather than blocking on a new one.
- **FcLink** (dedicated thread, added after a real integration issue surfaced
  it — see *Engineering process* below) — owns the flight-controller serial
  connection exclusively, servicing it at a fixed ~50 Hz independent of camera
  frame timing, so a transient capture stall can't drop the command rate below
  the flight controller's failsafe threshold.

The fly loop and FcLink run at elevated (`SCHED_FIFO`) OS scheduling priority
relative to the Deliberator, so CPU-bound inference cannot preempt either
control-path thread under load — a soft guarantee (separate threads) upgraded
to a hard one (scheduler-enforced) once field testing showed the difference
mattered.

### Shared state: a mutex-guarded blackboard

Cross-thread communication goes through a single struct (`WorldState`) behind
one lock — a blackboard architecture. Perception writes findings with
per-field write timestamps; the control path reads a snapshot. Fields are
treated as **latches, not heartbeats**: a stalled thread leaves its last value
in place indefinitely, so every control-path consumer checks a field's
freshness against a configurable staleness threshold rather than trusting a
validity flag alone. This was a deliberate response to a real failure mode
(see below), not a default assumption.

### Plugin interfaces (four extension points)

Every capability is added by implementing an interface and registering it —
never by editing a central switch statement:

| Interface | Purpose | Concrete implementations |
|---|---|---|
| `IControlMode` | A control policy: given world state, emit a command or release control | 9 modes — manual passthrough, assisted (bumpless trim), autonomous move-stop-sense, GPS-route supervision, hold, road-follow, lock-on (sensing only), advisory shadow mode, standoff-follow (placeholder) |
| `IPerceptionModule` | Reads a frame, writes findings into the blackboard | Monocular depth corridor, ToF corridor, appearance road-follow, object detector, lock-on tracker |
| `IFlightController` | Abstracts the FC link | MSP (iNAV) — implemented and verified against firmware; a kinematic simulator for hardware-free testing; MAVLink — stubbed |
| `ITofSource` | A metric depth sensor | VL53L5CX/L9CX I²C backends, a serial-framed MCU sensor-hub backend, a simulated source |

Two safety layers are applied uniformly above whichever mode is active:
low-battery/operator-abort → return-to-home (driven over a configured RC AUX
channel), and an obstacle reflex that overrides a motion-capable mode to hold
position when the sensed corridor closes — with an explicit opt-out for modes
(like the autonomous cycle) that implement their own, better avoidance.

### Perception

The video input is not a dedicated camera: it's a USB capture device reading
the *same analog composite signal* already routed from the FPV camera to the
video transmitter — a passive tap that adds no latency to the pilot's own
feed. From that signal: a small monocular depth model (via OpenCV DNN) feeds
**VFH+ (Vector Field Histogram Plus)** steering — a polar openness histogram
with hysteresis weighted toward the previous heading, which is what prevents
oscillation between two similarly-open headings. The same steering algorithm
accepts a metric time-of-flight sensor in place of the monocular estimate
through the `ITofSource` interface, with no change to the algorithm itself.
A known geometry problem specific to FPV — the camera is mounted at a fixed
up-tilt so the horizon centers during fast forward flight, which means it
points at the sky during a hover — is handled by feeding the effective
(airframe-attitude-plus-mount-tilt) elevation into the de-rotation step and
suppressing grid observations taken outside a usable elevation window.

### State estimation

A loosely-coupled Kalman filter fuses GPS, barometric altitude, and
flight-controller attitude (taken directly from the FC's AHRS rather than
re-estimated, since a companion computer re-deriving attitude from the same
raw sensors the FC already fused adds noise, not information). Its output is
fed back to the flight controller as a synthetic GPS fix (`MSP2_SENSOR_GPS`),
subject to the FC's own glitch-radius and fix-age gating — so the companion
computer never reimplements position hold or waypoint navigation; it only
supplies a position when the real GPS can't.

### Autonomous motion: move-stop-sense, with a local-minimum fix

A pure "stop, sense, move" cycle (deliberately analogous to early Mars-rover
autonomy, which used the identical pattern for the identical reason — limited
onboard compute) has a known failure mode: a purely reactive version forgets
an obstacle the moment it leaves the field of view and can re-approach it,
stalling on any obstacle sitting on the direct path to the goal. This was
reproduced deliberately in the test suite (below) before being fixed: a
rolling local occupancy grid (log-odds, accumulated from the perception
corridor scan) feeds a wavefront planner whose output becomes the goal
direction for the existing reactive blend — the live sensor corridor still
governs speed and the stop reflex, so a stale or wrong map can never drive the
aircraft into something the live sensor sees. Getting this to actually work
required two non-obvious fixes found by instrumenting a failing case rather
than guessing: planning with a wider obstacle inflation than the live safety
margin (otherwise the planned path skims the edge and crawls), and making the
planner robust to the vehicle's own position falling inside that wider margin
(snap the search start to the nearest reachable free cell instead of failing).

---

## Engineering process

The project was built test-first wherever the target hardware wasn't
available, which was most of the time. Rather than deferring validation until
hardware existed, each subsystem got a standalone test compiled against the
real production sources:

- A kinematic flight-controller simulator (`SimFcBackend`) that actually
  integrates commanded input into evolving telemetry — commanding forward
  pitch measurably moves its simulated GPS position — used for both
  interactive bench testing and headless scenario runs.
- A **software-in-the-loop scenario suite** driving the real mission
  controller, state estimator, and simulated flight controller against
  synthetic obstacle fields, with deliberate fault injection (perception
  dropout mid-leg, GPS loss mid-leg) — asserting minimum standoff distance is
  held in every scenario, and goal completion in the scenarios the reactive
  layer is designed to handle. This suite is what *found* the local-minimum
  failure mode described above, before it was fixed — the fix's acceptance
  criterion was flipping two previously-asserted-failing scenarios to passing
  without regressing any of the others.
- Eight further unit-test binaries covering the estimator (glitch gating,
  re-acquisition, uncertainty growth under GPS loss), the mode-arbitration
  safety layers, the wire protocol to the flight controller (verified against
  a PTY-attached simulated FC — including channel order, checksum, and
  telemetry decode), the FC-link thread's behavior under a stalled control
  loop (it must substitute a neutral hover, not keep repeating a stale motion
  command indefinitely), the config parser, the RC command source, real-time
  thread scheduling, and the occupancy-grid planner in isolation.

The full suite — 9 CTest targets — runs in under 3 seconds with no camera, no
flight controller, and no network, the same discipline major open-source
flight-controller projects (ArduPilot, PX4) apply to their own CI.

One integration issue is worth naming specifically because it shaped the
architecture: early testing showed that coupling the flight-controller link's
service cadence to the camera-capture loop risked the FC's own failsafe
triggering on a transient camera stall that had nothing to do with the link
itself. The fix — giving the FC link its own thread and, subsequently, its
own elevated OS scheduling priority — is the kind of failure mode that's easy
to miss in a synchronous prototype and only shows up under the kind of
adversarial, fault-injecting testing described above.

---

## Design decisions, stated as trade-offs

- **iNAV over MAVLink/PX4.** iNAV is already the dominant firmware on
  low-cost analog FPV airframes and requires no companion-computer-class
  hardware to fly standalone; MSP (its native protocol) was implemented
  directly rather than adopting the MAVLink-centric tooling ecosystem
  (MAVROS, PX4-Avoidance) most companion-computer references assume. Cost:
  none of that tooling is directly reusable — the MSP integration (control
  framing, telemetry parsing, synthetic-GPS injection, failsafe triggering)
  was built from the protocol up.
- **Monocular + optional ToF over LiDAR/stereo.** Matches the cost and mass
  budget; the trade-off is an occupancy grid that's only *geometrically*
  sound with metric input, so the monocular path runs at a documented nominal
  scale until a ranging sensor is fitted — stated as a limitation, not hidden.
- **Move-stop-sense over continuous replanning.** The right answer for the
  stated compute budget today; the occupancy grid is the first step toward
  the continuous-replanning upgrade path this trade-off explicitly anticipates.
- **A stated scope boundary.** The system deliberately does not implement
  target-homing flight control — a tracked subject can be sensed and kept
  centered in view, but the control path from "here is the tracked box" to
  "here is a flight command toward it" does not exist in the codebase. This
  was a deliberate, revisited-and-held engineering decision, not an
  unfinished feature: detecting and tracking is a sensing problem; steering an
  aircraft onto something is a categorically different one, and this project
  stays on the sensing side of that line.

---

## Current status (honest accounting)

**Validated in software-in-the-loop, with no flight hardware required:**
two/three-tier threading and real-time scheduling; the blackboard and
staleness gating; the mode-arbitration safety layers; the state estimator's
GPS glitch/re-acquire behavior and uncertainty growth under signal loss; the
MSP wire protocol; the occupancy-grid planner and its fix for the
local-minimum failure mode; the full autonomous mission cycle against
synthetic obstacle fields, including fault injection.

**Not yet flight-tested.** This is the honest boundary: hardware for the
sensor suite (forward time-of-flight sensor, GPS/compass module) is on order
at the time of writing, and no claim is made here about behavior beyond what
the software-in-the-loop suite actually exercises. Everything above is stated
at the level of confidence the tests support — no more.

**Explicitly out of scope for this document:** the `desktop/` Python
perception-prototyping tool, and forward roadmap items (an on-device
supervisory LLM, a flight data recorder, visual-inertial odometry) that exist
as scoped, sequenced backlog items but are not built.

---

## Tech stack

C++17 · CMake · OpenCV (DNN inference, image processing) · CTest · POSIX
threads and real-time scheduling (`SCHED_FIFO`) · MSP (iNAV serial protocol,
implemented directly) · I²C (Linux `i2c-dev`) sensor backends · a hand-rolled
log-odds occupancy grid and wavefront planner · a loosely-coupled Kalman
filter · Raspberry Pi 5 (target deployment platform, CPU-only)

## Skills this project demonstrates

Real-time systems design under a hard control-latency constraint · concurrent
programming with explicit thread-safety guarantees (not just "it hasn't
crashed yet") · sensor fusion and state estimation · control-systems tuning
(VFH+ hysteresis, EKF gating) · algorithm design and debugging (the occupancy
planner's two non-obvious fixes were found by instrumenting a reproducible
failing test case, not by guessing) · safety-critical software practices
(dry-run-by-default, layered failsafes, fail-safe-not-fail-silent staleness
handling) · protocol-level embedded integration (MSP, verified against real
firmware behavior) · test-first development without target hardware · and
technical documentation written to be both a working reference and an honest
account of what is and isn't proven.
