# kestrel — a companion-computer autonomy stack for analog FPV drones

**Status: waiting for parts.** The full software stack is built and validated
in software-in-the-loop (see *Current status*, below) — the sensor suite is on
order to validate it end to end on real hardware.

*Project demonstration. This is a working technical demonstrator built to explore
what's achievable on cheap, CPU-only hardware — not a certified or commercially
flown product. Scope and current validation status are stated explicitly
throughout.*

**Repository:** [`henriharmaala1-source/Assembly-grapher`](https://github.com/henriharmaala1-source/Assembly-grapher)
(`onboard/` — the C++ runtime this document describes in detail; `desktop/` —
a separate Python perception-prototyping tool, referenced below for its role
in validating computer-vision algorithms off the aircraft, not otherwise
covered here)

---

## Prologue

Early February 2026, a reconnaissance variant of the Russian Geran/Shahed
strike-drone line was recovered and taken apart. Open-source technical
breakdowns of the wreckage found a **Raspberry Pi 5** on board, running video
processing, alongside a Windows mini-PC handling other functions. Not a
custom board built to a military spec. A single-board computer anyone can
order for a few hundred euros, doing onboard video AI on a live combat
airframe.

That's not where this project's hardware bet came from — the hypothesis this
project was already built on, that cheap, CPU-only, general-purpose compute
is enough for real onboard perception rather than a compromise to be
replaced once better hardware is affordable, predates that find. What the
Geran wreckage did was **confirm** it: independent, real-world evidence,
from an active combat example, that the exact hardware-scope bet this
project holds to is one a live war has separately converged on too.

It isn't isolated. On the other side of the same conflict, Ukraine's
**Hornet** — a fixed-wing strike drone built by the American firm Perennial
Autonomy, in service since March 2026 and used to cut Russian logistics 50–150
km behind the front — navigates primarily by **optical flow**: a downward
camera and a processor tracking terrain motion beneath it, GPS reduced to an
occasional cross-check rather than the primary reference, specifically
because satellite navigation is the first thing a contested electromagnetic
environment takes away. It costs roughly **$5,000** a unit. And on the
Russian side, the production Geran-2 MS variant now carries an **Nvidia
Jetson Orin** module for onboard AI object recognition, correcting its own
terminal-phase trajectory against a moving target from its own video feed —
manufactured at a reported 5,000+ units a month.

A fourth data point: since 2025, a commercial category of **AI lock-on
add-on modules for FPV airframes** has gone from prototype to mass-fielded —
**TFL-1**, from the firm The Fourth Law, chief among them, reportedly around
**$442** a unit. Strapped onto an otherwise ordinary FPV, it takes over
final-approach guidance: detecting, tracking, and closing on a target
independent of the radio link. One Ukrainian brigade using it reported hit
rate going from 20% to 80%. This isn't a research paper or a demo — it's a
live commercial market, at a price point in the same neighborhood as this
project's own BOM.

Four data points, not three, and not confined to one side of the war: when
the radio link and the satellite fix are the first casualties of contact,
the airframe has to be able to see and decide for itself, on hardware cheap
enough to be attritable — and, increasingly, cheap enough to be sold as a
module rather than built once per program. That isn't a future requirement
this project is planning for — it's the ground this project already stood
on, and the Geran wreckage, Hornet, and the TFL-1 category are each
confirmation of it from a real, active-combat or active-market example, not
the source of it. This project runs the same underlying bet at a fraction of
even Hornet's budget, with a scope boundary this document states plainly and
holds to (see *Design decisions*, below).

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

**Scope.** This is a development and defense-research platform: a low-cost
testbed for evaluating autonomy approaches on analog FPV airframes — the class
of platform already in wide field use for reconnaissance and inspection roles
where GPS is degraded and commercial companion-computer hardware is cost- or
mass-prohibitive at scale. It is not built or positioned as a consumer/hobbyist
product, even though it is built from hobbyist-market hardware (see *Market
context*, below) for cost reasons.

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

### Perception scheduling and compute budget

Heavy perception (monocular depth inference, object detection) runs on the
Deliberator thread behind a compute-budgeted scheduler: cheap, always-on
modules first, then whichever heavy module is "hot" for the current
behavior, then the rest — each gated against one flat per-tick millisecond
budget, dispatched **sequentially**, one module completing before the next
starts. Core pinning (the fly loop and FcLink confined to one core; the
Deliberator left the remaining cores) exists to guarantee the control
threads are never starved — but because the Deliberator is itself a single
thread, it can only actively drive one of its allotted cores at a time,
regardless of how many sit free. During the stationary phases of the
move-stop-sense cycle (settle, think, scan) the control threads are doing
almost nothing and several cores sit idle while depth and detection still
run one after another instead of concurrently. That's a scheduling question,
not a compute-budget one: the pieces needed to close it already exist in the
architecture — a mutex-guarded blackboard that's already safe for concurrent
writers, and a flight-phase signal the mission controller already computes —
so the lever is to reschedule cores toward the workload actually present
during a stationary phase, then revert to the flight-speed-safe topology
before motion resumes, so a burst-mode worker can never be live during a
control-critical moment.

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

### Control modes

One arbiter, exactly one mode active at a time, both safety layers above
wrapping all of them:

| Mode | Function |
|---|---|
| `FLY` | Manual passthrough — pilot has full control (the default) |
| `ASSIST` | Bumpless-trim assisted control, blended with pilot input |
| `LOCK_ON` | Sensing only — tracks a subject and publishes a bearing/box for a gimbal or operator; no forward-pursuit control path exists (removed on purpose, see *Design decisions*, below) |
| `HOLD` | Position hold |
| `FOLLOW_ROAD` | Steers along the appearance-based (CIELab) road corridor |
| `WAYPOINT` | The flight controller flies its own GPS route; the OS supervises rather than commands — the obstacle reflex and detection still run over it |
| `AUTONOMY` | The move-stop-sense cycle (settle → think → scan → move → arrive), with the occupancy-grid planner as a goal-bias layer over the live reactive corridor |
| `SHADOW` | Operator flies manually; `AUTONOMY` runs live underneath in dry-run and overlays its intended command on the video feed — zero-risk validation of the autonomy before arming it |
| `FOLLOW_SUBJECT` | Standoff-keeping only, by design — never closes to impact. Registered and wired into the arbiter; the standoff-hold control logic itself is the one entry in this table not yet implemented, so it currently releases control rather than holding station |

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

## Leadership and how AI was used

This project is directed by its author, who owns every architectural and
design decision described in this document — hardware selection, the
CPU-only/analog-FPV cost constraint, the safety and scope boundaries, what
gets built next and in what order. AI (Claude) was used throughout as a
resource multiplier on top of that direction, not as an independent author.
Concretely, across four recurring roles:

- **Market research.** Surveying the competitive landscape for drone
  companion computers and autonomy bridges — comparable products, their
  architectures, and their price points — to sanity-check where this project
  sits (see *Market context*, below).
- **Method research.** Literature and prior-art sweeps across the specific
  algorithm families this project touches — monocular and metric depth
  estimation, visual-inertial odometry, local and global path planning,
  edge-CPU object detection, sensor fusion — rated for feasibility on this
  exact hardware rather than accepted at face value, to separate "worth
  trying now" from "watch, don't build yet."
- **Idea validation.** Using the test suite adversarially: fault-injecting
  scenarios that reproduced a real design flaw (the move-stop-sense local
  minimum) before it was fixed, so the fix had a concrete failing case to
  satisfy rather than a hoped-for improvement. The same scrutiny was applied
  to this document itself — an incorrect duration estimate and a factually
  backwards description of the FC-link failure behavior were both caught and
  corrected during drafting, not left in because they sounded plausible.
- **Cross-referencing against past and current solutions.** Checking design
  choices against precedent before committing to them — the move-stop-sense
  cycle against how Mars rover autonomy evolved from stop-and-think to
  continuous replanning, the VFH+ steering algorithm against its use in
  established open-source obstacle-avoidance systems, and the overall
  reactive/deliberative thread split against the classical three-layer
  architecture pattern it descends from — so decisions were made with
  awareness of who else has solved adjacent problems, and how.

---

## Market context

**Independent market-research estimates** (360iResearch; ResearchAndMarkets)
put the global drone onboard-computer market at roughly **$340–450M in 2025**,
projected to grow at a **13–15% CAGR** to **$890M–970M by 2032**, driven by
demand for AI-enabled autonomy, sensor fusion, and GPS-denied/BVLOS-capable
operation across defense, public safety, agriculture, and infrastructure
inspection. This project sits at the extreme low-cost end of that market and
was not built to compete commercially in it — the comparison below exists to
show where the design choices land, not to claim traction.

**The specific gap this project targets:** a cheap companion computer, built
from deliberately limited (CPU-only, no GPU) hardware, integrated directly
onto an existing analog FPV airframe. Every comparison point below misses
that gap in a different direction — ModalAI and Auterion are onboard, but at
10x+ the cost and with GPU-class compute assumed; Droneforge is cheap and
matches the analog-FPV premise, but moves the compute off the aircraft
entirely onto a ground PC. Nothing in the table is both cheap *and* onboard
*and* built to run on hardware this constrained — that combination is the
gap, and it's the reason the CPU-only constraint in *Problem*, above, is
treated as a first-class design input rather than a limitation to route
around.

**A separate, larger market underlies the hardware this project is built
from**, even though the software targets development/defense-research use
rather than that market directly: the hobbyist/DIY FPV drone market itself.
Estimates vary widely by methodology — from **~$558M in 2025 growing at a
13.7% CAGR** (Virtue Market Research) to **$1.5–1.6B in 2025 growing at
~22.5% CAGR to ~$6.2B by 2032** (Verified Market Research; Market Research
Intellect) — but converge on one point: hobbyists and enthusiasts are the
largest segment by unit volume, the low barrier to entry from ready-to-fly
kits and an active builder community being the primary demand driver. This
project's entire airframe BOM (see `onboard/docs/bom.md`) is sourced from
that hobbyist market, which is precisely what makes the cost target in the
*Problem* section above achievable — commodity analog FPV parts, not
purpose-built or defense-grade hardware.

| | This project | ModalAI VOXL 2 | Auterion Skynode | Droneforge Nimbus |
|---|---|---|---|---|
| Approach | Onboard companion computer, CPU-only | Onboard companion computer + autopilot | Onboard flight-controller-integrated compute | **Ground-based** — compute stays off the aircraft entirely |
| List price | ~€100 add-on (~€476 full platform, airframe included) | **$1,199.99** | Not publicly listed | Not publicly listed (hardware bridges an existing FPV drone) |
| Target buyer | Development and defense-research use on low-cost analog FPV airframes | Commercial/defense integrators | Commercial/defense OEMs building on AuterionOS | Developers building on existing FPV hardware |
| Funding/stage | Personal demonstrator project | Established commercial product | Established commercial product | $2.5M pre-seed (2026) |

The market's commercial players (ModalAI, Auterion) target integrators who
are already buying purpose-built compute at the $1,000+ price point — a
segment this project doesn't compete in and isn't trying to. Droneforge is
the closer conceptual peer: it validates the same underlying premise this
project relies on — that an analog FPV video/telemetry link carries enough
information for real autonomy — but resolves the cost/mass constraint by
moving compute to the ground instead of onto the airframe. This project's
distinct bet is the opposite one: keep the compute onboard, at roughly
**1/12th the cost of a single VOXL 2 unit**, accepting a CPU-only ceiling on
what's achievable in exchange for an aircraft that's autonomous without
depending on a ground link at all.

---

## Design decisions, stated as trade-offs

- **iNAV over MAVLink/PX4.** Betaflight is the dominant firmware on low-cost
  analog FPV airframes, but it's flight-mode-only — no GPS navigation, no
  position hold beyond what its own limited modes offer. iNAV sits between
  Betaflight and ArduPilot: it keeps the FPV-native flight modes and requires
  no companion-computer-class hardware to fly standalone, while adding the
  GPS/nav layer (RTH, position hold, waypoint capability) that Betaflight
  doesn't have — which is exactly the layer this project's synthetic-GPS
  feedback and mode arbitration hook into. MSP (its native protocol) was
  implemented directly rather than adopting the MAVLink-centric tooling
  ecosystem (MAVROS, PX4-Avoidance) most companion-computer references
  assume, which targets ArduPilot/PX4, not Betaflight/iNAV. Cost: none of
  that tooling is directly reusable — the MSP integration (control framing,
  telemetry parsing, synthetic-GPS injection, failsafe triggering) was built
  from the protocol up.
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

## Current status (checkup, re-verified against the live repo)

The project tracks its own backlog against named, numbered phases
(`ROADMAP.md`). This section states exactly what's built and tested today,
re-confirmed by rebuilding and re-running the full suite rather than quoted
from memory.

**Done and currently green — 9/9 tests, 0 failures, full suite under 10 seconds,
no camera or flight hardware attached:**

| Area | What's built |
|---|---|
| Operational hardening (8 items) | Perception-staleness gating + think-tier watchdog; the flight-controller link on its own thread, decoupled from camera timing; mission legs gated on estimator health; ATTITUDE-priority telemetry polling; the altitude-authority invariant written down as an explicit contract, not an implicit assumption; the full test suite itself, brought in-tree; a runtime config file so tuning doesn't require a rebuild; `SCHED_FIFO` real-time scheduling for the two control-critical threads |
| Command layer (4 items) | Failsafe return-to-home wired to a real flight-controller AUX channel (not a no-op); a radio-based command source, so the aircraft is flyable without a laptop; an advisory "shadow" mode that runs the autonomy live and shows its intended commands without ever sending them — a zero-risk way to build trust in the autonomy before arming it; a written, props-off validation procedure for bumpless manual-to-assisted control handoff |
| Autonomous navigation | The occupancy-grid planner (above), plus the FPV-specific camera-tilt handling and the decision to commit a metric ranging sensor as the primary obstacle source rather than depending on monocular depth alone |

**Software-in-the-loop scenario results, from the current build:**

| Scenario | Result |
|---|---|
| Clear field, obstacle off-path, obstacle grazing path | Goal reached, obstacle standoff maintained |
| Obstacle partly blocking the direct path | Goal reached (39 s) — previously stalled before the planner fix |
| Obstacle dead-centre on the direct path | Goal reached (42 s) — previously stalled before the planner fix |
| Perception dropout mid-leg | Aircraft holds position (0.00 m drift after the fault), does not fly blind |
| GPS loss mid-leg | Aircraft holds position, does not fly on a degraded estimate |

7/7 scenarios keep the obstacle standoff distance; 5/5 of the scenarios
designed to reach a goal do so; both fault-injection scenarios correctly stop
rather than continue.

**Deliberately not yet built**, in the order the project intends to take them
next: a faster on-device inference backend (scoped, deferred until it's
actually the bottleneck); a flight data recorder and replay tool; visual
odometry for better GPS-denied velocity; an on-device advisory LLM supervisor,
constrained to a whitelisted command schema and never in the control loop;
a ground-station view; and mission-capability extensions (a broader object
detector, standoff-only subject following, multi-goal missions, a geofence).
None of these are represented as built anywhere in this document.

**Core computer-vision algorithms were validated separately, on a desktop
workstation, before this integration existed.** Two lines of that work: the
onboard tracker (CSRT/KCF/optical-flow/MOSSE + Kalman-filtered lock-on) was
originally built and iterated as its own standalone C++ tool, run against
live camera input, before being wrapped as the runtime's tracking module; and
the `desktop/` app is a separate, parallel perception-prototyping line that
evaluates heavier backends (DINOv2 + SAM 2) against real video off the
aircraft, deliberately kept independent so perception ideas can be tested
without needing the drone. That's real, separate validation from the
software-in-the-loop suite above — the tracking algorithms have been run
against live video; the *onboard integration* (threading, the mode arbiter,
the estimator, the planner, the flight-controller link, all running
together) has not yet been run against real hardware.

**Not yet flight-tested.** This is the honest boundary that remains: the
project is currently waiting on parts — the sensor suite (forward
time-of-flight sensor, GPS/compass module) is on order at the time of
writing — to validate the full onboard integration end to end. No claim is
made here about behavior beyond what the software-in-the-loop suite and the
desktop-validated algorithms actually support.

---

## Tech stack

C++17 · CMake · OpenCV (DNN inference, image processing) · CTest · POSIX
threads and real-time scheduling (`SCHED_FIFO`) · MSP (iNAV serial protocol,
implemented directly) · I²C (Linux `i2c-dev`) sensor backends · a hand-rolled
log-odds occupancy grid and wavefront planner · a loosely-coupled Kalman
filter · Raspberry Pi 5 (target deployment platform, CPU-only)

## What this project demonstrates

The author did not write the code — see *Leadership and how AI was used*,
above. What this project demonstrates is the set of skills required to direct
work like this at all, independent of who typed the implementation:

- **Project management.** Turning an ambiguous, hardware-constrained problem
  into a phased, trackable backlog (`ROADMAP.md`), sequencing work so each
  phase is independently testable without the target hardware, and adjusting
  scope in response to what testing actually found (the P5b local-minimum fix
  was scheduled and prioritized after the SITL suite surfaced it, not before).
- **Product management.** Defining and holding a scope boundary under real
  pressure to let it creep — the target-homing exclusion (see *Design
  decisions*, above) is a product call, not a technical limitation, revisited
  and deliberately held. Same for the cost constraint itself: treating "cheap,
  CPU-only, attaches to an existing airframe" as a non-negotiable requirement
  that shapes every downstream decision, rather than an aspiration abandoned
  the first time it became inconvenient.
- **Autonomy and machine-vision research.** Directing the literature and
  prior-art sweep across depth estimation, VIO/SLAM, local and global
  planning, and sensor fusion (see *Method research*, above), and rating each
  option for feasibility on this specific hardware budget rather than
  accepting vendor or paper claims at face value.
- **Research testing and validation methodology.** Designing the validation
  strategy itself: what a hardware-free software-in-the-loop suite needs to
  prove before hardware is trusted with it, what fault conditions are worth
  deliberately injecting (perception dropout, GPS loss), and what counts as
  sufficient evidence versus an unverified claim — the standard this document
  itself was held to throughout (see the corrections noted under *Idea
  validation*, above).
