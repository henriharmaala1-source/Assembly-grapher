# Project CV — what was built, what is claimable, what it takes to claim it

The other documents describe the work. This one describes the *position*: what
exists, what role was actually played, what can be said honestly, and the
specific missing artifacts that turn an arguable claim into a checkable one.

| file | is |
|---|---|
| `THESIS.md` | what the project claims technically, and what would prove it |
| `NOTES.md` | the lab notebook — tried, measured, rejected |
| `PROJECT.md` | the public technical write-up |
| `ROADMAP.md` | phase plan, P2–P6 |
| `onboard/docs/MAVLINK_BRIDGE_PLAN.md` | where state estimation lives under ArduPilot |
| **this** | role, inventory, claims, and the gaps in the record |

Rewritten 2026-08-12 after a full survey of the tree. Motivation is stated
plainly in `THESIS.md` §1.3: portfolio piece and plausible route to funding.

---

## 1. Inventory — what actually exists

**~42,000 lines across five subsystems**, none of it framework glue: no ROS, no
Gazebo, no GPU, and (in `nav-sim`) no dependency beyond OpenCV.

| subsystem | lines | what it is |
|---|---|---|
| `nav-sim/` | 13.8 k | the world model, planner, sim, and live viewer |
| `onboard/` | 13.7 k | the C++ flight runtime |
| `desktop/` | 10.2 k | the tracker's validation harness — 26 eval scripts |
| `android-tracker/` | 3.1 k | the lock-on tracker + field test rig |
| `android/` | 1.2 k | navviz, the navigation-validation rig |

### 1.1 Perception and mapping (`nav-sim/`)

* **Three-state log-odds voxel map** with the doctrine that drives everything:
  free / occupied / **unknown**, where unknown is never treated as free.
* **Honest range derived, not assumed.** `Z_max = √(cell·f·B/σ_d) × 0.75`, with
  the derate measured against a false-free table (stereo at 8 m gives 2.95 %
  false-free, 25 m gives 13.22 %; truth depth gives 0.00 % at every range, which
  is what proves the mapper rather than the sensor).
* **Carving and marking as separate decisions** (`maxCarveM` vs `maxIntegM`) —
  the fix for a bug that left the map completely empty and the aircraft
  motionless for 1200 steps.
* **Minimum trusted range** from Intel's own `MinZ = f·B/126`, not a guess.
* **Multi-resolution ladder**, 0.10 / 0.25 / 2.0 m, each rung honest to its own
  `Z_max`, banded so no level is consulted inside a finer level's range.
* **Stereo camera model** including the occlusion shadow — `f·B·(1/Z_n − 1/Z_f)`
  px falling on the *left* of near objects — the IR projector as a texture floor
  with `1/(1+(r/R)²)` falloff, speckle, and a subpixel noise model.
* **Trajectory planner**: 210 body-frame primitives rolled out with the
  vehicle's own velocity lag, swept-volume admissibility, and a speed budget
  that only pays for **confirmed-free** distance.
* **`voxel_live`** — the same map and planner over real depth, with runtime
  loading of librealsense (no build-time SDK, no headers, no import lib, API
  version read from the library itself), `.kdr` recording, replay, and a
  four-pane diagnostic view.

### 1.2 Lock-on tracker — "TFL1" (`android-tracker/`, `desktop/`)

Reverse-engineered from reference terminal-guidance footage. Designed to hold a
50–800 m target on a low-Hz, low-contrast analog feed **without a gimbal and
without a learned model**:

* **Followed crop** — resampled around the prediction so the target stays a
  *constant size inside the crop* whatever the range. That is what keeps
  correlation stable across a 16× range change.
* **NCC template matching** with selectable appearance filters, plus a
  hand-written **MOSSE/FFT** correlation filter measured at ~4.4× cheaper for
  equal coverage (validated numerically, deliberately not the default until
  real footage decides it).
* **PSR (peak-to-sidelobe ratio) as a per-frame confidence**, driving a
  lock / coast / lost / searching state machine — the health signal a boolean
  flag cannot give.
* **Optical-flow ego-motion feed-forward**, consensus-gated (sim pan: edge
  error 3.7 → 0.4 px, zero change elsewhere).
* **Appearance bank** of diverse-pose keyframes with targeted consult.
* **Occlusion-aware adaptation** — a PSR-drop detector freezes adaptation and
  scale (noisy-occlusion edge 91 → 98 %).
* **`desktop/simtrack.py`** — a faithful Python mirror of the Kotlin tracker, so
  every change is A/B'd in minutes before being ported. Plus ~26 eval scripts:
  noise floor, re-detect, search capacity, learned-baseline comparison, drift.

**And the test rig is itself a design decision worth claiming.** The tracker is
*camera-bound, not compute-bound*, so a phone running the same analog capture
dongle over USB-OTG — same ~30 fps, same ~150 ms latency, same interlacing —
faithfully predicts Pi behaviour while being a self-contained field rig you can
carry to a real 800 m target. That equivalence is stated explicitly and its
limits are stated with it (it holds only because there is no model; a heavy
model would run faster on the phone and flatter the Pi).

### 1.3 Flight runtime (`onboard/`)

* **Dependency-free MAVLink v2 codec** — 14 messages, CRC_EXTRA table,
  trailing-zero truncation, size-sorted field order, zero-extension — pinned
  against pymavlink golden frames. Signed frames are *rejected* rather than
  mis-parsed, and payloads are zero-extended so a short frame reads as defaults
  rather than garbage.
* **Loosely-coupled Kalman state estimator**: linear core, all nonlinearity
  (geodetic projection, body→world rotation) pushed to the measurement
  boundary — a deliberate choice over a fragile full EKF.
* **Behaviour arbiter** over nine modes, a crash-survivable black box, a
  real-time scheduler, an I²C HAL, ToF sources, and a SITL suite.

### 1.4 Deliberately unbuilt

Four plan documents and four `ideas/` notes for components that do **not**
exist, each with its own argument, cost model and validation plan:
depth supervisor, appearance/blobs, pose & bearing-space openness,
context-gated perception, learned decision-making, mission language.

---

## 2. The role, named accurately

**Systems integration + technical architecture + research direction.**

Not project management — no schedule, budget, team or stakeholders. Not software
implementation of the perception and planning code.

Reviewing work you did not write, and finding defects in it, is **engineering,
not management**. On 2026-08-12 alone, five real map defects were found by
looking at screenshots from a real camera and asking why the picture disagreed
with the claim: a ladder that collapsed to its coarsest rung, a missing minimum
range, a carve guard switched off, every carve guard written in metres against
cells, and a range threshold that rendered as a circular blind spot. None was
found by the tests, the review, or the sim.

## 3. Claimable

* **Thesis, scope and constraint framing.** Autonomy in a GNSS-denied and
  video-denied zone on cheap CPU-only compute — three chosen constraints, two
  from one operational scenario. `THESIS.md` §1.0.
* **The vehicle.** Part selection, airframe build, and the integration that makes
  it one system — including LiPo → Pi 5 over USB-C PD, which is not a lesser
  layer: the Pi 5 will not grant full performance without a source that
  *negotiates* 5 V/5 A, and from 6S that is ~22 V to 5 V at 25 W on a bus shared
  with ESCs switching tens of amps. Inrush, ripple, ground loops, pack sag
  browning out the companion computer on a punch-out, vibration, thermal.
  **Integration is the definitional core of systems engineering and it is where
  most hobby autonomy attempts die.**
* **The instrument designs.** The phone-as-Pi-equivalent rig; `simtrack.py` as a
  fast mirror of flight code; `.kdr` record/replay so a change is A/B'd on
  identical input rather than on a scene that moved.
* **Architecture decisions and trade studies.** Sensor vs achievable speed,
  memory vs drift, IMU vs VO, awareness vs permission, 2D vs 3D representation
  by information content.
* **The epistemic standard.** Negative and inconclusive results kept *with their
  numbers* rather than deleted — the depth improver and the sideslip coupling are
  both in the tree, both off by default, both with the measurement that says why.

## 4. Not claimable — say so first, before anyone asks

* **Implementation of the perception and planning code.** AI-assisted. Git
  history makes this checkable, so honesty is strictly the better play.
* **Novelty of the ideas.** Scan matching, submapping, local SLAM, VFH, canopy
  gap fraction, MOSSE, PSR gating — all predate this project. The true and
  stronger claim: **independently derived from constraints, then located in the
  literature**, which the notes' timestamps support.

---

## 5. TODO — the systems-engineering artifacts

A weekend of **writing**, not building, and what a reviewer will probe.

- [ ] **Requirements baseline.** No requirement *set* exists: speed, endurance,
      environment, lighting, failure tolerance, recovery behaviour. **If the
      co-founder conversation goes ahead, get their definition of "a proper
      point" in writing — that is this artifact, handed over for free.**
- [ ] **Budgets with allocations and reserve.** Mass, power, thermal, latency,
      compute. The power engineering is done; the budget is not written. Latency
      has a live consequence — `reactS = 0.25 s` is assumed exactly like σ_d.
- [ ] **Interface control.** `grep VoxelMap onboard/` returns nothing: `nav-sim`
      and `onboard` share zero code. `THESIS.md` P3, a flight blocker.
- [ ] **THE STATE MODEL DID NOT FOLLOW THE FLIGHT-STACK DECISION.**
      *Architecture designed: `onboard/docs/MAVLINK_BRIDGE_PLAN.md`.* The answer
      is that **v1 should estimate position nowhere at all** — the planner is
      body-frame, the map is local and short-lived, and nothing in the thesis
      needs a global position. The Pi does perception and local planning; the FC
      does attitude and rate. That deletes the Pi-side Kalman from the flight
      path rather than porting it. Two flight blockers fall out: an `ATTITUDE`
      decoder (~30 lines, in the enum, no decoder), and a `SET_ATTITUDE_TARGET`
      command path — because **GUIDED velocity setpoints need a horizontal
      velocity estimate, and GNSS-denied there isn't one.** (The cheap way out
      of that constraint later is an optical-flow sensor giving
      `EK3_SRC1_VELXY = OpticalFlow`; see `MAVLINK_BRIDGE_PLAN.md` §3.2.)
      `state_estimator.hpp` is still written against **iNAV** throughout: its
      constants are copied from iNAV (`gpsTimeoutS` "matches
      INAV_GPS_TIMEOUT_MS", `glitchRadiusM` "matches INAV_GPS_GLITCH_RADIUS"),
      its rationale is that iNAV's complementary filter cannot fuse VO, and its
      output path is a synthetic GPS fix for **`MSP2_SENSOR_GPS`**.
      **Under ArduPilot every one of those is the wrong shape**: EKF3 takes
      `VISION_POSITION_ESTIMATE` over MAVLink, `EK3_SRC*` selects sources
      directly, and the iNAV timeout and glitch constants have no meaning. The
      MAVLink *bridge* was rewritten; the estimator's premise was not. Resolve
      before any GNSS-denied flight — and note it interacts with the
      `THESIS.md` §1.0 exclusion claim, since the estimator is *designed* to
      feed a position back to the FC.
- [ ] **GNSS-exclusion verification** as a standing pre-flight item: `EK3_SRC*`
      logged per flight, plus the connected-vs-unplugged controlled pair.
- [ ] **V&V matrix.** Good tests exist (8 nav-sim ctest targets, paired A/Bs,
      golden-frame MAVLink checks, a SITL suite, 26 tracker evals). What is
      missing is requirement → demonstration traceability.

- [ ] **Run the `THESIS.md` P1 benchmark under flight power and thermals**, on
      **outdoor** data — indoor recordings flatter the number by ~30 %, because
      cost scales with scene depth. This is a *measurement*, and it is the one
      that decides whether the compute claim survives contact with the field.
- [ ] **`docs/HARDWARE.md` — write down the airframe that already exists.** The
      modules are built and integrated; the record is not. Parts, the power
      chain and *why that chain*, what failed first, measured rail behaviour
      under throttle, thermal, vibration mounting. **The hardest integration
      work in this project is currently invisible in the artifact**, and by this
      project's own rule undocumented work did not happen. Writing, not buying.

## 6. TODO — demo assets

> A video of a drone not hitting trees is weak evidence. Every viewer has seen
> DJI do it since 2016, and **the constraints that make this interesting are
> invisible in footage.**

- [ ] **Four-pane overlay as picture-in-picture**, with
      `Raspberry Pi 5 · N ms/frame · no GPU · GPS: 14 sats, not used for nav`
      burned in. *Having* a fix and declining to use it is a far stronger claim
      than not having one, which could just mean flying under canopy.
- [ ] **Texture-mapped rendering.** The competitor's reconstruction reads far
      better than coloured cubes, and it is a rendering choice rather than a
      quality difference — their depth has the same black trunks yours does.
- [ ] **Keep the raw `.kdr` of every demo run.** Video gets attention; the repo
      survives the scrutiny that follows.

## 7. TODO — personal

- [ ] **Read this repository closely — the derivations are in it.** The exposure
      is the gap between directing and building, and someone will probe it: why
      `carveSigK` exists, derive `Z_max`, what `recentre` does at the boundary.
      The mitigation is unusually available here, because the comments and
      `NOTES.md` were written to be read.

Two things are true at once and both should be said out loud: this is a capable
systems-integration and architecture role across five subsystems and ~42 k
lines, and the software implementation layer is directed rather than written.
The second is the more fixable of the two.
