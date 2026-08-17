# ARCHITECTURE — pipeline, choices, and what is actually decided

Written 2026-08-17, in answer to "is it planned already?" — **yes, substantially,
in two good documents that are each partly overtaken.** This file is the map
between them, the current code, and reality.

**Source-vs-behaviour warning.** Per the freshness map in `NOTES.md`:

| area | state |
|---|---|
| `nav-sim/` — `voxel_sim`, `voxel_live`, `bearing_field`, `voxel_map` | **fresh** |
| `desktop/tracker/` | **fresh** |
| `onboard/` — world model, deliberator, perception, `depth_nav.cpp` | **stale** |
| `android/` — `navviz` | mobile test bed; source of the close-field result |

Everything below marked *(code)* was read from source, which records what was
written, not what worked. Everything marked *(measured)* has a number behind it.

---

## 0. What is already planned, and what has overtaken it

### `onboard/docs/MAVLINK_BRIDGE_PLAN.md` — 163 lines, **still valid**

The FC↔Pi link is **designed and the design is right**. It picks between three
architectures and commits to one:

| | who estimates position | Pi sends | verdict |
|---|---|---|---|
| A | the Pi; FC consumes it | synthetic GPS / vision pose | **retire** — double filtering |
| B | the FC; Pi sends measurements | odometry increments + covariance | right for v2 |
| **C** | **nobody** | body-frame attitude commands | **right for v1** |

The argument for C is the strongest single decision in the project: the thesis
needs **no global position**, the planner is body-frame by construction, the map
is local and short-lived, and the output is a bearing and a speed. So v1 should
not estimate position anywhere — not on the Pi, and not by feeding ArduPilot a
fiction. Under A, EKF3 receives an already-filtered Pi estimate and treats it as
an independent measurement: two filters in series, neither aware of the other,
and a covariance that means nothing.

**Status: design only, and the two v1 blockers are still open.** *(code)*
`MSG_ATTITUDE` (30) and `MSG_SET_ATTITUDE_TARGET` (82) are in the enum and in
the CRC_EXTRA table (`mavlink_v2.cpp:14,20`) — **and there is no decoder and no
command path for either.** Those are items 1 and 3 of that plan's order of work,
and nothing flies without them.

### `nav-sim/docs/POSE_AND_OPENNESS_PLAN.md` — 215 lines, **§1 now DONE**

Says "planned, nothing implemented". That is out of date: **§1, the angular
far-field openness map, is built** — `nav-sim/bearing_field.{hpp,cpp}`, with a
9th ctest target pinning its behaviour. It shipped *better* than planned: 1°
azimuth bins rather than 2°, plus confirmation, a fill-fraction gate, and
neighbour consensus, none of which were in the design.

§2 (far field supplies the goal, not a score term), §3 (bounded memory), §4 (tile
index), §5 (scan-matching odometry), §6 (IMU attitude) are **not built**.

§7's precondition is still unmet and was re-confirmed today: **nothing in this
project has ever run on a Pi.** Every timing is dev-box.

---

## 1. The pipeline, end to end

```
   analog FPV camera ──► USB capture ──┐
                                       ├──► frame
   D435i (depth + IR + IMU) ───────────┘      │
                                              ▼
                    ┌───────────────── PERCEPTION (§4) ─────────────────┐
                    │  near: VoxelMap        far: BearingField          │
                    │  0.25 m cells          360 az × 48 el, 1° bins    │
                    │  3-state log-odds      nearest confident return   │
                    │  honest to Z_max=3.54m awareness only, no veto    │
                    └──────────┬─────────────────────┬─────────────────-┘
                               │                     │
                          near map                far profile
                               │                     │
                               ▼                     ▼
                    ┌──── PLANNING ────┐    ┌── OBSTACLE_DISTANCE ──┐
                    │ TrajectoryPlanner│    │  72 bins → ArduPilot  │
                    │ primitives +     │    │  independent avoidance│
                    │ sphereClear veto │    └───────────────────────┘
                    └────────┬─────────┘
                             │  bearing + speed
                             ▼
                    ┌──── WORLD MODEL (§3) ────┐
                    │  WorldState blackboard   │
                    │  latch + stamp + Fresh() │
                    └────────┬─────────────────┘
                             ▼
                    ┌──── FcLink (own thread) ──┐
                    │ SET_ATTITUDE_TARGET       │  ← NOT BUILT
                    │ stale cmd → NEUTRAL hover │
                    └────────┬──────────────────┘
                             ▼
                        ArduPilot (attitude + rate control)
```

**The load-bearing split** is near/far, and it is the project's best idea. Stereo
uncertainty is anisotropic and the anisotropy grows with range: along the ray
`δZ = Z²σ/(fB)`, across it `Z/f`, ratio exactly `Z·σ/B` — 10:1 at 2 m, 100:1 at
20 m. A cube must be sized for the worse of the two, so a voxel honest in range
at 20 m is 8 m wide and throws away the 4.5 cm of lateral detail the sensor still
has. Hence: **cubes near, bearings far.** *(measured)*

**And they have different authority.** The near field may veto a primitive;
the far field may not. `sphereClear` hard-rejects OCCUPIED anywhere in the robot
ball, so an invented near cell deletes a manoeuvre — whereas an invented far
bearing costs only a bad openness score. *Inventing awareness is cheap;
inventing permission is not.* That asymmetry is why a learned model is
admissible in the far field and nowhere else (`FAR_FIELD_MODELS.md`).

---

## 2. The FC ↔ Pi link

**Transport.** MAVLink v2 over serial, codec pinned against pymavlink golden
frames. `SYSID_MYGCS` gating already implemented. *(code)*

**Downlink (FC → Pi).**

| msg | id | why | status |
|---|---|---|---|
| `ATTITUDE` | 30 | gravity-align depth, rotate the map | **enum only, no decoder** |
| `HEARTBEAT` | 0 | mode, armed, link health | present |
| `RC_CHANNELS` | 65 | assist switch, pilot intent | present |
| `SYS_STATUS` | 1 | battery → speed budget | present |
| `EKF_STATUS_REPORT` | 193 | "do I trust myself" | **absent** |
| `GPS_RAW_INT` | 24 | logged, deliberately routed nowhere | present |

`ATTITUDE` is the one v1 actually needs and it is ~30 lines. `EKF_STATUS_REPORT`
closes the self-trust gap: every safety mechanism assumes the *map* might be
wrong, none assumes the *system* might be, and the FC's own estimator health is a
free independent second opinion that should gate the speed budget.

**Uplink (Pi → FC): attitude, not velocity.** The consequence most likely to bite
on a field day, stated rather than discovered: **GUIDED velocity setpoints
require a horizontal velocity estimate, and GNSS-denied there isn't one.** No
GPS, no external nav, no optical flow leaves EKF3 with IMU and baro — enough for
attitude, not velocity. So `SET_POSITION_TARGET_LOCAL_NED` in a velocity mask
will not work for v1, and the planner's speed has to be expressed as pitch angle.
Crude, and correct for the constraint.

**The free win: `OBSTACLE_DISTANCE` (330).** ArduPilot's own proximity layer
consumes 72 distances by bearing — exactly what `BearingField::obstacleDistance()`
now emits, with a test pinning bin 0 to the nose and bin 36 astern. Publishing it
buys **a second, independent avoidance layer inside ArduPilot**, different code,
different failure modes, no dependence on our planner. For a project whose safety
argument rests on independent paths to a veto, that is close to free defence in
depth. **The producer exists in `nav-sim`; the publisher does not exist in
`onboard`.**

**Failure handover** mostly comes free: Pi stops sending → ArduPilot times out
offboard setpoints and reverts to the pilot. `FcLink` adds the matching property
on our side — if the fly loop stalls, the thread keeps RC alive but substitutes a
**neutral hover**, never repeating a stale *motion* command. *(code)* That is the
same freshness doctrine as the world model, and it is right.

**GNSS exclusion is checkable, not a promise:** `EK3_SRC*` logged per flight and
never GPS; `GPS_RAW_INT` received, written to the black box, routed nowhere else
— one grep proves it. Under architecture C the claim is stronger still, because
there is no position estimate anywhere for GNSS to contaminate.

---

## 3. The world model — and the fact that there are two

### 3.1 What `WorldState` is *(code)*

A flat blackboard struct, ~60 fields, in `onboard/include/world_model.hpp`.
Written by perception modules and the MAVLink bridge, read by the behaviour
arbiter, telemetry, and the planned LLM supervisor. `WorldModel` is a
mutex-guarded holder with `with()` and `snapshot()`.

**Its best design decision, and it should survive any rewrite:** perception
results are **latches**, so a crashed think thread leaves them set. Every result
therefore carries a `monoNowS` stamp, the fly loop stamps `tickMonoS` once per
tick, and consumers must use `corridorFresh(maxAge)` / `targetFresh` /
`roadFresh` rather than the bare valid flags. *A latched value from a dead thread
is indistinguishable from a live one without a clock.* This is the same doctrine
as `FcLink`'s neutral-hover substitution and as `unknown ≠ free`.

### 3.2 The problem: two world models that share no code

`NOTES.md` already records that `nav-sim/` and `onboard/` share **zero** code.
The consequence for the world model is sharper than "some duplication":

| concern | `onboard/` | `nav-sim/` |
|---|---|---|
| occupancy | `LocalMap` — 80 m, 0.5 m cells, **2-state** log-odds | `VoxelMap` — multi-res ladder, 0.25 m, **3-state**, unknown ≠ free |
| robot radius | `robotR = 1.5 m` | 0.35 m |
| angular field | `corridorScan[41]` across the FoV | `BearingField` 360×48, `obstacleDistance(72)` |
| planner | wavefront + gradient descent over a grid | primitive rollouts + `sphereClear` |
| authority | grid gives direction, live corridor gives speed/stop | near vetoes, far advises |

**These are not two implementations of one design; they are two designs.** Three
different angular resolutions (41 / 72 / 360), two robot radii differing by 4×,
and — most importantly — `LocalMap` is 2-state, so it has no way to express
"unknown", which is the single rule the `nav-sim` safety argument is built on.

### 3.3 Architecture A residue

`WorldState` still carries the retired architecture *(code)*:

```
estValid, estPe/estPn/estPu, estVe/estVn/estVu, estEphM,
estGpsDenied, estFeedingFc   // "injecting synthetic GPS into the FC"
```

`estFeedingFc` **is** architecture A, which `MAVLINK_BRIDGE_PLAN.md` §2 says to
retire as statistically wrong under ArduPilot. The plan's own item 2 is "retire
the Pi-side Kalman from the flight path — not delete; it is correct code for
architecture B, so keep it, mark it, leave it unwired." That has not happened.

### 3.4 What the world model should become

The honest v1 answer follows from architecture C: **the world model holds no
position.** It holds

* attitude from the FC (roll/pitch/yaw) — the only pose v1 needs
* the near map, in body frame, short-lived
* the far bearing profile, accumulated on attitude alone (a yaw rotation is an
  index shift, which is why the far field can carry memory without odometry and
  the near field cannot)
* freshness stamps on all of it
* the command actually sent

and nothing else. Position, velocity, and the ENU frame all leave the flight
path. That is not a simplification for its own sake — it removes the only place
GNSS could contaminate the solution, and it deletes a filter that would otherwise
fight EKF3.

---

## 4. Perception

**Near — `VoxelMap`** *(fresh, measured)*. Three-state log-odds on a
multi-resolution ladder with banded handovers. Carving and marking are separate
decisions with separate limits: carve is σ-bound at 11.17 m, marking honest to
`Z_max = √(cell·f·B/σ_d)·0.75 = 3.54 m`. An angular carve guard (min-pool
pyramid) stops a single ray erasing a thin obstacle it merely passed beside.
Measured cost 6.8–9.6 ms depending on stride; **the 0.10 m near rung is retired
because it cost 37 % of the frame and *reduced* near coverage** (76.3 % vs 100 %
at 2 m standoff).

**Far — `BearingField`** *(fresh, measured)*. 2.5 ms. Per-pixel body-frame bin
table so yaw is an exact index shift (15.75 → 3.2 ms). Four filters, each earning
its place from a specific observed failure: `minSamples` (one outlier is the
nearest sample in its bin by construction), `minFillFrac` (empty sky was becoming
a ceiling), neighbour consensus (no-return regions are coherent segments, not
random), and confirmation-by-existence — the last rewritten after measuring that
demanding *range* stability rejected 22.2 % of visible surface, because a bin
straddling a silhouette has no stable range. Existence confirms; range need not
hold still. Gaps fell 22.2 % → 5.6 %.

**Monocular depth — close field only** *(measured, mobile)*. Two structural
causes, both confirmed in the fresh code: MiDaS/DAv2 emit **inverse** depth so
resolution per metre falls as 1/Z²; and `desktop/tracker/depth_nav.py:92–102`
min-max normalises **per frame**, so any near object rescales the image and
crushes the far field. The second is a fixable bug worth fixing for the close
field it *is* good at. See `FAR_FIELD_MODELS.md`.

**Scheduling** *(code)*. `PerceptionScheduler` runs modules sequentially on one
Deliberator thread under a flat 60 ms/tick budget, hot modules first, with
per-module intervals. ROADMAP F11 notes the limitation: one thread cannot drive
more than one core, so the move-stop-sense cycle's idle phases waste the other
three.

---

## 5. Tracker

`desktop/tracker/` is the fresh implementation; `onboard/lock_tracker_fused.hpp`
is the C++ port. Architecture, in run order *(code)*:

1. **Ego-motion feed-forward** — median grid flow, box-excluded, forward-backward
   gated, added to the prediction so a camera pan doesn't push the target out of
   the crop before the filter catches up.
2. **Followed crop** sized from the box, so the target stays ~constant size from
   50 to 800 m and one template works across the range.
3. **Cue fusion** — luma / edge / chroma matched simultaneously, each weighted by
   its *own* PSR and by agreement with the prediction, so a cue that is useless
   right now contributes nothing rather than voting wrong.
4. **Anchor + adaptive + keyframe bank** — the fixed anchor re-anchors a drifted
   adaptive template; keyframes are a targeted fallback, not a primary.
5. **Staple-style histogram cue** — chroma fg/bg with no spatial layout, so it
   survives deformation and occlusion boundaries that break spatial NCC.
   **Biggest single win: sim occlusion lock 51 % → 95 %.** *(measured)*
6. **Occlusion-aware adaptation** — a PSR collapse against the running clean
   baseline freezes adaptation, banking and scale, hysteresis on both ends.
7. **Searching** — once coasting has clearly failed, zoom out and re-acquire with
   the **anchor alone**, because a drifted adaptive template driving a coarse
   scan is how you re-lock onto background.

**Staleness item:** the header states plainly that this supersedes
`LockOnTracker` in `lock_tracker.hpp`, "**the flight runtime still uses it**, so
both live here until the runtime is switched over." Two trackers, the runtime on
the older one.

Threading *(code)*: `desktop/tracker/pipeline.py` `SharedState` — capture writes
latest frame and drops old ones, inference reads latest and writes latest result,
display reads both. Frame-dropping rather than queueing is correct for control.

---

## 6. Decision register — the choices, and why

| decision | choice | why |
|---|---|---|
| flight stack | ArduPilot | EKF3, mature GNSS-denied support, `OBSTACLE_DISTANCE` |
| who estimates position (v1) | **nobody** | thesis needs no global position; avoids double filtering |
| uplink interface (v1) | `SET_ATTITUDE_TARGET` | velocity setpoints need a velocity estimate we don't have |
| near representation | 3-state voxels | need free space and volume to answer "is this tube clear" |
| far representation | bearing bins | stereo anisotropy makes far cubes wasteful and dishonest |
| unknown | **not free** | an unobserved direction is not a measured zero |
| far-field authority | none | may advise a goal, may never grant permission |
| memory | latch + stamp + explicit freshness | a latch from a dead thread looks live without a clock |
| stale command | neutral hover | never repeat a stale *motion* command |
| GNSS | received, logged, routed nowhere | makes the exclusion claim checkable by grep |
| coarse voxel rung | retired | bearings are 33× finer for less time |
| 0.10 m near rung | retired | −37 % frame time *and* better coverage |
| learned far field | parked | monocular depth measured close-field-only |

---

## 7. What is actually blocking, in order

1. **`ATTITUDE` decoder + golden-frame test** — ~30 lines. Nothing flies without
   it. Bridge plan item 1.
2. **`SET_ATTITUDE_TARGET` command path** with the pitch-angle speed mapping.
   Bridge plan item 3. Items 1 and 3 are *the* v1 flight blockers.
3. **Run the existing stack on an actual Pi 5.** An afternoon, and it reprices
   every plan in this repo. All timings are dev-box; a Cortex-A76 on
   memory-bound work is plausibly 1.5–2.5× slower.
4. **Reconcile the two world models** — decide whether `LocalMap` or `VoxelMap`
   is the map, and delete the other. Three angular resolutions and two robot
   radii is not a design, it is an accident of two codebases.
5. **Retire architecture A from `WorldState`** — mark the estimator fields
   unwired rather than deleting them; they are correct code for architecture B.
6. **`EKF_STATUS_REPORT` decoder**, gating the speed budget.
7. **`OBSTACLE_DISTANCE` publisher** — producer already exists in `nav-sim`.
8. **Non-GPS ArduPilot setup written down** — `EK3_SRC*`, pre-arm checks, what
   fails first. GUIDED will refuse to engage until it's done and the symptom
   looks like a broken link.

Items 4 and 5 are the architecture work. Items 1–3 are what stands between this
project and a first flight, and none of them is large.
