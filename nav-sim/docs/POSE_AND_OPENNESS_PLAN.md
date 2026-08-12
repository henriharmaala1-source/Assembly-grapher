# Pose, bounded memory, and bearing-space openness — PLAN

Status: **planned, nothing implemented.** Written from a design conversation
rather than from code, so every number below is either measured elsewhere in this
tree (marked) or an estimate (marked). Companion to `DEPTH_SUPERVISOR_PLAN.md`
and `APPEARANCE_AND_BLOBS_PLAN.md`.

---

## 0. The one claim everything here follows from

**The near field and the far field need different amounts of pose, because they
contain different information.**

| | near (≤ ~3.5 m) | far (≳ 10 m) |
|---|---|---|
| what the sensor measures well | range **and** bearing | bearing only |
| bearing precision | 0.128°/px (1/447 rad) | 0.128°/px — same |
| range precision, `σ = 0.0112·Z²` | 0.14 m at 3.5 m | 1.1 m at 10 m, 4.5 m at 20 m, 7 m at 25 m |
| right representation | 3D voxels | 2D angular bins |
| pose needed to accumulate | full 6-DOF | **attitude only** |
| what it may authorise | flight | nothing — awareness only |

Translating shifts far structure by `t/D` radians and near structure by
everything. That is why the far layer can carry a memory on attitude alone,
which is the pose information we can get cheaply and nearly drift-free, and the
near layer cannot.

Everything below is a consequence of that table.

## 1. Angular far-field openness map  — *cheapest, do first*

Replace the coarse voxel layer **and** the ray march that reads it with a 2D
histogram indexed by (azimuth, elevation).

`TrajParams::farWeight` currently makes `voxel_traj.cpp` march ~20 samples
through `coarse` levels per primitive, 210 primitives per frame, to produce **one
scalar per bearing**. The 3D structure is scaffolding for a 2D answer.

| | today | proposed |
|---|---|---|
| storage | `Mfar` 128×128×42 = 688 k cells, 2.75 MB | 2° bins, 180×45 = 8.1 k, **32 kB** |
| update | DDA raycast per ray | one precomputed LUT lookup per pixel, no traversal |
| read, per primitive | ~20 `stateAt` steps | **one array read** |
| accumulate across frames | needs full pose | **attitude only — a rotation is an index shift** |

Store per bin the nearest confident return (a depth panorama) or the occupancy
fraction, matching whichever `FarMode` wins.

**Parallax sets the memory length.** A background at `D` shifts by `t/D`; to stay
inside one bin `b`, `t < b·D`. At 5° bins and 20 m that is 1.75 m ≈ **1.2 s** at
1.5 m/s. Consistent with §3's bracket, which is reassuring rather than a
coincidence.

Ancestors worth reading: spherical range image; **VFH** (Borenstein & Koren), a
polar obstacle histogram driving reactive steering — the same idea, from the 90s.

## 2. The far field should supply the GOAL, not a score term

Structurally the more important half, and it is not about 2D at all.

Today far openness is a weighted term inside the per-primitive score, competing
with near clearance and smoothness in one sum, in different units, balanced by a
tuned weight. Instead: **the far field picks the bearing, the near field decides
what is flyable.**

This drops into an input that already exists. `TrajectoryPlanner::plan()` takes
`goalAzDeg, goalElDeg`, and `voxel_live` passes `0` or the current yaw because on
a bench there is no goal. The openness argmax is the missing *source* of a local
goal. So the change **removes** a weight rather than adding one: `farWeight → 0`.

Principle: *represent at the resolution of the decision, not of the sensor.* The
decision is one bearing; the far information is bearing-only; the vehicle can act
on a bearing only over the next few seconds. Nothing in that chain is 3D.

Reuse `GeneralPlanner`'s existing hysteresis (`lastAz_`, `haveLast_`, 96×9 = 864
bins). Argmax over a noisy field flips, and "the aircraft is spinning" is already
logged in `NOTES.md` with argmax churn as one of three causes. Do not grow a
second hysteresis mechanism.

**Two honest limits.** Openness is not traversability — a bright gap can be sky
above a wall — but since it only sets the goal and the near field keeps its veto,
being wrong costs efficiency, not safety. And it is greedy and memoryless, so it
will enter cul-de-sacs; it is a good local goal, not a route.

Passive-RGB sky brightness (`APPEARANCE_AND_BLOBS_PLAN.md` §3.0) lands in these
same bins as a second, independent estimate of the same quantity.

**Test:** A/B against the current `farWeight` arrangement, 8 seeds, lanes world,
primary metric goal-closing efficiency.

## 3. Bounded memory — decay the near map

The near map currently has unlimited memory and assumes zero drift, which is the
worst corner of that trade for a moving aircraft with no odometry.

**The window is bracketed, not chosen.**

* **Lower bound ≳ 2–3 s.** At 60 °/s with an 87° FOV, something at the frame edge
  is unseen for ~1.5 s across a turn; the trajectory horizon is 2 s. Below this
  you cannot plan through a turn, which is the whole reason a map exists rather
  than a bare depth image.
* **Upper bound: drift over the window < one cell.** At 1.5 m/s and 1–2 %
  scan-matching drift, error is 0.02–0.045·T m, so `T < 5–12 s`. **At 5 % drift
  the bound falls to ~3.3 s and the bracket nearly closes** — which is exactly
  why §5's measurement is load-bearing rather than ritual.

**Decay per metre travelled, not per second.** Drift is a fraction of distance,
so forgetting should be too: hovering, you barely drift and should not forget.

**The strongest argument is blast radius, not the memory/drift trade.** Without
decay one bad pose poisons the map permanently; with it, the error fades on its
own. If the matcher loses lock, stale evidence decays to unknown, unknown is not
free, and the aircraft slows to a stop. That is a safe degradation and it is the
same dividend the unknown ≠ free rule already pays.

**Cost caveat:** 240×240×96 = 5.5 M cells is 22 MB of memory traffic per frame,
so a naive per-frame multiply is ~2–5 ms (estimate) and worse on a Pi. Amortise
(every 8th frame at the equivalent factor) or decay lazily on read. See §4.

## 4. Tile index — makes §3 affordable, useful on its own

The camera's cone is roughly 1.4 sr out to ~12 m ≈ 800 m³, against a
60×60×24 m = 86,400 m³ box. **About 1 % of the map is ever observed**, and every
full-grid pass walks the other 99 %.

An 8³ tile index is 30×30×12 = **10,800 tiles, 10.8 kB**, marked dirty when a ray
writes into them. Cuts the decay pass by one to two orders of magnitude, speeds
up `recentre`, and gives the raycaster an early-out through empty regions.

This is also the local half of submapping. It is *not* a route to a global map —
compression fixes memory, and memory was never the binding constraint. Drift is.

## 5. Scan-matching odometry — local SLAM, no loop closure

Align the new **frame** to the existing **map** (not map to map, which is
circular — building the second map already needed a pose).

* **3 DOF, not 6.** Roll and pitch come from the IMU drift-free via gravity;
  altitude from baro or a downward rangefinder; leave x, y, yaw to matching.
* **Correlative, not ICP.** Reduce to a horizontal slice (trunks are vertical;
  `sliceImage` already builds one), precompute the rotated point set once per
  candidate yaw so the x,y search is an index offset with no arithmetic.
* **The search window is set by the frame interval**, not the map: 5 cm and 2°
  between frames at 1.5 m/s and 30 fps. So ±0.15 m, ±3°.
* **Cost (estimate):** ~640 candidates × ~300 points ≈ 0.8 ms; ~2 ms coarse-to-
  fine over the existing ladder. Reacquisition after a dropout needs a wide
  search at 10–20 ms and must be a separate code path with its own budget.
  6-DOF ICP on the full cloud is 10–20 ms and far more fragile — avoid.
* **Degeneracy is the real risk for us.** An avenue of parallel trunks constrains
  lateral and yaw but slides freely along-track; a flat wall constrains only its
  normal; and our 3.5 m honest range means matching on a thin shell rather than a
  rich 30 m scan. Detect it from the information matrix eigenvalues and **refuse
  to claim a pose in an unobservable direction** — an unobserved direction is not
  a measured zero, the same doctrine as unknown ≠ free.
* Frame-to-map matching against a map built from your own drifting poses is
  self-confirming. Keyframing is what interrupts that.

**Explicitly not building:** loop closure, place recognition, pose graph, global
map. An aircraft going forwards through a wood does not revisit, and the map is
never read past 3.5 m. This is Cartographer's *local SLAM* and not its global
half, on purpose.

## 6. IMU attitude

Scaffolding is further along than expected: `CamPose::rollDeg` exists and
`DepthCamera::rayFor` **already implements full R·P·Y**; `PoseHint` already has
`attitudeOnly`; `voxel_live` already does `if (hint.valid) pose = hint.pose;`.
The runtime loader has all six symbols needed to pull motion frames out of a
frameset and needs **two more** — `rs2_get_frame_timestamp`, `rs2_get_extrinsics`.

Estimate ~2.5 days: loader 2 h, motion frames 2 h, attitude filter 4 h, gyro→
depth extrinsic 1 h + test, `PoseHint` 1 h, `.kdr` IMU sidecar 3 h, tests 4 h,
roll in the FPV *renderer* 3 h (the mapper already handles it).

**Limits.** Roll and pitch are gravity-referenced and drift-free. Yaw has no
absolute reference — no magnetometer — so it drifts; budget 2° (0.14 m at 4 m,
under one cell), which at a realistic 0.05–0.2 °/s buys **20–40 s**. Show an
elapsed-since-reset timer; do not claim the map simply rotates.

**Do not use this IMU for translation.** The killer is not accel bias but
attitude coupling: 1° of tilt error leaks 9.81·sin 1° = **0.17 m/s²** into
horizontal acceleration, which double-integrates to 0.86 m in 3 s.

**In flight, take attitude from the flight controller instead.** `MSG_ATTITUDE`
(30) is already in the MAVLink enum with no decoder; adding one to a codec that
already carries 14 messages and the CRC_EXTRA table is ~30 lines plus a pymavlink
golden-frame test. ArduPilot's EKF handles the case a complementary filter on the
camera IMU cannot — that under thrust, "down" is not gravity.

## 7. Order, and the two measurements that decide everything

1. **Angular openness map + goal-not-score (§1, §2).** Cheapest, removes a tuning
   weight, needs no new hardware or pose, and three separately-motivated ideas
   want the same 32 kB structure (this, the supervisor's sector grid, and the
   RGB sky term).
2. **Tile index (§4).** A day, useful immediately, and §3 depends on it.
3. **MEASUREMENT A — sweep decay length with the TRUE pose in sim.** No matcher.
   Isolates "how much memory does the planner need" and gives the lower bound
   alone.
4. **MEASUREMENT B — build the matcher, measure drift per metre against truth in
   sim.** Gives the upper bound. `voxel_sim` has perfect ground truth, so both
   measurements need no camera, no IMU and no flight.
5. If the bracket survives both, commit to the architecture. If it does not, the
   answer is a shorter memory, not more estimation.

**Before any of it:** run the existing stack on an actual Pi 5. Every timing in
this project is dev-box — 22 ms integrate, 1 ms plan, 2 ms estimated matcher —
and a Cortex-A76 on memory-bound work is plausibly 2–3× slower, which would put
mapping alone at 50–60 ms inside a 100 ms cycle before anything here is added.
That is an afternoon and it reprices this entire document.
