# NOTES — working memory, not a document

Terse running notes: decisions taken, why, and what is still unknown. Not for
showing anyone. Append, date entries, delete what goes stale.

---

## 2026-08-06 — staging decision + feasibility verdict

### Where we are

Ordering stereo cameras. Integrating the rest onto the frame. Validating the
Pi↔FC connection. No autonomy work in flight until that is solid.

### Agreed staging

1. Airframe + Pi integration, connection validated
2. Lock-on (tracker from a hover)
3. Basic flight modes + autonomy
4. Stereo autonomy — **last**

Rationale: every stage flies. Each one builds the telemetry / failsafe /
mode-switch / black-box infrastructure that stage 4 needs anyway, but where a
bug is an annoyance rather than a tree. FGI's published work is all-or-nothing
60 m autonomous runs; that is the wrong shape for a one-person team.

### Feasibility verdict (this is what the paper review was for)

Risk is **not uniform** across the four stages.

* Stages 1–3: **not research.** Hover tracking is a solved CPU problem and
  `lock_tracker.*` already exists. iNAV + companion RC override is well-trodden.
  Risk is integration time and failsafe discipline, not feasibility.
* Stage 4: the only real question — and **not binary**.

Key evidence, Karjalainen 2501.12073, Evo-medium:

> 650 trees/ha, **7/7 successful**, 1 m/s, RealSense D435 with the IR emitter
> **disabled** (passive), EGO-Planner-v2, 1153 g.

Same stack at 2000 trees/ha: 8/9 successful but **0/9 smooth** — every flight
needed an emergency stop, all caused by late detection of thin dry spruce
branches. DeFoP with an Orin NX + active stereo is still only 12/15 in the hard
stand.

**So the expected outcome of stage 4 is a density and a speed at which it
works, not success/failure.** "0.6 m/s in medium boreal forest, degrades in
dense" is a real capability. Plan for that, not for a binary.

Nothing we are attempting is impossible. What is unproven is the *budget*
variant, not the task.

### The two load-bearing unknowns

Everything else is arithmetic already done (~16 ms planner + <10 ms VO + depth
inside a 100 ms budget).

1. **Passive stereo on bark.** DeFoP projects IR. Karjalainen disabled it and
   their honest depth was ~3 m with late branch detection as the failure mode.
   Nobody in this literature demonstrated passive stereo carrying the job. Our
   silhouette model (`depth_camera.hpp`: `edgeBoost 0.75`, `edgeDepthM 0.8`)
   says it can — calibrated on **one screenshot**.
2. **State estimation on a Pi 5 CPU.** VINS-Fusion will not fit alongside
   everything else. SMF-VO (2511.09072) claims <10 ms/frame on exactly our
   hardware; **unread** — arXiv is 403 through this proxy. Slot already exists:
   `StateEstimator::updateVisionVelocity()`, sized `sigmaVoVel = 0.25` m/s.

### Highest-value experiment available — do it while parts ship

**Stereo camera on a tripod, in a forest. No drone, no flight, no integration.**

Point at spruce trunks at 2 / 5 / 8 / 12 m, in canopy shade and against bright
sky. Record raw stereo + the depth output. Look at *where depth returns and
where it is black*.

* If trunks come back as black columns with one lit edge → our silhouette model
  is right and the stack has a chance.
* If trunks vanish entirely by 6 m → we learn it now, before three more layers
  are built on top of it.

This answers unknown #1 for the cost of an afternoon and does not compete with
the frame work. Also gives the first real-footage input for the depth model,
which is currently fitted to a single screenshot.

Secondary while there: check whether the emitter helps or hurts. The two FGI
papers **contradict each other** on this (DeFoP on, Karjalainen off because the
projected dots move with the camera and corrupt VIO). If we ever want active
stereo we cannot use one imager for both depth and VO without resolving it.

---

## Lean-out — "new kestrel OS"

Not a rewrite. The spine already exists in `onboard/`: two-tier threading,
`WorldModel` blackboard, mode arbiter, FC link, black box, realtime, SITL
suite. The lean-out is **making `nav-sim/`'s proven pieces into modules that
plug into that spine, and deleting the rest.**

Note: `nav-sim/` is not even listed in AGENTS.md §0. That table is stale.

### Carry forward — validated, load-bearing

* Three-state occupancy, `unknown ≠ free`. Measured: false-free 7.814 % → 0.003 %
  after the carve guard.
* `Z_max = √(cell·f·B/σ_d)` with the ~25 % derate, and `maxIntegM`. Carving is a
  separate decision from marking occupancy.
* Body-frame precomputed trajectory library (`voxel_traj.*`). Independently
  arrived at by DeFoP (256 primitives, same construction).
* Stopping-distance speed budget against **confirmed-free** length only.
* Exhaustive swept-volume clearance test (the sampled version failed 3×).
* Escape primitives — only defensible *because* there is a map behind us.
* The measurement discipline itself: seeded runs, ablation tables, ideas
  disabled with their numbers attached.
* `onboard/`: MSP backend, scheduler, black box, realtime, Kalman + VO hook,
  nav_map, SITL.

### Delete — keep the numbers, drop the code

* `voxel_planner.*` histogram planner — superseded by the library.
* OMPL dependency — heavy; #20's answer is topological memory, not RRT.
* `fieldEma`, `switchMargin`, `revPenalty`, `coreFrac` — all measured
  neutral-or-harmful. Tables move to a doc, code goes.

### Reframe carried over from the review

The map's job is **memory, not foresight**. Not "see 25 m ahead" — nobody
flying successfully has that (their honest depth is 2–3 m). It is "remember
where you have been so you do not re-enter a pocket you already failed out of."
That turns #20 from a perception-range problem into cheap topological memory,
and it attacks the one bug **both** published systems admit they still have
(DeFoP: "stop and rotate in place"; escapes are emergent, not planned).

---

## Do not

* More planner tuning. Two field-proven systems converged on what we have.
* More sim polish before real footage exists. The depth model is fitted to one
  screenshot; sim numbers cannot get more honest than that input.
* Chase the 25 m horizon.
* Adopt the learned half of DeFoP (CPN / autoencoder / ensemble+UT). Needs a
  GPU, a training pipeline, and a simulator good enough to train in — and their
  own geometric supervisor exists *because* the learned part can't be trusted.
  We already have the supervisor; the learned part is the expensive half.

## Known sim↔real gaps — real, deliberately deferred

Ranked by how much each would move our numbers.

1. **Thin branches do not exist in `voxel_world.cpp`.** Stems, scrub, canopy —
   no twigs. Both papers name late thin-branch detection as *the* cause of real
   failure. We are not simulating the failure that ends real flights.
2. No state estimation in the loop (default runs have perfect pose). Real
   number: ATE 0.33–0.50 m over 34–42 m, systematic scale shrinkage.
3. Plant circularity — `tau = 0.35` both predicts and produces motion. See
   `nav-sim/docs/CONTROL_PLAN.md`. Bounded, but "flyable by construction" is an
   assumption not a result.
4. Idealised timing: fixed-dt lockstep, zero sense→act latency.
5. Depth model fitted to one screenshot; no auto-exposure, backlight, canopy
   gloom.
6. We score any contact as a collision; they allow "minor touches to thin
   vegetation." Keep ours — but the comparison is not like-for-like.

## Open / unresolved

* SMF-VO unread. Re-check when arXiv is reachable; if it holds, it becomes the
  state-estimation plan.
* `nav-sim/` and `onboard/` share **zero** code (`grep VoxelMap onboard/` → nothing).
  The stack we measure is not the stack that would fly. Resolve during the lean-out.
* Stereo camera choice not yet locked; baseline drives `Z_max` directly
  (12 cm → ~8 m measured; D435i's 5 cm → ~2–3 m).
