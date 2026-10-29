# Where this project sits, and what to expect from the literature

Researched September 2026. **Verification caveat, stated first because it bounds
everything below:** this session's egress proxy returned 403 at CONNECT for
arxiv.org, nature.com, science.org, PMC, rpg.ifi.uzh.ch, roboticsproceedings.org,
openreview, semanticscholar and sciencedirect. **No primary paper was read.**
Claims sourced to papers are search-engine extraction and should be re-checked
against the PDF before being quoted anywhere that matters. Claims sourced to
CONFIG FILES were fetched from raw.githubusercontent.com and verified directly;
those are marked [read].

---

## 1. The architecture is not unusual, and one project arrived at it independently

**ntnu-arl ORACLE** (https://github.com/ntnu-arl/ORACLE) builds a motion-primitive
library, rolls it out, and has a network choose among the candidates.

[read] `config.py`: `NUM_VEL_X = 1`, `NUM_VEL_Z = 8`, `NUM_YAW = 32`, so
`NUM_SEQUENCE_TO_EVALUATE = 256`. `ACTION_HORIZON = 14` at `DEPTH_TS = 1/15` s
(~0.93 s). `CMD_VELOCITY = 2.5`, `MAX_RANGE = 10.0`, steering spread over the
87-degree horizontal FOV.

**256 = 1 x 8 x 32 against kestrel's 210 = 13 x 5 x 3.** Same idea, same order of
magnitude, arrived at separately.

**The difference favours kestrel.** ORACLE's collision check IS a network -- a
Collision Prediction Network, made trustworthy with a deep ensemble for epistemic
uncertainty. kestrel's check is `sphereClear`, geometry, and the network only
ranks what geometry already admitted. ORACLE needed an ensemble to buy back a
guarantee kestrel gets by construction.

The formal name for kestrel's design is **preemptive shielding** (Alshiekh et al.,
AAAI 2018, https://arxiv.org/abs/1708.08611): the shield computes the set of safe
actions and the agent picks from that list, as opposed to *post-posed* shielding
where the agent picks and the shield corrects. In the run-time-assurance
vocabulary it is a **monitor plus backup controller**. Action masking is the
deep-RL name; Huang & Ontanon (https://arxiv.org/abs/2006.14171) show the masked
gradient IS a valid policy gradient, so MaskablePPO is the theoretically sound
choice here rather than a convenience.

## 2. Calibration: kestrel is in-family, coarse on one axis, slow on another

| | map res | integrate range | horizon | replan | speed |
|---|---|---|---|---|---|
| **kestrel** | 0.25 m | ~3.5 m | 2.0 s | 10 Hz | vMax 3.0 |
| Fast-Planner [read] | **0.1 m** | ray 4.5 m | 7.0 m | on demand | ~3 m/s |
| EGO-Planner [read] | **0.1 m** | ray 4.5 m | replan at 1.5 m | ~1 ms/plan | ~3 m/s |
| FASTER [read] | inflate 0.47 m | Ra 4.0 m | 6 pts | 100 Hz ctrl | v_max 5 |
| ORACLE [read] | depth direct | 10 m | 0.93 s | **15 Hz** | 2.5 / 3.5 |
| nvblox [read] | 0.05 m | 5 m | -- | depth **40 Hz** | -- |

- **0.25 m is 2.5x coarser than the 0.1 m consensus.** Defensible on compute, but
  it makes the inflated obstacle blockier. Worth an ablation at 0.15 m before
  concluding anything about the residual collision rate.
- **~3.5 m honest marking is normal.** Fast-Planner and EGO-Planner both cut rays
  at 4.5 m; nvblox integrates to 5 m. Nobody trusts a depth camera past ~5 m.
  kestrel is not the conservative outlier it might feel like.
- **10 Hz is on the slow end.** ORACLE and agile_autonomy run 15 Hz, nvblox
  integrates at 40. Above 3 m/s the replan rate binds before map resolution does.
- **3 m/s is the standard comparison speed**, so vMax = 3.0 is directly
  comparable to published numbers without apology.

## 3. THE BIGGEST FINDING IS IN OUR OWN CODE: there is no momentum

`TrajectoryPlanner`'s library is rolled out ONCE at construction, and every
rollout starts from `vx = vy = vz = 0` (voxel_traj.cpp). `primFirstStep` returns
`pts[0]`. So **every control step applies the first dt of a from-rest
trajectory** -- velocity never accumulates between steps.

This is deliberate and the reasoning is sound: the veto checked a from-rest
rollout, so flying a from-rest first step keeps the check consistent with the
motion. But the consequences are large:

    k = dt/tau = 0.1/0.35 = 0.286
    top reachable speed = 0.286 x 3.0 = 0.857 m/s
    displacement per step = 0.0857 m
    measured: freeM 0.0816, score 0.0808, CRUISE_M_PER_STEP hard-coded 0.09

**The aircraft flies at 29% of vMax permanently and cannot exceed it.**
`CRUISE_M_PER_STEP = 0.09` is documented as "a property of the vehicle rather
than of a map" -- the observation is right, the explanation is not. It is one lag
step from standstill.

Downstream, three things rest on it: `journey_fit`'s "steps needed" (so every
TOO FAR verdict, including the city-goal analysis), the 260-278 m saturation it
was measured from, and the conclusion that "speed is not the lever" (score 0.081
vs freeM 0.082 -- true, but they matched because both were pinned at the ceiling).

**And it is precisely the documented sim-to-real killer.** SimpleFlight
(https://arxiv.org/abs/2412.11764) identifies four parameters that must be
system-identified and randomised: mass, inertia, thrust coefficient, and **motor
time constant**. Multiple sources state that without modelling observation delay
and motor dynamics, RL control policies do not transfer to hardware at all. A
policy that has never experienced momentum has never needed a stopping distance,
and on real hardware the from-rest rollouts would no longer bound the true
motion -- which is exactly the assumption the veto's guarantee rests on.

## 4. The veto treats unknown as free -- and the obvious fix was already rejected

`sphereClear` rejects a cell only when `l > occThresh` (confirmed occupied). The
confirmed-free requirement is gated on `coreFrac`, which **defaults to 0**.
Unknown passes. Out-of-map passes.

This is the **optimistic** unknown-space policy. Oleynikova
(https://arxiv.org/abs/1710.00604) calls it "inherently unsafe"; the conservative
alternative "guarantees safety but limits moving speed since a stopping condition
has to be met in short-range free space". It is the mechanism by which 100% of
this project's collisions landed in unmapped space.

**But the obvious fix was measured here and failed.** voxel_traj.hpp carries the
sweep: coreFrac 0.65 was "never safer, usually slower", and requiring the whole
swept ball to be free deadlocks outright -- "with a forward camera the sides and
rear are permanently unknown, and this project has already spent 638 of 700 steps
stationary learning that". Only 0 and 0.65 were tested.

**The literature's answer is a different construction, and it is designed around
exactly that deadlock.** Three systems converge on it independently:

- **FASTER** (https://arxiv.org/abs/2001.04420, T-RO 2021): optimise in free-known
  AND unknown space, but only commit if a **backup trajectory terminating at rest
  in free-known space** is simultaneously valid.
- **SUPER** (Science Robotics, Jan 2025,
  https://www.science.org/doi/10.1126/scirobotics.ado6187): two trajectories per
  replan, one in known-free for safety and one through unknown for speed.
  Reported **35.9x lower failure rate while flying FASTER**, >20 m/s in real
  flight. [unverified -- headline figure from search extraction]
- **gatekeeper** (https://arxiv.org/abs/2211.14361, T-RO 2024): the same idea as a
  theorem -- commit only to trajectories verified to admit an infinite-horizon
  safe continuation, explicitly "despite sensing limitations".

The distinction that matters: coreFrac makes EVERY primitive conservative, which
deadlocks. The backup rule lets the committed primitive be aggressive provided a
*separate* braking option remains admissible. kestrel already has escape
manoeuvres in the library; this promotes them from a fallback to an invariant.

SUPER's claim of simultaneously safer and faster is the direct counter-argument
to "never safer, usually slower" -- if it holds, the coreFrac result does not
generalise to the backup construction.

## 5. What else to anticipate

**Perfect depth is flattering us, in a specific predictable way.** Loquercio et al.
(Science Robotics 2021, https://arxiv.org/abs/2110.05113) re-ran their benchmark
with stereo-matched depth instead of perfect depth: **Fast-Planner failed
completely at >=5 m/s** while their learned policy was "only minimally affected".
The stated mechanism -- a mapping-based planner needs multiple observations to
reject depth outliers and fully map an obstacle, so at speed it detects too late.
**kestrel's veto is a mapping-based component.** Expect the veto, not the policy,
to be what degrades on real depth.

**Their real-world failures were sensing-geometry failures, not policy failures.**
At 7 m/s, 8 of 10 real flights succeeded; both failures are attributed to the
same cause -- **objects entered the field of view too late because of the high
angular velocity of the platform**. For a system that vetoes against a map: an
aggressive primitive rotates the vehicle, rotation empties and refills the map,
and the veto is only as good as what has been observed.

**Abstraction transfers, and we are further along that axis than anyone.** Deep
Drone Acrobatics (https://arxiv.org/abs/2006.05768) gives the clean A/B: raw
images 80% success with 58% higher tracking error, versus 100% for the identical
policy on abstracted feature tracks. kestrel consumes per-primitive rollout
features from an occupancy map -- an abstraction of an abstraction. That is
strongly in our favour for policy transfer, and it relocates essentially all the
risk into the mapping and primitive-tracking layers.

**Latency is the cheapest thing we are not simulating.** FlightBench
(https://arxiv.org/abs/2406.05687) concludes with "the importance of latency
randomization for learning-based methods". A ROS stack alone costs ~45 ms. In our
terms: the map the veto checks is always stale by one sense-plus-compute
interval, and the primitive executes from a state the policy never saw.

**Mask dependence is real and now has a mechanism.** "Overcoming Valid Action
Suppression" (https://arxiv.org/abs/2603.09090): training with masks and
deploying without fails because the policy learns no mechanism to infer validity;
shared network parameters propagate suppression of an action to states where it
is valid, **exponentially in the shared-feature setting, worst for rarely-valid
actions**. Our mask is itself a function of map coverage, so primitives usually
masked in cluttered mazes are suppressed even in open space.

**The teacher-student trap is already formalised.** Student-Informed Teacher
Training (ICLR 2025, https://arxiv.org/abs/2412.09149) shows a teacher with full
state learns behaviours a partially-observing student physically cannot imitate.
If we ever train a teacher against the ground-truth voxel world and distil to a
student that sees only the mapped world, the teacher will dive through fog. Their
evaluation task is vision-based quadrotor obstacle avoidance, and notably
perception-aware flight emerged without explicit reward tuning.

## 6. What appears to be genuinely ours

- **The blind/mapped collision split.** No paper found reports collisions broken
  out as "into mapped-occupied" versus "into unmapped" space. The framing is
  everywhere; the measurement does not appear to be standard reporting.
- **Metres-before-collision as the headline metric.** Published work scores goal
  success rate, collision rate, lap time and tracking error. Our objective has no
  external baseline to quote, which is why `freeM` carries that load.
- **freeM's pathology has a name in the literature.** 29.5x loop ratio, 201 m
  flown to finish 6.8 m out, coverage growing sub-linearly while displacement
  falls -- that is Oleynikova's "stopping condition in short-range free space
  limits moving speed" in its purest form.

## 7. Ranked, what to actually do

1. **Fix or expose the momentum model.** It caps the vehicle at 29% of vMax,
   invalidates `CRUISE_M_PER_STEP` as a vehicle property, and is the single
   largest sim-to-real gap. Carrying velocity across steps would require the veto
   to check from the current velocity, which is a real change -- but the present
   state is a vehicle that cannot build speed.
2. **The backup-trajectory invariant** (FASTER / SUPER / gatekeeper). The cheapest
   available attack on blind collisions, needs no retraining, and is a different
   construction from the coreFrac experiment that failed here.
3. **Measure mask dependence.** Evaluate with the mask removed and report the
   collision rate. Cheap, and it says whether the learner internalised any
   avoidance or is purely a ranker over a pre-filtered list.
4. **Latency randomisation**, then depth range-dependent error and edge dropout.
5. **Try coreFrac between 0 and 0.65** -- the sweep here tested only the endpoints
   and the comment says so.
