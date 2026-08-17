# Independent arrivals — constraint-derived solutions that already have names

Each entry is the same shape: a constraint was reasoned about from the platform
up, a solution followed, and the solution turned out to be an established method
with a literature behind it.

**Why this is worth writing down.** Not for credit — for the fourth column.
Arriving independently at a named method means you can stop deriving and start
reading: the named version has thirty years of known failure modes, published
fixes, and parameter guidance already attached. Every row below is a place where
the project can inherit work instead of repeating it.

It is also the strongest available evidence that the constraints were read
correctly. When a chain of reasoning from *"the compute budget is too small"*
lands on the same design a Mars rover uses for the same reason, the reasoning was
probably sound.

**Citation confidence.** Marked ●● where I am confident of author and year, ● where
the method and attribution are right but the exact reference should be checked
before it goes in a thesis.

---

## 1. Occupancy grids with three states, and "unknown is not free"

**The constraint.** A map that stores only *obstacle / not-obstacle* has nowhere
to put "I have not looked there". Treating unobserved space as free is the
failure that kills, because the aircraft's whole safety argument rests on knowing
where its knowledge ends — and the sensor's cone leaves most of the world
unobserved at any instant.

**What was built.** Three-state log-odds occupancy — FREE / OCCUPIED / UNKNOWN —
with carving and marking as *separate* decisions with separate range limits, and
a hard rule that unknown is never treated as free.

**What it is called.** **Occupancy grid mapping**, and the three-state
probabilistic formulation is the whole point of it. Moravec & Elfes, *High
Resolution Maps from Wide Angle Sonar* (1985) ●●; Elfes, *Using Occupancy Grids
for Mobile Robot Perception and Navigation* (1989) ●●. The log-odds update with
an **inverse sensor model** is the standard treatment — Thrun, Burgard & Fox,
*Probabilistic Robotics*, ch. 9 ●●.

**What the named version knows that we do not.** The independence assumption
between cells is known to be wrong and known *how* it is wrong — it over-counts
evidence along a ray, which is exactly the over-confidence the carve guard was
invented to patch. There is also a standard **clamping** rule (bound the log-odds
so a cell can always be revised) that keeps a map updatable in dynamic scenes;
worth checking against the current thresholds.

---

## 2. Stopping to think when the compute budget is low

**The constraint.** Planning cannot keep up with flying. If the plan is stale by
the time it is executed, speed is bounded by *perception latency*, not by the
airframe — and on a CPU-only Pi the latency is the binding constraint.

**What was built.** `MissionController`'s move-stop-sense cycle —
`SETTLE → THINK → MOVE → ARRIVE`, plus `SCAN` when cornered. Its own header says
it: *"THINK WHILE STILL, then move a committed leg, then stop and think again."*

**What it is called.** Several names converge here, and all three are worth
having:

* **Stop-and-go / stop-and-stare autonomous navigation** — the operational
  pattern of the Mars Exploration Rovers and MSL AutoNav, adopted for precisely
  this reason: a ~20 MHz radiation-hardened processor could not process stereo
  while driving, so the rover drives a short committed leg, stops, images, builds
  a map, plans, repeats ●●. This is the closest match to what was derived, and it
  is the flagship example of the constraint driving the architecture.
* **Anytime algorithms** — Dean & Boddy (1988) ●●, developed by Zilberstein ●.
  The formal treatment of "trade computation time for solution quality".
* **Perception-latency-bounded speed** — Falanga, Kim & Scaramuzza, *How Fast Is
  Too Fast? The Role of Perception Latency in High-Speed Sense and Avoid* (2019)
  ●. This one derives the actual speed limit from latency and sensor range, which
  is the see-ahead/plan-ahead ratio already used informally in this project,
  written down properly.

**What the named version knows that we do not.** The anytime framing says you do
**not** have to fully stop. An anytime planner returns a valid-but-worse plan
whenever it is interrupted, so the aircraft can degrade plan *quality* with speed
rather than switching between moving and thinking. That is a strictly larger
design space than the current binary, and it is the natural upgrade path for
`MissionController`. Falanga et al. give the closed-form speed bound to check the
current 1 m/s target against.

---

## 3. The bearing field is a polar obstacle histogram

**The constraint.** Stereo anisotropy grows as `Z·σ/B` — 100:1 at 20 m — so a
cube in the far field must be sized for its worst axis and throws away the
lateral detail the sensor still has. The far field measures *bearing* well and
range badly, so it should be stored by bearing.

**What was built.** `BearingField` — 360 × 48 bins at 1°, nearest confident
return per bearing, accumulated on attitude alone.

**What it is called.** **VFH — the Vector Field Histogram**, Borenstein & Koren
(1991) ●●: a polar obstacle histogram driving reactive steering. Refined as
**VFH+** (Ulrich & Borenstein, 1998) ●● and **VFH\*** (2000) ●. The
`POSE_AND_OPENNESS_PLAN.md` §1 already spotted the ancestry — *"the same idea,
from the 90s"*.

**And the repo contains an independent second arrival.** `DepthNav` in `onboard/`
implements **VFH+ by name** — polar openness histogram, free/blocked hysteresis,
sector widening by vehicle half-width, previous-heading cost. Two halves of this
project independently reached the same 1970s-to-90s structure from two different
starting points. That is a strong signal the representation is forced by the
problem rather than chosen.

**What the named version knows that we do not.** VFH+ already solved the argmax
flapping problem that `NOTES.md` logs as one of three causes of "the aircraft is
spinning" — via threshold hysteresis plus a cost term on the previous heading.
`GeneralPlanner` grew its own hysteresis separately. **Do not build a third one.**

---

## 4. `sphereClear` is configuration-space inflation

**The constraint.** A planner that treats the aircraft as a point will fly the
airframe into things the centreline misses.

**What was built.** `sphereClear` — reject a primitive if any OCCUPIED cell lies
within the robot ball, with UNKNOWN tolerated only inside `coreFrac·robotR`.

**What it is called.** **Configuration space** and obstacle inflation by the
Minkowski sum of the robot's shape — Lozano-Pérez, *Spatial Planning: A
Configuration Space Approach* (1983) ●●.

**What the named version knows that we do not.** Inflate the *map* once with a
**distance transform** rather than testing a ball per sample. That converts an
O(cells-in-ball) test per query into an O(1) lookup, and the distance field is
reusable — it also gives a graded clearance cost for free, which the scorer
currently approximates.

---

## 5. Primitive rollouts scored and vetoed

**The constraint.** No time for a full search, and the aircraft cannot follow an
arbitrary path anyway — it flies smooth arcs.

**What was built.** `TrajectoryPlanner` — a fixed library of ~210 dynamically
feasible primitives, rolled forward, scored, and hard-vetoed by `sphereClear`.

**What it is called.** The **Dynamic Window Approach** (Fox, Burgard & Thrun,
1997) ●● in its velocity-space form, and **state lattice / motion primitive**
planning (Pivtoraiko & Kelly) ● in its sampled form. The veto-then-score
structure is standard.

**What the named version knows that we do not.** DWA derives the admissible set
from the *dynamics* — accelerations and braking distance — rather than from a
fixed library, which guarantees every candidate is stoppable within the known
free space. That is a safety property the current fixed library does not
explicitly carry.

---

## 6. The multi-resolution ladder is an octree

**The constraint.** One cell size cannot serve both a 0.35 m robot radius near
and a 25 m horizon far.

**What was built.** A multi-resolution voxel ladder with banded handovers and a
borrow rule.

**What it is called.** **OctoMap** — Hornung, Wurm, Bennewitz, Stachniss &
Burgard (2013) ●●: multi-resolution probabilistic occupancy mapping on an octree.

**What the named version knows that we do not.** The octree gives the ladder
*and* compression from one structure, with a documented inner-node update rule
for querying at any resolution — which is exactly the banded-handover logic that
had to be written and debugged by hand here (the borrow rule, the render slack
that was shipped and reverted).

---

## 7. Local SLAM without loop closure

**The constraint.** An aircraft going forward through a wood does not revisit, and
the map is never read past 3.5 m. Global consistency buys nothing; drift over a
few seconds is the only error that matters.

**What was built** *(planned — `POSE_AND_OPENNESS_PLAN.md` §5)*. 3-DOF
correlative frame-to-map matching, explicitly no loop closure, no pose graph, no
global map.

**What it is called.** **Cartographer's local SLAM** — Hess, Kohler, Rapp &
Andor (2016) ●● — and **correlative scan matching**, Olson (2009) ●●. The plan
already names Cartographer, which is the pattern this whole document is about.

**What the named version knows that we do not.** Cartographer's local half brings
a worked submap lifecycle and the branch-and-bound trick that makes wide
reacquisition searches affordable — directly relevant to the "reacquisition after
dropout needs 10–20 ms and its own code path" note.

---

## 8. Refusing to claim a pose in an unobservable direction

**The constraint.** An avenue of parallel trunks constrains lateral position and
yaw but slides freely along-track. Reporting a confident estimate in the free
direction is worse than reporting nothing, because downstream cannot tell them
apart.

**What was built** *(planned)*. Detect degeneracy from the information matrix
eigenvalues and refuse to claim the unobservable direction — *"an unobserved
direction is not a measured zero"*, explicitly the same doctrine as unknown ≠ free.

**What it is called.** **Degeneracy detection in optimisation-based state
estimation** — Zhang, Kaess & Singh (2016) ●, which introduced solution
remapping: project the update onto the well-conditioned subspace and leave the
degenerate directions untouched.

**What the named version knows that we do not.** It gives a *threshold* on the
eigenvalues and a concrete remapping operator, rather than a binary refuse. That
is the difference between stopping and continuing on partial information.

---

## 9. Confirmation before belief

**The constraint.** One frame of evidence is one outlier away from a phantom
obstacle, and a bearing bin's nearest sample is *by construction* the most
outlier-sensitive statistic available.

**What was built.** `confirmFrames` — a bin must produce an accepted measurement
on consecutive frames before it is reported; plus `minSamples` as a floor.

**What it is called.** **Track initiation / M-of-N confirmation logic** from
radar target tracking — standard in Bar-Shalom's treatment ●. The same idea as
the log-odds threshold in occupancy mapping: require accumulated evidence before
declaring.

**What the named version knows that we do not.** M-of-N (e.g. 3 hits in 5 frames)
rather than N-consecutive is the standard form, and it is strictly more robust to
the intermittent dropout this sensor produces. Notably, **the fix that took 22.2 %
→ 5.6 % gaps was moving from range-stability to existence** — which is a step
toward M-of-N, arrived at empirically.

---

## 10. Neutral-on-stale, and freshness stamps

**The constraint.** A latched value from a dead thread is indistinguishable from
a live one. A stalled producer looks exactly like a steady one.

**What was built.** `FcLink` substitutes a **neutral hover** if the fly loop stops
updating the command — never repeating a stale *motion* command. `WorldState`
stamps every perception result and forces consumers through `corridorFresh(maxAge)`.

**What it is called.** A **watchdog** / **deadman switch** on the control side ●●,
and **time-to-live / staleness checking** on the data side — the same discipline
ROS enforces with message timestamps and TF timeouts ●●.

**What the named version knows that we do not.** Little, honestly — this one is
already done properly, and the *same* doctrine appearing independently at three
layers (unknown ≠ free, neutral-on-stale, freshness gates) is the sign of a
principle rather than a patch. Worth stating as such in the thesis.

---

## Summary

| # | constraint | what it is called | inherit |
|---|---|---|---|
| 1 | binary maps cannot say "unlooked-at" | occupancy grids, log-odds (Moravec & Elfes 1985) | clamping; the known independence flaw |
| 2 | planning cannot keep up with flying | stop-and-go (Mars AutoNav); anytime algorithms (Dean & Boddy 1988) | degrade plan quality instead of stopping |
| 3 | far field measures bearing, not range | VFH / VFH+ (Borenstein & Koren 1991) | its hysteresis already fixes the spin |
| 4 | a point-robot planner flies the airframe into things | configuration space (Lozano-Pérez 1983) | distance transform → O(1) clearance |
| 5 | no time to search; arcs only | Dynamic Window Approach (Fox et al. 1997) | admissible set from dynamics = stoppable |
| 6 | one cell size cannot serve near and far | OctoMap (Hornung et al. 2013) | ladder + compression from one structure |
| 7 | forward flight never revisits | Cartographer local SLAM (Hess et al. 2016) | submap lifecycle; branch-and-bound search |
| 8 | parallel trunks slide along-track | degeneracy detection (Zhang et al. 2016) | solution remapping, not binary refusal |
| 9 | the nearest sample is the most outlier-prone | M-of-N track initiation (radar tracking) | M-of-N beats N-consecutive |
| 10 | a dead thread's latch looks alive | watchdog; TTL / staleness gating | already correct — name it and cite it |

**The pattern.** Ten constraints, ten established methods. In three cases
(**2, 3, 7**) the constraint reasoning reached a design that a flagship system
uses for *the same stated reason* — which is the useful kind of confirmation,
because it means the constraint was the real driver and not a rationalisation.

**The immediate wins** are rows 3 and 4: VFH+'s hysteresis is already implemented
elsewhere in this repo and should not be written a third time, and the distance
transform is a straight speed-up of the hottest safety check in the planner.
