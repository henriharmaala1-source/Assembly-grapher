# What this project claims, and what would prove it

This is the direction document. `NOTES.md` is the lab notebook — what was tried,
measured and rejected. This is the shorter, slower-changing question of what all
of it is *for*, and what would count as done.

Written 2026-08-12, after a design review that put the whole architecture on
trial. It should be edited when the thesis changes, not when the code does.

---

## 1. The thesis

**Autonomous obstacle-avoiding flight for a cheap FPV airframe, on cheap
CPU-only compute.**

A Raspberry Pi 5. A used D435i. A 7" 6S quad that already exists. No GPU, no
lidar, no Jetson, no cloud, no training cluster.

**The constraint is the claim, not an accident of budget.** This problem is
solved at the €2000 tier — Skydio ships it, and the research quadrotors that fly
forests at 10 m/s carry a Jetson and a learned policy. Reproducing that
demonstrates you had €2000. Doing it inside a hobby budget demonstrates
engineering. That is the whole point of the exercise, and it is why "just use a
Jetson" is out of scope rather than overlooked.

### 1.1 The second claim, which is stronger and less often stated

The method. Every number in this project is measured or explicitly marked as an
assumption. Negative and inconclusive results are kept with their numbers
attached rather than quietly deleted — the depth improver and the sideslip
coupling are both in the tree, both off by default, both with the measurement
that says why. Plans exist in full for components deliberately *not* built.
Instruments are checked before the experiments that use them.

Cheap CPU-only avoidance projects exist and someone will always find a cheaper
board. That discipline, sustained, is rarer — in hobby robotics and in published
work — and it is the more defensible differentiator of the two.

### 1.2 Motivation, stated plainly

Career and credibility. A working, honest demonstration of autonomy on this
budget is a portfolio piece and a plausible route to funding, in the way that an
expensive demonstration is for a funded lab. That is a legitimate reason to build
something and it is recorded here so that the goal does not quietly drift into
"an interesting simulator", which is a different and less valuable artifact.

## 2. What follows from the thesis — the pivots

These reorder the work. They were not obvious before the thesis was written down.

### P1. The Pi 5 benchmark is the headline experiment, not housekeeping

**The central claim of this project is currently unmeasured.** Every timing here
is dev-box: 22 ms integrate, 1 ms plan, ~2 ms estimated scan match. A Cortex-A76
on memory-bound work is plausibly 2–3× slower, which would put mapping alone at
50–60 ms inside a 100 ms cycle *before* anything planned is added.

You cannot claim "cheap compute suffices" while the only evidence comes from a
machine that isn't the constraint. Run `voxel_live --replay` on the actual Pi 5
and publish the number. An afternoon, and it reprices every other document in
this repo.

### P2. Algorithmic efficiency is the product, not an optimisation

Under this thesis, wasted cycles attack the claim directly. Three specific items
move from "nice to have" to "the argument":

* **Projective integration instead of raycasting.** `integrate` casts one DDA ray
  per pixel: ~100 k rays × ~20 cells ≈ **2 M cell visits**. Iterating the voxels
  in the frustum and projecting each into the depth image is ~800 m³ / 0.0156 m³
  ≈ **51 k voxels** — roughly **40× fewer**. Raycasting massively oversamples the
  near field, which is precisely the redundancy `integrateStride` exists to fight.
  **That knob discards angular resolution — the one quantity the sensor measures
  precisely — to pay for an inefficient method.** Wrong trade for this thesis.
* **An ESDF instead of `sphereClear`.** The sphere test iterates 343 cells to
  return a boolean, and the score then sums that boolean as a proxy for clearance.
  A distance field returns the metres directly, in one lookup, and carries
  gradients — which is the prerequisite for any trajectory refinement later.
* **librealsense's own post-processing.** Decimation, spatial, temporal and
  hole-filling blocks ship with the SDK, cost nothing to try, and are tuned by the
  vendor against this sensor. The temporal filter with persistence is exactly what
  a slow-moving camera wants. At minimum, A/B them against `depth_improve`.

### P3. `nav-sim` / `onboard` convergence is on the critical path

`grep VoxelMap onboard/` returns nothing. The stack we measure is not the stack
that would fly. Under this thesis the deliverable is **a flight**, so this stops
being tidy-up and becomes a blocker. Every week it is deferred, more measured
evidence describes a program that will never leave the desktop.

### P4. Everything specced on 2026-08-12 is deferred behind the flight

`APPEARANCE_AND_BLOBS_PLAN.md` and `POSE_AND_OPENNESS_PLAN.md` — blobs, sky
openness, the angular far map, goal-not-score, decay, tile index, scan matching,
IMU attitude — are good ideas in the wrong order. They are written down, which
was the point. They stay written down until something flies.

The exception is any item that is *also* a P2 efficiency win, which earns its way
forward on the thesis rather than on the feature.

### P5. Speed is a stated operating point, not an embarrassment

`Z_max = √(cell·f·B/σ_d)` is code, not a constant, so the sensor genuinely is a
parameter — a 12 cm baseline is a config change, not a rewrite. With the D435i's
50 mm baseline at 0.25 m cells the honest marking range is 3.5 m, and stopping
distance `v²/2a + v·t_react` caps the safe cruise at roughly **2–3 m/s**.

State that as the v1 operating point and move on. It is not a flaw to apologise
for; it is a derived consequence of a deliberately cheap sensor, and the
architecture already parameterises the way out.

## 3. Standards — the reframe

One person, evenings, no team, no lab. The standards below are the ones that
actually apply, and they are deliberately not the standards of a funded group.

* **Measured and honest beats optimal.** A 22 ms integrate that flies is worth
  more than a 2 ms one that does not exist. P2 matters *because it is the thesis*,
  not because slow code is shameful.
* **Slack is allowed. Unmeasured claims are not.** Rough edges, unoptimised
  paths, missing features and deferred plans are all fine. Asserting a number
  nobody measured is the one thing that isn't, because the second claim (§1.1) is
  the one that would not survive it.
* **Negative results are assets.** They are the evidence that the method is real.
  Keep them.
* **Rough weather is expected, and does not falsify anything.** The Pi benchmark
  will probably be worse than hoped. σ_d will probably be worse than the assumed
  0.25 px. Early flights will fail. First real-forest depth will look nothing like
  the sim. All of that sets the operating point; none of it touches the thesis.
* **The one result that WOULD falsify the thesis** is discovering that the task
  genuinely requires a GPU. If that happens, say so plainly and in public — a
  clean negative on "can this be done cheaply" is a real contribution and a better
  outcome than a quiet pivot to a Jetson.

## 4. Definition of done — v1

The project has had no stated finish line, which is how a project like this
improves forever. This is v1, and it is deliberately modest:

> **Sixty seconds of continuous autonomous flight in a real wood, no contact with
> anything, at ≥ 2 m/s, with all perception and planning running on the Raspberry
> Pi 5, and a logged compute figure showing headroom.**

Supporting evidence that makes it a demonstration rather than an anecdote:

* three or more runs, at least one not in flat midday light;
* the onboard log: per-frame integrate, plan and total, plus valid-pixel fraction;
* the raw `.kdr` recording of each run, so the claim is re-checkable offline;
* an honest account of every failed attempt alongside it.

Explicitly **not** in v1: goal-directed navigation, returning home, global
mapping, loop closure, dynamic obstacles, speeds above 3 m/s, night flight.

## 5. Order of work

1. **Pi 5 benchmark of the current stack** (P1). Afternoon. Reprices everything.
2. **`nav-sim` → `onboard` convergence** (P3). The flight code has to be the
   measured code.
3. **P2 efficiency, in the order the benchmark says.** Projective integration
   first if mapping dominates, which it almost certainly will.
4. **Real forest recordings.** Thirty minutes of `.kdr` from a real wood, various
   light. Worth more than any plan document in this repo, because every threshold
   in them is denominated in properties of real depth nobody has looked at.
5. **σ_d measured** (`d435i_probe.py --preset sweep`), retiring the last
   load-bearing assumption.
6. **First tethered / low-and-slow flight.** Then v1.

Everything in the plan documents comes after this list, and the plan documents
say so.
