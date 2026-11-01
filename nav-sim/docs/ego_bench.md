# EGO-Planner-style optimisation against the classical bar

Does smooth trajectory optimisation (what the common ROS forest stacks fly:
RTAB-Map plus EGO-Planner) travel more safely than picking motion primitives?
`ego` in `kestrel bench` is a re-implementation of the EGO-Planner method
(`ego_planner.hpp`):

- a B-spline toward a local target, pushed out of obstacles by A*-derived
  anchors;
- optimised with L-BFGS, with rebound and time reallocation;
- unknown space treated as free, as EGO-Planner treats it.

It patrols goal → spawn → goal, and advises through the same mask as every
other planner: the admissible primitive closest to its trajectory is flown,
so the veto still applies.

**Setup:** all six worlds × seeds 101–104 (four seeds per arm, the
repository's minimum), 3000 steps, truth depth, corefrac 0. Raw:
[ego_bench_3000.txt](ego_bench_3000.txt).

## Results

| planner | collisions | travel | net | cells | net × cells |
|---|---|---|---|---|---|
| **freeM** | **0 / 24** | **235.2 m** | 43.9 m | **165** | 11487 |
| ego | **4 / 24** | 128.8 m | **84.6 m** | 104 | 14062 |
| goal | 1 / 24 | 112.2 m | 64.6 m | 78 | 7136 |

Per world (mean of 4; collisions out of 4):

| world | freeM: travel / net / cells / crashes | ego: travel / net / cells / crashes |
|---|---|---|
| maze | 243 / 32 / 172 / 0 | 96 / 18 / 25 / **1** |
| forest | 284 / 166 / 320 / 0 | 192 / 161 / 242 / **2** |
| corridor | 225 / 18 / 184 / 0 | 95 / 28 / 46 / **1** |
| city | 155 / 5 / 67 / 0 | 74 / 16 / 20 / 0 |
| road | 242 / 34 / 115 / 0 | 171 / 171 / 172 / 0 |
| culdesac | 263 / 10 / 131 / 0 | 145 / 114 / 116 / 0 |

## What it says

- **Ego crashed in 4 of 24 runs; freeM in none.** Crashes are the failure
  this project scores (CLAUDE.md). Ego hit something in the maze, the
  corridor and twice in the forest, despite the same veto under it.
  - That fits ego treating unknown as free: it optimises toward targets
    through space nothing has measured, and the veto lets unknown space
    through at corefrac 0.
  - **4 vs 0 is suggestive, not resolved.** As a sign test on 24 paired
    runs, four discordant pairs all one way give p ≈ 0.13. More seeds, and
    a run with `unknownFree = false`, would settle it.
- **Ego goes further from the spawn** (85 m net against 44 m). That's
  because it is goal-directed: it flies the line to the waypoint, so in
  open worlds (forest, road, culdesac) it travels nearly straight. The
  higher net × cells is that straight-line flying, not better coverage:
  - it covers fewer cells (104 against 165);
  - it travels about half as far.
- **Mazes, corridors and the city are local-minimum country.** A local
  target 7.5 m along a straight line sits behind walls. With no global
  path, EGO steers into dead ends: 20–46 cells against freeM's 67–184.
  EGO-Planner was designed to be fed a global path or waypoints, which is
  how the ROS stacks use it.
- **Ego ran slower,** a handicap in the travel column. Its trajectories run
  at 2.5 m/s (`EgoParams::maxVel`), and the closest primitives are the
  2 m/s ones. freeM takes the 3 m/s primitives when the map allows. Road
  shows it plainly: ego flew 171.4 m in a straight line on every seed,
  exactly twice the goal planner's 86 m, which picks the 1 m/s primitives.
  So **the travel gap is partly speed, not planning.** The crash gap is
  not: ego crashed while flying slower.

## Second run: matched speed, and unknown as a wall

Following the research survey (`reports/GNSS denied navigation deep
survey.md`), ego was re-run on the same worlds and seeds with two changes:

- its top speed matches the primitives' (3 m/s, no longer 2.5);
- a variant treats unknown space as an obstacle (`--ego-known-only`).

Raw: [ego_bench_matched_3000.txt](ego_bench_matched_3000.txt).

| ego variant | crashes | travel | net | cells | net × cells |
|---|---|---|---|---|---|
| unknown = free (EGO-Planner's convention) | **5 / 24** | 126.3 m | 81.6 m | 110 | 14004 |
| unknown = wall | **3 / 24** | 129.9 m | 71.4 m | 86 | 9178 |
| (freeM, first run) | 0 / 24 | 235.2 m | 43.9 m | 165 | 11487 |

- **At the faster speed, unknown-as-free crashed in all four maze runs,**
  flying 13 m on average. Treating unknown as a wall brought the maze to
  1/4 and 100 m.
- **Unknown-as-wall still crashed 3 times; freeM never did.** So "unknown
  is free" explains part of ego's crashes, not all of them. The brake mask
  below is the other part.
- The research survey's statistics apply to the first comparison:
  freeM 0/24 against ego 4/24 is a one-sided Fisher p ≈ 0.055.

## The brake mask: stop inside what has been seen

`kestrel bench --brake-mask` admits a primitive only if it can stop inside
its own confirmed-free length at its own speed. That is the aircraft
planner's speed law used as a veto. Without it, the bench flies a chosen
primitive at full commanded speed with only 0.4 m seen ahead. The rule is
the one the literature's safe planners share (FASTER, SUPER).

Same worlds and seeds. The "no mask" rows for freeM, goal and ego come from
the runs above. Raw: [brake_mask_3000.txt](brake_mask_3000.txt).

| planner | crashes, no mask | crashes, brake mask | travel | net × cells |
|---|---|---|---|---|
| score | 6 / 24 | **0 / 24** | 109.1 → 109.8 m | 7758 → **11377** |
| ego | 5 / 24 | **2 / 24** | 126.3 → 110.5 m | 14004 → 13741 |
| random | 4 / 24 | 4 / 24 | 115.0 → 85.5 m | 3093 → 1390 |
| frontRaw | 3 / 24 | 3 / 24 | 200.5 → 169.3 m | 9273 → 10782 |
| goal | 1 / 24 | **0 / 24** | 112.2 → 75.7 m | 7136 → 5927 |
| freeM | 0 / 24 | 0 / 24 | 235.2 → 183.9 m | 11487 → 10137 |
| **all six** | **19** | **9** | | |

- **It halves the crashes across the six planners (19 → 9).**
  - It removes them for `score` (6 → 0 on the same seeds; a sign test gives
    p ≈ 0.03) and for `goal`.
  - It cuts `ego` from 5 to 2.
  - `score`'s net × cells rises, from 7758 to 11377: it was being let fly
    faster than it could stop, and crashing for it.
- **It does nothing for `random` and `frontRaw`.** The likely cause, not
  yet checked (bench does not print whether a crash was into unmapped
  space), is the body brushing unknown space beside a confirmed-free
  centreline. That would make it the corefrac question, not stopping
  distance. `frontRaw` steers at unknown space on purpose.
- **The cost is distance for the planners that were already safe.** freeM
  flies 22% less (235 → 184 m) with the same zero crashes.
- So it is a large safety gain for planners that chase a goal or a score,
  and a speed tax on the one that already respected stopping distance
  implicitly. It stays an option, off by default, so every earlier number
  remains comparable. **The obvious next step is the learned policy under
  the mask**, since the policy chooses from the same mask.

## Not measured yet

- ~~Matched speed and `unknownFree = false`~~: done, above.
- **The brake mask with stereo depth, and on the learned policy.**
- **More seeds.** The crash difference is not resolved at four per world.
- **Stereo depth.** These runs used truth depth.
