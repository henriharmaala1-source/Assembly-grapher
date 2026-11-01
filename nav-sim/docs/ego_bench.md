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

## Not measured yet

- **`maxVel = 3`,** the same top speed as the primitives, for a speed-fair
  travel column.
- **`unknownFree = false`.** Does EGO's crash rate come from flying into
  unknown space? This is the experiment this tree's rule predicts.
- **More seeds.** The crash difference is not resolved at four per world.
- **Stereo depth.** These runs used truth depth.
