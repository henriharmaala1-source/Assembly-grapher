# nav-sim — a path-planning testbench

A small, self-contained desktop simulator for **comparing path-planning methods**
on a drone driving to a goal through obstacles, over an occupancy grid it builds
from its own forward scan as it moves (partial observability → replanning).

It is deliberately standalone: no dependency on any other project, no GPU, no
ROS. OpenCV is the only dependency (for image ops + optional visualization).

![example](docs/example.png)

## What it does

A kinematic drone starts at the origin and must reach a (random or fixed) goal.
Each tick it casts a forward range scan, integrates a **log-odds occupancy grid**,
runs the selected **planner** on the latest (partial) map, and follows the plan
with a lookahead + a reactive slow-down. Everything is measured against ground
truth: did it reach the goal, did it keep standoff, how long was the path, how
long did planning take.

## Planners

| name | method | character |
|---|---|---|
| `wavefront` | BFS flood from the goal + gradient descent | complete, near-optimal, but floods the whole grid every tick (slow) |
| `dijkstra`  | 8-connected weighted search from the start | complete, near-optimal, faster than wavefront |
| `astar`     | Dijkstra + octile heuristic | complete, near-optimal, fastest of the exhaustive methods |
| `potential` | attractive-to-goal + repulsive-from-obstacles, one step | reactive, near-free, **traps in local minima** (included to show it) |
| `rrt`       | sampling-based tree growth with goal bias | scales to open space, jagged/suboptimal paths |

## Build

```bash
cmake -B build && cmake --build build -j4     # needs OpenCV (core imgproc imgcodecs [highgui])
```

`highgui` is optional — without it everything works headless and `--save=<dir>`
still dumps PNGs.

## Run

```bash
./build/nav_sim --list-planners
./build/nav_sim --planner=astar --goal=random --seed=7        # one run, verbose
./build/nav_sim --compare --seed=7                            # all planners, same world
./build/nav_sim --planner=potential --batch=200               # mass stats (reach %, traps)
./build/nav_sim --planner=astar --seed=7 --display            # live window (needs highgui)
./build/nav_sim --planner=astar --seed=7 --save=/tmp/frames   # PNG dump (headless-friendly)
```

Flags: `--planner=`, `--goal=random|E,N`, `--world=random|empty`, `--obstacles=N`,
`--seed=`, `--batch=N`, `--compare`, `--display`, `--save=<dir>`.

Example comparison output (same world + goal, so the differences are the methods):

```
  wavefront  reached=YES collided=no standoff=1.11m pathLen=18.87m (0.98x) avgPlan=12.725ms
  dijkstra   reached=YES collided=no standoff=1.13m pathLen=18.96m (0.98x) avgPlan= 1.071ms
  astar      reached=YES collided=no standoff=0.55m pathLen=18.77m (0.97x) avgPlan= 0.056ms
  potential  reached=YES collided=no standoff=3.39m pathLen=22.46m (1.17x) avgPlan= 0.001ms
  rrt        reached=YES collided=no standoff=1.40m pathLen=21.32m (1.11x) avgPlan= 0.035ms
```

## Honest scope

This validates planning **logic and behaviour** under partial observability and
replanning. The world is clean synthetic geometry — it is **not** a
photorealistic renderer and says nothing about real-camera/analog-capture
survival. It is a planner sandbox, not a hardware simulator.

## License

TBD by the author.

---

*This directory is fully self-contained. To lift it into its own git repository:*
```bash
git subtree split --prefix=nav-sim -b nav-sim-standalone   # from the parent repo
# then push nav-sim-standalone to a fresh empty remote, or just copy the folder
# and `git init` — it has no external file dependencies.
```
