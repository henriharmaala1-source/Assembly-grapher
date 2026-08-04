# Voxel navigation sim — build and run

Plain C++17 + OpenCV. No ROS, no Gazebo, no GPU, no Python. Builds and runs the
same on Windows 11, Linux and macOS.

## Windows 11

Install [vcpkg](https://vcpkg.io) once, then:

```powershell
git clone https://github.com/microsoft/vcpkg
.\vcpkg\bootstrap-vcpkg.bat
.\vcpkg\vcpkg install opencv4[core,imgproc,imgcodecs,highgui]:x64-windows

cd nav-sim
cmake -B build -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
.\build\Release\voxel_sim.exe --world forest --display
```

If you would rather not use vcpkg, download the prebuilt OpenCV Windows release,
unzip it, and point CMake at it:

```powershell
cmake -B build -DOpenCV_DIR=C:/opencv/build/x64/vc16/lib
cmake --build build --config Release
```

Add `C:\opencv\build\x64\vc16\bin` to `PATH` so the DLLs are found at runtime.

Visual Studio users: `cmake -B build` then open `build\nav_sim.sln`.

## Linux / macOS

```bash
sudo apt install libopencv-dev cmake build-essential   # or: brew install opencv cmake
cd nav-sim && cmake -B build && cmake --build build -j
./build/voxel_sim --world forest --display
```

## voxel_gui — point and click, no flags

```
./build/voxel_gui          # Windows: build\Release\voxel_gui.exe
```

A menu appears. Click a world (Forest, City, Hervanta, Tampere centre, Helsinki
centre), pick **Simulated stereo** or **Perfect (control)**, step the seed and
step count with +/-, then **FLY**. During flight: `space` pause, `r` restart the
same run, `m` back to the menu, `q` quit. On a collision it holds the frame so
you can see where it happened rather than the window vanishing.

Same three live panes as before — truth world with the flown path and the
current OMPL path, the voxel map slice with unknown space in grey, and the depth
image with unmatched pixels grey — plus a two-line status bar.

`voxel_gui` is for looking at one run with your eyes. **`voxel_sim` stays the
scriptable entry point** and `sweep.sh` drives it for multi-seed batches, which
is where any number worth quoting comes from — a single run you watched is an
anecdote, however convincing it looked.

The OSM worlds need their footprint files first:

```bash
python3 worlds/make_fi_cities.py
```

## Multiple maps, batch runs, and data

Worlds are **procedurally seeded**, so `--seed N` gives a different forest or
city each time. This was hard-coded to 1 until recently, which meant every result
quoted from this harness was one sample of one world presented as a property of
the planner. Use `sweep.sh` rather than single runs:

```bash
./sweep.sh --truth 5          # 2 worlds x 5 seeds, perfect-depth control
./sweep.sh "" 8               # stereo, 8 seeds
./sweep.sh "" 5 forest        # one world
```

Measured, truth depth, 5 seeds per world:

```
world    seed  outcome           travel  end-dist   minClr  falseFree
forest   1     no collision       225.2      75.6     0.39     0.000%
forest   2     no collision       225.1      70.7     0.44     0.575%
forest   3     no collision       237.3      72.9     0.49     0.000%
forest   4     no collision       232.8      75.8     0.38     0.000%
forest   5     no collision       230.6      78.0     0.35     0.405%
city     1     no collision       297.3     188.5     0.78     1.838%
city     2     no collision       334.4     198.8     0.42     6.434%
city     3     COLLIDED           191.6     160.8     0.16     0.642%
city     4     COLLIDED           110.6     215.7     0.18     1.795%
city     5     no collision       234.4     130.4     0.66     0.385%
---  10 runs, 2 collisions (20%), 0 goals reached
```

The forest numbers are genuinely tight across seeds (225-237 m travelled,
70-78 m end distance) -- that is a reproducible property. The city is not:
2 of 5 collide even with perfect depth. And **no seed in any world reaches its
goal**, which remains the open problem.

`--csv path` writes a per-step log (position, yaw, speed, freeM, openM, blocked,
path found, waypoint count, true clearance, distance to goal) for offline
analysis.

## Running

`--display` opens a live window with three panes and a status bar:

| pane | shows |
|---|---|
| **TRUTH + flown path** | the real world at the current flight height, the flown trail in red, the current A\* path in green, goal circle |
| **VOXEL MAP slice** | what the aircraft actually believes — white free, black occupied, **grey unknown** |
| **DEPTH** | the depth image being fed in; grey pixels are where stereo found no match |

Keys: `space` pause/step, `q` or `Esc` quit.

```
voxel_sim --world forest|city     which world
          --goal E N U            goal in metres
          --truth                 perfect depth instead of simulated stereo  <-- ALWAYS RUN THIS
          --general-only          reactive layer only, no A*
          --cell 0.25             voxel size, m
          --steps 900             step limit (dt = 0.1 s)
          --replan 25             A* replan interval, steps
          --display               live GUI
          --out /tmp/nav          PNG output prefix
```

Exit code: `0` reached goal, `1` ran out of steps, `2` collided.

## Always run `--truth` as a control

The single most useful habit with this harness. Perfect depth removes the sensor
from the experiment:

* fails with `--truth` → **the planner is broken**
* works with `--truth`, fails without → **the map is the limit**, and the number
  you are looking at is a sensor result rather than a bug

Three real planner bugs were found this way within minutes of the harness first
running, each of which would have been invisible without the control:

1. Speed was gated on unknown-discounted *openness*, so an entirely unmapped
   direction scored ~6.6 m of "clearance" and the aircraft flew into a tree at
   1.5 m/s on step 18 — with perfect depth.
2. Replacing that with a *threshold* on confirmed-free range deadlocked: in
   dense forest the free run sits near 2 m, so "stop below 2 m" meant stop
   forever, and you cannot see further without moving. It is a stopping-distance
   budget, not a threshold.
3. Direction was scored on openness while speed was gated on free range, so the
   planner chose whichever direction held the most *unseen* space — precisely
   the direction with the least room to move into — and then refused to move.

## Status — read this before trusting a number

The precise planner is now **OMPL RRTConnect**, and the question it answers
changed with it. It no longer plans to a distant mission goal; it plans a short
path AHEAD along a requested bearing, ending on a horizon sphere ~25 m out. That
is what "keep going that way, safely" actually needs, and the local map is only
60 m across so the old formulation was projecting onto the boundary anyway.

**Cost, measured on a desktop x86 core:**

```
                    hand-rolled A*      OMPL RRTConnect
forward planner       236 ms/replan        1.10 ms/replan     214x
onboard total        20.2 ms/step         11.7 ms/step
```

The 236 ms figure was a 0.35-0.7 s stall once scaled to a Pi 5 — 3 to 7 lost
control cycles. At 1.1 ms it is irrelevant, and `planTimeS` is a **hard cap**,
which the A* had no equivalent of. Onboard total is now ~12 ms/step here, so
roughly 18-29 ms on a Pi 5, alongside the tracker (~16 ms) and StereoBM
(~22-36 ms).

**Flight quality, 900 steps:**

```
world/depth      outcome     travelled   min clearance   false-free
forest/truth     no collision   233.1 m          0.36 m       0.000%
forest/stereo    no collision   224.4 m          0.49 m       7.893%
city/truth       no collision   329.1 m          0.34 m       1.626%
city/stereo      COLLIDED        45.9 m          0.14 m       4.892%
```

### The same sparse-sampling error, three times

Every collision traced to one class of bug, and it took three occurrences to see
the pattern:

1. **The collision detector** sampled 26 rays outward. At r = 0.6 m those points
   are ~0.6 m apart, so a 0.2 m trunk read as clear. It reported 3.00 m of
   clearance one step before a collision 0.36 m away.
2. **The direction probe** used 48 azimuth bins = 0.65 m between rays at 5 m.
   Trees fitted between them.
3. **`sphereClear`** sampled 7 points — centre plus six axis directions at
   r = 0.6 m, which are **0.85 m apart**. A trunk at 45° azimuth sat exactly
   between two samples and was invisible.

Number 3 is why the aircraft still hit trees with *perfect depth*, a map with
*0.000% false-free cells*, and *voxels finer than the trunks* — a sweep over cell
sizes (0.40 / 0.25 / 0.125 m) collided at all three, which is what ruled out the
map and the raycasting and left only the planner. All three are now exhaustive
cell scans rather than point samples.

### What remains is a real sensor limitation, not a bug

`city/stereo` is the only remaining collision, and it carries **4.892%
false-free** while `city/truth` flies 329 m cleanly. The city has 25% glass
facades with texture below the matcher's threshold: **stereo returns nothing
from them, so they never become OCCUPIED, and the aircraft flies into a wall it
genuinely cannot see.** That is the failure this whole exercise was built to
surface, and it is physics rather than a defect. Mitigating it needs a sensor
that sees untextured surfaces, or a policy that refuses to enter large connected
UNKNOWN regions below the horizon.

### Cost of correctness

The exhaustive clearance test moved the general planner from 3.09 to 11.76
ms/step at 0.25 m cells (2.57 -> 2.93 ms at 0.40 m). Roughly 18-29 ms on a Pi 5.
Still affordable, but the obvious optimisation is to precompute a dilated
occupancy or distance field once per frame instead of scanning a ball at every
probe step — the ESDF approach EGO-Planner uses.

## Ready-made planners worth benchmarking against

This planner was written from scratch, which was reasonable for the reactive
layer and questionable for the A*. Before investing more in it, compare against:

* **OMPL** — the standard motion-planning library. BSD, plain C++, no ROS
  dependency, installable via vcpkg on Windows. RRT\*, BIT\*, informed RRT\*, PRM.
* **FCL** — collision checking, pairs with OMPL.
* **EGO-Planner / Fast-Planner** (ZJU / HKUST) — purpose-built for quadrotor
  flight in cluttered 3D from a local map. State of the art for this exact
  problem, but ROS-coupled and would need lifting out.

Benchmarking against OMPL would separate "this planner is buggy" from "this
problem is hard", which the current numbers cannot do.
