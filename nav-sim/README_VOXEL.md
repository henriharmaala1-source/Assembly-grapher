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

**Flight quality, 1200 steps:**

```
world/depth      outcome     travelled   min clearance   false-free
forest/truth     COLLIDED       76.7 m          0.09 m       0.000%
forest/stereo    COLLIDED      111.2 m          0.27 m       7.360%
city/truth       COLLIDED      152.4 m          0.04 m       3.286%
city/stereo      COLLIDED       45.6 m          0.18 m       4.880%
```

### The deadlock is fixed, and it was not the log-odds

The hypothesis was that `lHit` (0.85) outweighing `lMiss` (0.40) made FREE
unreachable. **That was wrong** — a sweep of five (lHit, lMiss, occThresh,
freeThresh) combinations gave 0.0 m travelled in every one.

The real cause, found by histogramming what the camera actually returns at the
city spawn:

```
truth    invalid 35259   <8m     0   8-25m 33330   >25m 8211   nearest 10.39 m
stereo   invalid 56975   <8m    16   8-25m 19131   >25m  678   nearest  0.64 m
```

**Zero truth pixels under 8 m.** With `maxIntegM = 8`, not a single ray was
integrated, the map stayed completely empty, nothing was ever FREE, and the
aircraft sat still for 1200 steps. And the only reason the *stereo* runs moved
was those 16 sub-8 m pixels — 0.4% **speckle outliers**. They were flying on
sensor noise.

The bug was conflating two separate decisions. `maxIntegM` exists because a far
return's POSITION is untrustworthy (error grows as Z²), but the code discarded
the ENTIRE RAY when the endpoint was too far, throwing away the free space along
it too. A ray returning 30 m still proves the first several metres are empty.
Now free space is carved out to `r − 2σ(r)` (capped at `maxCarveM`), while
OCCUPIED is marked only when `r ≤ maxIntegM`.

`city/truth` went 0.0 m → 152.4 m. Forest regressed (299 m → 77 m), so this
traded one failure for another rather than fixing everything.

### The new problem: false-free under motion

`city/truth` now reports **3.286% false-free where it previously reported
0.000%** — but the old figure was an artifact of never moving. The same pattern
holds in the forest: ~7% on moving runs against 2.95% in the static sweep at the
same integration range.

**Motion is corrupting the map.** That is now the top suspect for all four
collisions, and it is upstream of anything in the planner. The prime candidate is
`VoxelMap::recentre()`, which shifts the whole grid by whole cells every step
and was effectively never exercised before, since the runs that scored 0.000%
were the runs that never moved. Next step is to test with recentring disabled and
a map large enough not to need it.

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
