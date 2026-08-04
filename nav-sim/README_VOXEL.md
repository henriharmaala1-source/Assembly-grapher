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

**The harness works. The planners still collide.** Current matrix, 1200 steps:

```
world/depth      outcome        travelled   to-goal   min clearance
forest/truth     COLLIDED         104.0 m   156.9 m          0.19 m
forest/stereo    COLLIDED         358.0 m   138.1 m          0.29 m
city/truth       deadlocked         0.0 m   240.1 m          2.00 m
city/stereo      timeout          425.1 m   227.6 m          0.86 m
```

Three causes were found, and **two of them were the harness lying, not the
planner**:

1. **The collision detector itself was broken.** It sampled 26 rays outward and
   took the first hit. At r = 0.6 m those sample points are ~0.6 m apart on the
   sphere, so a 0.2 m forest trunk sits between them and reads as clear. It
   reported 3.00 m of clearance one step before a collision 0.36 m away, which
   is geometrically impossible and is what gave it away. The aircraft was
   *spawning inside trees* — the real clearance at the fixed start point was
   0.50 m, not the 3.00 m reported. Replaced with an exhaustive voxel scan.
2. **The planner probe had the identical bug.** 48 azimuth bins is 7.5°, i.e.
   0.65 m between rays at 5 m, against trunks of 0.10–0.35 m. Trees were
   literally invisible to it. Fixed with a swept-sphere test over the robot's
   full width, plus 96 bins.
3. **Genuine planner immaturity.** Even after both fixes it flies 104 m
   (truth) / 358 m (stereo) and then hits something, and `city/truth`
   deadlocks outright. Not solved.

The general lesson, which cost real time twice in one session: *check the
instrument before believing the experiment.* A detector that can miss obstacles
makes every number the harness prints meaningless.

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
