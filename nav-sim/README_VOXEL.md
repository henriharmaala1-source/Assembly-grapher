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
centre), pick **Simulated stereo** or **Perfect (control)**, choose **Follow a
trail** or **Open stand** for the forest, step the seed and step count with +/-,
then **FLY**.

During flight:

| key | does |
|---|---|
| `space` | pause |
| `r` / `m` / `q` | restart the same run / back to the menu / quit |
| `-` `+` | **playback speed** — 0.25x, 0.5x, 1x, 2x, 4x, max |
| `v` | **first person** ⟷ outside view in the top-right pane |
| `[` `]` | **view distance** of the voxel pane — 20 m / 32 m / 44 m / 60 m |
| `<-` `->` | rotate the voxel model (`s` toggles auto-spin) |

On a collision it holds the frame so you can see where it happened rather than
the window vanishing.

Four live panes in a 2x2 grid:

| pane | shows |
|---|---|
| **TRUTH + path** | the real world at flight height, flown trail in red, current path in green, forest trails in pale blue, goal circle |
| **FIRST PERSON** *(default)* | the map from inside, out of the aircraft's own eyes — one raycast per pixel, shaded by cube face. `v` swaps this pane for the outside view |
| **VOXEL MODEL (built)** | the same map from outside, as solid blocks, **rotatable**, with the flown path, the planned path and the commanded heading drawn into it |
| **VOXEL MAP** | a horizontal slice — white free, black occupied, **grey unknown** |
| **DEPTH** | the depth image going in; grey pixels are where stereo found no match |

The voxel-model pane is the one worth watching. A 2D slice shows a single
height and hides everything above and below it, so "the map looks nothing like
the world" is invisible in it. The 3D view makes that obvious at a glance — and
comparing it against the truth pane beside it is the fastest way to see whether
a failure is the sensor or the planner.

Blocks are coloured by height **relative to the aircraft**: red is at your
altitude and is what you are about to hit, green is below, blue is above.
Absolute height was tried first and coloured the entire model one shade of blue,
because a forest occupies 8 m of a 24 m map — it answered a question nobody was
asking.

The blocks are drawn at a chosen **display pitch** (1.5 m by default), which is
not the map's resolution (0.25 m). This matters: the first version drew one cube
per map cell, which is 240 cubes across a 440-pixel pane — 1.05 px each. The
cube renderer with its three shaded faces was working perfectly and was
completely invisible. Downsampling for display is an OR, never an average, so a
block is drawn if anything solid is inside it and no obstacle is ever lost to
the display. `build/iso_render_check` dumps the pane at every pitch headlessly
and prints the pixels-per-cube, which is the number that decides whether you can
see anything at all.

### First person

`v` puts you inside the map the aircraft built. One raycast per pixel through
the same Amanatides & Woo traversal the depth camera uses, shaded by *which cube
face* was hit — a grid of cubes with one brightness per cube reads as noise, and
with one per face reads as geometry. About 0.5 µs per ray, so a 440 px pane is
~50 ms. That is fine for a desktop window and would never run onboard; it is a
visualisation, not part of the flight loop.

It renders the **map, never the world**, and that distinction is the entire
point. The outside view shows you the model; this shows you what flying inside
that model would be like. Where the model is wrong, you fly into fog.

Two things it deliberately does not flatter:

* **UNKNOWN is drawn as fog, not as air.** A first-person view that showed
  unmapped space as clear would be the most convincing possible way to tell the
  one lie this whole pipeline exists to prevent. Surfaces seen *through* unknown
  space are faded in proportion, so a wall behind 4 m of nothing looks like the
  guess it is.
* **The horizon is short, and it should be.** At 0.25 m cells on a 12 cm
  baseline the map can only honestly mark obstacles to
  `Z_max = √(cell·f·B/σ_d)` ≈ **5.2 m**. That is a sensor property, not a
  rendering limit — a 25 cm baseline would give 7.6 m, and 2 m cells would give
  14.8 m. Seeing where the world stops is the useful part.

The truth pane takes the **max over ±1.5 m** of flight height and dilates the
obstacle mask by one cell before downscaling. Neither is cosmetic: a 0.10–0.35 m
trunk on a 0.25 m grid is one or two cells, and an 800-cell map resized into a
440-pixel pane averages that straight into the background. A 1200 stems/ha
forest rendered as an empty field with a few specks in it.

`voxel_gui` is for looking at one run with your eyes. **`voxel_sim` stays the
scriptable entry point** and `sweep.sh` drives it for multi-seed batches, which
is where any number worth quoting comes from — a single run you watched is an
anecdote, however convincing it looked.

If you want to see the layout without building the GUI — over ssh, in CI, or
before deciding whether it is worth compiling — `build/gui_preview out.png`
flies a real planned run and writes the same 2×2 composite to a file.

The OSM worlds need their footprint files first:

```bash
python3 worlds/make_fi_cities.py
```

### Forest trails

`genForest` threads three winding cleared corridors, 3.5 m wide, through the
plot. They are a rejection rule rather than a post-hoc clearing: stems and scrub
are never placed within half the corridor width plus their own trunk radius, so
the clear width is 3.5 m whatever thickness the nearest tree happens to be. The
trail floor is retextured to packed earth (0.5 against the forest floor's 0.7),
which makes it very slightly *harder* for stereo than the ground beside it — a
planner that hugs a trail is not being handed easier perception as a reward.

```bash
./build/voxel_sim --world forest --trail 0 --seed 3
```

starts at one end of trail 0 and puts the goal at the other, then reports what
fraction of the flight stayed within 2.5 m of the centreline. Before flying it
walks the centreline and prints the worst true clearance on it — check the
instrument before believing the experiment, because one stem left standing in
the corridor would make a perfect follower look like a failure.

A managed boreal forest genuinely is threaded with skid roads and ditch lines,
so this is not a synthetic convenience. It is also a sharper test than open
stand: crossing a trail tells you nothing, while following one asks whether the
planner will turn *down* a lane that is narrower than the gaps either side of
it.

**The planner currently fails that test**, and the trail exists so that the
failure is visible rather than assumed away. Stereo depth, 900 steps:

```
seed  corridor min clr   on-trail   mean deviation   end distance
 1        1.67 m           22%           4.6 m       102 of 213 m
 2        1.67 m           61%           3.3 m        75 of 190 m
 3        1.63 m           37%           6.6 m        99 of 204 m
```

The corridor check passes in all three (1.63-1.67 m against a designed 1.75 m
half-width), so the low following scores are the planner's, not the world's.
Nothing in the reactive layer biases it toward a corridor: the goal direction
pulls it at the far end of the trail and it cuts across the stand, arriving in
roughly the right place having ignored the lane entirely. Trail-aware steering
is not implemented, and this is the measurement that says how much it is worth.

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

> **The tables below predate the trail change and are not comparable to a run
> today.** Trail generation draws from the same RNG before the stem loop, so
> `--seed 1` now builds a different forest — every tree moved. Re-run rather
> than reading old and new numbers side by side. This is exactly the trap
> `ablate.sh` warns about in its header.

Measured on the *pre-trail* worlds, truth depth, 5 seeds per world:

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

The forest numbers were genuinely tight across seeds (225-237 m travelled,
70-78 m end distance) -- that is a reproducible property. The city was not:
2 of 5 collided even with perfect depth. And **no seed in any world reached its
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

## Heading churn — what fixed it and what did not

The aircraft used to spend most of its motion turning. Measured on the forest,
*churn* is mean absolute yaw change per step and *advance* is net displacement ÷
distance travelled (1.0 is a straight line, 0.0 is a closed loop).

Two separate causes. `vMax` was 6 m/s in a stand whose typical trunk gap is
2.67 m — the aircraft crossed a gap in under half a second while the map updated
at 10 Hz. And the planner re-ran its 864-bin argmax **every step**, 10 Hz, while
moving 0.3 m per step, so it re-decided about ten times per meaningful change in
the scene and the winning bin flipped between equally-open gaps.

Dropping to 3 m/s took churn from 42.6 to 30.0 on its own. The hold length was
then **swept rather than picked** — `commit_sweep.sh`, forest, 600 steps, 4 seeds:

```
hold   churn   reversals   advance             collisions
 0     30.01      7.6%     0.579 [0.49,0.68]      1/4
 1     18.89      2.2%     0.592 [0.57,0.61]      1/4   <- shipped
 2     17.09      1.3%     0.534 [0.52,0.56]      0/4
 3     13.65      1.3%     0.524 [0.48,0.61]      1/4
 5     11.78      0.5%     0.510 [0.47,0.54]      0/4
 8     10.18      0.2%     0.482 [0.44,0.50]      1/4
```

**One step is the only arm that dominates no-commitment** — less churn, fewer
reversals, *more* progress, and a much tighter spread across seeds. Everything
from 2 upward is a trade, buying smoothness with progress at a steadily worse
rate: 1 → 8 halves the churn and costs a fifth of the distance made good.

Halving the decision rate does most of the work, which is the useful lesson.
The problem was never that the planner decided badly. It was that it decided
again before its previous decision had produced any motion — the vehicle needs
~0.35 s just to turn, and it was being re-aimed every 0.1 s.

Collisions do not separate the arms at four seeds (1,1,0,1,0,1); that column is
noise at this count and is not a safety ordering.

### It was never the reactive planner

After all of the above the aircraft still turned aggressively, and the obvious
next move was a stronger direction bias. One measurement killed that idea:

```
GOAL bearing churn   21.5 deg/step     the reference handed to the planner
CMD  bearing churn   20.9 deg/step     what the planner does with it
```

The reactive layer's output churned *as much as its input*. It was not
spinning — it was faithfully tracking a reference that was, and no bias,
hysteresis or commitment applied downstream can fix a wobbling target. That is
also why `revPenalty` did nothing and why commitment plateaued: both act on the
wrong stage.

Three fixes, all **upstream** of the reactive layer, ablated one at a time
(`ref_sweep.sh`, forest, 300 steps, 2 seeds):

```
arm                  goalChurn  cmdChurn   advance          coll
none (as before)        20.02     19.07    0.569 [0.52,0.61]  0
pursuit only             2.17      2.44    0.889 [0.89,0.89]  0
reuse only              37.62     34.92    0.239 [0.16,0.32]  0
filter only             10.75      9.39    0.471 [0.46,0.48]  0
pursuit + reuse          2.67      2.42    0.908 [0.83,0.98]  1
all three                2.35      1.95    0.908 [0.89,0.93]  0   <- shipped
```

**Pure pursuit is the fix.** The old rule aimed at "the first path waypoint
more than 3 m away"; the path is interpolated to roughly every 2 m, so that
waypoint switched every few steps and each switch was a discrete bearing jump —
at 3 m lookahead, 1 m of lateral offset is an 18° step, for nothing. Aiming at
a fixed *arclength* along the path instead makes the target slide continuously.
Churn drops 9×, and the fraction of motion that gets you somewhere goes from
57% to 89%.

**Path reuse alone is actively harmful** — advance 0.24, worse than doing
nothing. A long-lived path with a hopping carrot is worse than a fresh one,
because the carrot has more path to hop along. It is only safe combined with
pursuit, and even then it buys ~0.02 advance. Enable it *only* with pursuit.

**The bearing filter alone halves the churn and costs progress** (0.569 →
0.471) — a low-pass adds lag, and lag is distance. Its value is in combination,
smoothing the step change a replan still causes.

Read the caveats: 2 seeds, 300 steps. The pursuit effect is 9× and unambiguous.
The differences *between* the last three arms are 0.02 in advance and cannot be
resolved at this sample size — "all three" is shipped because it is at worst
equal on every column, not because it is provably best. The 1 collision in
`pursuit + reuse` is one run out of two; do not read it as the filter fixing a
safety problem.

Side benefit, and a large one: a stable reference means the heading is held far
more often, so the general planner's single-bin fast path fires more.
**4.96 ms/step, down from 13.99 before any of this work**, and onboard total
22.9 ms/step down from 35.8.

Three further mechanisms were tried and **are off by default because they did
not work**: smoothing the direction field over time, requiring a challenger to
beat the incumbent by a margin, and charging extra for deviations beyond 90°.
None reduced churn further; all three cost advance (0.452–0.492 against 0.531).
`ablate.sh` reproduces the table, and the parameters are still there behind
`--ema`, `--dwell` and `--revpen`.

Read the spread before the means, though. Across three baseline seeds advance
ranged 0.460–0.642 — a spread of 0.182 — while the largest gap between arm means
is 0.080. **Three seeds cannot resolve a 0.08 effect.** The honest conclusion is
"no measurable benefit, possible harm", not "these are 8% worse".

One hypothesis died on the way: the field variance looked like stereo speckle
collapsing a direction's free run for one frame. The perfect-depth control
refuted it — 34% large swings on truth depth against 39% on stereo. The variance
is geometry sampled from a moving origin, not sensor noise.

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
