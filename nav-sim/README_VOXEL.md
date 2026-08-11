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
          --seed 1                world seed -- vary it before believing anything

          --lanes                 thickets and clear lanes side by side (route choice)
          --mixed                 clutter banded ACROSS the path (speed modulation)
          --camw/--camh/--hfov/--baseline    camera geometry; Z_max follows from it
          --vmax 1.5              speed cap, m/s

          --farcell 2.0           coarse awareness map cell size
          --farmode 0|1           openness metric: 0 first-blocked, 1 density (default)
          --mid                   add a 1 m rung to the ladder (measured: does not help)
          --nofar                 no coarse map at all

          --improve               depth improver: fill holes against NEAR returns only
          --impnear 2.0           how near a return has to be to seed a fill, m
          --imprad 4              fill kernel half-width, px
          --impseed 6             near returns needed in the kernel before filling
          --slip 20               max sideslip, deg -- velocity leads heading while turning
          --slipknee 40           yaw rate at which sideslip reaches half of --slip
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

### Realistic trunks break the stack

Real depth images from a boreal stand show the ground resolved beautifully and
the **trunks as solid holes** — bark in shadow against bright sky is a
low-contrast surface, and a matcher cannot correlate what it cannot see. Our
world model had the opposite: trunks carried `tex = 0.85`, the *highest* texture
in the world, on the stated reasoning that "bark is strongly textured, which is
why forests are navigable at all." That reasoning never accounted for
backlighting, and it made every safety number here optimistic about the one
obstacle that kills you.

Three corrections, all in the direction of *harder*:

* `trunkTexMin/Max` = 0.15–0.55, straddling `texThresh`. Some trunks vanish
  entirely, most drop out in patches, a few in good side light resolve fine.
* **Dropout is per-BLOCK, not per-pixel** (`blockPx = 8`). A matcher correlates
  a window, so a marginal surface fails in window-sized patches. The old
  per-pixel coin flip produced salt-and-pepper — a lace curtain instead of a
  hole — and a mapper that never sees a coherent hole is never tested against
  the failure that actually occurs.
* **Speckle/consistency rejection** after matching, modelling left-right checks
  and `filterSpeckles`. This is what gives real depth images clean hole *edges*.

The result, forest, 400 steps, 3 seeds:

```
arm          cmdChurn  advance  endDist  steps  meanMinClr  worstClr
histogram        1.20    0.995    131.7    148        0.38      0.32
trajlib          0.50    0.989    125.3    234        0.43      0.36
```

**Both arms now collide** — 148 and 234 steps of a requested 400. On seed 1 the
histogram hits a tree at step 17. That is the honest state: everything measured
before this section assumed trunks were visible to stereo, and they are not.

This is unwelcome and correct. The parameters are explicit and tunable
precisely so they can be calibrated against a real camera rather than against
my reading of somebody's screenshot — see the ten-minute backlit-bark contrast
experiment described above.

### Trunks have visible EDGES

A second batch of real depth imagery settled the question the first one raised.
Trunks appear as black columns **with vivid coloured stripes down one edge** —
a block matcher succeeding on the silhouette and failing on the interior. A
trunk against bright sky is a strong horizontal gradient; the inside of a smooth
shadowed cylinder is not.

**So a trunk is never invisible — only its middle is.** The outline comes back,
and an outline is enough to know something is there.

`renderStereo` is now three passes: raycast geometry, detect depth
discontinuities, then match. A pixel whose neighbourhood spans a large depth
step — or borders a no-return, which is the trunk-against-sky case and the
strongest cue of all — has its effective texture raised to `edgeBoost`, because
the discontinuity *is* the feature being correlated.

Trunk texture also moved 0.15–0.55 → **0.30–0.75**. The old range was an
over-correction from a single screenshot: it put the median at 0.35, below the
cliff, with a quarter of trunks wholly invisible. Bark in reasonable light
genuinely is textured — the original 0.85 was wrong about backlighting, not
about bark.

Forest, 400 steps, 4 seeds:

```
arm                   coll/4   travel   endDist   minClr
silhouette + far        0/4     69.3     108.6     0.31
silhouette, no far      0/4     68.2     108.1     0.31
```

**No collisions**, where the pre-silhouette model was flying into trees. Note
`minClr` 0.31 against a 0.30 collision threshold — it survives, but not with
much to spare.

### How good must the pose estimate be?

The map is only ever as good as the pose it is written with, and "do we need
VIO?" is not answerable by argument. `--drift` injects a random walk into the
pose the **mapper** believes while the true pose stays exact — which is
precisely what dead reckoning does — so the requirement can be measured.

Forest, 400 steps, 3 seeds. `--drift D` gives a position error std of `D·√T`;
for white velocity error `σ_v`, `D = σ_v·√dt`, so the second column is the spec
you would actually shop for or measure on a bench:

```
--drift  ~sigma_v     coll/3   travel   endDist  falseFree
0.00     perfect         0/3     91.1      87.1     0.002%
0.01     0.03 m/s        0/3    103.5      89.4     0.048%
0.03     0.09 m/s        0/3    106.3      69.6     0.149%
0.10     0.32 m/s        3/3     75.9      99.7     0.439%
0.30     0.95 m/s        1/3     94.9      80.8     1.095%
1.00     3.16 m/s        2/3     73.0     102.8     3.662%
```

**The requirement is roughly 0.1 m/s of velocity accuracy.** Clean at 0.09,
every seed collides at 0.32.

Read the **false-free** column rather than the collision count — it is
monotonic where the collisions are noisy at n=3. It also shows the mechanism:
pose error scatters observations of the same trunk across different cells, the
obstacle dissolves, and the map begins claiming free space where wood is.

Two things worth more than the headline:

* **Below ~0.1 m/s the odometry is not the bottleneck.** Drift contributes
  0.149% false-free there, against the 7.8% the *stereo sensor* contributed
  before the carve guard. Two orders of magnitude apart — there is no point
  buying better pose until the perception improves.
* **This is the friendly case.** A random walk models white velocity noise.
  Real optical-flow error carries bias and scale-factor terms that drift
  *systematically*, which is far more damaging at the same RMS. Treat 0.1 m/s
  as an optimistic bar.

So VIO is probably not a prerequisite: a PMW3901-class flow sensor over moss
and litter — the one surface every real depth image here resolves perfectly —
plausibly clears it. "Plausibly" is doing work in that sentence, and it is
directly testable: hover with GPS lock, log flow-derived velocity against GPS
velocity, take the RMS.

**Heading drift is deliberately not swept.** At 2 °/min it is 0.17° over a 5 s
fusion window — about 3 cm at 10 m. The local map does not care; yaw only costs
you when flying a bearing for minutes.

### Coarse far-field map

2 m cells alongside the 0.25 m map. Not a compromise: depth error grows as Z²,
so at 12 m a return genuinely has metres of uncertainty along the ray and a
0.25 m voxel claims precision the measurement does not contain. Sizing the cell
to the uncertainty is the honest thing, and it triples the range —
**Z_max 5.2 m → 14.8 m**.

Affordable because it takes every 4th pixel (`integrateStride`): a 2 m cell does
not need 76,800 rays when hundreds land inside it. Cost is inside the noise
(14.05 → 14.67 ms). It carries **877 occupied cells, 814 beyond the fine map's
8 m marking limit**.

The router consults it wherever the fine map has no opinion; the fine map always
wins where it *has* one, so the coarse layer can add an obstacle but never erase
one. It is wired to the router rather than the reactive layer because the
trajectory rollout is only ~6 m and never leaves the fine map.

**It has not yet earned its keep**, and the table above says so: 69.3 m against
68.2 m travelled, 0/4 collisions either way. Like the stall-triggered router, it
costs nothing and has not yet been the difference between success and failure —
because no world here has geometry that would reward it. And a coarser voxel
does not make an unmatched obstacle visible; it extends the horizon you can
steer by, nothing more.

### How visible must a trunk be? — the tolerance curve

> **This curve predates the silhouette model.** It was measured when a trunk
> resolved whole or vanished whole, which the real imagery says never happens.
> The shape of the argument stands — sweep the parameter rather than guess it —
> but the numbers need re-running before they mean anything about the current
> stack.

The trunk-visibility number came from **one screenshot** of somebody else's
depth output: unknown vintage, unknown pipeline, possibly a filter bug. A point
estimate from that is not evidence. So sweep it, and report the range over which
the stack survives — that turns the question into a spec you can go and measure.

Forest, 400 steps, 4 seeds, `--trunktex`:

```
trunkTex   coll/4   travel   endDist   minClr  falseFree
0.85         0/4     53.9     122.4     0.31    0.012%
0.70         0/4     53.9     122.4     0.31    0.012%
0.55         0/4     53.9     122.4     0.31    0.012%
0.40         1/4     69.2     107.3     0.30    0.001%
0.25         1/4     36.9     139.5     0.30    0.018%
0.15         4/4     57.9     118.9     0.26    0.001%
```

The identical top three rows are a **consistency check passing**, not a bug:
above `texThresh × 2 = 0.50` the dropout probability is exactly zero, so those
three worlds really are the same world. The meaningful range is 0.25–0.50 and
**the cliff is at 0.40**.

Run `build/bark_contrast` on a photograph of a backlit trunk to place a real
forest on that scale. Through its calibration that works out as roughly:

* **≥ 10 grey levels** of 10th-percentile per-window σ → safe
* **~6.5** → one run in four hits a tree
* **below the matcher's threshold** → every run hits a tree

Two caveats, because they matter more than the numbers. The *sim-texture*
thresholds are measured; the *grey-level* conversion is a calibration constant
invented in `bark_contrast.cpp` and flagged as such in its source — check it
against real depth output before trusting the physical figures. And four seeds
cannot distinguish 1/4 from 0/4: read 0.40 and 0.25 as "degraded", not as a
rate.

**The curve also exposed the cost of the safety fix — and the fix lost.**
Travel sat at ~54 m regardless of trunk visibility, where before the core-free
requirement the aircraft covered 118 m. So `coreFrac` was swept against the
thing it was supposed to buy:

```
trunkTex  coreFrac   coll/4   travel  endDist  stopped
0.70      0.00         0/4     68.3    107.9      169
0.70      0.65         0/4     53.9    122.4      217
0.25      0.00         0/4     56.1    120.6      210
0.25      0.65         1/4     36.9    139.5      180
0.15      0.00         4/4      6.9    169.1        0
0.15      0.65         4/4     57.9    118.9        1
```

**Never safer, usually slower.** At 0.25 it is the arm *with* the requirement
that collides — one run of four, so not significant, but certainly no evidence
for it. At 0.15 both fail completely and `coreFrac` only delays the crash,
which is cosmetic. It is now **off by default**, kept behind `--corefrac` with
the numbers attached.

The reasoning behind it still looks right: unknown space inside your own body's
volume is exactly where an unmatched obstacle hides. The measurement disagrees,
and the measurement wins. Note only 0 and 0.65 were tested — nothing in between.

What actually freezes the aircraft is worth recording, because "it only flew
54 m" sounds like a dead end and is not one. The stall is perfectly bimodal: on
every stopped step the confirmed-free run ahead is **exactly 0.15 m**, on every
moving step about 5 m, and only 2 of 173 stops report "blocked". It is a
sense-move-sense stutter — creep forward, outrun the confirmed-free frontier in
two steps, freeze until the map catches up. Distance to goal falls monotonically
throughout and advance is 0.97 while moving.

And one assumption that turned out backwards: **the carve guard improves
progress.** Without it a cell is marked occupied by one ray and carved free by
another, log-odds settles near zero, the cell reads UNKNOWN, and the planner
refuses to enter it. The guard removes that contradiction. Better on safety and
on speed:

```
corefrac 0.65 + carve guard    travel 59.6   stopped 197
corefrac 0.65, no guard        travel 50.6   stopped 229
corefrac 0    + carve guard    travel 91.7   stopped  92
corefrac 0,    no guard        travel 51.1   stopped 228
```

### Trajectory library instead of heading commands

The reactive layer used to answer "which bearing looks most open" and hand that
to a vehicle needing 0.35 s to turn. A bearing is not a thing the aircraft can
do; it is a wish. Every steering fix in this project — commitment, hysteresis,
the dwell margin, the reference low-pass — existed to paper over that mismatch.

`voxel_traj.*` replaces it with a library of motion primitives, precomputed in
the body frame by integrating **the vehicle's own first-order lag**, so every
candidate is a path the airframe can actually fly. Collision checking and the
stopping-distance speed budget are unchanged in kind.

It is also cheaper, which stops being surprising once stated — the histogram
ray-marches 864 directions and throws away 863:

```
openness field   96 x 9 bins, 1.88 M cell lookups     4.52 ms
trajectory lib   195 primitives, 0.23 M lookups       1.51 ms
onboard total                          20.7 ms  ->   18.3 ms
```

On the harsh world it also **halves command churn and survives 58% longer**
before hitting something. One subtlety was worth measuring: commanding a
straight bearing to a point on a *curved* rollout flies a chord, which bows
inside the tube that was collision-checked by `R(1-cos(θ/2))` — 0.10 m at
100 °/s and 0.4 s of lookahead. Aiming at 0.4 s cost 0.17 m of minimum
clearance; `aimS = 0.2` brings the deviation to 0.026 m and the planner then
*beats* the histogram on clearance instead of losing to it.

### Textured reconstruction

`integrate()` optionally takes an intensity image aligned to the depth and
stores one byte per cell — 5.5 MB at 240×240×96. `isoImage(..., colourByTexture)`
then draws the map with the world's own appearance instead of the height ramp.

Be clear what this buys: **nothing for the avionics.** It exists so a human can
tell at a glance whether the reconstruction resembles the place, which a
height-coloured blob cannot show. Debug instrumentation, not perception.

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

### The router is a fallback, not the normal case

Having fixed how the path is followed, the obvious next question is whether it
should be followed at all. Forest, 300 steps, 2 seeds:

```
arm            goalChurn  cmdChurn  advance   endDist
with OMPL           2.35      1.95    0.908      95.5
bearing only        0.01      0.68    0.979      87.9
```

**Pointing at the goal beats following a routed path on every column.** A stand
of scattered trunks has nothing large enough to route around, so the router
contributes no information and does contribute noise. And note `cmdChurn 0.68`:
given a steady reference, the reactive layer barely turns at all. It never had
a spinning problem — everything above was compensating for a middleman.

That is an argument about forests, not about routers. A reactive planner with a
12 m horizon cannot see out of a dead end, and a courtyard or a long facade is
exactly that. So the choice is not made once. `--router` selects:

* `never` — pure bearing following
* `stall` *(default)* — reactive until progress stops, then route
* `always` — the old behaviour

Stall means **no new closest approach to the goal for 4 s**. Closest approach
rather than current distance, because a vehicle circling a building has a
distance that oscillates without ever improving — precisely the case this must
catch. The router hands back only after buying 5 m of real progress, so the two
layers cannot flap.

Measured across both worlds, 400 steps, 3 seeds:

```
forest    cmdChurn  advance  endDist  steps  router  coll
never         0.57    0.988     57.3    400      0%     0
stall         0.57    0.988     57.3    400      0%     0
always        2.47    0.848     77.2    400      5%     0

city      cmdChurn  advance  endDist  steps  router  coll
never         4.85    0.812    172.5    338     12%*    1
stall         5.00    0.817    172.3    338     12%     1
always        3.91    0.750    181.2    279     10%     2
```
<sub>*under `never` the stall monitor still runs, so that column is when the
router *would* have been called, not when it was.</sub>

Three things, in order of confidence:

1. **`always` is worse everywhere.** Forest advance 0.848 against 0.988; city
   advance 0.750, twice the collisions, and it dies 59 steps sooner. Routing
   every step is a net negative in both worlds.
2. **`stall` is free.** In the forest it is bit-identical to `never`. In the
   city it engages 12% of the time and changes essentially nothing —
   advance 0.817 against 0.812, same collisions, same steps survived.
3. **The router has not yet earned its place anywhere in this harness.** Not
   harmful on stall, but not helpful either.

Point 3 is a statement about **these worlds, not about global planners**. The
city test compares two failures — both arms crash and neither gets close — and
neither world contains a genuine dead end. A fair test needs geometry where a
12 m reactive horizon provably traps the aircraft and a router provably frees
it: a U-shaped courtyard, a walled cul-de-sac. Until this harness has one, "the
router is useless" is not a claim the data supports; "nothing here rewards it"
is. The default stays `stall` because it costs nothing and is the only thing
that could handle that geometry when it exists.

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

## voxel_live — the real map and planner over REAL depth

Everything else in this directory runs on synthetic depth. `voxel_live` runs the
**same `VoxelMap` and the same `TrajectoryPlanner`** over frames from an actual
D435i, or from a recording of one.

**Double-click `voxel_live.exe`.** No arguments, no terminal. A menu opens:
pick **LIVE CAMERA**, **REPLAY A RECORDING** or **SIMULATED FOREST**, click a
recording, cycle the voxel size / speed cap / emitter / spin rate, then
**START**. `m` returns to the menu, `q` quits.

Recordings are found beside the exe and in the working directory, and each one
is **opened before it is listed** — the row shows its frame count, resolution
and emitter state, and a file that will not parse is greyed out with the reason
rather than failing after you click START.

Passing any argument takes the command-line path instead, which is what the
tests and sweep scripts use:

```
voxel_live --replay walk.kdr     a recording — NO CAMERA NEEDED
voxel_live --live                a connected D435i (needs the RealSense SDK)
voxel_live --sim                 the raycaster, as a control
voxel_live --menu-preview m.png  dump the menu layout to a PNG (no window)
```

Four panes in a 2×2 grid: **DEPTH** (grey = no return, not far), **FIRST
PERSON** (the map from inside — pale is UNKNOWN drawn as fog, never as air),
**MAP SLICE** (green ring = the occupancy MARK limit, orange = the free-space
CARVE limit), **PLAN** (faint = admissible primitives, green = chosen).

Press `v` (or use the menu) to cycle the top-right pane through **FIRST
PERSON**, **OVERLAY** and **CHASE** — the same voxel render composited *onto the depth
image it was built from*, at the camera's own resolution and horizontal FOV so
the two line up pixel for pixel.

The overlay is the pane that catches a class of fault nothing else here can. The
map and the depth image are independent estimates of the same scene: if the
voxels sit where the returns are, then the intrinsics, the frame convention and
the pose all agree. If they are offset, sheared or mirrored, one of the three is
wrong — and in every other view both halves would look individually plausible.
`overlay_align_check` pins it to sub-pixel and explicitly rejects a left/right
and an up/down flip, because on a forest a mirrored render looks entirely
convincing.

**CHASE** is the same renderer from a viewpoint behind and above the aircraft,
with every admissible rollout and the chosen one drawn into the scene. That
viewpoint is not decoration: forward paths are *invisible* from the aircraft's
own eye, because they run along the optical axis and project to a dot at the
vanishing point. Seeing a path as a path needs a viewpoint the path is not
pointing at. It is framed on the plan's own length (`horizonS × vMax`) rather
than on the map's range, for the same reason.

Two details in that pane are worth knowing, because both were wrong first and
both are invisible from the code:

* It renders the **0.25 m map**, not the finest layer available. The near layer's
  grid covers its own honest range and no more — that is what makes a fine layer
  cheap — so a camera two metres behind the aircraft is already at its edge.
* Its **unknown fog is turned down** (`FpvStyle`). Fog for metres of unmeasured
  space is honest when the eye is the aircraft's; when the eye is two metres
  behind it, that unknown is a property of where the eye was put, and at
  first-person strength it saturates immediately and whites out the pane.

The viewpoint sits 0.55 × span back and 0.62 × span up, aimed at the middle of
the plan — about 31° above it. Elevation is the number that matters: a path's
forward extent projects to `sin(elevation)` of itself, so a shallow chase angle
turns three metres of rollout into half a metre of picture.

Paths are projected with `VoxelMap::fpvProject`, the exact inverse of the ray
`fpvImageWH` casts — pinned to 0.0001 px by `overlay_align_check`, which also
asserts that a point behind the camera is rejected rather than mirrored to the
front. A projection written separately from the render it draws into is a sign
error waiting to happen, and one that looks entirely plausible.

The first-person pane renders the **map, never the world**: where the model is
wrong, you fly into fog. Its horizon is short because the map's honest marking
limit is short, which is a sensor property and the useful part to see. It is a
visualisation and is *not* counted in the onboard cost the run reports.

### Steering with no goal

On a bench there is no destination, so scoring alignment with a fixed bearing
would quietly pull every chosen path North. Two modes:

* **OPENNESS ONLY** (default, `--openness`) — `goalWeight = 0`. The score is
  clearance, smoothness and far-field openness: *where would you go if you only
  wanted room.*
* **GENERAL DIRECTION** (`--forward`) — the goal bearing is wherever the camera
  is pointing: *where would you go, roughly forwards.*

Keys: `space` pause, `s` save a PNG, `q` quit.

### Pose is the honest limit

A world-anchored map needs to know where each frame was taken from. The sim has
perfect pose by construction; a handheld camera has none, and inventing one
produces a map that looks plausible and means nothing. So the camera is assumed
**fixed**, or rotating in place at a rate you state with `--yawrate`. Translation
is not offered, because we cannot measure it yet — that is the missing odometry
(`StateEstimator::updateVisionVelocity()`), and it plugs in here when it exists.

**Move the camera and the map will be wrong.** That is not a bug in this tool.

### Recording a walk

`onboard/tools/d435i_probe.py --record N` writes both an `.npz` (for the noise
analysis) and a `.kdr` (for this). The `.kdr` format is a 64-byte header plus
raw `uint16` frames — see `depth_record.hpp`. It carries the camera's own
intrinsics, because the mapper carves along rays and a ray built from an assumed
pinhole is systematically wrong in a way that looks like a calibration fault once
it reaches the map.

### Live camera: nothing to configure

**There is no build-time RealSense dependency.** librealsense is loaded at *run*
time, so `voxel_live` builds identically whether or not the SDK is installed,
and `--live` works the moment the library is present anywhere the loader looks —
which includes the folder the Viewer put it in, and the PATH entry its installer
added.

Check it in one command, no camera needed:

```powershell
voxel_live --rs-check
```

That prints either `librealsense 2.58.3 (realsense2.dll)` or the exact list of
paths it tried. If it fails, the fix is to make the DLL findable — put its
folder on `PATH`, or copy `realsense2.dll` beside the exe — rather than to
reconfigure anything.

This replaced three rounds of increasingly clever CMake search paths that kept
failing to find an SDK that *was* installed. A dependency only needed at runtime
should not be able to break a build, and a build should not have to guess at an
install directory whose name changes between vendors.

**If it says "this build has no RealSense SDK" and you HAVE installed the SDK**,
CMake is simply not looking in the right place. A bare `find_package(realsense2)`
searches `<prefix>/lib/cmake/realsense2` for prefixes like
`C:/Program Files (x86)`, while the installer puts the package one directory
deeper, at `C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/cmake/realsense2`.
`cmake/find_realsense.cmake` now hints those locations and falls back to finding
the header and library by hand, so it should just work. If it still does not:

```powershell
cmake -B build -Drealsense2_DIR="C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/cmake/realsense2"
# or
$env:REALSENSE2_DIR = "C:/Program Files (x86)/Intel RealSense SDK 2.0"
```

Watch the configure output — it prints either `librealsense found` or a warning
with the exact flag to pass. On Windows the build also copies `realsense2.dll`
beside the exe, because otherwise the link succeeds and the program then refuses
to start.
