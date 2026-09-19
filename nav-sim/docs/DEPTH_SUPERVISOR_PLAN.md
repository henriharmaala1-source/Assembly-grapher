# Geometric safety supervisor on the raw depth image — PLAN

Status: **planned, not implemented.** Nothing in this document exists in the
tree yet. It is written now so that the design is fixed before the first line,
because the whole value of this component is that it is *simple enough to
trust*, and components like that are the easiest to accidentally complicate.

Deferred behind the v1 flight milestone by `../../THESIS.md` P4.

This is item (1) of the three things still worth taking from the two reference
papers. Items (2) and (3) — the depth improver and yaw-coupled lateral velocity
— are implemented; see `depth_improve.hpp` and `TrajParams::latSlipDeg`.

See also `APPEARANCE_AND_BLOBS_PLAN.md`, which is about extracting more from the
*existing* path from photons to map, where this document is about adding a second
independent one. The two share the rule in §3 — a mechanism that can only
withhold is a mechanism that cannot introduce a new way to die.

---

## 1. What it is, and why it is not the map

Everything that currently decides whether the aircraft may move reads the voxel
map: `sphereClear()` walks map cells, the speed budget counts confirmed-FREE
map distance, the openness term marches map levels. That is one estimator, and
every safety property this project has rests on it being right.

The supervisor is a **second, independent path from photons to a veto**. It
reads the depth image directly and answers one question:

> Is there enough *stuff* close enough in the direction I am about to move?

It does not build state, does not integrate over time, does not need a pose,
and does not care whether the map is correct. That independence is the whole
point. It catches:

- **obstacles not yet integrated** — the map is one frame behind at best, and
  after a recentre or a pose glitch it can be much worse;
- **a map that is wrong** — drift, a bad extrinsic, a stale region that scrolled
  back in as UNKNOWN and then got carved by a mismatched return;
- **the window between seeing and believing** — log-odds needs several hits
  before a cell crosses `occThresh`. A branch at 1.5 m closing at 3 m/s gets
  five frames. The supervisor needs one.

It is emphatically **not** a planner. It cannot choose a direction, cannot score,
cannot improve a path. It can only forbid.

## 2. The algorithm

One pass over the depth image, then a small 1-D array.

### 2.1 Sector grid

Divide the image into `nAz × nEl` angular sectors — not pixel blocks. A sector
is a bearing range, because a bearing is what the planner commands. With
`hfov = 87°` (D435i) and `δ_az = 3°`, that is 29 azimuth sectors. Elevation
matters less (the vehicle climbs slowly) so `nEl` can be 3–5.

Pixel → sector is a lookup table, built once from the intrinsics. No per-frame
trigonometry.

### 2.2 Per-sector occupancy fraction

For every sector, count pixels whose depth is **valid and below `d_min`**, and
divide by the number of **valid** pixels in that sector:

```
occ_frac[s] = |{p ∈ s : 0 < depth(p) < d_min}| / |{p ∈ s : depth(p) > 0}|
```

A sector is **BLOCKED** when `occ_frac[s] ≥ ε`.

Three decisions are buried in that line and each is load-bearing:

- **Fraction, not nearest.** The nearest pixel is one speckle away from vetoing
  everything. We already measured this in the map: the far-field openness term
  switched from distance-to-first-blocked to occupancy *density* precisely
  because a single-cell statistic stopped discriminating once the resolution got
  fine enough to see individual trunks (`voxel_traj.hpp`, `FarMode`). The same
  argument applies here with the same force, and a fraction is robust to the
  speckle rate the stereo model already reproduces.
- **Denominator is valid pixels, not all pixels.** A sector that is 90 % holes
  and 10 % near returns is *more* alarming than one that is 90 % far surface.
  Dividing by all pixels would make an untextured near wall look empty, which is
  the exact failure this project exists to avoid.
- **But an all-holes sector is not free.** If `|valid| / |pixels| < v_min` the
  sector is **UNKNOWN**, and unknown is treated as blocked for the purposes of
  *increasing* speed, exactly as in the map. See §3 for why that is not the same
  as vetoing.

### 2.3 Vehicle radius expansion

A blocked sector is a bearing where something is close. The vehicle is not a
point: a return at distance `d` off to the side still hits us if our radius `r`
overlaps that bearing. So every blocked sector is dilated by

```
k = ⌈ arctan(r / d) / δ_az ⌉    sectors either side
```

where `d` is the *representative distance* of the blocking pixels in that sector
— use the 10th percentile of the near depths, not the minimum, for the same
speckle reason as above. With `r = 0.6 m`, `δ_az = 3°`:

| d (m) | arctan(r/d) | k sectors |
|---|---|---|
| 0.5 | 50.2° | 17 |
| 1.0 | 31.0° | 11 |
| 2.0 | 16.7° | 6 |
| 4.0 | 8.5° | 3 |

Note the shape: a near obstacle blocks almost the whole frame, which is correct
and is the reason `d_min` cannot be large. At `d_min = 4 m` a single trunk 1 m
away closes 66° of a 87° FOV, and the supervisor becomes a stop button. The
dilation is what makes `d_min` a *tight* parameter rather than a free one.

### 2.4 The verdict

`supervisorAllows(azDeg, elDeg)` → the sector containing that bearing is not
blocked after dilation. That is the whole public surface.

## 3. Where it sits, and what it is allowed to do

Inserted in `TrajectoryPlanner::plan()`, at exactly one place: **the admissibility
test of each rollout**, alongside `sphereClear()`.

```
for each rollout point i:
    if (!sphereClear(map, ...))                    break;   // existing
    if (map.stateAt(...) != FREE)                  break;   // existing
    if (!sup.allows(bearing of point i from px,py)) break;  // NEW
```

Three consequences follow from putting it there rather than anywhere else:

- **It truncates, it does not veto the frame.** A blocked bearing shortens
  `nClear` for the primitives that go that way, which shortens `freeLen`, which
  lowers the speed budget through the existing stopping-distance solve. There is
  no new stopping mechanism and no new failure mode: the aircraft slows for the
  same reason it always slows.
- **It cannot deadlock the escape primitives.** Rearward and lateral primitives
  point at bearings outside the FOV. Bearings the camera cannot see are **not
  supervised** — the supervisor has no opinion there and must not manufacture
  one. This is the difference between "unknown is not free" (a map rule about
  space we have looked at) and "silence is not a veto" (a sensor rule about
  space we have not). Getting these two confused is how the cul-de-sac deadlock
  came back the first time.
- **It never raises anything.** No score term, no speed grant, no openness
  contribution. It is a monotone restriction on the admissible set, so any run
  it changes can only have been made slower or shorter — which makes the A/B
  trivially interpretable.

Bearings are computed in the **camera frame at the time of the depth image**,
not the world frame, and not the believed pose. That is deliberate: the whole
value is that it does not depend on the pose estimate. The rollout point is
transformed into the camera frame using the known body→camera extrinsic and the
*rollout's own* body-frame coordinates, which are pose-free by construction.

## 4. Parameters, and what each one costs

| name | proposed | what moving it does |
|---|---|---|
| `dMinM` | 2.0 | the supervised depth. Larger = earlier braking and, via §2.3, rapidly wider blocked arcs. This is the parameter to sweep first. |
| `epsFrac` | 0.06 | fraction of valid pixels that must be near. Lower = jumpier; at 320×240 with 29×3 sectors a sector holds ~880 px, so 0.06 ≈ 53 px ≈ a 7×7 patch. Below ~0.02 the speckle rate (0.4 %) starts to matter. |
| `vMinFrac` | 0.15 | below this valid fraction the sector is UNKNOWN rather than clear. |
| `nAz`, `nEl` | 29, 3 | resolution. `δ_az` enters the dilation, so these are not free either. |
| `robotR` | 0.6 | must equal `TrajParams::robotR`. Should be read from it, not duplicated. |

`epsFrac` and `dMinM` interact and must be swept **together**; the single-knob
sweeps in this project have twice produced a conclusion that a paired sweep then
reversed.

## 5. Cost

One pass over the depth image with a table lookup and two integer increments per
pixel: 76 800 pixels at 320×240. Measured comparators from this tree —
map integration is ~4 ms/step, the trajectory library ~0.55 ms — put this at
well under 0.5 ms. It is the cheapest safety mechanism available and that is a
large part of the argument for it.

Memory: one `int16` LUT the size of the depth image (150 kB at 320×240), plus
`nAz × nEl` counters.

## 6. How it gets validated

In this order, and the order matters — steps 1 and 2 are instrument checks and
this project has twice shipped a conclusion that was really an instrument fault.

1. **Synthetic frames, unit test.** Hand-built depth images with a known near
   patch: assert the correct sectors block, assert the dilation count matches
   `⌈arctan(r/d)/δ_az⌉` exactly, assert an all-holes sector reads UNKNOWN and not
   clear, assert bearings outside the FOV are unsupervised.
2. **Does it fire when it should, in sim, with the map disabled?** Run the sim
   with the supervisor as the *only* obstacle mechanism (`sphereClear` forced
   true) on the forest world. It should collide far less than nothing at all and
   far more than the map does. If it collides at the same rate as no mechanism,
   it is not firing and every number after this is noise.
3. **Paired A/B against the current stack**, lanes world, D435i geometry,
   1.5 m/s, 400 steps, 8 seeds — the same protocol as the openness sweep, so the
   numbers are comparable to the ones already in the tree. Primary metric is
   goal-closing efficiency; the secondary and more interesting one is
   **corridor lies acted upon**, since the supervisor's claim is specifically
   about catching what the map gets wrong.
4. **Adversarial arm: inject pose drift.** `--drift 0.05 --driftyaw 2`. The map
   degrades; the supervisor should not, because it never reads the pose. If the
   supervisor's advantage does not *widen* under drift, the independence claim is
   false and the component is not earning its place.
5. **Real D435i frames, offline.** Record a bag walking toward trunks and a
   wall. Assert on the recording that the supervisor blocks before the operator
   judged it too close, and — the harder direction — count how often it blocks
   in open air. A supervisor with a false-positive rate above a few percent will
   be turned off in the field, which is the same as not having it.

Step 5 is the one that decides whether the parameters in §4 survive. Everything
before it is a check that the code does what the document says.

## 7. What this does *not* do

- It does not see anything stereo does not return. An untextured wall at 1.5 m
  is invisible to this exactly as it is invisible to the map. The depth improver
  (item 2) helps at the margin by thickening partial returns; nothing helps with
  a total dropout except a different sensor modality.
- It does not replace the map. It has no memory, so it cannot know about the
  trunk that just left the FOV, which is precisely the thing that kills you
  during a turn.
- It does not make the vehicle faster, ever. If a run gets faster with the
  supervisor on, that is a bug or a seed.
