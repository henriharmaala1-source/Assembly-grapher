# Canopy Horizon Localization — proof of concept

GPS-denied **absolute** positioning for a drone above the forest canopy, using
a monocular camera + a DSM (LiDAR surface model) — **with no real-world data and
no training**.

## The core idea

The LiDAR knows *geometry*; the camera captures *appearance*. The only thing
both can produce without anyone learning what a forest looks like is the
**skyline silhouette** — the canopy/sky boundary as a 1-D curve of elevation
angle vs azimuth. So we match *shape to shape*:

```
  H_lidar : ray-cast the DSM height-field            (geometry, auto-labeled)
  H_cam   : classical sky/canopy segmentation        (training-free)
  match   : align + amplitude-aware residual on the curves -> position + heading
```

Because matching lives in curve space (appearance-free), LiDAR-generated curves
are valid references for the real camera's curve — that's how the sim-to-real
gap is crossed without any real-world training.

## What's here

| file | role |
|------|------|
| `horizon/raycaster.py`     | DSM height-field horizon ray-caster (occlusion + Earth curvature) |
| `horizon/synthetic_dsm.py` | Finnish-style test DSM **and** `load_geotiff()` for real MML tiles |
| `horizon/camera.py`        | classical, training-free sky/canopy segmentation (+ test renderer) |
| `horizon/matcher.py`       | curve matcher: arc-limited search, heading + position fix |
| `horizon/dataset.py`       | mass-generate auto-labeled horizon curves |
| `demo.py`                  | end-to-end: DSM → segment → localize, saves `out/demo.png` |
| `montecarlo.py`            | accuracy distribution + confidence calibration |
| `diag.py`                  | residual-surface diagnostics |

Run: `python3 demo.py`  (needs `numpy`, `matplotlib`)

## Measured results (synthetic DSM, no real data, no training)

- **Segmentation** (classical, no model): horizon RMS **0.4–0.6°** in *clear and
  overcast* — the texture-variance cue survives low colour contrast.
- **Mass generation**: ~14 panoramas/s/core on CPU, **1.4 KB/curve**; a 100 km
  radius ≈ **~5 M curves / ~7 GB / ~hours CPU (minutes on GPU)**, auto-labeled.
- **Matcher correctness**: exact curve → **1.9 m** (true pose is the global min).
- **Single-frame fix** (45 random poses, 70 m GPS-prior error, 6° heading error):
  median **24 m**, p90 81 m; in high-structure views (lake/clearing/relief in
  frame) median **~5–14 m**.

## Honest limitations / next steps

- **Confidence is not yet calibrated.** Skyline structure (saliency) gates the
  good fixes (conf≥0.2 → 5.5 m median) but a single scalar isn't a robust abstain
  signal (rank corr ~−0.2). Needs multi-signal calibration on a large set — this
  is "Test 2" in the project plan.
- **Single frames are noisy by design.** ~tens of metres single-shot → the
  architecture fuses *sparse* horizon fixes with continuous SLAM in an EKF; that
  averaging is where ~5–10 m steady-state comes from.
- **IMU is required**, not optional: the skyline is measured in *angles*, so
  pitch/roll (and a heading prior) are load-bearing inputs.
- **Real data swap**: replace `make_synthetic_dsm()` with
  `load_geotiff(path, window=...)` on an MML DSM tile (EPSG:3067, 2 m/px). The
  ray-caster and matcher are unchanged; only the height-field source differs.

## Dataset planner app (`app/canopy_planner.py`)

A Tkinter desktop app to plan a training run: drag-select the area on a map of
Finland (or type EPSG:3067 coordinates) and it live-estimates the number of
horizon "pictures" (curves), storage, generation time, and training time.

```
python3 app/canopy_planner.py        # Tkinter ships with Python on Windows
```

- **Estimate** updates as you change the area/parameters (pure Python, no deps).
- **Calibrate on this PC** remeasures ray-cast throughput on your machine
  (needs `numpy`).
- **Generate dataset…** runs the real pipeline over the AOI from a DSM GeoTIFF
  (needs `numpy` + `rasterio`), or a synthetic DSM if you cancel the file dialog,
  writing curves + coordinates to an `.npz`.
- Throughput defaults are calibrated from `bench.py`; gen time scales with
  ray-march *positions* (one march yields all altitude curves).

`app/estimator.py` holds the pure estimation logic (`python3 app/estimator.py`
prints a self-test).

## Getting the data

**Camera calibration (`calibrate_camera.py`)** — produces `cam.json` for
`locate_video.py --calib` (intrinsics + lens distortion). Print a chessboard,
take ~15-20 photos (or a short video) at varied angles/distances:
```
python3 calibrate_camera.py --images calib/*.jpg --rows 6 --cols 9 --out cam.json
python3 calibrate_camera.py --video calib.mp4   --rows 6 --cols 9 --out cam.json
python3 calibrate_camera.py --selftest          # synthetic check, no inputs
```
(`--rows/--cols` are INNER corners.) Self-test recovers a known camera to RMS
0.03 px.

**DSM download (`download_mml.py`)** — fetches an MML DSM tile via the open WCS
(needs a free MML API key; set `MML_API_KEY`):
```
python3 download_mml.py --capabilities                 # list coverage IDs
python3 download_mml.py --coverage <id> --bbox Emin Emax Nmin Nmax --out dsm.tif
python3 download_mml.py --check tile.tif               # validate any GeoTIFF
```
BBox is EPSG:3067 metres. If the API/network is unavailable it prints manual
download routes (MML MapSite / Paituli / funet). Run it on your own machine —
this sandbox's network policy blocks MML.

## End-to-end: DSM + video -> location (`locate_video.py`)

The one-command glue. Loads a DSM GeoTIFF (MML EPSG:3067, or any projected/
geographic raster), reads a forward-looking video, extracts the skyline per
frame, and runs blind cold start to output a global position (in the DSM's CRS,
+ lat/lon if `pyproj` is installed).

```
# sanity-check the plumbing on a DSM by rendering a test clip from it:
python3 locate_video.py --dsm tile.tif --make-test-video --alt 15 --fov 65
# localize your own video against an MML DSM tile:
python3 locate_video.py --dsm mml_dsm.tif --video clip.mp4 --alt 15 --fov 65 \
    [--en-bounds Emin Emax Nmin Nmax] [--grid 80] [--range-step 5] [--speeds 30 60 90 120]
```

Verified end-to-end on a real **EPSG:3067 (MML-format) GeoTIFF** and a real
Copernicus GeoTIFF: reports the fix in the DSM's CRS + lat/lon, no GPS/IMU/VO.
Cold start localizes to about the **reference grid spacing** (`--grid`); tighten
`--grid` for a finer fix (slower) — tracking then refines. `--range-step` is the
build speed/accuracy knob (e.g. 5 m cut a 3 km-tile build from 172 s to ~50 s
with no accuracy loss, since grid spacing dominates).

Also wired into the **planner app**: the "Test on video…" button now *localizes*
(prompts for FOV / altitude / optional calibration) when you give it a DSM, or
falls back to a horizon-overlay pass without one.

**Preprocessing (`video_preproc.py`, on by default)** canonicalizes each frame so
it's comparable to the reference: lens **undistort** + FOV from `--calib`, and
**CLAHE** contrast normalization. Roll must come from the **IMU** (`--roll DEG`);
skyline-based auto-level (`--level`) is *only* safe for near-flat horizons because
it can't tell camera roll from genuine terrain slope (it broke the Lapland test
until disabled). Pitch (a constant) is absorbed by the curve's zero-meaning.
Demonstrated by `test_preproc.py`: a clip corrupted with roll + barrel distortion
+ low contrast is recovered from **2.88° -> 0.84°** skyline error.

**You must supply:** correct `--fov` (or `--calib`); `--alt` (AGL above the DSM
surface); a DSM tile covering the flight area; `--speeds` covering the real
per-frame motion; roll from IMU for tilted footage; footage from **above the
canopy looking forward** with skyline structure. The one field unknown remains
the **classical segmentation on real camera imagery** — the core sim-to-real test
this enables.

## Cold start — prior-free relocalization (`coldstart.py`)

Finds the global position with **no GPS and no prior**. A single 90° look is
massively ambiguous (≈1000 look-alikes over a 2 km area), so this fuses a short
**sequence** of looks with known relative motion (IMU heading + VO/baro
displacement) against a reference grid, collapsing the ambiguity to a unique fix.

```
python3 coldstart.py --demo        # 12-frame sequence -> 36 m start fix (IMU heading + VO motion)
python3 coldstart.py --blind       # NO IMU, NO VO: also brute-forces heading + speed
```

**Blind mode (`--blind`, `cold_start_blind`)** needs no IMU and no VO — just the
frames. Assuming roughly straight forward flight, it searches position × heading
× speed. In the demo a single look was ambiguous over the *whole* 2.2×2.2 km area
(1354 cells, heading unknown), yet the 12-frame sequence recovered the start to
**10 m** and the heading (90°) and speed (40 m/frame) **exactly**. Cost grows
with the search grid, so country-scale needs the ANN index + coarse-to-fine, but
the method is the same.

API: `build_reference(rc, bounds, spacing)` then `cold_start(ref, frames,
daz, headings, rel_xy)`. After cold start hands off an initial fix + trajectory,
the prior-based matcher (`matcher.localize`) takes over for tracking. Note: the
confidence metric is not yet calibrated, and low-saliency (flat uniform forest)
needs a longer sequence — keep accumulating frames until the peak is distinct.

## Video test (`video_test.py`)

Process a forward-looking video: extract the skyline from every frame (classical
segmenter) and, on a cadence, match it to the DSM reference to produce an
absolute fix. Writes an annotated video (detected horizon in red, DSM-expected
in cyan) and a summary report (truth vs fixes, GPS-prior error vs horizon-fix
error). Also wired to the planner's **Test on video…** button.

```
python3 video_test.py --demo                  # synthetic flythrough, then test it
python3 video_test.py --video clip.mp4 [--dsm tile.tif --fov 65]
```

Needs `numpy`, `opencv-python`, `matplotlib` (+ `rasterio` for `--dsm`). On the
synthetic GPS-denied demo the horizon fix (~70 m) bounds a degraded ~113 m
prior; occasional per-frame outliers are why the live system gates by confidence
and fuses with the EKF.

## Real-data run (`real_demo.py`)

Runs the *same* ray-caster + matcher on a **real Finnish DEM** — Copernicus
GLO-30 over the Lapland fells (Ylläs area), auto-downloaded from AWS open data.
Result: segmentation RMS 0.41°, fix error **14 m** from a 75 m-off prior on real
terrain. Caveat: Copernicus is a **30 m radar** DEM, so it resolves *terrain
relief* (fells), not canopy. Canopy-scale needs the **2 m MML LiDAR** (DSM),
which this sandbox's network policy blocks; drop a real MML tile in via
`load_geotiff()` and the code path is identical.

## Status

Proves the concept end to end. Not flight code. The on-Pi port is the same
ray-caster + segmenter + matcher in C++ (the matcher's arc-limited search is
sub-millisecond-class once the panorama is restricted to the IMU heading prior).
