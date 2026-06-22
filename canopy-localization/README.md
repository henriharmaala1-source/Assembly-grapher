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
