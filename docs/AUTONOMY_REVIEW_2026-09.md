# Autonomy stack review, September 2026: what others do, what we measured, what changed

A critical review of the whole stack — perception, world model, estimation and
planning, both classical and learned — against published systems. It works under
this project's constraints: **no lidar, a Raspberry Pi 5 (CPU), a RealSense D435i,
ArduPilot, GNSS-denied forest and indoor flight**, and the objective set in
`CLAUDE.md` (safe travel, not goal-reaching). Jetson-class hardware is out of
bounds.

**Evidence labels:**
- **[V]** read at the source;
- **[S]** from a search-engine extract of the named page. The egress proxy blocked arxiv, ardupilot.org, IEEE and most university sites, so treat these numbers as probably right, not checked;
- **[I]** inference or arithmetic;
- **[M]** measured here, with the raw data in the repo.

Three research agents produced the literature material, about 45 sources each, on
perception and hardware, estimation and world models, and planning and learned
methods. Everything marked **[M]** was run in this repo on 2026-09-23.

---

## 1. The current stack, stated plainly

| Layer | What it is |
|---|---|
| Perception | D435i stereo depth at 848×480 plus IMU (`VoxelNavModule`). An older monocular path (MiDaS or DepthAnything, DepthNav VFH+) is still the default. A ToF path is compiled in but never constructed. The lock-on tracker is a classical cue-fusion core plus the legacy OpenCV tracker; VitTrack is desktop-only. |
| World model | 0.25 m three-state log-odds voxel map, with an honest marking range of about 3.5 m, derived from an **assumed** 0.25 px stereo noise. A far-field bearing field out to 20 m is used for scoring only. Architecture C: **no position estimate**, and a new map at every stop. |
| Planning | 210 motion primitives with a swept-sphere veto; unknown space passes the veto but earns no speed. Onboard, a straight leg is certified on its own geometry, then the mission runs move-stop-sense. In simulation the greedy `freeM` beats everything, including PPO. |
| Control and safety | ArduPilot through MAVLink RC override or attitude target. A reflex layer runs on the Pi. |

## 2. How it compares

| System | Sensing | Estimation | Map | Planner | Compute | Relevance |
|---|---|---|---|---|---|---|
| **Parrot ANAFI Ai** | 1 stereo pair on a 330° gimbal | VIO | probabilistic voxel occupancy [S] | on board | SoC | **Closest to ours.** It confirms the three-state voxel map and look-then-fly approach are industry-standard. |
| **Skydio 2 / X10** | 6 fisheye cameras, 360° | VIO | learned depth into a 3D map | classical planner; the "Deep Neural Pilot" **imitates** it [S] | TX2 / Orin | Out of reach (GPU, 360° cameras). Lessons: a published 1.3 cm thin-object floor, and learning by imitating the classical planner. |
| **DJI APAS** | stereo, 0.5–20 m | VIO + GNSS | — | — | ASIC | Sensing-speed limit ≈ range / 1.3 s, which for our 3.5 m gives **about 2.5 m/s** [I] |
| **ArduPilot d4xx_to_mavlink** | D435, one horizontal band, 72 sectors | none | none | FC BendyRuler / simple avoidance | UP² (x86); Raspberry Pi 4 "not supported" [V] | A 2D proximity fence only. We now send the same message from the same kind of data (§5.4). |
| **PX4-Avoidance** (3DVFH*) | D435 | VIO | polar histogram | reactive | NUC / TX2 | **Archived August 2024**; its interface was declared abandoned [V]. Nothing off the shelf does our planner's job. |
| **RAPPIDS** (Berkeley) | D435i + T265 | T265 VIO | none, depth-image pyramids | primitives + veto | **ODROID-XU4, ARM, 30 Hz** [S] | Proof that primitives plus a veto fit on less CPU than a Pi 5. It still had **ego-motion**. |
| **NanoMap** (MIT) | depth | noisy odometry | recent frames + relative poses, uncertainty-aware | primitives | — | The grown-up form of per-stop maps. **5% unmodelled drift: crashes in >1 of 4 flights; modelled: 2%** [S]. |
| **EGO-Planner / FUEL** (HKUST/ZJU) | D435 | **VINS-Fusion** | 0.1 m raycast grid | gradient B-spline / frontier TSP | NUC / Xavier / TX2 | Planner ~1 ms; **the enabling cost is VIO**. |
| **ETH OKVIS2-X** | stereo + IMU | VI-SLAM | submaps anchored to keyframes | — | onboard | Forest flight up to 3 m/s, "not a single collision" [S]. |
| **UZH agile flight** (2021) | D435 + T265 | T265 VIO | none | learned (DAgger from a privileged expert) | TX2 | 60% success at 10 m/s where baselines failed. **Less safe than classical planners in AvoidBench's independent test** [S]. |
| **NTNU ORACLE / seVAE** | depth | VIO | none | **learned collision probability per primitive** + library | Xavier NX (11 + 29 ms) | The learned design that fits ours. The 2025 boreal-forest follow-up completed 12 of 15 very-dense runs [S]. |
| **Nano drones** (PULP-DroNet, NanoFlowNet, multizone ToF) | camera or 8×8 ToF | — | none | reactive | milliwatts | Their most robust result is **metric ToF plus a simple rule** (100%, 0.3% of compute) [S], echoing freeM beating PPO here. |

**The one sentence that summarises the literature:**
- Every system that navigates unknown terrain without GNSS has **ego-motion** (VIO, T265 or flow).
- The drift-tolerant ones drop **global consistency**, never the motion estimate.
- **Architecture C dropped both.** That is its defining weakness, and §4 measures what it costs.

## 3. Learned versus classical, honestly

**What published learned systems that worked had in common** (planning report; [S]/[V] per source):
- **Supervision, not reward.** They imitated a privileged expert with DAgger (UZH 2021, PILOT 2026), used analytic gradients (DiffPhysDrone, which flew on a $21 ARM board), or learned collision probabilities (Kahn 2018, CAD2RL, ORACLE).
  - Kahn et al. found collision-probability targets "extremely sample efficient, stable", unlike bootstrapped values.
- **Collision-focused learned representations:** seVAE, DCE, Depth Transfer.
- **A classical safety shield outside the network:** NavRL's velocity-obstacle shield, CBF filters, FGI's action filter.
- **Independent benchmarks agree:**
  - AvoidBench: motion-primitive and EGO planners are *safer* than learned Agile Autonomy.
  - FlightBench: optimisation planners keep "a competitive edge" in success rate and compute.

**Why our PPO lost** [I, consistent with the above]:
- A greedy veto-plus-openness rule already solves most of the objective in one step. PPO had to rediscover it from a scalar return.
- At 0.15–3M steps it had 3–4 orders of magnitude less experience than DD-PPO's 2.5 billion.
- Sampled evaluation handed it exploration noise its classical competitor never got.

**Statistics** (Henderson 2018; Colas 2018; Agarwal 2021 [V/S]):
- With our between-seed SD (about 760 on net × cells), a two-sample test needs about **9 seeds per arm to detect Δ = 1000, and about 36 to detect Δ = 500** [I].
- `CLAUDE.md`'s "≥ 4 seeds" is enough only for effects the size of freeM against PPO.

## 4. What we measured [M]

### 4.1 The privileged ceiling: is the planner or the map the bottleneck?

`kestrel bench --oracle` overwrites the planner's fine map with the true world
within 8 m, leaving everything else (veto, primitives, speed rule) unchanged.
Setup: freeM, held-out maze, seeds 101–112, 3000 steps, paired by seed.
Raw: `nav-sim/docs/oracle_maze_3000.txt`.

| | freeM | freeM + oracle map | Paired difference (± se) |
|---|---|---|---|
| Perfect depth: net × cells | 4418 | 17654 | +13236 ± 6800 |
| Perfect depth: travel | 225.2 m | 253.6 m | +28.4 ± 10.0 m |
| Perfect depth: net | 27.5 m | 73.4 m | +45.9 ± 25.0 m |
| Stereo: net × cells | 1679 | 17654 | +15975 ± 7000 |
| Stereo: travel | 197.4 m | 253.6 m | +56.2 ± 18.1 m |

- **The planner is limited by what the map knows, not by how it decides.** Better map knowledge is worth up to about 4× on the headline metric.
- The standard errors are wide, since the net-displacement tails are heavy, but every column moves the same way.
- **This decides the learning question.** A smarter policy on the same map has little to gain (freeM already beats PPO). The value is in **range** (seeing further honestly) and **memory** (not forgetting at every stop).

### 4.2 Architecture C's assumption: the hover is stationary

`test_voxel_nav` closed loop (24 m room, 22 pillars, 150 s, 4 worlds per arm). An unmeasured
Ornstein-Uhlenbeck hover drift is added (2 s correlation), which the module does not see. Raw:
`onboard/docs/arch_sweep_2026-09-23.txt`.

| Hover drift (1σ per axis) | Collisions, perfect depth | Collisions, stereo | Of which during hover |
|---|---|---|---|
| 0 | 0 | 0 | — |
| 0.05 m/s | 1 | 3 | all |
| 0.1 m/s | 3 | 3 | all |
| 0.2 m/s | 15 | 16 | 29 of 31 |

- **The certified legs are fine. The aircraft slides into a pillar while it thinks.**
- The research estimates 0.1–0.5 m/s for tilt-only hover drift under canopy, more in wind, and about 5–10 cm (1σ) for flow-aided LOITER.
- **Without a horizontal velocity source, architecture C is not safe to fly.**

### 4.3 Architecture B: keep one map, placed by odometry, and hold position on it

The FC holds position on the estimate, so drift becomes estimator error. Odometry error is a **systematic bias**, a fraction of the distance moved. (A first, white-noise model averaged away to about 0.25% and flattered B. It was discarded, and the B rows were re-run.)

Stereo means, 4 worlds:

| Arm | Travel | Cells | Collisions | Stuck | Final odometry error |
|---|---|---|---|---|---|
| C (per-stop maps, no drift) | 38.5 m | 38 | 0 | 2 | — |
| B, 1% drift | 61.1 m | 53 | 0 | 0 | 0.5 m |
| B + map during legs, 1% | 60.2 m | 63 | 0 | 0 | 0.5 m |
| B + map during legs, 3% | 63.9 m | 73 | 0 | 0 | 1.7 m |
| B + map during legs, 10% | 57.0 m | 57 | 0 | 1 | 5.0 m |

- **At VIO- or flow-class drift (1–3%), one persistent map nearly doubles stereo coverage and travel** against per-stop maps, with no collisions.
- On perfect depth, B at 1% roughly equals C. The gain comes from accumulating sparse, noisy stereo.
- At 10% drift, map contamination costs coverage and causes getting stuck. The safety margin erodes (minimum clearance 0.30 m on perfect depth) but held.
- **That is exactly the regime NanoMap-style bounded memory exists for,** and it's the next experiment.
- n = 4 per arm: indicative, not resolved (see §3).

### 4.5 The same, with a real VIO instead of a drift model (added 2026-09-24)

§4.3 modelled odometry as a bias. `navcore/vio.hpp` (`DepthVio`) is now an actual estimator: keyframe VO on the D435i's left IR image, which is registered with depth, so no triangulation and no scale; IMU roll/pitch locked; 4-DOF robust solve; LK seeded at predicted pixels on high-passed images. Run in the same closed loop on the same four worlds, stereo depth:

| Arm | Travel | Cells | Collisions | Stuck | VIO error at end |
|---|---|---|---|---|---|
| C (per-stop maps, no drift) | 38.5 m | 38 | 0 | 2 | — |
| B, placed by VIO | 58.4 m | 48 | 0 | 0 | 1.3–5.8% |
| B + map during legs, VIO | 59.9 m | 64 | 0 | 1 | 0.7–5.7% |

- **The real estimator lands where §4.3's 1–3% arm assumed**: 0 collisions in 16 runs (truth and stereo), nearest approach 0.53 m, 18 of ~46,000 frames lost.
- Standalone (`onboard/test/test_vio.cpp`, known trajectories, IMU with 0.5°/s gyro bias): 0.6–2.8% drift at 424×240, 0.1–1.3% at 848×480, including 3 m/s at 15 Hz. A blank wall reads LOST on stereo depth, never as a confident wrong step.
- Three things had to be found to get there, each measured: (1) with perfect depth, corners on pillar silhouettes took the far side's depth — reject depth discontinuities; (2) projector falloff moves with the camera and dominates the coarse pyramid levels, pulling LK to zero motion — high-pass first (seeded tracks within 2 px at 0.2 m/frame: 17/139 → 135/139); (3) a 6-DOF PnP left free to trade tilt against translation produced 10¹⁴ m solutions — lock tilt to the IMU and gate the jump.
- **Simulation only.** Real IR brings auto-exposure, blur and the projector; the Pi 5's cost and the latency are unmeasured.

### 4.6 A ready-made SLAM instead: ORB-SLAM3 on the D435i's IR pair (added 2026-09-24)

The survey of what others run on a D435i (VINS-Fusion, Basalt, OpenVINS, ORB-SLAM3, RTAB-Map) found ORB-SLAM3 the one with an official D435i stereo-inertial example. It is now integrated as `kestrel-orbslam` (`onboard/orbslam/`): a separate process, so its GPLv3 stays out of `kestrel`, fed the stereo IR pair over a local socket. Its poses go where DepthVio's go.

- **Rendered stereo IR, 848×480:** a straight flight, a square with turns in place, and a hover–leg–hover all tracked every frame, with 0.2–0.4% drift (1.8 cm on the hover). About 20 ms/frame on a desktop.
- **In the closed loop** (same four worlds, stereo depth, CPU-contended):
  - With mapping during legs it covered 57 m and 59 cells, against the per-stop baseline's 38.5 m and 38.
  - There were no collisions.
  - Final error was 0.1–1.5 m.
- **Losses happen in fast turns in place.** A jump guard was needed: before it, 3 of 8 runs ended 10–19 m off after a loss moved the SLAM frame under the estimate. `mission.max_yaw_stick` caps the turn rate.
- 424×240 is too coarse for it: ORB-SLAM3's stereo start wants more than 500 features with depth.

### 4.4 Found while doing this

- **The GNSS-denied mission could never fly a leg.**
  - The Pi-side estimator initialises from a GPS fix only, so `estValid` stayed false and the mission sat in `SETTLE(no-est)`.
  - Fixed (§5.3). With no horizontal source it still hovers, but now as a stated, safe failure.
- **The stereo noise behind the 3.5 m range was assumed, never measured.**
  - Intel quotes 0.08–0.11 px with the projector. At 0.1 px the honest range is about 5.5 m [I].
  - `onboard/tools/d435i_probe.py` already measures it. Nothing consumed the measurement until now (§5.5).
- **No librealsense post-processing is used** [M, by grep]. That is correct: its default hole-filling copies the *farthest* neighbour, turning unknown into free [V].
- **`onboard/docs/bom.md` describes a different aircraft:** a 7″ analog FPV build whose committed obstacle sensor is a VL53L5CX ToF, with "No stereo camera pair". It must be reconciled with the D435i stack before anything is bought.
- **Three librealsense enum values were wrong, and each failed silently** (found 2026-09-24, checked against v2.55.1 headers). The emitter option was 12, which is `VISUAL_PRESET`. The depth-sensor extension was 12, which is `DEPTH_FRAME`, so no sensor ever matched: the emitter was never set and the depth scale was never read. The 0.001 default happens to be the D435i's. The USB-type field was 12, which is `FIRMWARE_UPDATE_ID`.
- **`flow_odometry_check` passes partly on aliasing.** The synthetic IR point-samples a texture finer than a pixel at range. Band-limit it and the flow estimator's fixed variance floor rejects most patches (2–4 of 19 frames solved). The realistic render is opt-in (`CamParams::irBandLimit`) and the VIO tests use it. The flow finding is recorded, not fixed.
- **D435/D435i on a Raspberry Pi is unsupported by both ArduPilot and Intel.** librealsense issues report a D435f timing out on a Pi 5 and ARM64 frame drops [V/S]. The USB link has never been proven on this hardware.

## 5. What was implemented in this round

1. **Hover drift and systematic odometry models** in `test_voxel_nav`: `VOXTEST_DRIFT`, `VOXTEST_ODOM`, `VOXTEST_ODOM_MOVING`, `VOXTEST_ODOM_NOHOLD`. Collisions are counted and attributed to hover or leg.
2. **Architecture B in `VoxelNavModule`:** `persistMap` and `integrateMoving` params, with config keys `nav.vox_persist_map` and `nav.vox_integrate_moving`. Off by default, because it needs an ego-motion source.
3. **FC local position as the leg displacement** (`fc_odometry.hpp`):
   - `LOCAL_POSITION_NED` added to the codec, pinned against a pymavlink golden frame, and requested at 10 Hz.
   - Used **only** when EKF3 vouches for it (`POS_HORIZ_REL` set, `CONST_POS_MODE` clear).
   - It is read, never fed back: one filter, not two in series.
4. **A second, independent avoidance path:**
   - `obstacleDistanceFromFrame` turns every depth frame into 72 horizontal sectors. It uses the 12th-nearest return per sector (a lone speckle is not an obstacle), reports unknown where nothing returned, and needs attitude only, so it stays valid **while moving**.
   - `FcLink` sends it as `OBSTACLE_DISTANCE` at ≤ 10 Hz, only while fresh, and never repeats a stale set. It is tested.
   - The MAVLink encoder existed and was tested but was never connected.
5. **Measured stereo noise as a parameter:** `NavPipelineParams::subpixelPx`, config key `nav.vox_subpixel_px`. The default stays 0.25 until `d435i_probe.py` measures the real unit.
6. **The privileged-ceiling tool:** `VoxelMap::imprint`, `EnvConfig::oracleMap`, `kestrel bench --oracle`, and a GUI toggle.
7. **Docs:** `onboard/docs/gnss-denied-setup.md` §9 (flow is required for `--voxel`, with parameters) and §10 (the `OBSTACLE_DISTANCE` path, with parameters and one unverified case).
8. **VIO** (2026-09-24, §4.5): `navcore/vio.hpp`, run by `VoxelNavModule` on dark IR frames.
   - `--voxel-vio` / `nav.vox_vio` makes it the mission's displacement when neither the Pi estimate nor FC flow exists.
   - `--voxel-vio-fc` / `nav.vox_vio_to_fc` also feeds it to EKF3 as ExternalNav (`VISION_POSITION_ESTIMATE` with `reset_counter`, golden frame from pymavlink), so the FC can hold position on it.
   - The emitter strobes (`EMITTER_ON_OFF`) with per-frame metadata. With no metadata, VIO turns itself off rather than track the dots.
   - Parameters and caveats: `gnss-denied-setup.md` §11.
9. **ORB-SLAM3** (2026-09-24, §4.6): `onboard/orbslam/` (pinned, headless build script, bridge), `--voxel-slam` / `nav.vox_slam*`.
   - Onboard streams the right IR imager, the pair's device time, the device's own baseline, and raw IMU.
   - Dark frames are chosen **from the image** (`navcore/emitter_gate.hpp`), because librealsense's strobe metadata has been reported inverted.
   - A jump guard and velocity-carried re-anchoring handle map changes.

## 6. Recommendations, ranked

1. **Fit an optical-flow sensor with an 8 m rangefinder (e.g. MicoAir MTF-01) and set `EK3_SRC1_VELXY=5`** — or, now that it exists, **bench-test the D435i's own VIO** (`--voxel-vio-fc`, §4.5) as the ExternalNav source. Flow is still the lower-risk first step (no Pi CPU, ArduPilot-native, flown by many); VIO needs no extra hardware but has only been simulated, and the Pi's frame metadata (RSUSB backend) is a prerequisite. Having both, as EKF3 source sets 1 and 2, is the robust end state.
   - About $30–60, no Pi CPU, ArduPilot-native.
   - It makes the stationary-hover assumption true (§4.2), gives the mission its displacement (§4.4), and enables architecture B (§4.3).
   - Test it over forest litter in propeller wash before trusting it.
2. **Reconcile the hardware plan and prove the D435i → Pi 5 USB link under vibration and flight power.** Until then every number above is simulation.
3. **Measure the unit's stereo noise** (`d435i_probe.py`) and set `nav.vox_subpixel_px`. If it is near Intel's 0.1 px, the honest range grows about 1.6×, which is the "range" half of §4.1's headroom.
4. **Turn on the ArduPilot proximity feed** (`PRX1_TYPE=2`, `AVOID_ENABLE=7`) and fly the RC-override uplink in LOITER or ALT_HOLD, where its behaviour is documented.
5. **Next experiment: NanoMap-style bounded memory.** Keep the last N stop maps with relative pose and inflate clearance by their accumulated σ, then re-run §4.3 at 3–10% drift. This targets the contamination that architecture B shows at high drift.
6. **Learning, if at all: a predictor, not a policy** (ORACLE / Kahn pattern).
   - Predict per-primitive true free length from depth, supervised from simulator truth, and let freeM act on it for **direction only**. Speed stays on confirmed-free space.
   - The oracle result says there is up to 4× on the table, and it lives in *knowledge beyond the honest range*, which is exactly what such a predictor would learn.
   - Use at least 9 seeds per arm, deterministic evaluation, and IQM with bootstrap confidence intervals.
7. **Hardware worth evaluating, in order:**
   1. **Flow sensor** (above).
   2. **OAK-D Pro (not W):** 7.5 cm baseline at 1280×800, about 3× lower depth noise, about 1.75× the honest range [I]. Luxonis figures still need confirming.
   3. **Hailo-8L:** for the tracker's learned cue only. Expect about half the model-zoo fps on the Pi 5's PCIe x1 [S].
   4. **Skip:** Coral (unmaintained), Pi-native SGBM stereo (eats the CPU), event cameras (see nothing while hovering), and zero-shot metric monocular depth (AbsRel 53–98% on aerial views [S]).

## Key sources

- ArduPilot RealSense and avoidance docs (wiki source on GitHub): https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-realsense-depth-camera.rst
- PX4-Avoidance (archived): https://github.com/PX4/PX4-Avoidance
- RAPPIDS: https://arxiv.org/abs/2003.01245
- NanoMap: https://arxiv.org/abs/1802.09076
- EGO-Planner: https://github.com/ZJU-FAST-Lab/ego-planner
- FUEL: https://github.com/HKUST-Aerial-Robotics/FUEL
- Learning High-Speed Flight in the Wild: https://github.com/uzh-rpg/agile_autonomy
- ORACLE: https://arxiv.org/abs/2201.03254
- seVAE-ORACLE: https://arxiv.org/abs/2307.11522
- Kahn et al. 2018: https://arxiv.org/abs/1709.10489
- AvoidBench: https://arxiv.org/abs/2301.07430
- FlightBench: https://arxiv.org/abs/2406.05687
- DiffPhysDrone: https://arxiv.org/abs/2407.10648
- OKVIS2-X: https://github.com/ethz-mrl/OKVIS2-X
- OpenVINS on Pi 5: https://github.com/Eryk-Mozdzen/open_vins_example
- ArduPilot non-GPS position estimation: https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/mavlink-nongps-position-estimation.rst
- MTF-01 manual: https://github.com/micoair/MTF-01_USER_MANUAL
- librealsense post-processing filters: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
- Deep RL that Matters: https://arxiv.org/abs/1709.06560
- Statistical precipice: https://arxiv.org/abs/2108.13264
- Skydio Deep Neural Pilot: https://www.skydio.com/blog/deep-neural-pilot-skydio-2
- Hailo model zoo (Hailo-8L depth): https://github.com/hailo-ai/hailo_model_zoo
