# NOTES — working memory, not a document

Terse running notes: decisions taken, why, and what is still unknown. Not for
showing anyone. Append, date entries, delete what goes stale.

---

## 2026-08-06 — staging decision + feasibility verdict

### Where we are

Ordering stereo cameras. Integrating the rest onto the frame. Validating the
Pi↔FC connection. No autonomy work in flight until that is solid.

### Agreed staging

1. Airframe + Pi integration, connection validated
2. Lock-on (tracker from a hover)
3. Basic flight modes + autonomy
4. Stereo autonomy — **last**

Rationale: every stage flies. Each one builds the telemetry / failsafe /
mode-switch / black-box infrastructure that stage 4 needs anyway, but where a
bug is an annoyance rather than a tree. FGI's published work is all-or-nothing
60 m autonomous runs; that is the wrong shape for a one-person team.

### Feasibility verdict (this is what the paper review was for)

Risk is **not uniform** across the four stages.

* Stages 1–3: **not research.** Hover tracking is a solved CPU problem and
  `lock_tracker.*` already exists. iNAV + companion RC override is well-trodden.
  Risk is integration time and failsafe discipline, not feasibility.
* Stage 4: the only real question — and **not binary**.

Key evidence, Karjalainen 2501.12073, Evo-medium:

> 650 trees/ha, **7/7 successful**, 1 m/s, RealSense D435 with the IR emitter
> **disabled** (passive), EGO-Planner-v2, 1153 g.

Same stack at 2000 trees/ha: 8/9 successful but **0/9 smooth** — every flight
needed an emergency stop, all caused by late detection of thin dry spruce
branches. DeFoP with an Orin NX + active stereo is still only 12/15 in the hard
stand.

**So the expected outcome of stage 4 is a density and a speed at which it
works, not success/failure.** "0.6 m/s in medium boreal forest, degrades in
dense" is a real capability. Plan for that, not for a binary.

Nothing we are attempting is impossible. What is unproven is the *budget*
variant, not the task.

### The two load-bearing unknowns

Everything else is arithmetic already done (~16 ms planner + <10 ms VO + depth
inside a 100 ms budget).

1. **Passive stereo on bark.** DeFoP projects IR. Karjalainen disabled it and
   their honest depth was ~3 m with late branch detection as the failure mode.
   Nobody in this literature demonstrated passive stereo carrying the job. Our
   silhouette model (`depth_camera.hpp`: `edgeBoost 0.75`, `edgeDepthM 0.8`)
   says it can — calibrated on **one screenshot**.
2. **State estimation on a Pi 5 CPU.** VINS-Fusion will not fit alongside
   everything else. SMF-VO (2511.09072) claims <10 ms/frame on exactly our
   hardware; **unread** — arXiv is 403 through this proxy. Slot already exists:
   `StateEstimator::updateVisionVelocity()`, sized `sigmaVoVel = 0.25` m/s.

### Highest-value experiment available — do it while parts ship

**Stereo camera on a tripod, in a forest. No drone, no flight, no integration.**

Point at spruce trunks at 2 / 5 / 8 / 12 m, in canopy shade and against bright
sky. Record raw stereo + the depth output. Look at *where depth returns and
where it is black*.

* If trunks come back as black columns with one lit edge → our silhouette model
  is right and the stack has a chance.
* If trunks vanish entirely by 6 m → we learn it now, before three more layers
  are built on top of it.

This answers unknown #1 for the cost of an afternoon and does not compete with
the frame work. Also gives the first real-footage input for the depth model,
which is currently fitted to a single screenshot.

Secondary while there: check whether the emitter helps or hurts. The two FGI
papers **contradict each other** on this (DeFoP on, Karjalainen off because the
projected dots move with the camera and corrupt VIO). If we ever want active
stereo we cannot use one imager for both depth and VO without resolving it.

**Correction, 2026-08-11: "the emitter barely matters" was too strong**, and it
was repeated several times on the strength of one true condition — bright open
daylight. The projector's value is a function of AMBIENT IR and RANGE, not a
constant:

* bright open daylight, 5 m+ — negligible, as claimed
* **closed canopy, dusk, autumn, indoors — potentially decisive.** Sunlight is
  what drowns a ~1 W pattern; a boreal stand under closed canopy in October has
  little of it. That is not an edge case for us, it is a large part of the
  intended operating envelope.

So the emitter is a real operating-point decision with a real conflict attached
(it fights VIO on the same imagers), not a footnote. And `depth_camera.cpp`
modelled neither ambient light nor the projector: `texThresh` was a fixed gate,
so every number in this tree was implicitly "passive stereo in adequate light".

**Now modelled** (`emitterOn`, `ambientIR`, `emitterRangeM`, `emitterTex`; off
by default so prior numbers stand). The projector enters as a FLOOR under the
scene's own texture, because putting features on a featureless surface is
exactly what `texThresh` gates on — a richly textured trunk gains nothing, a
smooth shadowed one gains everything, and that asymmetry is the real behaviour.
Falloff is `1/(1 + (r/emitterRangeM)^2)`, so `emitterRangeM` is a half-strength
range rather than a cutoff. `--emitter` / `--ambient` on `voxel_sim`.

**First measurement, and it does NOT settle anything.** Low-texture forest
(trunkTex 0.20), whole-frame valid fraction:

    passive                    50.6 %
    emitter, ambient 1.0       50.6 %
    emitter, ambient 0.2       50.6 %
    emitter, ambient 0.0       51.3 %

Nearly flat — but the metric is wrong, not the projector. Whole-frame valid
fraction is dominated by ground and canopy, which already clear the texture gate
with or without help. The projector only changes MARGINAL surfaces within a few
metres, which is a small pixel count and the entire safety-relevant one: those
are the trunks about to be hit. A frame-wide average is exactly the statistic
that cannot see it.

**The measurement that would settle it is a collision sweep** at low trunkTex
with the emitter on and off — the same shape as the trunkTex tolerance curve in
`voxel_traj.hpp`, which found 4/4 collisions at 0.15 and 0/4 at 0.25. If the
projector moves that threshold, it is load-bearing for dim conditions. Not yet
run.

---

## Lean-out — "new kestrel OS"

Not a rewrite. The spine already exists in `onboard/`: two-tier threading,
`WorldModel` blackboard, mode arbiter, FC link, black box, realtime, SITL
suite. The lean-out is **making `nav-sim/`'s proven pieces into modules that
plug into that spine, and deleting the rest.**

Note: `nav-sim/` is not even listed in AGENTS.md §0. That table is stale.

### Carry forward — validated, load-bearing

* Three-state occupancy, `unknown ≠ free`. Measured: false-free 7.814 % → 0.003 %
  after the carve guard.
* `Z_max = √(cell·f·B/σ_d)` with the ~25 % derate, and `maxIntegM`. Carving is a
  separate decision from marking occupancy.
* Body-frame precomputed trajectory library (`voxel_traj.*`). Independently
  arrived at by DeFoP (256 primitives, same construction).
* Stopping-distance speed budget against **confirmed-free** length only.
* Exhaustive swept-volume clearance test (the sampled version failed 3×).
* Escape primitives — only defensible *because* there is a map behind us.
* The measurement discipline itself: seeded runs, ablation tables, ideas
  disabled with their numbers attached.
* `onboard/`: MSP backend, scheduler, black box, realtime, Kalman + VO hook,
  nav_map, SITL.

### Delete — keep the numbers, drop the code

* `voxel_planner.*` histogram planner — superseded by the library.
* OMPL dependency — heavy; #20's answer is topological memory, not RRT.
* `fieldEma`, `switchMargin`, `revPenalty`, `coreFrac` — all measured
  neutral-or-harmful. Tables move to a doc, code goes.

### Reframe carried over from the review

The map's job is **memory, not foresight**. Not "see 25 m ahead" — nobody
flying successfully has that (their honest depth is 2–3 m). It is "remember
where you have been so you do not re-enter a pocket you already failed out of."
That turns #20 from a perception-range problem into cheap topological memory,
and it attacks the one bug **both** published systems admit they still have
(DeFoP: "stop and rotate in place"; escapes are emergent, not planned).

---

## Do not

* More planner tuning. Two field-proven systems converged on what we have.
* More sim polish before real footage exists. The depth model is fitted to one
  screenshot; sim numbers cannot get more honest than that input.
* Chase the 25 m horizon.
* Adopt the learned half of DeFoP (CPN / autoencoder / ensemble+UT). Needs a
  GPU, a training pipeline, and a simulator good enough to train in — and their
  own geometric supervisor exists *because* the learned part can't be trusted.
  We already have the supervisor; the learned part is the expensive half.

## Known sim↔real gaps — real, deliberately deferred

Ranked by how much each would move our numbers.

1. **Thin branches do not exist in `voxel_world.cpp`.** Stems, scrub, canopy —
   no twigs. Both papers name late thin-branch detection as *the* cause of real
   failure. We are not simulating the failure that ends real flights.
2. No state estimation in the loop (default runs have perfect pose). Real
   number: ATE 0.33–0.50 m over 34–42 m, systematic scale shrinkage.
3. Plant circularity — `tau = 0.35` both predicts and produces motion. See
   `nav-sim/docs/CONTROL_PLAN.md`. Bounded, but "flyable by construction" is an
   assumption not a result.
4. Idealised timing: fixed-dt lockstep, zero sense→act latency.
5. Depth model fitted to one screenshot; no auto-exposure, backlight, canopy
   gloom.
6. We score any contact as a collision; they allow "minor touches to thin
   vegetation." Keep ours — but the comparison is not like-for-like.

## 2026-08-06 — stereo camera: Waveshare IMX296 M12 mono

1456x1088, 3.45 um pixels, global shutter, M12 (lens bought separately).
Back-of-board pads: `XTR+ XTR- XVS XHS MAS 3V3`. (Read as XYS; it is XVS —
Sony naming, V for vertical.)

| pad | what | direction |
|---|---|---|
| XVS | frame sync | output in master, **input in slave** |
| XHS | line sync | output in master, **input in slave** |
| MAS | XMASTER strap, master vs slave | **latched at reset — a solder decision** |
| XTR+/- | external trigger in | input |
| 3V3 | supply pad | not a signal reference |

### DANGER — 1.8 V logic

**XVS, XHS, XTR and XMASTER are 1.8 V. 3.3 V on any of them likely destroys
the sensor.** Do not drive from a Pi GPIO directly. Divider commonly cited:
1.5k series + 1.8k to GND. Do NOT assume the `3V3` pad is the drive level.
Camera-to-camera XVS/XHS is fine unshifted — both ends are already 1.8 V.

Order **three** boards. A 3.3 V slip kills one silently.

Reported working startup order: master first, wait >= 2 s, then slave.

### Why the pads matter — the number that decides it

Exposure time skew becomes disparity error. The ROTATIONAL term is
**independent of range**, which is what makes it lethal:

    delta_d = f * omega * delta_t

f ~ 1159 px (4 mm lens). Planner commands maxYawRate = 100 deg/s = 1.745 rad/s.
Our matching-noise budget is sigma_d = 0.25 px.

    skew      disparity err   vs budget
    1 ms          2.0 px       8x  -- ruinous
    100 us        0.20 px      marginal
    10 us         0.02 px      negligible

2 px at 5 m reads as 4.66 m; at 8 m reads as 7.16 m (0.84 m, >3 voxels) and
carves free space through the obstacle.

**Yaw rate is highest exactly when avoiding something. Unsynced stereo fails
hardest at the moment it matters most.** Software timestamp alignment buys
milliseconds; these pads buy sub-microsecond. That is the whole argument.

### Lens sets Z_max — choose deliberately

`Z_max = sqrt(cell*f*B/sigma_d)`, B = 0.12 m, cell = 0.25 m, sigma_d = 0.25 px,
25 % derate (measured):

    lens      f(px)   HFOV    honest Z_max
    ~1.6 mm    450    130 deg    5.5 m
    2.8 mm     812     83 deg    7.4 m
    4 mm      1159     64 deg    8.8 m   <- take this
    6 mm      1739     46 deg   10.8 m

4 mm: 64 deg is near the 70 deg the sim assumes, and 2x binned (f ~ 580) it
still gives ~6.3 m — well past the D435i's 2-3 m. The wide 130 deg option
throws away most of the range advantage that justified this sensor.

### Calibration vs stability — 26 um is NOT a machining tolerance

Corrects an earlier framing. Relative rotation eps between the cameras gives
disparity error `f*eps`; at f = 1159 px, 0.25 px costs eps = 2.2e-4 rad
(0.012 deg) = **26 um differential over a 12 cm bar**. That is a **stability**
budget, not a build tolerance.

**Calibration absorbs any constant misalignment.** Build limit is ~1-2 deg, and
only because rectifying a badly rotated pair crops the usable overlap. Mount it
straight by eye, calibrate, done.

**The three rotational DOF are not equal:**

    drift   effect                          self-correctable?
    roll    vertical shift varying with x   YES
    pitch   uniform vertical shift          YES
    yaw     horizontal shift ~ f*eps        NO

Roll/pitch break the epipolar constraint, so **residual vertical disparity is a
direct target-free measurement of both** — computable from ordinary matches in
any scene (this is what RealSense's self-calibration does). Yaw shifts points
horizontally, indistinguishable from everything being closer/further; epipolar
geometry cannot see it. Needs external scale: known-size target, ground plane,
or stereo-vs-IMU-scaled-SfM disagreement.

**Which drift sources bite:**

* vibration — mostly benign. Both cameras expose simultaneously, so flex at
  exposure is a RANDOM per-frame disparity error. Random -> noise -> log-odds
  averages it out. Raises effective sigma_d, does not bias the map.
* baseline length change — negligible. Al over 12 cm across 20 K is ~55 um,
  but that is a change in B, i.e. a 0.05 % scale error.
* **asymmetric thermal + mechanical creep — the killers.** Slow, systematic,
  and systematic error is exactly what a probabilistic map cannot average away.

Design consequence is not exotic material: **symmetric mount, thermally
uniform, no heat source (ESC) next to one camera, cables strain-relieved so
they cannot preload one side.**

**Failure mode is graceful.** A yaw drift of delta px is a constant disparity
offset, `Z' = fB/(d+delta)`:

    true 3 m  ->  3.07 m   (7 cm)
    true 8 m  ->  8.48 m   (48 cm)

It degrades exactly the long range the wide baseline bought and barely touches
the near field. A drifted rig behaves like a rig with lower Z_max — which is
the condition `maxIntegM` and the carve guard already defend against. Risk is
"lose the range advantage", not "fly into a tree".

**Build:** symmetric mount; target calibration (Kalibr/OpenCV) after assembly
and after any hard landing; **log mean residual vertical disparity every
frame** — free, direct measurement of roll/pitch drift, and the early warning
to redo the full calibration including the yaw you cannot see.

This moves mechanical risk DOWN the list. CPU stereo cost is now the most
likely killer of this route.

### Module comparison — every vendor "range" priced on our own formula

Vendor range figures are disparity-quantisation numbers, not usable-depth
numbers. OAK-D Lite's "up to 20 m": f = 432 px, B = 75 mm, so f*B = 32.4 px*m.
At 20 m the disparity is 1.6 px and `dZ = Z^2*sigma/(f*B)` = **+/-3.1 m** against
our 0.25 m voxels. Honest `Z_max = sqrt(cell*f*B/sigma)*0.75` = **4.3 m**.

    module                        base   depth res   f_px  Z_max   CPU   IMU    wt
    D435i @480x270 (DeFoP cfg)    50mm    480x270     253   2.5m     0   yes   72g
    D435i @848x480                50mm    848x480     447   3.5m     0   yes   72g
    OAK-D Lite                    75mm    640x480     432   4.3m     0   NONE  61g
    D455 @848x480                 95mm    848x480     447   4.9m     0   yes  110g
    D455 @1280x720                95mm   1280x720     674   6.0m     0   yes  110g
    IMX296 pair 4mm @728x544     120mm    728x544     580 6.3-8.6m 13-24ms no ~63g
    OAK-D LR                     150mm   ~1280x800   ~985  ~9m       0   yes ~100g

OAK-D LR's lens FOV and weight are ESTIMATES — verify before ordering.

**Three consequences:**

1. **OAK-D Lite is not the upgrade it looks like.** 4.3 m, and confirmed **no
   IMU**. With state estimation now the project's top risk, a depth camera
   without an IMU is the wrong trade. Roughly a D435i-plus.
2. **OAK-D LR is genuinely competitive with the custom build, maybe better.**
   Bigger baseline than ours, has an IMU, factory-rigid (no 26 um mount
   problem, no calibration ritual), and **zero Pi CPU for depth** — which is
   worth exactly the 13-24 ms stereo_bench measured.
3. **D455 should have been on the list earlier.** 95 mm baseline is nearly
   double the D435's: ~6 m at 720p, IMU, 0 host CPU. A strictly better version
   of the camera Karjalainen got 7/7 with.

**OAK-D Pro** carries an IR laser dot projector — active stereo, the DeFoP
configuration, in a rigid factory-calibrated housing. That is the direct answer
if the tripod trip says passive stereo cannot see spruce bark.

Caveat on OAK: DepthAI is a host library dependency and USB transfer adds
latency we do not control. Raw stereo pairs are still obtainable, so VO is not
foreclosed.

### NPU route — settled, do not re-litigate

Cost claim is CORRECT: IMX296 + Hailo AI Kit is cheaper than a ready-made
long-range stereo camera. But the NPU cannot do the job it would be bought for.

**Checked the Hailo-8L model zoo. Its entire depth-estimation section is two
models, both MONOCULAR:**

    fast_depth    224x224    1090 FPS   RMSE 0.61
    scdepthv3     256x320     145 FPS   RMSE 0.48

Single-image, indoor-trained, and **scale-ambiguous**. Monocular depth gives
relative structure, not metres. Our speed budget and voxel map are metric end
to end; an unscaled depth field feeds neither. **There is no stereo matcher in
the zoo.**

Porting one (HITNet, MobileStereoNet) through the Dataflow Compiler is a
project not a purchase: op-support risk (cost volumes and 3D convs are exactly
what NPUs handle badly -- HITNet exists BECAUSE 3D cost aggregation is 60-75 %
of inference in PSMNet-class nets), INT8 quantisation loss, and forest training
data we do not have.

**The architectural objection is stronger than the practical one.** stereo_bench
measured that BM fails safe (declines -> holes -> UNKNOWN -> no speed earned)
and SGBM fails dangerous (interpolates -> confident wrong depth -> carves
through a trunk). A learned network is that axis taken further: its entire
value proposition is filling in where geometry is ambiguous, which on
low-texture bark means hallucinating a smooth surface at the wrong depth. For a
map built on "unknown != free" that is backwards. DeFoP hit this exactly --
their learned CPN "occasionally proposes unsafe velocity commands", which is
why they bolted a geometric supervisor on top. We would be buying the thing
they had to defend against.

**Cost, with the NPU's weight penalty included:**

    IMX296 x3 + lenses + tube, CPU stereo   ~EUR 105    ~63 g
    IMX296 + AI Kit                         ~EUR 175    ~90 g
    OAK-D LR                                ~EUR 400   ~100 g
    Gemini 336L                             ~EUR 300-400    ?

The NPU erases most of the weight advantage. And **the strongest version of the
cheap argument drops the NPU entirely**: if the Pi runs classical stereo in
25-35 ms, that is EUR 105 vs EUR 400 with the fail-safe matcher intact.

**Where the NPU DOES earn its place: stage 2, lock-on.** Detection and tracking
are genuinely neural and Hailo support is mature (YOLOv8n etc.). Moving the
tracker and detector off the CPU FREES budget for classical stereo. Correct
split: **NPU does the neural things, CPU does the geometric things.** Buy it
for lock-on if it helps there, never as a stereo accelerator.

Note: the AI Kit is an M.2 HAT+. If HATs are back on the table, so is the
Arducam Camarray (hardware I2C/clock sync at any baseline you choose, one CSI
port, but the rigid-mount problem stays since the boards are separate).

### OAK-FFC — the option that is BOTH ready-made and custom-baseline

Missed in the earlier "ready-made vs roll-your-own" framing. **OAK-FFC-3P/4P**:
camera modules on FFC flex cables, so **the baseline is yours to choose**, while
the Myriad X still computes depth in silicon.

* OV9282 global-shutter mono, 1280x800
* **zero Pi CPU for depth**
* sync solved (one baseboard clocks both)
* ~EUR 240 (board + 2 modules) — VERIFY

**Lens variants (CORRECTED — an earlier note here assumed 72 deg HFOV; the
standard part is 80 deg):**

    variant                    DFOV   HFOV
    standard OAK-FFC-OV9282     89     80
    wide -W (PY097W)           150    127
    OAK-FFC-OV9282 M12          --    your lens

**What 150 deg costs is LESS than the naive answer.** Treating it as
rectilinear gives a brutal number and is wrong: at 127 deg it is a FISHEYE, and
a fisheye spreads pixels uniformly per degree rather than concentrating them
centrally, so centre resolution does not collapse.

Effective centre focal length over 1280 px, and Z_max at a 15 cm baseline:

    lens            HFOV   f_centre   Z_max@15cm
    M12 4 mm         52      1333       10.6 m
    M12 2.8 mm       69       933        8.9 m
    standard         80       763        8.0 m
    wide 150 deg    127      ~578       ~7.0 m

**~13 % range loss for 1.6x the horizontal coverage.** Far better trade than it
looks. Fisheye row is an ESTIMATE (assumed equidistant projection) — check
Luxonis's actual calibration before banking it.

**Coverage is something we have MEASURED a need for.** With a narrow forward
camera the sides and rear are permanently UNKNOWN and the aircraft spent
**638 of 700 steps stationary** because nothing could be confirmed safe. Escape
primitives exist purely to work around that. 127 deg HFOV would see most of the
space the escape set wants to move into, so the wide variant is arguably the
better fit for our specific failure mode.

Two real objections: (1) **fisheye breaks the pinhole assumption** that
rectification, the Z_max derivation, `depth_camera.hpp` and `voxel_map`
raycasting all rest on — `cv::fisheye` makes it tractable but the sim would
model the wrong camera until updated; (2) edge quality — worse calibration
residual, more distortion, shrinking stereo overlap at the extremes.

**Pick the M12 variant.** It makes OAK-FFC identical to the IMX296 plan in the
one respect that matters (we choose the lens) while keeping zero-CPU depth and
hardware sync. Start at 2.8 mm — 69 deg is close to what the sim already models
— and go wider later if coverage proves binding.

**What the third FFC port enables: narrow stereo pair for RANGE plus a separate
wide camera for COVERAGE.** Both, without compromising either. No fixed-housing
camera can do that; it is the actual value of the modularity.

Matches or beats the custom IMX296 build on range AND returns the 13-24 ms of
CPU. **Does NOT solve the mount** — flex cables mean two separate boards, so
the 26 um rigidity problem and the recalibration ritual are unchanged. Same
tube, same analysis. OAK-D LR is the variant where rigidity is pre-solved in a
fixed housing; OAK-FFC trades that back for baseline freedom.

Caveats: USB3 to the Pi (bandwidth, power, latency we do not control), DepthAI
host dependency, and a fixed matcher — though StereoDepth exposes confidence
thresholding and LR-check, **so it can likely be configured to leave holes
rather than interpolate. VERIFY THIS** — it decides whether it fails safe or
fails dangerous, which stereo_bench showed is the distinction that matters for
a map built on "unknown != free".

**Luxonis open-sources the hardware:** `luxonis/depthai-hardware` (Altium +
community KiCad, **MIT**) and `luxonis/oak-hardware`. `BG0250TG` is a carrier
for a SINGLE OV9282, documented as "typically paired with another to create a
stereo camera pair" — the modular building block, design files public.

**Do NOT spin our own MIPI PCB.** CSI-2 is impedance-controlled differential
pairs with length matching; a subtle layout error shows up as intermittent
frame corruption that costs weeks. The Luxonis files are a reference for how it
is done, not a shortcut past learning it. **Our custom-hardware sweet spot is
mechanical, not electrical: buy the boards, build the bar.**

### Three viable builds, all sharing the same mount problem

    build                              baseline    Z_max     Pi CPU   ~cost
    IMX296 x2 + bar                    your choice  6.3-8.6m  13-24ms  EUR 105
    OAK-FFC-3P + 2x OV9282 + bar       your choice  8.6-11m   0 ms     EUR 240
    OAK-D LR                           fixed 15cm   ~9 m      0 ms     EUR 400

## 2026-08-11 — SIM RUN on D435i geometry. One real warning, three noisy numbers.

Added `--camw/--camh/--hfov/--baseline/--maxinteg` to voxel_sim, and made Z_max
DERIVE from the camera rather than sit at a stale default -- swapping sensors now
moves the carve limit with it instead of silently keeping the old one.

Forest world, 848x480 / 87 deg / 50 mm baseline, 220 steps, seed 1, vMax 3.0:

    cell   Z_max    travelled   stopped    false-free
    0.15   2.75 m    62.0 m      6/220      1.24 %
    0.25   3.54 m     5.6 m    201/220      0.80 %
    0.40   4.48 m    50.9 m     45/220      0.95 %
    0.60   5.49 m    35.9 m      0/220      4.81 %   <-- COLLISION

**BELIEVE THIS ONE: coarse cells bought range and paid for it in a crash.** At
0.60 m cells Z_max reaches 5.49 m, the aircraft never stops, and it hits a tree
at step 122 doing 2.96 m/s. The diagnostic is unambiguous -- "of 3 truth-SOLID
cells in the robot volume, the map called 3 FREE, 0 UNKNOWN, 0 OCCUPIED". Not
unknown, FREE. That is the carve-through failure the Z_max discipline exists to
prevent, and false-free jumped 6x (0.80 -> 4.81 %).

**Coarse voxels are NOT a free way to buy range.** That answers the "accurate
and coarse pixels" question directionally.

**DO NOT BELIEVE THE REST.** The ordering is non-monotonic: 0.15 has the
SHORTEST range yet travelled FURTHEST (62 m, 6 stops) while 0.25 has longer
range and got completely stuck (5.6 m, 201 stops). That cannot be a real effect
of range -- single-seed noise dominates, exactly the trap eval_noisefloor.py
documented on the tracker side (mean swinging 18 points on perturbations that
could not matter).

**The test was also harsher than reality.** vMax = 3.0 m/s; BOTH papers flew
1.0 m/s. Stopping distance at 3 m/s is ~2.25 m against a Z_max of 2.75-3.5 m --
no margin, and very likely what produced the 201-stop result. At 1 m/s the
budget is 0.42 m and the picture should change completely.

So this run does NOT say the D435i cannot work. It says the D435i AT 3 m/s in
dense forest WITH COARSE VOXELS is dangerous -- narrower, and believable.

**Next, to make it decisive:** add `--vmax`, run at 1.0 m/s, 4+ seeds per
configuration so the noise floor is visible. Until then no cell size is chosen.

## 2026-08-11 — coarse map: measured, 8 paired seeds

Lanes world (thickets and clear lanes side by side, trails OFF), D435i geometry,
0.25 m cells, 1.5 m/s, 400 steps. Coarse far map ON vs OFF, same seed both arms.

Metric: distance CLOSED toward goal / elapsed time / vMax. Raw "travelled" is
useless across speeds — the sim runs a FIXED STEP COUNT, so distance is just
time x velocity. That mistake made 3 m/s look 3x better than 1 m/s.

    seed      ON     OFF     diff
    1      0.807   0.692   +0.115
    2      0.780   0.595   +0.185
    3      0.723   0.722   +0.002
    4      0.755   0.705   +0.050
    5      0.638   0.707   -0.068
    6      0.832   0.545   +0.287
    7      0.725   0.725   +0.000
    8      0.783   0.670   +0.113

    n=8  mean +0.085  sd 0.114  SE 0.040  t=2.12  better in 6/8
    ON 0.755   OFF 0.670

**Verdict: real but modest, and weaker than 3 seeds suggested.** t=2.12 is at
the conventional threshold, not past it. One seed negative, two dead level. The
per-seed SIGN (6/8) is the honest summary, not the mean. Do not quote +13 %.

Long runs, 1500 steps (150 s), seed 1: ON closed 56.8 m (eff 0.252), OFF 44.9 m
(eff 0.200). Same sign, same magnitude. Both ended on step exhaustion, neither
crashed. False-free < 0.7 % in both — duration did not degrade the map.
**Efficiency is NOT comparable across run lengths**, only within them (longer
runs spend more time working laterally across lanes).

Collisions across the whole matrix: 2, both in the OFF arm. Suggestive only —
the coarse map cannot prevent a collision, it only picks a different route.

**Still open:** the 1 m/s mixed-world result (+32 %) was 2 seeds and has not been
re-run at n=8. If the coarse map's value really is largest at low speed, that is
the run that matters, because 1 m/s is what both reference papers flew.

## 2026-08-11 — STEREO: DECIDED. Buy a used D435i, do not build.

**Decision: buy a used D435i (~EUR 200). The custom stereo bar is shelved.**

Reasoning, in order of weight:

1. **Stereo is stage 4 of our own plan** and nothing is blocking on it. The
   lock-on tracker runs on the analog feed. Software is validated, the airframe
   flies, and the two have never met. Building a stereo rig now is working out
   of order on the one stage that blocks nothing.
2. **It buys away four problems we did real analysis on**: the 26 um mount
   budget, hardware sync + the 1.8 V trigger wiring, calibration drift and the
   recalibration ritual, and 13-24 ms of Pi CPU per frame.
3. **Both published systems that actually fly under canopy use this camera.**
   Karjalainen 7/7 at 650 trees/ha on a D435 with the emitter DISABLED; DeFoP
   15/15 on a D435i. 2.5 m of honest range is the configuration that works in
   the field, not a compromise.
4. It carries an IMU rigidly co-located with the cameras -- exactly the problem
   flagged for a soft-mounted custom rig.

Used-unit checks: genuine Intel via `rs-enumerate-devices`; IMU actually present
(some listings mislabel a plain D435); `realsense-viewer` for dropped USB3 frames
and dead columns; budget 0.7-1.5 A on USB3 out of the PD chain; librealsense on
ARM means building from source.

### The range fear, and why Z_max was the wrong number to fear

**Z_max is the CARVING limit, not the seeing limit.** The camera returns depth
far past it; what degrades is precision, and `dZ = Z^2*sigma_d/(f*B)` says how
much. D435i at 848x480 (f = 447 px, B = 50 mm):

    range    dZ        as %
    2 m      +-4.5 cm   2 %     <- matches Intel's own "<2% at 2 m" spec
    3 m      +-10 cm    3 %
    5 m      +-28 cm    5.6 %
    8 m      +-72 cm    9 %
    10 m     +-1.1 m    11 %

**At 5 m you get +-28 cm.** That is not blind. For "is there a 3 m gap over
there" it is overwhelming precision. Z_max landed at ~2.5 m only because it was
defined as dZ <= one voxel, which is the budget for CARVING FREE SPACE -- the
one error that kills you. It is the wrong budget for every other decision.

**Three ranges, not one** (voxel_map.hpp already says this: "a ray that returns
30 m still proves the first several metres are empty"):

* carve free space -- short and conservative, ~2.5-3 m. Unchanged.
* mark occupied -- much further. An obstacle at 5 +- 0.28 m is still an
  obstacle; 28 cm is small against a 0.6 m robot radius.
* **assess openness for steering -- read the RAW DEPTH IMAGE, no map involved.**
  This is DeFoP's geometric supervisor: sector the frame, check the fraction of
  pixels nearer than a threshold. Works to whatever range the sensor returns,
  because "this sector is empty to 5 m" needs no sub-voxel precision.

The third is the piece we do not have and the direct answer to the range worry.
We do not need the MAP to see a gap at 5 m; we need the depth image.

Also: the map is a rolling window. At 1-3 m/s, space 5 m away is 2.5 m away in
under two seconds and fills in properly as we approach.

**If range still binds after real flights, the fix is a D455, not custom
stereo.** Same family, same software, 95 mm baseline instead of 50 -- doubles
f*B, halves dZ at every range (+-15 cm at 5 m), Z_max ~2.5 m -> ~4.9 m. Drop-in
swap, ~EUR 100 more, ~40 g.

**What survives from the custom-stereo work:** the error model itself. It is
what tells us to trust ~2.5 m rather than the datasheet's 10 m, and what stops
the mapper carving through a trunk at 8 m. Used on day one.

**stereo_bench is now diagnostic, not a gate** -- the D435i computes depth in
silicon, so CPU matching cost no longer decides anything. Keep it for the day a
custom pipeline is reconsidered.

### DECISION GATE — do not buy until stereo_bench runs on the Pi

That one measurement now prices the alternatives, not just the custom route:

* under ~35 ms  -> **custom IMX296 wins.** Cheapest (~EUR 100 vs ~400),
  lightest (63 g), raw pairs for VO and photogrammetry, 8.6 m beats everything
  except OAK-D LR.
* over ~60 ms   -> **buy OAK-D LR.** Beats the custom build on range, ships the
  mount problem pre-solved, hands back the whole CPU budget.
* bark invisible on the tripod trip -> **OAK-D Pro**, whichever way timing went.

The custom route's case rests entirely on that timing number.

### The mount — decided

**Rule 1, non-negotiable: soft-mount the ASSEMBLY, never the cameras.**
TPU under each camera separately creates two independently resonating masses
coupled only through rubber; relative pose then oscillates at the isolator's
natural frequency and the extrinsic calibration is meaningless.

    RIGHT                          WRONG
    [cam]==rigid spine==[cam]      [cam]~TPU~        ~TPU~[cam]
            ~ TPU ~                      \  airframe  /
           airframe

**PICK: aluminium rectangular tube 20x10x1 mm, ~23 g at 150 mm**, oriented
20 mm FRONT-TO-BACK, 10 mm vertical. Hacksaw + drill, no milling.

(Superseded an earlier pick of 20x20x2 angle once the flex was actually
computed -- see the stiffness table below. Angle loses on torsion and on
section symmetry, and is 8 g heavier.)

Nut access in a 1 mm closed wall, three ways: M2 rivnuts (cleanest); a 6 mm
access hole drilled in the opposite face to drop a nut in with tweezers or a
magnet; or simply tapping the 1 mm wall -- loads are 0.78 N, so ~2 threads
carries it, though preload for a friction-stable joint is the weaker argument.

### Stiffness — computed, not asserted

20 g per camera (board + M12 lens), 4 g manoeuvre -> F = 0.78 N per end.
Mounts at +/-30 mm, cameras at +/-60 mm, so 30 mm cantilever each side.
E = 69 GPa. Tip rotation `theta = F*l^2 / 2EI`.

Budget: 220 urad (0.25 px), 880 urad (1 px working).

    section            mode                      I or J     theta    vs 0.25px
    20x20x2 angle      bending, weak diagonal    1175 mm4   4.4 urad     2 %
    20x20x2 angle      TORSION, offset load       101 mm4   135 urad    61 %
    20x10x1 tube       bending, yaw              2779 mm4   1.8 urad     1 %
    20x10x1 tube       bending, vertical          899 mm4   5.7 urad     3 %
    20x10x1 tube       torsion                   2089 mm4   6.8 urad     3 %
    20x3 flat bar      bending, vertical           45 mm4   114 urad    52 %
    PETG, angle sect.  bending                       --     148 urad    67 %

First natural frequencies 700 Hz - 3.4 kHz, all >= 3.5x the ~200 Hz prop
fundamental. So quasi-static analysis holds, no resonance, and the TPU
(50-80 Hz) is the compliant element with the bar rigid on top of it.

**Bending flex is a non-issue** -- 2-3 % of budget. Even the worst geometry
(single central mount, 60 mm cantilevers) reaches only 35 urad relative, 6x
inside budget.

**Why the angle lost:**

1. Torsionally weak. Open sections have tiny J (101 vs 2089 mm4, 20x worse).
   A camera bolted ~15 mm off the shear centre eats 61 % of the tight budget
   on torque alone. It lands in PITCH, which self-calibrates, so it is
   survivable -- but it is the largest term on the table and it is dynamic
   under vibration, which degrades matching rather than merely biasing it.
2. Unsymmetric section: I_xy != 0, so a purely vertical load ALSO deflects it
   sideways by 59 %. That breaks the "vertical bending -> harmless roll"
   separation the orientation argument depends on. A closed tube is symmetric.

Everything scales linearly with camera mass. At 20 g the tube is 30x inside
budget so even 60 g cameras are fine; the angle's torsion term would not be.

**Headline: flex is not what gets you. Thermal gradient and creep are.**
5 K front-to-back costs more than a 4 g pull-up.

Weight, 150 mm (120 mm baseline + board width), Al at 2.70 g/cm3:

    20x10x1.0 rect tube    56 mm2    23 g
    20x10x1.5 U-channel    55 mm2    23 g
    20x3 flat bar          60 mm2    24 g
    20x20x2 ANGLE          76 mm2    31 g   <- pick
    15x15x1.5 sq tube      81 mm2    33 g
    20x20x1.5 sq tube     111 mm2    45 g
    2020 T-slot           ~200 mm2  ~75 g

**T-slot extrusion rejected.** 2-3x the weight, and a T-nut is a FRICTION
joint on a continuous slot — free to creep under vibration, which is precisely
the systematic drift the map cannot average out. A bolt through a drilled hole
is positively located. If the slot is wanted for prototyping the baseline:
slide to find it, then drill through and pin it.

**Orientation matters more than section.** Long dimension FRONT-TO-BACK (along
the optical axis). Tolerated front-back gradient at the 1 px working budget:

    depth 10 mm -> 3.2 K
    depth 15 mm -> 4.9 K
    depth 20 mm -> 6.4 K

Pairs with the error asymmetry: vertical-plane bending -> ROLL (self-calibrates
free from vertical disparity); horizontal-plane bending -> YAW (does not). Deep
front-to-back, shallow vertically, puts stiffness where the uncorrectable error
is. Caveat: do not take "roll is free" to a flimsy section — dynamic roll above
~1 px within a frame breaks row-wise matching outright rather than just adding
noise. Angle or channel, not thin flat bar.

**Materials.** PETG disqualified for the spine: 0.4 K of gradient blows the
0.25 px budget (alpha ~70 ppm/K), plus room-temperature creep under bolt
preload. Non-structural use only, and never holding preload alone — metal
washers. Carbon is the later weight upgrade, not the start: near-zero axial CTE
is excellent (55 K tolerated) but pultruded tube is torsionally floppy, and
bonded end fittings creep. Aluminium's 150 W/mK also erases the gradients that
CF's ~1 W/mK transverse would hold, clawing back most of CF's CTE advantage.

**Hole accuracy is a non-issue.** Calibration absorbs constant misalignment to
~1-2 deg (limited only by rectification crop). Hand drill and centre punch are
fine. Building something that does not MOVE, not something that is ACCURATE.

**TPU:** 4 grommets, spine to frame, target 50-80 Hz natural frequency (5"
prop at ~6000 rpm, 2 blades = ~200 Hz fundamental).

**Open consequence:** soft-mounting means the rig's pose relative to the FC's
IMU is not fixed, and VIO depends on that extrinsic. This is why a D435i has
its own IMU inside the housing. If VIO happens on the IMX296 pair, budget an
IMU **on the stereo bar**. It is a hole in the bar, not a redesign — decide
before drilling.

### Sync is officially supported — use trigger mode, not master/slave

`imx296.trigger_mode=1` in `/boot/firmware/cmdline.txt` enables XTRIG in the
**mainline RPi kernel module**. Not a patch. RPi docs explicitly endorse the
stereo use: "multiple cameras can be connected to the same pulse, allowing for
an alternative way to synchronise two cameras." Exposure is deterministic:
**low pulse width + 14.26 us**.

**Better than XVS/XHS master/slave, for a reason missed earlier: if both
cameras see the SAME trigger edge, pulse jitter cancels.** Skew between them is
only the difference in their response — nanoseconds. Even a sloppy userspace
GPIO syncs them TO EACH OTHER; jitter only affects frame-rate regularity, which
matters for VO timestamps and not at all for disparity.

**Catch: trigger mode sets exposure by pulse width, so auto-exposure is gone.**
Under dappled canopy that is a real cost and it becomes our job — vary pulse
width from a control loop on the Pi. Known requirement now, not a surprise
later.

Still 1.8 V — divide any Pi GPIO down before it touches XTR.

### CPU stereo cost — the remaining top risk, but ReS2tAC suggests it is OK

**ReS2tAC** (Ruf et al., Sensors 21(11):3938, arXiv 2106.07927) — SGM optimised
with **NEON intrinsics for embedded ARM**, built for UAV use. Reported up to
**46 FPS at VGA**, 3.3 % error. That is ~22 ms/frame at 640x480, which fits
alongside a 16 ms planner.

**UNVERIFIED — the paper covers both ARM-NEON and CUDA targets and the abstract
does not say which target the 46 FPS belongs to.** Could well be the GPU
number. Could not read the paper (arXiv 403s through this proxy) or confirm a
code release. **Pinning this down is the highest information-per-effort item
available.**

### Forkable code found

    gishi523/semi-global-matching   SGM CPU, census, OpenMP -- SSE4.1, needs NEON port
    kbatsos/Real-Time-Stereo        CENSUS/NCC/SAD block matchers -- closest to our
                                    sim model (blockPx = 8)
    ermig1979/Simd                  NEON + SVE for ARM -- the porting substrate
    Eryk-Mozdzen/open_vins_example  ROS-FREE OpenVINS usage -- matters, no ROS here
    guglielmo610/DeFoP              the paper's own stack; read the geometric
                                    supervisor, do not fork (Orin + TensorRT + ROS)
    ZJU-FAST-Lab/EGO-Planner-v2     Karjalainen's base; MINCO, ring-buffer occupancy,
                                    A*.  **GPLv3** -- check against licensing intent
    mzahana/roboeye                 VIO on RPi, unassessed

**CLOSEST MATCH TO OUR CONSTRAINTS — TU Delft MAVLab.** "Efficient Optical Flow
and Stereo Vision for Velocity Estimation and Obstacle Avoidance on an
Autonomous Pocket Drone", arXiv 1612.06702. Our exact two-problem architecture
-- **stereo for obstacles, optical flow for velocity** -- solved together at
10 Hz on a **20 g flapping-wing MAV with a microcontroller**. Its premise is our
problem statement verbatim: even efficient stereo is too heavy for pocket
drones, so they built one that is not. Code lives in **Paparazzi** (C, no ROS,
**GPL — check against licensing intent**).

Caveats: their imagery is ~128x96 at short range, so the NUMBERS do not
transfer to 728x544 at 8 m. What transfers is the APPROACH — sub-sampled/sparse
stereo, integer arithmetic, exactly what a NEON port would want. And it is
strong evidence the Pi 5 anxiety is overblown, given they had orders of
magnitude less compute. It also independently validates the SMF-VO direction:
flow-derived velocity plus stereo obstacles, on a budget, flying.

**Full stereo camera pipeline references (calibrate -> rectify -> disparity):**

    shubhamwagh/stereo-calib              Charuco calib + rectify + disparity +
                                          depth. Cleanest structural reference.
    0xShamil/stereo-calibration-raspberry-pi   C++ on the Pi specifically
    realizator/stereopi-tutorial          Python, but has the Pi-specific
                                          real-world gotchas
    libelas (ELAS)                        efficient large-scale stereo, CPU C++,
                                          fast on big images -- GPL, check

**PX4/PX4-Avoidance** — VFH+ local planner PLUS an octomap global planner, i.e.
structurally what we built, arrived at independently. ROS, and PX4 has since
deprecated it, so read rather than fork. Worth noting it pairs local histogram
with global octomap **for exactly the dead-end reason the cul-de-sac world
found** — third independent group, same conclusion.

**CHEAPEST HIGH-VALUE READ:** `librealsense/doc/depth-from-stereo.md` — Intel's
own writeup of the stereo error model, from people shipping this commercially.
It will confirm or correct `Z_max = sqrt(cell*f*B/sigma_d)`, the formula every
purchase decision in these notes rests on. Half an hour.

**SMF-VO has no public code found.** Read-and-reimplement, not fork. Upside:
sparse flow into a linear velocity solve is a small algorithm, unlike
reimplementing VINS-Fusion. Confirmed it is benchmarked against Basalt and
ORB-SLAM3 **on a Pi 5** at 100 Hz — a real comparison on our exact hardware.
Fallback if it stays unavailable: OpenVINS (mature, mostly single-threaded,
would eat most of the budget) or Basalt (faster, wants more compute).

**Next actions from this:** (1) pin down whether ReS2tAC's 46 FPS is ARM or
CUDA; (2) budget for writing the auto-exposure loop that trigger mode forces
on us.

### stereo_bench — BUILT. Run it on the Pi.

`onboard/tools/stereo_bench.cpp`. No cameras, no other deps.
`cmake --build build --target stereo_bench && ./build/stereo_bench`

Synthetic forest stereo pair (ground plane, backdrop, vertical trunks with
deliberately low bark contrast), swept over resolution x {BM, SGBM_3WAY, SGBM}.
Reports ms/frame, valid%, bad%, measured inlier sigma_d, and Z_max both as
ASSUMED (0.25 px, from voxel_map.hpp) and as RE-DERIVED from the sigma_d each
matcher actually delivered.

**x86 dev-host baseline (Xeon 2.8 GHz, 4 threads) — NOT the Pi:**

    res        algo        f_px   D |     ms  valid%  bad% | sd_px  Zmax_m
    1456x1088  BM          1159 144 |  89.1*   78.7%  0.2% |  0.12   12.5m
    1456x1088  SGBM_3WAY   1159 144 | 157.8*   84.2%  0.5% |  0.18   10.3m
    728x544    BM           580  80 |  13.0    76.6%  1.3% |  0.13    8.6m
    728x544    SGBM_3WAY    580  80 |  23.9    83.3%  0.9% |  0.23    6.5m
    485x362    BM           386  48 |   7.7    76.0%  1.8% |  0.13    7.1m
    364x272    SGBM_3WAY    290  48 |   7.1    81.7%  2.1% |  0.24    4.5m
    242x181    BM           193  32 |   2.4    67.5%  5.4% |  0.23    3.8m

Full resolution does NOT fit a 40 ms budget in any mode. 728x544 does, at
13-24 ms, with ~8.6 m of range. **Expect the Pi 5 to be 1.5-3x slower** (A76
2.4 GHz, 128-bit NEON vs the host's 256-bit AVX2), so 13 ms here plausibly
becomes 20-40 ms there — straddling the budget. The tool prints a loud warning
on non-ARM hosts for exactly this reason.

**Cost scales as scale^3, range as sqrt(scale).** Halving resolution is ~8x
cheaper (measured 6.2x, since D rounds to multiples of 16) and costs only 29 %
of the range. That asymmetry is the whole reason the tool prints Z_max beside
milliseconds.

**Measured inlier sigma_d is 0.12-0.24 px, i.e. voxel_map.hpp's assumed 0.25 px
is conservative** — for inliers, on this synthetic scene. Do not bank it: real
bark, real foliage and real lighting will be worse, and the scene here is
mostly fronto-parallel slabs with band-limited texture.

**The finding worth keeping: valid% and bad% are NOT symmetric for us.** A
missing pixel is UNKNOWN, and unknown is safe — the speed budget earns nothing
through it. A confidently WRONG pixel is what carves free space through a
trunk. Drop bark contrast (`--trunktex 0.06`) and the families separate exactly
along that line: BM declines to guess and sheds valid pixels (76.6 -> 71.4 %),
SGBM interpolates and gains wrong ones (bad% 0.9 -> 2.9 %). **BM fails safe,
SGBM fails dangerous.** For a map built on "unknown != free", that inverts the
usual benchmark preference — BM is the better fit despite looking worse on a
leaderboard.

### Driver sync (superseded — see trigger mode above)

RPi's `imx296` driver historically had no external-sync support; dtoverlay work
exists and people have master/slave running, but it is fiddly.

**Gate on this before anything touches the frame:**

1. one camera streams on Pi 5
2. both stream simultaneously on the two CSI ports (check CSI bandwidth at the
   chosen resolution)
3. solder MAS on one, wire XVS + XHS + common GND, master-first with the 2 s gap
4. **verify sync by the failure mode:** compute disparity on a static scene
   while rotating the rig by hand. Synced -> disparity stays put. Unsynced ->
   disparity shifts with rotation rate, and the slope gives the skew.

Step 4 needs no extra gear and measures exactly the thing that would ruin us.
It also composes with the tripod/bark experiment above — same rig, same trip.

---

## 2026-08-11 — the last two ideas from the papers, measured

Implemented items (2) and (3) of the three remaining borrowings. Item (1), the
geometric depth-image supervisor, is planned in
`nav-sim/docs/DEPTH_SUPERVISOR_PLAN.md` and deliberately not built.

**Protocol, recorded properly this time** — the previous sweeps did not write
down their command lines, and the baseline has since moved (0.794 here against
0.761 recorded for the same nominal arm), which I cannot now account for. Every
comparison below is therefore *within one batch*, and the absolute numbers are
not comparable to the earlier tables.

```
voxel_sim --world forest [--lanes] --camw 848 --camh 480 --hfov 87 \
          --baseline 0.05 --vmax 1.5 --steps 400 --cell 0.25 --seed N
efficiency = (startDist - endDist) / (steps * 0.1 s) / vMax
```

### Sideslip (item 3): positive in sign, not yet significant

Lanes world, 8 paired seeds:

    arm       mean    vs base                          minclear
    base      0.794   --                               0.43
    slip 20   0.815   +0.021 SE .015 t=+1.43  6/8      0.50
    slip 35   0.818   +0.024 SE .020 t=+1.19  5/8      0.37

No collisions in any arm. The gain is **concentrated where the baseline did
worst** — seed 4, base 0.642, gains +0.112 at 20° and +0.148 at 35°; on the
seeds where base already ran at 0.82 the effect is ±0.01. That is the shape you
would expect from a mechanism that only matters when a turn is forced, and it is
a better argument than the mean is.

**20° is the setting, not 35°.** Same mean within noise, but minimum true
clearance goes the wrong way at 35° (0.37 m against base 0.43, with the
collision threshold at 0.30 m) while 20° improves it to 0.50 m. Buying a
statistically invisible mean with 6 cm of real clearance is a bad trade.

### Depth improver (item 2): NEGATIVE, and the sim cannot show its upside

Two batches, both against the same base:

    world                    arm     mean    vs base                   fill
    lanes                    imp     0.787   -0.007 SE .007 t=-1.05 1/8   0.1%/frame
    plain forest             imp3    0.506   -0.114 SE .070 t=-1.63 2/8   3.7%/frame

The lanes result is near-null because the improver **barely fires there**: probed
across thresholds, essentially nothing gets within 4 m of the camera in that
world at 1.5 m/s (near-return fraction 0.0 % at 2, 3 and 4 m; 2.4 % at 6 m). The
lanes world was built so the aircraft can keep its distance, which makes it the
wrong instrument for a near-field component.

The plain forest is where it fires, and there it **costs 11 % of holes filled and
0.114 of efficiency**, losing 6 of 8 seeds with two catastrophic ones (−0.52,
−0.30). No collisions either way, and minimum clearance slightly *worse*
(0.38 against 0.43) — so it did not even buy safety with the speed.

The mechanism of the harm is not mysterious and is the same one that was
supposed to be the benefit: a filled pixel becomes an OCCUPIED cell, and
`sphereClear()` hard-rejects OCCUPIED anywhere in the robot ball. Every
fabricated cell removes primitives from the library. In a world where the real
obstacles are already mapped, that is pure cost.

**And the sim cannot currently show the other side of the ledger.** The
improver's target is thin obstacles — a 3 cm branch that returns only its
silhouette — and `voxel_world.cpp` has no branches in it. Trunks are 0.2–0.4 m
and either resolve or do not. So this measurement says the improver is a net
loss *on obstacles that are already visible*, which is a real and useful
finding, but it is not a test of the claim.

Left in the tree, off by default, with the numbers attached. Two things would
have to change before revisiting it: thin branches in the world model, and the
depth supervisor, which is a cheaper way to get conservatism near obstacles
without fabricating map cells to get it.

**Cost, measured:** 848×480, r = 2/4/8 → 15.7 / 15.1 / 15.1 ms per frame on the
dev host including a clone. Flat in radius, as designed. The first version was
9–10 ms with a gather/scatter column pass on a *quiet* machine; the row-major
rewrite is the version benchmarked here under load. Either way this is not the
sub-millisecond component the supervisor will be — a Pi would spend a large
fraction of its cycle here, which is a further argument against it.

### Follow-ups: both verdicts firmed up, and neither goes on by default

**Sideslip, 16 seeds now (8 fresh).** The first batch said +0.021, t=+1.43, 6/8.
Fresh seeds 9–16 say **−0.007, t=−0.22, and 6/8 again**. Pooled:

    n=16   mean +0.007   sd 0.067   SE 0.017   t=+0.44   better 12/16
    median +0.015

**Wins often, loses big.** 12 of 16 seeds improve and the median gain is +0.015,
but two seeds lose 0.090 and 0.182 and wipe the mean out. A sign test on 12/16 is
p≈0.04 one-sided; the mean is indistinguishable from zero. Both statements are
true and the second is the one that decides it — a mechanism that occasionally
costs 0.18 does not go on by default on the strength of frequent small gains.

The asymmetry is a lead, not noise. Sideslip puts the aim bearing off the
aircraft's heading, and voxel_sim slaves heading to course, so the simulated
vehicle turns to follow the sideslip and ends up looking somewhere the map is
thinner. Worth checking on the bad seeds before either fixing or dropping this —
and it is exactly the coupling that disappears on ArduPilot, where yaw and
course are genuinely independent.

**Depth improver: a clean dose-response, and no operating point that helps.**
Plain forest at trunktex 0.25 (trunks partly resolving — the regime where its
claim should bite most):

    arm          mean    vs base                       fill        minclear
    base         0.538   --                            --          0.46
    imp3         0.453   -0.085 SE .068 t=-1.26  2/8   10% of holes 0.35
    imp3strict   0.535   -0.003 SE .044 t=-0.07  4/8    2% of holes 0.42

Strict settings (radius 2, 9 seeds) fill a fifth as much and the harm vanishes —
along with any effect at all. **The damage scales with the fill rate and so does
nothing else.** Seed 6 is the picture: 0.578 → 0.023 at the loose setting, an
aircraft essentially walled in by fabricated cells, recovering to 0.295 when the
fill is throttled. Minimum clearance tracks the same way.

That closes it as far as this simulator can. The improver stays off, and the
thing that would reopen it is thin branches in the world model, not a better
parameter.

---

## 2026-08-11 — FLIGHT STACK: ArduPilot, decided

Moving off iNAV/MSP. The reasoning is not about acro quality, it is about what
each stack lets a companion computer be:

* **MSP has no companion concept.** The only way in is RC override — the Pi
  impersonates a pilot's sticks and the FC never knows. That works (it is what
  `msp_backend.cpp` does) but it forecloses velocity setpoints, vision pose into
  the estimator, and any mode the autopilot understands as "something else is
  flying". Every one of those is a first-class message on ArduPilot.
* **ACRO is mode 1** (`COPTER_MODE_ACRO`), confirmed against the dialect, and it
  sits on the same mode switch as GUIDED. Manual flying and autonomy do not have
  to be separate aircraft.
* **VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE** are the supported path
  into the EKF. That is the missing state-estimation subsystem's landing spot,
  and it is the strongest single argument here.

**Honest caveats, none of them blocking:**

1. **ACRO is not Betaflight.** ArduCopter's rate controller runs a 400 Hz loop
   and is tuned for stability, not freestyle feel. It is flyable; it will not
   feel locked-in the way a BF quad does. If the aim were freestyle this would
   be the wrong call — it is not.
2. **Board support is the real constraint.** F405 targets are flash-squeezed on
   ArduCopter 4.4+ and some features get trimmed or dropped. **H743 is the safe
   choice.** Check the specific FC against the ArduPilot hardware list before
   ordering anything, because this is the one decision that is expensive to undo.
3. **Non-GPS setup is not the default.** GUIDED needs a position estimate;
   without GPS that means `EK3_SRC*` pointed at an external nav source, and the
   pre-arm checks have to be told what is legitimately absent. Getting this
   wrong looks like "it refuses to arm" and eats an afternoon.

**Two configuration facts that will cost an afternoon each if missed** (both now
printed by the backend on connect):

* `SYSID_MYGCS` (`MAV_GCS_SYSID` on 4.5+) gates who may send
  RC_CHANNELS_OVERRIDE and mode changes. Default 255, so the backend's sysid
  defaults to 255. If a real GCS is also on the link, one of them has to move.
* `RC_OVERRIDE_TIME` (default 3 s) releases an override that stops being resent,
  and a channel value of **0 means "give this channel back to the receiver"**.
  Both are safety features. The backend writes 0 to every channel it is not
  driving, so the pilot's mode switch always reaches the FC — that is the single
  most important line in `sendControl()`.

**Implemented** (`onboard/`): `mavlink_v2.{hpp,cpp}` — dependency-free MAVLink v2
framing and the fourteen messages actually used; `mavlink_backend.{hpp,cpp}` —
the real backend replacing the stub that used to refuse to connect. Two control
paths: RC override (works in any mode, no EKF, same ControlCmd as MSP) and
body-frame velocity setpoints (needs GUIDED, and therefore needs the estimator
that does not exist yet — implemented so it has somewhere to plug in, not
because it is ready to fly).

**Not generated from common.xml, and pinned by golden frames instead.** Fourteen
messages did not justify vendoring mavgen into a tree whose whole discipline is
having no dependencies. The risk of hand-rolling is that a wrong CRC_EXTRA or a
transposed field does not error — the autopilot silently drops the frame, which
on a bench is indistinguishable from a bad solder joint. So every message is
checked byte-for-byte against what pymavlink emits for the same arguments.

That caught two real defects before any hardware existed:
* `RC_CHANNELS.chancount` is a **uint8 at offset 40**, not a uint16 at 38 (which
  is chan18). The backend read the wrong one: on a 16-channel link that is zero,
  so assist mode would never have latched a baseline.
* `connect()` reported `linkUp()` **before the autopilot had ever spoken**,
  copied from the MSP backend where a grace period is right (MSP is
  request/response). ArduPilot heartbeats unprompted at 1 Hz, so silence means
  silence — and claiming a link tells `main.cpp` it may take control of a serial
  port with nothing on the other end.

**Next on this path:** set the FC parameters above on real hardware, confirm the
link with `--fc=mavlink`, and fly RC-override first. GUIDED waits for the
estimator.

---

## 2026-08-11 — first real D435i numbers

Seller meet, RealSense Viewer only.

* **On-chip calibration health: −0.42.** Absolute value is what counts, so 0.42
  — the middle band (0.25–0.75, "could be improved"). Not a walk-away: this is
  precisely the recoverable drift a used unit is expected to have, and on-chip
  calibration is the tool for it. The test that matters is whether it *comes
  down* after applying: a high first reading is normal, a reading that will not
  improve after two rounds is mechanical.
* **Stereo works** — depth streams. Worth more than it sounds: producing depth
  at all requires both IR imagers to be functional enough to match, so a
  totally dead imager is ruled out. A *degraded* one is not.
* **Motion Module present.** It is a genuine D435i, not a D435. That was the
  only walk-away item that could not be worked around later.
* **The camera passes on every axis checked.** Nothing outstanding on the
  hardware.

**`voxel_live --live` COMPILES, LINKS, RUNS, AND HAS NEVER SEEN A CAMERA.**

Verified without hardware and without the SDK, and the method is worth keeping:
pyrealsense2's wheel exports the entire `rs2_*` C API (456 symbols), and
librealsense's C++ interface is a header-only inline wrapper over exactly that.
Headers from git plus the wheel's `.so` gives a real link.
`test/build_with_realsense.sh` does it in one command. Confirmed: the branch
compiles, the LIVE button enables, `--live` executes and reports
`No device connected` cleanly.

**librealsense is now loaded at RUNTIME** (`realsense_dyn.*`), after three
rounds of CMake search paths failed to find an SDK that was installed and whose
Viewer streamed fine. The C++ API is a header-only inline wrapper over a flat C
ABI, so the two dozen `rs2_*` functions this needs are declared locally and
resolved with `dlopen`/`LoadLibrary`. Consequences: no build-time dependency at
all, one binary for both cases, the branch is compiled on every machine so it
cannot rot behind an `#ifdef`, and the API version handed to
`rs2_create_context` is read FROM the library so a point-release mismatch is
impossible. `voxel_live --rs-check` answers "can this machine do live" in one
command with no camera.

**Confirmed on the Windows target machine:** the runtime loader works. The CI
artifact — built on a runner with no SDK — loads librealsense on the laptop once
`realsense2.dll` is beside the exe, and `--rs-check` reports the version. That
is the whole point of the approach proved end to end on the machine it was for.

The DLL had to be copied by hand because the four hard-coded paths all missed;
the loader now enumerates `*realsense*` folders under Program Files and checks
`bin/x64`, `bin`, `lib/x64`, `lib` in each, so the next build should find it
unaided.

## 2026-08-11 — FIRST REAL DEPTH THROUGH THE REAL STACK

Indoors, handheld, D435i -> VoxelMap -> TrajectoryPlanner. The whole thing runs.

    integrate  10.6 - 11.2 ms      plan  0.35 - 0.37 ms
    valid pixels  85 - 97 %        (indoors, close range, emitter on)

~90 Hz capable, and the planner is a rounding error next to the mapper, exactly
as in sim. The first-person pane reconstructs a recognisable room in voxels.

**The planner reported BLOCKED, 0 admissible, in a corridor and against a near
wall — and that is almost certainly correct rather than a fault.** `robotR` is
0.6 m, i.e. a 1.2 m wide vehicle, and the map is FOV-limited: with an 87 deg
forward camera the cells beside the robot ball are UNKNOWN, never OCCUPIED, so
`sphereClear` passes but the centreline FREE test truncates. Indoors at arm's
length there is genuinely nowhere a 1.2 m ball is confirmed clear. This is the
same deadlock the escape primitives exist for, and the 638-of-700-steps-
stationary note in `voxel_traj.hpp` is the same phenomenon in sim.

But "0 admissible" does not distinguish blocked-by-something-SEEN from
blocked-by-something-UNSEEN, and those want opposite responses — back off versus
look around. `TrajectoryPlanner::lastReject()` now counts occupied / unknown /
at-the-first-step, and the PLAN pane prints the dominant reason plus the robot
diameter. Confirming that the indoor BLOCKED is dominated by UNKNOWN is the next
five seconds of work with the camera.

### The short range was OUR POLICY, and one missing feature

Observed live: nothing beyond ~3.5 m, and no coarse voxels at all. Two separate
causes, neither of them the camera:

1. **`voxel_live` never built a far map.** `voxel_sim` has had one for months.
   That is why there were no larger voxels anywhere — there was no coarse level,
   not a coarse level that failed. Now built, integrated, fed to the openness
   term, and drawn under the fine slice in a distinct tint. Fine 0.25 m to
   3.5 m, coarse 2.0 m to **10.0 m**. Costs ~6 ms/frame (stride 4).

2. **3.5 m is `Z_max = sqrt(cell·f·B/sigma_d)·0.75` with sigma_d ASSUMED at
   0.25 px.** The depth image is full of valid returns well past it; we choose
   not to mark obstacles there because a 0.25 m voxel would be claiming a
   precision the measurement does not contain.

**Where range actually comes from**, f = 425 px, B = 50 mm:

    cell 0.25  sigma 0.25   ->  3.5 m      (today)
    cell 0.25  sigma 0.15   ->  4.5 m      (a better preset or calibration)
    cell 0.25  sigma 0.25  B 95 mm -> 4.8 m (a D455)
    cell 1.0   sigma 0.25   ->  6.9 m
    cell 2.0   sigma 0.25   ->  9.8 m      (the coarse map, now in)

**Cell size dominates.** Measuring sigma_d is worth doing and buys perhaps 30 %;
it is not the lever it looks like. The ladder is. That is an argument for
measuring sigma_d to stop ASSUMING it, not for expecting it to transform the
range.

Also: 0.25 m cells are a forest default and look absurdly chunky at 1 m indoors.
The menu cycles 0.15–0.50; use 0.15 on a bench.

### A FINE near layer, and the stride result that came out of it

The coarse map's argument runs both ways. Cell size should match the depth
uncertainty at the range it covers, which is why 2 m cells are honest at 10 m —
and equally why 0.25 m cells are needlessly coarse at 1.5 m, where
dZ = Z^2*sigma/(f*B) is only 10 cm. Indoors the 0.25 m map throws away
resolution the measurement actually contains.

    cell 0.05 -> honest to 1.55 m       cell 0.15 -> 2.68 m
    cell 0.10 -> honest to 2.19 m       cell 0.25 -> 3.46 m

**Finer does not mean bigger**, because a level only has to cover its OWN honest
range: 0.10 m cells over a 5 m box is 53x53x26 = **0.07 M cells**, a seventieth
of the 0.25 m grid. Memory was never the constraint.

**TIME was, and the fix inverts the obvious.** Measured per integrate, 848x480:

    fine 0.25 m, stride 2   14.8 ms
    near 0.10 m, stride 2   19.4 ms      finer AND dearer
    near 0.10 m, stride 4    6.4 ms      and no less useful

Cost per ray rises as the cell shrinks (DDA steps scale as 1/cell), while a
NEAR object is angularly LARGE and needs less angular sampling — at f = 425 a
10 cm object at 1.5 m spans 28 px, so every fourth pixel still puts seven
samples across it. **A finer layer wants FEWER rays.** Stride should follow
angular size, not resolution; both the coarse and the fine companion levels end
up wanting a bigger stride than the middle one, for opposite reasons.

(`recentre` was the first suspect and was not the culprit: 14.81 vs 14.85 ms.)

Three layers together: **21.7 ms/frame, 46 Hz**, against 14.3 ms for the fine
map alone. `--nearcell` / `--nonear`, on by default at 0.10 m.

**AWARENESS AND DISPLAY ONLY.** `sphereClear` still reads the 0.25 m map alone.
Wiring a second level into the swept-volume test changes the one piece of code
that must not be wrong, and it earns that only with a measurement — the MID rung
of the coarse ladder was reverted for exactly this reason. But the real prize is
there: a 3 cm branch cannot cross occThresh in a 0.25 m cell and is most of a
cell at 0.05 m, and late thin-branch detection is the documented cause of real
failure in both reference papers.

### 3D plan view, and why the obvious viewpoint does not work

Drew the trajectory library into the FIRST-PERSON view first. It renders
correctly and is useless: forward paths run along the optical axis, so they
project to a cluster a few pixels across at the vanishing point. Rendering it
was the only way to find that out — the maths was right and the picture was
empty.

So: a CHASE view, the same renderer from behind and above the aircraft, framed
on the PLAN's own length (`horizonS * vMax`, 3 m at 1.5 m/s) rather than on the
map's range. The first attempt sat a whole map-range back with a 90 deg lens and
put the aircraft alone in an empty field. `v` cycles first-person / overlay /
chase.

Then three more things had to be fixed before the pane showed anything, and all
three were found by rendering it and reading the picture rather than the code:

* **It was drawing the NEAR layer.** That grid is sized to its own honest range
  and no more — 2.4 x 2.2 m across, which is the whole reason a fine layer is
  cheap. A camera two metres *behind* the aircraft stands at its edge. Correct
  for a near layer, wrong for an overview; the chase pane uses the 0.25 m map.
* **The unknown fog whited it out.** That fog is an honesty device for a view
  from the aircraft: it says "you are looking through space nobody measured".
  Move the eye outside the aircraft and the number stops meaning that — the
  unknown between a chase camera and the scene is unknown because I chose to
  stand there. At first-person strength (`min(0.85, unknownM/6)`) it saturates
  on the first ray. `FpvStyle` makes it a parameter; chase uses 24 m / 0.30 cap,
  measured at 2.2x the contrast against fog, pinned in `overlay_align_check`.
* **Elevation, not distance, is what makes it read as 3D.** A path's forward
  extent projects to `sin(elevation)` of itself. At the 14 deg the first framing
  gave, three metres of rollout became half a metre of picture. 0.55 span back,
  0.62 up, aimed at the *middle* of the plan, is 31 deg — half the forward
  extent survives — and a 55 deg lens magnifies the plan without pushing the
  camera further from it.

### The panes were drawing one rung of the ladder

Even after all that the chase pane had almost no voxels in it, and so did the
first-person pane — which is exactly what I saw on the real camera and wrote up
as "the range was very very small, no larger voxels". It was not the range and
it was not the mapper. **Every view picked the FINEST map available and rendered
only that**, and a fine map's honest *marking* range is short by construction:
3.5 m at 0.25 m cells, 2.2 m at 0.10 m, both on the D435i's 50 mm baseline. A
room or a wood whose nearest surface is four metres away therefore rendered as
an empty pane. The coarse level had already marked those trunks — 2 m cells
reach 10 m — and simply had nowhere to appear.

`VoxelMap::renderLadder` casts every level from one eye and keeps the **nearest
hit** per pixel. No priority order is needed: where two levels both know about a
surface, the fine one is in front of the coarse one's blocky approximation and
wins on its own merits; where only the coarse one knows anything, it is all
there is. `overlay_align_check` pins it with two posts and a fine map that stops
between them — fine alone sees 0 px of the far post, coarse sees 327, the ladder
keeps 327.

Costs, measured on the chase pane over 30 frames, and the reason for two
decisions:

| | view render |
|---|---|
| three levels, full pane resolution | 47.5 ms |
| three levels, half resolution, upscaled | 47.5 ms → *the near layer dominates* |
| without the 0.10 m layer | 22.9 ms |

So the chase view drops the fine layer: its grid is ±2.6 m and the chase eye
sits 1.65 m behind the aircraft, so it contributes a sliver for half the render
budget. DDA steps scale as 1/cell, which is why the finest layer is always the
dearest to *draw* as well as to build. Both first-person views cast at half the
pane's resolution and upscale — a picture made of cubes loses nothing, and the
plan is still drawn at full size. Order matters there: the first-person render
is square (vfov = hfov = 90°) and the pane is 4:3, so stretching before
projecting would draw the plan through an aspect the render was never made with.

Render cost is now reported separately and explicitly excluded from ONBOARD
TOTAL — the aircraft never draws any of this.

Unrelated but found the same way: `voxel_live` silently ignored unknown flags,
so `--steps 40` (there is no such flag; it is `--frames`) opened a windowed
session that ran forever and wrote nothing, and looked exactly like a hang. It
now exits 2 and says so.

`VoxelMap::fpvProject` is the exact inverse of `fpvRay`, which the renderer now
calls too — so the projection cannot drift from the render. Pinned at 0.0001 px
over 165 pixels, plus an assertion that a point BEHIND the camera is rejected
rather than folded to the front, which is the failure that would draw a retreat
primitive as though it went forwards.

**Still untested: everything after a device is found** — stream negotiation,
intrinsics readback, depth scale, whether the baseline comes from the
extrinsics or falls through to the nominal 50 mm, and the uint16→metres loop.

And the `--live` branch had **never been compiled anywhere** until the
librealsense headers were fetched by hand — it sits behind `#ifdef
NAVSIM_HAVE_REALSENSE`, and no machine that built this tree had the SDK. It did
not compile: `prof.get_device().first_depth_sensor()` is a PYTHON binding, and
the C++ API is the templated `device::first<rs2::depth_sensor>()`. An
`#ifdef`-ed branch that never compiles anywhere is not code, it is a plan. It
compiles now.

**Mapping cost, measured, forest world, 0.25 m cells:**

    848x480  stride 1   407 k rays   58.8 ms    17 Hz
             stride 2   102 k rays   19.2 ms    52 Hz
             stride 4    25 k rays    5.3 ms   190 Hz
    424x240  stride 1   102 k rays    8.4 ms   119 Hz

Integration dominates completely — the planner is 1.5 ms against 59 ms of
mapping. **Stride 1 at full resolution cannot keep a 30 fps camera**, on a
desktop, never mind a laptop or a Pi. `voxel_live` now defaults to stride 2
(16 ms/frame end to end, 62 Hz) and exposes it. Costs nothing detectable: a
0.2 m trunk at 4 m spans ~11 px, so every second pixel still puts five samples
across it.

 An earlier
version of this entry claimed the real map and planner had been fed real depth.
They have not. What was confirmed is that CMake now finds the SDK and the LIVE
button is enabled — the pipeline itself is unexercised on real frames and no
number has come out of it. Correcting rather than deleting, because "we tested
this" written into project memory is exactly the sort of thing that quietly
justifies a decision six weeks later.

The `--live` button had been reporting "this build has no RealSense SDK" on a
machine where the SDK was installed and the Viewer worked. Not a missing SDK —
`find_package(realsense2)` searches `<prefix>/lib/cmake/realsense2`, and the
installer puts it one directory deeper, inside `Intel RealSense SDK 2.0/`. Fixed
in `cmake/find_realsense.cmake`, with a search-path test.

**Next, and it is the one that replaces assumptions:** record a walk
(`d435i_probe.py --record 600 --record-every 6 --emitter off`, which writes both
the `.npz` and a `.kdr`), then `--replay` it for sigma_d on real bark and
`voxel_live --replay` for the map's behaviour on it. That gives the first honest
`Z_max` and the first real valid-return fraction — the two figures the whole
range budget currently rests on as assumptions (sigma_d = 0.25 px, and a depth
model fitted to one screenshot).

## 2026-08-11 — stereo occlusion shadow: real, now modelled, and negligible for us

Observed on the real camera: a hand held up casts a shadow in the depth image.
That is the classic stereo occlusion band — every foreground object hides a
strip of background from the SECOND imager, so a band down one side returns
nothing. Width is exactly the disparity difference:

    shadow_px = f*B*(1/Z_near - 1/Z_far)

`depth_camera.cpp` modelled only the FRAME-EDGE band (`u < fB/t`), not this.
Now it does, via one right-to-left sweep per row — the same left-right
consistency test a real matcher runs, which is why the hardware produces exactly
this artefact rather than merely something like it. `modelOcclusion=false`
restores the old renderer bit-identically, so earlier numbers stay comparable.

`test/occlusion_check.cpp` pins the SIDE as well as the width. The side matters
more than it looks: a correct-width shadow on the wrong side would look entirely
plausible and would mirror every obstacle boundary in the map, biasing the free
space beside every trunk in a direction nothing downstream could detect. It
falls LEFT of a near object, because the right imager sits at +X. Measured 6 px
against a predicted 7.0 px at 1.5 m / 4.0 m.

**And the point of measuring it: it barely matters at forest ranges.** Forest
world, 848x480, 87 deg, 50 mm baseline, eight yaws:

    valid pixels   60.7 % with shadow   61.0 % without   -0.2 points

Because the effect scales with the disparity DIFFERENCE, and at forest distances
everything is far so that difference is small. A trunk at 4 m against 10 m of
background is a 3 px strip; a hand at 0.4 m against a 2 m wall is 45 px. **This
is a near-field phenomenon on a 50 mm baseline** — spectacular at arm's length,
almost invisible past 4 m.

Kept because it is correct, not because it changed anything. It would matter on
a wider baseline (a D455's 95 mm doubles it) or flying in close quarters.

## 2026-08-12 — does the 3D view turn? Yes, and the two panes cannot check it

Asked whether the chase view actually turns when the plan is not straight ahead.
It does — `--yawrate 25` puts a sharply left-curving green path in the chase
pane — but the obvious way to verify that is worthless, and worth writing down
before someone tries it: **the two panes that show the plan use different
references.** The top-down PLAN pane is drawn in WORLD axes and does not rotate
with the aircraft; the chase pane is drawn from an eye that does. At any heading
other than North the same path appears at two different angles, both correct.
Comparing them by eye proves nothing, and I spent a minute doing exactly that
and getting confused before writing the test.

`chase_turn_check` pins the invariant the chase view actually claims: **a path
with fixed BODY shape must land on the same pixels at every heading.** Rotating
the world and the eye must cancel exactly. Seven headings including negative and
non-multiples of 90: worst drift 0.0005 px. Partnered with assertions that would
catch a view satisfying it vacuously — left curves draw left, right draws right,
symmetrically, and a hard-left escape leaves the frame rather than being tracked.
Then a third part ties the drawing chain to the planner's own body→world
rotation, so the test cannot pass while those two disagree: forward distance
identical to six decimals at yaw 0, 90 and 200.

Three yaw conventions in that chain (planner rotation, chase eye placement,
renderer) and any one of them could invert without a single view looking wrong.

**The test's first version failed for a good reason.** I built a 60 m room to
plan in and every primitive was rejected at its first point. Not a bug — carving
stops at `r - carveSigK·σ` and `σ = Z²σ_d/(fB)` is **20 m at 30 m** on a 50 mm
baseline, so a wall that far away is worth nothing and the map correctly carved
nothing from it. Shrunk the room to 6 m (σ 0.81 m, carve reaches 4.4 m). A
reminder of how short the honest range really is: a room the size of a car park
is, to this sensor, entirely unknown.

**One real defect found by reading the chain:** `yaw += yawRateDps·dt` sat
immediately after `plan()`, so the map and the plan were built at one heading and
every view rendered at the next — a frame of rotation (0.8° at 25 °/s) between a
path and the map it was planned in. Under a pixel, and invisible in the picture,
which is the same reason the unsequenced `check(centroid(...))` argument was
worth fixing. Moved to after everything that draws.

## 2026-08-12 — a turn arrow on the first-person pane, and a wrap bug it found

Asked for a toggleable arrow saying which way to turn. Bearing tape plus caret
on the FPV pane, `a` to toggle. Justification is the same finding that produced
the chase view: the first-person pane cannot show a forward path, so it should
show the COMMAND instead.

Three states, and the middle one had been invisible. `--robot 2.0` gave
"64 admissible, free 0.63 m, cmd 0.00 m/s" — a direction exists, there is simply
no confirmed-free room to accelerate into. That is NOT the same as BLOCKED, and
rendering it as `AHEAD 0.0 m/s` looks like a display fault rather than the speed
budget doing exactly its job. Now `STOPPED  0.6 m free`, with `HOLD` reserved
for nothing-flyable (verified with `--robot 6.0`).

**The bug the test found, which is the real content of this entry.** The HUD
needs a signed relative bearing, so I hoisted the planner's file-static
`angDiff` into the header as `angDiffDeg` rather than write a second copy of a
convention that inverts silently. Then asserted its range over ±4000° — and it
failed, returning 360.

`fmod(a - b + 540, 360) - 180` is only correct while `a - b` stays inside ±540,
because fmod keeps the sign of its argument. Safe in the planner, where both
arguments come out of atan2 and are bounded. **Not** safe for the HUD, whose
second argument is `yaw`, which `voxel_live` accumulated without ever wrapping:
at 25 °/s for two minutes yaw is 3000°, and the function returns −480 — the
aircraft told to turn the wrong way, on a picture that would have looked
entirely plausible. Fixed both ends: fmod first then one correction, and yaw
wrapped to [0, 360).

Nothing about the rendered arrow would ever have revealed this. It was found by
asserting a range, which is the argument for testing conventions rather than
appearances.

## 2026-08-12 — a LINEAR depth ramp cannot show near and far at once

Reported from the street scene: "the lamppost is only visible at the furthest
scale setting -- **even in the depth vision**". That is the sharpest instrument
finding of the day, because it is about the DISPLAY, not the map: the sensor had
measured the lamppost the whole time and the ramp could not show it.

The arithmetic is unavoidable. At a 10 m ramp everything past 10 m saturates to
one blue, so a pole at 12 m and a wall at 30 m are the same pixel value. At a
20 m ramp the near field collapses into a few shades of red instead. **One
linear ramp cannot serve both ends**, and no choice of maximum fixes it.

Equalisation does: build a 256-bin histogram of the valid depths, map through the
CDF, and colour is allocated where the DATA is rather than uniformly over a range
that is mostly empty. That is exactly why the RealSense Viewer's own display
looked so much more informative than ours -- it defaults to equalised, which I
noted earlier as "not real detail, just display". Half right: it is not extra
data, but it IS extra visible information, and dismissing it was wrong. `h`
toggles it.

**And a bug I introduced yesterday, caught the same way.** Because `dMax` follows
the coarsest layer when `depthMaxM` is 0, pressing `f` moved the map range AND
the picture's colour scale together -- **two variables on one key**, which makes
the very comparison you press it to make impossible. `f` now latches the display
scale first; `0` returns it to automatic.

That is the third time today an instrument has been the fault rather than the
system, after the 8 m ramp and the 12 m slice crop. The pattern is worth naming
properly: **a display that derives from a parameter you are sweeping is not an
instrument, it is a second experiment running at the same time.**

Confirmed in the same session: `--cell 0.25 --farcell 2` made the roof appear,
which pins the earlier ladder-gap diagnosis.

## 2026-08-12 — the far layer was ERASING thin structure, not missing it

A street scene, 76 % valid, a lamppost plainly visible at ~10 m in the depth
image, and **not one occupied cell anywhere in the map.** Not a coarse
approximation of the lamppost -- nothing.

`fp.carveWinPx = 0`, with the comment "the min-filter is fine-scale". **That had
it exactly backwards.** The local-nearest-return clamp matters MORE on a coarse
layer, not less: an 8 m cell is far more likely to contain a thin object
surrounded by rays that miss it, and the log-odds arithmetic is merciless --
one hit at +0.85 against a hundred carves at -0.40 clamps the cell to -4, FREE.

Which is the failure `voxel_map.hpp` already documents, in its own words:

> "The map is not failing to see the trunk; it is actively claiming the trunk's
> cells are empty, because the pixels either side of it returned the background
> and the DDA carved right through."

The guard against exactly that was switched off on the one layer responsible for
everything past 3 m. Now `max(5, integrateStride*2 + 1)`, scaled by the layer's
own stride -- a 5 px window on a grid sampled every 4th pixel spans barely one
sample, so the default would have been nearly inert here too.

**Two other things the same frame shows, both by design and worth stating:**

* Anything past the far layer's `maxIntegM` (19.5 m at 8 m cells) can only ever
  be CARVED, never marked. A street's far end at 30-50 m will always read free.
  That is correct -- delta-Z at 30 m is about 10 m -- but it means a street scene
  is mostly free space by construction, not by fault.
* The ladder had a hole: `0.10 / 0.15 / 8.0` gives honest ranges 2.2 / 2.7 /
  19.5, so the middle rung covers **half a metre** and everything from 2.7 m out
  is 8 m blocks. Sweeping the two knobs independently pushed the rungs apart
  until the middle of the sensor's range was unserved. Sane default remains
  0.10 / 0.25 / 2.0.

## 2026-08-12 — the settings belong in the viewer, not on the command line

**Keys chosen for the keyboard they will be pressed on.** `[` and `]` were the
first choice and are wrong: on a Finnish layout they are AltGr+8 and AltGr+9,
which highgui may never deliver. `-` and `+` now, with `=` and the brackets kept
as aliases for a US layout, and `0` for automatic. Obvious in hindsight, and the
sort of thing that makes a feature quietly unusable rather than visibly broken.


Fair complaint: every knob added today was a CLI flag, which means killing the
session and losing the accumulated map to answer "what does the next value look
like". That is a bad loop, and it is the one being used most.

Live keys in the four-pane view now, listed along the bottom of the map slice:

| key | does |
|---|---|
| `f` | far layer: off / 1 / 2 / 4 / 8 m cells -> 0 / 7.1 / 10 / 14.2 / 20.1 m |
| `t` | ray stride 1 / 2 / 4 |
| `[` `]` | depth colour range down / up, `\` back to automatic |
| `r` | start / stop .kdr recording |
| `v` `a` `space` `s` `m` `q` | view, arrow, pause, save PNG, menu, quit |

`f` rebuilds only the coarse grid and its `CoarseLevel` entry, recentred on the
current pose, so the fine and medium maps survive the change. `t` just moves the
stride on all three layers. Neither restarts the session or drops the camera.

The startup flags all still work and are still what a scripted run uses; this is
about the interactive loop being interactive.

## 2026-08-12 — toggles for fixes: mostly no, and what to do instead

Asked whether today's fixes should be runtime toggles so they can be A/B'd. The
honest split is by KIND, not by convenience:

* **A trade gets a toggle.** Depth improver, sideslip, emitter, stride, farcell,
  preset, decay-when-it-comes. All genuine choices with a cost on both sides, and
  the project's convention -- off by default, with the measurement attached -- is
  already right.
* **A bug fix gets a TEST, not a toggle.** Shipping a known-broken code path
  behind a flag doubles the test surface and eventually someone flies with the
  flag in the wrong position. `overlay_align_check` already carries the before
  and after numbers permanently (896 -> 896 against 3605 for the ladder, 5 -> 0
  samples for minIntegM), which is better evidence than a runtime switch because
  it runs automatically and cannot rot.
* **A derived value gets a parameter with a documented disabling value.**
  `minIntegM = 0` restores the old behaviour exactly and the test exercises both
  paths; `depthMaxM = 0` means "follow the map". That IS the toggle, done the way
  that does not fork the code.

Banding is the one with no sensible toggle: there is no useful amount of
"nearest-hit". It is an arbitration rule, and the wrong one collapses the ladder.

**And the mechanism that actually serves the want: `--record FILE.kdr`.**

A runtime toggle on a live camera is **not a controlled comparison** -- you move,
the scene moves, and you compare two different inputs while believing you are
comparing two code paths. Record once, replay the same file through two builds,
and the input is identical by construction. `DepthRecordWriter` already existed
and was tested; `voxel_live` simply could not reach it. Now it can, and it is
also the `THESIS.md` section 4 deliverable "the raw .kdr recording of each run,
so the claim is re-checkable offline".

Round trip verified: 12 frames written at 848x480, reopened by `--replay` with
fx 446.8, ppx 423.5, baseline 50.0 mm intact.

## 2026-08-12 — what the D435i can actually be tuned for, from Intel's own docs

Searched for range-maximising settings. The short version: **there are almost
none, because range is baseline-limited and no setting changes the baseline.**
Two things did come out of it that are worth having.

**MinZ is a formula, not a guess.** Intel's:

    MinZ(mm) = focal length(px) * baseline(mm) / 126

which at 848x480 on a D435 gives ~16.8 cm and matches their published figure.
`minIntegM` now derives from it (`f*B/126 * 1.2` = 0.21 m here) instead of the
0.25 m I picked this afternoon, which was a sensible guess and nothing more.

**Disparity shift is the wrong lever and moves the wrong way.** Raising it lowers
MinZ *and lowers MaxZ*; shift 0 is the setting that preserves maximum range, and
that is already the default. Nothing to gain.

**848x480 is already the optimum.** The whitepapers say depth is computed at that
resolution and anything higher is extrapolated after rectification, so 1280x720
gives similar or slightly worse depth. We are on the right setting by luck.

**High Accuracy is the preset our doctrine asks for**, in Intel's own words:
stricter criteria, only high-confidence depth, "very good for autonomous robots
where false depth is a concern" -- against High Density, which fills holes at the
cost of confidence. That is exactly "a hole is safe, a confident wrong depth is
not". `Preset: Custom` on the bench means nobody has ever chosen.

**The only real range lever is sigma_d, and it is the one we have never
measured.** Z_max = sqrt(cell*f*B/sigma_d), so range scales as 1/sqrt(sigma_d):
taking sigma_d from the assumed 0.25 px to 0.15 px would buy **29 % more range at
every cell size, for free.** Temporal filtering and the High Accuracy preset both
attack sigma_d directly. That makes the sigma_d measurement, already outstanding,
the highest-leverage item on the sensor side rather than a loose end.

One claim from the forums I am NOT repeating as fact: a garbled line about laser
power and subpixel error, ~30 % in some direction. Directionally unclear and
easily measured here with the emitter A/B, so it belongs in the measurement queue
rather than in the notes as a number.

Sources: dev.intelrealsense.com tuning guide and D400 visual presets pages,
librealsense issues #8104 / #11180 / #6207, and the BKM tuning whitepaper.

## 2026-08-12 — --farcell 8 made the ladder bug FAR worse, and hid it as "no range"

Reported: "even on farcell 8 the voxels don't appear far at all. Also, now they
don't appear close either!" Both halves are the same fault, and raising farcell
amplified it.

The first-person pane in those frames is not distant structure. It is **two or
three 8 m cells seen from inside one of them.** Under the old nearest-hit rule a
coarse cell's near FACE wins against every finer level, and an 8 m cell's face
can sit a metre away, so the pane fills with a handful of enormous slabs at
fixed distance -- looking neither near nor far, because it is a single cell
boundary rather than a scene. At `--farcell 2` some fine detail still showed
through; at 8 it is buried completely. **Turning the range up made the picture
worse, which is exactly backwards and is why it read as "no range".**

Fixed by the banding change (fca8cca), which was pushed before these frames were
taken and is not in that build.

**And a second instrument scaled for an older configuration**, in the same day as
the 8 m depth ramp: the MAP SLICE crop was hard-coded to +-12 m, chosen when the
far layer reached 10 m. At `--farcell 8` the map is honest to 19.5 m, so the one
pane that shows EXTENT was cropping off everything past 12 m. Now derived from
the coarsest honest range (0.65x, so the wedge fills the pane), with the range
rings spacing themselves 1/2/5 m so a 20 m pane does not get ten of them.

Worth naming the pattern, because it has now happened twice in one session:
**every hard-coded display constant is a lie waiting for the configuration to
move.** The depth ramp, the slice crop. Both were correct when written, both
silently became wrong when `--farcell` grew, and both made the map look worse
than it was.

## 2026-08-12 — first outdoor forest frames, and an instrument fault they exposed

Real trees, handheld, `--farcell 8` (honest to 19.5 m). The stack works: the
command arrow tracks, the three states all fire, and one frame reads
`RIGHT 90 deg  STOPPED  0.3 m free  CLIMB` with the arrow clamped at 80 deg and
the pinned-tick showing the angle is a floor rather than a reading. That is the
HUD doing its job on a case the sim never produced.

**Numbers, first outdoor data in the project:**

* valid fraction **74-85 %** outdoors against 97 % indoors. Sky returns nothing
  and the projector buys little at range. The grey is honest.
* integrate **21.4-24.4 ms** outdoors against 17.5-18.6 ms indoors. Two variables
  moved at once so it is not controlled, but the direction is diagnosable:
  **fewer valid pixels and MORE time means the per-ray cost rose, so the rays got
  longer.** DDA steps scale as carve-distance / cell -- indoors a 2 m return
  carves ~1.5 m through 0.25 m cells (six steps), outdoors an 8 m return carves
  ~6 m (twenty-four). The far layer at 8 m cells is nearly free; the MIDDLE layer
  got dearer because the scene is deeper.
* **Consequence for THESIS P1: the Pi benchmark must run on OUTDOOR data.**
  Benchmarking the indoor recording would have flattered the headline number by
  about 30 %.

**The instrument fault.** The depth pane's colour ramp was hard-coded to 8 m. The
RealSense Viewer at a 16 m scale shows plenty of real returns at 10-16 m in a
wood -- on an 8 m ramp every one of them is the same blue, so **the pane
saturates below where the question starts** and cannot answer "does the far field
have data". Now derived from the coarsest layer's honest range, so it grows with
`--farcell`, with `--depthmax` to override. The banner prints the range it used.

Also confirmed from the Viewer's panel: resolution matches ours exactly
(848x480, 30 fps, Z16), so the earlier indoor "more detail" was not resolution.
Two things do differ: the Viewer had **Emitter: Laser ON** while `voxel_live`
defaults it off, and its **Preset was "Custom"** -- meaning nobody has ever
deliberately chosen between High Accuracy and High Density, which is precisely
the holes-versus-lies trade this project's doctrine rests on. `rs2_set_option` is
already in the runtime loader, so setting it is a flag rather than new API.

And a calibration worth recording: the Viewer's OUTDOOR image is about as holey
as ours. The indoor case where it looked much better was largely post-processing
plus histogram equalisation on a scene that was mostly valid anyway. That lowers
the urgency of the filter-chain work in THESIS P2 relative to what the first
comparison suggested.

## 2026-08-12 — CORRECTION: the blocked pane was my ladder, not the odometry

I blamed the missing odometry. Wrong, and the evidence was in the same
screenshot: the MAP SLICE shows the cells around the camera **white — free**.
There was no accumulated shell. The depth image was clean at 97 % valid. It was
a bug I introduced this morning.

**Cause: nearest-hit arbitration across the ladder.** `renderLadder` took the
nearest hit across all levels, on the argument that "where two levels both know
about a surface the finer one is in front of the coarser one's blocky
approximation and wins on its own merits."

**That argument is false, and the failure is systematic rather than occasional.**
A cell of size `c` containing a surface at distance `d` has its near FACE
somewhere in `[d - c, d]`, so a coarse level's hit arrives on average **half its
cell size early**. Against a 0.25 m level, an 8 m level wins by ~4 m — which is
to say it wins *always*, wherever it has any data at all.

So nearest-hit does not blend the levels, it **collapses the ladder to its
coarsest rung**. The tell, reported independently: "the voxels all look the same
size." They did. Three levels were being integrated and one was ever drawn. Indoors, where everything is within a few metres, a single 2 m cube
subtends 50 deg or more and a handful of them fill the pane. Exactly the reported
symptom: "it's like there's a voxel in the way."

And the second report — *move very close and you see voxels behind* — is the
same mechanism, not carving as I first said: getting inside the coarse cell's
near face lets the fine level's hit finally win.

**Fix: band each level by its own honest range, finest first.** Fine owns
0–2.2 m, medium 2.2–3.5 m, coarse 3.5–9.8 m. A coarse level is never consulted
inside a finer one's range. **The planner already had this rule** —
`voxel_traj.cpp` marches "fine first ... if (t <= c.rangeM)" — so the renderer
was the odd one out, and I wrote the wrong rule while the right one was three
files away.

Pinned: the coarse level cannot intrude on the fine level's range (896 -> 896 px
unchanged), the ladder still gains what fine cannot reach (896 -> 1223), and it
does **not** inherit the coarse near field (1223 vs 3605 coarse-alone). Note the
ladder is deliberately no longer the union — the old test asserted that it was,
and it caught this change, which is the test doing its job.

The lesson is the one this project keeps relearning: **a rule that sounds right
in a comment is not a measurement.** I wrote that justification into the header
this morning and never rendered a scene close enough to falsify it.

## 2026-08-12 — real camera: voxels appear AT THE EYE, and why

Two things at once on live frames, and only one of them is a bug.

**[SUPERSEDED — see the correction above. The blocked view was the ladder's
nearest-hit rule, not this. Kept because the reasoning below is still true of a
moving handheld camera, just not the cause of what was observed.]**

`voxel_live` assumes the pose is FIXED. Over 900 handheld frames every surface ever pointed at gets
painted at its measured range from that one assumed origin, so a shell of
occupied cells accumulates around the camera at every radius ever measured. The
first-person raycast then terminates at t ~ 0 in every direction: a solid red
pane, red because the height key says "at your altitude" and the shell is, with
no distance haze because there is no distance. Faint bands above and below are
the same shell a little higher and lower.

And the tell that confirms it: **moving the camera very close to something
reveals what is behind.** That is carving working correctly -- a return at 0.3 m
carves 0 to 0.3 m along its ray and punches a hole through the accumulated
shell. Not a fix, but it is the only mechanism that can undo the damage.

**The real bug: there was no minimum trusted range.** Any return was marked
OCCUPIED if `r <= maxIntegM`, including returns inside the sensor's own minimum
where it is not measuring anything. Holding a hand near the lens produced
flickering voxels at the eye, which is exactly the signature.

Inside the minimum range a stereo camera does not fail quietly, it fails
CONFIDENTLY: the disparity is past the matcher's search range, the occlusion
band is `f*B/Z` = 149 px at 0.15 m (18 % of an 848-wide frame with no
counterpart in the right image at all), and a close surface saturates both
imagers with the projector. That is the speckle class `depth_camera.hpp` already
warns about -- "a hole is safe, a confident wrong depth is not".

**Why it flickers, quantitatively.** `lHit/lMiss` = 0.85/0.40, so one spurious
hit cancels two carving passes: a spurious rate above roughly **one frame in
three** holds a cell OCCUPIED indefinitely. Near-field garbage is far above that,
so it wins the argument -- in the cells the aircraft occupies, where
`sphereClear`'s core test then rejects every primitive at its first point. That
is the BLOCKED-at-start signature, and it would have happened in flight.

Fixed with `VoxelMapParams::minIntegM` (0.25 m; 0 restores the old behaviour).
Below it a return is treated exactly like an invalid pixel: **no mark AND no
carve**, because it is not a measurement. Pinned in `overlay_align_check`: a
0.12 m frame marks 5 sampled cells occupied without the gate and 0 with it, and
carves nothing either way.

Measured on real depth at 97 % valid with the full three-layer ladder:
**integrate 17.5-18.6 ms**, better than the sim's ~22 ms. Still a laptop, not a Pi.

## 2026-08-12 — the pale line across the FPV pane was correct, and useless

Asked what the horizontal line in the first-person pane was and whether the path
visualisation was broken. Bisected it by switching the two polyline calls off
behind env vars and re-rendering: the CANDIDATES draw it (`{210,170,120}` is
BGR, so pale blue), and it survives with the chosen path off.

It is not a bug. **Every level rollout lies in the horizontal plane through the
eye, and that plane projects to a single line.** A circular arc of radius R
leaving along the optical axis is at `(R(1-cos t), R sin t)` in (right, forward),
so `u = f*(1-cos t)/sin t = f*tan(t/2)` and `v = cy` EXACTLY, for every t. The
whole fan collapses onto one row: left turns sweep u one way, right turns the
other, and together they paint the full width. Correct, and meaning nothing --
which is much harder to spot than wrong, and it had been there since the first
version of this pane.

Same finding as the chase view, in its final form: from the aircraft's own eye a
forward path is a dot and a turning path is the horizon. **There is no
first-person drawing of a path that works.** Removed them from the FPV; they
stay in CHASE, where the eye is not in the plane they lie in. The aim point
stays, because it is a real point in space and projects like one.

The compass tape went too -- replaced with a single black arrow, direction =
commanded turn, length = commanded speed, white halo under black because the
pane swings from near-white fog to dark voxels within one frame. STOPPED draws a
stub rather than a confident arrow with 0.0 beside it.

Window: menu 1000x740, session 840x640, which is most of a 1366x768 laptop.
Scaled once at imshow with clicks divided back out, so all the layout maths stays
in layout pixels -- threading a scale through every caption offset is how text
metrics go wrong. Default `--ui 0.75`.

Also found while looking: the menu's row-2 explanation labels at x = 218 and
x = 530 were being painted over by the buttons beside them. Legible only because
nothing had been put in the right-hand column yet, and the new toggle put
something there.

## 2026-08-12 — correction: the brightness argument was for the wrong question

I argued that brightness cannot indicate free space because intensity goes as
albedo/Z^2. That is right for the IR projector and **wrong for passive RGB**,
which is what was actually being proposed -- along with far field rather than
near, which I also missed.

For an extended surface under fixed ambient light, image brightness is
**independent of distance**: the inverse-square falloff of received power is
exactly cancelled by the pixel footprint growing as Z^2. The 1/Z^2 term exists
only when the illumination rides on the camera. (Sources smaller than a pixel do
dim -- so a thin branch appears as a dark thread against a bright gap, which
helps.)

And the idea is a real instrument. Foresters measure *canopy gap fraction* by
thresholding upward hemispherical photographs; sky segmentation is long
established in outdoor robotics. My alignment objection to RGB is also weakest
exactly where it would be used: parallax goes to zero with distance, so a 15 mm
depth-RGB offset subtends nothing at 20 m.

The fit I missed entirely: **the far field is where the map is worst.** Past
3.5 m we carve but never mark; past 10 m even the coarse layer stops. `farWeight`
currently picks a bearing from coarse occupancy density at ranges where stereo
has almost nothing to say. A sky/gap signal estimates the same quantity
independently and is strongest where the geometry is weakest -- the reverse of
the usual situation. And the far map's existing contract, *AWARENESS ONLY, NEVER
PERMISSION*, is already the right safety semantics for it, unchanged.

Survives: still one-sided but with the opposite polarity (bright means open;
dark means nothing, since a shaded corridor is both open and dark); exposure
must be locked or used relatively within a frame; and it dies at dusk under
canopy with no fallback, there being no visible-light projector.

`APPEARANCE_AND_BLOBS_PLAN.md` gains section 3.0 and the old argument is kept,
scoped to the active case where it is correct.

## 2026-08-12 — appearance and blobs: planned, and the polarity is a trap

Question raised: feed pixel intensity into contested cells, bright meaning
"probably free", especially in the far field — or, on second thought, blob
detection. Written up as `nav-sim/docs/APPEARANCE_AND_BLOBS_PLAN.md`. The short
version, because the first half is the kind of idea that looks obviously good:

**The polarity runs backwards.** Under active illumination intensity goes as
albedo/Z². A *bright* pixel with no depth return means something close and
textureless bounced the projector back — a blank wall, a smooth trunk. That is
the opposite of free, and it is the most alarming thing in a depth image. Only a
*dark* return-less pixel is ambiguous.

**And the ambiguity is worst where it kills us.** NIR albedos are perverse for
flying through woods: foliage has a very high NIR plateau (~50 % at 850 nm) while
bark is dark — which is why the sim already models `trunkTex` 0.30–0.75 and why
`bark_contrast` exists. So "dark ⇒ probably free" fails hardest on dark thin
branches at range, which is the documented dominant failure mode in both papers.
A rule whose error mode is aligned with the system's failure mode is not weak,
it is negative-value.

Intensity survives only as a **veto** — bright + no return ⇒ refuse to carve —
and honestly it buys little, because a return-less pixel already carves nothing
(`if (!(r > 0.f)) continue;`) and `carveWinPx` covers much of the neighbour case.

**Blobs are the real idea**, in two forms. (a) Connected components on the
INVALID mask, then explain each blob with the occlusion shadow we already model
and already test — read the occluder off its right edge, predict
`f·B·(1/Z_near − 1/Z_far)`, and if it matches, the blob's content is bounded
below by the occluder and can be carved. That turns "a dead strip beside EVERY
trunk" into a bounded claim using code that exists. (b) Thin structures: a 3 cm
branch is 3.4 px at 4 m and can never cross `occThresh` in a 0.25 m cell, but a
3-px-wide, 200-px-long component that is consistently nearer than its background
is overwhelming evidence. Mark by blob, not by pixel. That is the mechanism that
could change a flight outcome.

Blocked on two things already open: **σ_d is still assumed** (every threshold in
the thin-structure test is in units of it) and **`voxel_world.cpp` has no thin
branches**, so there is nothing to test (b) against.

Worth noting the earlier occlusion conclusion still stands and is not
contradicted: the shadow is negligible *as a hazard* on a 50 mm baseline past
4 m. (a) does not dispute that — it exploits the shadow as a *predictable
signal* in the near field, which is a different use of the same measurement.

## Open / unresolved

* **`PROJECT_CV.md`** — role, defensible claims, and the TODO list that makes
  them checkable: the four systems-engineering artifacts nobody has written
  (requirements baseline, budgets with margin, interface control, V&V
  matrix), a `docs/HARDWARE.md` for the airframe and power chain — the
  hardest integration work in this project and currently invisible in the
  record — and the demo overlay, since the constraint that makes this
  interesting does not appear in flight footage.
* **GNSS is CARRIED, LOGGED and EXCLUDED FROM NAVIGATION** -- not absent. That
  gives real-world ground truth for the drift-per-metre measurement that
  `POSE_AND_OPENNESS_PLAN.md` §5 assumed needed a simulator, and a geofence that
  never touches the planner. The risk: ArduPilot's EKF fuses a present receiver
  unless `EK3_SRC*` forbids it, and it would fly beautifully while voiding the
  claim. Verify with a connected-vs-unplugged controlled pair.
* **GNSS denial and video denial are CHOSEN constraints, not omissions** -- see
  `THESIS.md` §1.0. One scenario, the contested zone, generates both: the same
  emitters jam satellite and 5.8 GHz video, spatially rather than
  continuously. The crux is that RTL needs a position estimate and there is
  none, so onboard vision autonomy is the only remaining option rather than an
  enhancement; and because jamming is zoned, the required behaviour is
  *continue through*, not turn back.
* **`THESIS.md` is the direction document; this file is the lab notebook.** The
  thesis is autonomy on cheap CPU-only compute, where the *constraint is the
  claim* rather than a budget accident. Five pivots follow from it, and they
  reorder everything below: the Pi 5 benchmark is the headline experiment and
  not housekeeping (the central claim is currently unmeasured); algorithmic
  efficiency is the product, which promotes projective integration, an ESDF and
  the SDK's own filters from nice-to-have to the argument itself;
  `nav-sim`/`onboard` convergence is a blocker because the deliverable is a
  flight; everything specced on 2026-08-12 is deferred behind that flight; and
  ~2-3 m/s is a stated operating point derived from the sensor, not a flaw.
  `THESIS.md` §4 also sets the first definition of done this project has had.
* **`nav-sim/docs/POSE_AND_OPENNESS_PLAN.md`** — the state-estimation and
  bearing-space thread, planned and deliberately not built. Angular far-field
  openness map (32 kB, replaces `Mfar` and the `farOpen` march, accumulates on
  ATTITUDE ALONE because rotating an angular map is an index shift); the far
  field supplying `goalAzDeg` instead of `farWeight`, which removes a tuning
  weight rather than adding one; decay per metre travelled, bracketed between a
  2-3 s turn and 5-12 s of drift; an 8^3 tile index that makes the decay pass
  affordable, since ~1% of the map is ever observed; 3-DOF correlative scan
  matching (local SLAM, explicitly no loop closure); and IMU attitude, whose
  scaffolding turned out to be mostly present already.
* **Two sim measurements decide that whole plan** and need no hardware: sweep
  decay length against TRUE pose for the memory lower bound, then measure the
  matcher's drift per metre against truth for the upper bound. If the bracket
  closes, the answer is a shorter memory, not more estimation.
* **Nothing in this project has ever been timed on a Pi 5.** 22 ms integrate,
  1 ms plan, ~2 ms estimated scan match are all dev-box. A Cortex-A76 on
  memory-bound work is plausibly 2-3x slower, which would put mapping alone at
  50-60 ms inside a 100 ms cycle. An afternoon, and it reprices everything.
* SMF-VO unread. Re-check when arXiv is reachable; if it holds, it becomes the
  state-estimation plan.
* **`depth_camera.hpp` treats `subpixelPx` as a CONSTANT (0.25 px).** It is not.
  Real disparity noise scales with texture contrast, and the sim currently makes
  texture a binary gate (`texThresh` decides match/no-match) with a fixed noise
  on everything that survives. So low-contrast bark that *does* match is modelled
  as being as precise as a resolution chart, and `Z_max` is optimistic wherever
  the scene is marginal — which is most of a forest. Also preset-dependent:
  high_accuracy and high_density are different `subpixelPx` values on the same
  hardware. Measure both with `d435i_probe.py --preset sweep`, then make the
  noise a function of texture rather than a constant.
* **No thin obstacles in `voxel_world.cpp`.** Trunks only. This is now blocking:
  it is the reason the depth improver cannot be evaluated on the case it exists
  for, and it flatters every other result too, since branches are what a forest
  actually hits you with.
* Sideslip at 20°: 12/16 seeds better, mean +0.007 (t=+0.44). Wins often, loses
  big — two seeds cost 0.09 and 0.18. Off by default. **Lead worth chasing:**
  voxel_sim slaves heading to course, so a sideslipping aircraft turns to look
  where it is sliding. That coupling is a simulator artefact and does not exist
  on ArduPilot, where yaw and course are independent — so this may be measuring
  the sim rather than the idea.
* **FC hardware not confirmed against the ArduPilot board list.** H743 is the
  safe target; F405 is flash-squeezed on 4.4+. This is the expensive-to-undo
  decision and it is still open.
* Non-GPS ArduPilot setup (`EK3_SRC*`, pre-arm checks) is unwritten. GUIDED will
  refuse to engage until it is done, and the symptom looks like a broken link.
* Earlier sweep command lines were never recorded and the baseline has drifted
  (0.794 vs 0.761 for the same nominal arm). Record the full command line with
  every table from now on.
* `nav-sim/` and `onboard/` share **zero** code (`grep VoxelMap onboard/` → nothing).
  The stack we measure is not the stack that would fly. Resolve during the lean-out.
* Stereo camera: Waveshare IMX296 M12 mono is the candidate (see above).
  Baseline drives `Z_max` directly (12 cm → ~8 m; D435i's 5 cm → ~2–3 m).
  **Open:** whether two can be hardware-synced on a Pi 5 with the current
  driver. This gates the whole stereo path — resolve on the bench first.
* Lens not ordered. 4 mm M12 recommended; verify what the Waveshare bundle
  ships with, since the part number hints at a much wider lens.
* Stereo matching cost on a Pi 5 CPU is unbudgeted. 728×544 with ~96 disparities
  is likely too slow for OpenCV SGBM inside a 100 ms cycle. Measure before
  assuming a resolution.
