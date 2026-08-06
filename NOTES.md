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

## Open / unresolved

* SMF-VO unread. Re-check when arXiv is reachable; if it holds, it becomes the
  state-estimation plan.
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
