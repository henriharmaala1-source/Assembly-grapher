# Related work — read in full, compared against our measurements

Three papers. Two were read end to end from PDF; the third could not be
retrieved and is summarised at abstract level only, which is marked where it
matters.

| | what | read |
|---|---|---|
| **DeFoP** | arXiv 2512.17553v1, Del Col, Karjalainen, Hakala, Zhang, Honkavaara (FGI / National Land Survey of Finland) | full text, 47 pp |
| **Karjalainen** | arXiv 2501.12073v4, Karjalainen, Koivumäki, Hakala, Muhojoki, Hyyppä, George, Suomalainen, Honkavaara (same institute) — Int. J. Remote Sensing, Dec 2025 | full text, 37 pp |
| **SMF-VO** | arXiv 2511.09072, Yang, Yoon, Jung, Lim | **abstract level only** — arXiv is blocked at this environment's proxy (403 on every mirror tried). Everything below about SMF-VO comes from search-result summaries and must be re-checked against the paper. |

Both FGI papers fly under boreal canopy in Finland. Karjalainen and Honkavaara
appear on both. Their earlier system is one of the baselines DeFoP beats.

---

## 1. DeFoP — what it actually is

A **map-free, learned reactive avoider** with a geometric safety net:

```
D435i depth 480x270 @30Hz
  -> "depth improver" (local interpolation, near field only)
  -> 7-layer conv autoencoder -> 128-d latent
  -> Collision Prediction Network (ensemble of 3, LSTM, Unscented Transform)
       scores 256 motion primitives
  -> geometric safety supervisor vetoes primitives from raw depth
  -> velocity command -> MAVROS -> PX4 (Pixhawk 6C Mini)
VINS-Fusion for state estimation, on the same camera's IR stereo pair.
Jetson Orin NX. 0.4 m frame, 0.96 kg.
```

Modules at 30 Hz; the CPN issues velocity commands at 10 Hz.

**There is no occupancy map anywhere in this system.** No voxels, no
free/occupied/unknown, no carving. The scene representation is a 128-d latent
vector of the current frame plus whatever the LSTM retains. That is the single
biggest architectural difference from us, and it explains most of the others.

### Results, as reported

60 m corridors, 15 flights per configuration, Paloheinä (Helsinki).

| environment | density | method | success |
|---|---|---|---|
| difficult (baseline) | 2220 trees/ha | DeFoP @1.0 m/s | 15/15 |
| difficult | | DeFoP @1.3 m/s | 15/15 |
| difficult | | SEVAE-ORACLE (Nguyen 2024) | 5/15 |
| difficult | | SEVAE fine-tuned (Del Col 2025) | 8/15 |
| difficult | | LiDAR: LTA-OM + IPC (Karhunen 2025) | 15/15 @0.75 m/s, **5/15 at higher speed** |
| difficult | | vision: Karjalainen 2023 | 9/19 |
| medium | 1040 trees/ha | DeFoP @1.0 m/s | 15/15 |
| very difficult | 2000 trees/ha | DeFoP @1.0 m/s | 12/15 |

### Reviewer's note: Table 2 is wrong

The prose and the table disagree, and the table looks like a copy-paste slip.

* Table 2(b) "Medium Forest" gives DeFoP `59.0454 ± 1.2942 / 0.8183 / 0.9022` —
  **byte-identical to the baseline row in 2(a)**.
* §3.2's prose for the medium forest says `58.51 ± 0.40 / 0.923 / 0.957`.
* Table 2(c) "Very Difficult" gives `58.5077 / 0.9227 / 0.9571` — which is
  §3.2's *medium* numbers.
* §3.3's prose for very difficult says `52.3 ± 14.3 / 0.78 / 0.90`.

So rows (b) and (c) are each shifted up one environment. Separately, §4.2 claims
"full reliability was only achieved by the proposed method (15/15 flights)"
while discussing the very difficult forest, where §3.3, §4.1 and the conclusion
all say 12/15 (80 %). The prose numbers are self-consistent and the table is
not; use the prose.

This does not change the conclusion — the success rates are the headline and
those are consistent — but it is a reason not to quote their velocity or
smoothness figures to three decimal places.

---

## 2. Where they independently arrived at what we built

This is the useful part. We did not read this paper before building; the
convergences are independent.

| DeFoP | ours | verdict |
|---|---|---|
| **256 motion primitives**: forward vx × 8 vz × 32 yaw-rate bins, *"precomputed in the body frame and transformed into the world frame"* | `voxel_traj.cpp`: 195 arcs, 13 yaw × 5 climb × 3 speed, precomputed in body frame at construction, rotated by heading at runtime | **identical architecture.** Same reason, too: the rollout integration cannot be afforded per step. |
| Lateral velocity added as an enhancement, *"not discretized but dynamically determined... set to the arctangent of the yaw command"*, to "execute large, fast curves with improved responsiveness" | escape primitives — pure body-frame translation, heading unchanged, `nEscape = 8`, `escapeMinDeg = 60` | **same insight, different use.** They exploit holonomy to *widen turns*. We exploit it to *retreat*. Theirs is continuous and coupled to yaw; ours is a discrete set. Theirs is better for smoothness; ours is what got the aircraft out of the cul-de-sac (562 → 59 stopped steps). These are complementary, not competing. |
| *"frequent indecision at the planning level... rapid oscillations in the selected yaw direction. The system monitors the **sign change frequency** of yaw commands over a temporal window... a bias is applied to favor directional consistency"* | our `reversals` metric is literally the yaw-command sign-change count; commitment, `switchMargin`, `revPenalty`, and the reference-bearing low-pass all attack it | **same failure, same detector, same remedy.** Worth noting they report it as a *heuristic they had to add*, exactly as we did — and, like us, they do not quantify how much it bought. |
| Speed budget: they train the CPN with an *expanded* collision margin so it "develops a conservative bias" | explicit stopping-distance budget `d = v·t_react + v²/2a` against **confirmed-free** length only | **different mechanism, same intent.** Ours is a closed-form guarantee you can audit; theirs is a learned prior you cannot. Theirs generalises to shapes ours cannot represent. |
| *"the drone may enter a dead-end condition, causing it to stop and rotate in place while searching for a safe direction"* | exactly what our cul-de-sac world reproduced | **they have our open bug too**, and they did not solve it either. Fig. 6(c) shows loop-like escape manoeuvres in the very difficult forest, i.e. the recovery is emergent, not planned. Neither system has a global planner. |
| Geometric safety supervisor: depth split into yaw/vertical sectors, sector blocked if a fraction `ε` of pixels are within `d_min`; blocked span expanded by `⌈arctan(r/d)/δ_yaw⌉` sectors for vehicle radius | swept-sphere clearance test over the whole ball, exhaustive cell scan, `robotR = 0.6` | **same job, cruder input.** Their supervisor works on the *raw depth image*, not on a map — so it can only veto what is currently in frame. Ours works on accumulated occupancy, so it also rejects paths into things we saw ten seconds ago and can no longer see. That is a real advantage of keeping a map, and it is the reason we can have escape primitives at all: retreating into unmapped space would be indefensible. |

The paper's own future-work section reads as a description of our design:
*"the present planner operates using a discrete motion primitive library, which,
while computationally efficient, limits system flexibility."* We are at the same
point in the same design space, having got there from the opposite direction.

---

## 3. The number that reframes everything: their sensing range

The depth improver *"applies a kernel around regions where obstacles are
detected closer than **2 m**"*, and the caption is explicit that the refinement
is deliberately local: *"Undefined pixels in the vicinity of obstacles are
reconstructed, while regions far from obstacles remain undefined."*

Two metres is not a tuning choice. It is roughly the honest range of their
sensor, by our own formula.

D435i, depth at 480×270, HFOV ≈ 87° → `f ≈ 240/tan(43.5°) ≈ 253 px`;
baseline `B = 0.05 m`; so `f·B = 12.65 px·m`. With
`Z_max = √(cell · f·B / σ_d)` and the ~25 % derate we measured:

| σ_d | cell 0.10 m | cell 0.25 m |
|---|---|---|
| 0.25 px | 1.7 m | 2.7 m |
| 0.15 px (active stereo helps) | 2.2 m | 3.4 m |

**Their trustworthy depth is 2–3.5 m.** Ours, on a 12 cm baseline at 320×240, is
`maxIntegM = 8 m` — measured, not assumed (false-free cells: 0.07 % at 6 m,
2.95 % at 8 m, 13.22 % at 25 m). A 12 cm baseline is 2.4× a D435i's, and it
shows.

The consequences are worth stating plainly, because they answer a question
asked earlier in this project about a competitor's screenshots showing distant
trunks:

1. **A published, field-proven, 100 %-success forest autonomy stack operates on
   ~2 m of trustworthy depth at 1.0 m/s.** Long-range perception is not what
   makes it work.
2. At 1.0 m/s our own speed budget wants `0.25 + 1/6 = 0.42 m` of stopping
   distance; at 1.3 m/s, `0.61 m`. Both fit inside 2 m with room to spare. The
   sensing range and the flight speed are matched, and that matching is the
   design.
3. So the 25 m horizon we keep failing to plan around (issue #20) is a problem
   we *invented*. Nobody flying successfully in a forest has it.

This does not mean range is worthless — it is exactly what would let us solve
#20 rather than route around it — but it removes range from the critical path.

---

## 4. Where they are ahead of us, honestly

* **Real flights.** 60 m, 15 repetitions, three stands, GNSS-denied, fully
  autonomous takeoff to landing. We have a simulator whose plant model is
  defined by the same `tau` the planner predicts with (see `CONTROL_PLAN.md`).
  Every number in this repository is downstream of that circularity.
* **State estimation exists.** VINS-Fusion on the camera's own IR stereo pair,
  Kalibr + VINS online extrinsic refinement. We have measured that we need
  ~0.1 m/s velocity accuracy and have built nothing that provides it.
* **Uncertainty is propagated.** 3-model CPN ensemble plus an Unscented
  Transform over the input state. We have no uncertainty representation beyond
  three-state occupancy.
* **Active illumination.** The D435i's IR pattern projector is on, *"to enhance
  scene texture in low-feature regions."* Our whole silhouette-boost model
  exists because we assume passive stereo on untextured bark.
* **Compute.** Orin NX with TensorRT. CPN inference went 4 Hz → 10 Hz on
  TensorRT alone. We are targeting a Pi 5 CPU at ~16 ms/step.

Where we are ahead: we have a persistent map with an explicit unknown state, an
auditable safety argument that does not depend on a learned prior, and a
measured false-free rate (0.003 % after the carve guard, from 7.814 %). DeFoP
cannot state a comparable number because it has nothing to state it about — its
supervisor is a per-frame veto, and §4.1 concedes the CPN *"occasionally
proposes unsafe velocity commands"*, which is why the supervisor exists.

---

## 5. The two papers contradict each other on the IR projector

This is the most actionable single finding, and it is only visible because both
papers are from the same institute flying the same camera in the same forests.

* **DeFoP (§2.1.1):** projector **on** — *"An infrared pattern projector is used
  to enhance scene texture in low-feature regions, improving correspondence
  reliability under challenging lighting conditions."* And VINS-Fusion runs on
  that same IR stereo stream.
* **Karjalainen (§3.2):** projector **off** — *"the dots of the laser emitter
  cause problems for VIO since they are moving with the camera. Therefore, in
  this study, the infrared emitter was disabled. Since sunlight contains
  infrared light, which interferes with the artificial features emitted, and
  forests typically do not contain homogeneous surfaces, disabling the emitter
  should not affect the camera performance in outdoor conditions."*

Karjalainen's objection is not a matter of taste: a projected pattern is fixed
in the camera frame, so a feature tracker locks onto it and reports zero motion.
DeFoP feeds the IR stereo pair to VINS-Fusion *with the projector on* and does
not mention the interaction at all. Either they solved it, or their canopy is
dark enough that the pattern is weak relative to real texture, or it is a latent
problem in their pipeline. The paper does not say.

**What this means for us.** If we ever consider active stereo to fix the bark
texture problem, we cannot use the same imager for both depth and VIO without
resolving this. The clean answers are: separate cameras, or alternate the
emitter frame-by-frame (RealSense supports this), or accept passive stereo and
keep the silhouette model — which is what we are doing.

---

## 6. Karjalainen 2501.12073 — the drift numbers we needed

Cheaper and closer to us than DeFoP: Jetson Orin NX, **Holybro Kakute H7** FC,
RealSense D435, 330 mm frame, 791 g dry / 1153 g with batteries, EGO-Planner-v2
(MINCO trajectories, probabilistic occupancy grid with circular buffers,
A* global search) + VINS-Fusion + `px4ctrl` tracking controller.

Flights: Evo, Finland. 34–42 m goals, 1 m/s target.

| site | density | success | smooth (no emergency stop) |
|---|---|---|---|
| Evo-medium | 650 trees/ha | 7/7 | 5/7 |
| Evo-difficult | 2000 trees/ha | 8/9 | **0/9** |

*"The reason for emergency stops in both environments was the late detection of
thin and dry spruce branches."* Every single difficult-forest flight needed at
least one emergency stop. This is the same wall DeFoP hits and the same one our
trunk-visibility work is about.

**VIO accuracy against photogrammetric ground truth (ATE, position):**

| | constant t_d | online t_d |
|---|---|---|
| Evo-medium (5 flights) | 0.43–0.56 m, mean 0.50 | mean 0.47 |
| Evo-difficult (3 flights) | 0.27–0.39 m, mean 0.34 | mean 0.33 |

Over 34–42 m paths that is ~1 % of distance travelled, and crucially the error
is *systematic*: *"VINS-Fusion accurately captured the trajectory shape but
systematically underestimated the total flight distance."* A scale error, not
noise — which they attribute to calibration and to lacking hardware-level
camera/IMU sync.

Three things follow for us:

1. **~0.5 m of position drift over a 40 m flight is what a good open-source VIO
   on a good IMU actually delivers under canopy.** Our drift sweep should be
   re-read against that figure rather than against an aspiration.
2. Their fix list is ours: hardware sync between camera and IMU, better
   calibration, loop closure. Their loop-closure test in a homogeneous snowy
   spruce stand *did* find closures (20 pairs on a 340 m walk at default
   settings; 82 pairs when the minimum feature spacing dropped 40 → 30 px, at
   real compute cost). So loop closure is viable in forest, which is not
   obvious.
3. Post-processing beat real-time by 4.5×: *"the real-time trajectory error was
   50 cm, the post-processed 3D error was approximately 11 cm without any
   GCPs."* Irrelevant for control, decisive for mapping products.

Also worth stealing: their **failure taxonomy is honest**. Success = reached the
goal without a *major* collision; "minor touches to thin vegetation or branches
were allowed if the drone was able to continue." Our sim scores any contact as a
collision. That is stricter than the literature, and we should keep it, but we
should know the comparison is not like-for-like when we quote their 15/15
against our numbers.

Their other admission is a design bug we do not have: EGO-Planner-v2's virtual
floor and ceiling are **static relative to the takeoff point**, so low flying is
restricted to flat ground. Our map is egocentric and scrolls.

---

## 7. SMF-VO (2511.09072) — abstract level only

**Could not read the paper.** arXiv and every mirror tried returned 403 through
this environment's proxy. The following is from search-result summaries and is
flagged as unverified; the paper needs to be read before anything is built on
it.

Claimed: a "motion-centric" VO that estimates instantaneous **linear and angular
velocity** directly from sparse optical flow via a generalised 3D ray-based
motion field, with no explicit pose estimation and no landmark tracking. Works
across camera models including wide-FOV. Reported **>100 FPS (<10 ms/frame) on a
Raspberry Pi 5, quad-core CPU, no GPU**. ATE RMSE ≈ 0.13 m on EuRoC (third best
among compared methods), 2.89 m on KITTI; also evaluated on TUM-VI. Claimed 4–10×
faster than pose-centric pipelines at comparable accuracy.

Reported limitations: no loop closure, so unbounded drift on long trajectories;
depends on feature tracking, so it degrades in textureless scenes; needs stereo
or another cue for metric scale; depth from a short stereo baseline is noisy for
distant features and that noise enters the velocity estimate.

**Why this is the most operationally relevant of the three.** Our
state-estimation requirement is a *velocity* requirement — the drift sweep put
it at roughly 0.1 m/s — and SMF-VO estimates velocity **directly** rather than
differentiating a pose. It is the only one of these three that runs on our
actual target hardware, and the compute claim (<10 ms/frame) sits alongside our
~16 ms/step planner inside a 100 ms budget.

The obvious objection is equally important: a velocity-only estimator with
unbounded position drift is fine for *control* and wrong for *mapping*. Our
occupancy map is egocentric and scrolls, so it needs velocity to be right over
seconds, not position to be right over minutes — which is exactly the regime
SMF-VO claims. Whether that argument survives contact with the paper is the
first thing to check when arXiv is reachable.

Both FGI papers, by contrast, use VINS-Fusion, which is a full optimisation-based
VIO with loop closure and would not fit our budget.

---

## 8. What to take

Ranked by value per unit of work.

1. **Read SMF-VO properly and, if it holds up, make it the state-estimation
   plan.** Right hardware, right output (velocity), right cost. Everything else
   in our stack assumes a state estimate that currently does not exist.
2. **Continuous lateral velocity coupled to yaw rate**, alongside the discrete
   escape set. DeFoP's `v_y = f(yaw command)` widens turns instead of only
   enabling retreat, and it is a few lines in `voxel_traj.cpp`'s rollout. Cheap
   to try, measurable against our existing sweeps.
3. **Stop treating the 25 m horizon as a requirement (#20).** Nobody who flies
   successfully has it. Either solve the dead-end problem with a coarse
   topological memory of where we have already been — which our map already has
   and theirs does not — or accept stop-and-rotate as they both do, and say so.
4. **Selective near-field depth refinement.** Their depth improver only touches
   pixels near obstacles inside ~2 m and deliberately leaves the far field
   undefined. That is precisely our "unknown ≠ free" rule applied to the depth
   image instead of the map, and it is the correct place to apply it. Our
   silhouette-boost model already does something adjacent; making it explicit
   and range-limited would make it defensible.
5. **Do not adopt**: the learned CPN, the autoencoder latent, or the ensemble +
   Unscented Transform. All three need a GPU, a training pipeline, and a
   simulator good enough to train in — and their own supervisor exists precisely
   because the learned part cannot be trusted. We have the supervisor already;
   the learned part is what we would be buying, and it is the expensive half.

## 9. The one-line summary

Two independent field-proven forest stacks converged on the same planner
architecture we built — body-frame precomputed motion primitives, a geometric
safety veto, a yaw-oscillation heuristic, and no answer for dead ends — while
operating on 2–3 m of trustworthy depth at 1 m/s with a real state estimator we
do not have. The gap between us and them is not perception range and not the
planner. It is state estimation and flight hardware.
