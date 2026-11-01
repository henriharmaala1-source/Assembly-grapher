# Model landscape — every learned model that could plausibly run on this drone

A survey, organised by **the job it would do**, because "which model" is the
wrong first question and "what task is worth spending compute on" is the right
one.

> **All figures are from memory and are order-of-magnitude.** Parameter counts
> and FLOPs drift between variants, resolutions and papers. Verify anything
> before it goes in a thesis or a purchase decision. The verdict column is the
> part I stand behind; the numbers are there to justify it.

---

## The budget, so "too heavy" means something

Pi 5, four Cortex-A76 at 2.4 GHz, NEON with int8 dot product, no GPU, no NPU.
A well-optimised int8 model through ncnn or ONNX Runtime + XNNPACK should reach
a few GOPS sustained across the cores. The mapper already wants ~10 ms and the
planner ~1 ms of a 100 ms cycle.

Working budget: **~200 MMAC/frame at 10 Hz**, and that is *generous* — it
assumes the runtime swap in `ROADMAP.md` F10 has happened. Bands used below:

| band | meaning |
|---|---|
| **✅ fits** | under ~50 MMAC — runs without thinking about it |
| **⚠️ tight** | 50–200 MMAC — fits if it earns its place and nothing else competes |
| **🟥 heavy** | 200 MMAC – 1 GMAC — only at low rate, or if it replaces the mapper |
| **⛔ no** | over ~1 GMAC — not on this hardware, at any frame rate |

---

## 1. Monocular depth

**Task:** metres (or relative depth) per pixel from one camera.

| model | scale | verdict |
|---|---|---|
| MiDaS Small | ~20 M params | 🟥 already in `onboard`, never timed on Pi |
| Depth Anything V2 Small | ~25 M, ViT-S | 🟥 better quality, ViT is CPU-hostile |
| Depth Anything V2 B/L | 100 M+ | ⛔ |
| Depth Pro, Metric3D v2, UniDepth, ZoeDepth | large | ⛔ metric but seconds/frame |
| FastDepth | ~4 M | ⚠️ MobileNet encoder, built for embedded |
| Lite-Mono | ~3 M | ⚠️ self-supervised, modern, small |
| GuideDepth | ~6 M | ⚠️ |

**Verdict: measured dead in the far field.** Your mobile test plus the mechanism
(inverse-depth output + per-frame min-max normalisation) — see
`FAR_FIELD_MODELS.md`. Close field it works, and close field you already have
metric stereo. **This whole row buys nothing you don't have.**

## 2. Stereo matching

**Task:** disparity from the pair you already own — replacing SGBM.

| model | scale | verdict |
|---|---|---|
| StereoNet | ~0.4 M | ⚠️ designed for real-time, 4× downsample |
| AnyNet | small, anytime | ⚠️ genuinely interesting — anytime refinement |
| HITNet, RAFT-Stereo, CREStereo | large | ⛔ |
| **OpenCV SGBM** (classical) | — | ✅ the incumbent, unbudgeted on Pi |

**Verdict: the one depth row worth a second look.** Not because learned stereo
is better in the abstract, but because it could reduce the *dropout* that drives
your fill-fraction and consensus filters. AnyNet's anytime structure also maps
onto the stop-and-think cycle. **But measure SGBM on the Pi first** — it's the
unbudgeted item in `NOTES.md` and it may already be the bottleneck.

## 3. End-to-end navigation policy

**Task:** image → steering + collision probability, no map.

| model | scale | verdict |
|---|---|---|
| **PULP-DroNet V2** | **41 MMAC** | ✅ trivially fits |
| Learning High-Speed Flight in the Wild (Loquercio 2021) | depth → trajectories | 🟥 the closest work to your actual problem |
| Agile-flight RL policies (Swift and descendants) | small nets, big training | 🟥 |

**Verdict: architecturally wrong for you, worth reading closely.** These replace
the map; you have a map with a veto and that is the thesis. But the 2021
Science Robotics forest-flight work is *your task*, done learned — the failure
cases and the speed numbers are the best available yardstick. **PULP-DroNet's
41 MMAC is the existence proof that useful vision fits in this envelope.**

## 4. Object detection

**Task:** boxes and classes — people, vehicles, obstacles.

| model | scale | verdict |
|---|---|---|
| **FOMO** (Edge Impulse) | ~100 k params, centroids not boxes | ✅ runs on microcontrollers |
| **YOLO-Fastest / v2** | ~0.25 GFLOP | ✅ built for exactly this |
| **NanoDet-Plus** | ~0.9 M, ~0.7 GFLOP @320 | ⚠️ good accuracy/cost point |
| PicoDet-S | small | ⚠️ |
| MobileNet-SSD | ~1.2 GFLOP | 🟥 dated |
| YOLOv8n / v11n | ~3 M, ~6.5 GFLOP @640 | 🟥 @320 maybe; @640 no |
| YOLOv8s and up | — | ⛔ |

**Verdict: cheap and available, but ask what consumes it.** `DetectModule`
exists. Detection doesn't help forest navigation — trees aren't a class you need
named. It matters only if the mission is *find something*, which is a different
project. **NanoDet-Plus or YOLO-Fastest if you ever want it.**

## 5. Visual tracking

**Task:** follow a designated target across frames.

| model | scale | verdict |
|---|---|---|
| NanoTrack | very small | ✅ mobile-targeted siamese |
| LightTrack | ~0.5 GFLOP | ⚠️ NAS-designed for mobile |
| SiamFC / SiamRPN++ | 2–50 M | 🟥 |
| MixFormer, OSTrack, transformer trackers | large | ⛔ |
| **your fused classical tracker** | — | ✅ already built, already measured |

**Verdict: you are probably already at or past the useful frontier.** Your
cue-fusion + Staple histogram + occlusion-aware adaptation stack took sim
occlusion lock **51 % → 95 %**. A small siamese net would need to beat that at
higher cost. **Don't replace; consider a learned re-detection head only if
reacquisition after long occlusion is the measured failure.**

## 6. Semantic segmentation

**Task:** label every pixel — sky, ground, foliage, trunk, traversable.

| model | scale | verdict |
|---|---|---|
| **tiny 2-class sky/not-sky** | can be <10 MMAC | ✅ **see below** |
| ENet | ~0.37 M | ✅ |
| Fast-SCNN | ~1.1 M | ⚠️ real-time semantic seg |
| MobileNetV3 + LR-ASPP | small | ⚠️ |
| BiSeNetV2 | ~3 M | 🟥 |
| SegFormer-B0 | ~3.7 M | 🟥 |
| SAM / SAM2 | huge | ⛔ (you have `sam2_engine.py` — desktop only) |

**Verdict: the sleeper, and today made it urgent.** You just shipped a fix for
the planner flying at the sky. A **two-class sky segmenter is tiny** — sky is
the easiest segmentation task that exists (bright, smooth, top of frame,
no texture) — and it gives an *independent* signal that a bearing is not a
route. `APPEARANCE_AND_BLOBS_PLAN.md` §3.0 already proposed passive-RGB sky
brightness for the same bins; a tiny net is the robust version of that, and it
would work in the far field where depth cannot.

## 7. Optical flow

**Task:** per-pixel motion → velocity, odometry, ego-motion.

| model | scale | verdict |
|---|---|---|
| **Farnebäck / Lucas-Kanade** (classical) | — | ✅ already in `optical_flow.cpp` |
| LiteFlowNet3 | moderate | 🟥 |
| PWC-Net | ~90 GFLOP | ⛔ |
| RAFT / FlowFormer | 100s of GFLOP | ⛔ |
| **PMW3901 sensor** | hardware | ✅ €28, does it in silicon |

**Verdict: no. Buy the sensor.** Dense learned flow is the worst
compute-per-value on this list, and a €28 module plus a rangefinder gives you
metric velocity directly. The tracker's grid-flow ego-motion is already enough
for its own purposes.

## 8. Feature extraction and matching

**Task:** repeatable keypoints for odometry and scan matching.

| model | scale | verdict |
|---|---|---|
| **XFeat** (2024) | explicitly CPU-real-time | ✅ **the one I'd actually try** |
| ALIKED | light | ⚠️ |
| SuperPoint | ~26 GFLOP @VGA | 🟥 tiny variants exist |
| SuperGlue | heavy matcher | ⛔ |
| LightGlue | lighter matcher | 🟥 |
| **ORB / FAST+BRIEF** | classical | ✅ essentially free |

**Verdict: the strongest learned candidate in the whole survey.** XFeat was
designed for exactly your constraint — accelerated features on CPU — and
features are what motion-parallax odometry (`FAR_FIELD_MODELS.md` §4, now the
main far-field line) needs. Bark and foliage are hard for ORB; learned features
are meaningfully more repeatable on natural texture. **Benchmark ORB first**, but
this is where a model could genuinely beat classical.

## 9. Visual odometry / SLAM

**Task:** pose from images.

| model | scale | verdict |
|---|---|---|
| DROID-SLAM, DPVO | GPU | ⛔ |
| TartanVO, DeepVO | large | ⛔ |
| **SVO / VINS-Fusion / ORB-SLAM3** (classical) | — | ⚠️ plausible on Pi, heavy |
| **correlative scan matching** (your plan §5) | ~2 ms est. | ✅ |

**Verdict: stay classical.** Your own plan already scopes 3-DOF correlative
matching at ~2 ms. Learned VO buys nothing at 100× the cost.

## 10. Place recognition / relocalisation

NetVLAD and descendants — 🟥 to ⛔, and **irrelevant**: an aircraft going forward
through a wood does not revisit. Your plan already excludes loop closure on
purpose.

## 11. Image enhancement

Low-light, denoise, deinterlace, super-resolution — mostly 🟥. **One exception
worth noting:** your capture chain is analog CVBS and `hardware-bringup-checklist.md`
flags that the perception models have never seen comb artifacts. A *classical*
deinterlacer is the fix; a learned one is not worth the budget.

---

## What is actually worth doing

Three, in order:

1. **A tiny sky/not-sky segmenter** (§6). Cheapest useful model on the list, and
   today's bug is the argument for it. Gives the far field an independent "this
   bearing is not a route" signal that depth structurally cannot provide.
2. **XFeat, benchmarked against ORB** (§8). The only place a learned model
   plausibly beats classical *at lower total system cost*, and it feeds the
   motion-parallax line that is now the main far-field plan.
3. **Learned stereo, but only after SGBM is measured on the Pi** (§2). Might
   reduce the dropout your fill-fraction and consensus filters exist to paper
   over. Might also be irrelevant if SGBM already blows the budget.

## What to stop considering

* **Monocular depth for the far field** — measured dead, mechanism understood.
* **Dense learned optical flow** — a €28 sensor does it better.
* **Learned VO/SLAM, place recognition, transformer trackers** — wrong hardware,
  and in two cases wrong problem.
* **Detection** — cheap and available, but nothing in the navigation stack
  consumes a class label.

**The honest summary: the best learned model for this drone is a very small
one doing a very specific job that geometry cannot do.** Sky is that job.
Everything else on this list is either already solved classically at lower cost,
or is a different project.
