# FAR_FIELD_MODELS — candidates for a learned far field

Review document. Nothing here is a decision, and nothing here is built.

**Context.** The far field is the one place in this stack where a learned model
is structurally safe, because that layer has no authority — it scores a bearing
and can neither veto a primitive nor grant speed (`nav-sim/bearing_field.hpp`).
A hallucination costs a bad openness score, not a crash. That asymmetry is the
whole reason this is worth reviewing at all, and it does **not** extend to the
near field, which keeps its cubes and keeps the veto.

**Numbers below are order-of-magnitude and several are from memory.** Verify
parameter counts and MACs against the source repos before trusting any of them.
The only figures in this file that were measured on this project are the 2.5 ms
bearing field and the 3.54 m `Z_max`, and even those are Xeon numbers.

---

## 1. The spec, which rules out most of the field

Almost every monocular depth model exists to produce a **dense, pretty depth
image**. We do not want one. Writing the spec down first eliminates about ninety
per cent of the candidates before we compare anything:

| requirement | consequence |
|---|---|
| output is a **range profile**, ~72–360 azimuth bins, not H×W pixels | the decoder — where most of the MACs live — can be mostly deleted |
| **metric**, in metres | relative inverse depth is useless to a planner; needs an anchor (§5) |
| only needs to work **beyond 3.5 m** | the near field owns everything closer and is already metric |
| may be **wrong** without being dangerous | no safety case to make; this is the cheap part |
| must beat **2.5 ms** | `BearingField::update`, the incumbent |
| CPU only, ARM64, no GPU, no NPU | rules out anything that assumes CUDA |

That last row plus the 2.5 ms bar is the whole difficulty. Everything else is
easier than it looks.

**Budget.** Rough Pi 5 arithmetic — four A76 cores at 2.4 GHz with NEON
dot-product, generously discounted for real-world efficiency — puts the ceiling
somewhere around **200 MMAC/frame at 10 Hz**. Arithmetic, not measurement. The
one real datapoint in this repo is that `NavigateModule` was estimated at
~110 ms, and `onboard/docs/hardware-bringup-checklist.md:42` is explicit that
this is a **desktop estimate that has never been checked on Pi silicon**.

---

## 2. What already exists in this repo

This is the most important section and it was a surprise. **Half of this idea is
already built and running.**

`onboard/include/depth_nav.hpp` — `DepthNav`:

* runs **MiDaS Small** or **Depth Anything V2 Small** through OpenCV DNN on CPU
  (`DepthBackend::{MIDAS_SMALL, DEPTH_ANYTHING_V2}`), 256×256 input
* collapses the result to a **96×72 working grid**, de-rolled and de-pitched
  from FC attitude, so the horizon band does not swing into ground or sky
* emits `openHist()` — **a per-column polar openness histogram**, which is the
  bearing profile this document was written to ask for
* runs VFH+ over it with hysteresis and a previous-heading cost
* has a second entry point, `updateFromGrid()`, that takes a **metric** depth
  grid from ToF/stereo and feeds the identical downstream pipeline

And it already knows its own central weakness. `depth_nav.hpp:86–90`:

> *METRIC ONLY on the ToF/stereo path (updateFromGrid sets a real maxRange); on
> the monocular path it is a NOMINAL scale (relative openness × scanMaxNominalM)
> — good enough to bias routing, not a true map, until a ranging sensor exists.*

**"Until a ranging sensor exists."** It exists. It is the near voxel map, it is
metric out to 3.54 m, and it looks at the same image every frame.

**The blocker is not modelling.** `NOTES.md` already records it: `nav-sim/` and
`onboard/` share **zero** code (`grep VoxelMap onboard/` → nothing). Both halves
of this idea are built, in two codebases that do not talk to each other. That
integration — not architecture search — is the actual work.

---

## 3. Candidates

### Tier A — already wired, cost unknown on target

| model | size | notes |
|---|---|---|
| **MiDaS Small** (v2.1) | ~20 M params, 256² | Already in `DepthNav`. Relative inverse depth. Robust, old, well understood. |
| **Depth Anything V2 Small** | ~25 M params, ViT-S | Already in `DepthNav`. Substantially better than MiDaS on hard scenes; ViT attention is less CPU-friendly than pure conv. |

Both are *far* over the 200 MMAC guess. Neither has ever been timed on a Pi.
**First action for either: measure, don't theorise.**

### Tier B — smaller, embedded-oriented, would need integrating

| model | size | why it's here |
|---|---|---|
| **FastDepth** (MIT, 2019) | ~4 M params | MobileNet encoder + heavily pruned NNConv5 decoder, explicitly designed for embedded real-time. The closest existing thing to our shape. |
| **Lite-Mono** (CVPR 2023) | ~3 M params | Self-supervised, hybrid conv/transformer. Small and modern. |
| **GuideDepth** (2022) | ~6 M params | Guided decoder, targets real-time embedded. |
| **Monodepth2** (ICCV 2019) | ~14 M (ResNet-18) | Too big, but the **self-supervised training recipe** is the valuable part, not the net. See §5. |

### Tier C — accurate, hopeless on CPU, useful only as teachers

**Depth Anything V2 Base/Large**, **Depth Pro** (Apple), **Metric3D v2**,
**UniDepth**, **ZoeDepth**, **DPT-Large**. All are seconds-per-frame on an ARM
CPU. Their role, if any, is **distillation**: run one offline over recorded
flights to label a training set for a small student. That is a real technique
and it is cheap, because it happens on a desktop and never flies.

### Tier D — right size, wrong output; backbone donors

**MobileNetV3-Small**, **EfficientNet-Lite0**, **ShuffleNetV2**, **MCUNet**. None
does depth. All are the correct order of magnitude for a *profile* regressor
with the decoder deleted. This is where a custom head would attach.

**PULP-DroNet** deserves its own line: 41 MMAC/frame, 12.8 fps on a
microcontroller, ~1.6 % of a 30 g drone's power budget. It is the existence proof
that a useful vision policy fits in this envelope — but it is 41 MMAC *because it
emits two scalars*. It is a lower bound on what is possible, not a candidate.

---

## 4. Rivals that are not models at all

A learned far field must beat these, not just beat nothing.

* **`BearingField` (incumbent).** 2.5 ms, no training, no dataset, no sim-to-real
  risk, already shipping and already good — "the far field white gaps are OK and
  supposed to be there". **This is the baseline and it is a strong one.**
* **Motion parallax / SfM.** The strongest non-learned rival and possibly the
  strongest option full stop. `Z_max ∝ √B`. The D435i baseline is 50 mm; one
  second of flight at 1 m/s is a **1 m** baseline — twenty times larger, so
  `Z_max` grows by √20 and 3.54 m becomes roughly **16 m**, with *geometry*
  rather than a prior. It needs odometry, which is **P5a (VIO), already on the
  roadmap**. This should be evaluated before any model is trained, because if it
  works it is metric, explainable, and needs no data at all.
* **Horizon / vanishing point + texture gradient.** Classical, microseconds,
  decades old. Weak, but nearly free, and a fair floor to measure against.

---

## 5. Custom model work: what is and is not needed

**Short answer: no custom architecture. Yes to a custom head, a custom loss, and
a custom anchor — and the anchor is the whole idea.**

### Not needed

* **A new architecture.** This is the classic waste, and the PULP-Dronet paper is
  the cautionary tale in the useful direction: they changed *nothing* about
  DroNet's topology and got 2× memory and 1.6× throughput purely from deployment
  work. Take a pretrained encoder.
* **A deployment runtime.** Already scoped as **ROADMAP F10** — NCNN or ONNX
  Runtime + XNNPACK, typically 2–3× over OpenCV DNN on A76, with real INT8. That
  task exists, is written down, and is deferred pending exactly this bottleneck.
* **A consumer for the output.** `openHist()` and
  `BearingField::obstacleDistance()` both already emit bearing profiles.

### Needed, and it is smaller than it sounds

1. **A metric anchor — the single highest-value piece.** Monocular depth is
   scale-free; that is its fundamental limitation and the reason `DepthNav` says
   "nominal". But the near voxel map is **metric ground truth for the far field's
   own camera, every frame, in the same image**. A per-frame least-squares fit of
   `a·d + b` over the overlap band converts relative inverse depth into metres.
   On the order of **thirty lines**. Nobody ships this because nobody else has
   both halves — we do, in two repos that don't link.
2. **A profile head instead of a depth decoder.** We want ~72 numbers, not
   H×W pixels. Truncating the decoder is where the MMAC savings actually live and
   is the only genuinely custom modelling work. Small.
3. **Self-supervised fine-tuning on real frames.** The same anchor is a training
   signal: the aircraft labels its own far field by flying toward things and
   watching what the near map says when it arrives. No dataset, no annotation.
   This is Monodepth2's recipe with our stereo as supervision.

### The trap, stated plainly

PULP-Dronet's accuracy went **83 % → 90 % purely by adding 1.3 K images from the
real onboard camera**. Training a far field on `voxel_world` renders alone will
produce something that looks superb in `voxel_sim` and dies on the D435i. That is
the exact failure this project has already hit four times — every one of them
found by looking at a screenshot, none by a test.

---

## 6. How to decide

In order, cheapest first. **Stop as soon as something clears the bar.**

1. **Time MiDaS Small and DAv2-Small on actual Pi 5 silicon.** One afternoon.
   `onboard` already loads both. If they land near ~110 ms, that is 44× the
   bearing field and the answer is no until F10 and a smaller net exist. If
   INT8 + a truncated decoder gets it under ~10 ms, everything changes.
2. **Evaluate motion parallax first anyway.** It is metric, needs no data, and
   `Z_max` 3.54 → ~16 m would be a larger win than any model in this file.
3. **Then, and only then, score a learned far field on `--compare`**, which was
   built for exactly this: hold the scene, the projection and the near map fixed,
   swap only the far representation. A new representation is a **comparable, not
   a proposal** — the standing rule in this project, learned by getting it wrong.

**Acceptance:** beats `BearingField` on far-field coverage and median range error
at equal or lower frame cost, on recorded **real** D435i footage, with the near
field byte-identical. Anything less is a picture, and a picture is not a test.
