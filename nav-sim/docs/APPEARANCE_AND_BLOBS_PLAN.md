# Appearance and connected regions — PLAN

Status: **planned, not implemented.** Nothing in this document exists in the
tree. It is written now because the first half of it (per-pixel intensity as
evidence about free space) is an idea that *looks* obviously good, is cheap to
build, and would be wrong in a way that kills the aircraft — and the second half
(reasoning about connected regions rather than pixels) is the one worth the
effort. Writing the argument down is how the ordering survives contact with a
free afternoon.

Companion to `DEPTH_SUPERVISOR_PLAN.md`. Where the supervisor is a *second
independent path* to a veto, everything here is about extracting more from the
path we already have.

---

## 1. The two proposals

1. **Appearance on contested cells.** Feed image intensity into cells whose
   log-odds sit near a threshold, and let brightness break the tie — the
   suggestion being that a bright pixel argues for FREE, particularly in the far
   field where the geometric evidence is thinnest.
2. **Connected-region (blob) reasoning.** Stop treating each pixel as an
   independent vote and start reasoning about connected regions of the depth
   image — regions of return, and regions of *no* return.

They are separable. (2) does not need (1). The conclusion of this document is
that (2) is where the value is, and that (1) survives only in a much weaker,
one-sided form than it first appears.

## 2. Use IR, not RGB

The D435i computes depth **in the left IR camera's frame**. Depth and left-IR
are the same pixels: no extrinsic, no `rs2::align` pass, no resampling, and — the
part that matters — no *new* holes introduced by aligning one sensor's occlusions
onto another's.

The RGB imager is a physically separate camera on a different baseline. Using it
means an alignment pass every frame, and the alignment's own occlusion shadow
lands beside every near object, which is exactly where §4.1 wants clean data.

There is also an envelope argument. A large part of what this project is for is
closed canopy at dusk and indoor flight. There, RGB is noise and the IR band
still has a ~1 W projector throwing pattern into it. The one modality that
degrades gracefully across our whole envelope is the one that is already
pixel-aligned to the depth. That is not a coincidence; it is the same sensor.

### 2.1 What already exists

More than expected, which changes the size of this job:

- `VoxelMap::integrate(depth, intensity, cam, pose)` — already takes an
  intensity image and already exists.
- `VoxelMap::tex_`, `texAt()`, `hasTexture()` — a per-cell `uint8` appearance
  channel, allocated lazily.
- The integrator paints appearance **only on marked hits**, with the reason in
  the code: *"painting a carved cell would colour thin air."* That restraint is
  correct and §3 is largely an argument for why it should stay.
- `DepthCamera` already models the projector: `emitterOn`, `emitterRangeM = 3.0`,
  `emitterTex = 0.9`, `ambientIR`, with falloff `1/(1 + (r/emitterRangeM)²)`.

What is missing is one thing: **`FrameSource::next()` hands back depth alone.**
Adding an optional intensity output, and enabling `RS2_STREAM_INFRARED` index 1
at the depth stream's own resolution in `RealSenseSource`, is the entire
plumbing job. `SimFrameSource` can synthesise it from the world's existing
`trunkTexMin/Max` (0.30–0.75) the same way it synthesises depth, and
`.kdr` would need a second plane per frame or a sibling file.

## 3. Brightness: two different questions, and only one of them is what was asked

**Correction, and it matters.** Sections 3.1-3.4 below were written against the
*active illumination* reading of the proposal -- the D435i's IR projector, near
field, per-pixel. The question actually asked was about **passive RGB in the FAR
field**: find open areas at a distance. That is a different physical problem and
most of the argument below does not apply to it. Section 3.0 is the answer to
the question that was asked; 3.1-3.4 remain correct for the active case and are
kept because the active case is also worth knowing about.

### 3.0 Passive far-field openness -- and why the objection in 3.1 does not apply

For an extended surface under fixed ambient light, **image brightness is
independent of distance.** The inverse-square falloff of received power is
exactly cancelled by the pixel footprint growing as Z^2; radiance is
distance-invariant. The 1/Z^2 term in 3.1 exists only because the illumination
rides on the camera, so the surface's own irradiance falls off. Passive imaging
has no such term.

(The exception is a source *smaller than one pixel*. A thin branch at range does
dim, because it fills less of the footprint -- so it appears as a dark thread
against a bright gap, which helps rather than hurts.)

**Bright far-field regions in a wood are sky, and sky is open.** This is not an
analogy: foresters measure *canopy gap fraction* by thresholding upward
hemispherical photographs, and it is a standard instrument. Sky segmentation is
long established in outdoor robotics for horizon and attitude work.

**The alignment objection in section 2 is also weakest here.** RGB is a separate
imager needing an `rs2::align` pass whose parallax opens holes beside near
objects -- but parallax goes to zero with distance. At 20 m a ~15 mm extrinsic
offset subtends essentially nothing. The cost of using RGB is worst in the near
field and negligible in the far field, which is exactly where this would be used.

**The architectural fit is the strongest argument.** The far field is where the
map is worst: past `maxIntegM` (3.5 m at 0.25 m cells) we carve but never mark,
and past ~10 m even the coarse layer stops. The planner's `farWeight` openness
term currently picks a bearing from the coarse map's occupancy density at ranges
where stereo has almost nothing to say. A sky/gap signal is an **independent
estimate of the same quantity, strongest precisely where the geometry is
weakest** -- the reverse of the usual situation.

And the slot already exists with the right safety semantics. The far map is
documented *AWARENESS ONLY, NEVER PERMISSION*: it says which bearing looks open
beyond the honest range and never grants leave to fly there. A brightness
openness term drops into that contract unchanged.

What still holds from the sections below:

* **One-sided, but with the OPPOSITE polarity to 3.3.** Bright is strong
  evidence of open; dark is evidence of nothing, because a shaded corridor
  between trunks is both open and dark. It contributes to *choosing a bearing*
  and never to carving a cell.
* **Auto-exposure makes absolute brightness meaningless between frames.** Use
  relative brightness within a frame, or lock the exposure. This is the first
  thing to get wrong.
* **It dies at dusk under closed canopy**, which is a real part of the envelope.
  Unlike the IR path, this one has no fallback -- there is no visible-light
  projector.

Validation is the far-field openness A/B that already exists: does the chosen
bearing agree better with truth openness at 10-25 m with the term on than off,
over the standard 8 seeds? That is measurable in sim today with a synthetic sky.

### 3.1 The polarity is backwards -- UNDER ACTIVE ILLUMINATION

Under active illumination, received intensity goes as **albedo / Z²**. So for a
pixel with *no depth return*:

| intensity | most likely cause | what it means for occupancy |
|---|---|---|
| **bright** | something close and textureless bounced the projector back — blank wall, smooth trunk, painted surface | **not free.** The opposite of the proposal. |
| **dark** | open space past the emitter's reach, **or** a low-albedo surface at any range | genuinely ambiguous |

The proposal reads the table upside down. A bright return-less pixel is the
single most alarming thing in a depth image: it is a surface that is definitely
there and definitely unmatched.

### 3.2 And the ambiguity is worst exactly where it kills you

Near-infrared albedos are perverse for a machine that flies through woods.
Vegetation has a very high NIR plateau — leaves reflect on the order of 50 % at
850 nm — while bark is dark, which is why the sim already models trunks at
`trunkTex` 0.30–0.75 and why `bark_contrast` exists as a test at all.

So a "dark ⇒ probably free" rule fails hardest on **dark thin branches at
range**, which is the documented cause of real failure in *both* reference
papers and is already logged in `NOTES.md` as the open item that matters most.
A rule whose error mode is aligned with the system's dominant failure mode is
not a weak rule; it is a negative-value rule.

### 3.3 What survives: a veto, never a grant

Intensity may be used to **withhold** free space and never to grant it:

> **bright + no return ⇒ refuse to carve through this pixel's neighbourhood.**

That is monotone — it can only make the map more conservative — so it cannot
introduce a new way to die, exactly like the supervisor's rule in
`DEPTH_SUPERVISOR_PLAN.md` §3.

Now the honest part, which is the reason this is item (2) of two and not item
(1). **It buys less than it looks like.** A return-less pixel already carves
nothing — the integrator's first rule is `if (!(r > 0.f)) continue;`, "no
measurement → no information". The veto would only bite on *neighbouring* rays
that carve past a blank patch, and `carveWinPx` (the min-filter clamp, "never
claim free space beyond the nearest thing seen nearby") already covers a good
share of that case. The residue is: a blank patch with **no** valid returns
anywhere in the min-filter window, flanked by rays that do have distant returns.
That is a real case — a bare wall seen across an open room — but it is a
narrower one than "far-away contested voxels".

### 3.4 The one case where brightness could grant free space

For completeness, because it is the only defensible version and it should be
written down before someone reinvents it badly:

> emitter ON **and** intensity below a floor **and** no return
> ⇒ nothing within the emitter's effective range.

This is an active range gate — the same logic a ToF confidence channel uses. It
is the only way to claim free space from an *absence* of return, and it would be
genuinely valuable in the dark corridor case where stereo returns nothing at all.

It also requires: a measured irradiance falloff for this specific unit, a bound
on the darkest albedo we are willing to be wrong about, and a range so short
that the `1/Z²` ambiguity cannot reach across it. §3.2 says the darkest albedo
we care about is a wet dark branch. My estimate is that the usable gate ends up
inside 1–1.5 m with a large margin, which is inside stopping distance anyway.
**Do not build this before measuring it.** If it is built, it needs its own
adversarial arm: a dark branch at 2 m must not be carved.

## 4. Blobs — the part worth building

The map accumulates evidence **per cell, independently**. That is the right
default and it is also the reason two specific things are invisible to it. Both
become visible the moment the unit of evidence is a *connected region* instead of
a pixel.

### 4.1 Explaining no-return regions with the shadow we already model

`DepthCamera` already computes the stereo occlusion shadow — width exactly
`f·B·(1/Z_near − 1/Z_far)` px — and already knows it falls on the **LEFT** of a
near object, the side away from the right imager. `occlusion_check` pins both the
side and the width to 15 %. At f = 447 px, B = 50 mm:

| near object | background | shadow |
|---|---|---|
| 0.4 m | 2 m | 45 px |
| 1 m | 5 m | 18 px |
| 2 m | 6 m | 7 px |

The plan:

1. Connected components on the **invalid** mask.
2. For each blob, read the valid depth along its **right** edge — that is the
   candidate occluder.
3. Predict the shadow width that occluder would cast against the depth found on
   the blob's **left** edge.
4. If the blob's width matches the prediction, the blob is **explained**.

An explained blob is no longer a mystery region. Two things follow, and the
second is the useful one:

- Its true content is **bounded below** by the occluder's depth — there is
  nothing between the camera and the occluder, or the occluder would not be the
  nearest thing along those rays.
- Therefore free space may be carved through it out to the occluder's depth,
  which is a claim the per-pixel rule cannot make and currently refuses to make.

This converts a systematic, predictable, everywhere-in-a-forest class of UNKNOWN
("a dead strip beside EVERY trunk — precisely where the map most needs data,
since it is the obstacle boundary") into a bounded claim, using code that is
already written and already tested. That is the best ratio in this document.

The failure mode to watch: a blob that *happens* to match a shadow width but is
really a textureless surface. Mitigate with §3.3 — an explained blob that is
**bright** is not explained, it is a wall. This is the one place the two halves
of this document compose.

### 4.2 Thin structures — the one that could change a flight outcome

The open item, verbatim from `NOTES.md`: *a 3 cm branch cannot cross `occThresh`
in a 0.25 m cell.* Per-pixel log-odds is simply the wrong statistic for a long
thin object. A 3 cm branch spans

```
f · 0.03 / Z   →   6.7 px at 2 m,  3.4 px at 4 m
```

— a handful of pixels wide, but potentially *hundreds* long. Individually those
pixels are noise. Collectively they are one of the most distinctive objects in
the image.

The plan: connected components over pixels that are **significantly nearer than
their local background** (nearer by more than a few σ_d of their own depth, so
the threshold is range-dependent and falls out of the existing `depthSigCoef`).
Then classify by shape from `connectedComponentsWithStats`:

```
elongated  :=  major/minor axis ratio > ~8
thin       :=  minor axis < ~12 px
coherent   :=  depth spread along the component < a few σ_d
```

A component satisfying all three is a branch, wire or railing. It should mark
its cells **as a blob** — one decision for the whole object — rather than
waiting for each cell to win a vote it cannot win.

This is a deliberate, bounded exception to "evidence accumulates per cell", and
it should be written into the code as such, with the argument next to it. It is
the only mechanism proposed here that lets the map assert an obstacle the
log-odds would never reach.

Note what this does *not* need: intensity, colour, learning, or a second sensor.
It is a shape argument about the depth image we already have.

### 4.3 Sky and ground

A large invalid blob touching the **top** image border, with low IR intensity and
no valid returns anywhere inside it, is open sky. This is the one place where
"carve on absence" is defensible, because the alternative hypothesis (a
featureless surface filling the top of the frame at close range) is refutable by
the intensity floor from §3.3.

Lowest value of the three — an FPV aircraft rarely needs to be told the sky is
empty — but it is nearly free once §4.1's connected-components pass exists, and
it removes a persistent grey lid from the first-person view.

## 5. Parameters

| name | proposed | what moving it does |
|---|---|---|
| `shadowTolFrac` | 0.25 | how closely a blob's width must match the predicted shadow to be explained. Tighter = fewer explained blobs, more honest UNKNOWN. |
| `brightVetoLvl` | TBD — **must be measured** | intensity above which a return-less pixel blocks carving. Unit-specific and exposure-dependent; a hard-coded constant here would be a lie. |
| `thinMaxMinorPx` | 12 | above this it is a trunk, and trunks are already handled well. |
| `thinMinAxisRatio` | 8 | below this it is a blob, not a branch. |
| `thinNearSigK` | 3 | how many σ_d nearer than background before a pixel joins a thin candidate. Shares `depthSigCoef` with the map, and must not be duplicated. |
| `blobMinPx` | 40 | floor on component size. At a 0.4 % speckle rate, isolated components this large do not occur by chance. |

`thinNearSigK` is the one that will decide whether §4.2 works, and it is
calibrated against σ_d — which is **still the assumed 0.25 px**, not a measured
number. See §8.

## 6. Cost

`cv::connectedComponentsWithStats` is the only new per-frame pass:

| resolution | approx |
|---|---|
| 848×480 | 3–5 ms |
| 424×240 (the stride-2 grid) | ~1 ms |

Against a measured integrate of ~22 ms and a plan of ~1 ms, running the blob
pass on the **stride-2 grid** is affordable; running it at full resolution is
not, and would not help — a 3 px branch at 4 m is 1.7 px at half resolution,
which is still a component.

Intensity costs one extra `uint8` plane per frame off the camera (407 kB at
848×480), one `uint8` per cell in `tex_` (already allocated lazily and already
paid for wherever appearance is used), and roughly nothing in time.

## 7. Validation, in order

Same discipline as `DEPTH_SUPERVISOR_PLAN.md` §6, and for the same reason: this
project has more than once shipped a conclusion that was really an instrument
fault.

1. **Synthetic, unit test.** A slab at a known depth against a known background:
   assert the shadow blob is found on the LEFT, assert its width matches
   `f·B·(1/Z_near − 1/Z_far)`, assert it is declared explained, and assert that
   the same blob with a bright intensity plane is declared **not** explained.
   `occlusion_check` already builds this scene — extend it rather than writing a
   second one.
2. **Thin structure, synthetic.** `voxel_world.cpp` has no thin branches, which
   is itself an open item. Add them, then assert: a 3 cm branch at 2 m is marked
   by §4.2 and is **not** marked without it. If it is marked without it, the
   whole mechanism is unnecessary and this document is wrong.
3. **The false-positive direction, which is the one that matters.** Count how
   often §4.1 carves through a blob that truth says was occupied, on the forest
   world across seeds. A shadow-explainer that occasionally carves a real trunk
   is worse than no shadow-explainer, because it manufactures free space — the
   one thing the map's whole design refuses to do.
4. **Paired A/B**, lanes world, D435i geometry, 1.5 m/s, 400 steps, 8 seeds —
   the same protocol as the openness sweep so the numbers are comparable to what
   is already in the tree. Report collisions and goal-closing efficiency
   separately; §4.1 should improve efficiency (more carved space, fewer
   spurious BLOCKED verdicts) and §4.2 should reduce collisions. If either moves
   the other's metric, something is wrong.
5. **Real frames.** Record a `.kdr` walking past trunks and a doorframe. The
   shadow explainer is directly checkable: the strips beside trunks are visible
   by eye in the depth pane, and either they get explained or they do not.

Step 2 has a prerequisite the others do not: **there are no thin branches in the
sim world.** That is a real blocker for §4.2 and should be fixed first, since it
also blocks the existing thin-structure open item.

## 8. Ordering, and the thing that gates all of it

Recommended order:

1. **Measure σ_d** — `d435i_probe.py --range X --preset sweep`. Everything in
   §4.2 is thresholded in units of σ_d, and it is still an assumed 0.25 px. Also
   gates `maxIntegM`, `maxCarveM`, and the whole ladder's honest ranges, so it is
   the highest-leverage measurement outstanding regardless of this document.
2. **Thin branches in `voxel_world.cpp`** — a prerequisite for testing §4.2 and
   an open item in its own right.
3. **§4.2, thin structures** — targets the known dominant failure mode.
4. **§4.1, shadow explanation** — best effort-to-value ratio, but it *adds* free
   space, so it should go in when there is appetite for the validation in §7.3.
5. **Intensity plumbing + §3.3 veto** — cheap, safe, and modest.
6. **§3.4 active range gate** — only with measurements in hand, and only with its
   own adversarial arm.

## 9. What this does not do

- It does not see anything stereo does not return. §4.1 explains *why* a region
  is empty of data; it does not recover depth there.
- §4.2 marks obstacles the map would miss. It does not make them *accurate* —
  a blob-marked branch gets the blob's depth, not a per-cell reconstruction.
- Nothing here needs, or benefits from, colour. If a future version reaches for
  RGB, §2 is the argument it has to beat first.
- None of it is a substitute for measuring σ_d. Several thresholds above are
  written in units of a number this project has assumed and never checked.
