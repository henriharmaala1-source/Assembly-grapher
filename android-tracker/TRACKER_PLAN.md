# Lock-on tracker — review & improvement plan

Review of `track/LockTracker.kt` (+ `Filters`, `MotionDetector`, `OpticalFlow`,
`BlobFinder`, `CenterFilter`) and a prioritized plan for Opus. Validation tool is
`desktop/simtrack.py` (a faithful Python mirror of the tracker); every change
should be A/B'd there first, then on real recorded footage.

## STATUS (2026-07): P0-A … P2-B all implemented + validated

All six planned items are done, each A/B'd in `simtrack.py` and ported to Kotlin
(the pure-Kotlin `track/` files type-check; `Mosse.kt` was compiled and run to
verify its FFT numerically):
- **P0-A** SEARCHING state + anchor-only wide re-acquire — `LockTracker.State`,
  `wide` search path.
- **P0-B** real-footage eval harness — `desktop/eval_tracker.py` (centre error,
  hold-time, re-acquire, id-switches; `--video`/`--labels` or `--synthetic`).
- **P1-A** optical-flow ego-motion feed-forward — `OpticalFlow` wired into
  `update()`, consensus-gated (sim pan: edge 3.7→0.4px, zero change elsewhere).
- **P1-B** appearance bank — diverse-pose keyframes, clean-view add + targeted
  consult (non-regressing; real-footage robustness).
- **P2-A** MOSSE/FFT correlation filter — `track/Mosse.kt` (validated matcher,
  ~4.4× cheaper than NCC for equal coverage; **not the default yet** — measured
  swap pending real footage via P0-B).
- **P2-B** occlusion-aware adaptation — PSR-drop detector freezes adaptation +
  scale (sim noisy-occlusion edge 91→98%).

## STATUS (2026-07, round 2): literature-research Tier-1 items implemented

A follow-up review researched classical tracker literature (DSST, STAPLE,
CSR-DCF, KCF, TLD) for techniques worth porting into this architecture — full
research + rationale in the session transcript, summary here. Five Tier-1
items implemented, each A/B'd in `simtrack.py`, ported to Kotlin (type-checks
clean):

- **Box-excluded, forward-backward-gated optical flow** (`OpticalFlow.kt`) —
  ego-motion grid points inside the tracked box are skipped (a
  large/dominant target could otherwise win the median vote), and every
  surviving point is re-matched backward (TLD-style FB check) before being
  trusted — an unreliable/aliased match is discarded before it can pollute
  the median or consensus. Confirmed harmless in sim (byte-identical where
  it doesn't fire); the intended failure case (a texture-rich target
  dominating the frame) isn't reproducible with the sim's smooth synthetic
  targets, so the benefit is argued from the mechanism, not measured — worth
  checking on real footage via `eval_tracker.py` once available.
- **STAPLE-style histogram appearance cue** — a chroma (fg/bg) score with NO
  spatial layout at all, fused into the same weighted-sum as every NCC cue.
  **The single biggest win of this round**: sim `occlusion` scenario went
  from 51%→95% lock (single-cue), 59%→82–95% across every cue set, because
  the histogram cue survives the occlusion boundary where spatial NCC can't.
  Requires chroma (skipped gracefully on luma-only frames) and gated to the
  normal-FOV search only (skipped during the SEARCHING wide re-acquire).
  **Tuning note:** an unbounded weight let it dominate over a merely-*noisy*
  (not truly occluded) spatial cue and regressed the noisy-feed case
  (98%→86% lock) — damping it to `HIST_WEIGHT_CAP=0.5` (closer to STAPLE's
  own fixed-α fusion) recovered that case *and* kept most of the occlusion
  win; the sweep (1.0/0.5/0.3/0.15) was non-monotonic (a knife-edge
  argmax-flip effect), so 0.5 is an empirically swept value, not derived.
- **Early cue termination** — skip remaining cues once one is already
  overwhelmingly dominant (PSR > 10); a straightforward frame-cost win.
- **Diversity-preserving keyframe eviction** — the appearance bank now
  evicts the most *redundant* slot (highest similarity to another kept
  slot) when full, instead of blindly evicting the oldest.

## CLOSED WITHOUT IMPLEMENTING: discrete rotation search

Rotation search was ranked the top deferred item ("the strongest valid
criticism", from the DSST/KCF/CSR-DCF literature review). **Measured, and it
does not reproduce — the item is struck, not deferred.**

Tested in the sim against progressively harsher rotation:

| case | result |
|---|---|
| continuous camera roll 30 / 60 / 120 °/s | 100 % lock, mean err ≤ 3.2 px |
| snap roll 450 °/s | 100 % lock |
| roll **through a 15-frame occlusion** (adaptation frozen, re-acquire runs off the stale fixed anchor ~45° out) | 100 % lock |
| **elongated 4:1 bar** target (chosen because a near-symmetric blob is an unfairly easy test) at 60 / 180 / 450 °/s | 100 % lock, worst mean err ~10 px |

Why the literature's criticism doesn't transfer: it targets DSST/KCF-family
trackers, which match against a *single* filter. This tracker already has two
mechanisms that absorb rotation — the **EMA-adaptive template** re-learns the
rotating appearance faster than rotation accumulates, and the **P1-B keyframe
bank** stores distinct rotated poses. A discrete rotation search would roughly
triple per-cue NCC cost for no measurable gain.

Caveat, stated honestly: these are synthetic targets on a synthetic
background. If real footage ever shows a rotation-driven loss, reopen this —
but do not build it on the literature's say-so alone, which is what the
original ranking was based on.

**Deferred from the same research round** (documented, not implemented):
DSST-style scale-decision smoothing, CSR-DCF-style
per-pixel reliability-weighted NCC for partial occlusion, MotionDetector
wired into lost-recovery as true re-detection, a richer multi-signal
confidence metric, and debug visualization (per-cue/fused/flow-field views).
A separate research pass concluded a small CNN re-identification embedding
is *not* worth adding yet — real gap (distractor discrimination) but a
different scope of work (training pipeline, not parameter tuning) and an
unresolved domain-transfer risk on our analog/thermal capture; ranked below
all of the above, revisit only after real-footage validation shows
distractors as a recurring failure mode.

The prose below is the original review, kept for rationale.

## What's good (don't touch without cause)

The design is a sound, layered correlation tracker:
- **Followed crop** sized from the box → scale-normalised target (good for 50–800 m).
- **Cue fusion** (luma/chroma/edge), each weighted by its own PSR **and** by
  agreement with the prediction (`prox`) — this is the right way to stop a
  sharp-but-wrong cue from winning.
- **Anchor + adaptive templates** (anti-drift) and **conditional distractor prior**.
- **Sub-pixel peak, latency-compensated aim, scale-on-luma.**

Keep all of this. The improvements below are additive.

## The core weakness: the crop IS the field of view

`update()` builds one crop at `cf.predict()` and searches only inside it
(`workingCropRaw(frame, pcx, pcy, bsize)`, search window ≤ `maxHalf` crop px).
**There is no mechanism to look beyond that crop.** Consequences:

- If the target leaves the crop — fast motion, a turn the constant-velocity
  `CenterFilter` didn't predict, or an occlusion during which the coasted crop
  drifts off on stale velocity — it is **gone forever**. Coasting only widens the
  search *within* the (mis-placed) crop.
- This is almost certainly the dominant cause of "tracking escapes/stops."

Everything in P0 below targets this.

## Secondary gaps

- **Motion model** is constant-velocity alpha-beta only → lags on accel/turns,
  mis-centring the crop exactly when you need it centred.
- **`OpticalFlow` exists but the tracker doesn't use it** — free ego-motion +
  target-motion signal is being left on the table.
- **Appearance model** is 1 anchor + 1 adaptive; a small diverse **bank** holds
  better through pose/lighting swings.
- **No occlusion reasoning** — partial occlusion can still poison the adaptive
  template (only softly gated by the conf threshold).
- **Perf:** the anchor doubles NCC cost (2 response maps × N cues). Fine now, but
  it caps headroom for P0's wider search.
- **No measurement on real footage** — all tuning is synthetic (`simtrack.py`).

---

## Plan (prioritized)

### P0-A — Wide re-acquisition ("SEARCHING" state)   [biggest win]
**Problem:** target lost outside the crop is unrecoverable.
**Change:** add a `SEARCHING` state entered after ~N coasting frames (or when the
peak is at the crop edge). In it, scan a **large region** (e.g. a 3–4× crop-sized
window around the last position + Kalman projection, or a coarse full-frame grid
of candidate boxes) using the **fixed anchor** templates (not the possibly-drifted
adaptive ones), ordered nearest-first to the predicted reappearance point. On a
confident match (PSR ≥ `psrLock` and prediction-consistent), re-enter LOCKED.
This is exactly the desktop tracker's `SEARCHING` batch-scan, ported.
**Validate:** new `simtrack.py` scenarios — "target jumps out of crop then
returns", "sharp 90° turn", "3 s full occlusion". Metric: re-acquire rate + frames
-to-reacquire.
**Risk:** cost (scan many positions) — keep it coarse (big stride) + anchor-only.

### P0-B — Real-footage eval harness   [measure, don't guess]
**Problem:** the sim is synthetic; real analog/thermal footage behaves differently.
**Change:** record a few dongle clips (already have `capture_stats.py`/dumps),
hand- or SAM2-label ground-truth boxes, and add a headless runner that replays a
clip through `LockTracker` and reports **hold-time, re-acquire rate, centre error,
identity-switch count**. Then every P-item is A/B'd on real data, not vibes.
**Risk:** none; pure tooling. Do this early so P0-A/P1 are measured.

### P1-A — Optical-flow-assisted prediction
**Problem:** constant-velocity prediction mis-centres the crop on maneuvers and
under ego-motion (drone).
**Change:** run `OpticalFlow` on the followed crop each frame; use the **median
flow inside the current box** as a target-motion prior to seed `CenterFilter`
(better than pure constant-velocity), and the **global/background flow** as an
ego-motion term. Feed both into the predicted crop centre.
**Validate:** sim "accelerating target" + "panning camera" scenarios; real
handheld pan.
**Risk:** flow noise on low-texture targets — gate by flow inlier count.

### P1-B — Appearance bank (replace anchor+adaptive)
**Problem:** 1 anchor + 1 adaptive can't represent a target seen from multiple
poses; long holds through rotation still drop.
**Change:** keep a small bank (K≈4) of mean-subtracted templates per cue: the
original + up to K−1 high-confidence, sufficiently-*different* snapshots (add only
when max-similarity-to-bank < threshold, so it stays diverse). Response = max over
the bank. Evict the least-useful (not the anchor). This subsumes the current
anchor+adaptive.
**Validate:** sim "target rotates 180°" long scenario; measure hold-time vs the
current 2-template scheme.
**Risk:** cost scales with K — cap K, and only run the full bank during LOCKED
(anchor-only during SEARCHING).

### P2-A — MOSSE / FFT correlation filter (algorithmic)
**Problem:** brute-force NCC is `O(positions × template²)`; it's the frame-cost
ceiling and limits how big P0-A's search can be.
**Change:** replace per-cue NCC with a MOSSE-style adaptive correlation filter
(FFT, `O(N log N)`), which also yields PSR natively and adapts every frame. Bigger
change; do it behind the same per-cue interface so fusion/prior code is untouched.
**Validate:** same-scenario latency + accuracy vs NCC in the sim.
**Risk:** needs an FFT in Kotlin; larger effort. Reserve until P0/P1 are in and
measured. (This is also the biggest onboard-C++/NEON win.)

### P2-B — Occlusion-aware adaptation
**Problem:** partial occlusion can drift the adaptive template.
**Change:** detect occlusion from the response signature (peak drop + rising
secondary mass) → freeze adaptation, hold scale, coast; on recovery, verify
against the anchor bank before resuming adaptation. (Full segmentation-mask
boundary detection is the heavier version — defer to onboard P6.)

---

## Suggested order
`P0-B (harness) → P0-A (re-acquire) → P1-A (flow) → P1-B (bank) → P2`.
Do the harness first so P0-A is measured on real footage, not just the sim.

## Guardrails
- Everything must stay lean enough to run on the cheap tracker chip / Pi and port
  to the onboard C++. If a change can't, it belongs on the nav brain, not here.
- A/B each change in `simtrack.py` first; keep the winning params documented.
- Watch per-frame cost — P0-A's search and P1-B's bank both add NCC work; P2-A
  (MOSSE) is what buys the headroom back.
