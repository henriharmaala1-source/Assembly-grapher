# Lock-on tracker — review & improvement plan

Review of `track/LockTracker.kt` (+ `Filters`, `MotionDetector`, `OpticalFlow`,
`BlobFinder`, `CenterFilter`) and a prioritized plan for Opus. Validation tool is
`desktop/simtrack.py` (a faithful Python mirror of the tracker); every change
should be A/B'd there first, then on real recorded footage.

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
