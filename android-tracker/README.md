# Kestrel Tracker

A **portable, feed-faithful lock-on test rig** for the Pi navigation/tracking
software. It runs the analog capture dongle over USB-OTG on an Android phone —
the *same* sensor path the Pi will fly (≈30 fps, ~150 ms latency, interlaced) —
so a lock that holds here holds on the Pi, as long as the tracker stays lean
enough for the Pi to run at 30 fps. This one is: pure NCC-in-a-crop, no model.

Separate from `../android/` (navviz), which stays as the navigation-validation
rig. This app is *only* the lock-on tracker.

## Why the phone

Bench-testing a 50–800 m target tracker in a room is impossible. The phone is a
self-contained field rig — camera + dongle + compute + screen + battery — that
you carry outside and point at real targets at real distances. And because a
lean correlation tracker is **camera-bound, not compute-bound**, the phone
(more powerful than a Pi 5) runs it at the same 30 fps ceiling the Pi will, on
the same feed. So it faithfully predicts Pi behaviour. (This equivalence holds
only for the lean tracker; heavy models run faster on the phone and would
flatter the Pi — not relevant here, there is no model.)

## The tracker — "TFL1" architecture

Reverse-engineered from reference terminal-guidance footage; the design that
locks on a low-Hz, low-contrast analog/thermal feed without a gimbal or a model
(`track/LockTracker.kt`):

- **Followed crop.** Each frame a crop around the predicted target is resampled
  to a fixed working size. The crop is sized from the tracked box, so the target
  stays a *constant size inside it* whatever the range — the "zoom window that
  never changes size" from the footage, and what keeps correlation stable across
  50–800 m.
- **NCC template matching** inside that crop (target large + centred → sharp,
  trustworthy peak), with a selectable appearance filter on template *and* crop.
- **PSR** (peak-to-sidelobe ratio) confidence — the per-frame health signal a
  boolean flag never gives you; drives lock/coast/lost.
- **Coarse multi-scale** to follow apparent-size change (= range), and an
  **alpha-beta centre filter** to predict + coast through missed frames.

Validated in Python: tracks a moving+growing synthetic target to ~3.6 px, box
grows 64→143 px as it approaches. Pure Kotlin on `GrayFrame`, no Android deps —
so it ports straight to the onboard C++ lock tracker.

## Modes

Top-right **MODE** button toggles between:

- **LOCK** — tap a target to lock and track it (the correlation tracker).
- **MOTION** — acquire by *movement*. A running-average background is subtracted
  each frame; what moved is boxed (largest = primary). **Tap a mover to lock it**
  — its box is handed to the tracker and you drop into LOCK. This is the cue that
  works when colour/brightness/texture can't separate the target (a car the same
  colour as the road): the instant it moves, it lights up. Assumes a roughly
  stationary camera — which is exactly the move-stop-sense *acquire-while-settled*
  case; ego-motion compensation is the moving-camera upgrade.

## Controls

- **Tap a target** — lock the target under your finger (LOCK mode); or tap a
  mover to lock it (MOTION mode).
- **Filter chips** (bottom row) — tap any chip to switch the crop filter, or tap
  **`off`** to disable filtering entirely. One tap, no cycling. The active chip
  is highlighted. Chips: `off · stretch · edge · threshold · sharpen · chroma`
  (threshold ≈ the thermal hot-blob style; edge for hard-edged man-made objects;
  **chroma** tracks on colourfulness, so a colour-distinct target pops even when
  it's the same brightness as its background — a colour cue the mono channel
  throws away). Double-tap anywhere still quick-cycles them if you prefer.

The feed itself is shown in **colour** (NV21→RGB); the chroma filter is how you
put that colour to work in the *tracker*, not just the display.
- **Long-press** — reset.
- Top-right **zoom PiP** shows the tracking window (like the footage). HUD shows
  state / confidence / filter / fps.

## Build

Same hard-won toolchain pin as navviz — **do not let Android Studio upgrade it**:

- **AGP 8.7.2 · Gradle 8.9 (wrapper) · Kotlin 2.0.20 · JDK 17 · compileSdk 34**

Open `android-tracker/` in Android Studio and Run. If the Gradle wrapper jar/
scripts are missing (they're gitignored), let Studio generate them or copy
`gradlew`, `gradlew.bat`, and `gradle/wrapper/gradle-wrapper.jar` from
`../android/`.

## Camera input — UVC over USB Host

The Magic V5 does **not** expose the capture dongle as a Camera2 external camera
(many phones don't), so `camera/UvcFrameSource.kt` opens it over USB Host with
libuvc via **herohan/UVCAndroid** (`com.herohan:UVCAndroid`, a single AAR on
**Maven Central** — chosen because the more popular AUSBC/libausbc is a
multi-module JitPack build whose `libuvc` module fails to resolve,
jiangdongguo/AndroidUSBCamera #727/#728). We use it only to open the dongle and
receive NV21 frames; the Y plane (first width*height bytes) is the luminance the
tracker wants — no colour conversion, no JPEG.

USB permission is handled by the library on device attach (no CAMERA runtime
permission). The frame-callback wiring in `UvcFrameSource.kt` is the most likely
spot to need a small on-device tweak against the resolved library version — it's
isolated behind `FrameSource`, so nothing else is affected.

`camera/Camera2FrameSource.kt` is kept as a built-in-camera fallback for phones
that *do* expose UVC through Camera2 — swap it in MainActivity if ever useful.

## Status

- ✅ Tracker logic — validated in Python (tracking + scale adaptation).
- ✅ Pure-Kotlin core (`track/`) — type-checks against the real Kotlin compiler.
- ⏳ Android layers — build on device. The UVCAndroid dependency resolves from
  Maven Central; `UvcFrameSource.kt`'s frame-callback wiring is the expected
  device-iteration point (library API shape), isolated behind `FrameSource`.

## Lessons carried from navviz (so we don't repeat them)

AGP 8.7.2 pin (never AGP 9); RGBA/luma path with no JPEG round-trip; luminance
-only pipeline (cheap, and the feed is mono); tracker decoupled from Android
types for testability + onboard reuse.
