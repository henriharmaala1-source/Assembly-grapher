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

## Controls

- **Tap** — lock the target under your finger.
- **Double-tap** — cycle the crop filter: `none → stretch → edge → threshold →
  sharpen`. Test which best reveals your target on this feed (threshold ≈ the
  thermal hot-blob style; edge for hard-edged man-made objects).
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

## The one device-verify seam: UVC input

`camera/UvcFrameSource.kt` is the only file that couldn't be compile-checked
here. It uses **AUSBC (AndroidUSBCamera / libausbc)** — declared in
`app/build.gradle.kts` via JitPack — *only* to open the dongle and receive raw
NV21 preview frames (whose Y-plane is handed straight to the tracker; no colour
conversion, the feed is mono). The AUSBC API (class/callback names, the
`CameraRequest` builder, the preview-format enum) can differ between library
versions — if the build complains, reconcile that one file against the resolved
`libausbc` version. Everything else depends on it only through the `FrameSource`
interface, so nothing else changes.

The USB-camera app you tested already proved the phone can ingest this dongle;
this is about wiring that same capability into the app.

## Status

- ✅ Tracker logic — validated in Python (tracking + scale adaptation).
- ✅ Pure-Kotlin core (`track/`) — type-checks against the real Kotlin compiler.
- ⏳ Android + UVC layers — build on device; `UvcFrameSource.kt` is the expected
  iteration point (library API), exactly as the navviz Android layers were.

## Lessons carried from navviz (so we don't repeat them)

AGP 8.7.2 pin (never AGP 9); RGBA/luma path with no JPEG round-trip; luminance
-only pipeline (cheap, and the feed is mono); tracker decoupled from Android
types for testability + onboard reuse.
