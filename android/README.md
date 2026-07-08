# navviz — Android app that runs the real nav algorithm on live phone data

A validation/demo app for the kestrel navigation logic. It compiles the
**actual** move-stop-sense controller (`nav-sim/move_stop_sense.cpp`, unmodified,
via the NDK) and drives it with a real phone camera + depth + pose instead of
nav-sim's simulated raycasts — so you can watch the real
`SETTLE → THINK → SCAN → MOVE → ARRIVE → STUCK` decision logic react to your
actual room. Target device: Honor Magic V5 (Snapdragon 8 Elite, Android 15,
Google Play Services present → ARCore viable).

> **Status: scaffold, not a finished app.** This was written without an Android
> SDK or device in the loop, so **nothing here has been compiled or run.** The
> parts most likely to be correct are the ones grounded in code that WAS
> validated: the JNI bridge into the real C++ controller, and `Openness.kt`
> (a port of the Python-tested `tilt_bench.py`/`spin_map.py` math). The
> device-facing plumbing (CameraX, TFLite I/O, sensor wiring) follows documented
> patterns but WILL need the on-device iterate loop to shake out — dependency
> versions, the exact MiDaS tensor layout, and the YUV→Bitmap path especially.
> Treat it as a strong starting point that boots you past the boilerplate, not
> as turnkey.

## What actually runs the algorithm

```
CameraX preview ─► MiDaS TFLite (on-device NPU, throttled ~10 Hz)
                     │
                     ▼
                Openness.kt  (horizon-band openness -> corridorOpen/Offset)   ← ported from tilt_bench.py
                     │
   GyroPoseProvider (yaw)    │
        (Stage 1-2)          ▼
                        NavCore (JNI) ─► nav-sim/move_stop_sense.cpp   ← the REAL controller, unmodified
                     │
                     ▼
                NavOverlayView  (phase, bearing, openness flag, bar strip, radar)
```

The C++ side is deliberately the genuine article: `CMakeLists.txt` references
`../../../../../nav-sim/move_stop_sense.cpp` directly (single source of truth),
and `nav_bridge.cpp` only marshals data — it contains no decision logic. That
file is self-contained (only `<algorithm>`/`<cmath>`), which is what makes the
reuse clean. The occupancy-grid goal-bias (`planValid`/`planBearing`) is left
off for the MVP (`planValid=false` = pure reactive) — wiring it in needs a small
OpenCV-free shim for `nav-sim/occupancy_grid.hpp`'s `sim_world.hpp` include; a
documented follow-on, not part of this scaffold.

## Staged build order (validate the riskiest thing first)

1. **MiDaS on-device speed.** Drop a MiDaS-small `.tflite` into
   `app/src/main/assets/midas_small.tflite` (see below), build, and confirm real
   inference latency on the Magic V5. This validates the biggest claimed win —
   the Snapdragon NPU should run this in single-digit ms vs. ~90–110 ms on the
   Pi 5 CPU path. If this doesn't pan out, everything downstream reconsiders.
2. **Openness + flag.** Already ported (`Openness.kt`). Point the camera at a
   wall vs. the ceiling and confirm USABLE/SUSPECT flips the way `tilt_bench.py`
   does on the desktop.
3. **Gyro yaw + the real controller.** `GyroPoseProvider` + `NavCore`. Rotate in
   place and watch the phase/bearing overlay — this is the SCAN-phase testbed
   (the real aircraft's SCAN is also yaw-only-in-place).
4. **ARCore 6-DoF (upgrade).** Swap `GyroPoseProvider` for an ARCore-backed
   provider to get real position + heading, enabling meaningful MOVE legs and
   walk-around mapping. **This is an architectural change, not a drop-in:**
   ARCore owns the camera, so the depth stage must read ARCore's acquired frames
   instead of CameraX. Deliberately not stubbed half-working here so Stage 1–3
   actually run. The `com.google.ar:core` dep and manifest hooks are already in
   place for when you take this on.

## Get the MiDaS model

The `.tflite` is fetched separately (gitignored, not committed):

- Official isl-org MiDaS Android sample + model: <https://github.com/isl-org/MiDaS/tree/master/mobile/android>
- Qualcomm-optimised exports (great fit for the Snapdragon NPU): <https://huggingface.co/qualcomm/Midas-V2>

Put it at `app/src/main/assets/midas_small.tflite`. **Verify its input size and
output layout against the model card** and reconcile `MidasDepth.kt`'s `inW/inH`
and the output-tensor shape — that's the single most likely thing to need
adjusting, since exports differ (256 vs other sizes, NCHW vs NHWC, inverse-depth
vs not).

## Build

Open the `android/` folder in Android Studio (Giraffe+), let it sync, plug in the
phone (USB debugging on), Run. If Gradle/AGP/Kotlin versions in
`build.gradle.kts` don't match your installed toolchain, bump them — they're a
known-good starting set, not pinned to your exact SDK.

The app module must stay at `<repo>/android/` so the out-of-tree
`nav-sim/move_stop_sense.cpp` reference in CMake resolves; the build fails with a
clear message if it can't find it.
