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
> validated: the JNI bridge into the real C++ controller (compiled and linked
> against a real `jni.h` + the real `MoveStopSense` API), and `Openness.kt`
> (transcribed back to Python and re-run against the same room/sky/wall scenes
> `tilt_bench.py`/`spin_map.py` use — this caught and fixed a real center-bias
> scaling bug). The device-facing plumbing (CameraX, ONNX Runtime I/O, sensor
> wiring, Gradle/AGP versions) follows documented patterns but needs the
> on-device iterate loop to shake out. Treat it as a strong starting point that
> boots you past the boilerplate, not as turnkey.

## What actually runs the algorithm

```
CameraX preview ─► midas_small.onnx via ONNX Runtime (NNAPI, throttled ~10 Hz)
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

**Depth model runs via ONNX Runtime, not TFLite** — deliberately, so it reuses
the *exact same* `midas_small.onnx` you already have for `tilt_bench.py`/
`spin_map.py` (the one at `~/depth_models/midas_small.onnx`), instead of
sourcing a separate, often gated/awkward-to-download TFLite export. Same model
file, same preprocessing (`MidasDepth.kt`'s ImageNet-mean/std NCHW input matches
the desktop `DepthNav`), one less thing to go wrong.

## Staged build order (validate the riskiest thing first)

1. **MiDaS on-device speed.** Copy the `midas_small.onnx` you already have
   (from `~/depth_models/midas_small.onnx`, downloaded for `tilt_bench.py`) into
   `app/src/main/assets/midas_small.onnx` (see below), build, and confirm real
   inference latency on the Magic V5. This validates the biggest claimed win —
   the Snapdragon 8 Elite NPU should run this in single-digit ms vs. ~90–110 ms
   on the Pi 5 CPU path. If this doesn't pan out, everything downstream
   reconsiders.
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

## Get the model — you likely already have it

No separate download needed. Copy the exact file `tilt_bench.py`/`spin_map.py`
already use:

```powershell
mkdir "app\src\main\assets" -Force
copy "$env:USERPROFILE\depth_models\midas_small.onnx" "app\src\main\assets\midas_small.onnx"
```

If you *don't* have it yet, see the desktop tool setup — same file, same
`curl.exe`/`Invoke-WebRequest` fetch from the MiDaS GitHub release, no gated
Hugging Face download required.

## Build

Open the `android/` folder in Android Studio, let it sync, plug in the phone
(USB debugging on), Run.

**If sync fails with a `NoSuchMethodError` mentioning `Project.exec` inside
native-build/CMake code:** that's an AGP/Gradle version mismatch — AGP < 8.7
calls a `Project.exec()` overload Gradle 9.0 removed. This project pins
AGP 9.2.0 + Gradle 9.1.0 (`gradle/wrapper/gradle-wrapper.properties`) to avoid
it; if your Android Studio still tries to use a different Gradle, force it via
**Settings → Build, Execution, Deployment → Gradle → Gradle JVM/Distribution →
"gradle-wrapper.properties file"**, then **File → Sync Project with Gradle
Files**. If `gradlew`/`gradlew.bat` are missing (only matters for CLI builds,
not Android-Studio-driven ones), regenerate them from Android Studio's
Terminal tab with a real Gradle install: `gradle wrapper --gradle-version
9.1.0 --distribution-type all`.

The app module must stay at `<repo>/android/` so the out-of-tree
`nav-sim/move_stop_sense.cpp` reference in CMake resolves; the build fails with a
clear message if it can't find it.
