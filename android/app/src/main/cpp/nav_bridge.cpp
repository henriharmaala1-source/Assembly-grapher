// nav_bridge.cpp — JNI bridge from the Android app to the REAL move-stop-sense
// controller. This does NOT reimplement the algorithm: it compiles and calls
// nav-sim/move_stop_sense.cpp verbatim (see CMakeLists.txt — single source of
// truth, the same file nav-sim and the onboard port share in spirit). The point
// of this app is to run that exact decision code against real phone camera +
// pose data instead of simulated raycasts, so anything reimplemented here would
// defeat the purpose.
//
// Handle-based: NavCore.kt holds an opaque jlong pointer to one MoveStopSense
// instance for the life of a nav session. All state (phase, stuck tally, latched
// scan direction) lives in that C++ object, untouched by this bridge.

#include <jni.h>
#include <cstring>

#include "move_stop_sense.hpp"

using navsim::MoveStopSense;
using navsim::MssInput;
using navsim::MssOutput;

namespace {
// Phase string -> stable index for the Kotlin side. Kept in the enum's order
// (SETTLE, THINK, SCAN, MOVE, ARRIVE, STUCK) so NavCore.Phase.values()[idx]
// lines up. Uses the phase NAME (not the private enum) so the C++ header stays
// unmodified.
int phaseIndex(const char* name) {
    if (std::strcmp(name, "SETTLE") == 0) return 0;
    if (std::strcmp(name, "THINK")  == 0) return 1;
    if (std::strcmp(name, "SCAN")   == 0) return 2;
    if (std::strcmp(name, "MOVE")   == 0) return 3;
    if (std::strcmp(name, "ARRIVE") == 0) return 4;
    if (std::strcmp(name, "STUCK")  == 0) return 5;
    return -1;
}
}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_kestrel_navviz_NavCore_nativeCreate(JNIEnv*, jobject) {
    return reinterpret_cast<jlong>(new MoveStopSense());
}

JNIEXPORT void JNICALL
Java_com_kestrel_navviz_NavCore_nativeReset(JNIEnv*, jobject, jlong handle) {
    if (handle) reinterpret_cast<MoveStopSense*>(handle)->reset();
}

JNIEXPORT void JNICALL
Java_com_kestrel_navviz_NavCore_nativeDestroy(JNIEnv*, jobject, jlong handle) {
    delete reinterpret_cast<MoveStopSense*>(handle);
}

// One controller tick. Inputs mirror MssInput 1:1; output is packed into a
// 6-float array: [bearingDeg, speedScale, yawScan(0/1), phaseIdx, wpE, wpN].
JNIEXPORT jfloatArray JNICALL
Java_com_kestrel_navviz_NavCore_nativeUpdate(
        JNIEnv* env, jobject, jlong handle,
        jfloat e, jfloat n, jfloat yawDeg, jfloat speedMs,
        jfloat corridorOpen, jfloat corridorOffset,
        jfloat goalBearing, jboolean planValid, jfloat planBearing,
        jfloat dt) {
    jfloatArray out = env->NewFloatArray(6);
    if (!handle) return out;

    MssInput in;
    in.e = e; in.n = n; in.yawDeg = yawDeg; in.speedMs = speedMs;
    in.corridorOpen = corridorOpen; in.corridorOffset = corridorOffset;
    in.goalBearing = goalBearing;
    in.planValid = (planValid == JNI_TRUE); in.planBearing = planBearing;

    MssOutput o = reinterpret_cast<MoveStopSense*>(handle)->update(in, dt);

    jfloat buf[6] = {
        o.bearingDeg, o.speedScale, o.yawScan ? 1.f : 0.f,
        static_cast<jfloat>(phaseIndex(o.phase)), o.wpE, o.wpN
    };
    env->SetFloatArrayRegion(out, 0, 6, buf);
    return out;
}

}  // extern "C"
