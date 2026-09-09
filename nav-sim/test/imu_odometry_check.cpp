// Short-memory IMU odometry, measured against a known trajectory.
//
// `POSE_AND_OPENNESS_PLAN.md` §6 says do not use this IMU for translation, and
// is right about the mechanism: attitude coupling, not accel bias. This pins
// how long the warning takes to bite, because the answer decides whether the
// near map can have a memory at all.
//
//   g++ -O2 -std=c++17 -I. test/imu_odometry_check.cpp -o /tmp/imu && /tmp/imu

#include <cmath>
#include <cstdio>
#include <string>

#include "imu_odometry.hpp"

using namespace sim;
static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-62s %s%s%s\n", what, ok ? "ok" : "FAIL",
                d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++fails;
}

// Fly a KNOWN trajectory, feed the integrator accelerations corrupted only by a
// constant tilt error, and compare against truth. A constant bias is the honest
// worst case: a tilt error does not average out, which is exactly why it beats
// white noise as the dominant term.
static float driftAfter(float seconds, float attErrDeg, float dt = 0.005f) {
    ImuOdometryParams p; p.attErrDeg = attErrDeg; p.maxDriftM = 1e9f;  // no cutoff
    ImuOdometry o; o.init(p);
    const float leak = 9.81f * std::sin(attErrDeg * 3.14159265f / 180.f);
    float tx = 0.f, tvx = 0.f;                 // truth, constant-velocity cruise
    const float trueA = 0.f;
    for (float t = 0; t < seconds; t += dt) {
        o.step(trueA + leak, 0.f, 0.f, dt);    // measured = truth + tilt leak
        tvx += trueA * dt; tx += tvx * dt;      // truth integrates the truth
    }
    float x, y, z; o.position(x, y, z);
    return std::fabs(x - tx);
}

int main() {
    std::printf("short-memory IMU odometry\n");

    // --- the formula must match reality, not just sound right --------------
    for (float deg : {0.5f, 1.0f, 2.0f}) {
        ImuOdometryParams p; p.attErrDeg = deg;
        ImuOdometry o; o.init(p);
        const float win = o.usableWindowS();
        const float measured = driftAfter(win, deg);
        check(std::fabs(measured - p.maxDriftM) < 0.02f,
              "predicted window really does cost one cell of drift",
              std::to_string(deg) + " deg -> " + std::to_string(win) +
              " s, measured " + std::to_string(measured) + " m");
    }

    // --- and the window is where the plan implies -------------------------
    {
        ImuOdometryParams p; p.attErrDeg = 1.0f;
        ImuOdometry o; o.init(p);
        check(o.usableWindowS() > 1.5f && o.usableWindowS() < 2.0f,
              "at 1 deg attitude error the window is ~1.7 s",
              std::to_string(o.usableWindowS()) + " s");
        // §3 wants 2-3 s to plan through a turn. So this DOES NOT reach it at
        // 1 deg -- which is the finding, not a failure of the code.
        check(o.usableWindowS() < 2.0f,
              "which is BELOW the 2-3 s the map wants: honest shortfall");
    }

    // --- it must refuse rather than extrapolate ---------------------------
    {
        ImuOdometryParams p; p.attErrDeg = 1.0f; p.maxDriftM = 0.25f;
        ImuOdometry o; o.init(p);
        for (int i = 0; i < 200; ++i) o.step(0.f, 0.f, 0.f, 0.01f);   // 2 s
        check(!o.valid(), "past its budget the pose is marked INVALID");
        o.zupt();
        check(o.valid(), "and a zero-velocity update makes it usable again");
        check(o.sinceResetS() == 0.f, "the error clock restarts on ZUPT");
    }

    // --- ZUPT is what makes it viable, and by how much ---------------------
    {
        // Move-stop-sense: 1.5 s of motion, then stationary. Without a ZUPT the
        // velocity error persists and position runs away; with one it does not.
        ImuOdometryParams p; p.attErrDeg = 1.0f; p.maxDriftM = 1e9f;
        const float leak = 9.81f * std::sin(1.0f * 3.14159265f / 180.f);
        const float dt = 0.005f;

        ImuOdometry noZ; noZ.init(p);
        ImuOdometry withZ; withZ.init(p);
        for (int leg = 0; leg < 4; ++leg) {
            for (float t = 0; t < 1.5f; t += dt) {          // MOVE
                noZ.step(leak, 0, 0, dt); withZ.step(leak, 0, 0, dt);
            }
            for (float t = 0; t < 1.0f; t += dt) {          // SETTLE / THINK
                noZ.step(leak, 0, 0, dt); withZ.step(leak, 0, 0, dt);
            }
            withZ.zupt();                                    // free, from the mode
        }
        float ax, ay, az, bx, by, bz;
        noZ.position(ax, ay, az); withZ.position(bx, by, bz);
        check(std::fabs(bx) < std::fabs(ax) * 0.5f,
              "ZUPT at every stop more than halves accumulated drift",
              std::to_string(ax) + " m -> " + std::to_string(bx) + " m");
    }

    // --- and the tilt term really does dominate the noise term -------------
    {
        // 1 mg of accel bias over 2 s, against 1 deg of tilt over 2 s.
        const float biasDrift = 0.5f * 0.0098f * 4.f;
        const float tiltDrift = driftAfter(2.f, 1.0f);
        check(tiltDrift > 5.f * biasDrift,
              "attitude coupling beats accel bias by an order of magnitude",
              std::to_string(tiltDrift) + " m vs " + std::to_string(biasDrift) + " m");
    }

    std::printf(fails ? "FAILED (%d failures)\n" : "all checks passed (%d failures)\n",
                fails);
    return fails ? 1 : 0;
}
