// What the VIO pose goes through before EKF3 (VioFrameAligner), and the IMU
// timestamp gate in front of the D435i's integrator (ImuGate).
//
// The aligner exists because ArduPilot reads a reset_counter change as "snap
// to this pose" and aligns VIO yaw at most once; these checks pin that a VIO
// re-anchor never reaches the FC as a jump, and that the VIO heading is held
// to the FC's without the position ever stepping.

#include <cmath>
#include <cstdio>

#include "frame_source.hpp"
#include "vio_frame_align.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {
constexpr float kD2R = 3.14159265f / 180.f;
float wrapDeg(float a) { while (a > 180.f) a -= 360.f; while (a < -180.f) a += 360.f; return a; }
}  // namespace

int main() {
    const float nan = std::nanf("");

    // --- no reference, no resets: a pass-through ----------------------------
    {
        VioFrameAligner a;
        VioFrameAligner::Pose in; in.n = 3; in.e = -1; in.d = -2; in.yaw = 0.4f; in.vn = 1;
        const auto o = a.update(in, 0, 0.0, nan);
        CHECK(std::fabs(o.n - 3) < 1e-6f && std::fabs(o.e + 1) < 1e-6f && std::fabs(o.d + 2) < 1e-6f);
        CHECK(std::fabs(o.yaw - 0.4f) < 1e-6f && std::fabs(o.vn - 1) < 1e-6f);
    }

    // --- the first sample snaps the VIO frame onto the FC heading ------------
    // so ArduPilot's own one-shot yaw trim (VOXL backend) comes out zero.
    {
        VioFrameAligner a;
        VioFrameAligner::Pose in; in.yaw = 10 * kD2R; in.vn = 1;   // VIO thinks 10 deg
        const auto o = a.update(in, 0, 0.0, 40 * kD2R);              // FC says 40
        CHECK(std::fabs(wrapDeg(o.yaw / kD2R - 40.f)) < 1e-3f);
        // Velocity rotated with it: 1 m/s "north" in a frame 30 deg off.
        CHECK(std::fabs(o.vn - std::cos(30 * kD2R)) < 1e-5f);
        CHECK(std::fabs(o.ve - std::sin(30 * kD2R)) < 1e-5f);
        std::printf("  first sample: VIO 10 deg -> sent 40 deg (the FC's)\n");
    }

    // --- a VIO re-anchor is absorbed: continuity, not a jump -----------------
    // Flying north at 2 m/s, sampled at 10 Hz. At t = 1.0 tracking is lost for
    // 0.3 s and the VIO re-anchors 5 m off. The FC must see the track carry
    // on at 2 m/s, and the reset_counter it is sent never changes.
    {
        VioFrameAligner a;
        VioFrameAligner::Pose o{};
        double t = 0.0;
        for (int k = 0; k <= 10; ++k, t += 0.1) {
            VioFrameAligner::Pose in; in.n = 2.f * float(t); in.vn = 2.f;
            o = a.update(in, 0, t, nan);
        }
        const float before = o.n;                                   // 2.0 at t = 1.0
        t += 0.2;                                                   // the 0.3 s gap
        VioFrameAligner::Pose in; in.n = 2.f * float(t) + 5.f; in.e = -3.f; in.vn = 2.f;
        o = a.update(in, 1, t, nan);
        CHECK(std::fabs(o.n - (before + 2.f * 0.3f)) < 1e-4f);      // extrapolated, not +5
        CHECK(std::fabs(o.e) < 1e-4f);
        CHECK(a.resetsAbsorbed() == 1);
        // And the new frame's own motion continues from there.
        t += 0.1;
        VioFrameAligner::Pose in2 = in; in2.n += 0.2f;
        const auto o2 = a.update(in2, 1, t, nan);
        CHECK(std::fabs(o2.n - o.n - 0.2f) < 1e-4f);
        std::printf("  re-anchor 5 m off after a 0.3 s gap: sent +0.60 m (2 m/s carried), no jump\n");
    }

    // --- yaw slews toward the FC, slowly, rotating about the CURRENT position -
    {
        VioFrameAligner a;
        VioFrameAligner::Pose in; in.n = 10; in.e = 0; in.yaw = 0;
        a.update(in, 0, 0.0, 0.f);                                   // aligned at 0
        // The FC's heading now says 20 deg (VIO drifted -20). One second later:
        float maxStep = 0.f;
        VioFrameAligner::Pose prev = a.update(in, 0, 0.02, 20 * kD2R);
        for (int k = 2; k <= 50; ++k) {
            const auto o = a.update(in, 0, 0.02 * k, 20 * kD2R);
            maxStep = std::max(maxStep, std::hypot(o.n - prev.n, o.e - prev.e));
            prev = o;
        }
        // Position never moves while the frame turns under it...
        CHECK(maxStep < 1e-4f);
        CHECK(std::fabs(prev.n - 10.f) < 1e-3f && std::fabs(prev.e) < 1e-3f);
        // ...and the turn is rate-limited (2 deg/s: at most 2 deg in 1 s).
        const float turned = a.yawOffsetRad() / kD2R;
        CHECK(turned > 1.5f && turned <= 2.0f + 1e-3f);
        // After a minute the gap is closed.
        for (int k = 51; k <= 3000; ++k) prev = a.update(in, 0, 0.02 * k, 20 * kD2R);
        CHECK(std::fabs(wrapDeg(prev.yaw / kD2R - 20.f)) < 0.5f);
        // Motion after alignment comes out in the FC's frame: VIO "north" is 20 deg.
        VioFrameAligner::Pose mv = in; mv.n += 1.f;
        const auto o = a.update(mv, 0, 0.02 * 3001, 20 * kD2R);
        const float hdg = std::atan2(o.e - prev.e, o.n - prev.n) / kD2R;
        CHECK(std::fabs(hdg - 20.f) < 0.5f);
        std::printf("  yaw: 20 deg gap, %.2f deg closed in 1 s, position fixed while turning; "
                    "closed in a minute\n", double(turned));
    }

    // --- the D435i IMU gate ---------------------------------------------------
    {
        sim::ImuGate g;
        CHECK(g.accept(true, 100.0));
        CHECK(g.accept(true, 105.0));
        CHECK(!g.accept(true, 105.0));          // a repeat
        CHECK(!g.accept(true, 103.0));          // a small step back
        CHECK(g.accept(false, 101.0));          // accel has its own clock
        CHECK(g.accept(true, 110.0));
        CHECK(g.rejected == 2);
        CHECK(!g.accept(true, std::nan("")));  // NaN is not a time
        CHECK(g.accept(true, 5000.0));
        CHECK(g.accept(true, 20.0));            // ~5 s back: the clock restarted
        CHECK(g.rebased == 1 && g.accept(true, 25.0));
        std::printf("  IMU gate: repeats and steps back dropped per stream; a clock restart re-bases\n");
    }

    std::printf("test_vio_align: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
