// Flow x depth -> metric velocity, against KNOWN motion.
//
// The sim has no appearance model, so real IR frames cannot be rendered here.
// Instead: synthesise a textured image, warp it by an EXACTLY known camera
// motion at an exactly known depth, and check the recovered velocity. That
// tests the estimator -- the de-rotation, the solve, the gating -- which is the
// part that can be wrong in ways a real camera would not reveal until flight.
//
//   g++ -O2 -std=c++17 -I. test/flow_velocity_check.cpp flow_velocity.cpp \
//       depth_camera.cpp voxel_world.cpp -I/usr/include/opencv4 \
//       -lopencv_core -lopencv_imgproc -o /tmp/fvc && /tmp/fvc

#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/imgproc.hpp>

#include "flow_velocity.hpp"

using namespace sim;
static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-60s %s%s%s\n", what, ok ? "ok" : "FAIL",
                d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++fails;
}

// Deterministic high-frequency texture. Real bark is not this cooperative, but
// the estimator's correctness does not depend on the texture being hard -- only
// its yield does, and yield is what a real-footage test would measure.
static cv::Mat texture(int W, int H, float ox = 0.f, float oy = 0.f) {
    cv::Mat m(H, W, CV_8U);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const float u = x + ox, v = y + oy;
            // Deliberately APERIODIC. A purely periodic pattern aliases, and a
            // forward-backward check cannot reject an alias that matches
            // consistently in both directions -- the first version of this
            // texture did exactly that and produced confident nonsense.
            const float s = std::sin(u * 0.7f) * std::sin(v * 0.9f)
                          + std::sin((u + v) * 0.31f) * 0.7f
                          + std::sin(u * 0.113f - v * 0.171f) * 0.5f
                          + std::sin(u * 0.041f + v * 0.029f) * 0.9f;
            m.at<uint8_t>(y, x) = uint8_t(128 + 60 * s);
        }
    return m;
}

int main() {
    std::printf("flow x depth -> metric velocity\n");

    CamParams cp; cp.width = 320; cp.height = 240; cp.hfovDeg = 87.f;
    DepthCamera cam(cp);
    const float f = cam.fpx();

    FlowVelocityParams fp; FlowVelocityEstimator est; est.init(fp);
    const float dt = 0.1f;

    // --- pure lateral translation at a known depth -------------------------
    {
        const float Z = 3.f, vxTrue = 1.2f;          // m/s to the right
        // Camera moving +x makes the image move -x by f*tx/Z pixels.
        const float shiftPx = f * (vxTrue * dt) / Z;
        cv::Mat a = texture(cp.width, cp.height);
        cv::Mat b = texture(cp.width, cp.height, shiftPx, 0.f);
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(Z));
        FlowVelocity r = est.estimate(a, b, d, cam, 0, 0, 0, dt);
        check(r.valid, "a textured pair at known depth yields a solution",
              std::to_string(r.points) + " points");
        check(std::fabs(r.vx - vxTrue) < 0.15f, "lateral velocity recovered",
              std::to_string(r.vx) + " vs " + std::to_string(vxTrue) + " m/s");
        check(std::fabs(r.vy) < 0.15f, "and no phantom vertical motion",
              std::to_string(r.vy));
    }

    // --- DE-ROTATION: a pure yaw must NOT read as translation --------------
    // This is the check that matters most. During a turn, rotational flow dwarfs
    // translational flow; undivided, a yaw looks like enormous sideways motion.
    {
        const float Z = 3.f;
        // 2 deg per frame = 20 deg/s. Kept small ON PURPOSE: at 6 deg the
        // image shift is 17.6 px, past the 10 px search window, and the
        // matcher would be failing rather than the de-rotation working.
        const float dYawDeg = 2.f;
        const float shiftPx = f * (dYawDeg * 3.14159265f / 180.f);
        cv::Mat a = texture(cp.width, cp.height);
        // +ox samples the texture FURTHER RIGHT, so content appears to move
        // LEFT -- which is what yawing RIGHT does. The first version had this
        // inverted, and de-rotation then DOUBLED the error instead of
        // cancelling it. The estimator was right; the test built the wrong
        // world for it.
        cv::Mat b = texture(cp.width, cp.height, shiftPx, 0.f);
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(Z));

        FlowVelocity naive = est.estimate(a, b, d, cam, 0, 0, 0, dt);
        FlowVelocity derot = est.estimate(a, b, d, cam, dYawDeg, 0, 0, dt);
        check(naive.valid && std::fabs(naive.vx) > 0.5f,
              "UNDE-ROTATED, a 20 deg/s yaw reads as large translation",
              std::to_string(naive.vx) + " m/s of phantom motion");
        check(derot.valid && std::fabs(derot.vx) < 0.2f,
              "de-rotated with the FC attitude, it reads as ~zero",
              std::to_string(derot.vx) + " m/s");
    }

    // --- textureless input must be REFUSED, not guessed --------------------
    {
        cv::Mat flat(cp.height, cp.width, CV_8U, cv::Scalar(120));
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(3.f));
        FlowVelocity r = est.estimate(flat, flat, d, cam, 0, 0, 0, dt);
        check(!r.valid, "a flat wall yields no velocity at all, rather than zero",
              std::to_string(r.points) + " points survived");
    }

    // --- depth gating: far-only scenes carry no parallax -------------------
    {
        cv::Mat a = texture(cp.width, cp.height);
        cv::Mat b = texture(cp.width, cp.height, 2.f, 0.f);
        cv::Mat far(cp.height, cp.width, CV_32F, cv::Scalar(40.f));  // past maxRange
        FlowVelocity r = est.estimate(a, b, far, cam, 0, 0, 0, dt);
        check(!r.valid, "points beyond the useful depth band are excluded",
              std::to_string(r.points) + " points");
    }

    // --- scale really does come from depth ---------------------------------
    // The same pixel flow at twice the range is twice the velocity. If this
    // fails, the estimator is reporting pixels wearing metric units.
    {
        cv::Mat a = texture(cp.width, cp.height);
        cv::Mat b = texture(cp.width, cp.height, 12.f, 0.f);
        cv::Mat d1(cp.height, cp.width, CV_32F, cv::Scalar(2.f));
        cv::Mat d2(cp.height, cp.width, CV_32F, cv::Scalar(4.f));
        FlowVelocity r1 = est.estimate(a, b, d1, cam, 0, 0, 0, dt);
        FlowVelocity r2 = est.estimate(a, b, d2, cam, 0, 0, 0, dt);
        check(r1.valid && r2.valid && std::fabs(r2.vx / r1.vx - 2.f) < 0.15f,
              "doubling the depth doubles the reported speed",
              std::to_string(r1.vx) + " -> " + std::to_string(r2.vx) + " m/s");
    }

    std::printf(fails ? "FAILED (%d failures)\n" : "all checks passed (%d failures)\n",
                fails);
    return fails ? 1 : 0;
}
