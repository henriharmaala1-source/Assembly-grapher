// The bearing field's three subtle behaviours, pinned.
//
// This is new code carrying the whole far field, and every one of these was a
// bug I wrote and then found by looking at a picture. A picture is not a test.
//
//   g++ -O2 -std=c++17 -I. test/bearing_field_check.cpp bearing_field.cpp \
//       depth_camera.cpp voxel_world.cpp -I/usr/include/opencv4 \
//       -lopencv_core -lopencv_imgproc -o /tmp/bf && /tmp/bf

#include <cmath>
#include <cstdio>
#include <string>

#include "bearing_field.hpp"
#include "depth_camera.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-62s %s%s%s\n", what, ok ? "ok" : "FAIL",
                d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++fails;
}

int main() {
    std::printf("bearing field\n");

    CamParams cp;
    cp.width = 424; cp.height = 240;
    cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    DepthCamera cam(cp);
    CamPose pose;                       // origin, level, facing North

    auto flat = [&](float r) { return cv::Mat(cp.height, cp.width, CV_32F,
                                              cv::Scalar(r)); };

    // --- confirmation: one frame is not enough --------------------------
    // The voxel map needs several hits to cross occThresh, and that is most of
    // why it does not speckle. A field that believes the first frame has no
    // such filter, and the pane filled with single-bin outliers until it did.
    {
        BearingField bf; BearingFieldParams p; bf.init(p);
        bf.update(flat(6.f), cam, pose);
        check(bf.rangeAt(0.f, 0.f) < 0.f,
              "a surface seen ONCE is not yet reported");
        bf.update(flat(6.f), cam, pose);
        const float r = bf.rangeAt(0.f, 0.f);
        check(r > 5.5f && r < 6.5f, "and is reported once it agrees with itself",
              std::to_string(r) + " m");
    }

    // --- the running-minimum trap ---------------------------------------
    // Minimum-of-N is biased low and the bias GROWS with N, so a bin that keeps
    // a running minimum creeps toward the nearest outlier it has ever seen and
    // never recovers. Within a frame take the nearest; across frames REPLACE.
    {
        BearingField bf; BearingFieldParams p; bf.init(p);
        for (int i = 0; i < 4; ++i) bf.update(flat(8.f), cam, pose);
        // One frame of a much nearer surface, then the truth returns.
        bf.update(flat(2.f), cam, pose);
        for (int i = 0; i < 4; ++i) bf.update(flat(8.f), cam, pose);
        const float r = bf.rangeAt(0.f, 0.f);
        check(r > 7.f, "a one-frame near outlier does not capture the bin for ever",
              std::to_string(r) + " m, expected ~8");
    }

    // --- the sample floor ------------------------------------------------
    // A single stereo outlier IS the nearest sample in its bin by construction,
    // so a bare minimum is maximally sensitive to the one thing this sensor
    // produces most.
    {
        BearingField bf; BearingFieldParams p; p.minSamples = 4; bf.init(p);
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(-1.f));
        d.at<float>(cp.height / 2, cp.width / 2) = 5.f;      // exactly one pixel
        for (int i = 0; i < 6; ++i) bf.update(d, cam, pose);
        check(bf.rangeAt(0.f, 0.f) < 0.f,
              "one lone pixel never becomes a surface");
        // Enough pixels in the same bin, and it does.
        for (int v = cp.height/2 - 3; v <= cp.height/2 + 3; ++v)
            for (int u = cp.width/2 - 3; u <= cp.width/2 + 3; ++u)
                d.at<float>(v, u) = 5.f;
        for (int i = 0; i < 6; ++i) bf.update(d, cam, pose);
        const float r = bf.rangeAt(0.f, 0.f);
        check(r > 4.5f && r < 5.5f, "a patch of them does", std::to_string(r) + " m");
    }

    // --- yaw is an exact index shift -------------------------------------
    // The whole per-frame cost collapses to a table lookup because a rotation
    // about the vertical is a rotation of the azimuth index. If that is wrong
    // the field smears every time the aircraft turns, and it would look like
    // sensor noise rather than like a bug.
    {
        BearingField bf; BearingFieldParams p; bf.init(p);
        CamPose east; east.yawDeg = 90.f;
        for (int i = 0; i < 3; ++i) bf.update(flat(7.f), cam, east);
        const float ahead = bf.rangeAt(90.f, 0.f);   // world bearing 90 = East
        const float behind = bf.rangeAt(270.f, 0.f);
        check(ahead > 6.5f && ahead < 7.5f,
              "yawed 90 deg, the surface lands at world bearing 90",
              std::to_string(ahead) + " m");
        check(behind < 0.f, "and nothing appears behind the aircraft");
    }

    // --- forgetting -------------------------------------------------------
    // A bearing bin cannot be carved -- there is no "I looked through it" -- so
    // the only way it releases a stale obstacle is by expiring. Without that it
    // would remember a tree for ever after flying past it.
    {
        BearingField bf; BearingFieldParams p; p.forgetFrames = 5; bf.init(p);
        for (int i = 0; i < 3; ++i) bf.update(flat(6.f), cam, pose);
        check(bf.rangeAt(0.f, 0.f) > 0.f, "a seen surface is held");
        cv::Mat empty(cp.height, cp.width, CV_32F, cv::Scalar(-1.f));
        for (int i = 0; i < 8; ++i) bf.update(empty, cam, pose);
        check(bf.rangeAt(0.f, 0.f) < 0.f, "and released once it expires");
    }

    // --- OBSTACLE_DISTANCE shape -----------------------------------------
    // ArduPilot wants distances by bearing, clockwise from the nose. Getting
    // the origin or the direction wrong would steer its avoidance layer into
    // the obstacle rather than away, which is worse than not sending it.
    {
        BearingField bf; BearingFieldParams p; bf.init(p);
        // A surface ahead only: a narrow band of columns about the centre.
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(-1.f));
        for (int v = 0; v < cp.height; ++v)
            for (int u = cp.width/2 - 12; u <= cp.width/2 + 12; ++u)
                d.at<float>(v, u) = 4.f;
        for (int i = 0; i < 3; ++i) bf.update(d, cam, pose);
        const std::vector<float> od = bf.obstacleDistance(0.f, 72);
        check(od.size() == 72, "72 bins");
        check(od[0] > 3.5f && od[0] < 4.5f,
              "bin 0 is straight ahead and reads the surface",
              std::to_string(od[0]) + " m");
        check(od[36] < 0.f, "bin 36 is astern and knows nothing",
              std::to_string(od[36]));
    }

    std::printf(fails ? "FAILED (%d failures)\n" : "all checks passed (%d failures)\n",
                fails);
    return fails ? 1 : 0;
}
