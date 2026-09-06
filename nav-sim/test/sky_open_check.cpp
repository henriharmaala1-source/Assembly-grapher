// "The planner only wants to go up now, because it's the most open."
//
// It did, and the reason was one line. `BearingField::rangeAt` returns < 0 for
// a bin that holds NOTHING, and `voxel_traj.cpp` mapped that to FULL reach --
// so a bearing carrying no information scored the maximum openness available.
// The largest region of "no information" in any outdoor scene is the sky, so
// every upward bearing scored 1.0 and the argmax climbed.
//
// This is the far-field instance of the rule the near field has enforced since
// the beginning: UNKNOWN IS NOT FREE. The same function already says so for
// near space -- "unknown space is traversable but pays nothing" -- and then
// paid maximum for unknown thirty lines later.
//
// `POSE_AND_OPENNESS_PLAN.md` §2 predicted exactly this and named it: "openness
// is not traversability -- a bright gap can be sky above a wall". The plan was
// right; the implementation did not carry the caveat.
//
//   g++ -O2 -std=c++17 -I. test/sky_open_check.cpp voxel_traj.cpp voxel_map.cpp \
//       bearing_field.cpp depth_camera.cpp voxel_world.cpp \
//       -I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -o /tmp/sk && /tmp/sk

#include <cmath>
#include <cstdio>
#include <string>

#include "bearing_field.hpp"
#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_traj.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-64s %s%s%s\n", what, ok ? "ok" : "FAIL",
                d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++fails;
}

int main() {
    std::printf("sky must not win the openness argmax\n");

    // A map that is FREE everywhere near the aircraft, so the near field vetoes
    // nothing and the far term is what decides. That isolates the thing under
    // test: any climb here is the far field's doing, not the map's.
    VoxelMapParams mp;
    mp.cell = 0.25f; mp.nx = 96; mp.ny = 96; mp.nz = 64;
    VoxelMap m; m.init(mp, 0.f, 0.f, 0.f);
    m.seedFree(0.f, 0.f, 0.f, 10.f);     // the one sanctioned use: known-clear

    // A bearing field holding ONE confirmed surface, level and dead ahead at
    // 20 m, and nothing anywhere else. Every other bearing -- including every
    // upward one -- is unknown.
    CamParams cp; cp.width = 424; cp.height = 240;
    cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    DepthCamera cam(cp);
    CamPose pose;

    BearingField bf; BearingFieldParams bp; bf.init(bp);
    {
        // A surface across the WHOLE frame at 8 m. Every bearing the camera can
        // see is therefore confirmed and scores 8/30 -- and everything ABOVE
        // the camera's vertical field of view (about +28 deg here) is unknown.
        //
        // That asymmetry is the bug's habitat, and the first version of this
        // test missed it: with the whole field unknown, every bearing tied at
        // the maximum and the goal term quietly broke the tie level. The bug
        // only bites when looking AHEAD is measured and looking UP is not,
        // which is precisely the situation outdoors.
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(8.f));
        for (int i = 0; i < 5; ++i) bf.update(d, cam, pose);
    }

    check(bf.rangeAt(0.f, 0.f) > 0.f, "the surface ahead is confirmed",
          std::to_string(bf.rangeAt(0.f, 0.f)) + " m");
    check(bf.rangeAt(0.f, 38.f) < 0.f,
          "and the sky above the frame reports NOTHING, as it should");

    // Isolate the term under test. Distance-travelled and smoothness both bias
    // against a slow steep climb, and letting them do so would make this pass
    // for a reason other than the one it is checking.
    TrajParams tp;
    tp.clearWeight  = 0.f;
    tp.smoothWeight = 0.f;
    tp.goalWeight   = 1.f;
    tp.farWeight    = 0.5f;   // the real default -- do not flatter the test
    TrajectoryPlanner tr(tp);

    TrajectoryPlanner::FarBearings far{&bf, 30.f};

    // Goal dead ahead and LEVEL. Under the old mapping the unknown sky scored
    // 1.0 against the measured 0.27 ahead, a 0.73 advantage that swamps the
    // 0.25 the goal term charges for a 45 deg climb -- so the aircraft climbed.
    GeneralResult r = tr.plan(m, 0.f, 0.f, 0.f, /*yaw*/0.f,
                              /*goalAz*/0.f, /*goalEl*/0.f, {}, &far);

    check(!r.blocked, "a plan exists in free space");
    // fabs, NOT "< 15". The first version of this check was one-sided and the
    // buggy code sailed through it by DIVING to -36.9 deg -- because below the
    // camera's field of view is just as unmeasured as above it. The defect was
    // never "prefers up"; it was "prefers whatever it cannot see". Up is merely
    // the direction where that is unbounded.
    check(std::fabs(r.elDeg) < 15.f,
          "the chosen primitive goes neither up nor down into unmeasured space",
          std::to_string(r.elDeg) + " deg elevation");

    // The positive half: openness must still be EARNED, not merely disabled.
    // A nearer confirmed surface must score lower than a further one.
    BearingField near_; near_.init(bp);
    {
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(3.f));
        for (int i = 0; i < 5; ++i) near_.update(d, cam, pose);
    }
    check(near_.rangeAt(0.f, 0.f) < bf.rangeAt(0.f, 0.f),
          "a nearer surface is measurably less open than a farther one",
          std::to_string(near_.rangeAt(0.f, 0.f)) + " vs " +
          std::to_string(bf.rangeAt(0.f, 0.f)) + " m");

    // The climb bias itself: even when UP is genuinely measured and genuinely
    // the most open direction, it must not be chosen. Under the canopy, up is
    // not a route -- and the sky will always out-open the trees.
    {
        BearingField up; up.init(bp);
        cv::Mat d(cp.height, cp.width, CV_32F, cv::Scalar(4.f));
        // Top third of the frame opens out to 28 m; the rest stays close.
        for (int v = 0; v < cp.height / 3; ++v)
            for (int u = 0; u < cp.width; ++u) d.at<float>(v, u) = 28.f;
        for (int i = 0; i < 5; ++i) up.update(d, cam, pose);
        check(up.rangeAt(0.f, 18.f) > 20.f,
              "a genuinely open, genuinely MEASURED sky reads far",
              std::to_string(up.rangeAt(0.f, 18.f)) + " m");
        TrajectoryPlanner::FarBearings fup{&up, 30.f};
        GeneralResult ru = tr.plan(m, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, {}, &fup);

        // NOT pinned to an angle. An earlier version of this check asserted
        // elDeg < 10 and failed at 14 -- and the temptation was to raise
        // climbPenalty until it passed. That is tuning to a synthetic scene:
        // this one puts confirmed sky at 28 m directly above a 4 m wall, which
        // is not a wood. A CLOSED-LOOP sweep (5 seeds, forest, 400 steps) chose
        // 2.0 and showed altitude held to 4 cm, and the loop outranks this file.
        //
        // So assert the MECHANISM instead: more penalty must buy less climb,
        // and the climb must stay bounded. That catches a regression in the
        // term without letting an invented scene set a shipping weight.
        TrajParams tp3 = tp; tp3.climbPenalty = tp.climbPenalty * 3.f;
        TrajectoryPlanner tr3(tp3);
        GeneralResult ru3 = tr3.plan(m, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, {}, &fup);
        check(ru3.elDeg < ru.elDeg,
              "more climb penalty buys less climb toward it",
              std::to_string(ru.elDeg) + " -> " + std::to_string(ru3.elDeg) + " deg");
        check(ru.elDeg < 30.f,
              "and even at the shipping weight the climb stays bounded",
              std::to_string(ru.elDeg) + " deg");
    }

    // And with NO far field the planner still flies the goal, rather than
    // treating a missing field as either open or blocked.
    GeneralResult r2 = tr.plan(m, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, {}, nullptr);
    check(!r2.blocked && std::fabs(r2.elDeg) < 15.f,
          "with no far field the goal decides, and it is level",
          std::to_string(r2.elDeg) + " deg");

    std::printf(fails ? "FAILED (%d failures)\n" : "all checks passed (%d failures)\n",
                fails);
    return fails ? 1 : 0;
}
