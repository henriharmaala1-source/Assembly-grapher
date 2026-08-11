// voxel_live -- point the REAL navigation stack at real depth.
//
//   voxel_live --replay walk.kdr          a recording, no camera needed
//   voxel_live --live                     a D435i, if this build has librealsense
//   voxel_live --sim                      the raycaster, as a control
//
// WHAT THIS IS FOR. Every number in this project comes from synthetic depth.
// The map, the carve limits and the trajectory library are all arithmetic that
// behaves identically on a desktop and on the aircraft -- but the depth image
// is the one thing that can lie, and it has never been a real one. This runs
// the SAME VoxelMap and the SAME TrajectoryPlanner over frames from an actual
// camera, and draws what each of them believes.
//
// It is not a flight. There is no vehicle, no control loop and no odometry:
// the camera sits still (or you turn it by hand with --yawrate) and the map
// accumulates from a known pose. That is a deliberate limit, not an oversight
// -- see POSE below.
//
// POSE IS THE HONEST LIMIT. A world-anchored map needs to know where each frame
// was taken from. The simulator has perfect pose by construction; a handheld
// camera has none, and inventing one would produce a map that looks plausible
// and means nothing. So this offers only what can be justified:
//
//   --pose fixed        the camera does not move. The map is built from one
//                       viewpoint. Everything the mapper does -- carving,
//                       marking, the three states, the honest range -- is
//                       exercised, and it is all trustworthy.
//   --pose spin         the camera rotates in place at a rate YOU state. Useful
//                       on a turntable or a tripod head with a marked scale;
//                       nonsense if you just wave it about.
//
// Translation is not offered at all, because we cannot measure it yet. When
// StateEstimator grows odometry it plugs in here and the rest is unchanged.
//
// WHAT TO LOOK FOR, in the order it matters:
//   1. the DEPTH pane -- how much comes back at all, and where the holes are
//   2. the MAP pane -- grey is UNKNOWN and is not empty. Compare its extent
//      with the honest-range ring: past that ring we deliberately stop carving
//   3. the PLAN pane -- which primitives survive the swept-volume test. A
//      forest of rejected candidates and one thin green survivor is the system
//      working, not failing

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#if SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "depth_record.hpp"
#include "frame_source.hpp"
#include "voxel_map.hpp"
#include "voxel_traj.hpp"
#include "voxel_world.hpp"

using namespace sim;

namespace {

// Depth -> colour. Near red, far blue, INVALID mid-grey and not black, because
// black reads as "far" to the eye and the entire point of the three-state map
// is that missing data is not distance.
cv::Mat colourDepth(const cv::Mat& d, float maxM) {
    cv::Mat out(d.rows, d.cols, CV_8UC3, cv::Scalar(90, 90, 90));
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            const float t = std::min(1.f, r[x] / std::max(0.1f, maxM));
            // OpenCV 8-bit hue is 0..179, NOT 0..359. The first version added a
            // 120 offset on top of a 0..120 ramp, so everything past ~6 m wrapped
            // through magenta and read as NEARER than the foreground -- a colour
            // map that inverts its own meaning at range.
            const int hue = int(t * 120.f);          // 0 = red near, 120 = blue far
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(hue), 200, 230);
        }
    }
    cv::Mat bgr;
    cv::cvtColor(out, bgr, cv::COLOR_HSV2BGR);
    // Repaint the invalid pixels AFTER the conversion, so they are a flat grey
    // rather than whatever the hue ramp does at the ends.
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x)
            if (!(r[x] > 0.f)) bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(90, 90, 90);
    }
    return bgr;
}

void banner(cv::Mat& img, const std::string& text, int y = 22) {
    cv::putText(img, text, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                {20, 20, 20}, 3, cv::LINE_AA);
    cv::putText(img, text, {10, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                {245, 245, 245}, 1, cv::LINE_AA);
}

cv::Mat fit(const cv::Mat& src, int w, int h) {
    cv::Mat out;
    cv::resize(src, out, {w, h}, 0, 0, cv::INTER_NEAREST);
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    std::string mode = "replay", path, out = "/tmp/voxel_live";
    float cell = 0.25f, robotR = 0.6f, vmax = 1.5f;
    float yawRateDps = 0.f, pitchDeg = 0.f, maxIntegOverride = -1.f;
    int   camW = 848, camH = 480, fps = 30, steps = -1;
    bool  emitter = false, headless = false, showTruth = false;

    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--replay")) { mode = "replay"; path = next(""); }
        else if (!std::strcmp(argv[i], "--live")) mode = "live";
        else if (!std::strcmp(argv[i], "--sim")) mode = "sim";
        else if (!std::strcmp(argv[i], "--cell")) cell = float(std::atof(next("0.25")));
        else if (!std::strcmp(argv[i], "--vmax")) vmax = float(std::atof(next("1.5")));
        else if (!std::strcmp(argv[i], "--robot")) robotR = float(std::atof(next("0.6")));
        else if (!std::strcmp(argv[i], "--yawrate")) yawRateDps = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--pitch")) pitchDeg = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--maxinteg")) maxIntegOverride = float(std::atof(next("-1")));
        else if (!std::strcmp(argv[i], "--camw")) camW = std::atoi(next("848"));
        else if (!std::strcmp(argv[i], "--camh")) camH = std::atoi(next("480"));
        else if (!std::strcmp(argv[i], "--fps")) fps = std::atoi(next("30"));
        else if (!std::strcmp(argv[i], "--emitter")) emitter = true;
        else if (!std::strcmp(argv[i], "--frames")) steps = std::atoi(next("-1"));
        else if (!std::strcmp(argv[i], "--out")) out = next("/tmp/voxel_live");
        else if (!std::strcmp(argv[i], "--headless")) headless = true;
        else if (!std::strcmp(argv[i], "--truth")) showTruth = true;
        else if (!std::strcmp(argv[i], "--help")) {
            std::printf(
                "voxel_live -- the real map and planner over real depth\n"
                "  --replay FILE.kdr   a recording (no camera needed)\n"
                "  --live              a connected D435i%s\n"
                "  --sim               the raycaster, as a control\n"
                "  --cell 0.25         voxel size, m\n"
                "  --yawrate 0         camera rotation in place, deg/s (KNOWN rate only)\n"
                "  --pitch 0           camera pitch, deg (nose-up positive)\n"
                "  --vmax 1.5          speed cap for the planner's budget\n"
                "  --emitter           IR projector on (default off: the outdoor case)\n"
                "  --frames N          stop after N frames\n"
                "  --headless          no window; write PNGs to --out\n",
                haveLiveSupport() ? "" : "  [NOT in this build -- no librealsense]");
            return 0;
        }
    }

    // --- source ------------------------------------------------------------
    std::unique_ptr<FrameSource> src;
    std::unique_ptr<VoxelWorld> world;
    std::string err;

    if (mode == "replay") {
        if (path.empty()) { std::fprintf(stderr, "--replay needs a .kdr file\n"); return 2; }
        auto r = std::unique_ptr<ReplayFrameSource>(new ReplayFrameSource());
        if (!r->open(path, &err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 2; }
        const DepthRecordHeader& h = r->header();
        std::printf("[replay] %s: %u frames %ux%u, fx %.1f fy %.1f ppx %.1f ppy %.1f, "
                    "baseline %.1f mm, emitter %s\n",
                    path.c_str(), h.frames, h.width, h.height, h.fx, h.fy, h.ppx, h.ppy,
                    h.baselineM * 1000.f,
                    (h.flags & DepthRecordHeader::FLAG_EMITTER_ON) ? "ON" : "off");
        src = std::move(r);
    } else if (mode == "live") {
        src = makeLiveSource(camW, camH, fps, emitter, &err);
        if (!src) { std::fprintf(stderr, "[live] %s\n", err.c_str()); return 2; }
        std::printf("[live] streaming %dx%d @ %d, emitter %s\n",
                    camW, camH, fps, emitter ? "ON" : "off");
    } else {
        world.reset(new VoxelWorld());
        ForestParams fp; fp.cell = cell; fp.seed = 1;
        genForest(*world, fp, nullptr);
        CamParams cp;
        cp.width = camW; cp.height = camH; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
        auto s = std::unique_ptr<SimFrameSource>(new SimFrameSource(*world, cp, showTruth));
        CamPose p0; p0.e = 15.f; p0.n = 10.f; p0.u = 6.f;
        s->setPose(p0);
        src = std::move(s);
        std::printf("[sim] synthetic forest, %s depth\n", showTruth ? "PERFECT" : "stereo");
    }

    const DepthCamera& cam = src->camera();
    const CamParams& cp = src->params();

    // --- map, sized from the camera that is actually producing the frames ---
    VoxelMapParams mp;
    mp.cell = cell;
    mp.depthSigCoef = 0.25f / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
    if (maxIntegOverride > 0.f) mp.maxIntegM = maxIntegOverride;

    // The camera sits at the middle of the grid looking +y (North), so the map
    // has room behind it as well -- a mapper that can only grow forwards would
    // hide exactly the recentring bugs this is meant to expose.
    const float px = mp.nx * cell * 0.5f, py = mp.ny * cell * 0.25f, pz = mp.nz * cell * 0.5f;
    VoxelMap M;
    M.init(mp, px, py, pz);

    std::printf("[map] cell %.2f m -> honest carve range %.2f m "
                "(f %.0f px, B %.0f mm)\n",
                mp.cell, mp.maxIntegM, cam.fpx(), cp.baselineM * 1000.f);
    if (mode != "sim")
        std::printf("[pose] %s -- NO translation is estimated. Move the camera and the\n"
                    "       map is wrong; that is the missing odometry, not a bug here.\n",
                    yawRateDps != 0.f ? "spin at a stated rate" : "fixed");

    TrajParams tp;
    tp.robotR = robotR; tp.vMax = vmax; tp.dt = 0.1f;
    TrajectoryPlanner traj(tp);

    const int total = src->frameCount();
    std::printf("[plan] %zu primitives, robot r %.2f m, vMax %.1f m/s\n",
                traj.librarySize(), robotR, vmax);

#if SIM_HAVE_HIGHGUI
    const char* WIN = "voxel_live";
    if (!headless) cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    bool paused = false;
#endif

    cv::Mat depth;
    PoseHint hint;
    float yaw = 0.f;
    int n = 0;
    long integUs = 0, planUs = 0;

    for (;;) {
        if (steps > 0 && n >= steps) break;

        CamPose pose;
        pose.e = px; pose.n = py; pose.u = pz;
        pose.yawDeg = yaw; pose.pitchDeg = pitchDeg;

        if (auto* s = dynamic_cast<SimFrameSource*>(src.get())) s->setPose(pose);
        if (!src->next(depth, hint)) break;
        if (hint.valid) pose = hint.pose;      // the sim knows; nothing else does

        int64 t0 = cv::getTickCount();
        M.integrate(depth, cam, pose);
        integUs += (cv::getTickCount() - t0) * 1000000 / cv::getTickFrequency();

        t0 = cv::getTickCount();
        GeneralResult gr = traj.plan(M, pose.e, pose.n, pose.u, yaw, 0.f, 0.f);
        planUs += (cv::getTickCount() - t0) * 1000000 / cv::getTickFrequency();

        ++n;
        yaw += yawRateDps * (1.f / std::max(1, fps));

        // --- panes ---------------------------------------------------------
        const int PW = 420, PH = 320;
        cv::Mat dPane = fit(colourDepth(depth, 8.f), PW, PH);
        banner(dPane, "DEPTH  grey = NO RETURN, not far");
        {
            int valid = 0;
            for (int y = 0; y < depth.rows; ++y) {
                const float* r = depth.ptr<float>(y);
                for (int x = 0; x < depth.cols; ++x) if (r[x] > 0.f) ++valid;
            }
            char b[128];
            std::snprintf(b, sizeof(b), "valid %.1f %%   frame %d%s",
                          100.0 * valid / double(depth.total()), n,
                          total > 0 ? ("/" + std::to_string(total)).c_str() : "");
            banner(dPane, b, 44);
        }

        // Crop the slice to a window around the camera. The full grid is 60 m
        // across and the honest carve range is under 4 m, so the whole-map view
        // renders the only interesting part as a smudge a few pixels wide.
        cv::Mat sliceFull = M.sliceImage(pose.u);
        cv::Mat sPane;
        {
            const float halfM = 12.f;
            const int half = int(halfM / mp.cell);
            const int cx = sliceFull.cols / 2, cy = sliceFull.rows / 2;
            cv::Rect roi(std::max(0, cx - half), std::max(0, cy - half),
                         std::min(2 * half, sliceFull.cols), std::min(2 * half, sliceFull.rows));
            roi &= cv::Rect(0, 0, sliceFull.cols, sliceFull.rows);
            sPane = fit(sliceFull(roi).clone(), PW, PH);
            // Range rings on the slice too, so "how far does the map reach" is
            // answerable by eye rather than by trusting the caption.
            const float pxPerM = PW / (2.f * halfM);
            const cv::Point c(PW / 2, PH / 2);
            for (float r = 2.f; r <= halfM; r += 2.f)
                cv::circle(sPane, c, int(r * pxPerM), {180, 180, 180}, 1);
            cv::circle(sPane, c, int(mp.maxIntegM * pxPerM), {60, 160, 60}, 2, cv::LINE_AA);
            if (mp.maxCarveM < halfM)
                cv::circle(sPane, c, int(mp.maxCarveM * pxPerM), {200, 130, 60}, 1, cv::LINE_AA);
        }
        banner(sPane, "MAP SLICE  grey = UNKNOWN");
        {
            char b[128];
            // MARK and CARVE are different limits and conflating them in a
            // caption is how the two-state confusion gets back in. Occupancy is
            // marked only inside maxIntegM, where a return's POSITION is
            // trustworthy; free space is carved much further, to maxCarveM,
            // because "the ray got through" survives an imprecise endpoint.
            // The pane shows free space well past the green ring, and that is
            // correct rather than a bug.
            std::snprintf(b, sizeof(b), "mark %.1f m (green)  carve %.0f m  cell %.2f m",
                          mp.maxIntegM, mp.maxCarveM, mp.cell);
            banner(sPane, b, 44);
        }

        // Planner pane: the admissible candidates, top-down, in the body frame.
        cv::Mat pPane(PH, PW, CV_8UC3, cv::Scalar(250, 248, 245));
        {
            const float scale = PH / 8.0f;                 // 8 m tall
            auto toPix = [&](float wx, float wy) {
                return cv::Point(int(PW * 0.5f + (wx - pose.e) * scale),
                                 int(PH - 10 - (wy - pose.n) * scale));
            };
            for (float r = 1.f; r <= 7.f; r += 1.f)
                cv::circle(pPane, toPix(pose.e, pose.n), int(r * scale), {225, 225, 225}, 1);
            cv::circle(pPane, toPix(pose.e, pose.n), int(mp.maxIntegM * scale),
                       {120, 180, 120}, 1, cv::LINE_AA);
            for (const auto& c : traj.candidates())
                for (size_t i = 1; i < c.size(); ++i)
                    cv::line(pPane, toPix(c[i-1][0], c[i-1][1]), toPix(c[i][0], c[i][1]),
                             {200, 200, 230}, 1, cv::LINE_AA);
            const auto& ch = traj.chosen();
            for (size_t i = 1; i < ch.size(); ++i)
                cv::line(pPane, toPix(ch[i-1][0], ch[i-1][1]), toPix(ch[i][0], ch[i][1]),
                         {40, 160, 40}, 2, cv::LINE_AA);
            cv::circle(pPane, toPix(pose.e, pose.n), int(robotR * scale), {60, 60, 200}, 1);
            banner(pPane, "PLAN  faint = admissible, green = chosen");
            char b[160];
            std::snprintf(b, sizeof(b), "%zu admissible  free %.2f m  cmd %.2f m/s%s",
                          traj.candidates().size(), gr.freeM, gr.speed,
                          gr.blocked ? "  BLOCKED" : "");
            banner(pPane, b, 44);
            std::snprintf(b, sizeof(b), "integrate %.1f ms   plan %.2f ms",
                          integUs / 1000.0 / n, planUs / 1000.0 / n);
            banner(pPane, b, PH - 14);
        }

        cv::Mat full(PH, PW * 3, CV_8UC3);
        dPane.copyTo(full(cv::Rect(0, 0, PW, PH)));
        sPane.copyTo(full(cv::Rect(PW, 0, PW, PH)));
        pPane.copyTo(full(cv::Rect(PW * 2, 0, PW, PH)));

#if SIM_HAVE_HIGHGUI
        if (!headless) {
            cv::imshow(WIN, full);
            const int k = cv::waitKey(paused ? 0 : 1);
            if (k == 'q' || k == 27) break;
            if (k == ' ') paused = !paused;
            if (k == 's') { cv::imwrite(out + "_frame.png", full);
                            std::printf("wrote %s_frame.png\n", out.c_str()); }
        }
#endif
        if (headless && (n == 1 || (total > 0 && n == total / 2))) {
            char b[256];
            std::snprintf(b, sizeof(b), "%s_%03d.png", out.c_str(), n);
            cv::imwrite(b, full);
            std::printf("wrote %s\n", b);
        }
        if (total > 0 && n >= total && mode == "replay") break;
    }

    std::printf("\n--- %d frames from %s ---\n", n, src->name());
    if (n) {
        std::printf("  map integrate   %6.2f ms/frame\n", integUs / 1000.0 / n);
        std::printf("  plan            %6.2f ms/frame\n", planUs / 1000.0 / n);
        std::printf("  ONBOARD TOTAL   %6.2f ms/frame  (%.0f Hz sustainable)\n",
                    (integUs + planUs) / 1000.0 / n,
                    1000.0 / std::max(0.001, (integUs + planUs) / 1000.0 / n));
    }
    if (n) {
        cv::imwrite(out + "_slice.png", M.sliceImage(pz));
        std::printf("  wrote %s_slice.png\n", out.c_str());
    }
    return n > 0 ? 0 : 1;
}
