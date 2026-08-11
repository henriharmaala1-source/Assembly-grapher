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
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

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


struct Config {
    std::string mode = "replay", path, out = "/tmp/voxel_live";
    float cell = 0.25f, robotR = 0.6f, vmax = 1.5f;
    float yawRateDps = 0.f, pitchDeg = 0.f, maxIntegOverride = -1.f;
    int   camW = 848, camH = 480, fps = 30, steps = -1;
    bool  emitter = false, headless = false, showTruth = false;
};

// ---------------------------------------------------------------------------
// THE MENU. Double-click the exe and you get this; no arguments, no terminal.
//
// Built on highgui's window and mouse callback rather than a real toolkit,
// because the alternative is adding Qt or a native UI to a tree whose whole
// discipline is not having dependencies -- and highgui is already linked for
// the panes. It is a few clickable rectangles. That is all this needs to be.
//
// Passing ANY argument still takes the old command-line path, so the headless
// tests and the sweep scripts keep working unchanged.
// ---------------------------------------------------------------------------
namespace {

struct Button {
    cv::Rect r;
    std::string label, sub;
    int id = 0;
    bool enabled = true;
    bool selected = false;
};

int g_click = -1;
std::vector<Button>* g_buttons = nullptr;

#if SIM_HAVE_HIGHGUI
void onMouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN || !g_buttons) return;
    for (const Button& b : *g_buttons)
        if (b.enabled && b.r.contains({x, y})) { g_click = b.id; return; }
}
#endif

void drawButton(cv::Mat& img, const Button& b) {
    const cv::Scalar face = !b.enabled ? cv::Scalar(232, 232, 232)
                          : b.selected ? cv::Scalar(215, 240, 215)
                                       : cv::Scalar(252, 252, 252);
    const cv::Scalar edge = !b.enabled ? cv::Scalar(200, 200, 200)
                          : b.selected ? cv::Scalar(60, 150, 60)
                                       : cv::Scalar(150, 150, 150);
    cv::rectangle(img, b.r, face, cv::FILLED);
    cv::rectangle(img, b.r, edge, b.selected ? 2 : 1);
    const cv::Scalar ink = b.enabled ? cv::Scalar(30, 30, 30) : cv::Scalar(150, 150, 150);
    cv::putText(img, b.label, {b.r.x + 12, b.r.y + (b.sub.empty() ? b.r.height / 2 + 6 : 26)},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, ink, 1, cv::LINE_AA);
    if (!b.sub.empty())
        cv::putText(img, b.sub, {b.r.x + 12, b.r.y + 48},
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, {110, 110, 110}, 1, cv::LINE_AA);
}

void label(cv::Mat& img, const std::string& t, int x, int y, double sc = 0.5,
           cv::Scalar c = {70, 70, 70}) {
    cv::putText(img, t, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, 1, cv::LINE_AA);
}

// A recording, already OPENED. Listing the filename alone is not enough: the
// first version auto-selected whatever sorted first, which in a directory
// containing a deliberately corrupt test file meant START failed with a console
// message the user never sees. Probing the header here costs one open per file
// and turns "it did not work" into "that file is not a recording", before the
// click rather than after it.
struct Recording {
    std::string path, base, detail;
    bool ok = false;
};

std::vector<Recording> findRecordings(const std::string& exeDir) {
    std::vector<std::string> out;
    for (const std::string& dir : {std::string("."), exeDir}) {
        if (dir.empty()) continue;
        try {
            for (const auto& e : std::filesystem::directory_iterator(dir)) {
                if (!e.is_regular_file()) continue;
                const std::string p = e.path().string();
                if (p.size() > 4 && p.compare(p.size() - 4, 4, ".kdr") == 0) {
                    bool dup = false;
                    for (const auto& q : out)
                        if (std::filesystem::path(q).filename() ==
                            std::filesystem::path(p).filename()) dup = true;
                    if (!dup) out.push_back(p);
                }
            }
        } catch (...) { /* unreadable directory is not an error worth stopping for */ }
    }
    std::sort(out.begin(), out.end());
    if (out.size() > 5) out.resize(5);

    std::vector<Recording> recs;
    for (const std::string& p : out) {
        Recording r;
        r.path = p;
        r.base = std::filesystem::path(p).filename().string();
        DepthRecordReader rd;
        std::string err;
        if (rd.open(p, &err)) {
            const DepthRecordHeader& h = rd.header();
            char b[128];
            std::snprintf(b, sizeof(b), "%u frames  %ux%u  %s", h.frames, h.width,
                          h.height,
                          (h.flags & DepthRecordHeader::FLAG_EMITTER_ON) ? "emitter on"
                                                                        : "emitter off");
            r.detail = b;
            r.ok = true;
        } else {
            r.detail = err;
        }
        recs.push_back(r);
    }
    return recs;
}

std::string humanSize(const std::string& p) {
    try {
        const auto n = std::filesystem::file_size(p);
        char b[64];
        if (n >= 1000000u) std::snprintf(b, sizeof(b), "%.0f MB", double(n) / 1e6);
        else               std::snprintf(b, sizeof(b), "%.0f kB", double(n) / 1e3);
        return b;
    } catch (...) { return ""; }
}

// Renders the menu. Split from the event loop so it can be dumped to a PNG and
// looked at without a window -- the same reason gui_preview exists.
// The state the app opens in. Shared so --menu-preview shows what a user
// really sees; the first version previewed a default-constructed Config, whose
// empty path greyed out START in a way that never happens in the app.
void applyStartupDefaults(Config& C, const std::vector<Recording>& recs) {
    // Only a READABLE recording counts. Defaulting to one that will not open is
    // how an app greets you with an error it could have avoided.
    const Recording* first = nullptr;
    for (const Recording& r : recs) if (r.ok) { first = &r; break; }
    C.mode = haveLiveSupport() ? "live" : (first ? "replay" : "sim");
    if (C.mode == "replay" && C.path.empty() && first) C.path = first->path;
}

cv::Mat renderMenu(const Config& C, const std::vector<Recording>& recs,
                   std::vector<Button>& btns, const std::string& note) {
    const int W = 1000, H = 690;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(246, 246, 248));
    btns.clear();

    cv::rectangle(img, {0, 0, W, 62}, {60, 70, 90}, cv::FILLED);
    cv::putText(img, "voxel_live", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                {245, 245, 245}, 2, cv::LINE_AA);
    cv::putText(img, "the real map and planner, over real depth", {230, 40},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 205, 215}, 1, cv::LINE_AA);

    label(img, "1.  WHERE DO THE FRAMES COME FROM", 20, 92, 0.55, {40, 40, 40});

    Button live{{20, 106, 290, 66}, "LIVE CAMERA",
                haveLiveSupport() ? "a connected D435i"
                                  : "this build has no RealSense SDK", 1,
                haveLiveSupport(), C.mode == "live"};
    int nGood = 0;
    for (const Recording& r : recs) if (r.ok) ++nGood;
    Button rep {{325, 106, 290, 66}, "REPLAY A RECORDING",
                nGood ? "pick one below" : "no readable .kdr files", 2,
                nGood > 0, C.mode == "replay"};
    Button sim {{630, 106, 290, 66}, "SIMULATED FOREST",
                "the control -- known ground truth", 3, true, C.mode == "sim"};
    btns.push_back(live); btns.push_back(rep); btns.push_back(sim);

    label(img, "2.  RECORDINGS FOUND", 20, 204, 0.55, {40, 40, 40});
    if (recs.empty()) {
        label(img, "None. Put a .kdr beside this program, or record one with", 20, 230);
        label(img, "d435i_probe.py --record 600 --record-every 6", 20, 252, 0.5, {40, 90, 160});
    }
    for (size_t i = 0; i < recs.size(); ++i) {
        const Recording& r = recs[i];
        btns.push_back(Button{{20, 216 + int(i) * 40, 470, 34}, r.base,
                              "", 100 + int(i), r.ok, C.path == r.path});
        label(img, r.ok ? r.detail : ("UNREADABLE: " + r.detail),
              500, 238 + int(i) * 40, 0.42,
              r.ok ? cv::Scalar(120, 120, 120) : cv::Scalar(60, 60, 190));
        if (r.ok) label(img, humanSize(r.path), 860, 238 + int(i) * 40, 0.42,
                        {150, 150, 150});
    }

    const int yset = 216 + int(std::max<size_t>(recs.size(), 1)) * 40 + 24;
    label(img, "3.  SETTINGS", 20, yset, 0.55, {40, 40, 40});

    char b[80];
    std::snprintf(b, sizeof(b), "voxel  %.2f m", C.cell);
    btns.push_back(Button{{20, yset + 12, 150, 40}, b, "", 10});
    label(img, "click to cycle", 178, yset + 38, 0.42, {140, 140, 140});

    std::snprintf(b, sizeof(b), "speed cap  %.1f m/s", C.vmax);
    btns.push_back(Button{{300, yset + 12, 210, 40}, b, "", 11});

    btns.push_back(Button{{530, yset + 12, 180, 40},
                          C.emitter ? "emitter ON" : "emitter OFF", "", 12, true, C.emitter});
    label(img, "off = the outdoor case", 718, yset + 38, 0.42, {140, 140, 140});

    std::snprintf(b, sizeof(b), "spin  %.0f deg/s", C.yawRateDps);
    btns.push_back(Button{{20, yset + 62, 210, 40}, b, "", 13, true, C.yawRateDps != 0.f});
    label(img, "only if you know the real rate -- there is no odometry",
          240, yset + 88, 0.42, {140, 140, 140});

    btns.push_back(Button{{20, H - 78, 250, 56}, "START", "", 20,
                          C.mode != "replay" || !C.path.empty()});
    btns.push_back(Button{{286, H - 78, 140, 56}, "QUIT", "", 21});
    label(img, "In the view:  space pause   s save PNG   m menu   q quit",
          446, H - 62, 0.46, {110, 110, 110});
    label(img, "Camera pose is assumed FIXED. Move it and the map",
          446, H - 42, 0.42, {150, 110, 60});
    label(img, "will be wrong -- that is the missing odometry.",
          446, H - 24, 0.42, {150, 110, 60});

    if (!note.empty())
        cv::putText(img, note, {20, H - 96}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    {40, 40, 200}, 1, cv::LINE_AA);

    for (const Button& bt : btns) drawButton(img, bt);
    return img;
}

}  // namespace


// Returns 0 normally, 1 if the session ended with nothing read, 2 on a setup
// error, and 3 when the user asked to go back to the menu.
static int runSession(Config cfg);

int mainCli(int argc, char** argv) {
    Config C;
    std::string& mode = C.mode; std::string& path = C.path; std::string& out = C.out;
    float& cell = C.cell; float& robotR = C.robotR; float& vmax = C.vmax;
    float& yawRateDps = C.yawRateDps; float& pitchDeg = C.pitchDeg;
    float& maxIntegOverride = C.maxIntegOverride;
    int& camW = C.camW; int& camH = C.camH; int& fps = C.fps; int& steps = C.steps;
    bool& emitter = C.emitter; bool& headless = C.headless; bool& showTruth = C.showTruth;

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
        else if (!std::strcmp(argv[i], "--menu-preview")) {
            // Dump the menu to a PNG. The window is the one artefact here that
            // cannot be checked over ssh or in CI, and a layout bug in it is
            // invisible to every other test.
            std::vector<Button> bs;
            std::vector<Recording> recs = findRecordings(".");
            applyStartupDefaults(C, recs);
            cv::imwrite(next("/tmp/menu.png"), renderMenu(C, recs, bs, ""));
            std::printf("wrote the menu layout (%zu buttons, %zu recordings)\n",
                        bs.size(), recs.size());
            return 0;
        }
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
    return runSession(C);
}

static int runSession(Config C) {
    const std::string& mode = C.mode; const std::string& path = C.path;
    const std::string& out = C.out;
    const float cell = C.cell, robotR = C.robotR, vmax = C.vmax;
    const float yawRateDps = C.yawRateDps, pitchDeg = C.pitchDeg;
    const float maxIntegOverride = C.maxIntegOverride;
    const int camW = C.camW, camH = C.camH, fps = C.fps, steps = C.steps;
    const bool emitter = C.emitter, headless = C.headless, showTruth = C.showTruth;

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

    bool backToMenu = false;
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
            if (k == 'm') { backToMenu = true; break; }
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
    if (backToMenu) return 3;
    return n > 0 ? 0 : 1;
}


// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    // Any argument at all -> the old command-line behaviour, so the headless
    // tests, ctest and the sweep scripts are untouched by the menu existing.
    if (argc > 1) return mainCli(argc, argv);

#if !SIM_HAVE_HIGHGUI
    std::printf("This build has no highgui, so there is no window to open.\n"
                "Run with arguments instead:  voxel_live --help\n");
    return mainCli(argc, argv);
#else
    std::string exeDir;
    try { exeDir = std::filesystem::path(argv[0]).parent_path().string(); } catch (...) {}

    Config C;
    std::vector<Recording> recs = findRecordings(exeDir);
    applyStartupDefaults(C, recs);

    const char* WIN = "voxel_live";
    // The hint goes out BEFORE the window is attempted, and that ordering is
    // the whole point. On a machine with no display, OpenCV's Qt backend calls
    // abort() rather than throwing -- measured: SIGABRT, exit 134 -- so a
    // try/catch around namedWindow never runs and the user gets a wall of Qt
    // plugin text with no clue this program has a headless mode. Printing first
    // is the only thing that survives. (Windows uses the Win32 backend and does
    // not hit this; it is for ssh and CI.)
    std::printf("voxel_live: opening a window. No display? Use the headless modes:\n"
                "  voxel_live --sim --frames 60 --headless --out out\n"
                "  voxel_live --replay FILE.kdr --headless --out out\n"
                "  voxel_live --menu-preview menu.png\n\n");
    std::fflush(stdout);
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    std::vector<Button> btns;
    g_buttons = &btns;
    cv::setMouseCallback(WIN, onMouse, nullptr);

    static const float CELLS[] = {0.15f, 0.20f, 0.25f, 0.35f, 0.50f};
    static const float VMAXS[] = {0.5f, 1.0f, 1.5f, 2.0f, 3.0f};
    static const float SPINS[] = {0.f, 5.f, 10.f, 20.f, 45.f};
    int iCell = 2, iVmax = 2, iSpin = 0;
    std::string note;

    for (;;) {
        cv::Mat menu = renderMenu(C, recs, btns, note);
        cv::imshow(WIN, menu);
        g_click = -1;
        const int key = cv::waitKey(30);
        if (key == 'q' || key == 27) break;
        if (key == 'r') { recs = findRecordings(exeDir); note = "rescanned for recordings"; }
        if (g_click < 0) continue;

        const int id = g_click;
        note.clear();
        if (id == 1) C.mode = "live";
        else if (id == 2) {
            C.mode = "replay";
            if (C.path.empty())
                for (const Recording& r : recs) if (r.ok) { C.path = r.path; break; }
        }
        else if (id == 3) C.mode = "sim";
        else if (id >= 100 && id < 100 + int(recs.size())) {
            C.path = recs[size_t(id - 100)].path;
            C.mode = "replay";
        }
        else if (id == 10) { iCell = (iCell + 1) % 5; C.cell = CELLS[iCell]; }
        else if (id == 11) { iVmax = (iVmax + 1) % 5; C.vmax = VMAXS[iVmax]; }
        else if (id == 12) C.emitter = !C.emitter;
        else if (id == 13) { iSpin = (iSpin + 1) % 5; C.yawRateDps = SPINS[iSpin]; }
        else if (id == 21) break;
        else if (id == 20) {
            cv::destroyWindow(WIN);
            const int rc = runSession(C);
            // 3 means the user pressed m. Anything else ended the session, and
            // returning to the menu beats the window vanishing -- a program that
            // disappears on a bad recording tells you nothing about why.
            if (rc == 2) note = "could not start that source -- see the console";
            else if (rc == 1) note = "no frames were read";
            cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
            g_buttons = &btns;
            cv::setMouseCallback(WIN, onMouse, nullptr);
            recs = findRecordings(exeDir);
        }
    }
    g_buttons = nullptr;
    return 0;
#endif
}
