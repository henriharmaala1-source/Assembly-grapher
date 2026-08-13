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
#include <cstdlib>
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
#include "bearing_field.hpp"
#include "voxel_world.hpp"

using namespace sim;

namespace {

// Depth -> colour. Near red, far blue, INVALID mid-grey and not black, because
// black reads as "far" to the eye and the entire point of the three-state map
// is that missing data is not distance.
// Histogram-equalised variant. A LINEAR ramp cannot show near and far structure
// at the same time: set it to 10 m and everything past 10 m is one blue; set it
// to 20 m and the near field collapses into a few shades of red. Equalisation
// allocates colour where the DATA is, which is why the RealSense Viewer's own
// display looks so much more informative than ours did.
//
// Found by noticing a lamppost was only visible at the longest scale setting --
// in the DEPTH image, not the map. The sensor had measured it the whole time;
// the ramp could not show it.
cv::Mat colourDepthEq(const cv::Mat& d, float maxM) {
    int hist[256] = {0};
    long n = 0;
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            int b = int(std::min(1.f, r[x] / std::max(0.1f, maxM)) * 255.f);
            ++hist[b]; ++n;
        }
    }
    float cdf[256];
    long acc = 0;
    for (int i = 0; i < 256; ++i) { acc += hist[i]; cdf[i] = n ? float(acc) / n : 0.f; }

    cv::Mat out(d.rows, d.cols, CV_8UC3, cv::Scalar(90, 90, 90));
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            int b = int(std::min(1.f, r[x] / std::max(0.1f, maxM)) * 255.f);
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(cdf[b] * 120.f), 200, 230);
        }
    }
    cv::Mat bgr; cv::cvtColor(out, bgr, cv::COLOR_HSV2BGR);
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x)
            if (!(r[x] > 0.f)) bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(90, 90, 90);
    }
    return bgr;
}

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
    // RAY STRIDE. Integration cost is linear in rays and it dominates
    // everything else; the planner is 1.5 ms against 59 ms of mapping at
    // 848x480 with every pixel. Measured on the dev box, forest world,
    // 0.25 m cells:
    //
    //     848x480  stride 1   407 k rays   58.8 ms    17 Hz
    //              stride 2   102 k rays   19.2 ms    52 Hz
    //              stride 4    25 k rays    5.3 ms   190 Hz
    //     424x240  stride 1   102 k rays    8.4 ms   119 Hz
    //
    // So stride 1 at full resolution CANNOT keep up with a 30 fps camera, on a
    // desktop, let alone a laptop or a Pi. Default 2, which is comfortable and
    // costs nothing detectable: a 0.2 m trunk at 4 m spans ~11 px, so every
    // second pixel still puts five samples across it.
    int   stride = 2;
    // DIRECTION. On a bench there is no goal, and steering toward a fictional
    // one makes the chosen path a lie: the default used to score alignment with
    // azimuth 0, so the green primitive was always being pulled North for no
    // reason anybody looking at it could have known.
    //
    //   0  OPENNESS ONLY -- goalWeight 0. The score is clearance, smoothness
    //      and far-field openness. "Where would you go if you only wanted room."
    //   1  GENERAL DIRECTION -- the goal bearing is wherever the camera points,
    //      so it answers "where would you go, roughly forwards."
    int   dirMode = 0;
    // VIEW for the top-right pane:
    //   0 FIRST PERSON  the map alone, rendered from inside
    //   1 OVERLAY       the same render composited ON the depth image it was
    //                   built from, pixel for pixel
    //   2 CHASE         the same renderer from BEHIND AND ABOVE, with the plan
    //                   drawn in. Forward paths are invisible from the
    //                   aircraft's own eye -- they run along the view axis and
    //                   project to a dot at the vanishing point, which is what
    //                   drawing them there actually produced. Seeing a path as
    //                   a path needs a viewpoint the path is not pointing at.
    //
    // The overlay is the pane that can catch a whole class of fault nothing
    // else here can. Map and depth are separate estimates of the same scene; if
    // the voxels sit where the returns are, the intrinsics, the frame
    // convention and the pose all agree. If they are offset, sheared or
    // mirrored, one of those three is wrong -- and every other view in this
    // program would show both halves looking individually plausible.
    int   viewMode = 0;
    // TURN HUD, first-person pane only. A bearing tape and a caret saying which
    // way the planner wants to go, relative to where the nose points.
    //
    // This exists because the first-person view CANNOT show a forward path.
    // Forward rollouts run along the optical axis and project to a dot at the
    // vanishing point -- the finding that produced the chase view in the first
    // place. So the pane that looks most like flying is the one that tells you
    // least about where the plan goes, and the fix is the same one aviation
    // reached for: stop drawing the path and draw the COMMAND.
    //
    // Deliberately NOT drawn on the overlay pane. That pane's whole job is
    // pixel-for-pixel comparison of map against depth, and painting a HUD over
    // the thing being compared works against the one question it answers.
    bool  turnHud = true;
    // DEPTH COLOUR RANGE, metres. 0 = follow the coarsest layer's honest range.
    //
    // It was hard-coded to 8 m, which quietly made the depth pane useless for
    // the one question it is most often asked. The RealSense Viewer at a 16 m
    // scale shows plenty of real returns at 10-16 m in a wood; on an 8 m ramp
    // every one of them is the same shade of blue, so the pane saturates below
    // where the interesting question starts and you cannot tell whether the far
    // field has data at all. Deriving it from the map means the display grows
    // when --farcell does, instead of needing to be remembered.
    float depthMaxM = 0.f;
    bool  depthEq = false;      // histogram-equalised depth colouring, 'h'
    // RECORD every frame to a .kdr. Empty = off.
    //
    // This is the mechanism that makes a change ARGUABLE. A runtime toggle on a
    // live camera is not a controlled comparison -- you move, the scene moves,
    // and you are comparing two different inputs while believing you are
    // comparing two code paths. Record once, replay the same file through two
    // builds, and the input is identical by construction.
    //
    // It is also a THESIS.md §4 deliverable: "the raw .kdr recording of each
    // run, so the claim is re-checkable offline".
    std::string recordPath;
    // COARSE FAR MAP. voxel_sim has had one for a long time; voxel_live did
    // not, which is why the live view showed nothing but the fine map's 3.5 m
    // and no larger voxels anywhere. That was a missing feature, not a camera
    // limit -- the depth image is full of valid returns far past 3.5 m.
    //
    // Sizing the cell to the uncertainty is the honest move rather than a
    // compromise: at 12 m a return genuinely has metres of error along the ray,
    // and a 0.25 m voxel claims a precision the measurement does not contain.
    // Z_max = sqrt(cell*f*B/sigma) so a 2 m cell roughly triples the range.
    //
    // AWARENESS ONLY, NEVER PERMISSION. The fine map alone decides what may be
    // flown through; this only says which bearing looks open beyond it.
    // OFF. The coarse voxel rung is superseded by the bearing field.
    //
    // It had to be a cube sized for stereo's RANGE error, which grows as Z^2,
    // while the lateral error grows only as Z -- ratio Z*sigma/B, a hundred to
    // one at 20 m. So it threw away every bit of bearing detail the sensor
    // still had, and then the banded handover between levels produced, in
    // order: a circular fog disc, two rungs that held obstacles and drew
    // nothing, and a wall of near faces when the band was loosened. Three
    // defects, one cause, and it is the representation.
    //
    // Measured against the 1.0 m rung, same frame and projection:
    //
    //     cubes     22 ms    11 independent bearings across 87 deg
    //     bearings  3.2 ms   ~1800 live bins, no handover seam
    //
    // Set --farcell to bring the rung back; the render and the direction score
    // both switch with it, so the two can still be compared on real data.
    float farCell = 0.f;       // 0 disables -- the bearing field replaces it
    // FINE NEAR LAYER -- the coarse map's argument run in the other direction.
    //
    // Cell size should match the depth uncertainty at the range it covers. That
    // is why 2 m cells are honest at 10 m; it is equally why 0.25 m cells are
    // needlessly coarse at 1.5 m, where dZ = Z^2*sigma/(f*B) is only 10 cm.
    // Indoors at arm's length the 0.25 m map is throwing away resolution the
    // measurement actually contains.
    //
    // AND IT IS CHEAP, because a level only has to cover its OWN honest range:
    // 0.10 m cells are honest to 2.2 m, so a 5 m box is enough -- 50x50x25
    // cells, a tenth of the 0.25 m map's grid. Finer does not mean bigger.
    //
    //     cell 0.05 -> honest to 1.55 m      cell 0.15 -> 2.68 m
    //     cell 0.10 -> honest to 2.19 m      cell 0.25 -> 3.46 m
    //
    // AWARENESS AND DISPLAY ONLY, exactly like the coarse map. The swept-volume
    // safety test still reads the 0.25 m map alone. Wiring a second level into
    // the safety path is a change to the one piece of code that must not be
    // wrong, and it earns that only with a measurement behind it -- see NOTES
    // for why the MID rung of the coarse ladder was reverted.
    float nearCell = 0.10f;    // 0 disables
    // THE AUDIT. Every complaint about this map has taken the same form -- "the
    // depth image plainly shows a thing and the voxel pane does not" -- and
    // every time the argument that followed was conducted on screenshots. This
    // turns that argument into a number: for a sample of returns, is the cell
    // the return LANDS IN actually OCCUPIED, in the layer that owns that range?
    //
    // It is deliberately not a test of the whole pipeline. It asks one
    // question, the one the eye asks, and it asks it per range bucket -- because
    // the failures found so far were all range-dependent and a single aggregate
    // percentage would have hidden every one of them.
    bool  audit = false;
    // Height above the terrain for the SIM aircraft, metres. An FPV airframe
    // under canopy flies a few metres up; the old harness put it at twelve.
    float altM = 2.5f;
    // Which synthetic world. "forest" is the general case; "hedge" is the one
    // the field disagreed with -- a porous bush fence at four metres with a
    // street behind it, which produced no voxels at all on real hardware.
    std::string scene = "forest";
    float hedgeFill = -1.f;    // <=0 uses the scene's own default
    float standoffM = -1.f;    // how far ahead the hedge stands, m
    // How far the bearing field is drawn and scored. Not an honest-range
    // formula like the voxel levels': a bearing bin's ANGULAR accuracy does not
    // decay with range, only its range accuracy does, and this field is used
    // for direction rather than distance. 20 m is where the stereo's own valid
    // fraction collapses on this baseline.
    float farRangeM = 20.f;
    // "Did most of what looked that way actually come back?" Empty sky is a few
    // per cent and fails; foliage is a third and passes. 0 disables the test,
    // which is how you see what it was rejecting.
    float fillFrac = 0.25f;
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

// WINDOW SCALE. Everything here is laid out at a fixed size -- the menu at
// 1000x740, the four-pane session at 840x640 -- and every caption, button and
// HUD offset is written in those pixels. Rather than thread a scale through all
// of that and get the text metrics wrong, the finished image is resized once on
// its way to the screen and clicks are divided back out.
//
// 740 px of menu is most of the height of a 1366x768 laptop, which is where
// this actually runs. Default 0.75; --ui 1 restores the old size.
float g_ui = 0.75f;

#if SIM_HAVE_HIGHGUI
// Downscaled with INTER_AREA, not INTER_NEAREST: this is text and UI, and
// nearest-neighbour on a 0.75 shrink eats whole strokes out of the glyphs.
void showScaled(const char* win, const cv::Mat& img) {
    if (g_ui > 0.999f && g_ui < 1.001f) { cv::imshow(win, img); return; }
    cv::Mat s;
    cv::resize(img, s, {}, g_ui, g_ui,
               g_ui < 1.f ? cv::INTER_AREA : cv::INTER_LINEAR);
    cv::imshow(win, s);
}

void onMouse(int event, int x, int y, int, void*) {
    if (event != cv::EVENT_LBUTTONDOWN || !g_buttons) return;
    const cv::Point p(int(x / g_ui), int(y / g_ui));   // back into layout pixels
    for (const Button& b : *g_buttons)
        if (b.enabled && b.r.contains(p)) { g_click = b.id; return; }
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
    const int W = 1000, H = 740;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(246, 246, 248));
    btns.clear();

    cv::rectangle(img, {0, 0, W, 62}, {60, 70, 90}, cv::FILLED);
    cv::putText(img, "voxel_live", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                {245, 245, 245}, 2, cv::LINE_AA);
    cv::putText(img, "the real map and planner, over real depth", {230, 40},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 205, 215}, 1, cv::LINE_AA);

    label(img, "1.  WHERE DO THE FRAMES COME FROM", 20, 92, 0.55, {40, 40, 40});

    const bool liveOk = haveLiveSupport();
    Button live{{20, 106, 290, 66}, "LIVE CAMERA",
                liveOk ? "a connected D435i" : "librealsense not loadable", 1,
                liveOk, C.mode == "live"};
    int nGood = 0;
    for (const Recording& r : recs) if (r.ok) ++nGood;
    Button rep {{325, 106, 290, 66}, "REPLAY A RECORDING",
                nGood ? "pick one below" : "no readable .kdr files", 2,
                nGood > 0, C.mode == "replay"};
    Button sim {{630, 106, 290, 66}, "SIMULATED FOREST",
                "the control -- known ground truth", 3, true, C.mode == "sim"};
    btns.push_back(live); btns.push_back(rep); btns.push_back(sim);

    {
        // Say WHY, always. "not available" with no reason is what turned a
        // missing DLL into three rounds of guessing.
        std::string d = liveSupportDetail();
        const size_t nl = d.find('\n');
        if (nl != std::string::npos) d = d.substr(0, nl);
        if (d.size() > 90) d = d.substr(0, 90) + "...";
        label(img, d, 20, 190, 0.40, liveOk ? cv::Scalar(120, 140, 120)
                                            : cv::Scalar(60, 60, 190));
    }
    label(img, "2.  RECORDINGS FOUND", 20, 210, 0.55, {40, 40, 40});
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

    // Three buttons per row and NO inline explanations on the crowded rows.
    // The labels used to sit at x = 218 and x = 530 on this row and were simply
    // painted over by the buttons beside them -- legible only because nothing
    // had been added to the right-hand column yet. The explanations moved below
    // the block, where they have the width they need.
    std::snprintf(b, sizeof(b), "ray stride  %d", C.stride);
    btns.push_back(Button{{20, yset + 62, 190, 40}, b, "", 14, true, C.stride > 1});

    btns.push_back(Button{{240, yset + 62, 260, 40},
                          C.dirMode == 0 ? "steer: OPENNESS only"
                                         : "steer: general direction", "", 15,
                          true, C.dirMode == 1});

    std::snprintf(b, sizeof(b), "spin  %.0f deg/s", C.yawRateDps);
    btns.push_back(Button{{20, yset + 112, 210, 40}, b, "", 13, true, C.yawRateDps != 0.f});
    label(img, "spin: only at a rate you KNOW", 20, yset + 196, 0.42,
          {140, 140, 140});
    label(img, "ray stride 1 cannot keep up at 30 fps;  no goal on a bench, so do not steer toward one",
          20, yset + 176, 0.42, {140, 140, 140});
    label(img, "far layer is AWARENESS ONLY -- it never grants permission to fly",
          300, yset + 196, 0.42, {140, 140, 140});

    btns.push_back(Button{{20, H - 78, 250, 56}, "START", "", 20,
                          C.mode != "replay" || !C.path.empty()});
    btns.push_back(Button{{286, H - 78, 140, 56}, "QUIT", "", 21});
    label(img, "window feels large?  run with  --ui 1.0  (or 0.6)", 20, H - 12, 0.42,
          {150, 150, 150});
    btns.push_back(Button{{530, yset + 112, 300, 40},
                          C.viewMode == 0 ? "view: FIRST PERSON"
                                          : C.viewMode == 1 ? "view: OVERLAY on depth"
                                                            : "view: CHASE (3D plan)",
                          "", 16, true, C.viewMode == 1});
    btns.push_back(Button{{520, yset + 62, 310, 40},
                          C.turnHud ? "command arrow: ON" : "command arrow: off", "", 17,
                          true, C.turnHud});

    // The coarse layer's cell size, with the range it buys printed on the
    // button -- the number people actually want is the metres, not the cell.
    if (C.farCell > 0.f)
        std::snprintf(b, sizeof(b), "far  %.0f m -> %.0f m",
                      C.farCell, std::sqrt(C.farCell * 447.f * 0.05f / 0.25f) * 0.75f);
    else
        std::snprintf(b, sizeof(b), "far layer: OFF");
    btns.push_back(Button{{240, yset + 112, 260, 40}, b, "", 18, true, C.farCell > 0.f});

    label(img, "In the view:  space  v view  a arrow  s save PNG  m menu  q quit",
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
    int& stride = C.stride; int& dirMode = C.dirMode; int& viewMode = C.viewMode;
    float& farCell = C.farCell; float& nearCell = C.nearCell;

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
        else if (!std::strcmp(argv[i], "--stride")) stride = std::max(1, std::atoi(next("2")));
        else if (!std::strcmp(argv[i], "--forward")) dirMode = 1;
        else if (!std::strcmp(argv[i], "--openness")) dirMode = 0;
        else if (!std::strcmp(argv[i], "--overlay")) viewMode = 1;
        else if (!std::strcmp(argv[i], "--chase")) viewMode = 2;
        else if (!std::strcmp(argv[i], "--compare")) viewMode = 3;
        else if (!std::strcmp(argv[i], "--noarrow")) C.turnHud = false;
        else if (!std::strcmp(argv[i], "--record")) C.recordPath = next("run.kdr");
        else if (!std::strcmp(argv[i], "--depthmax"))
            C.depthMaxM = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--ui"))
            g_ui = std::max(0.4f, std::min(2.f, float(std::atof(next("0.75")))));
        else if (!std::strcmp(argv[i], "--farcell")) farCell = float(std::atof(next("2.0")));
        else if (!std::strcmp(argv[i], "--nofar")) farCell = 0.f;
        else if (!std::strcmp(argv[i], "--nearcell")) nearCell = float(std::atof(next("0.10")));
        else if (!std::strcmp(argv[i], "--nonear")) nearCell = 0.f;
        else if (!std::strcmp(argv[i], "--emitter")) emitter = true;
        else if (!std::strcmp(argv[i], "--frames")) steps = std::atoi(next("-1"));
        else if (!std::strcmp(argv[i], "--out")) out = next("/tmp/voxel_live");
        else if (!std::strcmp(argv[i], "--headless")) headless = true;
        else if (!std::strcmp(argv[i], "--audit")) C.audit = true;
        else if (!std::strcmp(argv[i], "--alt")) C.altM = float(std::atof(next("2.5")));
        else if (!std::strcmp(argv[i], "--scene")) C.scene = next("forest");
        else if (!std::strcmp(argv[i], "--fill")) C.hedgeFill = float(std::atof(next("0.3")));
        else if (!std::strcmp(argv[i], "--standoff")) C.standoffM = float(std::atof(next("4")));
        else if (!std::strcmp(argv[i], "--farrange")) C.farRangeM = float(std::atof(next("20")));
        else if (!std::strcmp(argv[i], "--fillfrac")) C.fillFrac = float(std::atof(next("0.25")));
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
        else if (!std::strcmp(argv[i], "--rs-check")) {
            // "Can this machine do --live, and if not, why." One command, and
            // it needs no camera.
            std::printf("%s\n", liveSupportDetail().c_str());
            return haveLiveSupport() ? 0 : 1;
        }
        else if (!std::strcmp(argv[i], "--help")) {
            std::printf(
                "voxel_live -- the real map and planner over real depth\n"
                "  --replay FILE.kdr   a recording (no camera needed)\n"
                "  --live              a connected D435i%s\n"
                "  --rs-check          report whether librealsense can be loaded\n"
                "  --sim               the raycaster, as a control\n"
                "  --cell 0.25         voxel size, m\n"
                "  --yawrate 0         camera rotation in place, deg/s (KNOWN rate only)\n"
                "  --pitch 0           camera pitch, deg (nose-up positive)\n"
                "  --vmax 1.5          speed cap for the planner's budget\n"
                "  --emitter           IR projector on (default off: the outdoor case)\n"
                "  --stride 2          use every Nth pixel when mapping. 1 is\n"
                "                      59 ms/frame at 848x480 and cannot keep up\n"
                "                      with a 30 fps camera; 2 is 19 ms\n"
                "  --overlay           composite the voxel view ON the depth image\n"
                "  --compare           voxel ladder BESIDE a bearing field, same instant\n"
                "  --nearcell 0.10     fine near layer, 0 = off. Honest to 2.2 m\n"
                "  --farcell 1.0       coarse far layer, 0 = off. Honest to 7 m\n"
                "  --openness          score on room alone (default; no goal)\n"
                "  --forward           score toward wherever the camera points\n"
                "  --noarrow           no command arrow on the first-person pane\n"
                "  --depthmax 0        depth colour range, m; 0 = the map's own\n"
                "  --record FILE.kdr   save every frame, for replay and A/B\n"
                "  --ui 0.75           window scale; 1 is the old (large) size\n"
                "  --frames N          stop after N frames\n"
                "  --alt 2.5           SIM only: height above the terrain, m\n"
                "  --scene forest      SIM world: forest | hedge\n"
                "  --farrange 20       how far the bearing field is drawn, m\n"
                "  --fillfrac 0.25     a bearing bin needs this fraction of its\n"
                "                      pixels to have RETURNED; 0 disables.\n"
                "                      Empty sky is a few per cent and fails\n"
                "  --fill 0.30         hedge porosity; lower = more see-through\n"
                "  --standoff 4        how far ahead the hedge stands, m\n"
                "  --audit             report whether the map agrees with the depth\n"
                "  --headless          no window; write PNGs to --out\n",
                haveLiveSupport() ? "" : "  [NOT in this build -- no librealsense]");
            return 0;
        }
        // An unrecognised flag is an ERROR, not a shrug. This used to fall off
        // the end of the chain and be ignored, so `--steps 40` (there is no
        // such flag; it is --frames) ran an endless windowed session that wrote
        // nothing, and looked exactly like a hang.
        else {
            std::fprintf(stderr, "voxel_live: unknown option '%s'. --help lists them.\n",
                         argv[i]);
            return 2;
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
        if (C.scene == "hedge") {
            // Sized and placed around where the aircraft will be put, which is
            // derived from the map grid a few lines below. Kept in step by
            // using the same expression rather than a copied number.
            // NOTE the cell is NOT the map's. See HedgeParams.
            HedgeParams hp;
            if (C.hedgeFill > 0.f) hp.fill = C.hedgeFill;
            if (C.standoffM > 0.f) {
                hp.standoffM = C.standoffM;
                hp.backdropM = C.standoffM + 10.f;   // keep something behind it
            }
            hp.spawnE = 240 * cell * 0.5f;
            hp.spawnN = 240 * cell * 0.25f;
            genHedgeRow(*world, hp);
            std::printf("[sim] hedge: %.0f m world at %.2f m cells "
                        "(%.1f M), twigs %.0f cm, fill %.2f, %.1f m ahead\n",
                        hp.sizeM, hp.cell,
                        double(hp.sizeM/hp.cell)*(hp.sizeM/hp.cell)*(hp.topM/hp.cell)/1e6,
                        hp.twigM*100.f, hp.fill, hp.standoffM);
        } else {
            ForestParams fp; fp.cell = cell; fp.seed = 1;
            genForest(*world, fp, nullptr);
        }
        CamParams cp;
        cp.width = camW; cp.height = camH; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
        auto s = std::unique_ptr<SimFrameSource>(new SimFrameSource(*world, cp, showTruth));
        CamPose p0; p0.e = 15.f; p0.n = 10.f; p0.u = 6.f;
        s->setPose(p0);
        src = std::move(s);
        std::printf("[sim] synthetic %s, %s depth\n", C.scene.c_str(),
                    showTruth ? "PERFECT" : "stereo");
    }

    const DepthCamera& cam = src->camera();
    const CamParams& cp = src->params();

    // --- map, sized from the camera that is actually producing the frames ---
    VoxelMapParams mp;
    mp.cell = cell;
    mp.depthSigCoef = 0.25f / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
    if (maxIntegOverride > 0.f) mp.maxIntegM = maxIntegOverride;
    mp.integrateStride = C.stride;
    // MINIMUM RANGE IS A FORMULA, NOT A GUESS. Intel's own:
    //     MinZ(mm) = focal length(px) * baseline(mm) / 126
    // which at 848x480 on a D435 gives ~16.8 cm and matches their published
    // figure. 1.2x margin, because the number is where depth *starts* being
    // possible rather than where it starts being trustworthy. Beats the 0.25 m
    // that was there, which was a sensible guess and nothing more.
    mp.minIntegM = cam.fpx() * cp.baselineM / 126.f * 1.2f;

    // The camera sits at the middle of the grid looking +y (North), so the map
    // has room behind it as well -- a mapper that can only grow forwards would
    // hide exactly the recentring bugs this is meant to expose.
    const float px = mp.nx * cell * 0.5f, py = mp.ny * cell * 0.25f;
    float pz = mp.nz * cell * 0.5f;

    // ALTITUDE IS PHYSICAL, NOT A PROPERTY OF THE GRID.
    //
    // It used to be nz*cell*0.5, which put the simulated aircraft at 12 m in a
    // boreal stand -- ten metres up, in the canopy, with the forest floor
    // seventeen metres away along the ray and therefore outside every rung's
    // honest range. That is why the ground never appeared: not a mapping
    // failure, a scene where the ground was never in range to begin with.
    //
    // Worse, it made altitude a function of --cell: 4.8 m at 0.10, 12 m at
    // 0.25, 24 m at 0.50. Every comparison across cell sizes was quietly
    // comparing different scenes, and it is the kind of harness fault that
    // makes a whole day's measurements mean nothing.
    //
    // Measured at the old 12 m, bottom third of the frame: 48.9 % valid,
    // mean range 8.9 m, mean height 9.3 m, 0.3 % of returns inside the 0.25 m
    // map's honest range, and 10.8 % below the grid floor entirely.
    if (auto* s = dynamic_cast<SimFrameSource*>(src.get())) {
        // Ground under the start point, found by walking UP from the floor to
        // the first empty cell. Casting down from above finds the canopy, not
        // the terrain -- it reported 18.00 m in a stand whose floor is at 2 m,
        // and put the aircraft 2.5 m above the treetops instead of the ground.
        float gnd = 0.f;
        for (float z = 0.f; z < 40.f; z += cell * 0.5f) {
            int gx, gy, gz;
            world->worldToCell(px, py, z, gx, gy, gz);
            if (!world->inBounds(gx, gy, gz) || !world->solid(gx, gy, gz)) break;
            gnd = z;
        }
        pz = gnd + C.altM;
        std::printf("[sim] ground %.2f m, flying at %.2f m (%.2f m AGL, --alt)\n",
                    gnd, pz, C.altM);
        CamPose p0; p0.e = px; p0.n = py; p0.u = pz;
        s->setPose(p0);
    }
    VoxelMap M;
    M.init(mp, px, py, pz);

    // --- fine near layer ------------------------------------------------------
    VoxelMap Mnear;
    VoxelMapParams np;
    if (C.nearCell > 0.f && C.nearCell < cell) {
        np.cell = C.nearCell;
        np.maxIntegM = std::sqrt(np.cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
        // The grid covers its own honest range and no more. This is what makes
        // a finer level cheap rather than expensive.
        const int n = std::max(32, int(np.maxIntegM * 2.4f / np.cell));
        np.nx = n; np.ny = n; np.nz = std::max(16, n / 2);
        np.maxCarveM = np.maxIntegM * 2.f;
        np.depthSigCoef = mp.depthSigCoef;
        np.minIntegM = mp.minIntegM;
        // A FINER layer wants FEWER rays, not more, and that is not a typo.
        // Stride controls angular sampling density; a near object is
        // angularly LARGE, so it needs less of it. At f = 425 a 10 cm object
        // at 1.5 m spans 28 px, and every fourth pixel still puts seven
        // samples across it. Meanwhile the cost per ray goes UP as the cell
        // shrinks, because DDA steps scale as 1/cell.
        //
        // Measured, 848x480, this forest, per integrate:
        //     fine 0.25 m, stride 2   14.8 ms
        //     near 0.10 m, stride 2   19.4 ms      <- finer AND dearer
        //     near 0.10 m, stride 4    6.4 ms      <- and no less useful
        //
        // (recentre is free: 14.81 vs 14.85 ms with it. It was the first
        // suspect and it was not the culprit.)
        np.integrateStride = C.stride * 2;
        Mnear.init(np, px, py, pz);
        std::printf("[near] cell %.2f m -> marks to %.1f m, grid %dx%dx%d "
                    "(%.2f M cells)\n", np.cell, np.maxIntegM, np.nx, np.ny, np.nz,
                    double(np.nx) * np.ny * np.nz / 1e6);
    }

    // --- coarse companion, for the range the fine map cannot honestly claim ---
    VoxelMap Mfar;
    VoxelMapParams fp;
    std::vector<TrajectoryPlanner::CoarseLevel> coarse;
    if (C.farCell > 0.f) {
        fp.cell = C.farCell;
        const int n = std::max(48, int(256.f / C.farCell));
        fp.nx = n; fp.ny = n; fp.nz = std::max(16, n / 3);
        fp.maxIntegM = std::sqrt(fp.cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
        fp.maxCarveM = 40.f;
        fp.integrateStride = std::max(4, C.stride * 2);   // a sixteenth of the rays
        fp.depthSigCoef = mp.depthSigCoef;
        fp.minIntegM = mp.minIntegM;
        // CARVE GUARD ON, and the old comment here ("the min-filter is
        // fine-scale") had it exactly backwards. The local-nearest-return clamp
        // matters MORE on a coarse layer, not less: an 8 m cell is far more
        // likely to contain a thin object surrounded by rays that miss it, and
        // one hit at +0.85 cannot survive a hundred carves at -0.40.
        //
        // Observed: a street with a lamppost clearly visible at ~10 m in the
        // depth image, and not one occupied cell anywhere in the map. The far
        // layer was not failing to see it -- it was erasing it, which is the
        // exact failure voxel_map.hpp already documents for trunks.
        //
        // Scaled by the layer's own stride, since a 5 px window on a grid
        // sampled every 4th pixel spans barely one sample.
        fp.carveWinPx = std::max(5, fp.integrateStride * 2 + 1);
        Mfar.init(fp, px, py, pz);
        std::printf("[far] cell %.2f m -> marks to %.1f m  (the fine map stops at %.1f m)\n",
                    fp.cell, fp.maxIntegM, mp.maxIntegM);
    }

    // WHY THE GROUND IS MOSTLY ABSENT, PRINTED RATHER THAN RE-DERIVED.
    //
    // Reach is a SPHERE of radius R, and a forward-looking camera grazes the
    // ground: at altitude h the terrain first comes inside R at a depression of
    // asin(h/R), so it occupies only the bottom (vfov/2 - asin(h/R)) of the
    // frame. At 2.5 m with R = 5.8 m that is 2.5 degrees of 56 -- four per cent
    // of the picture -- and the ground carpet stops sqrt(R^2 - h^2) = 5.2 m
    // ahead.
    //
    // The same sphere is why a distant trunk contributes NOTHING rather than a
    // little: its nearest point is at eye level, at exactly its horizontal
    // distance, so once that exceeds R no part of it is in range at any height.
    // What looks like "the base of the far trunk" is the ground carpet ending
    // just short of it. Reported from the field as exactly that, twice.
    {
        const float R = (C.farCell > 0.f ? fp.maxIntegM : mp.maxIntegM) * 1.15f;
        const float vfov = 2.f * std::atan(float(cp.height) / (2.f * cam.fpx()))
                         * 180.f / 3.14159265f;
        const float h = C.altM;
        if (mode == "sim" && h < R) {
            const float dep = std::asin(h / R) * 180.f / 3.14159265f;
            const float band = std::max(0.f, vfov * 0.5f - dep);
            std::printf("[geom] ground within reach to %.1f m ahead, in the bottom "
                        "%.1f deg of a %.0f deg frame (%.0f %% of it). "
                        "A trunk past %.1f m horizontally is out of range at EVERY "
                        "height.\n",
                        std::sqrt(std::max(0.f, R*R - h*h)), band, vfov,
                        100.f * band / vfov, R);
        }
    }

    std::printf("[map] min trusted range %.2f m (f*B/126, Intel's MinZ)\n",
                mp.minIntegM);
    std::printf("[map] cell %.2f m -> honest carve range %.2f m "
                "(f %.0f px, B %.0f mm), ray stride %d -> %d rays/frame\n",
                mp.cell, mp.maxIntegM, cam.fpx(), cp.baselineM * 1000.f,
                mp.integrateStride,
                (cp.width / mp.integrateStride) * (cp.height / mp.integrateStride));
    if (mode != "sim")
        std::printf("[pose] %s -- NO translation is estimated. Move the camera and the\n"
                    "       map is wrong; that is the missing odometry, not a bug here.\n",
                    yawRateDps != 0.f ? "spin at a stated rate" : "fixed");

    TrajParams tp;
    tp.robotR = robotR; tp.vMax = vmax; tp.dt = 0.1f;
    // No goal means no goal term. Leaving goalWeight at 1 while feeding a fixed
    // bearing does not mean "no preference", it means "prefer North".
    if (C.dirMode == 0) tp.goalWeight = 0.f;
    TrajectoryPlanner traj(tp);

    const int total = src->frameCount();
    std::printf("[plan] %zu primitives, robot r %.2f m, vMax %.1f m/s, %s\n",
                traj.librarySize(), robotR, vmax,
                C.dirMode == 0 ? "OPENNESS ONLY (no goal)"
                               : "general direction = where the camera points");

#if SIM_HAVE_HIGHGUI
    const char* WIN = "voxel_live";
    if (!headless) cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    bool paused = false;
#endif

    bool backToMenu = false;
    cv::Mat lastComposite;
    cv::Mat depth;
    PoseHint hint;
    float yaw = 0.f;
    int n = 0;
    long integUs = 0, planUs = 0, drawUs = 0;

    const bool haveNear = C.nearCell > 0.f && C.nearCell < cell;

    // The COMPARABLE. Same frames, same pose, same projection -- the only thing
    // that differs from the voxel ladder is where a range comes from. Built
    // unconditionally so the cost is always on the log line and the comparison
    // is never one the viewer had to be put into.
    BearingField bfield;
    { BearingFieldParams bp; bp.maxRangeM = 30.f;
      bp.minFillFrac = C.fillFrac; bfield.init(bp); }
    long bfieldUs = 0;

    // Opened lazily on the first frame, when the depth size is known for sure.
    DepthRecordWriter rec;
    bool recOpen = false, recFailed = false;

    // Audit buckets. Edges chosen to straddle the ladder's handovers rather
    // than to be round numbers, because the handovers are where every defect
    // found so far actually lived.
    static const float kAuditEdge[] = { 0.f, 1.f, 2.2f, 3.5f, 5.f, 7.f, 10.f, 15.f };
    const int kAuditN = int(sizeof(kAuditEdge) / sizeof(kAuditEdge[0])) - 1;
    std::vector<long> auditSeen(kAuditN, 0), auditOcc(kAuditN, 0),
                      auditFree(kAuditN, 0), auditUnk(kAuditN, 0),
                      auditOob(kAuditN, 0);
    double auditFreeM = 0, auditSpeed = 0; long auditBlocked = 0;
    auto auditBucket = [&](float r) {
        for (int i = 0; i < kAuditN; ++i)
            if (r >= kAuditEdge[i] && r < kAuditEdge[i + 1]) return i;
        return -1;
    };

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
        if (C.farCell > 0.f) Mfar.integrate(depth, cam, pose);
        if (haveNear) { Mnear.integrate(depth, cam, pose); Mnear.recentre(pose.e, pose.n, pose.u); }
        integUs += (cv::getTickCount() - t0) * 1000000 / cv::getTickFrequency();
        { const int64 tb = cv::getTickCount();
          bfield.update(depth, cam, pose, 1);
          bfieldUs += (cv::getTickCount() - tb) * 1000000 / cv::getTickFrequency(); }

        // --- the audit ---------------------------------------------------
        // Sampled coarsely, because it is a diagnostic and not a product, and
        // because 1/16 of the returns is already tens of thousands per frame.
        if (C.audit) {
            const int as = std::max(1, C.stride * 2);
            for (int v = 0; v < depth.rows; v += as) {
                const float* row = depth.ptr<float>(v);
                for (int u = 0; u < depth.cols; u += as) {
                    const float r = row[u];
                    if (!(r > 0.f) || r < mp.minIntegM) continue;
                    const int b = auditBucket(r);
                    if (b < 0) continue;
                    ++auditSeen[b];
                    float dx, dy, dz; cam.rayFor(pose, u, v, dx, dy, dz);
                    // NORMALISE. rayFor returns the pinhole ray with a forward
                    // component of 1, not a unit vector, and the depth value is
                    // a RANGE along the ray (VoxelWorld::raycast normalises
                    // before marching, so what it reports is metric distance).
                    // Skipping this multiplies every off-axis point by 1/cos t
                    // -- 24 % at the corner of an 87 deg frame -- and the audit
                    // then probes a cell two behind the one the mapper wrote,
                    // which reads exactly like a mapper that marks nothing.
                    // It cost an hour and it was the instrument, not the map.
                    const float dl = std::sqrt(dx*dx + dy*dy + dz*dz);
                    dx /= dl; dy /= dl; dz /= dl;
                    const float wx = pose.e + dx * r, wy = pose.n + dy * r,
                                wz = pose.u + dz * r;
                    // The layer that OWNS this range -- the same arbitration the
                    // panes use. Asking a layer about a range it was banded out
                    // of would measure the banding, not the map.
                    const VoxelMap* L = &M;
                    if (haveNear && r < np.maxIntegM)      L = &Mnear;
                    else if (r > mp.maxIntegM && C.farCell > 0.f) L = &Mfar;
                    int cx2, cy2, cz2; L->worldToCell(wx, wy, wz, cx2, cy2, cz2);
                    if (!L->inBounds(cx2, cy2, cz2)) { ++auditOob[b]; continue; }
                    const VoxelMap::State s = L->stateAt(wx, wy, wz);
                    if (s == VoxelMap::OCCUPIED) ++auditOcc[b];
                    else if (s == VoxelMap::FREE) ++auditFree[b];
                    else ++auditUnk[b];
                }
            }
        }

        t0 = cv::getTickCount();
        // dirMode 1 aims along the camera's own heading; dirMode 0 has
        // goalWeight 0 so the bearing passed here is ignored entirely.
        if (C.farCell > 0.f && coarse.empty())
            coarse.push_back({&Mfar, fp.maxIntegM});
        // Awareness comes from the bearing field unless a coarse voxel rung was
        // explicitly asked for. Either way it scores a DIRECTION and nothing
        // else: the swept-volume test reads the fine map alone.
        const TrajectoryPlanner::FarBearings fb{&bfield, C.farRangeM};
        GeneralResult gr = traj.plan(M, pose.e, pose.n, pose.u, yaw,
                                     C.dirMode == 1 ? yaw : 0.f, 0.f, coarse,
                                     C.farCell > 0.f ? nullptr : &fb);
        planUs += (cv::getTickCount() - t0) * 1000000 / cv::getTickFrequency();
        if (C.audit) { auditFreeM += gr.freeM; auditSpeed += gr.speed;
                       if (gr.blocked) ++auditBlocked; }

        ++n;

        if (!C.recordPath.empty() && !recOpen && !recFailed) {
            DepthRecordHeader h;
            h.width = uint32_t(depth.cols); h.height = uint32_t(depth.rows);
            h.depthScale = 0.001f;                 // millimetres, as the D435i gives
            h.fx = cam.fpx(); h.fy = cam.fy();
            h.ppx = cam.ppx(); h.ppy = cam.ppy();
            h.baselineM = cp.baselineM;
            if (emitter) h.flags |= DepthRecordHeader::FLAG_EMITTER_ON;
            std::string err;
            if (rec.open(C.recordPath, h, &err)) {
                recOpen = true;
                std::printf("[rec] %s  %dx%d\n", C.recordPath.c_str(), depth.cols, depth.rows);
            } else {
                recFailed = true;
                std::fprintf(stderr, "[rec] could not open %s: %s\n",
                             C.recordPath.c_str(), err.c_str());
            }
        }
        if (recOpen) {
            static std::vector<uint16_t> mm;
            mm.resize(size_t(depth.cols) * depth.rows);
            for (int y = 0; y < depth.rows; ++y) {
                const float* r = depth.ptr<float>(y);
                uint16_t* o = &mm[size_t(y) * depth.cols];
                for (int x = 0; x < depth.cols; ++x)
                    o[x] = (r[x] > 0.f) ? uint16_t(std::min(65535.f, r[x] * 1000.f)) : 0;
            }
            rec.writeFrame(mm.data());
        }

        // --- panes ---------------------------------------------------------
        const int PW = 420, PH = 320;
        // Follows the coarsest layer unless overridden -- see Config::depthMaxM.
        const float dMax = C.depthMaxM > 0.f ? C.depthMaxM
                         : std::max(8.f, C.farCell > 0.f ? fp.maxIntegM : mp.maxIntegM);
        cv::Mat dPane = fit(C.depthEq ? colourDepthEq(depth, dMax)
                                      : colourDepth(depth, dMax), PW, PH);
        {
            char db[80];
            std::snprintf(db, sizeof(db), "DEPTH  grey = NO RETURN, not far   %s",
                          C.depthEq ? "colour spread by DATA (h)"
                                    : (std::snprintf(nullptr, 0, "") , "linear"));
            if (!C.depthEq)
                std::snprintf(db, sizeof(db), "DEPTH  grey = NO RETURN, not far   "
                              "red 0 -> blue %.0f m", dMax);
            banner(dPane, db);
        }
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
        // Composite the COARSE map underneath. Where the fine map is UNKNOWN
        // (its flat 128 grey) but the coarse one is not, show the coarse cell,
        // tinted so the two are never confused: a 2 m voxel is eight times the
        // robot and must not be mistaken for something you may fly through.
        if (C.farCell > 0.f) {
            cv::Mat farSlice = Mfar.sliceImage(pose.u);
            const float sc = fp.cell / mp.cell;   // coarse cells per fine cell
            for (int y = 0; y < sliceFull.rows; ++y)
                for (int x = 0; x < sliceFull.cols; ++x) {
                    cv::Vec3b& d = sliceFull.at<cv::Vec3b>(y, x);
                    if (d != cv::Vec3b(128, 128, 128)) continue;   // fine knows
                    const int fx2 = int((x - sliceFull.cols * 0.5f) / sc + farSlice.cols * 0.5f);
                    const int fy2 = int((y - sliceFull.rows * 0.5f) / sc + farSlice.rows * 0.5f);
                    if (fx2 < 0 || fy2 < 0 || fx2 >= farSlice.cols || fy2 >= farSlice.rows) continue;
                    const cv::Vec3b f2 = farSlice.at<cv::Vec3b>(fy2, fx2);
                    if (f2 == cv::Vec3b(40, 40, 40))        d = cv::Vec3b(60, 60, 150);
                    else if (f2 == cv::Vec3b(245, 245, 245)) d = cv::Vec3b(165, 165, 150);
                }
        }
        // The NEAR layer goes on top of the fine one, wherever it knows more.
        if (haveNear) {
            cv::Mat nearSlice = Mnear.sliceImage(pose.u);
            const float sc = mp.cell / np.cell;      // fine cells per near cell
            for (int y = 0; y < sliceFull.rows; ++y)
                for (int x = 0; x < sliceFull.cols; ++x) {
                    const int nx2 = int((x - sliceFull.cols * 0.5f) * sc + nearSlice.cols * 0.5f);
                    const int ny2 = int((y - sliceFull.rows * 0.5f) * sc + nearSlice.rows * 0.5f);
                    if (nx2 < 0 || ny2 < 0 || nx2 >= nearSlice.cols || ny2 >= nearSlice.rows) continue;
                    const cv::Vec3b n2 = nearSlice.at<cv::Vec3b>(ny2, nx2);
                    if (n2 != cv::Vec3b(128, 128, 128))
                        sliceFull.at<cv::Vec3b>(y, x) = n2;
                }
        }

        cv::Mat sPane;
        {
            // Crop follows the COARSEST layer's honest range, like the depth
            // ramp. It was a hard-coded 12 m, chosen when the far layer reached
            // 10 m -- at --farcell 8 the map is honest to 19.5 m and this pane
            // was cropping off everything past 12, so the far field could not be
            // judged from the one view that shows extent. Second instrument in a
            // day scaled for an older configuration.
            const float halfM = std::max(6.f,
                (C.depthMaxM > 0.f ? C.depthMaxM
                 : std::max(8.f, C.farCell > 0.f ? fp.maxIntegM : mp.maxIntegM)) * 0.65f);
            const int half = int(halfM / mp.cell);
            (void)half;
            const int cx = sliceFull.cols / 2, cy = sliceFull.rows / 2;
            cv::Rect roi(std::max(0, cx - half), std::max(0, cy - half),
                         std::min(2 * half, sliceFull.cols), std::min(2 * half, sliceFull.rows));
            roi &= cv::Rect(0, 0, sliceFull.cols, sliceFull.rows);
            sPane = fit(sliceFull(roi).clone(), PW, PH);
            banner(sPane, "f range  t stride  - + scale (0 auto)  h equalise  "
                          "r rec  v view", PH - 12);
            // Range rings on the slice too, so "how far does the map reach" is
            // answerable by eye rather than by trusting the caption.
            const float pxPerM = PW / (2.f * halfM);
            const cv::Point c(PW / 2, PH / 2);
            const float ringStep = halfM > 15.f ? 5.f : (halfM > 8.f ? 2.f : 1.f);
            for (float r = ringStep; r <= halfM; r += ringStep)
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
            // Short enough to fit the pane. The first version ran off the edge,
            // which loses exactly the part that says how far the coarse layer
            // reaches -- the number the caption exists for.
            {
                std::string cells, ranges;
                char t[32];
                if (haveNear) {
                    std::snprintf(t, sizeof(t), "%.2f/", np.cell); cells += t;
                    std::snprintf(t, sizeof(t), "%.1f/", np.maxIntegM); ranges += t;
                }
                std::snprintf(t, sizeof(t), "%.2f", mp.cell); cells += t;
                std::snprintf(t, sizeof(t), "%.1f", mp.maxIntegM); ranges += t;
                if (C.farCell > 0.f) {
                    std::snprintf(t, sizeof(t), "/%.1f", fp.cell); cells += t;
                    std::snprintf(t, sizeof(t), "/%.1f", fp.maxIntegM); ranges += t;
                }
                std::snprintf(b, sizeof(b), "cells %s m -> honest %s m",
                              cells.c_str(), ranges.c_str());
            }
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
            if (gr.blocked || traj.candidates().empty()) {
                // WHY, not just THAT. Blocked by something SEEN and blocked by
                // something merely UNSEEN want opposite responses -- back off
                // versus look around -- and they are indistinguishable from a
                // count of zero.
                const auto& rj = traj.lastReject();
                std::snprintf(b, sizeof(b),
                              "rejected: %d occupied, %d unknown  (%d at the very first step)",
                              rj.occupied, rj.unknown, rj.atStart);
                banner(pPane, b, 66);
                const char* hint =
                    rj.unknown > rj.occupied
                        ? "mostly UNKNOWN: the map has not seen enough beside you."
                        : "mostly OCCUPIED: something real is inside the robot radius.";
                banner(pPane, hint, 88);
                std::snprintf(b, sizeof(b), "robot radius %.2f m -- a %.1f m wide vehicle",
                              robotR, robotR * 2.f);
                banner(pPane, b, 110);
            }
            std::snprintf(b, sizeof(b), "integrate %.1f ms   plan %.2f ms",
                          integUs / 1000.0 / n, planUs / 1000.0 / n);
            banner(pPane, b, PH - 14);
        }

        // FIRST PERSON: the map, from inside, out of the camera's own eyes.
        // Renders the MAP and never the world, which is the entire point --
        // where the model is wrong you fly into fog, and UNKNOWN is drawn as
        // fog rather than as air. maxRange is the map's honest marking limit,
        // so the horizon SHOULD look short: that is a sensor property, and
        // seeing where the world stops is the useful part.
        cv::Mat fPane;
        const float fpvRange = std::max(4.f, mp.maxIntegM * 2.f);

        // Draw the plan INTO the first-person view: every candidate rollout and
        // the chosen one, projected through the same camera the scene was
        // rendered with. This is the pane where a plan is legible as a plan --
        // a top-down fan of lines shows you the geometry, this shows you where
        // the aircraft would actually go, in the scene it would go through.
        //
        // fpvProject is the exact inverse of the ray fpvImageWH casts (pinned to
        // 0.0001 px in overlay_align_check), so a path lands where the geometry
        // puts it rather than where a second implementation of the maths does.
        auto drawPlanInto = [&](cv::Mat& img, float fovDeg) {
            auto proj = [&](const std::array<float, 3>& w, cv::Point& out) {
                float u, v;
                if (!VoxelMap::fpvProject(pose.e, pose.n, pose.u, yaw, pitchDeg,
                                          img.cols, img.rows, fovDeg,
                                          w[0], w[1], w[2], u, v)) return false;
                out = cv::Point(int(u), int(v));
                return true;
            };
            auto polyline = [&](const std::vector<std::array<float, 3>>& pts,
                                cv::Scalar col, int thick) {
                cv::Point a;
                bool have = false;
                for (const auto& w : pts) {
                    cv::Point b;
                    const bool ok = proj(w, b);
                    if (ok && have) cv::line(img, a, b, col, thick, cv::LINE_AA);
                    a = b; have = ok;
                }
            };
            // NO ROLLOUTS HERE, and that is geometry rather than taste.
            //
            // Every LEVEL rollout lies in the horizontal plane through the eye,
            // and that plane projects to a single horizontal line. A circular
            // arc of radius R leaving along the optical axis projects to
            // u = f*tan(theta/2) at v = cy EXACTLY, for every theta -- the
            // whole fan collapses onto one row, left turns sweeping one way
            // along it and right turns the other. What that drew was a pale
            // blue line straight across the pane, present at all times and
            // meaning nothing. Not a bug: correct, and useless, which is much
            // harder to notice than wrong.
            //
            // The same finding that produced the chase view, sharpened. From
            // the aircraft's own eye a forward path is a dot and a turning path
            // is the horizon. Paths belong in CHASE. What belongs here is the
            // aim point -- a real point in space, which projects like one --
            // and the command arrow below.
            (void)polyline;
            // The aim point: what the controller is actually commanded toward,
            // which is a short way along the winning path and NOT its endpoint.
            if (!traj.chosen().empty()) {
                const size_t k = std::min(traj.chosen().size() - 1,
                                          size_t(std::max(1, int(tp.aimS / tp.dt))));
                cv::Point a;
                if (proj(traj.chosen()[k], a))
                    cv::circle(img, a, 6, {40, 190, 40}, 2, cv::LINE_AA);
            }
        };
        // WHICH WAY TO TURN. See Config::turnHud for why the first-person pane
        // needs this and the chase pane does not.
        //
        // Drawn AFTER the pane has been resized to 420x320, in pane pixels, so
        // the tape and the caret are not stretched by the 4:3 fit the way the
        // square render is. The one thing that must survive the stretch is the
        // caret's horizontal POSITION, and that is an angle mapped onto the
        // tape, not a projected point -- so it does not care about the render's
        // geometry at all. A HUD that had to agree with the projection would be
        // a fourth place for a yaw convention to disagree.
        auto drawTurnHud = [&](cv::Mat& pane) {
            // Signed relative bearing. Positive = the planner wants to go RIGHT
            // of where the nose points. angDiffDeg lives in voxel_planner.hpp
            // because the planner needs the same convention, and two copies of
            // a sign convention is one copy too many.
            const float rel = angDiffDeg(gr.azDeg, yaw);
            const float relEl = gr.elDeg - pitchDeg;

            const int  cxp = pane.cols / 2;
            const int  base = pane.rows - 34;         // tail of the arrow
            const cv::Scalar BLACK(20, 20, 20), HALO(250, 250, 250);
            auto stroke = [&](cv::Point a, cv::Point b, bool head) {
                // White underneath, black on top. The pane is voxels over fog
                // and swings from near-white to dark within one frame, so a
                // single-colour arrow disappears somewhere on every scene.
                if (head) {
                    cv::arrowedLine(pane, a, b, HALO, 9, cv::LINE_AA, 0, 0.34);
                    cv::arrowedLine(pane, a, b, BLACK, 4, cv::LINE_AA, 0, 0.34);
                } else {
                    cv::line(pane, a, b, HALO, 9, cv::LINE_AA);
                    cv::line(pane, a, b, BLACK, 4, cv::LINE_AA);
                }
            };

            // BLOCKED has no bearing worth pointing at, and drawing one anyway
            // would be the most misleading thing this pane could do: it would
            // say "go this way" at the moment the answer is "do not go".
            if (gr.blocked || gr.src == GeneralResult::BLOCKED) {
                const cv::Point c(cxp, base - 18);
                cv::line(pane, {c.x - 17, c.y - 17}, {c.x + 17, c.y + 17}, HALO, 9, cv::LINE_AA);
                cv::line(pane, {c.x - 17, c.y + 17}, {c.x + 17, c.y - 17}, HALO, 9, cv::LINE_AA);
                cv::line(pane, {c.x - 17, c.y - 17}, {c.x + 17, c.y + 17}, {40, 40, 200}, 4, cv::LINE_AA);
                cv::line(pane, {c.x - 17, c.y + 17}, {c.x + 17, c.y - 17}, {40, 40, 200}, 4, cv::LINE_AA);
                banner(pane, "HOLD  nothing in the library is flyable", pane.rows - 62);
                return;
            }

            // THE COMMAND, as one arrow. Direction is the turn; LENGTH is the
            // commanded speed, so a stopped aircraft draws a stub rather than a
            // confident arrow with a 0.0 beside it. Both are motion commands
            // and this is what the aircraft would actually be told to do.
            const float th = std::max(-80.f, std::min(80.f, rel)) * PI_F / 180.f;
            const float frac = std::min(1.f, gr.speed / std::max(0.1f, vmax));
            const float L = 16.f + 40.f * frac;
            const cv::Point tip(int(cxp + std::sin(th) * L),
                                int(base - std::cos(th) * L));
            stroke({cxp, base}, tip, true);

            // Clamped is not the same as measured: past 80 deg the arrow angle
            // is a floor, so it gets a tick rather than pretending to point at
            // something it cannot.
            if (std::fabs(rel) > 80.f) {
                const int s = rel > 0.f ? 1 : -1;
                stroke({tip.x + s * 6, tip.y - 8}, {tip.x + s * 6, tip.y + 8}, false);
            }

            // THREE states, not two, and the middle one is the interesting one.
            // "no direction at all" (HOLD, above) is not "a direction, but
            // nowhere near enough confirmed-free room to move". The second used
            // to read AHEAD 0.0 m/s, which looks like a display fault rather
            // than the speed budget doing its job.
            char t[112], spd[40];
            if (gr.speed < 0.05f) std::snprintf(spd, sizeof(spd), "STOPPED  %.1f m free", gr.freeM);
            else                  std::snprintf(spd, sizeof(spd), "%.1f m/s", gr.speed);
            const char* climb = relEl > 5.f ? "  CLIMB" : (relEl < -5.f ? "  DESCEND" : "");
            if (std::fabs(rel) < 3.f)
                std::snprintf(t, sizeof(t), "AHEAD   %s%s", spd, climb);
            else
                std::snprintf(t, sizeof(t), "%s %.0f deg   %s%s",
                              rel > 0.f ? "RIGHT" : "LEFT", std::fabs(rel), spd, climb);
            banner(pane, t, pane.rows - 62);
        };

        // EVERY view draws the whole ladder, not one rung of it.
        //
        // This was the single biggest thing wrong with these panes. They each
        // picked the FINEST map available and rendered only that -- and a fine
        // map's honest marking range is short by construction: 3.5 m at 0.25 m
        // cells, 2.2 m at 0.10 m, both on the D435i's 50 mm baseline. So a room
        // or a wood whose nearest surface is four metres off came out as an
        // empty pane. That is not a mapping failure and it is not a rendering
        // failure; the coarse level had already marked those trunks (2 m cells
        // reach 10 m) and simply had nowhere to appear.
        //
        // Nearest hit wins per pixel, so the fine detail sits in front of the
        // coarse blocks wherever both know about a surface, and the coarse
        // blocks carry the pane everywhere the fine map has stopped.
        //
        // extraBack: distance from the EYE to the aircraft, for views that do
        // not sit on it. Each level's cast range is its own honest range plus
        // that, or the level would run out exactly where the aircraft is.
        auto ladder = [&](float extraBack) {
            // FINEST FIRST, each level banded by its own honest range. A coarse
            // level must never be consulted inside a finer level's range: a 2 m
            // cell's near face can sit 2 m in front of the surface it contains,
            // so it would draw a wall far too close and, in a room, fill the
            // pane. See VoxelMap::Layer.
            std::vector<VoxelMap::Layer> ls;
            float band = 0.f;
            // The fine layer only when the eye is ON the aircraft. Its grid is
            // +-2.6 m and the chase eye sits 1.65 m behind, so from there it
            // contributes a sliver -- and it is the most expensive layer to
            // cast, because DDA steps scale as 1/cell. Measured, chase pane,
            // 30 frames: 47.5 ms/frame with it, 22.9 ms without. Half the
            // render budget for a sliver.
            //
            // HANDOVERS AT THE EXACT MARKING RANGE. The 1.15 inflation belongs
            // only on the OUTERMOST edge, where it covers cells marked slightly
            // beyond maxIntegM before the aircraft moved. Applying it to an
            // internal handover creates a shell a layer OWNS and has no data
            // for -- 3.54 to 4.07 m on the mid layer -- while banding the next
            // layer out of it.
            //
            // And that shell renders as a CIRCLE, because range along a ray
            // grows radially: a surface at D is at range D on-axis and D/cos(t)
            // off it. A wall at 3.8 m is dropped on-axis and drawn beyond
            // t = acos(3.8/4.07) = 21 deg, which is a round blind spot filling
            // half the pane. Reported from the field as exactly that.
            if (haveNear && extraBack <= 0.f) {
                ls.push_back({&Mnear, 0.f, np.maxIntegM});
                band = np.maxIntegM;
            }
            ls.push_back({&M, band, mp.maxIntegM + extraBack});
            band = mp.maxIntegM + extraBack;
            if (C.farCell > 0.f)
                ls.push_back({&Mfar, band, fp.maxIntegM * 1.15f + extraBack});
            else if (!ls.empty())
                ls.back().range = mp.maxIntegM * 1.15f + extraBack;
            return ls;
        };

        // The ladder with the coarse rung removed -- the fine levels only, each
        // banded by its own honest range. Everything past the last of them is
        // the bearing field's territory.
        auto fineLadder = [&]() {
            std::vector<VoxelMap::Layer> ls;
            float band = 0.f;
            if (haveNear) { ls.push_back({&Mnear, 0.f, np.maxIntegM}); band = np.maxIntegM; }
            ls.push_back({&M, band, mp.maxIntegM});
            return ls;
        };

        // NOTE FOR THE CHASE PANE, which does NOT get the far field.
        //
        // A bearing bin is a function of direction FROM THE AIRCRAFT and holds
        // no position -- so it cannot be re-projected from a displaced eye. The
        // chase view sits 1.65 m behind, and rendering the field from there
        // would draw every far surface at the wrong bearing while looking
        // entirely plausible. That is the worst kind of wrong, so it is not
        // drawn at all and the chase pane is honestly short-ranged.
        //
        // This is the price of an egocentric representation and it should be
        // stated rather than discovered: the far field exists only from where
        // the aircraft is.

        const int64_t tDraw = cv::getTickCount();
        if (C.viewMode == 3) {
            // THE COMPARABLE, and the first version of it was a bad
            // instrument in a way that took a field screenshot to notice.
            //
            // It put the LEFT HALF OF THE FRAME through cubes and the RIGHT
            // HALF through bearings. Those are different parts of the scene, so
            // whichever representation happened to be pointed at the near
            // clutter looked better. On the frame that exposed it the near mass
            // was entirely on the right, so the bearing side carried it and the
            // cube side looked empty for reasons that had nothing to do with
            // either representation.
            //
            // Now BOTH halves show the SAME central slice of the frame, one
            // rendered each way. Same content, same projection, same instant --
            // which is the only arrangement that can answer the question.
            const int fw = PH / 2 * cp.width / cp.height, fh = PH / 2;
            const float R = C.farCell > 0.f ? fp.maxIntegM * 1.15f
                                            : C.farRangeM;
            cv::Mat a = fit(VoxelMap::renderLadder(ladder(0.f), pose.e, pose.n,
                                                   pose.u, yaw, pitchDeg,
                                                   fw, fh, cp.hfovDeg,
                                                   FpvStyle(), nullptr),
                            fw * 2, fh * 2);
            cv::Mat b = fit(BearingField::render(bfield, yaw, pitchDeg, fw, fh,
                                                 cp.hfovDeg, 0.f, R, pose.u),
                            fw * 2, fh * 2);
            // The middle half of the frame, taken from each.
            const int cw = a.cols / 2, x0 = a.cols / 4;
            cv::Mat both(a.rows, cw * 2, CV_8UC3);
            a(cv::Rect(x0, 0, cw, a.rows)).copyTo(both(cv::Rect(0, 0, cw, a.rows)));
            b(cv::Rect(x0, 0, cw, b.rows)).copyTo(both(cv::Rect(cw, 0, cw, b.rows)));
            cv::line(both, {cw, 0}, {cw, both.rows}, {40, 40, 40}, 2);
            fPane = cv::Mat(PH, PW, CV_8UC3, cv::Scalar(206, 208, 212));
            { const int tw = PW, th = std::min(PH, tw * both.rows / both.cols);
              fit(both, tw, th).copyTo(fPane(cv::Rect(0, (PH - th) / 2, tw, th))); }
            banner(fPane, C.farCell > 0.f
                   ? "CUBES | BEARINGS   same slice, both ways"
                   : "FINE CUBES | BEARINGS   same slice, both ways");
            int live = 0, tot = 0; bfield.occupancy(live, tot);
            char cb[128];
            if (C.farCell > 0.f) {
                // Independent bearings the coarse rung carries at its own range.
                const float degPerCell = C.farCell / std::max(1.f, fp.maxIntegM)
                                       * 180.f / sim::PI_F;
                std::snprintf(cb, sizeof(cb),
                              "%.0f m, %.0f bearings | %.0f m, %d bins, %.1f ms",
                              fp.maxIntegM, 87.f / degPerCell, C.farRangeM, live,
                              n ? bfieldUs / 1000.0 / n : 0.0);
            } else {
                // No coarse rung: say so rather than printing "0 bearings",
                // which is what the old label did and it read as a measurement.
                std::snprintf(cb, sizeof(cb),
                              "%.1f m, no coarse rung | %.0f m, %d bins, %.1f ms",
                              mp.maxIntegM, C.farRangeM, live,
                              n ? bfieldUs / 1000.0 / n : 0.0);
            }
            banner(fPane, cb, 34);
        } else if (C.viewMode == 1) {
            // OVERLAY. Rendered at the DEPTH IMAGE's own size and horizontal
            // FOV, so the two line up pixel for pixel and any disagreement is
            // real rather than a rendering artefact. A square render stretched
            // onto a 16:9 frame would be wrong in the vertical by the aspect
            // ratio and would look exactly like a calibration fault.
            cv::Mat mask;
            cv::Mat vox = VoxelMap::renderLadder(ladder(0.f), pose.e, pose.n,
                                                 pose.u, yaw, pitchDeg,
                                                 depth.cols, depth.rows,
                                                 cp.hfovDeg, FpvStyle(), &mask);
            // The far field belongs here too -- this pane is the alignment
            // instrument, and an instrument that omits half the map cannot
            // show a disagreement in the half it omits.
            if (C.farCell <= 0.f) {
                cv::Mat maskFar;
                cv::Mat far_ = BearingField::render(bfield, yaw, pitchDeg,
                                                    depth.cols, depth.rows,
                                                    cp.hfovDeg, mp.maxIntegM,
                                                    C.farRangeM, pose.u, &maskFar);
                for (int vv = 0; vv < vox.rows; ++vv) {
                    const uchar* mf = maskFar.ptr<uchar>(vv);
                    uchar* mn = mask.ptr<uchar>(vv);
                    const cv::Vec3b* fr = far_.ptr<cv::Vec3b>(vv);
                    cv::Vec3b* out = vox.ptr<cv::Vec3b>(vv);
                    for (int uu = 0; uu < vox.cols; ++uu)
                        if (!mn[uu] && mf[uu]) { out[uu] = fr[uu]; mn[uu] = 255; }
                }
            }
            cv::Mat base = C.depthEq ? colourDepthEq(depth, dMax)
                                     : colourDepth(depth, dMax);
            // Darken the camera image and lay the voxels over it only where
            // something was actually HIT. Blending the fog too would wash the
            // whole frame and hide the one thing being compared.
            base *= 0.45;
            vox.copyTo(base, mask);
            drawPlanInto(base, cp.hfovDeg);
            fPane = fit(base, PW, PH);
            banner(fPane, "OVERLAY  voxels ON the depth they came from");
            banner(fPane, C.farCell > 0.f
                   ? "aligned = intrinsics, frame and pose all agree"
                   : "cubes to 3.5 m then bearings -- the WHOLE map, not half", 44);
        } else if (C.viewMode == 2) {
            // CHASE. Same renderer, same projection, a viewpoint moved back and
            // up along the aircraft's own heading -- so the plan has extent in
            // the image instead of vanishing down the optical axis.
            // Framed on the PLAN, not on the map: the rollouts are
            // horizonS * vMax long (3 m at 1.5 m/s), and that is what has to
            // fill the pane. Sitting a whole map-range back with a 90 deg lens
            // put the aircraft in the middle of an empty field -- rendered it,
            // saw it, moved in.
            const float span = std::max(2.5f, tp.horizonS * vmax);
            // ELEVATION is what makes this a 3D view, and it is the number to
            // reason about rather than guess. A path's forward extent projects
            // to sin(elevation) of itself: at the 14 deg the first framing gave,
            // three metres of rollout became half a metre of picture and the fan
            // stayed a smear. Standing 0.7*span back and 0.72*span up, aimed at
            // the MIDDLE of the plan, puts the sightline about 31 deg above the
            // path -- half the forward extent survives into the image, and the
            // lateral spread still fills the width.
            const float back = span * 0.55f, up = span * 0.62f;
            const float aimAhead = back + span * 0.5f;
            // 55 deg, not the 90 the first-person pane uses. Field of view and
            // standing-back are two ways to buy the same context and the lens is
            // the cheap one: a narrower lens magnifies the plan without pushing
            // the camera further from it, and the map beyond the plan is not
            // what this pane is for.
            const float chaseFov = 55.f;
            const float yr = yaw * sim::PI_F / 180.f;
            const float ex2 = pose.e - std::sin(yr) * back;
            const float ny2 = pose.n - std::cos(yr) * back;
            const float uz2 = pose.u + up;
            const float pitchDown = -std::atan2(up, aimAhead) * 180.f / sim::PI_F;
            FpvStyle cs;
            cs.unknownFogM = 24.f; cs.unknownFogMax = 0.30f;
            // Cast at HALF the pane's resolution and upscale. Three levels is
            // three raycasts per pixel and the ladder made this pane the most
            // expensive thing in the program; a quarter of the rays costs
            // nothing visible on a picture made of cubes. The plan is still
            // drawn at full size, and projecting at 2x the render's dimensions
            // is exact -- focal length and principal point both scale with the
            // image, so the upscale and the projection agree by construction.
            cv::Mat v3 = fit(VoxelMap::renderLadder(ladder(back), ex2, ny2, uz2,
                                                    yaw, pitchDown, PW / 2, PH / 2,
                                                    chaseFov, cs, nullptr),
                             PW, PH);
            // Project through the CHASE pose, not the aircraft's.
            auto projC = [&](const std::array<float, 3>& w, cv::Point& o) {
                float u, v;
                if (!VoxelMap::fpvProject(ex2, ny2, uz2, yaw, pitchDown,
                                          v3.cols, v3.rows, chaseFov,
                                          w[0], w[1], w[2], u, v)) return false;
                o = cv::Point(int(u), int(v));
                return true;
            };
            auto polyC = [&](const std::vector<std::array<float, 3>>& pts,
                             cv::Scalar col, int th) {
                cv::Point a; bool have = false;
                for (const auto& w : pts) {
                    cv::Point b; const bool ok = projC(w, b);
                    if (ok && have) cv::line(v3, a, b, col, th, cv::LINE_AA);
                    a = b; have = ok;
                }
            };
            for (const auto& c : traj.candidates()) polyC(c, {210, 170, 120}, 1);
            polyC(traj.chosen(), {40, 190, 40}, 3);
            {   // the aircraft itself, so the paths have somewhere to start
                cv::Point a;
                if (projC({pose.e, pose.n, pose.u}, a)) {
                    cv::circle(v3, a, 7, {60, 60, 200}, 2, cv::LINE_AA);
                    cv::circle(v3, a, 2, {60, 60, 200}, cv::FILLED);
                }
            }
            fPane = fit(v3, PW, PH);
            banner(fPane, "CHASE  the map and the plan, from behind");
            char cb[96];
            std::snprintf(cb, sizeof(cb), "blue = you   green = chosen   "
                          "eye %.1f m behind", back);
            banner(fPane, cb, 44);
        } else {
            // FIRST PERSON, all levels. The fine cells carry the near field and
            // the coarse ones carry everything past where the fine map stopped
            // -- which is most of any real scene.
            // THE PANE MUST SHOW THE SENSOR'S OWN FRUSTUM AND NOT A WIDER ONE.
            //
            // This used to render SQUARE at vfov = hfov = 90. The camera is
            // 87 deg horizontal by 56 deg vertical, so the top and bottom
            // SEVENTEEN DEGREES of the pane lay outside anything the sensor had
            // ever looked at -- permanently, structurally unknown, and drawn as
            // fog exactly like a genuine gap in the map.
            //
            // At 4 m that is 1.2 m of blank above and below the data, which is
            // why a trunk read as "voxels at the root and nothing above it".
            // The map had the whole trunk; the pane was asserting a field of
            // view the camera does not have. Same family as the 8 m depth ramp
            // and the +-12 m slice crop: the instrument was wrong, and it made
            // honest absence of data look like a mapping failure.
            //
            // So: render at the CAMERA's aspect and hfov, then letterbox rather
            // than stretch. A stretched render would put the plan through an
            // aspect the raycast was never made with.
            //
            // VOXELS NEAR, BEARINGS FAR, and that split is the architecture now
            // rather than a resolution ladder all the way out. The fine levels
            // own everything inside the 0.25 m map's honest marking range; past
            // it the surface comes from the bearing field, which is a rough
            // standing copy of the depth image indexed by direction.
            //
            // Why the coarse voxel rung went. A cube must be sized for stereo's
            // RANGE error, which grows as Z^2, while its lateral error grows
            // only as Z -- the ratio is Z*sigma/B, a hundred to one at 20 m. So
            // a cube honest in range throws away all the bearing detail the
            // sensor still has, and the levels then need banded handovers whose
            // seams produced a fog disc, an empty coarse rung and a wall of
            // near faces in turn. Measured against the 1.0 m rung it replaces:
            // 3.2 ms against 22, ~1800 live bins against eleven independent
            // bearings, and no seam because there are no levels to hand over.
            const int fw = PH / 2 * cp.width / cp.height, fh = PH / 2;
            const float nearEnd = mp.maxIntegM;
            // --farcell keeps the OLD architecture end to end -- coarse voxel
            // rung in the render and in the direction score -- so the two can
            // still be compared on real data. Without it, fine voxels only, and
            // the bearing field owns everything past them.
            const bool useCubesFar = C.farCell > 0.f;
            cv::Mat maskNear;
            cv::Mat fpv = VoxelMap::renderLadder(
                useCubesFar ? ladder(0.f) : fineLadder(), pose.e, pose.n,
                pose.u, yaw, pitchDeg, fw, fh, cp.hfovDeg,
                FpvStyle(), &maskNear);
            if (!useCubesFar) {
                // The far field fills only where the fine map had nothing to
                // say, and only beyond its honest range. It is AWARENESS: the
                // swept-volume test never reads it, exactly as the coarse voxel
                // rung was never allowed to grant permission.
                cv::Mat maskFar;
                cv::Mat far_ = BearingField::render(bfield, yaw, pitchDeg, fw, fh,
                                                    cp.hfovDeg, nearEnd,
                                                    C.farRangeM, pose.u, &maskFar);
                for (int vv = 0; vv < fpv.rows; ++vv) {
                    const uchar* mn = maskNear.ptr<uchar>(vv);
                    const uchar* mf = maskFar.ptr<uchar>(vv);
                    const cv::Vec3b* fr = far_.ptr<cv::Vec3b>(vv);
                    cv::Vec3b* out = fpv.ptr<cv::Vec3b>(vv);
                    for (int uu = 0; uu < fpv.cols; ++uu)
                        if (!mn[uu] && mf[uu]) out[uu] = fr[uu];
                }
            }
            fpv = fit(fpv, fw * 2, fh * 2);
            drawPlanInto(fpv, cp.hfovDeg);
            // Letterbox into the pane. The bars are the SENSOR's blind zone and
            // are drawn darker than the unknown fog so the two cannot be
            // confused: pale means "not seen yet", grey means "cannot be seen".
            fPane = cv::Mat(PH, PW, CV_8UC3, cv::Scalar(206, 208, 212));
            {
                const int tw = PW, th = std::min(PH, tw * fpv.rows / fpv.cols);
                cv::Mat scaled = fit(fpv, tw, th);
                scaled.copyTo(fPane(cv::Rect(0, (PH - th) / 2, tw, th)));
            }
            banner(fPane, "FIRST PERSON  map + plan, from inside");
            char nb[110];
            if (useCubesFar) {
                std::snprintf(nb, sizeof(nb),
                              "%.2f-%.1f m cells out to %.0f m   pale = UNKNOWN",
                              haveNear ? np.cell : mp.cell, fp.cell, fp.maxIntegM);
            } else {
                int solid = 0, partial = 0, none = 0;
                bfield.supportHistogram(solid, partial, none);
                // SOLID against PARTIAL is the silhouette, reported as a number
                // rather than left to the eye: a bin is partial where only some
                // of that direction returned, which is an edge or foliage.
                std::snprintf(nb, sizeof(nb),
                              "%.2f m cells to %.1f m, bearings to %.0f m   "
                              "%d solid / %d edge",
                              haveNear ? np.cell : mp.cell, mp.maxIntegM,
                              C.farRangeM, solid, partial);
            }
            banner(fPane, nb, 44);
            if (C.turnHud) drawTurnHud(fPane);
        }

        drawUs += (cv::getTickCount() - tDraw) * 1000000 / cv::getTickFrequency();

        // Advance the heading AFTER everything that draws this frame. It used
        // to sit just after plan(), so the map and the plan were built at one
        // yaw and the views rendered at the next -- a frame's worth of rotation
        // (0.8 deg at 25 deg/s) between a path and the map it was planned in.
        // Under a pixel here, and wrong for the same reason the test's own
        // unsequenced argument was wrong: nothing about the picture reveals it.
        yaw += yawRateDps * (1.f / std::max(1, fps));
        // Kept in [0, 360). Unbounded it is arithmetically harmless to sin/cos
        // but loses float precision without limit, and it is the argument that
        // broke angDiffDeg above.
        yaw = std::fmod(yaw, 360.f); if (yaw < 0.f) yaw += 360.f;

        cv::Mat full(PH * 2, PW * 2, CV_8UC3);
        dPane.copyTo(full(cv::Rect(0, 0, PW, PH)));
        fPane.copyTo(full(cv::Rect(PW, 0, PW, PH)));
        sPane.copyTo(full(cv::Rect(0, PH, PW, PH)));
        pPane.copyTo(full(cv::Rect(PW, PH, PW, PH)));

#if SIM_HAVE_HIGHGUI
        if (!headless) {
            showScaled(WIN, full);
            const int k = cv::waitKey(paused ? 0 : 1);
            if (k == 'q' || k == 27) break;
            if (k == 'm') { backToMenu = true; break; }
            if (k == 'v') C.viewMode = (C.viewMode + 1) % 4;  // fpv / overlay / chase / compare
            if (k == 'a') C.turnHud = !C.turnHud;             // the turn arrow
            if (k == ' ') paused = !paused;
            if (k == 's') { cv::imwrite(out + "_frame.png", full);
                            std::printf("wrote %s_frame.png\n", out.c_str()); }
            // --- live settings, so a sweep does not need a restart ----------
            // Everything below used to be a command-line flag, which meant
            // killing the session and losing the map to answer "what does the
            // next value look like". These rebuild only what has to be rebuilt.
            if (k == 'h') { C.depthEq = !C.depthEq;
                            std::printf("[view] depth colour %s\n",
                                        C.depthEq ? "equalised" : "linear"); }
            if (k == 'f') {                                   // far layer range
                // LATCH the display scale first. dMax follows the coarsest
                // layer when depthMaxM is 0, so pressing 'f' used to move the
                // map range AND the picture's colour scale together -- two
                // variables on one key, which makes the comparison you are
                // pressing it to make impossible. '0' returns to automatic.
                if (C.depthMaxM <= 0.f) C.depthMaxM = dMax;
                static const float FAR[] = {0.f, 1.f, 2.f, 4.f, 8.f};
                int i = 0;
                for (int j = 0; j < 5; ++j)
                    if (std::fabs(FAR[j] - C.farCell) < 1e-3f) i = j;
                C.farCell = FAR[(i + 1) % 5];
                coarse.clear();
                if (C.farCell > 0.f) {
                    fp = VoxelMapParams();
                    fp.cell = C.farCell;
                    const int fn = std::max(48, int(256.f / C.farCell));
                    fp.nx = fn; fp.ny = fn; fp.nz = std::max(16, fn / 3);
                    fp.maxIntegM = std::sqrt(fp.cell * cam.fpx() * cp.baselineM / 0.25f) * 0.75f;
                    fp.maxCarveM = 40.f;
                    fp.integrateStride = std::max(4, C.stride * 2);
                    fp.depthSigCoef = mp.depthSigCoef;
                    fp.carveWinPx = std::max(5, fp.integrateStride * 2 + 1);
                    fp.minIntegM = mp.minIntegM;
                    Mfar.init(fp, pose.e, pose.n, pose.u);
                    coarse.push_back({&Mfar, fp.maxIntegM});
                    std::printf("[far] %.0f m cells -> marks to %.1f m\n",
                                fp.cell, fp.maxIntegM);
                } else {
                    std::printf("[far] off\n");
                }
            }
            if (k == 't') {                                   // ray stride
                C.stride = (C.stride >= 4) ? 1 : C.stride * 2;
                mp.integrateStride = C.stride;
                if (haveNear) np.integrateStride = C.stride * 2;
                if (C.farCell > 0.f) fp.integrateStride = std::max(4, C.stride * 2);
                std::printf("[map] ray stride %d\n", C.stride);
            }
            // DEPTH COLOUR SCALE -- the metres at which the pane's red->blue
            // ramp saturates. Nothing to do with the map or the sensor; it is
            // how the top-left picture is painted, and it was stuck at 8 m,
            // which is why everything past 8 m was one shade of blue.
            //
            // '-' and '+' because this is used on a Finnish keyboard, where
            // '[' and ']' are AltGr+8 and AltGr+9 and highgui may never see
            // them. '=' and the brackets stay as aliases for a US layout.
            if (k == '+' || k == '=' || k == ']' ||
                k == '-' || k == '_' || k == '[') {
                const bool up = (k == '+' || k == '=' || k == ']');
                float d = C.depthMaxM > 0.f ? C.depthMaxM : dMax;
                d = up ? d * 1.25f : d / 1.25f;
                C.depthMaxM = std::max(2.f, std::min(40.f, d));
                std::printf("[view] depth colour scale %.1f m\n", C.depthMaxM);
            }
            if (k == '0') { C.depthMaxM = 0.f;                 // back to automatic
                            std::printf("[view] depth colour scale: automatic\n"); }
            if (k == 'r') {                                   // record on/off
                if (recOpen) { rec.close(); recOpen = false;
                               std::printf("[rec] stopped\n"); }
                else { recFailed = false;
                       if (C.recordPath.empty()) C.recordPath = out + ".kdr"; }
            }
        }
#endif
        // Save the FIRST and the LAST frame. Frame 1 is the map with nothing in
        // it yet, which is honest but uninformative -- the accumulated view is
        // the one worth looking at, and a headless run that only dumps frame 1
        // shows an empty first-person pane and invites the wrong conclusion.
        lastComposite = full.clone();
        if (headless && (n == 1 || (total > 0 && n == total / 2))) {
            char b[256];
            std::snprintf(b, sizeof(b), "%s_%03d.png", out.c_str(), n);
            cv::imwrite(b, full);
            std::printf("wrote %s\n", b);
        }
        if (total > 0 && n >= total && mode == "replay") break;
    }

    if (recOpen) { rec.close(); std::printf("[rec] wrote %d frames to %s\n",
                                            n, C.recordPath.c_str()); }

    std::printf("\n--- %d frames from %s ---\n", n, src->name());
    if (n) {
        std::printf("  map integrate   %6.2f ms/frame\n", integUs / 1000.0 / n);
        std::printf("  plan            %6.2f ms/frame\n", planUs / 1000.0 / n);
        // NOT part of the total below. Drawing is what the desktop window costs
        // and the aircraft never pays it -- but it is the number that decides
        // whether this app feels alive on a laptop, so it is reported.
        std::printf("  (view render    %6.2f ms/frame, desktop only)\n",
                    drawUs / 1000.0 / n);
        std::printf("  ONBOARD TOTAL   %6.2f ms/frame  (%.0f Hz sustainable)\n",
                    (integUs + planUs) / 1000.0 / n,
                    1000.0 / std::max(0.001, (integUs + planUs) / 1000.0 / n));
    }
    if (C.audit && n) {
        auto census = [](const char* nm, const VoxelMap& L) {
            const VoxelMapParams& q = L.params();
            long occ = 0, fre = 0, unk = 0;
            for (int z = 0; z < q.nz; ++z)
              for (int y = 0; y < q.ny; ++y)
                for (int x = 0; x < q.nx; ++x) {
                    const float l = L.logAt(x, y, z);
                    if (l > q.occThresh) ++occ; else if (l < q.freeThresh) ++fre; else ++unk;
                }
            std::printf("  %-5s cell %.2f m: %8ld occupied, %10ld free, %10ld unknown\n",
                        nm, q.cell, occ, fre, unk);
        };
        std::printf("\n  --- census: what is in each grid ---\n");
        if (haveNear) census("near", Mnear);
        census("mid", M);
        if (C.farCell > 0.f) census("far", Mfar);
        std::printf("  mobility: free %.2f m, cmd %.2f m/s, blocked on %ld of %d frames\n",
                    auditFreeM / n, auditSpeed / n, auditBlocked, n);
        std::printf("\n  --- audit: does the map agree with the depth image? ---\n");
        std::printf("  a return at range r lands in a cell. is that cell OCCUPIED,\n"
                    "  in the layer that owns r? FREE is the bad answer -- it means\n"
                    "  something carved through a surface it had already seen.\n");
        std::printf("  %10s %10s %8s %8s %8s %8s   owner\n",
                    "range m", "returns", "OCC", "FREE", "UNKN", "OFFGRID");
        for (int i = 0; i < kAuditN; ++i) {
            if (!auditSeen[i]) continue;
            const float mid = (kAuditEdge[i] + kAuditEdge[i + 1]) * 0.5f;
            const char* owner = "mid";
            if (haveNear && mid < np.maxIntegM) owner = "near";
            else if (mid > mp.maxIntegM) owner = C.farCell > 0.f ? "far" : "none";
            char rng[32];
            std::snprintf(rng, sizeof(rng), "%.1f-%.1f", kAuditEdge[i], kAuditEdge[i + 1]);
            const double t = double(auditSeen[i]);
            std::printf("  %10s %10ld %7.1f%% %7.1f%% %7.1f%% %7.1f%%   %s\n",
                        rng, auditSeen[i], 100.0 * auditOcc[i] / t,
                        100.0 * auditFree[i] / t, 100.0 * auditUnk[i] / t,
                        100.0 * auditOob[i] / t, owner);
        }
    }
    if (n && !lastComposite.empty()) {
        cv::imwrite(out + "_last.png", lastComposite);
        std::printf("  wrote %s_last.png (frame %d, the accumulated view)\n",
                    out.c_str(), n);
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
    // Coarse layer, and the honest range it buys: Z_max = sqrt(cell*f*B/sigma)
    // *0.75, so on the D435i's 50 mm baseline these are roughly
    // off / 7.1 / 10.0 / 14.2 / 20.1 m. 0 disables the layer.
    static const float FARS[] = {0.f, 1.f, 2.f, 4.f, 8.f};
    int iCell = 2, iVmax = 2, iSpin = 0, iFar = 2;
    std::string note;

    for (;;) {
        cv::Mat menu = renderMenu(C, recs, btns, note);
        showScaled(WIN, menu);
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
        else if (id == 14) { C.stride = (C.stride >= 4) ? 1 : C.stride * 2; }
        else if (id == 15) { C.dirMode = 1 - C.dirMode; }
        else if (id == 16) { C.viewMode = (C.viewMode + 1) % 4; }
        else if (id == 17) { C.turnHud = !C.turnHud; }
        else if (id == 18) { iFar = (iFar + 1) % 5; C.farCell = FARS[iFar]; }
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
