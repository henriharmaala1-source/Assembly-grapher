// `kestrel mission scenarios FILE.kms` -- THE BEHAVIOUR, SEEN FROM THE AIRCRAFT.
//
// A mission is only believed once it has been watched doing the thing. Each
// run here is one randomised encounter, flown by the aircraft's own SCRIPT
// mode (the C++ that runs on the Pi, not a copy):
//
//   * the aircraft starts at a RANDOM HEIGHT (3-25 m) over open ground,
//     facing north, and the program does whatever it says -- typically
//     `cruise` until something is seen;
//   * one TARGET (door, person, crate or lightpole, or the one asked for)
//     stands at a RANDOM point ahead -- 70-110 m out, up to 25 m to a side --
//     out of reach at the start, so it COMES INTO VIEW, at a different moment
//     and angle every run;
//   * a DETECTOR is simulated from the target's true geometry: its box is
//     the projection of its real extent, run at a few Hz, jittered, and only
//     when it is in frame, at least 8 px tall (so a 2 m door is found at
//     ~60 m, a 0.75 m crate at ~25 m), within 90 m and not hidden; `track` hands it to
//     a simulated lock tracker that follows it every tick, scaling its box;
//   * the airframe answers with lag (yaw rate 0.3 s, speed 0.35 s), 4 m/s
//     and 1.5 m/s per full stick -- the numbers SCRIPT mode is given.
//
// Every run writes an FPV VIDEO from the aircraft's camera -- the true world,
// shaded -- with the detection box, the lock, the point it is steering at,
// its state, step, sticks, height and range drawn on, and a map from above
// in the corner; plus a CONTACT SHEET of the moments that matter (first
// sight, every state change, the end). --show plays them in a window.
//
// A run PASSES when the aircraft hit nothing (ground, target, scenery) and
// the program ended without being stopped (a failure with no else, the
// fence, the timeout). report.csv says what happened in each.
#include "kestrel_scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if KESTREL_HAVE_VIDEOIO
#include <opencv2/videoio.hpp>
#endif
#ifdef SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "depth_camera.hpp"
#include "footage.hpp"
#include "mission_compile.hpp"
#include "mission_program.hpp"
#include "script_mode.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

namespace kscen {
namespace {

constexpr double kPi = 3.14159265358979;
constexpr double kD2R = kPi / 180.0;
constexpr float kDt = 0.05f;
constexpr float kGround = 0.5f;          // the top of the ground slab

struct Options {
    std::string kms;
    int runs = 6;
    unsigned seed = 1;
    std::string target = "random";
    std::string outDir = "scenarios";
    bool video = true, show = false;
    float tilt = 20.f, hfov = 87.f;      // the camera: deg down, horizontal FoV
    float detHz = 5.f, noisePx = 2.f;
    float seconds = 120.f;
    // The encounter's ranges, all drawn per run from the seed.
    float altLo = 3.f, altHi = 25.f;     // start height above the ground
    float distLo = 70.f, distHi = 110.f; // how far ahead the target stands
    float side = 25.f;                   // ... and up to this far to a side
    // The detector's and the air's imperfections.
    float dropout = 0.f;                 // chance a detector run misses it
    int   latency = 0;                   // ticks (50 ms) its picture is late
    float wind = 0.f;                    // m/s steady drift, random direction
    int W = 480, H = 270;
};

struct TargetKind { const char* label; float w, d, h; float tex; };
const TargetKind KINDS[] = {
    {"door", 1.0f, 0.25f, 2.0f, 0.85f},
    {"person", 0.5f, 0.5f, 1.75f, 0.80f},
    {"crate", 1.0f, 1.0f, 0.75f, 0.90f},
    {"lightpole", 0.25f, 0.25f, 6.0f, 0.55f},
};

struct Scenario {
    unsigned seed = 0;
    float alt0 = 10.f;                   // above the ground
    const TargetKind* kind = &KINDS[0];
    float te = 0, tn = 40;               // the target's centre on the ground
};

// ------------------------------------------------------------ the world
// 0.25 m cells over e -40..40, n -10..140, 0..32 m up: a ground slab, the
// target, and scenery kept OUT of the corridor the aircraft cruises down so
// a crash means the behaviour hit something it was flying at.
struct World {
    sim::VoxelWorld w;
    float x0, y0, x1, y1, z0, z1;        // the target's solid box (world m)
    void box(float e0, float n0, float u0, float e1, float n1, float u1, float tex) {
        int a0, b0, c0, a1, b1, c1;
        w.worldToCell(e0 + 1e-3f, n0 + 1e-3f, u0 + 1e-3f, a0, b0, c0);
        w.worldToCell(e1 - 1e-3f, n1 - 1e-3f, u1 - 1e-3f, a1, b1, c1);
        for (int z = c0; z <= c1; ++z)
            for (int y = b0; y <= b1; ++y)
                for (int x = a0; x <= a1; ++x)
                    if (w.inBounds(x, y, z)) { w.set(x, y, z, true); w.setTex(x, y, z, tex); }
    }
};

std::unique_ptr<World> buildWorld(const Scenario& sc, std::mt19937& rng) {
    auto W = std::make_unique<World>();
    W->w.init(0.25f, -40.f, -10.f, 0.f, 320, 600, 128);
    W->box(-40.f, -10.f, 0.f, 40.f, 140.f, kGround, 0.35f);
    std::uniform_real_distribution<float> u01(0.f, 1.f);
    // Scenery: trees and blocks either side of the corridor.
    for (int i = 0; i < 26; ++i) {
        const float side = u01(rng) < 0.5f ? -1.f : 1.f;
        const float e = side * (30.f + 8.f * u01(rng)), n = -5.f + 140.f * u01(rng);
        if (u01(rng) < 0.6f) {
            W->box(e - 0.2f, n - 0.2f, kGround, e + 0.2f, n + 0.2f, kGround + 4.f, 0.6f);
            W->box(e - 1.2f, n - 1.2f, kGround + 3.f, e + 1.2f, n + 1.2f, kGround + 6.f, 0.7f);
        } else {
            const float s = 2.f + 4.f * u01(rng), h = 2.f + 8.f * u01(rng);
            W->box(e - s / 2, n - s / 2, kGround, e + s / 2, n + s / 2, kGround + h, 0.3f);
        }
    }
    const TargetKind& k = *sc.kind;
    W->x0 = sc.te - k.w / 2; W->x1 = sc.te + k.w / 2;
    W->y0 = sc.tn - k.d / 2; W->y1 = sc.tn + k.d / 2;
    W->z0 = kGround; W->z1 = kGround + k.h;
    W->box(W->x0, W->y0, W->z0, W->x1, W->y1, W->z1, k.tex);
    return W;
}

// ------------------------------------------------------------ the camera
struct Air {
    double e = 0, n = 0, u = 10, yaw = 0, v = 0, vr = 0, rate = 0;
};
sim::CamPose camOf(const Air& a, const Options& o) {
    sim::CamPose p;
    p.e = float(a.e); p.n = float(a.n); p.u = float(a.u);
    p.yawDeg = float(a.yaw); p.pitchDeg = -o.tilt;
    return p;
}
// A world point into the image (pixels, top-left origin), or false behind.
bool proj(const sim::CamPose& c, const Options& o, double x, double y, double z, double& u, double& v) {
    const double yr = c.yawDeg * kD2R, pr = c.pitchDeg * kD2R;
    const double dx = x - c.e, dy = y - c.n, dz = z - c.u;
    const double fE = dx * std::cos(yr) - dy * std::sin(yr);
    const double fN = dx * std::sin(yr) + dy * std::cos(yr);
    const double lam = fN * std::cos(pr) + dz * std::sin(pr);
    if (lam <= 0.1) return false;
    const double f = (o.W * 0.5) / std::tan(o.hfov * 0.5 * kD2R);
    u = (fE / lam) * f + (o.W - 1) * 0.5;
    v = ((fN * std::sin(pr) - dz * std::cos(pr)) / lam) * f + (o.H - 1) * 0.5;
    return true;
}
// The target's box in the image, and how much of it is inside the frame.
bool targetBox(const World& W, const sim::CamPose& c, const Options& o, cv::Rect2d& box,
               double& inFrac) {
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (int i = 0; i < 8; ++i) {
        double u, v;
        if (!proj(c, o, (i & 1) ? W.x1 : W.x0, (i & 2) ? W.y1 : W.y0, (i & 4) ? W.z1 : W.z0, u, v))
            return false;
        x0 = std::min(x0, u); x1 = std::max(x1, u); y0 = std::min(y0, v); y1 = std::max(y1, v);
    }
    box = cv::Rect2d(x0, y0, x1 - x0, y1 - y0);
    const cv::Rect2d in = box & cv::Rect2d(0, 0, o.W, o.H);
    inFrac = box.area() > 0 ? in.area() / box.area() : 0;
    return true;
}

// ------------------------------------------------------------ drawing
void label(cv::Mat& im, const std::string& s, cv::Point p, double sc, cv::Scalar c, int th = 1) {
    cv::putText(im, s, p + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, sc, {0, 0, 0}, th + 1, cv::LINE_AA);
    cv::putText(im, s, p, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
}
void stickBar(cv::Mat& im, int x, int y, const char* name, float v) {
    cv::rectangle(im, {x, y, 60, 8}, {40, 40, 40}, cv::FILLED);
    const int mid = x + 30, end = mid + int(std::max(-1.f, std::min(1.f, v)) * 30);
    cv::rectangle(im, {std::min(mid, end), y, std::abs(end - mid) + 1, 8},
                  v >= 0 ? cv::Scalar(90, 200, 120) : cv::Scalar(90, 140, 240), cv::FILLED);
    label(im, name, {x + 64, y + 8}, 0.33, {230, 230, 230});
}

struct Frame { cv::Mat im; std::string why; };

// ------------------------------------------------------------ one run
struct Result {
    Scenario sc;
    double closestH = 1e9;             // horizontal: 0 is right over it
    bool crashed = false, failed = false, finished = false, timedOut = false;
    std::string end, crashOn;
    double firstSeenS = -1, firstSeenRange = -1, reactS = -1;
    double closest = 1e9, minAlt = 1e9, endRange = 0, endAlt = 0;
    std::string fcEnd = "none";
};

Result flyOne(const kms::Program& prog, const Scenario& sc, const Options& o, int idx,
              std::vector<Frame>* sheet) {
    std::mt19937 rng(sc.seed * 7919u + 13u);
    std::normal_distribution<double> jit(0.0, o.noisePx);
    auto world = buildWorld(sc, rng);
    const World& W = *world;

    ScriptMode::Params sp;
    sp.mission.useMap = false;
    sp.detHfovDeg = o.hfov;
    sp.detTiltDeg = -o.tilt;
    sp.mpsPerStick = 4.f;
    sp.vertMpsPerStick = 1.5f;
    ScriptMode m(sp);
    std::string err;
    Result r; r.sc = sc;
    if (!m.load(prog, &err)) { r.failed = true; r.end = "refused: " + err; return r; }

    Air a; a.u = kGround + sc.alt0;
    WorldState s;
    s.estValid = true; s.estEphM = 0.5f; s.vehLink = true; s.vehArmed = true; s.vehBattery = 0.9f;
    m.onEnter(s);
    s.missionGo = true;

    const int detEvery = std::max(1, int(std::lround(20.0 / std::max(0.5f, o.detHz))));
    int lastReq = 0, lastRel = 0;
    bool locked = false;
    std::vector<cv::Point2d> trail;
    std::string lastState = "-";
    bool sawFirst = false;

#if KESTREL_HAVE_VIDEOIO
    cv::VideoWriter vw;
    if (o.video) {
        const std::string path = o.outDir + cv::format("/run_%02d.avi", idx);
        vw.open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10.0, {o.W, o.H});
    }
#endif
    const int ticks = int(o.seconds / kDt);
    ControlCmd cmd;
    std::vector<cv::Rect2d> lastDets;
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double windDir = 2 * kPi * u01(rng);
    const double windE = o.wind * std::sin(windDir), windN = o.wind * std::cos(windDir);
    std::vector<Air> past;                       // for a late detector
    for (int t = 0; t < ticks; ++t) {
        const double now = t * kDt;
        const sim::CamPose cam = camOf(a, o);
        past.push_back(a);
        if (int(past.size()) > o.latency + 1) past.erase(past.begin());
        const sim::CamPose camLate = camOf(past.front(), o);
        // ---- the detector: the target's true box, at its rate, jittered,
        // when it is in frame, close enough, big enough and not hidden.
        cv::Rect2d tb; double inFrac = 0;
        const bool inView = targetBox(W, cam, o, tb, inFrac);
        const double cx = (W.x0 + W.x1) / 2, cy = (W.y0 + W.y1) / 2, cz = (W.z0 + W.z1) / 2;
        const double rng3 = std::sqrt((cx - a.e) * (cx - a.e) + (cy - a.n) * (cy - a.n) + (cz - a.u) * (cz - a.u));
        bool visible = false;
        if (inView && inFrac > 0.5 && tb.height >= 8 && rng3 < 90.0) {
            const double dx = (cx - a.e) / rng3, dy = (cy - a.n) / rng3, dz = (cz - a.u) / rng3;
            const float hit = W.w.raycast(float(a.e), float(a.n), float(a.u), float(dx), float(dy),
                                          float(dz), float(rng3) + 1.f, nullptr);
            visible = hit >= float(rng3) - 1.0f;
        }
        s.tickMonoS = now; s.corridorStampS = now;
        s.corridorValid = true; s.corridorOpen = 1.f; s.corridorOffset = 0.f;
        if (t % detEvery == 0) {
            s.detections.clear(); lastDets.clear();
            // A LATE picture: the box from where the camera was `latency`
            // ticks ago, handed over now. A DROPOUT: it ran and missed.
            cv::Rect2d lb = tb; double lf = inFrac;
            const bool lateIn = o.latency > 0 ? targetBox(W, camLate, o, lb, lf) : inView;
            const bool missed = u01(rng) < o.dropout;
            if (visible && lateIn && lf > 0.5 && !missed) {
                const cv::Rect2d tb = lb;
                Detection d; d.label = sc.kind->label; d.confidence = 0.8f;
                cv::Rect2d jb(tb.x + jit(rng), tb.y + jit(rng), tb.width + jit(rng), tb.height + jit(rng));
                d.box = cv::Rect(int(jb.x), int(jb.y), std::max(3, int(jb.width)), std::max(3, int(jb.height)));
                s.detections.push_back(d);
                lastDets.push_back(jb);
                if (!sawFirst) { sawFirst = true; r.firstSeenS = now; r.firstSeenRange = rng3; }
            }
            s.detStampS = now;
        }
        // ---- the lock tracker: designated by the script, follows every
        // tick, a square box that scales with the object (as `fused` does).
        if (s.trackRequestSeq != lastReq) { lastReq = s.trackRequestSeq; locked = visible; }
        if (s.trackReleaseSeq != lastRel) { lastRel = s.trackReleaseSeq; locked = false; }
        if (locked && !(inView && inFrac > 0.3)) locked = false;
        s.targetValid = s.targetLocked = locked;
        s.targetCore = "fused";
        if (locked) {
            const double side = std::max(tb.width, tb.height);
            s.targetBox = cv::Rect(int(tb.x + tb.width / 2 - side / 2), int(tb.y + tb.height / 2 - side / 2),
                                   int(side), int(side));
            s.targetStampS = now; s.targetFixAgeS = 0.f;
        }
        // ---- what the aircraft knows of itself
        s.estPe = float(a.e); s.estPn = float(a.n); s.estPu = float(a.u);
        s.vehAltM = float(a.u - kGround); s.vehYawDeg = float(a.yaw);
        s.vehGroundspeed = float(std::fabs(a.v)); s.estSpeed = s.vehGroundspeed;

        ControlCtx ctx; ctx.dt = kDt; ctx.frameW = o.W; ctx.frameH = o.H;
        cmd = m.update(s, ctx);
        s.control = cmd;
        if (!cmd.valid) cmd = ControlCmd();
        // ---- the airframe
        a.rate += (cmd.yaw * 90.0 - a.rate) * kDt / 0.3;
        a.yaw = std::fmod(a.yaw + a.rate * kDt + 360.0, 360.0);
        a.v += (cmd.pitch * 4.0 - a.v) * kDt / 0.35;
        a.vr += (cmd.roll * 4.0 - a.vr) * kDt / 0.35;
        a.u += cmd.throttle * 1.5 * kDt;
        const double yr = a.yaw * kD2R;
        a.e += std::sin(yr) * a.v * kDt + std::cos(yr) * a.vr * kDt + windE * kDt;
        a.n += std::cos(yr) * a.v * kDt - std::sin(yr) * a.vr * kDt + windN * kDt;
        trail.push_back({a.e, a.n});

        // ---- what happened
        const double dh = std::hypot(cx - a.e, cy - a.n);
        r.closest = std::min(r.closest, rng3);
        r.closestH = std::min(r.closestH, dh);
        r.minAlt = std::min(r.minAlt, a.u - kGround);
        const std::string stNow = m.stateName().empty() ? "-" : m.stateName();
        const bool stateChanged = stNow != lastState;
        if (stateChanged && sawFirst && r.reactS < 0 && t > 0) r.reactS = now;
        int cxl, cyl, czl;
        W.w.worldToCell(float(a.e), float(a.n), float(a.u), cxl, cyl, czl);
        bool hitSomething = a.u - 0.15 <= kGround;
        for (int dz = -1; dz <= 1 && !hitSomething; ++dz)
            for (int dy = -1; dy <= 1 && !hitSomething; ++dy)
                for (int dx = -1; dx <= 1 && !hitSomething; ++dx)
                    if (W.w.solid(cxl + dx, cyl + dy, czl + dz)) hitSomething = true;
        const bool landed = s.fcRequest == WorldState::FcRequest::LAND;

        // ---- the picture
        const bool wantFrame = (o.video || o.show || sheet) && (t % 2 == 0 || stateChanged);
        if (wantFrame) {
            sim::FootageStyle fs; fs.gridM = 2.f;
            cv::Mat im = sim::renderFootage(W.w, cam, o.W, o.H, o.hfov, 90.f, kGround + 0.05f, fs);
            for (const cv::Rect2d& d : lastDets) {
                cv::rectangle(im, d, {60, 230, 255}, 1, cv::LINE_AA);
                label(im, std::string(sc.kind->label), {int(d.x), int(d.y) - 3}, 0.35, {60, 230, 255});
            }
            if (locked) {
                cv::rectangle(im, s.targetBox, {40, 140, 255}, 2, cv::LINE_AA);
                label(im, "LOCK", {s.targetBox.x, s.targetBox.y + s.targetBox.height + 12}, 0.38, {40, 140, 255});
            }
            double apx, apy;
            if (m.aimPoint(apx, apy)) {
                const cv::Point c(int(o.W / 2 + apx), int(o.H / 2 + apy));
                cv::circle(im, c, 7, {255, 255, 255}, 2, cv::LINE_AA);
                cv::circle(im, c, 7, {230, 110, 40}, 1, cv::LINE_AA);
                cv::line(im, c + cv::Point(-12, 0), c + cv::Point(-5, 0), {230, 110, 40}, 2);
                cv::line(im, c + cv::Point(5, 0), c + cv::Point(12, 0), {230, 110, 40}, 2);
            }
            cv::drawMarker(im, {o.W / 2, o.H / 2}, {220, 220, 220}, cv::MARKER_CROSS, 10, 1);
            // Text: state, step, numbers.
            cv::rectangle(im, {0, 0, o.W, 30}, {20, 20, 20}, cv::FILLED);
            label(im, cv::format("run %d  t %.1f s  state %s  line %d", idx, now, stNow.c_str(), s.scriptLine),
                  {6, 12}, 0.38, {255, 255, 255});
            std::string st = m.status();
            if (st.size() > 70) st = st.substr(0, 70);
            label(im, st, {6, 25}, 0.36, {170, 220, 255});
            stickBar(im, 6, o.H - 46, "pitch", cmd.pitch);
            stickBar(im, 6, o.H - 34, "yaw", cmd.yaw);
            stickBar(im, 6, o.H - 22, "throttle", cmd.throttle);
            stickBar(im, 6, o.H - 10, "roll", cmd.roll);
            label(im, cv::format("height %.1f m  speed %.1f m/s", a.u - kGround, a.v), {130, o.H - 22},
                  0.38, {255, 255, 255});
            label(im, cv::format("%s: distance %.1f m (across)  range %.1f m", sc.kind->label, dh, rng3),
                  {130, o.H - 8}, 0.38, {255, 255, 255});
            // Where the RUNTIME remembers it to be -- what it flies by once
            // the object is out of view (under the aircraft).
            double me, mn, mu;
            if (m.memory(sc.kind->label, me, mn, mu)) {
                double pu, pv;
                if (proj(cam, o, me, mn, kGround, pu, pv) && pu > 0 && pv > 0 && pu < o.W && pv < o.H) {
                    const cv::Point p{int(pu), int(pv)};
                    cv::drawMarker(im, p, {255, 120, 255}, cv::MARKER_DIAMOND, 14, 2, cv::LINE_AA);
                    label(im, "memory", p + cv::Point(9, 4), 0.33, {255, 120, 255});
                }
            }
            // The map from above, top right: the course, the target, the trail.
            const int mw = 92, mh = 120, mx = o.W - mw - 4, my = 34;
            cv::rectangle(im, {mx, my, mw, mh}, {34, 34, 34}, cv::FILLED);
            cv::rectangle(im, {mx, my, mw, mh}, {90, 90, 90}, 1);
            auto P = [&](double e, double n) {
                return cv::Point(mx + int((e + 32) / 64.0 * mw), my + mh - int((n + 5) / 125.0 * mh));
            };
            for (size_t i = 1; i < trail.size(); i += 4)
                cv::line(im, P(trail[i - 1].x, trail[i - 1].y), P(trail[i].x, trail[i].y), {255, 170, 60}, 1);
            cv::rectangle(im, P(W.x0, W.y1), P(W.x1, W.y0) + cv::Point(1, 1), {40, 140, 255}, cv::FILLED);
            cv::circle(im, P(a.e, a.n), 3, {255, 255, 255}, cv::FILLED);
            if (hitSomething) label(im, "CRASH", {o.W / 2 - 40, o.H / 2}, 0.9, {60, 60, 255}, 2);
#if KESTREL_HAVE_VIDEOIO
            if (vw.isOpened()) vw.write(im);
#endif
#ifdef SIM_HAVE_HIGHGUI
            if (o.show) { cv::imshow("kestrel scenarios", im); if ((cv::waitKey(1) & 0xff) == 27) exit(0); }
#endif
            if (sheet) {
                std::string why;
                if (t == 0) why = "start";
                else if (sawFirst && r.firstSeenS == now) why = "first seen";
                else if (stateChanged) why = "-> " + stNow;
                if (!why.empty() && sheet->size() < 11) sheet->push_back({im.clone(), why});
                if (hitSomething || m.finished() || landed || t == ticks - 1) sheet->push_back({im.clone(), "end"});
            }
        }
        lastState = stNow;
        if (hitSomething) {
            r.crashed = true;
            r.crashOn = a.u - 0.15 <= kGround ? "ground"
                      : (a.e > W.x0 - 0.5 && a.e < W.x1 + 0.5 && a.n > W.y0 - 0.5 && a.n < W.y1 + 0.5) ? "target" : "scenery";
            break;
        }
        if (m.finished() || landed) break;
        if (t == ticks - 1) r.timedOut = true;
    }
    r.finished = m.finished();
    r.failed = m.failed();
    r.end = m.status();
    r.endRange = std::hypot((W.x0 + W.x1) / 2 - a.e, (W.y0 + W.y1) / 2 - a.n);
    r.endAlt = a.u - kGround;
    r.fcEnd = s.fcRequest == WorldState::FcRequest::LAND ? "land"
            : s.fcRequest == WorldState::FcRequest::RTL ? "rtl" : "none";
    return r;
}

cv::Mat contactSheet(const std::vector<Frame>& f, int W, int H) {
    if (f.empty()) return cv::Mat();
    const int cols = 3, rows = int((f.size() + cols - 1) / cols);
    cv::Mat sheet(rows * (H + 18), cols * W, CV_8UC3, cv::Scalar(24, 24, 24));
    for (size_t i = 0; i < f.size(); ++i) {
        const int x = int(i % cols) * W, y = int(i / cols) * (H + 18);
        f[i].im.copyTo(sheet(cv::Rect(x, y + 18, W, H)));
        label(sheet, f[i].why, {x + 6, y + 13}, 0.45, {255, 255, 255});
    }
    return sheet;
}

int usage() {
    std::fprintf(stderr,
        "kestrel mission scenarios FILE.kms|FILE.kmb [--runs N] [--seed S]\n"
        "    [--target random|door|person|crate|lightpole] [--out DIR] [--no-video]\n"
        "    [--show] [--tilt DEG] [--hfov DEG] [--det-hz HZ] [--noise PX] [--seconds S]\n"
        "    [--alt LO HI] [--dist LO HI] [--side M] [--dropout P] [--latency TICKS] [--wind M/S]\n");
    return 2;
}

}  // namespace

int run(const std::vector<std::string>& args) {
    if (args.empty()) return usage();
    Options o;
    o.kms = args[0];
    for (size_t i = 1; i < args.size(); ++i) {
        const bool more = i + 1 < args.size();
        const std::string& k = args[i];
        if (k == "--runs" && more) o.runs = std::max(1, std::atoi(args[++i].c_str()));
        else if (k == "--seed" && more) o.seed = unsigned(std::atoi(args[++i].c_str()));
        else if (k == "--target" && more) o.target = args[++i];
        else if (k == "--out" && more) o.outDir = args[++i];
        else if (k == "--no-video") o.video = false;
        else if (k == "--show") o.show = true;
        else if (k == "--tilt" && more) o.tilt = float(std::atof(args[++i].c_str()));
        else if (k == "--hfov" && more) o.hfov = float(std::atof(args[++i].c_str()));
        else if (k == "--det-hz" && more) o.detHz = float(std::atof(args[++i].c_str()));
        else if (k == "--noise" && more) o.noisePx = float(std::atof(args[++i].c_str()));
        else if (k == "--seconds" && more) o.seconds = float(std::atof(args[++i].c_str()));
        else if (k == "--alt" && i + 2 < args.size()) { o.altLo = float(std::atof(args[++i].c_str())); o.altHi = float(std::atof(args[++i].c_str())); }
        else if (k == "--dist" && i + 2 < args.size()) { o.distLo = float(std::atof(args[++i].c_str())); o.distHi = float(std::atof(args[++i].c_str())); }
        else if (k == "--side" && more) o.side = float(std::atof(args[++i].c_str()));
        else if (k == "--dropout" && more) o.dropout = float(std::atof(args[++i].c_str()));
        else if (k == "--latency" && more) o.latency = std::max(0, std::atoi(args[++i].c_str()));
        else if (k == "--wind" && more) o.wind = float(std::atof(args[++i].c_str()));
        else { std::fprintf(stderr, "[scenarios] unknown argument: %s\n", k.c_str()); return usage(); }
    }
    o.altLo = std::max(0.5f, o.altLo); o.altHi = std::min(30.f, std::max(o.altLo, o.altHi));
    o.distLo = std::max(5.f, o.distLo); o.distHi = std::min(135.f, std::max(o.distLo, o.distHi));
    o.side = std::min(35.f, std::max(0.f, o.side));
    o.dropout = std::min(0.95f, std::max(0.f, o.dropout));
    kms::Program prog;
    if (o.kms.size() > 4 && o.kms.compare(o.kms.size() - 4, 4, ".kmb") == 0) {
        std::string err;
        if (!kms::loadProgram(o.kms, prog, &err)) { std::fprintf(stderr, "%s: %s\n", o.kms.c_str(), err.c_str()); return 1; }
    } else {
        const kms::CompileResult cr = kms::compileFile(o.kms);
        if (!cr.ok) { std::fputs(cr.report(o.kms).c_str(), stderr); return 1; }
        prog = cr.program;
    }
    const bool wantFiles = o.video;
    if (wantFiles) {
#ifdef _WIN32
        std::system(("mkdir \"" + o.outDir + "\" 2>nul").c_str());
#else
        std::system(("mkdir -p \"" + o.outDir + "\"").c_str());
#endif
    }
    std::printf("[scenarios] \"%s\": %d run(s), seed %u, camera %.0f deg down, %.0f deg FoV%s\n",
                prog.name.c_str(), o.runs, o.seed, o.tilt, o.hfov,
                wantFiles ? (", videos in " + o.outDir + "/").c_str() : "");
    std::mt19937 rng(o.seed);
    std::uniform_real_distribution<float> u01(0.f, 1.f);
    std::ofstream csv;
    if (wantFiles) {
        csv.open(o.outDir + "/report.csv");
        csv << "run,seed,target,start_alt_m,target_e,target_n,first_seen_s,first_seen_range_m,"
               "reacted_s,closest_m,closest_across_m,min_alt_m,end_range_m,end_alt_m,fc,result,end\n";
    }
    int passed = 0;
    for (int i = 0; i < o.runs; ++i) {
        Scenario sc;
        sc.seed = o.seed * 1000u + unsigned(i);
        sc.alt0 = o.altLo + (o.altHi - o.altLo) * u01(rng);
        int kindIdx = int(u01(rng) * 4.f) % 4;
        for (int k = 0; k < 4; ++k) if (o.target == KINDS[k].label) kindIdx = k;
        sc.kind = &KINDS[kindIdx];
        sc.tn = o.distLo + (o.distHi - o.distLo) * u01(rng);
        sc.te = -o.side + 2.f * o.side * u01(rng);
        std::vector<Frame> frames;
        const Result r = flyOne(prog, sc, o, i, wantFiles ? &frames : nullptr);
        const bool ok = !r.crashed && !r.failed && !r.timedOut;
        passed += ok;
        const std::string result = r.crashed ? "CRASH (" + r.crashOn + ")" : r.failed ? "STOPPED"
                                 : r.timedOut ? "TIMED OUT" : "ok";
        std::printf("  run %d: %-9s from %4.1f m, %s at %.0f m %s %.0f m | seen %s | %s | closest %.1f m "
                    "(%.1f across), "
                    "lowest %.1f m, ends %.1f m from it at %.1f m (%s)\n      %s\n",
                    i, sc.kind->label, sc.alt0, sc.kind->label, sc.tn, sc.te >= 0 ? "right" : "left",
                    std::fabs(sc.te),
                    r.firstSeenS >= 0 ? cv::format("at %.1f s, %.0f m", r.firstSeenS, r.firstSeenRange).c_str() : "never",
                    result.c_str(), r.closest, r.closestH, r.minAlt, r.endRange, r.endAlt, r.fcEnd.c_str(), r.end.c_str());
        if (wantFiles) {
            const cv::Mat sheet = contactSheet(frames, o.W, o.H);
            if (!sheet.empty()) cv::imwrite(o.outDir + cv::format("/run_%02d.png", i), sheet);
            std::string endq = r.end;
            for (char& c : endq) if (c == ',' || c == '"') c = ';';
            csv << i << ',' << sc.seed << ',' << sc.kind->label << ',' << sc.alt0 << ',' << sc.te << ','
                << sc.tn << ',' << r.firstSeenS << ',' << r.firstSeenRange << ',' << r.reactS << ','
                << r.closest << ',' << r.closestH << ',' << r.minAlt << ',' << r.endRange << ',' << r.endAlt << ','
                << r.fcEnd << ',' << result << ",\"" << endq << "\"\n";
        }
    }
    std::printf("[scenarios] %d of %d passed (no crash, not stopped, finished in time)%s\n", passed, o.runs,
                wantFiles ? (" -- FPV videos and contact sheets in " + o.outDir + "/").c_str() : "");
#if !KESTREL_HAVE_VIDEOIO
    if (o.video) std::printf("[scenarios] this build has no video writer: contact sheets only\n");
#endif
    return passed == o.runs ? 0 : 1;
}

}  // namespace kscen
