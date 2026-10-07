// DepthVio against ground truth: does it recover a KNOWN trajectory?
//
// A textured pillar room, rendered through navcore's own IR model (intensity a
// function of the world point, so a surface looks the same from anywhere -- the
// assumption tracking relies on, and the only honest way to test it without a
// camera) and depth (perfect, then the simulated stereo matcher). The IMU prior
// is deliberately imperfect: roll/pitch with 0.3 deg noise, yaw with a 0.5 deg/s
// gyro bias the camera has to overrule.
//
// Scored as position drift in % of path length and final yaw error. The
// assertions are the bars a VIO must clear to be worth wiring in; the printed
// numbers are the measurement.
//
// Env: VIOTEST_FULLRES=1 runs 848x480 (what flies); default 424x240.
//      VIOTEST_ONLY=<substring> runs only the cases whose name contains it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "depth_camera.hpp"
#include "vio.hpp"
#include "voxel_world.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {
constexpr float kPi = 3.14159265f;

void box(sim::VoxelWorld& w, float x0, float y0, float z0, float x1, float y1, float z1,
         float tex) {
    int ax, ay, az, bx, by, bz;
    w.worldToCell(x0, y0, z0, ax, ay, az);
    w.worldToCell(x1, y1, z1, bx, by, bz);
    for (int z = az; z <= bz; ++z)
        for (int y = ay; y <= by; ++y)
            for (int x = ax; x <= bx; ++x) { w.set(x, y, z); w.setTex(x, y, z, tex); }
}

// 20 m room: textured floor and walls, 30 pillars with bark-like texture.
void buildWorld(sim::VoxelWorld& w, unsigned seed) {
    const float S = 20.f, H = 5.f, c = 0.1f;
    w.init(c, 0.f, 0.f, 0.f, int(S / c), int(S / c), int(H / c));
    box(w, 0, 0, 0, S, S, 0.1f, 0.5f);                  // floor
    // The east wall is BLANK (no texture): the case where there is nothing to
    // track and the only acceptable answer is "lost".
    box(w, 0, 0, 0, 0.3f, S, H, 0.4f); box(w, S - 0.3f, 0, 0, S, S, H, 0.f);
    box(w, 0, 0, 0, S, 0.3f, H, 0.4f); box(w, 0, S - 0.3f, 0, S, S, H, 0.4f);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> U(2.f, 18.f);
    for (int i = 0; i < 30; ++i) {
        const float x = U(rng), y = U(rng);
        if (std::hypot(x - 10.f, y - 10.f) < 3.5f) continue;   // the flight area
        box(w, x - 0.2f, y - 0.2f, 0, x + 0.2f, y + 0.2f, H, 0.55f);
    }
}

struct Waypoint { float e, n, u, yawDeg; };

// A trajectory as a dense list of poses at dt, flown at `vel` and `yawRate`.
std::vector<Waypoint> densify(const std::vector<Waypoint>& key, float dt, float vel,
                              float yawRate) {
    std::vector<Waypoint> out;
    for (size_t i = 0; i + 1 < key.size(); ++i) {
        const Waypoint& a = key[i]; const Waypoint& b = key[i + 1];
        const float dist = std::hypot(std::hypot(b.e - a.e, b.n - a.n), b.u - a.u);
        float dy = b.yawDeg - a.yawDeg;
        const float T = std::max(dist / vel, std::fabs(dy) / yawRate);
        const int n = std::max(1, int(T / dt));
        for (int k = 0; k < n; ++k) {
            const float s = float(k) / float(n);
            out.push_back({a.e + (b.e - a.e) * s, a.n + (b.n - a.n) * s,
                           a.u + (b.u - a.u) * s, a.yawDeg + dy * s});
        }
    }
    out.push_back(key.back());
    return out;
}

struct Score { float pathM = 0, finalErr = 0, maxErr = 0, yawErr = 0, meanMs = 0;
               int lost = 0, frames = 0, keyframes = 0;
               // VALID frames whose step from the previous valid frame is
               // more than 0.25 m off the true step: a GROSS error, reported
               // as a measurement -- the class this test first caught (a
               // solve 1e14 m away, flagged valid). Error inherited from an
               // earlier loss is drift, and is not counted. The fine-grain
               // per-frame noise is printed as worstStep: ~0.1 m at 848x480
               // where the track thins, corrected over the next frames.
               int wrong = 0; float worstStep = 0; };

Score fly(const sim::VoxelWorld& w, const sim::DepthCamera& cam, bool stereo,
          const std::vector<Waypoint>& traj, float dt, float pitchDeg) {
    sim::DepthVio vio;
    vio.init(cam);
    std::mt19937 rng(11);
    std::normal_distribution<float> N(0.f, 1.f);
    Score sc;
    const float gyroBiasDps = 0.5f;
    Waypoint prev = traj.front();
    double msSum = 0;
    bool prevValid = false;
    sim::CamPose prevEst;
    Waypoint prevTruth = traj.front();
    for (size_t i = 0; i < traj.size(); ++i) {
        const Waypoint& q = traj[i];
        sim::CamPose truth;
        truth.e = q.e; truth.n = q.n; truth.u = q.u;
        truth.yawDeg = q.yawDeg; truth.pitchDeg = pitchDeg;
        if (i == 0) vio.reset(truth);                     // start pose is known
        cv::Mat ir = cam.renderIR(w, truth);
        cv::Mat d = stereo ? cam.renderStereo(w, truth, nullptr) : cam.renderTruth(w, truth);
        sim::CamPose imu;
        imu.rollDeg  = 0.3f * N(rng);
        imu.pitchDeg = pitchDeg + 0.3f * N(rng);
        imu.yawDeg   = q.yawDeg + gyroBiasDps * float(i) * dt;   // biased gyro yaw
        const sim::VioResult r = vio.step(ir, d, imu);
        msSum += r.ms;
        if (!r.valid && i > 0) ++sc.lost;
        if (r.newKeyframe) ++sc.keyframes;
        sc.pathM += std::hypot(std::hypot(q.e - prev.e, q.n - prev.n), q.u - prev.u);
        prev = q;
        const float err = std::hypot(std::hypot(r.pose.e - q.e, r.pose.n - q.n), r.pose.u - q.u);
        sc.maxErr = std::max(sc.maxErr, err);
        if (r.valid && prevValid) {
            const float se = std::hypot(std::hypot((r.pose.e - prevEst.e) - (q.e - prevTruth.e),
                                                   (r.pose.n - prevEst.n) - (q.n - prevTruth.n)),
                                        (r.pose.u - prevEst.u) - (q.u - prevTruth.u));
            if (se > 0.25f) ++sc.wrong;
            sc.worstStep = std::max(sc.worstStep, se);
        }
        prevValid = r.valid; prevEst = r.pose; prevTruth = q;
        sc.finalErr = err;
        float ye = r.pose.yawDeg - q.yawDeg;
        while (ye > 180.f) ye -= 360.f;
        while (ye <= -180.f) ye += 360.f;
        sc.yawErr = std::fabs(ye);
        ++sc.frames;
    }
    sc.meanMs = float(msSum / std::max(1, sc.frames));
    return sc;
}
}  // namespace

int main() {
    sim::VoxelWorld w;
    buildWorld(w, 5);
    sim::CamParams cp;
    cp.width = 424; cp.height = 240; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    cp.irBandLimit = true;              // no aliased texture to swim (depth_camera.hpp)
    if (std::getenv("VIOTEST_FULLRES")) { cp.width = 848; cp.height = 480; }
    const sim::DepthCamera cam(cp);

    // blank: flies at the untextured wall until it fills the view. Nothing to
    // track, so the bar there is not accuracy but HONESTY: it must say lost,
    // and never report a valid pose that is wrong. "Lost" is asserted on
    // STEREO depth only: perfect depth on a textureless surface is something
    // the D4 cannot produce (it matches these same IR images), and there the
    // tracker coasts on noise corners -- reported, and each step still has to
    // be right, but no real sensor gets there.
    // hz: the frame rate the case runs at.
    struct Case { const char* name; std::vector<Waypoint> key; float pitch;
                  float vel, yawRate; bool blank; float hz = 15.f; };
    const std::vector<Case> cases = {
        {"straight 8 m",
         {{10, 7, 1.5f, 0}, {10, 13, 1.5f, 0}, {10, 15, 1.5f, 0}}, 0.f, 1.f, 36.f, false},
        {"square 5x5, turns in place",
         {{8, 8, 1.5f, 0}, {8, 13, 1.5f, 0}, {8, 13, 1.5f, 90}, {13, 13, 1.5f, 90},
          {13, 13, 1.5f, 180}, {13, 8, 1.5f, 180}, {13, 8, 1.5f, 270},
          {8, 8, 1.5f, 270}}, -15.f, 1.f, 36.f, false},
        {"slalom with climb",
         {{8, 7, 1.2f, 20}, {10, 10, 1.6f, 40}, {12, 12, 2.0f, -10}, {10, 14, 1.6f, -40},
          {8, 12, 1.2f, 200}}, -20.f, 1.f, 36.f, false},
        // 3 m/s at 15 Hz is 0.2 m a frame, where near corners scale and warp
        // enough that LK held 17 of 139 within 2 px -- until the prediction
        // (search where the prior says) and the high-pass (lighting that moves
        // with the camera is not texture) went in. Both rates are held to the
        // same bars; 15 Hz is what VIO gets from a 30 fps strobed stream.
        {"fast 3 m/s, 90 deg/s, 15 Hz",
         {{8, 7, 1.5f, 0}, {8, 13, 1.5f, 0}, {8, 13, 1.5f, 90}, {13, 13, 1.5f, 90},
          {13, 13, 1.5f, 180}, {13, 8, 1.5f, 180}}, -15.f, 3.f, 90.f, false, 15.f},
        {"fast 3 m/s, 90 deg/s, 30 Hz",
         {{8, 7, 1.5f, 0}, {8, 13, 1.5f, 0}, {8, 13, 1.5f, 90}, {13, 13, 1.5f, 90},
          {13, 13, 1.5f, 180}, {13, 8, 1.5f, 180}}, -15.f, 3.f, 90.f, false, 30.f},
        {"blank wall, 0.8 m",
         {{12, 10, 1.5f, 90}, {18.9f, 10, 1.5f, 90}}, 0.f, 1.f, 36.f, true},
    };
    std::printf("DepthVio vs ground truth, %dx%d, IMU: 0.3 deg roll/pitch noise, "
                "0.5 deg/s yaw bias\n", cp.width, cp.height);
    for (bool stereo : {false, true}) {
        for (const Case& c : cases) {
            if (const char* only = std::getenv("VIOTEST_ONLY"))
                if (!std::strstr(c.name, only)) continue;
            const float dt = 1.f / c.hz;
            const std::vector<Waypoint> traj = densify(c.key, dt, c.vel, c.yawRate);
            const Score s = fly(w, cam, stereo, traj, dt, c.pitch);
            const float drift = 100.f * s.finalErr / std::max(0.1f, s.pathM);
            std::printf("  %-6s %-30s path %5.1f m  final err %.2f m (%.1f %%)  max %.2f m  "
                        "yaw err %.1f deg  lost %d/%d  wrong %d (worst step %.2f m)  kf %d  %.1f ms/frame\n",
                        stereo ? "stereo" : "truth", c.name, s.pathM, s.finalErr, drift,
                        s.maxErr, s.yawErr, s.lost, s.frames, s.wrong, s.worstStep, s.keyframes,
                        s.meanMs);
            CHECK(s.wrong == 0);                 // never confidently wrong
            if (c.blank) { if (stereo) CHECK(s.lost > 0); continue; }
            // The bars: under 3 % of distance on perfect depth, 6 % through
            // the stereo model; yaw held to a few degrees against a gyro that
            // drifts 0.5 deg/s; and it must almost never lose track here.
            CHECK(drift < (stereo ? 6.f : 3.f));
            CHECK(s.yawErr < 5.f);
            CHECK(s.lost <= s.frames / 20);
        }
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
