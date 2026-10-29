// The voxel layer aboard: VoxelNavModule (D435i -> voxel -> plan) and the
// mission flying its plan, with no camera and no aircraft.
//
// The module takes any FrameSource. The one used here is navcore's simulated
// camera wrapped so that it reports ONLY ATTITUDE -- exactly what the D435i
// gives the aircraft -- so what is under test is the architecture-C path that
// flies: a map per vantage at a fixed origin, integrated only while still, and
// legs certified on the straight line they fly. Ground truth is kept on the
// side, and it is what every safety assertion is checked against.
//
//   1. the vantage contract: moving publishes nothing valid, and stopping
//      again starts a NEW map
//   2. a certified leg is clear IN GROUND TRUTH, facing a wall
//   3. the mission gates: THINK waits for a mature map, a blocked plan turns
//      to look, a leg is only as long as its certificate, no forward pitch
//      until the nose is on the certified line
//   4. closed loop in a pillar field: module + mission + a lagged kinematic
//      airframe, scored on what this project scores on -- distance flown
//      without hitting anything
//
// Env: VOXTEST_WORLD=<seed> re-rolls the pillars, VOXTEST_STEREO=1 renders
// through the simulated stereo matcher instead of perfect depth,
// VOXTEST_TRACE=1 prints every leg and phase change. What this found when it
// was first run is recorded in navcore/nav_pipeline.{hpp,cpp}: legs certified
// past the marking range (0.27 m truth clearance, then STUCK), and a wall at
// grazing incidence that stereo never returned (0.03 m).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <csignal>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "fc_odometry.hpp"
#include "slam_anchor.hpp"
#include "slam_client.hpp"
#include "slam_link.hpp"
#include "frame_source.hpp"
#include "mission.hpp"
#include "voxel_nav.hpp"
#include "vio.hpp"
#include "voxel_world.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

constexpr float kPi = 3.14159265358979f;

// The D435i as the aircraft sees it: depth plus attitude, and no position.
// The truth pose is held by the test and rendered from, then withheld.
class AttitudeOnlySource : public sim::FrameSource {
public:
    AttitudeOnlySource(const sim::VoxelWorld& w, const sim::CamParams& p,
                       const sim::CamPose* truth)
        : sim_(w, p, /*truth depth*/ !std::getenv("VOXTEST_STEREO")), truth_(truth) {}
    const char* name() const override { return "sim-attitude-only"; }
    bool ok() const override { return true; }
    const sim::CamParams& params() const override { return sim_.params(); }
    const sim::DepthCamera& camera() const override { return sim_.camera(); }
    bool next(cv::Mat& depth, sim::PoseHint& hint) override {
        sim_.setPose(*truth_);
        sim::PoseHint full;
        if (!sim_.next(depth, full)) return false;
        hint.valid = true;
        hint.attitudeOnly = true;          // translation is NOT given
        hint.pose = sim::CamPose{};
        hint.pose.rollDeg  = full.pose.rollDeg;
        hint.pose.pitchDeg = full.pose.pitchDeg;
        hint.pose.yawDeg   = full.pose.yawDeg;
        ++frames;
        return true;
    }
    int frames = 0;
private:
    sim::SimFrameSource sim_;
    const sim::CamPose* truth_;
};

// The D435i with its emitter STROBING, as VoxelNavModule sees it with vio on:
// depth, the left IR image, and a camera-IMU attitude whose yaw has its OWN
// zero (a gyro integral, not a compass). Odd frames are LIT: their IR carries
// a dot pattern fixed to the camera, which a tracker must never be fed -- it
// reads as zero motion.
class StrobedSource : public sim::FrameSource {
public:
    StrobedSource(const sim::VoxelWorld& w, const sim::CamParams& p,
                  const sim::CamPose* truth, float gyroZeroDeg)
        : w_(w), cam_(p), truth_(truth), gyroZero_(gyroZeroDeg) {}
    const char* name() const override { return "sim-strobed"; }
    bool ok() const override { return true; }
    const sim::CamParams& params() const override { return cam_.params(); }
    const sim::DepthCamera& camera() const override { return cam_; }
    bool next(cv::Mat& depth, sim::PoseHint& hint) override {
        depth = cam_.renderTruth(w_, *truth_);
        ir_ = cam_.renderIR(w_, *truth_);
        lit_ = (frames % 2) == 1;
        if (lit_)                                   // the projector's dots
            for (int v = 3; v < ir_.rows; v += 7)
                for (int u = (v * 3) % 7; u < ir_.cols; u += 7) ir_.at<uchar>(v, u) = 255;
        hint.valid = true;
        hint.attitudeOnly = true;
        hint.pose = sim::CamPose{};
        hint.pose.rollDeg  = truth_->rollDeg;
        hint.pose.pitchDeg = truth_->pitchDeg;
        hint.pose.yawDeg   = truth_->yawDeg + gyroZero_;
        ++frames;
        return true;
    }
    bool intensity(cv::Mat& out) const override { out = ir_.clone(); return !ir_.empty(); }
    int  intensityEmitter() const override { return lit_ ? 1 : 0; }
protected:
    void markLit() { ir_.at<uchar>(0, 0) = 255; }
public:
    int frames = 0;
private:
    const sim::VoxelWorld& w_;
    sim::DepthCamera cam_;
    const sim::CamPose* truth_;
    float gyroZero_;
    cv::Mat ir_;
    bool lit_ = false;
};

// The same, with the RIGHT imager too (50 mm to the right), for the SLAM path.
// Lit frames also carry a marker pixel (0,0) = 255, so a fake SLAM server can
// prove it never received one.
class StereoStrobedSource : public StrobedSource {
public:
    StereoStrobedSource(const sim::VoxelWorld& w, const sim::CamParams& p,
                        const sim::CamPose* truth, float gyroZeroDeg)
        : StrobedSource(w, p, truth, gyroZeroDeg), w2_(w), cam2_(p), truth2_(truth) {}
    bool next(cv::Mat& depth, sim::PoseHint& hint) override {
        if (!StrobedSource::next(depth, hint)) return false;
        const cv::Matx33d R = SlamAnchor::rotWc(*truth2_);
        sim::CamPose r = *truth2_;
        const float b = cam2_.params().baselineM;
        r.e += float(R(0, 0) * b); r.n += float(R(1, 0) * b); r.u += float(R(2, 0) * b);
        right_ = cam2_.renderIR(w2_, r);
        if (intensityEmitter() == 1) {
            for (int v = 3; v < right_.rows; v += 7)
                for (int u = (v * 3) % 7; u < right_.cols; u += 7) right_.at<uchar>(v, u) = 255;
            markLit();
            right_.at<uchar>(0, 0) = 255;
        }
        t_ = double(frames) / 30.0;
        return true;
    }
    bool   intensityRight(cv::Mat& out) const override { out = right_.clone(); return true; }
    double intensityTimeS() const override { return t_; }
private:
    const sim::VoxelWorld& w2_;
    sim::DepthCamera cam2_;
    const sim::CamPose* truth2_;
    cv::Mat right_;
    double t_ = 0;
};

sim::CamParams d435i() {
    // The D435i's depth geometry: 87 deg horizontal, 50 mm baseline. 424x240
    // rather than 848x480 so the suite stays quick; fineMapParams derives the
    // honest range from THIS camera, so the map is configured for it.
    sim::CamParams p;
    p.width = 424; p.height = 240; p.hfovDeg = 87.f; p.baselineM = 0.05f;
    p.irBandLimit = true;   // the IR VIO tracks: band-limited (depth_camera.hpp)
    // VOXTEST_FULLRES=1: the resolution onboard actually runs (848x480), which
    // halves stereo error at range and lengthens the honest marking range.
    if (std::getenv("VOXTEST_FULLRES")) { p.width = 848; p.height = 480; }
    return p;
}

// SURFACE TEXTURE, which the stereo model matches on. The first version of
// this test set none: every surface was texture 0, below the matcher's 0.25
// threshold, so under VOXTEST_STEREO the camera saw nothing but silhouette
// edges -- the most pessimistic world there is, and every stereo number it
// produced was measured in it. Values follow navcore's own worlds: plaster
// walls 0.35 (voxel_world.hpp says why not 0.10), ground and bark richer.
constexpr float kGroundTex = 0.5f, kWallTex = 0.35f, kPillarTex = 0.55f;

// Ground slab + perimeter walls: returns everywhere, so free space can be
// carved. Unknown is not free, and an empty world would be all unknown.
void buildRoom(sim::VoxelWorld& w, float sizeM, float heightM, float cell) {
    const int n = int(sizeM / cell), nz = int(heightM / cell);
    w.init(cell, 0.f, 0.f, 0.f, n, n, nz);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            w.set(x, y, 0); w.setTex(x, y, 0, kGroundTex);
            if (x < 2 || y < 2 || x >= n - 2 || y >= n - 2)
                for (int z = 0; z < nz; ++z) { w.set(x, y, z); w.setTex(x, y, z, kWallTex); }
        }
}

void box(sim::VoxelWorld& w, float x0, float y0, float x1, float y1, float h,
         float tex) {
    int ax, ay, az, bx, by, bz;
    w.worldToCell(x0, y0, 0.f, ax, ay, az);
    w.worldToCell(x1, y1, h, bx, by, bz);
    for (int z = 0; z <= bz; ++z)
        for (int y = ay; y <= by; ++y)
            for (int x = ax; x <= bx; ++x) { w.set(x, y, z); w.setTex(x, y, z, tex); }
}

// Distance from a point to the nearest solid voxel (box, not centre), up to
// `upTo`. Ground truth -- the map never sees this.
float clearance(const sim::VoxelWorld& w, float px, float py, float pz, float upTo) {
    const float c = w.cell();
    int cx, cy, cz; w.worldToCell(px, py, pz, cx, cy, cz);
    const int r = int(std::ceil(upTo / c)) + 1;
    float best = upTo;
    for (int z = cz - r; z <= cz + r; ++z)
        for (int y = cy - r; y <= cy + r; ++y)
            for (int x = cx - r; x <= cx + r; ++x) {
                if (!w.solid(x, y, z)) continue;
                const float bx0 = w.ox() + x * c, by0 = w.oy() + y * c, bz0 = w.oz() + z * c;
                const float dx = std::max({bx0 - px, 0.f, px - (bx0 + c)});
                const float dy = std::max({by0 - py, 0.f, py - (by0 + c)});
                const float dz = std::max({bz0 - pz, 0.f, pz - (bz0 + c)});
                best = std::min(best, std::sqrt(dx*dx + dy*dy + dz*dz));
            }
    return best;
}

// Worst truth clearance along a straight, level segment.
float segmentClearance(const sim::VoxelWorld& w, float e, float n, float u,
                       float azDeg, float len, float upTo) {
    const float a = azDeg * kPi / 180.f;
    float worst = upTo;
    for (float t = 0.f; t <= len; t += 0.05f)
        worst = std::min(worst, clearance(w, e + std::sin(a) * t,
                                          n + std::cos(a) * t, u, upTo));
    return worst;
}

WorldState stillState() {
    WorldState s;
    s.tickMonoS = monoNowS();
    s.vehGroundspeed = 0.f;
    return s;
}

}  // namespace

int main() {
    const sim::CamParams cp = d435i();
    VoxelNavModule::Params vp;
    vp.legMaxM = 6.f;

    // ---------------------------------------------------------------- 1
    std::printf("vantage contract\n");
    {
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        sim::CamPose truth; truth.e = 8.f; truth.n = 4.f; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);
        CHECK(mod.isReady());
        WorldModel wm;
        wm.with([](WorldState& s) { s = stillState(); });
        for (int i = 0; i < 4; ++i) mod.run(cv::Mat(), wm);
        WorldState s = wm.snapshot();
        CHECK(s.voxValid && !s.voxMoving);
        CHECK(s.voxFrames == 4);
        CHECK(mod.resets() == 1);

        wm.with([](WorldState& s) { s.vehGroundspeed = 1.2f; });   // flying
        mod.run(cv::Mat(), wm);
        s = wm.snapshot();
        CHECK(!s.voxValid && s.voxMoving && s.voxFrames == 0);

        wm.with([](WorldState& s) { s.vehGroundspeed = 0.f; });    // stopped
        mod.run(cv::Mat(), wm);
        s = wm.snapshot();
        CHECK(s.voxValid && s.voxFrames == 1);   // a NEW map, not the old one
        CHECK(mod.resets() == 2);

        // Under a mission, SETTLE is not a vantage even at zero speed: that is
        // where the last leg's drift is still bleeding off.
        wm.with([](WorldState& s) { s.missionActive = true; s.missionPhase = "SETTLE"; });
        mod.run(cv::Mat(), wm);
        CHECK(!wm.snapshot().voxValid);
        wm.with([](WorldState& s) { s.missionPhase = "THINK"; });
        mod.run(cv::Mat(), wm);
        CHECK(wm.snapshot().voxValid && wm.snapshot().voxFrames == 1);
        wm.with([](WorldState& s) { s.missionPhase = "SCAN"; });   // rotation:
        mod.run(cv::Mat(), wm);                                   // same map
        CHECK(wm.snapshot().voxFrames == 2);
        std::printf("  ok: moving -> invalid; still again -> new map; SETTLE excluded\n");
    }

    // ---------------------------------------------------------------- 2
    std::printf("a certified leg is clear in ground truth\n");
    {
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        // A wall 2 m ahead, 3 m wide, centred on the nose. Open either side.
        box(w, 6.5f, 6.0f, 9.5f, 6.4f, 4.f, kWallTex);
        sim::CamPose truth; truth.e = 8.f; truth.n = 4.f; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);
        WorldModel wm;
        wm.with([](WorldState& s) { s = stillState(); });
        for (int i = 0; i < 8; ++i) mod.run(cv::Mat(), wm);
        const WorldState s = wm.snapshot();
        std::printf("  planner %.1f deg (freeM %.2f, blocked %d)  leg %.1f deg "
                    "certified %.2f m\n", s.voxBearingDeg, s.voxFreeM,
                    int(s.voxBlocked), s.voxLegBearingDeg, s.voxLegFreeM);
        CHECK(s.voxValid);
        // Straight ahead is the wall: whatever it chose, its certificate must
        // not reach through it.
        const float ahead = mod.pipeline().straightFreeM(sim::CamPose{}, 0.f, 6.f);
        std::printf("  straight ahead certified %.2f m (wall face at 2.0 m)\n", ahead);
        CHECK(ahead < 2.0f - 0.3f);
        if (!std::getenv("VOXTEST_STEREO"))
            CHECK(s.voxLegFreeM > 1.f);       // there IS a way round it
        if (s.voxLegFreeM > 0.f) {
            const float worst = segmentClearance(w, truth.e, truth.n, truth.u,
                                                 s.voxLegBearingDeg, s.voxLegFreeM, 2.f);
            std::printf("  truth clearance along the certified leg: %.2f m\n", worst);
            CHECK(worst >= 0.3f);
        }
    }

    // ---------------------------------------------------------------- 2b
    std::printf("proximity from one frame (OBSTACLE_DISTANCE shape)\n");
    {
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        box(w, 6.5f, 6.0f, 9.5f, 6.4f, 4.f, kWallTex);          // wall 2 m ahead
        // A pillar 30 deg right of the nose, 3 m out -- behind the wall's end.
        const float pe = 8.f + 3.f * std::sin(30.f * kPi / 180.f);
        const float pn = 4.f + 3.f * std::cos(30.f * kPi / 180.f);
        box(w, pe - 0.15f, pn - 0.15f, pe + 0.15f, pn + 0.15f, 4.f, kPillarTex);
        sim::SimFrameSource cam(w, cp, true);
        sim::CamPose at; at.e = 8.f; at.n = 4.f; at.u = 1.5f;
        cam.setPose(at);
        cv::Mat d; sim::PoseHint h;
        cam.next(d, h);
        std::vector<float> px = sim::obstacleDistanceFromFrame(d, cam.camera(), at);
        std::printf("  nose %.2f m   30 deg right %.2f m   astern %.2f\n",
                    px[0], px[6], px[36]);
        CHECK(std::fabs(px[0] - 2.0f) < 0.2f);       // the wall face, horizontal
        // 30 deg right the WALL is still in front of the pillar: its face is
        // 2 / cos(30) = 2.31 m away, and the nearest surface is what counts.
        CHECK(std::fabs(px[6] - 2.31f) < 0.15f);
        CHECK(px[36] < 0.f);                         // behind: UNKNOWN, not clear
        CHECK(px[18] < 0.f && px[54] < 0.f);         // abeam: outside the FoV
        // One speckle pixel at 0.5 m in an otherwise empty frame is not an obstacle.
        cv::Mat e(d.size(), CV_32F, cv::Scalar(-1.f));
        e.at<float>(e.rows / 2, e.cols / 2) = 0.5f;
        std::vector<float> ps = sim::obstacleDistanceFromFrame(e, cam.camera(), at);
        CHECK(ps[0] < 0.f);
        // Tilted 20 deg down, the SAME wall still reads as the same distance
        // (only level returns count, measured horizontally).
        sim::CamPose tilt = at; tilt.pitchDeg = -20.f;
        cam.setPose(tilt); cam.next(d, h);
        std::vector<float> pt = sim::obstacleDistanceFromFrame(d, cam.camera(), tilt);
        std::printf("  tilted -20: nose %.2f m\n", pt[0]);
        CHECK(std::fabs(pt[0] - 2.0f) < 0.25f);
    }

    // ---------------------------------------------------------------- 2c
    std::printf("VIO wiring: strobed emitter, gyro yaw with its own zero\n");
    {
        // The module's own path, not DepthVio called directly: lit frames
        // skipped, attitude plumbed from the camera IMU, the VIO frame's
        // heading taken from the FC compass, and the estimate read back the
        // way main reads it (vioLocalEstimate).
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        std::mt19937 prng(3);
        std::uniform_real_distribution<float> PU(2.f, 14.f);
        for (int i = 0; i < 25; ++i) {
            const float x = PU(prng), y = PU(prng);
            if (std::fabs(x - 5.5f) < 1.5f && y > 3.f && y < 11.f) continue;  // the path
            box(w, x - 0.2f, y - 0.2f, x + 0.2f, y + 0.2f, 4.f, kPillarTex);
        }
        const float hdg = 25.f;                      // flying 25 deg east of North
        sim::CamPose truth; truth.e = 4.f; truth.n = 4.f; truth.u = 1.5f;
        truth.yawDeg = hdg; truth.pitchDeg = -15.f;
        auto* src = new StrobedSource(w, cp, &truth, /*gyro zero*/ 137.f);
        VoxelNavModule::Params vq = vp;
        vq.vio = true;
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vq);
        WorldModel wm;
        const float dt = 1.f / 30.f, v = 1.f;
        const float se = std::sin(hdg * kPi / 180.f), cn = std::cos(hdg * kPi / 180.f);
        int valid = 0, lit = 0;
        for (int i = 0; i < 180; ++i) {              // 6 s, 6 m, 30 fps strobed
            wm.with([&](WorldState& s) {
                s.tickMonoS = monoNowS(); s.vehYawDeg = hdg;   // the FC compass
                s.vehGroundspeed = v; s.vehAltM = truth.u;
            });
            mod.run(cv::Mat(), wm);
            if (src->intensityEmitter() == 1) ++lit;
            else if (wm.snapshot().vioValid) ++valid;
            truth.e += se * v * dt; truth.n += cn * v * dt;
        }
        WorldState s = wm.snapshot();
        const float de = s.vioPe - (truth.e - se * v * dt - 4.f);
        const float dn = s.vioPn - (truth.n - cn * v * dt - 4.f);
        const float path = 179.f * v * dt;
        std::printf("  %d dark frames valid of %d, %d lit skipped; VIO (%.2f, %.2f) vs "
                    "truth (%.2f, %.2f): error %.2f m over %.1f m, yaw %.1f\n",
                    valid, 180 - lit, lit, s.vioPe, s.vioPn, truth.e - se * v * dt - 4.f,
                    truth.n - cn * v * dt - 4.f, std::hypot(de, dn), path, s.vioYawDeg);
        CHECK(lit == 90);
        CHECK(valid >= 85);                          // the dark half, all but a start
        CHECK(std::hypot(de, dn) < 0.05f * path);    // under 5 % with the dots present
        float yawErr = s.vioYawDeg - hdg;
        while (yawErr > 180.f) yawErr -= 360.f;
        while (yawErr <= -180.f) yawErr += 360.f;
        CHECK(std::fabs(yawErr) < 3.f);              // compass frame, not gyro zero
        s.tickMonoS = monoNowS();
        CHECK(vioLocalEstimate(monoNowS(), s) && s.estValid);
        CHECK(std::fabs(s.estPn - s.vioPn) < 1e-6f);   // (speed is wall-clock: not checked)
    }

    // ---------------------------------------------------------------- 2d
    std::printf("SLAM wiring: fake server, arbitrary SLAM frame, a map change, a silent jump\n");
    {
        // Everything between the module and a SLAM process, without the SLAM:
        // the server answers each frame with the TRUE pose expressed in an
        // arbitrary SLAM world (yawed, tilted, offset), at frame 150 starts a
        // NEW map with a different one, and at frame 220 shifts that frame 5 m
        // without saying so. The module must never send a lit frame, must land
        // in ENU, and must stay continuous across both.
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        std::mt19937 prng(3);
        std::uniform_real_distribution<float> PU(2.f, 14.f);
        for (int i = 0; i < 25; ++i) {
            const float x = PU(prng), y = PU(prng);
            if (std::fabs(x - 5.5f) < 1.5f && y > 3.f && y < 11.f) continue;
            box(w, x - 0.2f, y - 0.2f, x + 0.2f, y + 0.2f, 4.f, kPillarTex);
        }
        const std::string sock = "/tmp/kestrel-voxtest-" + std::to_string(::getpid()) + ".sock";
        std::mutex tmu;
        std::map<long, sim::CamPose> truthAt;          // frame time (ms) -> truth
        std::atomic<int> litSeen{0}, served{0};
        const int lfd = slamlink::listenUnix(sock);
        CHECK(lfd >= 0);
        std::atomic<int> cfd{-1};
        std::thread srv([&] {
            const int fd = ::accept(lfd, nullptr, nullptr);
            if (fd < 0) return;
            cfd.store(fd);
            slamlink::FrameHeader h;
            std::vector<uint8_t> l, r;
            std::vector<slamlink::ImuSample> imu;
            // Two arbitrary SLAM worlds: ENU -> SLAM.
            sim::CamPose f0; f0.yawDeg = 70.f; f0.pitchDeg = 12.f; f0.e = 3.f; f0.n = -2.f; f0.u = 0.5f;
            sim::CamPose f1; f1.yawDeg = -35.f; f1.rollDeg = 5.f; f1.e = -7.f; f1.n = 4.f;
            while (slamlink::recvFrame(fd, h, l, r, imu)) {
                if (l[0] == 255 || r[0] == 255) litSeen.fetch_add(1);
                sim::CamPose q;
                {
                    std::lock_guard<std::mutex> lk(tmu);
                    q = truthAt[long(h.tS * 1000.0 + 0.5)];
                }
                const int k = served.fetch_add(1);
                // ~frame 150: a NEW map. ~frame 220: the frame silently shifts
                // 5 m with NO new map id -- what the jump guard is for.
                sim::CamPose F = k < 75 ? f0 : f1;
                if (k >= 110) { F.e += 5.f; F.n -= 1.f; }
                const cv::Matx33d Rsw = SlamAnchor::rotWc(F).t();
                const cv::Vec3d tsw(F.e, F.n, F.u);
                const cv::Matx33d Rwc = SlamAnchor::rotWc(q);
                const cv::Matx33d Rsc = Rsw * Rwc;
                const cv::Vec3d p = Rsw * (cv::Vec3d(q.e, q.n, q.u) - tsw);
                slamlink::PoseReply rep;
                rep.seq = h.seq; rep.state = slamlink::kOk; rep.mapId = k < 75 ? 0 : 1;
                rep.tracked = 300;
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) rep.Twc[i * 4 + j] = float(Rsc(i, j));
                    rep.Twc[i * 4 + 3] = float(p[i]);
                }
                if (!slamlink::sendPose(fd, rep)) break;
            }
            ::close(fd);
        });
        const float hdg = 25.f;
        sim::CamPose truth; truth.e = 4.f; truth.n = 4.f; truth.u = 1.5f;
        truth.yawDeg = hdg; truth.pitchDeg = -15.f;
        auto* src = new StereoStrobedSource(w, cp, &truth, 137.f);
        VoxelNavModule::Params vq = vp;
        vq.slam = true; vq.slamSocket = sock;
        auto modp = std::make_unique<VoxelNavModule>(std::unique_ptr<sim::FrameSource>(src), vq);
        VoxelNavModule& mod = *modp;
        WorldModel wm;
        const float dt = 1.f / 30.f, v = 1.f;
        const float se = std::sin(hdg * kPi / 180.f), cn = std::cos(hdg * kPi / 180.f);
        float worst = 0.f;
        bool have0 = false;
        float v0e = 0, v0n = 0, t0e = 0, t0n = 0;       // at the first valid pose
        for (int i = 0; i < 300; ++i) {                 // 10 s, 10 m, 30 fps strobed
            {
                std::lock_guard<std::mutex> lk(tmu);
                truthAt[long(double(i + 1) / 30.0 * 1000.0 + 0.5)] = truth;
            }
            wm.with([&](WorldState& s) {
                s.tickMonoS = monoNowS(); s.vehYawDeg = hdg;
                s.vehGroundspeed = v; s.vehAltM = truth.u;
            });
            mod.run(cv::Mat(), wm);
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            const WorldState s = wm.snapshot();
            if (s.vioValid) {
                // DISPLACEMENT since the first tracked pose (that pose is the
                // origin by definition). The reply read now answers a frame
                // sent a run or two ago: ~3-7 cm of lag at 1 m/s is in this.
                if (!have0) { have0 = true; v0e = s.vioPe; v0n = s.vioPn;
                              t0e = truth.e; t0n = truth.n; }
                const float err = std::hypot((s.vioPe - v0e) - (truth.e - t0e),
                                             (s.vioPn - v0n) - (truth.n - t0n));
                worst = std::max(worst, err);
            }
            truth.e += se * v * dt; truth.n += cn * v * dt;
        }
        const WorldState s = wm.snapshot();
        std::printf("  %d frames served, %d lit ever sent; moved (%.2f, %.2f) vs truth "
                    "(%.2f, %.2f); worst lag+error %.2f m; yaw %.1f; resets %d\n",
                    served.load(), litSeen.load(), s.vioPe - v0e, s.vioPn - v0n,
                    truth.e - t0e, truth.n - t0n, worst, s.vioYawDeg, s.vioResets);
        CHECK(litSeen.load() == 0);                       // never a dotted frame
        CHECK(served.load() >= 140);                      // the dark half, near enough
        CHECK(worst < 0.15f);                             // ~1-2 frames of lag, no jump
        CHECK(std::fabs(s.vioYawDeg - hdg) < 1.f);        // compass frame
        CHECK(s.vioResets >= 2);                          // the map change AND the jump
        modp.reset();                                     // closes the client side
        const int c = cfd.load();
        if (c >= 0) ::shutdown(c, SHUT_RDWR);
        ::shutdown(lfd, SHUT_RDWR); ::close(lfd);
        srv.join();
        ::unlink(sock.c_str());
    }

    // ---------------------------------------------------------------- 2e
    std::printf("SLAM wiring: the bridge dies and a supervisor restarts it\n");
    {
        // A restarted kestrel-orbslam starts a fresh map: id 0 again, origin
        // wherever the camera is when it comes up, its own yaw. Same id as the
        // map it replaces, different frame. After the vocabulary load (here
        // 2.5 s; 5-30 s for real) the jump guard would allow 4 m/s x that --
        // so the ONLY thing that can tell the estimate is the reconnect.
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        std::mt19937 prng(5);
        std::uniform_real_distribution<float> PU(2.f, 14.f);
        for (int i = 0; i < 25; ++i) {
            const float x = PU(prng), y = PU(prng);
            if (std::fabs(x - 5.5f) < 1.5f && y > 3.f && y < 11.f) continue;
            box(w, x - 0.2f, y - 0.2f, x + 0.2f, y + 0.2f, 4.f, kPillarTex);
        }
        const std::string sock = "/tmp/kestrel-voxtest-r" + std::to_string(::getpid()) + ".sock";
        std::mutex tmu;
        std::map<long, sim::CamPose> truthAt;
        std::atomic<bool> down{false};                 // the bridge is restarting
        std::atomic<int> served{0};
        const int lfd = slamlink::listenUnix(sock);
        CHECK(lfd >= 0);
        std::atomic<int> cfd{-1};
        std::thread srv([&] {
            // Process 1: frame f0. Process 2: frame at the camera's pose when
            // it came up, yawed 110 deg, first 5 frames still initialising.
            sim::CamPose F; F.yawDeg = 70.f; F.pitchDeg = 12.f; F.e = 3.f; F.n = -2.f; F.u = 0.5f;
            for (int proc = 0; proc < 2; ++proc) {
                const int fd = ::accept(lfd, nullptr, nullptr);
                if (fd < 0) return;
                cfd.store(fd);
                slamlink::FrameHeader h;
                std::vector<uint8_t> l, r;
                std::vector<slamlink::ImuSample> imu;
                int k = 0;
                while (slamlink::recvFrame(fd, h, l, r, imu)) {
                    sim::CamPose q;
                    {
                        std::lock_guard<std::mutex> lk(tmu);
                        q = truthAt[long(h.tS * 1000.0 + 0.5)];
                    }
                    if (proc == 1 && k == 0) {
                        F = sim::CamPose(); F.e = q.e; F.n = q.n; F.u = q.u; F.yawDeg = q.yawDeg + 110.f;
                    }
                    slamlink::PoseReply rep;
                    rep.seq = h.seq; rep.mapId = 0; rep.tracked = 300;
                    rep.state = proc == 1 && k < 5 ? slamlink::kNotInitialized : slamlink::kOk;
                    const cv::Matx33d Rsw = SlamAnchor::rotWc(F).t();
                    const cv::Matx33d Rsc = Rsw * SlamAnchor::rotWc(q);
                    const cv::Vec3d p = Rsw * (cv::Vec3d(q.e, q.n, q.u) - cv::Vec3d(F.e, F.n, F.u));
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) rep.Twc[i * 4 + j] = float(Rsc(i, j));
                        rep.Twc[i * 4 + 3] = float(p[i]);
                    }
                    ++k;
                    served.fetch_add(1);
                    if (!slamlink::sendPose(fd, rep)) break;
                    if (proc == 0 && k == 20) break;       // the crash
                }
                ::close(fd);
                cfd.store(-1);
                if (proc == 0) {
                    down.store(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
                    down.store(false);
                }
            }
        });
        const float hdg = -40.f;
        sim::CamPose truth; truth.e = 10.f; truth.n = 4.f; truth.u = 1.5f;
        truth.yawDeg = hdg; truth.pitchDeg = -15.f;
        auto* src = new StereoStrobedSource(w, cp, &truth, 137.f);
        VoxelNavModule::Params vq = vp;
        vq.slam = true; vq.slamSocket = sock;
        auto modp = std::make_unique<VoxelNavModule>(std::unique_ptr<sim::FrameSource>(src), vq);
        VoxelNavModule& mod = *modp;
        WorldModel wm;
        const float dt = 1.f / 30.f, v = 1.f;
        const float se = std::sin(hdg * kPi / 180.f), cn = std::cos(hdg * kPi / 180.f);
        // The gap is wall time, and so is the velocity a re-anchor carries
        // the estimate on: both come from the module's clock, whatever speed
        // this machine renders at. Ends 20 fresh poses after the restart.
        bool have0 = false, sawDown = false, havePost = false;
        float v0e = 0, v0n = 0, t0e = 0, t0n = 0, preE = 0, preN = 0, step = 0.f;
        float worstPre = 0.f, worstPost = 0.f;
        float p0e = 0, p0n = 0, pt0e = 0, pt0n = 0;
        int resetsPre = 0;
        double lastStamp = -1e9;
        const auto t0 = std::chrono::steady_clock::now();
        int nPost = 0;
        for (int i = 0; i < 3000 && nPost < 20 && std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - t0).count() < 120.0; ++i) {
            {
                std::lock_guard<std::mutex> lk(tmu);
                truthAt[long(double(i + 1) / 30.0 * 1000.0 + 0.5)] = truth;
            }
            wm.with([&](WorldState& s) {
                s.tickMonoS = monoNowS(); s.vehYawDeg = hdg;
                s.vehGroundspeed = down.load() ? 0.f : v; s.vehAltM = truth.u;
            });
            mod.run(cv::Mat(), wm);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            const WorldState s = wm.snapshot();
            if (down.load()) sawDown = true;
            const bool fresh = s.vioValid && s.vioStampS != lastStamp;
            lastStamp = s.vioStampS;
            if (fresh) {
                if (!have0) { have0 = true; v0e = s.vioPe; v0n = s.vioPn; t0e = truth.e; t0n = truth.n; }
                if (!sawDown) {
                    worstPre = std::max(worstPre, std::hypot((s.vioPe - v0e) - (truth.e - t0e),
                                                             (s.vioPn - v0n) - (truth.n - t0n)));
                    preE = s.vioPe; preN = s.vioPn; resetsPre = s.vioResets;
                } else if (!down.load()) {
                    if (!havePost) {
                        havePost = true;
                        step = std::hypot(s.vioPe - preE, s.vioPn - preN);
                        p0e = s.vioPe; p0n = s.vioPn; pt0e = truth.e; pt0n = truth.n;
                    }
                    ++nPost;
                    worstPost = std::max(worstPost, std::hypot((s.vioPe - p0e) - (truth.e - pt0e),
                                                               (s.vioPn - p0n) - (truth.n - pt0n)));
                }
            }
            // The mission stops while the estimate is gone: hold position.
            if (!down.load()) { truth.e += se * v * dt; truth.n += cn * v * dt; }
        }
        const WorldState s = wm.snapshot();
        std::printf("  %d frames served over two processes; tracked to %.2f m before the "
                    "crash, %.2f m after; step across the restart %.2f m; resets %d -> %d\n",
                    served.load(), worstPre, worstPost, step, resetsPre, s.vioResets);
        CHECK(sawDown && havePost);
        CHECK(worstPre < 0.15f);
        // Across the gap the aircraft held still; the re-anchor carries the
        // estimate at most 0.5 s on the last velocity (~1 m/s).
        CHECK(step < 0.7f);
        CHECK(worstPost < 0.15f);
        CHECK(s.vioResets > resetsPre);                   // the consumer is told
        CHECK(std::fabs(s.vioYawDeg - hdg) < 1.f);
        modp.reset();
        const int c = cfd.load();
        if (c >= 0) ::shutdown(c, SHUT_RDWR);
        ::shutdown(lfd, SHUT_RDWR); ::close(lfd);
        srv.join();
        ::unlink(sock.c_str());
    }

    // ---------------------------------------------------------------- 3
    std::printf("mission gates on the voxel plan\n");
    {
        MissionController::Params mp;
        mp.useVoxel = true; mp.useMap = false; mp.settleSec = 0.2f;
        MissionController m(mp);
        m.enable(true);
        WorldState s;
        s.estValid = true; s.missionGo = true; s.tickMonoS = 50.0;
        auto fresh = [&](int frames, bool blocked, float legFree, float brg) {
            s.voxValid = true; s.voxStampS = s.tickMonoS; s.voxFrames = frames;
            s.voxBlocked = blocked; s.voxLegFreeM = legFree; s.voxLegBearingDeg = brg;
        };
        for (int i = 0; i < 5; ++i) m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);

        fresh(2, false, 3.f, 0.f);                  // young map: keep hovering
        ControlCmd c = m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);
        CHECK(c.pitch == 0.f && c.yaw == 0.f);

        fresh(8, true, 0.f, 0.f);                   // mature, nothing: look round
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::SCAN);
        c = m.update(s, 0.05f);
        CHECK(c.yaw != 0.f && c.pitch == 0.f);      // yaw only, no translation

        fresh(9, false, 2.5f, 40.f);                // an opening mid-sweep
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::MOVE);
        // Leg = certificate - stop margin, NOT stepM: 2.5 - 0.5 = 2.0 m.
        const float wpDist = std::hypot(s.missionWpE, s.missionWpN);
        std::printf("  leg %.2f m on bearing 40 (stepM %.1f)\n", wpDist, mp.stepM);
        CHECK(std::fabs(wpDist - 2.0f) < 0.01f);

        s.vehYawDeg = 0.f;                          // nose 40 deg off the line
        c = m.update(s, 0.05f);
        CHECK(c.yaw > 0.f && c.pitch == 0.f);       // turn first, do not fly
        s.vehYawDeg = 36.f;
        c = m.update(s, 0.05f);
        CHECK(c.pitch > 0.f);                       // on the line: go

        s.estPe = 2.0f * std::sin(40.f * kPi / 180.f);
        s.estPn = 2.0f * std::cos(40.f * kPi / 180.f);
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::ARRIVE);

        // A dead module: THINK gives it the stale window, then re-settles.
        MissionController m2(mp); m2.enable(true);
        WorldState s2; s2.estValid = true; s2.missionGo = true; s2.tickMonoS = 50.0;
        for (int i = 0; i < 5; ++i) m2.update(s2, 0.05f);
        CHECK(m2.phase() == MissionController::Phase::THINK);
        m2.update(s2, 0.05f);
        CHECK(m2.phase() == MissionController::Phase::THINK);   // not yet
        for (int i = 0; i < 20; ++i) m2.update(s2, 0.05f);
        CHECK(m2.phase() != MissionController::Phase::MOVE);
        std::printf("  ok\n");
    }

    // ---------------------------------------------------------------- 4
    std::printf("closed loop: pillar field, attitude only\n");
    {
        // VOXTEST_WORLD=<n> re-rolls the obstacles; VOXTEST_STEREO=1 swaps the
        // perfect depth for the simulated stereo matcher (holes, speckle,
        // range-dependent noise); VOXTEST_BIG=1 is a 48 m field with walls
        // that make dead ends, where seeing past the near map should matter;
        // VOXTEST_FAR=1 turns on the far tier's choice among safe legs (off
        // by default: measured worse, voxel_nav.hpp). CI runs the defaults.
        const bool big = std::getenv("VOXTEST_BIG") != nullptr;
        if (const char* f = std::getenv("VOXTEST_FAR")) vp.farChoose = std::atoi(f) != 0;
        const float sizeM = big ? 48.f : 24.f;
        const float spawnE = sizeM * 0.5f, spawnN = 4.f;
        sim::VoxelWorld w; buildRoom(w, sizeM, 5.f, 0.2f);
        const char* ws = std::getenv("VOXTEST_WORLD");
        std::mt19937 rng(ws ? unsigned(std::atoi(ws)) : 7u);
        std::uniform_real_distribution<float> U(3.f, sizeM - 3.f);
        int pillars = 0, walls = 0;
        const int wantPillars = big ? 45 : 22, wantWalls = big ? 10 : 0;
        for (int i = 0; i < 400 && walls < wantWalls; ++i) {
            // 6 m walls, either axis: the shapes that make a dead end, which a
            // 2.5 m near map walks into and a 20 m far tier can see.
            const float x = U(rng), y = U(rng);
            const bool alongE = (rng() & 1u) != 0;
            const float x1 = alongE ? x + 6.f : x + 0.4f, y1 = alongE ? y + 0.4f : y + 6.f;
            if (x1 > sizeM - 1.f || y1 > sizeM - 1.f) continue;
            const float cx = std::max(x, std::min(spawnE, x1));
            const float cy = std::max(y, std::min(spawnN, y1));
            if (std::hypot(cx - spawnE, cy - spawnN) < 3.f) continue;   // spawn clear
            box(w, x, y, x1, y1, 5.f, kWallTex);
            ++walls;
        }
        for (int i = 0; i < 400 && pillars < wantPillars; ++i) {
            const float x = U(rng), y = U(rng);
            if (std::hypot(x - spawnE, y - spawnN) < 2.5f) continue;   // spawn clear
            box(w, x - 0.2f, y - 0.2f, x + 0.2f, y + 0.2f, 5.f, kPillarTex);
            ++pillars;
        }
        // HOVER DRIFT (VOXTEST_DRIFT=<m/s>): architecture C maps while
        // "still", but GPS-denied position hold is not still -- without an
        // optical-flow sensor the FC does not even know it is drifting. An
        // Ornstein-Uhlenbeck velocity (1-sigma per axis, 2 s correlation) is
        // added to the truth position at all times and is NOT in the ground
        // speed the module sees.
        const char* dEnv = std::getenv("VOXTEST_DRIFT");
        const float driftSig = dEnv ? float(std::atof(dEnv)) : 0.f;
        // ODOMETRY (VOXTEST_ODOM=<fraction>): architecture B. The module keeps
        // ONE map, placed by an estimate that drifts by this fraction of the
        // distance moved (VIO-class ~1-3 %, flow ~1-3 %; dead reckoning far
        // worse). VOXTEST_ODOM_MOVING=1 also maps during legs.
        const char* oEnv = std::getenv("VOXTEST_ODOM");
        // VOXTEST_VIO=1: architecture B with the estimate from navcore's
        // DepthVio, fed rendered IR + depth and a noisy, gyro-biased IMU --
        // the real odometer instead of a model of one. Its error is whatever
        // it turns out to be.
        const bool useVio = std::getenv("VOXTEST_VIO") != nullptr;
        // VOXTEST_SLAM=<kestrel-orbslam> VOXTEST_ORBVOC=<ORBvoc.txt>: the same,
        // with ORB-SLAM3 (the real bridge process) on a rendered stereo IR
        // pair as the estimator.
        const char* slamBin = std::getenv("VOXTEST_SLAM");
        const char* slamVoc = std::getenv("VOXTEST_ORBVOC");
        const bool useSlam = slamBin && slamVoc;
        const float odomFrac = (useVio || useSlam) ? 0.f
                             : (oEnv ? float(std::atof(oEnv)) : -1.f);
        if (odomFrac >= 0.f) {
            vp.persistMap = true;
            vp.integrateMoving = std::getenv("VOXTEST_ODOM_MOVING") != nullptr;
        }
        std::mt19937 nrng(ws ? unsigned(std::atoi(ws)) * 31u + 5u : 222u);
        std::normal_distribution<float> N01(0.f, 1.f);
        float dvE = 0.f, dvN = 0.f, errE = 0.f, errN = 0.f;
        float biasDir = 6.2831853f * float(nrng() % 1000) / 1000.f;
        sim::CamPose truth; truth.e = spawnE; truth.n = spawnN; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        sim::DepthCamera vioCam(cp);
        sim::DepthVio vio;
        vio.init(vioCam);
        vio.reset(truth);                        // the start pose is known
        int vioFrames = 0, vioLost = 0;
        double vioMs = 0.0;
        // SLAM sees the IR pair at the resolution that flies (848x480 --
        // onboard's --voxel-width default), not the suite's 424x240 depth:
        // ORB-SLAM3 needs more than 500 features with depth to (re)start a
        // map, which 424x240 rarely has. VOXTEST_SLAMRES=424 to compare.
        sim::CamParams slamCp = cp;
        {
            const char* r = std::getenv("VOXTEST_SLAMRES");
            const int wpx = r ? std::atoi(r) : 848;
            slamCp.width = wpx; slamCp.height = wpx * 480 / 848;
        }
        sim::DepthCamera slamCam(slamCp);
        pid_t slamPid = -1;
        std::unique_ptr<SlamClient> slamCli;
        SlamAnchor slamAnchor;
        int32_t slamMap = -1, slamPrevState = -99;
        uint32_t slamPrevChanges = 0;
        bool slamHaveLast = false;
        float slamLastT = 0.f;
        int slamJumps = 0;
        int slamMaps = 0;
        const std::string slamSock = "/tmp/kestrel-voxslam-" + std::to_string(::getpid()) + ".sock";
        if (useSlam) {
            slamPid = ::fork();
            if (slamPid == 0) {
                if (!std::freopen("/dev/null", "w", stdout)) _exit(126);
                ::execl(slamBin, slamBin, "--vocab", slamVoc, "--socket", slamSock.c_str(),
                        (char*)nullptr);
                _exit(127);
            }
            slamCli.reset(new SlamClient(slamSock));
            for (int i = 0; i < 600 && !slamCli->connected(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            CHECK(slamCli->connected());
        }
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);

        MissionController::Params mp;
        mp.useVoxel = true; mp.useMap = false;
        // VOXTEST_YAWCAP=<0..1>: cap the turn-onto-a-leg yaw stick (the sim
        // maps a full stick to 90 deg/s).
        if (const char* yc = std::getenv("VOXTEST_YAWCAP")) mp.maxYawStick = float(std::atof(yc));
        MissionController m(mp);
        m.enable(true);

        WorldModel wm;
        const float dt = 0.05f, tau = 0.35f, vPerPitch = 1.0f / mp.cruise;
        float v = 0.f, travelled = 0.f, minClear = 1e9f;
        int legs = 0, ticks = 0;
        std::string lastPhase, minPhase;
        float minT = 0.f;
        // What this project scores on (CLAUDE.md): distance before a
        // collision, and -- against the degenerate answer of circling in a
        // safe clearing -- how far it ends from where it began and how much
        // ground it covers, in 1 m cells.
        std::vector<char> visited(size_t(sizeM) * size_t(sizeM), 0);
        int cells = 0, collisions = 0, collMove = 0, collHover = 0;
        bool inContact = false;
        const float simS = 150.f;
        for (float t = 0.f; t < simS; t += dt, ++ticks) {
            // Telemetry the aircraft would have: attitude, speed, and (for
            // the mission's leg-length gate only) where it is.
            if (useVio) {
                // Every tick is a camera frame (20 Hz). The IMU: roll/pitch
                // with 0.3 deg noise, yaw with a 0.5 deg/s gyro bias.
                const cv::Mat ir = vioCam.renderIR(w, truth);
                const cv::Mat dep = std::getenv("VOXTEST_STEREO")
                                  ? vioCam.renderStereo(w, truth, nullptr)
                                  : vioCam.renderTruth(w, truth);
                sim::CamPose imu;
                imu.rollDeg  = truth.rollDeg + 0.3f * N01(nrng);
                imu.pitchDeg = truth.pitchDeg + 0.3f * N01(nrng);
                imu.yawDeg   = truth.yawDeg + 0.5f * t;
                const sim::VioResult vr = vio.step(ir, dep, imu);
                ++vioFrames; vioMs += vr.ms;
                if (!vr.valid) ++vioLost;
                errE = vr.pose.e - truth.e; errN = vr.pose.n - truth.n;
            }
            if (useSlam && slamCli) {
                // Every tick a stereo pair, 50 mm apart, answered before the
                // tick goes on (the desk is fast enough; the Pi would drop
                // frames, which SlamClient handles and this does not model).
                const cv::Matx33d Rwc = SlamAnchor::rotWc(truth);
                sim::CamPose right = truth;
                right.e += float(Rwc(0, 0) * cp.baselineM);
                right.n += float(Rwc(1, 0) * cp.baselineM);
                right.u += float(Rwc(2, 0) * cp.baselineM);
                const cv::Mat il = slamCam.renderIR(w, truth), irr = slamCam.renderIR(w, right);
                slamlink::FrameHeader h;
                h.tS = double(t); h.width = slamCp.width; h.height = slamCp.height;
                h.fx = slamCam.fpx(); h.fy = slamCam.fy(); h.cx = slamCam.ppx(); h.cy = slamCam.ppy();
                h.baselineM = cp.baselineM; h.fps = 1.f / dt;
                const uint32_t seq = slamCli->submit(h, il.data, irr.data, {});
                slamlink::PoseReply rep;
                bool got = false;
                const auto tw = std::chrono::steady_clock::now();
                const double lim = vioFrames == 0 ? 300.0 : 30.0;
                bool bridgeDead = false;
                while (!got && std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                             tw).count() < lim) {
                    got = slamCli->takeReply(rep) && rep.seq == seq;
                    if (!got) {
                        // A DEAD BRIDGE is not a slow one: waiting 30 s a frame
                        // for 3000 frames once hung a sweep for seven hours.
                        int st = 0;
                        if (::waitpid(slamPid, &st, WNOHANG) == slamPid) { bridgeDead = true; break; }
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
                if (bridgeDead) {
                    std::printf("  SLAM bridge DIED at t=%.2f s (frame %d); run abandoned\n",
                                t, vioFrames);
                    CHECK(!bridgeDead);
                    slamPid = -1;
                    break;
                }
                ++vioFrames;
                const bool okT = got && (rep.state == slamlink::kOk || rep.state == slamlink::kOkKlt);
                if (got && vioFrames > 1) vioMs += rep.ms;
                // VOXTEST_SLAMTRACE=1: on every change of state; =2: also every
                // half second, to see an error that grows while tracking is OK.
                const char* trc = std::getenv("VOXTEST_SLAMTRACE");
                if (trc && got &&
                    (rep.state != slamPrevState || rep.mapChanges != slamPrevChanges ||
                     (std::atoi(trc) >= 2 && vioFrames % 10 == 0)))
                    std::printf("   t=%.2f slam state %d map %d changes %u tracked %d  phase %s  "
                                "at (%.2f,%.2f) yaw %.1f v %.2f  err %.2f (%.2f,%.2f)\n", t,
                                rep.state, rep.mapId, rep.mapChanges, rep.tracked,
                                wm.snapshot().missionPhase.c_str(), truth.e, truth.n,
                                truth.yawDeg, v, std::hypot(errE, errN), errE, errN);
                if (got) slamPrevChanges = rep.mapChanges;
                if (got) slamPrevState = rep.state;
                if (!okT) {
                    ++vioLost;                   // the estimate holds its last error
                } else {
                    // Anchored at the first tracked frame to the true pose (the
                    // start is known, as for VOXTEST_VIO) -- and, as the module
                    // does, RE-anchored to the current estimate whenever the
                    // SLAM starts a new map: a new map is a new frame.
                    if (!slamAnchor.anchored() || rep.mapId != slamMap) {
                        sim::CamPose a = truth;
                        a.e = truth.e + errE; a.n = truth.n + errN;
                        slamAnchor.anchor(rep.Twc, a);
                        if (slamMap != -1) ++slamMaps;
                        slamMap = rep.mapId;
                        if (std::getenv("VOXTEST_SLAMTRACE"))
                            std::printf("   t=%.2f map %d ANCHORED at truth (%.2f,%.2f) yaw %.1f, "
                                        "estimate off by (%.2f,%.2f)\n", t, rep.mapId, truth.e,
                                        truth.n, truth.yawDeg, errE, errN);
                    }
                    float se, sn, su, syaw;
                    slamAnchor.toEnu(rep.Twc, se, sn, su, syaw);
                    // The module's jump guard, on sim time.
                    const float pe = truth.e + errE, pn = truth.n + errN;
                    if (slamHaveLast && !SlamAnchor::plausible(se - pe, sn - pn, 0.f,
                                                               double(t - slamLastT))) {
                        sim::CamPose a = truth; a.e = pe; a.n = pn;
                        slamAnchor.anchor(rep.Twc, a);
                        slamAnchor.toEnu(rep.Twc, se, sn, su, syaw);
                        ++slamJumps;
                        if (std::getenv("VOXTEST_SLAMTRACE"))
                            std::printf("   t=%.2f JUMP GUARD fired\n", t);
                    }
                    slamHaveLast = true; slamLastT = t;
                    errE = se - truth.e; errN = sn - truth.n;
                }
            }
            wm.with([&](WorldState& s) {
                s.tickMonoS = monoNowS();
                s.vehYawDeg = truth.yawDeg; s.vehGroundspeed = std::fabs(v);
                s.estValid = true; s.estPe = truth.e + errE; s.estPn = truth.n + errN;
                s.estSpeed = std::fabs(v); s.missionGo = true;
                s.vehAltM = truth.u;
            });
            // The think tier runs at camera rate while a vantage is open, and
            // occasionally otherwise (it must still say "moving").
            const std::string ph = wm.snapshot().missionPhase;
            if (ph == "THINK" || ph == "SCAN" || ticks % 6 == 0) mod.run(cv::Mat(), wm);

            ControlCmd c;
            wm.with([&](WorldState& s) { c = m.update(s, dt); });
            const std::string now = wm.snapshot().missionPhase;
            if (now == "MOVE" && lastPhase != "MOVE") {
                ++legs;
                if (std::getenv("VOXTEST_TRACE")) {
                    const WorldState q = wm.snapshot();
                    std::printf("   t=%.1f leg from (%.2f,%.2f) brg %.1f cert %.2f "
                                "freeM %.2f frames %d truthSeg %.2f\n", t, truth.e,
                                truth.n, q.voxLegBearingDeg, q.voxLegFreeM, q.voxFreeM,
                                q.voxFrames,
                                segmentClearance(w, truth.e, truth.n, truth.u,
                                                 q.voxLegBearingDeg, q.voxLegFreeM, 2.f));
                }
            }
            if (std::getenv("VOXTEST_TRACE") && now != lastPhase)
                std::printf("   t=%.1f %s -> %s at (%.2f,%.2f) yaw %.1f v %.2f clr %.2f\n",
                            t, lastPhase.c_str(), now.c_str(), truth.e, truth.n,
                            truth.yawDeg, v, clearance(w, truth.e, truth.n, truth.u, 2.f));
            lastPhase = now;
            if (now == "STUCK") break;

            // Lagged kinematics: pitch -> forward speed, yaw -> yaw rate.
            truth.yawDeg += c.yaw * 90.f * dt;
            const float vCmd = c.pitch * vPerPitch;
            v += (vCmd - v) * dt / tau;
            const float a = truth.yawDeg * kPi / 180.f;
            // Drift: OU velocity, tau 2 s, stationary sigma driftSig per axis.
            // With odometry the FC can HOLD POSITION on the estimate (that is
            // what feeding VIO or flow into EKF3 is for), so free drift is
            // replaced by estimator error. VOXTEST_ODOM_NOHOLD keeps both.
            const bool held = odomFrac >= 0.f && !std::getenv("VOXTEST_ODOM_NOHOLD");
            if (driftSig > 0.f && !held) {
                const float tauD = 2.f, k = driftSig * std::sqrt(2.f * dt / tauD);
                dvE += -dvE * dt / tauD + k * N01(nrng);
                dvN += -dvN * dt / tauD + k * N01(nrng);
            }
            const float stepE = std::sin(a) * v * dt + dvE * dt;
            const float stepN = std::cos(a) * v * dt + dvN * dt;
            truth.e += stepE; truth.n += stepN;
            // Odometry error grows with distance MOVED (drift included -- a
            // visual odometer sees the drift, and errs on it like any motion).
            // SYSTEMATIC, not white: VIO and flow odometry err by a scale and
            // heading BIAS, so the error grows IN PROPORTION to distance. The
            // first version drew independent noise per 5 cm step, which
            // averages away as a random walk -- "10 %" came out as 0.15 m over
            // 60 m, about 0.25 %, and flattered architecture B. Now: a bias of
            // odomFrac per metre moved, whose direction wanders slowly
            // (0.1 rad/sqrt(s)), so 10 % over 60 m is ~6 m of error.
            if (odomFrac > 0.f && !useVio) {
                const float stepLen = std::hypot(stepE, stepN);
                biasDir += 0.1f * std::sqrt(dt) * N01(nrng);
                errE += odomFrac * stepLen * std::cos(biasDir);
                errN += odomFrac * stepLen * std::sin(biasDir);
            }
            travelled += std::hypot(stepE, stepN);
            const float cl = clearance(w, truth.e, truth.n, truth.u, 2.f);
            if (cl < minClear) { minClear = cl; minPhase = now; minT = t; }
            // A COLLISION is an ENTRY into the airframe's 0.3 m radius; the
            // sim does not stop the aircraft, so it is counted, not ended.
            if (cl < 0.3f && !inContact) {
                ++collisions;
                if (now == "MOVE") ++collMove; else ++collHover;
            }
            inContact = cl < 0.3f;
            const int ce = int(truth.e), cn = int(truth.n);
            if (ce >= 0 && cn >= 0 && ce < int(sizeM) && cn < int(sizeM) &&
                !visited[size_t(cn) * size_t(sizeM) + size_t(ce)]) {
                visited[size_t(cn) * size_t(sizeM) + size_t(ce)] = 1; ++cells;
            }
        }
        const float net = std::hypot(truth.e - spawnE, truth.n - spawnN);
        std::printf("  %d pillars %d walls  %.0f s  travelled %.1f m  net %.1f m  "
                    "cells %d  legs %d  min truth clearance %.2f m (in %s at %.1f s)  "
                    "final %s  frames %d\n",
                    pillars, walls, simS, travelled, net, cells, legs, minClear,
                    minPhase.c_str(), minT, lastPhase.c_str(), src->frames);
        if (slamPid > 0) {
            slamCli.reset();
            ::kill(slamPid, SIGTERM);
            ::waitpid(slamPid, nullptr, 0);
            ::unlink(slamSock.c_str());
        }
        if (useVio || useSlam)
            std::printf("  %s: %d frames, %d lost, %.1f ms/frame, final error %.2f m "
                        "over %.1f m (%.1f %%)\n", useSlam ? "SLAM" : "VIO", vioFrames, vioLost,
                        vioMs / std::max(1, vioFrames), std::hypot(errE, errN), travelled,
                        100.f * std::hypot(errE, errN) / std::max(0.1f, travelled));
        if (useSlam) std::printf("  SLAM maps started after the first: %d, jump guard fired %d\n",
                                 slamMaps, slamJumps);
        // One machine-readable line, for sweeps.
        std::printf("RESULT far=%d big=%d stereo=%d world=%s drift=%.2f odom=%s "
                    "moving=%d travel=%.2f net=%.2f cells=%d legs=%d minclr=%.3f "
                    "stuck=%d collisions=%d cmove=%d chover=%d odomerr=%.2f\n",
                    int(vp.farChoose), int(big), int(std::getenv("VOXTEST_STEREO") != nullptr),
                    ws ? ws : "7", driftSig,
                    useSlam ? "slam" : useVio ? "vio" : std::to_string(odomFrac).c_str(),
                    int(vp.integrateMoving),
                    travelled, net, cells, legs, minClear,
                    int(lastPhase == "STUCK"), collisions, collMove, collHover,
                    std::hypot(errE, errN));
        // The airframe is 0.3 m in radius; the planner's 0.6 m includes the
        // margin. Closer than 0.3 m to a solid voxel is a collision.
        CHECK(minClear >= 0.3f);
        // And it must actually fly: a planner that hovers for ever is safe
        // and useless, which is the degenerate answer to this objective.
        CHECK(legs >= 5);
        CHECK(travelled >= 8.f);
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
