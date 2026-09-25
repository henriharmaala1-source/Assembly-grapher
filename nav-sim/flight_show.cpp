#include "flight_show.hpp"

#include <algorithm>
#include <cmath>

namespace kshow {

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr float kBodyR = 0.3f;     // the airframe; the planner's 0.6 m has margin
}  // namespace

// The D435i as the aircraft has it: depth, and the camera's own attitude, but
// NOT its position -- exactly test_voxel_nav's AttitudeOnlySource. It also
// keeps the last frame for the display, which is the only thing added.
class FlightShow::Source : public sim::FrameSource {
public:
    Source(const sim::VoxelWorld& w, const sim::CamParams& p, bool stereo,
           const sim::CamPose* truth)
        : sim_(w, p, /*truth depth*/ !stereo), truth_(truth) {}
    const char* name() const override { return "sim-d435i-attitude-only"; }
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
        last = depth;                      // shared, read-only from here on
        lastPose = *truth_;
        ++count;
        return true;
    }
    cv::Mat last;
    sim::CamPose lastPose;
    long count = 0;
private:
    sim::SimFrameSource sim_;
    const sim::CamPose* truth_;
};

FlightShow::FlightShow(const FlightParams& p)
    : p_(p), world_(std::make_shared<sim::VoxelWorld>()) {
    // THE WORLD: the demo's gallery or hall -- VoxelEnv's generator and
    // parameter ranges (the room pitch is drawn from the seed here, so a seed
    // is not the same layout as VoxelEnv's).
    sim::GalleryParams g;
    g.cell = 0.25f; g.seed = p.seed; g.sizeM = 160.f;
    {
        unsigned r = p.seed * 2654435761u + 7u;
        r = r * 1664525u + 1013904223u;
        g.pitchM = 12.f + 4.f * float(r >> 8) / float(1u << 24);
    }
    const bool hall = p.world == "hall";
    g.wallFrac = hall ? 0.65f : 0.30f;
    g.ceiling  = hall;
    g.minHM = hall ? 10.f : 14.f;
    g.maxHM = hall ? 14.f : 24.f;
    float sx = 0, sy = 0, gx = 0, gy = 0;
    sim::genGallery(*world_, g, &sx, &sy, &gx, &gy);
    floorZ_ = g.latticeM;                  // the floor slab's top (genGallery)
    spawnE_ = sx; spawnN_ = sy;

    truth_ = sim::CamPose{};
    truth_.e = sx; truth_.n = sy; truth_.u = floorZ_ + p.altM;
    truth_.yawDeg = 0.f;
    vantage_ = truth_;

    sim::CamParams cp;
    cp.width = p.camW; cp.height = p.camH; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    cp.irBandLimit = true;

    // THE AIRCRAFT'S CONFIGURATION, as main.cpp builds it for --voxel: the
    // leg certified to stepM + the stop margin; everything else the defaults
    // that fly.
    MissionController::Params mp;
    mp.useVoxel = true; mp.useMap = false;
    VoxelNavModule::Params vp;
    vp.stillSpeedMs = mp.settleSpeedMs;
    vp.legMaxM = std::max(mp.stepM + mp.voxStopMarginM, 1.f);

    src_ = new Source(*world_, cp, p.stereo, &truth_);
    mod_.reset(new VoxelNavModule(std::unique_ptr<sim::FrameSource>(src_), vp));
    mission_.reset(new MissionController(mp));
    mission_->enable(true);

    const int n = int(g.sizeM);
    visited_.assign(size_t(n) * size_t(n), 0);
    trail_.push_back({truth_.e, truth_.n, truth_.u});
}

FlightShow::~FlightShow() = default;

const sim::DepthCamera& FlightShow::camera() const { return src_->camera(); }
const cv::Mat& FlightShow::lastDepth() const { return src_->last; }
const sim::CamPose& FlightShow::lastDepthPose() const { return src_->lastPose; }
long FlightShow::depthFrames() const { return src_->count; }

float FlightShow::clearance(float px, float py, float pz, float upTo) const {
    const sim::VoxelWorld& w = *world_;
    const float c = w.cell();
    int cx, cy, cz;
    w.worldToCell(px, py, pz, cx, cy, cz);
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
                best = std::min(best, std::sqrt(dx * dx + dy * dy + dz * dz));
            }
    return best;
}

void FlightShow::tick() {
    const float dt = kDt;
    // Telemetry the aircraft would have (test_voxel_nav's closed loop).
    wm_.with([&](WorldState& s) {
        s.tickMonoS = monoNowS();
        s.vehYawDeg = truth_.yawDeg; s.vehGroundspeed = std::fabs(v_);
        s.estValid = true; s.estPe = truth_.e; s.estPn = truth_.n;
        s.estSpeed = std::fabs(v_); s.missionGo = true;
        s.vehAltM = truth_.u - floorZ_;
    });
    // The think tier runs at camera rate while a vantage is open, and
    // occasionally otherwise (it must still say "moving").
    const std::string ph = wm_.snapshot().missionPhase;
    if (ph == "THINK" || ph == "SCAN" || ticks_ % 6 == 0) mod_->run(cv::Mat(), wm_);
    if (mod_->resets() != lastResets_) {       // a new vantage: a new map, here
        lastResets_ = mod_->resets();
        vantage_ = truth_;
        ++stats_.stops;
    }

    ControlCmd c;
    wm_.with([&](WorldState& s) { c = mission_->update(s, dt); });
    const WorldState s = wm_.snapshot();
    if (s.missionPhase == "MOVE" && phase_ != "MOVE") {
        ++stats_.legs;
        Leg l;
        l.e0 = truth_.e; l.n0 = truth_.n; l.bearingDeg = s.voxLegBearingDeg;
        l.lengthM = std::hypot(s.missionWpE - truth_.e, s.missionWpN - truth_.n);
        legs_.push_back(l);
    }
    phase_ = s.missionPhase;
    if (phase_ == "STUCK") { stats_.stuck = true; return; }

    // The airframe: pitch -> forward speed (lagged), yaw -> yaw rate.
    const float tau = 0.35f, vPerPitch = 1.f / MissionController::Params().cruise;
    truth_.yawDeg += c.yaw * 90.f * dt;
    while (truth_.yawDeg >= 360.f) truth_.yawDeg -= 360.f;
    while (truth_.yawDeg < 0.f) truth_.yawDeg += 360.f;
    v_ += (c.pitch * vPerPitch - v_) * dt / tau;
    const float a = truth_.yawDeg * kPi / 180.f;
    const float stepE = std::sin(a) * v_ * dt, stepN = std::cos(a) * v_ * dt;
    truth_.e += stepE; truth_.n += stepN;
    ++ticks_;

    stats_.timeS += dt;
    stats_.travelM += std::hypot(stepE, stepN);
    stats_.netM = std::hypot(truth_.e - spawnE_, truth_.n - spawnN_);
    const float cl = clearance(truth_.e, truth_.n, truth_.u, 2.f);
    stats_.minClearM = std::min(stats_.minClearM, cl);
    if (cl < kBodyR && !inContact_) ++stats_.collisions;
    inContact_ = cl < kBodyR;
    const int n = int(std::sqrt(double(visited_.size())));
    const int ce = int(truth_.e), cn = int(truth_.n);
    if (ce >= 0 && cn >= 0 && ce < n && cn < n && !visited_[size_t(cn) * n + ce]) {
        visited_[size_t(cn) * n + ce] = 1;
        ++stats_.cells;
    }
    const cv::Point3f& b = trail_.back();
    if (std::hypot(truth_.e - b.x, truth_.n - b.y) > 0.1f)
        trail_.push_back({truth_.e, truth_.n, truth_.u});
}

}  // namespace kshow
