#include "rl_env.hpp"

#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace sim {

namespace {
// True clearance from the WORLD, for scoring only -- never fed to the policy.
// Same rule as voxel_sim: a harness that scores the plan against the map it
// planned with passes no matter how wrong the map is.
float trueClearance(const VoxelWorld& w, float x, float y, float z, float maxR) {
    int cx, cy, cz; w.worldToCell(x, y, z, cx, cy, cz);
    const int r = int(maxR / w.cell()) + 1;
    float best = maxR;
    for (int dz = -r; dz <= r; ++dz)
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                if (!w.solid(cx + dx, cy + dy, cz + dz)) continue;
                float wx, wy, wz; w.cellCentre(cx + dx, cy + dy, cz + dz, wx, wy, wz);
                const float d = std::sqrt((wx-x)*(wx-x) + (wy-y)*(wy-y) + (wz-z)*(wz-z));
                if (d < best) best = d;
            }
    return best;
}
}  // namespace

struct VoxelEnv::Impl {
    VoxelWorld  world;
    VoxelMap    map;
    BearingField bfield;
    std::unique_ptr<DepthCamera> cam;
    TrajectoryPlanner traj;
    CamParams cp;
    VoxelMapParams mp;

    float px = 0, py = 0, pz = 0, yaw = 0;
    float goalE = 0, goalN = 0, goalU = 0;
    float startDist = 1.f, prevDist = 1.f;
    float travel = 0.f, minClear = 1e9f;
    int   steps = 0, stopped = 0, collisions = 0;
    // Visit counts, so a policy can know it has been here before. The maze
    // failure is a planner re-deriving the same local preference at a junction
    // it has already left once.
    std::unordered_map<long long, float> visits;

    Impl(const TrajParams& tp) : traj(tp) {}

    long long visitKey(float x, float y, float cellM) const {
        const long long ix = (long long)std::floor(x / cellM);
        const long long iy = (long long)std::floor(y / cellM);
        return ix * 1000003LL + iy;
    }
};

VoxelEnv::VoxelEnv(const EnvConfig& c) : cfg_(c) {
    TrajParams tp;
    tp.robotR = cfg_.robotR;
    tp.horizonS = cfg_.horizonS;
    tp.dt = cfg_.dt;
    im_.reset(new Impl(tp));
    reset(cfg_.world, cfg_.seed);
}

VoxelEnv::~VoxelEnv() = default;

int VoxelEnv::nPrims() const { return int(im_->traj.librarySize()); }

void VoxelEnv::reset(const std::string& world, unsigned seed) {
    Impl& I = *im_;
    I.world = VoxelWorld();
    I.visits.clear();

    if (world == "maze") {
        MazeParams m; m.cell = cfg_.cell; m.seed = seed;
        m.cellsX = 6; m.cellsY = 6; m.corridorM = 4.0f;
        float sx = 0, sy = 0, gx = 0, gy = 0;
        genMaze(I.world, m, &sx, &sy, &gx, &gy);
        I.px = sx; I.py = sy; I.pz = 1.5f;
        I.goalE = gx; I.goalN = gy; I.goalU = 1.5f;
    } else {                                   // forest
        ForestParams f; f.cell = cfg_.cell; f.seed = seed;
        genForest(I.world, f, nullptr);
        I.px = 15.f; I.py = 10.f; I.pz = 6.f;
        I.goalE = 120.f; I.goalN = 150.f; I.goalU = 8.f;
    }
    I.yaw = std::atan2(I.goalE - I.px, I.goalN - I.py) * 180.f / sim::PI_F;

    // Camera and map exactly as voxel_sim derives them, so a policy trained
    // here meets the same Z_max and the same dropout the classical planner did.
    I.cp = CamParams();
    I.cp.width = cfg_.camW; I.cp.height = cfg_.camH;
    I.cp.hfovDeg = 70.f; I.cp.baselineM = 0.12f; I.cp.maxRangeM = 12.f;
    I.cam.reset(new DepthCamera(I.cp));

    I.mp = VoxelMapParams();
    I.mp.cell = cfg_.cell; I.mp.nx = 240; I.mp.ny = 240; I.mp.nz = 96;
    I.mp.maxCarveM = 11.f;
    I.mp.depthSigCoef = I.cp.subpixelPx / (I.cam->fpx() * I.cp.baselineM);
    I.mp.maxIntegM = std::sqrt(I.mp.cell * I.cam->fpx() * I.cp.baselineM
                               / I.cp.subpixelPx) * 0.75f;
    I.map.init(I.mp, I.px, I.py, I.pz);
    I.map.seedFree(I.px, I.py, I.pz, cfg_.robotR * 1.5f);
    I.bfield = BearingField();
    // init() is not optional -- rangeAt() on a default-constructed field is a
    // segfault, and the planner queries it on the very first observation.
    I.bfield.init(BearingFieldParams{});

    I.startDist = std::hypot(I.goalE - I.px, I.goalN - I.py);
    I.prevDist = I.startDist;
    I.travel = 0.f; I.minClear = 1e9f;
    I.steps = 0; I.stopped = 0; I.collisions = 0;

    last_ = EnvStep();
    last_.distToGoalM = I.startDist;
    buildObservation();
}

EnvStep VoxelEnv::step(int action) {
    Impl& I = *im_;
    EnvStep out;

    // --- act ---------------------------------------------------------------
    // A masked or out-of-range index is treated as HOLD rather than as an
    // error: a policy that has not learned the mask yet should be penalised by
    // the reward, not crash the trainer.
    float speed = 0, yawRate = 0, climb = 0;
    const bool legal = action >= 0 && action < nPrims()
                    && action < (int)mask_.size() && mask_[action];
    if (legal) I.traj.primCommand((size_t)action, speed, yawRate, climb);

    // FLY THE ROLLOUT, not the nominal speed. The path sphereClear approved was
    // integrated with the vehicle's first-order velocity lag, so applying the
    // commanded speed directly travels further than what was checked -- and
    // collides on a primitive the veto had passed.
    const float dt = cfg_.dt;
    float bx = 0, by = 0, bz = 0;
    if (legal) I.traj.primFirstStep((size_t)action, bx, by, bz);
    const float ca = std::cos(I.yaw * sim::PI_F / 180.f);
    const float sa = std::sin(I.yaw * sim::PI_F / 180.f);
    const float nx = I.px + bx * ca + by * sa;
    const float ny = I.py - bx * sa + by * ca;
    const float nz = I.pz + bz;
    I.yaw += yawRate * dt;
    const float moved = std::sqrt((nx-I.px)*(nx-I.px) + (ny-I.py)*(ny-I.py)
                                + (nz-I.pz)*(nz-I.pz));
    I.px = nx; I.py = ny; I.pz = nz;
    I.travel += moved;
    ++I.steps;
    if (speed < 0.1f) ++I.stopped;

    // --- score against TRUTH, which the policy never sees ------------------
    const float clr = trueClearance(I.world, I.px, I.py, I.pz, 1.2f);
    I.minClear = std::min(I.minClear, clr);
    const bool hit = clr < cfg_.robotR;
    if (hit) ++I.collisions;

    // --- sense: render, integrate. The map is one the policy BUILT ---------
    CamPose pose; pose.e = I.px; pose.n = I.py; pose.u = I.pz; pose.yawDeg = I.yaw;
    cv::Mat d = cfg_.truthDepth ? I.cam->renderTruth(I.world, pose)
                                : I.cam->renderStereo(I.world, pose, nullptr);
    I.map.integrate(d, *I.cam, pose);
    I.map.recentre(I.px, I.py, I.pz);
    I.bfield.update(d, *I.cam, pose, 1);

    // --- reward ------------------------------------------------------------
    const float dist = std::hypot(I.goalE - I.px, I.goalN - I.py);
    const float progress = I.prevDist - dist;
    I.prevDist = dist;

    // COVERAGE, not raw distance. Rewarding metres travelled pays a policy that
    // orbits; rewarding NEWLY VISITED cells pays one that goes somewhere.
    const long long key = I.visitKey(I.px, I.py, cfg_.visitCellM);
    auto it = I.visits.find(key);
    const float novelty = (it == I.visits.end()) ? 1.f : 0.f;
    I.visits[key] = (it == I.visits.end()) ? 1.f : it->second + 1.f;

    float r = cfg_.wProgress * progress
            + cfg_.wCoverage * novelty
            - cfg_.wTime * dt
            - cfg_.wStop * (speed < 0.1f ? 1.f : 0.f)
            - cfg_.wClear * std::max(0.f, cfg_.clearTarget - clr);
    if (!legal) r -= cfg_.wStop;      // choosing a masked action is a hold

    out.reachedGoal = dist <= cfg_.goalTolM;
    if (out.reachedGoal) r += cfg_.rGoal;
    if (hit)             r -= cfg_.rCollide;

    out.done      = out.reachedGoal || hit;
    out.truncated = !out.done && I.steps >= cfg_.maxSteps;
    out.reward = r;
    out.travelM = I.travel; out.distToGoalM = dist; out.minClearM = I.minClear;
    out.collisions = I.collisions; out.stoppedSteps = I.stopped; out.steps = I.steps;

    buildObservation();
    last_ = out;
    return out;
}

void VoxelEnv::buildObservation() {
    Impl& I = *im_;
    const int F = obsFeaturesPerPrim();
    const int n = nPrims();
    obs_.assign(size_t(n) * F + obsGlobalFeatures(), 0.f);
    mask_.assign(n, 0);

    // Re-plan to refresh the per-primitive evaluation against the new map. The
    // classical planner's own argmax is discarded -- only its ROLLOUTS are used,
    // so the policy chooses from exactly the set sphereClear admitted.
    const float gAz = std::atan2(I.goalE - I.px, I.goalN - I.py) * 180.f / sim::PI_F;
    const float gEl = std::atan2(I.goalU - I.pz,
                                 std::hypot(I.goalE - I.px, I.goalN - I.py))
                    * 180.f / sim::PI_F;
    TrajectoryPlanner::FarBearings fb{&I.bfield, 20.f};
    I.traj.plan(I.map, I.px, I.py, I.pz, I.yaw, gAz, gEl, {}, &fb);

    const auto& ev = I.traj.evals();
    const float reach = std::max(0.1f, 3.f * cfg_.horizonS);
    int nAdm = 0;
    float sumFree = 0.f, maxFree = 0.f;
    for (int i = 0; i < n && i < (int)ev.size(); ++i) {
        const auto& e = ev[i];
        float* o = &obs_[size_t(i) * F];
        o[0] = e.freeM / reach;                       // confirmed-free length
        o[1] = e.clear;
        o[2] = e.farOpen;                             // advisory only
        o[3] = e.goalErr;
        o[4] = e.endEl / 90.f;
        o[5] = e.speed / 3.f;
        o[6] = e.yawRate / 100.f;
        // OBSERVABILITY, and this is the channel that must exist. why==2 means
        // the rollout stopped on UNKNOWN rather than on a surface: the
        // primitive is not blocked, it is unexplored, and those want opposite
        // responses. A single scalar cannot say it.
        o[7] = (e.why == 2) ? 1.f : 0.f;
        // Visit count at the endpoint, decayed -- "have I been down here".
        const float ex = I.px + std::sin(e.endAz * sim::PI_F / 180.f) * e.freeM;
        const float ey = I.py + std::cos(e.endAz * sim::PI_F / 180.f) * e.freeM;
        auto it = I.visits.find(I.visitKey(ex, ey, cfg_.visitCellM));
        o[8] = (it == I.visits.end()) ? 0.f : std::min(1.f, it->second / 20.f);

        if (e.admissible) { mask_[i] = 1; ++nAdm; }
        sumFree += e.freeM; maxFree = std::max(maxFree, e.freeM);
    }

    float* g = &obs_[size_t(n) * F];
    const float dist = std::hypot(I.goalE - I.px, I.goalN - I.py);
    g[0] = std::min(1.f, dist / std::max(1.f, I.startDist));
    g[1] = std::sin(gAz * sim::PI_F / 180.f);
    g[2] = std::cos(gAz * sim::PI_F / 180.f);
    g[3] = gEl / 90.f;
    g[4] = float(nAdm) / std::max(1, n);
    g[5] = (n ? sumFree / n : 0.f) / reach;
    g[6] = maxFree / reach;
    g[7] = std::min(1.f, float(I.stopped) / std::max(1, I.steps));
    g[8] = std::min(1.f, float(I.steps) / std::max(1, cfg_.maxSteps));
    g[9] = std::min(1.f, I.travel / std::max(1.f, I.startDist));
    // A coarse bearing summary, WITH its mask: 12 azimuth sectors, each a
    // normalised range and a "did anything answer here" flag. Absence of a
    // return is absence of knowledge, and the policy has to be able to tell.
    for (int k = 0; k < 6; ++k) {
        const float az = -90.f + k * 36.f;
        const float rr = I.bfield.rangeAt(I.yaw + az, 0.f);
        g[10 + k * 2]     = (rr < 0.f) ? 0.f : std::min(1.f, rr / 20.f);
        g[10 + k * 2 + 1] = (rr < 0.f) ? 0.f : 1.f;      // the mask
    }
}

}  // namespace sim
