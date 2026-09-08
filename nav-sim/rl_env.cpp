#include "rl_env.hpp"

#include <cstring>

#include <opencv2/imgproc.hpp>

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
    // THE LADDER, as voxel_live and voxel_sim run it. One 0.25 m map is honest
    // to about 3.5 m and blind past it, so a planner marching it alone rejects
    // everything further out as UNCONFIRMED -- not because anything is there,
    // but because nothing has been measured. The coarser levels buy range at
    // the cost of resolution, and the planner queries the FINEST level whose
    // honest range still covers the distance being marched.
    VoxelMap    mapMid, mapFar;
    VoxelMapParams mpMid, mpFar;
    BearingField bfield;
    std::unique_ptr<DepthCamera> cam;
    TrajectoryPlanner traj;
    CamParams cp;
    VoxelMapParams mp;

    float px = 0, py = 0, pz = 0, yaw = 0;
    float goalE = 0, goalN = 0, goalU = 0;
    float startDist = 1.f, prevDist = 1.f;
    float travel = 0.f, minClear = 1e9f;
    float minGoalDist = 1e9f; int minGoalStep = 0;
    // Episode totals per reward term. EnvStep is built fresh every step, so
    // these have to live with the episode or they accumulate nothing.
    float aProgress = 0, aCoverage = 0, aTime = 0, aStop = 0, aClear = 0, aTerm = 0;
    int   steps = 0, stopped = 0, collisions = 0;
    // Visit counts, so a policy can know it has been here before. The maze
    // failure is a planner re-deriving the same local preference at a junction
    // it has already left once.
    std::unordered_map<long long, float> visits;
    // The flown path, world metres. Kept only for the watch view -- nothing in
    // the observation or the reward reads it, so recording it cannot change
    // what the policy learns.
    std::vector<cv::Point2f> trail;
    // The last depth image, kept only so it can be looked at.
    cv::Mat lastDepth;

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

const char* baselineName(BaselinePolicy p) {
    switch (p) {
        case BaselinePolicy::Random: return "random";
        case BaselinePolicy::FreeM:  return "freeM";
        case BaselinePolicy::Goal:   return "goal";
        default:                     return "score";
    }
}

int chooseBaseline(BaselinePolicy pol, const std::vector<float>& obs,
                   const std::vector<uint8_t>& mask, int nPrims, unsigned& rng) {
    const int F = VoxelEnv::obsFeaturesPerPrim();
    std::vector<int> legal;
    for (int i = 0; i < nPrims && i < (int)mask.size(); ++i)
        if (mask[i]) legal.push_back(i);
    if (legal.empty()) return 0;
    if (pol == BaselinePolicy::Random) {
        rng = rng * 1664525u + 1013904223u;
        return legal[rng % legal.size()];
    }
    int best = legal[0];
    float bestV = -1e30f;
    for (int i : legal) {
        const float* o = &obs[size_t(i) * F];
        const float vv =
            (pol == BaselinePolicy::FreeM) ? o[0]
          : (pol == BaselinePolicy::Goal)  ? -o[3]
          : 0.7f * o[1] - 1.0f * o[3] - 0.25f * std::fabs(o[6])
            + 0.5f * o[2] - 2.0f * std::max(0.f, o[4]);
        if (vv > bestV) { bestV = vv; best = i; }
    }
    return best;
}

int VoxelEnv::nPrims() const { return int(im_->traj.librarySize()); }

void VoxelEnv::reset(const std::string& world, unsigned seed) {
    Impl& I = *im_;
    I.world = VoxelWorld();
    I.visits.clear();
    I.aProgress = I.aCoverage = I.aTime = I.aStop = I.aClear = I.aTerm = 0.f;
    I.trail.clear();
    I.trail.push_back(cv::Point2f(I.px, I.py));

    // FIVE STYLES, NOT TWO, AND VARIED WITHIN EACH. Training on forest and maze
    // alone lets a policy learn "forest behaviour" and "maze behaviour" -- two
    // modes selected by what the first frame looks like -- rather than
    // obstacle avoidance. The obstacle geometry differs completely between
    // these: trunks, corridor walls, building blocks, thin poles and wires, and
    // a trap with one way out. A policy that handles all five is doing
    // something more general than one that handles two.
    //
    // The PARAMETERS are drawn per seed as well, so "forest" is not one forest:
    // stem density, block pitch, street width and pole spacing all move. A
    // fixed generator with a moving seed still presents one distribution.
    unsigned ws = seed * 2246822519u + 374761393u;
    auto wrand = [&ws](float lo, float hi) {
        ws = ws * 1664525u + 1013904223u;
        return lo + (float(ws >> 8) * (1.0f / 16777216.0f)) * (hi - lo);
    };
    // Where a start or goal may be sampled. Every generator here builds from
    // the origin, so the box is the world's own extent inset by a margin.
    float bx0, bx1, by0, by1;

    if (world == "maze") {
        MazeParams m; m.cell = cfg_.cell; m.seed = seed;
        m.cellsX = int(wrand(5, 9)); m.cellsY = int(wrand(5, 9));
        m.corridorM = wrand(3.0f, 5.5f);
        float sx = 0, sy = 0, gx = 0, gy = 0;
        genMaze(I.world, m, &sx, &sy, &gx, &gy);
        I.px = sx; I.py = sy; I.pz = 1.5f;
        I.goalE = gx; I.goalN = gy; I.goalU = 1.5f;
        bx0 = std::min(sx, gx); bx1 = std::max(sx, gx);
        by0 = std::min(sy, gy); by1 = std::max(sy, gy);
    } else if (world == "corridor") {
        CorridorParams p; p.cell = cfg_.cell; p.seed = seed;
        p.sizeM = 140.f;
        p.widthMin = wrand(2.2f, 3.2f); p.widthMax = wrand(4.5f, 7.5f);
        p.nPaths = int(wrand(3, 7)); p.turnDeg = wrand(40.f, 80.f);
        float sx = 0, sy = 0, gx = 0, gy = 0;
        genCorridor(I.world, p, &sx, &sy, &gx, &gy);
        I.px = sx; I.py = sy; I.pz = 1.5f;
        I.goalE = gx; I.goalN = gy; I.goalU = 1.5f;
        bx0 = std::min(sx, gx); bx1 = std::max(sx, gx);
        by0 = std::min(sy, gy); by1 = std::max(sy, gy);
    } else if (world == "city") {
        CityParams p; p.cell = 0.5f; p.seed = seed; p.sizeM = 300.f;
        p.blockM = wrand(40.f, 80.f); p.streetM = wrand(10.f, 20.f);
        genCity(I.world, p);
        I.px = 20.f; I.py = 20.f; I.pz = 8.f;
        I.goalE = 280.f; I.goalN = 280.f; I.goalU = 8.f;
        bx0 = by0 = 15.f; bx1 = by1 = 285.f;
    } else if (world == "road") {
        // Thin, tall and few against a wide open background: the worst case
        // for a stereo mapper and the easiest for an openness histogram, so it
        // is where two planners are most likely to differ.
        RoadParams p; p.cell = 0.10f; p.seed = seed;
        p.poleEveryM = wrand(18.f, 34.f); p.nVehicles = int(wrand(6, 20));
        genRoad(I.world, p);
        I.px = p.widthM * 0.5f; I.py = 10.f; I.pz = 3.f;
        I.goalE = p.widthM * 0.5f; I.goalN = p.lengthM - 10.f; I.goalU = 3.f;
        bx0 = 4.f; bx1 = p.widthM - 4.f; by0 = 8.f; by1 = p.lengthM - 8.f;
    } else if (world == "culdesac") {
        CulDeSacParams p; p.cell = 0.5f; p.seed = seed; p.sizeM = 200.f;
        p.widthM = wrand(40.f, 70.f); p.depthM = wrand(45.f, 75.f);
        genCulDeSac(I.world, p);
        I.px = 100.f; I.py = 20.f; I.pz = 6.f;
        I.goalE = 100.f; I.goalN = 175.f; I.goalU = 6.f;
        bx0 = by0 = 15.f; bx1 = by1 = 185.f;
    } else {                                   // forest
        ForestParams f; f.cell = cfg_.cell; f.seed = seed;
        f.stemsPerHa = wrand(600.f, 1800.f);
        f.undergrowth = wrand(0.05f, 0.30f);
        genForest(I.world, f, nullptr);
        I.px = 15.f; I.py = 10.f; I.pz = 6.f;
        I.goalE = 120.f; I.goalN = 150.f; I.goalU = 8.f;
        bx0 = by0 = 15.f; bx1 = by1 = 185.f;
    }
    // VARIED START AND GOAL, deterministic in the seed so a run is still
    // reproducible. See EnvConfig::varyGoal for why fixed geometry was a
    // problem: a policy could pass every test here by learning one bearing.
    if (cfg_.varyGoal) {
        unsigned rs = seed * 2654435761u + 1013904223u;
        auto urand = [&rs]() {
            rs = rs * 1664525u + 1013904223u;
            return float(rs >> 8) * (1.0f / 16777216.0f);
        };
        // The box the world actually occupies. For the maze the two corners
        // genMaze handed back ARE its extent; for the forest it is the plot
        // inset far enough that a goal is never outside the trees.
        const float x0 = bx0, x1 = bx1, y0 = by0, y1 = by1;
        // Six tenths of the box diagonal: scale-free, so it means the same
        // thing in a 30 m maze and a 300 m city.
        const float minSep = 0.6f * std::hypot(x1 - x0, y1 - y0);
        const float want = cfg_.robotR * 1.6f;
        auto sampleClear = [&](float& ox, float& oy, float z,
                               float fromX, float fromY, float sep) {
            for (int i = 0; i < 200; ++i) {
                const float x = x0 + urand() * (x1 - x0);
                const float y = y0 + urand() * (y1 - y0);
                if (sep > 0.f && std::hypot(x - fromX, y - fromY) < sep) continue;
                if (trueClearance(I.world, x, y, z, want + 0.5f) >= want) {
                    ox = x; oy = y; return true;
                }
            }
            return false;                      // keep the nominal one
        };
        float sx = I.px, sy = I.py, gx = I.goalE, gy = I.goalN;
        if (sampleClear(sx, sy, I.pz, 0.f, 0.f, 0.f)) { I.px = sx; I.py = sy; }
        if (sampleClear(gx, gy, I.goalU, I.px, I.py, minSep)) {
            I.goalE = gx; I.goalN = gy;
        }
    }

    // THE SPAWN WAS NEVER CHECKED AGAINST THE WORLD IT LANDED IN. The forest
    // start was hardcoded at (15, 10, 6) whatever the trees did, so on 4 of 20
    // seeds the aircraft began with LESS clearance than its own radius: 0.19 m,
    // 0.38 m, 0.43 m, 0.17 m against robotR 0.6. Those episodes are lost before
    // the first action -- the run either scores an immediate collision or sits
    // still for the whole budget -- and every planner suffers it equally: on
    // forest seed 103 the random baseline also travelled 0.1 m in 600 steps.
    //
    // For training that is worse than a bad benchmark. Roughly a fifth of
    // forest episodes handed the policy -50 for a state no action could have
    // avoided, which teaches that the opening position is catastrophic rather
    // than teaching anything about flying.
    //
    // The maze never had this because genMaze returns a corridor start. The
    // forest now gets the same courtesy: nudge outward until there is room.
    {
        const float want = cfg_.robotR * 1.6f;
        if (trueClearance(I.world, I.px, I.py, I.pz, want + 0.5f) < want) {
            const float sx = I.px, sy = I.py;
            bool found = false;
            for (float r = 1.f; r <= 12.f && !found; r += 1.f)
                for (int a = 0; a < 24 && !found; ++a) {
                    const float th = a * 2.f * sim::PI_F / 24.f;
                    const float x = sx + r * std::cos(th), y = sy + r * std::sin(th);
                    if (trueClearance(I.world, x, y, I.pz, want + 0.5f) >= want) {
                        I.px = x; I.py = y; found = true;
                    }
                }
            // Nothing within 12 m: climb instead. A forest is open above the
            // canopy, so this always terminates somewhere flyable, and leaving
            // the aircraft inside a trunk is not an option worth preserving.
            if (!found) I.pz += 4.f;
        }
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

    // Same numbers voxel_sim uses, so a policy trained here meets the same
    // ladder the sim and the live tool fly. maxIntegM is the honest marking
    // range for that cell size: sqrt(cell * f * B / sigma) * 0.75.
    const float fpx = I.cam->fpx();
    I.mpMid = VoxelMapParams();
    I.mpMid.cell = 1.0f; I.mpMid.nx = 128; I.mpMid.ny = 128; I.mpMid.nz = 40;
    I.mpMid.maxIntegM = std::sqrt(I.mpMid.cell * fpx * I.cp.baselineM
                                  / I.cp.subpixelPx) * 0.75f;
    I.mpMid.maxCarveM = 25.f;
    I.mpMid.integrateStride = 2;      // a quarter of the rays
    I.mpMid.depthSigCoef = I.mp.depthSigCoef;
    I.mpMid.carveWinPx = 0;           // the min-filter is a fine-scale guard
    I.mapMid.init(I.mpMid, I.px, I.py, I.pz);

    I.mpFar = VoxelMapParams();
    I.mpFar.cell = 2.0f; I.mpFar.nx = 128; I.mpFar.ny = 128; I.mpFar.nz = 42;
    I.mpFar.maxIntegM = std::sqrt(I.mpFar.cell * fpx * I.cp.baselineM
                                  / I.cp.subpixelPx) * 0.75f;
    I.mpFar.maxCarveM = 40.f;
    I.mpFar.integrateStride = 4;      // a sixteenth of the rays
    I.mpFar.depthSigCoef = I.mp.depthSigCoef;
    I.mpFar.carveWinPx = 0;
    I.mapFar.init(I.mpFar, I.px, I.py, I.pz);
    I.bfield = BearingField();
    // init() is not optional -- rangeAt() on a default-constructed field is a
    // segfault, and the planner queries it on the very first observation.
    I.bfield.init(BearingFieldParams{});

    I.startDist = std::hypot(I.goalE - I.px, I.goalN - I.py);
    I.prevDist = I.startDist;
    I.travel = 0.f; I.minClear = 1e9f;
    I.minGoalDist = I.startDist; I.minGoalStep = 0;
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
    I.trail.push_back(cv::Point2f(I.px, I.py));
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
    I.lastDepth = d.clone();
    I.map.integrate(d, *I.cam, pose);
    I.map.recentre(I.px, I.py, I.pz);
    I.mapMid.integrate(d, *I.cam, pose);
    I.mapMid.recentre(I.px, I.py, I.pz);
    I.mapFar.integrate(d, *I.cam, pose);
    I.mapFar.recentre(I.px, I.py, I.pz);
    I.bfield.update(d, *I.cam, pose, 1);

    // --- reward ------------------------------------------------------------
    const float dist = std::hypot(I.goalE - I.px, I.goalN - I.py);
    if (dist < I.minGoalDist) { I.minGoalDist = dist; I.minGoalStep = I.steps; }
    const float progress = I.prevDist - dist;
    I.prevDist = dist;

    // COVERAGE, not raw distance. Rewarding metres travelled pays a policy that
    // orbits; rewarding NEWLY VISITED cells pays one that goes somewhere.
    const long long key = I.visitKey(I.px, I.py, cfg_.visitCellM);
    auto it = I.visits.find(key);
    const float novelty = (it == I.visits.end()) ? 1.f : 0.f;
    I.visits[key] = (it == I.visits.end()) ? 1.f : it->second + 1.f;

    // Fraction of the journey, not metres -- see EnvConfig::progressScaleM.
    const float tProgress = cfg_.wProgress * progress
                          * (cfg_.progressScaleM / std::max(1.f, I.startDist));
    const float tCoverage = cfg_.wCoverage * novelty;
    const float tTime     = -cfg_.wTime * dt;
    const float tStop     = -cfg_.wStop * (speed < 0.1f ? 1.f : 0.f)
                            - (legal ? 0.f : cfg_.wStop);
    const float tClear    = -cfg_.wClear * std::max(0.f, cfg_.clearTarget - clr);
    float r = tProgress + tCoverage + tTime + tStop + tClear;
    I.aProgress += tProgress; I.aCoverage += tCoverage; I.aTime += tTime;
    I.aStop += tStop;         I.aClear += tClear;

    out.reachedGoal = dist <= cfg_.goalTolM;
    // Scaled against the most progress this episode could ever have paid, so a
    // crash cannot be bought with metres in a big world -- see EnvConfig.
    // The same in every world now, which is the point: a collision costs the
    // same relative to the best possible episode wherever it happens.
    const float maxProgress = cfg_.wProgress * cfg_.progressScaleM;
    const float goalR    = std::max(cfg_.rGoal,    cfg_.goalScale    * maxProgress);
    const float collideR = std::max(cfg_.rCollide, cfg_.collideScale * maxProgress);
    if (out.reachedGoal) { r += goalR;    I.aTerm += goalR; }
    if (hit)             { r -= collideR; I.aTerm -= collideR; }

    out.done      = out.reachedGoal || hit;
    out.truncated = !out.done && I.steps >= cfg_.maxSteps;
    out.reward = r;
    out.travelM = I.travel; out.distToGoalM = dist; out.minClearM = I.minClear;
    out.minDistToGoalM = I.minGoalDist; out.minDistStep = I.minGoalStep;
    out.rProgress = I.aProgress; out.rCoverage = I.aCoverage; out.rTime = I.aTime;
    out.rStop = I.aStop; out.rClear = I.aClear; out.rTerminal = I.aTerm;
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
    // The ladder was empty here, so every rollout past ~3.5 m marched into
    // unmeasured space and came back "unconfirmed". The policy saw a wall of
    // why==2 on anything far and had no way to tell unexplored from blocked
    // beyond the fine map's honest range.
    const std::vector<TrajectoryPlanner::CoarseLevel> coarse = {
        {&I.mapMid, I.mpMid.maxIntegM},
        {&I.mapFar, I.mpFar.maxIntegM},
    };
    I.traj.plan(I.map, I.px, I.py, I.pz, I.yaw, gAz, gEl, coarse, &fb);

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

        // maskUnsafe=false hands the policy every primitive, including the
        // ones that end inside something. nAdm still counts what geometry
        // WOULD have approved, so the observation and the logging do not change
        // meaning between the two modes -- only what is selectable does.
        if (e.admissible) ++nAdm;
        if (e.admissible || !cfg_.maskUnsafe) mask_[i] = 1;
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



// --------------------------------------------------------------------- watch
// THE FPV VIEW, from inside the map the aircraft has built -- the same
// renderLadder the sim labels "VOXEL FPV (what it believes)". Not a top-down
// slice: a slice shows a plan, and what you want while a policy trains is what
// the policy is looking at. Pale is UNKNOWN, drawn as fog and never as air,
// which is the whole point -- where the map is wrong the aircraft flies into
// haze, and that is visible here and invisible in a reward curve.
//
// One layer, not three. The env keeps a single map at one cell size, so the
// ladder that voxel_live builds across fine/mid/far collapses to its one rung.
// The 1.15 inflation on the outer edge is kept for the same reason it exists
// there: it covers cells marked slightly beyond maxIntegM before the aircraft
// moved, which otherwise render as a round blind spot.
std::vector<uint8_t> VoxelEnv::renderDepth(int w, int h) const {
    const Impl& I = *im_;
    w = std::max(80, std::min(1600, w));
    h = std::max(60, std::min(1200, h));
    cv::Mat out(h, w, CV_8UC3, cv::Scalar(60, 60, 60));
    if (!I.lastDepth.empty()) {
        // Same ramp as the sim's DEPTH pane, so the two cannot disagree about
        // what a colour means. Grey stays grey: no stereo match is not "far".
        cv::Mat dv(I.lastDepth.rows, I.lastDepth.cols, CV_8UC3,
                   cv::Scalar(60, 60, 60));
        for (int y = 0; y < I.lastDepth.rows; ++y)
            for (int x = 0; x < I.lastDepth.cols; ++x) {
                const float r = I.lastDepth.at<float>(y, x);
                if (!(r > 0.f)) continue;
                const float f = std::min(1.f, r / I.cp.maxRangeM);
                dv.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(255 * (1 - f)),
                                                   uchar(80 + 100 * f),
                                                   uchar(255 * f));
            }
        cv::resize(dv, out, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    }
    std::vector<uint8_t> buf(size_t(out.rows) * out.cols * 3);
    if (out.isContinuous()) std::memcpy(buf.data(), out.data, buf.size());
    else for (int y = 0; y < out.rows; ++y)
        std::memcpy(buf.data() + size_t(y) * out.cols * 3, out.ptr(y),
                    size_t(out.cols) * 3);
    return buf;
}

std::vector<uint8_t> VoxelEnv::renderFrame(int w, int h, bool topDown) const {
    const Impl& I = *im_;
    w = std::max(80, std::min(1600, w));
    h = std::max(60, std::min(1200, h));

    cv::Mat img;
    if (topDown) {
        img = I.map.sliceImage(I.pz, std::min(w, h));
        const VoxelMapParams& mp = I.map.params();
        const int px = img.cols;
        auto toPx = [&](float wx, float wy) {
            int cx, cy, cz;
            I.map.worldToCell(wx, wy, I.pz, cx, cy, cz);
            return cv::Point(int((cx + 0.5f) * float(px) / float(mp.nx)),
                             int((mp.ny - 1 - cy + 0.5f) * float(px) / float(mp.ny)));
        };
        // The goal is usually OFF this frame -- the map is a local grid and the
        // goal often a hundred metres past its edge -- so clamped to the border
        // it still reads as a direction instead of being silently absent.
        cv::Point g = toPx(I.goalE, I.goalN);
        const bool off = g.x < 0 || g.y < 0 || g.x >= px || g.y >= px;
        g.x = std::max(6, std::min(px - 7, g.x));
        g.y = std::max(6, std::min(px - 7, g.y));
        cv::drawMarker(img, g, cv::Scalar(60, 200, 60),
                       off ? cv::MARKER_TRIANGLE_UP : cv::MARKER_TILTED_CROSS,
                       std::max(8, px / 22), 2, cv::LINE_AA);
        for (size_t i = 1; i < I.trail.size(); ++i)
            cv::line(img, toPx(I.trail[i-1].x, I.trail[i-1].y),
                          toPx(I.trail[i].x,   I.trail[i].y),
                     cv::Scalar(40, 90, 220), 2, cv::LINE_AA);
        const cv::Point a = toPx(I.px, I.py);
        const float yr = I.yaw * sim::PI_F / 180.f;
        const float lead = std::max(6.f, px / 16.f);
        cv::line(img, a, cv::Point(a.x + int(std::sin(yr) * lead),
                                   a.y - int(std::cos(yr) * lead)),
                 cv::Scalar(255, 210, 90), 2, cv::LINE_AA);
        cv::circle(img, a, std::max(3, px / 60), cv::Scalar(255, 210, 90), -1, cv::LINE_AA);
        if (img.cols != w || img.rows != h) cv::resize(img, img, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    } else {
        // All three rungs, as the sim draws them. The 1.15 inflation belongs
        // only on the OUTERMOST edge: on an internal handover it creates a
        // shell one layer owns and has no data for, which renders as a round
        // blind spot.
        std::vector<VoxelMap::Layer> ladder;
        ladder.push_back({&I.map,    0.f,               I.mp.maxIntegM});
        ladder.push_back({&I.mapMid, I.mp.maxIntegM,    I.mpMid.maxIntegM});
        ladder.push_back({&I.mapFar, I.mpMid.maxIntegM, I.mpFar.maxIntegM * 1.15f});
        img = VoxelMap::renderLadder(ladder, I.px, I.py, I.pz, I.yaw, 0.f,
                                     w, h, I.cp.hfovDeg);
        if (img.empty()) img = cv::Mat(h, w, CV_8UC3, cv::Scalar(30, 30, 36));

        // A heading tape, because an FPV view of fog looks the same at every
        // yaw and a policy that is spinning would otherwise be indistinguishable
        // from one that is flying straight.
        const float bearing = std::atan2(I.goalE - I.px, I.goalN - I.py) * 180.f / sim::PI_F;
        float rel = bearing - I.yaw;
        while (rel > 180.f) rel -= 360.f;
        while (rel < -180.f) rel += 360.f;
        const int gx = int(w * 0.5f + (rel / (I.cp.hfovDeg * 0.5f)) * (w * 0.5f));
        if (gx >= 0 && gx < w)
            cv::drawMarker(img, cv::Point(gx, 14), cv::Scalar(60, 220, 60),
                           cv::MARKER_TRIANGLE_DOWN, 12, 2, cv::LINE_AA);
        else
            cv::putText(img, rel < 0 ? "<goal" : "goal>",
                        cv::Point(rel < 0 ? 4 : w - 46, 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(60, 220, 60), 1, cv::LINE_AA);
    }

    std::vector<uint8_t> out(size_t(img.rows) * img.cols * 3);
    if (img.isContinuous()) std::memcpy(out.data(), img.data, out.size());
    else for (int y = 0; y < img.rows; ++y)
        std::memcpy(out.data() + size_t(y) * img.cols * 3, img.ptr(y), size_t(img.cols) * 3);
    return out;
}

}  // namespace sim
