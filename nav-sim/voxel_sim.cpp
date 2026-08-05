// Closed-loop navigation harness over a voxel world.
//
//   sense -> map -> plan -> move -> repeat,  scored against ground truth.
//
//   ./voxel_sim --world forest --goal 150 170 8
//   ./voxel_sim --world city --cell 0.4 --truth        # perfect-depth control
//   ./voxel_sim --world forest --general-only          # no A*, reactive only
//
// This is the file that answers "does the planner work", and it answers it the
// only way that means anything: by flying the thing and checking whether it hit
// something REAL. The aircraft plans against its own noisy voxel map; collisions
// are detected against VoxelWorld. Those are different data structures on
// purpose -- a harness that checks the plan against the map it planned with
// would pass no matter how wrong the map is.
//
// Always run --truth as a control. If the run fails with perfect depth, the
// planner is broken. If it succeeds with perfect depth and fails with stereo,
// the map is the limit and the number is a sensor result.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "ompl_planner.hpp"
#include "voxel_planner.hpp"
#include "voxel_traj.hpp"
#include "voxel_world.hpp"

using namespace sim;

// True clearance from the world, used ONLY for scoring -- never fed to the
// planner.
//
// EXACT, by scanning the actual voxels in a box around the point. The first
// version sampled 26 rays outward and took the first hit, and it was wrong in
// the same way the planner was: at r = 0.6 m the 26 sample points are ~0.6 m
// apart on the sphere, so a 0.2 m tree trunk sits between them and reads as
// clear. It reported 3.00 m of clearance one step before a collision 0.36 m
// away -- geometrically impossible, and the tell that the DETECTOR was broken
// rather than the planner.
//
// A collision detector that can miss obstacles makes every number the harness
// prints meaningless, so this one is exhaustive: 125 cells at 0.25 m and a
// 0.6 m radius, which costs nothing at sim rates.
static float trueClearance(const VoxelWorld& w, float x, float y, float z, float maxR) {
    int cx, cy, cz;
    w.worldToCell(x, y, z, cx, cy, cz);
    const int R = int(std::ceil(maxR / w.cell()));
    float best = maxR;
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (!w.solid(cx + dx, cy + dy, cz + dz)) continue;
                float wx, wy, wz;
                w.cellCentre(cx + dx, cy + dy, cz + dz, wx, wy, wz);
                // distance to the cell's nearest face, not its centre
                float ex = std::max(0.f, std::fabs(wx - x) - w.cell() * 0.5f);
                float ey = std::max(0.f, std::fabs(wy - y) - w.cell() * 0.5f);
                float ez = std::max(0.f, std::fabs(wz - z) - w.cell() * 0.5f);
                best = std::min(best, std::sqrt(ex * ex + ey * ey + ez * ez));
            }
    return best;
}

int main(int argc, char** argv) {
    std::string world = "forest", out = "/tmp/nav";
    int steps = 600;
    float cell = 0.25f, dt = 0.1f;
    float goalE = 150, goalN = 170, goalU = 8;
    bool useTruth = false, generalOnly = false, display = false;
    int replanEvery = 25;
    float lHit=-1, lMiss=-1, occT=-99, freeT=-99;
    // Pose drift injected into the pose the MAPPER believes, while the true
    // pose stays exact. This is what dead reckoning without SLAM looks like:
    // the aircraft flies correctly but writes its observations into the map at
    // slightly wrong places, smearing it. Measuring where that starts to hurt
    // is how you size a SLAM requirement instead of guessing at one.
    float driftMps = 0.f;      // metres of position error accumulated per second
    float driftDps = 0.f;      // degrees of yaw error accumulated per second
    // World seed. This was previously hard-coded to 1, which meant every run
    // ever shown was the SAME forest and the SAME city -- and every "it flew
    // 233 m" was one sample of one world dressed up as a property of the
    // planner. Exposing it is the precondition for any honest number here.
    unsigned seed = 1;
    std::string csvPath;       // per-step log, for offline analysis
    // Start on a forest trail and aim along it, rather than at a fixed corner.
    // Crossing a trail by accident tells you nothing; being placed on one and
    // asked to follow it is a different and much sharper question, because the
    // planner must reject wider openings off to the side in favour of the
    // corridor that actually leads somewhere.
    int trailRun = -1;         // -1 off, otherwise the trail index
    // Steering-behaviour knobs, exposed because they must be ABLATED rather
    // than reasoned about. Three plausible anti-churn mechanisms went in
    // together and the pair made progress worse (advance ratio 0.60 -> 0.47);
    // with no way to vary them one at a time that is an unusable measurement.
    float emaA = -1, dwellM = -1, revP = -1; int commitN = -1;
    // Reference-side steering. The reactive planner's commanded bearing churns
    // 20.9 deg/step; the GOAL bearing handed to it churns 21.5. It is tracking
    // a wobbling reference, not wobbling on its own, so these three act
    // upstream. All ablatable, because the last three ideas that sounded this
    // obvious were measured and turned off.
    float lookaheadM = 6.f;    // 0 = old "first waypoint past 3 m" carrot rule
    float goalEma    = 0.25f;  // 1 = no filtering of the reference bearing
    bool  reuse      = true;   // replan on demand instead of every --replan steps
    // 0 never, 1 only when progress stalls, 2 always. Default 1: the router
    // costs progress in open forest and is the only thing that can get out of a
    // dead end, so the honest answer is "when needed" rather than either
    // extreme.
    int   routerMode = 1;
    // Which reactive layer. The histogram answers "which bearing looks open"
    // and hands it to a vehicle that needs 0.35 s to turn; the library answers
    // "which path can I actually fly". Both, so they can be compared on
    // identical worlds rather than argued about.
    bool  useTraj = true;
    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--world")) world = next("forest");
        else if (!std::strcmp(argv[i], "--steps")) steps = std::atoi(next("600"));
        else if (!std::strcmp(argv[i], "--cell")) cell = float(std::atof(next("0.25")));
        else if (!std::strcmp(argv[i], "--out")) out = next("/tmp/nav");
        else if (!std::strcmp(argv[i], "--truth")) useTruth = true;
        else if (!std::strcmp(argv[i], "--general-only")) generalOnly = true;
        else if (!std::strcmp(argv[i], "--display")) display = true;
        else if (!std::strcmp(argv[i], "--lhit")) lHit = float(std::atof(next("0.85")));
        else if (!std::strcmp(argv[i], "--lmiss")) lMiss = float(std::atof(next("0.4")));
        else if (!std::strcmp(argv[i], "--occt")) occT = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--freet")) freeT = float(std::atof(next("-0.4")));
        else if (!std::strcmp(argv[i], "--seed")) seed = unsigned(std::atoi(next("1")));
        else if (!std::strcmp(argv[i], "--csv")) csvPath = next("");
        else if (!std::strcmp(argv[i], "--drift")) driftMps = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--driftyaw")) driftDps = float(std::atof(next("0")));
        else if (!std::strcmp(argv[i], "--replan")) replanEvery = std::atoi(next("25"));
        else if (!std::strcmp(argv[i], "--trail")) trailRun = std::atoi(next("0"));
        else if (!std::strcmp(argv[i], "--ema")) emaA = float(std::atof(next("0.3")));
        else if (!std::strcmp(argv[i], "--dwell")) dwellM = float(std::atof(next("0.12")));
        else if (!std::strcmp(argv[i], "--revpen")) revP = float(std::atof(next("1.2")));
        else if (!std::strcmp(argv[i], "--commit")) commitN = std::atoi(next("5"));
        else if (!std::strcmp(argv[i], "--lookahead")) lookaheadM = float(std::atof(next("6")));
        else if (!std::strcmp(argv[i], "--goalema")) goalEma = float(std::atof(next("0.25")));
        else if (!std::strcmp(argv[i], "--noreuse")) reuse = false;
        else if (!std::strcmp(argv[i], "--histogram")) useTraj = false;
        else if (!std::strcmp(argv[i], "--traj")) useTraj = true;
        else if (!std::strcmp(argv[i], "--router")) {
            const char* m = next("stall");
            routerMode = !std::strcmp(m, "never") ? 0
                       : !std::strcmp(m, "always") ? 2 : 1;
        }
        else if (!std::strcmp(argv[i], "--goal")) {
            goalE = float(std::atof(next("150")));
            goalN = float(std::atof(next("170")));
            goalU = float(std::atof(next("8")));
        }
    }

    // Deterministic sampling planner: same seed -> same flight.
    setPlannerSeed(unsigned(seed));
    VoxelWorld W;
    std::vector<Trail> trails;
    float px, py, pz;
    if (world == "city") {
        CityParams p; p.cell = cell; p.seed = seed; genCity(W, p);
        px = p.streetM * 0.5f; py = 5.f; pz = 6.f;
    } else {
        ForestParams p; p.cell = cell; p.seed = seed; genForest(W, p, &trails);
        px = 15.f; py = 10.f; pz = 6.f;
    }
    if (trailRun >= 0 && !trails.empty()) {
        const Trail& t = trails[size_t(trailRun) % trails.size()];
        px = t.front()[0]; py = t.front()[1];
        goalE = t.back()[0]; goalN = t.back()[1];
        printf("  trail %d: (%.0f,%.0f) -> (%.0f,%.0f), %zu vertices\n",
               trailRun, px, py, goalE, goalN, t.size());

        // CHECK THE INSTRUMENT BEFORE BELIEVING THE EXPERIMENT. A "trail
        // following" score is meaningless if the trail is not actually clear
        // end to end -- one stem left standing in it would make a perfect
        // follower look like a failure, and this project has already shipped
        // two conclusions that were really instrument faults. So walk the
        // centreline and report the worst clearance on it before flying.
        //
        // And pick the altitude by measurement rather than by assumption. The
        // first version fixed it at 4.5 m and the check immediately reported
        // 1.00 m of corridor clearance -- which was not a tree in the trail at
        // all but the GROUND, since the terrain undulates by +-1.7 m and the
        // floor sits 2 m above that. The trail was exactly as clear as designed;
        // the aircraft was simply being flown too low over a hill.
        auto corridor = [&](float z, float& worst, float& mean) {
            worst = 1e9f; float sum = 0; int nS = 0;
            for (size_t i = 0; i + 1 < t.size(); ++i) {
                float ax=t[i][0], ay=t[i][1], bx=t[i+1][0], by=t[i+1][1];
                int nSeg = std::max(1, int(std::hypot(bx-ax, by-ay) / 0.5f));
                for (int k = 0; k <= nSeg; ++k) {
                    float u = float(k)/nSeg;
                    float c = trueClearance(W, ax+(bx-ax)*u, ay+(by-ay)*u, z, 3.0f);
                    worst = std::min(worst, c); sum += c; ++nS;
                }
            }
            mean = sum / std::max(1, nS);
        };
        // Print the whole profile, not just the winner. "It chose 4.5 m" tells
        // you nothing about why; "clearance falls off above 6 m" tells you the
        // canopy is the limit and the corridor is a tunnel, not a slot.
        float bestZ = 4.5f, bestW = -1, bestM = 0;
        printf("  corridor clearance by altitude:");
        for (float z = 3.5f; z <= 10.5f; z += 1.0f) {
            float w, m; corridor(z, w, m);
            printf("  %.1f m:%.2f", z, w);
            if (w > bestW) { bestW = w; bestM = m; bestZ = z; }
        }
        printf("\n");
        pz = bestZ;
        goalU = pz;
        printf("  trail corridor at %.1f m altitude: min clearance %.2f m, mean %.2f m\n",
               pz, bestW, bestM);
        if (bestW < 1.4f)
            printf("  !! corridor is narrower than designed -- the trail-following "
                   "score below is not trustworthy\n");
    }
    // VALIDATE THE SPAWN before blaming the planner for anything. A fixed start
    // point in a procedurally generated forest lands inside a tree often enough
    // that "collision at 0.4 m travelled" is far more likely to be a bad initial
    // condition than a planning failure -- and a harness that cannot tell those
    // apart is worse than no harness.
    {
        float c0 = trueClearance(W, px, py, pz, 3.0f);
        if (c0 < 1.5f) {
            printf("  spawn clearance only %.2f m -- searching for a clear start\n", c0);
            bool ok = false;
            for (float rad = 1.f; rad <= 25.f && !ok; rad += 1.f)
                for (int a = 0; a < 24 && !ok; ++a)
                    for (float dzs = 0.f; dzs <= 8.f && !ok; dzs += 1.f) {
                        float th = a * sim::PI_F / 12.f;
                        float tx = px + rad * std::cos(th), ty = py + rad * std::sin(th),
                              tz = pz + dzs;
                        if (trueClearance(W, tx, ty, tz, 3.0f) >= 1.5f) {
                            px = tx; py = ty; pz = tz; ok = true;
                        }
                    }
            if (!ok) { printf("  !! no clear spawn found within 25 m -- aborting\n"); return 3; }
        }
        printf("  spawn clearance %.2f m at (%.1f,%.1f,%.1f)\n",
               trueClearance(W, px, py, pz, 3.0f), px, py, pz);
    }
    printf("  seed %u\n", seed);
    printf("world '%s' %dx%dx%d @ %.2f m   start (%.1f,%.1f,%.1f) -> goal (%.0f,%.0f,%.0f)\n",
           world.c_str(), W.nx(), W.ny(), W.nz(), W.cell(), px, py, pz, goalE, goalN, goalU);

    CamParams cp; DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = cell;
    if (lHit  > 0)   mp.lHit = lHit;
    if (lMiss > 0)   mp.lMiss = lMiss;
    if (occT  > -90) mp.occThresh = occT;
    if (freeT > -90) mp.freeThresh = freeT;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    VoxelMap M; M.init(mp, px, py, pz);   // after spawn validation, not before

    GeneralParams gp; gp.robotR = 0.6f;
    if (emaA    >= 0) gp.fieldEma     = emaA;
    if (dwellM  >= 0) gp.switchMargin = dwellM;
    if (revP    >= 0) gp.revPenalty   = revP;
    if (commitN >= 0) gp.commitSteps  = commitN;
    GeneralPlanner gen(gp);
    TrajParams tp;
    tp.robotR = gp.robotR; tp.vMax = gp.vMax;
    tp.decelMs2 = gp.decelMs2; tp.reactS = gp.reactS; tp.minFreeM = gp.minFreeM;
    tp.dt = dt;              // the rollout must use the control period
    TrajectoryPlanner traj(tp);
    if (useTraj)
        printf("  reactive layer: trajectory library, %zu primitives\n", traj.librarySize());
    else
        printf("  reactive layer: openness histogram, %d x %d bins\n", gp.nAz, gp.nEl);
    ForwardParams fwp; fwp.robotR = gp.robotR;
    ForwardPath path;
    BearingFilter gfilt;
    StallMonitor stall;
    printf("  forward planner: %s\n", forwardPlannerName());

    float vx = 0, vy = 0, vz = 0, yaw = 0;
    float travelled = 0, minClear = 1e9f;
    // Trail-following score. Distance from the corridor centreline, not distance
    // to the goal: a run can end near the goal having ignored the trail
    // completely, and that is a different (easier) achievement.
    float trailDev = 0; long trailN = 0, trailIn = 0;
    int collisions = 0, stopped = 0, noPath = 0, replans = 0;
    int stepsRun = 0;
    bool reached = false;
    std::vector<cv::Point2f> trail;
    double tPlan = 0, tSense = 0, tGen = 0, tPrec = 0, tInteg = 0;
    int nPrec = 0;

    const float startDist = std::hypot(goalE - px, goalN - py);
    FILE* csv = csvPath.empty() ? nullptr : std::fopen(csvPath.c_str(), "w");
    if (csv) std::fprintf(csv, "step,e,n,u,yaw,speed,freeM,openM,blocked,"
                               "pathFound,pathWp,trueClear,distToGoal,"
                               "cmdAz,goalAz,src\n");

#if SIM_HAVE_HIGHGUI
    // Static truth backdrop for the live top-down view, rendered once.
    cv::Mat truthTop;
    if (display) {
        cv::namedWindow("kestrel voxel sim", cv::WINDOW_AUTOSIZE);
        truthTop = cv::Mat(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250, 248, 245));
    }
    bool paused = false;
#endif

    for (int s = 0; s < steps; ++s) {
        stepsRun = s + 1;
        // --- sense -----------------------------------------------------------
        // TRUE pose -- what the camera actually observes from.
        CamPose pose; pose.e = px; pose.n = py; pose.u = pz;
        pose.yawDeg = yaw; pose.pitchDeg = -5.f; pose.rollDeg = 0.f;
        // BELIEVED pose -- what the mapper thinks it observed from. Identical
        // unless drift is injected. A random walk, not a constant bias, because
        // dead-reckoning error is integrated noise.
        static float dE = 0, dN = 0, dU = 0, dY = 0;
        if (driftMps > 0.f || driftDps > 0.f) {
            auto rnd = []() { return (float(rand()) / RAND_MAX) * 2.f - 1.f; };
            float sd = driftMps * std::sqrt(dt);
            dE += rnd() * sd; dN += rnd() * sd; dU += rnd() * sd * 0.5f;
            dY += rnd() * driftDps * std::sqrt(dt);
        }
        CamPose mpose = pose;
        mpose.e += dE; mpose.n += dN; mpose.u += dU; mpose.yawDeg += dY;
        int64 t0 = cv::getTickCount();
        cv::Mat d = useTruth ? cam.renderTruth(W, pose) : cam.renderStereo(W, pose, nullptr);
        int64 tm = cv::getTickCount();
        M.integrate(d, cam, mpose);   // believed pose, not true pose
        tInteg += double(cv::getTickCount() - tm) / cv::getTickFrequency();
        tSense += double(cv::getTickCount() - t0) / cv::getTickFrequency();
        M.recentre(px + dE, py + dN, pz + dU);

        // --- precise plan, occasionally ---------------------------------------
        int64 t1 = cv::getTickCount();
        // Plan AHEAD along the mission bearing, not to the distant goal.
        float mAz = std::atan2(goalE - px, goalN - py) * 180.f / sim::PI_F;
        float mEl = std::atan2(goalU - pz,
                               std::hypot(goalE - px, goalN - py)) * 180.f / sim::PI_F;
        // IS THE ROUTER WANTED AT ALL RIGHT NOW? Following a routed path is
        // measurably worse than pointing at the goal in open forest -- see the
        // table in ompl_planner.hpp -- but a 12 m reactive horizon cannot see
        // out of a dead end. So run reactive, and call the router only once
        // progress has actually stalled.
        stall.update(std::hypot(goalE - px, goalN - py));
        bool wantRouter = !generalOnly &&
            (routerMode == 2 || (routerMode == 1 && stall.engaged));
        if (!wantRouter) path = ForwardPath();     // drop it; aim at the goal

        // REPLAN ON DEMAND, not on a timer. Replanning every N steps with a
        // randomised planner threw away a perfectly good path and got back an
        // arbitrarily different one, which is most of why the reference bearing
        // churned. `--replan N` is now the FALLBACK period, not the period.
        bool needReplan = wantRouter &&
            (reuse ? !pathStillGood(M, path, px + dE, py + dN, pz + dU, mAz, fwp)
                   : (s % replanEvery == 0));
        if (needReplan) {
            int64 tp = cv::getTickCount();
            path = planForward(M, px + dE, py + dN, pz + dU, mAz, mEl, fwp);
            tPrec += double(cv::getTickCount() - tp) / cv::getTickFrequency(); ++nPrec;
            ++replans;
            if (!path.found) ++noPath;
        }
        // Where on the path to aim. If there is no path, fall back to the
        // straight-line goal bearing -- the reactive layer still keeps us safe,
        // we just explore rather than follow a route.
        float tgtE = goalE, tgtN = goalN, tgtU = goalU;
        if (path.found) {
            if (lookaheadM > 0) {
                pursuitPoint(path, px, py, pz, lookaheadM, tgtE, tgtN, tgtU);
            } else {
                // Original carrot rule, kept only so it can be ablated.
                for (const auto& w : path.pts) {
                    float dd = std::hypot(w[0] - px, w[1] - py);
                    if (dd > 3.0f) { tgtE = w[0]; tgtN = w[1]; tgtU = w[2]; break; }
                }
            }
        }
        float rawAz = std::atan2(tgtE - px, tgtN - py) * 180.f / sim::PI_F;
        float rawEl = std::atan2(tgtU - pz, std::hypot(tgtE - px, tgtN - py)) * 180.f / sim::PI_F;
        gfilt.update(rawAz, rawEl, goalEma);
        float gAz = gfilt.azDeg, gEl = gfilt.elDeg;

        // --- general plan, every step ----------------------------------------
        int64 tg = cv::getTickCount();
        GeneralResult gr = useTraj
            ? traj.plan(M, px + dE, py + dN, pz + dU, yaw, gAz, gEl)
            : gen.plan(M, px + dE, py + dN, pz + dU, gAz, gEl);
        tGen += double(cv::getTickCount() - tg) / cv::getTickFrequency();
        tPlan += double(cv::getTickCount() - t1) / cv::getTickFrequency();
        if (gr.speed <= 0.01f) ++stopped;

        // --- move -------------------------------------------------------------
        float dx, dy, dz;
        {
            float a = gr.azDeg * sim::PI_F / 180.f, e = gr.elDeg * sim::PI_F / 180.f;
            dx = std::cos(e) * std::sin(a); dy = std::cos(e) * std::cos(a); dz = std::sin(e);
        }
        // First-order lag toward the commanded velocity: an aircraft cannot
        // change direction instantly, and a planner that assumes it can will
        // look far better in sim than in the air.
        const float tau = 0.35f;
        float k = std::min(1.f, dt / tau);
        vx += (dx * gr.speed - vx) * k;
        vy += (dy * gr.speed - vy) * k;
        vz += (dz * gr.speed - vz) * k;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        travelled += std::sqrt(vx * vx + vy * vy + vz * vz) * dt;
        if (std::hypot(vx, vy) > 0.2f) yaw = std::atan2(vx, vy) * 180.f / sim::PI_F;
        trail.push_back({px, py});
        if (trailRun >= 0 && !trails.empty()) {
            const Trail& t = trails[size_t(trailRun) % trails.size()];
            float best = 1e9f;
            for (size_t i = 0; i + 1 < t.size(); ++i) {
                float ax=t[i][0], ay=t[i][1], vxs=t[i+1][0]-ax, vys=t[i+1][1]-ay;
                float l2 = vxs*vxs + vys*vys;
                float u = l2 > 1e-9f ? ((px-ax)*vxs + (py-ay)*vys)/l2 : 0.f;
                u = std::max(0.f, std::min(1.f, u));
                best = std::min(best, std::hypot(px-(ax+u*vxs), py-(ay+u*vys)));
            }
            trailDev += best; ++trailN;
            if (best <= 2.5f) ++trailIn;    // within the corridor plus a margin
        }

        // --- score against TRUTH ---------------------------------------------
        float clr = trueClearance(W, px, py, pz, 2.0f);
        minClear = std::min(minClear, clr);
        if (clr <= gp.robotR * 0.5f) {
            ++collisions;
            printf("  !! COLLISION at step %d, (%.1f, %.1f, %.1f), clearance %.2f m\n",
                   s, px, py, pz, clr);
            break;
        }
        if (std::hypot(goalE - px, goalN - py) < 4.f && std::fabs(goalU - pz) < 4.f) {
            reached = true;
            printf("  reached goal at step %d\n", s);
            break;
        }
#if SIM_HAVE_HIGHGUI
        if (display) {
            // Repaint the truth slice at the CURRENT flight height each frame --
            // the world is 3D, so a fixed-height backdrop would be misleading.
            int zc; { int a, b; W.worldToCell(0, 0, pz, a, b, zc); }
            truthTop.setTo(cv::Scalar(250, 248, 245));
            for (int y = 0; y < W.ny(); ++y)
                for (int x = 0; x < W.nx(); ++x)
                    if (W.solid(x, y, zc))
                        truthTop.at<cv::Vec3b>(W.ny() - 1 - y, x) = cv::Vec3b(70, 70, 70);
            for (size_t i = 1; i < trail.size(); ++i) {
                int x0, y0, z0, x1, y1, z1;
                W.worldToCell(trail[i-1].x, trail[i-1].y, pz, x0, y0, z0);
                W.worldToCell(trail[i].x, trail[i].y, pz, x1, y1, z1);
                cv::line(truthTop, {x0, W.ny()-1-y0}, {x1, W.ny()-1-y1}, {40,40,220}, 2);
            }
            if (path.found)
                for (size_t i = 1; i < path.pts.size(); ++i) {
                    int x0,y0,z0,x1,y1,z1;
                    W.worldToCell(path.pts[i-1][0], path.pts[i-1][1], pz, x0,y0,z0);
                    W.worldToCell(path.pts[i][0], path.pts[i][1], pz, x1,y1,z1);
                    cv::line(truthTop, {x0, W.ny()-1-y0}, {x1, W.ny()-1-y1}, {40,170,40}, 1);
                }
            { int gx,gy,gz; W.worldToCell(goalE, goalN, pz, gx,gy,gz);
              cv::circle(truthTop, {gx, W.ny()-1-gy}, 9, {40,180,40}, 2); }
            cv::Mat topV; cv::resize(truthTop, topV, cv::Size(480,480), 0,0, cv::INTER_AREA);
            cv::Mat sliceV = M.sliceImage(pz, 480);
            cv::Mat depthV(480, 480, CV_8UC3, cv::Scalar(60,60,60));
            { cv::Mat dv(d.rows, d.cols, CV_8UC3, cv::Scalar(60,60,60));
              for (int y=0;y<d.rows;++y) for (int x=0;x<d.cols;++x) {
                  float r = d.at<float>(y,x); if(!(r>0.f)) continue;
                  float f = std::min(1.f, r/cp.maxRangeM);
                  dv.at<cv::Vec3b>(y,x) = cv::Vec3b(uchar(255*(1-f)), uchar(80+100*f), uchar(255*f)); }
              cv::resize(dv, depthV, cv::Size(480,480), 0,0, cv::INTER_NEAREST); }
            cv::putText(topV,  "TRUTH + flown path", {10,22}, cv::FONT_HERSHEY_SIMPLEX,0.6,{30,30,30},2);
            cv::putText(sliceV,"VOXEL MAP slice",   {10,22}, cv::FONT_HERSHEY_SIMPLEX,0.6,{30,30,30},2);
            cv::putText(depthV,useTruth?"DEPTH (truth)":"DEPTH (stereo)", {10,22},
                        cv::FONT_HERSHEY_SIMPLEX,0.6,{240,240,240},2);
            cv::Mat row; cv::hconcat(std::vector<cv::Mat>{topV, sliceV, depthV}, row);
            char hud[240];
            std::snprintf(hud, sizeof hud,
                "[%s seed %u] step %d  pos %.0f,%.0f,%.1f  v %.2f m/s  free %.2f m  open %.2f m  %s  path %s  [space]=pause [q]=quit",
                world.c_str(), seed, s, px, py, pz, std::hypot(vx,vy), gr.freeM, gr.openM,
                gr.blocked?"BLOCKED":"ok", path.found?"yes":"NONE");
            cv::Mat bar(34, row.cols, CV_8UC3, cv::Scalar(25,25,30));
            cv::putText(bar, hud, {10,23}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {235,235,235}, 1);
            cv::Mat full; cv::vconcat(row, bar, full);
            cv::imshow("kestrel voxel sim", full);
            int key = cv::waitKey(paused ? 0 : 1);
            if (key == 'q' || key == 27) break;
            if (key == ' ') paused = !paused;
        }
#endif
        if (csv) std::fprintf(csv, "%d,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%d,%d,%zu,%.3f,%.2f,"
                                   "%.2f,%.2f,%d\n",
                              s, px, py, pz, yaw, std::hypot(vx, vy), gr.freeM, gr.openM,
                              gr.blocked ? 1 : 0, path.found ? 1 : 0, path.pts.size(),
                              trueClearance(W, px, py, pz, 3.0f),
                              std::hypot(goalE - px, goalN - py),
                              gr.azDeg, gAz, int(gr.src));
        if (s % 40 == 0)
            printf("  step %4d  pos (%6.1f,%6.1f,%5.1f)  v %.2f  cmd %.2f  free %.2f  "
                   "open %.2f  %s  path %s(%zu wp)\n", s, px, py, pz,
                   std::hypot(vx, vy), gr.speed, gr.freeM, gr.openM,
                   gr.blocked ? "BLOCKED" : "ok    ",
                   path.found ? "" : "NONE ", path.pts.size());
    }

    // --- report -------------------------------------------------------------
    const float endDist = std::hypot(goalE - px, goalN - py);
    printf("\n--- %s depth, %s ---\n",
           useTruth ? "GROUND-TRUTH" : "simulated stereo",
           generalOnly ? "reactive only" : "reactive + A*");
    printf("  outcome            %s\n",
           collisions ? "COLLIDED" : (reached ? "reached goal" : "ran out of steps"));
    printf("  distance to goal   %.1f m  (started %.1f m away)\n", endDist, startDist);
    printf("  path travelled     %.1f m   (straight line %.1f m, ratio %.2f)\n",
           travelled, startDist, travelled / std::max(1.f, startDist));
    printf("  min true clearance %.2f m   <- scored against the WORLD, not the map\n", minClear);
    if (trailN > 0)
        printf("  trail following    %.0f%% of steps within 2.5 m of the corridor, "
               "mean deviation %.1f m\n",
               100.0 * double(trailIn) / double(trailN), trailDev / float(trailN));
    printf("  stopped on         %d of %d steps\n", stopped, steps);
    printf("  A* replans         %d, of which no path %d\n", replans, noPath);
    // The stall monitor runs in every mode, so under --router never this
    // reports when the router WOULD have been called, not when it was. Saying
    // "engaged 12%" for a run that never engaged it would be a lie of exactly
    // the kind this harness exists to avoid.
    {
        double pct = stepsRun ? 100.0 * double(stall.engagedSteps) / stepsRun : 0.0;
        if (routerMode == 0)
            printf("  router             never used; stall condition met on %.0f%% of steps\n", pct);
        else if (routerMode == 2)
            printf("  router             always on\n");
        else
            printf("  router             on stall, engaged %d time(s) for %ld of %d steps (%.0f%%)\n",
                   stall.engagements, stall.engagedSteps, stepsRun, pct);
    }
    // Split out what would actually run ON THE AIRCRAFT. The depth RENDER is
    // sim-only -- on the Pi that is the stereo matcher, benchmarked separately.
    // Map integration and both planners are real onboard cost.
    int nsteps = std::max(1, (int)trail.size());
    printf("  --- onboard cost (per step unless noted) ---\n");
    printf("  map integrate      %6.2f ms\n", 1000 * tInteg / nsteps);
    printf("  general planner    %6.2f ms\n", 1000 * tGen / nsteps);
    printf("  forward planner    %6.2f ms per replan (%d replans, every %d steps)\n",
           nPrec ? 1000 * tPrec / nPrec : 0.0, nPrec, replanEvery);
    printf("  ONBOARD TOTAL      %6.2f ms/step amortised\n",
           1000 * (tInteg + tGen + tPrec) / nsteps);
    printf("  [sim-only] depth render %.1f ms/step\n",
           1000 * (tSense - tInteg) / nsteps);

    VoxelMap::Score sc = M.score(W, px, py, pz, 25.f, 30.f);
    printf("  map false-free     %.3f%%\n", 100.0 * sc.falseFreeRate());

    // Top-down: truth occupancy at flight height, plus the flown trail.
    cv::Mat top(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250, 248, 245));
    int zc; { int a, b; W.worldToCell(0, 0, pz, a, b, zc); }
    for (int y = 0; y < W.ny(); ++y)
        for (int x = 0; x < W.nx(); ++x)
            if (W.solid(x, y, zc))
                top.at<cv::Vec3b>(W.ny() - 1 - y, x) = cv::Vec3b(70, 70, 70);
    for (size_t i = 1; i < trail.size(); ++i) {
        int x0, y0, z0, x1, y1, z1;
        W.worldToCell(trail[i - 1].x, trail[i - 1].y, pz, x0, y0, z0);
        W.worldToCell(trail[i].x, trail[i].y, pz, x1, y1, z1);
        cv::line(top, {x0, W.ny() - 1 - y0}, {x1, W.ny() - 1 - y1}, {40, 40, 220}, 2);
    }
    { int gx, gy, gz; W.worldToCell(goalE, goalN, pz, gx, gy, gz);
      cv::circle(top, {gx, W.ny() - 1 - gy}, 8, {40, 180, 40}, 2); }
    cv::Mat topOut; cv::resize(top, topOut, cv::Size(700, 700), 0, 0, cv::INTER_AREA);
    cv::putText(topOut, collisions ? "COLLIDED" : (reached ? "REACHED GOAL" : "TIMEOUT"),
                {14, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                collisions ? cv::Scalar(30, 30, 220) : cv::Scalar(30, 140, 30), 2);
    cv::imwrite(out + "_top.png", topOut);
    cv::imwrite(out + "_slice.png", M.sliceImage(pz));
    if (csv) { std::fclose(csv); printf("  wrote %s\n", csvPath.c_str()); }
    printf("  wrote %s_top.png (flown path over truth) and %s_slice.png\n",
           out.c_str(), out.c_str());
    return collisions ? 2 : (reached ? 0 : 1);
}
