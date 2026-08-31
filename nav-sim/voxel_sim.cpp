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
#include "depth_improve.hpp"
#include "voxel_map.hpp"
#include "ompl_planner.hpp"
#include "voxel_planner.hpp"
#include "voxel_traj.hpp"
#include "imu_odometry.hpp"
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

// Wrap to (-180,180]. The dump stores the bearing RELATIVE TO THE NOSE, because
// a world-frame bearing is unlearnable from an egocentric image -- the same
// picture means "turn left" or "turn right" depending on a heading the model
// cannot see.
static inline float wrapDeg180(float d) { return std::fmod(d + 540.f, 360.f) - 180.f; }

int main(int argc, char** argv) {
    std::string world = "forest", out = "/tmp/nav", pointsFile;
    // VOXEL ONLY. Plan straight against a saved map, with no camera in the
    // loop at all -- no stereo render, no integration, no sensing.
    //
    // WHY IT IS WORTH HAVING. Loading a saved space as a WORLD and then flying
    // it normally puts the geometry through a depth model a second time: it was
    // already built from noisy depth when it was captured, and the sim then
    // renders synthetic stereo from it and rebuilds a map from that. Two rounds
    // of dropout, range limit and occlusion on geometry that only ever had one.
    // If the question is "does the planner handle this shape", that is the
    // wrong instrument.
    //
    // WHAT IT GIVES UP, and it is not small. With no sensing loop the map never
    // changes and is complete from the first step -- so this is a KNOWN-MAP
    // planning problem, which is not the problem the aircraft has. And because
    // the map and the collision truth are then the same data, the harness can
    // no longer catch a mapping error: it is testing the planner and nothing
    // else. The normal path stays the one that answers "will this fly".
    bool voxelOnly = false;
    int steps = 600;
    float cell = 0.25f, dt = 0.1f;
    int   camW = 320, camH = 240;
    float hfov = 70.f, baseline = 0.12f, maxIntegOverride = -1.f;
    bool  mixed = false, lanes = false;
    float farCell = 2.0f; int farMode = 1; bool useMid = false;
    float vmax = -1.f;
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
    // Trunk visibility to stereo, swept rather than guessed. We set this from
    // ONE screenshot of somebody else's depth output, of unknown vintage and
    // unknown pipeline. A point estimate from that is not evidence; a
    // TOLERANCE CURVE across the plausible range is, because it answers "how
    // visible do trunks have to be for this to work" -- which is a spec that
    // can be checked with a real camera.
    float trunkTex = -1.f;    // <0 = use the world default range
    // IR projector. Every result in this tree was measured with it off, i.e.
    // "passive stereo in adequate light" -- which is right for open daylight
    // and wrong for closed canopy at dusk, a large part of the real envelope.
    bool  emitterOn = false;
    float ambientIR = 1.0f;   // 1 = bright daylight, 0 = dark
    // The two knobs the safety fixes introduced, exposed so they can be swept
    // rather than asserted. Both were picked by reasoning and neither has been
    // measured against the progress they cost.
    float coreFrac  = -1.f;
    float climbPen = -1.f;   // <0 = leave TrajParams default
    float horizonS = -1.f;
    float rollCap  = -1.f;
    float yawRateLim = -1.f;  // deg/s; <0 = instant (the original, unphysical)
    float imuAttErr  = -1.f;  // deg of tilt error for the IMU odometry probe
    float flowRelErr = -1.f;  // optical-flow velocity error, fraction of speed
    float freeMargin = -1.f;
    std::string dumpNN;
    int dumpView = -1;   // step at which to write the 4-pane view; <0 = off
    // Pane edge in pixels for that view. 480 is fine on a screen and too coarse
    // for a figure -- the four-pane grid is 2x this, and a document wants the
    // voxel FPV legible rather than merely present.
    int viewPx   = 480;
    float sinkPen  = -1.f;
    int   carveWin  = -1;     // <0 = map default
    // Which reactive layer. The histogram answers "which bearing looks open"
    // and hands it to a vehicle that needs 0.35 s to turn; the library answers
    // "which path can I actually fly". Both, so they can be compared on
    // identical worlds rather than argued about.
    bool  useTraj = true;
    // Coarse far-field companion map -- see where it is constructed.
    bool  useFar = true;
    // Depth improver (depth_improve.hpp) and yaw-coupled sideslip
    // (TrajParams::latSlipDeg). Both OFF by default: each is an unproven idea
    // taken from a paper, and the point of putting them behind a flag is that
    // the arm without them is the same binary rather than a different one.
    bool  improve = false;
    DepthImproveParams dip;
    float latSlip = 0.f, latKnee = -1.f;
    // The router's search cone about the goal bearing. 75 deg means it can
    // never plan a path that goes BACKWARDS -- which is exactly the manoeuvre
    // escaping a dead end requires. Exposed to test that.
    float coneDeg = -1.f;
    float unkCost = -1.f;      // <0 keeps GeneralParams' own default
    // SWEPT RADIUS. 0.6 m means a 1.2 m vehicle, which is a large quadcopter --
    // and it is the term that decides whether MORE MAP helps or hurts, because
    // an OCCUPIED cell blocks anywhere inside this ball while an UNKNOWN one
    // blocks only on the centre line. Extending the marking range therefore
    // grows the blocking volume from a line to a ball, and how much that costs
    // depends entirely on how big the ball is.
    float robotR = 0.6f;
    bool  robotSet = false, cellSet = false;
    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--world")) world = next("forest");
        else if (!std::strcmp(argv[i], "--points")) pointsFile = next("");
        else if (!std::strcmp(argv[i], "--voxelonly")) voxelOnly = true;
        else if (!std::strcmp(argv[i], "--steps")) steps = std::atoi(next("600"));
        else if (!std::strcmp(argv[i], "--cell")) { cell = float(std::atof(next("0.25"))); cellSet = true; }
        else if (!std::strcmp(argv[i], "--mixed")) mixed = true;
        else if (!std::strcmp(argv[i], "--lanes")) lanes = true;
        else if (!std::strcmp(argv[i], "--farcell")) farCell = float(std::atof(next("2.0")));
        else if (!std::strcmp(argv[i], "--farmode")) farMode = std::atoi(next("1"));
        else if (!std::strcmp(argv[i], "--mid")) useMid = true;
        else if (!std::strcmp(argv[i], "--improve")) improve = true;
        else if (!std::strcmp(argv[i], "--impnear")) { improve = true; dip.nearM = float(std::atof(next("2.0"))); }
        else if (!std::strcmp(argv[i], "--imprad")) { improve = true; dip.radiusPx = std::atoi(next("4")); }
        else if (!std::strcmp(argv[i], "--impseed")) { improve = true; dip.minSeeds = std::atoi(next("6")); }
        else if (!std::strcmp(argv[i], "--slip")) latSlip = float(std::atof(next("20")));
        else if (!std::strcmp(argv[i], "--slipknee")) latKnee = float(std::atof(next("40")));
        else if (!std::strcmp(argv[i], "--vmax")) vmax = float(std::atof(next("3.0")));
        else if (!std::strcmp(argv[i], "--camw")) camW = std::atoi(next("320"));
        else if (!std::strcmp(argv[i], "--camh")) camH = std::atoi(next("240"));
        else if (!std::strcmp(argv[i], "--hfov")) hfov = float(std::atof(next("70")));
        else if (!std::strcmp(argv[i], "--baseline")) baseline = float(std::atof(next("0.12")));
        else if (!std::strcmp(argv[i], "--maxinteg")) maxIntegOverride = float(std::atof(next("-1")));
        else if (!std::strcmp(argv[i], "--unkcost")) unkCost = float(std::atof(next("0.45")));
        else if (!std::strcmp(argv[i], "--robot")) { robotR = float(std::atof(next("0.6"))); robotSet = true; }
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
        else if (!std::strcmp(argv[i], "--nofar")) useFar = false;
        else if (!std::strcmp(argv[i], "--cone")) coneDeg = float(std::atof(next("75")));
        else if (!std::strcmp(argv[i], "--traj")) useTraj = true;
        else if (!std::strcmp(argv[i], "--trunktex")) trunkTex = float(std::atof(next("0.5")));
        else if (!std::strcmp(argv[i], "--emitter")) emitterOn = true;
        else if (!std::strcmp(argv[i], "--ambient")) { emitterOn = true; ambientIR = float(std::atof(next("0.3"))); }
        else if (!std::strcmp(argv[i], "--corefrac")) coreFrac = float(std::atof(next("0.65")));
        else if (!std::strcmp(argv[i], "--climbpen")) climbPen = float(std::atof(next("6.0")));
        else if (!std::strcmp(argv[i], "--horizon")) horizonS = float(std::atof(next("2.0")));
        else if (!std::strcmp(argv[i], "--rollcap")) rollCap = float(std::atof(next("3.6")));
        else if (!std::strcmp(argv[i], "--yawrate")) yawRateLim = float(std::atof(next("100")));
        else if (!std::strcmp(argv[i], "--imuatt")) imuAttErr = float(std::atof(next("1.0")));
        else if (!std::strcmp(argv[i], "--flow")) flowRelErr = float(std::atof(next("0.05")));
        else if (!std::strcmp(argv[i], "--freemargin")) freeMargin = float(std::atof(next("1.0")));
        else if (!std::strcmp(argv[i], "--dumpnn")) dumpNN = next("/tmp/nn.csv");
        else if (!std::strcmp(argv[i], "--dumpview")) dumpView = std::atoi(next("0"));
        else if (!std::strcmp(argv[i], "--viewpx"))   viewPx   = std::atoi(next("480"));
        else if (!std::strcmp(argv[i], "--sinkpen")) sinkPen = float(std::atof(next("2.0")));
        else if (!std::strcmp(argv[i], "--carvewin")) carveWin = std::atoi(next("5"));
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
    bool loadedPoints = false;
    std::vector<Trail> trails;
    float px, py, pz;
    if (world == "lidar" || world == "saved") {
        // A SPACE SAVED FROM REAL DATA. voxel_live writes one with 'p'; this
        // flies the planner through it, deterministically and as often as you
        // like. The point of the whole exercise: a walk becomes a fixture.
        // fillGround FALSE, and the default is wrong for this input. The
        // loader normally closes the ground beneath the lowest return in each
        // column, which is right for an aerial lidar survey and invents
        // geometry for a MAP: a map is a partial observation from inside, and
        // everything it never saw would become solid rock. Measured on the
        // first round trip -- a hedge scan reloaded as a near-solid block.
        if (!loadLidarXyz(W, pointsFile, cell, false)) {
            std::fprintf(stderr, "--world lidar needs --points FILE.xyz\n");
            return 1;
        }
        // Spawn at the middle of the loaded extent and aim across it. Nothing
        // clever: the validator below moves it if that lands inside geometry.
        px = W.nx() * cell * 0.5f; py = W.ny() * cell * 0.2f;
        pz = W.nz() * cell * 0.5f;
        goalE = px; goalN = W.ny() * cell * 0.9f; goalU = pz;
        loadedPoints = true;
    } else if (world == "culdesac") {
        // Spawn SOUTH of the pocket, goal NORTH beyond it, so the straight
        // line to the goal runs in through the mouth and into the closed end.
        // Escaping means abandoning the goal bearing and going around.
        CulDeSacParams p; p.cell = cell; p.seed = seed; genCulDeSac(W, p);
        px = p.sizeM * 0.5f; py = 30.f; pz = 8.f;
        goalE = p.sizeM * 0.5f; goalN = p.sizeM - 20.f; goalU = 8.f;
    } else if (world == "city") {
        CityParams p; p.cell = cell; p.seed = seed; genCity(W, p);
        px = p.streetM * 0.5f; py = 5.f; pz = 6.f;
        goalE = p.streetM * 0.5f; goalN = p.sizeM - 10.f; goalU = 6.f;
    } else if (world == "indoor") {
        // Fly the diagonal: the straight line to the goal crosses every internal
        // wall, so the route is a sequence of doorways rather than one corridor.
        //
        // TRUTH RESOLUTION IS ITS OWN CONCERN, as in genHedgeRow. A 0.12 m wall
        // and a 0.9 m door do not exist at 0.25 m, and the first run of this
        // world proved it: built at 0.25 m the spawn validator could find no
        // clear cell inside the house and put the aircraft at 5.2 m, above a
        // 2.4 m ceiling. The world was not the problem; the resolution was.
        IndoorParams p; p.cell = 0.05f; p.seed = seed;
        genIndoor(W, p);
        // And a 0.6 m radius aircraft cannot pass a 0.9 m door -- 1.2 m of
        // diameter through 0.9 m of opening. Indoors is a small-airframe
        // problem, so default to one unless told otherwise.
        if (!robotSet) robotR = 0.20f;
        if (!cellSet)  cell   = 0.10f;
        const int nrm = std::max(2, int(p.sizeM / p.roomM));
        const float rp = p.sizeM / float(nrm);
        px = rp * 0.5f; py = rp * 0.5f; pz = 1.1f;   // centre of the first room
        goalE = p.sizeM - 1.5f; goalN = p.sizeM - 1.5f; goalU = 1.1f;
    } else if (world == "road") {
        RoadParams p; p.cell = 0.10f; p.seed = seed;
        genRoad(W, p);
        if (!robotSet) robotR = 0.35f;
        px = p.widthM * 0.5f; py = 3.f; pz = 2.0f;
        goalE = p.widthM * 0.5f; goalN = p.lengthM - 5.f; goalU = 2.0f;
    } else {
        ForestParams p; p.cell = cell; p.seed = seed;
        if (lanes) {
            // Thickets and clear lanes side by side. A goal straight ahead is
            // reachable through either; the open lane costs path length and saves
            // risk. Trails OFF, so the clear route is a DENSITY feature the
            // planner has to find, not a carved corridor handed to it.
            p.bandMul = {0.15f, 2.5f, 0.15f, 2.5f, 0.15f};
            p.bandAlongX = true;
            p.trails = 0;
        } else if (mixed) {
            // Heavy clutter -> medium -> open, banded along the flight axis, so a
            // single run crosses all three regimes and the metrics are comparable
            // within one seed instead of across three worlds.
            p.bandMul = {2.0f, 1.0f, 0.35f, 1.0f, 2.0f};
            p.trails  = 2;      // a clear route exists, but it is not handed to you
        }
        if (trunkTex >= 0.f) { p.trunkTexMin = trunkTex; p.trunkTexMax = trunkTex; }
        genForest(W, p, &trails);
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
        // SCALE THE REQUIREMENT TO THE AIRFRAME. This was a hard-coded 1.5 m,
        // which is right for a 0.6 m forest aircraft and absurd for a 0.20 m
        // indoor one: a house corner legitimately offers ~1.0 m, so the
        // validator declared a perfectly good spawn unusable and then searched
        // UPWARD ONLY -- straight through the ceiling, putting the aircraft
        // outside the building at 4.1 m. It reported success.
        const float needClear = std::max(0.5f, robotR * 2.5f);
        float c0 = trueClearance(W, px, py, pz, 3.0f);
        if (c0 < needClear) {
            printf("  spawn clearance only %.2f m -- searching for a clear start\n", c0);
            bool ok = false;
            for (float rad = 1.f; rad <= 25.f && !ok; rad += 1.f)
                for (int a = 0; a < 24 && !ok; ++a)
                    // Search DOWN as well as up: indoors there is a ceiling,
                    // and 'up' is not always the way out of a tight spawn.
                    for (float dzs = 0.f; dzs <= 8.f && !ok; dzs += 1.f)
                    for (int sgn = 1; sgn >= -1 && !ok; sgn -= 2) {
                        float th = a * sim::PI_F / 12.f;
                        float tx = px + rad * std::cos(th), ty = py + rad * std::sin(th),
                              tz = pz + sgn * dzs;
                        if (tz > 0.2f && trueClearance(W, tx, ty, tz, 3.0f) >= needClear) {
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

    CamParams cp;
    cp.width = camW; cp.height = camH; cp.hfovDeg = hfov; cp.baselineM = baseline;
    cp.emitterOn = emitterOn; cp.ambientIR = ambientIR;
    if (emitterOn)
        std::printf("[cam] IR projector ON, ambient %.2f "
                    "(1 = bright daylight, 0 = dark)\n", ambientIR);
    DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = cell;
    // Z_max = sqrt(cell * f * B / sigma_d), derated 25 % -- the measured
    // over-estimate. Derived from the CAMERA rather than left at the default, so
    // swapping in a different sensor moves the carve limit with it instead of
    // silently keeping the old one.
    const float fpx = (camW * 0.5f) / std::tan(hfov * 0.5f * sim::PI_F / 180.f);
    {
        const float f = fpx;
        mp.maxIntegM = std::sqrt(cell * f * baseline / cp.subpixelPx) * 0.75f;
        if (maxIntegOverride > 0.f) mp.maxIntegM = maxIntegOverride;
        std::printf("[cam] %dx%d hfov %.0f deg baseline %.0f mm -> f %.0f px, "
                    "cell %.2f m -> Z_max %.2f m\n",
                    camW, camH, hfov, baseline * 1000.f, f, cell, mp.maxIntegM);
    }
    if (lHit  > 0)   mp.lHit = lHit;
    if (lMiss > 0)   mp.lMiss = lMiss;
    if (occT  > -90) mp.occThresh = occT;
    if (freeT > -90) mp.freeThresh = freeT;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    if (carveWin >= 0) mp.carveWinPx = carveWin;
    VoxelMap M; M.init(mp, px, py, pz);   // after spawn validation, not before
    if (voxelOnly) {
        if (!loadedPoints) {
            std::fprintf(stderr, "--voxelonly needs a saved space: "
                                 "--world lidar --points FILE.xyz\n");
            return 1;
        }
        if (!M.importXyz(pointsFile, true)) return 1;   // the file IS the world here
        std::printf("[mode] VOXEL ONLY -- no camera in the loop. The map is complete\n"
                    "       from step 1 and never changes, so this measures the PLANNER\n"
                    "       against real geometry and cannot catch a mapping error.\n");
    }

    // FAR-FIELD MAP. Depth error grows as Z^2, so the range at which a
    // measurement can be placed within one voxel is Z_max = sqrt(cell*f*B/sigma)
    // -- 5.2 m at 0.25 m cells on this camera. Coarser cells are not a
    // compromise: at 12 m a return genuinely has metres of uncertainty along
    // the ray, and a 0.25 m voxel claims a precision the measurement does not
    // contain. Sizing the cell to the uncertainty is the honest thing, and it
    // happens to triple the range:
    //
    //     0.25 m cells ->  5.2 m        2.0 m cells -> 14.8 m
    //
    // Affordable because it takes every 4th pixel (see integrateStride): a 2 m
    // cell does not need 76,800 rays when hundreds of them land inside it.
    //
    // WHAT IT DOES NOT DO, so nobody expects it to: a coarser voxel does not
    // make an invisible trunk visible. This extends the horizon the aircraft
    // can STEER by; it does nothing for obstacles stereo never returns.
    VoxelMapParams fp2;
    fp2.cell = farCell;
    { const int n = std::max(48, int(256.f / farCell)); fp2.nx = n; fp2.ny = n; fp2.nz = std::max(16, n / 3); }
    // Its own Z_max, derived like every other level rather than hardcoded.
    fp2.maxIntegM = std::sqrt(fp2.cell * fpx * baseline / 0.25f) * 0.75f;
    fp2.maxCarveM = 40.f;
    fp2.integrateStride = 4;       // a sixteenth of the rays
    fp2.depthSigCoef = mp.depthSigCoef;
    fp2.carveWinPx = 0;            // the min-filter is a fine-scale guard
    VoxelMap Mfar;
    if (useFar) Mfar.init(fp2, px, py, pz);

    // MID LEVEL. 0.25 m straight to 2.0 m is an 8x jump in one step, and
    // everything between the fine map's honest 3.5 m and the far map's 14 m was
    // being answered at 2 m resolution -- cells three times the robot. A 1 m rung
    // covers that band at its own honest range, so resolution degrades with
    // distance rather than collapsing at the first handover.
    std::vector<TrajectoryPlanner::CoarseLevel> coarseLadder;
    VoxelMapParams midp;
    midp.cell = 1.0f; midp.nx = 128; midp.ny = 128; midp.nz = 40;   // 128 x 128 x 40 m
    midp.maxIntegM = std::sqrt(midp.cell * fpx * baseline / 0.25f) * 0.75f;
    midp.maxCarveM = 25.f;
    midp.integrateStride = 2;      // a quarter of the rays
    midp.depthSigCoef = mp.depthSigCoef;
    midp.carveWinPx = 0;
    VoxelMap Mmid;
    if (useFar && useMid) Mmid.init(midp, px, py, pz);
    if (useFar && useMid) coarseLadder.push_back({&Mmid, midp.maxIntegM});
    if (useFar)           coarseLadder.push_back({&Mfar, fp2.maxIntegM});
    if (useFar)
        std::printf("[map] fine %.2f m -> %.1f m | mid %.2f m -> %.1f m | far %.2f m -> %.1f m\n",
                    mp.cell, mp.maxIntegM, midp.cell, midp.maxIntegM, fp2.cell, fp2.maxIntegM);
    // Takeoff bootstrap -- see VoxelMap::seedFree. The spawn was validated
    // against truth above, so this asserts something already checked.
    M.seedFree(px, py, pz, 1.5f);

    GeneralParams gp; gp.robotR = robotR;
    // HISTOGRAM PLANNER ONLY. `unknownCost` lives in GeneralParams, and the
    // default reactive layer here is the TRAJECTORY LIBRARY, which has no such
    // term -- so this changes nothing unless --general is in play. Exposed with
    // that written down because I swept it against the trajectory planner and
    // got four identical rows, which is the only reason I noticed I had been
    // explaining a result with a knob that was not connected to it.
    if (unkCost >= 0.f) gp.unknownCost = unkCost;
    if (emaA    >= 0) gp.fieldEma     = emaA;
    if (dwellM  >= 0) gp.switchMargin = dwellM;
    if (revP    >= 0) gp.revPenalty   = revP;
    if (commitN >= 0) gp.commitSteps  = commitN;
    GeneralPlanner gen(gp);
    TrajParams tp;
    if (vmax > 0.f) gp.vMax = vmax;
    tp.robotR = gp.robotR; tp.vMax = gp.vMax;
    tp.farMode = farMode ? TrajParams::FarMode::DENSITY
                         : TrajParams::FarMode::FIRST_BLOCKED;
    tp.decelMs2 = gp.decelMs2; tp.reactS = gp.reactS; tp.minFreeM = gp.minFreeM;
    tp.dt = dt;              // the rollout must use the control period
    if (coreFrac >= 0.f) tp.coreFrac = coreFrac;
    tp.latSlipDeg = latSlip;
    if (horizonS > 0.f) tp.horizonS = horizonS;
    if (rollCap  > 0.f) tp.rollCapM = rollCap;
    if (freeMargin >= 0.f) tp.freeMarginFrac = freeMargin;
    if (climbPen >= 0.f) tp.climbPenalty  = climbPen;
    if (sinkPen  >= 0.f) tp.descentPenalty = sinkPen;
    if (latKnee > 0.f) tp.latKneeDps = latKnee;
    TrajectoryPlanner traj(tp);
    if (latSlip != 0.f)
        printf("  sideslip           %.0f deg max, knee %.0f deg/s "
               "(velocity leads heading while turning)\n", tp.latSlipDeg, tp.latKneeDps);
    if (improve)
        printf("  depth improver     near %.1f m, radius %d px, %d seeds\n",
               dip.nearM, dip.radiusPx, dip.minSeeds);
    if (useTraj)
        printf("  reactive layer: trajectory library, %zu primitives\n", traj.librarySize());
    else
        printf("  reactive layer: openness histogram, %d x %d bins\n", gp.nAz, gp.nEl);
    ForwardParams fwp; fwp.robotR = gp.robotR;
    if (coneDeg > 0.f) fwp.coneDeg = coneDeg;
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
    // IMU ODOMETRY PROBE. Measures what short-memory dead reckoning would cost
    // on THIS trajectory, against truth, rather than against its own formula.
    ImuOdometry imu; ImuOdometryParams imuP, imuBudget;
    if (imuAttErr >= 0.f) { imuP.attErrDeg = imuAttErr; imuBudget.attErrDeg = imuAttErr; }
    imuP.maxDriftM = 1e9f;          // the probe never refuses; we want raw error
    imu.init(imuP);
    ImuOdometry imuRef; imuRef.init(imuBudget);   // only to report the real window
    float imuLegS = 0.f, imuLegMax = 0.f; double imuLegSum = 0.0; int imuLegs = 0;
    float imuPrevVx = 0.f, imuPrevVy = 0.f, imuPrevVz = 0.f;
    float imuTrueX = 0.f, imuTrueY = 0.f, imuTrueZ = 0.f;
    double imuErrSum = 0.0; float imuErrMax = 0.f; long imuN = 0;
    int   imuZupts = 0;
    // STALL DIAGNOSIS. "stopped" conflates two very different failures:
    //   BLOCKED   nothing in the library was admissible at all
    //   THROTTLED something was admissible, but carried too little
    //             CONFIRMED-FREE length to earn any speed
    // They have opposite fixes, so counting them together explains nothing.
    int stalBlocked = 0, stalThrottled = 0;
    double freeMStop = 0.0, openMStop = 0.0;
    long rjOcc = 0, rjUnk = 0, rjShort = 0, rjStart = 0, rjSteps = 0;
    int stepsRun = 0;
    long corridorLies = 0;
    bool reached = false;
    std::vector<cv::Point2f> trail;
    double tPlan = 0, tSense = 0, tGen = 0, tPrec = 0, tInteg = 0, tImprove = 0;
    int nPrec = 0;
    long impFilled = 0, impHoles = 0, impNear = 0;

    const float startDist = std::hypot(goalE - px, goalN - py);
    FILE* csv = csvPath.empty() ? nullptr : std::fopen(csvPath.c_str(), "w");
    // TRAINING DUMP: a 16x12 downsampled depth frame plus the direction the
    // planner chose from it. The sim is the only place this pairing is free --
    // it has both the sensor image and a trusted answer for the same instant.
    FILE* nn = dumpNN.empty() ? nullptr : std::fopen(dumpNN.c_str(), "w");
    const int NW = 32, NH = 24;
    if (nn) {
        for (int i = 0; i < NW * NH; ++i) std::fprintf(nn, "d%d,", i);
        std::fprintf(nn, "goalaz,az,el,speed,blocked\n");
    }
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
        // VOXEL ONLY: no camera, no integration. The map was loaded once and
        // does not change; recentring would only scroll cells out of a map
        // nothing is refilling, so that is skipped too.
        cv::Mat d;
        if (!voxelOnly)
            d = useTruth ? cam.renderTruth(W, pose) : cam.renderStereo(W, pose, nullptr);
        if (!voxelOnly) {
        // Fill holes that sit against a NEAR return, and only those. Applied to
        // the depth image before anything reads it, so every consumer -- fine
        // map, mid, far -- sees the same frame. Once per frame: the improver is
        // not idempotent by design (a fill is not a seed only within one call),
        // so running it per-map would dilate obstacles once per level.
        if (improve) {
            int64 ti = cv::getTickCount();
            DepthImproveStats ist = improveDepth(d, dip);
            tImprove += double(cv::getTickCount() - ti) / cv::getTickFrequency();
            impFilled += ist.filled; impHoles += ist.holes; impNear += ist.nearPx;
        }
        int64 tm = cv::getTickCount();
        M.integrate(d, cam, mpose);   // believed pose, not true pose
        if (useFar) { Mfar.integrate(d, cam, mpose); Mfar.recentre(px + dE, py + dN, pz + dU); }
        if (useFar && useMid) { Mmid.integrate(d, cam, mpose); Mmid.recentre(px + dE, py + dN, pz + dU); }
        tInteg += double(cv::getTickCount() - tm) / cv::getTickFrequency();
        tSense += double(cv::getTickCount() - t0) / cv::getTickFrequency();
        M.recentre(px + dE, py + dN, pz + dU);
        }
        tSense += 0.0;

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
        // A VALID PATH IS NOT A PATH THAT IS WORKING. pathStillGood tests
        // validity -- states safe, enough length ahead, bearing still sensible
        // -- and a STUCK aircraft satisfies every one of those. Measured in the
        // cul-de-sac: the router ran twice in 900 steps, found a path both
        // times, and the aircraft then sat stalled for 520 steps because its
        // one path never became invalid. It was simply a path the reactive
        // layer would not fly.
        //
        // So while the stall monitor says we are not making progress, keep
        // re-rolling the plan on the fallback period. RRTConnect is randomised,
        // so each attempt is a genuinely different route -- which is a liability
        // when things are going well and precisely what is wanted when they are
        // not.
        bool stuckReplan = stall.engaged && (s % replanEvery == 0);
        bool needReplan = wantRouter &&
            (reuse ? (!pathStillGood(M, path, px + dE, py + dN, pz + dU, mAz, fwp,
                                     useFar ? &Mfar : nullptr) || stuckReplan)
                   : (s % replanEvery == 0));
        if (needReplan) {
            int64 tp = cv::getTickCount();
            path = planForward(M, px + dE, py + dN, pz + dU, mAz, mEl, fwp, useFar ? &Mfar : nullptr);
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
            ? traj.plan(M, px + dE, py + dN, pz + dU, yaw, gAz, gEl, coarseLadder)
            : gen.plan(M, px + dE, py + dN, pz + dU, gAz, gEl);
        tGen += double(cv::getTickCount() - tg) / cv::getTickFrequency();
        tPlan += double(cv::getTickCount() - t1) / cv::getTickFrequency();
        if (nn) {
            cv::Mat small; cv::resize(d, small, cv::Size(NW, NH), 0, 0, cv::INTER_AREA);
            for (int v = 0; v < NH; ++v)
                for (int u = 0; u < NW; ++u) {
                    float z = small.at<float>(v, u);
                    // No-return is a real state, not a number: encode it as -1
                    // rather than as a distance, so the model cannot read it as
                    // "very close" or "very far".
                    std::fprintf(nn, "%.3f,", (z > 0.f && std::isfinite(z)) ? z : -1.f);
                }
            // THE GOAL IS AN INPUT, not a constant. Without it the task is
            // unlearnable: the same depth image with the goal to the left or to
            // the right demands opposite turns, so any model trained on the
            // image alone is being asked to predict a coin flip.
            std::fprintf(nn, "%.2f,%.2f,%.2f,%.3f,%d\n",
                         wrapDeg180(gAz - yaw), wrapDeg180(gr.azDeg - yaw),
                         gr.elDeg, gr.speed, gr.blocked ? 1 : 0);
        }
        if (gr.speed <= 0.01f) {
            ++stopped;
            if (gr.blocked) ++stalBlocked; else ++stalThrottled;
            freeMStop += gr.freeM; openMStop += gr.openM;
            if (useTraj) {
                const auto& rj = traj.lastReject();
                rjOcc += rj.occupied; rjUnk += rj.unknown;
                rjShort += rj.tooShort; rjStart += rj.atStart; ++rjSteps;
            }
        }

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
        // --- IMU odometry probe, measured against the truth it sits beside ---
        if (imuAttErr >= 0.f) {
            // The sim's own velocity change IS the true specific force. Corrupt
            // it only with a constant tilt leak, because that is the term §6
            // identifies and the one that does not average away.
            const float tax = (vx - imuPrevVx) / dt;
            const float tay = (vy - imuPrevVy) / dt;
            const float taz = (vz - imuPrevVz) / dt;
            const float leak = 9.81f * std::sin(imuP.attErrDeg * sim::PI_F / 180.f);
            imu.step(tax + leak, tay, taz, dt);
            imuTrueX += vx * dt; imuTrueY += vy * dt; imuTrueZ += vz * dt;
            float ex, ey, ez; imu.position(ex, ey, ez);
            const float err = std::sqrt((ex-imuTrueX)*(ex-imuTrueX) +
                                        (ey-imuTrueY)*(ey-imuTrueY) +
                                        (ez-imuTrueZ)*(ez-imuTrueZ));
            imuErrSum += err; imuErrMax = std::max(imuErrMax, err); ++imuN;
            // MOVE-STOP-SENSE GIVES A FREE ZUPT. The planner commanding zero is
            // the aircraft declaring itself stationary, which is exactly the
            // condition a zero-velocity update needs -- and it is the whole
            // reason bounded inertial odometry is viable on this airframe.
            if (gr.speed <= 0.01f && std::hypot(vx, vy) < imuP.zuptSpeedMs) {
                if (imuLegS > 0.2f) {          // a real leg, not a momentary dip
                    imuLegSum += imuLegS; imuLegMax = std::max(imuLegMax, imuLegS);
                    ++imuLegs;
                }
                imuLegS = 0.f;
                imu.zupt(); ++imuZupts;
                // Re-anchor truth too, so the error measured is the error WITHIN
                // a leg rather than the sum of every leg before it.
                float zx, zy, zz; imu.position(zx, zy, zz);
                imuTrueX = zx; imuTrueY = zy; imuTrueZ = zz;
            } else { imuLegS += dt; }
            // OPTICAL-FLOW VELOCITY UPDATE. Simulated as the true velocity with
            // a relative error (depth scale) plus an absolute floor (matching
            // noise) -- the two terms a flow-times-depth estimate actually has.
            // This tests the FUSION, not the flow extraction; the sim has no
            // appearance model, so extracting real flow here is not possible.
            if (flowRelErr >= 0.f && (s % 3) == 0) {      // ~10 Hz against 30 Hz IMU
                const float rel = flowRelErr;
                const float absN = 0.03f;
                auto jit = [&](float v, int k) {
                    const float r = std::sin(float(s) * 12.9898f + k * 78.233f) * 43758.5453f;
                    return v * (1.f + rel * (r - std::floor(r) - 0.5f) * 2.f)
                         + absN * (std::cos(float(s) * 3.7f + k) );
                };
                imu.velocityUpdate(jit(vx, 0), jit(vy, 1), jit(vz, 2), 0.35f);
            }
            imuPrevVx = vx; imuPrevVy = vy; imuPrevVz = vz;
        }
        px += vx * dt; py += vy * dt; pz += vz * dt;
        travelled += std::sqrt(vx * vx + vy * vy + vz * vz) * dt;
        if (std::hypot(vx, vy) > 0.2f) {
            // HEADING WAS SLAVED TO COURSE WITH NO RATE LIMIT, i.e. the vehicle
            // turned instantly. That is not a detail: the trajectory planner
            // exists BECAUSE "a bearing is not a thing the aircraft can do", and
            // a simulator that grants any bearing for free deletes the very
            // mismatch it was built to fix -- biasing every planner comparison
            // toward whichever planner ignores turning cost.
            const float want = std::atan2(vx, vy) * 180.f / sim::PI_F;
            if (yawRateLim > 0.f) {
                const float d = wrapDeg180(want - yaw);
                const float lim = yawRateLim * dt;
                yaw += std::max(-lim, std::min(lim, d));
                yaw = wrapDeg180(yaw);
            } else {
                yaw = want;
            }
        }
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
        // CORRIDOR LIE DETECTOR. A false-free RATE cannot see a tree: a 0.2 m
        // trunk crossing the flight path is ~30 cells of 5.5 M, which is
        // 0.0005% -- indistinguishable from zero in a percentage, and a
        // collision in reality. Measured: the carve guard took whole-map
        // false-free from 7.814% to 0.003% and the aircraft hit the SAME tree
        // at the SAME step, because the residual cells were exactly the ones
        // that mattered.
        //
        // So count what actually matters: cells just ahead of the aircraft,
        // inside the volume it is about to occupy, that the map calls FREE and
        // truth calls SOLID. Each one is a lie the planner is about to act on.
        {
            float sp = std::hypot(vx, vy);
            float ahead = std::max(1.0f, sp * 1.0f);      // one second of travel
            float ux = sp > 0.1f ? vx / sp : 0.f, uy = sp > 0.1f ? vy / sp : 0.f;
            for (float t = 0.f; t <= ahead; t += cell) {
                float qx = px + ux * t, qy = py + uy * t, qz = pz;
                int mx, my, mz; M.worldToCell(qx, qy, qz, mx, my, mz);
                if (!M.inBounds(mx, my, mz)) continue;
                int wx2, wy2, wz2; W.worldToCell(qx, qy, qz, wx2, wy2, wz2);
                bool mapFree = M.stateAt(qx, qy, qz) == VoxelMap::FREE;
                if (mapFree && W.solid(wx2, wy2, wz2)) { ++corridorLies; break; }
            }
        }
        float clr = trueClearance(W, px, py, pz, 2.0f);
        minClear = std::min(minClear, clr);
        if (clr <= gp.robotR * 0.5f) {
            ++collisions;
            printf("  !! COLLISION at step %d, (%.1f, %.1f, %.1f), clearance %.2f m\n",
                   s, px, py, pz, clr);
            // What did the MAP believe about the thing we just hit? Three
            // possibilities with three different fixes: FREE means the map
            // lied, UNKNOWN means the planner flew into space it had no
            // opinion about, OCCUPIED means the planner ignored its own map.
            {
                int nf=0, nu=0, no=0, solid=0;
                for (float dz2=-0.6f; dz2<=0.6f; dz2+=cell)
                for (float dy2=-0.6f; dy2<=0.6f; dy2+=cell)
                for (float dx2=-0.6f; dx2<=0.6f; dx2+=cell) {
                    float qx=px+dx2, qy=py+dy2, qz=pz+dz2;
                    int a,b,c2; W.worldToCell(qx,qy,qz,a,b,c2);
                    if (!W.solid(a,b,c2)) continue;
                    ++solid;
                    switch (M.stateAt(qx,qy,qz)) {
                        case VoxelMap::FREE: ++nf; break;
                        case VoxelMap::OCCUPIED: ++no; break;
                        default: ++nu; break;
                    }
                }
                printf("     of %d truth-SOLID cells in the robot volume, the map called "
                       "%d FREE, %d UNKNOWN, %d OCCUPIED\n", solid, nf, nu, no);
            }
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
    printf("  corridor lies      %ld of %d steps (%.1f%%)  <- map said FREE, truth said SOLID,\n"
           "                     within one second of travel ahead. A false-free RATE cannot\n"
           "                     see a tree; this can.\n",
           corridorLies, stepsRun, stepsRun ? 100.0*double(corridorLies)/stepsRun : 0.0);
    if (trailN > 0)
        printf("  trail following    %.0f%% of steps within 2.5 m of the corridor, "
               "mean deviation %.1f m\n",
               100.0 * double(trailIn) / double(trailN), trailDev / float(trailN));
    printf("  stopped on         %d of %d steps\n", stopped, steps);
    if (stopped) {
        printf("    of which BLOCKED   %d  (no admissible primitive at all)\n", stalBlocked);
        printf("    of which THROTTLED %d  (admissible, but no confirmed-free length)\n",
               stalThrottled);
        printf("    mean when stopped  freeM %.2f m   openM %.2f m\n",
               freeMStop / stopped, openMStop / stopped);
        if (rjSteps)
            printf("    rejects/step while stopped: occupied %.0f  unknown %.0f  "
                   "tooShort %.0f  atStart %.0f  (of %zu prims)\n",
                   double(rjOcc)/rjSteps, double(rjUnk)/rjSteps,
                   double(rjShort)/rjSteps, double(rjStart)/rjSteps,
                   traj.librarySize());
    }
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
    if (improve) {
        // Report the fill against BOTH denominators. Against holes it says how
        // much of the missing image was recovered; against the frame it says
        // how much of the map is now fabricated. The second is the one that can
        // hurt you, so it does not get to hide behind the first.
        const double px = double(camW) * camH * std::max(1, nsteps);
        printf("  depth improver     %6.2f ms;  filled %.2f%% of holes, %.2f%% of the frame"
               "  (holes %.1f%%, near returns %.1f%%)\n",
               1000 * tImprove / nsteps,
               impHoles ? 100.0 * double(impFilled) / double(impHoles) : 0.0,
               100.0 * double(impFilled) / px,
               100.0 * double(impHoles) / px, 100.0 * double(impNear) / px);
    }
    printf("  map integrate      %6.2f ms\n", 1000 * tInteg / nsteps);
    if (useFar) {
        // What did the extra range actually buy? Count cells the coarse map
        // calls OCCUPIED beyond the fine map's honest marking limit -- structure
        // the aircraft knows about and would not otherwise.
        long farOnly = 0, farAll = 0;
        for (int z = 0; z < fp2.nz; ++z)
          for (int y = 0; y < fp2.ny; ++y)
            for (int x = 0; x < fp2.nx; ++x) {
                if (!(Mfar.logAt(x,y,z) > fp2.occThresh)) continue;
                ++farAll;
                float wx, wy, wz; Mfar.cellCentre(x,y,z,wx,wy,wz);
                if (std::hypot(wx-px, wy-py) > mp.maxIntegM) ++farOnly;
            }
        printf("  far map            %ld occupied cells, %ld of them beyond the fine\n"
               "                     map's %.0f m marking limit\n",
               farAll, farOnly, mp.maxIntegM);
    }
    printf("  general planner    %6.2f ms\n", 1000 * tGen / nsteps);
    printf("  forward planner    %6.2f ms per replan (%d replans, every %d steps)\n",
           nPrec ? 1000 * tPrec / nPrec : 0.0, nPrec, replanEvery);
    printf("  ONBOARD TOTAL      %6.2f ms/step amortised\n",
           1000 * (tInteg + tGen + tPrec) / nsteps);
    printf("  [sim-only] depth render %.1f ms/step\n",
           1000 * (tSense - tInteg) / nsteps);

    VoxelMap::Score sc = M.score(W, px, py, pz, 25.f, 30.f);
    if (imuAttErr >= 0.f && imuN) {
        printf("  --- IMU odometry probe (%.1f deg tilt error) ---\n", imuP.attErrDeg);
        printf("    drift vs truth   mean %.3f m   worst %.3f m\n",
               imuErrSum / double(imuN), imuErrMax);
        printf("    move legs        %d, mean %.2f s, longest %.2f s\n",
               imuLegs, imuLegs ? imuLegSum / imuLegs : 0.0, imuLegMax);
        printf("    ZUPTs fired      %d\n", imuZupts);
        printf("    budget: one cell of drift lasts %.2f s at this tilt error\n",
               imuRef.usableWindowS());
    }
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
    // FOUR-PANE VIEW. The first three panes existed only behind --display, and
    // the FOURTH -- the aircraft's own first-person render of its VOXEL MAP --
    // existed only in voxel_live. That is the pane that shows what the vehicle
    // actually believes, as opposed to what is true, and leaving it out of the
    // offline sim meant every non-forest world had been evaluated on numbers
    // alone.
    if (dumpView >= 0) {
        const int PW = std::max(240, viewPx);
        cv::Mat truthTop2(W.ny(), W.nx(), CV_8UC3, cv::Scalar(250,248,245));
        int zc; { int a,b; W.worldToCell(0,0,pz,a,b,zc); }
        for (int y = 0; y < W.ny(); ++y)
            for (int x = 0; x < W.nx(); ++x)
                if (W.solid(x,y,zc))
                    truthTop2.at<cv::Vec3b>(W.ny()-1-y, x) = cv::Vec3b(70,70,70);
        for (size_t i = 1; i < trail.size(); ++i) {
            int x0,y0,z0,x1,y1,z1;
            W.worldToCell(trail[i-1].x, trail[i-1].y, pz, x0,y0,z0);
            W.worldToCell(trail[i].x,   trail[i].y,   pz, x1,y1,z1);
            cv::line(truthTop2, {x0,W.ny()-1-y0}, {x1,W.ny()-1-y1}, {40,40,220}, 2);
        }
        cv::Mat topV;  cv::resize(truthTop2, topV, cv::Size(PW,PW), 0,0, cv::INTER_AREA);
        cv::Mat sliceV = M.sliceImage(pz, PW);
        cv::Mat depthV(PW, PW, CV_8UC3, cv::Scalar(60,60,60));
        {
            cv::Mat dd = useTruth ? cam.renderTruth(W, {px,py,pz,yaw,0.f,0.f})
                                  : cam.renderStereo(W, {px,py,pz,yaw,0.f,0.f}, nullptr);
            cv::Mat dv(dd.rows, dd.cols, CV_8UC3, cv::Scalar(60,60,60));
            for (int y=0;y<dd.rows;++y) for (int x=0;x<dd.cols;++x) {
                float rr = dd.at<float>(y,x); if(!(rr>0.f)) continue;
                float ff = std::min(1.f, rr/cp.maxRangeM);
                dv.at<cv::Vec3b>(y,x) = cv::Vec3b(uchar(255*(1-ff)), uchar(80+100*ff), uchar(255*ff));
            }
            cv::resize(dv, depthV, cv::Size(PW,PW), 0,0, cv::INTER_NEAREST);
        }
        // THE LADDER, NOT THE FINE LEVEL. This pane used to call fpvImageWH on
        // M alone, which draws 0.25 m cells honest to 3.54 m and nothing else
        // -- so a scene whose nearest obstacle is four metres away rendered as
        // an empty room. That is exactly the fault VoxelMap::Layer's header
        // warns about, and it made a correctly-scoped mapper look broken in
        // every figure the sim produced. The planner has always consulted all
        // three levels (`coarseLadder`); the picture now shows what the planner
        // sees.
        //
        // Banded finest-first at each level's own honest range, and NOT
        // overlapping: a 2 m cell's near face can sit 2 m in front of the
        // surface it contains, so consulting it inside the fine level's range
        // draws a wall far too close.
        std::vector<VoxelMap::Layer> fpvLadder;
        {
            float band = 0.f;
            fpvLadder.push_back({&M, band, mp.maxIntegM});
            band = mp.maxIntegM;
            if (useFar && useMid) { fpvLadder.push_back({&Mmid, band, midp.maxIntegM}); band = midp.maxIntegM; }
            // The 1.15 inflation belongs only on the OUTERMOST edge, where it
            // covers cells marked slightly beyond maxIntegM before the aircraft
            // moved. On an internal handover it creates a shell one layer owns
            // and has no data for, which renders as a round blind spot.
            if (useFar) fpvLadder.push_back({&Mfar, band, fp2.maxIntegM * 1.15f});
            else        fpvLadder.back().range = mp.maxIntegM * 1.15f;
        }
        cv::Mat fpv = VoxelMap::renderLadder(fpvLadder, px, py, pz, yaw, 0.f,
                                             PW, PW, cp.hfovDeg);
        auto label = [&](cv::Mat& im, const char* t, cv::Scalar c) {
            cv::putText(im, t, {10,24}, cv::FONT_HERSHEY_SIMPLEX, 0.62, c, 2); };
        label(topV,   "TRUTH + flown path", {30,30,30});
        label(sliceV, "VOXEL MAP slice",    {30,30,30});
        label(depthV, useTruth?"DEPTH (truth)":"DEPTH (stereo)", {240,240,240});
        label(fpv,    "VOXEL FPV (what it believes)", {240,240,240});
        cv::Mat r1, r2, grid;
        cv::hconcat(std::vector<cv::Mat>{topV, sliceV}, r1);
        cv::hconcat(std::vector<cv::Mat>{depthV, fpv},  r2);
        cv::vconcat(r1, r2, grid);
        cv::imwrite(out + "_view.png", grid);
        printf("  wrote %s_view.png (truth | slice | depth | voxel FPV)\n", out.c_str());
    }
    cv::imwrite(out + "_top.png", topOut);
    cv::imwrite(out + "_slice.png", M.sliceImage(pz));
    if (csv) { std::fclose(csv); printf("  wrote %s\n", csvPath.c_str()); }
    if (nn)  { std::fclose(nn);  printf("  wrote %s (nn training rows)\n", dumpNN.c_str()); }
    printf("  wrote %s_top.png (flown path over truth) and %s_slice.png\n",
           out.c_str(), out.c_str());
    return collisions ? 2 : (reached ? 0 : 1);
}
