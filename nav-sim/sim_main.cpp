// nav-sim — a standalone path-planning testbench.
//
// Drop a drone in a 2.5D world, give it a (random or fixed) goal, and watch /
// measure different planners drive it there over a PARTIALLY-OBSERVED occupancy
// grid built from its own forward scan. Self-contained: no dependency on any
// other project.
//
//   ./nav_sim --list-planners
//   ./nav_sim --planner=astar --goal=random --seed=7
//   ./nav_sim --planner=potential --world=random --seed=3 --display
//   ./nav_sim --compare --goal=random --seed=7          # all planners, same world
//   ./nav_sim --planner=astar --batch=300               # mass stats
//   ./nav_sim --planner=wavefront --goal=random --save=/tmp/f   # PNG dump
//
// Planners: wavefront (BFS, the onboard method) · dijkstra · astar · potential
// (reactive, traps in local minima — included to SHOW it) · rrt (sampling).
//
// Honest scope: clean synthetic geometry. This validates planner LOGIC and
// behaviour under partial observability + replanning — not real-camera survival.

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "drone.hpp"
#include "explore.hpp"
#include "move_stop_sense.hpp"
#include "occupancy_grid.hpp"
#include "planners.hpp"
#include "sim_world.hpp"

#ifdef SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float wrap180(float d){ while(d>180.f)d-=360.f; while(d<=-180.f)d+=360.f; return d; }

// Build a random obstacle field (circles) between the drone start and a goal.
sim::World randomWorld(std::mt19937& rng, int nObs) {
    sim::World w;
    std::uniform_real_distribution<float> ue(-18.f, 18.f), un(-18.f, 18.f), ur(1.0f, 3.0f);
    for (int i = 0; i < nObs; ++i) {
        sim::Circle c; c.e = ue(rng); c.n = un(rng); c.r = ur(rng);
        if (std::hypot(c.e, c.n) < 4.f) continue;   // keep the start clear
        w.circles.push_back(c);
    }
    return w;
}

// A random reachable-ish goal 14–24 m away in a random direction, not inside an
// obstacle.
navsim::Vec2 randomGoal(std::mt19937& rng, const sim::World& w) {
    std::uniform_real_distribution<float> ang(0.f, 2.f*kPi), dist(14.f, 24.f);
    for (int tries = 0; tries < 200; ++tries) {
        const float a = ang(rng), d = dist(rng);
        navsim::Vec2 g{ std::sin(a)*d, std::cos(a)*d };
        if (w.clearanceAt(g.e, g.n) > 2.5f) return g;
    }
    return { 20.f, 0.f };
}

// -------------------------------------------------------------- L2 realism
// Localization drift: a real GPS-denied EKF (no VIO yet) accumulates position
// error without bound. Here the "believed" pose = true pose + a random-walk
// drift, so the occupancy grid is built at the WRONG place and smears — exactly
// the real onboard weakness. Metrics are still judged on the TRUE pose.
struct NoisyLoc {
    float be=0, bn=0, byaw=0; std::mt19937 rng; bool on=false;
    void reset(unsigned seed, bool enable){ rng.seed(seed?seed:1u); be=bn=byaw=0; on=enable; }
    // believed pose from the true pose (drift advances by dt)
    void believed(float te,float tn,float tyaw,float dt, float& e,float& n,float& yaw){
        if(!on){ e=te; n=tn; yaw=tyaw; return; }
        std::normal_distribution<float> g(0.f,1.f);
        const float s=std::sqrt(std::max(dt,1e-3f));
        be = be*0.9995f + g(rng)*0.05f*s;      // ~5cm/√s random walk, mild decay
        bn = bn*0.9995f + g(rng)*0.05f*s;
        byaw = byaw*0.999f + g(rng)*0.6f*s;    // deg
        e=te+be; n=tn+bn; yaw=tyaw+byaw;
    }
};

// Range-sensor noise: Gaussian range error + occasional dropouts (miss) and the
// odd spurious short return — what a real ToF/mono corridor scan actually gives.
void noiseRanges(std::vector<float>& r, std::mt19937& rng, float maxRange) {
    std::normal_distribution<float> g(0.f, 0.12f);      // ~12cm sigma
    std::uniform_real_distribution<float> u(0.f,1.f);
    for (auto& x : r) {
        const float p = u(rng);
        if (p < 0.02f) { x = maxRange; continue; }       // 2% dropout
        if (p > 0.995f) { x = std::min(x, 1.0f + u(rng)); continue; } // rare false-near
        x = std::max(0.1f, std::min(maxRange, x + g(rng)));
    }
}

// Structured arenas — not just scattered circles. Each returns a world and a
// fixed goal chosen so the direct line is NOT a free shot: the drone must turn,
// weave, route through a doorway, or escape a dead-end. The drone always starts
// at the origin facing the goal.
struct Arena { sim::World world; navsim::Vec2 goal; bool fixedGoal = true; };

Arena buildArena(const std::string& name, std::mt19937& rng, int nObs) {
    Arena a;
    auto wall = [&](float e0,float n0,float e1,float n1){ a.world.walls.push_back({e0,n0,e1,n1}); };
    if (name == "empty") {
        a.goal = {0.f, 22.f};
    } else if (name == "slalom") {
        // alternating barriers -> forced S-weave, gap on alternating sides
        wall(-12,  6,  3,  6);
        wall( -3, 12, 12, 12);
        wall(-12, 18,  3, 18);
        a.goal = {0.f, 26.f};
    } else if (name == "rooms") {
        // a dividing wall with a single narrow doorway; goal in the far room
        wall(-14, 13, -2, 13);
        wall(  2, 13, 14, 13);
        wall(-14, 13,-14, 26); wall(14,13,14,26); wall(-14,26,14,26);  // far room box
        a.goal = {0.f, 22.f};
    } else if (name == "maze") {
        // zig-zag corridor
        wall(-8,  6,  8,  6);      // wall 1 (gap on left below it via start)
        wall(-8,  6, -8, 13);
        wall(-8, 13,  6, 13);      // step right
        wall( 6, 13,  6, 20);
        wall(-8, 20,  6, 20);
        a.goal = {-4.f, 24.f};
    } else if (name == "trap") {
        // U-shaped cul-de-sac opening toward the start, straddling the direct
        // line -> potential-field traps in it; the grid planners route around.
        wall(-5, 10,  5, 10);
        wall(-5, 10, -5, 17);
        wall( 5, 10,  5, 17);
        a.goal = {0.f, 26.f};
    } else if (name == "cluttered") {
        // dense mixed obstacles (circles + short walls), no clean lane
        std::uniform_real_distribution<float> ue(-14,14), un(4,22), ur(0.8f,2.0f);
        const int n = std::max(nObs, 12);
        for (int i=0;i<n;++i){ sim::Circle c{ue(rng),un(rng),ur(rng)}; a.world.circles.push_back(c); }
        wall(-6, 9, -1, 9); wall(4, 15, 10, 15);
        a.goal = {0.f, 24.f};
    } else if (name == "comb") {
        // parallel dead-end "teeth" open at the bottom: entering a slot dead-ends
        // at the top and must be backed out of. Tests dead-end recovery.
        for (float e=-8; e<=8.01f; e+=4.f) wall(e, 6, e, 15);
        wall(-10, 15, 10, 15);                 // cap the tops so slots are dead ends
        a.goal = {0.f, 22.f};
    } else if (name == "bottleneck") {
        // a full-width wall with ONE narrow, OFF-CENTRE gap. Tests finding a gap
        // that isn't on the direct line.
        wall(-14, 12, 6, 12); wall(9, 12, 14, 12);   // gap at e in [6,9]
        a.goal = {0.f, 22.f};
    } else if (name == "gap-choice") {
        // two gaps in a wall: behind the LEFT one is a sealed pocket (dead end),
        // the RIGHT one leads to the goal. Tests committing to the correct gap.
        wall(-14,10,-8,10); wall(-5,10,5,10); wall(8,10,14,10);   // gaps [-8,-5] and [5,8]
        wall(-8,10,-8,16); wall(-5,10,-5,16); wall(-8,16,-5,16);  // left pocket = dead end
        a.goal = {6.5f, 22.f};
    } else if (name == "double-trap") {
        // two cul-de-sacs in series straddling the direct line — nested local
        // minima that pull a greedy planner in twice.
        wall(-5, 8, 5, 8); wall(-5, 8, -5, 13); wall(5, 8, 5, 13);      // trap 1 (opens up)
        wall(-4, 18, 4, 18); wall(-4, 13, -4, 18); wall(4, 13, 4, 18);  // trap 2 (opens down)
        a.goal = {0.f, 26.f};
    } else if (name == "pillars") {
        // a dense but STRUCTURED pillar field — a weaving corridor exists.
        for (float n=6; n<=20.01f; n+=3.5f)
            for (float e=-9; e<=9.01f; e+=4.5f) {
                const bool gap = (std::fmod(n,7.f) < 1.f) ? (std::fabs(e-2.f)<2.5f) : (std::fabs(e+2.f)<2.5f);
                if (!gap) a.world.circles.push_back({e, n, 1.0f});
            }
        a.goal = {0.f, 24.f};
    } else {   // "random"
        a.world = randomWorld(rng, nObs);
        a.goal = randomGoal(rng, a.world);
        a.fixedGoal = false;
        return a;
    }
    // Per-seed variation for the structured arenas: shift the whole layout a
    // little (keeps walls connected) and sprinkle a couple of extra obstacles,
    // so different seeds give different-but-same-character maps.
    if (name != "empty") {
        std::uniform_real_distribution<float> j(-2.0f, 2.0f);
        const float dx = j(rng), dy = j(rng);
        for (auto& w : a.world.walls) { w.e0+=dx; w.n0+=dy; w.e1+=dx; w.n1+=dy; }
        std::uniform_int_distribution<int> ne(0, 2);
        std::uniform_real_distribution<float> ue(-13,13), un(5,21), ur(0.9f,1.8f);
        for (int i = 0, extra = ne(rng); i < extra; ++i) {
            sim::Circle c{ ue(rng), un(rng), ur(rng) };
            if (std::hypot(c.e, c.n) > 4.f && std::hypot(c.e-a.goal.e, c.n-a.goal.n) > 3.f)
                a.world.circles.push_back(c);
        }
    }
    return a;
}

struct RunOpts {
    bool  display = false;
    std::string saveDir;
    std::string sensorName = "camera";
    int   nRays = 61; float hFov = 90.f, maxRange = 8.f;
    int   viz = 720;
    bool  noise = false;            // L2 realism: localization drift + sensor noise
    unsigned noiseSeed = 1;
};

// Sensor FOV presets — the real hardware tradeoff. A forward ToF (VL53L5CX/L9CX)
// is a NARROW metric cone; a monocular camera is WIDE but nominal-scale. A narrow
// FOV maps a thin strip per tick (more turning/scanning to build a map); a wide
// FOV sees more at once. Same planners, different sensing footprint.
struct Sensor { const char* name; float fovDeg; float rangeM; int rays; };
Sensor sensorPreset(const std::string& s) {
    if (s == "tof")      return {"tof",      45.f, 4.0f, 31};   // VL53L5CX-like: narrow, short, metric
    if (s == "tof-wide") return {"tof-wide", 63.f, 9.0f, 45};   // VL53L9CX-like: wider, longer
    if (s == "camera")   return {"camera",   90.f, 8.0f, 61};   // monocular: wide, nominal range
    return {"camera", 90.f, 8.0f, 61};
}

struct RunResult {
    bool   reached = false, collided = false;
    float  minStandoff = 1e9f, pathLen = 0.f, straightLen = 0.f, simTime = 0.f;
    int    replans = 0; double planMsTotal = 0.0; long planCalls = 0;
};

// Sentinel "planner" names for the non-path-planner modes.
const char* kMssName = "move-stop-sense";
const char* kExpName = "explore-rth";

// Derive a VFH-like corridor signal from a forward range scan: forward-cone
// clearance (openness) + the openest direction (offset in [-1,1] over the FoV).
void corridorFromScan(const std::vector<float>& ranges, float hFovDeg, float maxRange,
                      float& open, float& offset) {
    const int N = (int)ranges.size();
    if (N == 0) { open = 0.f; offset = 0.f; return; }
    float fwd = maxRange;                                   // forward cone clearance
    for (int i = N/2-3; i <= N/2+3; ++i) if (i>=0 && i<N) fwd = std::min(fwd, ranges[i]);
    open = std::max(0.f, std::min(1.f, fwd / maxRange));
    // openest ray, with a mild center bias so an equally-open field steers
    // straight ahead (not to the leftmost ray) — approximates VFH+'s forward pref.
    int best = N/2; float bestScore = -1e9f;
    for (int i = 0; i < N; ++i) { float sc = ranges[i] - 0.01f*std::abs(i - N/2);
        if (sc > bestScore) { bestScore = sc; best = i; } }
    const float half = hFovDeg * 0.5f;
    const float rel = (N==1) ? 0.f : (-half + 2.f*half*best/(N-1));
    offset = std::max(-1.f, std::min(1.f, rel / half));
}

// Grid route bearing toward the goal (mirrors P5b LocalMap.plan feeding the
// mission's planBearing): run a wavefront/A* on the current partial grid, return
// the bearing to a lookahead point along the route.
void planBearingFor(navsim::IPlanner& mapPlanner, const navsim::OccupancyGrid& g,
                    float e, float n, navsim::Vec2 goal, int inflate,
                    bool& valid, float& bearingDeg) {
    valid = false;
    navsim::PlanResult pr = mapPlanner.plan(g, {e,n}, goal, inflate);
    if (!pr.ok || pr.path.size() < 2) return;
    float acc = 0.f; navsim::Vec2 wp = pr.path.back();
    for (size_t i=1;i<pr.path.size();++i){ acc += std::hypot(pr.path[i].e-pr.path[i-1].e, pr.path[i].n-pr.path[i-1].n);
        if (acc >= 2.5f) { wp = pr.path[i]; break; } }
    bearingDeg = std::atan2(wp.e-e, wp.n-n) * 180.f/kPi; valid = true;
}

// Draw the occupancy grid + planned path + trail + the live sensor FOV footprint,
// top-down. The FOV wedge + scan hits show HOW the drone maps: the grid fills in
// behind the sweeping cone, and a narrow (ToF) vs wide (camera) FOV is visible.
cv::Mat renderGridView(const navsim::OccupancyGrid& g, const sim::World& w,
                       const navsim::Drone& d, navsim::Vec2 goal,
                       const std::vector<navsim::Vec2>& path,
                       const std::vector<cv::Point2f>& trail,
                       const std::vector<float>& ranges, float hFovDeg, float maxRange,
                       const char* planner, const char* sensor,
                       float centerE, float centerN, int size, float spanM) {
    cv::Mat img(size, size, CV_8UC3, cv::Scalar(24,24,26));
    const float ppm = size / spanM;
    auto toPx = [&](float e, float n){
        return cv::Point((int)(size/2 + (e-centerE)*ppm), (int)(size/2 - (n-centerN)*ppm)); };
    const cv::Point dp = toPx(d.e,d.n);

    // occupancy belief (grey = known-free, red = believed-occupied)
    for (int cy=0; cy<g.cells(); ++cy) for (int cx=0; cx<g.cells(); ++cx) {
        const float L = g.logAt(cx,cy); if (std::fabs(L) < 0.05f) continue;
        float e,n; g.cellToWorld(cx,cy,e,n);
        if (std::fabs(e) > spanM/2 || std::fabs(n) > spanM/2) continue;
        const cv::Point p = toPx(e,n); const int s = std::max(1,(int)(g.cellM()*ppm));
        cv::Vec3b col = (L>0)? cv::Vec3b(50,60,(uchar)std::min(255.f,120+L*20))
                             : cv::Vec3b(48,48,50);
        cv::rectangle(img, cv::Rect(p.x-s/2,p.y-s/2,s,s), col, -1);
    }
    // live sensor FOV wedge (translucent) — the current sensing footprint
    {
        const float half = hFovDeg * 0.5f;
        std::vector<cv::Point> cone{ dp };
        for (float a=-half; a<=half+0.1f; a+=4.f) {
            const float b=(d.yawDeg+a)*kPi/180.f;
            cone.push_back(toPx(d.e+std::sin(b)*maxRange, d.n+std::cos(b)*maxRange));
        }
        cv::Mat ov = img.clone();
        cv::fillConvexPoly(ov, cone, cv::Scalar(70,65,40), cv::LINE_AA);
        cv::addWeighted(ov, 0.22, img, 0.78, 0, img);
        cv::polylines(img, cone, true, cv::Scalar(120,110,70), 1, cv::LINE_AA);
    }
    // scan hits this tick (bright = the fresh measurements building the map)
    {
        const int N=(int)ranges.size(); const float half=hFovDeg*0.5f;
        for (int i=0;i<N;++i){ if (ranges[i] >= maxRange-1e-3f) continue;
            const float rel=(N==1)?0.f:(-half+2.f*half*i/(N-1));
            const float b=(d.yawDeg+rel)*kPi/180.f;
            cv::circle(img, toPx(d.e+std::sin(b)*ranges[i], d.n+std::cos(b)*ranges[i]), 2, {90,200,255}, -1, cv::LINE_AA);
        }
    }
    // true obstacles (outline, so belief vs truth is visible)
    for (auto& c : w.circles) cv::circle(img, toPx(c.e,c.n), (int)(c.r*ppm), {80,110,220}, 1, cv::LINE_AA);
    for (auto& wl : w.walls)  cv::line(img, toPx(wl.e0,wl.n0), toPx(wl.e1,wl.n1), {80,110,220}, 2, cv::LINE_AA);
    // planned path
    for (size_t i=1;i<path.size();++i) cv::line(img, toPx(path[i-1].e,path[i-1].n), toPx(path[i].e,path[i].n), {80,220,255}, 2, cv::LINE_AA);
    // trail
    for (size_t i=1;i<trail.size();++i) cv::line(img, toPx(trail[i-1].x,trail[i-1].y), toPx(trail[i].x,trail[i].y), {90,200,90}, 1, cv::LINE_AA);
    // goal + drone
    cv::drawMarker(img, toPx(goal.e,goal.n), {80,255,80}, cv::MARKER_STAR, 16, 2, cv::LINE_AA);
    const float yr = d.yawDeg*kPi/180.f;
    cv::arrowedLine(img, dp, cv::Point(dp.x+(int)(22*std::sin(yr)), dp.y-(int)(22*std::cos(yr))), {255,255,255}, 2, cv::LINE_AA,0,0.3);
    cv::circle(img, dp, 4, {255,255,255}, -1, cv::LINE_AA);
    char hud[96]; std::snprintf(hud,sizeof(hud),"%s  |  %s  %.0f deg / %.0f m", planner, sensor, hFovDeg, maxRange);
    cv::putText(img, hud, {8,22}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {235,235,235}, 1);

    // legend (bottom-left): what each colour means
    int ly = size - 74;
    auto leg = [&](cv::Scalar c, const char* s){
        cv::rectangle(img, cv::Rect(8, ly-8, 12, 10), c, -1);
        cv::putText(img, s, {26, ly}, cv::FONT_HERSHEY_SIMPLEX, 0.36, {200,200,200}, 1); ly += 15; };
    leg({80,110,220}, "true obstacle");
    leg({60,70,150},  "mapped occupied");
    leg({48,48,50},   "mapped free");
    leg({80,220,255}, "plan / scan");
    leg({90,200,90},  "flown trail");
    return img;
}

RunResult run(navsim::IPlanner& planner, const sim::World& worldIn, navsim::Vec2 goal,
              const RunOpts& opt, bool verbose) {
    planner.reset();   // clear any per-episode state (bug2)
    sim::World world = worldIn;
    navsim::OccupancyGrid grid;
    navsim::Drone drone;   // starts at origin, facing the goal
    drone.yawDeg = std::atan2(goal.e, goal.n) * 180.f / kPi;
    const int inflate = (int)std::ceil(1.5f / grid.cellM());   // ~1.5 m robot berth

    RunResult R; R.straightLen = std::hypot(goal.e, goal.n);
    std::vector<cv::Point2f> trail; std::vector<navsim::Vec2> lastPath;
    const float dt = 0.05f; float t = 0.f; float lastE = drone.e, lastN = drone.n;
    NoisyLoc nloc; nloc.reset(opt.noiseSeed, opt.noise);

    for (int step = 0; step < 6000 && !R.reached; ++step) {
        world.advance(dt);
        // ---- perception: forward scan (at TRUE pose) -> integrate at BELIEVED pose
        std::vector<float> ranges;
        sim::castScan(world, drone.e, drone.n, drone.yawDeg, opt.hFov, opt.nRays, opt.maxRange, ranges);
        if (opt.noise) noiseRanges(ranges, nloc.rng, opt.maxRange);
        float be, bn, byaw; nloc.believed(drone.e, drone.n, drone.yawDeg, dt, be, bn, byaw);
        grid.integrate(be, bn, byaw, ranges, opt.hFov, opt.maxRange);

        // ---- plan (every tick, on the latest partial map, from the BELIEVED pose)
        navsim::PlanResult pr = planner.plan(grid, {be,bn}, goal, inflate);
        R.planMsTotal += pr.planMs; ++R.planCalls; if (pr.ok) ++R.replans;
        lastPath = pr.path;

        // ---- follow: lookahead waypoint along the path (steer from BELIEVED pose)
        float targetBearing = std::atan2(goal.e-be, goal.n-bn) * 180.f/kPi;
        if (pr.ok && pr.path.size() >= 2) {
            const float lookahead = 2.5f; float acc = 0.f; navsim::Vec2 wp = pr.path.back();
            for (size_t i=1;i<pr.path.size();++i){ acc += std::hypot(pr.path[i].e-pr.path[i-1].e, pr.path[i].n-pr.path[i-1].n);
                if (acc >= lookahead){ wp = pr.path[i]; break; } }
            targetBearing = std::atan2(wp.e-be, wp.n-bn) * 180.f/kPi;
        }
        // ---- reactive safety: slow/stop if the forward cone is closing ------
        float fwdClear = opt.maxRange;
        for (int i = opt.nRays/2-3; i <= opt.nRays/2+3; ++i)
            if (i>=0 && i<(int)ranges.size()) fwdClear = std::min(fwdClear, ranges[i]);
        const float speedScale = std::max(0.f, std::min(1.f, (fwdClear - 1.5f) / 3.f));

        drone.step(targetBearing, dt, speedScale);

        // ---- metrics (truth) ----------------------------------------------
        trail.emplace_back(drone.e, drone.n);
        R.pathLen += std::hypot(drone.e-lastE, drone.n-lastN); lastE=drone.e; lastN=drone.n;
        const float clr = world.clearanceAt(drone.e, drone.n);
        R.minStandoff = std::min(R.minStandoff, clr);
        if (clr < 0.f) R.collided = true;
        if (std::hypot(goal.e-drone.e, goal.n-drone.n) <= 1.5f) R.reached = true;

        if (verbose && step % 40 == 0)
            std::printf("  t=%5.1f pos=(%6.2f,%6.2f) yaw=%5.1f dGoal=%5.2f planOk=%d expand=%d clr=%5.2f\n",
                        t, drone.e, drone.n, drone.yawDeg, std::hypot(goal.e-drone.e,goal.n-drone.n),
                        (int)pr.ok, pr.expanded, clr);

#if defined(SIM_HAVE_HIGHGUI)
        if (opt.display || !opt.saveDir.empty()) {
            // frame the view on the whole journey (origin<->goal midpoint), zoomed to fit
            const float cE = goal.e * 0.5f, cN = goal.n * 0.5f;
            const float span = std::max(26.f, std::hypot(goal.e, goal.n) * 1.7f + 10.f);
            cv::Mat top = renderGridView(grid, world, drone, goal, lastPath, trail, ranges,
                                         opt.hFov, opt.maxRange, planner.name(), opt.sensorName.c_str(),
                                         cE, cN, opt.viz, span);
            if (!opt.saveDir.empty() && step % 4 == 0) {
                char p[512]; std::snprintf(p,sizeof(p),"%s/%s_%05d.png", opt.saveDir.c_str(), planner.name(), step);
                cv::imwrite(p, top);
            }
            if (opt.display) { cv::imshow("nav-sim", top); if ((cv::waitKey(1)&0xFF)=='q') break; }
        }
#else
        (void)lastPath;
#endif
        t += dt;
    }
    R.simTime = t;
    return R;
}

// Headless run of the ported move-stop-sense controller (the drone's real
// navigation mode). Same world/metrics as run(), but the drone is driven by the
// phase machine + reactive corridor + grid route instead of a path planner.
RunResult runMss(const sim::World& worldIn, navsim::Vec2 goal, const RunOpts& opt, bool verbose) {
    sim::World world = worldIn;
    navsim::OccupancyGrid grid;
    navsim::Drone drone; drone.yawDeg = std::atan2(goal.e, goal.n) * 180.f/kPi;
    const int inflate = (int)std::ceil(1.5f / grid.cellM());
    navsim::MoveStopSense mss; mss.reset();
    auto mapPlanner = navsim::makePlanner("astar");   // supplies the grid route bearing

    RunResult R; R.straightLen = std::hypot(goal.e, goal.n);
    const float dt = 0.05f; float t = 0.f, lastE = drone.e, lastN = drone.n, speed = 0.f;
    for (int step = 0; step < 6000 && !R.reached; ++step) {
        world.advance(dt);
        std::vector<float> ranges;
        sim::castScan(world, drone.e, drone.n, drone.yawDeg, opt.hFov, opt.nRays, opt.maxRange, ranges);
        grid.integrate(drone.e, drone.n, drone.yawDeg, ranges, opt.hFov, opt.maxRange);

        navsim::MssInput in;
        in.e=drone.e; in.n=drone.n; in.yawDeg=drone.yawDeg; in.speedMs=speed;
        corridorFromScan(ranges, opt.hFov, opt.maxRange, in.corridorOpen, in.corridorOffset);
        planBearingFor(*mapPlanner, grid, drone.e, drone.n, goal, inflate, in.planValid, in.planBearing);
        in.goalBearing = std::atan2(goal.e-drone.e, goal.n-drone.n) * 180.f/kPi;
        const double t0 = 0; navsim::MssOutput out = mss.update(in, dt);
        R.planMsTotal += 0; ++R.planCalls; (void)t0;

        drone.step(out.bearingDeg, dt, out.speedScale);
        speed = std::hypot(drone.e-lastE, drone.n-lastN) / dt;
        R.pathLen += std::hypot(drone.e-lastE, drone.n-lastN); lastE=drone.e; lastN=drone.n;
        const float clr = world.clearanceAt(drone.e, drone.n);
        R.minStandoff = std::min(R.minStandoff, clr);
        if (clr < 0.f) R.collided = true;
        if (std::hypot(goal.e-drone.e, goal.n-drone.n) <= 1.5f) R.reached = true;
        if (verbose && step % 40 == 0)
            std::printf("  t=%5.1f pos=(%6.2f,%6.2f) %-6s dGoal=%5.2f open=%.2f plan=%d clr=%.2f\n",
                        t, drone.e, drone.n, out.phase, std::hypot(goal.e-drone.e,goal.n-drone.n),
                        in.corridorOpen, (int)in.planValid, clr);
        t += dt;
    }
    R.simTime = t;
    return R;
}

// Headless run of explore-and-return-home. "reached" == returned to home after
// mapping the reachable area. Home = origin; goal input is ignored.
RunResult runExplore(const sim::World& worldIn, const RunOpts& opt, bool verbose) {
    sim::World world = worldIn;
    navsim::OccupancyGrid grid; navsim::Drone drone;
    const int inflate = (int)std::ceil(1.5f / grid.cellM());
    navsim::Explore expl; navsim::Vec2 home{0.f,0.f}; expl.reset(home);
    auto mapPlanner = navsim::makePlanner("astar");
    navsim::MoveStopSense drv; drv.reset();   // robust low-level driver (has SCAN recovery)
    RunResult R; R.straightLen = 1.f;
    const float dt = 0.05f; float t=0, lastE=drone.e, lastN=drone.n, speed=0;
    for (int step=0; step<16000 && !R.reached; ++step) {
        world.advance(dt);
        std::vector<float> ranges;
        sim::castScan(world, drone.e, drone.n, drone.yawDeg, opt.hFov, opt.nRays, opt.maxRange, ranges);
        grid.integrate(drone.e, drone.n, drone.yawDeg, ranges, opt.hFov, opt.maxRange);
        navsim::Explore::Out eo = expl.step(grid, {drone.e,drone.n}, inflate);
        if (eo.done) { R.reached = true; break; }
        navsim::MssInput in; in.e=drone.e; in.n=drone.n; in.yawDeg=drone.yawDeg; in.speedMs=speed;
        corridorFromScan(ranges, opt.hFov, opt.maxRange, in.corridorOpen, in.corridorOffset);
        planBearingFor(*mapPlanner, grid, drone.e, drone.n, eo.goal, inflate, in.planValid, in.planBearing);
        in.goalBearing = std::atan2(eo.goal.e-drone.e, eo.goal.n-drone.n)*180.f/kPi;
        auto out = drv.update(in, dt);
        drone.step(out.bearingDeg, dt, out.speedScale);
        speed = std::hypot(drone.e-lastE, drone.n-lastN)/dt;
        R.pathLen += std::hypot(drone.e-lastE, drone.n-lastN); lastE=drone.e; lastN=drone.n;
        float clr=world.clearanceAt(drone.e,drone.n); R.minStandoff=std::min(R.minStandoff,clr);
        if(clr<0)R.collided=true;
        if (verbose && step%80==0)
            std::printf("  t=%5.1f pos=(%6.2f,%6.2f) %-7s/%-6s subgoal=(%.1f,%.1f) clr=%.2f\n",
                        t, drone.e, drone.n, eo.phase, out.phase, eo.goal.e, eo.goal.n, clr);
        t += dt;
    }
    R.simTime = t; return R;
}

void printRow(const char* name, const RunResult& r) {
    std::printf("  %-10s reached=%-3s collided=%-3s standoff=%5.2fm pathLen=%6.2fm (%.2fx) "
                "avgPlan=%.3fms\n",
                name, r.reached?"YES":"no", r.collided?"YES":"no", r.minStandoff, r.pathLen,
                r.straightLen>0? r.pathLen/r.straightLen : 0.f,
                r.planCalls? r.planMsTotal/r.planCalls : 0.0);
}

// ===========================================================================
// Interactive GUI: an episode you can step, plus a clickable control panel.
// ===========================================================================

const std::vector<std::string> kArenas = {"random","empty","slalom","rooms","maze","trap","cluttered",
                                          "comb","bottleneck","gap-choice","double-trap","pillars"};
const std::vector<std::string> kSensors = {"camera","tof","tof-wide"};

// One steppable episode: world + grid + drone + planner + trail. Same physics
// as run(), but advanced one tick at a time so the GUI can drive it live.
struct SimEpisode {
    RunOpts opt;
    sim::World world; navsim::OccupancyGrid grid; navsim::Drone drone; navsim::Vec2 goal{};
    std::unique_ptr<navsim::IPlanner> planner;
    std::unique_ptr<navsim::IPlanner> mapPlanner;   // grid route for MSS / explore
    navsim::MoveStopSense mssCtl; bool mss=false; std::string phase;
    navsim::Explore expl; bool explore=false; navsim::Vec2 home{}; navsim::Vec2 subGoal{};
    std::string plannerName="astar", arenaName="maze", sensorName="camera";
    unsigned seed=1;
    std::vector<cv::Point2f> trail; std::vector<navsim::Vec2> lastPath; std::vector<float> ranges;
    int inflate=3, steps=0; float dt=0.05f, speed=0.f;
    bool reached=false, collided=false; float minStandoff=1e9f, pathLen=0.f, lastE=0, lastN=0;
    double lastPlanMs=0.0;
    NoisyLoc nloc; float be_=0, bn_=0, byaw_=0;   // L2 realism: believed (drifting) pose

    void restart() {
        Sensor sp = sensorPreset(sensorName); sensorName = sp.name;
        opt.hFov = sp.fovDeg; opt.maxRange = sp.rangeM; opt.nRays = sp.rays;
        std::mt19937 rng(seed);
        Arena a = buildArena(arenaName, rng, 6);
        world = a.world; goal = a.goal;
        grid = navsim::OccupancyGrid();
        drone = navsim::Drone(); drone.yawDeg = std::atan2(goal.e, goal.n) * 180.f/kPi;
        inflate = (int)std::ceil(1.5f / grid.cellM());
        mss = (plannerName == kMssName);
        explore = (plannerName == kExpName);
        if (mss)     { mssCtl.reset(); mapPlanner = navsim::makePlanner("astar"); planner.reset(); }
        else if (explore) { home = {0.f, 0.f}; expl.reset(home); subGoal = home; mssCtl.reset();
                            mapPlanner = navsim::makePlanner("astar"); planner.reset(); }
        else         { planner = navsim::makePlanner(plannerName); if (planner) planner->reset(); }
        trail.clear(); lastPath.clear(); ranges.clear(); phase.clear();
        steps=0; speed=0; reached=collided=false; minStandoff=1e9f; pathLen=0; lastE=drone.e; lastN=drone.n;
        nloc.reset(seed, opt.noise); be_=drone.e; bn_=drone.n; byaw_=drone.yawDeg;
    }
    void step() {
        if (reached || collided) return;
        world.advance(dt);
        // scan at the TRUE pose; noise it; integrate + plan at the BELIEVED (drifting) pose
        sim::castScan(world, drone.e, drone.n, drone.yawDeg, opt.hFov, opt.nRays, opt.maxRange, ranges);
        if (opt.noise) noiseRanges(ranges, nloc.rng, opt.maxRange);
        nloc.believed(drone.e, drone.n, drone.yawDeg, dt, be_, bn_, byaw_);
        grid.integrate(be_, bn_, byaw_, ranges, opt.hFov, opt.maxRange);

        if (mss) {
            navsim::MssInput in;
            in.e=be_; in.n=bn_; in.yawDeg=byaw_; in.speedMs=speed;
            corridorFromScan(ranges, opt.hFov, opt.maxRange, in.corridorOpen, in.corridorOffset);
            planBearingFor(*mapPlanner, grid, be_, bn_, goal, inflate, in.planValid, in.planBearing);
            in.goalBearing = std::atan2(goal.e-be_, goal.n-bn_) * 180.f/kPi;
            navsim::MssOutput out = mssCtl.update(in, dt);
            phase = out.phase; lastPath = { {be_,bn_}, {out.wpE,out.wpN} };
            drone.step(out.bearingDeg, dt, out.speedScale);
        } else if (explore) {
            navsim::Explore::Out eo = expl.step(grid, {be_,bn_}, inflate);
            subGoal = eo.goal; goal = eo.goal;   // render toward the sub-goal
            phase = std::string(eo.phase);
            if (eo.done) { reached = true; lastPath.clear(); }
            else {
                navsim::MssInput in; in.e=be_; in.n=bn_; in.yawDeg=byaw_; in.speedMs=speed;
                corridorFromScan(ranges, opt.hFov, opt.maxRange, in.corridorOpen, in.corridorOffset);
                planBearingFor(*mapPlanner, grid, be_, bn_, eo.goal, inflate, in.planValid, in.planBearing);
                in.goalBearing = std::atan2(eo.goal.e-be_, eo.goal.n-bn_)*180.f/kPi;
                auto out = mssCtl.update(in, dt);
                lastPath = { {be_,bn_}, {out.wpE,out.wpN} };
                drone.step(out.bearingDeg, dt, out.speedScale);
            }
        } else if (planner) {
            navsim::PlanResult pr = planner->plan(grid, {be_,bn_}, goal, inflate);
            lastPath = pr.path; lastPlanMs = pr.planMs;
            float tb = std::atan2(goal.e-be_, goal.n-bn_) * 180.f/kPi;
            if (pr.ok && pr.path.size()>=2) { float look=2.5f, acc=0; navsim::Vec2 wp=pr.path.back();
                for (size_t i=1;i<pr.path.size();++i){ acc+=std::hypot(pr.path[i].e-pr.path[i-1].e,pr.path[i].n-pr.path[i-1].n); if(acc>=look){wp=pr.path[i];break;} }
                tb = std::atan2(wp.e-be_, wp.n-bn_) * 180.f/kPi; }
            float fwd=opt.maxRange; for(int i=opt.nRays/2-3;i<=opt.nRays/2+3;++i) if(i>=0&&i<(int)ranges.size()) fwd=std::min(fwd,ranges[i]);
            drone.step(tb, dt, std::max(0.f,std::min(1.f,(fwd-1.5f)/3.f)));
        } else return;

        speed = std::hypot(drone.e-lastE, drone.n-lastN) / dt;
        trail.emplace_back(drone.e, drone.n);
        pathLen += std::hypot(drone.e-lastE, drone.n-lastN); lastE=drone.e; lastN=drone.n;
        float clr = world.clearanceAt(drone.e, drone.n); minStandoff=std::min(minStandoff,clr);
        if (clr<0) collided=true;
        if (std::hypot(goal.e-drone.e, goal.n-drone.n) <= 1.5f) reached=true;
        if (++steps > 3500) reached = true;   // stuck -> end so the demo can loop
    }
};

// Planner names for the GUI/compare = the 10 path planners + the higher-level
// modes (the real move-stop-sense nav, and explore-and-return-home).
std::vector<std::string> allModeNames() {
    auto v = navsim::plannerNames(); v.push_back(kMssName); v.push_back(kExpName); return v;
}

struct Button { cv::Rect r; int group; int value; std::string label; };
// groups: 0 planner, 1 arena, 2 sensor, 3 action(value: 0 restart,1 pause,2 newseed,3 quit)

std::vector<Button> buildButtons(int panelW) {
    std::vector<Button> b; int y = 30; const int x=8, w=panelW-16, h=20, gap=2;
    auto section=[&](int group, const std::vector<std::string>& items){
        for (int i=0;i<(int)items.size();++i){ b.push_back({cv::Rect(x,y,w,h), group, i, items[i]}); y+=h+gap; }
        y += 14;
    };
    section(0, allModeNames());
    section(1, kArenas);
    section(2, kSensors);
    b.push_back({cv::Rect(x, y, w/2-2, 24), 3, 0, "Restart"});
    b.push_back({cv::Rect(x+w/2+2, y, w/2-2, 24), 3, 2, "New seed"}); y+=28;
    b.push_back({cv::Rect(x, y, w/2-2, 24), 3, 1, "Pause"});
    b.push_back({cv::Rect(x+w/2+2, y, w/2-2, 24), 3, 3, "Quit"});
    return b;
}

cv::Mat renderComposite(SimEpisode& ep, const std::vector<Button>& btns,
                        int selP, int selA, int selS, bool paused) {
    const int panelW=200, viewW=620, H=640;
    cv::Mat canvas(H, panelW+viewW, CV_8UC3, cv::Scalar(18,18,20));
    // ---- control panel
    cv::rectangle(canvas, cv::Rect(0,0,panelW,H), cv::Scalar(30,30,34), -1);
    auto hdr=[&](const char* t,int y){ cv::putText(canvas,t,{8,y},cv::FONT_HERSHEY_SIMPLEX,0.42,{150,180,255},1); };
    hdr("PLANNER", 22);
    for (auto& bt : btns) {
        if (bt.group==1 && bt.value==0) hdr("ARENA",  bt.r.y-6);
        if (bt.group==2 && bt.value==0) hdr("SENSOR", bt.r.y-6);
        bool sel = (bt.group==0&&bt.value==selP)||(bt.group==1&&bt.value==selA)||(bt.group==2&&bt.value==selS);
        bool isPause = (bt.group==3&&bt.value==1);
        cv::Scalar bg = sel ? cv::Scalar(70,120,60) : cv::Scalar(50,50,55);
        if (bt.group==3) bg = cv::Scalar(60,60,75);
        if (isPause && paused) bg = cv::Scalar(60,110,150);
        cv::rectangle(canvas, bt.r, bg, -1);
        cv::rectangle(canvas, bt.r, cv::Scalar(80,80,88), 1);
        std::string lbl = (isPause && paused) ? "Play" : bt.label;
        cv::putText(canvas, lbl, {bt.r.x+6, bt.r.y+bt.r.height-6}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {235,235,235}, 1);
    }
    // ---- FPV (top right)
    const int fpvH=250;
    cv::Mat fpv = sim::renderFPV(ep.ranges, ep.opt.maxRange, viewW, fpvH);
    if (!ep.ranges.empty()) {   // corridor arrow (offset from the openest ray)
        cv::putText(fpv, "FPV", {8,20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 1);
    }
    fpv.copyTo(canvas(cv::Rect(panelW, 0, viewW, fpvH)));
    // ---- top-down mapped (bottom right)
    const float cE=ep.goal.e*0.5f, cN=ep.goal.n*0.5f;
    const float span=std::max(26.f, std::hypot(ep.goal.e,ep.goal.n)*1.7f+10.f);
    const int tdSize = H - fpvH - 8;
    cv::Mat td = renderGridView(ep.grid, ep.world, ep.drone, ep.goal, ep.lastPath, ep.trail,
                                ep.ranges, ep.opt.hFov, ep.opt.maxRange,
                                ep.plannerName.c_str(), ep.sensorName.c_str(), cE, cN, tdSize, span);
    td.copyTo(canvas(cv::Rect(panelW + (viewW-tdSize)/2, fpvH+4, tdSize, tdSize)));
    // ---- status line
    char st[220];
    std::snprintf(st, sizeof(st), "%s | seed %u | %s%s%s | %s%s  steps %d  standoff %.2fm",
        ep.arenaName.c_str(), ep.seed,
        ep.opt.noise?"[NOISE] ":"",
        paused?"[PAUSED] ":"",
        ep.reached?"REACHED":(ep.collided?"COLLIDED":"running"),
        ep.plannerName.c_str(),
        (ep.mss||ep.explore) ? ("  phase:"+ep.phase).c_str() : "",
        ep.steps, ep.minStandoff);
    cv::putText(canvas, st, {panelW+8, H-10}, cv::FONT_HERSHEY_SIMPLEX, 0.42, {200,220,200}, 1);
    return canvas;
}

#if defined(SIM_HAVE_HIGHGUI)
struct Click { int x=-1,y=-1; bool pending=false; };
static void onMouse(int e,int x,int y,int,void* u){ if(e==cv::EVENT_LBUTTONDOWN){ auto*c=(Click*)u; c->x=x;c->y=y;c->pending=true; } }

int runGui(bool noise) {
    SimEpisode ep; int selP=2 /*astar*/, selA=4 /*maze*/, selS=0 /*camera*/;
    ep.opt.noise = noise;
    ep.plannerName=allModeNames()[selP]; ep.arenaName=kArenas[selA]; ep.sensorName=kSensors[selS];
    ep.restart();
    auto btns = buildButtons(200);
    Click click; bool paused=false; int doneDwell=0;
    const std::string win="nav-sim"; cv::namedWindow(win); cv::setMouseCallback(win, onMouse, &click);
    std::printf("[gui] click the panel to choose planner/arena/sensor. q or Quit to exit.\n");
    while (true) {
        if (!paused) { ep.step(); ep.step(); }        // 2 ticks/frame for pace
        if ((ep.reached||ep.collided) && !paused) { if (++doneDwell>40){ ep.restart(); doneDwell=0; } }
        cv::Mat frame = renderComposite(ep, btns, selP, selA, selS, paused);
        cv::imshow(win, frame);
        const int k = cv::waitKey(20) & 0xFF;
        if (k=='q' || k==27) break;
        if (k==' ') paused=!paused;
        if (k=='r') { ep.restart(); doneDwell=0; }
        if (k=='n') { ep.seed++; ep.restart(); doneDwell=0; }
        if (click.pending) {
            click.pending=false;
            for (auto& bt : btns) if (bt.r.contains({click.x,click.y})) {
                if (bt.group==0){ selP=bt.value; ep.plannerName=allModeNames()[selP]; ep.restart(); }
                else if (bt.group==1){ selA=bt.value; ep.arenaName=kArenas[selA]; ep.restart(); }
                else if (bt.group==2){ selS=bt.value; ep.sensorName=kSensors[selS]; ep.restart(); }
                else if (bt.group==3){ if(bt.value==0) ep.restart();
                    else if(bt.value==1) paused=!paused;
                    else if(bt.value==2){ ep.seed++; ep.restart(); }
                    else if(bt.value==3) return 0; }
                doneDwell=0; break;
            }
        }
    }
    return 0;
}
#endif
}  // namespace

int main(int argc, char** argv) {
    RunOpts opt; std::string planner="astar", worldSel="random", goalSel="random";
    unsigned seed=1; int batch=0, nObs=6; bool compare=false, listP=false, gui=false;
    std::string guiShot;

    for (int i=1;i<argc;++i){ std::string a=argv[i];
        auto val=[&](const char*k)->std::string{ size_t L=std::strlen(k);
            return (a.rfind(k,0)==0&&a.size()>L)?a.substr(L):std::string(); };
        if(a=="--display")opt.display=true; else if(a=="--compare")compare=true;
        else if(a=="--gui")gui=true;
        else if(a=="--noise")opt.noise=true;
        else if(!val("--gui-shot=").empty())guiShot=val("--gui-shot=");
        else if(a=="--list-planners")listP=true;
        else if(!val("--planner=").empty())planner=val("--planner=");
        else if(!val("--world=").empty())worldSel=val("--world=");
        else if(!val("--goal=").empty())goalSel=val("--goal=");
        else if(!val("--seed=").empty())seed=(unsigned)std::atoi(val("--seed=").c_str());
        else if(!val("--batch=").empty())batch=std::atoi(val("--batch=").c_str());
        else if(!val("--obstacles=").empty())nObs=std::atoi(val("--obstacles=").c_str());
        else if(!val("--save=").empty())opt.saveDir=val("--save=");
        else if(!val("--sensor=").empty())opt.sensorName=val("--sensor=");
    }
    if (listP){ std::printf("modes:\n"); for(auto&n:allModeNames()) std::printf("  %s\n",n.c_str());
                std::printf("  (move-stop-sense = the drone's real onboard navigation, ported;\n");
                std::printf("   explore-rth = frontier exploration then return-to-home)\n");
                std::printf("sensors:\n  tof (45 deg/4m)  tof-wide (63 deg/9m)  camera (90 deg/8m)\n");
                std::printf("arenas (--world=):\n  random empty slalom rooms maze trap cluttered\n");
                std::printf("  comb bottleneck gap-choice double-trap pillars\n"); return 0; }

    // apply the sensor FOV preset
    { Sensor sp = sensorPreset(opt.sensorName); opt.sensorName = sp.name;
      opt.hFov = sp.fovDeg; opt.maxRange = sp.rangeM; opt.nRays = sp.rays; }

    // ---- headless one-frame GUI render (for verifying layout without a display)
    if (!guiShot.empty()) {
        SimEpisode ep;
        ep.plannerName = (planner!="astar"||worldSel=="random") ? planner : "astar";
        ep.arenaName = (worldSel!="random") ? worldSel : "maze";
        ep.sensorName = opt.sensorName; ep.seed = seed; ep.opt.noise = opt.noise; ep.restart();
        for (int s=0; s<160; ++s) ep.step();
        int sp=0; { auto ns=allModeNames(); for(int i=0;i<(int)ns.size();++i) if(ns[i]==ep.plannerName) sp=i; }
        int sa=4; for(int i=0;i<(int)kArenas.size();++i) if(kArenas[i]==ep.arenaName) sa=i;
        int ss=0; for(int i=0;i<(int)kSensors.size();++i) if(kSensors[i]==ep.sensorName) ss=i;
        cv::Mat f = renderComposite(ep, buildButtons(200), sp, sa, ss, false);
        cv::imwrite(guiShot, f);
        std::printf("wrote %s\n", guiShot.c_str());
        return 0;
    }

    // ---- interactive GUI
    if (gui) {
#if defined(SIM_HAVE_HIGHGUI)
        return runGui(opt.noise);
#else
        std::fprintf(stderr, "--gui needs an OpenCV built with highgui (this build has none).\n");
        return 2;
#endif
    }

    auto makeGoal=[&](std::mt19937&rng,const sim::World&w)->navsim::Vec2{
        if(goalSel!="random"){ float e=0,n=0; if(std::sscanf(goalSel.c_str(),"%f,%f",&e,&n)==2) return {e,n}; }
        return randomGoal(rng,w); };
    // dispatch a mode by name: the move-stop-sense controller or a path planner.
    auto runMode=[&](const std::string& name, const sim::World& w, navsim::Vec2 g, bool vb)->RunResult{
        if (name==kMssName) return runMss(w, g, opt, vb);
        if (name==kExpName) return runExplore(w, opt, vb);
        auto p = navsim::makePlanner(name); if (!p) return RunResult{};
        return run(*p, w, g, opt, vb); };

    // ---- batch (mass data) ------------------------------------------------
    if (batch>0){
        if (planner!=kMssName && planner!=kExpName && !navsim::makePlanner(planner)){ std::fprintf(stderr,"unknown planner '%s'\n",planner.c_str()); return 2; }
        int reached=0, collided=0; float worst=1e9f; double planMs=0; long calls=0; float lenRatio=0;
        for(int i=0;i<batch;++i){ std::mt19937 rng(seed+i);
            Arena ar = buildArena(worldSel, rng, nObs);
            navsim::Vec2 g = (goalSel!="random") ? makeGoal(rng, ar.world) : ar.goal;
            RunResult r = runMode(planner, ar.world, g, false);
            if(r.reached)reached++; if(r.collided)collided++; worst=std::min(worst,r.minStandoff);
            planMs+=r.planMsTotal; calls+=r.planCalls; if(r.straightLen>0&&r.reached) lenRatio+=r.pathLen/r.straightLen;
            if((i+1)%50==0||i+1==batch) std::printf("  [%4d/%d] reached=%d collided=%d worst=%.2fm\n",i+1,batch,reached,collided,worst);
        }
        std::printf("\n=== BATCH: planner=%s, %d fields (%d obstacles), seed base %u ===\n",planner.c_str(),batch,nObs,seed);
        std::printf("reached      : %d/%d (%.1f%%)\n",reached,batch,100.f*reached/batch);
        std::printf("collisions   : %d/%d\n",collided,batch);
        std::printf("worst standoff: %.2fm\n",worst);
        std::printf("avg path/straight (reached): %.2fx\n", reached? lenRatio/reached : 0.f);
        std::printf("avg plan time: %.3f ms/call\n", calls? planMs/calls : 0.0);
        return collided==0?0:1;
    }

    // one world+goal from the seed
    std::mt19937 rng(seed);
    Arena arena = buildArena(worldSel, rng, nObs);
    sim::World world = arena.world;
    navsim::Vec2 goal = (goalSel!="random") ? makeGoal(rng, world) : arena.goal;
    std::printf("world: %s (%zu obstacles)  goal: (%.1f, %.1f)  seed: %u\n",
                worldSel.c_str(), world.circles.size()+world.walls.size(), goal.e, goal.n, seed);

    // ---- compare all modes on the SAME world+goal (planners + the real nav) ----
    if (compare){
        std::printf("\n=== MODE COMPARISON (same world+goal) ===\n");
        for(auto&nm:allModeNames()){ RunResult r = runMode(nm, world, goal, false); printRow(nm.c_str(), r); }
        return 0;
    }

    // ---- single mode ------------------------------------------------------
    if(planner!=kMssName && planner!=kExpName && !navsim::makePlanner(planner)){ std::fprintf(stderr,"unknown planner '%s' (try --list-planners)\n",planner.c_str()); return 2; }
    std::printf("planner: %s\n\n", planner.c_str());
    RunResult r = runMode(planner, world, goal, true);
    std::printf("\n"); printRow(planner.c_str(), r);
    std::printf("VERDICT: %s\n", (!r.collided && r.reached)?"PASS":(r.collided?"COLLISION":"did not reach"));
    return (!r.collided && r.reached)?0:1;
}
