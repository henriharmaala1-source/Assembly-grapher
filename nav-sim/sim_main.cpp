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

struct RunOpts {
    bool  display = false;
    std::string saveDir;
    int   nRays = 61; float hFov = 90.f, maxRange = 8.f;
    int   viz = 480;
};

struct RunResult {
    bool   reached = false, collided = false;
    float  minStandoff = 1e9f, pathLen = 0.f, straightLen = 0.f, simTime = 0.f;
    int    replans = 0; double planMsTotal = 0.0; long planCalls = 0;
};

// Draw the occupancy grid + planned path + trail, top-down.
cv::Mat renderGridView(const navsim::OccupancyGrid& g, const sim::World& w,
                       const navsim::Drone& d, navsim::Vec2 goal,
                       const std::vector<navsim::Vec2>& path,
                       const std::vector<cv::Point2f>& trail,
                       const char* planner, int size, float spanM) {
    cv::Mat img(size, size, CV_8UC3, cv::Scalar(24,24,26));
    const float ppm = size / spanM;
    auto toPx = [&](float e, float n){ return cv::Point((int)(size/2 + e*ppm), (int)(size/2 - n*ppm)); };
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
    // true obstacles (outline, so belief vs truth is visible)
    for (auto& c : w.circles) cv::circle(img, toPx(c.e,c.n), (int)(c.r*ppm), {80,110,220}, 1, cv::LINE_AA);
    for (auto& wl : w.walls)  cv::line(img, toPx(wl.e0,wl.n0), toPx(wl.e1,wl.n1), {80,110,220}, 2, cv::LINE_AA);
    // planned path
    for (size_t i=1;i<path.size();++i) cv::line(img, toPx(path[i-1].e,path[i-1].n), toPx(path[i].e,path[i].n), {80,220,255}, 2, cv::LINE_AA);
    // trail
    for (size_t i=1;i<trail.size();++i) cv::line(img, toPx(trail[i-1].x,trail[i-1].y), toPx(trail[i].x,trail[i].y), {90,200,90}, 1, cv::LINE_AA);
    // goal + drone
    cv::drawMarker(img, toPx(goal.e,goal.n), {80,255,80}, cv::MARKER_STAR, 16, 2, cv::LINE_AA);
    const cv::Point dp = toPx(d.e,d.n); const float yr = d.yawDeg*kPi/180.f;
    cv::arrowedLine(img, dp, cv::Point(dp.x+(int)(22*std::sin(yr)), dp.y-(int)(22*std::cos(yr))), {255,255,255}, 2, cv::LINE_AA,0,0.3);
    cv::circle(img, dp, 4, {255,255,255}, -1, cv::LINE_AA);
    cv::putText(img, planner, {8,22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {235,235,235}, 1);
    return img;
}

RunResult run(navsim::IPlanner& planner, const sim::World& worldIn, navsim::Vec2 goal,
              const RunOpts& opt, bool verbose) {
    sim::World world = worldIn;
    navsim::OccupancyGrid grid;
    navsim::Drone drone;   // starts at origin, facing the goal
    drone.yawDeg = std::atan2(goal.e, goal.n) * 180.f / kPi;
    const int inflate = (int)std::ceil(1.5f / grid.cellM());   // ~1.5 m robot berth

    RunResult R; R.straightLen = std::hypot(goal.e, goal.n);
    std::vector<cv::Point2f> trail; std::vector<navsim::Vec2> lastPath;
    const float dt = 0.05f; float t = 0.f; float lastE = drone.e, lastN = drone.n;

    for (int step = 0; step < 6000 && !R.reached; ++step) {
        world.advance(dt);
        // ---- perception: forward scan -> integrate the grid ----------------
        std::vector<float> ranges;
        sim::castScan(world, drone.e, drone.n, drone.yawDeg, opt.hFov, opt.nRays, opt.maxRange, ranges);
        grid.integrate(drone.e, drone.n, drone.yawDeg, ranges, opt.hFov, opt.maxRange);

        // ---- plan (every tick, on the latest partial map) ------------------
        navsim::PlanResult pr = planner.plan(grid, {drone.e,drone.n}, goal, inflate);
        R.planMsTotal += pr.planMs; ++R.planCalls; if (pr.ok) ++R.replans;
        lastPath = pr.path;

        // ---- follow: lookahead waypoint along the path ---------------------
        float targetBearing = std::atan2(goal.e-drone.e, goal.n-drone.n) * 180.f/kPi;
        if (pr.ok && pr.path.size() >= 2) {
            const float lookahead = 2.5f; float acc = 0.f; navsim::Vec2 wp = pr.path.back();
            for (size_t i=1;i<pr.path.size();++i){ acc += std::hypot(pr.path[i].e-pr.path[i-1].e, pr.path[i].n-pr.path[i-1].n);
                if (acc >= lookahead){ wp = pr.path[i]; break; } }
            targetBearing = std::atan2(wp.e-drone.e, wp.n-drone.n) * 180.f/kPi;
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
            cv::Mat top = renderGridView(grid, world, drone, goal, lastPath, trail, planner.name(), opt.viz, 44.f);
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

void printRow(const char* name, const RunResult& r) {
    std::printf("  %-10s reached=%-3s collided=%-3s standoff=%5.2fm pathLen=%6.2fm (%.2fx) "
                "avgPlan=%.3fms\n",
                name, r.reached?"YES":"no", r.collided?"YES":"no", r.minStandoff, r.pathLen,
                r.straightLen>0? r.pathLen/r.straightLen : 0.f,
                r.planCalls? r.planMsTotal/r.planCalls : 0.0);
}
}  // namespace

int main(int argc, char** argv) {
    RunOpts opt; std::string planner="astar", worldSel="random", goalSel="random";
    unsigned seed=1; int batch=0, nObs=6; bool compare=false, listP=false;

    for (int i=1;i<argc;++i){ std::string a=argv[i];
        auto val=[&](const char*k)->std::string{ size_t L=std::strlen(k);
            return (a.rfind(k,0)==0&&a.size()>L)?a.substr(L):std::string(); };
        if(a=="--display")opt.display=true; else if(a=="--compare")compare=true;
        else if(a=="--list-planners")listP=true;
        else if(!val("--planner=").empty())planner=val("--planner=");
        else if(!val("--world=").empty())worldSel=val("--world=");
        else if(!val("--goal=").empty())goalSel=val("--goal=");
        else if(!val("--seed=").empty())seed=(unsigned)std::atoi(val("--seed=").c_str());
        else if(!val("--batch=").empty())batch=std::atoi(val("--batch=").c_str());
        else if(!val("--obstacles=").empty())nObs=std::atoi(val("--obstacles=").c_str());
        else if(!val("--save=").empty())opt.saveDir=val("--save=");
    }
    if (listP){ std::printf("planners:\n"); for(auto&n:navsim::plannerNames()) std::printf("  %s\n",n.c_str()); return 0; }

    auto makeGoal=[&](std::mt19937&rng,const sim::World&w)->navsim::Vec2{
        if(goalSel!="random"){ float e=0,n=0; if(std::sscanf(goalSel.c_str(),"%f,%f",&e,&n)==2) return {e,n}; }
        return randomGoal(rng,w); };

    // ---- batch (mass data) ------------------------------------------------
    if (batch>0){
        auto p = navsim::makePlanner(planner);
        if(!p){ std::fprintf(stderr,"unknown planner '%s'\n",planner.c_str()); return 2; }
        int reached=0, collided=0; float worst=1e9f; double planMs=0; long calls=0; float lenRatio=0;
        for(int i=0;i<batch;++i){ std::mt19937 rng(seed+i);
            sim::World w = randomWorld(rng,nObs); navsim::Vec2 g = makeGoal(rng,w);
            RunResult r = run(*p,w,g,opt,false);
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
    sim::World world = (worldSel=="random")? randomWorld(rng,nObs) : sim::World{};
    navsim::Vec2 goal = makeGoal(rng,world);
    std::printf("world: %s (%zu obstacles)  goal: (%.1f, %.1f)  seed: %u\n",
                worldSel.c_str(), world.circles.size()+world.walls.size(), goal.e, goal.n, seed);

    // ---- compare all planners on the SAME world+goal ----------------------
    if (compare){
        std::printf("\n=== PLANNER COMPARISON (same world+goal) ===\n");
        for(auto&nm:navsim::plannerNames()){ auto p=navsim::makePlanner(nm);
            RunResult r = run(*p,world,goal,opt,false); printRow(nm.c_str(), r); }
        return 0;
    }

    // ---- single planner ---------------------------------------------------
    auto p = navsim::makePlanner(planner);
    if(!p){ std::fprintf(stderr,"unknown planner '%s' (try --list-planners)\n",planner.c_str()); return 2; }
    std::printf("planner: %s\n\n", planner.c_str());
    RunResult r = run(*p, world, goal, opt, true);
    std::printf("\n"); printRow(planner.c_str(), r);
    std::printf("VERDICT: %s\n", (!r.collided && r.reached)?"PASS":(r.collided?"COLLISION":"did not reach"));
    return (!r.collided && r.reached)?0:1;
}
