#include "ompl_planner.hpp"

#include <algorithm>
#include <cmath>

#ifdef NAVSIM_HAVE_OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
#endif

namespace sim {

const char* forwardPlannerName() {
#ifdef NAVSIM_HAVE_OMPL
    return "OMPL RRTConnect";
#else
    return "fallback (straight-line probe)";
#endif
}

// Shared validity test, used by both backends so they cannot disagree about
// what "safe" means. A sphere of robotR must contain no OCCUPIED cell; UNKNOWN
// is permitted or not per unknownOk.
static bool stateOk(const VoxelMap& m, float x, float y, float z,
                    float r, bool unknownOk) {
    static const float O[7][3] = {{0,0,0},{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (const auto& o : O) {
        VoxelMap::State s = m.stateAt(x + o[0]*r, y + o[1]*r, z + o[2]*r);
        if (s == VoxelMap::OCCUPIED) return false;
        if (!unknownOk && s == VoxelMap::UNKNOWN) return false;
    }
    return true;
}

static inline void dirFrom(float azDeg, float elDeg, float& dx, float& dy, float& dz) {
    float a = azDeg * float(M_PI) / 180.f, e = elDeg * float(M_PI) / 180.f;
    dx = std::cos(e) * std::sin(a);
    dy = std::cos(e) * std::cos(a);
    dz = std::sin(e);
}

ForwardPath planForward(const VoxelMap& m, float sx, float sy, float sz,
                        float goalAzDeg, float goalElDeg,
                        const ForwardParams& p) {
    ForwardPath out;
    const bool uok = p.unknownOk != 0;

    // Candidate horizon targets, ordered best-first: straight ahead, then
    // alternating left/right within the cone. Trying several is what lets the
    // planner route AROUND something without needing a global goal -- the first
    // reachable candidate wins, and the reactive layer handles the rest.
    std::vector<float> cand;
    cand.push_back(0.f);
    for (int i = 1; i <= p.nCandidates / 2; ++i) {
        float f = p.coneDeg * float(i) / float(std::max(1, p.nCandidates / 2));
        cand.push_back(+f);
        cand.push_back(-f);
    }

#ifdef NAVSIM_HAVE_OMPL
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);   // OMPL is chatty by default
    const float H = p.horizonM;
    // Search box: a cube around the vehicle big enough to hold any candidate
    // path, clipped to the map. Bounding the space tightly is most of why this
    // is fast -- an unbounded RealVectorStateSpace would sample uselessly.
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    ob::RealVectorBounds b(3);
    b.setLow(0, sx - H * 1.2); b.setHigh(0, sx + H * 1.2);
    b.setLow(1, sy - H * 1.2); b.setHigh(1, sy + H * 1.2);
    b.setLow(2, sz - H * 0.5); b.setHigh(2, sz + H * 0.5);
    space->setBounds(b);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([&m, &p, uok](const ob::State* st) {
        const auto* v = st->as<ob::RealVectorStateSpace::StateType>();
        return stateOk(m, float((*v)[0]), float((*v)[1]), float((*v)[2]),
                       p.robotR, uok);
    });
    // Motion validation resolution: a fraction of the space extent. Too coarse
    // and the planner tunnels through thin obstacles -- exactly the failure the
    // hand-rolled probe had -- so tie it to the voxel size.
    ss.getSpaceInformation()->setStateValidityCheckingResolution(
        std::min(0.02, double(m.params().cell * 0.5f) / (H * 2.4)));
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    ob::ScopedState<> start(space);
    start[0] = sx; start[1] = sy; start[2] = sz;
    if (!stateOk(m, sx, sy, sz, p.robotR, true)) {
        // Vehicle believes it is inside something. Do not plan; the reactive
        // layer's escape behaviour is the right response, and a path from an
        // invalid start is meaningless.
        return out;
    }

    for (float off : cand) {
        float dx, dy, dz; dirFrom(goalAzDeg + off, goalElDeg, dx, dy, dz);
        float gxp = sx + dx * H, gyp = sy + dy * H, gzp = sz + dz * H;
        if (!stateOk(m, gxp, gyp, gzp, p.robotR, uok)) continue;

        ob::ScopedState<> goal(space);
        goal[0] = gxp; goal[1] = gyp; goal[2] = gzp;
        ss.clear();
        ss.setStartAndGoalStates(start, goal, p.robotR);

        auto t0 = std::chrono::steady_clock::now();
        ob::PlannerStatus solved = ss.solve(p.planTimeS);
        if (!solved || !ss.haveExactSolutionPath()) continue;
        ss.simplifySolution(p.simplifyS);
        auto pth = ss.getSolutionPath();
        pth.interpolate(std::max<unsigned>(8, unsigned(pth.length() / 2.0)));

        out.solveMs = float(std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t0).count());
        out.found = true;
        out.lengthM = float(pth.length());
        out.bearingDeg = goalAzDeg + off;
        for (const auto* st : pth.getStates()) {
            const auto* v = st->as<ob::RealVectorStateSpace::StateType>();
            out.pts.push_back({float((*v)[0]), float((*v)[1]), float((*v)[2])});
        }
        return out;
    }
    return out;
#else
    // No OMPL: straight-line probe to the first clear candidate. Deliberately
    // dumb -- it exists so the build works without the dependency, not to
    // compete. The reactive layer does the real work in this configuration.
    for (float off : cand) {
        float dx, dy, dz; dirFrom(goalAzDeg + off, goalElDeg, dx, dy, dz);
        bool clear = true;
        float step = m.params().cell * 0.75f;
        for (float t = step; t <= p.horizonM; t += step)
            if (!stateOk(m, sx + dx*t, sy + dy*t, sz + dz*t, p.robotR, uok)) { clear = false; break; }
        if (!clear) continue;
        out.found = true; out.lengthM = p.horizonM; out.bearingDeg = goalAzDeg + off;
        for (int i = 1; i <= 8; ++i) {
            float t = p.horizonM * float(i) / 8.f;
            out.pts.push_back({sx + dx*t, sy + dy*t, sz + dz*t});
        }
        return out;
    }
    return out;
#endif
}

}  // namespace sim
