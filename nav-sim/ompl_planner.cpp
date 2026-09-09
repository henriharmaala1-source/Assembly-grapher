#include "ompl_planner.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#ifdef NAVSIM_HAVE_OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
#endif

namespace sim {

// RRTConnect is a RANDOMISED planner, and until this existed its RNG was seeded
// from the clock. The consequence was worse than it sounds: two runs of the
// same binary with the same --seed produced different trajectories, so the
// harness could not answer "did my change alter behaviour?" at all, and every
// multi-seed table in this tree silently mixed world variation with run-to-run
// variation. Nothing was measurably wrong -- the effects that mattered were far
// larger than the noise -- but the precision was overstated and a bit-identical
// refactor was untestable.
//
// Must be called before any ompl::RNG is constructed, i.e. before the first
// planForward. Seeding from the world seed keeps one knob rather than two.
void setPlannerSeed(unsigned seed) {
#ifdef NAVSIM_HAVE_OMPL
    ompl::RNG::setSeed(seed ? seed : 1u);
#else
    (void)seed;
#endif
}

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
                    float r, bool unknownOk, const VoxelMap* far = nullptr) {
    static const float O[7][3] = {{0,0,0},{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (const auto& o : O) {
        float qx = x + o[0]*r, qy = y + o[1]*r, qz = z + o[2]*r;
        VoxelMap::State s = m.stateAt(qx, qy, qz);
        // Fine map first, always. It only defers where it genuinely has nothing
        // to say, so the coarse map can add obstacles but never erase one.
        if (s == VoxelMap::UNKNOWN && far) {
            VoxelMap::State fs = far->stateAt(qx, qy, qz);
            if (fs != VoxelMap::UNKNOWN) s = fs;
        }
        if (s == VoxelMap::OCCUPIED) return false;
        if (!unknownOk && s == VoxelMap::UNKNOWN) return false;
    }
    return true;
}

static inline float wrapDeg(float d) {
    return std::fmod(d + 540.f, 360.f) - 180.f;
}

bool pursuitPoint(const ForwardPath& p, float px, float py, float pz,
                  float lookaheadM, float& tx, float& ty, float& tz) {
    if (!p.found || p.pts.size() < 2) return false;

    // Nearest point on the polyline, as a (segment, fraction) pair. Projecting
    // onto segments rather than snapping to the nearest VERTEX matters: with
    // 2 m spacing, vertex snapping quantises the lookahead origin and puts back
    // most of the jumping this function exists to remove.
    size_t bestSeg = 0; float bestU = 0, bestD2 = 1e30f;
    for (size_t i = 0; i + 1 < p.pts.size(); ++i) {
        float ax=p.pts[i][0], ay=p.pts[i][1], az_=p.pts[i][2];
        float vx=p.pts[i+1][0]-ax, vy=p.pts[i+1][1]-ay, vz=p.pts[i+1][2]-az_;
        float l2 = vx*vx + vy*vy + vz*vz;
        float u = l2 > 1e-9f ? ((px-ax)*vx + (py-ay)*vy + (pz-az_)*vz) / l2 : 0.f;
        u = std::max(0.f, std::min(1.f, u));
        float dx=px-(ax+u*vx), dy=py-(ay+u*vy), dz=pz-(az_+u*vz);
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestD2) { bestD2 = d2; bestSeg = i; bestU = u; }
    }

    // Walk forward that much arclength from there.
    float remain = lookaheadM;
    for (size_t i = bestSeg; i + 1 < p.pts.size(); ++i) {
        float ax=p.pts[i][0], ay=p.pts[i][1], az_=p.pts[i][2];
        float vx=p.pts[i+1][0]-ax, vy=p.pts[i+1][1]-ay, vz=p.pts[i+1][2]-az_;
        float segLen = std::sqrt(vx*vx + vy*vy + vz*vz);
        float u0 = (i == bestSeg) ? bestU : 0.f;
        float avail = segLen * (1.f - u0);
        if (remain <= avail || i + 2 == p.pts.size()) {
            float u = u0 + (segLen > 1e-9f ? std::min(remain, avail) / segLen : 0.f);
            u = std::min(1.f, u);
            tx = ax + vx*u; ty = ay + vy*u; tz = az_ + vz*u;
            return true;
        }
        remain -= avail;
    }
    // Path shorter than the lookahead: aim at its end. Correct rather than a
    // fallback -- there is nothing further along to aim at.
    tx = p.pts.back()[0]; ty = p.pts.back()[1]; tz = p.pts.back()[2];
    return true;
}

void BearingFilter::update(float az, float el, float alpha) {
    if (!have || alpha >= 1.f) { azDeg = az; elDeg = el; have = true; return; }
    // Angular EMA, which must go the short way round: filtering 359 -> 1
    // linearly sweeps the aircraft through 180 degrees to travel 2.
    azDeg = wrapDeg(azDeg + alpha * wrapDeg(az - azDeg) + 360.f);
    elDeg += alpha * (el - elDeg);
}

static inline void dirFrom(float azDeg, float elDeg, float& dx, float& dy, float& dz) {
    float a = azDeg * sim::PI_F / 180.f, e = elDeg * sim::PI_F / 180.f;
    dx = std::cos(e) * std::sin(a);
    dy = std::cos(e) * std::cos(a);
    dz = std::sin(e);
}

bool pathStillGood(const VoxelMap& m, const ForwardPath& p,
                   float px, float py, float pz,
                   float wantAzDeg, const ForwardParams& fp,
                   const VoxelMap* far) {
    if (!p.found || p.pts.size() < 2) return false;
    const bool uok = fp.unknownOk != 0;

    // Where are we on it, and how much is left ahead?
    size_t bestSeg = 0; float bestD2 = 1e30f;
    for (size_t i = 0; i < p.pts.size(); ++i) {
        float dx=px-p.pts[i][0], dy=py-p.pts[i][1], dz=pz-p.pts[i][2];
        float d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < bestD2) { bestD2 = d2; bestSeg = i; }
    }
    // Drifted off it. The reactive layer is allowed to deviate -- that is its
    // job -- but past a few metres the path is describing a route we are no
    // longer on, and following it is worse than admitting that.
    if (std::sqrt(bestD2) > 4.f) return false;

    float ahead = 0;
    for (size_t i = bestSeg; i + 1 < p.pts.size(); ++i)
        ahead += std::sqrt(
            (p.pts[i+1][0]-p.pts[i][0])*(p.pts[i+1][0]-p.pts[i][0]) +
            (p.pts[i+1][1]-p.pts[i][1])*(p.pts[i+1][1]-p.pts[i][1]) +
            (p.pts[i+1][2]-p.pts[i][2])*(p.pts[i+1][2]-p.pts[i][2]));
    if (ahead < fp.horizonM * 0.4f) return false;      // nearly consumed

    // Still pointing somewhere useful?
    if (std::fabs(wrapDeg(p.bearingDeg - wantAzDeg)) > fp.coneDeg) return false;

    // Still safe? Only the part AHEAD -- whether the bit we already flew is
    // still clear is not a question that can affect anything.
    for (size_t i = bestSeg; i < p.pts.size(); ++i)
        if (!stateOk(m, p.pts[i][0], p.pts[i][1], p.pts[i][2], fp.robotR, uok, far))
            return false;
    return true;
}

ForwardPath planForward(const VoxelMap& m, float sx, float sy, float sz,
                        float goalAzDeg, float goalElDeg,
                        const ForwardParams& p, const VoxelMap* far) {
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
    ss.setStateValidityChecker([&m, &p, uok, far](const ob::State* st) {
        const auto* v = st->as<ob::RealVectorStateSpace::StateType>();
        return stateOk(m, float((*v)[0]), float((*v)[1]), float((*v)[2]),
                       p.robotR, uok, far);
    });
    // Motion validation resolution: a fraction of the space extent. Too coarse
    // and the planner tunnels through thin obstacles -- exactly the failure the
    // hand-rolled probe had -- so tie it to the voxel size.
    ss.getSpaceInformation()->setStateValidityCheckingResolution(
        std::min(0.02, double(m.params().cell * 0.5f) / (H * 2.4)));
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    ob::ScopedState<> start(space);
    start[0] = sx; start[1] = sy; start[2] = sz;
    if (!stateOk(m, sx, sy, sz, p.robotR, true, far)) {
        // Vehicle believes it is inside something. Do not plan; the reactive
        // layer's escape behaviour is the right response, and a path from an
        // invalid start is meaningless.
        return out;
    }

    for (float off : cand) {
        float dx, dy, dz; dirFrom(goalAzDeg + off, goalElDeg, dx, dy, dz);
        float gxp = sx + dx * H, gyp = sy + dy * H, gzp = sz + dz * H;
        if (!stateOk(m, gxp, gyp, gzp, p.robotR, uok, far)) continue;

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
            if (!stateOk(m, sx + dx*t, sy + dy*t, sz + dz*t, p.robotR, uok, far)) { clear = false; break; }
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
