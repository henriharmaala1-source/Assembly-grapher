// Headless software-in-the-loop validation of the "pick a direction, press GO"
// AUTONOMY mode. Wires the REAL components:
//
//   SimFcBackend (kinematic FC) -> StateEstimator (ENU EKF) -> synthetic
//   goal-aware VFH* ray-cast perception (corridorOffset/corridorOpen) ->
//   MissionController (move-stop-sense) -> back to the FC.
//
// Runs a suite of scenarios (goal 25 m East, press-GO). Each tick sets the
// operator input (goal bearing + GO latch). It PASSES when the drone never
// breaches its obstacle standoff in ANY scenario AND reaches the goal in the
// scenarios the reactive layer is designed for (clear / off-path / grazing).
// Obstacles sitting ON the path are expected NOT to complete (reactive local
// minima) but MUST still keep standoff — that completion needs the roadmap's
// deferred occupancy-grid + global planner.
//
// The synthetic perception models what a goal-aware local planner produces:
// obstacles inflated by the vehicle+margin radius, the openest gap nearest the
// goal bearing chosen (with previous-direction hysteresis), and forward-cone
// clearance as the "can I move / keep standoff" openness signal.

#include <cmath>
#include <cstdio>
#include <vector>

#include "sim_fc_backend.hpp"
#include "state_estimator.hpp"
#include "mission.hpp"
#include "world_model.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrap180(float d) {
    while (d > 180.f)  d -= 360.f;
    while (d <= -180.f) d += 360.f;
    return d;
}

// One circular obstacle in the local ENU plane.
struct Obstacle { float e, n, r; };

// Ray clearance from p along unit dir d to circle (c,r); capped at maxRange.
float rayClearance(float pe, float pn, float de, float dn,
                   const Obstacle& o, float maxRange) {
    const float me = pe - o.e, mn = pn - o.n;
    const float b  = me * de + mn * dn;
    const float cc = me * me + mn * mn - o.r * o.r;
    const float disc = b * b - cc;
    if (disc < 0.f) return maxRange;              // ray misses the disc
    const float sq = std::sqrt(disc);
    const float t1 = -b - sq, t2 = -b + sq;
    if (t2 < 0.f) return maxRange;                // disc entirely behind
    if (t1 < 0.f) return 0.f;                     // origin inside the disc
    return std::min(t1, maxRange);
}

// Synthetic VFH+-like perception: cast rays across the camera FoV around the
// vehicle heading, pick the most-open direction, publish it the way the real
// NavigateModule does (normalised lateral offset + openness).
void synthPerception(WorldState& s, const std::vector<Obstacle>& obs,
                     float hFovDeg, float maxRange, float rBot, float goalRelDeg,
                     float& prevRel) {
    const int   N = 41;                           // rays across the FoV
    const float half = hFovDeg * 0.5f;
    // Inflate each obstacle by the vehicle footprint + margin, exactly like
    // VFH+'s enlargement: plan for a point robot against fattened obstacles so
    // the committed path keeps a real clearance.
    std::vector<Obstacle> infl;
    infl.reserve(obs.size());
    for (const auto& o : obs) infl.push_back({o.e, o.n, o.r + rBot});

    // Goal-aware local selection (VFH*): among directions with enough clearance,
    // prefer the one closest to the goal bearing. This is what a goal-biased
    // local planner produces; a purely reactive, goal-agnostic "openest ray"
    // spirals around an obstacle sitting directly between start and goal.
    const float goalClamped = clampf(goalRelDeg, -half, half);
    float bestRel = 0.f, bestScore = -1e9f;
    for (int i = 0; i < N; ++i) {
        const float rel = -half + (2.f * half) * i / (N - 1);
        const float bearing = s.vehYawDeg + rel;
        const float b  = bearing * kPi / 180.f;
        const float de = std::sin(b), dn = std::cos(b);
        float clear = maxRange;
        for (const auto& o : infl) clear = std::min(clear, rayClearance(s.estPe, s.estPn, de, dn, o, maxRange));
        const float openf = clear / maxRange;                 // [0,1]
        // Only sufficiently-open directions are candidates; among them, minimise
        // angular distance to the goal, with a small bias toward the previous
        // choice (VFH_W_PREV) to commit to ONE side of a symmetric obstacle
        // instead of chattering across its centre. Blocked directions score by
        // openness so there is always a "best" (steers the scan when boxed).
        const float goalPenalty = std::fabs(rel - goalClamped) / 180.f;
        const float prevPenalty = std::fabs(rel - prevRel) / 180.f;
        const float score = (openf >= 0.5f) ? (2.f - goalPenalty - 0.5f * prevPenalty)
                                            : openf;
        if (score > bestScore) { bestScore = score; bestRel = rel; }
    }
    prevRel = bestRel;
    // Openness is the clearance of the FORWARD travel corridor, not the openest
    // tangent ray: the drone cruises roughly along its heading, so what gates
    // "can I move / must I stop for standoff" is the room straight ahead (a small
    // vehicle-width cone), while `offset` above says which way to steer. Using a
    // grazing tangent's openness lets the body creep into an obstacle it's only
    // tangent to; forward-cone clearance keeps a real standoff.
    float clearFwd = maxRange;
    for (float relC = -6.f; relC <= 6.f; relC += 3.f) {   // ±6° forward cone
        const float bc = (s.vehYawDeg + relC) * kPi / 180.f;
        for (const auto& o : infl)
            clearFwd = std::min(clearFwd, rayClearance(s.estPe, s.estPn, std::sin(bc), std::cos(bc), o, maxRange));
    }

    s.corridorValid  = true;
    s.corridorOffset = clampf(bestRel / half, -1.f, 1.f);
    s.corridorOpen   = clampf(clearFwd / maxRange, 0.f, 1.f);
    s.corridorMargin = s.corridorOpen;
}
}  // namespace

struct Scenario {
    const char*           name;
    std::vector<Obstacle> obs;
    float                 goalE, goalN;
    bool                  expectReach;   // do we require goal completion to pass?
};

struct Result { bool reached; float minObsEdge; float finalGoalDist; float simTime; };

// Run one scenario headless through the REAL loop; returns metrics.
Result runScenario(const Scenario& sc, bool verbose) {
    const float hFovDeg = 60.f, maxRange = 8.f, rBot = 1.5f, dt = 0.02f;

    SimFcBackend fc; StateEstimator est; MissionController mission;
    mission.enable(true); fc.connect("sim", 0);

    WorldState s; s.missionGo = true;
    float prevRel = 0.f, t = 0.f, minObsEdge = 1e9f, dg = std::hypot(sc.goalE, sc.goalN);
    bool  reached = false;

    for (int step = 0; step < 8000 && !reached; ++step) {
        fc.advance(dt);
        FcTelemetry tel; fc.poll(tel);

        est.predict(dt);
        est.setHeading(tel.yawDeg);
        est.updateGps(tel.lat, tel.lon, tel.altM, tel.fixType,
                      true, tel.groundspeedMs * std::cos(tel.groundCourseDeg * kPi / 180.f),
                      tel.groundspeedMs * std::sin(tel.groundCourseDeg * kPi / 180.f), 0.f,
                      1.0f, 1.5f);
        auto e = est.state();

        s.vehYawDeg = tel.yawDeg; s.vehArmed = tel.armed; s.estValid = e.valid;
        s.estPe = e.pe; s.estPn = e.pn; s.estPu = e.pu;
        s.estVe = e.ve; s.estVn = e.vn; s.estSpeed = e.speedMs;

        if (e.valid)
            s.missionGoalBearing = std::atan2(sc.goalE - e.pe, sc.goalN - e.pn) * 180.f / kPi;
        s.missionGo = true;

        const float goalRel = wrap180(s.missionGoalBearing - s.vehYawDeg);
        synthPerception(s, sc.obs, hFovDeg, maxRange, rBot, goalRel, prevRel);

        fc.sendControl(mission.update(s, dt));

        if (e.valid) {
            for (const auto& o : sc.obs)
                minObsEdge = std::min(minObsEdge, std::hypot(e.pe - o.e, e.pn - o.n) - o.r);
            dg = std::hypot(sc.goalE - e.pe, sc.goalN - e.pn);
            if (dg <= 2.0f) reached = true;
            if (verbose && step % 50 == 0)
                std::printf("   t=%5.1f pos=(%6.2f,%6.2f) yaw=%5.1f phase=%-6s dGoal=%5.2f dObs=%5.2f\n",
                            t, e.pe, e.pn, tel.yawDeg, s.missionPhase.c_str(), dg,
                            sc.obs.empty() ? 99.f : minObsEdge);
        }
        t += dt;
    }
    return { reached, sc.obs.empty() ? 99.f : minObsEdge, dg, t };
}

int main() {
    // Suite: separate the loop/goal-seeking validation from the reactive-avoidance
    // limitation. All scenarios: goal 25 m East, start facing East, press-GO.
    std::vector<Scenario> suite = {
        { "clear field (no obstacle)",        {},                    25.f, 0.f, true  },
        { "obstacle well off the path",       {{12.f, 8.f, 2.0f}},   25.f, 0.f, true  },
        { "obstacle grazing the path",        {{12.f, 4.2f, 2.0f}},  25.f, 0.f, true  },
        { "obstacle partly blocking path",    {{12.f, 3.2f, 2.0f}},  25.f, 0.f, false },
        { "obstacle dead-centre on path",     {{12.f, 0.f, 3.0f}},   25.f, 0.f, false },
    };

    std::printf("=== AUTONOMY SITL VALIDATION (pick-a-direction, press-GO) ===\n");
    std::printf("Real components: SimFcBackend -> StateEstimator(ENU EKF) -> "
                "goal-aware VFH* perception -> MissionController -> FC.\n\n");

    int loopOk = 0, reachOk = 0, safeOk = 0, reachExpected = 0;
    for (const auto& sc : suite) {
        Result r = runScenario(sc, true);
        const bool safe = r.minObsEdge > 0.5f;              // never breached standoff
        std::printf("[%-32s] reached=%-3s  minObstacleEdge=%5.2fm  finalGoalDist=%5.2fm  t=%4.0fs  %s\n",
                    sc.name, r.reached ? "YES" : "no", r.minObsEdge, r.finalGoalDist, r.simTime,
                    safe ? "SAFE" : "*** UNSAFE ***");
        std::printf("\n");
        loopOk++;                                            // loop ran end-to-end
        if (safe) safeOk++;
        if (sc.expectReach) { reachExpected++; if (r.reached) reachOk++; }
    }

    std::printf("=== SUMMARY ===\n");
    std::printf("Loop closed (all real components) : %d/%zu scenarios\n", loopOk, suite.size());
    std::printf("Safety standoff never breached    : %d/%zu scenarios\n", safeOk, suite.size());
    std::printf("Goal reached where expected       : %d/%d scenarios\n", reachOk, reachExpected);
    std::printf("\nInterpretation: the SITL loop, arming/GO gating, move-stop-sense cycle,\n"
                "goal-seeking, and obstacle STANDOFF are validated. Goal COMPLETION around an\n"
                "obstacle sitting on the path is NOT guaranteed by the reactive layer (local\n"
                "minima) — that needs the roadmap's deferred occupancy-grid + global planner.\n");

    const bool pass = (safeOk == (int)suite.size()) && (reachOk == reachExpected);
    std::printf("\nVERDICT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
