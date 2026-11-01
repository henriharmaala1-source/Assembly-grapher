#pragma once

#include <array>
#include <vector>

#include "voxel_map.hpp"

// ---------------------------------------------------------------------------
// LOCAL FORWARD planner, backed by OMPL (RRT-Connect / RRT*).
//
// This replaces the hand-rolled A*, and it also changes the QUESTION being
// asked, which is the more important half.
//
// The A* planned to a distant mission goal. That was the wrong problem: the
// local map is only ~60 m across, mission goals are hundreds of metres away, so
// every replan projected the goal onto the map boundary anyway -- after
// re-coarsening and re-inflating all 691k cells to get there. Measured 236-283
// ms per replan, which on a Pi 5 is a 0.35-0.7 s stall, i.e. 3-7 lost control
// cycles.
//
// What is actually needed, and what this does: plan a short path AHEAD, in a
// requested direction, ending on a horizon sphere a few tens of metres out. The
// mission direction comes from outside (a bearing, a waypoint, an operator); the
// planner's only job is to find a safe way to keep going that way. Same
// arbitration rule as before -- the path is a SUGGESTION handed to the reactive
// layer, never a command.
//
// Why this is cheap: sampling planners never build a grid. OMPL asks "is this
// state valid?" and "is this motion valid?", both answered directly against the
// VoxelMap by a sphere test. Nothing is coarsened, nothing is inflated, and cost
// scales with path difficulty rather than with map volume.
//
// WHY OMPL RATHER THAN MORE HAND-ROLLING. Two of the five bugs found in this
// harness were in code I wrote to measure things, and the planner itself needed
// three fixes before it flew 100 m. A BSD-licensed, widely-used library removes
// "is my planner broken?" from the list of explanations when a run fails, which
// is worth more than any speed difference. It is also ROS-free and builds on
// Windows via vcpkg, so it costs nothing architecturally.
//
// SLAM NOTE: nothing here assumes a globally consistent map. The planner reads
// whatever VoxelMap holds right now, in whatever frame it is in. If a SLAM
// backend later replaces dead-reckoned pose with a corrected one, this code does
// not change -- it only ever plans a few tens of metres ahead, which is the
// regime where drift has not yet accumulated.
// ---------------------------------------------------------------------------

namespace sim {

struct ForwardParams {
    float horizonM   = 25.f;   // how far ahead to plan
    float robotR     = 0.6f;   // clearance required, metres
    float unknownOk  = 1;      // may the path cross UNKNOWN space? (it must, or
                               // a forward-facing camera can never advance)
    float planTimeS  = 0.05f;  // OMPL solve budget -- a HARD cap, which is the
                               // property the A* lacked
    float coneDeg    = 75.f;   // how far off the requested bearing the horizon
                               // target may sit
    int   nCandidates = 9;     // horizon targets tried, best-first by bearing
    float simplifyS  = 0.01f;  // path shortcutting budget
};

struct ForwardPath {
    std::vector<std::array<float, 3>> pts;
    bool  found = false;
    float lengthM = 0;
    float solveMs = 0;
    float bearingDeg = 0;      // bearing of the accepted horizon target
};

// --- steering the reactive layer at this path -------------------------------
//
// These three exist because of one measurement. The reactive planner was
// accused of "spinning aggressively"; its commanded bearing churns 20.9 deg per
// step. But the GOAL bearing handed to it churns 21.5 deg per step. The
// reactive layer is not spinning -- it is faithfully tracking a reference that
// is, and no amount of bias, hysteresis or commitment applied downstream can
// fix a wobbling target. All three fixes below are therefore upstream.

// PURE PURSUIT. The old rule was "aim at the first path waypoint more than 3 m
// away", which is the classic carrot-chasing artefact: the path is interpolated
// to roughly every 2 m, so the chosen waypoint switches every few steps and each
// switch is a discrete bearing jump. At 3 m lookahead, 1 m of lateral offset
// between consecutive waypoints is an 18 degree step -- for nothing.
//
// Instead take the point at a fixed ARCLENGTH along the path, measured from the
// point nearest the vehicle, interpolating between waypoints. The target then
// slides continuously as the vehicle moves instead of hopping.
bool pursuitPoint(const ForwardPath& p, float px, float py, float pz,
                  float lookaheadM, float& tx, float& ty, float& tz);

// PATH REUSE. RRTConnect is randomised: replanning from scratch every 25 steps
// produced an arbitrarily different path each time even when nothing about the
// world had changed -- two runs of an identical situation gave goal bearings
// 23 degrees apart. So do not replan on a timer. Replan when the path you have
// has actually stopped being usable: some state on it is no longer safe, the
// vehicle has drifted off it, too little of it remains ahead, or it no longer
// points near where you want to go.
//
// This is plan reuse rather than warm-starting RRTConnect's internals, and it
// is both simpler and stronger: a path that is still good produces EXACTLY the
// same reference as last step, not merely a similar one.
bool pathStillGood(const VoxelMap& m, const ForwardPath& p,
                   float px, float py, float pz,
                   float wantAzDeg, const ForwardParams& fp,
                   const VoxelMap* far = nullptr);

// BEARING FILTER. Even with pure pursuit and path reuse a replan is a step
// change in the reference. Low-passing it costs a little responsiveness and no
// safety: this smooths a REFERENCE, not a measurement. The reactive layer still
// gates speed on the live map, so a lagging reference can send the aircraft the
// long way round but cannot fly it into anything.
//
// Worth being precise about, because the same idea was tried on the direction
// FIELD earlier and rightly abandoned: smoothing what you brake on is
// dangerous, smoothing what you aim at is not.
struct BearingFilter {
    float azDeg = 0, elDeg = 0;
    bool  have = false;
    // alpha 1.0 = no filtering. 0.25 is about a 0.35 s time constant at 10 Hz,
    // which is the same order as the vehicle's own turn lag -- filtering much
    // harder than the airframe responds buys nothing.
    void update(float az, float el, float alpha);
};

// --- when to use the global planner at all -----------------------------------
//
// Measured, forest, 300 steps, 2 seeds:
//
//     arm            goalChurn  cmdChurn  advance   endDist
//     with OMPL           2.35      1.95    0.908      95.5
//     bearing only        0.01      0.68    0.979      87.9
//
// Following a routed path is WORSE than just pointing at the goal, on every
// column, in a forest. A stand of scattered trunks has nothing large enough to
// route around, so the router contributes no information and does contribute
// noise: a randomised path, re-rolled periodically, sampled by a moving target.
// Given a steady bearing the reactive layer churns 0.68 deg/step -- it never had
// a spinning problem at all.
//
// But that is an argument about FORESTS, not about routers. A reactive planner
// with a 12 m horizon cannot see out of a dead end, and a courtyard or a long
// facade is exactly that. So do not choose once: run reactive, notice when it
// stops making progress, and call the router only then.
//
// Stall is defined as "no new closest approach to the goal for stallSteps".
// Closest approach rather than distance-now, because a vehicle circling a
// building has a distance that oscillates without ever improving, and that is
// precisely the situation this is meant to catch. Hand back only after the
// router has bought recoverM of real progress, so the two layers cannot flap.
struct StallMonitor {
    int   stallSteps = 40;      // 4 s at 10 Hz with no improvement
    float recoverM   = 5.f;     // progress before the router hands back
    float epsM       = 0.25f;   // improvement that counts as improvement

    float bestDist = 1e30f;     // closest approach achieved so far
    float engagedAt = 0;
    int   sinceBest = 0;
    bool  engaged = false;
    long  engagedSteps = 0;     // for reporting: how much of the flight needed it
    int   engagements = 0;

    void update(float distToGoal) {
        if (distToGoal < bestDist - epsM) { bestDist = distToGoal; sinceBest = 0; }
        else ++sinceBest;
        if (!engaged) {
            if (sinceBest > stallSteps) { engaged = true; engagedAt = bestDist; ++engagements; }
        } else if (bestDist < engagedAt - recoverM) {
            engaged = false;
        }
        if (engaged) ++engagedSteps;
    }
};

// planner backend, selected at build time
const char* forwardPlannerName();

// Seed the sampling planner's RNG. RRTConnect is randomised; without this two
// runs of the same binary with the same world seed diverge, which makes any
// before/after comparison unfalsifiable. Call once, before the first
// planForward. No-op in the non-OMPL fallback build, which is deterministic
// already.
void setPlannerSeed(unsigned seed);

// Plan a path from (sx,sy,sz) heading roughly along goalAzDeg / goalElDeg,
// ending near the horizon. Returns found=false if no candidate was reachable.
// `far` is an optional coarser companion map. Where the fine map has no
// opinion, the router consults it -- which is the point of having one: the
// reactive rollout is ~6 m and never leaves the fine map, but this plans to
// 25 m, well past the range at which 0.25 m voxels can honestly mark anything.
// The fine map always wins where it HAS an opinion; the coarse map only fills
// in unknowns, and can therefore only ever make the router more conservative.
ForwardPath planForward(const VoxelMap& m,
                        float sx, float sy, float sz,
                        float goalAzDeg, float goalElDeg,
                        const ForwardParams& p = ForwardParams(),
                        const VoxelMap* far = nullptr);

}  // namespace sim
