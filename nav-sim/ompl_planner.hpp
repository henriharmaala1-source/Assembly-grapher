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

// planner backend, selected at build time
const char* forwardPlannerName();

// Plan a path from (sx,sy,sz) heading roughly along goalAzDeg / goalElDeg,
// ending near the horizon. Returns found=false if no candidate was reachable.
ForwardPath planForward(const VoxelMap& m,
                        float sx, float sy, float sz,
                        float goalAzDeg, float goalElDeg,
                        const ForwardParams& p = ForwardParams());

}  // namespace sim
