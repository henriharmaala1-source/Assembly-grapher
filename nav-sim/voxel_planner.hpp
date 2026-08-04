#pragma once

#include <vector>

#include "voxel_map.hpp"

// ---------------------------------------------------------------------------
// Two planners over the voxel map, for two genuinely different jobs.
//
// GeneralPlanner  — "go roughly that way, don't hit anything."
//     A 3D openness histogram over azimuth x elevation, scored against a goal
//     direction with hysteresis toward the previous choice. Reactive, runs every
//     frame, needs no path and no goal position — only a direction. This is the
//     layer that keeps the aircraft alive, and it is deliberately the one that
//     governs SPEED, because it is the one reading the live sensor.
//
// PrecisePlanner  — "fly to that point."
//     3D A* over a coarsened copy of the map, with obstacle inflation and an
//     explicit cost for traversing UNKNOWN. Runs occasionally, produces a
//     waypoint list. Its output is a SUGGESTION, handed to the general planner
//     as a direction to prefer.
//
// THE ARBITRATION RULE, which is the whole safety argument:
//     The precise planner never commands the vehicle. It only changes which
//     direction the general planner is biased toward. So a stale, wrong or
//     empty path can slow the aircraft down or send it the long way round, but
//     it can never drive it into something the live map can see. This mirrors
//     the rule the 2D stack in this directory already follows, and it is the
//     reason a map built from noisy stereo is safe to use at all.
//
// UNKNOWN IS NOT FREE, and it is not blocked either. Treating it as free flies
// you into untextured walls; treating it as blocked means you can never enter
// space you have not already seen, which on a forward-facing camera means you
// can never move. Both planners therefore price it: traversable, expensive.
// ---------------------------------------------------------------------------

namespace sim {

// --- general (reactive direction) planner -----------------------------------

struct GeneralParams {
    int   nAz         = 48;      // azimuth bins over 360 deg
    int   nEl         = 9;       // elevation bins
    float elMinDeg    = -35.f;
    float elMaxDeg    =  35.f;
    float horizonM    = 12.f;    // how far to probe
    float robotR      = 0.6f;    // clearance the aircraft needs
    float unknownCost = 0.45f;   // an unknown cell counts as this fraction of a
                                 // real obstacle when scoring openness
    // Direction must be scored on BOTH openness and confirmed-free room. See
    // the comment in plan(): scoring on openness alone picks the most promising
    // direction and then discovers it has nowhere to move into.
    float freeWeight  = 1.4f;
    float goalWeight  = 1.0f;    // pull toward the requested direction
    float hystWeight  = 0.55f;   // stickiness toward last frame's choice; this
                                 // is what stops oscillation between two
                                 // equally-open gaps, and it matters more than
                                 // it looks like it should
    float vMax        = 6.0f;    // m/s
    float decelMs2    = 3.0f;    // usable deceleration
    float reactS      = 0.25f;   // sense+plan+actuate latency before decel starts
    float minFreeM    = 0.4f;    // below this confirmed-free range, hold
};

struct GeneralResult {
    float azDeg = 0, elDeg = 0;  // chosen direction
    float speed = 0;             // commanded speed, m/s
    // TWO different ranges, and conflating them is a collision.
    //   openM — unknown-discounted openness. Used to CHOOSE a direction, because
    //           refusing to steer toward unseen space means never moving.
    //   freeM — contiguous CONFIRMED-FREE distance. Used to set SPEED, because
    //           the aircraft may only travel as fast as it can stop within what
    //           it has actually seen. Unknown space earns no speed at all.
    float openM = 0;
    float freeM = 0;
    bool  blocked = false;       // nothing acceptable anywhere -> hold
};

class GeneralPlanner {
public:
    explicit GeneralPlanner(const GeneralParams& p = GeneralParams()) : p_(p) {}
    const GeneralParams& params() const { return p_; }

    // goalAz/goalEl: the direction you would like to go if the world were empty.
    GeneralResult plan(const VoxelMap& m, float px, float py, float pz,
                       float goalAzDeg, float goalElDeg);

    // Last openness field, for drawing. Row-major [el][az], metres.
    const std::vector<float>& field() const { return field_; }
    const std::vector<float>& freeField() const { return free_; }
    int nAz() const { return p_.nAz; }
    int nEl() const { return p_.nEl; }

private:
    GeneralParams p_;
    std::vector<float> field_, free_;
    float lastAz_ = 0, lastEl_ = 0;
    bool  haveLast_ = false;
};

// --- precise (A* to a point) planner ----------------------------------------

struct PreciseParams {
    int   coarsen      = 2;     // plan on every Nth voxel; 0.25 m -> 0.5 m
    float robotR       = 0.6f;  // physical clearance
    // Plan with a WIDER margin than the reactive layer uses. If they are equal,
    // the planned path skims obstacle faces, the reactive layer then fights it
    // every frame, and the aircraft crawls. The 2D stack in this directory
    // learned this the same way.
    float planMarginM  = 0.9f;
    float unknownCost  = 2.5f;  // multiplier on step cost through unknown space
    int   maxExpand    = 250000;
    float goalTolM     = 1.5f;
};

struct PrecisePath {
    std::vector<std::array<float, 3>> pts;   // world ENU waypoints
    bool  found = false;
    int   expanded = 0;
    float costM = 0;
    // Set when the start cell itself was blocked and had to be nudged. Worth
    // surfacing rather than hiding: it usually means the map thinks you are
    // inside an obstacle, which is either drift or a bad inflation radius.
    bool  startWasBlocked = false;
};

class PrecisePlanner {
public:
    explicit PrecisePlanner(const PreciseParams& p = PreciseParams()) : p_(p) {}
    const PreciseParams& params() const { return p_; }

    PrecisePath plan(const VoxelMap& m,
                     float sx, float sy, float sz,
                     float gx, float gy, float gz);

private:
    PreciseParams p_;
};

}  // namespace sim
