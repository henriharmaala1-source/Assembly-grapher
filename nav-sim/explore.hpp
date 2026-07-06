#pragma once

#include "occupancy_grid.hpp"
#include "planners.hpp"

namespace navsim {

// Explore-and-return-home: a coverage behaviour, not point-to-point nav. Drives
// toward the nearest FRONTIER (a known-free cell bordering unknown space) to
// grow the map — the classic Yamauchi frontier-exploration idea — until no
// reachable frontier remains, then plans a route back to home and lands there.
//
// It doesn't move the drone itself; each step it returns the current sub-goal
// (a frontier, or home) + a phase, and the caller drives to it with a planner.
class Explore {
public:
    enum class Phase { EXPLORE, RETURN, DONE };

    void reset(Vec2 home) { home_ = home; phase_ = Phase::EXPLORE; haveTgt_ = false; maxDist_ = 0.f; }

    struct Out { Vec2 goal; const char* phase; bool done; };

    // Pick the sub-goal for this tick given the current (partial) grid + pose.
    Out step(const OccupancyGrid& g, Vec2 pos, int inflate);

    Phase phase() const { return phase_; }

private:
    // BFS over known-free space from `pos`; the first cell bordering unknown is
    // the nearest reachable frontier. Returns false if the reachable area is
    // fully mapped (no frontier left).
    bool nearestFrontier(const OccupancyGrid& g, Vec2 pos, int inflate, Vec2& out) const;

    Vec2  home_{}, tgt_{};
    Phase phase_ = Phase::EXPLORE;
    bool  haveTgt_ = false;
    float maxDist_ = 0.f;   // furthest from home — guards premature "done"
};

const char* explorePhaseName(Explore::Phase p);

}  // namespace navsim
