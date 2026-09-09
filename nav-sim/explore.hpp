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

    void reset(Vec2 home) { home_ = home; phase_ = Phase::EXPLORE; haveTgt_ = false;
                             maxDist_ = 0.f; stuckAnchor_ = home; stuckTicks_ = 0; }

    // resetDriver: true on the tick Explore gives up on reaching home under the
    // driver's CURRENT state and switches to RETURN (the natural "area fully
    // mapped" case, or the stuck-frontier watchdog above). Heading home is a
    // genuinely new objective; if the driver latched a terminal state (e.g.
    // MoveStopSense's own STUCK) while chasing the abandoned frontier, that
    // state has no bearing on whether home is reachable and must not silently
    // carry over — a new goal alone can't undo a STUCK latch by design (it's
    // meant to hold until the WORLD changes, not until asked nicely), so the
    // caller must explicitly reset its driver when this is set.
    struct Out { Vec2 goal; const char* phase; bool done; bool resetDriver = false; };

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
    // Progress watchdog: the frontier picked is only ever REACHABLE-in-theory
    // (known-free space in the grid) — the actual driver flying to it can still
    // get genuinely wedged short of it (the same funnel/local-minimum class of
    // problem MoveStopSense's own STUCK phase exists for), and unlike that
    // driver, Explore has no visibility into the driver's state to notice.
    // So it watches its OWN net displacement: no real progress for too long ->
    // this frontier is unreachable in practice, give up on it and try to get
    // home instead of re-issuing the same stuck target forever.
    Vec2 stuckAnchor_{};
    int  stuckTicks_ = 0;
};

const char* explorePhaseName(Explore::Phase p);

}  // namespace navsim
