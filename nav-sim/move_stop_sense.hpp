#pragma once

namespace navsim {

// Move-stop-sense navigation — a faithful, self-contained port of the drone's
// real onboard MissionController (onboard/src/mission.cpp). It is NOT a path
// planner: it's a phased reactive CONTROLLER that stops to think, commits a leg,
// flies it re-steering live, and rotates in place to look around when boxed in
// (SETTLE -> THINK -> SCAN -> MOVE -> ARRIVE). The occupancy-grid route (planned
// externally and passed in as planBearing, mirroring P5b's LocalMap) supplies
// the global goal direction; the live corridor still governs speed and steering.
//
// Ported here (copied, not linked) so nav-sim stays standalone. The phase
// machine, thresholds, and desired-bearing blend match the onboard source; the
// bits that only exist on the real aircraft (estimator health, corridor
// staleness, the GO latch) are dropped — the sim's estimate is exact and its
// scan is always fresh.

struct MssInput {
    float e = 0, n = 0, yawDeg = 0, speedMs = 0;   // vehicle state
    float corridorOpen = 1.f;                      // [0,1] forward clearance
    float corridorOffset = 0.f;                    // [-1,1] openest direction
    bool  planValid = false; float planBearing = 0.f;  // grid route (deg, 0=N)
    float goalBearing = 0.f;                        // to the goal (deg, 0=N)
};

struct MssOutput {
    float bearingDeg = 0.f;   // heading to steer toward
    float speedScale = 0.f;   // [0,1] translation (0 = hover / scan-in-place)
    bool  yawScan = false;    // rotating in place to look around
    const char* phase = "SETTLE";
    float wpE = 0.f, wpN = 0.f;   // committed leg target (for display)
};

class MoveStopSense {
public:
    struct Params {
        float settleSec     = 1.5f;   // min hover/think time before a leg
        float settleSpeedMs = 0.4f;   // "stopped" below this
        float stepM         = 4.0f;   // leg length before re-thinking
        float moveTimeoutSec= 12.f;
        float cruise        = 1.0f;   // forward speed scale while moving (sim units)
        float kpYaw         = 1.2f;
        float hFovDeg       = 90.f;   // sensor FoV (goal clamp / offset scale)
        float minOpenToMove = 0.12f;
        float minOpenToKeep = 0.06f;
        float scanYawRate   = 1.0f;   // (sim: just a turn intent)
        float scanTimeoutSec= 9.f;
        bool  useMap        = true;   // use the grid route as the goal term
    };

    MoveStopSense() = default;
    explicit MoveStopSense(Params p) : p_(p) {}

    void reset();
    MssOutput update(const MssInput& in, float dt);
    const char* phaseName() const;

private:
    enum class Phase { SETTLE, THINK, SCAN, MOVE, ARRIVE };
    float desiredBearing_(const MssInput& in) const;   // goal+corridor blend (ported)

    Params p_;
    Phase  phase_ = Phase::SETTLE;
    float  tPhase_ = 0.f;
    float  legE_ = 0, legN_ = 0, wpE_ = 0, wpN_ = 0;
    float  roundSign_ = 0.f;
    bool   haveWp_ = false;
};

}  // namespace navsim
