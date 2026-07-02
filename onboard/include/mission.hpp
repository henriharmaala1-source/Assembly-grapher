#pragma once

#include "control_types.hpp"
#include "world_model.hpp"

// Move-stop-sense autonomous cycle — the paradigm for a compute-limited drone:
// THINK WHILE STILL, then move a committed leg, then stop and think again.
//
//   SETTLE : command a HOVER and hold it until ground speed has died down and a
//            minimum think-time has passed. THE AIRCRAFT MUST ACTUALLY HOVER
//            here (see note) — this is the stable vantage the deliberative tier
//            (SLAM / map / plan) needs.
//   THINK  : read the plan the think tier produced (for now: the open-corridor
//            direction) and COMMIT a waypoint one step ahead, in the local ENU
//            frame. If nothing is open, drop back to SETTLE and keep thinking.
//   MOVE   : fly to the committed waypoint using the estimator's position +
//            heading; slow/stop and re-plan if the corridor ahead closes.
//   ARRIVE : within the arrival radius (or the leg timed out) -> back to SETTLE.
//
// NOTE ON HOVER: the ControlCmd for SETTLE is all-zero, which on the real FC
// means "hold current sticks". For it to truly hold POSITION (not just level
// attitude and drift), the FC must be in an altitude+position-hold-capable mode
// (iNAV NAV POSHOLD / ArduPilot LOITER). With that, zero override = stationary
// hover, which is exactly what THINK needs. The controller here does not fight
// the FC's hold; it just refrains from commanding motion during SETTLE/THINK.
class MissionController {
public:
    // SCAN — cornered: nothing ahead is open enough to move into, so rotate in
    // place (yaw only, no translation) to bring open space into the camera FoV,
    // then re-plan. A forward-facing narrow-FoV camera cannot see a lateral
    // escape around an obstacle wider than its FoV without first turning to look.
    enum class Phase { SETTLE, THINK, SCAN, MOVE, ARRIVE };

    struct Params {
        float settleSec      = 1.5f;   // min hover time before committing a leg
        float settleSpeedMs  = 0.4f;   // "stopped" below this ground speed
        float stepM          = 4.0f;   // distance flown per leg before re-SLAM
        float moveTimeoutSec = 12.f;   // abort a leg that stalls
        float cruise         = 0.25f;  // forward pitch while moving
        float kpYaw          = 1.2f;   // heading error -> yaw
        float hFovDeg        = 60.f;   // camera h-FoV: corridor offset -> bearing
        float corridorStaleSec = 0.8f; // corridor older than this = blind (stop)
        float maxEphM        = 3.0f;   // est 1σ above this = degraded (hover) —
                                       // estValid stays true while coasting
                                       // GPS-denied, so uncertainty is the gate
        float minOpenToMove  = 0.12f;  // openness needed to START a leg
        float minOpenToKeep  = 0.06f;  // openness needed to CONTINUE a leg
                                       // (hysteresis: start > keep, no chatter).
                                       // Low because obstacles are already
                                       // inflated by the vehicle+margin radius,
                                       // so skimming the inflated edge is safe.
        float scanYawRate    = 0.4f;   // yaw stick while scanning ([-1,1])
        float scanTimeoutSec = 9.f;    // give up scanning after ~a full rotation
    };

    MissionController() = default;
    explicit MissionController(Params p) : p_(p) {}

    void  enable(bool on);
    bool  enabled() const { return enabled_; }
    Phase phase()   const { return phase_; }
    static const char* phaseName(Phase p);

    // One fly-loop tick. Reads the world state, advances the cycle, and returns
    // the control command for this tick (hover during SETTLE/THINK, move toward
    // the waypoint during MOVE). Also fills the mission fields in `s` for
    // telemetry/display via the caller.
    ControlCmd update(WorldState& s, float dt);

private:
    // Absolute heading (deg, 0=N) to steer this tick: the operator's goal bearing
    // blended with the vetted-open corridor. Used both to commit a leg target and
    // to RE-STEER live while moving (reactive avoidance, not a blind waypoint run).
    float desiredBearing_(const WorldState& s) const;
    void  commitWaypoint_(const WorldState& s);

    Params p_;
    bool   enabled_ = false;
    Phase  phase_   = Phase::SETTLE;
    float  tPhase_  = 0.f;
    float  wpE_ = 0.f, wpN_ = 0.f;      // committed leg target (ENU, for display)
    float  legE_ = 0.f, legN_ = 0.f;    // leg start position (ENU) — leg length gate
    float  roundSign_ = 0.f;            // which way we're rounding an obstacle
                                        // (-1 left / +1 right / 0 none) — steers
                                        // the scan out of a concave dead-end
    bool   haveWp_  = false;
};
