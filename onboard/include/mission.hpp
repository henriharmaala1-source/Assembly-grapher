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
    enum class Phase { SETTLE, THINK, MOVE, ARRIVE };

    struct Params {
        float settleSec      = 1.5f;   // min hover time before committing a leg
        float settleSpeedMs  = 0.4f;   // "stopped" below this ground speed
        float stepM          = 4.0f;   // waypoint distance committed per leg
        float arriveRadiusM  = 0.8f;   // reached-waypoint threshold
        float moveTimeoutSec = 12.f;   // abort a leg that stalls
        float cruise         = 0.25f;  // forward pitch while moving
        float kpYaw          = 1.2f;   // heading error -> yaw
        float hFovDeg        = 60.f;   // camera h-FoV: corridor offset -> bearing
        float minOpenToMove  = 0.35f;  // corridor openness below this -> re-think
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
    void commitWaypoint_(const WorldState& s);

    Params p_;
    bool   enabled_ = false;
    Phase  phase_   = Phase::SETTLE;
    float  tPhase_  = 0.f;
    float  wpE_ = 0.f, wpN_ = 0.f;
    bool   haveWp_  = false;
};
