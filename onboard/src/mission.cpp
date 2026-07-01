#include "mission.hpp"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrap180(float d) {                        // -> (-180, 180]
    while (d > 180.f)  d -= 360.f;
    while (d <= -180.f) d += 360.f;
    return d;
}
}  // namespace

const char* MissionController::phaseName(Phase p) {
    switch (p) {
        case Phase::SETTLE: return "SETTLE";
        case Phase::THINK:  return "THINK";
        case Phase::MOVE:   return "MOVE";
        case Phase::ARRIVE: return "ARRIVE";
    }
    return "?";
}

void MissionController::enable(bool on) {
    enabled_ = on;
    phase_   = Phase::SETTLE;    // always (re)start stopped — think before moving
    tPhase_  = 0.f;
    haveWp_  = false;
}

// Commit a waypoint one step ahead. Head toward the OPERATOR'S GOAL bearing, but
// deflect toward the open corridor when the direct path is blocked, so it goes
// AROUND obstacles while trending toward the goal. The goal is clamped to the
// sensor FoV (can only verify "open" within it) so the drone turns toward a
// side/behind goal gradually, sensing as it goes.
void MissionController::commitWaypoint_(const WorldState& s) {
    const float goalRel     = wrap180(s.missionGoalBearing - s.vehYawDeg);
    const float goalClamped = clampf(goalRel, -p_.hFovDeg * 0.5f, p_.hFovDeg * 0.5f);
    const float corridorRel = s.corridorOffset * (p_.hFovDeg * 0.5f);
    const float blocked     = clampf(1.f - s.corridorOpen, 0.f, 1.f);
    const float relBearing  = (1.f - blocked) * goalClamped + blocked * corridorRel;
    const float bearing     = s.vehYawDeg + relBearing;                   // deg, 0=N
    const float b           = bearing * kPi / 180.f;
    wpE_    = s.estPe + p_.stepM * std::sin(b);   // ENU east
    wpN_    = s.estPn + p_.stepM * std::cos(b);   // ENU north
    haveWp_ = true;
}

ControlCmd MissionController::update(WorldState& s, float dt) {
    ControlCmd c;
    c.valid = true;                 // we always command something (hover or move)
    tPhase_ += dt;
    s.missionActive = true;

    // Without a position estimate we cannot do waypoint nav — hold a safe hover.
    if (!s.estValid) {
        phase_ = Phase::SETTLE; tPhase_ = 0.f;
        s.missionPhase = "SETTLE(no-est)";
        return c;                   // all-zero = hover
    }

    // Armed but waiting for the operator to press GO — hover in place. Toggling
    // GO off mid-leg drops straight back to a hover here too.
    if (!s.missionGo) {
        phase_ = Phase::SETTLE; tPhase_ = 0.f;
        s.missionPhase = "ARMED";
        return c;                   // hover until GO
    }

    switch (phase_) {
        case Phase::SETTLE: {
            // Hover; leave once settled AND a minimum think window has elapsed.
            const bool settled = s.estSpeed < p_.settleSpeedMs;
            if (tPhase_ >= p_.settleSec && settled) {
                phase_ = Phase::THINK; tPhase_ = 0.f;
            }
            break;                  // hover (all-zero)
        }
        case Phase::THINK: {
            // Read the (think-tier) plan and commit a leg — or keep hovering if
            // nothing is open enough to move into.
            if (s.corridorValid && s.corridorOpen >= p_.minOpenToMove) {
                commitWaypoint_(s);
                phase_ = Phase::MOVE; tPhase_ = 0.f;
            } else {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;   // re-settle & keep thinking
            }
            break;                  // hover this tick
        }
        case Phase::MOVE: {
            const float vE = wpE_ - s.estPe, vN = wpN_ - s.estPn;
            const float dist = std::hypot(vE, vN);
            const bool  blocked = s.corridorValid && s.corridorOpen < p_.minOpenToMove;

            if (dist <= p_.arriveRadiusM || tPhase_ >= p_.moveTimeoutSec || blocked) {
                phase_ = Phase::ARRIVE; tPhase_ = 0.f;
                break;              // hover this tick
            }
            // Yaw toward the waypoint; cruise forward, scaled by how open it is.
            const float desired = std::atan2(vE, vN) * 180.f / kPi;   // deg, 0=N
            const float err     = wrap180(desired - s.vehYawDeg);
            c.yaw   = clampf(p_.kpYaw * (err / 90.f), -1.f, 1.f);
            const float openScale = s.corridorValid
                ? clampf(s.corridorOpen, 0.3f, 1.f) : 0.6f;
            c.pitch = p_.cruise * openScale;
            break;
        }
        case Phase::ARRIVE: {
            phase_ = Phase::SETTLE; tPhase_ = 0.f;   // stop, then think again
            break;                  // hover
        }
    }

    s.missionPhase = phaseName(phase_);
    s.missionWpE   = wpE_;
    s.missionWpN   = wpN_;
    return c;
}
