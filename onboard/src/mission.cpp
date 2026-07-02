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
        case Phase::SCAN:   return "SCAN";
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
    roundSign_ = 0.f;
}

// Commit a waypoint one step ahead. Head toward the OPERATOR'S GOAL bearing, but
// deflect toward the open corridor when the direct path is blocked, so it goes
// AROUND obstacles while trending toward the goal. The goal is clamped to the
// sensor FoV (can only verify "open" within it) so the drone turns toward a
// side/behind goal gradually, sensing as it goes.
float MissionController::desiredBearing_(const WorldState& s) const {
    const float goalRel     = wrap180(s.missionGoalBearing - s.vehYawDeg);
    const float goalClamped = clampf(goalRel, -p_.hFovDeg * 0.5f, p_.hFovDeg * 0.5f);
    const float corridorRel = s.corridorOffset * (p_.hFovDeg * 0.5f);   // vetted-open dir
    const float deflect     = clampf(std::fabs(s.corridorOffset), 0.f, 1.f);

    // The corridor direction is the only heading perception has certified as
    // open. Head to the GOAL only while the way ahead is essentially clear
    // (deflect≈0); as the corridor swings aside to avoid an obstacle, the goal's
    // influence falls off QUADRATICALLY so the heading tracks the vetted-open
    // direction instead of cutting the corner back toward the obstacle.
    float goalW = 1.f - deflect;
    goalW *= goalW;
    float relBearing = goalW * goalClamped + (1.f - goalW) * corridorRel;

    // Hard safety clamp: never steer LESS deflected than the corridor while it is
    // actively avoiding — that region is on the obstacle side and was not
    // certified open. (No effect when the corridor is clear ahead.)
    if (corridorRel < 0.f) relBearing = std::min(relBearing, corridorRel);
    if (corridorRel > 0.f) relBearing = std::max(relBearing, corridorRel);

    return s.vehYawDeg + relBearing;                                  // deg, 0=N
}

void MissionController::commitWaypoint_(const WorldState& s) {
    const float b = desiredBearing_(s) * kPi / 180.f;
    legE_   = s.estPe;                            // leg start — gates leg length
    legN_   = s.estPn;
    wpE_    = s.estPe + p_.stepM * std::sin(b);   // ENU east (display target)
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
            // Read the (think-tier) plan and commit a leg. If the way ahead is
            // open enough, move; if we have a reading but it's blocked, rotate in
            // place to look for a way around; if there's no reading, keep hovering.
            if (s.corridorValid && s.corridorOpen >= p_.minOpenToMove) {
                commitWaypoint_(s);
                phase_ = Phase::MOVE; tPhase_ = 0.f;
            } else if (s.corridorValid) {
                phase_ = Phase::SCAN; tPhase_ = 0.f;     // cornered -> turn to look
            } else {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;   // no data -> re-settle
            }
            break;                  // hover this tick
        }
        case Phase::SCAN: {
            // Rotate in place (no translation) toward the openest side until a
            // corridor opens up ahead, then re-plan. Give up after ~a full turn.
            if (s.corridorValid && s.corridorOpen >= p_.minOpenToMove) {
                phase_ = Phase::THINK; tPhase_ = 0.f;    // opening found -> commit
                break;                                   // hover this tick
            }
            if (tPhase_ >= p_.scanTimeoutSec) {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;   // dead end -> settle & retry
                break;
            }
            // Choose a turn direction. While rounding an obstacle, KEEP turning
            // that same way (wall-follow) so the escape — which can lie outside
            // the forward FoV — sweeps into view; the openest ray inside a boxed
            // FoV is noise and must not override it. With no go-around committed,
            // steer toward whichever side reads more open (default left).
            const float dir = (roundSign_ != 0.f)         ? roundSign_
                            : (s.corridorOffset > 0.02f)  ?  1.f
                            : (s.corridorOffset < -0.02f) ? -1.f : -1.f;
            c.yaw = dir * p_.scanYawRate;
            break;                  // yaw only, no pitch
        }
        case Phase::MOVE: {
            // Leg ends after stepM travelled (then stop & re-SLAM), on timeout, or
            // when the way ahead closes below the keep threshold.
            const float legDist = std::hypot(s.estPe - legE_, s.estPn - legN_);
            const bool  blocked = s.corridorValid && s.corridorOpen < p_.minOpenToKeep;

            if (legDist >= p_.stepM || tPhase_ >= p_.moveTimeoutSec || blocked) {
                phase_ = Phase::ARRIVE; tPhase_ = 0.f;
                break;              // hover this tick
            }
            // RE-STEER LIVE toward the current goal+corridor blend (reactive
            // avoidance) — not a blind run at a frozen waypoint. Cruise forward,
            // scaled by how open it is so it slows as clearance tightens.
            const float desired = desiredBearing_(s);                 // deg, 0=N
            const float err     = wrap180(desired - s.vehYawDeg);
            c.yaw   = clampf(p_.kpYaw * (err / 90.f), -1.f, 1.f);
            const float openScale = s.corridorValid
                ? clampf(s.corridorOpen, 0.3f, 1.f) : 0.6f;
            c.pitch = p_.cruise * openScale;
            // Remember which way we're rounding the obstacle (the corridor's
            // deflection side); forget it once the way ahead is clearly open
            // again (obstacle passed). This steers the scan out of a dead-end.
            if (std::fabs(s.corridorOffset) > 0.3f)
                roundSign_ = s.corridorOffset < 0.f ? -1.f : 1.f;
            else if (s.corridorOpen > 0.6f && std::fabs(s.corridorOffset) < 0.1f)
                roundSign_ = 0.f;
            // Keep the displayed leg target following the live heading.
            const float b = desired * kPi / 180.f;
            wpE_ = s.estPe + p_.stepM * std::sin(b);
            wpN_ = s.estPn + p_.stepM * std::cos(b);
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
