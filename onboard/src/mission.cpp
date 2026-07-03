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
    if (on) {                          // fresh occupancy grid per mission (P5b)
        p_.map.robotR = p_.planBerthM; // route with a wider berth than live safety
        map_ = LocalMap(p_.map);
    }
}

// P5b — integrate the live corridor scan into the occupancy grid and (re)plan a
// global route toward the goal bearing. The grid remembers obstacles the live
// FoV has forgotten, so the planner routes AROUND one sitting on the direct
// path (the reactive layer's local-minimum failure). Only runs on a fresh
// scan; a stale scan would pollute the map with observations from a dead
// perception tier. Sets s.planValid / s.planBearing for the steering + display.
void MissionController::updateMap_(WorldState& s) {
    s.planValid = false;
    if (!p_.useMap || s.corridorScanN < 1) return;
    if (!s.corridorFresh(p_.corridorStaleSec)) return;
    map_.integrate(s.estPe, s.estPn, s.vehYawDeg, s.corridorScan,
                   s.corridorScanN, s.corridorScanFovDeg, s.corridorScanMaxM);
    float b;
    if (map_.plan(s.estPe, s.estPn, s.missionGoalBearing, b)) {
        s.planValid   = true;
        s.planBearing = b;
    }
}

// Commit a waypoint one step ahead. Head toward the OPERATOR'S GOAL bearing, but
// deflect toward the open corridor when the direct path is blocked, so it goes
// AROUND obstacles while trending toward the goal. The goal is clamped to the
// sensor FoV (can only verify "open" within it) so the drone turns toward a
// side/behind goal gradually, sensing as it goes.
float MissionController::desiredBearing_(const WorldState& s) const {
    // P5b: the occupancy-grid planner supplies a smarter GOAL bearing — routed
    // around obstacles the grid has seen but the live FoV has forgotten, so the
    // goal term no longer pulls back through an obstacle on the direct path (the
    // reactive local-minimum). The live corridor blend below still handles local
    // avoidance around what's actually in front right now.
    const float goalBearing = (p_.useMap && s.planValid) ? s.planBearing
                                                         : s.missionGoalBearing;
    const float goalRel     = wrap180(goalBearing - s.vehYawDeg);
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

    // Valid-but-degraded is not flyable either: the filter keeps estValid while
    // coasting GPS-denied and its uncertainty grows without bound. Legs are
    // flown on this position, so gate on the uncertainty itself.
    if (s.estEphM > p_.maxEphM) {
        phase_ = Phase::SETTLE; tPhase_ = 0.f;
        s.missionPhase = "SETTLE(est-degraded)";
        return c;                   // hover until the estimate recovers
    }

    // Armed but waiting for the operator to press GO — hover in place. Toggling
    // GO off mid-leg drops straight back to a hover here too.
    if (!s.missionGo) {
        phase_ = Phase::SETTLE; tPhase_ = 0.f;
        s.missionPhase = "ARMED";
        return c;                   // hover until GO
    }

    updateMap_(s);   // P5b: accumulate the grid + (re)plan the global route

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
            // Read the (think-tier) plan and commit a leg. A FRESH corridor is
            // required to do anything but hover: valid-but-stale is a latched
            // value from a stalled perception tier, not a plan. Fresh + open →
            // move; fresh + blocked → rotate to look around; stale/none → settle.
            const bool fresh = s.corridorFresh(p_.corridorStaleSec);
            if (fresh && s.corridorOpen >= p_.minOpenToMove) {
                commitWaypoint_(s);
                phase_ = Phase::MOVE; tPhase_ = 0.f;
            } else if (fresh) {
                phase_ = Phase::SCAN; tPhase_ = 0.f;     // cornered -> turn to look
            } else {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;   // no live data -> re-settle
            }
            break;                  // hover this tick
        }
        case Phase::SCAN: {
            // Rotate in place (no translation) toward the openest side until a
            // corridor opens up ahead, then re-plan. Give up after ~a full turn.
            // Scanning blind is pointless — stale perception drops us to SETTLE.
            if (!s.corridorFresh(p_.corridorStaleSec)) {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;
                break;
            }
            if (s.corridorOpen >= p_.minOpenToMove) {
                phase_ = Phase::THINK; tPhase_ = 0.f;    // opening found -> commit
                break;                                   // hover this tick
            }
            if (tPhase_ >= p_.scanTimeoutSec) {
                phase_ = Phase::SETTLE; tPhase_ = 0.f;   // dead end -> settle & retry
                break;
            }
            // Choose a turn direction. WITH a grid route, turn toward it — the
            // planner knows from memory where the opening is, even when it lies
            // well outside the blocked forward FoV (that's the whole point of the
            // map). WITHOUT one, fall back to the reactive wall-follow: keep
            // turning the way we're rounding the obstacle (the openest ray in a
            // boxed FoV is noise), defaulting to a fixed side.
            float dir;
            if (p_.useMap && s.planValid) {
                const float e = wrap180(s.planBearing - s.vehYawDeg);
                dir = (std::fabs(e) < 3.f) ? (roundSign_ != 0.f ? roundSign_ : -1.f)
                                           : (e >= 0.f ? 1.f : -1.f);
            } else {
                dir = (roundSign_ != 0.f)         ? roundSign_
                    : (s.corridorOffset > 0.02f)  ?  1.f
                    : (s.corridorOffset < -0.02f) ? -1.f : -1.f;
            }
            c.yaw = dir * p_.scanYawRate;
            break;                  // yaw only, no pitch
        }
        case Phase::MOVE: {
            // Leg ends after stepM travelled (then stop & re-SLAM), on timeout,
            // when the way ahead closes below the keep threshold — or when the
            // corridor goes STALE mid-leg (perception died: flying blind → stop).
            const float legDist = std::hypot(s.estPe - legE_, s.estPn - legN_);
            const bool  fresh   = s.corridorFresh(p_.corridorStaleSec);
            const bool  blocked = fresh && s.corridorOpen < p_.minOpenToKeep;
            const bool  blind   = !fresh;

            if (legDist >= p_.stepM || tPhase_ >= p_.moveTimeoutSec || blocked || blind) {
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
