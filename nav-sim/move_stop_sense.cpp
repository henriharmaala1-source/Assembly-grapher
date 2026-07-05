#include "move_stop_sense.hpp"

#include <cmath>

namespace navsim {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
inline float wrap180(float d){ while(d>180.f)d-=360.f; while(d<=-180.f)d+=360.f; return d; }
}

void MoveStopSense::reset() {
    phase_ = Phase::SETTLE; tPhase_ = 0.f; haveWp_ = false; roundSign_ = 0.f;
}

const char* MoveStopSense::phaseName() const {
    switch (phase_) {
        case Phase::SETTLE: return "SETTLE";
        case Phase::THINK:  return "THINK";
        case Phase::SCAN:   return "SCAN";
        case Phase::MOVE:   return "MOVE";
        case Phase::ARRIVE: return "ARRIVE";
    }
    return "?";
}

// Ported verbatim in spirit from MissionController::desiredBearing_: head toward
// the goal (grid route if valid, else the raw goal bearing), but deflect toward
// the vetted-open corridor when the way ahead is blocked; the goal's influence
// falls off quadratically with the corridor's deflection, and a hard clamp keeps
// the heading no less deflected than the corridor while it's avoiding.
float MoveStopSense::desiredBearing_(const MssInput& in) const {
    const float goalBearing = (p_.useMap && in.planValid) ? in.planBearing : in.goalBearing;
    const float goalRel     = wrap180(goalBearing - in.yawDeg);
    const float goalClamped = clampf(goalRel, -p_.hFovDeg*0.5f, p_.hFovDeg*0.5f);
    const float corridorRel = in.corridorOffset * (p_.hFovDeg*0.5f);
    const float deflect     = clampf(std::fabs(in.corridorOffset), 0.f, 1.f);
    float goalW = 1.f - deflect; goalW *= goalW;
    float relBearing = goalW*goalClamped + (1.f-goalW)*corridorRel;
    if (corridorRel < 0.f) relBearing = std::min(relBearing, corridorRel);
    if (corridorRel > 0.f) relBearing = std::max(relBearing, corridorRel);
    return in.yawDeg + relBearing;
}

MssOutput MoveStopSense::update(const MssInput& in, float dt) {
    MssOutput o; o.speedScale = 0.f; o.bearingDeg = in.yawDeg;   // default: hover
    tPhase_ += dt;

    switch (phase_) {
        case Phase::SETTLE: {
            const bool settled = in.speedMs < p_.settleSpeedMs;
            if (tPhase_ >= p_.settleSec && settled) { phase_ = Phase::THINK; tPhase_ = 0.f; }
            break;                                         // hover
        }
        case Phase::THINK: {
            if (in.corridorOpen >= p_.minOpenToMove) {
                const float b = desiredBearing_(in) * kPi/180.f;
                legE_ = in.e; legN_ = in.n;
                wpE_ = in.e + p_.stepM*std::sin(b); wpN_ = in.n + p_.stepM*std::cos(b);
                haveWp_ = true;
                phase_ = Phase::MOVE; tPhase_ = 0.f;
            } else { phase_ = Phase::SCAN; tPhase_ = 0.f; } // cornered -> look
            break;                                         // hover this tick
        }
        case Phase::SCAN: {
            if (in.corridorOpen >= p_.minOpenToMove) { phase_ = Phase::THINK; tPhase_ = 0.f; break; }
            if (tPhase_ >= p_.scanTimeoutSec)        { phase_ = Phase::SETTLE; tPhase_ = 0.f; break; }
            float dir;
            if (p_.useMap && in.planValid) {
                const float e = wrap180(in.planBearing - in.yawDeg);
                dir = (std::fabs(e) < 3.f) ? (roundSign_!=0.f?roundSign_:-1.f)
                                           : (e>=0.f?1.f:-1.f);
            } else {
                dir = (roundSign_!=0.f)        ? roundSign_
                    : (in.corridorOffset>0.02f)?  1.f
                    : (in.corridorOffset<-0.02f)? -1.f : -1.f;
            }
            o.yawScan = true; o.speedScale = 0.f;
            o.bearingDeg = in.yawDeg + dir*90.f;           // keep turning `dir` in place
            break;
        }
        case Phase::MOVE: {
            const float legDist = std::hypot(in.e-legE_, in.n-legN_);
            const bool blocked = in.corridorOpen < p_.minOpenToKeep;
            if (legDist >= p_.stepM || tPhase_ >= p_.moveTimeoutSec || blocked) {
                phase_ = Phase::ARRIVE; tPhase_ = 0.f; break;   // hover
            }
            const float desired = desiredBearing_(in);
            o.bearingDeg = desired;
            const float openScale = clampf(in.corridorOpen, 0.3f, 1.f);
            o.speedScale = clampf(p_.cruise * openScale, 0.f, 1.f);
            if (std::fabs(in.corridorOffset) > 0.3f)
                roundSign_ = in.corridorOffset < 0.f ? -1.f : 1.f;
            else if (in.corridorOpen > 0.6f && std::fabs(in.corridorOffset) < 0.1f)
                roundSign_ = 0.f;
            const float b = desired*kPi/180.f;
            wpE_ = in.e + p_.stepM*std::sin(b); wpN_ = in.n + p_.stepM*std::cos(b);
            break;
        }
        case Phase::ARRIVE: { phase_ = Phase::SETTLE; tPhase_ = 0.f; break; }  // stop, re-think
    }

    o.phase = phaseName(); o.wpE = wpE_; o.wpN = wpN_;
    return o;
}

}  // namespace navsim
