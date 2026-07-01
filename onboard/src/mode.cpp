#include "mode.hpp"

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::FLY:            return "FLY";
        case Mode::ASSIST:         return "ASSIST";
        case Mode::LOCK_ON:        return "LOCK_ON";
        case Mode::FOLLOW_ROAD:    return "FOLLOW_ROAD";
        case Mode::FOLLOW_SUBJECT: return "FOLLOW_SUBJECT";
        case Mode::WAYPOINT:       return "WAYPOINT";
        case Mode::AUTONOMY:       return "AUTONOMY";
        case Mode::HOLD:           return "HOLD";
    }
    return "?";
}

namespace {
// A representative reactive Behavior per mode, for the deliberator's hot-module
// cadence and the display — not the control source (that's the arbiter).
Behavior modeBehavior(Mode m) {
    switch (m) {
        case Mode::FOLLOW_ROAD:    return Behavior::ROAD_FOLLOW;
        case Mode::LOCK_ON:
        case Mode::FOLLOW_SUBJECT: return Behavior::TRACK;
        case Mode::WAYPOINT:
        case Mode::AUTONOMY:       return Behavior::NAVIGATE;  // heat depth/avoid
        case Mode::HOLD:           return Behavior::HOLD;
        default:                   return Behavior::MANUAL;
    }
}
}  // namespace

void ModeArbiter::setMode(Mode m, MissionController& mission) {
    mode_ = m;
    mission.enable(m == Mode::AUTONOMY);   // AUTONOMY owns the stop-think cycle
}

ControlCmd ModeArbiter::tick(WorldState& s, Controller& ctl, MissionController& mission,
                             int frameW, int frameH, float dt, bool& rthTrigger) {
    rthTrigger = false;

    // ---- safety layer 1: failsafe -> iNAV RTH (the OS releases control) ------
    const bool lowBatt = s.vehLink && s.vehArmed && s.vehBattery < p_.rtlBattPct;
    if (abort_ || lowBatt) {
        rthTrigger   = true;
        s.opMode     = "RTH";
        s.behavior   = Behavior::RTL;
        s.modeReason = abort_ ? "operator abort" : "low battery";
        ControlCmd c; c.valid = false;      // hand the aircraft to iNAV's RTH
        return c;
    }

    const bool motion = (mode_ == Mode::FOLLOW_ROAD || mode_ == Mode::FOLLOW_SUBJECT ||
                         mode_ == Mode::WAYPOINT   || mode_ == Mode::AUTONOMY);
    const bool obstacle = s.corridorValid && s.corridorOpen < p_.blockedOpen;

    // ---- safety layer 2: obstacle reflex -> HOLD (stop) ---------------------
    if (motion && obstacle) {
        s.opMode     = mode_name(mode_);
        s.behavior   = Behavior::HOLD;
        s.modeReason = "obstacle -> HOLD";
        return ctl.compute(s, Behavior::HOLD, frameW, frameH);
    }

    // ---- active operator mode ----------------------------------------------
    s.opMode   = mode_name(mode_);
    s.behavior = modeBehavior(mode_);
    s.modeReason.clear();

    switch (mode_) {
        case Mode::FLY:            // pure pilot passthrough
        case Mode::ASSIST:         // pilot flies; obstacle pre-empt above is the help
        case Mode::LOCK_ON:        // sensing only; pilot flies
        case Mode::FOLLOW_SUBJECT: // standoff-follow control not built yet
        case Mode::WAYPOINT: {     // iNAV flies the route; the OS only supervises
            ControlCmd c; c.valid = false; return c;   // no OS override
        }
        case Mode::HOLD:
            return ctl.compute(s, Behavior::HOLD, frameW, frameH);
        case Mode::FOLLOW_ROAD:
            return ctl.compute(s, Behavior::ROAD_FOLLOW, frameW, frameH);
        case Mode::AUTONOMY:
            return mission.update(s, dt);
    }
    ControlCmd c; c.valid = false; return c;
}
