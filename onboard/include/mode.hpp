#pragma once

#include "control_types.hpp"
#include "controller.hpp"
#include "mission.hpp"
#include "world_model.hpp"

// Top-level operator-intent modes. Exactly ONE is active at a time. A safety
// layer inside ModeArbiter pre-empts any of them, so these are NOT siblings of
// EVADE/RTL/HOLD-failsafe — those live underneath (see ModeArbiter::tick):
//   1. failsafe : low battery / operator abort            -> iNAV RTH (release)
//   2. reflex   : imminent obstacle while a motion mode   -> HOLD (stop)
//   3. else     : the active operator mode drives control
enum class Mode {
    FLY,            // manual passthrough — the OS does not override control
    ASSIST,         // pilot flies; the OS only pre-empts obstacles
    LOCK_ON,        // designate + track a subject (SENSING only; pilot flies)
    FOLLOW_ROAD,    // follow a road / line centreline
    FOLLOW_SUBJECT, // standoff-follow a tracked subject     [control TODO]
    WAYPOINT,       // iNAV flies the GPS route; the OS supervises (detect + avoid)
    AUTONOMY,       // move-stop-sense SLAM cycle (demo)
    HOLD,           // position hold / hover
};
const char* mode_name(Mode m);

// Single control arbiter — replaces the old FSM-auto-ladder + Mission override
// glue with one coherent hierarchy. The operator picks the Mode; the arbiter
// applies the safety layers and produces the command (delegating to the
// Controller for reactive modes and the MissionController for AUTONOMY).
class ModeArbiter {
public:
    struct Params {
        float rtlBattPct  = 0.15f;   // battery fraction below which -> failsafe RTH
        float blockedOpen = 0.30f;   // corridor openness below which -> obstacle
    };
    ModeArbiter() = default;
    explicit ModeArbiter(Params p) : p_(p) {}

    void setMode(Mode m, MissionController& mission);  // (re)configures AUTONOMY
    Mode mode() const { return mode_; }
    void requestAbort() { abort_ = true; }             // operator kill -> RTH

    // One control tick. Returns the command; sets `rthTrigger` when the FC should
    // be handed to iNAV RTH (failsafe / abort). Writes s.opMode / s.behavior /
    // s.modeReason for telemetry + the deliberator's hot-module cadence.
    ControlCmd tick(WorldState& s, Controller& ctl, MissionController& mission,
                    int frameW, int frameH, float dt, bool& rthTrigger);

private:
    Params p_;
    Mode   mode_  = Mode::FLY;
    bool   abort_ = false;
};
