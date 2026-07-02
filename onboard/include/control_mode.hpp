#pragma once

#include <memory>
#include <string>
#include <vector>

#include "control_types.hpp"
#include "controller.hpp"
#include "world_model.hpp"

// ---------------------------------------------------------------------------
// Control modules ("modes"), the control-side twin of IPerceptionModule.
//
// A mode is a self-contained control policy: given the world state, it returns
// the command for this tick (or an invalid command to RELEASE control to the
// pilot / iNAV). Modes are REGISTERED with the ModeManager and selected by name;
// adding a new one is "write a class + mgr.add(...)" — no enum, no switch, no
// touching the manager or main. The safety layers (failsafe → RTH, obstacle →
// HOLD) live once in the manager and apply to any mode via isMotion().
// ---------------------------------------------------------------------------

// Per-tick context handed to every mode: shared control primitives + geometry.
struct ControlCtx {
    Controller* controller = nullptr;   // reactive-control math (hover/road/etc.)
    int         frameW = 0, frameH = 0;
    float       dt     = 0.f;
};

class IControlMode {
public:
    virtual ~IControlMode() = default;

    // Stable identifier, also the selection key (e.g. "FLY", "WAYPOINT").
    virtual const char* name() const = 0;

    // The command for this tick. Return {valid=false} to release control (the
    // pilot or iNAV flies — FLY, WAYPOINT-supervise, LOCK_ON, ...).
    virtual ControlCmd update(WorldState& s, const ControlCtx& ctx) = 0;

    // Does this mode command airframe motion? Motion modes get the obstacle
    // reflex (imminent-obstacle → HOLD) applied by the manager.
    virtual bool isMotion() const { return false; }

    // Does this mode run its OWN obstacle avoidance? If so, the manager's blanket
    // reflex (open<blockedOpen → HOLD) is suppressed for it — the mode is trusted
    // to keep its own standoff (e.g. AUTONOMY: stop/scan/round). Default false:
    // the crude reflex protects the "dumb" motion modes that just release control.
    virtual bool ownsObstacleAvoidance() const { return false; }

    // Behaviour to advertise for the deliberator's hot-module cadence + display.
    virtual Behavior heatBehavior() const { return Behavior::MANUAL; }

    // Entering/leaving this mode (setup / teardown, e.g. enable a sub-cycle).
    virtual void onEnter(WorldState&) {}
    virtual void onExit(WorldState&)  {}
};

// Registry + arbiter. Owns the modes, selects the active one by name, and
// applies the two safety layers uniformly on top of whichever mode is active.
class ModeManager {
public:
    struct Params {
        float rtlBattPct  = 0.15f;   // battery fraction below which → failsafe RTH
        float blockedOpen = 0.30f;   // corridor openness below which → obstacle
    };
    ModeManager() = default;
    explicit ModeManager(Params p) : p_(p) {}

    void add(std::unique_ptr<IControlMode> m);   // register a mode
    bool select(const std::string& name, WorldState& s);   // switch active mode
    IControlMode* active() const { return active_; }
    std::vector<std::string> names() const;

    void requestAbort() { abort_ = true; }        // operator kill → RTH

    // One control tick: failsafe → obstacle reflex → active mode. Sets
    // rthTrigger when the FC should be handed to iNAV RTH; writes opMode /
    // behavior / modeReason into `s`.
    ControlCmd tick(WorldState& s, const ControlCtx& ctx, bool& rthTrigger);

private:
    std::vector<std::unique_ptr<IControlMode>> modes_;
    IControlMode* active_ = nullptr;
    Params        p_;
    bool          abort_  = false;
};
