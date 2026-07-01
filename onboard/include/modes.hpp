#pragma once

#include "control_mode.hpp"
#include "mission.hpp"

// The standard control modes for the demonstrator. Each is a small, independent
// IControlMode — a template for writing your own: implement update() (+ a couple
// of trivial overrides) and register it with ModeManager::add(). See
// register_standard_modes() at the bottom.

// FLY — pure pilot passthrough; the OS never overrides control.
class FlyMode : public IControlMode {
public:
    const char* name() const override { return "FLY"; }
    ControlCmd  update(WorldState&, const ControlCtx&) override { return {}; }
};

// ASSIST — the pilot flies; the OS's only intervention is the obstacle reflex
// (handled by the manager), so this mode itself releases control.
class AssistMode : public IControlMode {
public:
    const char* name() const override { return "ASSIST"; }
    bool        isMotion() const override { return true; }   // let the reflex help
    ControlCmd  update(WorldState&, const ControlCtx&) override { return {}; }
};

// LOCK_ON — designate + track a subject (SENSING only; the tracker runs in
// perception). The airframe is the pilot's; this mode adds no motion.
class LockOnMode : public IControlMode {
public:
    const char* name() const override { return "LOCK_ON"; }
    Behavior    heatBehavior() const override { return Behavior::TRACK; }
    ControlCmd  update(WorldState&, const ControlCtx&) override { return {}; }
};

// HOLD — position hold / hover.
class HoldMode : public IControlMode {
public:
    const char* name() const override { return "HOLD"; }
    Behavior    heatBehavior() const override { return Behavior::HOLD; }
    ControlCmd  update(WorldState& s, const ControlCtx& c) override {
        return c.controller->compute(s, Behavior::HOLD, c.frameW, c.frameH);
    }
};

// FOLLOW_ROAD — steer along the detected road/line centreline.
class RoadFollowMode : public IControlMode {
public:
    const char* name() const override { return "FOLLOW_ROAD"; }
    bool        isMotion() const override { return true; }
    Behavior    heatBehavior() const override { return Behavior::ROAD_FOLLOW; }
    ControlCmd  update(WorldState& s, const ControlCtx& c) override {
        return c.controller->compute(s, Behavior::ROAD_FOLLOW, c.frameW, c.frameH);
    }
};

// WAYPOINT — iNAV flies the GPS route; the OS supervises. It releases control
// (so iNAV commands the airframe) but is a motion mode, so the obstacle reflex
// still pre-empts it to HOLD. Detection runs independently in the deliberator.
class WaypointMode : public IControlMode {
public:
    const char* name() const override { return "WAYPOINT"; }
    bool        isMotion() const override { return true; }
    Behavior    heatBehavior() const override { return Behavior::NAVIGATE; }
    ControlCmd  update(WorldState&, const ControlCtx&) override { return {}; }
};

// AUTONOMY — the move-stop-sense cycle. Owns its MissionController; enabling and
// disabling it is tied to entering/leaving the mode.
class AutonomyMode : public IControlMode {
public:
    const char* name() const override { return "AUTONOMY"; }
    bool        isMotion() const override { return true; }
    Behavior    heatBehavior() const override { return Behavior::NAVIGATE; }
    void onEnter(WorldState& s) override {
        mission_.enable(true);
        s.missionGoalBearing = s.vehYawDeg;   // default: straight ahead
        s.missionGo = false;                  // armed, waiting for GO
    }
    void onExit(WorldState& s) override { mission_.enable(false); s.missionGo = false; }
    ControlCmd  update(WorldState& s, const ControlCtx& c) override {
        return mission_.update(s, c.dt);
    }
private:
    MissionController mission_;
};

// FOLLOW_SUBJECT — standoff-follow a tracked subject. Control not built yet
// (standoff-keeping only); releases for now so it's a safe placeholder.
class FollowSubjectMode : public IControlMode {
public:
    const char* name() const override { return "FOLLOW_SUBJECT"; }
    Behavior    heatBehavior() const override { return Behavior::TRACK; }
    ControlCmd  update(WorldState&, const ControlCtx&) override { return {}; }
};

// Register the standard demonstrator set. The FIRST added (FLY) is the default.
inline void register_standard_modes(ModeManager& mgr) {
    mgr.add(std::make_unique<FlyMode>());
    mgr.add(std::make_unique<AssistMode>());
    mgr.add(std::make_unique<LockOnMode>());
    mgr.add(std::make_unique<HoldMode>());
    mgr.add(std::make_unique<RoadFollowMode>());
    mgr.add(std::make_unique<WaypointMode>());
    mgr.add(std::make_unique<AutonomyMode>());
    mgr.add(std::make_unique<FollowSubjectMode>());
}
