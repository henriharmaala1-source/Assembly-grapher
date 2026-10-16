# Adding a control mode

A **control mode** is a self-contained policy that decides what the aircraft
does: given the current world state, it returns the control command for this
tick — or releases control to the pilot / iNAV. Modes are the control-side twin
of perception modules (`IPerceptionModule`): you **implement an interface and
register it**, never edit an enum or a switch.

This guide is everything you need to add one.

---

## 1. Where a mode fits

Control runs in the fast **fly loop** (not the deliberative think thread), once
per tick. The `ModeManager` owns all registered modes, keeps exactly one active,
and wraps two **safety layers** around whichever mode is active:

```
ModeManager::tick(state, ctx, rthTrigger):
   1. failsafe : low battery / operator abort   →  release + rthTrigger=true
                 (main then calls fc->setMode(RTL) — iNAV flies the return)
   2. reflex   : if the active mode isMotion() and the corridor ahead is
                 blocked (corridorOpen < blockedOpen)   →  HOLD (stop)
   3. else     : active_->update(state, ctx)            →  your command
```

**You write only step 3.** The failsafe and the obstacle-stop are done for you —
your mode just opts into the obstacle reflex by returning `true` from
`isMotion()`.

---

## 2. The interface — `IControlMode` (`control_mode.hpp`)

```cpp
class IControlMode {
    virtual const char* name() const = 0;                    // selection key
    virtual ControlCmd  update(WorldState& s, const ControlCtx& ctx) = 0;
    virtual bool        isMotion()   const { return false; } // opt into obstacle reflex
    virtual Behavior    heatBehavior() const { return Behavior::MANUAL; }
    virtual void        onEnter(WorldState&) {}              // becoming active
    virtual void        onExit(WorldState&)  {}              // leaving
};
```

| Method | What it does | When you override it |
| ------ | ------------ | -------------------- |
| `name()` | Stable id **and** the selection key (`"WAYPOINT"`, `"FLY"`). | Always. |
| `update()` | Return the command for this tick. **This is the mode.** | Always. |
| `isMotion()` | `true` = this mode drives the airframe forward/around → it gets the obstacle-stop reflex. `false` = it releases or only hovers. | If your mode moves. |
| `heatBehavior()` | The reactive `Behavior` to advertise, so the deliberator runs the *relevant* heavy module hot (e.g. `NAVIGATE` heats depth). | If your mode needs a heavy module fresh. |
| `onEnter/onExit()` | Setup / teardown — enable a sub-cycle, reset state, arm a timer. | If your mode has internal state. |

### The context — `ControlCtx`

```cpp
struct ControlCtx {
    Controller* controller;   // shared reactive-control primitives (below)
    int         frameW, frameH;
    float       dt;           // seconds since last tick
};
```

`controller` gives you tested reactive mappings so you don't re-derive them:
`ctx.controller->compute(s, Behavior::HOLD, ctx.frameW, ctx.frameH)` for a level
hover, `Behavior::ROAD_FOLLOW` to steer a road, etc. Use it or ignore it.

---

## 3. What a `ControlCmd` means

```cpp
struct ControlCmd {
    float roll;      // + = right          [-1, 1]
    float pitch;     // + = forward        [-1, 1]
    float yaw;       // + = clockwise      [-1, 1]
    float throttle;  // 0 = hover-hold, 1 = full climb   [0, 1]
    bool  valid;     // false = DO NOT OVERRIDE (pilot / iNAV keeps control)
};
```

- Return `{}` (default — `valid == false`) to **release** control. FLY, LOCK_ON,
  and WAYPOINT (iNAV flies) all do this.
- Axes are normalised; the FC backend maps them to RC µs. They are clamped to a
  gentle authority downstream, so you can't command a violent manoeuvre.
- Throttle `0` is *hold altitude* (the FC's alt-hold holds), not "motor off".

---

## 4. Reading the world — what `update()` has to work with

Everything perception and the estimator produce is on `WorldState s` (already a
consistent snapshot for this tick). The usual suspects:

| You want… | Fields |
| --------- | ------ |
| open-corridor steering | `corridorValid`, `corridorOffset` (−1..1, L..R), `corridorOpen` (0..1) |
| road/line steering | `roadValid`, `roadOffset`, `roadHeading` |
| a tracked subject | `targetValid`, `targetLocked`, `targetBox`, `targetVel` |
| detections | `detections[]` (label, box, confidence) |
| position / motion | `estValid`, `estPe`, `estPn`, `estPu`, `estSpeed`, `vehYawDeg` |
| vehicle status | `vehArmed`, `vehBattery`, `vehLink`, `vehAltM` |

Write your own mode-specific outputs back into `s` too (as the mission cycle does
with `missionPhase`/`missionWpE`) for telemetry/display.

---

## 5. Write it — a full worked example

A `CREEP` mode: inch forward along the open corridor at a fraction of cruise —
a cautious NAVIGATE. It reads the corridor, yaws toward the opening, and creeps.
Because it moves, it's a motion mode, so it automatically gets the obstacle stop.

```cpp
// creep_mode.hpp
#include "control_mode.hpp"

class CreepMode : public IControlMode {
public:
    const char* name() const override { return "CREEP"; }
    bool        isMotion()   const override { return true; }          // obstacle reflex
    Behavior    heatBehavior() const override { return Behavior::NAVIGATE; } // heat depth

    ControlCmd update(WorldState& s, const ControlCtx& c) override {
        ControlCmd cmd;
        if (!s.corridorValid) return cmd;            // nothing to steer by → release
        cmd.valid = true;
        cmd.yaw   = 0.8f * s.corridorOffset;         // turn toward the opening
        cmd.pitch = 0.10f * s.corridorOpen;          // creep, scaled by how clear it is
        return cmd;
    }
};
```

That's the entire mode. No manager changes, no enum, no switch.

---

## 6. Register it — and bind a key

Registration (in `modes.hpp`'s `register_standard_modes`, or anywhere with the
manager):

```cpp
mgr.add(std::make_unique<CreepMode>());   // now selectable by name "CREEP"
```

The first mode registered is the **default** at startup. Selecting a mode:

```cpp
wm.with([&](WorldState& s){ modes.select("CREEP", s); });  // triggers onExit/onEnter
```

Bind a key in `main.cpp`'s display handler (next to the others):

```cpp
if (k == 'c') setm("CREEP");
```

An unknown name is rejected (`select` returns `false`) and the current mode is
kept — safe to wire selection from anywhere (keys, an RC AUX channel, the LLM
supervisor later).

---

## 7. Rules & gotchas

- **`update()` runs in the fly loop** — keep it cheap and non-blocking (µs, not
  ms). Heavy work (SLAM, planning, a NN) belongs in a **perception module** on
  the Deliberator thread; your mode *reads its result* from the world model.
- **Release when you can't act.** No valid input (no corridor, no target, no
  estimate) → return `{}` and let the pilot/FC hold. Don't command blind.
- **`isMotion()` is your safety opt-in.** If your mode ever drives the airframe
  forward/around, return `true` so the obstacle reflex can stop it. Hover-only or
  release-only modes stay `false`.
- **Don't re-implement the safety layers.** Failsafe→RTH and obstacle→HOLD are
  the manager's job; your mode only does the nominal case.
- **State lives in the mode object**, initialised in `onEnter`. The manager calls
  `onExit` on the old mode and `onEnter` on the new one across a switch.

---

## 8. Templates to copy

The standard modes in `modes.hpp` are minimal, real examples:

- `FlyMode` / `AssistMode` / `LockOnMode` — release control (`return {}`).
- `HoldMode` / `RoadFollowMode` — delegate to `ctx.controller`.
- `WaypointMode` — release **but** `isMotion()` (iNAV flies, OS supervises).
- `AutonomyMode` — owns a sub-controller (`MissionController`), enabled in
  `onEnter` / disabled in `onExit`.

Pick the closest and adapt.
