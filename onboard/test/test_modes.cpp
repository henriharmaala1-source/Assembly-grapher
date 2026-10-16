// ModeManager safety layers + MissionController gates, against stub modes.
// Covers: failsafe (battery/abort), the obstacle reflex, staleness handling
// (F1), the no-perception surfacing (F4), ownsObstacleAvoidance suppression,
// and the mission's no-est / est-degraded (F3) / ARMED / blind-stop gates.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "control_mode.hpp"
#include "mission.hpp"
#include "modes.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

// A dumb motion mode: always cruises forward. The reflex must protect it.
struct StubMotion : IControlMode {
    const char* name() const override { return "STUB_MOTION"; }
    bool isMotion() const override { return true; }
    ControlCmd update(WorldState&, const ControlCtx&) override {
        ControlCmd c; c.valid = true; c.pitch = 0.3f; return c;
    }
};

// Same, but claims its own avoidance — the blanket reflex must stand down.
struct StubOwnAvoid : StubMotion {
    const char* name() const override { return "STUB_OWN"; }
    bool ownsObstacleAvoidance() const override { return true; }
};

WorldState healthyState() {
    WorldState s;
    s.vehLink = true; s.vehArmed = true; s.vehBattery = 1.f;
    s.tickMonoS = 100.0;
    return s;
}

}  // namespace

int main() {
    Controller ctl;
    ControlCtx ctx;
    ctx.controller = &ctl; ctx.frameW = 640; ctx.frameH = 480; ctx.dt = 0.02f;

    { // failsafe: low battery while armed + linked → release + RTH trigger
        ModeManager mgr; mgr.add(std::make_unique<StubMotion>());
        WorldState s = healthyState(); s.vehBattery = 0.10f;
        bool rth = false;
        const ControlCmd c = mgr.tick(s, ctx, rth);
        CHECK(rth);
        CHECK(!c.valid);                       // released — iNAV RTH flies
        CHECK(s.opMode == "RTH");
    }

    { // failsafe: operator abort
        ModeManager mgr; mgr.add(std::make_unique<StubMotion>());
        mgr.requestAbort();
        WorldState s = healthyState();
        bool rth = false;
        mgr.tick(s, ctx, rth);
        CHECK(rth);
    }

    { // reflex: FRESH blocked corridor pre-empts a motion mode to HOLD
        ModeManager mgr; mgr.add(std::make_unique<StubMotion>());
        WorldState s = healthyState();
        s.corridorValid = true; s.corridorStampS = 99.9; s.corridorOpen = 0.1f;
        bool rth = false;
        const ControlCmd c = mgr.tick(s, ctx, rth);
        CHECK(!rth);
        CHECK(s.modeReason == "obstacle -> HOLD");
        CHECK(c.valid);
        CHECK(c.pitch == 0.f);                 // HOLD, not the stub's cruise
    }

    { // STALE blocked corridor: no reflex (it would act on fiction), and the
      // unprotected state is SURFACED — the mode still runs (that's the risk)
        ModeManager mgr; mgr.add(std::make_unique<StubMotion>());
        WorldState s = healthyState();
        s.corridorValid = true; s.corridorStampS = 90.0;   // 10 s old
        s.corridorOpen = 0.1f;                             // "blocked", but stale
        bool rth = false;
        const ControlCmd c = mgr.tick(s, ctx, rth);
        CHECK(s.modeReason == "no obstacle perception");
        CHECK(c.pitch == 0.3f);                // stub ran un-reflexed
    }

    { // no corridor at all: same surfacing
        ModeManager mgr; mgr.add(std::make_unique<StubMotion>());
        WorldState s = healthyState();
        bool rth = false;
        mgr.tick(s, ctx, rth);
        CHECK(s.modeReason == "no obstacle perception");
    }

    { // ownsObstacleAvoidance: fresh blocked corridor does NOT pre-empt
        ModeManager mgr;
        mgr.add(std::make_unique<StubMotion>());
        mgr.add(std::make_unique<StubOwnAvoid>());
        WorldState s = healthyState();
        CHECK(mgr.select("STUB_OWN", s));
        s.corridorValid = true; s.corridorStampS = 99.9; s.corridorOpen = 0.1f;
        bool rth = false;
        const ControlCmd c = mgr.tick(s, ctx, rth);
        CHECK(c.pitch == 0.3f);                // mode kept control
        CHECK(s.modeReason.empty());           // and is not flagged unprotected
    }

    { // mission gates: no-est → est-degraded (F3) → ARMED → flies → blind-stop
        MissionController m;
        m.enable(true);
        WorldState s; s.tickMonoS = 0.0; s.missionGo = true;

        ControlCmd c = m.update(s, 0.02f);
        CHECK(s.missionPhase == "SETTLE(no-est)");
        CHECK(c.valid && c.pitch == 0.f);      // hover

        s.estValid = true; s.estEphM = 5.f;    // valid but degraded
        m.update(s, 0.02f);
        CHECK(s.missionPhase == "SETTLE(est-degraded)");

        s.estEphM = 1.f; s.missionGo = false;  // healthy, waiting for GO
        m.update(s, 0.02f);
        CHECK(s.missionPhase == "ARMED");

        // GO + fresh open corridor: settle through to a moving leg.
        s.missionGo = true;
        s.corridorValid = true; s.corridorOpen = 1.f; s.corridorOffset = 0.f;
        for (int i = 0; i < 150; ++i) {        // 3 s
            s.tickMonoS += 0.02; s.corridorStampS = s.tickMonoS;
            c = m.update(s, 0.02f);
        }
        CHECK(s.missionPhase == "MOVE");
        CHECK(c.valid && c.pitch > 0.f);

        // Corridor stops updating mid-leg (stamps freeze): must stop cruising.
        for (int i = 0; i < 150; ++i) {
            s.tickMonoS += 0.02;               // stamps stay behind → stale
            c = m.update(s, 0.02f);
        }
        CHECK(c.pitch == 0.f);                 // no forward motion while blind
        CHECK(s.missionPhase != "MOVE");
    }

    { // SHADOW (P2.3): runs the full cycle but NEVER takes control
        ShadowMode sh;
        ControlCtx sctx; sctx.controller = &ctl; sctx.frameW = 640; sctx.frameH = 480;
        sctx.dt = 0.02f;
        WorldState s = healthyState();
        sh.onEnter(s);
        CHECK(s.shadowActive);

        // GO + fresh open corridor: in AUTONOMY this reaches a MOVE that commands
        // forward pitch. SHADOW must still release, every single tick.
        s.estValid = true; s.estEphM = 1.f; s.missionGo = true;
        s.corridorValid = true; s.corridorOpen = 1.f; s.corridorOffset = 0.f;
        ControlCmd c;
        bool everValid = false;
        for (int i = 0; i < 250; ++i) {
            s.tickMonoS += 0.02; s.corridorStampS = s.tickMonoS;
            c = sh.update(s, sctx);
            everValid = everValid || c.valid;
        }
        CHECK(!everValid);                     // NEVER commands the airframe
        CHECK(s.missionPhase == "MOVE");       // …though the cycle really ran
        CHECK(s.shadowCmd.valid);              // …and published its intent
        CHECK(s.shadowCmd.pitch > 0.f);        // (forward, as a MOVE leg would)
        sh.onExit(s);
        CHECK(!s.shadowActive);
    }

    std::printf("test_modes: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
