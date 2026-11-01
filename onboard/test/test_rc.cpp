// RC command source: switch-band mode select, GO latch, and rate-based goal
// steer with a centre deadband. Uses the real ModeManager + standard modes.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "rc_command.hpp"
#include "modes.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {
FcTelemetry rcFrame(int mode, int go, int steer) {
    FcTelemetry t{};
    t.rcCount = 8;
    for (int i = 0; i < 8; ++i) t.rc[i] = 1500;
    if (mode  >= 0) t.rc[4] = (uint16_t)mode;
    if (go    >= 0) t.rc[5] = (uint16_t)go;
    if (steer >= 0) t.rc[6] = (uint16_t)steer;
    return t;
}
}  // namespace

int main() {
    RcConfig cfg;
    cfg.modeAux = 4; cfg.modeMap = {"FLY", "AUTONOMY", "HOLD"};   // 3 bands
    cfg.goAux = 5;   cfg.goUs = 1700;
    cfg.steerAux = 6; cfg.steerRateDps = 90.f; cfg.steerDeadbandUs = 40;
    cfg.modeDwellS = 0.f;                    // immediate, for the band checks here
    RcCommandSource rc(cfg);
    CHECK(rc.enabled());

    ModeManager modes;
    register_standard_modes(modes);
    WorldState s; s.vehYawDeg = 100.f;

    // MODE bands: low→FLY, mid→AUTONOMY, high→HOLD.
    rc.update(rcFrame(1050, -1, 1500), modes, s, 0.02f);
    CHECK(std::string(modes.active()->name()) == "FLY");
    rc.update(rcFrame(1500, -1, 1500), modes, s, 0.02f);
    CHECK(std::string(modes.active()->name()) == "AUTONOMY");
    rc.update(rcFrame(1950, -1, 1500), modes, s, 0.02f);
    CHECK(std::string(modes.active()->name()) == "HOLD");

    // GO latch: high = go, low = stop.
    rc.update(rcFrame(1500, 1800, 1500), modes, s, 0.02f);
    CHECK(s.missionGo);
    rc.update(rcFrame(1500, 1200, 1500), modes, s, 0.02f);
    CHECK(!s.missionGo);

    // STEER: centred (within deadband) holds the goal; deflection nudges it.
    s.missionGoalBearing = 100.f;
    rc.update(rcFrame(1500, -1, 1510), modes, s, 0.10f);   // within 40µs deadband
    CHECK(std::fabs(s.missionGoalBearing - 100.f) < 1e-4);
    // Full right (2000) for 0.1 s at 90 deg/s → +9°.
    rc.update(rcFrame(1500, -1, 2000), modes, s, 0.10f);
    CHECK(std::fabs(s.missionGoalBearing - 109.f) < 0.5f);
    // Full left symmetric.
    s.missionGoalBearing = 100.f;
    rc.update(rcFrame(1500, -1, 1000), modes, s, 0.10f);
    CHECK(std::fabs(s.missionGoalBearing - 91.f) < 0.5f);

    // A WHEEL parked on the FLY|AUTONOMY edge (1334 us is the first AUTONOMY
    // microsecond) with +-8 us of jitter must not chatter, and a real move
    // must still go through -- after the dwell, not on the first frame.
    {
        RcConfig w;
        w.modeAux = 4; w.modeMap = {"FLY", "AUTONOMY", "HOLD"};
        w.modeHystUs = 30; w.modeDwellS = 0.3f;
        RcCommandSource wheel(w);
        ModeManager m2;
        register_standard_modes(m2);
        WorldState s2;
        wheel.update(rcFrame(1100, -1, -1), m2, s2, 0.02f);
        CHECK(std::string(m2.active()->name()) == "FLY");
        int changes = 0;
        std::string last = m2.active()->name();
        for (int k = 0; k < 200; ++k) {
            const int us = 1334 + ((k * 7) % 17) - 8;          // 1326..1342
            wheel.update(rcFrame(us, -1, -1), m2, s2, 0.02f);
            if (last != m2.active()->name()) { ++changes; last = m2.active()->name(); }
        }
        CHECK(changes == 0);
        CHECK(std::string(m2.active()->name()) == "FLY");
        // Rolled properly into AUTONOMY: held 0.2 s -- not yet; 0.4 s -- yes.
        for (int k = 0; k < 10; ++k) wheel.update(rcFrame(1500, -1, -1), m2, s2, 0.02f);
        CHECK(std::string(m2.active()->name()) == "FLY");
        for (int k = 0; k < 10; ++k) wheel.update(rcFrame(1500, -1, -1), m2, s2, 0.02f);
        CHECK(std::string(m2.active()->name()) == "AUTONOMY");
        // A brush through HOLD shorter than the dwell changes nothing.
        for (int k = 0; k < 5; ++k) wheel.update(rcFrame(1900, -1, -1), m2, s2, 0.02f);
        wheel.update(rcFrame(1500, -1, -1), m2, s2, 0.02f);
        for (int k = 0; k < 30; ++k) wheel.update(rcFrame(1500, -1, -1), m2, s2, 0.02f);
        CHECK(std::string(m2.active()->name()) == "AUTONOMY");
        std::printf("  wheel: 200 jittery frames on a band edge, 0 mode changes; "
                    "a real move lands after 0.3 s\n");
    }

    // Unset channels are ignored (rcCount too small / -1 values).
    RcConfig off; RcCommandSource none(off);
    CHECK(!none.enabled());

    std::printf("test_rc: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
