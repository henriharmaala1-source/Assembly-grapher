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

    // Unset channels are ignored (rcCount too small / -1 values).
    RcConfig off; RcCommandSource none(off);
    CHECK(!none.enabled());

    std::printf("test_rc: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
