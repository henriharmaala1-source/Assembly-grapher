// Standalone checks for the ENU Kalman estimator: origin + ENU round-trip,
// the iNAV-style glitch gate, the forced re-acquire after persistent glitches,
// and uncertainty growth while coasting (the input to the F3 mission gate).
// Compiles against the real state_estimator.cpp — no hardware, CI-safe.

#include <cmath>
#include <cstdio>

#include "state_estimator.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

int main() {
    constexpr double kLat0 = 60.0, kLon0 = 25.0;
    constexpr double kR    = 6378137.0;                    // matches estimator
    constexpr double kD2R  = 3.14159265358979323846 / 180.0;
    const double mLat = 1.0 / (kR * kD2R);                 // deg per metre north
    const double mLon = mLat / std::cos(kLat0 * kD2R);     // deg per metre east

    { // origin on first 3-D fix, then track a 10 m eastward walk
        StateEstimator est;
        CHECK(!est.state().valid);
        est.updateGps(kLat0, kLon0, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        CHECK(est.hasOrigin());
        CHECK(est.state().valid);

        for (int i = 1; i <= 50; ++i) {                    // 5 s, 2 m/s east
            est.predict(0.1f);
            est.updateGps(kLat0, kLon0 + (i * 0.2) * mLon, 100.f, 3,
                          true, 0.f, 2.f, 0.f, 1.f, 1.5f);
        }
        const auto s = est.state();
        CHECK(std::fabs(s.pe - 10.f) < 1.0f);
        CHECK(std::fabs(s.pn) < 0.5f);
        CHECK(std::fabs(s.ve - 2.f) < 0.5f);
        CHECK(s.ephM < 1.5f);
        // Round-trip: reprojected geodetic ≈ the fix we fed
        CHECK(std::fabs(s.lon - (kLon0 + 10.0 * mLon)) < 2.0 * mLon);
    }

    { // glitch gate: a 30 m jump while GPS is continuous must be rejected
        StateEstimator est;
        est.updateGps(kLat0, kLon0, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        for (int i = 0; i < 20; ++i) {
            est.predict(0.1f);
            est.updateGps(kLat0, kLon0, 100.f, 3, true, 0, 0, 0, 1.f, 1.5f);
        }
        est.predict(0.1f);
        est.updateGps(kLat0, kLon0 + 30.0 * mLon, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        CHECK(std::fabs(est.state().pe) < 2.0f);           // did not follow the jump
    }

    { // re-acquire: 5 consecutive glitching fixes mean WE are wrong — snap
        StateEstimator est;
        est.updateGps(kLat0, kLon0, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        for (int i = 0; i < 20; ++i) {
            est.predict(0.05f);
            est.updateGps(kLat0, kLon0, 100.f, 3, true, 0, 0, 0, 1.f, 1.5f);
        }
        for (int i = 0; i < 5; ++i) {
            est.predict(0.05f);
            est.updateGps(kLat0, kLon0 + 30.0 * mLon, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        }
        CHECK(std::fabs(est.state().pe - 30.f) < 2.0f);    // snapped to the GPS
    }

    { // coasting: no measurements → gpsDenied + eph inflates past the F3 gate
        StateEstimator est;
        est.updateGps(kLat0, kLon0, 100.f, 3, false, 0, 0, 0, 1.f, 1.5f);
        for (int i = 0; i < 20; ++i) {
            est.predict(0.1f);
            est.updateGps(kLat0, kLon0, 100.f, 3, true, 0, 0, 0, 1.f, 1.5f);
        }
        const float eph0 = est.state().ephM;
        for (int i = 0; i < 100; ++i) est.predict(0.1f);   // 10 s blind
        const auto s = est.state();
        CHECK(s.gpsDenied);
        CHECK(s.ephM > eph0);
        CHECK(s.ephM > 3.0f);                              // crosses maxEphM in 10 s
    }

    std::printf("test_estimator: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
