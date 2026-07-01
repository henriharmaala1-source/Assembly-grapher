#pragma once

#include <chrono>

#include "flight_controller.hpp"

// Software-in-the-loop flight controller: a simple kinematic model that responds
// to sendControl() and produces plausible, evolving telemetry — so the whole OS
// (the --bench-test dashboard, the FSM, controller, estimator, control modes,
// synthetic-GPS feed) can be exercised headless with NO hardware attached.
//
// It is NOT a flight-dynamics sim — just enough fidelity to drive every data
// path: attitude follows the sticks, forward pitch drives ground velocity that
// integrates into GPS lat/lon, throttle drives climb, heading integrates from
// yaw, the battery drains, and it auto-arms a couple seconds after connect().
// Send it a forward-pitch command and its GPS position actually moves — which
// means the estimator sees real motion and the loop closes, all in software.
class SimFcBackend : public IFlightController {
public:
    const char* name() const override { return "sim"; }

    bool connect(const std::string& port, int baud) override;
    void disconnect() override { connected_ = false; }
    bool linkUp() const override { return connected_; }

    void tick() override;
    bool poll(FcTelemetry& out) override;
    bool sendControl(const ControlCmd& cmd) override;

    bool feedExternalGps(const ExtGps&) override { return connected_; }  // accept, no-op
    void setAssistMode(bool) override {}
    void latchBaseline() override {}

private:
    using clock = std::chrono::steady_clock;
    void integrate_(float dt);

    bool              connected_ = false;
    clock::time_point t0_{}, tLast_{};
    ControlCmd        cmd_{};                 // last commanded control

    // Model state.
    double lat_  = 60.1699, lon_ = 24.9384;   // origin (Helsinki)
    float  alt_  = 0.f;
    float  yaw_  = 90.f;                       // facing East
    float  velN_ = 0.f, velE_ = 0.f;
    float  battV_ = 16.8f;                     // 4S full

    FcTelemetry tel_{};
};
