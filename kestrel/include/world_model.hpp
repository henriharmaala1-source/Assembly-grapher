#pragma once

#include <opencv2/core.hpp>
#include <mutex>
#include <string>
#include <vector>

// ---------------------------------------------------------------- behaviour
enum class Behavior { IDLE, NAVIGATE, TRACK, SEARCH, EVADE, RTL, HOLD };
const char* behavior_name(Behavior b);

// ---------------------------------------------------------------- detections
struct Detection {
    std::string label;
    float       confidence = 0.f;
    cv::Rect    box;
};

// ----------------------------------------------------------------- world state
// Single snapshot of everything the runtime knows. Perception modules and the
// MAVLink bridge (P2) write into it; the behaviour arbiter, telemetry logger,
// and the on-device LLM supervisor (P3) read from it. brief() is the compact
// text form the LLM consumes — keep it short and stable.
struct WorldState {
    long   frameId = 0;
    double fps     = 0.0;

    Behavior behavior = Behavior::IDLE;

    // --- Track (lock-on) ---
    bool        targetValid  = false;
    bool        targetLocked = false;
    bool        targetCoast  = false;
    cv::Rect    targetBox;
    cv::Point2f targetVel;            // px / frame
    float       targetConf   = 0.f;
    long        targetAge    = 0;
    int         targetLosses = 0;

    // --- Navigate (monocular corridor) ---
    bool        corridorValid    = false;
    bool        corridorDecisive = false;
    cv::Point2f corridorHeading;      // steer target, frame px
    float       corridorOpen   = 0.f; // clearance at heading [0,1]
    float       corridorMargin = 0.f; // decisiveness [0,1]

    // --- Detect ---
    std::vector<Detection> detections;

    // --- Vehicle telemetry (filled by MAVLink in P2; stubbed for now) ---
    bool        vehArmed   = false;
    float       vehBattery = 1.0f;    // [0,1]
    float       vehAltM    = 0.f;
    std::string vehMode    = "SIM";

    std::string brief() const;        // one-line scene state (LLM input)
};

// ----------------------------------------------------------------- world model
// Thread-safe holder. Today the loop is single-threaded, but the LLM sidecar
// (P3) reads asynchronously, so the lock is here from the start.
class WorldModel {
public:
    template <class F>
    void with(F&& fn) {
        std::lock_guard<std::mutex> lk(mu_);
        fn(state_);
    }
    WorldState snapshot() const {
        std::lock_guard<std::mutex> lk(mu_);
        return state_;
    }

private:
    mutable std::mutex mu_;
    WorldState         state_;
};
