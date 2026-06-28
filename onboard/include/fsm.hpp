#pragma once

#include <opencv2/core.hpp>
#include <string>

#include "world_model.hpp"

// Operator intent forwarded into the FSM (from keys, or later the LLM / RC).
struct OperatorCmd {
    enum Type { NONE, SET_MODE, LOCK, RESET } type = NONE;
    Behavior  mode = Behavior::IDLE;   // for SET_MODE
    cv::Point lockPt;                  // for LOCK
};

// Hysteretic behaviour arbiter. Replaces the P1 stub arbitrate().
//
// Priority (highest first):
//   1. Failsafe   — low battery / lost FC link → RTL (or HOLD if no GPS).
//   2. Operator   — an explicit SET_MODE latches until changed or pre-empted.
//   3. Autonomy   — TRACK > EVADE > ROAD_FOLLOW > NAVIGATE > SEARCH.
// Counters debounce transitions so the mode doesn't flap frame-to-frame.
class BehaviorFsm {
public:
    Behavior update(const WorldState& s, const OperatorCmd& op,
                    double dt, std::string& reason);
    Behavior current() const { return cur_; }

private:
    // Tunables (frames / fractions).
    static constexpr int   TARGET_DROP_FRAMES = 15;  // TRACK→SEARCH after lost this long
    static constexpr int   ROAD_ON_FRAMES     = 5;   // confident road before ROAD_FOLLOW
    static constexpr int   CORRIDOR_ON_FRAMES = 3;   // decisive corridor before NAVIGATE
    static constexpr float ROAD_CONF_ON       = 0.50f;
    static constexpr float BATT_RTL           = 0.15f;
    static constexpr float DETECT_EVADE_CONF  = 0.80f;

    Behavior cur_ = Behavior::IDLE;

    bool     manualLatched_ = false;
    int      roadGood_   = 0;
    int      corridorGood_ = 0;
    int      targetLost_ = 0;
};
