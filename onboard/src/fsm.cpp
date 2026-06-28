#include "fsm.hpp"

Behavior BehaviorFsm::update(const WorldState& s, const OperatorCmd& op,
                             double /*dt*/, std::string& reason) {
    // ---- operator latch ----------------------------------------------------
    if (op.type == OperatorCmd::SET_MODE) {
        manualLatched_ = (op.mode == Behavior::MANUAL);
        cur_ = op.mode;
        reason = "operator set mode";
        return cur_;
    }

    // ---- 1. failsafe (overrides everything except an explicit operator set) -
    if (!s.vehLink && s.vehMode != "SIM") {
        cur_ = (s.vehFix >= 3) ? Behavior::RTL : Behavior::HOLD;
        reason = "FC link lost";
        return cur_;
    }
    if (s.vehBattery <= BATT_RTL) {
        cur_ = (s.vehFix >= 3) ? Behavior::RTL : Behavior::HOLD;
        reason = "battery low";
        return cur_;
    }

    // A latched MANUAL stays until the operator changes it.
    if (manualLatched_) { cur_ = Behavior::MANUAL; reason = "manual (latched)"; return cur_; }

    // ---- debounce counters -------------------------------------------------
    targetLost_   = s.targetValid && s.targetLocked ? 0 : targetLost_ + 1;
    roadGood_     = (s.roadValid && s.roadConf >= ROAD_CONF_ON) ? roadGood_ + 1 : 0;
    corridorGood_ = (s.corridorValid && s.corridorDecisive) ? corridorGood_ + 1 : 0;

    // ---- 2. close high-confidence intruder → EVADE -------------------------
    for (const auto& d : s.detections) {
        if (d.confidence >= DETECT_EVADE_CONF) {
            cur_ = Behavior::EVADE;
            reason = "intruder " + d.label;
            return cur_;
        }
    }

    // ---- 3. autonomy priority ladder ---------------------------------------
    if (s.targetValid && s.targetLocked) {
        cur_ = Behavior::TRACK; reason = "target locked"; return cur_;
    }
    if (cur_ == Behavior::TRACK && targetLost_ < TARGET_DROP_FRAMES) {
        reason = "coasting lost target"; return cur_;       // stay in TRACK briefly
    }
    if (s.targetValid && !s.targetLocked && targetLost_ < TARGET_DROP_FRAMES) {
        cur_ = Behavior::SEARCH; reason = "re-acquiring target"; return cur_;
    }
    if (roadGood_ >= ROAD_ON_FRAMES) {
        cur_ = Behavior::ROAD_FOLLOW; reason = "road locked"; return cur_;
    }
    if (corridorGood_ >= CORRIDOR_ON_FRAMES) {
        cur_ = Behavior::NAVIGATE; reason = "open corridor"; return cur_;
    }

    // Nothing actionable — scan if we were doing something, else idle.
    if (cur_ == Behavior::ROAD_FOLLOW || cur_ == Behavior::NAVIGATE ||
        cur_ == Behavior::TRACK || cur_ == Behavior::SEARCH) {
        cur_ = Behavior::SEARCH; reason = "lost cue, scanning"; return cur_;
    }
    cur_ = Behavior::IDLE; reason = "idle";
    return cur_;
}
