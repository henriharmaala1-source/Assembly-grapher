#include "control_mode.hpp"

void ModeManager::add(std::unique_ptr<IControlMode> m) {
    if (!m) return;
    if (!active_) active_ = m.get();     // first registered becomes the default
    modes_.push_back(std::move(m));
}

bool ModeManager::select(const std::string& name, WorldState& s) {
    for (auto& m : modes_) {
        if (name == m->name()) {
            if (active_ == m.get()) return true;
            if (active_) active_->onExit(s);
            active_ = m.get();
            active_->onEnter(s);
            return true;
        }
    }
    return false;   // unknown mode name — caller keeps the current mode
}

std::vector<std::string> ModeManager::names() const {
    std::vector<std::string> out;
    out.reserve(modes_.size());
    for (const auto& m : modes_) out.emplace_back(m->name());
    return out;
}

ControlCmd ModeManager::tick(WorldState& s, const ControlCtx& ctx, bool& rthTrigger) {
    rthTrigger = false;

    // ---- safety layer 1: failsafe → iNAV RTH (the OS releases control) ------
    const bool lowBatt = s.vehLink && s.vehArmed && s.vehBattery < p_.rtlBattPct;
    if (abort_ || lowBatt) {
        rthTrigger   = true;
        s.opMode     = "RTH";
        s.behavior   = Behavior::RTL;
        s.modeReason = abort_ ? "operator abort" : "low battery";
        ControlCmd c; c.valid = false;      // hand the aircraft to iNAV's RTH
        return c;
    }

    if (!active_) { s.opMode = "none"; ControlCmd c; c.valid = false; return c; }

    // ---- safety layer 2: obstacle reflex for motion modes → HOLD (stop) ----
    const bool obstacle = s.corridorValid && s.corridorOpen < p_.blockedOpen;
    if (active_->isMotion() && obstacle) {
        s.opMode     = active_->name();
        s.behavior   = Behavior::HOLD;
        s.modeReason = "obstacle -> HOLD";
        return ctx.controller->compute(s, Behavior::HOLD, ctx.frameW, ctx.frameH);
    }

    // ---- active mode --------------------------------------------------------
    s.opMode   = active_->name();
    s.behavior = active_->heatBehavior();
    s.modeReason.clear();
    return active_->update(s, ctx);
}
