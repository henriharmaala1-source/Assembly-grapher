#include "rc_command.hpp"

#include <algorithm>

void RcCommandSource::update(const FcTelemetry& t, ModeManager& modes,
                             WorldState& s, float dt) {
    auto ch = [&](int i) -> int {
        return (i >= 0 && i < t.rcCount && i < 18) ? (int)t.rc[i] : -1;
    };

    // MODE: split [1000,2000] into equal bands, one per configured mode name.
    if (c_.modeAux >= 0 && !c_.modeMap.empty()) {
        const int us = ch(c_.modeAux);
        if (us >= 1000 && us <= 2000) {
            const int n = (int)c_.modeMap.size();
            int band = (us - 1000) * n / 1001;          // 0..n-1
            band = std::max(0, std::min(n - 1, band));
            const std::string& name = c_.modeMap[band];
            if (name != lastMode_) {                    // re-select only on change
                modes.select(name, s);                  // no-op if already active
                lastMode_ = name;
            }
        }
    }

    // GO latch: switch high = go, low = stop (hover). Level, not edge.
    if (c_.goAux >= 0) {
        const int us = ch(c_.goAux);
        if (us >= 1000) s.missionGo = (us >= c_.goUs);
    }

    // STEER: deflect to nudge the goal bearing (same semantics as the arrow keys,
    // rate-based so centring the stick holds the current world-frame goal).
    if (c_.steerAux >= 0) {
        const int us = ch(c_.steerAux);
        if (us >= 1000) {
            const int d = us - 1500;
            if (std::abs(d) > c_.steerDeadbandUs) {
                const float f = std::max(-1.f, std::min(1.f, d / 500.f));
                s.missionGoalBearing += f * c_.steerRateDps * dt;
            }
        }
    }
}
