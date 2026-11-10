#include "rc_command.hpp"

#include <algorithm>

void RcCommandSource::update(const FcTelemetry& t, ModeManager& modes,
                             WorldState& s, float dt) {
    auto ch = [&](int i) -> int {
        return (i >= 0 && i < t.rcCount && i < 18) ? (int)t.rc[i] : -1;
    };

    // MODE: split [1000,2000] into equal bands, one per configured mode name,
    // with hysteresis and a dwell (RcConfig) so a wheel cannot chatter.
    if (c_.modeAux >= 0 && !c_.modeMap.empty()) {
        const int us = ch(c_.modeAux);
        if (us >= 1000 && us <= 2000) {
            const int n = (int)c_.modeMap.size();
            auto bandOf = [&](int u) { return std::max(0, std::min(n - 1, (u - 1000) * n / 1001)); };
            auto lo = [&](int b) { return 1000 + (b * 1001 + n - 1) / n; };   // first us in band b
            const int band = bandOf(us);
            int want = band_;
            if (band_ < 0) {
                want = band;                            // first reading: take it as it is
            } else if (band != band_) {
                // Clearly out of the current band, not just over its edge.
                const bool clear = us >= lo(band_ + 1) + c_.modeHystUs ||
                                   us <  lo(band_) - c_.modeHystUs;
                if (!clear) {
                    pendBand_ = -1;
                } else if (band != pendBand_) {
                    pendBand_ = band; pendS_ = 0.f;
                    if (c_.modeDwellS <= 0.f) want = band;
                } else if ((pendS_ += dt) >= c_.modeDwellS) {
                    want = band;
                }
            } else {
                pendBand_ = -1;                         // back where it was
            }
            if (want != band_) {
                band_ = want; pendBand_ = -1;
                const std::string& name = c_.modeMap[band_];
                if (name != lastMode_) {                // re-select only on change
                    modes.select(name, s);              // no-op if already active
                    lastMode_ = name;
                }
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
