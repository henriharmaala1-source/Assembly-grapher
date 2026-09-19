#pragma once

#include <string>
#include <vector>

#include "control_mode.hpp"
#include "control_types.hpp"
#include "world_model.hpp"

// RC command source — the radio as a command source for the world model, so the
// drone is flyable with no laptop. It reads the RC channels the FC reports back
// (FcTelemetry.rc[], via MSP_RC) and maps configured channels to:
//   - MODE  : a switch position → select a mode by name (bands across 1000-2000)
//   - GO    : a switch high/low → the AUTONOMY/SHADOW "go" latch
//   - STEER : a stick/knob deflection → nudge the goal bearing (rate-based, like
//             the arrow keys), with a centre deadband
//
// Purely a producer into WorldState + ModeManager (the same surfaces the
// keyboard drives); it commands no motion itself. Any unset channel is ignored.
struct RcConfig {
    int         modeAux = -1;                 // channel index; <0 = off
    std::vector<std::string> modeMap;         // mode names, low→high band order
    int         goAux   = -1;
    int         goUs    = 1700;               // ≥ this on goAux = GO
    int         steerAux = -1;
    float       steerRateDps    = 45.f;       // deg/s at full deflection
    int         steerDeadbandUs = 40;         // ± around 1500 ignored
};

class RcCommandSource {
public:
    explicit RcCommandSource(RcConfig c) : c_(std::move(c)) {}

    bool enabled() const {
        return c_.modeAux >= 0 || c_.goAux >= 0 || c_.steerAux >= 0;
    }

    // Apply RC inputs. Call inside a wm.with() block (mutates s + selects modes).
    void update(const FcTelemetry& t, ModeManager& modes, WorldState& s, float dt);

private:
    RcConfig    c_;
    std::string lastMode_;   // only re-select on a band change
};
