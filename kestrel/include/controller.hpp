#pragma once

#include "control_types.hpp"
#include "world_model.hpp"

// Maps (world state, behaviour) → a normalised ControlCmd. Pure function of the
// world model; the FC backend turns the command into RC / setpoints.
//
// Outputs are deliberately gentle: every axis is clamped to ±maxAuthority so a
// perception glitch can't command a violent manoeuvre. MANUAL/IDLE return an
// invalid command (no override — pilot keeps the sticks). RTL is handed to the
// FC's own return mode, so the controller just holds level there.
class Controller {
public:
    struct Gains {
        float kpYawTrack  = 1.2f;   // target x-offset → yaw
        float kpPitchTrack = 0.5f;  // forward pursuit while tracking
        float kpYawNav    = 1.0f;   // corridor x-offset → yaw
        float kpYawRoad   = 1.0f;   // road offset → yaw
        float kHeadRoad   = 0.3f;   // road bend feed-forward
        float cruise      = 0.25f;  // nominal forward pitch
        float scanYaw     = 0.15f;  // slow yaw while searching
        float maxAuthority = 0.35f; // hard clamp on every axis
    };

    Controller() = default;
    explicit Controller(Gains g) : g_(g) {}

    // frameW/frameH let TRACK/NAVIGATE convert pixel offsets to [-1,1].
    ControlCmd compute(const WorldState& s, Behavior beh,
                       int frameW, int frameH) const;

private:
    Gains g_{};
    float clamp_(float v) const;
};
