#pragma once

#include <cmath>

namespace navsim {

// A deliberately simple kinematic drone (unicycle): position + heading, a max
// speed and a yaw-rate limit. Not a flight-dynamics model — this testbench is
// about comparing PLANNERS, so the vehicle is intentionally neutral. Heading
// convention matches the rest of the code: yaw in degrees, 0 = North, +E clockwise.
struct Drone {
    float e = 0.f, n = 0.f, yawDeg = 0.f;
    float maxSpeed   = 3.0f;   // m/s
    float yawRateDeg = 120.f;  // deg/s

    static float wrap180(float d) { while (d > 180.f) d -= 360.f; while (d <= -180.f) d += 360.f; return d; }

    // Turn toward targetBearing (deg) at the yaw-rate limit and translate forward.
    // `speedScale` in [0,1] lets the caller slow/stop for safety or sharp turns.
    void step(float targetBearingDeg, float dt, float speedScale) {
        constexpr float kPi = 3.14159265358979323846f;
        const float err  = wrap180(targetBearingDeg - yawDeg);
        const float dyaw = std::max(-yawRateDeg * dt, std::min(yawRateDeg * dt, err));
        yawDeg = wrap180(yawDeg + dyaw);
        // slow down when the heading error is large (don't sprint sideways)
        const float headScale = std::max(0.f, std::cos(err * kPi / 180.f));
        const float sp = maxSpeed * speedScale * headScale;
        e += std::sin(yawDeg * kPi / 180.f) * sp * dt;
        n += std::cos(yawDeg * kPi / 180.f) * sp * dt;
    }
};

}  // namespace navsim
