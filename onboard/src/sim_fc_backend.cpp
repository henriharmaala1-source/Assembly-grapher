#include "sim_fc_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
constexpr float  kPi   = 3.14159265358979323846f;
constexpr double kR    = 6378137.0;                 // WGS84 equatorial radius
constexpr double kD2R  = 3.14159265358979323846 / 180.0;
constexpr float  kMaxV      = 5.0f;                 // m/s at full pitch/roll
constexpr float  kMaxYawDps = 90.0f;                // deg/s at full yaw stick
constexpr float  kMaxClimb  = 3.0f;                 // m/s at full throttle
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

bool SimFcBackend::connect(const std::string&, int) {
    connected_ = true;
    t0_ = tLast_ = clock::now();
    simTime_ = 0.f;
    battV_ = 16.8f;
    std::printf("[sim-fc] simulated FC connected — no hardware; responds to control\n");
    return true;
}

void SimFcBackend::tick() {
    if (!connected_) return;
    const auto now = clock::now();
    float dt = std::chrono::duration<float>(now - tLast_).count();
    tLast_ = now;
    if (dt > 0.5f) dt = 0.5f;         // clamp scheduler stalls
    integrate_(dt);
}

void SimFcBackend::advance(float dt) {
    if (!connected_) return;
    if (dt < 0.f)    dt = 0.f;
    if (dt > 0.5f)   dt = 0.5f;
    integrate_(dt);
}

void SimFcBackend::integrate_(float dt) {
    simTime_ += dt;                                     // model time (wall- or sim-driven)
    const float elapsed = simTime_;
    const bool  armed   = elapsed > 2.0f;               // auto-arm after 2 s

    // Attitude follows the sticks, with a tiny idle wobble so the dashboard is
    // visibly live even with no control commanded.
    const float roll  = cmd_.roll  * 25.f + 0.5f * std::sin(elapsed * 1.7f);
    const float pitch = cmd_.pitch * 25.f + 0.4f * std::sin(elapsed * 2.1f);
    yaw_ += cmd_.yaw * kMaxYawDps * dt;
    if (yaw_ < 0.f)    yaw_ += 360.f;
    if (yaw_ >= 360.f) yaw_ -= 360.f;

    // Ground velocity (ENU): forward pitch drives forward, roll drives lateral,
    // both rotated into the world frame by heading (0 = North, clockwise).
    const float h   = yaw_ * kPi / 180.f;
    const float fwd = cmd_.pitch * kMaxV;
    const float rt  = cmd_.roll  * kMaxV;
    velN_ = fwd * std::cos(h) - rt * std::sin(h);
    velE_ = fwd * std::sin(h) + rt * std::cos(h);

    // Integrate position (metres → degrees) and altitude.
    lat_ += (velN_ * dt) / kR / kD2R;
    lon_ += (velE_ * dt) / (kR * std::cos(lat_ * kD2R)) / kD2R;
    alt_  = std::max(0.f, alt_ + cmd_.throttle * kMaxClimb * dt);

    battV_ -= 0.02f * dt;                               // slow, visible drain

    float course = std::atan2(velE_, velN_) * 180.f / kPi;
    if (course < 0.f) course += 360.f;

    tel_.armed          = armed;
    tel_.linkUp         = true;
    tel_.rollDeg        = roll;
    tel_.pitchDeg       = pitch;
    tel_.yawDeg         = yaw_;
    tel_.lat            = lat_;
    tel_.lon            = lon_;
    tel_.altM           = alt_;
    tel_.baroAltM       = alt_;
    tel_.fixType        = 3;
    tel_.sats           = 12;
    tel_.groundspeedMs  = std::hypot(velN_, velE_);
    tel_.groundCourseDeg = course;
    tel_.battV          = battV_;
    tel_.battPct        = clampf((battV_ - 3.3f * 4) / (4.2f * 4 - 3.3f * 4), 0.f, 1.f);

    // Neutral "operator" RC so the assist-baseline latch and AUX features have
    // plausible channels to read.
    for (int i = 0; i < 4; ++i) tel_.rc[i] = 1500;
    for (int i = 4; i < 8; ++i) tel_.rc[i] = 1000;
    tel_.rcCount = 8;
}

bool SimFcBackend::poll(FcTelemetry& out) {
    out = tel_;
    return connected_;
}

bool SimFcBackend::sendControl(const ControlCmd& cmd) {
    if (!connected_) return false;
    cmd_ = cmd;                    // the model responds to this on the next tick()
    return true;
}
