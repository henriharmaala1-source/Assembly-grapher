#pragma once

#include <cmath>

namespace sim {

// SHORT-MEMORY ODOMETRY FROM AN IMU, WITH AN HONEST EXPIRY DATE.
//
// `POSE_AND_OPENNESS_PLAN.md` §6 says "do not use this IMU for translation",
// and gives the reason: the killer is not accel bias but ATTITUDE COUPLING.
// One degree of tilt error leaks 9.81*sin(1 deg) = 0.171 m/s^2 into horizontal
// acceleration, and that double-integrates.
//
// That warning is right and this class does not contradict it. What it does is
// put a NUMBER on how long the warning takes to bite, because the near map's
// memory window has a lower bound too:
//
//     drift = 0.5 * 9.81 * sin(theta) * t^2  <  one cell (0.25 m)
//
//     theta      usable window
//     0.5 deg      2.4 s
//     1.0 deg      1.7 s
//     2.0 deg      1.2 s
//
// The map needs 2-3 s to plan through a turn (§3). So IMU-only translation
// lands right ON the edge of useful -- viable at good attitude, not at poor.
// Which makes it a thing to MEASURE and bound, not a thing to assume either
// way.
//
// THE ARCHITECTURE MAKES THIS WORK, and that is the real reason it is worth
// having. `MissionController` flies move-stop-sense: SETTLE and THINK are
// STATIONARY. A stationary vehicle has known-zero velocity, which is a free
// ZERO-VELOCITY UPDATE (ZUPT) -- the classic bound on inertial drift. Drift
// therefore accumulates over ONE MOVE LEG rather than over the whole flight,
// and the leg is exactly the window the map wants.
//
// So this is not dead reckoning with unbounded error. It is dead reckoning
// between resets, on a vehicle that stops on purpose.
//
// WHAT IT REFUSES TO DO: it does not extrapolate past its own budget. Once the
// predicted drift exceeds `maxDriftM` the pose is marked INVALID and stays that
// way until a ZUPT. A consumer that ignores `valid` gets a pose that is
// arithmetically fine and physically meaningless -- the same failure the map's
// "unknown is not free" rule exists to prevent, one layer down.
struct ImuOdometryParams {
    // 1-sigma attitude error, degrees. This is the parameter that matters and
    // it is NOT a property of the IMU -- it is a property of whatever supplies
    // attitude. EKF3 roll/pitch, gravity-referenced, is the good case; a
    // complementary filter on the camera's own IMU under thrust is the bad one.
    float attErrDeg   = 1.0f;
    // How much drift is allowed before the pose is refused. One voxel is the
    // natural unit: past that the map cannot tell where the evidence belongs.
    float maxDriftM   = 0.25f;
    // Accel white noise, m/s^2/sqrt(Hz), and bias stability, m/s^2. Included
    // for completeness and because the comparison is the point: at BMI055
    // grades these are an order of magnitude below the tilt term.
    float accelNoise  = 0.0015f;
    float accelBias   = 0.0098f;   // ~1 mg
    // Below this speed the vehicle is taken to be stationary and a ZUPT fires.
    float zuptSpeedMs = 0.05f;
};

class ImuOdometry {
public:
    void init(const ImuOdometryParams& p) { p_ = p; reset(); }
    const ImuOdometryParams& params() const { return p_; }

    // Hard reset: position, velocity and the error budget all go to zero.
    void reset() {
        px_ = py_ = pz_ = 0.f; vx_ = vy_ = vz_ = 0.f;
        tSinceResetS_ = 0.f; valid_ = true;
    }

    // ZERO-VELOCITY UPDATE. Call when the vehicle is known stationary -- which
    // move-stop-sense hands over for free during SETTLE and THINK.
    //
    // Velocity is the state that matters: it is the one that integrates into
    // position, so zeroing it is what actually stops the drift growing. Position
    // is NOT zeroed, because the accumulated translation since the last reset is
    // the thing the map wants; the error clock is what restarts.
    void zupt() {
        vx_ = vy_ = vz_ = 0.f;
        tSinceResetS_ = 0.f;
        valid_ = true;
    }

    // One step. `ax,ay,az` are SPECIFIC FORCE in the world frame with gravity
    // ALREADY REMOVED -- i.e. what is left after subtracting the gravity vector
    // rotated by the current attitude estimate. Doing the removal outside keeps
    // this class from needing to know the attitude convention, and the residual
    // tilt error is accounted for by `attErrDeg` rather than modelled here.
    void step(float ax, float ay, float az, float dt) {
        if (dt <= 0.f) return;
        // Trapezoidal would be marginally better; at 200-400 Hz against a tilt
        // term that dominates by an order of magnitude it is not the error that
        // matters, and saying so is cheaper than pretending otherwise.
        vx_ += ax * dt; vy_ += ay * dt; vz_ += az * dt;
        px_ += vx_ * dt; py_ += vy_ * dt; pz_ += vz_ * dt;
        tSinceResetS_ += dt;
        if (predictedDriftM() > p_.maxDriftM) valid_ = false;
    }

    // The whole point of the class: what the tilt coupling has cost by now.
    //   d = 0.5 * g * sin(attErr) * t^2
    float predictedDriftM() const {
        const float a = 9.81f * std::sin(p_.attErrDeg * 3.14159265f / 180.f);
        return 0.5f * a * tSinceResetS_ * tSinceResetS_;
    }
    // And how long this is good for from a fresh reset, which is the number a
    // caller actually needs when choosing a memory window.
    float usableWindowS() const {
        const float a = 9.81f * std::sin(p_.attErrDeg * 3.14159265f / 180.f);
        return (a > 1e-6f) ? std::sqrt(2.f * p_.maxDriftM / a) : 1e9f;
    }

    bool  valid() const { return valid_; }
    float sinceResetS() const { return tSinceResetS_; }
    void  position(float& x, float& y, float& z) const { x = px_; y = py_; z = pz_; }
    void  velocity(float& x, float& y, float& z) const { x = vx_; y = vy_; z = vz_; }
    float speed() const { return std::sqrt(vx_*vx_ + vy_*vy_ + vz_*vz_); }

private:
    ImuOdometryParams p_;
    float px_ = 0, py_ = 0, pz_ = 0;
    float vx_ = 0, vy_ = 0, vz_ = 0;
    float tSinceResetS_ = 0.f;
    bool  valid_ = true;
};

}  // namespace sim
