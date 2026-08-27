#pragma once

#include <algorithm>
#include <cmath>

namespace sim {

// ---------------------------------------------------------------------------
// ATTITUDE FROM AN IMU. Gravity levels; the gyro carries between levellings.
//
// WHY ROLL AND PITCH ARE FREE AND YAW IS NOT. An accelerometer at rest measures
// the gravity vector, and gravity is an ABSOLUTE reference for two of the three
// angles: it says which way is down, and nothing whatever about which way is
// north. So roll and pitch can be corrected forever and yaw cannot -- it is
// gyro integration alone, and it drifts. That asymmetry is not a limitation of
// this filter, it is a property of the measurement, and it is the reason the
// aircraft carries a compass and this file does not pretend to replace one.
//
// THE ACCELEROMETER IS ONLY A LEVEL WHEN NOT ACCELERATING. Under any sustained
// translational acceleration it reads gravity PLUS that acceleration and levels
// to the wrong vertical -- worst exactly when the vehicle is manoeuvring, which
// is when attitude matters most. So the correction is GATED on the magnitude
// looking like gravity, and during a manoeuvre the filter coasts on the gyro.
// Coasting briefly is a small error; levelling to a lie is a large one.
//
// FRAME. Camera convention throughout: +x right, +y down, +z forward, matching
// DepthCamera. Level means gravity along +y.
//
// THE D435i'S OWN AXES ARE NOT VERIFIED HERE. Intel's motion frames do not
// necessarily arrive in this convention, and the sign of each axis is a thing
// to CHECK ON THE BENCH rather than assume: point the camera at a wall, tilt it
// by hand, and confirm the reported angle moves the way the camera did. The
// mapping belongs at the call site for exactly that reason.
// ---------------------------------------------------------------------------

struct AttitudeParams {
    // Weight on the gyro per update. 0.98 at 200 Hz is a ~0.25 s time constant:
    // long enough that a footstep does not tip the horizon, short enough that
    // gyro bias cannot walk away.
    float alpha = 0.98f;
    // How far |a| may sit from g and still be believed, m/s^2.
    //
    // MAGNITUDE ALONE IS A WEAK GATE AND THE NUMBERS SAY SO. Adding a
    // PERPENDICULAR acceleration changes the magnitude only to second order:
    // 4 m/s^2 sideways against 9.81 of gravity gives |a| = 10.59, a deviation
    // of 0.78 -- under any sane threshold -- while tilting the apparent horizon
    // by 22 degrees. Measured in attitude_check, which fails against a
    // magnitude-only gate. So magnitude is kept as a cheap first filter and the
    // real work is done by disagreement, below.
    float accelGateMs2 = 1.5f;
    // DISAGREEMENT GATE. The gyro-propagated attitude is an independent
    // opinion, and lateral acceleration shows up here at FIRST order: the
    // accelerometer claims a tilt the gyro never saw. Trust falls off linearly
    // between these, rather than cutting off, because a hard cut lets a slow
    // gyro-bias walk lock the correction out permanently -- the estimate drifts
    // away, the disagreement grows, and the only thing that could fix it is
    // switched off by its own error.
    float disagreeFullDeg = 5.f;    // believed entirely below this
    float disagreeNoneDeg = 15.f;   // ignored entirely above it
    float gMs2 = 9.81f;
};

class AttitudeFilter {
public:
    void init(const AttitudeParams& p) { p_ = p; roll_ = pitch_ = yaw_ = 0.f; seeded_ = false; }

    // Level the filter from one accelerometer sample, with no blending. Use it
    // once at startup: a complementary filter started at zero in a tilted
    // vehicle takes its whole time constant to walk to the truth, and that walk
    // is indistinguishable from a real rotation to everything downstream.
    void seed(float ax, float ay, float az) {
        rollPitchFromAccel(ax, ay, az, roll_, pitch_);
        seeded_ = true;
    }

    // `w*` rad/s in camera axes, `a*` m/s^2 in the same axes, dt seconds.
    void update(float wx, float wy, float wz,
                float ax, float ay, float az, float dt) {
        if (dt <= 0.f || dt > 0.5f) return;      // a gap is not an integration
        if (!seeded_) { seed(ax, ay, az); return; }

        // Gyro first: rotation about the forward axis is roll, about the right
        // axis is pitch, about the down axis is yaw.
        roll_  += wz * dt * kRad2Deg;
        pitch_ += wx * dt * kRad2Deg;
        yaw_   += wy * dt * kRad2Deg;

        const float mag = std::sqrt(ax*ax + ay*ay + az*az);
        float trust = (std::fabs(mag - p_.gMs2) <= p_.accelGateMs2) ? 1.f : 0.f;

        float ar, ap;
        rollPitchFromAccel(ax, ay, az, ar, ap);
        const float dr = wrap180(ar - roll_), dp = wrap180(ap - pitch_);
        const float disagree = std::max(std::fabs(dr), std::fabs(dp));
        if (trust > 0.f) {
            const float lo = p_.disagreeFullDeg, hi = p_.disagreeNoneDeg;
            trust = (disagree <= lo) ? 1.f
                  : (disagree >= hi) ? 0.f
                  : (hi - disagree) / std::max(1e-3f, hi - lo);
        }
        accelTrusted_ = trust > 0.f;
        if (trust > 0.f) {
            const float k = (1.f - p_.alpha) * trust;
            roll_  += k * dr;
            pitch_ += k * dp;
        }
        yaw_ = wrap360(yaw_);
    }

    float rollDeg()  const { return roll_; }
    float pitchDeg() const { return pitch_; }
    float yawDeg()   const { return yaw_; }        // DRIFTS. No absolute reference.
    bool  accelTrusted() const { return accelTrusted_; }
    bool  seeded() const { return seeded_; }
    void  setYawDeg(float y) { yaw_ = wrap360(y); }   // from a compass, if there is one

private:
    static constexpr float kRad2Deg = 57.2957795f;

    // Level is gravity along +y, so both angles are zero there. Pinned by
    // attitude_check rather than argued for here.
    static void rollPitchFromAccel(float ax, float ay, float az,
                                   float& rollDeg, float& pitchDeg) {
        rollDeg  = std::atan2(ax, ay) * kRad2Deg;
        pitchDeg = std::atan2(-az, std::sqrt(ax*ax + ay*ay)) * kRad2Deg;
    }
    static float wrap180(float d) {
        while (d > 180.f) d -= 360.f;
        while (d < -180.f) d += 360.f;
        return d;
    }
    static float wrap360(float d) {
        while (d >= 360.f) d -= 360.f;
        while (d < 0.f) d += 360.f;
        return d;
    }

    AttitudeParams p_;
    float roll_ = 0, pitch_ = 0, yaw_ = 0;
    bool  seeded_ = false, accelTrusted_ = true;
};

}  // namespace sim
