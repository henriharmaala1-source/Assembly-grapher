#pragma once
// ---------------------------------------------------------------------------
// A PID for the OUTER loops: "where the target point is" -> a stick. The
// flight controller's own PIDs sit underneath (rates, attitude, altitude and
// position hold); this decides what the sticks ask of them.
//
//   P  the error itself
//   D  on the MEASUREMENT, low-pass filtered (time constant dTau): a jumpy
//      tracker box or a detector at a few Hz would otherwise put its noise
//      straight on the stick. Zero on the first step after reset(), so
//      entering a mode never kicks.
//   I  off by default (ki = 0). With it: clamped to +-iMax, and frozen while
//      the output is saturated in the direction it would grow (no windup).
//      Horizontally it competes with the FC's own position hold for the
//      same drift -- tune it in the playground before trusting it.
//
// The units are the caller's: here an error in fractions of 90 deg (angles)
// or metres (range), an output in stick (-1..1).
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>

struct PidGains {
    float kp = 0.f, ki = 0.f, kd = 0.f;
    float iMax = 0.3f;       // the integral's contribution, at most
    float dTau = 0.1f;       // s: derivative low-pass
    float outMax = 1.f;      // |output| cap
};

class Pid {
public:
    void reset() { i_ = 0.f; d_ = 0.f; prevMeas_ = 0.f; have_ = false; }

    // `err` = setpoint - measurement; `meas` the measurement itself (the
    // derivative is taken on it, with the sign that opposes its motion).
    float step(float err, float meas, float dt, const PidGains& g) {
        if (!(dt > 0.f)) dt = 1e-3f;
        float dMeas = 0.f;
        if (have_) dMeas = (meas - prevMeas_) / dt;
        prevMeas_ = meas;
        // First-order filter on the measurement's rate.
        const float a = dt / (g.dTau + dt);
        d_ = have_ ? d_ + a * (dMeas - d_) : 0.f;
        have_ = true;
        const float p = g.kp * err;
        const float d = -g.kd * d_;
        float out = p + i_ + d;
        // Integrate only when it would not push a saturated output further.
        if (g.ki != 0.f) {
            const bool satHi = out >= g.outMax && err > 0.f;
            const bool satLo = out <= -g.outMax && err < 0.f;
            if (!satHi && !satLo) {
                i_ += g.ki * err * dt;
                i_ = std::max(-g.iMax, std::min(g.iMax, i_));
            }
            out = p + i_ + d;
        }
        return std::max(-g.outMax, std::min(g.outMax, out));
    }
    float integral() const { return i_; }

private:
    float i_ = 0.f, d_ = 0.f, prevMeas_ = 0.f;
    bool  have_ = false;
};
