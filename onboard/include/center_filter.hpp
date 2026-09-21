#pragma once

#include <cmath>

// ---------------------------------------------------------------------------
// Alpha-beta filter on the target centre -- a lightweight constant-velocity
// estimator (the practical equivalent of a Kalman [x,y,vx,vy] here). Direct port
// of track/CenterFilter.kt.
// ---------------------------------------------------------------------------

namespace track {

class CenterFilter {
public:
    explicit CenterFilter(float alpha = 0.5f, float beta = 0.15f)
        : alpha_(alpha), beta_(beta) {}

    void start(float px, float py) { x_ = px; y_ = py; vx_ = vy_ = 0.f; init_ = true; }

    // Advance one step. (edx,edy) is this frame's EGO-motion (camera pan) from
    // optical flow -- added to POSITION, not folded into velocity, so the crop
    // follows the pan immediately while vx,vy stay target-relative. No
    // double-counting: the correction residual is measured against this
    // ego-inclusive prediction, so only target motion updates the velocity.
    // VELOCITY IS PIXELS PER SECOND, and every call here takes the elapsed
    // time. It used to be pixels per UPDATE, which silently assumed the frames
    // arrived at a fixed rate -- so a dropped frame changed what the stored
    // velocity physically meant, and a processing stall produced a prediction
    // that had not advanced far enough for the time that had actually passed.
    // On a Pi sharing the machine with depth and mapping, neither is
    // hypothetical.
    void predict(float edx, float edy, float dt, float& ox, float& oy) {
        x_ += vx_ * dt + edx; y_ += vy_ * dt + edy;
        ox = x_; oy = y_;
    }

    // The residual is a POSITION error, so the position gain is unitless and
    // the velocity gain is per unit time: beta * r / dt is the constant that
    // makes a steady residual produce the same steady velocity whatever the
    // frame rate.
    void correct(float mx, float my, float dt) {
        const float rx = mx - x_, ry = my - y_;
        x_ += alpha_ * rx; y_ += alpha_ * ry;
        if (dt > 1e-6f) { vx_ += beta_ * rx / dt; vy_ += beta_ * ry / dt; }
    }

    void project(float sec, float& ox, float& oy) const {
        ox = x_ + vx_ * sec; oy = y_ + vy_ * sec;
    }

    // Cap per-frame velocity. A noisy peak injects a huge residual into
    // correct(), and constant-velocity prediction then compounds it frame after
    // frame until the crop flies off the target. A ceiling sized from the target
    // keeps a real fast target moving while stopping the runaway.
    // maxV is PIXELS PER SECOND.
    void clampSpeed(float maxV) {
        const float s = std::sqrt(vx_ * vx_ + vy_ * vy_);
        if (s > maxV && s > 1e-6f) { const float k = maxV / s; vx_ *= k; vy_ *= k; }
    }

    // Bleed off velocity while coasting so a lost target's crop decelerates near
    // the last sighting instead of sailing out of frame on stale speed.
    //
    // AN EXPONENTIAL WITH A TIME CONSTANT, not a per-frame multiplier. The
    // swept value was 0.6 per frame at 30 fps, which is a 65 ms time constant;
    // expressed that way it decays by the same amount per SECOND however often
    // it is called.
    void decay(float tauSec, float dt) {
        const float k = std::exp(-dt / std::max(1e-6f, tauSec));
        vx_ *= k; vy_ *= k;
    }

    // Force the filtered position, e.g. when the LK coast assist REPLACES the
    // extrapolation for a frame that failed to lock. Velocity is untouched.
    void setPos(float px, float py) { x_ = px; y_ = py; }

    float x() const { return x_; }
    float y() const { return y_; }
    // Pixels per second.
    float speed() const { return std::sqrt(vx_ * vx_ + vy_ * vy_); }
    bool  initialized() const { return init_; }

private:
    float alpha_, beta_;
    float x_ = 0, y_ = 0, vx_ = 0, vy_ = 0;
    bool  init_ = false;
};

}  // namespace track
