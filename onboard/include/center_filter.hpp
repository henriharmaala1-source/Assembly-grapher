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
    void predict(float edx, float edy, float& ox, float& oy) {
        x_ += vx_ + edx; y_ += vy_ + edy;
        ox = x_; oy = y_;
    }

    void correct(float mx, float my) {
        const float rx = mx - x_, ry = my - y_;
        x_ += alpha_ * rx; y_ += alpha_ * ry;
        vx_ += beta_ * rx; vy_ += beta_ * ry;
    }

    void project(int steps, float& ox, float& oy) const {
        ox = x_ + vx_ * steps; oy = y_ + vy_ * steps;
    }

    // Cap per-frame velocity. A noisy peak injects a huge residual into
    // correct(), and constant-velocity prediction then compounds it frame after
    // frame until the crop flies off the target. A ceiling sized from the target
    // keeps a real fast target moving while stopping the runaway.
    void clampSpeed(float maxV) {
        const float s = std::sqrt(vx_ * vx_ + vy_ * vy_);
        if (s > maxV && s > 1e-6f) { const float k = maxV / s; vx_ *= k; vy_ *= k; }
    }

    // Bleed off velocity while coasting so a lost target's crop decelerates near
    // the last sighting instead of sailing out of frame on stale speed.
    void decay(float f) { vx_ *= f; vy_ *= f; }

    // Force the filtered position, e.g. when the LK coast assist REPLACES the
    // extrapolation for a frame that failed to lock. Velocity is untouched.
    void setPos(float px, float py) { x_ = px; y_ = py; }

    float x() const { return x_; }
    float y() const { return y_; }
    float speed() const { return std::sqrt(vx_ * vx_ + vy_ * vy_); }
    bool  initialized() const { return init_; }

private:
    float alpha_, beta_;
    float x_ = 0, y_ = 0, vx_ = 0, vy_ = 0;
    bool  init_ = false;
};

}  // namespace track
