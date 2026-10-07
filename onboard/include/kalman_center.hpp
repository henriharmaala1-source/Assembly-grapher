#pragma once

#include <opencv2/video/tracking.hpp>

// Constant-velocity Kalman filter over a 2-D point.
// Direct C++ port of tracker/motion.py — identical state vector, transition
// matrix, and noise constants so behaviour matches the Python app exactly.
//
// State   = [x, y, vx, vy]   (position + velocity, pixels / pixels-per-frame)
// Measure = [x, y]            (tracked centre each frame)
class KalmanCenter {
public:
    KalmanCenter();

    void init(cv::Point2f pos);

    // Advance one step; returns the a-priori predicted (x, y).
    cv::Point2f predict();

    // Fuse a measurement; returns the filtered (x, y).
    cv::Point2f correct(cv::Point2f measurement);

    cv::Point2f position() const;
    cv::Point2f velocity() const;

    // Predicted centre `steps` frames into the future (no state change).
    cv::Point2f project(float steps) const;

    // Bound the per-frame velocity. A noisy/false measurement injects a large
    // residual, and constant-velocity prediction then COMPOUNDS it every frame
    // until the box sails off the target ("wanders away in a straight line").
    // A ceiling sized from the target keeps a genuinely fast target moving while
    // stopping the runaway.
    void clampVelocity(float maxPxPerFrame);

    // Bleed velocity off while coasting, so a lost target's box decelerates to a
    // stop near the last sighting instead of flying out of frame on stale speed.
    void decayVelocity(float factor);

    bool initialized() const { return initialized_; }

private:
    cv::KalmanFilter kf_;
    bool             initialized_ = false;
};
