#pragma once

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <memory>
#include <vector>

#include "kalman_center.hpp"

enum class Backend { CSRT, KCF, FLOW };
const char* backend_name(Backend b);

// Sparse Lucas-Kanade tracker with forward-backward validation and
// median point-spread scale estimation.
class LKFlowTracker {
public:
    bool init(const cv::Mat& frame, const cv::Rect& box);
    bool update(const cv::Mat& frame, cv::Rect& box);

private:
    static constexpr int   MIN_POINTS = 8;
    static constexpr float FB_MAX_ERR = 1.5f;

    std::vector<cv::Point2f> detect(const cv::Mat& gray,
                                    const cv::Rect& box) const;

    cv::Mat                  prevGray_;
    std::vector<cv::Point2f> pts_;
    cv::Rect                 box_;
};

// Click-to-lock tracker with three layers of robustness:
//
//  1. Primary backend (CSRT / KCF / Optical Flow) — tracks every frame.
//  2. Kalman filter [x, y, vx, vy] — smooths position, predicts through
//     brief occlusion (coast), and scales the re-detection search radius
//     with target velocity.
//  3. Template re-detection — when the primary tracker fails, searches a
//     Kalman-predicted region using normalised cross-correlation and
//     re-initialises the primary tracker on a match.
//
// This is the C++ equivalent of the Python hybrid tracker in core.py,
// running without DINOv2 — lightweight enough for a single Pi 5 CPU core.
class LockOnTracker {
public:
    static constexpr int   LOSS_TIMEOUT       = 15;    // frames → LOST
    static constexpr float REACQUIRE_THRESH   = 0.45f; // NCC threshold
    static constexpr int   TMPL_UPDATE_FRAMES = 30;    // refresh template

    bool init(const cv::Mat& frame, cv::Point center,
              Backend backend, int boxSize);
    void update(const cv::Mat& frame);
    void reset();

    bool            hasTarget()   const { return hasTarget_; }
    bool            locked()      const { return locked_; }
    bool            coasting()    const { return lossFrames_ > 0 && locked_; }
    const cv::Rect& bbox()        const { return bbox_; }
    long            age()         const { return age_; }
    int             lossFrames()  const { return lossFrames_; }
    int             totalLosses() const { return totalLosses_; }
    float           confidence()  const { return confidence_; }

    // Projected centre N frames ahead (for motion-vector arrow).
    cv::Point2f     projected(float steps = 8.f) const;

private:
    void saveTemplate(const cv::Mat& frame, const cv::Rect& box);
    void maybeUpdateTemplate(const cv::Mat& frame, const cv::Rect& box);
    cv::Rect templateSearch(const cv::Mat& frame, float& outScore) const;
    float    computeNCC(const cv::Mat& frame, const cv::Rect& box) const;
    bool     reinitBackend(const cv::Mat& frame, const cv::Rect& box);

    float searchRadius() const;

    cv::Ptr<cv::Tracker>           cvTracker_;
    std::unique_ptr<LKFlowTracker> lkTracker_;
    Backend                        backend_      = Backend::CSRT;
    int                            boxSize_      = 80;

    KalmanCenter kalman_;
    cv::Mat      tmpl_;         // grayscale template for re-detection
    int          tmplAge_     = 0;

    cv::Rect bbox_;
    bool     hasTarget_   = false;
    bool     locked_      = false;
    float    confidence_  = 0.f;
    long     age_         = 0;
    int      lossFrames_  = 0;
    int      totalLosses_ = 0;
};
