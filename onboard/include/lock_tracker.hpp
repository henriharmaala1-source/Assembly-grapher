#pragma once

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>   // cv::legacy::TrackerMOSSE
#include <memory>
#include <vector>

#include "kalman_center.hpp"

enum class Backend { CSRT, KCF, FLOW, MOSSE };
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
    // Cost guards on the re-detection search (it lives in the scheduler's
    // alwaysOn slot, which is unbudgeted, so it must bound itself).
    static constexpr float REACQUIRE_MAX_RADIUS   = 200.f; // px; caps the search ROI
    static constexpr int   REACQUIRE_COLD_INTERVAL = 5;    // retry every Nth frame once LOST
    static constexpr float COAST_DECAY            = 0.6f;  // velocity bleed-off while coasting

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

    cv::Ptr<cv::Tracker>           cvTracker_;       // CSRT / KCF
    cv::Ptr<cv::legacy::Tracker>   legacyTracker_;   // MOSSE (legacy API)
    std::unique_ptr<LKFlowTracker> lkTracker_;       // Optical Flow
    Backend                        backend_      = Backend::CSRT;
    int                            boxSize_      = 80;

    KalmanCenter kalman_;
    cv::Mat      tmpl_;         // grayscale template for re-detection
    // THE SIZE THE TEMPLATE WAS TAKEN FROM, which is not the template's own
    // size once the 96 px cap bites. Two consumers read tmpl_ and they had
    // opposite conventions: computeNCC resizes the candidate ROI to
    // tmpl_.size() and is therefore scale-normalised, while templateSearch slid
    // tmpl_ over the frame at NATIVE resolution and so assumed 1:1. For any box
    // over 96 px those disagree, and nothing reconciled them.
    //
    // Keeping the source dimensions makes the convention explicit and lets
    // templateSearch scale the search region to match, and return a box the
    // size of the TARGET rather than the size of the capped template.
    int          tmplBoxW_    = 0, tmplBoxH_ = 0;
    int          tmplAge_     = 0;

    cv::Rect bbox_;
    bool     hasTarget_   = false;
    bool     locked_      = false;
    float    confidence_  = 0.f;
    long     age_         = 0;
    int      lossFrames_  = 0;
    int      totalLosses_ = 0;
};
