#pragma once

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <memory>
#include <vector>

// Backends mirror the Python app's drone mode (tracker/drone.py):
// CSRT  — best accuracy, ~30-60 fps on Pi 5 at 640x480
// KCF   — fastest, weaker on scale change and occlusion
// FLOW  — sparse Lucas-Kanade, most resilient to partial occlusion
enum class Backend { CSRT, KCF, FLOW };

const char* backend_name(Backend b);

// Sparse Lucas-Kanade tracker with forward-backward validation and
// median point-spread scale estimation. Port of OpticalFlowTracker
// in tracker/trackers.py.
class LKFlowTracker {
public:
    bool init(const cv::Mat& frame, const cv::Rect& box);
    bool update(const cv::Mat& frame, cv::Rect& box);

private:
    static constexpr int   MIN_POINTS = 8;
    static constexpr float FB_MAX_ERR = 1.5f;  // px, forward-backward gate

    std::vector<cv::Point2f> detect(const cv::Mat& gray, const cv::Rect& box) const;

    cv::Mat                  prevGray_;
    std::vector<cv::Point2f> pts_;
    cv::Rect                 box_;
};

// Click-to-lock fixed-box tracker with loss accounting.
// Port of DroneTracker in tracker/drone.py: a single click designates a
// fixed-size box, the backend tracks it every frame, and loss statistics
// (age, consecutive loss frames, total losses) are kept for reliability
// measurement.
class LockOnTracker {
public:
    static constexpr int LOSS_TIMEOUT = 15;  // frames before declaring LOST

    bool init(const cv::Mat& frame, cv::Point center, Backend backend, int boxSize);
    void update(const cv::Mat& frame);
    void reset();

    bool            hasTarget()   const { return hasTarget_; }
    bool            locked()      const { return locked_; }
    const cv::Rect& bbox()        const { return bbox_; }
    long            age()         const { return age_; }
    int             lossFrames()  const { return lossFrames_; }
    int             totalLosses() const { return totalLosses_; }

private:
    cv::Ptr<cv::Tracker>          cvTracker_;
    std::unique_ptr<LKFlowTracker> lkTracker_;

    cv::Rect bbox_;
    bool     hasTarget_   = false;
    bool     locked_      = false;
    long     age_         = 0;  // frames tracked since lock
    int      lossFrames_  = 0;  // consecutive failed updates
    int      totalLosses_ = 0;  // loss episodes survived (re-acquisitions)
};
