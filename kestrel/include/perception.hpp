#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <memory>
#include <string>
#include <vector>

#include "world_model.hpp"
#include "lock_tracker.hpp"   // rpi5_tracker/src
#include "depth_nav.hpp"      // rpi5_tracker/src

// A perception module reads the current frame and writes its findings into the
// WorldModel. The scheduler decides which modules run on a given tick based on
// their cost and the active behaviour, so on a CPU-only Pi 5 the heavy models
// never all fire on the same frame.
class IPerceptionModule {
public:
    virtual ~IPerceptionModule() = default;
    virtual const char* name()   const = 0;
    virtual float       costMs() const = 0;   // approx CPU cost per run
    virtual bool        isReady() const { return true; }
    virtual void        run(const cv::Mat& frame, WorldModel& wm) = 0;
};

// ------------------------------------------------------------------- track
// Wraps the rpi5_tracker LockOnTracker. Cheap enough to run every frame.
class TrackModule : public IPerceptionModule {
public:
    explicit TrackModule(Backend backend = Backend::MOSSE, int boxSize = 80);

    const char* name()   const override { return "track"; }
    float       costMs() const override { return 3.f; }
    void        run(const cv::Mat& frame, WorldModel& wm) override;

    void requestLock(cv::Point center);   // operator / LLM designates a target
    void setBackend(Backend b);
    void reset();

private:
    LockOnTracker trk_;
    Backend       backend_;
    int           boxSize_;
    bool          pendingLock_ = false;
    cv::Point     pendingPt_;
};

// ---------------------------------------------------------------- navigate
// Wraps DepthNav (monocular depth → navigable-corridor heading).
class NavigateModule : public IPerceptionModule {
public:
    NavigateModule(const std::string& model, DepthBackend backend);

    const char* name()   const override { return "navigate"; }
    float       costMs() const override { return ready_ ? 110.f : 0.f; }
    bool        isReady() const override { return ready_; }
    void        run(const cv::Mat& frame, WorldModel& wm) override;

    DepthNav& nav() { return nav_; }     // for the optional display overlay

private:
    DepthNav nav_;
    bool     ready_ = false;
};

// ------------------------------------------------------------------ detect
// YOLOv8-style detector (Drone-vs-Bird / MAV-VID) via OpenCV DNN on CPU.
class DetectModule : public IPerceptionModule {
public:
    DetectModule(const std::string& model, std::vector<std::string> labels,
                 float confThresh = 0.25f);

    const char* name()   const override { return "detect"; }
    float       costMs() const override { return ready_ ? 90.f : 0.f; }
    bool        isReady() const override { return ready_; }
    void        run(const cv::Mat& frame, WorldModel& wm) override;

private:
    cv::dnn::Net             net_;
    std::vector<std::string> labels_;
    float                    confThresh_;
    int                      inputSz_ = 640;
    bool                     ready_   = false;
};
