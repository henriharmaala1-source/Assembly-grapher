#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>

#include "kalman_center.hpp"

enum class DepthBackend { DEPTH_ANYTHING_V2, MIDAS_SMALL };

const char* depth_backend_name(DepthBackend b);

// Monocular depth-based traverse-direction analyser.
//
// Runs a lightweight depth model (Depth Anything v2 Small or MiDaS Small) via
// OpenCV DNN (CPU), then computes the best direction of traverse using
// navigable-corridor scoring rather than a naive furthest-point search:
//
//   1. Clearance field  = Gaussian-blur(depth, kernel ~ vehicle width).
//      An isolated far pixel (needle gap between branches) is blurred away;
//      only a far region with open margin around it survives.
//   2. Forward bias     = radial falloff from image centre, so straight-ahead
//      wins when two corridors tie (less control effort, no flip-flopping).
//   3. Kalman smoothing = the peak is filtered through KalmanCenter so the
//      steer arrow glides and coasts through a single noisy frame.
//
// A 3×3 sector grid is still computed for the optional heat overlay.
class DepthNav {
public:
    struct SectorMap {
        static constexpr int ROWS = 3, COLS = 3;
        float scores[ROWS][COLS]{};  // 0 = close / blocked, 1 = far / open
        int   bestRow = 1, bestCol = 1;
        bool  valid   = false;
    };

    // Best direction of traverse, in frame pixel coordinates.
    struct Traverse {
        cv::Point2f point;          // smoothed steer target (Kalman)
        cv::Point2f raw;            // unsmoothed peak (debug)
        float       openness = 0.f; // clearance value at the peak [0,1]
        float       margin   = 0.f; // peak − mean steer; how decisive [0,1]
        bool        valid    = false;
    };

    bool init(const std::string& modelPath, DepthBackend backend);
    bool isReady() const { return !net_.empty(); }

    bool update(const cv::Mat& frame);

    const SectorMap& sectors()  const { return sectors_; }
    const Traverse&  traverse() const { return traverse_; }
    const cv::Mat&   depthMap() const { return depthMap_; }

    // Draw heat grid + smoothed corridor arrow onto frame (in-place).
    void drawOverlay(cv::Mat& frame) const;

private:
    // Corridor scoring is done on a small working map for speed.
    static constexpr int   WORK_W   = 96;
    static constexpr int   WORK_H   = 72;
    static constexpr float BIAS_K   = 0.40f;  // forward-bias strength
    static constexpr float MIN_MARGIN = 0.04f; // below this → "scanning"

    cv::dnn::Net net_;
    DepthBackend backend_     = DepthBackend::MIDAS_SMALL;
    bool         invertDepth_ = false;
    cv::Size     inputSize_;

    cv::Mat      depthMap_;     // normalised [0,1] float32, frame-sized
    cv::Mat      bias_;         // cached radial forward-bias (WORK_W×WORK_H)
    SectorMap    sectors_;
    Traverse     traverse_;
    KalmanCenter steerKalman_;  // temporal smoothing of the steer target

    cv::Mat preprocess(const cv::Mat& frame) const;
    void    computeSectors(const cv::Size& frameSize);
    void    computeTraverse(const cv::Size& frameSize);
    void    buildBias(const cv::Size& sz);
};
