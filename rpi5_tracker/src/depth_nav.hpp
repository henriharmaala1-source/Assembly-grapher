#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>

enum class DepthBackend { DEPTH_ANYTHING_V2, MIDAS_SMALL };

const char* depth_backend_name(DepthBackend b);

// Monocular depth-based free-space analyser.
//
// Runs a lightweight depth estimation model (Depth Anything v2 Small or
// MiDaS Small) via OpenCV DNN (CPU), divides the frame into a 3×3 sector
// grid, and reports which sector is most open (furthest average depth).
// Designed to run every N frames alongside the lock-on tracker so the Pi 5
// CPU stays available for tracking on other frames.
class DepthNav {
public:
    struct SectorMap {
        static constexpr int ROWS = 3, COLS = 3;
        float scores[ROWS][COLS]{};  // 0 = close / blocked, 1 = far / open
        int   bestRow = 1, bestCol = 1;
        bool  valid   = false;
    };

    // Load the ONNX model. Returns false on failure (bad path, corrupt model).
    bool init(const std::string& modelPath, DepthBackend backend);
    bool isReady() const { return !net_.empty(); }

    // Run inference on frame. Returns false if not initialised.
    bool update(const cv::Mat& frame);

    const SectorMap& sectors()  const { return sectors_; }
    const cv::Mat&   depthMap() const { return depthMap_; }

    // Draw coloured sector grid + best-direction arrow onto frame (in-place).
    void drawOverlay(cv::Mat& frame) const;

private:
    cv::dnn::Net net_;
    DepthBackend backend_     = DepthBackend::MIDAS_SMALL;
    bool         invertDepth_ = false;  // MiDaS: closer = higher → invert
    cv::Size     inputSize_;

    cv::Mat   depthMap_;   // normalised [0,1] float32, frame-sized
    SectorMap sectors_;

    cv::Mat preprocess(const cv::Mat& frame) const;
    void    computeSectors(const cv::Size& frameSize);
};
