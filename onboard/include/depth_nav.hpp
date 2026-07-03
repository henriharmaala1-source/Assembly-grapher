#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "kalman_center.hpp"

enum class DepthBackend { DEPTH_ANYTHING_V2, MIDAS_SMALL };

const char* depth_backend_name(DepthBackend b);

// Depth-based traverse-direction analyser.
//
// Depth comes from EITHER a lightweight monocular model (Depth Anything v2 Small
// or MiDaS Small) via OpenCV DNN (CPU) — `update(frame)` — OR a metric depth grid
// from a ToF/stereo sensor (VL53L5CX/VL53L9/OAK-D) — `updateFromGrid(...)`. Both
// feed the SAME navigable-corridor scoring; the ToF path is metric and needs no
// model or GPU. From the depth map it computes the best direction of traverse:
//
//   1. Clearance field  = Gaussian-blur(depth, kernel ~ vehicle width).
//      An isolated far pixel (needle gap between branches) is blurred away;
//      only a far region with open margin around it survives.
//   2. VFH+ steering    = collapse the clearance field to a 1-D polar openness
//      histogram over headings, threshold it to free/blocked with hysteresis,
//      widen blocked sectors by the vehicle half-width, then pick the free
//      heading nearest straight-ahead and the previous heading. The hysteresis
//      and previous-heading cost stop the steer flapping at open doorways
//      (the failure mode of a plain argmax-of-the-clearance-field).
//   3. Kalman smoothing = the chosen heading is filtered through KalmanCenter so
//      the steer arrow glides and coasts through a single noisy frame.
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
    void enableTof() { tofReady_ = true; } // depth from a ToF/stereo grid, no ONNX
    bool isReady() const { return !net_.empty() || tofReady_; }

    // Ego-motion de-rotation: give the FC's current roll/pitch (deg) and the
    // camera/sensor vertical FoV, and the corridor analysis is stabilised to
    // WORLD-LEVEL before VFH+ runs — so the drone banking or pitching doesn't
    // tilt the horizon band into ground/sky and doesn't masquerade as the scene
    // moving. Call once per frame before update()/updateFromGrid(); if never
    // called, de-rotation is simply off (identical to the un-stabilised path).
    void setAttitude(float rollDeg, float pitchDeg) {
        roll_ = rollDeg; pitch_ = pitchDeg; haveAtt_ = true;
    }
    void setVerticalFov(float deg) { vFovDeg_ = deg > 1.f ? deg : vFovDeg_; }

    bool update(const cv::Mat& frame);

    // Sensor-agnostic depth entry point: feed a metric depth grid (metres,
    // higher = farther, <= 0 = invalid/no-return) from a ToF/stereo sensor
    // instead of the ONNX model, and run the same sector + VFH+ pipeline. Any
    // grid size works — VL53L5CX 8×8, VL53L9 54×42, an OAK-D depth map.
    bool updateFromGrid(const cv::Mat& metricGrid, const cv::Size& frameSize,
                        float maxRangeM);

    const SectorMap& sectors()  const { return sectors_; }
    const Traverse&  traverse() const { return traverse_; }
    const cv::Mat&   depthMap() const { return depthMap_; }

    // Horizontal FoV the openness histogram spans (for the occupancy grid scan).
    void  setHFov(float deg) { if (deg > 1.f) hFovDeg_ = deg; }
    float hFovDeg() const { return hFovDeg_; }

    // Polar openness histogram → a metric-ish clearance scan for the P5b
    // occupancy grid. Each bin is openness×maxRange (metres). METRIC ONLY on the
    // ToF/stereo path (updateFromGrid sets a real maxRange); on the monocular
    // path it is a NOMINAL scale (relative openness × scanMaxNominalM) — good
    // enough to bias routing, not a true map, until a ranging sensor exists.
    const std::vector<float>& openHist() const { return openHist_; }  // [0,1] per col
    float scanMaxM() const { return scanMaxM_; }

    // Draw heat grid + smoothed corridor arrow onto frame (in-place).
    void drawOverlay(cv::Mat& frame) const;

private:
    // Corridor scoring is done on a small working map for speed.
    static constexpr int   WORK_W   = 96;
    static constexpr int   WORK_H   = 72;
    static constexpr float MIN_MARGIN = 0.04f; // below this → "scanning"
    // VFH+ steering: hysteresis thresholds (as a fraction of the frame's max
    // openness) and the cost weights toward straight-ahead and the previous
    // heading. The previous-heading term is what stops the steer oscillating.
    static constexpr float VFH_FREE_FRAC  = 0.75f;  // openness ≥ this·max → free
    static constexpr float VFH_BLOCK_FRAC = 0.55f;  // openness ≤ this·max → blocked
    static constexpr float VFH_W_FWD      = 1.0f;   // prefer straight ahead
    static constexpr float VFH_W_PREV     = 1.5f;   // prefer last heading (hysteresis)

    cv::dnn::Net net_;
    DepthBackend backend_     = DepthBackend::MIDAS_SMALL;
    bool         invertDepth_ = false;
    bool         tofReady_    = false;   // depth from a ToF/stereo grid, not ONNX
    cv::Size     inputSize_;

    // Ego-motion de-rotation state (attitude in degrees; off until setAttitude).
    float        roll_ = 0.f, pitch_ = 0.f;
    bool         haveAtt_  = false;
    float        vFovDeg_  = 50.f;       // camera/sensor vertical FoV (VL53L9 ≈ 42)

    float        hFovDeg_  = 60.f;       // camera/sensor horizontal FoV
    float        scanMaxM_ = 8.f;        // openness→metres scale (nominal on mono,
                                         // real maxRange on the ToF path)
    static constexpr float SCAN_MAX_NOMINAL_M = 8.f;   // mono nominal range

    cv::Mat      depthMap_;     // normalised [0,1] float32, frame-sized
    SectorMap    sectors_;
    Traverse     traverse_;
    std::vector<float>   openHist_;                // per-column openness [0,1] (the scan)
    KalmanCenter steerKalman_;  // temporal smoothing of the steer target
    std::vector<uint8_t> blocked_;                 // VFH+ histogram (hysteresis state)
    float                prevCol_ = WORK_W * 0.5f; // last chosen heading column

    cv::Mat preprocess(const cv::Mat& frame) const;
    void    computeSectors(const cv::Size& frameSize);
    void    computeTraverse(const cv::Size& frameSize);
};
