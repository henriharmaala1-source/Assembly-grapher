#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"

namespace sim {

// METRIC VELOCITY FROM SPARSE FLOW x DEPTH.
//
// WHAT THE CAMERA DOES AND DOES NOT DO. The D435i does NOT compute optical
// flow -- its ASIC does stereo matching and nothing else. What it gives is the
// two ingredients in one hardware-timestamped frame: a GLOBAL-SHUTTER IR image
// to compute flow from, and the depth to scale it with. The flow itself costs
// Pi CPU and is this file's job.
//
// WHY IT IS WORTH THE CPU. Inertial position error grows as t^2, because a
// tilt error is a constant acceleration bias that integrates twice. A velocity
// measurement bounds the first integral and makes it LINEAR. Measured in
// voxel_sim: mean drift 1.417 m -> 0.343 m, worst 9.53 -> 1.38 m. And 5 % vs
// 15 % flow error changed it by 22 %, against a 4x gap to having none -- so
// this needs to EXIST far more than it needs to be good.
//
// WHY IR AND NOT RGB. The IR pair is global shutter; the RGB is rolling
// shutter, which smears under motion and is poison for matching. The IR is also
// already registered with the depth, so no alignment step is needed.
//
// THE TWO THINGS THAT MUST HAPPEN IN ORDER:
//
//   1. DE-ROTATE FIRST. During a turn, rotational flow dwarfs translational
//      flow -- a 100 deg/s yaw at 450 px focal length moves the whole image
//      ~78 px per 0.1 s frame, while 1 m/s at 5 m moves it ~9 px. Read
//      undivided, a turn looks like enormous sideways translation. Attitude
//      comes from the FC, which is drift-free in roll and pitch.
//
//   2. USE NEAR PIXELS. Parallax IS the signal: translational flow scales as
//      1/Z, so a point at 20 m carries a twentieth the information of one at
//      1 m and the worst depth to scale it with.
//
// THE SOLVE. After de-rotation each tracked point gives two linear equations in
// the translation t = (tx,ty,tz), with its own depth as the scale:
//
//     du = (-f/Z) tx            + (u/Z) tz
//     dv =            (-f/Z) ty + (v/Z) tz
//
// which is a 3x3 least squares over all points. Forward motion is recovered
// from image DIVERGENCE (the u/Z, v/Z terms) and is the weakest axis, exactly
// as it is for any forward-looking camera -- a point dead ahead has no parallax
// at all. That is a property of the geometry, not of this code, and the
// returned condition number says when it bites.
struct FlowVelocityParams {
    int   gridX      = 12, gridY = 9;   // sample points across the frame
    int   patch      = 4;               // half-size of the SSD match patch
    int   search     = 10;              // half-size of the search window, px
    float minVar     = 25.f;            // reject flat patches: no texture, no match
    // Backward re-match window. Raised to at least `search` internally: a
    // narrower one guarantees the round trip hits its boundary and every
    // point is discarded, which looks exactly like "no texture".
    int   fbSearch   = 3;
    float fbMaxErrPx = 1.5f;            // and how far the round trip may miss
    float minRangeM  = 0.3f;            // depth band worth using -- parallax lives
    float maxRangeM  = 8.0f;            // near, and far points only add noise
    int   minPoints  = 8;               // below this the solve is not attempted
    float maxCondition = 5.0e3f;        // above this the geometry is degenerate
};

struct FlowVelocity {
    // Metric velocity in the CAMERA frame: +x right, +y down, +z forward.
    float vx = 0.f, vy = 0.f, vz = 0.f;
    int   points = 0;         // how many survived texture, FB and depth gating
    float condition = 0.f;    // of the 3x3 normal matrix; high = degenerate
    bool  valid = false;      // all gates passed -- consumers MUST check this
};

class FlowVelocityEstimator {
public:
    void init(const FlowVelocityParams& p) { p_ = p; }
    const FlowVelocityParams& params() const { return p_; }

    // `prevIr`/`curIr` are CV_8U, `depth` CV_32F metres aligned to them (which
    // the IR stream is, natively). `dYaw/dPitch/dRoll` are the ROTATION BETWEEN
    // THE TWO FRAMES in degrees, from the FC. `dt` seconds.
    FlowVelocity estimate(const cv::Mat& prevIr, const cv::Mat& curIr,
                          const cv::Mat& depth, const DepthCamera& cam,
                          float dYawDeg, float dPitchDeg, float dRollDeg,
                          float dt) const;

private:
    FlowVelocityParams p_;
};

}  // namespace sim
