#pragma once
// ---------------------------------------------------------------------------
// DepthVio -- visual-inertial odometry for a depth camera with an IMU.
//
// WHY THIS AND NOT OpenVINS / Basalt. The D435i hands us, every frame, a
// GLOBAL-SHUTTER IR image AND a depth map already registered to it (the D4
// computes depth in the left imager's frame). So every tracked corner has a
// METRIC 3D position the moment it is seen: no triangulation, no scale to
// estimate, no stereo matching on the Pi. What is left is a 3D->2D pose solve,
// which is small. The IMU contributes what it is good at -- roll and pitch
// from gravity, drift-free -- and a rotation prior; the camera contributes
// what the IMU is bad at: yaw and translation. Loosely coupled, on purpose:
// the tightly coupled systems are where the D435i-specific calibration pain
// lives (IMU time offset, noise, the "fails when stationary" reports), and
// this project cannot debug a filter it cannot see into.
//
// THE PIPELINE, per frame:
//   0. High-pass both images: lighting that moves with the camera is not
//      texture, and a tracker that sees it reports zero motion.
//   1. Track the previous frame's corners into this one, each search started
//      where the prior pose predicts the corner will be: pyramidal
//      Lucas-Kanade, then track BACK and keep only points that return within
//      fbMaxPx -- the forward-backward check that throws out occlusions and
//      aliased matches before they can vote.
//   2. Solve yaw and position from the tracked corners' WORLD points, with
//      roll and pitch held at the IMU's: 4 DOF, robust Gauss-Newton from the
//      IMU prior. Tilt error is what turns into position drift, and the IMU
//      knows tilt to a fraction of a degree forever; a camera re-deriving it
//      every keyframe does not.
//   3. Gate: a solve that jumps or turns further than a frame allows is LOST,
//      not reported -- confidently wrong is worse than silent.
//   4. New KEYFRAME when the track thins or the camera has moved or turned far
//      enough: surviving points keep their world coordinates, fresh corners are
//      added from this frame's depth at this frame's pose. Error accumulates
//      once per keyframe, not once per frame -- that is the whole difference
//      between keyframe odometry and integrating frame-to-frame velocity.
//
// WHAT IT DOES NOT DO, stated because each is a real limit:
//   - no loop closure: drift is unbounded over distance, like every odometer
//   - FRAME RATE BOUNDS SPEED. LK matches translation only; a near corner
//     that grows or warps much between frames stops matching. On the raw
//     image, LK started at the TRUE position kept 17 of 139 corners within
//     2 px at 0.2 m per frame. With the prediction and the high-pass, 3 m/s
//     at 15 Hz (0.2 m/frame) holds ~1-2 % in test_vio; faster is untested.
//   - no IMU PREINTEGRATION: a frame with too few tracked corners is LOST and
//     the estimate COASTS on the last velocity, decaying -- a guess, reported
//     as invalid, not inertial dead reckoning (imu_odometry.hpp exists for
//     short bridges and is not wired in here)
//   - NO TEXTURE GATE of its own, on purpose. A corner is anchored only where
//     the depth is valid, and the D4 computes depth by matching THESE IR
//     images: on a dark frame, valid depth already means texture to track,
//     and a blank wall gives no depth, no world points, and LOST. Measured
//     in test_vio (stereo, blank wall): lost, never a wrong step. Two gates
//     that tried to add this (an absolute eigenvalue floor; a Shi-Tomasi
//     test normalised by the image's noise) starved the straight-line case
//     at range before they caught anything the depth had not.
//   - the IR PROJECTOR MUST NOT BE IN THE IMAGE IT TRACKS. The emitter's dots
//     are fixed to the camera, so they track as ZERO motion and outvote the
//     world. Emitter off (outdoors, in texture) or strobed on alternate frames
//     (RS2_OPTION_EMITTER_ON_OFF) with VIO on the dark frames. The sim's IR
//     render has no dot pattern and cannot show this failure.
// ---------------------------------------------------------------------------

#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"

namespace sim {

struct VioParams {
    int    maxFeatures = 250;
    double quality     = 0.01;   // goodFeaturesToTrack quality level
    double minDistPx   = 8.0;    // between corners
    // Depth band a corner must fall in to anchor a world point. The far end is
    // where stereo range error starts to dominate the reprojection error; the
    // pose solve tolerates more of it than the voxel map does, but not
    // without limit.
    float  minDepthM   = 0.3f;
    float  maxDepthM   = 6.0f;
    int    highPassPx  = 25;     // box size of the lighting-removal prefilter
    float  highPassGain = 2.f;    // measured: 2 beats 1 on straight-line drift
    int    lkWin       = 21;
    int    lkLevels    = 3;
    float  fbMaxPx     = 1.0f;   // forward-backward round-trip tolerance
    float  ransacPx    = 2.0f;   // inlier gate on reprojection error
    // A solve is rejected (LOST) if it moves further than this from the last
    // pose in one frame, or turns further than this from the gyro's prior.
    float  maxJumpM    = 0.5f;
    float  maxYawDevDeg = 5.f;
    // The gyro's frame-to-frame yaw, as a weighted term in the solve (1-sigma,
    // degrees). Vision overrules it wherever the geometry is strong; where it
    // is weak -- few corners, all straight ahead -- yaw and sideways
    // translation trade against each other and the gyro breaks the tie.
    float  gyroYawSigmaDeg = 0.2f;
    // Constant-velocity motion model: the prior carries the last measured
    // per-frame velocity, and a LOST frame coasts on it, decaying by this
    // factor per frame, instead of stopping dead.
    float  coastDecay  = 0.85f;
    int    minInliers  = 20;     // below this the frame is LOST
    int    kfMinTracked = 80;    // re-anchor when the track thins below this
    float  kfDistM     = 0.6f;   // ...or the camera has moved this far
    float  kfYawDeg    = 15.f;   // ...or turned this far
};

struct VioResult {
    bool    valid = false;       // pose below is a measurement, not a hold
    CamPose pose;                // world ENU camera pose (roll/pitch from IMU)
    int     tracked = 0, inliers = 0;
    float   rmsPx  = 0.f;        // inlier reprojection RMS
    bool    newKeyframe = false;
    int     resets = 0;          // increments on every discontinuity
    float   ms     = 0.f;        // this step's cost
};

class DepthVio {
public:
    void init(const DepthCamera& cam, const VioParams& p = VioParams());

    // Start (or restart) at `start` -- position and full attitude. A restart
    // is a DISCONTINUITY and increments resets(), which is what ArduPilot's
    // ODOMETRY reset_counter wants to hear about. start.yawDeg sets the
    // heading of the VIO frame (e.g. the FC's compass); the IMU yaw passed to
    // step() may have ANY zero -- it is referenced at the first step after
    // this, and only its changes are used.
    void reset(const CamPose& start);

    // One frame. `ir8` CV_8U, `depthM` CV_32F RANGE ALONG THE RAY (the
    // FrameSource convention), both the same size and registered.
    // `attitude` is the IMU's roll, pitch and yaw for THIS frame: roll/pitch
    // are trusted, yaw only as a relative rotation prior between frames.
    VioResult step(const cv::Mat& ir8, const cv::Mat& depthM, const CamPose& attitude);

    bool started() const { return started_; }
    int  resets()  const { return resets_; }
    const CamPose& pose() const { return pose_; }

private:
    void makeKeyframe(const cv::Mat& ir8, const cv::Mat& depthM);

    const DepthCamera* cam_ = nullptr;
    VioParams p_;
    bool      started_ = false;
    int       resets_  = 0;
    CamPose   pose_;               // current estimate
    CamPose   kfPose_;             // pose at the current keyframe
    float     lastImuYaw_ = 0.f;
    bool      imuRef_ = false;       // lastImuYaw_ holds a real IMU reading
    float     vel_[3] = {0, 0, 0};   // per FRAME, ENU; frames assumed evenly spaced
    bool      lastValid_ = false;
    cv::Mat   prevImg_;
    std::vector<cv::Point2f> prevPts_;   // tracked corners, previous frame
    std::vector<cv::Point3f> worldPts_;  // their anchored world points
};

}  // namespace sim
