#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "depth_record.hpp"
#include "voxel_world.hpp"

// ---------------------------------------------------------------------------
// Where depth frames come from. One seam, three implementations, and the point
// is that everything downstream cannot tell them apart.
//
// The pipeline in voxel_gui/voxel_sim is four lines:
//
//     cv::Mat d = cam.renderStereo(world, pose);     <- ONLY this is synthetic
//     map.integrate(d, cam, pose);
//     map.recentre(...);
//     planner.plan(map, ...);
//
// Everything after line 1 -- the three-state occupancy rules, the carve limits,
// the swept-volume test, the trajectory library -- is arithmetic that behaves
// identically on a desktop and on the aircraft. So the honest way to point this
// stack at a real camera is not to write a second stack: it is to replace line
// 1 and change nothing else. Then a bug seen on real data is a bug in the code
// that would fly, not in a demo built alongside it.
//
// THE THREE SOURCES, and why each earns its place:
//
//   SIM       the existing raycaster. Ground truth is available, so it is the
//             only one that can say whether the MAP is wrong rather than just
//             surprising.
//   REPLAY    a .kdr recording. Deterministic, repeatable, needs no camera and
//             no aircraft -- and it is what turns a walk in a forest into a
//             measurement you can re-run after changing a threshold.
//   LIVE      librealsense. Compiled only when the SDK is present, so the tree
//             still builds for everyone else.
//
// POSE IS NOT SOLVED HERE, and pretending otherwise would be the whole
// project's worst possible bug. A depth frame without a pose cannot be
// integrated into a world-anchored map; the sim has perfect pose by
// construction and a real camera has none. Sources expose what they know
// (nothing, for a bare recording) and the CALLER decides -- fixed pose,
// IMU-derived attitude, or eventually odometry. See PoseHint.
// ---------------------------------------------------------------------------

namespace sim {

// What a source can say about where the camera was. `valid` false means the
// caller must supply a pose itself; it does NOT mean the origin.
struct PoseHint {
    bool  valid = false;
    bool  attitudeOnly = false;   // orientation known, translation is not
    CamPose pose;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual const char* name() const = 0;
    virtual bool ok() const = 0;
    virtual const CamParams& params() const = 0;

    // The camera model matching this source's frames -- the rays the mapper
    // must carve along. For SIM this is the camera that rendered them; for
    // REPLAY it is rebuilt from the recording's stored intrinsics; for LIVE
    // from the device's own.
    virtual const DepthCamera& camera() const = 0;

    // Next depth frame, CV_32F metres, <=0 invalid. `hint` receives whatever
    // the source knows about pose. Returns false at end of stream or on error.
    virtual bool next(cv::Mat& depth, PoseHint& hint) = 0;

    // Frames available, or -1 for an open-ended stream (live).
    virtual int  frameCount() const { return -1; }
    virtual int  index() const { return 0; }
    virtual bool seek(int) { return false; }
};

// --- synthetic ------------------------------------------------------------
// Wraps the raycaster. The pose is supplied by the caller each frame, because
// in the sim the vehicle state IS the pose and the source has no opinion.
class SimFrameSource : public FrameSource {
public:
    SimFrameSource(const VoxelWorld& w, const CamParams& p, bool truth)
        : w_(w), cam_(p), truth_(truth) {}

    const char* name() const override { return truth_ ? "sim-truth" : "sim-stereo"; }
    bool ok() const override { return true; }
    const CamParams& params() const override { return cam_.params(); }
    const DepthCamera& camera() const override { return cam_; }

    void setPose(const CamPose& p) { pose_ = p; }
    bool next(cv::Mat& depth, PoseHint& hint) override;

private:
    const VoxelWorld& w_;
    DepthCamera cam_;
    bool    truth_;
    CamPose pose_;
};

// --- replay ---------------------------------------------------------------
class ReplayFrameSource : public FrameSource {
public:
    bool open(const std::string& path, std::string* err = nullptr);

    const char* name() const override { return "replay"; }
    bool ok() const override { return rd_.isOpen(); }
    const CamParams& params() const override { return cam_->params(); }
    const DepthCamera& camera() const override { return *cam_; }

    bool next(cv::Mat& depth, PoseHint& hint) override;
    int  frameCount() const override { return rd_.frameCount(); }
    int  index() const override { return rd_.index(); }
    bool seek(int i) override { return rd_.readFrame(i, buf_); }

    const DepthRecordHeader& header() const { return rd_.header(); }

private:
    DepthRecordReader rd_;
    std::unique_ptr<DepthCamera> cam_;
    std::vector<float> buf_;
};

// --- live -----------------------------------------------------------------
// Declared unconditionally so callers do not need #ifdef; the factory returns
// null with a message when the SDK was not compiled in.
std::unique_ptr<FrameSource> makeLiveSource(int width, int height, int fps,
                                            bool emitter, std::string* err);

// Can a live camera be opened RIGHT NOW -- i.e. can librealsense be loaded.
// A runtime question, deliberately: this binary is built the same way whether
// or not the SDK is installed, so there is nothing to ask at compile time.
bool haveLiveSupport();

// Human-readable: the library version and path when available, and the paths
// that were tried when not.
std::string liveSupportDetail();

}  // namespace sim
