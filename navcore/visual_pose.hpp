#pragma once
// ---------------------------------------------------------------------------
// VisualPose -- where the camera is, from the camera. ONE implementation for
// the aircraft (onboard's VoxelNavModule) and the desk (voxel_live, the demo).
//
//   Fixed  no position estimate: the map is built where the camera stands
//          (architecture C -- valid only while it is not translating)
//   Vio    navcore's DepthVio on the left IR image + depth (vio.hpp)
//   Slam   ORB-SLAM3 in its own process, kestrel-orbslam, over a local socket
//          (slam_link.hpp); the stereo IR pair goes out, poses come back
//
// Both estimators need IR the PROJECTOR IS NOT IN: its dots are fixed to the
// camera and track as zero motion. EmitterMode says what the source does
// (Strobe: every other frame lit), and DarkFrameGate picks the dark frames
// from the IMAGE -- librealsense's metadata polarity is not trustworthy.
//
// Output is local ENU: metres from where tracking started, heading clockwise
// from North, with North taken from `headingDeg` at the first tracked frame
// (the FC compass aboard; the IMU's yaw on the desk). SLAM poses are anchored
// into that frame (slam_anchor.hpp) and re-anchored on every new map or
// implausible jump, each counted in `resets` -- the discontinuity counter an
// EKF (VISION_POSITION_ESTIMATE reset_counter) needs.
// ---------------------------------------------------------------------------

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "emitter_gate.hpp"
#include "frame_source.hpp"
#include "imu_sync.hpp"
#include "slam_anchor.hpp"
#include "slam_client.hpp"
#include "vio.hpp"

namespace sim {

enum class PoseMode { Fixed, Vio, Slam };
enum class EmitterMode { On, Off, Strobe };

const char* poseModeName(PoseMode m);          // "fixed" / "vio" / "slam"
bool        parsePoseMode(const std::string& s, PoseMode& out);

struct VisualPoseParams {
    PoseMode    mode = PoseMode::Fixed;
    EmitterMode emitter = EmitterMode::Strobe;
    VioParams   vio;
    std::string slamSocket = "/tmp/kestrel-slam.sock";
    bool        slamInertial = false;
    float       fps = 15.f;                    // told to the SLAM for its settings
};

struct PoseEstimate {
    bool   fresh = false;      // this step produced a new estimate (publish it)
    bool   valid = false;      // ...and it is a measurement, not a hold
    float  e = 0, n = 0, u = 0, yawDeg = 0;    // local ENU, camera
    float  ve = 0, vn = 0;                     // m/s, smoothed
    int    tracked = 0;        // corners / map points behind it
    int    resets = 0;         // discontinuities so far
    int    lost = 0;           // invalid estimates so far
    bool   dotFree = true;     // this frame reached (or could reach) the tracker
    bool   linkUp = true;      // SLAM: bridge connected
    double frameTimeS = -1.0;  // the time (step's nowS) of the frame this
                               // estimate describes: SLAM answers LATER, and
                               // nowS - frameTimeS is that latency
    std::string status;        // one line for a HUD
};

class VisualPose {
public:
    // `src` must outlive this. Nothing is started for Fixed.
    void init(const FrameSource& src, const VisualPoseParams& p);
    PoseMode mode() const { return p_.mode; }
    const VisualPoseParams& params() const { return p_; }

    // Once per frame, after src.next(). `attitude`: this frame's roll and
    // pitch (trusted) and yaw (used only frame to frame -- any zero).
    // `headingDeg`: true heading, fixes North when tracking starts.
    const PoseEstimate& step(FrameSource& src, const cv::Mat& depth,
                             const CamPose& attitude, float headingDeg, double nowS);
    const PoseEstimate& last() const { return est_; }
    // Has the estimator produced its FIRST pose? A SLAM loading its
    // vocabulary takes seconds; a simulation should hold the camera still
    // until then, as an aircraft stays on the ground (voxel_live does).
    bool ready() const { return p_.mode == PoseMode::Fixed || haveFirst_; }

private:
    void stepVio(FrameSource& src, const cv::Mat& depth, const CamPose& att,
                 float heading, double nowS);
    void stepSlam(FrameSource& src, const CamPose& att, float heading, double nowS);
    void publish(bool valid, float e, float n, float u, float yaw, int tracked,
                 int resets, double nowS, double frameS);

    const FrameSource* src_ = nullptr;
    VisualPoseParams p_;
    PoseEstimate est_;
    DarkFrameGate gate_;
    // VIO
    DepthVio vio_;
    // SLAM
    std::unique_ptr<SlamClient> slam_;
    ImuSync imuSync_;
    std::vector<slamlink::ImuSample> slamImu_;
    struct Ctx { CamPose att; float heading; double tS; };
    std::map<uint32_t, Ctx> ctx_;
    SlamAnchor anchor_;
    int32_t  slamMap_ = -1;
    uint32_t slamChanges_ = 0;
    int      resets_ = 0;
    bool     haveLast_ = false;
    float    lastE_ = 0, lastN_ = 0, lastU_ = 0, lastYaw_ = 0;
    double   lastSlamS_ = 0.0;
    bool     warnedStereo_ = false;
    bool     haveFirst_ = false;
    // velocity
    double   prevT_ = -1.0;
    float    prevE_ = 0, prevN_ = 0;
};

}  // namespace sim
