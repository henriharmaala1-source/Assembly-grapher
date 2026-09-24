#pragma once
// ---------------------------------------------------------------------------
// VoxelNavModule -- D435i stereo depth -> voxel map -> swept-volume plan.
//
// The navigation loop from nav-sim's voxel_live, aboard. It does not reimplement
// any of it: sim::NavPipeline (navcore/nav_pipeline.hpp) is the loop, built
// from navcore's map, bearing field and planner -- the same objects voxel_live
// runs, configured by the same fineMapParams(). A bug seen on the desk is a bug
// in the code that flies.
//
// WHAT IT PUBLISHES is the output contract onboard already has -- a bearing and
// a speed -- plus the two numbers that make it safe to act on: freeM, the
// contiguous CONFIRMED-FREE distance along that bearing (unknown space earns
// none), and `blocked`, which means no primitive survived the veto.
//
// THE POSE, and why this module is honest without odometry. Onboard is
// architecture C: nobody estimates position. A world-anchored map needs one,
// EXCEPT while the aircraft is not translating -- and move-stop-sense is built
// around exactly that. SETTLE and THINK hover in position hold; SCAN rotates in
// place. Rotation about a fixed point is fully described by attitude, which the
// module has (the D435i's own IMU for roll/pitch, the FC for heading). So:
//
//   still      -> integrate every frame into ONE map at a fixed origin
//   translating-> integrate nothing, publish nothing valid
//   still again-> reset() -- a new vantage is a new map
//
// "Still" is the mission in THINK or SCAN (not SETTLE: that is where the last
// leg's speed is still bleeding off), or, with no mission, the ground speed
// under stillSpeedMs -- which also vetoes in every case. The map this produces is never wrong
// by construction; it is only ever missing. That is the trade architecture C
// makes, and it is why THINK waits for minFrames before acting on it.
//
// THE D435i IS ITS OWN SENSOR. run() ignores the colour frame the scheduler
// passes; depth comes from this module's FrameSource, which blocks for the next
// depth frame. It therefore belongs on the Deliberator thread, never the fly
// loop -- the scheduler slot in main.cpp puts it there.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>

#include "frame_source.hpp"   // navcore
#include "nav_pipeline.hpp"   // navcore
#include "emitter_gate.hpp"   // navcore
#include "vio.hpp"            // navcore
#include "slam_anchor.hpp"
#include "slam_client.hpp"
#include "imu_sync.hpp"
#include <map>
#include "perception.hpp"

class VoxelNavModule : public IPerceptionModule {
public:
    struct Params {
        sim::NavPipelineParams nav;       // voxel_live's configuration
        float stillSpeedMs = 0.4f;        // above this the aircraft is moving
        float mountTiltDeg = 0.f;         // camera elevation vs airframe, + up
        // Roll/pitch from the D435i's IMU when it has settled, else from the FC.
        // The camera's own IMU measures the CAMERA, mount tilt included, so it
        // is the better source; the FC is the fallback, never the other way.
        bool  preferCameraImu = true;
        float legMaxM = 8.f;              // how far a straight leg is certified
        // The airframe. Its own volume is seeded FREE at each vantage (it is
        // standing in it), and a straight leg's horizontal core of this
        // radius must be CONFIRMED free, not merely unknown -- see
        // NavPipelineParams::legCoreM for the wall that made this necessary.
        float bodyR = 0.35f;
        // The leg search: bearings across the camera's view, every legStepDeg,
        // keeping legFovMarginDeg off each edge (the edge of the frame is the
        // least-observed air in it). legTieM is what 90 deg of departure from
        // the planner's bearing costs, in metres of leg -- a tie-breaker, not
        // a preference strong enough to buy a shorter certificate.
        float legStepDeg      = 3.f;
        float legFovMarginDeg = 8.f;
        float legTieM         = 0.25f;
        // THE FAR TIER CHOOSING AMONG SAFE LEGS -- OFF, BECAUSE IT MEASURED
        // WORSE. Two steps, near first:
        //   1. every bearing whose certified leg is within legKeepFrac of the
        //      longest one is a candidate -- the near map has no real
        //      preference between them (legs saturate at the marking range,
        //      so in open ground most of the fan ties)
        //   2. of those, the one the bearing field sees furthest along,
        //      capped at nav.farRangeM; ties to the planner's bearing
        // Unknown far bins earn 0, exactly as the planner's far term does:
        // outdoors the biggest region of "no return" is the sky. Safety is
        // untouched -- every candidate carries its own certificate, and the
        // far tier only reorders legs the near map already cleared.
        // false = the near tier alone (longest leg, ties to the planner).
        //
        // MEASURED, test_voxel_nav closed loop, 150 s, 4 worlds paired, the
        // 48 m field with dead-end walls (VOXTEST_BIG) at the 848x480 that
        // flies -- the scene this was meant to help in:
        //   perfect depth  net displacement -9.7 +/- 1.7 m (se), net x cells
        //                  -840 +/- 184; travel and coverage unchanged
        //   stereo         no resolved difference (travel -5.5 +/- 3.0 m)
        // and no resolved gain anywhere at 424x240 either, room or field.
        // Chasing the longest sight line from each stop ends the flight
        // closer to where it began; why is not established. Safety was the
        // same in both arms (every run >= 0.44 m from truth surfaces).
        bool  farChoose   = false;
        float legKeepFrac = 0.9f;

        // KEEP ONE MAP ACROSS STOPS, positioned by an ODOMETRY estimate
        // (WorldState estPe/estPn, altitude from vehAltM) instead of starting a
        // new map at every vantage. This is ARCHITECTURE B -- it needs an
        // ego-motion source (VIO, or optical flow into EKF3) -- and it exists
        // here so what that source would buy, and what its drift would cost,
        // can be MEASURED before anything is built. Off: architecture C.
        bool  persistMap     = false;
        // With persistMap, integrate during legs too, not only at stops.
        bool  integrateMoving = false;

        // VISUAL ODOMETRY on the D435i's own left IR image, which is
        // registered with depth by construction (navcore/vio.hpp). Runs on
        // every frame the emitter did NOT light -- see live()'s strobe -- and
        // publishes WorldState vio*; main uses it as the displacement source
        // when neither the Pi estimate nor the FC's flow position exists.
        // Off: nothing is computed and nothing is published.
        bool  vio = false;
        sim::VioParams vioParams;

        // EXTERNAL SLAM: ORB-SLAM3 in its own process (onboard/orbslam,
        // GPLv3, over slam_link.hpp). The module sends each dot-free STEREO
        // IR pair -- plus IMU for stereo-inertial -- and publishes the poses
        // it gets back as vio*, in ENU (slam_anchor.hpp), exactly where
        // DepthVio's go; so everything downstream (the displacement estimate,
        // the EKF3 feed) is unchanged. Takes precedence over `vio`.
        bool        slam = false;
        std::string slamSocket = "/tmp/kestrel-slam.sock";
        bool        slamInertial = false;

        // THE PROJECTOR. On: blank walls get depth, but no tracker can use
        // the IR. Off: every frame trackable, blank walls have no depth
        // (outdoors the sun swamps the dots anyway). Strobe: both, on
        // alternate frames, with DarkFrameGate picking the dark ones from the
        // image. live() uses Strobe when vio or slam is on, else On.
        enum class Emitter { On, Off, Strobe };
        Emitter emitter = Emitter::Strobe;
    };

    // Own a source. `src` null means "no camera": isReady() is false and the
    // scheduler never runs it -- the same contract every other module has.
    VoxelNavModule(std::unique_ptr<sim::FrameSource> src, const Params& p);

    // The D435i, through librealsense loaded at run time. Returns a module
    // that is not ready (and says why in `err`) when there is no camera.
    // With p.vio the emitter STROBES (alternate frames lit), so VIO has dark
    // frames: VIO then runs at half the depth rate -- ask for 30 fps.
    static std::unique_ptr<VoxelNavModule> live(const Params& p, int width,
                                                int height, int fps,
                                                std::string* err);

    const char* name()    const override { return "voxel-nav"; }
    float       costMs()  const override { return 40.f; }
    bool        isReady() const override { return src_ && src_->ok(); }
    void        run(const cv::Mat& frame, WorldModel& wm) override;

    const sim::NavPipeline& pipeline() const { return nav_; }
    const sim::FrameSource* source()   const { return src_.get(); }
    int resets() const { return resets_; }   // vantages started, for telemetry
    const sim::DepthVio& vio() const { return vio_; }

private:
    void runVio(const cv::Mat& depth, const sim::PoseHint& hint, const WorldState& s,
                WorldModel& wm);
    void runSlam(const sim::PoseHint& hint, const WorldState& s, WorldModel& wm);
    sim::CamPose attitudeFor(const sim::PoseHint& hint, const WorldState& s) const;
    void publishVisual(bool valid, float e, float n, float u, float yawDeg, int tracked,
                       int resets, WorldModel& wm);

    std::unique_ptr<sim::FrameSource> src_;
    Params            p_;
    sim::NavPipeline  nav_;
    bool              still_ = false;        // integrating into the current map
    bool              mapInit_ = false;      // persistMap: the one map exists
    int               resets_ = 0;
    sim::DepthVio     vio_;
    double            vioPrevT_ = -1.0;
    float             vioPrevE_ = 0.f, vioPrevN_ = 0.f, vioVe_ = 0.f, vioVn_ = 0.f;
    int               vioLost_ = 0;
    sim::DarkFrameGate gate_;
    bool              dotFree_ = true;       // this frame may be tracked
    // SLAM
    std::unique_ptr<SlamClient> slam_;
    ImuSync           imuSync_;
    std::vector<slamlink::ImuSample> slamImu_;
    struct SlamCtx { sim::CamPose att; float fcYaw; };
    std::map<uint32_t, SlamCtx> slamCtx_;   // seq -> attitude at capture
    SlamAnchor        anchor_;
    int32_t           slamMap_ = -1;
    uint32_t          slamChanges_ = 0;
    int               visResets_ = 0;
    bool              haveLast_ = false;
    float             lastE_ = 0, lastN_ = 0, lastU_ = 0, lastYaw_ = 0;
    bool              warnedStereo_ = false;
};
