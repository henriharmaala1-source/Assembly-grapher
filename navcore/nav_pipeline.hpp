#pragma once
// ---------------------------------------------------------------------------
// NavPipeline -- depth in, a flyable direction and speed out. ONE copy.
//
// This is voxel_live's navigation loop with everything that is not navigation
// taken out: no windows, no audit, no recording, no near-layer that the planner
// never reads. What is left is the part that flies -- integrate the frame into
// the voxel map, update the far-field bearing field, and ask the primitive
// planner for the best admissible motion under the swept-volume veto.
//
// WHY IT EXISTS AS A CLASS. nav-sim's front ends each write this loop out
// themselves, and every copy re-decides the map configuration, the order of
// operations and which map feeds the veto -- which is how voxel_live came to
// integrate a 0.10 m near map on every frame that its planner never once
// queried. The aircraft gets this instead of another copy, and
// fineMapParams() is where the configuration formulas live, once, for
// voxel_live and the aircraft alike.
//
// WHAT THE AIRCRAFT ADDS: straightFreeM(). onboard's mission flies straight
// legs, not primitives, and certifying those found two ways a leg could be
// "clear" in the map and not in the world. Both are written up where they
// are fixed, in nav_pipeline.cpp and NavPipelineParams::legCoreM.
//
// THE POSE CONTRACT, and it is the whole design. A world-anchored map needs to
// know where each frame was taken from. Onboard v1 is architecture C --
// nobody estimates position (see onboard/include/world_model.hpp) -- so there
// is no translation to give it. That is honest ONLY while the aircraft is
// still, which is exactly when onboard's move-stop-sense mission builds its
// map: SETTLE and THINK hold a position-hold hover so the deliberative tier has
// a stable vantage. So the caller feeds a FIXED origin plus the attitude it
// has, and must reset() whenever the vantage changes. Integrating across
// translation with a fixed origin produces a map that looks plausible and is
// wrong -- the failure frame_source.hpp and voxel_live both warn about.
// ---------------------------------------------------------------------------

#include <memory>

#include <opencv2/core.hpp>

#include "bearing_field.hpp"
#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_traj.hpp"

namespace sim {

struct NavPipelineParams {
    // voxel_live's defaults. They are the configuration the swept-volume and
    // map measurements in docs/ were taken with, so they are the ones to fly.
    float cell      = 0.25f;  // fine map; the ONLY map the veto reads
    int   stride    = 2;      // every 2nd pixel into the fine map
    float robotR    = 0.6f;
    float vMax      = 1.5f;   // voxel_live's, not rl_env's 3.0 -- a D435i's
                              // confident range bounds how fast it may fly
    float farRangeM = 20.f;   // bearing field: scores directions, never vetoes
    float fillFrac  = 0.25f;
    // THE VETO'S ATTITUDE TO UNKNOWN SPACE. 0 lets a primitive sweep through
    // air nothing has measured. Measured on nav-sim's maze: 0.45 took pooled
    // collisions from 12 in 4357 m to 1 in 5125 m at no travel cost. Left at 0
    // here to match voxel_live exactly; see TrajParams::coreFrac.
    float coreFrac  = 0.f;
    // 0 = openness only, no preferred direction. voxel_live's dirMode 0.
    float goalWeight = 0.f;
    // THE AIRFRAME'S OWN VOLUME, marked FREE at every reset. Not a guess: the
    // aircraft is occupying it. Without it a map started in a hover has an
    // unobserved hole exactly where the aircraft is -- behind the minimum
    // range and outside the FoV -- and with coreFrac > 0 every primitive then
    // fails at its FIRST point. 0 = off, which is what voxel_live does.
    // The PHYSICAL radius, not robotR: robotR includes a margin the aircraft
    // does not occupy, and seeding that would certify air nobody looked at.
    float seedBodyM = 0.f;
    // CONFIRMED-FREE CORE of a straight leg, horizontal, metres. Every map
    // cell in the aircraft's own altitude layer within this distance of the
    // leg's centre line must be FREE -- not merely not-OCCUPIED. 0 = off.
    //
    // WHY, measured. With only the planner's test (coreFrac 0: unknown
    // passes), test_voxel_nav's closed loop under simulated stereo walked the
    // aircraft along a wall it never saw: at grazing incidence the matcher
    // returns nothing, the wall stays UNKNOWN, and each leg certified a
    // little closer -- 0.47, 0.29, 0.07, 0.03 m. Unknown is not free.
    //
    // WHY HORIZONTAL AND NOT A BALL. A forward camera with a 56 deg vertical
    // FoV never observes the cells directly above and below the first half-
    // metre of any leg, so a 3-D core rejects every leg at its first sample
    // (measured: coreFrac 0.45 flew 0 m). The airframe's own layer IS
    // observable, and a quad is flat.
    float legCoreM = 0.f;
};

// THE MAP CONFIGURATION, ONE COPY. Every number is derived from the camera
// that is actually producing the frames rather than typed in:
//   maxIntegM   honest marking range, sqrt(cell * f * B / sigma) -- the range
//               past which depth error exceeds a cell and carving would cut
//               through real obstacles
//   minIntegM   Intel's own minimum range, f * B / 126, with 1.2x margin
//   depthSigCoef stereo error growth, Z^2 * sigma / (f * B)
VoxelMapParams fineMapParams(const DepthCamera& cam, float cell, int stride);

class NavPipeline {
public:
    // The camera must outlive the pipeline: it is the model the rays are
    // carved along, and a FrameSource owns it.
    void init(const DepthCamera& cam, const NavPipelineParams& p,
              const CamPose& origin);

    // One frame. `depthM` is RANGE ALONG THE RAY in metres, <= 0 invalid --
    // the convention every FrameSource delivers (see DepthCamera::rangePerZ).
    GeneralResult step(const cv::Mat& depthM, const CamPose& pose);

    // A NEW VANTAGE. Forget the map and start again at `origin`. The caller
    // decides when; see the pose contract above.
    void reset(const CamPose& origin);

    // CONFIRMED-FREE LENGTH OF A STRAIGHT, LEVEL LEG from `from` on compass
    // bearing `azDeg`, up to `maxM`. The same two tests the planner's rollouts
    // pass -- the body-sized ball clears (sphereClear, same robotR and
    // coreFrac) and the centre is CONFIRMED free -- stopping at the first
    // sample that fails either.
    //
    // WHY THE PLANNER'S OWN freeM IS NOT ENOUGH for a move-stop-sense leg. It
    // is measured along a primitive -- a curved, possibly climbing rollout --
    // and a waypoint leg flies the STRAIGHT line to a point on that bearing.
    // Where the primitive curves around a trunk, the straight line goes
    // through it. So the planner chooses the direction and this certifies the
    // geometry that will actually be flown.
    //
    // Capped at maxIntegM - robotR: past the marking range an obstacle can
    // never become OCCUPIED, so a clear ball there proves nothing.
    float straightFreeM(const CamPose& from, float azDeg, float maxM) const;
    // Is every cell of the altitude layer within `r` of (x,y) confirmed FREE?
    // Cells within `r` of (sx,sy) -- the footprint the airframe is standing
    // in at the start of the leg -- are exempt: the aircraft is there, so it
    // is not in collision with them, and a forward camera cannot see beside
    // itself. Everything the leg sweeps INTO must have been seen.
    bool coreFree(float x, float y, float z, float r, float sx, float sy) const;

    bool ready()  const { return cam_ != nullptr; }
    int  frames() const { return frames_; }
    const VoxelMap&          map()     const { return map_; }
    const VoxelMapParams&    mapParams() const { return mp_; }
    const NavPipelineParams& params()  const { return p_; }

private:
    const DepthCamera* cam_ = nullptr;
    NavPipelineParams  p_;
    VoxelMapParams     mp_;
    VoxelMap           map_;
    BearingField       bfield_;
    std::unique_ptr<TrajectoryPlanner> traj_;
    int frames_ = 0;
};

}  // namespace sim
