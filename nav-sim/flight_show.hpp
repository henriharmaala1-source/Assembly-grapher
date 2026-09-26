#pragma once
// ---------------------------------------------------------------------------
// FlightShow -- the AIRCRAFT'S OWN autonomy, flown in a simulated world.
//
// Not a model of it and not a copy: onboard's VoxelNavModule and
// MissionController, compiled from onboard/src into this binary, fed exactly
// what they get aboard --
//
//   depth    a simulated D435i (navcore's stereo failure model: holes,
//            speckle, range-dependent disparity noise), at a real D435i mode
//   attitude roll/pitch from the camera, heading from the "FC"
//   speed    ground speed, and the altitude the FC holds
//
// and NO POSITION to map with, which is architecture C as it flies: the module
// maps only while the aircraft is still, one fresh map per stop, and the
// mission moves in certified straight legs between stops (move-stop-sense).
// The mission's leg-length gate gets the true position, as it does in
// test_voxel_nav's closed loop (a flight estimate would be there aboard).
//
// What is simulated is only what cannot run here: the world, the camera's
// pixels, and the airframe -- yaw rate = stick x 90 deg/s, forward speed
// lagging the pitch stick with tau 0.35 s, altitude held. That plant and the
// wiring are test_voxel_nav's, so a flight seen here is a flight that test
// would score. Collisions are counted against the TRUE world (clearance under
// the airframe's 0.3 m radius), never against the map.
//
// Everything a viewer sees is drawn FROM this object by flight_view.cpp; none
// of it feeds back. The showcase can be as pretty as it likes without being
// able to change a single decision.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "frame_source.hpp"
#include "voxel_world.hpp"

#include "mission.hpp"      // onboard
#include "voxel_nav.hpp"    // onboard
#include "world_model.hpp"  // onboard

namespace kshow {

struct FlightParams {
    std::string world = "gallery";   // gallery | hall
    unsigned    seed  = 101;
    // 848x480, the mode the aircraft flies (main.cpp --voxel-width/height).
    // The map derives its honest marking range from THIS camera
    // (fineMapParams), so resolution is not cosmetic: at 424x240 the leg
    // certificate tops out near 1.9 m and the same 60 s covers ~35 % less
    // ground. Measured: 120 s of flight in 81 s on a desktop core, so the
    // real configuration costs the showcase nothing.
    int   camW = 848, camH = 480;
    float altM = 1.5f;               // above the floor, as test_voxel_nav flies
    // THE LAYOUT: room pitch drawn from [pitchMinM, pitchMaxM] per seed
    // (< 0: the world's own, below), and the fraction of room edges with no
    // doorway (< 0: the world's own).
    //
    // GALLERY FLIES 10-12 m ROOMS, hall 12-16. Measured, the aircraft's stack,
    // 150 s, seeds 201-204, 848x480 (flight_show_check FLIGHT_PITCH):
    //   gallery 12-16  net 12.6 m  cells 72.0      10-12  net 18.6  cells 84.8
    //   hall    12-16  net 13.2 m  cells 69.5      10-12  net  5.9  cells 74.5
    // 0 collisions in all 40 runs of the sweep, clearance 0.51 m or more. At
    // four seeds a side these are weak differences, not findings; the gallery
    // change is the one that held in both columns, hall's did not.
    float pitchMinM = -1.f, pitchMaxM = -1.f;
    float wallFrac = -1.f;
    bool  stereo = true;             // false: perfect depth (a control)
};

struct FlightStats {
    float timeS = 0, travelM = 0, netM = 0, minClearM = 1e9f;
    int   cells = 0, legs = 0, stops = 0, collisions = 0;
    bool  stuck = false;
};

// One straight leg as the mission committed it.
struct Leg {
    float e0 = 0, n0 = 0, bearingDeg = 0, lengthM = 0;
};

class FlightShow {
public:
    explicit FlightShow(const FlightParams& p);
    ~FlightShow();

    // One 0.05 s tick: telemetry in, the module (when the mission is at a
    // vantage, and occasionally otherwise), the mission, the airframe.
    void tick();
    static constexpr float kDt = 0.05f;

    const FlightParams&   params() const { return p_; }
    const sim::VoxelWorld& world() const { return *world_; }
    // Shared, so a picture of this flight can outlive it (the tour moves on).
    std::shared_ptr<const sim::VoxelWorld> worldPtr() const { return world_; }
    const sim::CamPose&   truth()  const { return truth_; }  // the airframe
    float                 speed()  const { return v_; }
    float                 floorZ() const { return floorZ_; }
    WorldState            state()  const { return wm_.snapshot(); }
    std::string           phase()  const { return phase_; }
    const FlightStats&    stats()  const { return stats_; }
    const std::vector<cv::Point3f>& trail() const { return trail_; }
    const std::vector<Leg>& legs() const { return legs_; }
    float spawnE() const { return spawnE_; }
    float spawnN() const { return spawnN_; }

    // The module and what it is working from. The map is in the VANTAGE
    // frame (origin where the aircraft stood when the stop began, altitude
    // 0 at the camera); vantage() says where that was in the world.
    const VoxelNavModule& module() const { return *mod_; }
    const sim::CamPose&   vantage() const { return vantage_; }
    const sim::DepthCamera& camera() const;
    // The depth frame the module was last given, and the true pose it was
    // taken from. Empty until the first frame.
    const cv::Mat&        lastDepth() const;
    const sim::CamPose&   lastDepthPose() const;
    long                  depthFrames() const;   // frames the module has taken

    // Distance from (x,y,z) to the nearest solid voxel of the TRUE world,
    // up to `upTo`.
    float clearance(float x, float y, float z, float upTo) const;

private:
    class Source;

    FlightParams p_;
    std::shared_ptr<sim::VoxelWorld> world_;
    float floorZ_ = 0.f, spawnE_ = 0.f, spawnN_ = 0.f;
    sim::CamPose truth_;
    Source* src_ = nullptr;                // owned by mod_
    std::unique_ptr<VoxelNavModule> mod_;
    std::unique_ptr<MissionController> mission_;
    WorldModel wm_;
    float v_ = 0.f;
    int   ticks_ = 0, lastResets_ = -1;
    std::string phase_;
    bool  inContact_ = false;
    sim::CamPose vantage_;
    FlightStats stats_;
    std::vector<char> visited_;
    std::vector<cv::Point3f> trail_;
    std::vector<Leg> legs_;
};

}  // namespace kshow
