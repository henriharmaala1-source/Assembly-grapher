#pragma once
// ---------------------------------------------------------------------------
// FlightView -- the pictures of a FlightShow. READ-ONLY by construction: they
// are drawn from a ShowSnap, a copy of the flight's state, so nothing here can
// reach a decision, and the pictures can be rendered on another thread while
// the flight carries on in real time.
//
// The five pictures, and what each one is allowed to be:
//
//   chase    the TRUE world from behind the aircraft, the aircraft drawn in,
//            its trail and the leg it is flying -- and, laid over it, what
//            this stop's map KNOWS: air confirmed free at flight height (blue)
//            and cells marked solid (red points). Captioned as the simulated
//            scene: it is never a sensor.
//   camera   the D435i's left IR image -- what the stereo matcher looks at.
//   depth    the depth frame the module was actually handed, holes and all:
//            red near, blue far on a FIXED scale, holes grey.
//   belief   this stop's map as a model: marked cells as cubes, confirmed-free
//            air as a sheet, the certified leg fan. Empty is UNKNOWN.
//   mission  from above: the true walls, this stop's map over them, the trail,
//            every leg, and the numbers CLAUDE.md scores on.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "flight_show.hpp"
#include "voxel_map.hpp"

namespace kshow {

// One bearing of the module's leg search: how far a straight leg is certified.
struct Ray { float bearingDeg = 0, freeM = 0; };

// THE LEG FAN, as VoxelNavModule searches it (voxel_nav.cpp): from the
// vantage (its map's origin), at the current heading, across the camera's
// view less 8 deg each side, every 3 deg, each certified with the pipeline's
// own const straightFreeM out to legMaxM. For display; call on the flight's
// thread.
std::vector<Ray> legFan(const FlightShow& f);

// Everything the pictures need, copied out of a FlightShow.
struct ShowSnap {
    std::shared_ptr<const sim::VoxelWorld> world;
    std::shared_ptr<const sim::VoxelMap>   map;     // this stop's map (a copy)
    int         mapFrames = 0;
    long        mapKey = -1;                        // which map, and how far along
    sim::CamParams cam;                             // the D435i
    float       floorZ = 0, speed = 0;
    sim::CamPose truth, vantage;
    std::string phase, worldName;
    int         mapNo = 0;                          // which map of the tour
    FlightStats stats;
    std::vector<cv::Point3f> trail;
    std::vector<Leg> legs;
    std::vector<Ray> fan;
    cv::Mat     depth;                              // the last frame the module had
    long        depthSeq = 0;                       // ... and which one it was

    // Build from the flight, reusing `prev`'s map copy (13 ms for 5.5 M cells)
    // when the map has not changed since, and its fan outside THINK/SCAN.
    static ShowSnap of(const FlightShow& f, const ShowSnap* prev);
};

class FlightView {
public:
    // Call once per rendered frame, before the pictures: moves the chase
    // camera (smoothed) so all the pictures agree on one viewpoint.
    void update(const ShowSnap& s);

    cv::Mat chase(const ShowSnap& s, int w, int h) const;
    cv::Mat camera(const ShowSnap& s, int w, int h) const;
    cv::Mat depth(const ShowSnap& s, int w, int h) const;
    cv::Mat belief(const ShowSnap& s, int w, int h) const;
    cv::Mat mission(const ShowSnap& s, int w, int h) const;

private:
    sim::CamPose chasePose(const ShowSnap& s) const;

    float  camYaw_ = 0.f;
    double lastT_ = -1.0;
    float  boomM_ = 4.5f;
    float  isoSpin_ = 0.f;
    const sim::VoxelWorld* planOf_ = nullptr;   // which world plan_ was drawn from
    cv::Mat plan_;        // the true world at flight altitude, 1 px per cell
    long cacheKey_ = -1;                        // which map free_/occ_ list
    std::vector<cv::Point2f> free_;             // FREE cells at flight height (map frame)
    std::vector<cv::Point3f> occ_;              // OCCUPIED cells near the vantage
    mutable long depthOf_ = -1;                 // the frame depthPane_ shows
    mutable cv::Mat depthPane_;
    float   planCell_ = 0.25f;
};

}  // namespace kshow
