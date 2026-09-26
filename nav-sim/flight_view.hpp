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
//   fpv      this stop's map from the aircraft's eye: near voxels, far
//            bearings, the certified leg fan on the floor. Pale is UNKNOWN.
//   mission  from above: the true walls, this stop's map over them, the trail,
//            every leg, and the numbers CLAUDE.md scores on.
// ---------------------------------------------------------------------------

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "flight_show.hpp"
#include "bearing_field.hpp"
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

// A PLANNED PATH DRAWN INTO A FIRST-PERSON VOXEL VIEW, as a ribbon lying
// flat: its edges offset `halfW` either side of the path, perpendicular to it,
// `drop` below the path's own height, and projected quad by quad from `eye` --
// so perspective narrows it into the distance -- and each pixel drawn only if
// it is NEARER than the surface `hitDist` (VoxelMap::renderLadder) has there,
// so the voxels in front of it hide it. Farther is darker. `arrow`: a flat
// arrowhead past the last point. `path` is in the same frame as `eye`.
void drawRibbon(cv::Mat& im, const sim::CamPose& eye, float hfovDeg, const cv::Mat& hitDist,
                const std::vector<std::array<float, 3>>& path, const cv::Scalar& colour,
                float halfW, double alpha, bool arrow, float drop);

// Everything the pictures need, copied out of a FlightShow.
struct ShowSnap {
    std::shared_ptr<const sim::VoxelWorld> world;
    std::shared_ptr<const sim::VoxelMap>   map;     // this stop's map (a copy)
    std::shared_ptr<const sim::BearingField> field; // and its far tier (a copy)
    float       maxIntegM = 0, farRangeM = 0;       // the ranges the two cover
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
    // The stop's map FROM THE AIRCRAFT'S EYE -- the first-person voxel view
    // the live camera pane draws, here for the simulated D435i. Pale is
    // UNKNOWN; the certified leg fan lies on the floor under the flight line.
    cv::Mat fpv(const ShowSnap& s, int w, int h) const;
    // compact: no legend and no numbers -- for an inset; the window's status
    // line carries the numbers.
    cv::Mat mission(const ShowSnap& s, int w, int h, bool compact = false) const;

private:
    sim::CamPose chasePose(const ShowSnap& s) const;

    float  camYaw_ = 0.f;
    double lastT_ = -1.0;
    float  boomM_ = 4.5f;
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
