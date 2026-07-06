#pragma once

#include <opencv2/core.hpp>
#include <vector>

// ---------------------------------------------------------------------------
// Tier-1 desktop simulation world + 2.5D raycaster.
//
// This is a DESKTOP tool, separate from the Pi. Its only job is to validate the
// navigation/planning LOGIC by driving the REAL onboard modules (DepthNav,
// LocalMap/MissionController, StateEstimator, SimFcBackend) with rendered input
// instead of a real camera — closing the one hole in the headless SITL suite:
// perception has never actually been IN the loop.
//
// The world is a flat ENU plane (metres, 0 = North/+N, East = +E) populated by
// circular and wall obstacles. The raycaster gives, per bearing:
//   * a ground-truth metric range (a "perfect ToF" — feeds DepthNav.updateFromGrid)
//   * a rendered first-person strip (feeds the real depth model via update()).
//
// HONEST SCOPE: a clean raycast render has none of the interlacing / analog
// noise / capture-dongle character of the real signal. This validates that
// perception -> planning -> control is wired correctly and behaves on real model
// output; it does NOT validate survival of the analog-capture domain. That stays
// a hardware-bring-up question.
// ---------------------------------------------------------------------------

namespace sim {

// A circular obstacle in the ENU plane. ve/vn allow a moving obstacle (m/s).
struct Circle {
    float e = 0, n = 0, r = 1.f;
    float ve = 0, vn = 0;
};

// A wall as a line segment (for corridors / mazes / box canyons).
struct Wall {
    float e0 = 0, n0 = 0, e1 = 0, n1 = 0;
};

struct World {
    std::vector<Circle> circles;
    std::vector<Wall>   walls;

    // Optional occupancy bitmap (e.g. loaded from a PNG / ROS map / floorplan):
    // occ[y*ow+x]==1 means that cell is solid. occDist holds each cell's distance
    // (m) to the nearest solid, precomputed at load for fast clearance queries.
    std::vector<unsigned char> occ;
    std::vector<float>         occDist;
    int   ow = 0, oh = 0;
    float ocell = 0.1f, oe0 = 0.f, on0 = 0.f;
    bool  hasOcc() const { return ow > 0 && oh > 0; }
    bool  occSolid(int cx, int cy) const {
        return cx>=0 && cy>=0 && cx<ow && cy<oh && occ[(size_t)cy*ow+cx];
    }

    void advance(float dt);   // moves any circle with a velocity

    // Ground-truth range (m) from p along `bearingDeg` (0 = North, +E clockwise)
    // to the nearest obstacle, capped at maxRange. This is the perfect-ToF value.
    float rayRange(float pe, float pn, float bearingDeg, float maxRange) const;

    // Signed clearance of point p to the nearest obstacle EDGE (negative = inside
    // an obstacle). Used for standoff / collision metrics against TRUTH.
    float clearanceAt(float pe, float pn) const;
};

// Load a PNG/PGM occupancy image into `w` as an occupancy bitmap: dark pixels
// (< threshold) are solid. `metersPerPixel` sets the scale. Returns false on
// read failure. Also picks a free START (near the bottom-centre) and a far free
// GOAL, returned in world coords, with the world origin placed so the start is
// at (0,0). Any existing circles/walls in `w` are left intact.
bool loadOccupancyImage(World& w, const std::string& path, float metersPerPixel,
                        float& startE, float& startN, float& goalE, float& goalN);

// Cast N rays across [yaw - hFov/2, yaw + hFov/2] and fill `ranges` (metres).
void castScan(const World& w, float pe, float pn, float yawDeg,
              float hFovDeg, int n, float maxRange, std::vector<float>& ranges);

// First-person 2.5D render from a per-bearing range scan: each screen column is
// a vertical wall strip whose height ~ 1/range, shaded by distance, over a
// floor/sky gradient. Deliberately simple and CPU-only — no GPU, no scene graph.
cv::Mat renderFPV(const std::vector<float>& ranges, float maxRange, int w, int h);

// Build the CV_32F metric depth grid (rows x cols, metres, <=0 = invalid) that
// DepthNav::updateFromGrid expects, from a per-bearing range scan (flat world =
// range constant down each column).
cv::Mat rangesToGrid(const std::vector<float>& ranges, int rows);

// Top-down "God's-eye" debug view: true obstacles, the drone + heading, the goal,
// the committed waypoint, the goal/plan bearing, and the flown trail.
cv::Mat renderTopDown(const World& w, float pe, float pn, float yawDeg,
                      float goalE, float goalN, float wpE, float wpN,
                      float planBearingDeg, bool planValid,
                      const std::vector<cv::Point2f>& trail,
                      const char* phase, int size, float spanM);

}  // namespace sim
