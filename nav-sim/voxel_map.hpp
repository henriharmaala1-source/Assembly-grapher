#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "voxel_world.hpp"

// ---------------------------------------------------------------------------
// The ESTIMATED 3D world model — log-odds occupancy built from noisy depth.
//
// This is the thing under test. VoxelWorld is truth; this is what the aircraft
// would actually believe, and score() measures the difference.
//
// THREE STATES, NOT TWO. Every cell is free, occupied, or UNKNOWN, and unknown
// is the default. This is not a stylistic choice: a two-state map has no way to
// say "I cannot see here", so an untextured wall that returns no disparity is
// indistinguishable from open air. Free space is carved ONLY along rays that
// terminated in a valid measurement; a ray with no return contributes nothing.
// That single rule is the difference between a map that is merely incomplete
// and one that will fly an aircraft into a glass facade.
//
// EGOCENTRIC AND SHORT-LIVED. The grid is world-anchored but recentred when the
// vehicle approaches an edge, and shifting decays what scrolls back in. The map
// is only ever as good as the pose estimate, and pretending otherwise is how
// map-driven avoidance kills aircraft. Keep it local, keep it recent.
// ---------------------------------------------------------------------------

namespace sim {

struct VoxelMapParams {
    float cell     = 0.25f;
    int   nx       = 240, ny = 240, nz = 96;   // ~60 x 60 x 24 m at 0.25 m
    float lHit     = 0.85f;    // log-odds added at a ray's termination
    float lMiss    = 0.40f;    // log-odds removed along the free part of a ray
    float lClamp   = 4.0f;     // saturation, so the map can still change its mind
    float occThresh= 0.0f;     // > this = occupied
    float freeThresh = -0.4f;  // < this = free; between the two = unknown
    // MEASURED, and the single most important number in this file.
    //
    // Depth error grows as dZ = Z^2 * sigma_d / (f*B). Carving free space is
    // safe only while dZ stays under about one voxel: past that the ray's
    // endpoint lands SHORT of the real surface and the DDA carves free space
    // straight through a solid obstacle. Solving dZ = cell:
    //
    //     Z_max = sqrt(cell * f * B / sigma_d)
    //
    // Sweep on the forest world (f=228 px, B=0.12 m, cell=0.25 m, so the
    // formula predicts 7.4 m), scoring FALSE-FREE cells -- solid in truth,
    // called free by the map, i.e. places a planner would fly into something:
    //
    //     integrate to    4 m     6 m     8 m    12 m    18 m    25 m
    //     truth depth   0.00%   0.00%   0.00%   0.00%   0.00%   0.00%
    //     stereo        0.00%   0.07%   2.95%   7.69%  10.81%  13.22%
    //
    // Truth depth is 0.00% at EVERY range, which is what proves the mapper and
    // the three-state rule are correct. Everything above is the sensor.
    //
    // So the formula is a slight over-estimate; derate it ~25%. Practical
    // Z_max at sigma_d = 0.25 px:
    //
    //     f px   B      cell 0.15   0.25    0.5
    //      228   12 cm       4.1     5.2    7.4    (this demo)
    //      520   12 cm       6.1     7.9   11.2    (IMX296 at 728x544)
    //     1040   12 cm       8.7    11.2   15.8    (IMX296 full res)
    //      520   25 cm       8.8    11.4   16.1
    //
    // The blunt consequence: a 12 cm baseline supports voxel mapping to roughly
    // 8-11 m, NOT to the 20-30 m the raw depth range suggests. Returns beyond
    // Z_max are not merely imprecise, they are actively dangerous, because the
    // error is what carves the hole. If you need map out to 20 m, either take
    // coarser voxels at range or stop carving and only mark occupancy.
    float maxIntegM= 8.f;
    // FREE-SPACE CARVING IS A SEPARATE DECISION FROM MARKING AN OBSTACLE, and
    // conflating them was a paralysing bug.
    //
    // maxIntegM exists because a far return's POSITION is untrustworthy -- the
    // error grows as Z^2 and past Z_max it carves through the very obstacle it
    // found. But the original code discarded the ENTIRE RAY when the endpoint
    // was too far, throwing away the free space along it as well. Measured at
    // the city spawn: the nearest truth return is 10.39 m and ZERO pixels are
    // under 8 m, so with maxIntegM = 8 not one ray was integrated, the map
    // stayed completely empty, nothing was ever FREE, and the aircraft sat
    // still for 1200 steps. The stereo runs only moved because 0.4% speckle
    // outliers landed inside 8 m -- they were flying on sensor noise.
    //
    // A ray that returns 30 m still proves the first several metres are empty.
    // So carve free space out to (r - k*sigma(r)), the point beyond which the
    // endpoint's own uncertainty makes the claim unsafe, capped at maxCarveM;
    // and mark OCCUPIED only when r <= maxIntegM.
    float maxCarveM = 25.f;   // free space may be claimed this far out
    float carveSigK = 2.0f;   // stop carving this many depth-sigmas short of the hit
    // sigma(Z) = Z^2 * subpixel / (f*B); supplied by the camera so the map does
    // not have to know the optics. 0 disables the shortening.
    float depthSigCoef = 0.f;  // = subpixelPx / (f_px * baseline_m)
};

class VoxelMap {
public:
    void init(const VoxelMapParams& p, float cx, float cy, float cz);
    const VoxelMapParams& params() const { return p_; }

    // Fold one depth image in. `cam` supplies the ray directions so the mapper
    // and the renderer cannot disagree about geometry.
    void integrate(const cv::Mat& depth, const DepthCamera& cam, const CamPose& pose);

    // Recentre on the vehicle, decaying anything that scrolls in from outside.
    void recentre(float cx, float cy, float cz);

    enum State : uint8_t { UNKNOWN = 0, FREE = 1, OCCUPIED = 2 };
    State stateAt(float wx, float wy, float wz) const;
    float logAt(int x, int y, int z) const {
        return inBounds(x, y, z) ? log_[idx(x, y, z)] : 0.f;
    }
    bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < p_.nx && y < p_.ny && z < p_.nz;
    }
    void worldToCell(float wx, float wy, float wz, int& x, int& y, int& z) const;
    void cellCentre(int x, int y, int z, float& wx, float& wy, float& wz) const;

    // --- scoring against truth ---------------------------------------------
    //
    // The numbers that actually answer "does the voxel model work".
    struct Score {
        long occTP = 0, occFP = 0, occFN = 0;   // occupied-cell confusion
        long freeTP = 0, freeFP = 0;            // free-cell confusion
        long unknown = 0, total = 0;
        // Cells the map has an OPINION about. IoU must be computed over these,
        // not over every solid cell in the region: in a forest most solid
        // voxels are inside a canopy, behind a trunk or underground, and no
        // camera can ever see them. Scoring them as misses measures occlusion,
        // not mapping quality, and buries the signal you actually want.
        long observed = 0;
        // THE SAFETY NUMBER. Cells the map calls FREE that are actually SOLID.
        // Every one of these is a potential collision. Weight it above all else.
        long falseFree = 0;
        // IoU over OBSERVED cells only (see `observed`).
        double iou() const {
            double d = double(occTP + occFP + falseFree);
            return d > 0 ? double(occTP) / d : 0.0;
        }
        // Of the cells the map calls occupied, how many really are.
        double precision() const {
            double d = double(occTP + occFP);
            return d > 0 ? double(occTP) / d : 0.0;
        }
        // Fraction of the scored region the map has any opinion about.
        double coverage() const {
            return total > 0 ? double(observed) / double(total) : 0.0;
        }
        double falseFreeRate() const {
            double d = double(freeTP + falseFree);
            return d > 0 ? double(falseFree) / d : 0.0;
        }
    };
    // Compares only cells within `radiusM` of (cx,cy,cz) and below `maxZ`, so
    // the score is about the region the vehicle could actually hit rather than
    // being dominated by unobserved sky.
    Score score(const VoxelWorld& truth, float cx, float cy, float cz,
                float radiusM, float maxZ) const;

    // --- visualisation ------------------------------------------------------
    // Horizontal slice at height wz: grey = unknown, white = free, black = occ.
    cv::Mat sliceImage(float wz, int outPx = 480) const;
    // Isometric projection of occupied cells, height-coloured. Cheap, no GPU,
    // and enough to see whether the map looks like the world.
    cv::Mat isoImage(int outPx = 640, float maxZ = 30.f) const;

private:
    size_t idx(int x, int y, int z) const { return (size_t(z) * p_.ny + y) * p_.nx + x; }
    void rayInsert(float px, float py, float pz, float dx, float dy, float dz,
                   float carveTo, float hitAt);

    VoxelMapParams p_;
    std::vector<float> log_;
    float ox_ = 0, oy_ = 0, oz_ = 0;   // world coord of cell (0,0,0) min corner
};

// Side-by-side truth vs estimate isometric view, with the score printed on it.
cv::Mat compareImage(const VoxelWorld& truth, const VoxelMap& map,
                     const VoxelMap::Score& s, int outPx = 640);

}  // namespace sim
