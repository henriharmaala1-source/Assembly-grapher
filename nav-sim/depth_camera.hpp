#pragma once

#include <cstdint>
#include <cstdlib>

#include <opencv2/core.hpp>

#include "voxel_world.hpp"

// ---------------------------------------------------------------------------
// Simulated STEREO depth camera looking into a VoxelWorld.
//
// This class is the sim-to-real gap. Everything else in the voxel pipeline is
// arithmetic that will behave identically on a desktop and on the aircraft; the
// only thing that can lie to you is the depth image. So this deliberately
// models the ways a real passive stereo pair fails, and each one is here
// because it is a documented killer rather than because it was easy:
//
//   TEXTURE DROPOUT   Stereo matches TEXTURE, not geometry. A blank wall, a
//                     glass facade, still water or overcast sky returns NOTHING.
//                     This is the failure that kills aircraft, because "no
//                     disparity" silently becomes "free space" in a two-state
//                     map. Every solid voxel carries a texture value and this
//                     model refuses to return depth below a threshold.
//
//   DISPARITY QUANT   Real depth is computed from an integer-plus-subpixel
//                     disparity, so error grows as Z^2/(f*B) -- NOT as a
//                     constant percentage. Quantising here reproduces that
//                     automatically and gets the far-field noise right.
//
//   NEAR BLIND ZONE   Below f*B/D_max the disparity exceeds the search range
//                     and there is simply no answer. On a 12 cm baseline at
//                     728x544 with 96 disparities that is ~0.8 m -- inside
//                     stopping distance, and the reason short baselines win.
//
//   OCCLUSION BAND    The leftmost f*B/Z pixels of the left image have no
//                     counterpart in the right image. At 1 m and a 25 cm
//                     baseline that is 18% of the frame with no stereo at all.
//
//   SPECKLE / OUTLIER A small fraction of pixels match wrongly and confidently,
//                     typically on repetitive texture. These are worse than
//                     holes: a hole is safe, a confident wrong depth is not.
//
// Not modelled, deliberately, and worth knowing: lens flare, rolling shutter
// (the recommendation is global shutter), rain on the lens, and calibration
// drift. Drift in particular is better tested by perturbing the extrinsics
// between the truth raycast and the reprojection than by anything here.
// ---------------------------------------------------------------------------

namespace sim {

struct CamParams {
    int   width      = 320;
    int   height     = 240;
    float hfovDeg    = 70.f;
    float baselineM  = 0.12f;
    int   maxDisp    = 96;       // disparity search range
    float subpixelPx = 0.25f;    // 1-sigma matching noise, pixels
    float texThresh  = 0.25f;    // below this texture richness -> no match
    float speckleFrac= 0.004f;   // fraction of pixels given a wrong-but-confident depth
    float maxRangeM  = 40.f;
    // BLOCK GRANULARITY, and this was a real modelling error. A stereo matcher
    // correlates a WINDOW, so when a surface has too little texture the whole
    // window fails together -- you get coherent holes the size of the obstacle.
    // The first version made the texture decision per PIXEL, which produces
    // salt-and-pepper: a lace curtain instead of a hole. Real depth images from
    // a forest show large solid regions of nothing, exactly where the trunks
    // are, and a mapper that never sees a coherent hole is never tested against
    // the failure that actually occurs.
    int   blockPx    = 8;        // matcher window; 0 reverts to per-pixel
    // Post-match rejection, modelling what every real pipeline does: left-right
    // consistency plus a speckle filter. A valid pixel surrounded mostly by
    // invalid ones is thrown away. This is why real output has clean hole
    // EDGES rather than a fringe of surviving pixels.
    bool  filterSpeckle = true;
    int   speckleWin = 2;        // half-window for the validity majority test
    float speckleKeep= 0.45f;    // keep only if this fraction of neighbours valid
    unsigned seed    = 7;
};

// Pose: position in ENU metres, yaw about +z (0 = +y/North, CW positive to
// match sim_world.hpp), pitch positive nose-up, roll positive right-wing-down.
struct CamPose {
    float e = 0, n = 0, u = 0;
    float yawDeg = 0, pitchDeg = 0, rollDeg = 0;
};

class DepthCamera {
public:
    explicit DepthCamera(const CamParams& p = CamParams());

    float fpx() const { return fpx_; }
    const CamParams& params() const { return p_; }

    // GROUND TRUTH depth (CV_32F, metres, <=0 invalid). No degradation at all.
    // Keep this: scoring the estimated map needs a truth reference, and having
    // both from the same raycaster removes a whole class of frame-convention
    // bugs from the comparison.
    cv::Mat renderTruth(const VoxelWorld& w, const CamPose& pose) const;

    // What the aircraft would actually receive. Same geometry, all the failure
    // modes above applied. `validFrac` optionally receives the fraction of
    // pixels that returned a usable depth -- log it, because a collapsing valid
    // fraction is a first-class health signal, not a curiosity.
    cv::Mat renderStereo(const VoxelWorld& w, const CamPose& pose,
                         float* validFrac = nullptr) const;

    // Unit ray direction in WORLD coords for pixel (u,v). Exposed because the
    // voxel mapper needs exactly the same rays to carve free space along.
    void rayFor(const CamPose& pose, int u, int v,
                float& dx, float& dy, float& dz) const;

private:
    CamParams p_;
    float fpx_ = 0;
    mutable uint32_t rng_ = 12345;
    float urand() const;   // xorshift, so rendering is deterministic per seed
};

}  // namespace sim
