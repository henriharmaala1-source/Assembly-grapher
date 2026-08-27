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
//   OCCLUSION SHADOW  and this one is not just at the frame edge. EVERY
//                     foreground object hides a strip of the background from
//                     the second imager, so a band down one side of it returns
//                     nothing. Width is exactly the disparity difference,
//                     f*B*(1/Z_near - 1/Z_far) pixels, and at f = 425 px and
//                     B = 50 mm a hand at 0.4 m against a 2 m wall casts a
//                     42 px shadow. In a forest that means a dead strip beside
//                     EVERY trunk -- precisely where the map most needs data,
//                     since it is the obstacle boundary. Modelling only the
//                     frame-edge band, as the first version did, understated
//                     the hole count everywhere that matters.
//
//   SPECKLE / OUTLIER A small fraction of pixels match wrongly and confidently,
//                     typically on repetitive texture. These are worse than
//                     holes: a hole is safe, a confident wrong depth is not.
//
//   IR PROJECTOR      the D435i throws a ~1 W speckle pattern, and whether that
//                     matters is a function of AMBIENT IR and RANGE rather than
//                     a constant. In bright open daylight at 5 m it is drowned
//                     and contributes nothing. Under closed canopy at dusk, or
//                     indoors, it is what carries the depth at all -- because
//                     the thing that drowns it is sunlight, and there is little
//                     of it there. A boreal stand in October is not an edge
//                     case for this project, it is the operating envelope.
//
//                     Modelled as ADDED TEXTURE on surfaces the pattern still
//                     reaches: the projector's entire job is to put features on
//                     a surface that has none, which is exactly what texThresh
//                     gates on. Off by default, so every number measured before
//                     it existed still stands as "passive stereo in adequate
//                     light" -- which is what they always silently were.
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
    // SILHOUETTE BOOST, and this is the correction that mattered most.
    //
    // Real depth imagery of a boreal stand shows trunks as black columns with
    // vivid coloured stripes down ONE EDGE. That is exactly what a block
    // matcher does: the silhouette of a trunk against bright sky is a strong
    // horizontal gradient and correlates easily, while the interior of a smooth
    // shadowed cylinder has nothing to lock onto. The trunk is not invisible --
    // its OUTLINE is visible, which is enough to know something is there.
    //
    // The first model gave each trunk one texture value, so a trunk either
    // resolved whole or vanished whole. Neither happens. Here, a pixel whose
    // neighbourhood spans a large depth discontinuity gets its effective
    // texture raised, because the discontinuity IS the feature being matched.
    float edgeBoost   = 0.75f;   // effective texture at a strong depth edge
    float edgeDepthM  = 0.8f;    // depth step across the window that counts as one
    int   edgeWinPx   = 3;       // neighbourhood for the discontinuity test
    // Post-match rejection, modelling what every real pipeline does: left-right
    // consistency plus a speckle filter. A valid pixel surrounded mostly by
    // invalid ones is thrown away. This is why real output has clean hole
    // EDGES rather than a fringe of surviving pixels.
    // Stereo shadow beside foreground objects (see OCCLUSION SHADOW above).
    // Off restores the old behaviour exactly, for comparing against numbers
    // measured before it existed.
    // IR PROJECTOR (see above). emitterOn=false is the old behaviour exactly.
    //
    // ambientIR is the thing the pattern competes with: 1.0 = bright open
    // daylight and the projector is useless, 0.0 = darkness and it is the only
    // source of texture there is. Canopy shade is perhaps 0.3-0.5 and dusk
    // lower still -- but those are GUESSES until measured with the real camera,
    // which is now possible and is the point of exposing the knob rather than
    // burying an assumption.
    //
    // Falloff is 1/(1 + (r/emitterRangeM)^2): irradiance drops as 1/r^2, so the
    // pattern's contrast against a fixed ambient does too. emitterRangeM is
    // therefore the HALF-STRENGTH range, not a hard cutoff.
    bool  emitterOn      = false;
    float emitterRangeM  = 3.0f;
    float emitterTex     = 0.9f;   // texture it imposes at point-blank in the dark
    float ambientIR      = 1.0f;   // 1 = bright daylight, 0 = dark
    bool  modelOcclusion = true;
    bool  filterSpeckle = true;
    int   speckleWin = 2;        // half-window for the validity majority test
    float speckleKeep= 0.45f;    // keep only if this fraction of neighbours valid
    unsigned seed    = 7;
    // MEASURED INTRINSICS, when we have them. Zero means "derive from hfovDeg
    // with a centred principal point", which is what the synthetic worlds use
    // and what every number in this tree was measured against -- so leaving
    // these at zero is bit-identical to the behaviour before they existed.
    //
    // A REAL camera is not that. The D435i reports fx != fy and a principal
    // point a few pixels off centre, and replaying a recording through rays
    // built from an assumed pinhole would quietly bend every one of them --
    // small, systematic, and indistinguishable from a calibration fault in the
    // map. Recording intrinsics alongside the pixels (see depth_record.hpp) is
    // only useful if something then uses them; this is that something.
    float fxPx  = 0.f, fyPx = 0.f;
    float ppxPx = 0.f, ppyPx = 0.f;
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
    float fy()  const { return fy_; }
    float ppx() const { return ppx_; }
    float ppy() const { return ppy_; }
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

    // Rotate a CAMERA-frame vector (+x right, +y down, +z forward) into world
    // ENU. rayFor is this function applied to a pixel's normalised ray, and it
    // is written once for the reason stated there: two conventions for the same
    // thing is a bug waiting to happen. The flow estimator solves in camera
    // frame and the odometry integrates in world, so somebody has to do this
    // and it must not be a second copy of the rotation.
    static void camToWorld(const CamPose& pose, float cx, float cy, float cz,
                           float& wx, float& wy, float& wz);

    // A SYNTHETIC IR IMAGE, for testing the flow estimator without a camera.
    //
    // The one property that makes it useful is that intensity is a function of
    // the WORLD POINT, not of the pixel: a surface point looks the same from a
    // different viewpoint, which is precisely the assumption optical flow is
    // built on. A render that hashed pixel coordinates would produce an image
    // that correlates with nothing and would fail the estimator for a reason
    // that has no counterpart on real hardware.
    //
    // Texture RICHNESS modulates the amplitude, so a blank wall renders nearly
    // flat and the estimator finds no points on it -- the same failure the real
    // sensor has, reproduced rather than papered over. The projector falloff is
    // modelled too, because IR illumination is what makes bark matchable at 2 m
    // and unmatchable at 8 m.
    cv::Mat renderIR(const VoxelWorld& w, const CamPose& pose) const;

private:
    CamParams p_;
    float fpx_ = 0, fy_ = 0, ppx_ = 0, ppy_ = 0;
    mutable uint32_t rng_ = 12345;
    float urand() const;   // xorshift, so rendering is deterministic per seed
};

}  // namespace sim
