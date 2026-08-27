#pragma once

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "voxel_map.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// FRAME-TO-MAP SCAN MATCHING. Pose from GEOMETRY, not from image texture.
//
// WHY THIS AND NOT OPTICAL FLOW. Flow needs a textured image; this needs
// structured geometry. Those are different failures, and indoors they are
// nearly opposite ones: a corridor of blank plaster is invisible to flow and
// perfectly visible to a depth frame. It also reuses the representation the
// rest of the stack already trusts, and needs no IR stream -- so it is not
// blocked on hardware plumbing.
//
// WHAT IT IS. Correlative scan matching (Olson 2009), which is what Hector SLAM
// and Cartographer's local stage do: take the current depth frame as a sparse
// point cloud, slide it over the accumulated map, and keep the offset where the
// points land most convincingly on occupied cells. Coarse pass then fine pass,
// because an exhaustive fine search over the whole capture radius is wasted
// work when the answer is almost always near the prediction.
//
// WHAT IT IS NOT. Not SLAM. There is no keyframe store, no place recognition
// and no pose graph, so error still accumulates -- it is bounded per frame by
// the map rather than corrected globally. That is the same deliberate limit as
// everywhere else in this stack, and it is why this is called odometry.
//
// IT MUST BE ABLE TO REFUSE, and that is most of the design. The failure that
// matters is not a wrong answer, it is a CONFIDENT wrong answer -- and scan
// matching has a specific, predictable way of producing one. Slide a frame of a
// long featureless corridor along the corridor and the score does not change,
// so the optimum is arbitrary along that axis while looking like a clean fit on
// the other two. That is the aperture problem at room scale.
//
// So the score's CURVATURE is measured per axis at the optimum and reported.
// A flat axis is an unobservable axis, and the caller is told which, rather
// than being handed a number that averages an observed axis with an invented
// one. Same doctrine as unknown-is-not-free: absence of evidence is reported,
// not filled in.
// ---------------------------------------------------------------------------

struct ScanMatchParams {
    // Two passes. The coarse one finds the basin, the fine one finds the
    // bottom of it. Range is a RADIUS about the predicted pose, so it should
    // cover the prediction's own error and no more -- a wider search is not
    // safer, it is more chances to land in the wrong basin.
    float coarseStepM  = 0.12f;
    float coarseRangeM = 0.48f;
    float fineStepM    = 0.03f;
    float fineRangeM   = 0.12f;

    int   strideX = 12, strideY = 12;   // depth pixels between sampled points
    int   minPoints = 40;               // below this the solve is not attempted

    // Only points the map could have MARKED are worth matching against. Beyond
    // the marking range the map holds carved free space and unknown, so a point
    // there scores nothing whatever the offset -- it is noise in the sum.
    float minRangeM = 0.35f;
    float maxRangeM = 0.f;              // 0 = take the map's own marking range

    // An axis counts as observed when moving one fine step off the optimum
    // costs at least this fraction of the peak score. Measured, not guessed --
    // see scan_match_check.
    float minCurvatureFrac = 0.004f;

    // And the fit itself has to be worth something: this fraction of sampled
    // points must land on cells the map calls OCCUPIED.
    float minHitFrac = 0.15f;
};

struct ScanMatch {
    float dE = 0.f, dN = 0.f, dU = 0.f;   // correction to the predicted pose, m
    int   points = 0;                     // sampled and inside the useful band
    float score = 0.f;                    // log-odds sum at the optimum
    float hitFrac = 0.f;                  // of `points`, fraction on OCCUPIED
    float curv[3] = {0.f, 0.f, 0.f};      // score drop per fine step, E/N/U
    bool  axisObserved[3] = {false, false, false};
    bool  valid = false;                  // ALL of: points, hitFrac, 3 axes
};

class ScanMatcher {
public:
    void init(const ScanMatchParams& p) { p_ = p; }
    const ScanMatchParams& params() const { return p_; }

    // `guess` is where the vehicle thinks it is -- from inertial propagation,
    // or from constant velocity, or from the last pose if nothing better.
    // Attitude in `guess` is TAKEN AS GIVEN and never solved for: roll and
    // pitch are gravity-referenced and drift-free, which is the whole reason
    // this can search three degrees of freedom instead of six.
    ScanMatch match(const VoxelMap& map, const cv::Mat& depth,
                    const DepthCamera& cam, const CamPose& guess) const;

private:
    ScanMatchParams p_;
};

}  // namespace sim
