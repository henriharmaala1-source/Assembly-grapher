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

    // YAW, searched as a fourth degree of freedom about the vertical through
    // the camera. Roll and pitch are NOT searched: gravity references them
    // absolutely, so the accelerometer already answers them better than any
    // amount of geometry could. Yaw is the one attitude the IMU cannot bound --
    // gravity says nothing about heading -- so it is the one the map has to.
    //
    // THE RESOLUTION LIMIT, which is geometry and not tuning. A yaw error
    // moves a point at range R sideways by R*theta, and that is only visible
    // once it exceeds a cell:
    //
    //     theta_min ~ cell / R = 0.25 m / 3 m = 4.8 degrees
    //
    // Measured, consistent with it: a 1.5 degree error against a pillar 2 m
    // away gives 0.0046 of score curvature where the best translation axis
    // gives 0.52 -- two orders down, and correctly refused.
    //
    // THE CONSEQUENCE IS THE POINT. Scan matching at this cell size cannot
    // bound the slow gyro drift that actually accumulates: half a degree per
    // second over a few seconds is far below what a 0.25 m map at 3 m can see.
    // It could only ever catch gross heading errors. Bounding real yaw drift
    // needs a finer map, a much longer range, or a compass -- which is why the
    // airframe carries one.
    // OFF BY DEFAULT, and honestly so: see the resolution limit below. The
    // search is implemented and reports observability, but it has not been
    // shown to recover a known heading error in any scene tried, so nothing
    // ships with it enabled.
    float yawStepDeg  = 0.5f;
    float yawRangeDeg = 0.0f;
    // A DEADBAND, and it earned its place immediately. On a CORRECT guess in a
    // corridor the search picked -0.5 degrees -- one quantisation step, chosen
    // by noise on a nearly flat score -- and that spurious rotation then made
    // the along-corridor axis look observable, because rotating a corridor
    // manufactures an apparent gradient along it. One bad degree of freedom
    // corrupted the honesty of another.
    //
    // So a heading correction has to PAY for itself: unless the best yaw beats
    // the predicted yaw by this fraction of the score, the prediction stands.
    // Same rule as everywhere else here -- do not act on evidence you do not
    // have.
    float yawMinGainFrac = 0.004f;

    // OBSERVABILITY IS A COMPARISON, not a threshold. What matters is whether
    // an axis is pinned like the others, and the absolute numbers move with the
    // scene: the same corridor gives 0.82 on the vertical and 0.02 laterally,
    // while a sparser scene scales all three down together. A fixed cut would
    // have to be retuned per environment, which is how a safety gate quietly
    // stops gating.
    //
    // So an axis is observed when it carries at least this fraction of the
    // best-constrained axis, with a small absolute floor beneath it for the
    // case where nothing is constrained at all.
    float relCurvatureFrac = 0.15f;
    float minCurvatureFrac = 0.01f;
    // HOW FAR OFF, and this is NOT the fine search step. Probing at 0.03 m
    // against a 0.25 m grid measures CELL QUANTISATION rather than the scene: a
    // featureless plane marked into a voxel grid has structure at the cell
    // pitch, so sliding a fraction of a cell along it changes which cells the
    // points fall in and manufactures a gradient where the world has none.
    //
    // Observed: a straight corridor reported its along-axis observable at
    // 0.0075 against a 0.004 threshold, on the strength of the floor alone.
    // Probing at one full cell asks about the SCENE instead.
    //
    // 0 takes the map's own cell size, which is the right default -- the map
    // knows its resolution and this file should not carry a second copy of it.
    float curvStepM = 0.f;

    // And the fit itself has to be worth something: this fraction of sampled
    // points must land on cells the map calls OCCUPIED.
    float minHitFrac = 0.15f;
};

struct ScanMatch {
    float dE = 0.f, dN = 0.f, dU = 0.f;   // correction to the predicted pose, m
    float dYawDeg = 0.f;                  // and to its heading
    int   points = 0;                     // sampled and inside the useful band
    float score = 0.f;                    // corroborating cells at the optimum
    float hitFrac = 0.f;                  // of `points`, fraction on OCCUPIED
    float curv[4] = {0, 0, 0, 0};         // score drop per fine step, E/N/U/yaw
    bool  axisObserved[4] = {false, false, false, false};
    bool  valid = false;                  // points, hitFrac, and E/N/U observed
    // Yaw is reported separately and is NOT required for `valid`: a translation
    // fix is useful without a heading fix, and in a rotationally symmetric room
    // yaw is unobservable while translation is fine. Consumers that want the
    // heading must check axisObserved[3] themselves.
    bool  yawObserved() const { return axisObserved[3]; }
};

class ScanMatcher {
public:
    void init(const ScanMatchParams& p) { p_ = p; }
    const ScanMatchParams& params() const { return p_; }

    // `guess` is where the vehicle thinks it is -- from inertial propagation,
    // or from constant velocity, or from the last pose if nothing better.
    //
    // ROLL AND PITCH IN `guess` ARE TAKEN AS GIVEN and never solved for. They
    // are gravity-referenced and drift-free, so geometry cannot improve on
    // them. YAW is solved, because nothing else can: it is the one attitude
    // with no absolute reference anywhere in this vehicle.
    ScanMatch match(const VoxelMap& map, const cv::Mat& depth,
                    const DepthCamera& cam, const CamPose& guess) const;

private:
    ScanMatchParams p_;
};

}  // namespace sim
