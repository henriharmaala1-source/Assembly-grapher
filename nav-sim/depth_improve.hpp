#pragma once

#include <opencv2/core.hpp>

// ---------------------------------------------------------------------------
// DEPTH IMPROVER -- fill undefined pixels NEAR an obstacle, and only there.
//
// Taken from Karjalainen et al. (arXiv 2501.12073), which applies a kernel
// around returns closer than a couple of metres and interpolates the undefined
// pixels inside it, deliberately leaving the far field alone.
//
// WHY THIS IS NOT JUST HOLE FILLING. Every general-purpose depth inpainter this
// project looked at fills the WHOLE image, which converts "I cannot see here"
// into a confident number -- the exact two-state collapse that VoxelMap's three
// states exist to prevent. The far field must stay undefined. What makes the
// near field different is that a hole adjacent to a near return is not an
// absence of information: it is a MATCHING failure on a surface we already know
// is close, because the block matcher found the silhouette and lost the
// interior. Filling it asserts something the neighbouring pixels already prove.
//
// WHAT IT IS ACTUALLY FOR: THIN OBSTACLES. A 3 cm branch at 2 m subtends about
// 8 px at f = 228. A block matcher with an 8 px window returns a handful of
// pixels along its silhouette and nothing in between, so the branch reaches the
// map as a scatter of hits that never accumulate past occThresh -- the cells
// stay UNKNOWN. That matters more than it sounds, because of exactly where
// UNKNOWN is checked in TrajectoryPlanner::plan():
//
//     sphereClear()   rejects OCCUPIED anywhere in the robot ball
//     stateAt()       requires FREE on the CENTRELINE only
//
// So an UNKNOWN branch a third of a metre off the centreline blocks nothing at
// all -- the ball test passes because unknown is not occupied, and the
// centreline test passes because the centreline misses it. The aircraft flies
// through. Turning that scatter into a connected OCCUPIED patch is what makes
// the ball test fire.
//
// THE FILL VALUE IS THE WINDOW MINIMUM, NOT AN INTERPOLATION, and this is a
// safety argument rather than an accuracy one. VoxelMap already carves free
// space only as far as the nearest return in a pixel neighbourhood
// (carveWinPx). A filled pixel that is NEARER than the truth can therefore only
// shorten a carve and lengthen an occupied mark; a filled pixel that is FARTHER
// could carve through the very obstacle that seeded it. Minimum is the only
// choice that is monotone in the safe direction, and it costs nothing:
// separable min filters are two 1-D passes.
//
// THE HONEST COST is phantom obstacles. This fabricates occupancy, and there is
// no version of it that does not. The seed count is the only defence: a hole is
// filled only when at least `minSeeds` genuinely near returns sit inside its
// window, so one speckle cannot grow an obstacle. Sweep it; do not trust it.
//
// O(1) PER PIXEL, deliberately. The obvious implementation scans a (2r+1)^2
// window per hole, which at 640x480 with r = 4 is 24 M operations per frame on
// a Pi. Here the window minimum comes from two separable 1-D min passes and the
// seed count from an integral image, so the whole thing is a handful of passes
// regardless of radius. Radius then stops being a compute decision and becomes
// purely a modelling one, which is what you want from a parameter.
// ---------------------------------------------------------------------------

namespace sim {

struct DepthImproveParams {
    // Only returns closer than this seed a fill. Past it, holes stay holes.
    // This is the whole "near field only" rule and it is the first thing to
    // sweep: too large and the far field's honest emptiness gets fabricated
    // away, too small and thin branches are only caught inside stopping
    // distance.
    float nearM     = 2.0f;
    // Half-width of the fill kernel, pixels. Should be comparable to the
    // matcher window (CamParams::blockPx), because the holes being closed are
    // the ones the matcher's own window produced.
    int   radiusPx  = 4;
    // How many near returns must sit inside the window before a hole is
    // filled. At the default speckle rate (0.4 %) a 9x9 window holds ~0.3
    // speckles, so 6 is roughly twenty sigma clear of filling from noise --
    // deliberately far clear, because the cost of a phantom is a stop and the
    // cost of a miss is a branch.
    int   minSeeds  = 6;
};

// Result of one call, for logging. A collapsing or exploding fill fraction is a
// health signal exactly like the stereo valid fraction is, so it is returned
// rather than discarded.
struct DepthImproveStats {
    long filled  = 0;    // pixels that gained a depth
    long holes   = 0;    // invalid pixels before the call
    long nearPx  = 0;    // valid pixels closer than nearM (the seeds)
};

// In-place on a CV_32F metric depth image, <= 0 meaning invalid.
//
// Never overwrites a pixel that already had a valid depth -- a real measurement
// always beats a fabricated one, including when it disagrees.
DepthImproveStats improveDepth(cv::Mat& depth, const DepthImproveParams& p);

}  // namespace sim
