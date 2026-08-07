#pragma once

#include <vector>

#include "gray_frame.hpp"

// ---------------------------------------------------------------------------
// Sparse optical flow -> global (ego-motion) translation estimate. Direct port
// of track/OpticalFlow.kt.
//
// Samples a grid of TEXTURED points, matches each into the next frame by a small
// SSD search, and takes the MEDIAN displacement. The median is the key: an
// independently-moving object (or a bad match) is an outlier the median ignores,
// so what is left is the camera's own motion. At 50-800 m the scene is far
// (little parallax), so one global translation models yaw/pitch pan well.
// ---------------------------------------------------------------------------

namespace track {

class OpticalFlow {
public:
    int   patch      = 5;      // half-size of the match patch
    int   search     = 12;     // half-size of the search window
    int   gridX      = 8, gridY = 6;
    float minVar     = 40.f;   // skip flat (ambiguous) patches
    int   fbSearch   = 4;      // backward-match half-window (TLD-style FB check)
    float fbMaxError = 1.5f;   // discard a point if the round trip exceeds this

    // Fraction of grid points agreeing with the median of the last estimate() --
    // high on a rigid camera pan, low under noise or a large independently
    // moving occluder. Trust the translation only when this is high.
    float consensus() const { return consensus_; }

    // Global translation mapping `prev` onto `cur`, px (0,0 if weak).
    //
    // (exCx,exCy,exHalf) EXCLUDE grid points inside the current tracked box -- if
    // the target is a large fraction of the frame its own motion could otherwise
    // win the median vote with high consensus even though it is not camera pan.
    //
    // Each surviving point is also checked FORWARD-BACKWARD: the found position
    // is re-matched back toward its origin, and a round trip that does not return
    // close to the start means the match was ambiguous (aliased texture, not real
    // motion) and is discarded before it pollutes the median or the consensus.
    void estimate(const GrayFrame& prev, const GrayFrame& cur,
                  float exCx, float exCy, float exHalf, float& odx, float& ody);

private:
    float patchVar(const GrayFrame& g, int cx, int cy) const;
    float ssd(const GrayFrame& a, int ax, int ay,
              const GrayFrame& b, int bx, int by, float bail) const;
    float medianOf(const std::vector<float>& src, int n);

    float consensus_ = 0.f;
    std::vector<float> dxs_, dys_, sortBuf_;
};

}  // namespace track
