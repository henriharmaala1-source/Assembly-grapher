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

// ---------------------------------------------------------------------------
// CoastFlow -- LK COAST ASSIST, portable (SSD) variant.
//
// When the appearance match fails the tracker coasts on constant velocity: it
// stops looking at the image and extrapolates. That is fine for two or three
// frames and wrong for twenty -- a manoeuvring target's whole problem is that
// its velocity is NOT constant, which is exactly when the match is most likely
// to have failed in the first place.
//
// This tracks textured points from inside the last good box by coarse-to-fine
// SSD block matching and takes the MEDIAN displacement. It does not care what
// the target looks like, so it survives the pose change / partial occlusion /
// motion blur that broke the template.
//
// WHY A PYRAMID AND NOT A FLAT SEARCH. On a target crossing ~18 px per frame
// with the motion blur that comes with it, a single-scale match must search
// +-20 px (1681 positions) and still fails, because at full resolution the
// blurred target no longer matches its own sharp template. Halving twice turns
// 18 px of motion into 4.5 px and blur into structure. It is also CHEAPER:
// ~5 px of search at the coarsest level plus two +-2 refinements is ~12k
// operations per point against ~200k for the flat search it replaces.
//
// Measured (paired ensemble, 14 draws): +2.35 +/- 0.97 points, t = 2.43. The
// per-clip SIGN is what justifies it, not the mean -- manoeuvre better in 12 of
// 14, worst-case better in 12 of 14, camera shake WORSE in 13 of 14, which is
// exactly what the mechanism predicts.
//
// This is the SSD variant deliberately kept in the reference so the phone and
// the Pi can run the same thing; the desktop's OpenCV Shi-Tomasi + pyramidal LK
// path is not portable and is not what this mirrors.
// ---------------------------------------------------------------------------
class CoastFlow {
public:
    int   patch   = 5;     // SSD_PATCH  -- half-size of the match patch
    int   search  = 12;    // SSD_SEARCH -- half-size at the coarsest level
    int   levels  = 3;     // SSD_LEVELS
    float minEig  = 8.f;   // SSD_MINEIG -- min-eigenvalue corner gate
    int   maxPts  = 12;    // LK_MAX_PTS -- median over 12, chosen for cost
    int   minPts  = 4;     // LK_MIN_PTS -- fewer survivors = no verdict
    float inner   = 1.0f;  // LK_INNER   -- seed from this fraction of the box

    // Pick textured points inside the box and remember the frame they came from.
    void seed(const GrayFrame& g, float bcx, float bcy, float bsize);
    // Median displacement of the surviving points, prev frame -> g. False = no
    // verdict (too few survivors); the point set is dropped so the caller
    // re-seeds rather than trusting a decayed one.
    bool step(const GrayFrame& g, float& dx, float& dy);

    bool ready() const { return !pts_.empty() && !prev_.empty(); }
    void clear() { pts_.clear(); prev_.clear(); pw_.clear(); ph_.clear(); }

private:
    // 2x2 box-average pyramid. Decimation and an add are all the onboard side
    // needs -- no filter kernel, no interpolation.
    void buildPyr(const GrayFrame& g, std::vector<std::vector<float>>& out,
                  std::vector<int>& w, std::vector<int>& h) const;
    bool matchPoint(const std::vector<std::vector<float>>& pa,
                    const std::vector<int>& aw, const std::vector<int>& ah,
                    const std::vector<std::vector<float>>& pb,
                    const std::vector<int>& bw, const std::vector<int>& bh,
                    int cx, int cy, int& odx, int& ody) const;

    std::vector<std::vector<float>> prev_;   // pyramid of the seed/last frame
    std::vector<int> pw_, ph_;
    std::vector<std::pair<int,int>> pts_;
};

}  // namespace track
