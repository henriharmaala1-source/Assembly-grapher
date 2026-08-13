// The far field as BEARING SPACE rather than as cubes.
//
// WHY THIS EXISTS. Stereo uncertainty is anisotropic and the anisotropy grows
// with range: along the ray it is Z^2*sigma/(f*B), across it is Z/f, and the
// ratio is exactly Z*sigma/B -- 10:1 at 2 m, 50:1 at 10 m, 100:1 at 20 m on a
// 50 mm baseline. A CUBE has to be sized for the worse of the two, so a voxel
// honest in range at 20 m is 8 m wide and throws away the 4.5 cm of lateral
// detail the sensor still has. Measured against the 1.0 m far rung: eleven
// independent bearings across 87 degrees, against 848 here, for less time.
//
// It is also the shape ArduPilot's own avoidance layer wants -- OBSTACLE_DISTANCE
// is 72 distances by bearing -- and the shape `POSE_AND_OPENNESS_PLAN.md` section
// 1 already designed for openness scoring. Three uses, one object.
//
// WHAT IT IS NOT. It is not a replacement for the voxel map, and wiring it into
// the safety path would be a mistake. It has no free space and no volume: a bin
// says "the nearest thing on this bearing is at r", which cannot answer "is this
// robot-sized tube clear", and it cannot represent two surfaces along one
// bearing. The near field keeps its cubes for exactly those reasons. This
// replaces the COARSE RUNG, whose job was only ever awareness.
#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"

namespace sim {

struct BearingFieldParams {
    // 1 deg bins. The sensor resolves 0.10 deg per column at 848 px over 87
    // deg, so this is deliberately coarser than the measurement: a bin narrower
    // than the stereo matcher's own correlation window measures noise rather
    // than structure. 1 deg at 10 m is 17 cm, the scale of a trunk, and it is
    // still THIRTY-THREE times finer than the 1 m voxel rung it replaces.
    int   nAz = 360;            // over 360 deg -- 1 deg bins
    int   nEl = 48;             // over elMin..elMax
    float elMinDeg = -40.f;
    float elMaxDeg =  40.f;
    // Persistence. The map remembers what the current frame cannot see, which
    // is most of the value of a map at all -- but a bearing bin has no way to
    // know it has been vacated, so it must forget on a clock instead. Frames,
    // not seconds, because the caller owns the frame rate.
    int   forgetFrames = 90;
    // A bin must be backed by at least this many pixels. One stereo outlier is
    // the nearest sample in its bin by construction, so a bare minimum is
    // maximally sensitive to the one thing this sensor produces most.
    int   minSamples = 4;
    // FILL FRACTION -- "did most of what looked that way actually come back?"
    //
    // `minSamples` counts VALID returns only, so a bin with four good pixels out
    // of four hundred looking that way passes it. That is exactly what empty sky
    // is: a few thousand no-returns and a handful of spurious matches on cloud
    // edge or sensor noise. The bin then claims a surface, holds it for
    // forgetFrames, and the pane grows a ceiling that is not there.
    //
    // So a bin must also be BACKED: of the pixels pointing into it, this
    // fraction must have returned something. Sky is a few per cent and fails;
    // foliage is a third and passes; a wall is nearly all of it. This is the
    // difference between drawing the map and drawing the frame's silhouette of
    // found depth, and it is the second question after "how far".
    float minFillFrac = 0.25f;
    // CONFIRMATION, the bearing-space equivalent of log-odds. The voxel map
    // needs repeated hits before a cell crosses occThresh, and that is most of
    // why it does not speckle. A bearing field that believes the first frame
    // has no such filter, so it needs its own: a bin must be seen at
    // consistently the same range for this many frames before it is reported.
    // Without it the pane fills with single-bin outliers that the depth image
    // genuinely contains and the voxel map genuinely rejects -- which would
    // make the comparison flatter cubes for the wrong reason.
    int   confirmFrames = 2;
    float agreeM = 0.30f;       // and this fraction of range, whichever larger
    float agreeFrac = 0.10f;
    float minRangeM = 0.2f;
    float maxRangeM = 30.f;
};

class BearingField {
public:
    void init(const BearingFieldParams& p);
    const BearingFieldParams& params() const { return p_; }

    // Fold one depth frame in. Nearest valid return per (az, el) bin, in WORLD
    // bearing, so the field accumulates correctly as the aircraft yaws.
    //
    // `stride` samples every Nth pixel. Unlike the voxel mapper this is not a
    // cost/accuracy trade in the same way -- there is no DDA, the whole pass is
    // one linear scan of the depth image -- so it can afford stride 1.
    void update(const cv::Mat& depth, const DepthCamera& cam,
                const CamPose& pose, int stride = 1);

    // Nearest surface on a bearing, or < 0 if that bin has nothing.
    float rangeAt(float azDeg, float elDeg) const;

    // Distances by bearing at the aircraft's own altitude band, in the shape
    // ArduPilot's OBSTACLE_DISTANCE wants: `bins` entries, clockwise from the
    // vehicle's nose, in metres, negative where nothing is known.
    std::vector<float> obstacleDistance(float yawDeg, int bins = 72) const;

    // Bins carrying anything, and how many bins there are. For the log line.
    void occupancy(int& live, int& total) const;

    // First-person render, deliberately through the SAME projection, the same
    // height colour key and the same haze as VoxelMap::fpvImageWH -- because
    // the only honest way to compare two representations is to change nothing
    // else. There are no cube faces here, so face shading is replaced by the
    // surface's local slope in bearing space, which is what gives a wall its
    // form instead of leaving it a flat wash.
    // `minRange` lets the caller band it exactly as a voxel level is banded, so
    // it can be composited BEYOND the fine map rather than competing with it.
    static cv::Mat render(const BearingField& bf,
                          float yawDeg, float pitchDeg,
                          int outW, int outH, float hfovDeg,
                          float minRange, float maxRange, float eyeAltM,
                          cv::Mat* hitMask = nullptr);

private:
    int azIdx(float azDeg) const;
    int elIdx(float elDeg) const;

    static constexpr float kNone = 1e9f;

    void rebuildTable(const DepthCamera& cam, float pitchDeg, float rollDeg,
                      int rows, int cols);

    // Per-pixel BODY-frame bin index, built once. A yaw rotation is an exact
    // index shift on a uniform azimuth grid, so the whole per-frame cost
    // collapses to a table lookup and a comparison -- no trigonometry at all.
    // That is the difference between 15.8 ms and something worth shipping.
    std::vector<int32_t> bin_;
    int   tblRows_ = 0, tblCols_ = 0;
    float tblPitch_ = 1e9f, tblRoll_ = 1e9f;

    BearingFieldParams p_;
    std::vector<float>   r_;      // nearest range last seen, or kNone
    std::vector<float>   cur_;    // this frame's minimum per bin
    std::vector<int32_t> cnt_;    // valid returns that voted for it
    std::vector<int32_t> look_;   // and how many pixels looked that way at all
    std::vector<int32_t> conf_;   // consecutive frames agreeing on r_
    std::vector<int32_t> age_;    // frame index last written
    int32_t frame_ = 0;
};

}  // namespace sim
