#include "scan_match.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {

namespace {

struct Pt { float x, y, z; };

// CORRELATION with the occupied set: a point votes +1 when it lands on a cell
// the map calls OCCUPIED, and nothing otherwise. This is Olson's correlative
// score, and getting here took two wrong versions worth recording.
//
// (1) SUM OF LOG-ODDS. Log-odds grows without bound inside a solid, so the
//     score is maximised by sliding into the densest thing in view rather than
//     by aligning with it.
//
// (2) OCCUPIED +1 / FREE -1 / UNKNOWN 0. This looks like the project's own
//     three-state rule and is exactly backwards for matching: it makes UNKNOWN
//     *better* than FREE for a point that claims to be a surface, so the best
//     offset is the one that slides the whole cloud out of the mapped volume
//     into unobserved space. Measured: a correct guess was corrected by 0.35 m,
//     0.33 m of it straight up out of the map.
//
// Scoring only corroboration has neither failure. A cell the map has no opinion
// about contributes nothing, which is the honest weight for no evidence -- and
// crucially it is the SAME weight as disagreement, so there is no gradient
// pointing out of the map.
inline float scoreAt(const VoxelMap& map, const std::vector<Pt>& pts,
                     float ox, float oy, float oz, int* hits) {
    int h = 0;
    for (const Pt& p : pts)
        if (map.stateAt(p.x + ox, p.y + oy, p.z + oz) == VoxelMap::OCCUPIED) ++h;
    if (hits) *hits = h;
    return float(h);
}

// Rotate the cloud about the VERTICAL THROUGH THE CAMERA, which is what a yaw
// error actually is -- a heading mistake pivots the whole observation about the
// vehicle, it does not slide it. Translating instead would search the wrong
// manifold and find a plausible-looking answer on it.
//
// Sense follows the project's convention: yaw is clockwise from North, so a
// positive offset turns a North-pointing ray toward East. Checked against
// DepthCamera::camToWorld rather than reasoned about twice.
inline void rotateCloud(const std::vector<Pt>& in, float cx, float cy,
                        float yawDeg, std::vector<Pt>& out) {
    out.resize(in.size());
    const float t = yawDeg * 3.14159265f / 180.f;
    const float c = std::cos(t), s = std::sin(t);
    for (size_t i = 0; i < in.size(); ++i) {
        const float dx = in[i].x - cx, dy = in[i].y - cy;
        out[i].x = cx + dx * c + dy * s;
        out[i].y = cy - dx * s + dy * c;
        out[i].z = in[i].z;
    }
}

}  // namespace

ScanMatch ScanMatcher::match(const VoxelMap& map, const cv::Mat& depth,
                             const DepthCamera& cam, const CamPose& guess) const {
    ScanMatch out;
    if (depth.empty() || depth.type() != CV_32F) return out;

    const float maxR = (p_.maxRangeM > 0.f) ? p_.maxRangeM : map.params().maxIntegM;

    // Sample the frame into a world-frame cloud AT THE GUESS. The search then
    // translates that cloud rigidly, which is valid precisely because attitude
    // is not being solved for.
    std::vector<Pt> pts;
    pts.reserve(size_t((depth.rows / std::max(1, p_.strideY) + 1) *
                       (depth.cols / std::max(1, p_.strideX) + 1)));
    for (int v = 0; v < depth.rows; v += std::max(1, p_.strideY)) {
        const float* row = depth.ptr<float>(v);
        for (int u = 0; u < depth.cols; u += std::max(1, p_.strideX)) {
            const float z = row[u];
            if (!(z > p_.minRangeM) || z > maxR) continue;
            float dx, dy, dz;
            cam.rayFor(guess, u, v, dx, dy, dz);
            pts.push_back({guess.e + dx * z, guess.n + dy * z, guess.u + dz * z});
        }
    }
    out.points = int(pts.size());
    if (out.points < p_.minPoints) return out;

    // COORDINATE DESCENT over translation and yaw, coarse then fine.
    //
    // A joint 4-D sweep would be honest and far too slow: the coarse
    // translation grid is already 9x9x9, and multiplying it by a yaw axis
    // multiplies the cost. Alternating is standard and cheap, and it is safe
    // here for a specific reason -- over one frame the gyro's yaw error is a
    // fraction of a degree, which at 3 m is under 3 cm of apparent lateral
    // motion. The translation stage is therefore not meaningfully biased by
    // starting at the predicted heading.
    std::vector<Pt> rot;
    auto sweepT = [&](const std::vector<Pt>& cloud, float cx, float cy, float cz,
                      float step, float range,
                      float& bx, float& by, float& bz, float& best) {
        const int n = int(range / step);
        best = -1e30f; bx = cx; by = cy; bz = cz;
        for (int i = -n; i <= n; ++i)
            for (int j = -n; j <= n; ++j)
                for (int k = -n; k <= n; ++k) {
                    const float ox = cx + i * step, oy = cy + j * step, oz = cz + k * step;
                    const float sc = scoreAt(map, cloud, ox, oy, oz, nullptr);
                    if (sc > best) { best = sc; bx = ox; by = oy; bz = oz; }
                }
    };
    auto sweepYaw = [&](float ox, float oy, float oz, float centreDeg,
                        float step, float range, float& bestYaw, float& best) {
        const int n = int(range / step);
        best = -1e30f; bestYaw = centreDeg;
        for (int i = -n; i <= n; ++i) {
            const float y = centreDeg + i * step;
            rotateCloud(pts, guess.e, guess.n, y, rot);
            const float sc = scoreAt(map, rot, ox, oy, oz, nullptr);
            if (sc > best) { best = sc; bestYaw = y; }
        }
    };

    float bx, by, bz, best, byaw = 0.f;
    sweepT(pts, 0.f, 0.f, 0.f, p_.coarseStepM, p_.coarseRangeM, bx, by, bz, best);
    if (p_.yawRangeDeg > 0.f) {
        sweepYaw(bx, by, bz, 0.f, p_.yawStepDeg * 2.f, p_.yawRangeDeg, byaw, best);
        rotateCloud(pts, guess.e, guess.n, byaw, rot);
        sweepT(rot, bx, by, bz, p_.fineStepM, p_.fineRangeM, bx, by, bz, best);
        sweepYaw(bx, by, bz, byaw, p_.yawStepDeg, p_.yawStepDeg * 2.f, byaw, best);
        // THE DEADBAND. Compare the winner against holding the predicted
        // heading, at the same translation, and keep the prediction unless the
        // rotation genuinely pays. See yawMinGainFrac.
        const float atZero = scoreAt(map, pts, bx, by, bz, nullptr);
        if (best <= atZero * (1.f + p_.yawMinGainFrac)) byaw = 0.f;
    } else {
        sweepT(pts, bx, by, bz, p_.fineStepM, p_.fineRangeM, bx, by, bz, best);
    }

    // Everything below scores against the cloud AT THE WINNING HEADING.
    rotateCloud(pts, guess.e, guess.n, byaw, rot);
    const std::vector<Pt>& fin = rot;

    int hits = 0;
    out.score = scoreAt(map, fin, bx, by, bz, &hits);
    out.hitFrac = float(hits) / float(out.points);
    out.dE = bx; out.dN = by; out.dU = bz; out.dYawDeg = byaw;

    // OBSERVABILITY. Step one fine step off the optimum along each axis and see
    // what it costs. A corridor costs nothing along its length; a rotationally
    // symmetric room costs nothing in yaw. Either must be reported, not averaged
    // into a number that looks like a measurement.
    const float peak = std::fabs(out.score) > 1e-6f ? std::fabs(out.score) : 1.f;
    const float d = (p_.curvStepM > 0.f) ? p_.curvStepM : map.params().cell;
    const float off[3][3] = {{d, 0.f, 0.f}, {0.f, d, 0.f}, {0.f, 0.f, d}};
    for (int a = 0; a < 3; ++a) {
        const float sp = scoreAt(map, fin, bx + off[a][0], by + off[a][1], bz + off[a][2], nullptr);
        const float sm = scoreAt(map, fin, bx - off[a][0], by - off[a][1], bz - off[a][2], nullptr);
        out.curv[a] = (out.score - std::max(sp, sm)) / peak;
    }
    if (p_.yawRangeDeg > 0.f) {
        std::vector<Pt> tmp;
        rotateCloud(pts, guess.e, guess.n, byaw + p_.yawStepDeg, tmp);
        const float sp = scoreAt(map, tmp, bx, by, bz, nullptr);
        rotateCloud(pts, guess.e, guess.n, byaw - p_.yawStepDeg, tmp);
        const float sm = scoreAt(map, tmp, bx, by, bz, nullptr);
        out.curv[3] = (out.score - std::max(sp, sm)) / peak;
    }

    // Compare against the best-constrained TRANSLATION axis. Yaw is judged on
    // the same scale deliberately: a heading only as well pinned as a
    // degenerate axis is not a heading fix.
    const float bestTrans = std::max(out.curv[0], std::max(out.curv[1], out.curv[2]));
    const float gate = std::max(p_.minCurvatureFrac, p_.relCurvatureFrac * bestTrans);
    for (int a = 0; a < 4; ++a) out.axisObserved[a] = out.curv[a] >= gate;

    out.valid = out.hitFrac >= p_.minHitFrac
             && out.axisObserved[0] && out.axisObserved[1] && out.axisObserved[2];
    return out;
}

}  // namespace sim
