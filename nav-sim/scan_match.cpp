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

    // COARSE, then FINE about the coarse winner. Both are exhaustive over their
    // own grid: this is a correlation, not a descent, so it cannot be trapped by
    // a local minimum inside the searched box -- only by one outside it, which
    // is why the box is sized to the prediction error rather than made generous.
    auto sweep = [&](float cx, float cy, float cz, float step, float range,
                     float& bx, float& by, float& bz, float& best) {
        const int n = int(range / step);
        best = -1e30f;
        bx = cx; by = cy; bz = cz;
        for (int i = -n; i <= n; ++i)
            for (int j = -n; j <= n; ++j)
                for (int k = -n; k <= n; ++k) {
                    const float ox = cx + i * step, oy = cy + j * step,
                                oz = cz + k * step;
                    const float s = scoreAt(map, pts, ox, oy, oz, nullptr);
                    if (s > best) { best = s; bx = ox; by = oy; bz = oz; }
                }
    };

    float bx, by, bz, best;
    sweep(0.f, 0.f, 0.f, p_.coarseStepM, p_.coarseRangeM, bx, by, bz, best);
    sweep(bx, by, bz, p_.fineStepM, p_.fineRangeM, bx, by, bz, best);

    int hits = 0;
    out.score = scoreAt(map, pts, bx, by, bz, &hits);
    out.hitFrac = float(hits) / float(out.points);
    out.dE = bx; out.dN = by; out.dU = bz;

    // OBSERVABILITY. Step one fine step off the optimum along each axis and see
    // what it costs. A corridor costs nothing along its length, and that axis
    // must be reported unobserved rather than averaged in.
    const float peak = std::fabs(out.score) > 1e-6f ? std::fabs(out.score) : 1.f;
    const float d = p_.fineStepM;
    const float off[3][3] = {{d, 0.f, 0.f}, {0.f, d, 0.f}, {0.f, 0.f, d}};
    for (int a = 0; a < 3; ++a) {
        const float sp = scoreAt(map, pts, bx + off[a][0], by + off[a][1],
                                 bz + off[a][2], nullptr);
        const float sm = scoreAt(map, pts, bx - off[a][0], by - off[a][1],
                                 bz - off[a][2], nullptr);
        // Both neighbours are <= the optimum by construction; the shallower
        // side is the honest measure of how well the axis is pinned.
        out.curv[a] = (out.score - std::max(sp, sm)) / peak;
        out.axisObserved[a] = out.curv[a] >= p_.minCurvatureFrac;
    }

    out.valid = out.hitFrac >= p_.minHitFrac
             && out.axisObserved[0] && out.axisObserved[1] && out.axisObserved[2];
    return out;
}

}  // namespace sim
