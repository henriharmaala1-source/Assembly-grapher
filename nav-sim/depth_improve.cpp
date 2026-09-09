#include "depth_improve.hpp"

#include <algorithm>
#include <limits>
#include <vector>

namespace sim {

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

// Sliding-window minimum over one row of `n` values, half-width r, into `dst`.
//
// The van Herk / Gil-Werman two-pass trick: split the row into blocks of
// w = 2r+1, precompute a forward running minimum and a backward one within each
// block, and every window minimum is then the min of exactly two of those.
// Three comparisons per element, independent of r.
//
// A monotone deque would also be O(n) but with a data-dependent branch per
// element; this has none, which is what makes it cheap on a Pi's in-order
// pipeline. Written out rather than pulled from OpenCV because cv::erode on
// CV_32F with an infinity sentinel is both slower and easier to get wrong.
//
// THE ARRAY IS PADDED WITH INFINITY rather than clamped at the edges, and that
// is not a detail. The pairing bwd[lo] / fwd[hi] is only exact when lo and hi
// are exactly w-1 apart; clamping the indices at the border breaks that and
// silently returns the minimum over a WIDER window than asked for -- the sort
// of defect that produces plausible output everywhere and wrong output on the
// two edges where obstacles are largest. Padding keeps the pairing exact and
// gives the correct truncated window for free, since infinity contributes
// nothing. The seed count is truncated the same way by the integral image.
void minFilter1D(const float* src, float* dst, int n, int r,
                 std::vector<float>& pad, std::vector<float>& fwd,
                 std::vector<float>& bwd) {
    if (r <= 0) { std::copy(src, src + n, dst); return; }
    const int w = 2 * r + 1, N = n + 2 * r;
    pad.assign(size_t(N), kInf);
    std::copy(src, src + n, pad.begin() + r);
    fwd.assign(size_t(N), kInf);
    bwd.assign(size_t(N), kInf);
    for (int i = 0; i < N; ++i)
        fwd[size_t(i)] = (i % w == 0) ? pad[size_t(i)]
                                      : std::min(fwd[size_t(i - 1)], pad[size_t(i)]);
    for (int i = N - 1; i >= 0; --i)
        bwd[size_t(i)] = ((i + 1) % w == 0 || i == N - 1)
                       ? pad[size_t(i)] : std::min(bwd[size_t(i + 1)], pad[size_t(i)]);
    // src[i] sits at padded index i+r, so its window [i-r, i+r] is padded
    // [i, i+2r] -- always exactly w-1 wide, at every i.
    for (int i = 0; i < n; ++i)
        dst[i] = std::min(bwd[size_t(i)], fwd[size_t(i + 2 * r)]);
}

// The same filter down COLUMNS, but swept in row-major order over the whole
// image instead of one column at a time.
//
// This is not a micro-optimisation. The obvious version gathers a column into a
// scratch vector, filters it, and scatters it back -- two strided walks of a
// 1.6 MB array per column, which at 848x480 misses cache on essentially every
// access. Measured on the sim: 9-10 ms per frame, against a 0.55 ms trajectory
// planner. The recurrences here run down y, so a whole ROW can be advanced at
// once with contiguous loads, and the entire pass becomes three streaming
// sweeps. Same arithmetic, same result, and the differential test against brute
// force is what keeps that claim honest.
void minFilterVertical(const float* src, float* dst, int W, int H, int r,
                       std::vector<float>& fwd, std::vector<float>& bwd) {
    if (r <= 0) { std::copy(src, src + size_t(W) * H, dst); return; }
    const int w = 2 * r + 1, N = H + 2 * r;
    const size_t stride = size_t(W);
    // resize, not assign: every element below is written before it is read, and
    // an assign() here is two 1.6 MB memsets per frame at 848x480 -- pure
    // bandwidth on a Pi, spent to initialise values nothing ever observes. The
    // one branch that would otherwise leave a row untouched (a block-start row
    // that falls in the padding) fills it explicitly.
    fwd.resize(stride * size_t(N));
    bwd.resize(stride * size_t(N));
    // Padded row i holds source row i-r, or infinity outside.
    auto srcRow = [&](int i) -> const float* {
        return (i >= r && i < H + r) ? src + stride * size_t(i - r) : nullptr;
    };
    for (int i = 0; i < N; ++i) {
        const float* s = srcRow(i);
        float* f = &fwd[stride * size_t(i)];
        if (i % w == 0) {
            if (s) std::copy(s, s + W, f);
            else   std::fill(f, f + W, kInf);
        } else {
            const float* pf = &fwd[stride * size_t(i - 1)];
            if (s) for (int x = 0; x < W; ++x) f[x] = std::min(pf[x], s[x]);
            else   std::copy(pf, pf + W, f);
        }
    }
    for (int i = N - 1; i >= 0; --i) {
        const float* s = srcRow(i);
        float* b = &bwd[stride * size_t(i)];
        if ((i + 1) % w == 0 || i == N - 1) {
            if (s) std::copy(s, s + W, b);
            else   std::fill(b, b + W, kInf);
        } else {
            const float* nb = &bwd[stride * size_t(i + 1)];
            if (s) for (int x = 0; x < W; ++x) b[x] = std::min(nb[x], s[x]);
            else   std::copy(nb, nb + W, b);
        }
    }
    for (int y = 0; y < H; ++y) {
        const float* b = &bwd[stride * size_t(y)];
        const float* f = &fwd[stride * size_t(y + 2 * r)];
        float* d = dst + stride * size_t(y);
        for (int x = 0; x < W; ++x) d[x] = std::min(b[x], f[x]);
    }
}

}  // namespace

DepthImproveStats improveDepth(cv::Mat& depth, const DepthImproveParams& p) {
    DepthImproveStats st;
    if (depth.empty() || depth.type() != CV_32F) return st;
    const int H = depth.rows, W = depth.cols;
    const int r = std::max(0, p.radiusPx);

    // Seed field: the near returns, infinity everywhere else. Everything below
    // is a min or a count over THIS, never over the raw depth -- a far return
    // must not be able to seed a fill or to lower a window minimum.
    std::vector<float> near(size_t(W) * H, kInf);
    // Integral image of the seed mask, one row/col of padding, so a window
    // count is four lookups. int32 is ample: 640x480 is 307 k.
    std::vector<int> ii(size_t(W + 1) * (H + 1), 0);

    for (int y = 0; y < H; ++y) {
        const float* dr = depth.ptr<float>(y);
        int rowSum = 0;
        for (int x = 0; x < W; ++x) {
            const float d = dr[x];
            const bool valid = d > 0.f;
            if (!valid) ++st.holes;
            if (valid && d < p.nearM) {
                near[size_t(y) * W + x] = d;
                ++rowSum; ++st.nearPx;
            }
            ii[size_t(y + 1) * (W + 1) + (x + 1)] =
                ii[size_t(y) * (W + 1) + (x + 1)] + rowSum;
        }
    }
    if (st.nearPx == 0 || st.holes == 0) return st;

    // Separable window minimum of the seed field.
    std::vector<float> tmp(size_t(W) * H), wmin(size_t(W) * H);
    std::vector<float> pad, fwd, bwd;
    for (int y = 0; y < H; ++y)
        minFilter1D(&near[size_t(y) * W], &tmp[size_t(y) * W], W, r, pad, fwd, bwd);
    minFilterVertical(tmp.data(), wmin.data(), W, H, r, fwd, bwd);

    auto seedsIn = [&](int y, int x) {
        const int x0 = std::max(0, x - r), x1 = std::min(W - 1, x + r);
        const int y0 = std::max(0, y - r), y1 = std::min(H - 1, y + r);
        return ii[size_t(y1 + 1) * (W + 1) + (x1 + 1)]
             - ii[size_t(y0)     * (W + 1) + (x1 + 1)]
             - ii[size_t(y1 + 1) * (W + 1) + x0]
             + ii[size_t(y0)     * (W + 1) + x0];
    };

    for (int y = 0; y < H; ++y) {
        float* dr = depth.ptr<float>(y);
        for (int x = 0; x < W; ++x) {
            if (dr[x] > 0.f) continue;                       // never overwrite
            const float m = wmin[size_t(y) * W + x];
            if (!(m < kInf)) continue;                       // no near return in reach
            if (seedsIn(y, x) < p.minSeeds) continue;        // one speckle is not an obstacle
            dr[x] = m;
            ++st.filled;
        }
    }
    return st;
}

}  // namespace sim
