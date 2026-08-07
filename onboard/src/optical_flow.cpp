#include "optical_flow.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace track {

static constexpr float FMAX = std::numeric_limits<float>::max();

float OpticalFlow::patchVar(const GrayFrame& g, int cx, int cy) const {
    float s = 0, s2 = 0; int n = 0;
    for (int j = -patch; j <= patch; ++j)
        for (int i = -patch; i <= patch; ++i) {
            const float v = g.d[(cy + j) * g.w + (cx + i)];
            s += v; s2 += v * v; ++n;
        }
    const float mean = s / n;
    return s2 / n - mean * mean;
}

// SSD with EARLY EXIT: the caller passes the best score so far and we bail out of
// the pixel loop the moment the running sum exceeds it. Most candidate positions
// are obviously wrong within a few rows, so this typically cuts the inner loop
// several-fold -- and this loop dominated the whole tracker (measured on Android:
// flow was 79 % of per-frame cost, 3.9x the entire NCC fusion).
float OpticalFlow::ssd(const GrayFrame& a, int ax, int ay,
                       const GrayFrame& b, int bx, int by, float bail) const {
    if (bx - patch < 0 || by - patch < 0 || bx + patch >= b.w || by + patch >= b.h)
        return FMAX;
    float s = 0;
    for (int j = -patch; j <= patch; ++j) {
        const int ao = (ay + j) * a.w + ax;
        const int bo = (by + j) * b.w + bx;
        for (int i = -patch; i <= patch; ++i) {
            const float d = a.d[ao + i] - b.d[bo + i];
            s += d * d;
        }
        if (s >= bail) return FMAX;                 // cannot win -- stop early
    }
    return s;
}

float OpticalFlow::medianOf(const std::vector<float>& src, int n) {
    std::copy(src.begin(), src.begin() + n, sortBuf_.begin());
    std::sort(sortBuf_.begin(), sortBuf_.begin() + n);
    return sortBuf_[n / 2];
}

void OpticalFlow::estimate(const GrayFrame& prev, const GrayFrame& cur,
                           float exCx, float exCy, float exHalf,
                           float& odx, float& ody) {
    consensus_ = 0.f;
    odx = ody = 0.f;
    const int w = prev.w, h = prev.h;
    const int m = patch + search;
    if (w <= 2 * m || h <= 2 * m) return;

    const int cap = gridX * gridY;
    if ((int)dxs_.size() < cap) { dxs_.resize(cap); dys_.resize(cap); sortBuf_.resize(cap); }

    int nPts = 0;
    for (int gy = 1; gy <= gridY; ++gy)
        for (int gx = 1; gx <= gridX; ++gx) {
            const int cx = m + (w - 2 * m) * gx / (gridX + 1);
            const int cy = m + (h - 2 * m) * gy / (gridY + 1);
            if (exHalf > 0.f && std::fabs(cx - exCx) <= exHalf
                             && std::fabs(cy - exCy) <= exHalf)
                continue;                                    // skip the target region
            if (patchVar(prev, cx, cy) < minVar) continue;   // skip flat

            float best = FMAX; int bdx = 0, bdy = 0;
            for (int oy = -search; oy <= search; ++oy)
                for (int ox = -search; ox <= search; ++ox) {
                    const float s = ssd(prev, cx, cy, cur, cx + ox, cy + oy, best);
                    if (s < best) { best = s; bdx = ox; bdy = oy; }
                }
            if (best == FMAX) continue;                      // no valid forward match

            float bestBack = FMAX; int bbx = 0, bby = 0;
            for (int oy = -fbSearch; oy <= fbSearch; ++oy)
                for (int ox = -fbSearch; ox <= fbSearch; ++ox) {
                    const float s = ssd(cur, cx + bdx, cy + bdy, prev,
                                        cx + ox, cy + oy, bestBack);
                    if (s < bestBack) { bestBack = s; bbx = ox; bby = oy; }
                }
            if (std::hypot(float(bbx), float(bby)) > fbMaxError) continue;

            dxs_[nPts] = float(bdx); dys_[nPts] = float(bdy); ++nPts;
        }

    if (nPts < 4) return;
    // Median on SCRATCH copies so the dxs/dys pairing survives for consensus.
    const float mdx = medianOf(dxs_, nPts);
    const float mdy = medianOf(dys_, nPts);
    int inl = 0;
    for (int i = 0; i < nPts; ++i)
        if (std::hypot(dxs_[i] - mdx, dys_[i] - mdy) <= 2.f) ++inl;
    consensus_ = float(inl) / nPts;
    odx = mdx; ody = mdy;
}

}  // namespace track
