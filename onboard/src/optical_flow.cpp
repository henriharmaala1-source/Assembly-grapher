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


// --- CoastFlow --------------------------------------------------------------

void CoastFlow::buildPyr(const GrayFrame& g, std::vector<std::vector<float>>& out,
                         std::vector<int>& w, std::vector<int>& h) const {
    out.clear(); w.clear(); h.clear();
    out.emplace_back(g.d, g.d + size_t(g.w) * g.h);
    w.push_back(g.w); h.push_back(g.h);
    for (int i = 1; i < levels; ++i) {
        const int aw = w.back(), ah = h.back();
        const int nh = (ah / 2) * 2, nw = (aw / 2) * 2;
        if (nh < 16 || nw < 16) break;
        const std::vector<float>& a = out.back();
        std::vector<float> b(size_t(nw / 2) * (nh / 2));
        for (int y = 0; y < nh / 2; ++y)
            for (int x = 0; x < nw / 2; ++x)
                b[size_t(y) * (nw / 2) + x] = 0.25f *
                    (a[size_t(2*y) * aw + 2*x]     + a[size_t(2*y+1) * aw + 2*x] +
                     a[size_t(2*y) * aw + 2*x + 1] + a[size_t(2*y+1) * aw + 2*x + 1]);
        out.push_back(std::move(b));
        w.push_back(nw / 2); h.push_back(nh / 2);
    }
}

// Coarse-to-fine block match, returning the level-0 displacement.
bool CoastFlow::matchPoint(const std::vector<std::vector<float>>& pa,
                           const std::vector<int>& aw, const std::vector<int>& ah,
                           const std::vector<std::vector<float>>& pb,
                           const std::vector<int>& bw, const std::vector<int>& bh,
                           int cx, int cy, int& odx, int& ody) const {
    const int p = patch;
    const int L = int(pa.size()) - 1;
    int dx = 0, dy = 0;
    for (int lv = L; lv >= 0; --lv) {
        const int sc = 1 << lv;
        const std::vector<float>& a = pa[lv];
        const std::vector<float>& b = pb[lv];
        const int px = int(std::lround(double(cx) / sc));
        const int py = int(std::lround(double(cy) / sc));
        // Full search only at the COARSEST level; finer levels just refine +-2,
        // which is what makes this cheaper than the flat search it replaces.
        const int rad = (lv == L)
            ? std::max(1, int(std::ceil(double(search) / sc))) : 2;
        if (px - p < 0 || py - p < 0 || px + p >= aw[lv] || py + p >= ah[lv])
            return false;
        const int x0 = px + dx - p - rad, y0 = py + dy - p - rad;
        const int x1 = px + dx + p + rad + 1, y1 = py + dy + p + rad + 1;
        if (x0 < 0 || y0 < 0 || x1 > bw[lv] || y1 > bh[lv]) return false;
        float best = FMAX; int bi = 0, bj = 0;
        for (int oy = 0; oy <= 2 * rad; ++oy)
            for (int ox = 0; ox <= 2 * rad; ++ox) {
                float s = 0.f;
                for (int j = -p; j <= p; ++j) {
                    const float* ar = &a[size_t(py + j) * aw[lv] + (px - p)];
                    const float* br = &b[size_t(y0 + oy + j + p) * bw[lv] + (x0 + ox)];
                    for (int i = 0; i <= 2 * p; ++i) {
                        const float d = ar[i] - br[i];
                        s += d * d;
                    }
                    if (s >= best) break;              // cannot win -- stop early
                }
                if (s < best) { best = s; bi = ox; bj = oy; }
            }
        dx += bi - rad; dy += bj - rad;
        if (lv > 0) { dx *= 2; dy *= 2; }
    }
    odx = dx; ody = dy;
    return true;
}

void CoastFlow::seed(const GrayFrame& g, float bcx, float bcy, float bsize) {
    pts_.clear();
    const float hbox = bsize * 0.5f * inner;
    const int m = patch + search;
    const int W = g.w, H = g.h;
    auto clampi = [](int v, int lo, int hi) { return std::min(std::max(v, lo), hi); };
    if (W - m - 1 < m || H - m - 1 < m) { clear(); return; }
    const int x0 = clampi(int(bcx - hbox), m, W - m - 1);
    const int x1 = clampi(int(bcx + hbox), m, W - m - 1);
    const int y0 = clampi(int(bcy - hbox), m, H - m - 1);
    const int y1 = clampi(int(bcy + hbox), m, H - m - 1);
    if (x1 - x0 < 4 || y1 - y0 < 4) { clear(); return; }

    const int gN = std::max(2, int(std::sqrt(double(maxPts))) + 1);
    const int p = patch;
    std::vector<std::pair<float, std::pair<int,int>>> cand;
    for (int gy = 0; gy < gN; ++gy)
        for (int gx = 0; gx < gN; ++gx) {
            const int cx = x0 + (gN > 1 ? (x1 - x0) * gx / (gN - 1) : 0);
            const int cy = y0 + (gN > 1 ? (y1 - y0) * gy / (gN - 1) : 0);
            // Min-eigenvalue (Shi-Tomasi) gate: a flat patch is ambiguous to
            // match, and matching it would poison the median.
            double sxx = 0, syy = 0, sxy = 0; int n = 0;
            for (int j = -p + 1; j <= p - 1; ++j)
                for (int i = -p + 1; i <= p - 1; ++i) {
                    const int o = (cy + j) * W + (cx + i);
                    const float ggx = 0.5f * (g.d[o + 1] - g.d[o - 1]);
                    const float ggy = 0.5f * (g.d[o + W] - g.d[o - W]);
                    sxx += ggx * ggx; syy += ggy * ggy; sxy += ggx * ggy; ++n;
                }
            if (!n) continue;
            const double a = sxx / n, b = syy / n, c = sxy / n;
            const double tr = a + b, det = a * b - c * c;
            const double v = 0.5 * (tr - std::sqrt(std::max(0.0, tr * tr - 4 * det)));
            if (v < minEig) continue;
            cand.push_back({float(v), {cx, cy}});
        }
    if ((int)cand.size() < minPts) { clear(); return; }
    // Strongest-textured first, so a small point budget spends itself on the
    // patches that can actually be matched unambiguously.
    std::sort(cand.begin(), cand.end(),
              [](const auto& l, const auto& r) { return l.first > r.first; });
    if ((int)cand.size() > maxPts) cand.resize(maxPts);
    for (const auto& c : cand) pts_.push_back(c.second);
    buildPyr(g, prev_, pw_, ph_);
}

bool CoastFlow::step(const GrayFrame& g, float& dx, float& dy) {
    if ((int)pts_.size() < minPts || prev_.empty()) return false;
    if (pw_.empty() || pw_[0] != g.w || ph_[0] != g.h) { clear(); return false; }
    std::vector<std::vector<float>> cur; std::vector<int> cw, ch;
    buildPyr(g, cur, cw, ch);

    std::vector<float> ds, dsy;
    std::vector<std::pair<int,int>> keep;
    for (const auto& q : pts_) {
        int mx = 0, my = 0;
        if (!matchPoint(prev_, pw_, ph_, cur, cw, ch, q.first, q.second, mx, my))
            continue;
        ds.push_back(float(mx)); dsy.push_back(float(my));
        keep.push_back({q.first + mx, q.second + my});
    }
    if ((int)ds.size() < minPts) { clear(); return false; }
    auto med = [](std::vector<float> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    dx = med(ds); dy = med(dsy);
    pts_ = keep;
    prev_ = std::move(cur); pw_ = cw; ph_ = ch;
    return true;
}

}  // namespace track
