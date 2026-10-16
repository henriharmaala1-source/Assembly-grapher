#include "nav_map.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void LocalMap::reset() {
    W_ = std::max(1, (int)std::lround(p_.sizeM / p_.cellM));
    H_ = W_;
    // Centre the grid on (originE, originN): cell (0,0)'s centre sits half the
    // span down-left of it.
    e0_ = p_.originE - 0.5f * (W_ - 1) * p_.cellM;
    n0_ = p_.originN - 0.5f * (H_ - 1) * p_.cellM;
    log_.assign((size_t)W_ * H_, 0.f);
    dist_.assign((size_t)W_ * H_, -1);
    blocked_.assign((size_t)W_ * H_, 0);
}

void LocalMap::worldToCell_(float e, float n, int& ix, int& iy) const {
    ix = (int)std::lround((e - e0_) / p_.cellM);
    iy = (int)std::lround((n - n0_) / p_.cellM);
}
void LocalMap::cellToWorld_(int ix, int iy, float& e, float& n) const {
    e = e0_ + ix * p_.cellM;
    n = n0_ + iy * p_.cellM;
}

float LocalMap::logodds(int ix, int iy) const {
    return inBounds_(ix, iy) ? log_[idx_(ix, iy)] : 0.f;
}
bool LocalMap::occupied(int ix, int iy) const {
    return inBounds_(ix, iy) && log_[idx_(ix, iy)] > p_.occThresh;
}

void LocalMap::integrate(float pe, float pn, float yawDeg,
                         const float* scanClear, int n, float fovDeg, float maxM) {
    if (n < 1 || !scanClear) return;
    const float half = fovDeg * 0.5f;
    const float step = p_.cellM * 0.5f;             // march at half-cell resolution

    for (int i = 0; i < n; ++i) {
        const float rel = (n == 1) ? 0.f : (-half + fovDeg * i / (n - 1));
        const float b   = (yawDeg + rel) * kPi / 180.f;
        const float de  = std::sin(b), dn = std::cos(b);
        const float clr = clampf(scanClear[i], 0.f, maxM);
        const bool  hit = scanClear[i] < maxM - 1e-3f;

        // Free along the ray up to the (inflated-free) hit point.
        const float freeTo = hit ? std::max(0.f, clr - step) : clr;
        for (float d = step; d <= freeTo; d += step) {
            int ix, iy; worldToCell_(pe + de * d, pn + dn * d, ix, iy);
            if (inBounds_(ix, iy)) {
                float& l = log_[idx_(ix, iy)];
                l = clampf(l - p_.lFree, -p_.lClamp, p_.lClamp);
            }
        }
        // The hit cell is occupied. Cells beyond it stay unknown (0).
        if (hit) {
            int ix, iy; worldToCell_(pe + de * clr, pn + dn * clr, ix, iy);
            if (inBounds_(ix, iy)) {
                float& l = log_[idx_(ix, iy)];
                l = clampf(l + p_.lOcc, -p_.lClamp, p_.lClamp);
            }
        }
    }
}

bool LocalMap::plan(float pe, float pn, float goalBearingDeg, float& outBearingDeg) {
    // 1. Inflated obstacle mask (Minkowski-dilate occupied cells by robotR).
    const int r = std::max(0, (int)std::lround(p_.robotR / p_.cellM));
    std::fill(blocked_.begin(), blocked_.end(), (uint8_t)0);
    for (int iy = 0; iy < H_; ++iy)
        for (int ix = 0; ix < W_; ++ix)
            if (log_[idx_(ix, iy)] > p_.occThresh)
                for (int dy = -r; dy <= r; ++dy)
                    for (int dx = -r; dx <= r; ++dx) {
                        if (dx * dx + dy * dy > r * r) continue;
                        const int jx = ix + dx, jy = iy + dy;
                        if (inBounds_(jx, jy)) blocked_[idx_(jx, jy)] = 1;
                    }

    // 2. Goal cell: project far along the goal bearing, clamp into the grid.
    const float gb = goalBearingDeg * kPi / 180.f;
    const float R  = 0.45f * p_.sizeM;              // just inside the far edge
    int gx, gy; worldToCell_(pe + std::sin(gb) * R, pn + std::cos(gb) * R, gx, gy);
    gx = std::max(0, std::min(W_ - 1, gx));
    gy = std::max(0, std::min(H_ - 1, gy));
    // If the projected goal cell is blocked, spiral out to the nearest free one.
    if (blocked_[idx_(gx, gy)]) {
        int best = -1, bx = gx, by = gy;
        for (int rad = 1; rad < W_ && best < 0; ++rad)
            for (int dy = -rad; dy <= rad && best < 0; ++dy)
                for (int dx = -rad; dx <= rad; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != rad) continue;
                    const int jx = gx + dx, jy = gy + dy;
                    if (inBounds_(jx, jy) && !blocked_[idx_(jx, jy)]) {
                        bx = jx; by = jy; best = 1; break;
                    }
                }
        if (best < 0) return false;                 // grid fully blocked
        gx = bx; gy = by;
    }

    // 3. Wavefront BFS from the goal over non-blocked cells (unknown = free).
    std::fill(dist_.begin(), dist_.end(), -1);
    std::deque<int> q;
    dist_[idx_(gx, gy)] = 0;
    q.push_back(idx_(gx, gy));
    static const int NX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int NY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!q.empty()) {
        const int cur = q.front(); q.pop_front();
        const int cx = cur % W_, cy = cur / W_;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + NX[k], ny = cy + NY[k];
            if (!inBounds_(nx, ny)) continue;
            const int ni = idx_(nx, ny);
            if (blocked_[ni] || dist_[ni] >= 0) continue;
            dist_[ni] = dist_[cur] + 1;
            q.push_back(ni);
        }
    }

    // 4. Descent start = the drone cell — but the planner inflates obstacles by
    //    a WIDER berth than the live safety margin, so the drone (flying at the
    //    live margin) can sit inside the berth, i.e. in a "blocked" cell with no
    //    wavefront distance. Snap the start to the nearest reachable free cell;
    //    the resulting bearing then leads out to the berth distance and around,
    //    while the live corridor keeps the immediate move safe.
    int dx0, dy0; worldToCell_(pe, pn, dx0, dy0);
    if (!inBounds_(dx0, dy0)) return false;
    int sx = dx0, sy = dy0;
    if (blocked_[idx_(sx, sy)] || dist_[idx_(sx, sy)] < 0) {
        int found = -1;
        for (int rad = 1; rad < W_ && found < 0; ++rad)
            for (int dy = -rad; dy <= rad && found < 0; ++dy)
                for (int dx = -rad; dx <= rad; ++dx) {
                    if (std::max(std::abs(dx), std::abs(dy)) != rad) continue;
                    const int jx = dx0 + dx, jy = dy0 + dy;
                    if (inBounds_(jx, jy) && !blocked_[idx_(jx, jy)] && dist_[idx_(jx, jy)] >= 0) {
                        sx = jx; sy = jy; found = 1; break;
                    }
                }
        if (found < 0) return false;                // no reachable free cell near us
    }

    int cx = sx, cy = sy;
    for (int s = 0; s < p_.descentSteps; ++s) {
        int bestx = cx, besty = cy, bestd = dist_[idx_(cx, cy)];
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + NX[k], ny = cy + NY[k];
            if (!inBounds_(nx, ny)) continue;
            const int ni = idx_(nx, ny);
            if (dist_[ni] < 0) continue;
            if (dist_[ni] < bestd) { bestd = dist_[ni]; bestx = nx; besty = ny; }
        }
        if (bestx == cx && besty == cy) break;      // local min (== goal region)
        cx = bestx; cy = besty;
    }
    // Bearing from the DRONE's true position to the descended target cell.
    float ce, cn; cellToWorld_(cx, cy, ce, cn);
    if (std::fabs(ce - pe) < 1e-3f && std::fabs(cn - pn) < 1e-3f) return false;
    outBearingDeg = std::atan2(ce - pe, cn - pn) * 180.f / kPi;
    return true;
}
