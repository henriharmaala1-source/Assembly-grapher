#include "occupancy_grid.hpp"

#include <cmath>

namespace navsim {

namespace { constexpr float kPi = 3.14159265358979323846f; }

OccupancyGrid::OccupancyGrid() : OccupancyGrid(Params{}) {}
OccupancyGrid::OccupancyGrid(Params p)
    : p_(p), log_((size_t)p.cells * p.cells, 0.f), originE_(0), originN_(0) {}

bool OccupancyGrid::worldToCell(float e, float n, int& cx, int& cy) const {
    const float half = p_.cells * p_.cellM * 0.5f;
    cx = (int)((e - originE_ + half) / p_.cellM);
    cy = (int)((n - originN_ + half) / p_.cellM);
    return inBounds(cx, cy);
}

void OccupancyGrid::cellToWorld(int cx, int cy, float& e, float& n) const {
    const float half = p_.cells * p_.cellM * 0.5f;
    e = originE_ - half + (cx + 0.5f) * p_.cellM;
    n = originN_ - half + (cy + 0.5f) * p_.cellM;
}

void OccupancyGrid::integrate(float pe, float pn, float yawDeg,
                              const std::vector<float>& ranges, float hFovDeg, float maxRange) {
    const int   N = (int)ranges.size();
    if (N == 0) return;
    const float half = hFovDeg * 0.5f;
    for (int i = 0; i < N; ++i) {
        const float rel = (N == 1) ? 0.f : (-half + 2.f * half * i / (N - 1));
        const float b   = (yawDeg + rel) * kPi / 180.f;
        const float de  = std::sin(b), dn = std::cos(b);
        const float rng = ranges[i];
        const bool  hit = rng < maxRange - 1e-3f;
        // March free space along the ray up to just before the hit.
        const float stopM = hit ? rng - p_.cellM * 0.5f : maxRange;
        for (float m = 0.f; m < stopM; m += p_.cellM * 0.5f) {
            int cx, cy;
            if (worldToCell(pe + de * m, pn + dn * m, cx, cy)) {
                float& L = log_[cy * p_.cells + cx];
                L = std::max(-p_.lClamp, L - p_.lMiss);
            }
        }
        if (hit) {
            int cx, cy;
            if (worldToCell(pe + de * rng, pn + dn * rng, cx, cy)) {
                float& L = log_[cy * p_.cells + cx];
                L = std::min(p_.lClamp, L + p_.lHit);
            }
        }
    }
}

bool OccupancyGrid::blocked(int cx, int cy, int inflateCells) const {
    for (int dy = -inflateCells; dy <= inflateCells; ++dy)
        for (int dx = -inflateCells; dx <= inflateCells; ++dx) {
            if (dx * dx + dy * dy > inflateCells * inflateCells) continue;
            if (occupied(cx + dx, cy + dy)) return true;
        }
    return false;
}

}  // namespace navsim
