// ego_planner_check -- the EGO-style baseline does what the method says,
// before any bench number is read from it:
//
//   1. a trunk dead on the straight line to the target: the optimised
//      trajectory goes ROUND it (the straight line through it is where it
//      starts), keeping about robotR from it, and still ends near the target;
//   2. open space: the trajectory stays a straight line (nothing to push);
//   3. UNKNOWN: with unknownFree (EGO's own convention) an unobserved map is
//      flown through as if empty; with unknownFree=false the same map is a
//      wall and the planner finds nothing to fly. That difference is the one
//      this tree cares about, so it is pinned here.

#include <cmath>
#include <cstdio>

#include "ego_planner.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

namespace {
int bad = 0;
void check(bool ok, const char* what) {
    std::printf("  %-64s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++bad;
}

// True distance from p to the nearest solid cell's centre minus half a cell.
float clearance(const VoxelWorld& w, const EgoPlanner::V3& p) {
    int cx, cy, cz; w.worldToCell(p[0], p[1], p[2], cx, cy, cz);
    const int r = int(2.0f / w.cell()) + 1;
    float best = 2.f;
    for (int dz = -r; dz <= r; ++dz)
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                if (!w.solid(cx + dx, cy + dy, cz + dz)) continue;
                float x, y, z; w.cellCentre(cx + dx, cy + dy, cz + dz, x, y, z);
                best = std::min(best, std::sqrt((x - p[0]) * (x - p[0]) + (y - p[1]) * (y - p[1]) +
                                                (z - p[2]) * (z - p[2])) - 0.5f * w.cell());
            }
    return best;
}
}  // namespace

int main() {
    VoxelWorld w;
    const float c = 0.25f;
    w.init(c, -10.f, -5.f, 0.f, int(20 / c), int(25 / c), int(6 / c));
    // a 1 m square trunk centred on (0, 5), floor to ceiling
    for (int z = 0; z < int(6 / c); ++z)
        for (float y = 4.5f; y < 5.5f; y += c)
            for (float x = -0.5f; x < 0.5f; x += c) {
                int ix, iy, iz; w.worldToCell(x + 0.01f, y + 0.01f, z * c + 0.01f, ix, iy, iz);
                w.set(ix, iy, iz);
            }
    VoxelMapParams mp;
    VoxelMap m;
    m.init(mp, 0.f, 3.f, 2.f);
    m.imprint(w, 0.f, 3.f, 2.f, 12.f, 3.f);       // the map knows the truth here

    EgoPlanner ego;
    const EgoPlanner::V3 start{0.f, 0.f, 2.f}, target{0.f, 7.f, 2.f};
    check(ego.plan(m, start, {0.f, 0.f, 0.f}, target), "trunk ahead: plans");
    float minClr = 9.f, maxDev = 0.f;
    for (float t = 0.f; t <= ego.duration(); t += 0.02f) {
        const EgoPlanner::V3 p = ego.sample(t);
        minClr = std::min(minClr, clearance(w, p));
        maxDev = std::max(maxDev, std::fabs(p[0]));
    }
    const EgoPlanner::V3 e = ego.sample(ego.duration());
    const float endErr = std::hypot(e[0] - target[0], e[1] - target[1]);
    std::printf("    min clearance %.2f m (robotR %.2f), widest detour %.2f m, end %.2f m from target\n",
                minClr, ego.params().robotR, maxDev, endErr);
    check(maxDev > 0.8f, "trunk ahead: the trajectory leaves the straight line");
    check(minClr > 0.45f, "trunk ahead: keeps ~robotR from the trunk (> 0.45 m)");
    check(endErr < 1.0f, "trunk ahead: still ends near the target (< 1 m)");

    // open space beside the trunk: nothing to push
    check(ego.plan(m, {-6.f, 0.f, 2.f}, {0.f, 0.f, 0.f}, {-6.f, 7.f, 2.f}), "open: plans");
    float dev = 0.f;
    for (float t = 0.f; t <= ego.duration(); t += 0.02f) dev = std::max(dev, std::fabs(ego.sample(t)[0] + 6.f));
    std::printf("    open space: widest deviation %.3f m\n", dev);
    check(dev < 0.05f, "open: stays a straight line");

    // a map that has seen NOTHING
    VoxelMap blank;
    blank.init(mp, 0.f, 3.f, 2.f);
    EgoPlanner optimistic;
    check(optimistic.plan(blank, start, {0.f, 0.f, 0.f}, target) && optimistic.stats().astarFails == 0,
          "unknownFree (EGO's convention): an unobserved map is flown as empty");
    EgoParams cautious;
    cautious.unknownFree = false;
    EgoPlanner careful(cautious);
    careful.plan(blank, start, {0.f, 0.f, 0.f}, target);
    check(careful.stats().astarFails > 0, "unknownFree=false: the same map is a wall (no route found)");

    std::printf("ego_planner_check: %s\n", bad ? "FAILED" : "ok");
    return bad ? 1 : 0;
}
