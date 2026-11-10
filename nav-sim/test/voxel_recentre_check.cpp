// voxel_recentre_check -- VoxelMap::recentre shifts the map IN PLACE, and the
// order it walks the rows in is what keeps it from reading a cell it has
// already overwritten. So: every shift from -N to +N on each axis, alone and
// mixed (signs disagreeing between axes is the case an ordering bug hides
// in), past the map's own size too, against the obvious copy-to-a-new-grid
// definition: a cell takes the value of the cell `shift` away, or 0 (unknown)
// when that falls outside.

#include <cstdio>
#include <random>
#include <vector>

#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

int main() {
    VoxelMapParams mp;
    mp.nx = 19; mp.ny = 13; mp.nz = 7;
    const float c = mp.cell;
    // A world dense enough that the imprinted map is a busy mix of occupied,
    // free and (outside the imprint radius) unknown.
    VoxelWorld w;
    w.init(c, -4.f, -4.f, -2.f, 40, 40, 20);
    std::mt19937 g(11);
    std::uniform_int_distribution<int> X(0, 39), Z(0, 19);
    for (int i = 0; i < 9000; ++i) w.set(X(g), X(g), Z(g));

    std::vector<std::array<int, 3>> shifts;
    for (int a = -3; a <= 3; ++a)
        for (int b = -3; b <= 3; ++b)
            for (int d = -3; d <= 3; ++d) shifts.push_back({a, b, d});
    for (int s : {-25, -19, -13, -8, -7, 7, 8, 13, 19, 25}) {       // up to and past each axis' size
        shifts.push_back({s, 0, 0}); shifts.push_back({0, s, 0}); shifts.push_back({0, 0, s});
        shifts.push_back({s, -2, 1}); shifts.push_back({2, s, -1}); shifts.push_back({-1, 2, s});
    }

    long bad = 0, cells = 0, nonzero = 0;
    for (const auto& sh : shifts) {
        VoxelMap m;
        m.init(mp, 0.f, 0.f, 0.f);
        m.imprint(w, 0.f, 0.f, 0.f, 1.9f, 0.7f);
        std::vector<float> before(size_t(mp.nx) * mp.ny * mp.nz);
        for (int z = 0; z < mp.nz; ++z)
            for (int y = 0; y < mp.ny; ++y)
                for (int x = 0; x < mp.nx; ++x)
                    before[(size_t(z) * mp.ny + y) * mp.nx + x] = m.logAt(x, y, z);
        m.recentre(sh[0] * c, sh[1] * c, sh[2] * c);
        for (int z = 0; z < mp.nz; ++z)
            for (int y = 0; y < mp.ny; ++y)
                for (int x = 0; x < mp.nx; ++x) {
                    const int ox = x + sh[0], oy = y + sh[1], oz = z + sh[2];
                    const bool in = ox >= 0 && oy >= 0 && oz >= 0 && ox < mp.nx && oy < mp.ny && oz < mp.nz;
                    const float want = in ? before[(size_t(oz) * mp.ny + oy) * mp.nx + ox] : 0.f;
                    const float got = m.logAt(x, y, z);
                    bad += (got != want);
                    nonzero += (want != 0.f);
                    ++cells;
                }
    }
    std::printf("recentre in place vs reference: %ld/%ld cells differ over %zu shifts (%ld non-zero expected)\n",
                bad, cells, shifts.size(), nonzero);
    return (bad || nonzero < 1000) ? 1 : 0;
}
