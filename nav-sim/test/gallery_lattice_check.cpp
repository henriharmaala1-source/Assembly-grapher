// IS THE DEMO WORLD ACTUALLY ON THE LATTICE? An assertion, not an opinion.
//
// gallery and hall exist so the first-person voxel view is CONSISTENT: the
// same wall drawn as 2.0 m voxels far away, 1.0 m at mid range and 0.25 m up
// close should cover the same volume every time, so approaching it subdivides
// it and never moves it. That property rests on one geometric claim --
//
//     every solid voxel lies in a 2.0 m lattice cell that is ENTIRELY solid
//
// -- and nothing else. A lattice cell that is part solid and part air cannot
// be represented the same way at 2.0 m as at 0.25 m, by definition: the coarse
// rung has to round it one way or the other, and which way it rounds changes
// with the map's phase. One such cell is a wall that moves as you fly at it.
//
// This is cheap to state and easy to break by accident. fillBox walks an
// INCLUSIVE cell range, so a box asked for [0, 2] is 2.25 m wide and every
// face of it is off-lattice; the erase that clears the spawn had the same bug.
// Both were found by this check rather than by looking at the render, which is
// the point -- at 2 m voxels a one-cell overhang is invisible until you are
// close to it, which is exactly when a demo is being watched.
#include <cmath>
#include <cstdio>
#include <vector>

#include "voxel_world.hpp"

using namespace sim;

namespace {

// Returns the number of MIXED lattice cells: neither empty nor full.
long long mixedCells(const VoxelWorld& w, float latticeM, long long* solidOut,
                     long long* totalOut) {
    const float cell = w.cell();
    const int step = int(std::lround(latticeM / cell));
    long long mixed = 0, solid = 0, total = 0;
    const int nx = w.nx(), ny = w.ny(), nz = w.nz();
    // Whole lattice cells only. A partial one at the far edge is an artefact of
    // the world's extent, not of its geometry, and counting it would make the
    // check fail for a reason it is not about.
    for (int z = 0; z + step <= nz; z += step)
        for (int y = 0; y + step <= ny; y += step)
            for (int x = 0; x + step <= nx; x += step) {
                int on = 0;
                for (int c = 0; c < step; ++c)
                    for (int b = 0; b < step; ++b)
                        for (int a = 0; a < step; ++a)
                            on += w.solid(x + a, y + b, z + c) ? 1 : 0;
                const int all = step * step * step;
                ++total;
                if (on == all) ++solid;
                else if (on != 0) ++mixed;
            }
    if (solidOut) *solidOut = solid;
    if (totalOut) *totalOut = total;
    return mixed;
}

int checkOne(const char* label, const GalleryParams& base, unsigned seed) {
    GalleryParams p = base;
    p.seed = seed;
    VoxelWorld w;
    float sx = 0, sy = 0, gx = 0, gy = 0;
    genGallery(w, p, &sx, &sy, &gx, &gy);

    int bad = 0;
    long long solid = 0, total = 0;
    const long long mixed = mixedCells(w, p.latticeM, &solid, &total);
    std::printf("  %-8s seed %u: %lld/%lld lattice cells solid, %lld mixed\n",
                label, seed, solid, total, mixed);
    if (mixed) {
        std::printf("    FAIL: %lld lattice cells are part solid and part air, "
                    "so the coarse rung must round them and the geometry moves "
                    "as you approach it.\n", mixed);
        ++bad;
    }
    if (solid == 0) {
        std::printf("    FAIL: nothing solid at all -- an empty world renders "
                    "as pure fog, which reads as a broken map.\n");
        ++bad;
    }

    // THE SPAWN SETS THE PHASE of all three map rungs, so it must be on the
    // lattice; see GalleryParams. An off-lattice spawn undoes everything above
    // and changes nothing visible in the generator, which is why it is asserted
    // here rather than trusted.
    auto onLattice = [&](float v) {
        const float r = v / p.latticeM;
        return std::fabs(r - std::round(r)) < 1e-3f;
    };
    if (!onLattice(sx) || !onLattice(sy)) {
        std::printf("    FAIL: start (%.3f, %.3f) is not on the %.1f m lattice\n",
                    sx, sy, p.latticeM);
        ++bad;
    }
    if (!onLattice(gx) || !onLattice(gy)) {
        std::printf("    FAIL: goal (%.3f, %.3f) is not on the %.1f m lattice\n",
                    gx, gy, p.latticeM);
        ++bad;
    }

    // AND THE SPAWN MUST BE IN AIR. A start inside a block makes every
    // primitive inadmissible, so the aircraft sits still -- indistinguishable
    // from a policy that has failed, which is the worst thing a demo can show.
    int cx, cy, cz;
    w.worldToCell(sx, sy, 4.f, cx, cy, cz);
    if (w.solid(cx, cy, cz)) {
        std::printf("    FAIL: the spawn is inside solid geometry\n");
        ++bad;
    }
    return bad;
}

}  // namespace

int main() {
    std::printf("[gallery lattice] every solid voxel must fill a whole 2 m cell\n");
    int bad = 0;
    GalleryParams g;                       // the outdoor one
    GalleryParams h;                       // the indoor one
    h.wallFrac = 0.65f; h.ceiling = true; h.minHM = 6.f; h.maxHM = 10.f;
    for (unsigned s : {101u, 102u, 103u}) {
        bad += checkOne("gallery", g, s);
        bad += checkOne("hall", h, s);
    }
    std::printf("[gallery lattice] %d failure(s)\n", bad);
    return bad ? 1 : 0;
}
