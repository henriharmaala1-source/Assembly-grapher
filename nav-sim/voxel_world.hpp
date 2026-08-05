#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Dense 3D voxel WORLD — the ground truth a simulated depth camera looks at.
//
// WHY A VOXEL GRID AND NOT A TRIANGLE MESH.
//
// The obvious way to build a forest or a city is a mesh, and then raycast it
// with a BVH. That is the wrong choice here for three reasons:
//
//  1. COST IS INDEPENDENT OF SCENE COMPLEXITY. A DDA walk through a grid costs
//     O(ray length in voxels) whether the cell it enters belongs to one building
//     or to the ten-thousandth branch of a forest. A BVH costs O(log N) per ray
//     with a large constant and terrible cache behaviour on a scene of a million
//     leaves. A forest is exactly the scene that kills mesh raycasters and
//     exactly the scene we most want to test.
//
//  2. IT MAKES THE SIM-TO-REAL GAP MEASURABLE. The thing being validated is a
//     VoxelMap built from noisy stereo. If the ground truth is itself a voxel
//     grid at the same resolution, then "did the mapper get it right" is a
//     direct per-voxel comparison -- IoU, false-free rate, false-occupied rate.
//     Against a mesh you would have to invent a sampling scheme first, and the
//     scheme would become part of the answer.
//
//  3. REAL DATA IS ALREADY VOXEL-SHAPED. Airborne LiDAR (Finland's MML open
//     data is 5 pts/m^2 nationwide, CC BY 4.0) is a point cloud; the natural
//     way to consume it is to drop points into cells. OSM buildings are
//     footprint polygons plus a height, which extrude into columns of cells.
//     Both import in a few lines. A mesh pipeline would need a mesher.
//
// The cost is memory: one bit per cell. At 0.5 m over 400x400x60 m that is
// 76.8M cells = 9.6 MB. Cheap on a desktop, and the flight code never sees this
// class -- it only ever sees the depth images it produces.
//
// FRAME: ENU metres, +x = East, +y = North, +z = Up. Matches sim_world.hpp.
// ---------------------------------------------------------------------------

namespace sim {

// M_PI IS NOT STANDARD C++. glibc and libc++ expose it as a POSIX extension, so
// GCC and Clang compile happily; MSVC does not define it unless _USE_MATH_DEFINES
// is set before <cmath>, which is fragile because it depends on include order.
// This built clean on Linux and failed every Windows CI target with
// "error C2065: 'M_PI': undeclared identifier". Use this instead.
constexpr float PI_F = 3.14159265358979323846f;

class VoxelWorld {
public:
    // origin is the world coord of cell (0,0,0)'s min corner.
    void init(float cell, float ox, float oy, float oz, int nx, int ny, int nz);

    float cell() const { return cell_; }
    int   nx() const { return nx_; }
    int   ny() const { return ny_; }
    int   nz() const { return nz_; }

    bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < nx_ && y < ny_ && z < nz_;
    }
    bool solid(int x, int y, int z) const {
        if (!inBounds(x, y, z)) return false;
        size_t i = idx(x, y, z);
        return (bits_[i >> 3] >> (i & 7)) & 1u;
    }
    void set(int x, int y, int z, bool v = true) {
        if (!inBounds(x, y, z)) return;
        size_t i = idx(x, y, z);
        if (v) bits_[i >> 3] |= uint8_t(1u << (i & 7));
        else   bits_[i >> 3] &= uint8_t(~(1u << (i & 7)));
    }

    // World <-> cell. worldToCell floors, so a point exactly on a boundary
    // belongs to the cell above/right of it -- consistent with the DDA below.
    void worldToCell(float wx, float wy, float wz, int& x, int& y, int& z) const {
        x = int(std::floor((wx - ox_) / cell_));
        y = int(std::floor((wy - oy_) / cell_));
        z = int(std::floor((wz - oz_) / cell_));
    }
    void cellCentre(int x, int y, int z, float& wx, float& wy, float& wz) const {
        wx = ox_ + (x + 0.5f) * cell_;
        wy = oy_ + (y + 0.5f) * cell_;
        wz = oz_ + (z + 0.5f) * cell_;
    }

    // Amanatides & Woo voxel traversal. Returns the metric distance to the
    // first solid cell along the ray, or maxRange if none. `hitTex` receives a
    // per-surface "texture richness" in [0,1] -- see MATERIAL below.
    float raycast(float px, float py, float pz,
                  float dx, float dy, float dz,
                  float maxRange, float* hitTex = nullptr) const;

    // MATERIAL / texture richness, 0 = featureless, 1 = richly textured.
    // Stereo matching succeeds or fails on TEXTURE, not on geometry, and a
    // simulator that hands the mapper perfect depth on a blank wall validates
    // nothing. Every solid cell carries a texture byte so the depth camera can
    // decide, per ray, whether a match would plausibly have been found.
    void setTex(int x, int y, int z, float t) {
        if (inBounds(x, y, z)) tex_[idx(x, y, z)] = uint8_t(t * 255.f + 0.5f);
    }
    float texAt(int x, int y, int z) const {
        return inBounds(x, y, z) ? tex_[idx(x, y, z)] / 255.f : 0.f;
    }

    size_t solidCount() const;

private:
    size_t idx(int x, int y, int z) const {
        return (size_t(z) * ny_ + y) * nx_ + x;
    }
    float cell_ = 0.5f, ox_ = 0, oy_ = 0, oz_ = 0;
    int nx_ = 0, ny_ = 0, nz_ = 0;
    std::vector<uint8_t> bits_;   // 1 bit/cell occupancy
    std::vector<uint8_t> tex_;    // 1 byte/cell texture richness
};

// --- world generators ------------------------------------------------------
//
// These are PROCEDURAL but structurally real: the city uses a street grid with
// realistic block sizes and a building-height distribution, the forest uses
// published stem-density and diameter statistics for boreal forest. They are
// not a substitute for real geodata -- loadOsmBuildings() and loadLidarXyz()
// below take that -- but they run with no network and no downloads, which is
// what makes them useful as a regression fixture.

struct CityParams {
    float sizeM      = 400.f;   // square side
    float cell       = 0.5f;
    float blockM     = 60.f;    // building block pitch
    float streetM    = 14.f;    // street width (gap between blocks)
    float hMin       = 6.f;     // building height range
    float hMax       = 42.f;
    float glassFrac  = 0.25f;   // fraction of facades that are low-texture glass
    unsigned seed    = 1;
};
void genCity(VoxelWorld& w, const CityParams& p);

struct ForestParams {
    float sizeM      = 200.f;
    float cell       = 0.25f;
    float topM       = 26.f;
    // Boreal managed forest: ~800-1600 stems/ha is typical. 1200/ha over a
    // 200x200 m plot (4 ha) is ~480 trees.
    float stemsPerHa = 1200.f;
    float dbhMinM    = 0.10f;   // trunk diameter at breast height
    float dbhMaxM    = 0.35f;
    float canopyBase = 0.45f;   // crown starts at this fraction of tree height
    float undergrowth= 0.15f;   // fraction of ground cells with low scrub
    // TRAILS. A managed boreal forest is not a uniform Poisson field of stems;
    // it is threaded with skid roads, ditch lines and footpaths, and those are
    // precisely the structures a low-flying aircraft should use. They also make
    // the harness discriminating: uniform forest rewards any planner that
    // wanders competently, whereas a trail asks whether the planner can FIND
    // and HOLD a corridor, which is a strictly harder and more useful question.
    int   trails     = 3;
    float trailWidthM= 3.5f;    // cleared width; 0 disables trails entirely
    // TRUNK TEXTURE, and getting this wrong made every safety number in this
    // project optimistic about the one obstacle that kills you.
    //
    // The original value was 0.85 -- the highest texture in the world -- on the
    // reasoning that "bark is strongly textured, which is why forests are
    // navigable at all". That reasoning never accounted for BACKLIGHTING. Real
    // depth images from a boreal stand show the ground resolved beautifully and
    // the trunks as solid holes: bark in shadow against bright sky is a
    // low-contrast surface, and a matcher cannot correlate what it cannot see.
    //
    // The range straddles texThresh (0.25) deliberately. Some trunks vanish
    // entirely, most are marginal and drop out in patches, a few in good side
    // light resolve fine. That is the distribution the real images show, and it
    // is far harsher than what we had.
    float trunkTexMin = 0.15f;
    float trunkTexMax = 0.55f;
    unsigned seed    = 1;
};

// Trail centrelines, in the same ENU metres as the world. Optional output --
// the sim uses them to start on a trail and aim along it, which is the only way
// to test trail-following rather than trail-crossing.
using Trail = std::vector<std::array<float, 2>>;
void genForest(VoxelWorld& w, const ForestParams& p,
               std::vector<Trail>* trailsOut = nullptr);

// --- real-data importers ---------------------------------------------------
//
// Both are deliberately dumb text formats so you can produce them with a few
// lines of Python from whatever source you have, rather than linking a GIS
// stack into a flight-adjacent repo.

// Buildings as extruded footprints. One building per line:
//     height_m  x0 y0  x1 y1  x2 y2 ...        (metres, ENU, polygon closed
//                                               implicitly, any vertex count)
// Produce with Overpass + pyproj:
//     [out:json];way["building"](bbox);out geom;
// then project lat/lon to a local ENU metre frame and write this format.
bool loadOsmBuildings(VoxelWorld& w, const std::string& path,
                      float cell, float pad = 20.f);

// LiDAR / photogrammetry point cloud, one "x y z" per line in ENU metres.
// Every point marks its cell solid. Works directly on national open LiDAR
// (e.g. Finland's MML 5 pts/m^2) after a coordinate shift.
// `groundZ` fills everything below the lowest return in each column so the
// ground is solid rather than a one-voxel shell -- otherwise rays pass under it.
bool loadLidarXyz(VoxelWorld& w, const std::string& path,
                  float cell, bool fillGround = true);

}  // namespace sim
