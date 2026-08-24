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

// INDOOR -- the regime every outdoor assumption breaks in.
//
// Three things change at once and each has bitten this project already:
//   * THERE IS A CEILING. "Up" is not sky, it is a surface 2.4 m away. Every
//     far-field openness argument that leans on the sky being unbounded is
//     inverted here, and the climb/descent penalties are being asked to hold
//     altitude in a 2.4 m slot rather than an open one.
//   * DOORWAYS ARE NARROWER THAN THE HORIZON. A 0.9 m door against a 0.6 m
//     robot radius leaves 0.15 m each side. A planner that requires a clear
//     2 s swept tube cannot fit one through a door at any speed.
//   * WALLS ARE FLAT AND OFTEN UNTEXTURED. Painted plaster is the glass-facade
//     problem again, at 2 m instead of 40.
struct IndoorParams {
    float sizeM      = 24.f;    // building footprint, square
    float cell       = 0.05f;   // 5 cm -- door frames and chair legs are the point
    float ceilM      = 2.4f;    // ceiling height
    float wallM      = 0.12f;   // internal wall thickness
    float roomM      = 5.0f;    // nominal room pitch
    float doorW      = 0.9f;    // doorway width
    float doorH      = 2.0f;
    // PAINTED PLASTER IS NOT FEATURELESS, and the first value here (0.10) was
    // pessimism rather than measurement. It sat below `texThresh` 0.25, so every
    // wall returned NOTHING and the indoor map came out empty -- which then read
    // as a planner failure. Real interior walls carry skirting, door frames,
    // sockets, switches, picture rails, scuffs, wallpaper weave, corner shadows
    // and non-uniform lighting; a genuinely blank five-metre wall is the
    // exception. 0.35 clears the threshold with margin, which is the honest
    // default. `blankFrac` keeps the hard case available deliberately rather
    // than imposing it everywhere.
    float wallTex    = 0.35f;
    float blankTex   = 0.08f;   // the genuinely featureless wall
    float blankFrac  = 0.15f;   // ...and how many walls are like that
    float clutterTex = 0.55f;
    float furnishFrac= 0.55f;   // fraction of rooms that get furniture
    unsigned seed    = 1;
};
void genIndoor(VoxelWorld& w, const IndoorParams& p);

// ROAD -- a corridor problem with thin vertical furniture and overhead wires.
//
// Chosen because it isolates the failure mode the forest hides: the obstacles
// that matter are FEW, THIN and TALL against a wide open background, which is
// the worst case for a stereo mapper (sub-pixel width at range) and the easiest
// case for an openness histogram (almost everything is open). If the two
// planners differ anywhere, they differ here.
struct RoadParams {
    float lengthM    = 200.f;
    float widthM     = 40.f;    // full corridor including verges
    float cell       = 0.10f;
    float laneM      = 7.0f;    // carriageway width
    float poleEveryM = 25.f;    // lamp post pitch
    float poleR      = 0.08f;
    float poleH      = 8.0f;
    float wireH      = 6.5f;    // catenary height -- 8 cm wire, the thin case
    float wireR      = 0.04f;
    int   nVehicles  = 12;
    int   nSigns     = 10;
    bool  wires      = true;
    unsigned seed    = 1;
};
void genRoad(VoxelWorld& w, const RoadParams& p);

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
    // DENSITY BANDS. A real stand is not uniform: it has thickets, ordinary
    // forest, and open ground you can actually make progress through. A uniform
    // Poisson field rewards any planner that wanders competently; banded density
    // asks the harder question -- does it FIND the open route and commit to it,
    // and does it slow down when the stand closes in?
    //
    // bandsY divides the world along +y into equal bands and multiplies
    // stemsPerHa by bandMul[i % bandMul.size()]. Empty = uniform (old behaviour).
    std::vector<float> bandMul;
    // Band axis. false = bands run ACROSS the flight path (+y), so the aircraft
    // must fly THROUGH each density in turn -- no choice; that tests speed
    // modulation. true = bands run ALONG the flight path (+x), giving parallel
    // lanes of differing density side by side, so a goal straight ahead forces a
    // CHOICE: plough through the thicket or divert to the clear lane. The second
    // is the realistic question -- you do not want to navigate extreme clutter,
    // you want to avoid it.
    bool  bandAlongX = false;
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
    // The range straddles texThresh (0.25) deliberately. Some trunks are
    // marginal and drop out in patches, most in reasonable light resolve fine.
    //
    // 0.15-0.55 was tried first and was an OVER-CORRECTION from a single
    // screenshot: it put the median at 0.35, below the 0.40 cliff in the
    // tolerance curve, and made a quarter of all trunks completely invisible.
    // One image of unknown provenance does not justify assuming the worst case
    // is typical. Bark in reasonable light genuinely is textured -- the
    // original 0.85 was wrong about backlighting, not about bark.
    //
    // 0.30-0.75 puts the median at 0.525, so most trunks resolve, a minority
    // drop out in patches, and none are wholly invisible. Still a guess, and
    // still the parameter to calibrate first: photograph a backlit trunk, run
    // build/bark_contrast, and set this from the measurement rather than from
    // anyone's screenshot.
    float trunkTexMin = 0.30f;
    float trunkTexMax = 0.75f;
    unsigned seed    = 1;
};

// Trail centrelines, in the same ENU metres as the world. Optional output --
// the sim uses them to start on a trail and aim along it, which is the only way
// to test trail-following rather than trail-crossing.
using Trail = std::vector<std::array<float, 2>>;
void genForest(VoxelWorld& w, const ForestParams& p,
               std::vector<Trail>* trailsOut = nullptr);

// --- cul-de-sac: the world that tests whether a ROUTER is needed ------------
//
// Every world in this harness so far rewards a purely reactive planner, and
// the measurements say so: routing was never safer and usually slower, in
// forest and city alike. But that is an argument about THESE worlds, not about
// routers. A reactive planner with a 12 m horizon cannot see out of a dead
// end, and until the harness contains one, "the router does not earn its keep"
// is a statement about a gap in the test set.
//
// So: a three-sided pocket straddling the straight line from spawn to goal.
// The aircraft is lured in -- the goal bearing points directly through it --
// and to escape it must abandon that bearing entirely, fly back out of the
// mouth, and go around the outside. That is precisely the manoeuvre a horizon
// of 12 m cannot plan and a horizon of 25 m can.
//
// The walls are WELL TEXTURED on purpose. This is a planning test, and if the
// walls were hard to see it would silently become a perception test instead --
// the aircraft would fly into one rather than being trapped by it, and the
// result would answer a different question than the one asked.
struct CulDeSacParams {
    float sizeM   = 200.f;
    float cell    = 0.5f;
    float wallH   = 25.f;   // taller than the map is deep, so climbing out is
                            // not a loophole the test accidentally allows
    float wallT   = 4.f;
    float mouthY  = 70.f;   // pocket opens toward -y, closed end at mouthY+depth
    float depthM  = 60.f;
    float widthM  = 56.f;   // internal clear width
    float tex     = 0.65f;  // easy to see: this tests planning, not perception
    float clutter = 0.4f;   // scattered pillars outside, so it is not a bare plane
    unsigned seed = 1;
};
void genCulDeSac(VoxelWorld& w, const CulDeSacParams& p);


// --- the hedge, and it exists because the field disagreed with the harness ---
//
// A real bush fence, plainly visible in the depth image at eight metres,
// produced NOT ONE occupied cell. The mechanism turned out to be the rays that
// pass THROUGH its gaps carving free the very cells its own twigs had marked --
// and the harness could not reproduce it, because `genForest` contains trunks
// and nothing else. `NOTES.md` has recorded that gap as blocking for weeks:
// trunks are the easy case, and thin porous structure is what a hedge, a wire
// fence, a branch and a power line all are.
//
// So this scene is built to be hard in exactly that way and no other:
//
//   * a fence line of THIN members -- posts an eighth of a metre across, rails
//     thinner still -- with gaps far wider than a map cell, so most rays go
//     straight past and only a few terminate on it;
//   * a hedge band grown along it at a settable FILL FRACTION, which is the one
//     parameter that decides whether the carve wins or the mark does;
//   * a BACKDROP well behind it, and this is the part that is easy to leave out.
//     Without something for the passing rays to hit, they return nothing, carve
//     nothing, and the failure cannot occur at all. The field had a street
//     behind the hedge; the harness needs one too.
//
// Everything is well textured EXCEPT the hedge, which is deliberately not: a
// bush fence in shade against a bright sky is the case that failed, and making
// it easy to see would quietly turn a perception test into a geometry one.
struct HedgeParams {
    // THE WORLD IS BUILT FINER THAN THE MAP, AND IT HAS TO BE.
    //
    // Every other generator takes the MAP's cell size, so the finest thing that
    // can exist in the harness is one map cell -- a quarter of a metre. A hedge
    // made of quarter-metre twigs is not a hedge, it is a perforated wall, and
    // it passes mappers that a real hedge defeats. That is exactly why the field
    // disagreed with the simulator and why `NOTES.md` has "no thin obstacles"
    // recorded as blocking.
    //
    // So this scene carries its own resolution and its own extent, sized around
    // the spawn rather than around the map grid: 30 m of world at 0.04 m is
    // 750 x 750 x 300 cells, which is affordable precisely because it does not
    // have to be 200 m wide.
    float sizeM      = 30.f;
    float cell       = 0.04f;
    float topM       = 12.f;
    float spawnE     = 30.f;   // where the aircraft will be placed
    float spawnN     = 15.f;
    float standoffM  = 4.0f;   // hedge this far in front of the spawn
    float backdropM  = 14.0f;  // and a treeline this far, for the rays that pass
    float hedgeH     = 1.8f;
    float hedgeT     = 0.5f;   // thickness front to back
    float twigM      = 0.04f;  // twig thickness -- ONE CENTIMETRE-SCALE cell
    float fill       = 0.10f;  // fraction of twig cells actually solid
    float hedgeTex   = 0.18f;  // BARK IN SHADE. The case that failed.
    float postEveryM = 1.2f;
    float postW      = 0.12f;
    float groundTex  = 0.60f;
    unsigned seed    = 1;
};
void genHedgeRow(VoxelWorld& w, const HedgeParams& p);

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
