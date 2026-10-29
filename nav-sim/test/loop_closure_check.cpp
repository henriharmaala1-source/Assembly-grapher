// CAN THE MAP RECOGNISE A PLACE IT HAS BEEN BEFORE?
//
// Loop closure is the one mechanism that BOUNDS drift rather than slowing it.
// Scan matching corrects each frame against the local map, but that map was
// itself built at drifted poses, so its error is self-reinforcing; only
// recognising somewhere already visited can pull a whole trajectory back.
//
// THE HARD CASE IS THE ONE TESTED. The vehicle flies out along a route and
// comes BACK along it, so every revisit happens with the heading reversed --
// exactly where appearance-based recognition fails, because a corridor
// traversed backwards does not look like itself. Geometry should not care.
//
// TWO SCENES, and the pair is the experiment rather than either alone:
//   corridor     every point looks like every other, so recognition SHOULD
//                fail. A method that "succeeds" here is reporting an artefact.
//   three rooms  genuinely different shapes. If it fails here TOO then the
//                descriptor is broken rather than the corridor degenerate, and
//                those want completely different fixes.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include "depth_camera.hpp"
#include "place_descriptor.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}
static std::string f2(double v) { char b[48]; std::snprintf(b, sizeof b, "%.2f", v); return b; }

struct Pose { float e, n, u, yaw; };

struct Outcome {
    double sameMean = 0, sameLo = 0, diffMean = 0, diffHi = 0, worstYawErr = 0;
    size_t nSame = 0, nDiff = 0;
    int    refused = 0;
};

// Three rooms of DIFFERENT shape off one corridor. Same-size rooms would be a
// symmetry test rather than a recognition test, so the sizes differ on purpose.
// Three rooms of DIFFERENT SHAPE in a line, each opening into the next. The
// path flies through all of them, so a descriptor taken inside one is
// describing that room.
//
// An earlier version put the rooms OFF a corridor and flew the corridor -- so
// every descriptor described the corridor, the rooms were never entered, and
// the "distinctive geometry" case was silently the degenerate case again. The
// scene has to make the vehicle actually visit the places being told apart.
static void buildRooms(VoxelWorld& w) {
    const float c = 0.25f;
    const int nx = 120, ny = 260, nz = 24;
    w.init(c, 0, 0, 0, nx, ny, nz);
    for (int x = 0; x < nx; ++x)
        for (int y = 0; y < ny; ++y) { w.set(x, y, 3); w.setTex(x, y, 3, .5f); }

    auto walls = [&](int x0, int y0, int x1, int y1) {
        for (int x = x0; x <= x1; ++x)
            for (int z = 4; z <= 16; ++z) {
                // Doorways at the shared edges, centred on x = 60 cells, so the
                // rooms are connected and the flight line passes through them.
                const bool door = (x > 54 && x < 66);
                if (!door) { w.set(x, y0, z); w.setTex(x, y0, z, .5f); }
                if (!door) { w.set(x, y1, z); w.setTex(x, y1, z, .5f); }
            }
        for (int y = y0; y <= y1; ++y)
            for (int z = 4; z <= 16; ++z) {
                w.set(x0, y, z); w.setTex(x0, y, z, .5f);
                w.set(x1, y, z); w.setTex(x1, y, z, .5f);
            }
    };
    walls(20, 10,  100, 70);    // WIDE and long   (20 x 15 m)
    walls(44, 70,   76, 150);   // NARROW and long ( 8 x 20 m)
    walls(10, 150, 110, 200);   // VERY wide, short (25 x 12.5 m)
}

static Outcome runScene(const char* label, const VoxelWorld& world,
                        const std::vector<Pose>& path, float startE, float startN) {
    CamParams cp;
    cp.width = 240; cp.height = 180; cp.hfovDeg = 70.f;
    cp.baselineM = 0.12f; cp.maxRangeM = 12.f;
    DepthCamera cam(cp);
    VoxelMapParams mp;
    mp.cell = 0.25f; mp.nx = 320; mp.ny = 320; mp.nz = 60;
    mp.maxCarveM = 11.f;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(mp.cell * cam.fpx() * cp.baselineM / cp.subpixelPx) * 0.75f;
    VoxelMap map;
    map.init(mp, startE, startN, 1.5f);
    map.seedFree(startE, startN, 1.5f, 1.0f);

    std::vector<PlaceDescriptor> descs;
    PlaceParams pp;
    for (size_t i = 0; i < path.size(); ++i) {
        const Pose& q = path[i];
        CamPose cpz; cpz.e = q.e; cpz.n = q.n; cpz.u = q.u; cpz.yawDeg = q.yaw;
        cv::Mat d = cam.renderTruth(world, cpz);
        map.integrate(d, cam, cpz);
        if (i % 4 == 0) descs.push_back(describePlace(map, q.e, q.n, q.u, q.yaw, pp));
    }

    Outcome o;
    std::vector<double> same, diff;
    for (size_t i = 0; i < descs.size(); ++i)
        for (size_t j = i + 1; j < descs.size(); ++j) {
            const double sep = std::hypot(descs[i].e - descs[j].e,
                                          descs[i].n - descs[j].n);
            const PlaceMatch m = matchPlace(descs[i], descs[j]);
            if (!m.usable) { ++o.refused; continue; }
            if (sep < 1.0) {
                same.push_back(m.score);
                double want = std::fabs(descs[i].yawDeg - descs[j].yawDeg);
                if (want > 180) want = 360 - want;
                double got = m.yawDeg; if (got > 180) got = 360 - got;
                o.worstYawErr = std::max(o.worstYawErr, std::fabs(got - want));
            } else if (sep > 6.0) diff.push_back(m.score);
        }
    auto mean = [](const std::vector<double>& v) {
        return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size(); };
    std::sort(same.begin(), same.end());
    std::sort(diff.begin(), diff.end());
    o.nSame = same.size(); o.nDiff = diff.size();
    o.sameMean = mean(same); o.diffMean = mean(diff);
    o.sameLo = same.empty() ? 0 : same.front();
    o.diffHi = diff.empty() ? 0 : diff.back();
    std::printf("  [%-11s] %2zu descriptors | same n=%2zu mean %.3f min %.3f | "
                "diff n=%2zu mean %.3f max %.3f | refused %d\n",
                label, descs.size(), o.nSame, o.sameMean, o.sameLo,
                o.nDiff, o.diffMean, o.diffHi, o.refused);
    return o;
}

int main() {
    std::printf("loop closure from voxel geometry\n");

    // WHAT THIS TEST CURRENTLY ESTABLISHES, stated plainly: the machinery runs
    // and is deterministic. It does NOT yet establish that place recognition
    // works on this map, and the numbers below are printed rather than asserted
    // for that reason. Asserting a threshold that does not exist would turn an
    // open question into a green tick.
    Outcome corridor, rooms;

    {   // NEGATIVE CONTROL: a straight corridor. Every point along it should
        // look like every other, so this is the scene where success would be
        // the suspicious result.
        MazeParams mzp; mzp.cell = 0.25f; mzp.seed = 3;
        mzp.cellsX = 6; mzp.cellsY = 6; mzp.corridorM = 4.0f;
        VoxelWorld world;
        float sx = 0, sy = 0, gx = 0, gy = 0;
        genMaze(world, mzp, &sx, &sy, &gx, &gy);
        std::vector<Pose> path;
        for (float y = sy; y <= sy + 14.f; y += 0.5f) path.push_back({sx, y, 1.5f, 0.f});
        for (float y = sy + 14.f; y >= sy; y -= 0.5f) path.push_back({sx, y, 1.5f, 180.f});
        corridor = runScene("corridor", world, path, sx, sy);
    }
    {   // Three rooms of different shape, flown through rather than past.
        VoxelWorld world; buildRooms(world);
        const float cx = 60 * 0.25f;
        std::vector<Pose> path;
        for (float y = 5.f;  y <= 47.f; y += 0.5f) path.push_back({cx, y, 1.5f, 0.f});
        for (float y = 47.f; y >= 5.f;  y -= 0.5f) path.push_back({cx, y, 1.5f, 180.f});
        rooms = runScene("three rooms", world, path, cx, 5.f);
    }

    // --- what IS established -------------------------------------------------
    check("descriptors are built and comparisons are made in both scenes",
          corridor.nSame + corridor.nDiff > 0 && rooms.nSame + rooms.nDiff > 0);
    check("both revisit and non-revisit pairs exist to compare",
          corridor.nSame > 0 && corridor.nDiff > 0
          && rooms.nSame > 0 && rooms.nDiff > 0);
    check("scores stay in range", corridor.sameLo >= 0 && corridor.diffHi <= 1.0001
          && rooms.sameLo >= 0 && rooms.diffHi <= 1.0001);

    // --- what is NOT established, reported rather than asserted --------------
    const bool corridorSep = corridor.sameLo > corridor.diffHi;
    const bool roomsSep    = rooms.sameLo > rooms.diffHi;
    std::printf("\n  SEPARATION (worst revisit vs best impostor -- a threshold "
                "needs a gap here):\n");
    std::printf("    corridor     %.3f vs %.3f   %s\n",
                corridor.sameLo, corridor.diffHi, corridorSep ? "GAP" : "none");
    std::printf("    three rooms  %.3f vs %.3f   %s\n",
                rooms.sameLo, rooms.diffHi, roomsSep ? "GAP" : "none");
    std::printf("  yaw recovered from the matching shift: corridor %.0f deg, "
                "rooms %.0f deg error (truth is a 180 deg reversal)\n",
                corridor.worstYawErr, rooms.worstYawErr);
    std::printf("\n  STATUS: place recognition is NOT yet usable on this map.\n"
                "  Three defects in the similarity metric were found and fixed on\n"
                "  the way here -- scoring empty-vs-empty bins as a perfect match\n"
                "  (which made everything match everything), cosine similarity on\n"
                "  near-binary heights (which crushed every score to ~0.95), and\n"
                "  comparing ranges truncated by UNEXPLORED space rather than by a\n"
                "  surface (which made an early visit and a later one describe the\n"
                "  same place differently). The scores now saturate instead, which\n"
                "  is a fourth problem and not a solved one.\n");

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
