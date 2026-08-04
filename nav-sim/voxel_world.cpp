#include "voxel_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>

namespace sim {

void VoxelWorld::init(float cell, float ox, float oy, float oz,
                      int nx, int ny, int nz) {
    cell_ = cell; ox_ = ox; oy_ = oy; oz_ = oz;
    nx_ = nx; ny_ = ny; nz_ = nz;
    size_t n = size_t(nx) * ny * nz;
    bits_.assign((n + 7) / 8, 0);
    tex_.assign(n, 0);
}

size_t VoxelWorld::solidCount() const {
    size_t c = 0;
    for (uint8_t b : bits_) c += size_t(__builtin_popcount(b));
    return c;
}

// Amanatides & Woo (1987). The classic; nothing clever, but the details matter:
// tMax must be initialised from the ray's position WITHIN its starting cell,
// not from the cell boundary, or every ray is biased by up to one cell.
float VoxelWorld::raycast(float px, float py, float pz,
                          float dx, float dy, float dz,
                          float maxRange, float* hitTex) const {
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-9f) return maxRange;
    dx /= len; dy /= len; dz /= len;

    int x, y, z;
    worldToCell(px, py, pz, x, y, z);

    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    const int sz = dz > 0 ? 1 : (dz < 0 ? -1 : 0);

    const float INF = 1e30f;
    // Distance along the ray to the next boundary crossing on each axis.
    auto firstT = [&](float p, float o, int c, int s, float d) -> float {
        if (s == 0) return INF;
        float bound = o + (c + (s > 0 ? 1 : 0)) * cell_;
        return (bound - p) / d;
    };
    float tMaxX = firstT(px, ox_, x, sx, dx);
    float tMaxY = firstT(py, oy_, y, sy, dy);
    float tMaxZ = firstT(pz, oz_, z, sz, dz);
    const float tDeltaX = sx ? cell_ / std::fabs(dx) : INF;
    const float tDeltaY = sy ? cell_ / std::fabs(dy) : INF;
    const float tDeltaZ = sz ? cell_ / std::fabs(dz) : INF;

    float t = 0.f;
    // A ray that starts outside the grid still has to be able to enter it, so
    // we march regardless of inBounds and only bail when we can no longer
    // re-enter (i.e. we have passed the far side on some axis).
    for (int guard = 0; guard < 100000; ++guard) {
        if (solid(x, y, z)) {
            if (hitTex) *hitTex = texAt(x, y, z);
            return t;
        }
        if (t > maxRange) break;
        // Cheap escape: once outside and moving further out on that axis, done.
        if ((x < 0 && sx <= 0) || (x >= nx_ && sx >= 0) ||
            (y < 0 && sy <= 0) || (y >= ny_ && sy >= 0) ||
            (z < 0 && sz <= 0) || (z >= nz_ && sz >= 0)) break;

        if (tMaxX < tMaxY && tMaxX < tMaxZ)      { t = tMaxX; x += sx; tMaxX += tDeltaX; }
        else if (tMaxY < tMaxZ)                  { t = tMaxY; y += sy; tMaxY += tDeltaY; }
        else                                     { t = tMaxZ; z += sz; tMaxZ += tDeltaZ; }
    }
    return maxRange;
}

// --- helpers ---------------------------------------------------------------

static void fillBox(VoxelWorld& w, float x0, float y0, float z0,
                    float x1, float y1, float z1, float tex) {
    int a0, b0, c0, a1, b1, c1;
    w.worldToCell(x0, y0, z0, a0, b0, c0);
    w.worldToCell(x1, y1, z1, a1, b1, c1);
    for (int z = c0; z <= c1; ++z)
        for (int y = b0; y <= b1; ++y)
            for (int x = a0; x <= a1; ++x) {
                w.set(x, y, z, true);
                w.setTex(x, y, z, tex);
            }
}

static void fillCylinder(VoxelWorld& w, float cx, float cy, float z0, float z1,
                         float radius, float tex) {
    int a0, b0, c0, a1, b1, c1;
    w.worldToCell(cx - radius, cy - radius, z0, a0, b0, c0);
    w.worldToCell(cx + radius, cy + radius, z1, a1, b1, c1);
    for (int z = c0; z <= c1; ++z)
        for (int y = b0; y <= b1; ++y)
            for (int x = a0; x <= a1; ++x) {
                float wx, wy, wz;
                w.cellCentre(x, y, z, wx, wy, wz);
                float dx = wx - cx, dy = wy - cy;
                if (dx * dx + dy * dy <= radius * radius) {
                    w.set(x, y, z, true);
                    w.setTex(x, y, z, tex);
                }
            }
}

// --- city ------------------------------------------------------------------

void genCity(VoxelWorld& w, const CityParams& p) {
    const int n = int(p.sizeM / p.cell);
    const int nz = int((p.hMax + 6.f) / p.cell);
    w.init(p.cell, 0, 0, 0, n, n, nz);
    std::mt19937 rng(p.seed);
    std::uniform_real_distribution<float> u01(0.f, 1.f);

    // Ground. Asphalt and pavement are moderately textured -- not the failure
    // case, but not rich either.
    fillBox(w, 0, 0, 0, p.sizeM, p.sizeM, p.cell * 0.99f, 0.45f);

    const float pitch = p.blockM + p.streetM;
    for (float by = p.streetM; by + p.blockM < p.sizeM; by += pitch) {
        for (float bx = p.streetM; bx + p.blockM < p.sizeM; bx += pitch) {
            // Split a block into 1-4 buildings with setbacks, so facades are not
            // one continuous plane -- corners and re-entrant angles are where a
            // planner actually has to make decisions.
            int split = 1 + int(u01(rng) * 3.99f);
            float sub = p.blockM / split;
            for (int j = 0; j < split; ++j) {
                for (int i = 0; i < split; ++i) {
                    if (u01(rng) < 0.15f) continue;          // courtyard / gap
                    float inset = 1.f + u01(rng) * 3.f;
                    float h = p.hMin + u01(rng) * (p.hMax - p.hMin);
                    // Glass curtain walls are the stereo killer: a large, flat,
                    // specular, essentially untextured facade. Give a fraction
                    // of buildings near-zero texture so the mapper has to cope.
                    float tex = (u01(rng) < p.glassFrac) ? 0.04f
                                                         : 0.55f + u01(rng) * 0.4f;
                    float x0 = bx + i * sub + inset, y0 = by + j * sub + inset;
                    float x1 = bx + (i + 1) * sub - inset, y1 = by + (j + 1) * sub - inset;
                    if (x1 - x0 < 2.f || y1 - y0 < 2.f) continue;
                    fillBox(w, x0, y0, 0, x1, y1, h, tex);
                }
            }
        }
    }
    // Street furniture: lamp posts. Thin vertical obstacles are the documented
    // hard case for stereo avoidance (sub-pixel at range), so the world must
    // contain them or the test is too easy.
    int posts = int(p.sizeM * p.sizeM / 900.f);
    for (int i = 0; i < posts; ++i) {
        float x = u01(rng) * p.sizeM, y = u01(rng) * p.sizeM;
        fillCylinder(w, x, y, 0, 5.f + u01(rng) * 3.f, 0.09f, 0.6f);
    }
}

// --- forest ----------------------------------------------------------------

void genForest(VoxelWorld& w, const ForestParams& p) {
    const int n = int(p.sizeM / p.cell);
    const int nz = int((p.topM + 4.f) / p.cell);
    w.init(p.cell, 0, 0, 0, n, n, nz);
    std::mt19937 rng(p.seed);
    std::uniform_real_distribution<float> u01(0.f, 1.f);
    std::normal_distribution<float> gauss(0.f, 1.f);

    // Undulating ground rather than a plane: a flat floor lets a planner cheat
    // by holding one altitude, and hides the "is the ground carved as free
    // space" failure entirely.
    const int gn = n;
    std::vector<float> gz(size_t(gn) * gn);
    for (int y = 0; y < gn; ++y)
        for (int x = 0; x < gn; ++x) {
            float fx = x * p.cell / p.sizeM, fy = y * p.cell / p.sizeM;
            gz[size_t(y) * gn + x] =
                1.2f * std::sin(fx * 6.28f * 1.5f) * std::cos(fy * 6.28f * 1.1f)
              + 0.5f * std::sin(fx * 6.28f * 4.3f + 1.7f);
        }
    for (int y = 0; y < gn; ++y)
        for (int x = 0; x < gn; ++x) {
            float wx, wy, wz;
            w.cellCentre(x, y, 0, wx, wy, wz);
            float top = gz[size_t(y) * gn + x] + 2.0f;
            int zt; int dx, dy;
            w.worldToCell(wx, wy, top, dx, dy, zt);
            // Forest floor: litter, moss, roots. Genuinely well textured.
            for (int z = 0; z <= zt; ++z) { w.set(x, y, z, true); w.setTex(x, y, z, 0.7f); }
        }

    const float areaHa = (p.sizeM * p.sizeM) / 10000.f;
    const int nTrees = int(areaHa * p.stemsPerHa);
    for (int i = 0; i < nTrees; ++i) {
        float x = u01(rng) * p.sizeM, y = u01(rng) * p.sizeM;
        int cx, cy, cz;
        w.worldToCell(x, y, 0, cx, cy, cz);
        float base = 0.f;
        if (cx >= 0 && cy >= 0 && cx < gn && cy < gn) base = gz[size_t(cy) * gn + cx] + 2.0f;

        float dbh = p.dbhMinM + u01(rng) * (p.dbhMaxM - p.dbhMinM);
        // Height-diameter allometry: taller trees are thicker. Roughly h ~ dbh^0.6
        // scaled into the plot's height range, plus scatter.
        float hf = std::pow((dbh - p.dbhMinM) / (p.dbhMaxM - p.dbhMinM + 1e-6f), 0.6f);
        float h = (0.45f + 0.55f * hf) * p.topM * (0.85f + 0.3f * u01(rng));

        // Trunk. Bark is strongly textured -- trunks are the one thing stereo
        // reliably sees in a forest, which is why forests are navigable at all.
        fillCylinder(w, x, y, base, base + h, dbh * 0.5f, 0.85f);

        // Crown: a cluster of foliage blobs from canopyBase upward. Foliage is
        // texture-rich but geometrically thin and porous, which is exactly the
        // regime where a stereo matcher returns confident nonsense. Modelled as
        // sparse occupancy so rays sometimes pass through.
        float cb = base + h * p.canopyBase;
        int blobs = 6 + int(u01(rng) * 8);
        for (int b = 0; b < blobs; ++b) {
            float bz = cb + u01(rng) * (base + h - cb);
            float spread = 0.8f + 2.2f * (1.f - (bz - cb) / std::max(0.1f, base + h - cb));
            float bx = x + gauss(rng) * spread * 0.5f;
            float by = y + gauss(rng) * spread * 0.5f;
            float r = 0.5f + u01(rng) * 1.1f;
            int a0, b0, c0, a1, b1, c1;
            w.worldToCell(bx - r, by - r, bz - r, a0, b0, c0);
            w.worldToCell(bx + r, by + r, bz + r, a1, b1, c1);
            for (int z = c0; z <= c1; ++z)
                for (int yy = b0; yy <= b1; ++yy)
                    for (int xx = a0; xx <= a1; ++xx) {
                        float wx, wy, wz;
                        w.cellCentre(xx, yy, z, wx, wy, wz);
                        float ddx = wx - bx, ddy = wy - by, ddz = wz - bz;
                        if (ddx * ddx + ddy * ddy + ddz * ddz > r * r) continue;
                        if (u01(rng) > 0.55f) continue;   // porous: 55% fill
                        w.set(xx, yy, z, true);
                        w.setTex(xx, yy, z, 0.8f);
                    }
        }
    }

    // Undergrowth: low scrub. Cheap, and it is what makes "fly low" unsafe.
    int scrub = int(p.sizeM * p.sizeM * p.undergrowth / 4.f);
    for (int i = 0; i < scrub; ++i) {
        float x = u01(rng) * p.sizeM, y = u01(rng) * p.sizeM;
        int cx, cy, cz; w.worldToCell(x, y, 0, cx, cy, cz);
        float base = 0.f;
        if (cx >= 0 && cy >= 0 && cx < gn && cy < gn) base = gz[size_t(cy) * gn + cx] + 2.0f;
        fillCylinder(w, x, y, base, base + 0.4f + u01(rng) * 1.2f,
                     0.3f + u01(rng) * 0.5f, 0.75f);
    }
}

// --- importers -------------------------------------------------------------

bool loadOsmBuildings(VoxelWorld& w, const std::string& path,
                      float cell, float pad) {
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "loadOsmBuildings: cannot open %s\n", path.c_str()); return false; }

    struct Bld { float h; std::vector<float> xs, ys; };
    std::vector<Bld> blds;
    float minx = 1e30f, miny = 1e30f, maxx = -1e30f, maxy = -1e30f, maxh = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        Bld b; if (!(ss >> b.h)) continue;
        float x, y;
        while (ss >> x >> y) {
            b.xs.push_back(x); b.ys.push_back(y);
            minx = std::min(minx, x); maxx = std::max(maxx, x);
            miny = std::min(miny, y); maxy = std::max(maxy, y);
        }
        if (b.xs.size() >= 3) { maxh = std::max(maxh, b.h); blds.push_back(std::move(b)); }
    }
    if (blds.empty()) { std::fprintf(stderr, "loadOsmBuildings: no polygons in %s\n", path.c_str()); return false; }

    const float ox = minx - pad, oy = miny - pad;
    const int nx = int((maxx - minx + 2 * pad) / cell) + 1;
    const int ny = int((maxy - miny + 2 * pad) / cell) + 1;
    const int nz = int((maxh + 6.f) / cell) + 1;
    w.init(cell, ox, oy, 0, nx, ny, nz);
    fillBox(w, ox, oy, 0, ox + nx * cell, oy + ny * cell, cell * 0.99f, 0.45f);

    // Even-odd point-in-polygon per column. O(cells x edges) and entirely fast
    // enough offline; this runs once at load, not per frame.
    for (const Bld& b : blds) {
        float bx0 = *std::min_element(b.xs.begin(), b.xs.end());
        float bx1 = *std::max_element(b.xs.begin(), b.xs.end());
        float by0 = *std::min_element(b.ys.begin(), b.ys.end());
        float by1 = *std::max_element(b.ys.begin(), b.ys.end());
        int a0, c0, d0, a1, c1, d1;
        w.worldToCell(bx0, by0, 0, a0, c0, d0);
        w.worldToCell(bx1, by1, b.h, a1, c1, d1);
        for (int y = c0; y <= c1; ++y) {
            for (int x = a0; x <= a1; ++x) {
                float wx, wy, wz; w.cellCentre(x, y, 0, wx, wy, wz);
                bool in = false;
                for (size_t i = 0, j = b.xs.size() - 1; i < b.xs.size(); j = i++) {
                    if (((b.ys[i] > wy) != (b.ys[j] > wy)) &&
                        (wx < (b.xs[j] - b.xs[i]) * (wy - b.ys[i]) /
                                  (b.ys[j] - b.ys[i] + 1e-12f) + b.xs[i]))
                        in = !in;
                }
                if (!in) continue;
                for (int z = 0; z <= d1; ++z) { w.set(x, y, z, true); w.setTex(x, y, z, 0.5f); }
            }
        }
    }
    std::fprintf(stderr, "loadOsmBuildings: %zu buildings, grid %dx%dx%d @ %.2f m\n",
                 blds.size(), nx, ny, nz, cell);
    return true;
}

bool loadLidarXyz(VoxelWorld& w, const std::string& path, float cell, bool fillGround) {
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "loadLidarXyz: cannot open %s\n", path.c_str()); return false; }
    std::vector<float> xs, ys, zs;
    float minx = 1e30f, miny = 1e30f, minz = 1e30f;
    float maxx = -1e30f, maxy = -1e30f, maxz = -1e30f;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        float x, y, z;
        if (!(ss >> x >> y >> z)) continue;
        xs.push_back(x); ys.push_back(y); zs.push_back(z);
        minx = std::min(minx, x); maxx = std::max(maxx, x);
        miny = std::min(miny, y); maxy = std::max(maxy, y);
        minz = std::min(minz, z); maxz = std::max(maxz, z);
    }
    if (xs.empty()) { std::fprintf(stderr, "loadLidarXyz: no points in %s\n", path.c_str()); return false; }

    const int nx = int((maxx - minx) / cell) + 2;
    const int ny = int((maxy - miny) / cell) + 2;
    const int nz = int((maxz - minz) / cell) + 2;
    w.init(cell, minx, miny, minz, nx, ny, nz);
    // Lowest return per column, used to close the ground.
    std::vector<int> lowest(size_t(nx) * ny, INT32_MAX);
    for (size_t i = 0; i < xs.size(); ++i) {
        int x, y, z; w.worldToCell(xs[i], ys[i], zs[i], x, y, z);
        w.set(x, y, z, true);
        w.setTex(x, y, z, 0.75f);
        if (x >= 0 && y >= 0 && x < nx && y < ny)
            lowest[size_t(y) * nx + x] = std::min(lowest[size_t(y) * nx + x], z);
    }
    if (fillGround) {
        // A one-voxel LiDAR shell is not a surface -- rays slip between returns
        // and the map carves free space through the ground. Close every column
        // from its lowest return down.
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                int lo = lowest[size_t(y) * nx + x];
                if (lo == INT32_MAX) continue;
                for (int z = 0; z < lo; ++z) { w.set(x, y, z, true); w.setTex(x, y, z, 0.7f); }
            }
    }
    std::fprintf(stderr, "loadLidarXyz: %zu points, grid %dx%dx%d @ %.2f m\n",
                 xs.size(), nx, ny, nz, cell);
    return true;
}

}  // namespace sim
