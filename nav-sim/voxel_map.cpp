#include "voxel_map.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

namespace sim {

// Half-width of the height colour key, metres: red at your altitude, green at
// HEIGHT_KEY_M below, blue at HEIGHT_KEY_M above. Shared by the isometric and
// first-person renderers so the two panes cannot disagree about what a colour
// means -- that is the whole value of having a key.
//
// It was 8 m, which was wrong in both views for the same reason: the terrain
// sits ~2 m under the aircraft, and at 8 m that is still three quarters of the
// way to "at your altitude". The ground rendered red. Red has to mean "this
// will hit you", so the band has to be about the size of the thing that would.
static const float HEIGHT_KEY_M = 3.5f;

void VoxelMap::init(const VoxelMapParams& p, float cx, float cy, float cz) {
    p_ = p;
    log_.assign(size_t(p_.nx) * p_.ny * p_.nz, 0.f);
    ox_ = cx - p_.nx * p_.cell * 0.5f;
    oy_ = cy - p_.ny * p_.cell * 0.5f;
    oz_ = cz - p_.nz * p_.cell * 0.25f;   // more headroom above than below
}

void VoxelMap::worldToCell(float wx, float wy, float wz, int& x, int& y, int& z) const {
    x = int(std::floor((wx - ox_) / p_.cell));
    y = int(std::floor((wy - oy_) / p_.cell));
    z = int(std::floor((wz - oz_) / p_.cell));
}
void VoxelMap::cellCentre(int x, int y, int z, float& wx, float& wy, float& wz) const {
    wx = ox_ + (x + 0.5f) * p_.cell;
    wy = oy_ + (y + 0.5f) * p_.cell;
    wz = oz_ + (z + 0.5f) * p_.cell;
}

VoxelMap::State VoxelMap::stateAt(float wx, float wy, float wz) const {
    int x, y, z; worldToCell(wx, wy, wz, x, y, z);
    if (!inBounds(x, y, z)) return UNKNOWN;
    float l = log_[idx(x, y, z)];
    if (l > p_.occThresh) return OCCUPIED;
    if (l < p_.freeThresh) return FREE;
    return UNKNOWN;
}

VoxelMap::Hit VoxelMap::raycast(float px, float py, float pz,
                                float dx, float dy, float dz,
                                float maxRange) const {
    Hit h;
    float len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len < 1e-9f) return h;
    dx /= len; dy /= len; dz /= len;

    int x, y, z; worldToCell(px, py, pz, x, y, z);
    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    const int sz = dz > 0 ? 1 : (dz < 0 ? -1 : 0);

    const float INF = 1e30f;
    auto firstT = [&](float p, float o, int c, int s, float d) -> float {
        if (s == 0) return INF;
        float bound = o + (c + (s > 0 ? 1 : 0)) * p_.cell;
        return (bound - p) / d;
    };
    float tMaxX = firstT(px, ox_, x, sx, dx);
    float tMaxY = firstT(py, oy_, y, sy, dy);
    float tMaxZ = firstT(pz, oz_, z, sz, dz);
    const float tDX = sx ? p_.cell / std::fabs(dx) : INF;
    const float tDY = sy ? p_.cell / std::fabs(dy) : INF;
    const float tDZ = sz ? p_.cell / std::fabs(dz) : INF;

    float t = 0.f;
    int face = 5;
    for (int guard = 0; guard < 100000; ++guard) {
        if (inBounds(x, y, z)) {
            float l = log_[idx(x, y, z)];
            if (l > p_.occThresh) {
                h.hit = true; h.t = t; h.face = face;
                h.x = x; h.y = y; h.z = z;
                return h;
            }
        }
        if (t > maxRange) break;
        if ((x < 0 && sx <= 0) || (x >= p_.nx && sx >= 0) ||
            (y < 0 && sy <= 0) || (y >= p_.ny && sy >= 0) ||
            (z < 0 && sz <= 0) || (z >= p_.nz && sz >= 0)) break;

        float tPrev = t;
        if (tMaxX < tMaxY && tMaxX < tMaxZ) { t = tMaxX; x += sx; tMaxX += tDX; face = sx > 0 ? 0 : 1; }
        else if (tMaxY < tMaxZ)             { t = tMaxY; y += sy; tMaxY += tDY; face = sy > 0 ? 2 : 3; }
        else                                { t = tMaxZ; z += sz; tMaxZ += tDZ; face = sz > 0 ? 4 : 5; }
        // Segment length just travelled, attributed to the cell we were in.
        if (inBounds(x, y, z)) {
            float l = logAt(x, y, z);
            if (!(l < p_.freeThresh) && !(l > p_.occThresh)) h.unknownM += t - tPrev;
        } else {
            h.unknownM += t - tPrev;      // outside the map is unknown too
        }
    }
    h.t = std::min(t, maxRange);
    return h;
}

cv::Mat VoxelMap::fpvImage(float px, float py, float pz,
                           float yawDeg, float pitchDeg,
                           int outPx, float hfovDeg, float maxRange) const {
    cv::Mat img(outPx, outPx, CV_8UC3);
    const float f = (outPx * 0.5f) / std::tan(hfovDeg * 0.5f * PI_F / 180.f);
    const float c = (outPx - 1) * 0.5f;
    const float cy_ = std::cos(yawDeg * PI_F / 180.f), sy_ = std::sin(yawDeg * PI_F / 180.f);
    const float cp = std::cos(pitchDeg * PI_F / 180.f), sp = std::sin(pitchDeg * PI_F / 180.f);
    // Same colour key as the isometric pane so the two views agree: red is at
    // your altitude and is what you would hit, green below, blue above.
    const cv::Vec3f LOW(90, 150, 80), AT(55, 60, 235), HIGH(210, 160, 90);
    // Face shading, and the ordering matters more than the values: a cube grid
    // with one brightness per cube reads as noise, and with one per FACE reads
    // as geometry. 0/1 = -x/+x, 2/3 = -y/+y, 4/5 = -z/+z (5 is a top face).
    static const float FACE[6] = {0.62f, 0.62f, 0.45f, 0.45f, 0.30f, 1.00f};
    const cv::Vec3f FOG(238, 240, 244);      // matches the iso pane background

    for (int v = 0; v < outPx; ++v) {
        cv::Vec3b* row = img.ptr<cv::Vec3b>(v);
        for (int u = 0; u < outPx; ++u) {
            // Camera frame: +X right, +Y down, +Z forward, then pitch, then yaw
            // clockwise from North -- identical to DepthCamera::rayFor, because
            // two conventions for the same thing is a bug waiting to happen.
            float rx = (u - c) / f, ry = (v - c) / f;
            float y2 = ry * cp - sp, z2 = ry * sp + cp;
            float fE = rx, fN = z2, fU = -y2;
            float dx = fE * cy_ + fN * sy_;
            float dy = -fE * sy_ + fN * cy_;
            float dz = fU;

            Hit h = raycast(px, py, pz, dx, dy, dz, maxRange);
            cv::Vec3f col;
            if (h.hit) {
                float wx, wy, wz; cellCentre(h.x, h.y, h.z, wx, wy, wz);
                float k = (wz - pz) / HEIGHT_KEY_M;
                k = std::max(-1.f, std::min(1.f, k));
                col = (k < 0.f) ? LOW + (AT - LOW) * (1.f + k) : AT + (HIGH - AT) * k;
                col *= FACE[h.face];
                // Distance haze, so depth is readable without a depth number.
                float d = std::min(1.f, h.t / maxRange);
                col = col * (1.f - 0.55f * d) + FOG * (0.55f * d);
                // Uncertainty fog: how much UNKNOWN we looked through to see
                // this. A surface behind 4 m of unknown is a guess, and it
                // should look like one.
                float ufog = std::min(0.85f, h.unknownM / 6.f);
                col = col * (1.f - ufog) + FOG * ufog;
            } else {
                col = FOG;
            }
            row[u] = cv::Vec3b(uchar(col[0]), uchar(col[1]), uchar(col[2]));
        }
    }
    // Caption strips, top and bottom. Unlike every other pane here the content
    // of this one is arbitrary -- a trunk can be anywhere -- so labels drawn
    // straight onto it become unreadable exactly when the view is interesting.
    // A translucent band costs two rectangles and makes them always legible.
    for (const cv::Rect& r : {cv::Rect(0, 0, outPx, 50),
                              cv::Rect(0, outPx - 44, outPx, 44)}) {
        cv::Mat band = img(r);
        cv::Mat wash(band.size(), band.type(), cv::Scalar(238, 240, 244));
        cv::addWeighted(band, 0.55, wash, 0.45, 0.0, band);
    }
    return img;
}

// Carve free space along the ray, mark the endpoint occupied. Same DDA as the
// world raycaster; kept separate because this one writes rather than reads and
// must stop exactly at `range`.
// carveTo: how far to mark FREE. hitAt: where to mark OCCUPIED, or < 0 for
// "this return is too far to trust as an obstacle -- carve only".
void VoxelMap::rayInsert(float px, float py, float pz,
                         float dx, float dy, float dz,
                         float carveTo, float hitAt) {
    const float range = carveTo;
    float len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len < 1e-9f) return;
    dx /= len; dy /= len; dz /= len;

    int x, y, z; worldToCell(px, py, pz, x, y, z);
    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    const int sz = dz > 0 ? 1 : (dz < 0 ? -1 : 0);
    const float INF = 1e30f;
    auto firstT = [&](float p, float o, int c, int s, float d) -> float {
        if (s == 0) return INF;
        float bound = o + (c + (s > 0 ? 1 : 0)) * p_.cell;
        return (bound - p) / d;
    };
    float tMaxX = firstT(px, ox_, x, sx, dx);
    float tMaxY = firstT(py, oy_, y, sy, dy);
    float tMaxZ = firstT(pz, oz_, z, sz, dz);
    const float tdx = sx ? p_.cell / std::fabs(dx) : INF;
    const float tdy = sy ? p_.cell / std::fabs(dy) : INF;
    const float tdz = sz ? p_.cell / std::fabs(dz) : INF;

    float t = 0.f;
    for (int guard = 0; guard < 20000 && t < range; ++guard) {
        if (inBounds(x, y, z)) {
            float& l = log_[idx(x, y, z)];
            l = std::max(l - p_.lMiss, -p_.lClamp);
        }
        if (tMaxX < tMaxY && tMaxX < tMaxZ) { t = tMaxX; x += sx; tMaxX += tdx; }
        else if (tMaxY < tMaxZ)             { t = tMaxY; y += sy; tMaxY += tdy; }
        else                                { t = tMaxZ; z += sz; tMaxZ += tdz; }
    }
    // Endpoint: occupied. Note this happens AFTER the free carve so a cell that
    // is both traversed and terminated-in ends up occupied, which is correct --
    // the return is the stronger evidence.
    if (hitAt < 0.f) return;         // far return: free space only, no obstacle
    int ex, ey, ez;
    worldToCell(px + dx * hitAt, py + dy * hitAt, pz + dz * hitAt, ex, ey, ez);
    if (inBounds(ex, ey, ez)) {
        float& l = log_[idx(ex, ey, ez)];
        l = std::min(l + p_.lHit, p_.lClamp);
    }
}

void VoxelMap::integrate(const cv::Mat& depth, const DepthCamera& cam,
                         const CamPose& pose) {
    for (int v = 0; v < depth.rows; ++v) {
        const float* row = depth.ptr<float>(v);
        for (int u = 0; u < depth.cols; ++u) {
            float r = row[u];
            // THE RULE. No measurement -> no information. Not free, not
            // occupied, nothing. Do not be tempted to carve free space out to
            // maxRange on a miss "because there was clearly nothing there" --
            // on an untextured wall there very clearly was.
            if (!(r > 0.f)) continue;
            float dx, dy, dz; cam.rayFor(pose, u, v, dx, dy, dz);
            // How far can this ray's free space be trusted? Up to the hit minus
            // a few sigma of its own depth uncertainty, capped at maxCarveM.
            float sig = p_.depthSigCoef > 0.f ? p_.depthSigCoef * r * r : 0.f;
            float carve = std::min(p_.maxCarveM, r - p_.carveSigK * sig);
            bool markHit = (r <= p_.maxIntegM);
            if (carve <= 0.f && !markHit) continue;
            rayInsert(pose.e, pose.n, pose.u, dx, dy, dz,
                      std::max(0.f, carve), markHit ? r : -1.f);
        }
    }
}

void VoxelMap::recentre(float cx, float cy, float cz) {
    float nox = cx - p_.nx * p_.cell * 0.5f;
    float noy = cy - p_.ny * p_.cell * 0.5f;
    float noz = cz - p_.nz * p_.cell * 0.25f;
    int sx = int(std::lround((nox - ox_) / p_.cell));
    int sy = int(std::lround((noy - oy_) / p_.cell));
    int sz = int(std::lround((noz - oz_) / p_.cell));
    if (!sx && !sy && !sz) return;

    std::vector<float> out(log_.size(), 0.f);
    for (int z = 0; z < p_.nz; ++z)
        for (int y = 0; y < p_.ny; ++y)
            for (int x = 0; x < p_.nx; ++x) {
                int ox = x + sx, oy = y + sy, oz = z + sz;
                if (ox < 0 || oy < 0 || oz < 0 ||
                    ox >= p_.nx || oy >= p_.ny || oz >= p_.nz) continue;
                out[idx(x, y, z)] = log_[idx(ox, oy, oz)];
            }
    log_.swap(out);
    ox_ += sx * p_.cell; oy_ += sy * p_.cell; oz_ += sz * p_.cell;
}

VoxelMap::Score VoxelMap::score(const VoxelWorld& truth, float cx, float cy, float cz,
                                float radiusM, float maxZ) const {
    Score s;
    for (int z = 0; z < p_.nz; ++z)
        for (int y = 0; y < p_.ny; ++y)
            for (int x = 0; x < p_.nx; ++x) {
                float wx, wy, wz; cellCentre(x, y, z, wx, wy, wz);
                float dx = wx - cx, dy = wy - cy;
                if (dx * dx + dy * dy > radiusM * radiusM) continue;
                if (wz > maxZ) continue;
                int tx, ty, tz; truth.worldToCell(wx, wy, wz, tx, ty, tz);
                if (!truth.inBounds(tx, ty, tz)) continue;
                bool solid = truth.solid(tx, ty, tz);
                float l = log_[idx(x, y, z)];
                State st = (l > p_.occThresh) ? OCCUPIED
                         : (l < p_.freeThresh) ? FREE : UNKNOWN;
                ++s.total;
                if (st == UNKNOWN) { ++s.unknown; continue; }
                ++s.observed;
                if (st == OCCUPIED) { if (solid) ++s.occTP; else ++s.occFP; }
                else { if (solid) { ++s.falseFree; } else { ++s.freeTP; } }
            }
    s.freeFP = s.falseFree;
    return s;
}

// --- visualisation ---------------------------------------------------------

cv::Mat VoxelMap::sliceImage(float wz, int outPx) const {
    int z; { int a, b; worldToCell(ox_, oy_, wz, a, b, z); }
    cv::Mat img(p_.ny, p_.nx, CV_8UC3, cv::Scalar(128, 128, 128));
    if (z >= 0 && z < p_.nz) {
        for (int y = 0; y < p_.ny; ++y)
            for (int x = 0; x < p_.nx; ++x) {
                float l = log_[idx(x, y, z)];
                cv::Vec3b c(128, 128, 128);                       // unknown
                if (l > p_.occThresh) c = cv::Vec3b(40, 40, 40);  // occupied
                else if (l < p_.freeThresh) c = cv::Vec3b(245, 245, 245);
                img.at<cv::Vec3b>(p_.ny - 1 - y, x) = c;
            }
    }
    cv::Mat out; cv::resize(img, out, cv::Size(outPx, outPx), 0, 0, cv::INTER_NEAREST);
    return out;
}

// Simple painter's-algorithm isometric. Draw back-to-front so nearer cells
// overwrite; height gives colour. Not pretty, but it is honest about what the
// map contains and needs no GPU or extra dependency.
static void isoDraw(cv::Mat& img, int gx, int gy, int gz,
                    int nx, int ny, int nz, float cell, int outPx,
                    const cv::Vec3b& col, float yawDeg = 0.f, int rad = 1) {
    float sx = float(outPx) / float(nx + ny);
    // Rotate in the ground plane about the grid centre before projecting, so
    // the model can be spun without rebuilding anything.
    float cxg = nx * 0.5f, cyg = ny * 0.5f;
    float a = yawDeg * sim::PI_F / 180.f, ca = std::cos(a), sa = std::sin(a);
    float rx = (gx - cxg) * ca - (gy - cyg) * sa + cxg;
    float ry = (gx - cxg) * sa + (gy - cyg) * ca + cyg;
    int px = int((rx - ry) * sx * 0.5f + outPx * 0.5f);
    int py = int((rx + ry) * sx * 0.25f - gz * sx * 0.5f + outPx * 0.30f);
    if (px < rad || py < rad || px >= img.cols - rad || py >= img.rows - rad) return;
    cv::rectangle(img, cv::Point(px - rad, py - rad), cv::Point(px + rad, py + rad),
                  cv::Scalar(col[0], col[1], col[2]), cv::FILLED);
}

// Isometric CUBE, three visible faces. Top is brightest, then left, then right,
// which is what makes a grid of them read as solid blocks rather than as dots.
static void isoCube(cv::Mat& img, float rx, float ry, float gz, float s,
                    int outPx, const cv::Vec3b& base) {
    const float hw = s * 0.5f, hh = s * 0.25f, vh = s * 0.5f;
    float px = (rx - ry) * hw + outPx * 0.5f;
    float py = (rx + ry) * hh - gz * vh + outPx * 0.58f;
    if (px < -s || py < -s || px > img.cols + s || py > img.rows + s) return;
    auto P = [&](float dx, float dy) { return cv::Point(int(px + dx), int(py + dy)); };
    auto shade = [&](float k) {
        return cv::Scalar(std::min(255.f, base[0]*k), std::min(255.f, base[1]*k),
                          std::min(255.f, base[2]*k));
    };
    cv::Point top[4]  = {P(0,-hh), P(hw,0),  P(0,hh),      P(-hw,0)};
    cv::Point left[4] = {P(-hw,0), P(0,hh),  P(0,hh+vh),   P(-hw,vh)};
    cv::Point rght[4] = {P(hw,0),  P(0,hh),  P(0,hh+vh),   P(hw,vh)};
    cv::fillConvexPoly(img, left, 4, shade(0.62f));
    cv::fillConvexPoly(img, rght, 4, shade(0.42f));
    cv::fillConvexPoly(img, top,  4, shade(1.00f));
}

cv::Mat VoxelMap::isoImage(int outPx, float maxZ, float yawDeg, IsoView* view,
                           float blockM, float spanM) const {
    cv::Mat img(outPx, outPx, CV_8UC3, cv::Scalar(238, 240, 244));
    // RENDER PITCH, and this is why the pane looked like static rather than like
    // blocks. The old code drew one cube per map cell: 240 x 240 cells across a
    // 440 px pane is 1.05 px per cube, so every "cube" was a single pixel and
    // the three shaded faces were invisible. Cube rendering was there; you just
    // could not see it.
    //
    // So draw at a chosen pitch in METRES instead, independent of the map's
    // resolution. 1.5 m blocks over a 60 m map is 40 across, ~6 px each -- big
    // enough to read as geometry. The map itself is unchanged; this is purely
    // how it is displayed, and the pane is labelled with the pitch so nobody
    // mistakes the display resolution for the map's.
    //
    // Downsampling is an OR, never an average: a block is drawn if ANY cell in
    // it is occupied. On a map whose whole purpose is not to lose obstacles,
    // the only acceptable rounding is the one that keeps them.
    const int step = std::max(1, int(std::lround(blockM / p_.cell)));
    const int nbx = (p_.nx + step - 1) / step, nby = (p_.ny + step - 1) / step;
    const int nbz = (p_.nz + step - 1) / step;
    const float a = yawDeg * PI_F / 180.f, ca = std::cos(a), sa = std::sin(a);
    const float cxg = float(nbx) * 0.5f, cyg = float(nby) * 0.5f;
    // Span in blocks, half-width. The rotation means a square crop can present
    // its diagonal to the viewer, so scale on the diagonal to guarantee the
    // requested span is visible at every yaw rather than clipping at 45 deg.
    const int spanB = (spanM > 0.f)
        ? std::max(4, std::min(std::min(nbx, nby) / 2,
                               int(std::lround(spanM * 0.5f / (step * p_.cell)))))
        : std::min(nbx, nby) / 2;
    const float s = float(outPx) / (2.f * float(spanB) * 1.42f) * 1.15f;
    // Vertical datum: the vehicle's own height, which is the map's z centre.
    // Anchoring on the map floor instead pushes tall city blocks off the top of
    // the pane and leaves the aircraft's own altitude band unreadable.
    const float czg = float(nbz) * 0.5f;
    if (view) *view = IsoView{outPx, step, s, ca, sa, cxg, cyg, czg,
                              ox_, oy_, oz_, p_.cell, true};

    // Block occupancy: OR over the cells inside it.
    std::vector<uint8_t> occ(size_t(nbx) * nby * nbz, 0);
    auto bidx = [&](int bx, int by, int bz) {
        return (size_t(bz) * nby + by) * nbx + bx;
    };
    for (int z = 0; z < p_.nz; ++z)
        for (int y = 0; y < p_.ny; ++y) {
            const float* row = &log_[idx(0, y, z)];
            for (int x = 0; x < p_.nx; ++x)
                if (row[x] > p_.occThresh) occ[bidx(x/step, y/step, z/step)] = 1;
        }

    // Painter's algorithm: far blocks first. After rotation the depth key is
    // (rx + ry), so sort on it and draw ascending.
    struct Cell { float rx, ry, z; float f; };
    std::vector<Cell> cells;
    cells.reserve(8000);
    const int bx0 = std::max(0, int(cxg) - spanB), bx1 = std::min(nbx, int(cxg) + spanB);
    const int by0 = std::max(0, int(cyg) - spanB), by1 = std::min(nby, int(cyg) + spanB);
    for (int bz = 0; bz < nbz; ++bz)
        for (int by = by0; by < by1; ++by)
            for (int bx = bx0; bx < bx1; ++bx) {
                if (!occ[bidx(bx, by, bz)]) continue;
                float wz = oz_ + (bz + 0.5f) * step * p_.cell;
                if (wz > maxZ) continue;
                // EXPOSED-FACE CULLING. A block with occupied neighbours on all
                // six sides is invisible, and in a solid map that is most of
                // them. Same trick Minecraft uses, and it is what makes filled
                // cubes affordable: typically 90%+ never reach the rasteriser.
                auto solidB = [&](int x, int y, int z) {
                    return x >= 0 && y >= 0 && z >= 0 && x < nbx && y < nby && z < nbz
                           && occ[bidx(x, y, z)];
                };
                if (solidB(bx+1,by,bz) && solidB(bx-1,by,bz) &&
                    solidB(bx,by+1,bz) && solidB(bx,by-1,bz) &&
                    solidB(bx,by,bz+1) && solidB(bx,by,bz-1)) continue;
                float dx = bx - cxg, dy = by - cyg;
                // Colour key is height RELATIVE TO THE VEHICLE, not absolute
                // height. Absolute height coloured the whole model one shade of
                // blue -- the map spans 24 m and a forest occupies 8 of them --
                // and it answered a question nobody asked. Relative height
                // answers "what is at my altitude", i.e. what I am going to
                // hit, which is the reason to look at this pane at all.
                cells.push_back({dx*ca - dy*sa, dx*sa + dy*ca, bz - czg,
                                 (wz - (oz_ + p_.nz * p_.cell * 0.5f)) / HEIGHT_KEY_M});
            }
    std::sort(cells.begin(), cells.end(),
              [](const Cell& a, const Cell& b) { return (a.rx + a.ry) < (b.rx + b.ry); });
    // BGR anchors: below the aircraft, at its altitude, above it.
    const cv::Vec3f LOW(90, 150, 80), AT(55, 60, 235), HIGH(210, 160, 90);
    for (const Cell& c : cells) {
        float t = std::max(-1.f, std::min(1.f, c.f));
        cv::Vec3f m = (t < 0.f) ? LOW  + (AT - LOW ) * (1.f + t)
                                : AT   + (HIGH - AT) * t;
        isoCube(img, c.rx, c.ry, c.z, s, outPx,
                cv::Vec3b(uchar(m[0]), uchar(m[1]), uchar(m[2])));
    }
    return img;
}

static cv::Mat isoOfWorld(const VoxelWorld& w, int outPx, float maxZ) {
    cv::Mat img(outPx, outPx, CV_8UC3, cv::Scalar(250, 248, 245));
    int step = std::max(1, w.nx() / 200);
    for (int z = 0; z < w.nz(); z += step)
        for (int y = w.ny() - 1; y >= 0; y -= step)
            for (int x = 0; x < w.nx(); x += step) {
                if (!w.solid(x, y, z)) continue;
                float wx, wy, wz; w.cellCentre(x, y, z, wx, wy, wz);
                if (wz > maxZ) continue;
                float f = std::min(1.f, std::max(0.f, wz / std::max(1.f, maxZ)));
                cv::Vec3b col(uchar(220 - 150 * f), uchar(90 + 60 * f), uchar(40 + 180 * f));
                isoDraw(img, x, y, z, w.nx(), w.ny(), w.nz(), w.cell(), outPx, col);
            }
    return img;
}

cv::Mat compareImage(const VoxelWorld& truth, const VoxelMap& map,
                     const VoxelMap::Score& s, int outPx) {
    cv::Mat a = isoOfWorld(truth, outPx, 40.f);
    cv::Mat b = map.isoImage(outPx, 40.f);
    cv::putText(a, "TRUTH", {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {30, 30, 30}, 2);
    cv::putText(b, "ESTIMATED (stereo)", {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {30, 30, 30}, 2);
    char buf[160];
    std::snprintf(buf, sizeof buf, "IoU %.2f   false-free %.3f%%   unknown %.0f%%",
                  s.iou(), 100.0 * s.falseFreeRate(),
                  100.0 * double(s.unknown) / std::max(1L, s.total));
    cv::putText(b, buf, {12, outPx - 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {20, 20, 140}, 1);
    cv::Mat out; cv::hconcat(a, b, out);
    return out;
}

}  // namespace sim
