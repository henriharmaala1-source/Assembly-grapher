#include "voxel_map.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

namespace sim {

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

cv::Mat VoxelMap::isoImage(int outPx, float maxZ, float yawDeg) const {
    cv::Mat img(outPx, outPx, CV_8UC3, cv::Scalar(250, 248, 245));
    int step = std::max(1, p_.nx / 200);
    for (int z = 0; z < p_.nz; z += step)
        for (int y = p_.ny - 1; y >= 0; y -= step)
            for (int x = 0; x < p_.nx; x += step) {
                if (log_[idx(x, y, z)] <= p_.occThresh) continue;
                float wx, wy, wz; cellCentre(x, y, z, wx, wy, wz);
                if (wz > maxZ) continue;
                float f = std::min(1.f, std::max(0.f, (wz - oz_) / std::max(1.f, maxZ - oz_)));
                cv::Vec3b col(uchar(220 - 150 * f), uchar(90 + 60 * f), uchar(40 + 180 * f));
                isoDraw(img, x, y, z, p_.nx, p_.ny, p_.nz, p_.cell, outPx, col, yawDeg);
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
