#include "footage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <opencv2/core/utility.hpp>

namespace sim {

namespace {

// A stable per-cell value in [0,1), so each voxel keeps its own shade from
// frame to frame. Hashing PIXELS instead would make the footage shimmer as
// the aircraft moves -- noise that looks like a sensor artefact.
float cellHash(int x, int y, int z) {
    uint32_t h = uint32_t(x) * 73856093u ^ uint32_t(y) * 19349663u ^
                 uint32_t(z) * 83492791u;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return float(h & 0xffffu) / 65536.f;
}

cv::Vec3f mix(const cv::Vec3f& a, const cv::Vec3f& b, float t) {
    return a * (1.f - t) + b * t;
}

}  // namespace

cv::Mat renderFootage(const VoxelWorld& world, const CamPose& pose,
                      int w, int h, float hfovDeg, float maxRangeM,
                      float groundZ) {
    CamParams cp;
    cp.width = std::max(16, w);
    cp.height = std::max(12, h);
    cp.hfovDeg = hfovDeg;
    cp.maxRangeM = maxRangeM;
    const DepthCamera cam(cp);

    // BGR, in 0..1. Sky pale at the horizon and deeper overhead; the haze
    // colour is the horizon, so distant things fade INTO the sky rather than
    // into grey -- which is what makes depth readable in a still frame.
    const cv::Vec3f skyTop(0.86f, 0.62f, 0.38f), skyHor(0.95f, 0.88f, 0.80f);
    // Low contrast between A and B on purpose: a strong per-voxel shade makes
    // every surface a checkerboard and the scene reads as blocks, not as a
    // place. The variation is there so a flat wall still shows its extent.
    const cv::Vec3f grassA(0.24f, 0.47f, 0.36f), grassB(0.27f, 0.52f, 0.40f);
    const cv::Vec3f warmA(0.36f, 0.46f, 0.57f), warmB(0.40f, 0.50f, 0.61f);
    const cv::Vec3f wallA(0.66f, 0.68f, 0.70f), wallB(0.72f, 0.73f, 0.74f);
    // Sun from the north-west, high. Unit length.
    const float sl = std::sqrt(0.35f * 0.35f + 0.5f * 0.5f + 0.8f * 0.8f);
    const float sx = -0.35f / sl, sy = 0.5f / sl, sz = 0.8f / sl;

    const float c = world.cell();
    cv::Mat out(cp.height, cp.width, CV_8UC3);
    // Rows are independent and every call below is const, so they split
    // across cores. Single-threaded this was 55-66 ms at 480x360 -- as much as
    // the step and the belief render together, halving the demo pane's rate.
    cv::parallel_for_(cv::Range(0, cp.height), [&](const cv::Range& rows) {
    for (int v = rows.start; v < rows.end; ++v) {
        cv::Vec3b* row = out.ptr<cv::Vec3b>(v);
        for (int u = 0; u < cp.width; ++u) {
            float dx, dy, dz;
            cam.rayFor(pose, u, v, dx, dy, dz);
            float tex = 0.f;
            const float t = world.raycast(pose.e, pose.n, pose.u, dx, dy, dz,
                                          maxRangeM, &tex);
            const float up = std::max(0.f, std::min(1.f, dz * 2.5f));
            const cv::Vec3f sky = mix(skyHor, skyTop, up);
            cv::Vec3f col;
            if (t >= maxRangeM) {
                col = sky;
            } else {
                const float hx = pose.e + dx * t, hy = pose.n + dy * t,
                            hz = pose.u + dz * t;
                // The cell the ray ENTERED: a hair past the hit point.
                int cx, cy, cz;
                world.worldToCell(hx + dx * 1e-3f, hy + dy * 1e-3f,
                                  hz + dz * 1e-3f, cx, cy, cz);
                // Which face: the axis on which the hit point sits closest to
                // a cell boundary. The normal faces back along the ray.
                const float fx = (hx - world.ox()) / c - float(cx);
                const float fy = (hy - world.oy()) / c - float(cy);
                const float fz = (hz - world.oz()) / c - float(cz);
                const float ex = std::min(fx, 1.f - fx), ey = std::min(fy, 1.f - fy),
                            ez = std::min(fz, 1.f - fz);
                float nx = 0, ny = 0, nz = 0;
                if (ez <= ex && ez <= ey)      nz = dz > 0 ? -1.f : 1.f;
                else if (ex <= ey)             nx = dx > 0 ? -1.f : 1.f;
                else                           ny = dy > 0 ? -1.f : 1.f;

                // Fine per-voxel shade plus a gentle large-scale one, so broad
                // surfaces vary across metres rather than cell by cell.
                const float k = 0.6f * cellHash(cx, cy, cz) +
                                0.4f * cellHash(cx >> 3, cy >> 3, cz >> 3);
                // GROUND is an upward face well below the aircraft, or at the
                // stated terrain height. Worlds put their floor at different
                // heights, so a fixed Z alone painted most floors as walls.
                const bool ground = nz > 0.5f &&
                                    (hz <= groundZ + c || hz < pose.u - 0.7f);
                cv::Vec3f base;
                if (ground)            base = mix(grassA, grassB, k);
                else if (tex >= 0.45f) base = mix(warmA, warmB, k);   // bark, brick
                else                   base = mix(wallA, wallB, k);   // plaster, glass
                // Height tint so tall structure reads as tall.
                if (!ground) base *= 0.85f + 0.15f * std::min(1.f, hz / 6.f);

                const float lam = std::max(0.f, nx * sx + ny * sy + nz * sz);
                col = base * (0.45f + 0.55f * lam);
                // Haze: nothing at 0, full sky at the far end of the range.
                const float fog = std::min(1.f, std::pow(t / maxRangeM, 1.3f));
                col = mix(col, sky, fog);
            }
            row[u] = cv::Vec3b(cv::saturate_cast<uint8_t>(col[0] * 255.f),
                               cv::saturate_cast<uint8_t>(col[1] * 255.f),
                               cv::saturate_cast<uint8_t>(col[2] * 255.f));
        }
    }
    });
    return out;
}

}  // namespace sim
