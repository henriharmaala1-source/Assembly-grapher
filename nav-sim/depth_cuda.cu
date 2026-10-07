// CUDA depth render. See depth_cuda.hpp for why ONLY this piece leaves the CPU.
//
// !!! NEVER COMPILED OR RUN BY ITS AUTHOR !!!
// Written on a machine with no CUDA toolkit and no GPU. cuda_depth_check is a
// hard gate: it renders the same scene on both paths and fails if they
// disagree. Do not trust this until that test has passed on your hardware.
//
// THE WORLD IS COPIED TO THE DEVICE ONCE PER RENDER, and that is deliberate
// rather than lazy. A voxel world is a bit array plus a texture byte array --
// at 0.25 m over a 6 m maze that is under a megabyte, and at forest scale a few
// megabytes. Uploading per render costs a fraction of a millisecond against a
// ~47 ms CPU render, and keeping a persistent device copy would mean tracking
// invalidation of a structure the sim mutates between episodes. Correct first;
// cache only if a profile says the upload matters.
#include <cuda_runtime.h>

#include <cstdio>
#include <cmath>

#include "depth_cuda.hpp"
#include "voxel_world.hpp"
#include "depth_camera.hpp"

namespace sim {

namespace {

struct DevWorld {
    const uint8_t* bits;
    const uint8_t* tex;
    float cell, ox, oy, oz;
    int nx, ny, nz;
};

__device__ inline bool devSolid(const DevWorld& w, int x, int y, int z) {
    if (x < 0 || y < 0 || z < 0 || x >= w.nx || y >= w.ny || z >= w.nz) return false;
    const size_t i = (size_t(z) * w.ny + y) * w.nx + x;
    return (w.bits[i >> 3] >> (i & 7)) & 1u;
}

__device__ inline float devTex(const DevWorld& w, int x, int y, int z) {
    if (x < 0 || y < 0 || z < 0 || x >= w.nx || y >= w.ny || z >= w.nz) return 0.f;
    return w.tex[(size_t(z) * w.ny + y) * w.nx + x] / 255.f;
}

// Amanatides-Woo DDA, the same march VoxelWorld::raycast does on the CPU. `t`
// is the ray PARAMETER, and the ray is deliberately NOT normalised -- the CPU
// side builds it with z = 1 in camera frame so t comes out as planar depth,
// which is what a depth camera actually reports. Normalising here would return
// slant range and every value would be quietly wrong off-axis.
__global__ void castKernel(DevWorld w, float px, float py, float pz,
                           float m00, float m01, float m02,
                           float m10, float m11, float m12,
                           float m20, float m21, float m22,
                           int width, int height,
                           float fx, float fy, float ppx, float ppy,
                           float maxRange, float* outRange, float* outTex) {
    const int u = blockIdx.x * blockDim.x + threadIdx.x;
    const int v = blockIdx.y * blockDim.y + threadIdx.y;
    if (u >= width || v >= height) return;
    const int idx = v * width + u;

    const float cxr = (u - ppx) / fx, cyr = (v - ppy) / fy, czr = 1.f;
    const float dx = m00 * cxr + m01 * cyr + m02 * czr;
    const float dy = m10 * cxr + m11 * cyr + m12 * czr;
    const float dz = m20 * cxr + m21 * cyr + m22 * czr;

    int ix = int(floorf((px - w.ox) / w.cell));
    int iy = int(floorf((py - w.oy) / w.cell));
    int iz = int(floorf((pz - w.oz) / w.cell));

    const int sx = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
    const int sy = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
    const int sz = dz > 0 ? 1 : (dz < 0 ? -1 : 0);

    const float BIG = 1e30f;
    float tdx = sx ? fabsf(w.cell / dx) : BIG;
    float tdy = sy ? fabsf(w.cell / dy) : BIG;
    float tdz = sz ? fabsf(w.cell / dz) : BIG;

    auto firstBound = [&](float p, float o, int i, int s, float d) -> float {
        if (s == 0) return BIG;
        const float b = o + (i + (s > 0 ? 1 : 0)) * w.cell;
        return (b - p) / d;
    };
    float tmx = firstBound(px, w.ox, ix, sx, dx);
    float tmy = firstBound(py, w.oy, iy, sy, dy);
    float tmz = firstBound(pz, w.oz, iz, sz, dz);

    float t = 0.f;
    float range = -1.f, tex = 0.f;
    while (t <= maxRange) {
        if (devSolid(w, ix, iy, iz)) { range = t; tex = devTex(w, ix, iy, iz); break; }
        if (tmx <= tmy && tmx <= tmz)      { t = tmx; ix += sx; tmx += tdx; }
        else if (tmy <= tmz)               { t = tmy; iy += sy; tmy += tdy; }
        else                               { t = tmz; iz += sz; tmz += tdz; }
        if (ix < -1 || iy < -1 || iz < -1 ||
            ix > w.nx || iy > w.ny || iz > w.nz) break;
    }
    outRange[idx] = (range >= 0.f && range <= maxRange) ? range : -1.f;
    outTex[idx]   = tex;
}

bool deviceOk() {
    int n = 0;
    return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
}

}  // namespace

bool cudaDepthAvailable() { return deviceOk(); }

const char* cudaDepthStatus() {
    static char buf[128];
    int n = 0;
    if (cudaGetDeviceCount(&n) != cudaSuccess || n == 0)
        return "CUDA built in, but no device found";
    cudaDeviceProp p{};
    if (cudaGetDeviceProperties(&p, 0) == cudaSuccess)
        std::snprintf(buf, sizeof buf, "CUDA depth render on %s", p.name);
    else
        std::snprintf(buf, sizeof buf, "CUDA depth render on device 0");
    return buf;
}

bool renderRangesCuda(const VoxelWorld& w, const CamPose& pose,
                      int width, int height, float /*hfovDeg*/,
                      float fxPx, float fyPx, float ppxPx, float ppyPx,
                      float maxRangeM,
                      std::vector<float>& out, std::vector<float>& tex) {
    if (!deviceOk()) return false;
    const size_t n = size_t(width) * height;
    out.assign(n, -1.f); tex.assign(n, 0.f);

    // Rotation built ONCE on the host from the same convention as
    // DepthCamera::camToWorld, so the kernel never re-derives a frame -- two
    // conventions for one thing is a bug waiting to happen, and this file is
    // exactly where such a bug would hide.
    float m[9];
    {
        float e0[3] = {1, 0, 0}, e1[3] = {0, 1, 0}, e2[3] = {0, 0, 1};
        float o[3];
        DepthCamera::camToWorld(pose, e0[0], e0[1], e0[2], o[0], o[1], o[2]);
        m[0] = o[0]; m[3] = o[1]; m[6] = o[2];
        DepthCamera::camToWorld(pose, e1[0], e1[1], e1[2], o[0], o[1], o[2]);
        m[1] = o[0]; m[4] = o[1]; m[7] = o[2];
        DepthCamera::camToWorld(pose, e2[0], e2[1], e2[2], o[0], o[1], o[2]);
        m[2] = o[0]; m[5] = o[1]; m[8] = o[2];
    }

    const std::vector<uint8_t>& hostBits = w.bitsRef();
    const std::vector<uint8_t>& hostTex  = w.texRef();
    uint8_t *dBits = nullptr, *dTex = nullptr;
    float *dRange = nullptr, *dTexOut = nullptr;
    bool ok = cudaMalloc(&dBits, hostBits.size()) == cudaSuccess
           && cudaMalloc(&dTex,  hostTex.size())  == cudaSuccess
           && cudaMalloc(&dRange, n * sizeof(float)) == cudaSuccess
           && cudaMalloc(&dTexOut, n * sizeof(float)) == cudaSuccess;
    if (ok) {
        ok = cudaMemcpy(dBits, hostBits.data(), hostBits.size(),
                        cudaMemcpyHostToDevice) == cudaSuccess
          && cudaMemcpy(dTex, hostTex.data(), hostTex.size(),
                        cudaMemcpyHostToDevice) == cudaSuccess;
    }
    if (ok) {
        DevWorld dw{dBits, dTex, w.cell(), w.ox(), w.oy(), w.oz(),
                    w.nx(), w.ny(), w.nz()};
        const dim3 blk(16, 16);
        const dim3 grd((width + blk.x - 1) / blk.x, (height + blk.y - 1) / blk.y);
        castKernel<<<grd, blk>>>(dw, pose.e, pose.n, pose.u,
                                 m[0], m[1], m[2], m[3], m[4], m[5],
                                 m[6], m[7], m[8],
                                 width, height, fxPx, fyPx, ppxPx, ppyPx,
                                 maxRangeM, dRange, dTexOut);
        ok = cudaDeviceSynchronize() == cudaSuccess
          && cudaMemcpy(out.data(), dRange, n * sizeof(float),
                        cudaMemcpyDeviceToHost) == cudaSuccess
          && cudaMemcpy(tex.data(), dTexOut, n * sizeof(float),
                        cudaMemcpyDeviceToHost) == cudaSuccess;
    }
    cudaFree(dBits); cudaFree(dTex); cudaFree(dRange); cudaFree(dTexOut);
    return ok;
}

}  // namespace sim
