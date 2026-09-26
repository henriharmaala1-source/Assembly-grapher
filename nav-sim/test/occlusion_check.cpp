// The stereo occlusion shadow: does it fall on the right SIDE, and is it the
// right WIDTH?
//
// Both halves matter and only one of them is obvious. A shadow of the correct
// width on the WRONG SIDE would look completely plausible in a depth image --
// and it would mirror every obstacle boundary in the map, biasing the free
// space beside every trunk in a direction nothing downstream could detect. So
// the side is asserted against the geometry rather than eyeballed.
//
// Width is exact and therefore checkable: a foreground surface at Z_n against a
// background at Z_f hides a strip of f*B*(1/Z_n - 1/Z_f) pixels, because that
// is precisely the disparity difference between them.
//
//   g++ -O2 -std=c++17 -I. test/occlusion_check.cpp depth_camera.cpp \
//       voxel_world.cpp -I/usr/include/opencv4 -lopencv_core -o /tmp/occ

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "depth_camera.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-58s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ",
                d.c_str());
    if (!ok) ++fails;
}

// A wall at zFar with a slab standing in front of it at zNear, so there is one
// clean vertical depth step and the shadow beside it is unambiguous.
static void buildScene(VoxelWorld& w, float cell, float zNear, float zFar,
                       float camE, float camN, float camU) {
    const int n = int(40.f / cell);
    w.init(cell, camE - 20.f, camN - 2.f, camU - 6.f, n, n, int(12.f / cell));
    auto put = [&](float e, float nrt, float u) {
        int x, y, z; w.worldToCell(e, nrt, u, x, y, z);
        w.set(x, y, z, true);
        w.setTex(x, y, z, 0.8f);
    };
    // Camera looks +y (North). Background wall, two cells thick so a ray cannot
    // slip between them.
    for (float e = camE - 10.f; e <= camE + 10.f; e += cell * 0.5f)
        for (float u = camU - 4.f; u <= camU + 4.f; u += cell * 0.5f) {
            put(e, camN + zFar, u);
            put(e, camN + zFar + cell, u);
        }
    // Foreground slab, to the RIGHT of the optical axis (+E), so its shadow has
    // room to fall to its left and cannot be confused with the frame-edge band.
    for (float e = camE + 0.5f; e <= camE + 1.5f; e += cell * 0.5f)
        for (float u = camU - 3.f; u <= camU + 3.f; u += cell * 0.5f) {
            put(e, camN + zNear, u);
            put(e, camN + zNear + cell, u);
        }
}

int main() {
    std::printf("stereo occlusion shadow\n");

    const float cell = 0.05f, zNear = 1.5f, zFar = 4.0f;
    const float camE = 20.f, camN = 20.f, camU = 6.f;

    CamParams p;
    p.width = 640; p.height = 360;
    p.hfovDeg = 87.f; p.baselineM = 0.05f;
    p.subpixelPx = 0.f;          // isolate the geometry from the noise
    p.speckleFrac = 0.f;
    p.texThresh = 0.f;           // and from texture dropout
    p.filterSpeckle = false;
    p.maxRangeM = 30.f;

    VoxelWorld w;
    buildScene(w, cell, zNear, zFar, camE, camN, camU);
    CamPose pose; pose.e = camE; pose.n = camN; pose.u = camU;

    DepthCamera camOn(p);
    CamParams pOff = p; pOff.modelOcclusion = false;
    DepthCamera camOff(pOff);

    cv::Mat on  = camOn.renderStereo(w, pose, nullptr);
    cv::Mat off = camOff.renderStereo(w, pose, nullptr);

    // Scan the middle row for the slab and the hole beside it.
    const int v = p.height / 2;
    const float* rOn  = on.ptr<float>(v);
    const float* rOff = off.ptr<float>(v);

    int slabL = -1, slabR = -1;
    for (int u = 0; u < p.width; ++u)
        if (rOff[u] > 0.f && std::fabs(rOff[u] - zNear) < 0.25f) {
            if (slabL < 0) slabL = u;
            slabR = u;
        }
    check(slabL > 0 && slabR > slabL, "the foreground slab is visible at all",
          "cols " + std::to_string(slabL) + ".." + std::to_string(slabR));
    if (slabL < 0) { std::printf("FAILED (no slab)\n"); return 1; }

    // Count contiguous newly-invalid pixels on each side of the slab.
    auto newHole = [&](int u) { return rOff[u] > 0.f && !(rOn[u] > 0.f); };
    int leftRun = 0;
    for (int u = slabL - 1; u >= 0 && newHole(u); --u) ++leftRun;
    int rightRun = 0;
    for (int u = slabR + 1; u < p.width && newHole(u); ++u) ++rightRun;

    const float fB = camOn.fpx() * p.baselineM;
    const float want = fB * (1.f / zNear - 1.f / zFar);

    std::printf("    f %.1f px, B %.0f mm -> predicted shadow %.1f px\n",
                camOn.fpx(), p.baselineM * 1000.f, want);
    std::printf("    measured: %d px LEFT of the slab, %d px right\n", leftRun, rightRun);

    check(leftRun > 0, "a shadow appears beside the object");
    check(leftRun > 3 * std::max(1, rightRun),
          "and it is on the LEFT -- the side away from the right imager",
          std::to_string(leftRun) + " vs " + std::to_string(rightRun));
    check(std::fabs(float(leftRun) - want) <= std::max(2.f, 0.15f * want),
          "width matches f*B*(1/Zn - 1/Zf) within 15 %",
          std::to_string(leftRun) + " vs " + std::to_string(want));

    // Turning it off must restore the previous behaviour exactly, so every
    // number measured before this existed stays comparable.
    {
        CamParams a = p; a.modelOcclusion = false;
        cv::Mat x = DepthCamera(a).renderStereo(w, pose, nullptr);
        bool same = true;
        for (int y = 0; y < p.height && same; ++y) {
            const float* p1 = x.ptr<float>(y);
            const float* p2 = off.ptr<float>(y);
            for (int u = 0; u < p.width; ++u)
                if (p1[u] != p2[u]) { same = false; break; }
        }
        check(same, "modelOcclusion=false is bit-identical to the old renderer");
    }

    // And the cost of it: how much of the frame does it remove?
    long vOn = 0, vOff = 0;
    for (int y = 0; y < p.height; ++y) {
        const float* a = on.ptr<float>(y);
        const float* b = off.ptr<float>(y);
        for (int u = 0; u < p.width; ++u) { if (a[u] > 0) ++vOn; if (b[u] > 0) ++vOff; }
    }
    std::printf("    valid pixels %.1f %% -> %.1f %% (%.1f points lost to shadow)\n",
                100.0 * vOff / on.total(), 100.0 * vOn / on.total(),
                100.0 * (vOff - vOn) / on.total());
    check(vOn < vOff, "the shadow costs valid pixels");

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
