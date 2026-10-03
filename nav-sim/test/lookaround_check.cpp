// LOOK AROUND: does the map stay coherent when the camera only rotates?
//
// This is the one motion the aircraft can map correctly with attitude alone.
// Rotation needs no translation estimate, so it is what a bench test should do
// first and what voxel_live has always supported -- but only ever with a yaw
// rate TOLD to it. Here the attitude comes from a simulated IMU through the
// same AttitudeFilter the live path uses, so a sign error or a gyro bias shows
// up as a smeared map rather than as a number nobody checks.
//
// The comparison is against a map built at TRUTH attitude from the SAME frames.
// Two maps, identical geometry, different beliefs about where the camera
// pointed: the difference is entirely attitude error, which is the point.
#include <cmath>
#include <cstdio>
#include <string>

#include "attitude_filter.hpp"
#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}
static std::string f2(float v) { char b[48]; std::snprintf(b, sizeof b, "%.2f", v); return b; }

static void makeRoom(VoxelWorld& w, bool pillar) {
    const float c = 0.25f; const int n = 48, nz = 24;
    w.init(c, 0.f, 0.f, 0.f, n, n, nz);
    const int lo = 8, hi = 40, zF = 4, zC = 16;
    for (int i = lo; i <= hi; ++i)
        for (int z = zF; z <= zC; ++z) {
            w.set(lo, i, z); w.setTex(lo, i, z, .4f);
            w.set(hi, i, z); w.setTex(hi, i, z, .4f);
            w.set(i, lo, z); w.setTex(i, lo, z, .4f);
            w.set(i, hi, z); w.setTex(i, hi, z, .4f);
        }
    for (int x = lo; x <= hi; ++x)
        for (int y = lo; y <= hi; ++y) {
            w.set(x, y, zF); w.setTex(x, y, zF, .4f);
            w.set(x, y, zC); w.setTex(x, y, zC, .4f);
        }
    // ONE ASYMMETRIC FEATURE, and it is the whole difference between a scene
    // that can reveal a yaw error and one that cannot.
    if (pillar)
        for (int x = 14; x <= 17; ++x)
            for (int y = 14; y <= 17; ++y)
                for (int z = zF; z <= zC; ++z) { w.set(x, y, z); w.setTex(x, y, z, .5f); }
}

// Of the cells EITHER map calls occupied, how many do both? A smeared map marks
// cells the clean one does not, so this falls as attitude error grows.
static float iou(const VoxelMap& a, const VoxelMap& b, const VoxelMapParams& p) {
    long both = 0, either = 0;
    for (int x = 0; x < p.nx; ++x)
        for (int y = 0; y < p.ny; ++y)
            for (int z = 0; z < p.nz; ++z) {
                const bool A = a.logAt(x, y, z) > 0.f, B = b.logAt(x, y, z) > 0.f;
                if (A || B) ++either;
                if (A && B) ++both;
            }
    return either ? float(both) / float(either) : 0.f;
}

// SMEARING, which is what attitude error actually does to a map: the same wall
// gets marked at several angles, so the wall THICKENS. Counting occupied cells
// detects that directly.
//
// IoU against a symmetric room does NOT. Measured here: 28.8 degrees of yaw
// error still scores 0.90, because a square room rotated by 28 degrees is still
// mostly a square room. That is worth knowing beyond this test -- in a
// symmetric space, a map that looks right is not evidence that attitude is
// right, and a bench check by eye will pass a badly drifting gyro.
static long occupiedCells(const VoxelMap& m, const VoxelMapParams& p) {
    long n = 0;
    for (int x = 0; x < p.nx; ++x)
        for (int y = 0; y < p.ny; ++y)
            for (int z = 0; z < p.nz; ++z)
                if (m.logAt(x, y, z) > 0.f) ++n;
    return n;
}

static float sweepTurn(float sweepDeg, float rateDps, float biasDps,
                       float& yawErrDeg, float& bloat, bool pillar) {
    CamParams cp; cp.width = 240; cp.height = 180; cp.hfovDeg = 70.f;
    cp.baselineM = 0.12f; cp.maxRangeM = 12.f;
    DepthCamera cam(cp);
    VoxelMapParams mp; mp.cell = 0.25f; mp.nx = 60; mp.ny = 60; mp.nz = 40;
    mp.maxCarveM = 11.f;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(mp.cell * cam.fpx() * cp.baselineM / cp.subpixelPx) * 0.75f;

    VoxelWorld room; makeRoom(room, pillar);
    const float e = 24 * 0.25f, n = 24 * 0.25f, u = 10 * 0.25f;

    VoxelMap Mt, Mf;
    Mt.init(mp, e, n, u); Mf.init(mp, e, n, u);

    AttitudeFilter f; f.init(AttitudeParams{});
    const float g = 9.81f;
    f.seed(0.f, g, 0.f);

    const float dt = 1.f / 200.f;
    const int   imuPerFrame = 200 / 30;
    float yawTruth = 0.f;
    const int frames = int(sweepDeg / (rateDps / 30.f));
    for (int i = 0; i < frames; ++i) {
        for (int k = 0; k < imuPerFrame; ++k) {
            // Yaw is rotation about the DOWN axis, +y in camera convention.
            const float wy = (rateDps + biasDps) * 3.14159265f / 180.f;
            f.update(0.f, wy, 0.f, 0.f, g, 0.f, dt);
            yawTruth += rateDps * dt;
        }
        CamPose pt; pt.e = e; pt.n = n; pt.u = u; pt.yawDeg = yawTruth;
        CamPose pf = pt; pf.yawDeg = f.yawDeg();
        pf.rollDeg = f.rollDeg(); pf.pitchDeg = f.pitchDeg();
        cv::Mat d = cam.renderTruth(room, pt);
        Mt.integrate(d, cam, pt);
        Mf.integrate(d, cam, pf);
    }
    yawErrDeg = std::fabs(f.yawDeg() - std::fmod(yawTruth, 360.f));
    const long ot = occupiedCells(Mt, mp), of = occupiedCells(Mf, mp);
    bloat = ot ? float(of) / float(ot) : 0.f;
    return iou(Mt, Mf, mp);
}

int main() {
    std::printf("look around: mapping on attitude alone\n");
    float b0 = 0, b1 = 0, b2 = 0;
    {
        float err = 0.f;
        const float v = sweepTurn(360.f, 45.f, 0.f, err, b0, false);
        check("a full turn on a clean gyro maps as if attitude were truth",
              v > 0.95f && b0 < 1.02f,
              "IoU " + f2(v) + ", yaw err " + f2(err) + " deg, bloat " + f2(b0));
    }
    {
        float err = 0.f;
        const float v = sweepTurn(360.f, 45.f, 0.5f, err, b1, false);
        std::printf("      0.5 deg/s bias: IoU %.2f  bloat %.2f  yaw err %.2f deg\n",
                    v, b1, err);
        check("a realistic gyro bias still maps usefully", v > 0.80f, "IoU " + f2(v));
    }
    {
        // Gravity cannot correct yaw, so bias integrates without limit. This is
        // the drift a compass or a loop closure exists to bound, and there is
        // neither here.
        float err = 0.f;
        const float v = sweepTurn(360.f, 45.f, 4.0f, err, b2, false);
        std::printf("      4.0 deg/s bias: IoU %.2f  bloat %.2f  yaw err %.2f deg\n",
                    v, b2, err);
        // NEITHER METRIC SEES IT, and that is the result rather than a gap in
        // the test. Cell count does not grow because the map is not smeared --
        // it is ROTATED, and a rotated square room lands on the same walls.
        check("in a symmetric room, cell count does not see 28 deg of drift",
              b2 > b0 * 0.95f && b2 < b0 * 1.05f,
              "occupied x" + f2(b2) + " against x" + f2(b0) + " clean");
        check("and neither does overlap", v > 0.80f,
              "IoU " + f2(v) + " at " + f2(err) + " deg of yaw error");
    }
    // THE SAME MOTION AND THE SAME ERROR, in a room with one pillar in it.
    //
    // This pair is the useful part of the file. A square room swept through a
    // full turn maps onto itself under any yaw offset, so neither overlap nor
    // cell count can see 28.8 degrees of drift -- the map looks correct and is
    // pointing the wrong way. Add a single asymmetric feature and the same
    // error becomes obvious immediately.
    //
    // The bench consequence: validating attitude by looking at a map of a
    // corridor, a hallway or an empty room proves nothing. Put something
    // distinctive and off-centre in the scene, or measure against a heading you
    // already know.
    {
        float e0 = 0, e1 = 0, p0 = 0, p1 = 0;
        const float clean = sweepTurn(360.f, 45.f, 0.f, e0, p0, true);
        const float drift = sweepTurn(360.f, 45.f, 4.0f, e1, p1, true);
        std::printf("      with a pillar: clean IoU %.2f -> drifted IoU %.2f\n",
                    clean, drift);
        check("one asymmetric feature makes the same drift plainly visible",
              drift < clean - 0.10f,
              f2(clean) + " -> " + f2(drift) + " at " + f2(e1) + " deg yaw error");
    }
    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
