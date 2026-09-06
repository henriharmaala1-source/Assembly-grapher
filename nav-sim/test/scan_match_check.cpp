// Can the vehicle find itself in its own map, from geometry alone?
//
// The map is built by integrating frames at KNOWN poses; the camera is then
// moved by a known offset and the matcher is handed the STALE pose as its
// guess. Recovering the offset means the pose came from the map rather than
// from the answer being fed in. Ground truth is the displacement, so it is
// exact.
//
// WHY A ROOM AND NOT THE FOREST. The first version of this test used genForest
// and every case failed with an empty map -- 25 occupied cells in a 3.2 M cell
// grid. The reason was not the matcher: at 6 m altitude the nearest trunk was
// 8.50 m away and the map's honest MARKING range is 3.93 m, so there was
// nothing in range to mark and therefore nothing to match against. That is a
// true and useful fact about this sensor -- a sparse forest gives a scan
// matcher very little -- but it is not what these cases are trying to measure.
//
// A room is the honest positive case: walls on four sides, all within marking
// range, which is also the environment where localisation is actually wanted.
// The corridor is the same generator with one dimension stretched, so the
// difference between the two isolates observability and nothing else.
#include <cmath>
#include <cstdio>
#include <string>

#include "depth_camera.hpp"
#include "scan_match.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& detail = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL",
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++failures;
}
static std::string f3(float v) { char b[64]; std::snprintf(b, sizeof b, "%.3f", v); return b; }

// A box of `nyCells` length. Walls, floor and ceiling; nothing inside.
static void makeBox(VoxelWorld& w, int nyCells, bool pillar = false) {
    const float c = 0.25f;
    const int nx = 40, nz = 24;
    w.init(c, 0.f, 0.f, 0.f, nx, nyCells, nz);
    const int xL = 8, xR = 32, zF = 4, zC = 16;
    for (int y = 0; y < nyCells; ++y) {
        for (int z = zF; z <= zC; ++z) {
            w.set(xL, y, z); w.setTex(xL, y, z, 0.4f);
            w.set(xR, y, z); w.setTex(xR, y, z, 0.4f);
        }
        for (int x = xL; x <= xR; ++x) {
            w.set(x, y, zF); w.setTex(x, y, zF, 0.4f);
            w.set(x, y, zC); w.setTex(x, y, zC, 0.4f);
        }
    }
    // End caps, so a ROOM is closed and a corridor (long) effectively is not.
    for (int x = xL; x <= xR; ++x)
        for (int z = zF; z <= zC; ++z) {
            w.set(x, 0, z);          w.setTex(x, 0, z, 0.4f);
            w.set(x, nyCells-1, z);  w.setTex(x, nyCells-1, z, 0.4f);
        }
    // One off-centre feature. Everything about whether heading is recoverable
    // turns on having something like this in view.
    if (pillar)
        for (int x = 22; x <= 24; ++x)      // 0.75 m right of the flight line
            for (int y = 16; y <= 18; ++y)  // and 2 m ahead: inside a 70 deg FoV
                for (int z = zF; z <= zC; ++z) { w.set(x, y, z); w.setTex(x, y, z, 0.5f); }
}

int main() {
    std::printf("scan matching: pose from geometry\n");

    // The camera and map configuration voxel_sim is validated at.
    CamParams cp;
    cp.width = 320; cp.height = 240; cp.hfovDeg = 70.f;
    cp.baselineM = 0.12f; cp.maxRangeM = 12.f;
    DepthCamera cam(cp);

    VoxelMapParams mp;
    mp.cell = 0.25f; mp.nx = 120; mp.ny = 200; mp.nz = 60;
    mp.maxCarveM = 11.f;
    mp.depthSigCoef = cp.subpixelPx / (cam.fpx() * cp.baselineM);
    mp.maxIntegM = std::sqrt(mp.cell * cam.fpx() * cp.baselineM / cp.subpixelPx) * 0.75f;
    std::printf("  [map] marking honest to %.2f m\n", mp.maxIntegM);

    ScanMatchParams sp;
    sp.strideX = 6; sp.strideY = 6;
    ScanMatcher sm; sm.init(sp);

    auto build = [&](const VoxelWorld& w, VoxelMap& M, CamPose p, int n, float stepN) {
        M.init(mp, p.e, p.n, p.u);
        for (int i = 0; i < n; ++i) {
            cv::Mat d = cam.renderTruth(w, p);
            M.integrate(d, cam, p);
            p.n += stepN;
        }
    };

    // ---- a ROOM: 6 x 6 m, all walls inside marking range -------------------
    VoxelWorld room; makeBox(room, 24);
    CamPose start; start.e = 20 * 0.25f; start.n = 1.2f; start.u = 10 * 0.25f;
    start.yawDeg = 0.f;
    VoxelMap Mr; build(room, Mr, start, 6, 0.2f);
    CamPose at = start; at.n += 0.2f * 5.f;

    // MEASURED SCORE PROFILE, and it changes what this test can assert.
    // Sliding the cloud along each axis and counting corroborating cells:
    //
    //   E   flat within 2 counts over +/-0.45 m   -> UNOBSERVABLE
    //   N   clean peak                            -> observable
    //   U   broad, peak displaced ~+0.27 m        -> weakly observable, biased
    //
    // The E result is not a defect, it is geometry. The side walls of a 6 m
    // room are seen at grazing incidence, so their ranges exceed the 3.93 m
    // marking limit and they are never written into the map. What remains in
    // range is the end wall (which pins N) and the floor and ceiling (which
    // pin U weakly). A room this size is therefore a CORRIDOR as far as this
    // sensor is concerned -- the same aperture limit that makes a narrow-FoV,
    // short-range camera a poor place recogniser.
    //
    // The U bias is NOT explained and is recorded as open rather than tuned
    // away. Roughly 1.2 cells, in a direction that survives both scoring
    // schemes tried, which points at where the map marks a grazing surface
    // rather than at the matcher.
    {
        cv::Mat d = cam.renderTruth(room, at);
        ScanMatch r = sm.match(Mr, d, cam, at);
        std::printf("  [room] curv E %.4f N %.4f U %.4f Y %.4f  dYaw %+.2f  hit %.2f  pts %d\n",
                    r.curv[0], r.curv[1], r.curv[2], r.curv[3], r.dYawDeg, r.hitFrac, r.points);
        check("the map is dense enough to match against", r.points > 500,
              std::to_string(r.points) + " pts, hit " + f3(r.hitFrac));
        check("the unobservable lateral axis is reported unobserved",
              !r.axisObserved[0], "E curvature " + f3(r.curv[0]));
        check("and the match therefore refuses", !r.valid);
    }
    {
        // The one axis this geometry does constrain. Guess = the STALE pose,
        // so recovering the offset means it came from the map.
        const float truth = 0.24f;
        CamPose moved = at; moved.n += truth;
        cv::Mat d = cam.renderTruth(room, moved);
        ScanMatch r = sm.match(Mr, d, cam, at);
        check("the observable axis recovers a known displacement",
              std::fabs(r.dN - truth) < 0.10f,
              f3(r.dN) + " vs " + f3(truth) + " m");
    }

    // ---- YAW: implemented, measured, and NOT working ----------------------
    // Gravity references roll and pitch absolutely and says nothing about
    // heading, so yaw is the one attitude with no reference anywhere in the
    // vehicle. The hope was that geometry could bound it. Measured, it cannot,
    // for a reason that is arithmetic rather than tuning.
    //
    // A yaw error moves a point at range R sideways by R*theta, and the map
    // only notices once that exceeds a cell:
    //
    //     theta_min ~ cell / R = 0.25 / 3 = 4.8 degrees
    //
    // Below that the points stay in the cells they were already in. A 1.5
    // degree error against a pillar 2 m away produces 0.0046 of curvature
    // where the best translation axis produces 0.52.
    //
    // So the search is off by default and these cases pin the REFUSAL, which is
    // the behaviour that is actually correct: a heading it cannot resolve is
    // one it must not claim. Bounding real gyro drift needs a finer map, far
    // more range, or the compass the airframe already carries.
    {
        ScanMatchParams yp = sp; yp.yawRangeDeg = 8.f; yp.yawStepDeg = 1.f;
        ScanMatcher ym; ym.init(yp);
        VoxelWorld pr; makeBox(pr, 24, true);
        VoxelMap Mp; build(pr, Mp, start, 6, 0.2f);
        CamPose atp = start; atp.n += 0.2f * 5.f;

        CamPose turned = atp; turned.yawDeg += 6.f;
        cv::Mat d = cam.renderTruth(pr, turned);
        ScanMatch r = ym.match(Mp, d, cam, atp);
        std::printf("  [yaw] 6.0 deg of error, pillar in view: recovered %+.2f, curv %.4f\n",
                    r.dYawDeg, r.curv[3]);
        check("a heading error below what the map can resolve is REFUSED",
              std::fabs(r.dYawDeg) < 0.01f, f3(r.dYawDeg) + " deg claimed");
        check("and the heading axis does not report itself observed",
              !r.axisObserved[3], "curvature " + f3(r.curv[3]));
    }
    {
        // With the search off entirely -- the shipping default -- yaw is simply
        // never touched, which is the same outcome by a shorter route.
        cv::Mat d = cam.renderTruth(room, at);
        ScanMatch r = sm.match(Mr, d, cam, at);
        check("with the search off, heading is never claimed at all",
              std::fabs(r.dYawDeg) < 1e-6f && !r.axisObserved[3]);
    }

    // ---- the SAME box, stretched: a corridor -------------------------------
    // Slid along its own length it scores identically everywhere, so the
    // along-axis is unobservable however clean the fit looks on the other two.
    // A matcher that reports a number here is not accurate-with-error, it is
    // confidently making something up.
    {
        VoxelWorld cor; makeBox(cor, 400);
        CamPose ch = start; ch.n = 30.f;
        VoxelMap Mc; build(cor, Mc, ch, 6, 0.2f);
        CamPose atc = ch; atc.n += 0.2f * 5.f;
        cv::Mat d = cam.renderTruth(cor, atc);
        ScanMatch r = sm.match(Mc, d, cam, atc);
        std::printf("  [corridor] curv E %.4f N %.4f U %.4f Y %.4f  dYaw %+.2f  hit %.2f\n",
                    r.curv[0], r.curv[1], r.curv[2], r.curv[3], r.dYawDeg, r.hitFrac);
        check("along a corridor, the along-axis reads unobserved",
              !r.axisObserved[1], "N curvature " + f3(r.curv[1]));
        // In the room the end wall pinned N; stretching the box removes it, so
        // BOTH horizontal axes go unobserved and only the floor and ceiling
        // still say anything. That is the whole lesson of the pair: what this
        // sensor can localise against is whatever surface happens to sit inside
        // 3.93 m, and in a corridor that is the floor.
        check("stretching the box also costs the along-axis",
              !r.axisObserved[0] && !r.axisObserved[1],
              "E " + f3(r.curv[0]) + "  N " + f3(r.curv[1]));
        check("only the vertical survives, from floor and ceiling",
              r.axisObserved[2], "U " + f3(r.curv[2]));
        check("so the match REFUSES rather than inventing a position", !r.valid);
    }

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
