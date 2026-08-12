// Does the first-person voxel render line up with the depth image it was built
// from?
//
// The overlay's entire value is that map and depth are INDEPENDENT estimates of
// the same scene, so agreement means the intrinsics, the frame convention and
// the pose are all consistent. That only holds if the render is actually
// aligned — and a render that is mirrored, offset or sheared looks completely
// plausible on a forest, where one trunk is much like another. Eyeballing it
// cannot distinguish "correct" from "flipped", so this measures it.
//
// Method: one distinctive object, deliberately OFF-CENTRE and OFF-AXIS in both
// image axes so a left/right or up/down flip cannot pass. Integrate a truth
// depth frame, render the map through the same intrinsics, and compare the
// centroid of the near-depth pixels with the centroid of the voxel hits.
//
//   g++ -O2 -std=c++17 -I. test/overlay_align_check.cpp voxel_map.cpp \
//       depth_camera.cpp voxel_world.cpp -I/usr/include/opencv4 \
//       -lopencv_core -lopencv_imgproc -o /tmp/oa && /tmp/oa

#include <cmath>
#include <cstdio>
#include <string>

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-58s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ",
                d.c_str());
    if (!ok) ++fails;
}

// Centroid of a mask, in pixels. Returns false if it is empty.
static bool centroid(const cv::Mat& mask, float& cx, float& cy, int& n) {
    double sx = 0, sy = 0; n = 0;
    for (int y = 0; y < mask.rows; ++y)
        for (int x = 0; x < mask.cols; ++x)
            if (mask.at<uchar>(y, x)) { sx += x; sy += y; ++n; }
    if (!n) return false;
    cx = float(sx / n); cy = float(sy / n);
    return true;
}

int main() {
    std::printf("overlay alignment (voxel render vs depth image)\n");

    const float cell = 0.10f;
    const float camE = 20.f, camN = 20.f, camU = 6.f;
    const float yaw = 0.f, pitch = 0.f;

    // A post standing UP AND TO THE RIGHT of the optical axis. Both offsets are
    // deliberate: an object on the axis would survive any flip.
    VoxelWorld w;
    const int n = int(40.f / cell);
    w.init(cell, camE - 20.f, camN - 2.f, camU - 6.f, n, n, int(12.f / cell));
    auto put = [&](float e, float nn, float u) {
        int x, y, z; w.worldToCell(e, nn, u, x, y, z);
        w.set(x, y, z, true); w.setTex(x, y, z, 0.9f);
    };
    for (float e = camE + 0.8f; e <= camE + 1.1f; e += cell * 0.5f)
        for (float u = camU + 0.5f; u <= camU + 1.1f; u += cell * 0.5f)
            for (float d = 2.0f; d <= 2.3f; d += cell * 0.5f)
                put(e, camN + d, u);

    CamParams cp;
    cp.width = 424; cp.height = 240;
    cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
    cp.maxRangeM = 30.f;
    DepthCamera cam(cp);
    CamPose pose; pose.e = camE; pose.n = camN; pose.u = camU;
    pose.yawDeg = yaw; pose.pitchDeg = pitch;

    // TRUTH depth, so the test measures geometry rather than the stereo model.
    cv::Mat d = cam.renderTruth(w, pose);

    VoxelMapParams mp;
    mp.cell = cell;
    mp.maxIntegM = 6.f; mp.maxCarveM = 8.f;
    mp.depthSigCoef = 0.25f / (cam.fpx() * cp.baselineM);
    VoxelMap M;
    M.init(mp, camE, camN, camU);
    for (int i = 0; i < 6; ++i) M.integrate(d, cam, pose);   // cross occThresh

    // Where is the post in the DEPTH image?
    cv::Mat depthMask(d.rows, d.cols, CV_8U, cv::Scalar(0));
    for (int y = 0; y < d.rows; ++y)
        for (int x = 0; x < d.cols; ++x) {
            const float z = d.at<float>(y, x);
            if (z > 1.5f && z < 3.0f) depthMask.at<uchar>(y, x) = 255;
        }
    // NOTE the sequencing. Writing check(centroid(...), ..., to_string(dn))
    // reads dn in the same full expression that writes it, and the arguments
    // are unsequenced -- so it printed "0 px" while the check itself passed.
    // Harmless here, but a test whose OUTPUT lies is worse than no output.
    float dx_, dy_; int dn = 0;
    const bool dok = centroid(depthMask, dx_, dy_, dn);
    check(dok && dn > 50,
          "the post is visible in the depth image", std::to_string(dn) + " px");
    if (!dn) { std::printf("FAILED\n"); return 1; }

    // Where is it in the VOXEL render, through the same intrinsics?
    cv::Mat hit;
    M.fpvImageWH(camE, camN, camU, yaw, pitch, d.cols, d.rows, cp.hfovDeg,
                 8.f, &hit);
    float vx_, vy_; int vn = 0;
    const bool vok = centroid(hit, vx_, vy_, vn);
    check(vok && vn > 20, "and in the voxel render", std::to_string(vn) + " px");
    if (!vn) { std::printf("FAILED\n"); return 1; }

    std::printf("    depth centroid (%.1f, %.1f)   voxel centroid (%.1f, %.1f)"
                "   image centre (%.1f, %.1f)\n",
                dx_, dy_, vx_, vy_, (d.cols - 1) * 0.5f, (d.rows - 1) * 0.5f);

    const float ex = std::fabs(vx_ - dx_), ey = std::fabs(vy_ - dy_);
    check(ex < 12.f && ey < 12.f, "centroids agree within 12 px",
          "dx " + std::to_string(ex) + "  dy " + std::to_string(ey));

    // The offsets must be REAL, or the agreement above is vacuous: an object at
    // the centre of the frame agrees with its own mirror image.
    const float cx0 = (d.cols - 1) * 0.5f, cy0 = (d.rows - 1) * 0.5f;
    check(dx_ - cx0 > 20.f, "the post really is right of centre in depth",
          std::to_string(dx_ - cx0) + " px");
    check(cy0 - dy_ > 15.f, "and really is above centre",
          std::to_string(cy0 - dy_) + " px");

    // --- the projection must be the exact inverse of the render ray --------
    // Paths are drawn into the first-person view with fpvProject; if it is not
    // the inverse of the ray the renderer casts, the plan is drawn somewhere
    // the geometry does not put it, and on a voxel scene that looks entirely
    // convincing.
    {
        float worst = 0.f; int tested = 0, behind = 0;
        for (int vv = 8; vv < d.rows; vv += 23)
            for (int uu = 8; uu < d.cols; uu += 29) {
                float rx, ry, rz;
                VoxelMap::fpvRay(yaw, pitch, d.cols, d.rows, cp.hfovDeg,
                                 float(uu), float(vv), rx, ry, rz);
                // A point 3 m along that ray must project back to that pixel.
                float bu, bv;
                if (!VoxelMap::fpvProject(camE, camN, camU, yaw, pitch,
                                          d.cols, d.rows, cp.hfovDeg,
                                          camE + rx * 3.f, camN + ry * 3.f,
                                          camU + rz * 3.f, bu, bv)) { ++behind; continue; }
                worst = std::max(worst, std::max(std::fabs(bu - uu), std::fabs(bv - vv)));
                ++tested;
            }
        check(tested > 50 && worst < 0.01f,
              "fpvProject inverts fpvRay to under 0.01 px",
              std::to_string(tested) + " pixels, worst " + std::to_string(worst));
        check(behind == 0, "and no in-frame pixel was reported as behind the camera");
    }

    // A point BEHIND the camera must be rejected, not folded to the front --
    // the failure that draws a retreat primitive as though it went forwards.
    {
        float bu, bv;
        check(!VoxelMap::fpvProject(camE, camN, camU, yaw, pitch, d.cols, d.rows,
                                    cp.hfovDeg, camE, camN - 3.f, camU, bu, bv),
              "a point behind the camera is rejected, not mirrored to the front");
    }

    // --- the ladder render: a coarse level must be able to carry the pane ---
    //
    // The fault this pins: every view used to draw ONE map, the finest
    // available, whose honest MARKING range is short by construction. A scene
    // whose nearest surface is past that range rendered empty -- and looked
    // like a broken mapper rather than a correctly-scoped one.
    //
    // Two posts, one at 2 m and one at 6 m, and a fine map that stops at 3 m.
    {
        VoxelWorld w2;
        const float c2 = 0.10f;
        const int n2 = int(40.f / c2);
        w2.init(c2, camE - 20.f, camN - 2.f, camU - 6.f, n2, n2, int(12.f / c2));
        auto put2 = [&](float e, float nn, float u) {
            int x, y, z; w2.worldToCell(e, nn, u, x, y, z);
            w2.set(x, y, z, true); w2.setTex(x, y, z, 0.9f);
        };
        // near post: up and right. far post: down and LEFT, so the two never
        // share an image region and "the far one is missing" is measurable.
        for (float s = 0.f; s <= 0.3f; s += c2 * 0.5f) {
            for (float t = 0.f; t <= 0.6f; t += c2 * 0.5f)
                for (float dd = 0.f; dd <= 0.3f; dd += c2 * 0.5f) {
                    put2(camE + 0.8f + s, camN + 2.0f + dd, camU + 0.5f + t);
                    put2(camE - 2.2f - s, camN + 6.0f + dd, camU - 1.6f - t);
                }
        }
        cv::Mat d2 = cam.renderTruth(w2, pose);

        auto build = [&](float cell_, float integ) {
            VoxelMapParams q;
            q.cell = cell_; q.maxIntegM = integ; q.maxCarveM = integ * 2.f;
            q.depthSigCoef = mp.depthSigCoef;
            auto* vm = new VoxelMap();
            vm->init(q, camE, camN, camU);
            for (int i = 0; i < 6; ++i) vm->integrate(d2, cam, pose);
            return vm;
        };
        VoxelMap* fine = build(0.10f, 3.0f);
        VoxelMap* crse = build(0.50f, 9.0f);

        // Count hits in the lower-LEFT quadrant, where only the far post is.
        auto quad = [&](const cv::Mat& m) {
            int c = 0;
            for (int y = m.rows / 2; y < m.rows; ++y)
                for (int x = 0; x < m.cols / 2; ++x) if (m.at<uchar>(y, x)) ++c;
            return c;
        };
        const int RW = 200, RH = 200;
        cv::Mat mf, mc, ml;
        VoxelMap::renderLadder({{fine, 9.f}}, camE, camN, camU, yaw, pitch,
                               RW, RH, 87.f, FpvStyle(), &mf);
        VoxelMap::renderLadder({{crse, 9.f}}, camE, camN, camU, yaw, pitch,
                               RW, RH, 87.f, FpvStyle(), &mc);
        VoxelMap::renderLadder({{crse, 9.f}, {fine, 9.f}}, camE, camN, camU,
                               yaw, pitch, RW, RH, 87.f, FpvStyle(), &ml);
        const int qf = quad(mf), qc = quad(mc), qm = quad(ml);
        check(qf < 5, "the FINE map alone cannot see a post past its own range",
              std::to_string(qf) + " px");
        check(qc > 20, "the coarse map can", std::to_string(qc) + " px");
        check(qm >= qc, "and the ladder keeps it", std::to_string(qm) + " px");

        // The ladder must never LOSE a surface either level had.
        int tf = 0, tc = 0, tm = 0;
        for (int y = 0; y < RH; ++y)
            for (int x = 0; x < RW; ++x) {
                tf += mf.at<uchar>(y, x) ? 1 : 0;
                tc += mc.at<uchar>(y, x) ? 1 : 0;
                tm += ml.at<uchar>(y, x) ? 1 : 0;
            }
        check(tm >= tf && tm >= tc, "the ladder is the union of what each level sees",
              std::to_string(tf) + " / " + std::to_string(tc) + " -> " +
              std::to_string(tm) + " px");
        delete fine; delete crse;
    }

    // --- the chase view's unknown fog -------------------------------------
    // Standing BEHIND the aircraft means every ray crosses metres of space
    // nobody ever measured before it reaches the scene. At first-person fog
    // strength that space is charged as uncertainty and the pane goes white --
    // which is right when the eye is the aircraft's and wrong when it is not,
    // because that unknown is a property of where the eye was PUT.
    //
    // Measured on the same post, from 2 m back and 1.2 m up.
    {
        const float eN = camN - 2.0f, eU = camU + 1.2f;
        const float pd = -std::atan2(1.2f, 3.0f) * 180.f / 3.14159265f;
        auto contrast = [&](const FpvStyle& st, int& hits) {
            cv::Mat hm;
            cv::Mat img = M.fpvImageWH(camE, eN, eU, yaw, pd, 200, 200, 60.f,
                                       9.f, &hm, st);
            const cv::Vec3f FOG(238, 240, 244);
            double sum = 0; hits = 0;
            for (int y = 0; y < img.rows; ++y)
                for (int x = 0; x < img.cols; ++x) {
                    if (!hm.at<uchar>(y, x)) continue;
                    const cv::Vec3b p = img.at<cv::Vec3b>(y, x);
                    sum += std::fabs(p[0] - FOG[0]) + std::fabs(p[1] - FOG[1]) +
                           std::fabs(p[2] - FOG[2]);
                    ++hits;
                }
            return hits ? float(sum / hits) : 0.f;
        };
        int hFp = 0, hCh = 0;
        FpvStyle chase; chase.unknownFogM = 24.f; chase.unknownFogMax = 0.30f;
        const float cFp = contrast(FpvStyle(), hFp);
        const float cCh = contrast(chase, hCh);
        check(hFp > 20 && hFp == hCh,
              "the fog setting changes colour only, never what was HIT",
              std::to_string(hFp) + " vs " + std::to_string(hCh) + " px");
        check(cCh > cFp * 1.4f,
              "and the chase setting leaves the map visible against the fog",
              "contrast " + std::to_string(cFp) + " -> " + std::to_string(cCh));
    }

    // Explicitly reject the two flips a plausible-looking render would pass.
    check(std::fabs((2 * cx0 - vx_) - dx_) > 20.f,
          "a LEFT/RIGHT flip of the render would NOT match");
    check(std::fabs((2 * cy0 - vy_) - dy_) > 15.f,
          "a UP/DOWN flip of the render would NOT match");

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
