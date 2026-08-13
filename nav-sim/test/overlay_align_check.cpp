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
        VoxelMap::renderLadder({{fine, 0.f, 9.f}}, camE, camN, camU, yaw, pitch,
                               RW, RH, 87.f, FpvStyle(), &mf);
        VoxelMap::renderLadder({{crse, 0.f, 9.f}}, camE, camN, camU, yaw, pitch,
                               RW, RH, 87.f, FpvStyle(), &mc);
        // FINEST FIRST, each banded by its own honest range.
        VoxelMap::renderLadder({{fine, 0.f, 3.f}, {crse, 3.f, 9.f}}, camE, camN,
                               camU, yaw, pitch, RW, RH, 87.f, FpvStyle(), &ml);
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
        // The near post sits at 2 m, inside the fine level's band. A coarse cell
        // containing it would have its near face up to 0.5 m in FRONT of it, so
        // a nearest-hit ladder draws it too close -- the fault that filled a
        // real first-person pane in a room. Banding means the fine level owns
        // that range outright.
        {
            cv::Mat mNearOnly, mBanded;
            VoxelMap::renderLadder({{fine, 0.f, 3.f}}, camE, camN, camU, yaw,
                                   pitch, RW, RH, 87.f, FpvStyle(), &mNearOnly);
            VoxelMap::renderLadder({{fine, 0.f, 3.f}, {crse, 3.f, 9.f}}, camE,
                                   camN, camU, yaw, pitch, RW, RH, 87.f,
                                   FpvStyle(), &mBanded);
            int upperFine = 0, upperBand = 0;      // where only the near post is
            for (int y = 0; y < RH / 2; ++y)
                for (int x = RW / 2; x < RW; ++x) {
                    upperFine += mNearOnly.at<uchar>(y, x) ? 1 : 0;
                    upperBand += mBanded.at<uchar>(y, x) ? 1 : 0;
                }
            // BOUNDED, not zero. A coarse cell that STRADDLES the handover may
            // legitimately speak: it could hold a surface at or beyond the band
            // start, and the fine level -- honest only to that range -- provably
            // cannot know. What must stay excluded is a cell lying ENTIRELY
            // inside the fine band, which is the one that draws a wall far too
            // close and filled a real pane in a room. The exact-equality version
            // of this check forbade both, and in doing so it cut a disc out of
            // the middle of every first-person view.
            check(upperFine > 20 && upperBand <= upperFine * 7 / 5,
                  "a straddling coarse cell may speak, but only just",
                  std::to_string(upperFine) + " -> " + std::to_string(upperBand) + " px");
            // And the cell that lies wholly inside the fine band must not.
            {
                VoxelMap* deep = build(0.50f, 9.0f);
                cv::Mat mDeep;
                // Band start pushed far out: every coarse cell holding the posts
                // now lies entirely inside it.
                VoxelMap::renderLadder({{fine, 0.f, 8.f}, {deep, 8.f, 12.f}},
                                       camE, camN, camU, yaw, pitch, RW, RH, 87.f,
                                       FpvStyle(), &mDeep);
                cv::Mat mFineOnly;
                VoxelMap::renderLadder({{fine, 0.f, 8.f}}, camE, camN, camU, yaw,
                                       pitch, RW, RH, 87.f, FpvStyle(), &mFineOnly);
                int a = 0, b = 0;
                for (int y = 0; y < RH; ++y)
                    for (int x = 0; x < RW; ++x) {
                        a += mFineOnly.at<uchar>(y, x) ? 1 : 0;
                        b += mDeep.at<uchar>(y, x) ? 1 : 0;
                    }
                check(b == a, "and a coarse cell wholly inside the fine band "
                              "still may not", std::to_string(a) + " -> " +
                              std::to_string(b) + " px");
                delete deep;
            }
        }

        // Banding means the ladder is deliberately NOT the union. It must gain
        // the far region the fine level cannot reach, and must NOT inherit the
        // coarse level's near-field claims -- those are 2 m cells whose near
        // faces sit in front of the surfaces they contain.
        check(tm > tf, "the ladder gains what the fine level cannot reach",
              std::to_string(tf) + " -> " + std::to_string(tm) + " px");
        check(tm < tc, "and does NOT inherit the coarse level's near field",
              std::to_string(tm) + " vs " + std::to_string(tc) + " px coarse-alone");
        delete fine; delete crse;
    }

    // --- minimum trusted range: a near return is not a measurement ---------
    // Held a hand near a real D435i and got flickering voxels at the eye with a
    // solid first-person pane. Inside the sensor's minimum range the matcher is
    // past its disparity search, the occlusion band is 18 % of the frame, and
    // the projector saturates -- what comes back is confidently wrong, not
    // missing. Marking it puts OCCUPIED cells where the aircraft is.
    {
        VoxelMapParams q;
        q.cell = 0.10f; q.maxIntegM = 6.f; q.maxCarveM = 8.f; q.carveWinPx = 0;
        q.depthSigCoef = 0.f;
        // A frame that is entirely one flat surface at 0.12 m -- inside the gate.
        cv::Mat near12(d.rows, d.cols, CV_32F, cv::Scalar(0.12f));

        auto countStates = [&](float minM, int& occ, int& freeN) {
            VoxelMapParams qq = q; qq.minIntegM = minM;
            VoxelMap M2; M2.init(qq, camE, camN, camU);
            for (int i = 0; i < 6; ++i) M2.integrate(near12, cam, pose);
            occ = freeN = 0;
            for (float t = 0.02f; t < 1.0f; t += 0.02f) {
                const VoxelMap::State st = M2.stateAt(camE, camN + t, camU);
                if (st == VoxelMap::OCCUPIED) ++occ;
                else if (st == VoxelMap::FREE) ++freeN;
            }
        };
        int occOff = 0, freeOff = 0, occOn = 0, freeOn = 0;
        countStates(0.f,    occOff, freeOff);     // old behaviour
        countStates(0.25f,  occOn,  freeOn);      // gated

        check(occOff > 0, "without the gate, a 0.12 m return marks cells OCCUPIED",
              std::to_string(occOff) + " samples");
        check(occOn == 0, "with it, nothing inside the minimum range is marked",
              std::to_string(occOn) + " samples");
        check(freeOn == 0, "and nothing is CARVED either -- no mark, no carve",
              std::to_string(freeOn) + " samples");
    }

    // --- no range gap between ladder bands -----------------------------
    // A band end past the layer's own marking range is territory it owns and
    // has no data for, and the next layer is banded out of it. Because range
    // along a ray grows as D/cos(theta), that shell renders as a CIRCLE on the
    // optical axis -- reported from the field as a round blind spot.
    {
        VoxelMapParams qf; qf.cell = 0.10f; qf.maxIntegM = 3.0f;
        qf.maxCarveM = 8.f; qf.depthSigCoef = 0.f; qf.minIntegM = 0.f;
        VoxelMapParams qc = qf; qc.cell = 1.0f; qc.maxIntegM = 12.f;
        VoxelMap Mf, Mc; Mf.init(qf, camE, camN, camU); Mc.init(qc, camE, camN, camU);
        // A frontal wall at 3.4 m -- past the fine layer's 3.0 m marking range,
        // so only the coarse layer can hold it.
        cv::Mat d4(d.rows, d.cols, CV_32F, cv::Scalar(3.4f));
        for (int i = 0; i < 6; ++i) { Mf.integrate(d4, cam, pose); Mc.integrate(d4, cam, pose); }

        auto centreHits = [&](float fineBandEnd) {
            cv::Mat m;
            VoxelMap::renderLadder({{&Mf, 0.f, fineBandEnd},
                                    {&Mc, fineBandEnd, 14.f}},
                                   camE, camN, camU, yaw, pitch, 120, 120, 87.f,
                                   FpvStyle(), &m);
            int c = 0;                                   // centre 20x20 block
            for (int y = 50; y < 70; ++y)
                for (int x = 50; x < 70; ++x) c += m.at<uchar>(y, x) ? 1 : 0;
            return c;
        };
        check(centreHits(3.0f) == 400,
              "handing over at the exact marking range fills the pane centre",
              std::to_string(centreHits(3.0f)) + "/400 px");
        // The 1.15x inflation used to blank the centre outright, and this test
        // pinned that symptom. It no longer does, because a level may now borrow
        // into the band below it by min(half a cell, kBorrowM) -- a 1 m cell
        // covers a 0.45 m inflation comfortably. Pin the property rather than
        // the symptom: an inflated handover must not COST coverage.
        check(centreHits(3.0f * 1.15f) >= 400 * 9 / 10,
              "and a modestly inflated one no longer blanks it",
              std::to_string(centreHits(3.0f * 1.15f)) + "/400 px");
        // ... but the borrow is capped in METRES, so a rung too coarse to place
        // a surface within that cap stays banded out instead of filling the
        // pane with near faces. Measured before the cap: an 8 m rung covered
        // 99.9 % of the pane with a solid wall.
        {
            VoxelMapParams q8 = qc; q8.cell = 8.0f; q8.maxIntegM = 20.f;
            VoxelMap M8; M8.init(q8, camE, camN, camU);
            for (int i = 0; i < 6; ++i) M8.integrate(d4, cam, pose);
            cv::Mat m8;
            VoxelMap::renderLadder({{&Mf, 0.f, 3.f}, {&M8, 3.f, 22.f}},
                                   camE, camN, camU, yaw, pitch, 120, 120, 87.f,
                                   FpvStyle(), &m8);
            int c8 = 0;
            for (int y = 50; y < 70; ++y)
                for (int x = 50; x < 70; ++x) c8 += m8.at<uchar>(y, x) ? 1 : 0;
            check(c8 < 100, "and a rung too coarse to place a surface inside the "
                            "borrow cap stays banded out",
                  std::to_string(c8) + "/400 px");
        }
    }

    // --- the carve clamp must scale with the CELL, not be a fixed metre ----
    // A porous obstacle (hedge, foliage, a pole) is hit by some rays and passed
    // by many. The clamp exists so the passing rays cannot carve the cell the
    // hitting rays marked -- but it was `lm + 0.5 m`, which on a 4 m grid is an
    // eighth of a cell, so the carve ran through the middle of the very cell it
    // was protecting. Observed as a hedge plainly visible at 8 m with no voxels.
    {
        auto porousSurvives = [&](float cellM) {
            VoxelMapParams q;
            q.cell = cellM; q.maxIntegM = 20.f; q.maxCarveM = 40.f;
            q.depthSigCoef = 0.f; q.carveWinPx = 9; q.minIntegM = 0.f;
            VoxelMap M2; M2.init(q, camE, camN, camU);
            // A porous surface at 8 m: every 4th column returns 8 m, the rest
            // see straight through to 30 m. Three hitting rays per twelve.
            cv::Mat d3(d.rows, d.cols, CV_32F, cv::Scalar(30.f));
            for (int y = 0; y < d3.rows; ++y)
                for (int x = 0; x < d3.cols; x += 4) d3.at<float>(y, x) = 8.f;
            for (int i = 0; i < 8; ++i) M2.integrate(d3, cam, pose);
            // Scan a cell either side rather than probing 8.0 m exactly. The
            // ray endpoint lands a hair short of the boundary and floors into
            // the previous cell, so an exact probe tests the wrong cell -- my
            // first version of this failed on BOTH grids for that reason, and
            // the instrument was the fault, not the map.
            for (float t = 8.f - cellM; t <= 8.f + cellM; t += cellM * 0.25f)
                if (M2.stateAt(camE, camN + t, camU) == VoxelMap::OCCUPIED) return true;
            return false;
        };
        check(porousSurvives(0.25f),
              "a porous surface survives on a fine grid (it always did)");
        check(porousSurvives(4.0f),
              "and now survives on a COARSE grid, where the clamp used to be "
              "an eighth of a cell");
    }

    // --- the carve guard is an ANGULAR question ---------------------------
    // The case above marks every return, because maxIntegM is 20 m. The real
    // map does not: a layer honest to 3.5 m treats a 10 m return as "the space
    // in front of me is empty" and NOT as "there is a surface here", so those
    // rays carve with nothing to weigh against them. In a forest that is 95 %
    // of the returns, and one hit at +0.85 does not survive a hundred carves
    // at -0.40.
    //
    // Against that, a fixed pixel window is the wrong instrument, because the
    // question is whether a surface could be inside the same CELL -- and a cell
    // subtends cell*f/r pixels, which varies by two orders of magnitude over
    // the ladder. Nine pixels answers it at one range and nowhere else.
    {
        auto foliageHolds = [&](float cellM, int gapPx) {
            VoxelMapParams q;
            q.cell = cellM; q.maxIntegM = 3.5f; q.maxCarveM = 25.f;
            q.depthSigCoef = 0.f; q.carveWinPx = 9; q.minIntegM = 0.f;
            // Foliage at 2.5 m: every gapPx-th pixel returns it, the rest see
            // 10 m straight past -- beyond the marking range, so those rays
            // carve and never mark.
            cv::Mat d4(d.rows, d.cols, CV_32F, cv::Scalar(10.f));
            for (int y = 0; y < d4.rows; y += gapPx)
                for (int x = 0; x < d4.cols; x += gapPx) d4.at<float>(y, x) = 2.5f;
            VoxelMap M4; M4.init(q, camE, camN, camU);
            for (int i = 0; i < 8; ++i) M4.integrate(d4, cam, pose);
            // Scan a BOX. camE lands exactly on a cell boundary in x, so an
            // on-axis probe reads whichever side floor() happens to pick while
            // the marks land on both -- the same instrument fault as the range
            // probe above, in a different coordinate, and it cost half an hour.
            int occ = 0;
            for (float t = 2.5f - cellM; t <= 2.5f + cellM; t += cellM * 0.5f)
                for (float e = -cellM; e <= cellM; e += cellM * 0.5f)
                    for (float uu = -cellM; uu <= cellM; uu += cellM * 0.5f)
                        if (M4.stateAt(camE + e, camN + t, camU + uu)
                            == VoxelMap::OCCUPIED) ++occ;
            return occ;
        };
        // Coarse cells are where the fixed window fails, because that is where
        // a cell is widest in pixels. Measured with the old single 9 px erode:
        // 8 occupied samples at 2 m cells, 40 at 1 m. With the pyramid: 32
        // and 53. The threshold sits in the gap so the test discriminates.
        check(foliageHolds(2.0f, 8) >= 24,
              "porous foliage survives on a COARSE grid when the rays past it "
              "are too far to mark",
              std::to_string(foliageHolds(2.0f, 8)) + " occupied samples");
        check(foliageHolds(1.0f, 8) >= 48,
              "and on a 1 m grid",
              std::to_string(foliageHolds(1.0f, 8)) + " occupied samples");
        // ... and the angular rule must never NARROW the guard. A fine cell at
        // long range spans fewer pixels than carveWinPx, and dropping to that
        // width lets an 8 px foliage gap carve through -- measured, 30
        // occupied samples became 0. carveWinPx is a floor, not a suggestion.
        {
            VoxelMapParams q;
            q.cell = 0.25f; q.maxIntegM = 10.f; q.maxCarveM = 40.f;
            q.depthSigCoef = 0.f; q.carveWinPx = 9; q.minIntegM = 0.f;
            cv::Mat d6(d.rows, d.cols, CV_32F, cv::Scalar(25.f));
            for (int y = 0; y < d6.rows; y += 8)
                for (int x = 0; x < d6.cols; x += 8) d6.at<float>(y, x) = 8.f;
            VoxelMap M6; M6.init(q, camE, camN, camU);
            for (int i = 0; i < 8; ++i) M6.integrate(d6, cam, pose);
            int occ = 0;
            for (float t = 8.f - 0.25f; t <= 8.f + 0.25f; t += 0.125f)
                for (float e = -0.25f; e <= 0.25f; e += 0.125f)
                    for (float uu = -0.25f; uu <= 0.25f; uu += 0.125f)
                        if (M6.stateAt(camE + e, camN + t, camU + uu)
                            == VoxelMap::OCCUPIED) ++occ;
            check(occ >= 24, "and a FINE cell at long range keeps the old fixed "
                             "window, which the angular rule alone would shrink",
                  std::to_string(occ) + " occupied samples");
        }
        // ... and none of this may simply refuse to carve. An unbroken wall at
        // 8 m, every return beyond the marking range, still has to leave
        // confirmed-free space in front of it -- a map that calls nothing free
        // is a vehicle that never moves, and this project has had that failure.
        VoxelMapParams q5;
        q5.cell = 0.25f; q5.maxIntegM = 3.5f; q5.maxCarveM = 25.f;
        q5.depthSigCoef = 0.f; q5.carveWinPx = 9; q5.minIntegM = 0.f;
        cv::Mat d5(d.rows, d.cols, CV_32F, cv::Scalar(8.f));
        VoxelMap M5; M5.init(q5, camE, camN, camU);
        for (int i = 0; i < 8; ++i) M5.integrate(d5, cam, pose);
        int freeAhead = 0;
        for (float t = 1.f; t <= 7.f; t += 0.25f)
            if (M5.stateAt(camE, camN + t, camU) == VoxelMap::FREE) ++freeAhead;
        check(freeAhead >= 20, "and open space in front of a plain wall is "
                               "still carved free",
              std::to_string(freeAhead) + " of 25 samples");
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
