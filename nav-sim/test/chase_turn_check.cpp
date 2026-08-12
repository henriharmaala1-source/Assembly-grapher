// Does the 3D chase view actually TURN when the plan is not straight ahead?
//
// The question is sharper than it sounds, because the two panes that show the
// plan use different references and so cannot be checked against each other by
// eye. The top-down PLAN pane is drawn in WORLD axes (North up, East right) and
// does not rotate with the aircraft; the CHASE pane is drawn from an eye that
// does. At any heading other than North the same path therefore appears at two
// different angles, both correct, and comparing them proves nothing.
//
// So the invariant to test is the one the chase view actually claims:
//
//   A path with fixed BODY-frame shape must land on the SAME PIXELS at every
//   heading.
//
// Rotating the world and the eye together must cancel exactly. Any disagreement
// anywhere in the chain -- the planner's body->world rotation, the chase eye
// placement, the renderer's yaw convention -- breaks that cancellation, and
// each of those three is a place where clockwise-from-North could quietly
// become anticlockwise-from-East without any single view looking wrong.
//
//   g++ -O2 -std=c++17 -I. test/chase_turn_check.cpp voxel_map.cpp \
//       voxel_traj.cpp voxel_planner.cpp depth_camera.cpp voxel_world.cpp \
//       -I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -o /tmp/ct && /tmp/ct

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_planner.hpp"
#include "voxel_traj.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& d = "") {
    std::printf("  %-58s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ",
                d.c_str());
    if (!ok) ++fails;
}

// The two formulas under test, written once here exactly as voxel_traj.cpp and
// voxel_live.cpp write them. Duplicated deliberately: if either moves, the
// comparison against the real planner in part 3 fails and says so.
static void bodyToWorld(float px, float py, float pz, float yawDeg,
                        const std::array<float, 3>& b,
                        float& wx, float& wy, float& wz) {
    const float ca = std::cos(yawDeg * PI_F / 180.f);
    const float sa = std::sin(yawDeg * PI_F / 180.f);
    wx = px + b[0] * ca + b[1] * sa;
    wy = py - b[0] * sa + b[1] * ca;
    wz = pz + b[2];
}

struct ChaseEye { float e, n, u, yaw, pitch, fov; };
static ChaseEye chaseEyeFor(float pe, float pn, float pu, float yawDeg, float span) {
    const float back = span * 0.55f, up = span * 0.62f;
    const float aimAhead = back + span * 0.5f;
    const float yr = yawDeg * PI_F / 180.f;
    return {pe - std::sin(yr) * back, pn - std::cos(yr) * back, pu + up, yawDeg,
            -std::atan2(up, aimAhead) * 180.f / PI_F, 55.f};
}

int main() {
    std::printf("chase view: does it turn with the aircraft?\n");

    const int W = 420, H = 320;
    const float cx = (W - 1) * 0.5f;
    const float pe = 30.f, pn = 30.f, pu = 6.f;
    const float span = 3.f;

    // A path that curves to the aircraft's LEFT. Body +y is forward and body +x
    // is right (yaw 90 sends +y East and +x South -- heading East, right hand
    // points South), so left is NEGATIVE x.
    std::vector<std::array<float, 3>> leftCurve, rightCurve;
    for (int i = 0; i <= 12; ++i) {
        const float t = i / 12.f;
        const float fwd = t * span, lat = 1.2f * t * t;
        leftCurve.push_back({-lat, fwd, 0.f});
        rightCurve.push_back({ lat, fwd, 0.f});
    }

    // --- 1. the same body-frame path lands on the same pixels at every heading
    const float YAWS[] = {0.f, 37.f, 90.f, 180.f, 250.f, 315.f, -73.f};
    std::vector<std::array<float, 2>> ref;
    float worstDrift = 0.f;
    int dropped = 0;
    for (float yaw : YAWS) {
        const ChaseEye eye = chaseEyeFor(pe, pn, pu, yaw, span);
        std::vector<std::array<float, 2>> px;
        for (const auto& b : leftCurve) {
            float wx, wy, wz; bodyToWorld(pe, pn, pu, yaw, b, wx, wy, wz);
            float u, v;
            if (!VoxelMap::fpvProject(eye.e, eye.n, eye.u, eye.yaw, eye.pitch,
                                      W, H, eye.fov, wx, wy, wz, u, v)) { ++dropped; continue; }
            px.push_back({u, v});
        }
        if (ref.empty()) { ref = px; continue; }
        if (px.size() != ref.size()) { worstDrift = 1e9f; break; }
        for (size_t i = 0; i < px.size(); ++i)
            worstDrift = std::max(worstDrift,
                                  std::max(std::fabs(px[i][0] - ref[i][0]),
                                           std::fabs(px[i][1] - ref[i][1])));
    }
    check(!ref.empty() && dropped == 0,
          "every point of a forward path is in front of the chase eye",
          std::to_string(dropped) + " dropped");
    check(worstDrift < 0.01f,
          "the same body path lands on the same pixels at every heading",
          std::to_string(int(sizeof(YAWS) / sizeof(float))) + " headings, worst " +
          std::to_string(worstDrift) + " px");

    // That invariant is satisfied by a view that ignores the path entirely, so
    // it needs a partner: the path must actually be drawn where it goes.
    {
        const ChaseEye eye = chaseEyeFor(pe, pn, pu, 0.f, span);
        auto tipU = [&](const std::vector<std::array<float, 3>>& path) {
            float wx, wy, wz; bodyToWorld(pe, pn, pu, 0.f, path.back(), wx, wy, wz);
            float u, v;
            VoxelMap::fpvProject(eye.e, eye.n, eye.u, eye.yaw, eye.pitch, W, H,
                                 eye.fov, wx, wy, wz, u, v);
            return u;
        };
        const float lu = tipU(leftCurve), ru = tipU(rightCurve);
        check(lu < cx - 20.f, "a LEFT-curving path is drawn left of centre",
              std::to_string(cx - lu) + " px left");
        check(ru > cx + 20.f, "and a RIGHT-curving one to the right",
              std::to_string(ru - cx) + " px right");
        check(std::fabs((lu - cx) + (ru - cx)) < 1.f,
              "symmetrically, so the view is not sheared");
    }

    // --- 2. and it is not simply centring on whatever it is given -----------
    // A path that leaves the aircraft SIDEWAYS should leave the frame. If it
    // does not, the eye is tracking the path rather than the heading, and the
    // "turn" in part 1 would be cosmetic.
    {
        const ChaseEye eye = chaseEyeFor(pe, pn, pu, 0.f, span);
        float wx, wy, wz; bodyToWorld(pe, pn, pu, 0.f, {-6.f, 0.5f, 0.f}, wx, wy, wz);
        float u, v;
        const bool on = VoxelMap::fpvProject(eye.e, eye.n, eye.u, eye.yaw, eye.pitch,
                                             W, H, eye.fov, wx, wy, wz, u, v);
        check(!on || u < 0.f || u >= W,
              "a hard-left escape leaves the frame, as a chase view should");
    }

    // --- 3. the REAL planner uses the same rotation -------------------------
    // Parts 1 and 2 test the drawing chain against a rotation written here. This
    // ties that rotation to the one TrajectoryPlanner actually applies, so the
    // test cannot pass while the planner disagrees with the view.
    {
        // A room, so there is confirmed-FREE space for rollouts to live in.
        // Rollouts only earn length through cells the map has SEEN to be free,
        // so the map has to be carved from the same yaw it is planned at.
        //
        // A SMALL room -- 6 m to each wall -- and that is not an aesthetic
        // choice. Carving stops at `r - carveSigK*sigma`, and sigma goes as
        // Z^2/(f*B): at 30 m on a 50 mm baseline it is 20 m, so a wall that far
        // away is worth nothing and the map correctly carves NOTHING from it.
        // The first version of this test used a 60 m room and every primitive
        // was rejected at its first point -- the map being right, not wrong.
        // At 6 m sigma is 0.81 m and carving reaches 4.4 m, comfortably past
        // the 3 m rollouts.
        VoxelWorld w;
        const float wc = 0.25f;
        const int n = int(12.f / wc), nz = int(12.f / wc);
        w.init(wc, pe - 6.f, pn - 6.f, pu - 6.f, n, n, nz);
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < nz; ++k) {
                w.set(i, 0, k, true);        w.setTex(i, 0, k, 0.8f);
                w.set(i, n - 1, k, true);    w.setTex(i, n - 1, k, 0.8f);
                w.set(0, i, k, true);        w.setTex(0, i, k, 0.8f);
                w.set(n - 1, i, k, true);    w.setTex(n - 1, i, k, 0.8f);
            }

        CamParams cp;
        cp.width = 424; cp.height = 240; cp.hfovDeg = 87.f; cp.baselineM = 0.05f;
        cp.maxRangeM = 40.f;
        DepthCamera cam(cp);

        // A SMALL robot radius, on purpose. The map is FOV-limited: cells
        // beside and behind the aircraft were never looked at and are UNKNOWN,
        // and unknown is not free -- so a 0.3 m sphere fails the clearance test
        // at the rollout's first point and every primitive is rejected before
        // it starts. That is the map behaving correctly and it is the same
        // BLOCKED-in-a-corridor mechanism seen on the real camera; it is just
        // not what this test is about.
        TrajParams tp;
        tp.vMax = 1.5f; tp.robotR = 0.05f; tp.goalWeight = 0.f;
        TrajectoryPlanner planner(tp);

        for (float yaw : {0.f, 90.f, 200.f}) {
            CamPose pose; pose.e = pe; pose.n = pn; pose.u = pu;
            pose.yawDeg = yaw; pose.pitchDeg = 0.f;
            cv::Mat d = cam.renderTruth(w, pose);

            VoxelMapParams mp;
            mp.cell = 0.25f; mp.maxIntegM = 6.f; mp.maxCarveM = 12.f;
            mp.depthSigCoef = 0.25f / (cam.fpx() * cp.baselineM);
            VoxelMap M;
            M.init(mp, pe, pn, pu);
            for (int i = 0; i < 6; ++i) M.integrate(d, cam, pose);

            GeneralResult gr = planner.plan(M, pe, pn, pu, yaw, 0.f, 0.f, {});
            const auto& ch = planner.chosen();
            if (ch.size() < 3) {
                const auto& rj = planner.lastReject();
                check(false, "the planner produced a path to compare",
                      "yaw " + std::to_string(int(yaw)) + ": " +
                      std::to_string(ch.size()) + " pts, rejected occ " +
                      std::to_string(rj.occupied) + " unk " + std::to_string(rj.unknown) +
                      " short " + std::to_string(rj.tooShort) + " atStart " +
                      std::to_string(rj.atStart));
                continue;
            }
            // Where did it actually go, in BODY terms? Undo the heading and the
            // forward component must dominate -- every primitive starts along
            // body +y, so this is true whatever the planner chose.
            const float dx = ch[2][0] - pe, dy = ch[2][1] - pn;
            const float ca = std::cos(yaw * PI_F / 180.f), sa = std::sin(yaw * PI_F / 180.f);
            const float fwd = dx * sa + dy * ca;       // inverse of bodyToWorld
            const float lat = dx * ca - dy * sa;
            check(fwd > 0.f && fwd > std::fabs(lat),
                  "the planner's own path leaves along the heading it was given",
                  "yaw " + std::to_string(int(yaw)) + ": fwd " +
                  std::to_string(fwd) + " m, lat " + std::to_string(lat) + " m");
            (void)gr;
        }
    }

    // --- 4. the turn HUD's bearing convention -------------------------------
    // The first-person pane cannot show a forward path, so it shows the COMMAND
    // instead: a caret offset by angDiffDeg(commanded, heading). That sign is
    // the entire content of "which way to turn" and it inverts silently — a
    // mirrored HUD looks exactly as convincing as a correct one. The wrap is
    // the other half: a command 20 deg right of a heading of 350 must read +20,
    // not -340, and the aircraft must not be told to turn most of the way round
    // to reach a bearing it is nearly pointing at.
    {
        check(angDiffDeg(30.f, 0.f) > 0.f, "a bearing clockwise of the nose reads RIGHT",
              std::to_string(angDiffDeg(30.f, 0.f)));
        check(angDiffDeg(330.f, 0.f) < 0.f, "and anticlockwise reads LEFT",
              std::to_string(angDiffDeg(330.f, 0.f)));
        check(std::fabs(angDiffDeg(10.f, 350.f) - 20.f) < 1e-3f,
              "the wrap is short-way-round across North",
              std::to_string(angDiffDeg(10.f, 350.f)));
        check(std::fabs(angDiffDeg(350.f, 10.f) + 20.f) < 1e-3f,
              "and in the other direction");
        // Over a range far wider than any bearing, because the HUD's second
        // argument is an ACCUMULATING heading, not an atan2 output. The
        // one-line form of this function silently returned -480 out here.
        float worst = 0.f, worstErr = 0.f;
        for (int a = -4000; a <= 4000; a += 7) {
            worst = std::max(worst, std::fabs(angDiffDeg(float(a), 0.f)));
            // and it must still be the right answer, not merely in range
            worstErr = std::max(worstErr,
                                std::fabs(angDiffDeg(float(a) + 3000.f, 3000.f)
                                          - angDiffDeg(float(a), 0.f)));
        }
        check(worst <= 180.f, "never exceeds 180 deg, over +-4000 deg of input",
              std::to_string(worst));
        check(worstErr < 0.01f, "and a 3000 deg heading gives the same answer as 0",
              std::to_string(worstErr));

        // Tied to the same rotation the rest of this file uses: a path curving
        // LEFT in the body frame must produce a NEGATIVE relative bearing, or
        // the caret and the picture disagree.
        for (float yaw : {0.f, 90.f, 200.f, 355.f}) {
            float wx, wy, wz;
            bodyToWorld(pe, pn, pu, yaw, leftCurve.back(), wx, wy, wz);
            const float az = std::atan2(wx - pe, wy - pn) * 180.f / PI_F;
            check(angDiffDeg(az, yaw) < -5.f,
                  "a left-curving path commands a LEFT turn at every heading",
                  "yaw " + std::to_string(int(yaw)) + ": " +
                  std::to_string(angDiffDeg(az, yaw)) + " deg");
        }
    }

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
