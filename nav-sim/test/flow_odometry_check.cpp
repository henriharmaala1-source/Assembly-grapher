// Does flow x depth, integrated, actually recover a trajectory?
//
// The pieces were each tested alone: flow_velocity_check pins the solve against
// a synthetic pair, imu_odometry_check pins the drift model. Neither answers
// the question that matters for voxel_live, which is whether the CHAIN --
// render, match, de-rotate, scale, rotate to world, integrate -- reproduces a
// known path. That needs a world, a camera and ground truth, so it lives here.
//
// The IR render is described in depth_camera.hpp. Its important property is
// that intensity is a function of the WORLD POINT, so a moving camera sees the
// same surface with the same appearance. Every error this test can catch is
// therefore an error in the estimator or in a frame convention.
#include <cmath>
#include <cstdio>
#include <string>

#include "depth_camera.hpp"
#include "flow_velocity.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& detail = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL",
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++failures;
}
static std::string f2(float v) { char b[64]; std::snprintf(b, sizeof b, "%.3f", v); return b; }

struct Leg { float vE = 0, vN = 0, vU = 0; float yawRateDps = 0.f; };
struct Run { float dE = 0, dN = 0, dU = 0; int solved = 0, attempted = 0; float meanPoints = 0.f; };

static Run fly(const VoxelWorld& w, const DepthCamera& cam,
               const FlowVelocityEstimator& est,
               CamPose p0, const Leg& leg, int steps, float dt) {
    Run r;
    cv::Mat prevIr;
    CamPose p = p0;
    long pts = 0;
    for (int s = 0; s < steps; ++s) {
        cv::Mat ir    = cam.renderIR(w, p);
        cv::Mat depth = cam.renderTruth(w, p);
        if (!prevIr.empty()) {
            ++r.attempted;
            FlowVelocity fv = est.estimate(prevIr, ir, depth, cam,
                                           leg.yawRateDps * dt, 0.f, 0.f, dt);
            if (fv.valid) {
                float wE, wN, wU;
                DepthCamera::camToWorld(p, fv.vx, fv.vy, fv.vz, wE, wN, wU);
                r.dE += wE * dt; r.dN += wN * dt; r.dU += wU * dt;
                ++r.solved; pts += fv.points;
            }
        }
        prevIr = ir;
        p.e += leg.vE * dt; p.n += leg.vN * dt; p.u += leg.vU * dt;
        p.yawDeg += leg.yawRateDps * dt;
    }
    r.meanPoints = r.solved ? float(pts) / float(r.solved) : 0.f;
    return r;
}

int main() {
    std::printf("flow -> odometry, over a rendered world\n");

    CamParams cp;
    cp.width = 320; cp.height = 240; cp.hfovDeg = 70.f;
    cp.baselineM = 0.05f; cp.maxRangeM = 12.f;
    DepthCamera cam(cp);

    ForestParams fp; fp.cell = 0.25f; fp.seed = 7;
    VoxelWorld forest; genForest(forest, fp, nullptr);

    FlowVelocityParams vp; vp.maxRangeM = 10.f;
    FlowVelocityEstimator est; est.init(vp);

    const float dt = 0.1f;
    const int   N  = 20;
    CamPose start; start.e = 15.f; start.n = 10.f; start.u = 6.f; start.yawDeg = 0.f;

    {
        Run r = fly(forest, cam, est, start, Leg{0.6f, 0.f, 0.f, 0.f}, N, dt);
        check("a rendered forest yields trackable points", r.meanPoints >= 8.f,
              f2(r.meanPoints) + " pts/frame, " + std::to_string(r.solved)
              + "/" + std::to_string(r.attempted) + " solved");
    }
    {
        const float v = 0.6f;
        Run r = fly(forest, cam, est, start, Leg{v, 0.f, 0.f, 0.f}, N, dt);
        const float truth = v * dt * float(r.attempted);
        check("east translation recovered", std::fabs(r.dE - truth) < 0.35f * truth + 0.05f,
              f2(r.dE) + " vs " + f2(truth) + " m");
        check("and no phantom north motion", std::fabs(r.dN) < 0.35f * truth + 0.08f,
              f2(r.dN) + " m");
    }
    {
        // Forward is the HARD axis: a point on the optical axis has no parallax
        // under forward motion, so the solve leans on the periphery.
        const float v = 0.6f;
        Run r = fly(forest, cam, est, start, Leg{0.f, v, 0.f, 0.f}, N, dt);
        const float truth = v * dt * float(r.attempted);
        check("north (forward) translation recovered",
              std::fabs(r.dN - truth) < 0.45f * truth + 0.05f,
              f2(r.dN) + " vs " + f2(truth) + " m");
    }
    {
        Run r = fly(forest, cam, est, start, Leg{0.f, 0.f, 0.f, 25.f}, N, dt);
        const float moved = std::sqrt(r.dE*r.dE + r.dN*r.dN + r.dU*r.dU);
        check("a pure 25 deg/s yaw integrates to ~no displacement", moved < 0.30f,
              f2(moved) + " m over " + f2(dt * float(r.attempted)) + " s");
    }
    // ---- YIELD vs TEXTURE ---------------------------------------------
    // The open question for real hardware is not whether the solve is correct
    // -- flow_velocity_check settles that -- but how often it FIRES on real
    // bark. This is the rig that can answer it, so it reports a curve rather
    // than a single verdict. Note the ground keeps its own texture here, which
    // is why yield never reaches zero: a forest floor is matchable even when
    // the trunks are not, and that is true of the real thing too.
    {
        std::printf("  yield vs trunk texture (texThresh for stereo is 0.25):\n");
        const float texes[] = {0.02f, 0.10f, 0.30f, 0.75f};
        float first = -1.f, last = -1.f;
        for (float tx : texes) {
            ForestParams bp; bp.cell = 0.25f; bp.seed = 7;
            bp.trunkTexMin = tx; bp.trunkTexMax = tx;
            VoxelWorld world; genForest(world, bp, nullptr);
            Run r = fly(world, cam, est, start, Leg{0.6f, 0.f, 0.f, 0.f}, N, dt);
            std::printf("      trunkTex %.2f -> %6.2f pts/frame, %2d/%2d solved\n",
                        tx, r.meanPoints, r.solved, r.attempted);
            if (first < 0.f) first = r.meanPoints;
            last = r.meanPoints;
        }
        // MEASURED, and not what I expected: yield is nearly FLAT in trunk
        // texture. The sparse grid samples 12x9 points over the whole frame, and
        // at 6 m altitude most of them land on ground and canopy rather than on
        // bark -- so the ground carries the solve whatever the trunks are doing.
        //
        // Treat that as a hypothesis about the real sensor, not a result. This
        // ground is synthetic and its brightness varies with range through the
        // modelled projector falloff, which is physical but is also exactly the
        // kind of structure a renderer can flatter. The real measurement still
        // needs the camera pointed at real ground.
        check("yield stays above the solver's minimum across the texture range",
              first >= 8.f && last >= 8.f,
              f2(first) + " -> " + f2(last) + " pts/frame, ground-dominated");
    }

    // ---- THE DANGEROUS FAILURE --------------------------------------------
    // A confident zero is what kills you: the estimator reports standing still,
    // odometry believes it, and drift is integrated as truth. Pinned at the
    // unit level in flow_velocity_check against a flat wall; asserted here at
    // the chain level -- when the solve does not fire, NOTHING is integrated.
    {
        Run r = fly(forest, cam, est, start, Leg{0.f, 0.f, 0.f, 0.f}, 3, dt);
        check("a stationary camera integrates no displacement",
              std::sqrt(r.dE*r.dE + r.dN*r.dN + r.dU*r.dU) < 0.05f,
              f2(std::sqrt(r.dE*r.dE + r.dN*r.dN + r.dU*r.dU)) + " m");
    }

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
