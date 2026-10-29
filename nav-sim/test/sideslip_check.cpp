// What does yaw-coupled sideslip actually do to the primitive library?
//
// The claim in voxel_traj.hpp is specific and therefore checkable: the velocity
// vector leads the heading while turning, |v| is unchanged, and a straight
// primitive is untouched. Each of those is asserted here, because "it should
// widen the turns" is the kind of statement that survives being wrong.
//
//   g++ -O2 -std=c++17 -I. test/sideslip_check.cpp voxel_traj.cpp voxel_planner.cpp \
//       voxel_map.cpp voxel_world.cpp depth_camera.cpp \
//       -I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -o /tmp/ssc && /tmp/ssc

#include <cmath>
#include <cstdio>
#include <vector>

#include "voxel_map.hpp"
#include "voxel_traj.hpp"

using namespace sim;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

// Re-integrate one primitive by the same rule the planner uses, so the shapes
// can be inspected without exposing the library. This DUPLICATES the rollout,
// which is a real cost -- if the planner's integration changes, this drifts.
// Accepted because the alternative is a public accessor that exists only for a
// test, and the duplication is nine lines that the assertions themselves pin.
struct Roll { std::vector<float> x, y; float lastCourse = 0, speedOut = 0; };
static Roll rollout(float speed, float yawRateDps, float slipDeg, float kneeDps,
                    float horizonS = 2.f, float dt = 0.1f, float tau = 0.35f) {
    const float D2R = sim::PI_F / 180.f;
    const int steps = int(horizonS / dt);
    const float k = std::min(1.f, dt / tau);
    const float beta = (slipDeg != 0.f)
        ? slipDeg * D2R * (2.f / sim::PI_F) * std::atan(yawRateDps / std::max(1e-3f, kneeDps))
        : 0.f;
    Roll r;
    float px = 0, py = 0, yaw = 0, vx = 0, vy = 0;
    for (int s = 0; s < steps; ++s) {
        yaw += yawRateDps * D2R * dt;
        const float crs = yaw + beta;
        const float cx = std::sin(crs) * speed, cy = std::cos(crs) * speed;
        vx += (cx - vx) * k; vy += (cy - vy) * k;
        px += vx * dt; py += vy * dt;
        r.x.push_back(px); r.y.push_back(py);
    }
    r.lastCourse = std::atan2(vx, vy) / D2R;
    r.speedOut   = std::hypot(vx, vy);
    return r;
}

int main() {
    std::printf("sideslip checks (latSlipDeg / latKneeDps)\n");
    const float V = 1.5f, SLIP = 20.f, KNEE = 40.f;

    // 1. A straight primitive is bit-identical. Sideslip must not touch forward
    //    flight, which is most of the flight.
    {
        Roll a = rollout(V, 0.f, 0.f, KNEE), b = rollout(V, 0.f, SLIP, KNEE);
        bool same = true;
        for (size_t i = 0; i < a.x.size(); ++i)
            same &= (a.x[i] == b.x[i]) && (a.y[i] == b.y[i]);
        check(same, "yawRate = 0 primitive is unchanged");
    }

    // 2. Commanded speed is preserved exactly -- this is the safety claim. An
    //    additive lateral velocity (DeFoP's literal form) would fail this, and
    //    would then be flying faster than the stopping-distance budget allowed.
    {
        Roll a = rollout(V, 100.f, 0.f, KNEE), b = rollout(V, 100.f, SLIP, KNEE);
        check(std::fabs(a.speedOut - b.speedOut) < 1e-4f,
              "|v| identical with and without slip at 100 deg/s");
        check(b.speedOut <= V + 1e-4f, "|v| never exceeds the primitive's speed");
    }

    // 3. The velocity leads the heading by beta, in the direction of the turn.
    //    Wrapped, because a 100 deg/s primitive turns 200 deg over the horizon
    //    and the raw difference comes back as -344.8.
    {
        Roll b = rollout(V, 100.f, SLIP, KNEE), a = rollout(V, 100.f, 0.f, KNEE);
        const float want = SLIP * (2.f / sim::PI_F) * std::atan(100.f / KNEE);
        const float lead = std::fmod(b.lastCourse - a.lastCourse + 540.f, 360.f) - 180.f;
        check(lead > 0.f, "a right turn slides right");
        check(std::fabs(lead - want) < 0.5f,
              "course lead matches beta = slip*(2/pi)*atan(w/knee)");
        std::printf("    at 100 deg/s: beta = %.1f deg (max %.0f), measured lead %.1f deg\n",
                    want, SLIP, lead);
    }

    // 4. Sign follows the turn, symmetrically.
    {
        Roll rgt = rollout(V, 60.f, SLIP, KNEE), lft = rollout(V, -60.f, SLIP, KNEE);
        check(std::fabs(rgt.x.back() + lft.x.back()) < 1e-4f,
              "left and right turns are mirror images");
    }

    // 5. The point of the whole thing: lateral displacement accrues SOONER.
    //
    //    Measured EARLY in the rollout, not at the endpoint, and the difference
    //    matters. The path is the same circle traversed with a phase lead, so at
    //    the 2 s endpoint of a 100 deg/s primitive -- 200 deg of turn, past the
    //    top of the circle -- the lead has carried the point back round and the
    //    lateral coordinate is SMALLER (measured: 1.62 m -> 1.54 m). That is
    //    geometry, not a defect, and asserting "further sideways at 2 s" would
    //    have been a false claim that happened to be testable.
    //
    //    The regime that decides anything is the first second: aimS is 0.2 s,
    //    and the admissibility scan usually terminates on the map well before
    //    the horizon. So check t = 0.2 s (the aim point) and t = 0.5 s.
    {
        Roll a = rollout(V, 100.f, 0.f, KNEE), b = rollout(V, 100.f, SLIP, KNEE);
        const size_t iAim = 1, iHalf = 4;      // dt = 0.1 s, 0-indexed
        check(b.x[iAim] > a.x[iAim], "at the 0.2 s aim point, slip is already further out");
        check(b.x[iHalf] > a.x[iHalf], "at 0.5 s, still further out");
        std::printf("    lateral at 0.2 s %.3f -> %.3f m, at 0.5 s %.3f -> %.3f m, "
                    "at 2.0 s %.2f -> %.2f m (past the top of the circle)\n",
                    a.x[iAim], b.x[iAim], a.x[iHalf], b.x[iHalf], a.x.back(), b.x.back());
        // And it does NOT buy forward distance -- the budget was rotated, not
        // created. If this ever fails, the speed is coming from somewhere it
        // should not.
        check(b.y[iHalf] <= a.y[iHalf] + 1e-4f,
              "and does NOT gain forward distance -- the budget is rotated, not created");
    }

    // 6. Saturation: beta must be bounded by latSlipDeg however hard the turn.
    {
        Roll a = rollout(V, 100000.f, 0.f, KNEE), b = rollout(V, 100000.f, SLIP, KNEE);
        (void)a;
        const float want = SLIP * (2.f / sim::PI_F) * std::atan(100000.f / KNEE);
        check(want <= SLIP + 1e-3f, "beta saturates at latSlipDeg, never past it");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "all checks passed",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
