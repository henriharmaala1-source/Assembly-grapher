// Does the RL environment hold together, and does it refuse what it should?
//
// This is the random-agent run the plan calls for BEFORE any training. These
// projects die on the plumbing -- a reward that is silently always zero, a mask
// that never admits anything, an observation with a NaN in it -- and none of
// that is visible in a learning curve until days have been spent.
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "rl_env.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}
static std::string f2(float v) { char b[48]; std::snprintf(b, sizeof b, "%.2f", v); return b; }

int main() {
    std::printf("RL environment -- random agent\n");

    EnvConfig c;
    c.world = "maze"; c.seed = 3; c.maxSteps = 300;
    c.camW = 160; c.camH = 120; c.truthDepth = true;   // cheap for a smoke test
    VoxelEnv env(c);

    std::printf("  library %d primitives, observation %d floats\n",
                env.nPrims(), env.obsSize());
    check("the observation is the advertised size",
          (int)env.observation().size() == env.obsSize(),
          std::to_string(env.observation().size()));

    // MULTIPLE EPISODES. A random agent in the maze flies into the floor within
    // a second or two -- which is the collision detector working, not a bug --
    // so a single episode is far too short to exercise the mask or the unknown
    // channel. Aggregate over episodes instead.
    std::mt19937 rng(7);
    int admittedSteps = 0, illegalTried = 0, totalSteps = 0, episodes = 0, collided = 0;
    float totalR = 0.f, minR = 1e9f, maxR = -1e9f;
    bool sawNaN = false, sawUnknownChannel = false, sawFarMask = false;

    EnvStep st;
    for (int ep = 0; ep < 8; ++ep) {
        env.reset("maze", 3 + ep);
        ++episodes;
        for (int t = 0; t < c.maxSteps; ++t) {
            const auto& m = env.actionMask();
            std::vector<int> legal;
            for (size_t i = 0; i < m.size(); ++i) if (m[i]) legal.push_back((int)i);
            if (!legal.empty()) ++admittedSteps;

            int a;
            if (legal.empty() || (t % 5) == 0) {   // pick blind on purpose
                a = int(rng() % (unsigned)env.nPrims());
                if (a < (int)m.size() && !m[a]) ++illegalTried;
            } else {
                a = legal[rng() % legal.size()];
            }
            st = env.step(a);
            ++totalSteps;
            totalR += st.reward;
            minR = std::min(minR, st.reward); maxR = std::max(maxR, st.reward);

            const auto& o = env.observation();
            for (float v : o) if (!std::isfinite(v)) sawNaN = true;
            const int F = VoxelEnv::obsFeaturesPerPrim();
            for (int i = 0; i < env.nPrims(); ++i)
                if (o[size_t(i) * F + 7] > 0.5f) sawUnknownChannel = true;
            for (int k = 0; k < 6; ++k)
                if (o[size_t(env.nPrims()) * F + 10 + k * 2 + 1] > 0.5f) sawFarMask = true;
            if (st.done || st.truncated) { if (st.collisions) ++collided; break; }
        }
    }
    std::printf("  %d episodes, %d steps total, %d ended in a collision\n",
                episodes, totalSteps, collided);

    std::printf("  %d steps, travelled %.1f m, dist-to-goal %.1f m, stopped %d\n",
                st.steps, st.travelM, st.distToGoalM, st.stoppedSteps);
    std::printf("  reward total %.1f  min %.2f  max %.2f\n", totalR, minR, maxR);

    check("the step loop runs to completion", totalSteps > 50, std::to_string(totalSteps));
    check("no NaN or inf anywhere in the observation", !sawNaN);
    check("something was admissible on most steps",
          admittedSteps > totalSteps / 2,
          std::to_string(admittedSteps) + "/" + std::to_string(totalSteps));
    check("illegal actions were offered and survived", illegalTried > 0,
          std::to_string(illegalTried) + " masked picks");
    // The channel that would silently not exist if the observation were a
    // single scalar per direction. If this never fires, the policy is being
    // told nothing about the difference between blocked and unexplored.
    // MEASURED: it does NOT fire in the maze, and that is correct rather than a
    // gap. voxel_sim's own reject histogram there reads "occupied 209 unknown
    // 0" -- the walls are close enough that every rollout stops on a surface
    // before it ever reaches unmapped space. The channel is exercised in the
    // forest below, where sightlines are long and rollouts run off the edge of
    // what has been seen.
    std::printf("  maze: UNKNOWN channel fired = %d (expected 0, walls are close)\n",
                sawUnknownChannel ? 1 : 0);
    // A random agent must sometimes hit something, or the collision term is
    // never exercised and the reward is untested in the direction that matters.
    check("a random agent does collide", collided > 0,
          std::to_string(collided) + "/" + std::to_string(episodes) + " episodes");
    check("the far-field mask actually fires", sawFarMask);
    check("reward is not constant", std::fabs(maxR - minR) > 1e-3f,
          f2(minR) + " .. " + f2(maxR));

    // Determinism: the same seed must give the same trajectory, or a paired
    // comparison against the classical planners means nothing.
    {
        VoxelEnv a(c), b(c);
        std::mt19937 r1(11), r2(11);
        EnvStep sa, sb;
        for (int t = 0; t < 40; ++t) {
            sa = a.step(int(r1() % a.nPrims()));
            sb = b.step(int(r2() % b.nPrims()));
        }
        check("two envs with the same seed agree exactly",
              std::fabs(sa.travelM - sb.travelM) < 1e-6f
              && std::fabs(sa.distToGoalM - sb.distToGoalM) < 1e-6f,
              f2(sa.travelM) + " vs " + f2(sb.travelM));
    }

    // And forest, because the user wants both and they exercise different
    // failure modes -- forest is sparse and open, the maze is enclosed.
    {
        EnvConfig fc = c; fc.world = "forest"; fc.seed = 1; fc.maxSteps = 120;
        // 2.0 s, which is forest's own measured optimum (NOTES.md) and also
        // what makes the rollout LONGER than the carved free region: at 0.6 s
        // the 1.8 m rollout never leaves mapped space, so the unknown channel
        // has nothing to report. The maze wants 0.6 and the forest wants 2.0 --
        // the optimum tracks the size of the space, not the scene type.
        fc.horizonS = 2.0f;
        VoxelEnv fe(fc);
        std::mt19937 r(3);
        EnvStep fs;
        bool fUnknown = false;
        for (int t = 0; t < fc.maxSteps; ++t) {
            const auto& m = fe.actionMask();
            std::vector<int> legal;
            for (size_t i = 0; i < m.size(); ++i) if (m[i]) legal.push_back((int)i);
            fs = fe.step(legal.empty() ? 0 : legal[r() % legal.size()]);
            const auto& fo = fe.observation();
            const int FF = VoxelEnv::obsFeaturesPerPrim();
            for (int i = 0; i < fe.nPrims(); ++i)
                if (fo[size_t(i) * FF + 7] > 0.5f) fUnknown = true;
            if (fs.done || fs.truncated) break;
        }
        std::printf("  forest: %d steps, travelled %.1f m\n", fs.steps, fs.travelM);
        check("forest also runs and moves", fs.steps > 0 && fs.travelM > 0.5f,
              f2(fs.travelM) + " m");
        // THE CHANNEL THAT MUST EXIST. In open forest a rollout runs past what
        // has been mapped, so it stops on UNKNOWN rather than on a surface --
        // not blocked, unexplored, and those want opposite responses. A single
        // scalar per direction cannot say it, which is the defect that makes a
        // learned planner unsafe on real sensors.
        check("the UNKNOWN channel fires in the forest", fUnknown,
              "rollouts ending in unmapped space");
    }

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
