// The BASELINE side of the comparison, as a standalone binary.
//
//   ./rl_bench --worlds forest maze --seeds 101 108
//
// WHY THIS EXISTS AND WHY IT IS NOT sweep.sh. A learned policy chooses among
// primitives inside VoxelEnv, with VoxelEnv's reward, VoxelEnv's episode
// termination and VoxelEnv's step timing. Comparing it against numbers produced
// by voxel_sim's own loop would compare two harnesses as much as two planners.
//
// So every baseline here is run through the SAME VoxelEnv, and differs from the
// policy only in how it picks an index:
//
//   random   uniform over the ADMISSIBLE set -- the floor. A policy that cannot
//            beat this has learned nothing, and that is cheaper to discover
//            before a training run than after one.
//   freeM    the longest confirmed-free rollout. A pure-safety greedy.
//   goal     the admissible primitive whose endpoint points nearest the goal.
//            A pure-progress greedy, and the one that walks into dead ends.
//   score    the classical TrajectoryPlanner's own weighted argmax, which is
//            what --traj actually flies.
//
// The last is the number that matters: it is the shipped planner, scored on the
// policy's own terms. Everything a learned policy wins has to be won against it.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "rl_env.hpp"

using namespace sim;

namespace {

enum class Policy { Random, FreeM, Goal, Score };

const char* policyName(Policy p) {
    switch (p) {
        case Policy::Random: return "random";
        case Policy::FreeM:  return "freeM";
        case Policy::Goal:   return "goal";
        default:             return "score";
    }
}

// Pick an index from the observation alone -- exactly the information a learned
// policy gets, so no baseline is quietly advantaged by seeing more.
int choose(Policy pol, const std::vector<float>& obs,
           const std::vector<uint8_t>& mask, int nPrims, std::mt19937& rng) {
    const int F = VoxelEnv::obsFeaturesPerPrim();
    std::vector<int> legal;
    for (int i = 0; i < nPrims; ++i) if (mask[i]) legal.push_back(i);
    if (legal.empty()) return 0;                     // hold

    if (pol == Policy::Random)
        return legal[rng() % legal.size()];

    int best = legal[0];
    float bestV = -1e30f;
    for (int i : legal) {
        const float* o = &obs[size_t(i) * F];
        float v = 0.f;
        if (pol == Policy::FreeM)      v = o[0];               // confirmed-free
        else if (pol == Policy::Goal)  v = -o[3];              // goal error
        else {
            // The shipped planner's own terms, on the same normalised features.
            v = 0.7f * o[1] - 1.0f * o[3] - 0.25f * std::fabs(o[6])
              + 0.5f * o[2] - 2.0f * std::max(0.f, o[4]);
        }
        if (v > bestV) { bestV = v; best = i; }
    }
    return best;
}

struct Row {
    std::string world; int seed; std::string outcome;
    float travel, endDist, minClear; int stopped, steps;
};

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> worlds = {"forest", "maze"};
    std::vector<int> seeds;
    int maxSteps = 1500;
    bool stereo = false;

    for (int i = 1; i < argc; ++i) {
        auto next = [&](const char* d) { return (i + 1 < argc) ? argv[++i] : d; };
        if (!std::strcmp(argv[i], "--worlds")) {
            worlds.clear();
            while (i + 1 < argc && argv[i + 1][0] != '-') worlds.push_back(argv[++i]);
        } else if (!std::strcmp(argv[i], "--seeds")) {
            const int a = std::atoi(next("101")), b = std::atoi(next("108"));
            for (int s = a; s <= b; ++s) seeds.push_back(s);
        } else if (!std::strcmp(argv[i], "--steps")) maxSteps = std::atoi(next("1500"));
        else if (!std::strcmp(argv[i], "--stereo")) stereo = true;
        else if (!std::strcmp(argv[i], "--help")) {
            std::printf("rl_bench --worlds forest maze --seeds 101 108 "
                        "[--steps N] [--stereo]\n");
            return 0;
        }
    }
    if (seeds.empty()) for (int s = 101; s <= 108; ++s) seeds.push_back(s);

    std::printf("baselines through VoxelEnv -- the same harness the policy uses\n");
    std::printf("%-8s %-8s %-5s %-16s %9s %9s %9s %8s\n",
                "policy", "world", "seed", "outcome", "travel", "end-dist",
                "minClr", "stopped");

    for (Policy pol : {Policy::Random, Policy::FreeM, Policy::Goal, Policy::Score}) {
        std::vector<Row> rows;
        for (const std::string& w : worlds) {
            for (int s : seeds) {
                EnvConfig c;
                c.world = w; c.seed = unsigned(s); c.maxSteps = maxSteps;
                c.truthDepth = !stereo;
                // Per-world horizon: the maze wants ~0.6 s and the forest ~2.0,
                // because the optimum tracks the size of the space. One value
                // for both handicaps whichever it does not suit.
                c.horizonS = (w == "maze") ? 0.6f : 2.0f;
                VoxelEnv env(c);
                std::mt19937 rng(unsigned(s) * 7919u + unsigned(pol));

                EnvStep st;
                for (int t = 0; t < maxSteps; ++t) {
                    st = env.step(choose(pol, env.observation(), env.actionMask(),
                                         env.nPrims(), rng));
                    if (st.done || st.truncated) break;
                }
                const char* outcome = st.reachedGoal ? "reached goal"
                                    : st.collisions  ? "COLLIDED"
                                                     : "ran out of steps";
                rows.push_back({w, s, outcome, st.travelM, st.distToGoalM,
                                st.minClearM, st.stoppedSteps, st.steps});
                std::printf("%-8s %-8s %-5d %-16s %9.1f %9.1f %9.2f %8d\n",
                            policyName(pol), w.c_str(), s, outcome, st.travelM,
                            st.distToGoalM, st.minClearM, st.stoppedSteps);
                std::fflush(stdout);
            }
        }
        int coll = 0, reach = 0; double sum = 0;
        for (const Row& r : rows) {
            coll  += (r.outcome == "COLLIDED");
            reach += (r.outcome == "reached goal");
            sum += r.travel;
        }
        std::printf("  -> %-8s runs %zu  collisions %d  goals %d  mean travel %.1f m\n\n",
                    policyName(pol), rows.size(), coll, reach, sum / std::max<size_t>(1, rows.size()));
    }
    return 0;
}
