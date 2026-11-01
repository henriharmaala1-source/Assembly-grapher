#pragma once
// ---------------------------------------------------------------------------
// EgoPlanner -- an EGO-Planner-style local planner, as a bench baseline.
//
// EGO-Planner (Zhou, Wang, Gao, Xu, "EGO-Planner: An ESDF-free Gradient-based
// Local Planner for Quadrotors", RA-L 2021) is what the common ROS forest
// stacks pair with RTAB-Map. It is here so that "would smooth trajectory
// optimisation fly safer than picking primitives?" is a MEASURED row in
// `kestrel bench` and not an opinion. This is a re-implementation of its
// method, not its code, so a result here is about the method:
//
//   1. a uniform cubic B-spline from the aircraft to a LOCAL TARGET a fixed
//      horizon along the line to the waypoint, initialised as a straight line
//      whether or not that line runs through something;
//   2. every control point inside an (inflated) obstacle starts a collision
//      segment; an A* path round it gives, per control point, an anchor p on
//      the obstacle surface and a direction v out of it -- no distance field;
//   3. the free control points are optimised (L-BFGS) for smoothness (jerk),
//      collision (penalty on d = (Q - p).v falling below a safe distance),
//      feasibility (velocity, acceleration) and reaching the target;
//   4. "rebound": control points that are newly in collision after the
//      optimisation get their own {p, v} and it runs again.
//
// UNKNOWN IS FREE TO IT, by default, because that is what EGO-Planner's map
// does -- a grid cell that has never been observed is not an obstacle. This
// tree's rule is the opposite (CLAUDE.md, "Unknown is not free"), which is
// exactly why it is worth measuring rather than assuming: `unknownFree =
// false` flips it.
//
// In the bench it ADVISES, like every other planner: VoxelEnv picks the
// admissible primitive whose rollout lies closest to the optimised
// trajectory, so the geometric veto still stands between it and a collision,
// and the comparison is planner against planner through one harness.
// ---------------------------------------------------------------------------

#include <array>
#include <vector>

namespace sim {

class VoxelMap;

struct EgoParams {
    float planHorizonM = 7.5f;   // local target distance (EGO's planning_horizon)
    float ctrlDistM    = 0.4f;   // control-point spacing (EGO's ctrl_pt_dist)
    float maxVel       = 2.5f;   // m/s
    float maxAcc       = 3.0f;   // m/s^2
    float robotR       = 0.6f;   // body radius; obstacles are inflated by
    float safeDistM    = 0.3f;   //   robotR - safeDistM, and the collision cost
                                 //   wants safeDistM beyond that
    float wSmooth = 1.0f, wColl = 10.0f, wFeas = 0.01f, wGoal = 1.0f;
    bool  unknownFree  = true;   // EGO-Planner's own convention; see above
    int   rebounds     = 3;
    int   maxIters     = 200;    // L-BFGS per optimisation
    float localHalfM   = 8.0f;   // the local grid around the aircraft
    float localHalfZM  = 2.5f;
    int   astarMaxExpand = 30000;
};

class EgoPlanner {
public:
    using V3 = std::array<float, 3>;
    explicit EgoPlanner(const EgoParams& p = EgoParams()) : p_(p) {}
    const EgoParams& params() const { return p_; }

    // Plan from position `pos` with velocity `vel` toward `waypoint` (world
    // ENU metres). False if no trajectory could be made (the start is boxed in,
    // or the optimiser failed) -- valid() then stays false.
    bool plan(const VoxelMap& m, const V3& pos, const V3& vel, const V3& waypoint);

    bool  valid() const { return valid_; }
    float duration() const;              // seconds
    V3    sample(float t) const;         // clamped to [0, duration]
    const std::vector<V3>& controlPoints() const { return q_; }

    // What the last plan did, for the bench to report.
    struct Stats {
        int plans = 0, failures = 0, astarFails = 0, rebounds = 0, iters = 0, reallocations = 0;
        int collidingAfter = 0;          // control points still inside after optimising
    };
    const Stats& stats() const { return st_; }
    void resetStats() { st_ = Stats(); }

private:
    struct Pair { V3 p, v; };
    // local occupancy (inflated), indexed like the map's own cells
    void buildGrid(const VoxelMap& m, const V3& centre);
    bool occupied(const V3& w) const;
    bool occCell(int x, int y, int z) const;
    bool astar(const V3& a, const V3& b, std::vector<V3>& path);
    int  addPairs();                     // returns how many control points got new pairs
    double cost(const std::vector<double>& x, std::vector<double>& g) const;
    bool optimise();

    EgoParams p_;
    std::vector<V3> q_;
    std::vector<std::vector<Pair>> pairs_;
    float ts_ = 0.16f;
    V3 goal_{0, 0, 0};
    bool valid_ = false;
    Stats st_;

    // grid
    float cell_ = 0.25f;
    int gx0_ = 0, gy0_ = 0, gz0_ = 0, gnx_ = 0, gny_ = 0, gnz_ = 0;
    std::vector<unsigned char> occ_;     // inflated
    const VoxelMap* map_ = nullptr;
};

}  // namespace sim
