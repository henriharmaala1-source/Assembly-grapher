#pragma once

#include <memory>
#include <string>
#include <vector>

#include "occupancy_grid.hpp"

namespace navsim {

struct Vec2 { float e = 0, n = 0; };

struct PlanResult {
    std::vector<Vec2> path;    // world points, path.front() ~ start, back ~ goal
    bool   ok    = false;      // a route to (near) the goal was found
    double planMs = 0.0;       // wall-clock planning time this call
    int    expanded = 0;       // nodes expanded (search effort, for comparison)
};

// A path planner over the occupancy grid. Given a start and goal in world ENU
// metres and the robot inflation radius (in cells), produce a path (or a single
// reactive step, for the potential-field method). Replanned every tick against
// the latest partially-observed grid.
class IPlanner {
public:
    virtual ~IPlanner() = default;
    virtual const char* name() const = 0;
    virtual PlanResult  plan(const OccupancyGrid& g, Vec2 start, Vec2 goal,
                             int inflateCells) = 0;
    // Called once at the start of each run — stateful reactive planners (bug2)
    // clear their per-episode memory here. Stateless planners ignore it.
    virtual void reset() {}
};

// Factory: name -> planner. Names: wavefront, dijkstra, astar, potential, rrt.
std::unique_ptr<IPlanner> makePlanner(const std::string& name);
std::vector<std::string>  plannerNames();

}  // namespace navsim
