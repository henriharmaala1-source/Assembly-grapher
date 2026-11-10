#pragma once

#include <string>
#include <vector>

#include "perception.hpp"
#include "world_model.hpp"

// Allocates the per-frame CPU budget across perception modules.
//
// On a CPU-only Pi 5 you cannot run depth + detection every frame. Each heavy
// module is given a cadence (run every N frames) that tightens when the module
// is "hot" for the active behaviour, and the per-tick millisecond budget stops
// two heavy models firing on the same frame. Cheap always-on modules (the
// tracker) run unconditionally.
class PerceptionScheduler {
public:
    struct Slot {
        IPerceptionModule* mod;
        bool      alwaysOn     = false;          // run every tick (cheap)
        int       baseInterval = 12;             // frames between runs when cold
        int       hotInterval  = 3;              // frames between runs when hot
        Behavior  hotFor       = Behavior::IDLE; // behaviour that heats this up
        long      lastRun      = -1;
    };

    void add(const Slot& s)       { slots_.push_back(s); }
    void setBudgetMs(float ms)    { budgetMs_ = ms; }

    // Run the due modules for this frame, hot modules first, under budget.
    void tick(const cv::Mat& frame, WorldModel& wm, long frameId, Behavior beh);

    const std::vector<std::string>& lastRan() const { return lastRan_; }

private:
    bool maybeRun(Slot& s, const cv::Mat& frame, WorldModel& wm,
                  long frameId, float& budget, int interval);

    std::vector<Slot>        slots_;
    float                    budgetMs_ = 60.f;   // CPU-only Pi 5: generous
    std::vector<std::string> lastRan_;
};
