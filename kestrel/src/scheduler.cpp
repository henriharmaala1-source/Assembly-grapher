#include "scheduler.hpp"

bool PerceptionScheduler::maybeRun(Slot& s, const cv::Mat& frame, WorldModel& wm,
                                   long frameId, float& budget, int interval) {
    if (!s.mod->isReady()) return false;
    if (frameId - s.lastRun < interval) return false;
    if (s.mod->costMs() > budget) return false;   // no room this tick

    s.mod->run(frame, wm);
    s.lastRun = frameId;
    budget -= s.mod->costMs();
    lastRan_.push_back(s.mod->name());
    return true;
}

void PerceptionScheduler::tick(const cv::Mat& frame, WorldModel& wm,
                               long frameId, Behavior beh) {
    lastRan_.clear();
    float budget = budgetMs_;

    // 1. Always-on cheap modules (the tracker) — run unconditionally.
    for (auto& s : slots_) {
        if (!s.alwaysOn) continue;
        s.mod->run(frame, wm);
        s.lastRun = frameId;
        budget -= s.mod->costMs();
        lastRan_.push_back(s.mod->name());
    }

    // 2. Heavy modules that are HOT for the active behaviour — first dibs.
    for (auto& s : slots_) {
        if (s.alwaysOn || s.hotFor != beh) continue;
        maybeRun(s, frame, wm, frameId, budget, s.hotInterval);
    }

    // 3. Remaining heavy modules at their cold cadence, budget permitting.
    for (auto& s : slots_) {
        if (s.alwaysOn || s.hotFor == beh) continue;
        maybeRun(s, frame, wm, frameId, budget, s.baseInterval);
    }
}
