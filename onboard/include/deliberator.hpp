#pragma once

#include <atomic>
#include <thread>

#include "frame_bus.hpp"
#include "scheduler.hpp"
#include "world_model.hpp"

// The deliberative ("think") tier, on its OWN thread — separated from the fast
// "fly" loop so slow thinking can never stall control.
//
// It owns the HEAVY perception (depth model, detector, and later SLAM + the
// planner): it pulls the newest frame from the FrameBus, ticks its scheduler,
// and writes results into the thread-safe WorldModel. The fly loop reads those
// results whenever it wants and is never blocked waiting for them — if a SLAM /
// plan update takes a second, the fly loop just keeps flying on the last result.
//
// This is the reactive/deliberative split the OS was designed around, made real:
//   fly loop  (main thread) : capture, FC I/O, cheap reactive perception, FSM,
//                             controller, control output — guaranteed fast.
//   think tier (this thread): depth / detection / SLAM / planning — best-effort.
class Deliberator {
public:
    ~Deliberator() { stop(); }     // never leave a joinable thread dangling

    // Add heavy modules via the scheduler (same Slot API as before).
    PerceptionScheduler& scheduler() { return sched_; }

    void start(FrameBus& bus, WorldModel& wm);
    void stop();
    bool running()          const { return run_.load(); }
    long framesProcessed()  const { return frames_.load(); }

private:
    void loop_(FrameBus* bus, WorldModel* wm);

    PerceptionScheduler sched_;
    std::thread         thr_;
    std::atomic<bool>   run_{false};
    std::atomic<long>   frames_{0};
};
