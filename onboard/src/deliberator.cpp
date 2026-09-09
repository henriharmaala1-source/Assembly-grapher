#include "deliberator.hpp"

#include <chrono>

void Deliberator::start(FrameBus& bus, WorldModel& wm) {
    if (run_.load()) return;
    run_.store(true);
    thr_ = std::thread(&Deliberator::loop_, this, &bus, &wm);
}

void Deliberator::stop() {
    run_.store(false);
    if (thr_.joinable()) thr_.join();
}

void Deliberator::loop_(FrameBus* bus, WorldModel* wm) {
    cv::Mat  frame;
    uint64_t lastSeq = 0;
    long     frameId = 0;
    while (run_.load()) {
        lastTickS_.store(monoNowS());   // liveness stamp — every pass, even idle
        // Take only the newest frame; skip if nothing new yet (don't busy-spin).
        if (!bus->latest(frame, &lastSeq) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        const Behavior beh = wm->snapshot().behavior;   // hot-module cadence input
        sched_.tick(frame, *wm, ++frameId, beh);         // writes results into wm
        frames_.fetch_add(1);
    }
}
