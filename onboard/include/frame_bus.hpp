#pragma once

#include <cstdint>
#include <mutex>

#include <opencv2/core.hpp>

// Thread-safe hand-off of the latest camera frame from the fast "fly" loop to
// the slow "think" thread. The fly loop publish()es every frame; the think
// thread grabs the newest one whenever it's ready. It never blocks the fly loop,
// and it's fine (intended) for the think thread to skip frames — it only ever
// wants the most recent one, not every one.
class FrameBus {
public:
    void publish(const cv::Mat& f) {
        std::lock_guard<std::mutex> lk(m_);
        f.copyTo(latest_);        // deep copy: the think thread gets its own buffer
        ++seq_;
    }

    // Copy the newest frame into `out`. Returns false if nothing has been
    // published yet, or — when `lastSeq` is given — if no NEW frame has arrived
    // since the caller last saw it (so the think thread doesn't reprocess).
    bool latest(cv::Mat& out, uint64_t* lastSeq = nullptr) {
        std::lock_guard<std::mutex> lk(m_);
        if (latest_.empty()) return false;
        if (lastSeq && *lastSeq == seq_) return false;
        latest_.copyTo(out);
        if (lastSeq) *lastSeq = seq_;
        return true;
    }

private:
    std::mutex m_;
    cv::Mat    latest_;
    uint64_t   seq_ = 0;
};
