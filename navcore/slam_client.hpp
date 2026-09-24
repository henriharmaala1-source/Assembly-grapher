#pragma once
// ---------------------------------------------------------------------------
// SlamClient -- onboard's side of SlamLink (slam_link.hpp).
//
// Non-blocking for the caller: submit() stores the frame and returns; an I/O
// thread sends the NEWEST frame whenever the SLAM process is idle and collects
// its reply. A frame replaced before it was sent is dropped -- its IMU samples
// are not: they are prepended to the next one, because stereo-inertial SLAM
// integrates every sample between the frames it sees.
//
// Connection loss is survivable: the thread reconnects every second, and the
// caller sees `connected() == false` and no new poses meanwhile -- the estimate
// then goes stale and the consumers treat it as absent.
// ---------------------------------------------------------------------------

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "slam_link.hpp"

class SlamClient {
public:
    explicit SlamClient(std::string socketPath);
    ~SlamClient();
    SlamClient(const SlamClient&) = delete;
    SlamClient& operator=(const SlamClient&) = delete;

    // Queue a frame (copied). Returns its sequence number.
    uint32_t submit(const slamlink::FrameHeader& h, const uint8_t* left,
                    const uint8_t* right, const std::vector<slamlink::ImuSample>& imu);

    // The newest reply not yet taken. False if there is none.
    bool takeReply(slamlink::PoseReply& out);

    bool connected() const { return connected_.load(); }
    long framesSent() const { return sent_.load(); }
    long framesDropped() const { return dropped_.load(); }

private:
    void loop();

    std::string path_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool       have_ = false;              // a frame waiting to be sent
    slamlink::FrameHeader hdr_;
    std::vector<uint8_t> left_, right_;
    std::vector<slamlink::ImuSample> imu_;
    bool       haveReply_ = false;
    slamlink::PoseReply reply_;
    uint32_t   seq_ = 0;
    std::atomic<bool> run_{true}, connected_{false};
    std::atomic<long> sent_{0}, dropped_{0};
    std::thread thr_;
};
