#include "slam_client.hpp"

#include <chrono>

SlamClient::SlamClient(std::string socketPath) : path_(std::move(socketPath)) {
    thr_ = std::thread([this] { loop(); });
}

SlamClient::~SlamClient() {
    run_.store(false);
    cv_.notify_all();
    if (thr_.joinable()) thr_.join();
}

uint32_t SlamClient::submit(const slamlink::FrameHeader& h, const uint8_t* left,
                            const uint8_t* right,
                            const std::vector<slamlink::ImuSample>& imu) {
    std::lock_guard<std::mutex> lk(mu_);
    if (have_) dropped_.fetch_add(1);      // replaced before it was sent
    else imu_.clear();                     // the last send took them
    hdr_ = h;
    hdr_.seq = ++seq_;
    const size_t px = size_t(h.width) * size_t(h.height);
    left_.assign(left, left + px);
    right_.assign(right, right + px);
    imu_.insert(imu_.end(), imu.begin(), imu.end());   // carried, never dropped
    have_ = true;
    cv_.notify_all();
    return hdr_.seq;
}

bool SlamClient::takeReply(slamlink::PoseReply& out, long* connection) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!haveReply_) return false;
    out = reply_;
    if (connection) *connection = replyConn_;
    haveReply_ = false;
    return true;
}

void SlamClient::loop() {
    int fd = -1;
    std::vector<uint8_t> l, r;
    std::vector<slamlink::ImuSample> imu;
    while (run_.load()) {
        if (fd < 0) {
            fd = slamlink::connectUnix(path_);
            connected_.store(fd >= 0);
            if (fd >= 0) ++conns_;
            if (fd < 0) {
                for (int i = 0; i < 10 && run_.load(); ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
        }
        slamlink::FrameHeader h;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(200),
                         [this] { return have_ || !run_.load(); });
            if (!have_) continue;
            h = hdr_;
            l.swap(left_); r.swap(right_); imu.swap(imu_);
            imu_.clear();
            have_ = false;
        }
        h.nImu = int32_t(imu.size());
        slamlink::PoseReply rep;
        if (!slamlink::sendFrame(fd, h, l.data(), r.data(), imu.data()) ||
            !slamlink::recvPose(fd, rep)) {
            slamlink::closeFd(fd);
            fd = -1;
            connected_.store(false);
            continue;
        }
        sent_.fetch_add(1);
        std::lock_guard<std::mutex> lk(mu_);
        reply_ = rep;
        replyConn_ = conns_;
        haveReply_ = true;
    }
    if (fd >= 0) slamlink::closeFd(fd);
}
