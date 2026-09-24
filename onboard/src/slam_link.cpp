#include "slam_link.hpp"

#include <cerrno>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace slamlink {

bool writeAll(int fd, const void* p, size_t n) {
    const uint8_t* b = static_cast<const uint8_t*>(p);
    while (n > 0) {
        const ssize_t k = ::send(fd, b, n, MSG_NOSIGNAL);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) return false;
        b += k; n -= size_t(k);
    }
    return true;
}

bool readAll(int fd, void* p, size_t n) {
    uint8_t* b = static_cast<uint8_t*>(p);
    while (n > 0) {
        const ssize_t k = ::recv(fd, b, n, 0);
        if (k < 0 && errno == EINTR) continue;
        if (k <= 0) return false;
        b += k; n -= size_t(k);
    }
    return true;
}

bool sendFrame(int fd, const FrameHeader& h, const uint8_t* left, const uint8_t* right,
               const ImuSample* imu) {
    const size_t px = size_t(h.width) * size_t(h.height);
    return writeAll(fd, &h, sizeof h) && writeAll(fd, left, px) && writeAll(fd, right, px) &&
           (h.nImu <= 0 || writeAll(fd, imu, sizeof(ImuSample) * size_t(h.nImu)));
}

bool recvFrame(int fd, FrameHeader& h, std::vector<uint8_t>& left,
               std::vector<uint8_t>& right, std::vector<ImuSample>& imu) {
    if (!readAll(fd, &h, sizeof h)) return false;
    // A peer speaking something else, or a corrupt stream: stop, do not guess.
    if (h.magic != kMagicFrame || h.version != kVersion) return false;
    if (h.width <= 0 || h.height <= 0 || h.width > 4096 || h.height > 4096) return false;
    if (h.nImu < 0 || h.nImu > 100000) return false;
    const size_t px = size_t(h.width) * size_t(h.height);
    left.resize(px); right.resize(px); imu.resize(size_t(h.nImu));
    return readAll(fd, left.data(), px) && readAll(fd, right.data(), px) &&
           (h.nImu == 0 || readAll(fd, imu.data(), sizeof(ImuSample) * size_t(h.nImu)));
}

bool sendPose(int fd, const PoseReply& r) { return writeAll(fd, &r, sizeof r); }

bool recvPose(int fd, PoseReply& r) {
    return readAll(fd, &r, sizeof r) && r.magic == kMagicPose && r.version == kVersion;
}

namespace {
bool fillAddr(const std::string& path, sockaddr_un& a, std::string* err) {
    std::memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    if (path.size() >= sizeof a.sun_path) {
        if (err) *err = "socket path too long: " + path;
        return false;
    }
    std::memcpy(a.sun_path, path.c_str(), path.size());
    return true;
}
}  // namespace

int listenUnix(const std::string& path, std::string* err) {
    sockaddr_un a;
    if (!fillAddr(path, a, err)) return -1;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { if (err) *err = "socket failed"; return -1; }
    ::unlink(path.c_str());
    if (::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 || ::listen(fd, 1) != 0) {
        if (err) *err = "cannot listen on " + path;
        ::close(fd);
        return -1;
    }
    return fd;
}

int connectUnix(const std::string& path, std::string* err) {
    sockaddr_un a;
    if (!fillAddr(path, a, err)) return -1;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { if (err) *err = "socket failed"; return -1; }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0) {
        if (err) *err = "cannot connect to " + path;
        ::close(fd);
        return -1;
    }
    return fd;
}

}  // namespace slamlink
