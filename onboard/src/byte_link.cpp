#include "byte_link.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

// The only platform-specific lines: headers and one-line socket calls. All the
// logic below is shared, so the Windows build (nav-sim's SITL bridge) runs the
// same code the Pi does.
#if defined(_WIN32)
#  include <winsock2.h>
#  include <ws2tcpip.h>
using SockLen = int;
using NativeSock = SOCKET;
static bool sockInit() { WSADATA w; return WSAStartup(MAKEWORD(2, 2), &w) == 0; }
static void sockClose(long long s) { closesocket(SOCKET(s)); }
static bool sockNonBlocking(long long s) { u_long on = 1; return ioctlsocket(SOCKET(s), FIONBIO, &on) == 0; }
static bool sockWouldBlock() { return WSAGetLastError() == WSAEWOULDBLOCK; }
static const long long kBadSock = (long long)INVALID_SOCKET;
#else
#  include <arpa/inet.h>
#  include <cerrno>
#  include <fcntl.h>
#  include <netdb.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <sys/socket.h>
#  include <unistd.h>
using SockLen = socklen_t;
using NativeSock = int;
static bool sockInit() { return true; }
static void sockClose(long long s) { ::close(int(s)); }
static bool sockNonBlocking(long long s) { return fcntl(int(s), F_SETFL, fcntl(int(s), F_GETFL, 0) | O_NONBLOCK) == 0; }
static bool sockWouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }
static const long long kBadSock = -1;
#endif

namespace {

// "HOST:PORT" -> sockaddr (IPv4). False if it does not resolve.
bool resolve(const std::string& hostPort, sockaddr_in& out) {
    const size_t colon = hostPort.rfind(':');
    if (colon == std::string::npos) return false;
    const std::string host = hostPort.substr(0, colon);
    const std::string port = hostPort.substr(colon + 1);
    addrinfo hints{};
    hints.ai_family = AF_INET;
    addrinfo* res = nullptr;
    if (getaddrinfo(host.empty() ? "127.0.0.1" : host.c_str(), port.c_str(), &hints, &res) != 0 || !res)
        return false;
    std::memcpy(&out, res->ai_addr, sizeof(sockaddr_in));
    freeaddrinfo(res);
    return true;
}

bool startsWith(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

}  // namespace

bool ByteLink::open(const std::string& spec, int baud) {
    close();
    const bool tcp = startsWith(spec, "tcp:");
    const bool udpOut = startsWith(spec, "udpout:");
    const bool udpIn = !udpOut && startsWith(spec, "udp:");
    if (!tcp && !udpOut && !udpIn) {
        if (!serial_.open(spec, baud)) return false;
        kind_ = Kind::SERIAL;
        return true;
    }
    if (!sockInit()) { std::fprintf(stderr, "[link] socket layer failed to start\n"); return false; }

    const std::string rest = spec.substr(spec.find(':') + 1);
    if (tcp) {
        sockaddr_in a{};
        if (!resolve(rest, a)) { std::fprintf(stderr, "[link] cannot resolve %s\n", rest.c_str()); return false; }
        sock_ = (long long)socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == kBadSock) return false;
        // Blocking connect (a SITL on loopback answers at once), then
        // non-blocking for the run.
        if (connect(NativeSock(sock_), (sockaddr*)&a, sizeof(a)) != 0) {
            std::fprintf(stderr, "[link] tcp connect to %s failed\n", rest.c_str());
            sockClose(sock_); sock_ = -1; return false;
        }
        int one = 1;
        setsockopt(NativeSock(sock_), IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        sockNonBlocking(sock_);
        kind_ = Kind::TCP;
        return true;
    }

    sock_ = (long long)socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ == kBadSock) return false;
    if (udpOut) {
        sockaddr_in a{};
        if (!resolve(rest, a)) { sockClose(sock_); sock_ = -1; return false; }
        std::memcpy(peer_, &a, sizeof(a)); peerLen_ = int(sizeof(a));
        kind_ = Kind::UDP_OUT;
    } else {
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        a.sin_port = htons(uint16_t(std::atoi(rest.c_str())));
        if (bind(NativeSock(sock_), (sockaddr*)&a, sizeof(a)) != 0) {
            std::fprintf(stderr, "[link] cannot listen on udp port %s\n", rest.c_str());
            sockClose(sock_); sock_ = -1; return false;
        }
        kind_ = Kind::UDP_IN;          // peer unknown until the first datagram
    }
    sockNonBlocking(sock_);
    return true;
}

void ByteLink::close() {
    serial_.close();
    if (sock_ != -1 && sock_ != kBadSock) sockClose(sock_);
    sock_ = -1;
    peerLen_ = 0;
    kind_ = Kind::NONE;
}

bool ByteLink::isOpen() const {
    return kind_ == Kind::SERIAL ? serial_.isOpen() : kind_ != Kind::NONE;
}

int ByteLink::read(uint8_t* buf, int max) {
    switch (kind_) {
    case Kind::NONE:   return 0;
    case Kind::SERIAL: return serial_.read(buf, max);
    case Kind::TCP: {
        const auto n = recv(NativeSock(sock_), (char*)buf, max, 0);
        if (n > 0) return int(n);
        if (n == 0 || !sockWouldBlock()) close();       // peer gone: a cut wire
        return 0;
    }
    case Kind::UDP_OUT:
    case Kind::UDP_IN: {
        sockaddr_in from{};
        SockLen fl = sizeof(from);
        const auto n = recvfrom(NativeSock(sock_), (char*)buf, max, 0,
                                (sockaddr*)&from, &fl);
        if (n <= 0) return 0;
        if (kind_ == Kind::UDP_IN) { std::memcpy(peer_, &from, sizeof(from)); peerLen_ = int(sizeof(from)); }
        return int(n);
    }
    }
    return 0;
}

bool ByteLink::write(const uint8_t* buf, int len) {
    switch (kind_) {
    case Kind::NONE:   return false;
    case Kind::SERIAL: return serial_.write(buf, len);
    case Kind::TCP:    return send(NativeSock(sock_), (const char*)buf, len, 0) == len;
    case Kind::UDP_OUT:
    case Kind::UDP_IN:
        if (peerLen_ == 0) return false;            // nobody has spoken to us yet
        return sendto(NativeSock(sock_), (const char*)buf, len, 0,
                      (const sockaddr*)peer_, SockLen(peerLen_)) == len;
    }
    return false;
}

const char* ByteLink::kind() const {
    switch (kind_) {
    case Kind::SERIAL:  return "serial";
    case Kind::TCP:     return "tcp";
    case Kind::UDP_OUT: return "udpout";
    case Kind::UDP_IN:  return "udp";
    case Kind::NONE:    break;
    }
    return "closed";
}
