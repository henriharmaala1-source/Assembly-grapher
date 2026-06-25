#include "msp_backend.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
inline uint16_t rdU16(const std::vector<uint8_t>& p, size_t o) {
    return (uint16_t)(p[o] | (p[o + 1] << 8));
}
inline int16_t rdS16(const std::vector<uint8_t>& p, size_t o) {
    return (int16_t)(p[o] | (p[o + 1] << 8));
}
inline int32_t rdS32(const std::vector<uint8_t>& p, size_t o) {
    return (int32_t)(p[o] | (p[o + 1] << 8) | (p[o + 2] << 16) | (p[o + 3] << 24));
}
}  // namespace

bool MspBackend::connect(const std::string& port, int baud) {
    connected_ = serial_.open(port, baud);
    if (connected_) {
        lastRx_   = clock::now();   // grace period before declaring link down
        lastPoll_ = clock::now();
        std::printf("[msp] connected on %s @ %d\n", port.c_str(), baud);
    }
    return connected_;
}

bool MspBackend::linkUp() const {
    if (!connected_) return false;
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                         clock::now() - lastRx_).count();
    return age < 1000;
}

uint16_t MspBackend::axisToUs(float v) {
    v = std::max(-1.f, std::min(1.f, v));
    int us = 1500 + (int)(v * 500.f);
    return (uint16_t)std::max(1000, std::min(2000, us));
}

uint16_t MspBackend::thrToUs(float v) {
    // Throttle is a climb bias around hover-hold: 0 → mid (FC in ALT_HOLD holds
    // altitude), 1 → full climb. Descent is left to the FC's altitude mode.
    v = std::max(0.f, std::min(1.f, v));
    int us = 1500 + (int)(v * 500.f);
    return (uint16_t)std::max(1000, std::min(2000, us));
}

void MspBackend::sendV1(uint8_t cmd, const uint8_t* payload, uint8_t size) {
    std::vector<uint8_t> f;
    f.reserve(6 + size);
    f.push_back('$'); f.push_back('M'); f.push_back('<');
    f.push_back(size); f.push_back(cmd);
    uint8_t crc = size ^ cmd;
    for (uint8_t i = 0; i < size; ++i) { f.push_back(payload[i]); crc ^= payload[i]; }
    f.push_back(crc);
    serial_.write(f.data(), (int)f.size());
}

bool MspBackend::sendControl(const ControlCmd& cmd) {
    if (!connected_ || !cmd.valid) return false;
    // AETR: [Roll, Pitch, Throttle, Yaw, AUX1..AUX4]. Throttle is index 2.
    uint16_t ch[8] = {
        axisToUs(cmd.roll),       // 0 Aileron / Roll
        axisToUs(cmd.pitch),      // 1 Elevator / Pitch
        thrToUs(cmd.throttle),    // 2 Throttle
        axisToUs(cmd.yaw),        // 3 Rudder / Yaw
        1000, 1000, 1000, 1000,   // AUX (arm left on the radio — not here)
    };
    uint8_t payload[16];
    for (int i = 0; i < 8; ++i) {
        payload[i * 2]     = (uint8_t)(ch[i] & 0xFF);
        payload[i * 2 + 1] = (uint8_t)(ch[i] >> 8);
    }
    sendV1(MSP_SET_RAW_RC, payload, sizeof(payload));
    return true;
}

void MspBackend::requestNextTelemetry() {
    static const uint8_t ids[] = {MSP_STATUS, MSP_ATTITUDE, MSP_RAW_GPS, MSP_ANALOG};
    sendV1(ids[pollIdx_ % 4], nullptr, 0);
    ++pollIdx_;
}

void MspBackend::tick() {
    if (!connected_) return;
    drainRx();
    // Poll telemetry ~50 Hz spread across the 4 messages (~12 Hz each).
    const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(
                           clock::now() - lastPoll_).count();
    if (since >= 20) { requestNextTelemetry(); lastPoll_ = clock::now(); }
}

void MspBackend::drainRx() {
    uint8_t buf[256];
    int n = serial_.read(buf, sizeof(buf));
    for (int i = 0; i < n; ++i) {
        const uint8_t b = buf[i];
        switch (st_) {
            case St::DOLLAR: st_ = (b == '$') ? St::M : St::DOLLAR; break;
            case St::M:      st_ = (b == 'M') ? St::DIR : St::DOLLAR; break;
            case St::DIR:    st_ = (b == '>') ? St::SIZE : St::DOLLAR; break;
            case St::SIZE:
                rxSize_ = b; rxCrc_ = b; rxCount_ = 0; rxBuf_.clear();
                st_ = St::CMD; break;
            case St::CMD:
                rxCmd_ = b; rxCrc_ ^= b;
                st_ = rxSize_ ? St::DATA : St::CRC; break;
            case St::DATA:
                rxBuf_.push_back(b); rxCrc_ ^= b;
                if (++rxCount_ >= rxSize_) st_ = St::CRC;
                break;
            case St::CRC:
                if (b == rxCrc_) { lastRx_ = clock::now(); onMessage(rxCmd_, rxBuf_); }
                st_ = St::DOLLAR; break;
        }
    }
}

void MspBackend::onMessage(uint8_t cmd, const std::vector<uint8_t>& p) {
    switch (cmd) {
        case MSP_STATUS:
            if (p.size() >= 10) {
                const uint32_t flags = (uint32_t)rdS32(p, 6);
                // Box bit positions vary by build; bit0 is ARM on stock layouts.
                tel_.armed = (flags & 0x1) != 0;
            }
            break;
        case MSP_ATTITUDE:
            if (p.size() >= 6) {
                tel_.rollDeg  = rdS16(p, 0) / 10.f;   // decidegrees
                tel_.pitchDeg = rdS16(p, 2) / 10.f;
                tel_.yawDeg   = (float)rdS16(p, 4);   // already degrees
            }
            break;
        case MSP_RAW_GPS:
            if (p.size() >= 16) {
                tel_.fixType       = p[0];
                tel_.sats          = p[1];
                tel_.lat           = rdS32(p, 2) / 1e7;
                tel_.lon           = rdS32(p, 6) / 1e7;
                tel_.altM          = (float)rdU16(p, 10);
                tel_.groundspeedMs = rdU16(p, 12) / 100.f;   // cm/s → m/s
            }
            break;
        case MSP_ANALOG:
            if (p.size() >= 7) {
                tel_.battV = p[0] / 10.f;                     // 0.1V steps
                const int rssi = rdU16(p, 3);                // 0..1023
                // Rough 3S–6S → [0,1]; refine per pack. Used only for failsafe cue.
                tel_.battPct = std::max(0.f, std::min(1.f,
                                  (tel_.battV - 3.3f * 3) / (4.2f * 3 - 3.3f * 3)));
                (void)rssi;
            }
            break;
        default: break;
    }
}

bool MspBackend::poll(FcTelemetry& out) {
    tel_.linkUp = linkUp();
    out = tel_;
    return tel_.linkUp;
}
