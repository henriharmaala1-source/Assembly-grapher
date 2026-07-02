#include "msp_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr float kPi = 3.14159265358979323846f;

inline uint16_t rdU16(const std::vector<uint8_t>& p, size_t o) {
    return (uint16_t)(p[o] | (p[o + 1] << 8));
}
inline int16_t rdS16(const std::vector<uint8_t>& p, size_t o) {
    return (int16_t)(p[o] | (p[o + 1] << 8));
}
inline int32_t rdS32(const std::vector<uint8_t>& p, size_t o) {
    return (int32_t)(p[o] | (p[o + 1] << 8) | (p[o + 2] << 16) | (p[o + 3] << 24));
}

// CRC8 / DVB-S2 (poly 0xD5, init 0, no reflection) — iNAV MSP v2 checksum.
inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
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

void MspBackend::sendV2(uint16_t function, const uint8_t* payload, uint16_t size) {
    std::vector<uint8_t> f;
    f.reserve(9 + size);
    f.push_back('$'); f.push_back('X'); f.push_back('<');
    uint8_t crc = 0;
    auto put = [&](uint8_t b) { f.push_back(b); crc = crc8_dvb_s2(crc, b); };
    put(0);                                          // flags
    put(function & 0xFF); put((function >> 8) & 0xFF);
    put(size & 0xFF);     put((size >> 8) & 0xFF);
    for (uint16_t i = 0; i < size; ++i) put(payload[i]);
    f.push_back(crc);
    serial_.write(f.data(), (int)f.size());
}

bool MspBackend::feedExternalGps(const ExtGps& fix) {
    if (!connected_) return false;

    std::vector<uint8_t> p;
    p.reserve(52);
    auto u8  = [&](uint8_t v)  { p.push_back(v); };
    auto u16 = [&](uint16_t v) { p.push_back(v & 0xFF); p.push_back((v >> 8) & 0xFF); };
    auto u32 = [&](uint32_t v) { for (int i = 0; i < 4; ++i) p.push_back((v >> (8 * i)) & 0xFF); };
    auto i32 = [&](int32_t v)  { u32((uint32_t)v); };

    // MSP2_SENSOR_GPS layout (iNAV io/gps_msp.c). All little-endian.
    u8(0);                                              // instance
    u16(0);                                             // gpsWeek (nav ignores)
    u32(0);                                             // msTOW
    u8((uint8_t)fix.fixType);                           // 3 = 3D
    u8((uint8_t)fix.sats);                              // satellitesInView
    u16((uint16_t)std::min(65535.f, fix.ephM * 100.f)); // horizontalPosAccuracy cm
    u16((uint16_t)std::min(65535.f, fix.epvM * 100.f)); // verticalPosAccuracy cm
    u16(50);                                            // horizontalVelAccuracy cm/s
    u16((uint16_t)std::min(9999.f, fix.hdop * 100.f));  // hdop ×100
    i32((int32_t)std::llround(fix.lon * 1e7));          // longitude deg×1e7
    i32((int32_t)std::llround(fix.lat * 1e7));          // latitude  deg×1e7
    i32((int32_t)std::lround(fix.altMslM * 100.f));     // mslAltitude cm
    i32((int32_t)std::lround(fix.velN * 100.f));        // nedVelNorth cm/s
    i32((int32_t)std::lround(fix.velE * 100.f));        // nedVelEast  cm/s
    i32((int32_t)std::lround(fix.velD * 100.f));        // nedVelDown  cm/s

    float course = fix.yawDeg;                          // ground course deg×10
    if (course < 0.f) { course = std::atan2(fix.velE, fix.velN) * 180.f / kPi; }
    if (course < 0.f) course += 360.f;
    u16((uint16_t)std::lround(course * 10.f));          // groundCourse
    u16(0xFFFF);                                        // trueYaw = invalid → FC keeps AHRS heading
    u16(0); u8(0); u8(0); u8(0); u8(0); u8(0);          // year/month/day/hour/min/sec (unused)

    sendV2(MSP2_SENSOR_GPS, p.data(), (uint16_t)p.size());   // fire-and-forget, no ACK
    return true;
}

uint16_t MspBackend::addDelta(uint16_t base, float v) {
    int us = (int)base + (int)(v * 500.f);              // ±1.0 → ±500 µs trim
    return (uint16_t)std::max(1000, std::min(2000, us));
}

void MspBackend::latchBaseline() {
    // Capture the operator's current sticks (read back via MSP_RC) as the assist
    // baseline. One-shot at engagement, so it reflects pure operator input and
    // doesn't drift by re-reading our own override frames.
    if (rcCount_ >= 4) {
        for (int i = 0; i < 8; ++i)
            baseline_[i] = (i < rcCount_) ? rc_[i] : (i == 2 ? 1500 : (i < 4 ? 1500 : 1000));
        baselineValid_ = true;
    } else {
        baselineValid_ = false;   // no operator RC seen → assist falls back to total
    }
}

bool MspBackend::setMode(FcMode m) {
    if (m == FcMode::RTL) {
        if (rthAuxIdx_ < 0) return false;   // unconfigured → caller falls back
        rthActive_ = true;
        return true;
    }
    rthActive_ = false;                     // any other mode releases the latch
    return true;
}

bool MspBackend::sendControl(const ControlCmd& cmd) {
    if (!connected_ || !cmd.valid) return false;
    // AETR: [Roll, Pitch, Throttle, Yaw, AUX1..AUX4]. Throttle is index 2.
    uint16_t ch[8];
    if (assist_ && baselineValid_) {
        // Flight assist: trim relative to the operator's latched sticks, so the
        // takeover starts from where their hands are (bumpless), not from neutral.
        ch[0] = addDelta(baseline_[0], cmd.roll);
        ch[1] = addDelta(baseline_[1], cmd.pitch);
        ch[2] = addDelta(baseline_[2], cmd.throttle);
        ch[3] = addDelta(baseline_[3], cmd.yaw);
        for (int i = 4; i < 8; ++i) ch[i] = baseline_[i];   // operator AUX passes through
    } else {
        // Total autonomy: absolute sticks from neutral; the OS is the source.
        ch[0] = axisToUs(cmd.roll);
        ch[1] = axisToUs(cmd.pitch);
        ch[2] = thrToUs(cmd.throttle);
        ch[3] = axisToUs(cmd.yaw);
        for (int i = 4; i < 8; ++i) ch[i] = baselineValid_ ? baseline_[i] : 1000;
    }
    // Failsafe RTH overlay: drive the configured AUX channel high so iNAV enters
    // NAV RTH. Sticks are left as commanded (the caller sends neutral); the arm
    // channel is untouched so the aircraft stays armed for the return.
    if (rthActive_ && rthAuxIdx_ >= 4 && rthAuxIdx_ < 8)
        ch[rthAuxIdx_] = (uint16_t)std::max(1000, std::min(2000, rthAuxUs_));

    uint8_t payload[16];
    for (int i = 0; i < 8; ++i) {
        payload[i * 2]     = (uint8_t)(ch[i] & 0xFF);
        payload[i * 2 + 1] = (uint8_t)(ch[i] >> 8);
    }
    sendV1(MSP_SET_RAW_RC, payload, sizeof(payload));
    return true;
}

void MspBackend::requestNextTelemetry() {
    // ATTITUDE in every other slot (~25 Hz at the 50 Hz poll cadence): roll/
    // pitch feed the depth-corridor de-rotation, which is off by a whole
    // maneuver if attitude is 200 ms stale. Everything else interleaves in the
    // odd slots — RC and GPS twice per cycle (assist latch / estimator),
    // altitude/status/analog once (~3.5 Hz each).
    static const uint8_t others[] = {MSP_RC, MSP_RAW_GPS, MSP_ALTITUDE, MSP_RC,
                                     MSP_RAW_GPS, MSP_STATUS, MSP_ANALOG};
    if (pollIdx_ % 2 == 0) sendV1(MSP_ATTITUDE, nullptr, 0);
    else                   sendV1(others[(pollIdx_ / 2) % 7], nullptr, 0);
    ++pollIdx_;
}

void MspBackend::tick() {
    if (!connected_) return;
    drainRx();
    // Poll telemetry at ~50 Hz, ATTITUDE-priority interleave (see above).
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
        case MSP_RC:                                      // operator/current RC, µs
            rcCount_ = std::min((int)(p.size() / 2), 18);
            for (int i = 0; i < rcCount_; ++i) rc_[i] = rdU16(p, i * 2);
            break;
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
                tel_.groundspeedMs = rdU16(p, 12) / 100.f;        // cm/s → m/s
                tel_.groundCourseDeg = rdU16(p, 14) / 10.f;       // decideg → deg
            }
            break;
        case MSP_ALTITUDE:
            if (p.size() >= 4)
                tel_.baroAltM = rdS32(p, 0) / 100.f;              // estimated alt, cm → m
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
    tel_.linkUp  = linkUp();
    tel_.rcCount = rcCount_;
    for (int i = 0; i < rcCount_ && i < 18; ++i) tel_.rc[i] = rc_[i];
    out = tel_;
    return tel_.linkUp;
}
