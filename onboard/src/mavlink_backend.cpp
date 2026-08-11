#include "mavlink_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {
constexpr float kPi = 3.14159265358979323846f;

double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Metres per degree of latitude, and of longitude at a given latitude. An
// equirectangular approximation, good to a few centimetres over the kilometre
// or so a local NED frame is meant to cover -- and the EKF only ever sees
// differences from the latched origin, so the datum itself does not matter.
constexpr double kMetresPerDegLat = 111320.0;
}  // namespace

bool MavlinkBackend::connect(const std::string& port, int baud) {
    if (!serial_.open(port, baud)) {
        std::fprintf(stderr, "[mavlink] cannot open %s @ %d\n", port.c_str(), baud);
        return false;
    }
    // NO STARTUP GRACE PERIOD, unlike the MSP backend. There, a grace period is
    // reasonable: MSP is request/response, so nothing arrives until we ask and
    // an immediate "link down" would be a startup race. ArduPilot heartbeats at
    // 1 Hz unprompted, so silence means silence -- and reporting linkUp() before
    // the autopilot has ever spoken would tell main.cpp it is allowed to take
    // control of a serial port with nothing on the other end.
    everRx_ = false;
    lastRxS_ = -1e9;
    lastHbSentS_ = -1e9;
    linkUp_ = false;
    std::printf("[mavlink] connected on %s @ %d, sysid %u -> target %u/%u\n",
                port.c_str(), baud, unsigned(codec_.sysid()), unsigned(tgtSys_),
                unsigned(tgtComp_));
    std::printf("[mavlink] ArduPilot must have SYSID_MYGCS (MAV_GCS_SYSID on 4.5+) "
                "= %u or it will ignore RC override and mode changes\n",
                unsigned(codec_.sysid()));
    sendHeartbeat();
    requestStreams();
    return true;
}

void MavlinkBackend::send(uint32_t msgid, const mav::Payload& p) {
    if (!serial_.isOpen()) return;
    uint8_t buf[300];
    const int n = codec_.frame(msgid, p, buf);
    if (n > 0) serial_.write(buf, n);
}

void MavlinkBackend::sendHeartbeat() {
    // Identifying as ONBOARD_CONTROLLER rather than GCS is the honest label and
    // costs nothing: ArduPilot's GCS failsafe timer is refreshed by a heartbeat
    // arriving on the channel, not by the sender's declared type.
    mav::Payload p;
    p.u32(0);                                     // custom_mode
    p.u8(mav::MAV_TYPE_ONBOARD_CONTROLLER);
    p.u8(mav::MAV_AUTOPILOT_INVALID);
    p.u8(0);                                      // base_mode
    p.u8(mav::MAV_STATE_ACTIVE);
    p.u8(3);                                      // mavlink_version
    send(mav::MSG_HEARTBEAT, p);
}

bool MavlinkBackend::commandLong(uint16_t cmd, float p1, float p2, float p3, float p4,
                                 float p5, float p6, float p7) {
    if (!serial_.isOpen()) return false;
    mav::Payload p;
    p.f32(p1); p.f32(p2); p.f32(p3); p.f32(p4); p.f32(p5); p.f32(p6); p.f32(p7);
    p.u16(cmd); p.u8(tgtSys_); p.u8(tgtComp_); p.u8(0);   // confirmation
    send(mav::MSG_COMMAND_LONG, p);
    return true;
}

void MavlinkBackend::requestStreams() {
    // Ask for exactly what the OS reads, at rates matched to what it does with
    // them, rather than turning on a stream group and taking whatever arrives.
    // On a shared serial link the difference is the whole bandwidth budget.
    struct { uint32_t id; int hz; } want[] = {
        { mav::MSG_ATTITUDE,             50 },   // the control loop's own rate
        { mav::MSG_GLOBAL_POSITION_INT,   5 },
        { mav::MSG_GPS_RAW_INT,           2 },
        { mav::MSG_SYS_STATUS,            2 },   // battery, and the failsafe reads it
        { mav::MSG_RC_CHANNELS,          10 },   // assist-mode baseline comes from here
    };
    for (const auto& w : want)
        commandLong(mav::CMD_SET_MESSAGE_INTERVAL, float(w.id), 1e6f / float(w.hz));
    lastStreamReqS_ = nowS();
}

void MavlinkBackend::tick() {
    drainRx();

    const double t = nowS();
    if (t - lastHbSentS_ >= 1.0) { sendHeartbeat(); lastHbSentS_ = t; }

    // A link that came back after a reboot has forgotten the stream rates, and
    // nothing would ever ask again. Re-request periodically while nothing is
    // arriving; this is idempotent and costs five frames a minute.
    if (!linkUp_ && t - lastStreamReqS_ > 5.0) requestStreams();

    linkUp_ = serial_.isOpen() && everRx_ && (t - lastRxS_) < 2.0;
    tel_.linkUp = linkUp_;
}

void MavlinkBackend::drainRx() {
    if (!serial_.isOpen()) return;
    uint8_t buf[512];
    for (;;) {
        const int n = serial_.read(buf, sizeof(buf));
        if (n <= 0) break;
        mav::Msg m;
        for (int i = 0; i < n; ++i)
            if (codec_.feed(buf[i], m)) { lastRxS_ = nowS(); everRx_ = true; onMessage(m); }
        if (n < int(sizeof(buf))) break;
    }
}

void MavlinkBackend::onMessage(const mav::Msg& m) {
    switch (m.id) {
    case mav::MSG_HEARTBEAT: {
        // Only the autopilot's own heartbeat, not another GCS's. Without this
        // check a ground station on the same link would set our idea of the
        // flight mode, which is exactly the sort of bug that shows up once.
        if (m.sysid != tgtSys_) break;
        copterMode_ = m.u32(0);
        tel_.armed  = (m.u8(6) & mav::MODE_FLAG_SAFETY_ARMED) != 0;
        break;
    }
    case mav::MSG_ATTITUDE:
        tel_.rollDeg  = m.f32(4)  * 180.f / kPi;
        tel_.pitchDeg = m.f32(8)  * 180.f / kPi;
        tel_.yawDeg   = m.f32(12) * 180.f / kPi;
        if (tel_.yawDeg < 0.f) tel_.yawDeg += 360.f;
        break;
    case mav::MSG_SYS_STATUS: {
        const uint16_t mv = m.u16(14);
        if (mv != 0 && mv != 0xFFFF) tel_.battV = float(mv) * 1e-3f;
        // ArduPilot's own battery_remaining is a configured-capacity estimate
        // and reads -1 when it has none. Per-cell voltage is cruder but always
        // available, and the low-battery failsafe must not depend on the pilot
        // having set BATT_CAPACITY.
        const int8_t rem = m.i8(30);
        if (rem >= 0) {
            tel_.battPct = float(rem) * 0.01f;
        } else if (tel_.battV > 0.f) {
            if (battCells_ <= 0) battCells_ = std::max(1, int(tel_.battV / 3.9f + 0.5f));
            const float perCell = tel_.battV / float(battCells_);
            tel_.battPct = std::max(0.f, std::min(1.f, (perCell - 3.3f) / (4.2f - 3.3f)));
        }
        break;
    }
    case mav::MSG_GLOBAL_POSITION_INT:
        tel_.lat      = m.i32(4)  * 1e-7;
        tel_.lon      = m.i32(8)  * 1e-7;
        tel_.altM     = m.i32(12) * 1e-3f;
        tel_.baroAltM = m.i32(16) * 1e-3f;      // relative_alt: what ALT_HOLD holds
        tel_.groundspeedMs = std::hypot(m.i16(20) * 0.01f, m.i16(22) * 0.01f);
        if (m.u16(26) != 0xFFFF) tel_.groundCourseDeg = m.u16(26) * 0.01f;
        break;
    case mav::MSG_GPS_RAW_INT:
        tel_.fixType = m.u8(28);
        tel_.sats    = m.u8(29);
        break;
    case mav::MSG_RC_CHANNELS: {
        // chancount is a uint8 at offset 40, AFTER all eighteen channels -- not
        // a uint16 at 38, which is chan18. The first version of this read
        // chan18 as the count, which on a 16-channel link is 0 (so the assist
        // baseline never latched) and on a link with channel 18 live is a
        // four-digit number silently clamped to 18. Pinned by a golden frame.
        const int count = std::min(18, int(m.u8(40)));
        for (int i = 0; i < count; ++i) tel_.rc[i] = m.u16(4 + 2 * i);
        tel_.rcCount = count;
        break;
    }
    default: break;
    }
}

bool MavlinkBackend::poll(FcTelemetry& out) {
    out = tel_;
    return linkUp_;
}

uint16_t MavlinkBackend::axisToUs(float v) {
    v = std::max(-1.f, std::min(1.f, v));
    return uint16_t(std::max(1000, std::min(2000, 1500 + int(v * 500.f))));
}
uint16_t MavlinkBackend::thrToUs(float v) {
    v = std::max(0.f, std::min(1.f, v));
    return uint16_t(std::max(1000, std::min(2000, 1500 + int(v * 500.f))));
}
uint16_t MavlinkBackend::addDelta(uint16_t base, float v) {
    return uint16_t(std::max(1000, std::min(2000, int(base) + int(v * 500.f))));
}

void MavlinkBackend::latchBaseline() {
    if (tel_.rcCount < 4) { baselineValid_ = false; return; }
    for (int i = 0; i < 8 && i < tel_.rcCount; ++i) baseline_[i] = tel_.rc[i];
    baselineValid_ = true;
}

bool MavlinkBackend::sendControl(const ControlCmd& cmd) {
    if (!serial_.isOpen() || !cmd.valid) return false;

    uint16_t ch[8]{};
    if (assist_) {
        if (!baselineValid_) return false;    // no baseline, no trim: refuse
        ch[0] = addDelta(baseline_[0], cmd.roll);
        ch[1] = addDelta(baseline_[1], cmd.pitch);
        ch[2] = addDelta(baseline_[2], cmd.throttle);
        ch[3] = addDelta(baseline_[3], cmd.yaw);
        for (int i = 4; i < 8; ++i) ch[i] = baseline_[i];
    } else {
        // RCMAP defaults on ArduPilot are roll/pitch/throttle/yaw on 1-4, the
        // same AETR order the MSP backend writes, so one ControlCmd maps
        // identically onto both stacks.
        ch[0] = axisToUs(cmd.roll);
        ch[1] = axisToUs(cmd.pitch);
        ch[2] = thrToUs(cmd.throttle);
        ch[3] = axisToUs(cmd.yaw);
        // 0 means RELEASE this channel back to the receiver, which is what we
        // want for every channel we are not driving -- notably the mode switch,
        // so the pilot can always take the aircraft back by flicking it. Writing
        // a value here instead would be the single most dangerous line in the
        // file.
        for (int i = 4; i < 8; ++i) ch[i] = 0;
    }

    mav::Payload p;
    for (int i = 0; i < 8; ++i) p.u16(ch[i]);
    p.u8(tgtSys_); p.u8(tgtComp_);
    for (int i = 0; i < 10; ++i) p.u16(0);        // chan9..18: untouched
    send(mav::MSG_RC_CHANNELS_OVERRIDE, p);
    return true;
}

bool MavlinkBackend::sendVelocityBody(float vFwd, float vRight, float vDown,
                                      float yawRateRadS) {
    if (!serial_.isOpen()) return false;
    // type_mask 0x0DC7: ignore position (bits 0-2), ignore acceleration
    // (bits 6-8), ignore the force bit (9) and the absolute-yaw bit (10); USE
    // velocity (bits 3-5) and yaw rate (bit 11 clear).
    constexpr uint16_t kVelYawRate = 0x0DC7;
    mav::Payload p;
    p.u32(bootMs_);
    p.f32(0); p.f32(0); p.f32(0);                       // x y z, masked off
    p.f32(vFwd); p.f32(vRight); p.f32(vDown);
    p.f32(0); p.f32(0); p.f32(0);                       // accelerations, masked off
    p.f32(0); p.f32(yawRateRadS);
    p.u16(kVelYawRate);
    p.u8(tgtSys_); p.u8(tgtComp_);
    p.u8(mav::FRAME_BODY_NED);
    send(mav::MSG_SET_POSITION_TARGET_LOCAL_NED, p);
    return true;
}

void MavlinkBackend::feedVisionPose(float xN, float yE, float zD,
                                    float rollRad, float pitchRad, float yawRad) {
    mav::Payload p;
    p.u64(uint64_t(nowS() * 1e6));
    p.f32(xN); p.f32(yE); p.f32(zD);
    p.f32(rollRad); p.f32(pitchRad); p.f32(yawRad);
    send(mav::MSG_VISION_POSITION_ESTIMATE, p);
}

void MavlinkBackend::feedVisionSpeed(float vN, float vE, float vD) {
    mav::Payload p;
    p.u64(uint64_t(nowS() * 1e6));
    p.f32(vN); p.f32(vE); p.f32(vD);
    send(mav::MSG_VISION_SPEED_ESTIMATE, p);
}

bool MavlinkBackend::feedExternalGps(const ExtGps& fix) {
    if (!serial_.isOpen()) return false;
    if (fix.fixType < 3) return false;
    if (!originValid_) {
        originLat_ = fix.lat; originLon_ = fix.lon; originAlt_ = fix.altMslM;
        originValid_ = true;
        std::printf("[mavlink] vision origin latched at %.7f %.7f %.1f m\n",
                    originLat_, originLon_, double(originAlt_));
    }
    const double mPerDegLon =
        kMetresPerDegLat * std::cos(originLat_ * kPi / 180.0);
    const float n = float((fix.lat - originLat_) * kMetresPerDegLat);
    const float e = float((fix.lon - originLon_) * mPerDegLon);
    const float d = -(fix.altMslM - originAlt_);          // NED: down positive
    feedVisionPose(n, e, d, 0.f, 0.f,
                   fix.yawDeg >= 0.f ? fix.yawDeg * kPi / 180.f : 0.f);
    feedVisionSpeed(fix.velN, fix.velE, fix.velD);
    return true;
}

bool MavlinkBackend::setMode(FcMode m) {
    uint32_t cm;
    switch (m) {
    case FcMode::STABILIZE: cm = mav::COPTER_STABILIZE; break;
    case FcMode::ALT_HOLD:  cm = mav::COPTER_ALT_HOLD;  break;
    // OFFBOARD is the PX4 name; on ArduPilot the equivalent is GUIDED, and
    // GUIDED_NOGPS when there is no position estimate to hold. We ask for
    // GUIDED: without a position source the FC will refuse, which is the
    // correct outcome, because a velocity setpoint it cannot close the loop on
    // is not something to silently accept.
    case FcMode::OFFBOARD:  cm = mav::COPTER_GUIDED;    break;
    case FcMode::ANGLE:     cm = mav::COPTER_STABILIZE; break;
    case FcMode::LOITER:    cm = mav::COPTER_LOITER;    break;
    case FcMode::RTL:       cm = mav::COPTER_RTL;       break;
    case FcMode::LAND:      cm = mav::COPTER_LAND;      break;
    default: return false;
    }
    return commandLong(mav::CMD_DO_SET_MODE,
                       float(mav::MODE_FLAG_CUSTOM_MODE_ENABLED), float(cm));
}

bool MavlinkBackend::arm(bool force) {
    // 21196 is ArduPilot's "yes I really mean it" magic, which skips the
    // pre-arm checks. It exists for a reason and is not a convenience: pass
    // force only where a human has already decided.
    return commandLong(mav::CMD_COMPONENT_ARM_DISARM, 1.f, force ? 21196.f : 0.f);
}

bool MavlinkBackend::disarm() {
    return commandLong(mav::CMD_COMPONENT_ARM_DISARM, 0.f, 0.f);
}
