#include "mavlink_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstring>
#include <string>

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

constexpr uint32_t kNoMode = 0xFFFFFFFF;

// Every ArduPilot parameter that decides what one of our commands DOES, or
// whether it is listened to at all. Read once at link-up (serviceParams).
constexpr const char* kParamNames[] = {
    "SYSID_MYGCS", "MAV_GCS_SYSID", "RC_OVERRIDE_TIME",
    "RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_THROTTLE", "RCMAP_YAW",
    "RC1_MIN", "RC1_TRIM", "RC1_MAX", "RC1_DZ", "RC1_REVERSED",
    "RC2_MIN", "RC2_TRIM", "RC2_MAX", "RC2_DZ", "RC2_REVERSED",
    "RC3_MIN", "RC3_TRIM", "RC3_MAX", "RC3_DZ", "RC3_REVERSED",
    "RC4_MIN", "RC4_TRIM", "RC4_MAX", "RC4_DZ", "RC4_REVERSED",
    "LOIT_SPEED", "PILOT_SPEED_UP", "PILOT_SPEED_DN", "THR_DZ",
    "PILOT_Y_RATE", "PILOT_Y_EXPO", "ACRO_YAW_P", "ANGLE_MAX",
    "WPNAV_SPEED", "WPNAV_SPEED_UP", "WPNAV_SPEED_DN", "GUID_OPTIONS",
};

const char* copterModeName(uint32_t m) {
    switch (m) {
    case mav::COPTER_STABILIZE:    return "STABILIZE";
    case mav::COPTER_ACRO:         return "ACRO";
    case mav::COPTER_ALT_HOLD:     return "ALT_HOLD";
    case mav::COPTER_AUTO:         return "AUTO";
    case mav::COPTER_GUIDED:       return "GUIDED";
    case mav::COPTER_LOITER:       return "LOITER";
    case mav::COPTER_RTL:          return "RTL";
    case mav::COPTER_LAND:         return "LAND";
    case mav::COPTER_POSHOLD:      return "POSHOLD";
    case mav::COPTER_BRAKE:        return "BRAKE";
    case mav::COPTER_GUIDED_NOGPS: return "GUIDED_NOGPS";
    case mav::COPTER_SMART_RTL:    return "SMART_RTL";
    case mav::COPTER_FLOWHOLD:     return "FLOWHOLD";
    case kNoMode:                  return "(no heartbeat yet)";
    default:                       return "other";
    }
}
}  // namespace

MavlinkBackend::MavlinkBackend() {
    static_assert(sizeof(kParamNames) / sizeof(kParamNames[0]) == kNParams,
                  "kNParams must match the parameter table");
    for (int i = 0; i < kNParams; ++i) params_[i] = {kParamNames[i], 0.f, false, 0};
}

bool MavlinkBackend::parseUplink(const std::string& s, Uplink& out) {
    if (s == "auto")     { out = Uplink::AUTO;            return true; }
    if (s == "rc")       { out = Uplink::RC_OVERRIDE;     return true; }
    if (s == "attitude") { out = Uplink::ATTITUDE_TARGET; return true; }
    if (s == "velocity") { out = Uplink::VELOCITY;        return true; }
    return false;
}

const char* MavlinkBackend::uplinkName(Uplink u) {
    switch (u) {
    case Uplink::AUTO:            return "auto";
    case Uplink::RC_OVERRIDE:     return "rc";
    case Uplink::ATTITUDE_TARGET: return "attitude";
    case Uplink::VELOCITY:        return "velocity";
    }
    return "?";
}

bool MavlinkBackend::param(const char* name, float& out) const {
    for (const Param& q : params_)
        if (q.have && std::strcmp(q.name, name) == 0) { out = q.v; return true; }
    return false;
}

float MavlinkBackend::pv(const char* name, float def) const {
    float v;
    return param(name, v) ? v : def;
}

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
        { mav::MSG_LOCAL_POSITION_NED,   10 },   // leg odometry (flow-aided EKF3)
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

    serviceParams(t);
    // Say which uplink is live whenever the pilot's switch changes it: the
    // single most useful line on a field day.
    const char* path = controlPath();     // string literals: compare by address
    if (linkUp_ && (copterMode_ != reportedMode_ || path != reportedPath_)) {
        reportedMode_ = copterMode_;
        reportedPath_ = path;
        std::printf("[mavlink] FC in %s -> control: %s\n",
                    copterModeName(copterMode_), path);
    }
}

// PARAMETER READBACK. PARAM_REQUEST_READ by name for every entry still
// unanswered, every half second, until all have answered or the timeout says
// the rest do not exist on this firmware (MAV_GCS_SYSID is 4.5+,
// PILOT_Y_RATE 4.3+). Control is held off until then.
void MavlinkBackend::serviceParams(double now) {
    if (paramsDone_ || !linkUp_) return;
    if (!paramsStarted_) { paramsStarted_ = true; paramsT0_ = now; }
    bool all = true;
    for (const Param& q : params_) all = all && q.have;
    if (all || now - paramsT0_ >= paramTimeoutS_) {
        paramsDone_ = true;
        std::printf("%s", paramReport().c_str());
        return;
    }
    if (now - lastParamReqS_ < 0.5) return;
    lastParamReqS_ = now;
    for (Param& q : params_) {
        if (q.have) continue;
        mav::Payload p;
        p.i16(-1);                                 // by name, not index
        p.u8(tgtSys_); p.u8(tgtComp_);
        char id[16]{};
        std::strncpy(id, q.name, sizeof(id));      // 16 bytes, NUL only if shorter
        for (char c : id) p.u8(uint8_t(c));
        send(mav::MSG_PARAM_REQUEST_READ, p);
        ++q.tries;
    }
}

void MavlinkBackend::onParamValue(const mav::Msg& m) {
    char id[17]{};
    for (int i = 0; i < 16; ++i) id[i] = char(m.u8(8 + i));
    for (Param& q : params_)
        if (std::strcmp(q.name, id) == 0) { q.v = m.f32(0); q.have = true; }
}

bool MavlinkBackend::rcMapOk() const {
    // A remapped stick would put our roll on someone else's channel. Refuse
    // rather than remap: the release-to-pilot rule assumes 1-4 are ours.
    return pv("RCMAP_ROLL", 1) == 1 && pv("RCMAP_PITCH", 2) == 2 &&
           pv("RCMAP_THROTTLE", 3) == 3 && pv("RCMAP_YAW", 4) == 4;
}

std::string MavlinkBackend::paramReport() const {
    std::string r;
    char b[256];
    auto line = [&](const char* fmt, auto... a) {
        if constexpr (sizeof...(a) == 0) {
            r += fmt;
        } else {
            std::snprintf(b, sizeof(b), fmt, a...);
            r += b;
        }
    };
    int have = 0;
    for (const Param& q : params_) have += q.have;
    line("[mavlink] ArduPilot parameters: %d of %d answered\n", have, kNParams);

    float v;
    const float our = float(codec_.sysid());
    if (param("MAV_GCS_SYSID", v) || param("SYSID_MYGCS", v)) {
        if (v != our)
            line("!! GCS sysid is %.0f, ours is %.0f: ArduPilot will IGNORE every "
                 "override and mode change. Set %s = %.0f.\n", double(v), double(our),
                 param("MAV_GCS_SYSID", v) ? "MAV_GCS_SYSID" : "SYSID_MYGCS", double(our));
        else
            line("  GCS sysid %.0f matches ours: overrides and mode changes accepted\n",
                 double(our));
    } else {
        line("!! could not read SYSID_MYGCS / MAV_GCS_SYSID: is anything answering?\n");
    }
    if (param("RC_OVERRIDE_TIME", v))
        line("  RC_OVERRIDE_TIME %.1f s%s\n", double(v),
             v < 0 ? "  !! never times out: a stalled Pi leaves its sticks in" : "");
    if (!rcMapOk())
        line("!! RCMAP is not roll/pitch/throttle/yaw on 1-4: the stick uplink is REFUSED\n");

    const StickScale& k = scale_;
    line("  a full stick here means %.1f m/s, %.1f m/s climb, %.0f deg/s yaw "
         "(fc.stick_mps / stick_climb_mps / stick_yaw_dps)\n",
         double(k.mps), double(k.climbMps), double(k.yawDps));
    if (param("LOIT_SPEED", v))
        line("  LOITER: full stick = LOIT_SPEED %.1f m/s -> horizontal rescaled x%.2f%s\n",
             double(v / 100.f), double(k.mps / std::max(0.01f, v / 100.f)),
             k.mps > v / 100.f ? "  !! slower than stick_mps: capped" : "");
    line("  ALT_HOLD / POSHOLD / FLOWHOLD / GUIDED_NOGPS: horizontal stick is a LEAN "
         "ANGLE, not a speed -- uncalibrated\n");
    float up, dz;
    if (param("PILOT_SPEED_UP", up) && param("THR_DZ", dz)) {
        const float dn = pv("PILOT_SPEED_DN", 0.f) > 0.f ? pv("PILOT_SPEED_DN", 0.f) : up;
        line("  climb: PILOT_SPEED_UP %.1f / DN %.1f m/s, THR_DZ %.0f -> deadband "
             "skipped, rescaled to %.1f m/s%s\n", double(up / 100.f), double(dn / 100.f),
             double(dz), double(k.climbMps),
             k.climbMps > std::min(up, dn) / 100.f ? "  !! faster than the FC allows: capped" : "");
    } else {
        line("!! PILOT_SPEED_UP / THR_DZ unknown: throttle sent unscaled\n");
    }
    float yr;
    if (param("PILOT_Y_RATE", yr) || (param("ACRO_YAW_P", yr) && (yr *= 45.f, true)))
        line("  yaw: full stick = %.0f deg/s -> rescaled x%.2f%s\n", double(yr),
             double(k.yawDps / std::max(1.f, yr)),
             pv("PILOT_Y_EXPO", 0.f) != 0.f ? "  !! PILOT_Y_EXPO is not 0: rate is not linear" : "");
    else
        line("!! yaw rate unknown: yaw sent unscaled\n");
    if (param("WPNAV_SPEED_UP", v))
        line("  GUIDED_NOGPS climb: WPNAV_SPEED_UP %.1f / DN %.1f m/s\n", double(v / 100.f),
             double(pv("WPNAV_SPEED_DN", 150.f) / 100.f));
    if (int(pv("GUID_OPTIONS", 0.f)) & 8)
        line("!! GUID_OPTIONS bit 3 set: attitude thrust is raw thrust, not a climb rate -- "
             "the attitude uplink is REFUSED\n");
    if (param("WPNAV_SPEED", v) && k.mps > v / 100.f)
        line("!! GUIDED caps speed at WPNAV_SPEED %.1f m/s, below stick_mps\n", double(v / 100.f));
    return r;
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
    case mav::MSG_PARAM_VALUE:
        if (m.sysid == tgtSys_) onParamValue(m);
        break;
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
        // KEEP [0,360). Every consumer in this tree and the MSP backend agree on
        // it; "improving" it to (-180,180] would rotate the world silently for
        // anything that compares headings without wrapping.
        tel_.yawDeg   = m.f32(12) * 180.f / kPi;
        if (tel_.yawDeg < 0.f) tel_.yawDeg += 360.f;
        // Rates come free in the same frame and are worth keeping: a large rate
        // at a small angle is a disturbance, not a commanded manoeuvre.
        tel_.rollRateDps  = m.f32(16) * 180.f / kPi;
        tel_.pitchRateDps = m.f32(20) * 180.f / kPi;
        tel_.yawRateDps   = m.f32(24) * 180.f / kPi;
        tel_.attFresh     = true;
        break;
    case mav::MSG_EKF_STATUS_REPORT:
        // Floats first, flags last -- MAVLink v2 orders fields by descending
        // size, so `flags` is the uint16 at offset 20, not at offset 0.
        tel_.ekfVelVar      = m.f32(0);
        tel_.ekfPosHorizVar = m.f32(4);
        tel_.ekfPosVertVar  = m.f32(8);
        tel_.ekfCompassVar  = m.f32(12);
        tel_.ekfFlags       = m.u16(20);
        tel_.ekfValid       = true;
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
    case mav::MSG_LOCAL_POSITION_NED:
        // time_boot_ms, then x y z vx vy vz -- all 4 bytes, so wire order is
        // declaration order. Pinned against a pymavlink golden frame.
        tel_.localN  = m.f32(4);
        tel_.localE  = m.f32(8);
        tel_.localD  = m.f32(12);
        tel_.localVn = m.f32(16);
        tel_.localVe = m.f32(20);
        tel_.localStampS = nowS();
        tel_.localValid  = true;
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
    // Around hover-hold, both ways: below mid is a descent rate in ALT_HOLD /
    // LOITER (ControlCmd::throttle).
    v = std::max(-1.f, std::min(1.f, v));
    return uint16_t(std::max(1000, std::min(2000, 1500 + int(v * 500.f))));
}
uint16_t MavlinkBackend::addDelta(uint16_t base, float v) {
    return uint16_t(std::max(1000, std::min(2000, int(base) + int(v * 500.f))));
}

// The inverse of ArduPilot's RC_Channel::pwm_to_angle_dz_trim: the pulse it
// reads as `frac` of full deflection, past its deadzone, on its own calibrated
// range. With nothing read this is 1500 +- 500, the old mapping.
uint16_t MavlinkBackend::angleUs(int ch, float frac) const {
    const char* n[4][5] = {
        {"RC1_MIN", "RC1_TRIM", "RC1_MAX", "RC1_DZ", "RC1_REVERSED"},
        {"RC2_MIN", "RC2_TRIM", "RC2_MAX", "RC2_DZ", "RC2_REVERSED"},
        {"RC3_MIN", "RC3_TRIM", "RC3_MAX", "RC3_DZ", "RC3_REVERSED"},
        {"RC4_MIN", "RC4_TRIM", "RC4_MAX", "RC4_DZ", "RC4_REVERSED"}};
    frac = std::max(-1.f, std::min(1.f, frac));
    if (pv(n[ch][4], 0.f) != 0.f) frac = -frac;
    const float lo = pv(n[ch][0], 1000.f), trim = pv(n[ch][1], 1500.f);
    const float hi = pv(n[ch][2], 2000.f), dz = pv(n[ch][3], 0.f);
    float us = trim;
    if (frac > 0.f)      us = trim + dz + frac * (hi - trim - dz);
    else if (frac < 0.f) us = trim - dz + frac * (trim - dz - lo);
    return uint16_t(std::lround(std::max(lo, std::min(hi, us))));
}

// A climb rate as a fraction of the FC's own full-stick rate, up or down.
float MavlinkBackend::climbFrac(float climbMps, float upMps, float dnMps) const {
    if (climbMps > 0.f) return  std::min(1.f, climbMps / std::max(0.01f, upMps));
    if (climbMps < 0.f) return -std::min(1.f, -climbMps / std::max(0.01f, dnMps));
    return 0.f;
}

// The inverse of ArduCopter's get_pilot_desired_climb_rate: the throttle pulse
// that ALT_HOLD / LOITER / POSHOLD read as `climbMps`. Its deadband (THR_DZ,
// +-100 of 1000 by default) is jumped, not fallen into -- a small correction
// used to do nothing at all -- and full stick is PILOT_SPEED_UP / _DN, not
// whatever this program assumed.
uint16_t MavlinkBackend::throttleUs(float climbMps) const {
    float up, dz;
    if (!param("PILOT_SPEED_UP", up) || !param("THR_DZ", dz))
        return thrToUs(climbMps / std::max(0.01f, scale_.climbMps));   // unknown: as before
    up /= 100.f;
    float dn = pv("PILOT_SPEED_DN", 0.f) / 100.f;
    if (dn <= 0.f) dn = up;
    dz = std::max(0.f, std::min(400.f, dz));
    // Throttle is a RANGE channel: control 0..1000 from (MIN + RC3_DZ)..MAX.
    const float lo = pv("RC3_MIN", 1000.f) + pv("RC3_DZ", 0.f), hi = pv("RC3_MAX", 2000.f);
    const float span = std::max(1.f, hi - lo);
    const float mid = 1000.f * ((pv("RC3_MIN", 1000.f) + hi) * 0.5f - lo) / span;
    const float f = climbFrac(climbMps, up, dn);
    float c = mid;
    if (f > 0.f)      c = mid + dz + f * (1000.f - (mid + dz));
    else if (f < 0.f) c = (mid - dz) * (1.f + f);
    float us = lo + c * span / 1000.f;
    if (pv("RC3_REVERSED", 0.f) != 0.f) us = pv("RC3_MIN", 1000.f) + hi - us;
    return uint16_t(std::lround(std::max(lo - pv("RC3_DZ", 0.f), std::min(hi, us))));
}

void MavlinkBackend::latchBaseline() {
    if (tel_.rcCount < 4) { baselineValid_ = false; return; }
    for (int i = 0; i < 8 && i < tel_.rcCount; ++i) baseline_[i] = tel_.rc[i];
    baselineValid_ = true;
}

// SET_ATTITUDE_TARGET (82). The ArduPilot-shaped uplink: hand the autopilot an
// attitude to hold and let its own rate loops fly it.
//
// It is the GUIDED_NOGPS path (pathFor): GUIDED velocity setpoints need a
// horizontal position/velocity estimate, and GNSS-denied without external nav
// EKF3 has IMU and baro only -- enough for attitude, not for velocity. With VIO
// fed in as ExternalNav, GUIDED and the velocity path become available.
//
// YAW IS COMMANDED ABSOLUTELY, as current heading plus the requested increment,
// which is why this depends on the ATTITUDE decoder above. Without a fresh
// attitude we do not know what "turn 10 degrees right" means in the frame the
// autopilot uses, and guessing would be a silent 180 in the worst case -- so
// this REFUSES rather than assumes.
bool MavlinkBackend::sendAttitudeTarget(const ControlCmd& cmd) {
    if (!tel_.attFresh) return false;      // no heading, no absolute yaw target

    const float kDeg = 3.14159265358979f / 180.f;
    const float roll  = cmd.roll  * maxTiltDeg_ * kDeg;
    const float pitch = cmd.pitch * maxTiltDeg_ * kDeg;
    // Body +pitch in ControlCmd is "forward"; a multirotor pitches NOSE DOWN to
    // go forward, and ArduPilot's pitch is positive nose-UP. Hence the sign.
    const float pitchCmd = -pitch;
    const float yaw = (tel_.yawDeg + cmd.yaw * maxTiltDeg_) * kDeg;

    // ZYX (yaw-pitch-roll) to quaternion, w first, as MAVLink wants.
    const float cr = std::cos(roll*0.5f),  sr = std::sin(roll*0.5f);
    const float cp = std::cos(pitchCmd*0.5f), sp = std::sin(pitchCmd*0.5f);
    const float cy = std::cos(yaw*0.5f),   sy = std::sin(yaw*0.5f);
    const float q0 = cr*cp*cy + sr*sp*sy;
    const float q1 = sr*cp*cy - cr*sp*sy;
    const float q2 = cr*sp*cy + sr*cp*sy;
    const float q3 = cr*cp*sy - sr*sp*cy;

    // ControlCmd.throttle is -1..1 around hover (0) -- the same convention the
    // MSP path uses when it writes 1500 us for 0. SET_ATTITUDE_TARGET's thrust
    // is 0..1 with 0.5 as hover, so the mapping is a half-scale offset, not an
    // identity. Getting this wrong is a climb or a drop, not a wobble.
    //
    // And it is a CLIMB RATE, not thrust: ArduPilot reads (thrust - 0.5) x 2 as
    // a fraction of WPNAV_SPEED_UP above 0.5 and of WPNAV_SPEED_DN below
    // (unless GUID_OPTIONS bit 3, which pathFor refuses). So rescale through
    // those, exactly as the stick path does through PILOT_SPEED_UP/DN.
    float thrust = 0.5f + std::max(-1.f, std::min(1.f, cmd.throttle)) * 0.5f;
    float wup;
    if (param("WPNAV_SPEED_UP", wup))
        thrust = 0.5f + 0.5f * climbFrac(std::max(-1.f, std::min(1.f, cmd.throttle)) * scale_.climbMps,
                                         wup / 100.f, pv("WPNAV_SPEED_DN", 150.f) / 100.f);
    thrust = std::max(0.f, std::min(1.f, thrust));

    mav::Payload p;
    p.u32(uint32_t(nowS() * 1000.0));
    p.f32(q0); p.f32(q1); p.f32(q2); p.f32(q3);
    p.f32(0.f); p.f32(0.f); p.f32(0.f);        // body rates, ignored by the mask
    p.f32(thrust);
    p.u8(tgtSys_); p.u8(tgtComp_);
    // Bits 0-2 set = ignore the three body rates; attitude + thrust are used.
    p.u8(0x07);
    send(mav::MSG_SET_ATTITUDE_TARGET, p);
    return true;
}

bool MavlinkBackend::sendObstacleDistance(const float* distM, int n,
                                          float minM, float maxM) {
    if (!serial_.isOpen() || !distM || n <= 0) return false;

    mav::Payload p;
    p.u64(uint64_t(nowS() * 1e6));
    // CENTIMETRES on the wire, and 65535 means "no reading on this bearing".
    // Writing max_distance instead of the sentinel would tell ArduPilot the
    // path is CLEAR where we simply have not looked -- the same unknown-is-not-
    // free rule the near map enforces, applied to a different consumer.
    const int kBins = 72;
    for (int i = 0; i < kBins; ++i) {
        uint16_t cm = 65535;
        if (i < n) {
            const float d = distM[i];
            if (d > 0.f && std::isfinite(d))
                cm = uint16_t(std::max(1.f, std::min(65534.f, d * 100.f)));
        }
        p.u16(cm);
    }
    p.u16(uint16_t(minM * 100.f));
    p.u16(uint16_t(maxM * 100.f));
    p.u8(0);                     // MAV_DISTANCE_SENSOR_LASER
    p.u8(0);                     // increment: 0 = use increment_f below
    p.f32(360.f / float(kBins)); // increment_f, degrees per bin
    // ANGLE_OFFSET 0, NOT -180. BearingField::obstacleDistance returns bins
    // clockwise FROM THE NOSE -- its own test pins bin 0 ahead and bin 36
    // astern. Declaring -180 here would rotate every bearing by half a turn and
    // ArduPilot would steer AWAY from clear air and INTO the obstacle. This is
    // the single most dangerous constant in the file.
    p.f32(0.f);                  // angle_offset: bin 0 is dead ahead
    p.u8(12);                    // MAV_FRAME_BODY_FRD -- bearings are OURS
    send(mav::MSG_OBSTACLE_DISTANCE, p);
    return true;
}

bool MavlinkBackend::holdsHeight(uint32_t m) {
    return m == mav::COPTER_ALT_HOLD || m == mav::COPTER_LOITER ||
           m == mav::COPTER_POSHOLD  || m == mav::COPTER_FLOWHOLD;
}

MavlinkBackend::Path MavlinkBackend::pathFor(uint32_t m) const {
    const bool guided = m == mav::COPTER_GUIDED, nogps = m == mav::COPTER_GUIDED_NOGPS;
    // Thrust-as-thrust turns the attitude uplink's climb rate into raw
    // collective, where 0.5 is not a hover: refuse it outright.
    const bool attOk = !(int(pv("GUID_OPTIONS", 0.f)) & 8);
    switch (uplink_) {
    case Uplink::RC_OVERRIDE:     return holdsHeight(m) && rcMapOk() ? Path::RC : Path::NONE;
    case Uplink::ATTITUDE_TARGET: return (guided || nogps) && attOk ? Path::ATTITUDE : Path::NONE;
    case Uplink::VELOCITY:        return guided ? Path::VELOCITY : Path::NONE;
    case Uplink::AUTO:
        if (guided) return Path::VELOCITY;
        if (nogps)  return attOk ? Path::ATTITUDE : Path::NONE;
        return holdsHeight(m) && rcMapOk() ? Path::RC : Path::NONE;
    }
    return Path::NONE;
}

const char* MavlinkBackend::controlPath() const {
    if (!paramsDone_) return "none (reading parameters)";
    if (assist_) return "sticks (assist)";
    switch (pathFor(copterMode_)) {
    case Path::RC:       return copterMode_ == mav::COPTER_LOITER
                                ? "sticks (speed-calibrated)" : "sticks (climb-calibrated; lean angle)";
    case Path::ATTITUDE: return "attitude target";
    case Path::VELOCITY: return "velocity setpoint";
    case Path::NONE:     break;
    }
    return "none (not a mode this program flies in)";
}

const char* MavlinkBackend::controlTag() const {
    if (!paramsDone_) return "PAR";
    if (assist_) return "AST";
    switch (pathFor(copterMode_)) {
    case Path::RC:       return "RC";
    case Path::ATTITUDE: return "ATT";
    case Path::VELOCITY: return "VEL";
    case Path::NONE:     break;
    }
    return "OFF";
}

bool MavlinkBackend::sendStatusText(const char* text) {
    if (!serial_.isOpen() || !text) return false;
    mav::Payload p;
    p.u8(6);                                       // MAV_SEVERITY_INFO
    char t[50]{};
    std::strncpy(t, text, sizeof(t));              // 50 bytes, NUL only if shorter
    for (char c : t) p.u8(uint8_t(c));
    send(mav::MSG_STATUSTEXT, p);
    return true;
}

bool MavlinkBackend::sendControl(const ControlCmd& cmd) {
    if (!serial_.isOpen() || !cmd.valid) return false;
    // Nothing until ArduPilot's own numbers are known (or known absent): the
    // first frame is calibrated or there is no first frame.
    if (!paramsDone_) return false;
    // Assist trims the pilot's own sticks, so it goes the stick way in any mode.
    if (assist_) return sendRc(cmd, copterMode_);
    switch (pathFor(copterMode_)) {
    case Path::RC:       return sendRc(cmd, copterMode_);
    case Path::ATTITUDE: releaseRc(); return sendAttitudeTarget(cmd);
    case Path::VELOCITY: releaseRc(); return sendVelocity(cmd);
    case Path::NONE:     break;
    }
    // STABILIZE, ACRO, RTL, LAND...: mid stick is not "hold", so a stick
    // command would mean something else. Send nothing, and hand back any
    // override now rather than after RC_OVERRIDE_TIME.
    releaseRc();
    return false;
}

void MavlinkBackend::releaseRc() {
    if (!rcActive_) return;
    mav::Payload p;
    for (int i = 0; i < 8; ++i) p.u16(0);         // 0 = release to the receiver
    p.u8(tgtSys_); p.u8(tgtComp_);
    for (int i = 0; i < 10; ++i) p.u16(0);
    send(mav::MSG_RC_CHANNELS_OVERRIDE, p);
    rcActive_ = false;
}

bool MavlinkBackend::sendRc(const ControlCmd& cmd, uint32_t mode) {
    uint16_t ch[8]{};
    // PITCH IS REVERSED ON ARDUPILOT: stick forward is LOW pulse (its own
    // autotest flies north with RC2 at 1300). iNAV is the other way round, and
    // this used to copy iNAV -- so "forward" flew backwards.
    if (assist_) {
        if (!baselineValid_) return false;    // no baseline, no trim: refuse
        ch[0] = addDelta(baseline_[0], cmd.roll);
        ch[1] = addDelta(baseline_[1], -cmd.pitch);
        // A throttle delta is a climb rate only where mid stick holds height;
        // anywhere else it would be raw collective on top of the pilot's.
        ch[2] = holdsHeight(mode) ? addDelta(baseline_[2], cmd.throttle) : baseline_[2];
        ch[3] = addDelta(baseline_[3], cmd.yaw);
        for (int i = 4; i < 8; ++i) ch[i] = baseline_[i];
    } else {
        // RATES, through ArduPilot's own scaling (paramReport says which).
        // LOITER: steady speed is stick x LOIT_SPEED (AC_Loiter's drag term
        // makes it so); in the other height-hold modes the horizontal stick is
        // a lean angle and there is nothing to calibrate it against.
        float h = 1.f, loit;
        if (mode == mav::COPTER_LOITER && param("LOIT_SPEED", loit) && loit > 1.f)
            h = scale_.mps / (loit / 100.f);
        float y = 1.f, yr;
        if (param("PILOT_Y_RATE", yr) && yr > 1.f)       y = scale_.yawDps / yr;
        else if (param("ACRO_YAW_P", yr) && yr > 0.01f)  y = scale_.yawDps / (yr * 45.f);
        ch[0] = angleUs(0, cmd.roll * h);
        ch[1] = angleUs(1, -cmd.pitch * h);
        ch[2] = throttleUs(std::max(-1.f, std::min(1.f, cmd.throttle)) * scale_.climbMps);
        ch[3] = angleUs(3, cmd.yaw * y);
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
    rcActive_ = true;
    return true;
}

// GUIDED: say the speed. ControlCmd is a rate command by definition
// (StickScale), so this is a multiplication, and nothing about ArduPilot's
// stick handling -- deadbands, expo, LOIT_SPEED -- is in the way.
bool MavlinkBackend::sendVelocity(const ControlCmd& cmd) {
    auto c = [](float v) { return std::max(-1.f, std::min(1.f, v)); };
    return sendVelocityBody(c(cmd.pitch) * scale_.mps, c(cmd.roll) * scale_.mps,
                            -c(cmd.throttle) * scale_.climbMps,
                            c(cmd.yaw) * scale_.yawDps * kPi / 180.f);
}

bool MavlinkBackend::sendVelocityBody(float vFwd, float vRight, float vDown,
                                      float yawRateRadS) {
    if (!serial_.isOpen()) return false;
    // type_mask 0x05C7: ignore position (bits 0-2), acceleration (6-8) and
    // absolute yaw (10); USE velocity (3-5) and yaw rate (11 CLEAR). It was
    // 0x0DC7, whose bit 11 is YAW_RATE_IGNORE: every turn silently dropped.
    constexpr uint16_t kVelYawRate = 0x05C7;
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
                                    float rollRad, float pitchRad, float yawRad,
                                    int resetCounter) {
    mav::Payload p;
    p.u64(uint64_t(nowS() * 1e6));
    p.f32(xN); p.f32(yE); p.f32(zD);
    p.f32(rollRad); p.f32(pitchRad); p.f32(yawRad);
    if (resetCounter >= 0) {
        p.f32(std::numeric_limits<float>::quiet_NaN());   // covariance: unknown
        for (int i = 1; i < 21; ++i) p.f32(0.f);
        p.u8(uint8_t(resetCounter & 0xFF));
    }
    send(mav::MSG_VISION_POSITION_ESTIMATE, p);
}

bool MavlinkBackend::sendVisionOdometry(const VisionOdom& v) {
    if (!serial_.isOpen()) return false;
    constexpr float kD2R = float(kPi / 180.0);
    feedVisionPose(v.n, v.e, -v.u, v.rollDeg * kD2R, v.pitchDeg * kD2R, v.yawDeg * kD2R,
                   v.resets);
    feedVisionSpeed(v.vn, v.ve, -v.vu);
    return true;
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
    if (m == FcMode::RESUME) {
        // Back to what the aircraft was flying before our RTL/LAND -- but only
        // if it is still where we put it (or already back). If the pilot has
        // since flicked it somewhere else, that was their decision; leave it.
        if (resumeMode_ == kNoMode || specialMode_ == kNoMode) return false;
        const bool ours = copterMode_ == specialMode_ || copterMode_ == resumeMode_;
        cm = resumeMode_;
        resumeMode_ = specialMode_ = kNoMode;
        if (!ours) return false;
        std::printf("[mavlink] resuming %s\n", copterModeName(cm));
        return commandLong(mav::CMD_DO_SET_MODE,
                           float(mav::MODE_FLAG_CUSTOM_MODE_ENABLED), float(cm));
    }
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
    if (cm == mav::COPTER_RTL || cm == mav::COPTER_LAND) {
        // Remember what to RESUME: the mode before the first of these, so
        // RTL -> LAND -> clear still goes back to the pilot's own.
        if (specialMode_ == kNoMode && copterMode_ != mav::COPTER_RTL &&
            copterMode_ != mav::COPTER_LAND)
            resumeMode_ = copterMode_;
        specialMode_ = cm;
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
