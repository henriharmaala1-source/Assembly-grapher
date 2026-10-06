// MavlinkBackend over a PTY — no hardware. The test process plays the
// autopilot: it owns the master end, the backend opens the slave like a serial
// device.
//
// test_mavlink.cpp proves the CODEC agrees with pymavlink byte for byte. This
// proves the BACKEND uses it correctly, which is a different failure mode and
// the one that actually bit: reading chancount from the wrong offset produced
// perfectly valid frames carrying the wrong meaning, and no amount of golden
// framing would have caught it.
//
// Covers: heartbeat keep-alive, stream requests on connect, telemetry decode
// from real ArduPilot messages, link-up/down tracking, RC override channel
// order and the release-to-pilot rule, assist-mode trim, and mode/arm commands;
// parameter readback and the uplink each flight mode selects, with the stick
// mapping checked against a forward model of ArduCopter's own stick handling.

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mavlink_backend.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

std::vector<uint8_t> drainMaster(int mfd, int waitMs = 40) {
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
    std::vector<uint8_t> out;
    uint8_t buf[1024];
    for (;;) {
        const ssize_t n = read(mfd, buf, sizeof(buf));
        if (n <= 0) break;
        out.insert(out.end(), buf, buf + n);
    }
    return out;
}

// Decode everything the backend sent, using the same codec it used to send.
// Using our own parser here is deliberate and safe: test_mavlink.cpp has
// already pinned that parser against pymavlink, so it is a checked instrument
// rather than a circular one.
std::vector<mav::Msg> decodeAll(const std::vector<uint8_t>& b) {
    mav::Codec c(1, 1);
    std::vector<mav::Msg> out;
    mav::Msg m;
    for (uint8_t byte : b) if (c.feed(byte, m)) out.push_back(m);
    return out;
}

const mav::Msg* findMsg(const std::vector<mav::Msg>& v, uint32_t id) {
    for (const mav::Msg& m : v) if (m.id == id) return &m;
    return nullptr;
}
int countMsg(const std::vector<mav::Msg>& v, uint32_t id) {
    int n = 0;
    for (const mav::Msg& m : v) if (m.id == id) ++n;
    return n;
}

// Build a frame as the autopilot (sysid 1, compid 1) and push it at the backend.
void fcSend(int mfd, uint32_t id, const mav::Payload& p) {
    static mav::Codec fc(1, 1);
    uint8_t buf[300];
    const int n = fc.frame(id, p, buf);
    CHECK(write(mfd, buf, n) == n);
}

void fcHeartbeat(int mfd, uint32_t mode, bool armed) {
    mav::Payload p;
    p.u32(mode);
    p.u8(2);                                     // MAV_TYPE_QUADROTOR
    p.u8(3);                                     // MAV_AUTOPILOT_ARDUPILOTMEGA
    p.u8(armed ? 0x81 : 0x01);
    p.u8(4);
    p.u8(3);
    fcSend(mfd, mav::MSG_HEARTBEAT, p);
}

// ---- the autopilot's side of parameter readback --------------------------
struct PV { const char* n; float v; };

void fcParam(int mfd, const char* name, float v) {
    mav::Payload p;
    p.f32(v); p.u16(39); p.u16(0);
    char id[16]{};
    std::strncpy(id, name, sizeof(id));
    for (char c : id) p.u8(uint8_t(c));
    p.u8(9);                                     // MAV_PARAM_TYPE_REAL32
    fcSend(mfd, mav::MSG_PARAM_VALUE, p);
}

// Answer the backend's PARAM_REQUEST_READs from `table` until it is satisfied.
// Returns how many requests it made.
int serveParams(int mfd, MavlinkBackend& fc, const std::vector<PV>& table) {
    int asked = 0;
    for (int k = 0; k < 200 && !fc.paramsResolved(); ++k) {
        fc.tick();
        for (const mav::Msg& m : decodeAll(drainMaster(mfd, 10))) {
            if (m.id != mav::MSG_PARAM_REQUEST_READ) continue;
            ++asked;
            char id[17]{};
            for (int i = 0; i < 16; ++i) id[i] = char(m.u8(4 + i));
            for (const PV& q : table) if (std::strcmp(q.n, id) == 0) fcParam(mfd, q.n, q.v);
        }
    }
    return asked;
}

// ---- what ArduCopter makes of a pulse, written from its source ------------
// RC_Channel::pwm_to_angle_dz_trim, as a fraction of full deflection.
float apAngle(float us, float lo, float trim, float hi, float dz) {
    if (us > trim + dz) return (us - (trim + dz)) / (hi - (trim + dz));
    if (us < trim - dz) return (us - (trim - dz)) / ((trim - dz) - lo);
    return 0.f;
}
// Copter::get_pilot_desired_climb_rate, m/s, from the throttle pulse
// (RC_Channel RANGE scaling with RC3_DZ at the bottom; get_control_mid).
float apClimb(float us, float lo, float hi, float dz3, float thrDz, float upMps, float dnMps) {
    const float base = lo + dz3;
    auto ctl = [&](float u) { return std::max(0.f, std::min(1000.f, (u - base) * 1000.f / (hi - base))); };
    const float c = ctl(us), mid = ctl((lo + hi) * 0.5f);
    const float top = mid + thrDz, bot = mid - thrDz;
    if (c < bot) return dnMps * (c - bot) / bot;
    if (c > top) return upMps * (c - top) / (1000.f - top);
    return 0.f;
}

// Parameters under which the new mapping equals the old 1500 +- 500 one, so
// the long-standing channel checks below still pin the same numbers.
std::vector<PV> identityParams() {
    std::vector<PV> t = {
        {"SYSID_MYGCS", 255}, {"MAV_GCS_SYSID", 255}, {"RC_OVERRIDE_TIME", 3},
        {"RCMAP_ROLL", 1}, {"RCMAP_PITCH", 2}, {"RCMAP_THROTTLE", 3}, {"RCMAP_YAW", 4},
        {"LOIT_SPEED", 400}, {"PILOT_SPEED_UP", 150}, {"PILOT_SPEED_DN", 0}, {"THR_DZ", 0},
        {"PILOT_Y_RATE", 90}, {"PILOT_Y_EXPO", 0}, {"ACRO_YAW_P", 2}, {"ANGLE_MAX", 3000},
        {"WPNAV_SPEED", 1000}, {"WPNAV_SPEED_UP", 150}, {"WPNAV_SPEED_DN", 150},
        {"GUID_OPTIONS", 0}};
    static const char* rc[4][5] = {
        {"RC1_MIN", "RC1_TRIM", "RC1_MAX", "RC1_DZ", "RC1_REVERSED"},
        {"RC2_MIN", "RC2_TRIM", "RC2_MAX", "RC2_DZ", "RC2_REVERSED"},
        {"RC3_MIN", "RC3_TRIM", "RC3_MAX", "RC3_DZ", "RC3_REVERSED"},
        {"RC4_MIN", "RC4_TRIM", "RC4_MAX", "RC4_DZ", "RC4_REVERSED"}};
    for (auto& r : rc) {
        t.push_back({r[0], 1000}); t.push_back({r[1], 1500}); t.push_back({r[2], 2000});
        t.push_back({r[3], 0});    t.push_back({r[4], 0});
    }
    return t;
}

const mav::Msg* lastOverride(const std::vector<mav::Msg>& v) {
    const mav::Msg* o = nullptr;
    for (const mav::Msg& m : v) if (m.id == mav::MSG_RC_CHANNELS_OVERRIDE) o = &m;
    return o;
}

// Send a heartbeat in `mode` and let the backend read it.
void enterMode(int mfd, MavlinkBackend& fc, uint32_t mode) {
    fcHeartbeat(mfd, mode, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    fc.tick();
}

}  // namespace

int main() {
    std::printf("MavlinkBackend over PTY\n");

    int mfd = -1, sfd = -1;
    char slaveName[128];
    if (openpty(&mfd, &sfd, slaveName, nullptr, nullptr) != 0) {
        std::printf("openpty failed — skipping\n");
        return 0;
    }
    fcntl(mfd, F_SETFL, O_NONBLOCK);
    close(sfd);   // the backend reopens it by name

    MavlinkBackend fc;
    CHECK(fc.connect(slaveName, 115200));

    // --- connect must announce itself and ask for what it reads -------------
    {
        auto msgs = decodeAll(drainMaster(mfd));
        CHECK(findMsg(msgs, mav::MSG_HEARTBEAT) != nullptr);
        // Six SET_MESSAGE_INTERVAL commands, one per stream it consumes --
        // LOCAL_POSITION_NED is the sixth, the flow-aided leg odometry.
        int intervals = 0;
        for (const mav::Msg& m : msgs)
            if (m.id == mav::MSG_COMMAND_LONG && m.u16(28) == mav::CMD_SET_MESSAGE_INTERVAL)
                ++intervals;
        CHECK(intervals == 6);
        std::printf("  connect: heartbeat + %d stream requests\n", intervals);
    }

    // --- link is DOWN until the autopilot says something --------------------
    fc.tick();
    CHECK(!fc.linkUp());

    // --- telemetry decode ---------------------------------------------------
    fcHeartbeat(mfd, mav::COPTER_ACRO, true);
    {
        mav::Payload p;
        p.u32(1000);
        p.f32(0.1f); p.f32(-0.2f); p.f32(1.57f);
        p.f32(0); p.f32(0); p.f32(0);
        fcSend(mfd, mav::MSG_ATTITUDE, p);
    }
    {
        mav::Payload p;                            // SYS_STATUS, 22.2 V, 77 %
        p.u32(0); p.u32(0); p.u32(0);
        p.u16(250); p.u16(22200); p.i16(1500);
        p.u16(0); p.u16(0); p.u16(0); p.u16(0); p.u16(0); p.u16(0);
        p.i8(77);
        fcSend(mfd, mav::MSG_SYS_STATUS, p);
    }
    {
        mav::Payload p;                            // RC_CHANNELS, 8 channels
        p.u32(1000);
        const uint16_t ch[18] = {1500, 1600, 1100, 1400, 1000, 2000, 1500, 1500};
        for (int i = 0; i < 18; ++i) p.u16(ch[i]);
        p.u8(8); p.u8(200);
        fcSend(mfd, mav::MSG_RC_CHANNELS, p);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    fc.tick();

    FcTelemetry t;
    CHECK(fc.poll(t));
    CHECK(fc.linkUp());
    CHECK(fc.copterMode() == mav::COPTER_ACRO);
    CHECK(t.armed);
    CHECK(std::fabs(t.rollDeg - 5.7296f) < 0.01f);
    CHECK(std::fabs(t.pitchDeg + 11.459f) < 0.01f);
    CHECK(std::fabs(t.yawDeg - 89.954f) < 0.01f);
    CHECK(std::fabs(t.battV - 22.2f) < 1e-3f);
    CHECK(std::fabs(t.battPct - 0.77f) < 1e-3f);
    CHECK(t.rcCount == 8);                       // the offset bug lands here
    CHECK(t.rc[0] == 1500 && t.rc[3] == 1400 && t.rc[7] == 1500);
    std::printf("  telemetry: ACRO, armed, %.1f V %.0f%%, %d RC channels\n",
                double(t.battV), double(t.battPct * 100), t.rcCount);

    drainMaster(mfd);   // discard heartbeats so far

    // --- no control until ArduPilot's parameters are known ------------------
    {
        ControlCmd c; c.valid = true;
        CHECK(!fc.sendControl(c));
        CHECK(!fc.paramsResolved());
        const int asked = serveParams(mfd, fc, identityParams());
        CHECK(fc.paramsResolved());
        CHECK(asked >= 39);                       // every one asked for, by name
        const std::string rep = fc.paramReport();
        CHECK(rep.find("39 of 39") != std::string::npos);
        CHECK(rep.find("matches ours") != std::string::npos);
        CHECK(rep.find("!!") == std::string::npos);
        std::printf("  parameters: %d requests, 39 answers, control held until then\n", asked);
    }

    // --- ACRO: mid stick is not "hold height", so no sticks at all -----------
    {
        ControlCmd c; c.valid = true; c.throttle = 0.3f;
        CHECK(!fc.sendControl(c));
        CHECK(std::string(fc.controlPath()).find("none") == 0);
        auto msgs = decodeAll(drainMaster(mfd));
        CHECK(countMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE) == 0);
        std::printf("  ACRO: nothing sent (throttle would be raw collective)\n");
    }

    enterMode(mfd, fc, mav::COPTER_ALT_HOLD);
    drainMaster(mfd);

    // --- RC override: channel order and the release rule --------------------
    {
        ControlCmd c;
        c.valid = true; c.roll = 0.5f; c.pitch = -0.5f; c.throttle = 0.6f; c.yaw = 0.2f;
        CHECK(fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* o = findMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE);
        CHECK(o != nullptr);
        if (o) {
            CHECK(o->u16(0) == 1750);              // roll  +0.5 -> 1500 + 250
            CHECK(o->u16(2) == 1750);              // pitch -0.5 = BACK = high on ArduPilot
            CHECK(o->u16(4) == 1800);              // throttle 0.6 -> 1500 + 300
            CHECK(o->u16(6) == 1600);              // yaw   +0.2
            // Channels we do not drive must be ZERO, which tells ArduPilot to
            // hand them back to the receiver. The mode switch lives there: if
            // this ever stops being zero, the pilot loses the ability to take
            // the aircraft back, which is the worst bug this file could have.
            for (int i = 4; i < 8; ++i) CHECK(o->u16(uint16_t(2 * i)) == 0);
            CHECK(o->u8(16) == 1 && o->u8(17) == 1);   // addressed to the autopilot
        }
        std::printf("  RC override: AETR order, channels 5-8 released to the pilot\n");
    }
    // --- FORWARD is a LOW pitch pulse on ArduPilot ---------------------------
    {
        ControlCmd c;
        c.valid = true; c.pitch = 0.5f;
        CHECK(fc.sendControl(c));
        auto held1 = decodeAll(drainMaster(mfd));
        const mav::Msg* o = lastOverride(held1);
        CHECK(o != nullptr);
        if (o) CHECK(o->u16(2) == 1250);           // its autotest flies north on 1300
        std::printf("  RC override: forward pitch is a low pulse (1250 us), not iNAV's high\n");
    }
    // --- a DESCENT is below mid, not clamped to it ---------------------------
    {
        ControlCmd c;
        c.valid = true; c.throttle = -0.6f;
        CHECK(fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* o = findMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE);
        CHECK(o != nullptr);
        if (o) CHECK(o->u16(4) == 1200);           // throttle -0.6 -> 1500 - 300: descend
        std::printf("  RC override: throttle below zero is a descent (1200 us)\n");
    }

    // --- assist mode trims from the latched baseline ------------------------
    {
        fc.setAssistMode(true);
        fc.latchBaseline();                       // from the RC_CHANNELS above
        ControlCmd c;
        c.valid = true; c.roll = 0.1f;            // +50 us on top of 1500
        CHECK(fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* o = findMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE);
        CHECK(o != nullptr);
        if (o) {
            CHECK(o->u16(0) == 1550);             // 1500 baseline + 50
            CHECK(o->u16(2) == 1600);             // pitch untouched at baseline
        }
        // A throttle delta is a climb rate in ALT_HOLD; in ACRO it would be
        // raw collective on the pilot's, so there the pilot's throttle stands.
        c.roll = 0.f; c.throttle = 0.2f;
        CHECK(fc.sendControl(c));
        auto held2 = decodeAll(drainMaster(mfd));
        o = lastOverride(held2);
        CHECK(o && o->u16(4) == 1200);            // 1100 + 100
        enterMode(mfd, fc, mav::COPTER_ACRO);
        CHECK(fc.sendControl(c));
        auto held3 = decodeAll(drainMaster(mfd));
        o = lastOverride(held3);
        CHECK(o && o->u16(4) == 1100);            // the pilot's own, untouched
        fc.setAssistMode(false);
        std::printf("  assist: trims the operator's sticks; no throttle delta outside height-hold\n");
    }

    // --- leaving a height-hold mode RELEASES the override at once -----------
    {
        enterMode(mfd, fc, mav::COPTER_ALT_HOLD);
        ControlCmd c; c.valid = true; c.throttle = 0.4f;
        CHECK(fc.sendControl(c));
        drainMaster(mfd);
        enterMode(mfd, fc, mav::COPTER_STABILIZE);
        CHECK(!fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        CHECK(countMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE) == 1);
        const mav::Msg* o = lastOverride(msgs);
        if (o) for (int i = 0; i < 8; ++i) CHECK(o->u16(uint16_t(2 * i)) == 0);
        CHECK(!fc.sendControl(c));                // once is enough
        CHECK(countMsg(decodeAll(drainMaster(mfd)), mav::MSG_RC_CHANNELS_OVERRIDE) == 0);
        std::printf("  STABILIZE: one all-zero override hands every stick back, then silence\n");
    }

    // --- an invalid command sends nothing at all ----------------------------
    {
        enterMode(mfd, fc, mav::COPTER_ALT_HOLD);
        drainMaster(mfd);
        ControlCmd c;                             // valid == false
        CHECK(!fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        CHECK(countMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE) == 0);
    }

    // --- CALIBRATED: what ArduPilot READS from our pulses is what we meant ---
    // Real-radio calibration and ArduPilot's defaults: 1100..1900 with a 20 us
    // deadzone, THR_DZ 100, PILOT_SPEED_UP 2.5 m/s, LOIT_SPEED 12.5 m/s,
    // PILOT_Y_RATE 202.5 deg/s. The old 1500 +- 500 mapping flew 3x too fast in
    // LOITER and ignored every climb correction under 0.2 stick.
    {
        const std::vector<PV> ap = {
            {"RC1_MIN", 1100}, {"RC1_MAX", 1900}, {"RC1_DZ", 20},
            {"RC2_MIN", 1100}, {"RC2_MAX", 1900}, {"RC2_DZ", 20},
            {"RC3_MIN", 1100}, {"RC3_MAX", 1900}, {"RC3_DZ", 30},
            {"RC4_MIN", 1100}, {"RC4_MAX", 1900}, {"RC4_DZ", 20},
            {"THR_DZ", 100}, {"PILOT_SPEED_UP", 250}, {"PILOT_SPEED_DN", 150},
            {"LOIT_SPEED", 1250}, {"PILOT_Y_RATE", 202.5f}};
        for (const PV& q : ap) fcParam(mfd, q.n, q.v);
        enterMode(mfd, fc, mav::COPTER_LOITER);
        drainMaster(mfd);

        struct Case { float pitch, roll, thr, yaw; };
        const Case cases[] = {{0.5f, -0.25f, 0.3f, 0.5f}, {-1.f, 0.05f, -0.4f, -0.1f},
                              {0.1f, 0.f, 0.05f, 0.02f}, {0.f, 0.f, -0.03f, 0.f}};
        float worstV = 0, worstC = 0, worstY = 0;
        for (const Case& k : cases) {
            ControlCmd c; c.valid = true;
            c.pitch = k.pitch; c.roll = k.roll; c.throttle = k.thr; c.yaw = k.yaw;
            CHECK(fc.sendControl(c));
            auto held4 = decodeAll(drainMaster(mfd));
            const mav::Msg* o = lastOverride(held4);
            CHECK(o != nullptr);
            if (!o) continue;
            // ArduPilot's view: pitch is negated back (forward = low).
            const float vFwd = -apAngle(o->u16(2), 1100, 1500, 1900, 20) * 12.5f;
            const float vRt  =  apAngle(o->u16(0), 1100, 1500, 1900, 20) * 12.5f;
            const float clmb =  apClimb(o->u16(4), 1100, 1900, 30, 100, 2.5f, 1.5f);
            const float yaw  =  apAngle(o->u16(6), 1100, 1500, 1900, 20) * 202.5f;
            worstV = std::max({worstV, std::fabs(vFwd - k.pitch * 4.f), std::fabs(vRt - k.roll * 4.f)});
            worstC = std::max(worstC, std::fabs(clmb - k.thr * 1.5f));
            worstY = std::max(worstY, std::fabs(yaw - k.yaw * 90.f));
        }
        // One microsecond of rounding is 1/380 of the range.
        CHECK(worstV < 0.05f);
        CHECK(worstC < 0.02f);
        CHECK(worstY < 0.6f);
        std::printf("  LOITER, real calibration: ArduPilot reads back within %.3f m/s, "
                    "%.3f m/s climb, %.2f deg/s\n", double(worstV), double(worstC), double(worstY));

        // A remapped stick is refused, not guessed at.
        fcParam(mfd, "RCMAP_PITCH", 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        ControlCmd c; c.valid = true;
        CHECK(!fc.sendControl(c));
        fcParam(mfd, "RCMAP_PITCH", 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        CHECK(fc.sendControl(c));
        drainMaster(mfd);
        // Back to the identity numbers for everything below.
        for (const PV& q : identityParams()) fcParam(mfd, q.n, q.v);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        fc.tick();
    }

    // --- GUIDED: say the speed (AUTO picks the velocity uplink) --------------
    {
        enterMode(mfd, fc, mav::COPTER_GUIDED);
        drainMaster(mfd);
        ControlCmd c; c.valid = true;
        c.pitch = 0.5f; c.roll = -0.25f; c.throttle = 0.5f; c.yaw = 0.5f;
        CHECK(fc.sendControl(c));
        CHECK(std::string(fc.controlPath()) == "velocity setpoint");
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* s = findMsg(msgs, mav::MSG_SET_POSITION_TARGET_LOCAL_NED);
        CHECK(s != nullptr);
        if (s) {
            CHECK(std::fabs(s->f32(16) - 2.f) < 1e-5f);              // 0.5 x 4 m/s forward
            CHECK(std::fabs(s->f32(20) + 1.f) < 1e-5f);              // 1 m/s left
            CHECK(std::fabs(s->f32(24) + 0.75f) < 1e-5f);            // 0.75 m/s UP = -z
            CHECK(std::fabs(s->f32(44) - 0.7853982f) < 1e-5f);       // 45 deg/s
            CHECK(s->u16(48) == 0x05C7);
        }
        // The override from LOITER was handed back on the way in.
        const mav::Msg* o = lastOverride(msgs);
        if (o) for (int i = 0; i < 4; ++i) CHECK(o->u16(uint16_t(2 * i)) == 0);
        // A FIXED stick uplink in GUIDED sends nothing at all.
        fc.setUplink(MavlinkBackend::Uplink::RC_OVERRIDE);
        CHECK(!fc.sendControl(c));
        msgs = decodeAll(drainMaster(mfd));
        CHECK(countMsg(msgs, mav::MSG_SET_POSITION_TARGET_LOCAL_NED) == 0);
        CHECK(countMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE) == 0);
        fc.setUplink(MavlinkBackend::Uplink::AUTO);
        std::printf("  GUIDED: velocity setpoint 2.0 fwd, 1.0 left, 0.75 up, 45 deg/s\n");
    }

    // --- GUIDED_NOGPS: attitude, thrust a climb rate through WPNAV_SPEED_UP --
    {
        { mav::Payload p; p.u32(1500);
          p.f32(0.f); p.f32(0.f); p.f32(0.f); p.f32(0); p.f32(0); p.f32(0);
          fcSend(mfd, mav::MSG_ATTITUDE, p); }
        fcParam(mfd, "WPNAV_SPEED_UP", 250);
        enterMode(mfd, fc, mav::COPTER_GUIDED_NOGPS);
        drainMaster(mfd);
        ControlCmd c; c.valid = true; c.throttle = 0.5f;          // 0.75 m/s
        CHECK(fc.sendControl(c));
        auto held5 = decodeAll(drainMaster(mfd));
        const mav::Msg* a = findMsg(held5, mav::MSG_SET_ATTITUDE_TARGET);
        CHECK(a != nullptr);
        if (a) CHECK(std::fabs(a->f32(32) - 0.65f) < 1e-4f);       // 0.5 + 0.5 x 0.75/2.5
        // Thrust-as-thrust would make 0.5 something other than a hover: refuse.
        fcParam(mfd, "GUID_OPTIONS", 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        CHECK(!fc.sendControl(c));
        CHECK(fc.paramReport().find("GUID_OPTIONS") != std::string::npos);
        fcParam(mfd, "GUID_OPTIONS", 0);
        fcParam(mfd, "WPNAV_SPEED_UP", 150);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        drainMaster(mfd);
        std::printf("  GUIDED_NOGPS: attitude, thrust 0.65 = 0.75 of 2.5 m/s; refused if thrust-as-thrust\n");
    }

    // --- RESUME: a cleared RTL goes back to the pilot's mode, not STABILIZE --
    {
        auto modeCmds = [&](std::vector<float>& out) {
            for (const mav::Msg& m : decodeAll(drainMaster(mfd)))
                if (m.id == mav::MSG_COMMAND_LONG && m.u16(28) == mav::CMD_DO_SET_MODE)
                    out.push_back(m.f32(4));
        };
        enterMode(mfd, fc, mav::COPTER_LOITER);
        drainMaster(mfd);
        std::vector<float> got;
        CHECK(fc.setMode(FcMode::RTL));
        enterMode(mfd, fc, mav::COPTER_RTL);
        CHECK(fc.setMode(FcMode::RESUME));
        modeCmds(got);
        CHECK(got.size() == 2 && got[0] == float(mav::COPTER_RTL) &&
              got[1] == float(mav::COPTER_LOITER));
        enterMode(mfd, fc, mav::COPTER_LOITER);    // the FC reports it is back
        // RTL then LAND then clear: still the pilot's LOITER.
        got.clear();
        CHECK(fc.setMode(FcMode::RTL));
        enterMode(mfd, fc, mav::COPTER_RTL);
        CHECK(fc.setMode(FcMode::LAND));
        enterMode(mfd, fc, mav::COPTER_LAND);
        CHECK(fc.setMode(FcMode::RESUME));
        modeCmds(got);
        CHECK(got.size() == 3 && got[2] == float(mav::COPTER_LOITER));
        // The pilot flicked it somewhere else meanwhile: theirs stands.
        enterMode(mfd, fc, mav::COPTER_ALT_HOLD);
        got.clear();
        CHECK(fc.setMode(FcMode::RTL));
        enterMode(mfd, fc, mav::COPTER_STABILIZE);
        CHECK(!fc.setMode(FcMode::RESUME));
        modeCmds(got);
        CHECK(got.size() == 1);                    // just the RTL
        // And with nothing to resume, nothing is sent.
        CHECK(!fc.setMode(FcMode::RESUME));
        std::printf("  RESUME: RTL/LAND clear back to LOITER; a pilot's own switch is left alone\n");
    }

    // --- a GCS sysid that is not ours is the loudest line in the report -----
    {
        fcParam(mfd, "MAV_GCS_SYSID", 254);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        CHECK(fc.paramReport().find("IGNORE") != std::string::npos);
        fcParam(mfd, "MAV_GCS_SYSID", 255);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        drainMaster(mfd);
    }

    // --- mode and arm -------------------------------------------------------
    {
        CHECK(fc.setMode(FcMode::OFFBOARD));
        CHECK(fc.arm(false));
        CHECK(fc.disarm());
        auto msgs = decodeAll(drainMaster(mfd));
        int setMode = 0, armed = 0, disarmed = 0;
        for (const mav::Msg& m : msgs) {
            if (m.id != mav::MSG_COMMAND_LONG) continue;
            const uint16_t cmd = m.u16(28);
            if (cmd == mav::CMD_DO_SET_MODE) {
                ++setMode;
                CHECK(std::fabs(m.f32(0) - 1.f) < 1e-6f);            // CUSTOM_MODE_ENABLED
                CHECK(std::fabs(m.f32(4) - float(mav::COPTER_GUIDED)) < 1e-6f);
            } else if (cmd == mav::CMD_COMPONENT_ARM_DISARM) {
                if (m.f32(0) > 0.5f) ++armed; else ++disarmed;
                CHECK(std::fabs(m.f32(4)) < 1e-6f);                  // NOT forced
            }
        }
        CHECK(setMode == 1 && armed == 1 && disarmed == 1);
        std::printf("  commands: OFFBOARD -> GUIDED(4), arm/disarm unforced\n");
    }

    // --- body-frame velocity setpoint --------------------------------------
    {
        CHECK(fc.sendVelocityBody(1.5f, -0.25f, -0.5f, 0.3f));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* s = findMsg(msgs, mav::MSG_SET_POSITION_TARGET_LOCAL_NED);
        CHECK(s != nullptr);
        if (s) {
            CHECK(std::fabs(s->f32(16) - 1.5f) < 1e-6f);     // vx forward
            CHECK(std::fabs(s->f32(20) + 0.25f) < 1e-6f);    // vy right
            CHECK(std::fabs(s->f32(24) + 0.5f) < 1e-6f);     // vz down
            CHECK(std::fabs(s->f32(44) - 0.3f) < 1e-6f);     // yaw_rate
            CHECK(s->u16(48) == 0x05C7);                     // velocity + yaw rate only
            CHECK(s->u8(52) == mav::FRAME_BODY_NED);
        }
        std::printf("  velocity setpoint: BODY_NED, mask 0x05C7 (yaw rate USED)\n");
    }

    // --- vision pose, and the origin latch ---------------------------------
    {
        ExtGps g;
        g.lat = 60.1234567; g.lon = 24.8765432; g.altMslM = 100.f;
        g.velN = 1.f; g.velE = 0.5f; g.velD = -0.25f; g.fixType = 3;
        CHECK(fc.feedExternalGps(g));                        // latches the origin
        drainMaster(mfd);
        g.lat += 0.0000898;                                  // ~10 m North
        g.altMslM = 105.f;                                   // 5 m up
        CHECK(fc.feedExternalGps(g));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* v = findMsg(msgs, mav::MSG_VISION_POSITION_ESTIMATE);
        CHECK(v != nullptr);
        if (v) {
            CHECK(std::fabs(v->f32(8) - 10.f) < 0.2f);        // x = North, metres
            CHECK(std::fabs(v->f32(12)) < 0.2f);              // y = East, unmoved
            CHECK(std::fabs(v->f32(16) + 5.f) < 0.01f);       // z is DOWN: up is negative
        }
        const mav::Msg* sp = findMsg(msgs, mav::MSG_VISION_SPEED_ESTIMATE);
        CHECK(sp != nullptr);
        if (sp) CHECK(std::fabs(sp->f32(8) - 1.f) < 1e-6f);
        // A fix the estimator does not trust must not reach the EKF.
        g.fixType = 2;
        CHECK(!fc.feedExternalGps(g));
        std::printf("  vision pose: origin latched, NED offsets, z down\n");
    }

    // --- VIO odometry: ENU in, NED on the wire, reset counter carried --------
    {
        drainMaster(mfd);
        VisionOdom v;
        v.e = 2.f; v.n = 5.f; v.u = 1.5f;             // 5 m North, 2 m East, 1.5 m up
        v.ve = 0.5f; v.vn = 1.f; v.vu = 0.2f;
        v.rollDeg = 2.f; v.pitchDeg = -3.f; v.yawDeg = 90.f;
        v.resets = 3;
        CHECK(fc.sendVisionOdometry(v));
        auto msgs = decodeAll(drainMaster(mfd));
        const mav::Msg* p = findMsg(msgs, mav::MSG_VISION_POSITION_ESTIMATE);
        CHECK(p != nullptr);
        if (p) {
            CHECK(std::fabs(p->f32(8) - 5.f) < 1e-6f);     // x = North
            CHECK(std::fabs(p->f32(12) - 2.f) < 1e-6f);    // y = East
            CHECK(std::fabs(p->f32(16) + 1.5f) < 1e-6f);   // z DOWN: up is negative
            CHECK(std::fabs(p->f32(28) - 1.5707963f) < 1e-5f);   // yaw, radians
            CHECK(std::isnan(p->f32(32)));                 // covariance: unknown
            CHECK(p->u8(116) == 3);                        // reset_counter
        }
        const mav::Msg* sp = findMsg(msgs, mav::MSG_VISION_SPEED_ESTIMATE);
        CHECK(sp != nullptr);
        if (sp) {
            CHECK(std::fabs(sp->f32(8) - 1.f) < 1e-6f && std::fabs(sp->f32(12) - 0.5f) < 1e-6f);
            CHECK(std::fabs(sp->f32(16) + 0.2f) < 1e-6f);
        }
        std::printf("  VIO odometry: ENU->NED, yaw in rad, covariance unknown, resets 3\n");
    }

    // --- link goes down when the autopilot stops talking --------------------
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2100));
        fc.tick();
        CHECK(!fc.linkUp());
        fcHeartbeat(mfd, mav::COPTER_GUIDED, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        CHECK(fc.linkUp());
        CHECK(fc.copterMode() == mav::COPTER_GUIDED);
        std::printf("  link: drops after 2 s of silence, recovers on the next heartbeat\n");
    }

    // --- another GCS on the link must not set our idea of the mode ----------
    {
        mav::Codec other(200, 190);
        mav::Payload p;
        p.u32(mav::COPTER_LAND);
        p.u8(6); p.u8(8); p.u8(0x01); p.u8(4); p.u8(3);
        uint8_t buf[64];
        const int n = other.frame(mav::MSG_HEARTBEAT, p, buf);
        CHECK(write(mfd, buf, n) == n);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        fc.tick();
        CHECK(fc.copterMode() == mav::COPTER_GUIDED);   // unchanged
        std::printf("  a second GCS's heartbeat does not overwrite the flight mode\n");
    }

    // --- SET_ATTITUDE_TARGET uplink ----------------------------------------
    // The codec is proven byte-exact in test_mavlink.cpp; what is unproven here
    // is the ENCODING -- quaternion signs, the yaw reference and the thrust
    // offset. Those are the failure modes that produce a perfectly valid frame
    // meaning the wrong thing, which is exactly how the chancount bug got
    // through, and a sign error here is a crash rather than a wobble.
    {
        fc.setUplink(MavlinkBackend::Uplink::ATTITUDE_TARGET);
        fc.setMaxTiltDeg(20.f);

        // Heading 90 deg (East), level. Yaw on the wire is radians.
        { mav::Payload p; p.u32(2000);
          p.f32(0.f); p.f32(0.f); p.f32(1.5707963f);
          p.f32(0); p.f32(0); p.f32(0);
          fcSend(mfd, mav::MSG_ATTITUDE, p); }
        fc.tick(); drainMaster(mfd);

        ControlCmd cmd; cmd.valid = true;
        cmd.roll = 0.f; cmd.pitch = 0.f; cmd.yaw = 0.f; cmd.throttle = 0.f;
        CHECK(fc.sendControl(cmd));
        auto amsgs = decodeAll(drainMaster(mfd));
        const mav::Msg* a = findMsg(amsgs, mav::MSG_SET_ATTITUDE_TARGET);
        CHECK(a != nullptr);
        if (a) {
            // Level while holding 90 deg: q = (cos45, 0, 0, sin45).
            CHECK(std::fabs(a->f32(4)  - 0.70710678f) < 1e-3f);   // q0
            CHECK(std::fabs(a->f32(8))                < 1e-3f);   // q1 roll
            CHECK(std::fabs(a->f32(12))               < 1e-3f);   // q2 pitch
            CHECK(std::fabs(a->f32(16) - 0.70710678f) < 1e-3f);   // q3 yaw
            // throttle 0 means HOVER, so thrust 0.5 -- not zero thrust. Getting
            // this wrong drops the aircraft out of the sky on the first command.
            CHECK(std::fabs(a->f32(32) - 0.5f) < 1e-3f);
            CHECK(a->u8(38) == 0x07);          // body rates ignored by the mask
        }

        // A forward pitch command must mean NOSE DOWN. ArduPilot's pitch is
        // positive nose-UP, so +1 forward has to arrive as a negative pitch.
        cmd.pitch = 1.f;
        CHECK(fc.sendControl(cmd));
        amsgs = decodeAll(drainMaster(mfd));
        const mav::Msg* b = findMsg(amsgs, mav::MSG_SET_ATTITUDE_TARGET);
        CHECK(b != nullptr);
        if (b) {
            const float q0=b->f32(4), q1=b->f32(8), q2=b->f32(12), q3=b->f32(16);
            const float s = std::max(-1.f, std::min(1.f, 2.f*(q0*q2 - q3*q1)));
            CHECK(std::asin(s) < -0.2f);       // ~ -20 deg, nose down
        }

        // And with NO attitude ever decoded it must REFUSE, not guess a
        // heading: an absolute yaw target from an unknown heading is a silent
        // turn to somewhere arbitrary.
        MavlinkBackend bare;
        ControlCmd c2; c2.valid = true;
        CHECK(!bare.sendControl(c2));

        fc.setUplink(MavlinkBackend::Uplink::AUTO);
    }

    CHECK(fc.crcErrors() == 0);
    fc.disconnect();
    close(mfd);

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
