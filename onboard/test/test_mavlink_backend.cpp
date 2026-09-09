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
// order and the release-to-pilot rule, assist-mode trim, and mode/arm commands.

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
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
        // Five SET_MESSAGE_INTERVAL commands, one per stream it consumes.
        int intervals = 0;
        for (const mav::Msg& m : msgs)
            if (m.id == mav::MSG_COMMAND_LONG && m.u16(28) == mav::CMD_SET_MESSAGE_INTERVAL)
                ++intervals;
        CHECK(intervals == 5);
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
            CHECK(o->u16(2) == 1250);              // pitch -0.5
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
        fc.setAssistMode(false);
        std::printf("  assist: trims from the operator's latched sticks\n");
    }

    // --- an invalid command sends nothing at all ----------------------------
    {
        ControlCmd c;                             // valid == false
        CHECK(!fc.sendControl(c));
        auto msgs = decodeAll(drainMaster(mfd));
        CHECK(countMsg(msgs, mav::MSG_RC_CHANNELS_OVERRIDE) == 0);
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
            CHECK(s->u16(48) == 0x0DC7);                     // velocity + yaw rate only
            CHECK(s->u8(52) == mav::FRAME_BODY_NED);
        }
        std::printf("  velocity setpoint: BODY_NED, mask 0x0DC7\n");
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

        fc.setUplink(MavlinkBackend::Uplink::RC_OVERRIDE);
    }

    CHECK(fc.crcErrors() == 0);
    fc.disconnect();
    close(mfd);

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
