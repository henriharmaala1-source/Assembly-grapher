// MAVLink v2 codec checks, against GOLDEN FRAMES from pymavlink.
//
// Every expected string below is the exact output of the canonical Python
// implementation for the same arguments, generated with srcSystem 42,
// srcComponent 191, seq 7. This is the whole validation strategy: a hand-written
// MAVLink encoder that is subtly wrong does not error, it just gets ignored by
// the autopilot, and on a bench that is indistinguishable from a bad solder
// joint. Byte equality against the reference implementation is the only check
// that catches a wrong CRC_EXTRA or a transposed field before flight.
//
// Regenerate with pymavlink if a message is added:
//   mav = MAVLink(buf, srcSystem=42, srcComponent=191); mav.seq = 7
//
//   g++ -O2 -std=c++17 -Iinclude test/test_mavlink.cpp src/mavlink_v2.cpp -o /tmp/tm

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "mavlink_v2.hpp"

using namespace mav;

static int failures = 0;

static std::string hex(const uint8_t* b, int n) {
    static const char* d = "0123456789abcdef";
    std::string s;
    for (int i = 0; i < n; ++i) { s += d[b[i] >> 4]; s += d[b[i] & 0xF]; }
    return s;
}
static std::vector<uint8_t> unhex(const std::string& s) {
    auto v = [](char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; };
    std::vector<uint8_t> o;
    for (size_t i = 0; i + 1 < s.size(); i += 2) o.push_back(uint8_t(v(s[i]) * 16 + v(s[i+1])));
    return o;
}
static void expectFrame(const char* what, Codec& c, uint32_t id, const Payload& p,
                        const char* want) {
    uint8_t out[300];
    int n = c.frame(id, p, out);
    std::string got = hex(out, n);
    const bool ok = (got == want);
    std::printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        std::printf("    want %s\n    got  %s\n", want, got.c_str());
        ++failures;
    }
}
static void check(bool ok, const char* what) {
    std::printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

// Each test rebuilds the codec so seq is always 7, matching how the goldens
// were generated.
static Codec fresh() {
    Codec c(42, 191);
    Payload dummy; uint8_t o[300];
    for (int i = 0; i < 7; ++i) c.frame(MSG_HEARTBEAT, dummy, o);   // advance seq to 7
    return c;
}

int main() {
    std::printf("MAVLink v2 codec, against pymavlink golden frames\n");

    // --- ENCODE ------------------------------------------------------------
    {
        Codec c = fresh();
        Payload p;
        p.u32(0);                              // custom_mode
        p.u8(6);                               // type = MAV_TYPE_GCS
        p.u8(8);                               // autopilot = INVALID
        p.u8(0);                               // base_mode
        p.u8(4);                               // system_status = ACTIVE
        p.u8(3);                               // mavlink_version
        expectFrame("HEARTBEAT", c, MSG_HEARTBEAT, p,
                    "fd090000072abf00000000000000060800040324e2");
    }
    {
        Codec c = fresh();
        Payload p;
        p.f32(1.f); p.f32(4.f);                // param1 = CUSTOM_MODE_ENABLED, param2 = GUIDED
        for (int i = 0; i < 5; ++i) p.f32(0.f);
        p.u16(CMD_DO_SET_MODE); p.u8(1); p.u8(1); p.u8(0);
        expectFrame("COMMAND_LONG DO_SET_MODE -> GUIDED(4)", c, MSG_COMMAND_LONG, p,
                    "fd200000072abf4c00000000803f000080400000000000000000000000000000000000000000b00001019609");
    }
    {
        Codec c = fresh();
        Payload p;
        p.f32(1.f); p.f32(21196.f);            // arm, force
        for (int i = 0; i < 5; ++i) p.f32(0.f);
        p.u16(CMD_COMPONENT_ARM_DISARM); p.u8(1); p.u8(1); p.u8(0);
        expectFrame("COMMAND_LONG ARM (force magic 21196)", c, MSG_COMMAND_LONG, p,
                    "fd200000072abf4c00000000803f0098a5460000000000000000000000000000000000000000900101013328");
    }
    {
        // The message the autonomy actually flies on: body-frame velocity plus
        // yaw rate, everything else masked off.
        Codec c = fresh();
        Payload p;
        p.u32(123456);
        p.f32(0); p.f32(0); p.f32(0);                   // x y z  (masked)
        p.f32(1.5f); p.f32(-0.25f); p.f32(-0.5f);       // vx vy vz
        p.f32(0); p.f32(0); p.f32(0);                   // afx afy afz (masked)
        p.f32(0.f); p.f32(0.3f);                        // yaw, yaw_rate
        p.u16(0x0DC7); p.u8(1); p.u8(1); p.u8(FRAME_BODY_NED);
        expectFrame("SET_POSITION_TARGET_LOCAL_NED vel+yawrate, BODY_NED",
                    c, MSG_SET_POSITION_TARGET_LOCAL_NED, p,
                    "fd350000072abf54000040e201000000000000000000000000000000c03f000080be000000bf00"
                    "0000000000000000000000000000009a99993ec70d0101087ac9");
    }
    {
        Codec c = fresh();
        Payload p;
        p.u32(123456);
        p.f32(1.f); p.f32(0.f); p.f32(0.f); p.f32(0.f);  // q
        p.f32(0.1f); p.f32(-0.2f); p.f32(0.3f);          // body rates
        p.f32(0.55f);                                     // thrust
        p.u8(1); p.u8(1); p.u8(0x80);
        expectFrame("SET_ATTITUDE_TARGET rates + thrust", c, MSG_SET_ATTITUDE_TARGET, p,
                    "fd270000072abf52000040e201000000803f000000000000000000000000cdcccc3dcdcc4cbe9a"
                    "99993ecdcc0c3f0101800080");
    }
    {
        // The truncation case, and the reason it has its own test: channels
        // 9-18 are all zero, so twenty bytes vanish from the wire and the
        // length field reads 18 rather than 38.
        Codec c = fresh();
        Payload p;
        p.u16(1500); p.u16(1500); p.u16(1100); p.u16(1500);
        p.u16(0); p.u16(0); p.u16(0); p.u16(0);
        p.u8(1); p.u8(1);
        for (int i = 0; i < 10; ++i) p.u16(0);
        expectFrame("RC_CHANNELS_OVERRIDE (v2 trailing-zero truncation)",
                    c, MSG_RC_CHANNELS_OVERRIDE, p,
                    "fd120000072abf460000dc05dc054c04dc0500000000000000000101b015");
    }
    {
        Codec c = fresh();
        Payload p;
        p.u64(1234567890);
        p.f32(1.f); p.f32(2.f); p.f32(-3.f);
        p.f32(0.01f); p.f32(0.02f); p.f32(0.03f);
        expectFrame("VISION_POSITION_ESTIMATE", c, MSG_VISION_POSITION_ESTIMATE, p,
                    "fd200000072abf660000d2029649000000000000803f00000040000040c00ad7233c0ad7a33c8f"
                    "c2f53c5580");
    }
    {
        Codec c = fresh();
        Payload p;
        p.u64(1234567890);
        p.f32(1.5f); p.f32(-0.5f); p.f32(0.25f);
        expectFrame("VISION_SPEED_ESTIMATE", c, MSG_VISION_SPEED_ESTIMATE, p,
                    "fd140000072abf670000d2029649000000000000c03f000000bf0000803e4bfc");
    }
    {
        Codec c = fresh();
        Payload p;
        p.f32(30.f); p.f32(20000.f);            // ATTITUDE at 50 Hz
        for (int i = 0; i < 5; ++i) p.f32(0.f);
        p.u16(CMD_SET_MESSAGE_INTERVAL); p.u8(1); p.u8(1); p.u8(0);
        expectFrame("COMMAND_LONG SET_MESSAGE_INTERVAL ATTITUDE 50 Hz",
                    c, MSG_COMMAND_LONG, p,
                    "fd200000072abf4c00000000f04100409c460000000000000000000000000000000000000000ff"
                    "0101015cd0");
    }

    // --- DECODE ------------------------------------------------------------
    // Same goldens driven the other way: bytes an autopilot would send us.
    auto decode = [](const char* h, Msg& m) {
        Codec c(255, 190);
        auto v = unhex(h);
        bool got = false;
        for (uint8_t b : v) if (c.feed(b, m)) got = true;
        return got;
    };
    {
        Msg m;
        check(decode("fd1c0000072abf1e0000e8030000cdcccc3dcdcc4cbec3f5c83f0ad7233c0ad7a33c8fc2f53cbf02", m)
              && m.id == MSG_ATTITUDE, "decode ATTITUDE");
        check(std::fabs(m.f32(4) - 0.1f) < 1e-6f && std::fabs(m.f32(8) + 0.2f) < 1e-6f
              && std::fabs(m.f32(12) - 1.57f) < 1e-6f, "  roll/pitch/yaw read back");
    }
    {
        Msg m;
        // NOTE the field order here. pymavlink's sys_status_send takes
        // battery_remaining as its SEVENTH argument, not its last, while on the
        // wire it is the last byte at offset 30. Generating this golden
        // positionally put 77 into errors_count4 and the first version of this
        // test then asserted the decoder was broken. The decoder was right.
        check(decode("fd1f0000072abf010000000000000000000000000000fa00b856dc0500000000000000000000"
                     "00004d6b05", m)
              && m.id == MSG_SYS_STATUS, "decode SYS_STATUS");
        check(m.u16(14) == 22200, "  voltage_battery = 22200 mV");
        check(m.i8(30) == 77, "  battery_remaining = 77 % (int8 at offset 30)");
    }
    {
        Msg m;
        check(decode("fd1c0000072abf210000e8030000871cd623f8dbd30ec0d40100983a00006400ceff19005046e022", m)
              && m.id == MSG_GLOBAL_POSITION_INT, "decode GLOBAL_POSITION_INT");
        check(m.i32(4) == 601234567 && m.i32(8) == 248765432, "  lat/lon read back");
        check(m.u16(26) == 18000, "  hdg = 180.00 deg");
    }
    {
        Msg m;
        check(decode("fd1e0000072abf180000d202964900000000871cd623f8dbd30ec0d401009600c800f4012823030b3f26", m)
              && m.id == MSG_GPS_RAW_INT, "decode GPS_RAW_INT");
        check(m.u8(28) == 3 && m.u8(29) == 11, "  fix_type 3, 11 sats");
    }
    {
        // RC_CHANNELS, and the offset that already bit once. chancount is a
        // uint8 at 40, after all eighteen channels; the backend originally read
        // a uint16 at 38, which is chan18 -- zero on a 16-channel link, so the
        // assist baseline would simply never have latched.
        Msg m;
        check(decode("fd2a0000072abf410000e8030000dc0540064c047805e803d007dc05dc0500000000000000"
                     "0000000000000000000000000008c8f683", m)
              && m.id == MSG_RC_CHANNELS, "decode RC_CHANNELS");
        check(m.u8(40) == 8, "  chancount = 8 (uint8 at offset 40, not u16 at 38)");
        check(m.u16(4) == 1500 && m.u16(10) == 1400 && m.u16(18) == 1500,
              "  channels 1, 4 and 8 read back");
        check(m.u16(38) == 0, "  and offset 38 is chan18, which is what made it look right");
    }
    {
        // The heartbeat that tells us the pilot is in ACRO and the aircraft is
        // armed -- the state this whole backend has to coexist with.
        Msg m;
        check(decode("fd090000072abf00000001000000020381040396a8", m) && m.id == MSG_HEARTBEAT,
              "decode HEARTBEAT from ArduCopter");
        check(m.u32(0) == COPTER_ACRO, "  custom_mode = ACRO (1)");
        check((m.u8(6) & MODE_FLAG_SAFETY_ARMED) != 0, "  base_mode says ARMED");
    }
    {
        Msg m;
        check(decode("fd020000072abf4d00009001a876", m) && m.id == MSG_COMMAND_ACK,
              "decode COMMAND_ACK (2-byte truncated payload)");
        check(m.u16(0) == 400 && m.u8(2) == 0, "  ACCEPTED for ARM_DISARM, zero-extended");
    }

    // --- ROBUSTNESS --------------------------------------------------------
    {
        // A corrupted payload byte must be REJECTED, not delivered. If this
        // fails, every telemetry number downstream is unverified.
        Codec c(255, 190);
        auto v = unhex("fd1c0000072abf1e0000e8030000cdcccc3dcdcc4cbec3f5c83f0ad7233c0ad7a33c8fc2f53cbf02");
        v[12] ^= 0x40;
        Msg m; bool got = false;
        for (uint8_t b : v) if (c.feed(b, m)) got = true;
        check(!got && c.crcErrors() == 1, "a flipped payload bit is rejected, not delivered");
    }
    {
        // Resynchronisation after noise. The property is BOUNDED LOSS, not
        // immunity, and being precise about which matters: a stray 0xFD
        // announces a frame, so the parser dutifully swallows the length byte's
        // worth of what follows, and a bogus length can straddle more than one
        // real frame. No MAVLink parser rescans for a start byte nested inside a
        // payload -- one that did would resynchronise onto payload data that
        // happened to contain 0xFD, which is worse. So the check is that the
        // link recovers in a bounded number of frames rather than dying, and
        // the number is printed rather than asserted tightly.
        Codec c(255, 190);
        Msg m;
        const char* hb = "fd090000072abf00000001000000020381040396a8";
        for (uint8_t b : {0x00, 0xFD, 0x11, 0x22, 0xFE, 0x99}) c.feed(b, m);
        int lost = 0;
        bool got = false;
        for (; lost < 8 && !got; ++lost)
            for (uint8_t b : unhex(hb)) if (c.feed(b, m)) got = true;
        check(got && m.id == MSG_HEARTBEAT && lost <= 3,
              "recovers after garbage, losing a bounded number of frames");
        std::printf("    a stray start byte cost %d frame(s) at 1 Hz heartbeat\n", lost);
    }
    {
        // An unknown message id is silently consumed and must NOT count as a
        // CRC error -- an autopilot streams dozens we never asked for.
        Codec c(255, 190);
        Msg m;
        uint8_t f[] = {0xFD, 0x02, 0, 0, 0, 1, 1, 0xEE, 0x00, 0x00, 0xAA, 0xBB, 0x12, 0x34};
        for (uint8_t b : f) c.feed(b, m);
        check(c.crcErrors() == 0 && c.framesRx() == 0, "unknown message id is not a CRC error");
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "all checks passed",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
