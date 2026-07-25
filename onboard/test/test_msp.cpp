// MSP backend framing over a PTY — no hardware. The test process is the "FC":
// it owns the master end, the backend opens the slave like a serial device.
// Covers: MSP v1 control framing (AETR channel order! throttle=ch2), the XOR
// checksum, telemetry decode (ATTITUDE), link-up tracking, and the F5
// ATTITUDE-priority poll schedule (no two consecutive non-ATTITUDE requests).

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "msp_backend.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

// Drain whatever the backend has written to the master end (with a short wait).
std::vector<uint8_t> drainMaster(int mfd, int waitMs = 30) {
    std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
    std::vector<uint8_t> out;
    uint8_t buf[512];
    for (;;) {
        const ssize_t n = read(mfd, buf, sizeof(buf));
        if (n <= 0) break;
        out.insert(out.end(), buf, buf + n);
    }
    return out;
}

// Parse '$M<' request/command frames out of a byte stream → list of cmd ids.
std::vector<uint8_t> parseCmds(const std::vector<uint8_t>& b) {
    std::vector<uint8_t> cmds;
    for (size_t i = 0; i + 5 < b.size() + 1 && i + 4 < b.size(); ++i) {
        if (b[i] == '$' && b[i + 1] == 'M' && b[i + 2] == '<') {
            cmds.push_back(b[i + 4]);
            i += 4 + b[i + 3] + 1;   // skip size, cmd, payload, crc
        }
    }
    return cmds;
}

}  // namespace

int main() {
    int mfd = -1, sfd = -1;
    char sname[128];
    CHECK(openpty(&mfd, &sfd, sname, nullptr, nullptr) == 0);
    fcntl(mfd, F_SETFL, O_NONBLOCK);

    MspBackend fc;
    CHECK(fc.connect(sname, 115200));

    { // SAFETY: with no operator RC seen and no baseline, the AUX (arm/mode
      // switch) positions are unknown. Synthesising them would write the arm
      // channel — disarming the aircraft in flight — so the backend must refuse
      // to send at all rather than invent switch values.
        ControlCmd c; c.valid = true; c.roll = 0.5f;
        CHECK(!fc.sendControl(c));                   // refused: AUX unknown
        CHECK(drainMaster(mfd).empty());             // nothing on the wire
    }

    { // Feed operator RC (MSP_RC) so the backend knows the real switch positions.
      // AUX1 (index 4) = 1800 µs stands in for a HIGH arm switch.
        const uint16_t opRc[8] = {1500, 1500, 1200, 1500, 1800, 1000, 1700, 1000};
        uint8_t p[16];
        for (int i = 0; i < 8; ++i) { p[i*2] = opRc[i] & 0xFF; p[i*2+1] = opRc[i] >> 8; }
        uint8_t f[32]; int k = 0;
        f[k++] = '$'; f[k++] = 'M'; f[k++] = '>'; f[k++] = 16; f[k++] = 105;  // MSP_RC
        uint8_t crc = 16 ^ 105;
        for (int i = 0; i < 16; ++i) { f[k++] = p[i]; crc ^= p[i]; }
        f[k++] = crc;
        CHECK(write(mfd, f, k) == k);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        fc.tick();                                   // drain + decode
        FcTelemetry t{}; fc.poll(t);
        CHECK(t.rcCount >= 8);
        CHECK(t.rc[4] == 1800);
        drainMaster(mfd);                            // discard telemetry polls
    }

    { // control frame: v1 header, AETR order, µs mapping, XOR checksum
        ControlCmd c;
        c.valid = true; c.roll = 0.5f; c.pitch = -0.2f; c.yaw = 1.0f; c.throttle = 0.f;
        CHECK(fc.sendControl(c));

        const auto b = drainMaster(mfd);
        CHECK(b.size() == 22);                       // 5 hdr + 16 payload + crc
        if (b.size() == 22) {
            CHECK(b[0] == '$' && b[1] == 'M' && b[2] == '<');
            CHECK(b[3] == 16);                       // payload size
            CHECK(b[4] == 200);                      // MSP_SET_RAW_RC
            auto ch = [&](int i) { return (uint16_t)(b[5 + 2 * i] | (b[6 + 2 * i] << 8)); };
            CHECK(ch(0) == 1750);                    // A: roll +0.5
            CHECK(ch(1) == 1400);                    // E: pitch -0.2
            CHECK(ch(2) == 1500);                    // T at index 2 (AETR!) 0 = hold
            CHECK(ch(3) == 2000);                    // R: yaw +1.0
            // AUX must MIRROR the operator's live switches, never be synthesised:
            // writing 1000 here pulls a high arm switch low = disarm in flight.
            CHECK(ch(4) == 1800);                    // arm switch preserved
            CHECK(ch(5) == 1000);
            CHECK(ch(6) == 1700);                    // mode switch preserved
            CHECK(ch(7) == 1000);
            uint8_t crc = 0;
            for (size_t i = 3; i < 21; ++i) crc ^= b[i];
            CHECK(crc == b[21]);
        }
        ControlCmd inv;                              // valid=false must not send
        CHECK(!fc.sendControl(inv));
        CHECK(drainMaster(mfd).empty());
    }

    { // telemetry decode: feed an ATTITUDE reply, poll() reflects it
        const int16_t roll10 = 123, pitch10 = -45, yaw = 90;   // 12.3°, -4.5°, 90°
        uint8_t p[6] = {(uint8_t)(roll10 & 0xFF),  (uint8_t)((roll10 >> 8) & 0xFF),
                        (uint8_t)(pitch10 & 0xFF), (uint8_t)((pitch10 >> 8) & 0xFF),
                        (uint8_t)(yaw & 0xFF),     (uint8_t)((yaw >> 8) & 0xFF)};
        uint8_t f[16]; int k = 0;
        f[k++] = '$'; f[k++] = 'M'; f[k++] = '>'; f[k++] = 6; f[k++] = 108;
        uint8_t crc = 6 ^ 108;
        for (int i = 0; i < 6; ++i) { f[k++] = p[i]; crc ^= p[i]; }
        f[k++] = crc;
        CHECK(write(mfd, f, k) == k);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        fc.tick();                                   // drain + decode

        FcTelemetry t{};
        CHECK(fc.poll(t));
        CHECK(std::fabs(t.rollDeg - 12.3f) < 0.01f);
        CHECK(std::fabs(t.pitchDeg + 4.5f) < 0.01f);
        CHECK(std::fabs(t.yawDeg - 90.f) < 0.01f);
        CHECK(t.linkUp);
    }

    { // F5: ATTITUDE-priority interleave — never two non-ATTITUDE in a row
        drainMaster(mfd);                            // discard tick()'s first poll
        for (int i = 0; i < 12; ++i) {               // ride the 20 ms poll gate
            fc.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        const auto cmds = parseCmds(drainMaster(mfd));
        CHECK(cmds.size() >= 10);
        int attitude = 0;
        for (size_t i = 0; i + 1 < cmds.size(); ++i)
            CHECK(cmds[i] == 108 || cmds[i + 1] == 108);   // alternation property
        for (uint8_t c : cmds) attitude += (c == 108);
        CHECK(attitude * 2 >= (int)cmds.size() - 1);       // ~every other slot
    }

    { // P2.1: RTH via AUX — setMode(RTL) latches the channel high in RC frames
        drainMaster(mfd);
        CHECK(!fc.setMode(FcMode::RTL));             // unconfigured → false (release)

        fc.setRthChannel(6, 1850);                   // AUX3 = channel index 6
        CHECK(fc.setMode(FcMode::RTL));              // now latches
        ControlCmd hold; hold.valid = true;          // neutral sticks
        CHECK(fc.sendControl(hold));
        {
            const auto b = drainMaster(mfd);
            CHECK(b.size() == 22);
            if (b.size() == 22) {
                auto ch = [&](int i) { return (uint16_t)(b[5 + 2 * i] | (b[6 + 2 * i] << 8)); };
                CHECK(ch(0) == 1500 && ch(1) == 1500);   // sticks neutral
                CHECK(ch(6) == 1850);                    // RTH AUX driven high
                // Other AUX still mirror the operator — crucially the ARM switch
                // stays high, so the aircraft remains armed to fly the return.
                CHECK(ch(4) == 1800 && ch(5) == 1000);
            }
        }
        CHECK(fc.setMode(FcMode::ANGLE));            // any other mode releases it
        CHECK(fc.sendControl(hold));
        {
            const auto b = drainMaster(mfd);
            if (b.size() == 22) {
                auto ch = [&](int i) { return (uint16_t)(b[5 + 2 * i] | (b[6 + 2 * i] << 8)); };
                CHECK(ch(6) == 1700);                    // back to the operator's switch
            }
        }
    }

    { // Battery state-of-charge must be PER CELL. A fixed 3S divisor reads ~1.00
      // all the way down on a 4S/6S pack, so the low-battery → RTH failsafe
      // (ModeManager: vehBattery < rtl_batt_pct) could never fire.
        auto sendAnalog = [&](float volts) {
            const uint8_t p[7] = {(uint8_t)std::lround(volts * 10.f), 0, 0, 0, 0, 0, 0};
            uint8_t f[16]; int k = 0;
            f[k++] = '$'; f[k++] = 'M'; f[k++] = '>'; f[k++] = 7; f[k++] = 110;  // MSP_ANALOG
            uint8_t crc = 7 ^ 110;
            for (int i = 0; i < 7; ++i) { f[k++] = p[i]; crc ^= p[i]; }
            f[k++] = crc;
            CHECK(write(mfd, f, k) == k);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            fc.tick();
            FcTelemetry t{}; fc.poll(t);
            drainMaster(mfd);
            return t.battPct;
        };

        MspBackend fc6;                              // explicit 6S pack
        int m6 = -1, s6 = -1; char n6[128];
        CHECK(openpty(&m6, &s6, n6, nullptr, nullptr) == 0);
        fcntl(m6, F_SETFL, O_NONBLOCK);
        CHECK(fc6.connect(n6, 115200));
        fc6.setBatteryCells(6);
        {
            const uint8_t p[7] = {(uint8_t)std::lround(21.0f * 10.f), 0, 0, 0, 0, 0, 0};
            uint8_t f[16]; int k = 0;
            f[k++] = '$'; f[k++] = 'M'; f[k++] = '>'; f[k++] = 7; f[k++] = 110;
            uint8_t crc = 7 ^ 110;
            for (int i = 0; i < 7; ++i) { f[k++] = p[i]; crc ^= p[i]; }
            f[k++] = crc;
            CHECK(write(m6, f, k) == k);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            fc6.tick();
            FcTelemetry t{}; fc6.poll(t);
            // 21.0 V / 6 = 3.50 V/cell → (3.50-3.3)/0.9 ≈ 0.22, i.e. "land now",
            // NOT the 1.00 the old fixed-3S maths reported.
            CHECK(t.battPct > 0.15f && t.battPct < 0.30f);
        }
        fc6.disconnect(); close(m6); close(s6);

        // Unconfigured (cells=0) → inferred from the first reading. 22.2 V ≈ 6S
        // → 3.70 V/cell → (3.70-3.3)/0.9 ≈ 0.44 (nominal LiPo, mid-pack).
        // The old fixed-3S maths saturated at 1.00 here, hiding the whole
        // discharge curve from the failsafe.
        const float pct = sendAnalog(22.2f);
        CHECK(pct > 0.35f && pct < 0.55f);
        CHECK(pct < 1.0f);
    }

    fc.disconnect();
    close(mfd); close(sfd);

    std::printf("test_msp: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
