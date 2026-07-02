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
            for (int i = 4; i < 8; ++i) CHECK(ch(i) == 1000);   // AUX low
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

    fc.disconnect();
    close(mfd); close(sfd);

    std::printf("test_msp: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
