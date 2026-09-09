// fake_fc — a fake ArduPilot on a pseudo-terminal, for developing the Pi
// harness without hardware.
//
//   fake_fc [seconds]        # prints the PTY path on stdout, then streams
//
// Emits HEARTBEAT + RC_CHANNELS at 10 Hz with a known pattern: channel 1 sweeps
// like a stick, channel 7 flicks between two detents like an aux switch, and
// the rest sit at centre. That makes it an EXPECTED ANSWER, not just a signal
// source — `rc_probe` should classify ch1 as a stick and ch7 as a SWITCH, and
// if it does not, the harness is wrong rather than the wiring.
//
// The repo already tests MSP over a PTY; this is the MAVLink equivalent, and it
// exists so that every later bring-up stage (ATTITUDE decode, the command path)
// can be written and debugged on a desk.
//
// Non-flight tool. Nothing here runs on the aircraft.
#include <pty.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>

#include "mavlink_v2.hpp"

int main(int argc, char** argv) {
    const double secs = argc > 1 ? atof(argv[1]) : 6.0;
    int m, s; char name[256];
    if (openpty(&m, &s, name, nullptr, nullptr) < 0) { perror("openpty"); return 1; }
    std::printf("%s\n", name); std::fflush(stdout);

    mav::Codec cx(1, 1);
    uint8_t buf[300];
    const auto t0 = std::chrono::steady_clock::now();

    for (;;) {
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (t >= secs) break;

        {   // HEARTBEAT: type 2 (quad), autopilot 3 (ArduPilot), custom mode 0
            mav::Payload p; p.u32(0); p.u8(2); p.u8(3); p.u8(0); p.u8(4); p.u8(3);
            int n = cx.frame(mav::MSG_HEARTBEAT, p, buf);
            if (n && write(m, buf, n) < 0) return 1;
        }
        {   // RC_CHANNELS: time_boot_ms, chancount is a u8 AFTER all 18 channels.
            mav::Payload p;
            p.u32(uint32_t(t * 1000));
            const uint16_t stick = uint16_t(1500 + 480 * std::sin(t * 2.0));
            const uint16_t sw    = (int(t) / 2) % 2 ? 1900 : 1100;  // flicks
            for (int i = 0; i < 18; ++i) {
                uint16_t v = 1500;
                if (i == 0) v = stick;
                else if (i == 6) v = sw;      // channel 7
                else if (i == 4) v = 1500;    // channel 5, fixed
                p.u16(v);
            }
            p.u8(8);     // chancount = 8
            p.u8(100);   // rssi
            int n = cx.frame(mav::MSG_RC_CHANNELS, p, buf);
            if (n && write(m, buf, n) < 0) return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    close(m); close(s);
    return 0;
}
