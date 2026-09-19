// rc_probe — Pi bring-up harness, stage one: do RC/AUX signals reach the Pi?
//
//   rc_probe [--port /dev/ttyAMA0] [--baud 115200] [--secs 0]
//
// The whole flight architecture (`onboard/docs/MAVLINK_BRIDGE_PLAN.md` §4)
// rests on the pilot keeping a veto: the assist switch is what engages and
// disengages autonomy. Nothing else can be tested until the Pi can *see* that
// switch, so this is the first thing to bring up on real hardware and the first
// thing to check after any wiring change.
//
// WHAT THIS IS FOR, concretely: plug the Pi into the FC, run this, and flick
// each switch on the transmitter in turn. The tool tells you which channel
// moved and what microsecond values its detents produce — which is exactly the
// information `--assist-chan` and the RC_OPTION mapping need, and which is
// otherwise obtained by guessing and then misreading the result.
//
// It deliberately reuses the SHIPPING codec (`MavlinkBackend`) rather than
// parsing frames itself. A bring-up tool that speaks its own dialect proves
// only that the bring-up tool works. The one bug this codec has already had was
// in exactly this message — chancount read from the wrong offset, which on a
// 16-channel link silently reported zero channels — and it is now pinned by a
// golden frame. Sharing the code means this tool inherits that fix and any
// future one.
//
// This is a non-flight tool: nothing here runs on the aircraft.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "control_types.hpp"
#include "mavlink_backend.hpp"

namespace {

// Switch-position bands, in microseconds. RC switches land near 1000/1500/2000
// but no transmitter is exact and endpoints are commonly trimmed, so these are
// wide bands rather than equality tests. Anything outside them is reported as
// its raw value rather than being forced into a category.
const char* band(uint16_t us) {
    if (us == 0)          return "----";   // channel not present in this frame
    if (us <  1300)       return "LOW ";
    if (us <  1700)       return "MID ";
    return "HIGH";
}

struct Chan {
    uint16_t last = 0;
    uint16_t lo   = 0xFFFF;
    uint16_t hi   = 0;
    int      moves = 0;          // how many times this channel changed materially
    bool     movedRecently = false;
};

// A channel counts as having MOVED only past a deadband. Raw RC jitters by a
// few microseconds continuously -- without this every channel is always "live"
// and the display tells you nothing about which switch you just flicked.
constexpr int kMoveDeadbandUs = 25;

}  // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyAMA0";
    int baud = 115200;
    double secs = 0;                    // 0 = run until Ctrl-C

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", what);
                                 std::exit(2); }
            return argv[++i];
        };
        if      (a == "--port") port = next("--port");
        else if (a == "--baud") baud = std::stoi(next("--baud"));
        else if (a == "--secs") secs = std::stod(next("--secs"));
        else if (a == "-h" || a == "--help") {
            std::printf("rc_probe [--port %s] [--baud %d] [--secs 0]\n",
                        port.c_str(), baud);
            return 0;
        } else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); return 2; }
    }

    std::printf("rc_probe — connecting to %s @ %d\n", port.c_str(), baud);

    MavlinkBackend fc;
    if (!fc.connect(port, baud)) {
        std::fprintf(stderr,
            "\nFAILED to open %s.\n"
            "  * Is the UART enabled?  On a Pi 5 the primary UART is /dev/ttyAMA0\n"
            "    and needs `enable_uart=1` plus the console released from it\n"
            "    (remove console=serial0,115200 from /boot/firmware/cmdline.txt).\n"
            "  * Is the user in the `dialout` group?\n"
            "  * Check TX<->RX are CROSSED and grounds are common.\n",
            port.c_str());
        return 1;
    }

    Chan ch[18];
    long  frames = 0, rcFrames = 0;
    int   lastRcCount = -1;
    bool  everSawRc = false;

    const auto t0 = std::chrono::steady_clock::now();
    auto lastPrint = t0;
    auto lastRcAt  = t0;

    std::printf("connected. Flick each switch in turn — the channel that moves "
                "is the one you want.\n"
                "Ctrl-C to stop.\n\n");

    for (;;) {
        fc.tick();

        FcTelemetry t;
        const bool up = fc.poll(t);
        ++frames;

        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - t0).count();
        if (secs > 0 && elapsed >= secs) break;

        if (t.rcCount > 0) {
            if (!everSawRc) {
                std::printf("[ok] first RC_CHANNELS frame at %.1fs — %d channels\n\n",
                            elapsed, t.rcCount);
                everSawRc = true;
            }
            ++rcFrames;
            lastRcAt = now;
            lastRcCount = t.rcCount;

            for (int i = 0; i < t.rcCount && i < 18; ++i) {
                const uint16_t v = t.rc[i];
                if (v == 0) continue;
                if (ch[i].last && std::abs(int(v) - int(ch[i].last)) > kMoveDeadbandUs) {
                    ++ch[i].moves;
                    ch[i].movedRecently = true;
                }
                ch[i].last = v;
                ch[i].lo = std::min(ch[i].lo, v);
                ch[i].hi = std::max(ch[i].hi, v);
            }
        }

        // Redraw at a readable rate, not at link rate.
        if (std::chrono::duration<double>(now - lastPrint).count() < 0.25) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        lastPrint = now;

        const double rcAge = std::chrono::duration<double>(now - lastRcAt).count();

        std::printf("\033[H\033[J");     // home + clear; this is a live display
        std::printf("rc_probe  %s @ %d      t=%.1fs\n", port.c_str(), baud, elapsed);
        std::printf("link %s   armed %s   batt %.2fV   att r%+.0f p%+.0f y%.0f\n",
                    up ? "UP  " : "DOWN", t.armed ? "yes" : "no ",
                    t.battV, t.rollDeg, t.pitchDeg, t.yawDeg);
        std::printf("RC_CHANNELS frames %ld (%.1f Hz)   channels %d\n\n",
                    rcFrames, elapsed > 0 ? rcFrames / elapsed : 0.0, lastRcCount);

        if (!everSawRc) {
            // THE most common bring-up failure, and it looks like a dead link
            // when it is actually a silent one: the heartbeat arrives, so the
            // link reads healthy, but the RC stream was never requested.
            std::printf("  no RC_CHANNELS yet%s\n\n",
                        up ? " — link is UP, so this is a STREAM RATE problem" : "");
            std::printf("  On ArduPilot, set the RC channel stream rate on the\n"
                        "  serial port the Pi is wired to, e.g. for SERIAL2:\n"
                        "      SR2_RC_CHAN = 10        (Hz; 0 = never sent)\n"
                        "      SERIAL2_PROTOCOL = 2    (MAVLink2)\n"
                        "      SERIAL2_BAUD = %d\n"
                        "  then reboot the FC. No firmware change is needed.\n",
                        baud / 1000);
        } else {
            if (rcAge > 1.0)
                std::printf("  !! RC_CHANNELS stalled %.1fs ago\n\n", rcAge);

            std::printf("  ch      us   band   seen lo..hi    moves\n");
            std::printf("  ------------------------------------------\n");
            for (int i = 0; i < lastRcCount && i < 18; ++i) {
                const Chan& c = ch[i];
                if (!c.last) continue;
                std::printf("  %2d  %6u   %s   %4u..%-4u   %5d%s\n",
                            i + 1, c.last, band(c.last),
                            c.lo == 0xFFFF ? 0 : c.lo, c.hi, c.moves,
                            c.movedRecently ? "  <-- MOVED" : "");
            }
            std::printf("\n  Channels 1-4 are usually roll/pitch/throttle/yaw.\n"
                        "  A switch shows a small number of moves and a wide lo..hi;\n"
                        "  a stick shows many moves. The assist switch is the one\n"
                        "  that jumps between bands when you flick it.\n");
        }
        std::fflush(stdout);

        // Clear the transient marker so "MOVED" means "since the last redraw".
        for (auto& c : ch) c.movedRecently = false;
    }

    fc.disconnect();

    // Final summary — the part worth copying into the config.
    std::printf("\n--- summary over %.1fs ---\n",
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count());
    if (!everSawRc) {
        std::printf("NO RC DATA. See the stream-rate note above.\n");
        return 1;
    }
    std::printf("  ch     lo    hi   span   moves   likely\n");
    for (int i = 0; i < lastRcCount && i < 18; ++i) {
        const Chan& c = ch[i];
        if (!c.last) continue;
        const int span = int(c.hi) - int(c.lo == 0xFFFF ? c.hi : c.lo);
        // A stick sweeps continuously and racks up many changes; a switch has a
        // wide span but few transitions. That ratio is what separates them, and
        // it is the question the operator actually has.
        const char* kind = span < 50   ? "fixed"
                         : c.moves > 40 ? "stick"
                         : "SWITCH";
        std::printf("  %2d  %5u %5u  %5d   %5d   %s\n",
                    i + 1, c.lo, c.hi, span, c.moves, kind);
    }
    std::printf("\nUse the SWITCH channel number as the assist channel.\n");
    return 0;
}
