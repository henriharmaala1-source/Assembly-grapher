// FcLink I/O thread against the SimFcBackend. Proves the F2 safety properties:
//   - RC frames keep flowing while the "fly loop" stalls (no new command) —
//     iNAV would otherwise failsafe below ~5 Hz.
//   - a STALE command is neutralised (the drone stops) rather than the last
//     motion command being repeated blind.
//   - dry-run (live=false) sends nothing.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

#include "fc_link.hpp"
#include "sim_fc_backend.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

int main() {
    auto sim = std::make_unique<SimFcBackend>();
    sim->connect("sim", 0);
    FcLink link(std::move(sim), 0.3f);   // 0.3 s command-staleness horizon
    link.start();

    sleep_ms(120);
    CHECK(link.linkUp());                            // thread polled telemetry

    // --- moving under a live forward command ---
    link.command([]{ ControlCmd c; c.valid = true; c.pitch = 0.3f; return c; }(), true);
    sleep_ms(250);
    const long sentMoving = link.framesSent();
    CHECK(sentMoving > 5);                           // ~50 Hz → >10 in 250 ms
    CHECK(link.telemetry().groundspeedMs > 0.5f);    // actually translating

    // --- fly loop STALLS: stop issuing commands for > staleCmdSec ---
    sleep_ms(500);
    const long sentStalled = link.framesSent();
    CHECK(sentStalled > sentMoving + 5);             // RC kept flowing (cadence!)
    CHECK(link.telemetry().groundspeedMs < 0.2f);    // neutralised — drone stopped

    // --- dry-run sends nothing ---
    link.command([]{ ControlCmd c; c.valid = true; c.pitch = 0.3f; return c; }(), false);
    const long before = link.framesSent();
    sleep_ms(200);
    CHECK(link.framesSent() == before);              // nothing sent while dry

    // --- failsafe RTH keeps RC alive (live) ---
    link.commandRth(true);
    const long beforeRth = link.framesSent();
    sleep_ms(200);
    CHECK(link.framesSent() > beforeRth);            // frames flowing during RTH

    link.stop();
    std::printf("test_fc_link: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
