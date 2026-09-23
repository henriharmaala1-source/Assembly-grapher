// FcLink I/O thread against the SimFcBackend. Proves the F2 safety properties:
//   - RC frames keep flowing while the "fly loop" stalls (no new command) —
//     iNAV would otherwise failsafe below ~5 Hz.
//   - a STALE command is neutralised (the drone stops) rather than the last
//     motion command being repeated blind.
//   - dry-run (live=false) sends nothing.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <atomic>
#include <memory>
#include <thread>

#include "fc_link.hpp"
#include "fc_odometry.hpp"
#include "sim_fc_backend.hpp"
#include "world_model.hpp"   // monoNowS

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// A sim FC that CAN carry proximity, and counts what it was given.
struct ProxFc : SimFcBackend {
    std::atomic<int> calls{0};
    float first = 0.f;
    bool sendProximity(const float* d, int n) override {
        if (n > 0) first = d[0];
        ++calls; return true;
    }
};

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

    // --- PROXIMITY: forwarded fresh, at <= 10 Hz, and NEVER repeated stale ---
    {
        auto pfc = std::make_unique<ProxFc>();
        pfc->connect("sim", 0);
        ProxFc* raw = pfc.get();
        FcLink pl(std::move(pfc), 0.3f);
        pl.start();
        float d[72];
        for (int i = 0; i < 72; ++i) d[i] = -1.f;
        d[0] = 2.5f;
        // A new set every 20 ms for 500 ms: at most ~5-6 may go out (10 Hz cap).
        for (int k = 0; k < 25; ++k) { pl.proximity(d, 72, monoNowS()); sleep_ms(20); }
        const int sent = raw->calls.load();
        CHECK(sent >= 3 && sent <= 7);
        CHECK(std::fabs(raw->first - 2.5f) < 1e-6f);
        CHECK(pl.proximitySent() == sent);
        // The producer dies: nothing more may be sent, however long we wait.
        sleep_ms(400);
        const int after = raw->calls.load();
        sleep_ms(400);
        CHECK(raw->calls.load() == after);
        // A set that is ALREADY stale when handed over is not sent at all.
        pl.proximity(d, 72, monoNowS() - 2.0);
        sleep_ms(200);
        CHECK(raw->calls.load() == after);
        pl.stop();
        std::printf("  proximity: %d sent for 25 offered in 0.5 s, none after the "
                    "producer stopped\n", sent);
    }

    // --- FC local position as the displacement source: only when EKF3 vouches ---
    {
        FcTelemetry t;
        t.localValid = true; t.localN = 3.f; t.localE = -1.f; t.localD = -2.f;
        t.localVn = 0.3f; t.localVe = 0.4f;
        t.ekfValid = true; t.ekfFlags = kEkfPosHorizRel;
        const double now = 100.0;
        t.localStampS = now - 0.1;
        WorldState s;
        CHECK(fcLocalEstimate(t, now, s));
        CHECK(s.estValid && s.estPe == -1.f && s.estPn == 3.f && s.estPu == 2.f);
        CHECK(std::fabs(s.estSpeed - 0.5f) < 1e-5f);
        WorldState s2;
        FcTelemetry u = t; u.ekfFlags = kEkfPosHorizRel | kEkfConstPosMode;
        CHECK(!fcLocalEstimate(u, now, s2) && !s2.estValid);   // no source
        u = t; u.ekfFlags = 0;
        CHECK(!fcLocalEstimate(u, now, s2) && !s2.estValid);   // not vouched
        u = t; u.localStampS = now - 1.0;
        CHECK(!fcLocalEstimate(u, now, s2) && !s2.estValid);   // stale
        u = t; u.ekfValid = false;
        CHECK(!fcLocalEstimate(u, now, s2) && !s2.estValid);   // no EKF report
        std::printf("  FC local position: used only when EKF3 vouches, fresh\n");
    }

    std::printf("test_fc_link: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
