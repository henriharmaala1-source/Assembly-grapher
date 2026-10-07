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
#include <mutex>
#include <string>
#include <thread>

#include "fc_link.hpp"
#include "fc_odometry.hpp"
#include "sim_fc_backend.hpp"
#include "world_model.hpp"   // monoNowS

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// A sim FC that CAN carry proximity and vision, and counts what it was given.
struct ProxFc : SimFcBackend {
    std::atomic<int> calls{0}, visCalls{0};
    float first = 0.f, visN = 0.f;
    bool sendProximity(const float* d, int n) override {
        if (n > 0) first = d[0];
        ++calls; return true;
    }
    bool sendVisionOdometry(const VisionOdom& v) override {
        visN = v.n; ++visCalls; return true;
    }
    std::atomic<int> stCalls{0};
    std::mutex stMu;
    std::string stLast;
    bool sendStatusText(const char* t) override {
        std::lock_guard<std::mutex> lk(stMu);
        stLast = t; ++stCalls; return true;
    }
    const char* controlTag() const override { return "VEL"; }
};

int main() {
    auto sim = std::make_unique<SimFcBackend>();
    sim->connect("sim", 0);
    sim->setMode(FcMode::ALT_HOLD);      // the pilot's mode before any RTH
    SimFcBackend* simRaw = sim.get();
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

    // --- DRY-RUN: an RTH request changes NOTHING on the FC, mode included ---
    link.commandRth(false);
    sleep_ms(100);
    CHECK(simRaw->mode() == FcMode::ALT_HOLD);
    link.command([]{ ControlCmd c; c.valid = true; return c; }(), false);
    sleep_ms(60);
    CHECK(simRaw->mode() == FcMode::ALT_HOLD);
    std::printf("  dry-run RTH: FC mode untouched (ALT_HOLD)\n");

    // --- failsafe RTH keeps RC alive (live) ---
    link.commandRth(true);
    const long beforeRth = link.framesSent();
    sleep_ms(200);
    CHECK(link.framesSent() > beforeRth);            // frames flowing during RTH
    CHECK(simRaw->mode() == FcMode::RTL);

    // --- the RTH clears: the FC goes back to the PILOT'S mode ---
    // It used to be told ANGLE, which ArduPilot maps to STABILIZE: mid stick
    // half throttle, a script's "down" a power cut.
    link.command([]{ ControlCmd c; c.valid = true; return c; }(), true);
    sleep_ms(100);
    CHECK(simRaw->mode() == FcMode::ALT_HOLD);
    std::printf("  RTH cleared: FC resumed ALT_HOLD, not ANGLE/STABILIZE\n");

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
    {
        // OSD STATUS LINE: tag spliced in, sent on change, not every tick.
        auto pfc = std::make_unique<ProxFc>();
        pfc->connect("sim", 0);
        ProxFc* raw = pfc.get();
        FcLink sl(std::move(pfc), 0.3f);
        sl.setStatusRepeatS(0.5);
        sl.start();
        sl.status("SCRIPT LIVE", " GO DOOR");
        sleep_ms(300);                                 // ~15 loop ticks
        CHECK(raw->stCalls.load() == 1);
        { std::lock_guard<std::mutex> lk(raw->stMu);
          CHECK(raw->stLast == "SCRIPT LIVE VEL GO DOOR"); }
        sl.status("SCRIPT LIVE", " GO LAND");
        sleep_ms(60);
        CHECK(raw->stCalls.load() == 2);               // a change goes at once
        sleep_ms(600);
        CHECK(raw->stCalls.load() == 3);               // and is repeated slowly
        sl.stop();
        std::printf("  status line: \"SCRIPT LIVE VEL GO DOOR\", on change + every 0.5 s\n");
    }
    {
        // VIO into the FC: the same contract as proximity, capped at 30 Hz.
        auto pfc = std::make_unique<ProxFc>();
        pfc->connect("sim", 0);
        ProxFc* raw = pfc.get();
        FcLink vl(std::move(pfc), 0.3f);
        vl.start();
        VisionOdom v; v.n = 4.f;
        // A new pose every 5 ms for 500 ms (200 Hz): ~15 may go out.
        for (int k = 0; k < 100; ++k) { vl.vision(v, monoNowS()); sleep_ms(5); }
        const int sent = raw->visCalls.load();
        CHECK(sent >= 8 && sent <= 18);
        CHECK(std::fabs(raw->visN - 4.f) < 1e-6f);
        CHECK(vl.visionSent() == sent);
        sleep_ms(300);                               // producer stopped
        const int after = raw->visCalls.load();
        sleep_ms(300);
        CHECK(raw->visCalls.load() == after);        // a stale pose is not repeated
        vl.vision(v, monoNowS() - 1.0);              // stale on arrival
        sleep_ms(100);
        CHECK(raw->visCalls.load() == after);
        vl.stop();
        std::printf("  vision: %d sent for 100 offered in 0.5 s, none once stale\n", sent);
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
