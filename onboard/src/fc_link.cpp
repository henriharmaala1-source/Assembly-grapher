#include "fc_link.hpp"

#include <chrono>

#include "world_model.hpp"   // monoNowS()

void FcLink::start() {
    if (!fc_ || run_.load()) return;
    run_.store(true);
    thr_ = std::thread(&FcLink::loop_, this);
}

void FcLink::stop() {
    run_.store(false);
    if (thr_.joinable()) thr_.join();
    if (fc_) fc_->disconnect();
}

bool FcLink::linkUp() const {
    std::lock_guard<std::mutex> lk(mu_);
    return tel_.linkUp;
}

FcTelemetry FcLink::telemetry() const {
    std::lock_guard<std::mutex> lk(mu_);
    return tel_;
}

void FcLink::command(const ControlCmd& cmd, bool live) {
    std::lock_guard<std::mutex> lk(mu_);
    cmd_ = cmd; live_ = live; rth_ = false; cmdStampS_ = monoNowS();
}

void FcLink::commandRth(bool live) {
    std::lock_guard<std::mutex> lk(mu_);
    rth_ = true; live_ = live; cmdStampS_ = monoNowS();
}

void FcLink::feedGps(const ExtGps& g) {
    std::lock_guard<std::mutex> lk(mu_);
    gps_ = g; gpsReq_ = true;
}

void FcLink::latchBaseline() {
    std::lock_guard<std::mutex> lk(mu_);
    latchReq_ = true;
}

void FcLink::loop_() {
    using namespace std::chrono;
    bool lastRth = false, rthCmding = false;

    while (run_.load()) {
        const auto t0 = steady_clock::now();

        // Snapshot the fly loop's intent under the lock.
        ControlCmd cmd; bool live, rth; double stamp;
        bool doLatch, doGps; ExtGps g;
        {
            std::lock_guard<std::mutex> lk(mu_);
            cmd = cmd_; live = live_; rth = rth_; stamp = cmdStampS_;
            doLatch = latchReq_; latchReq_ = false;
            doGps   = gpsReq_;   gpsReq_   = false;  g = gps_;
        }

        // Service the link (RX drain + telemetry polls) and publish telemetry.
        fc_->tick();
        FcTelemetry t; fc_->poll(t);
        { std::lock_guard<std::mutex> lk(mu_); tel_ = t; }

        // Marshalled one-shots.
        if (doLatch) fc_->latchBaseline();
        if (doGps)   fc_->feedExternalGps(g);

        // Mode latch only on transition (some backends send on setMode).
        if (rth != lastRth) {
            rthCmding = fc_->setMode(rth ? FcMode::RTL : FcMode::ANGLE) && rth;
            lastRth = rth;
        }

        // Emit control (dry-run sends nothing).
        if (live) {
            const double age = monoNowS() - stamp;
            if (rth) {
                // Failsafe: keep RC alive with neutral so the RTH AUX is driven.
                if (rthCmding) { ControlCmd hold; hold.valid = true;
                                 if (fc_->sendControl(hold)) framesSent_.fetch_add(1); }
            } else if (cmd.valid && age <= staleCmdSec_) {
                if (fc_->sendControl(cmd)) framesSent_.fetch_add(1);
            } else {
                // Stale command (fly loop stalled) → neutral hover, not a repeat
                // of the last motion command. RC stays alive; the aircraft holds.
                ControlCmd hold; hold.valid = true;
                if (fc_->sendControl(hold)) framesSent_.fetch_add(1);
            }
        }

        // Pace ~50 Hz regardless of how long the work took.
        const auto spent = steady_clock::now() - t0;
        const auto period = milliseconds(20);
        if (spent < period) std::this_thread::sleep_for(period - spent);
    }
}
