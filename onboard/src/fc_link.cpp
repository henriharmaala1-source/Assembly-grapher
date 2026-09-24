#include "fc_link.hpp"

#include <algorithm>
#include <chrono>

#include "realtime.hpp"
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

void FcLink::proximity(const float* distM, int n, double stampS) {
    std::lock_guard<std::mutex> lk(mu_);
    proxN_ = std::max(0, std::min(72, n));
    for (int i = 0; i < proxN_; ++i) prox_[i] = distM[i];
    proxStampS_ = stampS;
}

void FcLink::vision(const VisionOdom& v, double stampS) {
    std::lock_guard<std::mutex> lk(mu_);
    vis_ = v;
    visStampS_ = stampS;
}

void FcLink::loop_() {
    using namespace std::chrono;
    // Elevate this thread so inference on the Deliberator can't delay RC — the
    // hard deadline (iNAV failsafes below ~5 Hz). Non-fatal if not permitted.
    if (rtPrio_ > 0 || rtCpu_ >= 0) rt::make_realtime("fclink", rtPrio_, rtCpu_);
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

        // Proximity: the newest set, once, at <= 10 Hz, and only while fresh.
        {
            float d[72]; int n = 0; bool doProx = false;
            {
                std::lock_guard<std::mutex> lk(mu_);
                const double now = monoNowS();
                if (proxN_ > 0 && proxStampS_ > proxSentStampS_ &&
                    now - proxStampS_ <= proxStaleSec_ && now - proxLastTxS_ >= 0.1) {
                    n = proxN_;
                    for (int i = 0; i < n; ++i) d[i] = prox_[i];
                    proxSentStampS_ = proxStampS_;
                    proxLastTxS_ = now;
                    doProx = true;
                }
            }
            if (doProx && fc_->sendProximity(d, n)) proxSent_.fetch_add(1);
        }
        // Vision odometry: the same rule, at <= 30 Hz.
        {
            VisionOdom v; bool doVis = false;
            {
                std::lock_guard<std::mutex> lk(mu_);
                const double now = monoNowS();
                if (visStampS_ > visSentStampS_ && now - visStampS_ <= visStaleSec_ &&
                    now - visLastTxS_ >= 1.0 / 30.0) {
                    v = vis_;
                    visSentStampS_ = visStampS_;
                    visLastTxS_ = now;
                    doVis = true;
                }
            }
            if (doVis && fc_->sendVisionOdometry(v)) visSent_.fetch_add(1);
        }

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
