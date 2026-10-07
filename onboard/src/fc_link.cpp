#include "fc_link.hpp"

#include <cstdio>
#include <cstring>

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
    cmd_ = cmd; live_ = live; rth_ = false; land_ = false; cmdStampS_ = monoNowS();
}

void FcLink::commandRth(bool live) {
    std::lock_guard<std::mutex> lk(mu_);
    rth_ = true; land_ = false; live_ = live; cmdStampS_ = monoNowS();
}

void FcLink::commandLand(bool live) {
    std::lock_guard<std::mutex> lk(mu_);
    land_ = true; rth_ = false; live_ = live; cmdStampS_ = monoNowS();
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

void FcLink::status(const char* head, const char* tail) {
    std::lock_guard<std::mutex> lk(mu_);
    std::snprintf(stHead_, sizeof(stHead_), "%s", head ? head : "");
    std::snprintf(stTail_, sizeof(stTail_), "%s", tail ? tail : "");
}

void FcLink::loop_() {
    using namespace std::chrono;
    // Elevate this thread so inference on the Deliberator can't delay RC — the
    // hard deadline (iNAV failsafes below ~5 Hz). Non-fatal if not permitted.
    if (rtPrio_ > 0 || rtCpu_ >= 0) rt::make_realtime("fclink", rtPrio_, rtCpu_);
    // The FC mode this link is holding the aircraft in: RTL or LAND (handed to
    // the FC), or UNKNOWN = none (the OS's sticks).
    FcMode lastSpecial = FcMode::UNKNOWN;
    bool rthCmding = false;

    while (run_.load()) {
        const auto t0 = steady_clock::now();

        // Snapshot the fly loop's intent under the lock.
        ControlCmd cmd; bool live, rth, land; double stamp;
        bool doLatch, doGps; ExtGps g;
        {
            std::lock_guard<std::mutex> lk(mu_);
            cmd = cmd_; live = live_; rth = rth_; land = land_; stamp = cmdStampS_;
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

        // Status line: the backend's control tag goes in the middle, so the
        // OSD says how commands reach the FC in the mode it is in now.
        {
            char line[80]; bool doSt = false;          // cut to STATUSTEXT's 50 below
            {
                std::lock_guard<std::mutex> lk(mu_);
                if (stHead_[0]) {
                    const char* tag = fc_->controlTag();
                    std::snprintf(line, sizeof(line), "%s%s%s%s", stHead_, tag[0] ? " " : "",
                                  tag, stTail_);
                    line[50] = '\0';
                    const double now = monoNowS();
                    if (std::strcmp(line, stSent_) != 0 || now - stLastTxS_ >= statusRepeatS_) {
                        std::memcpy(stSent_, line, sizeof(stSent_));   // 51 incl. NUL
                        stLastTxS_ = now;
                        doSt = true;
                    }
                }
            }
            if (doSt && fc_->sendStatusText(line)) statusSent_.fetch_add(1);
        }

        // Marshalled one-shots.
        if (doLatch) fc_->latchBaseline();
        if (doGps)   fc_->feedExternalGps(g);

        // Mode latch only on transition (some backends send on setMode). When
        // the request clears, RESUME: give back the mode the aircraft was in
        // before, never a fixed one -- on ArduPilot "ANGLE" was STABILIZE.
        // DRY-RUN SENDS NOTHING -- that includes a mode change. A script's
        // `land`, or the low-battery failsafe, in a dry run used to switch the
        // real aircraft to LAND / RTL here, since only the sticks were gated.
        const FcMode special = !live ? FcMode::UNKNOWN
                             : rth ? FcMode::RTL : land ? FcMode::LAND : FcMode::UNKNOWN;
        if (special != lastSpecial) {
            const bool on = special != FcMode::UNKNOWN;
            rthCmding = fc_->setMode(on ? special : FcMode::RESUME) && on;
            lastSpecial = special;
        }
        rth = rth || land;                 // both: the FC flies, RC kept alive

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
