#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "control_types.hpp"
#include "flight_controller.hpp"

// FC link owned by its OWN thread — decouples flight-controller keep-alive from
// the fly loop. The fly loop can block (a slow/hung camera read) without the FC
// missing its RC deadline: iNAV failsafes below ~5 Hz MSP-RC, so servicing the
// link here, not behind cap.read(), is a safety requirement.
//
// The thread is the SOLE caller of the (non-thread-safe) IFlightController. The
// fly loop only touches this class's mutex-guarded intent:
//   command(cmd, live) — the control to send (live=false → dry-run, send nothing)
//   commandRth(live)   — failsafe: drive the FC's RTH, keep RC alive with neutral
//   feedGps / latchBaseline — marshalled one-shots run on the thread
//   telemetry()        — a snapshot of the latest poll
//
// KEY SAFETY PROPERTY: if the fly loop stops updating the command (it stalled),
// the thread keeps RC alive but substitutes a NEUTRAL hover — it never keeps
// repeating a stale *motion* command. Same freshness principle as the world
// model's perception gates.
class FcLink {
public:
    explicit FcLink(std::unique_ptr<IFlightController> fc, float staleCmdSec = 0.3f)
        : fc_(std::move(fc)), staleCmdSec_(staleCmdSec) {}
    ~FcLink() { stop(); }

    bool haveFc() const { return (bool)fc_; }

    // One-time backend setup; call before start().
    void configure(bool assist, int rthAuxIdx, int rthAuxUs) {
        if (!fc_) return;
        fc_->setAssistMode(assist);
        fc_->setRthChannel(rthAuxIdx, rthAuxUs);
    }

    void start();
    void stop();

    bool        linkUp() const;
    FcTelemetry telemetry() const;
    long        framesSent() const { return framesSent_.load(); }

    // --- fly-loop intent (thread-safe) ---
    void command(const ControlCmd& cmd, bool live);
    void commandRth(bool live);
    void feedGps(const ExtGps& g);
    void latchBaseline();

private:
    void loop_();

    std::unique_ptr<IFlightController> fc_;
    float staleCmdSec_;

    mutable std::mutex mu_;
    ControlCmd cmd_{};
    bool       live_      = false;
    bool       rth_       = false;
    double     cmdStampS_ = -1e9;
    bool       latchReq_  = false;
    bool       gpsReq_    = false;
    ExtGps     gps_{};
    FcTelemetry tel_{};

    std::atomic<bool> run_{false};
    std::atomic<long> framesSent_{0};
    std::thread       thr_;
};
