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
    void configure(bool assist, int rthAuxIdx, int rthAuxUs, int battCells = 0) {
        if (!fc_) return;
        fc_->setAssistMode(assist);
        fc_->setRthChannel(rthAuxIdx, rthAuxUs);
        fc_->setBatteryCells(battCells);
    }

    // Real-time scheduling for the I/O thread (F9); applied when the thread
    // starts. prio<=0 leaves normal priority; cpu>=0 pins. Call before start().
    void setRealtime(int prio, int cpu) { rtPrio_ = prio; rtCpu_ = cpu; }

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
    // PROXIMITY for the FC's own avoidance (OBSTACLE_DISTANCE on MAVLink). The
    // I/O thread forwards the LATEST set at <= 10 Hz while it is fresher than
    // proxStaleSec; a stale set is not repeated -- ArduPilot then times the
    // sensor out, which is the correct failure, rather than trusting a frozen
    // picture of the world. Counted, so a log can show it was really sent.
    void proximity(const float* distM, int n, double stampS);
    long proximitySent() const { return proxSent_.load(); }
    // VISUAL ODOMETRY for the FC's estimator. The same contract as proximity:
    // the LATEST pose, at <= 30 Hz, once, and only while fresher than
    // visionStaleSec -- a stale pose is not repeated; EKF3 then times the
    // source out and falls back, which is the correct failure.
    void vision(const VisionOdom& v, double stampS);
    long visionSent() const { return visSent_.load(); }

private:
    void loop_();

    std::unique_ptr<IFlightController> fc_;
    float staleCmdSec_;
    int   rtPrio_ = 0, rtCpu_ = -1;   // RT scheduling for loop_ (F9)

    mutable std::mutex mu_;
    ControlCmd cmd_{};
    bool       live_      = false;
    bool       rth_       = false;
    double     cmdStampS_ = -1e9;
    bool       latchReq_  = false;
    bool       gpsReq_    = false;
    ExtGps     gps_{};
    float      prox_[72] = {};
    int        proxN_    = 0;
    double     proxStampS_ = -1e9, proxSentStampS_ = -1e9, proxLastTxS_ = -1e9;
    float      proxStaleSec_ = 0.5f;
    VisionOdom vis_{};
    double     visStampS_ = -1e9, visSentStampS_ = -1e9, visLastTxS_ = -1e9;
    float      visStaleSec_ = 0.2f;
    FcTelemetry tel_{};

    std::atomic<bool> run_{false};
    std::atomic<long> framesSent_{0};
    std::atomic<long> proxSent_{0};
    std::atomic<long> visSent_{0};
    std::thread       thr_;
};
