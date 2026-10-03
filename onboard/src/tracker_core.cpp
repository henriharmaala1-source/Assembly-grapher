#include "tracker_core.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "gray_frame.hpp"
#include "lock_tracker_fused.hpp"

namespace track {
namespace {

// --- the tracker that currently flies ------------------------------------
class LegacyCore : public ITrackerCore {
public:
    explicit LegacyCore(Backend b) : backend_(b) {}
    const char* name() const override { return backend_name(backend_); }
    void reset() override { trk_.reset(); haveFix_ = false; }

    void designate(const cv::Mat& frame, cv::Point c, int boxSize) override {
        trk_.init(frame, c, backend_, boxSize);
        haveFix_ = false;
    }

    TrackObs update(const cv::Mat& frame, double captureSec) override {
        if (trk_.hasTarget()) trk_.update(frame);
        TrackObs o;
        o.core = name();
        o.captureSec = captureSec;
        // THE TWO CORES MEANT DIFFERENT THINGS BY "LOCKED", and translating
        // that is the adapter's whole job. LockOnTracker::locked() stays TRUE
        // through a loss -- coasting() is defined as lossFrames_ > 0 && locked_
        // -- so it means "has a target", while the fused core's LOCKED means
        // "is seeing it right now". Mapped naively, this core reported a
        // current fix through twelve frames of blank grey, with an age of 0.0 s,
        // and any consumer switched between the cores would have been fed two
        // different claims under one name. The interface's definition is the
        // strict one: locked means SEEN.
        o.valid    = trk_.hasTarget();
        o.coasting = trk_.coasting();
        o.locked   = trk_.locked() && !o.coasting;
        o.box      = trk_.bbox();
        o.conf     = trk_.confidence();
        o.age      = trk_.age();
        o.losses   = trk_.totalLosses();
        o.state    = o.locked ? "LOCKED" : o.coasting ? "COASTING"
                   : o.valid  ? "TRACKING" : "IDLE";
        // PIXELS PER SECOND. projected() is in the legacy tracker's own
        // per-step units, so one step is converted with the frame interval
        // this call actually observed rather than an assumed 30.
        const cv::Point ctr = (o.box.tl() + o.box.br()) / 2;
        const double dt = haveFix_ ? std::max(1e-3, captureSec - tPrev_) : 1.0 / 30.0;
        if (o.locked)
            o.vel = (trk_.projected(1.f) - cv::Point2f(ctr)) * float(1.0 / dt);
        if (o.locked) { tFix_ = captureSec; haveFix_ = true; }
        o.ageOfFixSec = haveFix_ ? float(captureSec - tFix_) : -1.f;
        tPrev_ = captureSec;
        return o;
    }

private:
    LockOnTracker trk_;
    Backend backend_;
    double  tPrev_ = 0.0, tFix_ = 0.0;
    bool    haveFix_ = false;
};

// --- the fused tracker, which is what the tests exercise ------------------
class FusedCore : public ITrackerCore {
public:
    const char* name() const override { return "fused"; }
    void reset() override { trk_.reset(); }

    void designate(const cv::Mat& frame, cv::Point c, int boxSize) override {
        const GrayFrame g = toGray(frame);
        trk_.designate(g, float(c.x), float(c.y), float(boxSize));
    }

    TrackObs update(const cv::Mat& frame, double captureSec) override {
        const GrayFrame g = toGray(frame);
        const LockTracker::Result r = trk_.update(g, captureSec);
        TrackObs o;
        o.core = name();
        o.captureSec = captureSec;
        o.state = LockTracker::stateName(r.state);
        o.locked   = (r.state == LockTracker::State::LOCKED);
        o.coasting = (r.state == LockTracker::State::COASTING
                   || r.state == LockTracker::State::SEARCHING);
        o.valid    = o.locked || o.coasting;
        o.box      = cv::Rect(r.x, r.y, r.w, r.h);
        o.conf     = r.conf;
        o.ageOfFixSec = trk_.ageOfFixSec();
        // predX/predY lead the estimate by a fixed interval; the difference
        // over that interval IS the velocity, in pixels per second.
        const float lead = 2.f / 30.f;
        o.vel = cv::Point2f((r.predX - (r.x + r.w * 0.5f)) / lead,
                            (r.predY - (r.y + r.h * 0.5f)) / lead);
        if (o.locked) ++age_; else if (!o.valid) age_ = 0;
        if (wasValid_ && !o.valid) ++losses_;
        wasValid_ = o.valid;
        o.age = age_; o.losses = losses_;
        return o;
    }

private:
    // THE ONLY PLACE A cv::Mat TOUCHES THE FUSED CORE. Its whole portability
    // claim is that it does not know about OpenCV; converting here keeps that
    // true while letting the runtime, which is full of cv::Mat, drive it.
    static GrayFrame toGray(const cv::Mat& bgr) {
        cv::Mat m = bgr;
        if (m.type() != CV_8UC3) {
            cv::Mat tmp;
            if (m.channels() == 1) cv::cvtColor(m, tmp, cv::COLOR_GRAY2BGR);
            else                   m.convertTo(tmp, CV_8UC3);
            m = tmp;
        }
        return GrayFrame::fromBgr(m.data, m.cols, m.rows, int(m.step), 3);
    }

    LockTracker trk_;
    long age_ = 0;
    int  losses_ = 0;
    bool wasValid_ = false;
};

}  // namespace

std::unique_ptr<ITrackerCore> makeLegacyCore(Backend b) {
    return std::unique_ptr<ITrackerCore>(new LegacyCore(b));
}
std::unique_ptr<ITrackerCore> makeFusedCore() {
    return std::unique_ptr<ITrackerCore>(new FusedCore());
}

}  // namespace track
