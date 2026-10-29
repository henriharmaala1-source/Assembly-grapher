#pragma once
// ---------------------------------------------------------------------------
// ONE TRACKER CORE, BEHIND ONE SEAM.
//
// There are two trackers in this tree. `LockOnTracker` is what TrackModule
// instantiates, so it is what flies. `LockTracker` -- the fused, OpenCV-free
// one -- is what the recorder builds and what every tracker test exercises.
// So a green tracker suite has never said anything about the tracker running
// inside kestrel, and that was documented staging rather than an accident, but
// it means the test results did not cover the deployed code.
//
// This is the seam that closes it: both are driven through the same interface,
// by the runtime, by the recorder and by replay, so a result from one of those
// is a result about the other two.
//
// EVERY OBSERVATION CARRIES ITS OWN PROVENANCE. A consumer that is handed a
// box needs to know which frame it describes (captureSec), how long since
// anything was actually SEEN (ageOfFixSec -- COASTING and SEARCHING both
// produce boxes from extrapolation, and neither is a measurement), and which
// implementation produced it. None of that was available before; the module
// published monoNowS() at the moment of publishing, which is the time the
// answer was written down rather than the time it describes.
// ---------------------------------------------------------------------------

#include <memory>
#include <opencv2/core.hpp>

#include "lock_tracker.hpp"

namespace track {

struct TrackObs {
    bool  valid    = false;   // there is a target to speak of
    bool  locked   = false;   // it is being SEEN, not extrapolated
    bool  coasting = false;
    cv::Rect    box;
    cv::Point2f vel;          // PIXELS PER SECOND, not per frame
    float conf = 0.f;
    long  age  = 0;
    int   losses = 0;
    // Provenance. See the header note -- these are the difference between a
    // box and a box you can decide how much to trust.
    double      captureSec   = -1e9;  // the frame this describes
    float       ageOfFixSec  = -1.f;  // since the last ACCEPTED observation
    const char* core         = "";    // which implementation produced it
    const char* state        = "";    // its own word for what it is doing
};

class ITrackerCore {
public:
    virtual ~ITrackerCore() = default;
    virtual const char* name() const = 0;
    virtual void reset() = 0;
    virtual void designate(const cv::Mat& frame, cv::Point centre, int boxSize) = 0;
    // captureSec is when the FRAME WAS TAKEN, from any steady clock. Not when
    // this call was reached: a consumer acting quickly on an old frame is
    // acting on old information, and only the capture time can say so.
    virtual TrackObs update(const cv::Mat& frame, double captureSec) = 0;
};

// The tracker that currently flies. OpenCV backends, cv::Mat throughout.
std::unique_ptr<ITrackerCore> makeLegacyCore(Backend b);

// The fused tracker: OpenCV-free internals, so the same core runs on a phone
// and an MCU. This adapter is the only place a cv::Mat touches it.
std::unique_ptr<ITrackerCore> makeFusedCore();

}  // namespace track
