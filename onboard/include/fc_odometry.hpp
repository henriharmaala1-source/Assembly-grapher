#pragma once
// ---------------------------------------------------------------------------
// The FC's own local position as the mission's DISPLACEMENT source, when -- and
// only when -- EKF3 vouches for it.
//
// Why this exists: the Pi-side StateEstimator initialises from a GPS fix and
// nothing else, so GNSS-denied it never starts, estValid stays false, and the
// move-stop-sense mission hovers in SETTLE(no-est) for ever. Architecture C
// removed the position ESTIMATE; the mission still needs a DISPLACEMENT to fly
// a leg of a given length. With an optical-flow sensor (EK3_SRC1_VELXY=5) EKF3
// has one, and says so in EKF_STATUS_REPORT.
//
// READ, NEVER FED BACK: one filter. The retired design sent a Pi estimate into
// the FC as a fake GPS -- two filters in series; this is the opposite direction.
// ---------------------------------------------------------------------------

#include <cmath>

#include "control_types.hpp"
#include "world_model.hpp"

// MAVLink ESTIMATOR_STATUS_FLAGS used here.
constexpr unsigned kEkfPosHorizRel = 8u;     // relative horizontal position OK
constexpr unsigned kEkfConstPosMode = 128u;  // "constant position" = no source

// Fill the estimate fields from FC telemetry. Returns false -- and touches
// nothing -- unless the FC's local position is fresh (< maxAgeS), an EKF report
// has been seen, it claims a relative horizontal position, and it is NOT in
// constant-position mode (what EKF3 does when it has no horizontal source).
inline bool fcLocalEstimate(const FcTelemetry& t, double nowS, WorldState& s,
                            double maxAgeS = 0.5) {
    if (!t.localValid || !t.ekfValid) return false;
    if (nowS - t.localStampS > maxAgeS) return false;
    if (!(t.ekfFlags & kEkfPosHorizRel)) return false;
    if (t.ekfFlags & kEkfConstPosMode) return false;
    s.estValid = true;
    s.estPe = t.localE; s.estPn = t.localN; s.estPu = -t.localD;
    s.estVe = t.localVe; s.estVn = t.localVn;
    s.estSpeed = std::hypot(t.localVe, t.localVn);
    // EKF3 reports a normalised test ratio, not metres; the mission's maxEphM
    // gate is in metres. Publish a nominal flow-aided figure rather than
    // mislabel a ratio as a distance.
    s.estEphM = 0.5f;
    s.estGpsDenied = true;
    s.estFeedingFc = false;
    return true;
}
