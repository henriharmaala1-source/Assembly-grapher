#pragma once

#include <opencv2/core.hpp>
#include <mutex>
#include <string>
#include <vector>

#include "control_types.hpp"

// ---------------------------------------------------------------- behaviour
// MANUAL     — pilot has the sticks; OS computes but does not send control
// IDLE       — armed-idle, nothing to do
// NAVIGATE   — follow the monocular-depth open corridor
// ROAD_FOLLOW— follow a visually-detected road/track
// TRACK      — centre and pursue a locked-on target
// SEARCH     — slow yaw scan looking for a target / corridor
// EVADE      — back off from a close high-confidence obstacle/intruder
// HOLD       — hover level, hold position
// RTL        — return to launch (handed to the flight controller's own RTL)
enum class Behavior { MANUAL, IDLE, NAVIGATE, ROAD_FOLLOW, TRACK,
                      SEARCH, EVADE, HOLD, RTL };
const char* behavior_name(Behavior b);

// Monotonic time in seconds (steady_clock) — the shared timebase for the
// perception staleness stamps below. Writers stamp with monoNowS() when they
// publish; the fly loop copies monoNowS() into WorldState::tickMonoS once per
// tick; consumers compare the two. Headless sims drive BOTH with sim time, so
// staleness logic is testable faster than real time.
double monoNowS();

// ---------------------------------------------------------------- detections
struct Detection {
    std::string label;
    float       confidence = 0.f;
    cv::Rect    box;
};

// ----------------------------------------------------------------- world state
// Single snapshot of everything the runtime knows. Perception modules and the
// MAVLink bridge (P2) write into it; the behaviour arbiter, telemetry logger,
// and the on-device LLM supervisor (P3) read from it. brief() is the compact
// text form the LLM consumes — keep it short and stable.
struct WorldState {
    long   frameId = 0;
    double fps     = 0.0;

    // "Now" for this tick (monoNowS timebase), set once per fly-loop tick.
    // Perception results are LATCHES — a crashed think thread leaves them set —
    // so consumers must judge them by AGE against this, via the *Fresh() helpers,
    // never by the valid flags alone.
    double tickMonoS = 0.0;

    Behavior behavior = Behavior::IDLE;

    // --- Track (lock-on) ---
    bool        targetValid  = false;
    bool        targetLocked = false;
    bool        targetCoast  = false;
    cv::Rect    targetBox;
    cv::Point2f targetVel;            // px / frame
    float       targetConf   = 0.f;
    long        targetAge    = 0;
    int         targetLosses = 0;
    double      targetStampS = -1e9;  // monoNowS at last tracker publish

    // --- Navigate (monocular corridor) ---
    bool        corridorValid    = false;
    bool        corridorDecisive = false;
    cv::Point2f corridorHeading;      // steer target, frame px
    float       corridorOffset = 0.f; // steer target, normalised [-1,1] (L..R)
    float       corridorOpen   = 0.f; // clearance at heading [0,1]
    float       corridorMargin = 0.f; // decisiveness [0,1]
    double      corridorStampS = -1e9;// monoNowS at last corridor publish

    // Freshness — valid AND recently written. This is what control-side
    // consumers (reflex, mission) must use; a bare `corridorValid` cannot tell
    // a live corridor from one latched by a dead think thread.
    bool corridorFresh(float maxAgeS) const {
        return corridorValid && (tickMonoS - corridorStampS) <= maxAgeS;
    }
    bool roadFresh(float maxAgeS) const {
        return roadValid && (tickMonoS - roadStampS) <= maxAgeS;
    }
    bool targetFresh(float maxAgeS) const {
        return targetValid && (tickMonoS - targetStampS) <= maxAgeS;
    }

    // --- Mission: move-stop-sense autonomous cycle ---
    bool        missionActive = false;
    std::string missionPhase;         // ARMED / SETTLE / THINK / MOVE / ARRIVE
    float       missionWpE = 0.f, missionWpN = 0.f;  // committed waypoint (ENU, m)
    // Operator inputs (set by keys / RC AUX / any command source):
    float       missionGoalBearing = 0.f;  // desired direction, deg (0 = North)
    bool        missionGo = false;         // "go" latch — cycle waits for this

    // --- Shadow: what AUTONOMY *would* command, computed but NOT sent (SHADOW
    // mode — operator flies; the overlay is drawn on the feed for validation) ---
    bool        shadowActive = false;
    ControlCmd  shadowCmd;             // intended command (advisory only)

    // --- Detect ---
    std::vector<Detection> detections;
    double      detStampS = -1e9;     // monoNowS at last detector publish

    // --- Road follow (appearance-based; pairs with the depth corridor) ---
    bool        roadValid   = false;
    float       roadOffset  = 0.f;    // lateral centreline offset [-1,1]
    float       roadHeading = 0.f;    // near→far bend [-1,1]
    float       roadConf    = 0.f;    // [0,1]
    double      roadStampS  = -1e9;   // monoNowS at last road publish

    // --- Vehicle telemetry (filled by the flight-controller backend) ---
    bool        vehArmed     = false;
    float       vehBattery   = 1.0f;  // [0,1]
    float       vehBattV     = 0.f;   // volts
    float       vehAltM      = 0.f;
    float       vehRollDeg   = 0.f;
    float       vehPitchDeg  = 0.f;
    float       vehYawDeg    = 0.f;   // heading, 0 = North
    double      vehLat       = 0.0;
    double      vehLon       = 0.0;
    int         vehSats      = 0;
    int         vehFix       = 0;     // 0 none, 2 = 2D, 3 = 3D
    float       vehGroundspeed = 0.f; // m/s
    bool        vehLink      = false; // FC serial link alive
    std::string vehMode      = "SIM";

    // --- Fused state estimate (Pi-side EKF; ENU local frame) ---
    bool        estValid     = false;
    float       estPe = 0.f, estPn = 0.f, estPu = 0.f;   // local ENU position (m)
    float       estVe = 0.f, estVn = 0.f, estVu = 0.f;   // local ENU velocity (m/s)
    float       estSpeed     = 0.f;   // horizontal speed (m/s)
    float       estEphM      = 0.f;   // horizontal 1σ uncertainty (m)
    bool        estGpsDenied = false; // coasting without a GPS fix
    bool        estFeedingFc = false; // injecting synthetic GPS into the FC

    // --- Control (what the OS wants the FC to do) ---
    ControlCmd  control;              // last computed command
    bool        controlActive = false;// true once actually sent to the FC
    std::string opMode;               // top-level operator mode (ModeArbiter)
    std::string modeReason;           // why control is what it is (pre-empt etc.)

    std::string brief() const;        // one-line scene state (LLM input)
};

// ----------------------------------------------------------------- world model
// Thread-safe holder. Today the loop is single-threaded, but the LLM sidecar
// (P3) reads asynchronously, so the lock is here from the start.
class WorldModel {
public:
    template <class F>
    void with(F&& fn) {
        std::lock_guard<std::mutex> lk(mu_);
        fn(state_);
    }
    WorldState snapshot() const {
        std::lock_guard<std::mutex> lk(mu_);
        return state_;
    }

private:
    mutable std::mutex mu_;
    WorldState         state_;
};
