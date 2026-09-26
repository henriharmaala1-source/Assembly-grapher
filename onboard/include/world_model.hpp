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
    double      targetStampS = -1e9;  // capture time of the frame just tracked
    // Seconds since the last ACCEPTED observation; < 0 when there has never
    // been one. targetStampS advances every frame the tracker speaks, which it
    // does while coasting on extrapolation, so it cannot answer "is this box
    // still a measurement".
    float       targetFixAgeS = -1.f;
    const char* targetCore     = "";   // which tracker core produced this

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

    // --- Corridor polar depth scan (P5b input to the occupancy grid) ---
    // Metric clearance (m) per ray across the FoV, centred on vehicle heading,
    // ray 0 at yaw−fov/2. corridorScanN=0 means "not provided". A ray at
    // corridorScanMaxM is a miss (no obstacle along it). Perception fills this;
    // the mission's LocalMap integrates it.
    static constexpr int kScanMax = 41;
    float       corridorScan[kScanMax] = {};
    int         corridorScanN    = 0;
    float       corridorScanFovDeg = 0.f;
    float       corridorScanMaxM = 0.f;

    // --- Planner output (LocalMap wavefront; P5b) ---
    bool        planValid   = false;  // a grid route exists this cycle
    float       planBearing = 0.f;    // routed direction, deg (0 = North)

    // --- Voxel navigation (D435i stereo -> voxel map -> swept-volume plan) ---
    // VoxelNavModule; the same pipeline as nav-sim's voxel_live. Valid only
    // while the aircraft is still: it has no position estimate, so it builds a
    // map per vantage (see voxel_nav.hpp). voxMoving says "blind because
    // translating", which is expected, rather than "blind because broken".
    bool        voxValid      = false;
    bool        voxMoving     = false;
    bool        voxBlocked    = false; // nothing survived the veto -> turn to look
    float       voxBearingDeg = 0.f;   // chosen direction, deg (0 = North, cw)
    float       voxElDeg      = 0.f;   // + up
    float       voxSpeed      = 0.f;   // m/s it may fly, stoppable within freeM
    float       voxFreeM      = 0.f;   // CONFIRMED-free distance on that bearing
    float       voxOpenM      = 0.f;   // unknown-discounted openness (choice only)
    // The CERTIFIED straight, level leg the module chose, and its bearing --
    // the geometry a move-stop-sense leg actually flies: the longest, ties to
    // the planner's bearing (voxel_nav.hpp: farChoose is the measured-worse
    // alternative that lets the far tier pick). voxFreeM is measured along a
    // curved primitive and does not certify a straight line, so the mission
    // flies these two, not voxBearingDeg/voxFreeM.
    float       voxLegFreeM   = 0.f;
    float       voxLegBearingDeg = 0.f; // the bearing that leg is on (0 = N, cw)
    float       voxLegFarM    = 0.f;   // far tier's confirmed range on it (0 = none)
    int         voxFrames     = 0;     // frames in the current vantage's map
    // PROXIMITY from the CURRENT depth frame (obstacleDistanceFromFrame):
    // 72 horizontal distances, 5 deg each, clockwise from the nose, < 0 unknown.
    // Unlike everything above it is valid WHILE MOVING -- it needs attitude,
    // not position -- and it goes to the FC's own avoidance (OBSTACLE_DISTANCE).
    static constexpr int kProxBins = 72;
    float       voxProx[kProxBins] = {};
    int         voxProxN      = 0;
    double      voxProxStampS = -1e9;
    double      voxStampS     = -1e9;  // monoNowS at last publish
    bool voxFresh(float maxAgeS) const {
        return voxValid && (tickMonoS - voxStampS) <= maxAgeS;
    }

    // VISUAL ODOMETRY (navcore DepthVio, run by VoxelNavModule on the D435i's
    // dark IR frames). ENU metres from where it started, yaw from North, cw.
    // vioValid is THIS frame's pose being a measurement; a lost frame coasts
    // and says so. vioResets counts discontinuities (restarts, losses): a
    // consumer holding an offset from an earlier pose must drop it when this
    // changes.
    bool        vioValid      = false;
    float       vioPe = 0.f, vioPn = 0.f, vioPu = 0.f;
    float       vioVe = 0.f, vioVn = 0.f;
    float       vioYawDeg     = 0.f;
    int         vioTracked    = 0;     // corners with a world point this frame
    int         vioResets     = 0;
    int         vioLost       = 0;     // frames lost since start
    double      vioStampS     = -1e9;

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
    //
    // RETIRED ON THE FLIGHT PATH -- ARCHITECTURE A. `MAVLINK_BRIDGE_PLAN.md` §2
    // retires this and says why: under ArduPilot it is not merely
    // non-idiomatic, it is STATISTICALLY WRONG. EKF3 would receive an
    // already-filtered Pi estimate and treat it as an independent measurement
    // -- two filters in series, each smoothing, neither aware of the other, and
    // a covariance that means nothing.
    //
    // v1 is architecture C: NOBODY estimates position. The thesis needs no
    // global position, the planner is body-frame by construction, the map is
    // local and short-lived, and the output is a bearing and a speed. That also
    // makes the GNSS-exclusion claim checkable rather than trusted: with no
    // position estimate anywhere, there is nothing for GNSS to contaminate.
    //
    // LEFT IN PLACE, NOT DELETED. These are correct code for architecture B
    // (the FC estimates; the Pi sends odometry increments with honest
    // covariance), which is the right v2 once VO exists. They must simply not
    // be wired to anything that flies. `estFeedingFc` in particular describes
    // injecting a synthetic GPS fix -- the exact thing architecture C exists to
    // avoid -- and must stay false on any ArduPilot build.
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
