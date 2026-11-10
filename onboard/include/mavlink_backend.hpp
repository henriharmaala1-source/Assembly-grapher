#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "flight_controller.hpp"
#include "mavlink_v2.hpp"
#include "vio_frame_align.hpp"
#include "byte_link.hpp"

// ---------------------------------------------------------------------------
// ArduPilot MAVLink backend.
//
// WHY ARDUPILOT AND NOT iNAV/MSP, now that the choice is made. MSP has no
// concept of a companion computer: the only way in is RC override, which means
// the Pi impersonates a pilot's sticks and the FC never knows the difference.
// That works, and it is what msp_backend.cpp does, but it forecloses everything
// interesting -- no velocity setpoints, no vision pose into the estimator, no
// mode the autopilot understands as "someone else is flying". ArduPilot has all
// three as first-class messages, and the mode the pilot flies manually (ACRO,
// custom_mode 1) sits next to them on the same mode switch.
//
// SO THIS BACKEND SUPPORTS THREE CONTROL PATHS, and the FC's flight mode
// picks between them (Uplink::AUTO, below), because they fail differently:
//
//   RC OVERRIDE (RC_CHANNELS_OVERRIDE) in ALT_HOLD / LOITER / POSHOLD /
//     FLOWHOLD -- the modes where mid throttle means "hold this height", which
//     is what ControlCmd's throttle assumes. Needs no position estimate, and
//     its failure mode is one the pilot already understands (flick the switch,
//     the override is released, sticks come back). NOT the same mapping as the
//     MSP backend: ArduPilot's pitch is reversed (forward = low pulse), and
//     every axis goes through ArduPilot's own RC ranges, deadbands and speed
//     parameters, read back at link-up.
//
//   ATTITUDE TARGET (SET_ATTITUDE_TARGET) in GUIDED_NOGPS -- a lean angle and
//     a climb rate. The GNSS-denied path with no external nav.
//
//   VELOCITY SETPOINT (SET_POSITION_TARGET_LOCAL_NED, BODY_NED frame) in
//     GUIDED -- a speed and a yaw rate, which is what every mode here actually
//     computes. Needs a position estimate (GPS, or VIO into EKF3 as
//     ExternalNav: --voxel-vio-fc) or ArduPilot refuses GUIDED.
//
// Every one of these is encoded and unit-tested against pymavlink. NONE has
// yet been flown against ArduPilot itself, in SITL or in the air: that is the
// next proof, and the parameter readback is what makes it measurable.
//
// TWO CONFIGURATION FACTS THAT WILL COST AN AFTERNOON IF MISSED:
//
//   SYSID. ArduPilot only accepts RC_CHANNELS_OVERRIDE and mode changes from
//     the system id in SYSID_MYGCS (MAV_GCS_SYSID on 4.5+), default 255. So the
//     default here is 255 and not something tidier. If a real GCS is also on the
//     link, one of the two has to move.
//
//   OVERRIDE TIMEOUT. ArduPilot ignores an override channel again after
//     RC_OVERRIDE_TIME (default 3 s) without a new message, and a value of 0
//     means "release this channel". Both are safety features and both mean the
//     override must be resent continuously -- which FcLink's own thread already
//     guarantees, because it was built for iNAV's 5 Hz MSP-RC failsafe.
// ---------------------------------------------------------------------------

class MavlinkBackend : public IFlightController {
public:
    MavlinkBackend();
    const char* name() const override { return "mavlink"; }

    bool connect(const std::string& port, int baud) override;
    void disconnect() override { link_.close(); linkUp_ = false; }
    bool linkUp() const override { return linkUp_; }

    void tick() override;
    bool poll(FcTelemetry& out) override;
    bool sendControl(const ControlCmd& cmd) override;

    bool setMode(FcMode m) override;
    bool arm(bool force) override;
    bool disarm() override;
    // MAV_CMD_NAV_TAKEOFF to `altM` above home. ArduCopter takes it only in
    // GUIDED, armed, on the ground -- the SITL runner's way into the air; a
    // pilot does this with the sticks.
    bool takeoff(float altM);
    // Motors off NOW, in the air too (ArduPilot's 21196 "forced" disarm). For
    // a simulated crash -- `kestrel sitl` ends a run on contact with it, so
    // the airframe falls as it would -- never for a flight.
    bool forceDisarm();
    // PARAM_SET, REAL32. Fire and forget: ArduPilot answers with PARAM_VALUE,
    // which the readback above picks up if the name is one it tracks. For the
    // SITL runner (SIM_SPEEDUP); the aircraft's own parameters are the
    // pilot's, set from a ground station, never from here.
    bool setParam(const char* name, float value);
    // The FC's clock: time_boot_ms of the newest ATTITUDE, 0 before one. On
    // SITL it is SIMULATED time, which is what `kestrel sitl` paces itself to.
    uint32_t fcTimeMs() const { return fcTimeMs_; }

    void setAssistMode(bool on) override { assist_ = on; }
    void latchBaseline() override;
    void setBatteryCells(int cells) override { battCells_ = cells; }

    // CONTROL INTERFACE SELECTION -- see onboard/docs/MAVLINK_BRIDGE_PLAN.md.
    //
    //   RC_OVERRIDE      pretend to be the pilot's sticks. Only in a mode where
    //                    mid throttle means "hold this height" (ALT_HOLD,
    //                    LOITER, POSHOLD, FLOWHOLD); ControlCmd's throttle is a
    //                    climb rate around hover and means nothing else.
    //   ATTITUDE_TARGET  a lean angle and a climb rate (GUIDED_NOGPS: no
    //                    position estimate needed, ArduPilot's default reading
    //                    of `thrust` is a climb rate, 0.5 = hold).
    //   VELOCITY         a body-frame velocity and yaw rate (GUIDED). The one
    //                    that says what the program means: metres per second.
    //                    Needs a position estimate -- GPS, or VIO fed to EKF3
    //                    as ExternalNav -- or ArduPilot refuses GUIDED.
    //   AUTO             (default) choose from the mode the FC reports: GUIDED
    //                    -> VELOCITY, GUIDED_NOGPS -> ATTITUDE, a height-hold
    //                    mode -> RC. The pilot's mode switch picks the uplink.
    //
    // In any other mode -- STABILIZE, ACRO, RTL, LAND, AUTO -- nothing is sent,
    // and an override already in place is RELEASED at once rather than left to
    // ArduPilot's RC_OVERRIDE_TIME, so the pilot's sticks are live immediately.
    // A fixed uplink that does not match the mode is the same: nothing is sent.
    enum class Uplink { AUTO, RC_OVERRIDE, ATTITUDE_TARGET, VELOCITY };
    void setUplink(Uplink u) { uplink_ = u; }
    Uplink uplink() const { return uplink_; }
    static bool parseUplink(const std::string& s, Uplink& out);
    static const char* uplinkName(Uplink u);
    // What sendControl does in the mode the FC is in right now ("none" if it
    // would send nothing) -- for the operator and the bench test.
    const char* controlPath() const;
    // The same, in three letters for the OSD: VEL ATT RC AST (assist), OFF
    // (this mode gets nothing), PAR (still reading parameters).
    const char* controlTag() const override;
    // STATUSTEXT, severity INFO, from this companion (sysid/compid as set).
    // ArduPilot only LOGS a received STATUSTEXT; kestrel_osd.lua shows it.
    bool sendStatusText(const char* text) override;

    // WHAT A FULL STICK MEANS. ControlCmd is a normalised RATE command: pitch
    // 1 = `mps` forward, throttle 1 = `climbMps` up, yaw 1 = `yawDps`
    // clockwise. The velocity uplink multiplies these out; the stick and
    // attitude uplinks rescale them through ArduPilot's own parameters (read
    // at link-up, below) so the FC flies the same speed either way. Defaults
    // are the nav-sim airframe's, which every gain in the tree was tuned on.
    struct StickScale { float mps = 4.f, climbMps = 1.5f, yawDps = 90.f; };
    void setStickScale(const StickScale& s) { scale_ = s; }
    const StickScale& stickScale() const { return scale_; }

    // ARDUPILOT'S OWN NUMBERS. On link-up the backend reads the parameters
    // that decide what a stick or a setpoint does -- LOIT_SPEED,
    // PILOT_SPEED_UP/DN, THR_DZ, PILOT_Y_RATE, the RC1-4 ranges, RCMAP,
    // WPNAV_SPEED_UP/DN, GUID_OPTIONS -- and the ones that decide whether it is
    // listened to at all (SYSID_MYGCS / MAV_GCS_SYSID). No control is sent
    // until every one has answered or timed out (absent on this firmware), so
    // the first frame is already calibrated.
    bool paramsResolved() const { return paramsDone_; }
    // The parameters as read, and what each one means for this program's
    // commands; problems are lines starting "!!".
    std::string paramReport() const;
    // Value of a parameter read above; false if absent or not yet answered.
    bool param(const char* name, float& out) const;
    void setParamTimeoutS(double s) { paramTimeoutS_ = s; }

    // Full-scale tilt for a +-1 ControlCmd axis. ArduPilot's own ANGLE_MAX
    // defaults to 30 deg; staying under it means the command is never clipped
    // by a limit we cannot see.
    void setMaxTiltDeg(float d) { maxTiltDeg_ = d; }

    // OBSTACLE_DISTANCE (330) -- 72 distances by bearing, clockwise from the
    // nose, metres, negative where nothing is known.
    //
    // This is the cheapest safety win available: ArduPilot's own proximity and
    // avoidance layer consumes exactly this shape, so publishing it buys a
    // SECOND, INDEPENDENT avoidance path running different code with different
    // failure modes and no dependence on our planner being correct. For a
    // project whose safety argument rests on independent routes to a veto, that
    // is close to free defence in depth.
    //
    // The producer already exists as BearingField::obstacleDistance() in
    // nav-sim; this is the consumer side of that interface.
    bool sendObstacleDistance(const float* distM, int n,
                              float minM = 0.2f, float maxM = 30.f);
    bool sendProximity(const float* distM, int n) override {
        return sendObstacleDistance(distM, n, 0.2f, 20.f);
    }
    // VIO into EKF3 as ODOMETRY -- ArduPilot's "preferred method", and the only
    // vision message whose `quality` it reads (VISION_POSITION_ESTIMATE's is
    // hard-wired to 0, "unknown"). Local NED position, BODY-FRD velocity,
    // frame_id LOCAL_FRD / child BODY_FRD exactly (anything else is silently
    // dropped). The ENU->NED swap happens here and nowhere else. On the way the
    // pose goes through VioFrameAligner: continuous across VIO re-anchors (the
    // reset_counter sent never changes) and its yaw held to the FC's.
    // ArduPilot: VISO_TYPE=3, EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=0 (NOT 6:
    // gnss-denied-setup.md section 11, measured in SITL), firmware
    // 4.7.1+ (onboard/docs/gnss-denied-setup.md section 11).
    bool sendVisionOdometry(const VisionOdom& v) override;
    // The FC will be asked to fuse vision: the parameter report then flags
    // VISO_TYPE / EK3 source settings and a missing EKF origin as problems.
    void setExpectVision(bool on) { expectVision_ = on; }
    // THE EKF ORIGIN. Without GPS, ArduPilot has no origin until one is set,
    // and with none GUIDED and LOITER have no frame to work in. After link-up
    // the backend asks for it (GPS_GLOBAL_ORIGIN); if the FC has none and one
    // was configured here (fc.origin_lat/lon/alt_m), it sends
    // SET_GPS_GLOBAL_ORIGIN until the FC confirms. The point should be near
    // where you fly: the EKF takes magnetic declination from it.
    void setEkfOrigin(double latDeg, double lonDeg, float altMslM) {
        cfgOriginValid_ = true; cfgLat_ = latDeg; cfgLon_ = lonDeg; cfgAlt_ = altMslM;
    }
    bool ekfOriginKnown() const { return ekfOriginKnown_; }
    // An RTL the FC does not take within this long (ArduPilot refuses RTL
    // without a position estimate -- exactly when VIO has failed) becomes LAND.
    void setModeConfirmS(double s) { modeConfirmS_ = s; }
    const VioFrameAligner& aligner() const { return aligner_; }
    void setRthChannel(int, int) override {}   // MAVLink has a real mode API

    // ExtGps is an MSP-shaped struct (lat/lon/alt). ArduPilot's supported
    // companion path is VISION_POSITION_ESTIMATE in LOCAL NED metres, so this
    // latches the first fix as the origin and sends offsets from it. The datum
    // is arbitrary and self-consistent, which is what the EKF wants; it is not a
    // GPS and must not be described as one.
    bool feedExternalGps(const ExtGps& fix) override;

    // --- backend-specific, for when a state estimator exists -----------------
    // Pose in LOCAL NED metres and radians (x North, y East, z DOWN). The z sign
    // is the one that catches people: everything else in this project is +up.
    // resetCounter < 0 sends the 32-byte base message (the synthetic-GPS path,
    // whose origin never jumps); >= 0 adds the v2 extensions -- covariance
    // marked UNKNOWN (NaN first element: ArduPilot then uses VISO_POS_M_NSE)
    // and the reset counter, which EKF3 needs to see a discontinuity as one.
    void feedVisionPose(float xN, float yE, float zD,
                        float rollRad, float pitchRad, float yawRad,
                        int resetCounter = -1);
    void feedVisionSpeed(float vN, float vE, float vD);

    // Body-frame velocity setpoint, m/s, +x forward +y right +z DOWN, plus a
    // yaw rate in rad/s. Requires the aircraft to be in GUIDED. sendControl's
    // VELOCITY path is this, with ControlCmd scaled by the StickScale.
    bool sendVelocityBody(float vFwd, float vRight, float vDown, float yawRateRadS);

    // SITL TRUTH. ArduPilot's simulator reports its own true state as SIM_STATE
    // (108); a real aircraft never sends it. nav-sim's SITL bridge asks for it
    // and renders the cameras from it, so the onboard stack sees the world from
    // where the simulated aircraft really IS -- not from where its EKF believes
    // it is, which is the very thing under test. Off unless asked for.
    struct SimTruth {
        bool   valid = false;
        double stampS = 0;                        // receipt time, steady clock
        long   count = 0;                         // frames received
        float  qw = 1, qx = 0, qy = 0, qz = 0;    // body FRD -> NED
        float  rollRad = 0, pitchRad = 0, yawRad = 0;
        float  accel[3] = {}, gyroRadS[3] = {};   // body frame
        double latDeg = 0, lonDeg = 0;
        float  altMslM = 0;
        float  vN = 0, vE = 0, vD = 0;            // m/s, NED
    };
    // The FC's own STATUSTEXT ("PreArm: ...", "EKF3 IMU0 is using GPS"): the
    // newest one and how many have arrived, for whoever wants to show them.
    const std::string& fcText() const { return fcText_; }
    long fcTextCount() const { return fcTextCount_; }
    // THE FC'S IMU, for a stereo-inertial SLAM: HIGHRES_IMU -- calibrated,
    // noisy, what the EKF itself integrates -- with its microsecond time on
    // the FC's clock (SITL: simulated time). Body FRD, m/s^2 (specific force:
    // -9.8 on z at rest) and rad/s. Off unless asked for; kept in a fixed ring
    // of the newest kImuRing, drained by takeImu().
    struct ImuSample { double tS = 0; float acc[3] = {}, gyro[3] = {}; };
    void requestImu(int hz) { imuHz_ = hz; if (link_.isOpen()) requestStreams(); }
    // Every sample since the last call, oldest first (`out` cleared first).
    void takeImu(std::vector<ImuSample>& out);
    long imuDropped() const { return imuDropped_; }
    void requestSimTruth(int hz) { simTruthHz_ = hz; if (link_.isOpen()) requestStreams(); }
    const SimTruth& simTruth() const { return simTruth_; }

    // What the FC says it is doing right now. The pilot's ACRO and our GUIDED
    // are both visible here, which is how the OS knows whether it is flying.
    uint32_t copterMode() const { return copterMode_; }
    bool     armedByFc()  const { return tel_.armed; }
    long     crcErrors()  const { return codec_.crcErrors(); }

    void setIds(uint8_t sysid, uint8_t compid) { codec_.setIds(sysid, compid); }
    void setTargets(uint8_t sysid, uint8_t compid) { tgtSys_ = sysid; tgtComp_ = compid; }

private:
    void send(uint32_t msgid, const mav::Payload& p);
    void drainRx();
    void onMessage(const mav::Msg& m);
    void sendHeartbeat();
    void requestStreams();
    bool commandLong(uint16_t cmd, float p1, float p2 = 0, float p3 = 0, float p4 = 0,
                     float p5 = 0, float p6 = 0, float p7 = 0);

    static uint16_t axisToUs(float v);            // [-1,1] -> [1000,2000]
    static uint16_t thrToUs(float v);             // [-1,1] -> [1000,2000]
    static uint16_t addDelta(uint16_t base, float v);

    // The path for a mode; NONE sends nothing.
    enum class Path { NONE, RC, ATTITUDE, VELOCITY };
    Path pathFor(uint32_t mode) const;
    static bool holdsHeight(uint32_t mode);      // mid throttle = hold
    bool sendRc(const ControlCmd& cmd, uint32_t mode);
    bool sendVelocity(const ControlCmd& cmd);
    void releaseRc();
    // ControlCmd axis -> the microseconds ArduPilot reads as that fraction of
    // its own input range, through RCn_MIN/TRIM/MAX/DZ/REVERSED.
    uint16_t angleUs(int ch, float frac) const;
    uint16_t throttleUs(float climbMps) const;
    float    climbFrac(float climbMps, float upMps, float dnMps) const;

    void serviceParams(double now);       // `now` on fcClock()
    double fcClock(double wallNow);
    double fcClock_ = 0.0, fcClockWall_ = -1.0;
    uint32_t fcClockFcMs_ = 0;
    void onParamValue(const mav::Msg& m);
    bool rcMapOk() const;

    // Default sysid 255 -- see the SYSID note above. compid 191 is
    // MAV_COMP_ID_ONBOARD_COMPUTER.
    mav::Codec  codec_{255, 191};
    // Serial on the aircraft, tcp:/udp: against ArduPilot SITL (byte_link.hpp).
    ByteLink    link_;
    FcTelemetry tel_;

    uint8_t  tgtSys_ = 1, tgtComp_ = 1;      // ArduPilot's defaults
    bool     linkUp_ = false;
    bool     everRx_ = false;   // heard from the autopilot at least once
    double   lastHbSentS_ = -1e9;
    double   lastRxS_     = -1e9;
    double   lastStreamReqS_ = -1e9;
    uint32_t copterMode_ = 0xFFFFFFFF;
    uint32_t bootMs_ = 0;

    bool     assist_        = false;
    bool     baselineValid_ = false;
    uint16_t baseline_[8]{};
    int      battCells_     = 0;
    Uplink   uplink_        = Uplink::AUTO;
    float    maxTiltDeg_    = 25.f;
    StickScale scale_;
    bool     sendAttitudeTarget(const ControlCmd& cmd);
    bool     rcActive_      = false;   // an override of ours is in force
    uint32_t reportedMode_  = 0xFFFFFFFF;  // last mode announced on stdout
    const char* reportedPath_ = nullptr;   // ...and the control path with it

    // RESUME: the mode before our RTL/LAND, and the one we asked for.
    uint32_t resumeMode_  = 0xFFFFFFFF;
    uint32_t specialMode_ = 0xFFFFFFFF;

    // Parameter readback. Fixed table, no allocation on the link thread.
    struct Param { const char* name; float v; bool have; int tries; };
    static constexpr int kNParams = 53;     // 46 meanings + 7 renamed in 4.7
    Param    params_[kNParams];
    bool     paramsStarted_ = false, paramsDone_ = false;
    bool     renamedSeen_ = false;          // answered under a 4.7 name
    double   paramsT0_ = 0, lastParamReqS_ = -1e9;
    double   paramTimeoutS_ = 3.0;
    float    pv(const char* name, float def) const;

    // Vision into EKF3 (sendVisionOdometry) and its EKF origin.
    VioFrameAligner aligner_;
    bool     expectVision_ = false;
    double   lastAttS_ = -1e9;              // newest ATTITUDE, for the yaw reference
    bool     cfgOriginValid_ = false;
    double   cfgLat_ = 0, cfgLon_ = 0;
    float    cfgAlt_ = 0;
    bool     ekfOriginKnown_ = false;
    double   ekfLat_ = 0, ekfLon_ = 0;
    float    ekfAlt_ = 0;
    double   originT0_ = -1, lastOriginReqS_ = -1e9;
    bool     originWarned_ = false, originSent_ = false;
    void     serviceOrigin(double now);
    // RTL -> LAND when the FC does not take RTL.
    double   specialReqS_ = -1e9, modeConfirmS_ = 2.5;
    bool     specialSeen_ = false;

    std::string fcText_;
    long     fcTextCount_ = 0;
    int      simTruthHz_ = 0;
    uint32_t fcTimeMs_ = 0;
    int      imuHz_ = 0;
    static constexpr int kImuRing = 512;
    ImuSample imuRing_[kImuRing];
    int      imuHead_ = 0, imuCount_ = 0;
    long     imuDropped_ = 0;              // overwritten before being taken
    SimTruth simTruth_;

    bool   originValid_ = false;
    double originLat_ = 0, originLon_ = 0;
    float  originAlt_ = 0;
};
