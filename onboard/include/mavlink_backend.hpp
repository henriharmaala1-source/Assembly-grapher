#pragma once

#include <cstdint>
#include <string>

#include "flight_controller.hpp"
#include "mavlink_v2.hpp"
#include "serial_port.hpp"

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
// SO THIS BACKEND SUPPORTS TWO CONTROL PATHS, deliberately, because they fail
// differently:
//
//   RC OVERRIDE (RC_CHANNELS_OVERRIDE) is the common denominator with the MSP
//     backend -- same ControlCmd, same stick semantics, same assist-mode trim.
//     It works in any mode including ACRO, needs no EKF, and is the path to use
//     for the first flights because it is the one whose failure mode the pilot
//     already understands (let go, the override times out, sticks come back).
//
//   VELOCITY SETPOINT (SET_POSITION_TARGET_LOCAL_NED, BODY_NED frame) is what
//     the trajectory planner actually produces: a speed and a direction in the
//     aircraft's own frame. It needs GUIDED and therefore a working position
//     estimate, which we do not have yet. It is implemented and tested here so
//     that the state estimator has something to plug into, NOT because it is
//     ready to fly.
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
    const char* name() const override { return "mavlink"; }

    bool connect(const std::string& port, int baud) override;
    void disconnect() override { serial_.close(); linkUp_ = false; }
    bool linkUp() const override { return linkUp_; }

    void tick() override;
    bool poll(FcTelemetry& out) override;
    bool sendControl(const ControlCmd& cmd) override;

    bool setMode(FcMode m) override;
    bool arm(bool force) override;
    bool disarm() override;

    void setAssistMode(bool on) override { assist_ = on; }
    void latchBaseline() override;
    void setBatteryCells(int cells) override { battCells_ = cells; }
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
    void feedVisionPose(float xN, float yE, float zD,
                        float rollRad, float pitchRad, float yawRad);
    void feedVisionSpeed(float vN, float vE, float vD);

    // Body-frame velocity setpoint, m/s, +x forward +y right +z DOWN, plus a
    // yaw rate in rad/s. Requires the aircraft to be in GUIDED.
    bool sendVelocityBody(float vFwd, float vRight, float vDown, float yawRateRadS);

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
    static uint16_t thrToUs(float v);             // [0,1]  -> [1500,2000]
    static uint16_t addDelta(uint16_t base, float v);

    // Default sysid 255 -- see the SYSID note above. compid 191 is
    // MAV_COMP_ID_ONBOARD_COMPUTER.
    mav::Codec  codec_{255, 191};
    SerialPort  serial_;
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

    bool   originValid_ = false;
    double originLat_ = 0, originLon_ = 0;
    float  originAlt_ = 0;
};
