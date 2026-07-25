#pragma once

#include <string>

#include "control_types.hpp"

// Abstract flight-controller link. Designed around the UNION of MSP (iNAV) and
// MAVLink (ArduPilot) so a backend can be swapped without touching the OS:
//   - sendControl() is the shared primitive — RC override (MSP_SET_RAW_RC) /
//     RC_CHANNELS_OVERRIDE — the common denominator across both stacks.
//   - the backend owns link keep-alive (MSP poll / MAVLink HEARTBEAT); the
//     caller just calls tick() every loop. Forgetting a heartbeat triggers the
//     MAVLink GCS failsafe, so it must never be the caller's job.
//   - feedExternalGps() pushes a companion-computed position INTO the FC's own
//     estimator. iNAV has no native vision-pose message, so the supported path
//     is a synthetic GPS (MSP2_SENSOR_GPS, gps_provider=MSP). MAVLink backends
//     map it to GPS_INPUT / VISION_POSITION_ESTIMATE.
enum class FcMode { STABILIZE, ALT_HOLD, OFFBOARD, ANGLE, LOITER, RTL, LAND, UNKNOWN };

class IFlightController {
public:
    virtual ~IFlightController() = default;
    virtual const char* name() const = 0;

    virtual bool connect(const std::string& port, int baud) = 0;
    virtual void disconnect() = 0;
    virtual bool linkUp() const = 0;

    // Service the link: drain RX, parse telemetry, send keep-alive. Every loop.
    virtual void tick() = 0;

    // Latest telemetry (non-blocking — returns the last decoded values).
    virtual bool poll(FcTelemetry& out) = 0;

    // Shared control primitive: normalised command → RC override / setpoint.
    virtual bool sendControl(const ControlCmd& cmd) = 0;

    // Best-effort; backends without support return false.
    virtual bool setMode(FcMode)  { return false; }
    virtual bool arm(bool /*force*/) { return false; }
    virtual bool disarm()         { return false; }

    // Configure which channel commands the FC's return-to-home, for backends
    // that trigger RTH by driving an AUX/mode channel (MSP RC override). idx is
    // the raw channel index in the RC frame (AUX1 = 4); idx<0 disables, and the
    // backend's setMode(RTL) then does nothing (caller falls back to release →
    // the FC's own RC-loss failsafe). No-op on backends with a real mode API.
    virtual void setRthChannel(int /*idx*/, int /*us*/) {}

    // Battery cell count, for backends that only report pack voltage and must
    // derive per-cell state of charge themselves (the low-battery → RTH
    // failsafe depends on it). <=0 means "infer from the first reading".
    virtual void setBatteryCells(int /*cells*/) {}

    // Inject a companion-computed position into the FC's navigation estimator.
    // MSP backend: MSP2_SENSOR_GPS (no ACK). Requires gps_provider=MSP on the FC.
    virtual bool feedExternalGps(const ExtGps& /*fix*/) { return false; }

    // Control blending:
    //   total autonomy (default) — sendControl() writes absolute sticks from
    //     neutral; the OS is the stick source.
    //   flight assist           — sendControl() trims relative to the operator's
    //     sticks latched at engagement (bumpless takeover); call latchBaseline()
    //     on the dry→live transition to capture the current operator input.
    virtual void setAssistMode(bool /*on*/) {}
    virtual void latchBaseline()            {}
};
