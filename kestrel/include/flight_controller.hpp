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
//   - feedVisionPose() exists for the localization pipeline (MAVLink
//     VISION_POSITION_ESTIMATE); MSP backends no-op it.
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
    virtual bool feedVisionPose(double /*x*/, double /*y*/, double /*z*/,
                                float /*yawDeg*/) { return false; }
};
