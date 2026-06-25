#pragma once

// Shared control/telemetry value types — used by the controller, the world
// model, and (P-FC) the flight-controller backends. Kept dependency-free so
// neither the perception layer nor the FC layer has to include the other.

// Normalised control command. All axes in [-1, 1] except throttle in [0, 1].
// A backend maps these to RC microseconds (MSP) or attitude/velocity setpoints
// (MAVLink). valid=false means "do not override" — the pilot keeps control.
struct ControlCmd {
    float roll     = 0.f;   // + = right
    float pitch    = 0.f;   // + = forward / nose-down translate
    float yaw      = 0.f;   // + = clockwise (turn right)
    float throttle = 0.f;   // 0 = idle/hover-bias, 1 = full
    bool  valid    = false;
};

// Telemetry read back from the flight controller.
struct FcTelemetry {
    bool   armed        = false;
    float  battV        = 0.f;     // volts
    float  battPct      = 1.f;     // [0,1]
    float  rollDeg      = 0.f;
    float  pitchDeg     = 0.f;
    float  yawDeg       = 0.f;     // heading, 0 = North
    double lat          = 0.0;
    double lon          = 0.0;
    float  altM         = 0.f;
    int    sats         = 0;
    int    fixType      = 0;       // 0 none, 2 = 2D, 3 = 3D
    float  groundspeedMs = 0.f;
    bool   linkUp       = false;   // FC serial link alive
};
