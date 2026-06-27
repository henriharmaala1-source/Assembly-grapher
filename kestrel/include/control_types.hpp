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
    float  altM         = 0.f;     // GPS altitude, m (coarse)
    float  baroAltM     = 0.f;     // baro/estimated altitude, m (MSP_ALTITUDE, cm res)
    int    sats         = 0;
    int    fixType      = 0;       // 0 none, 2 = 2D, 3 = 3D
    float  groundspeedMs = 0.f;
    float  groundCourseDeg = 0.f;  // GPS course over ground, deg (0 = North)
    bool   linkUp       = false;   // FC serial link alive
};

// A synthetic GPS fix produced by the Pi-side estimator and injected into iNAV
// as MSP2_SENSOR_GPS (gps_provider = MSP). This is how a Pi running SLAM gives
// the FC a position when there is no real GPS — iNAV's nav uses it like a real
// fix, subject to its glitch radius (2.5 m), accel limit (10 m/s²), 1.5 s
// timeout and 3D-fix/accuracy gating, so the values must be smooth and honest.
struct ExtGps {
    double lat      = 0.0;   // deg
    double lon      = 0.0;   // deg
    float  altMslM  = 0.f;   // m (MSL-ish; FC cares about deltas, not datum)
    float  velN     = 0.f;   // m/s, North
    float  velE     = 0.f;   // m/s, East
    float  velD     = 0.f;   // m/s, Down
    float  ephM     = 1.0f;  // horizontal 1σ accuracy, m
    float  epvM     = 1.5f;  // vertical 1σ accuracy, m
    float  hdop     = 1.0f;  // dilution of precision
    float  yawDeg   = -1.f;  // course/heading, deg; <0 = unknown
    int    fixType  = 3;     // 3 = 3D (iNAV needs ≥3 to use it)
    int    sats     = 12;    // synthetic sat count (report healthy)
};
