#pragma once

#include <cstdio>

#include "flight_controller.hpp"

// ArduPilot MAVLink backend — INTERFACE STUB.
//
// Implementing this fully needs the generated MAVLink C headers (mavlink/v2.0/
// common). The verified message set it must use:
//   - HEARTBEAT (#0) @1 Hz, auto, in tick()        — GCS failsafe depends on it
//   - DO_SET_MODE (cmd 176) custom_mode GUIDED(4) / GUIDED_NOGPS(20)
//   - COMPONENT_ARM_DISARM (cmd 400, param2=21196 force)
//   - SET_POSITION_TARGET_LOCAL_NED (#84) velocity type_mask 0x0DC7
//   - SET_ATTITUDE_TARGET (#82) for GUIDED_NOGPS / attitude-only
//   - RC_CHANNELS_OVERRIDE (#70) — maps from sendControl(), like MSP
//   - VISION_POSITION_ESTIMATE (#102) for feedVisionPose() (EK3_SRCx=6)
//   - read ATTITUDE(#30), GLOBAL_POSITION_INT(#33), GPS_RAW_INT(#24),
//     SYS_STATUS(#1); request via SET_MESSAGE_INTERVAL (cmd 511)
//
// Until those headers are wired in, connect() reports unavailable so the OS
// runs FC-less (dry-run control) without a hard dependency.
class MavlinkBackend : public IFlightController {
public:
    const char* name() const override { return "mavlink"; }

    bool connect(const std::string&, int) override {
        std::fprintf(stderr,
            "[mavlink] backend not built — needs MAVLink headers. "
            "Use --fc=msp (iNAV) or run FC-less. See mavlink_backend.hpp.\n");
        return false;
    }
    void disconnect() override {}
    bool linkUp() const override { return false; }
    void tick() override {}
    bool poll(FcTelemetry&) override { return false; }
    bool sendControl(const ControlCmd&) override { return false; }
};
