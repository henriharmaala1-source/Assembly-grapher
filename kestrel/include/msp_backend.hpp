#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "flight_controller.hpp"
#include "serial_port.hpp"

// iNAV / Betaflight MSP backend over UART.
//
// Control: MSP_SET_RAW_RC (id 200), 8× uint16 LE µs (1000–2000), channel order
// AETR — index 0=Roll, 1=Pitch, 2=THROTTLE, 3=YAW, then AUX (verified against
// iNAV firmware: NOT RPYT). Telemetry: STATUS(101), ATTITUDE(108),
// RAW_GPS(106), ANALOG(110), polled round-robin.
//
// iNAV requires receiver_type=MSP (or the MSP RC Override mode) and ≥5 Hz RC,
// else failsafe engages. Arming is intentionally NOT automated here — arm on the
// radio. Control is gated by the caller (dry-run by default).
class MspBackend : public IFlightController {
public:
    const char* name() const override { return "msp"; }

    bool connect(const std::string& port, int baud) override;
    void disconnect() override { serial_.close(); }
    bool linkUp() const override;

    void tick() override;
    bool poll(FcTelemetry& out) override;
    bool sendControl(const ControlCmd& cmd) override;

private:
    // MSP message IDs (decimal).
    enum : uint8_t { MSP_STATUS = 101, MSP_RAW_GPS = 106, MSP_ATTITUDE = 108,
                     MSP_ANALOG = 110, MSP_SET_RAW_RC = 200 };

    void sendV1(uint8_t cmd, const uint8_t* payload, uint8_t size);
    void requestNextTelemetry();
    void drainRx();
    void onMessage(uint8_t cmd, const std::vector<uint8_t>& p);

    static uint16_t axisToUs(float v);   // [-1,1] → [1000,2000]
    static uint16_t thrToUs(float v);    // [0,1]  → [1500,2000] (mid = hold)

    SerialPort   serial_;
    FcTelemetry  tel_;

    // RX parser state machine.
    enum class St { DOLLAR, M, DIR, SIZE, CMD, DATA, CRC } st_ = St::DOLLAR;
    uint8_t              rxSize_ = 0, rxCmd_ = 0, rxCrc_ = 0, rxCount_ = 0;
    std::vector<uint8_t> rxBuf_;

    using clock = std::chrono::steady_clock;
    clock::time_point lastRx_{};
    clock::time_point lastPoll_{};
    int               pollIdx_ = 0;
    bool              connected_ = false;
};
