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
    bool feedExternalGps(const ExtGps& fix) override;
    void setAssistMode(bool on) override { assist_ = on; }
    void latchBaseline() override;

    // RTH via AUX: setMode(RTL) latches the channel HIGH in every subsequent RC
    // frame (iNAV flies NAV RTH); any other mode releases it. Returns false — so
    // the caller can fall back to release — when no RTH channel is configured.
    void setRthChannel(int idx, int us) override { rthAuxIdx_ = idx; rthAuxUs_ = us; }

    // Battery cell count for the per-cell state-of-charge that drives the
    // low-battery failsafe. <=0 = infer from the first voltage reading.
    void setBatteryCells(int cells) override { battCells_ = cells; }
    bool setMode(FcMode m) override;

private:
    // MSP v1 message IDs (decimal).
    enum : uint8_t { MSP_RC = 105, MSP_STATUS = 101, MSP_RAW_GPS = 106,
                     MSP_ATTITUDE = 108, MSP_ALTITUDE = 109, MSP_ANALOG = 110,
                     MSP_SET_RAW_RC = 200 };
    // MSP v2 function IDs (16-bit). Sensor messages are fire-and-forget (no ACK).
    enum : uint16_t { MSP2_SENSOR_GPS = 0x1F03 };

    void sendV1(uint8_t cmd, const uint8_t* payload, uint8_t size);
    void sendV2(uint16_t function, const uint8_t* payload, uint16_t size);
    void requestNextTelemetry();
    void drainRx();
    void onMessage(uint8_t cmd, const std::vector<uint8_t>& p);

    static uint16_t axisToUs(float v);   // [-1,1] → [1000,2000] absolute
    static uint16_t thrToUs(float v);    // [0,1]  → [1500,2000] (mid = hold)
    static uint16_t addDelta(uint16_t base, float v);  // base + v*500, clamped

    SerialPort   serial_;
    FcTelemetry  tel_;

    // Control blending. Total autonomy writes absolute sticks; flight assist
    // trims relative to baseline_ (operator RC latched at engagement). rc_ holds
    // the live channels read back via MSP_RC.
    bool     assist_        = false;
    bool     baselineValid_ = false;
    uint16_t baseline_[8]{};
    uint16_t rc_[18]{};
    int      rcCount_       = 0;
    bool     warnedNoAux_   = false;   // one-shot: refused to send, AUX unknown
    int      battCells_     = 0;       // <=0 = infer from first voltage reading

    // Failsafe RTH-via-AUX (P2.1). While rthActive_, sendControl forces the
    // configured channel high so iNAV enters NAV RTH; the arm channel is left at
    // baseline so the aircraft stays armed for the return.
    int      rthAuxIdx_ = -1;      // raw RC channel index (AUX1 = 4); <0 = off
    int      rthAuxUs_  = 1800;    // µs written to it when RTH is active
    bool     rthActive_ = false;

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
