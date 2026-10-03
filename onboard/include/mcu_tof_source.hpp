#pragma once

#include <cstdint>
#include <vector>

#include "serial_port.hpp"
#include "tof_source.hpp"

// ITofSource backend fed by an MCU sensor hub over UART, instead of the Pi
// talking I2C to the ToF part directly.
//
// WHY: ST's VL53L9CX reference driver (ULD) targets an STM32 Nucleo natively —
// that's the eval kit's documented platform. Running the sensor on a cheap
// STM32 (or RP2040/ESP32) means the vendor driver runs where it's actually
// supported, with zero porting risk, and the Pi only has to parse a SIMPLE
// SERIAL FRAME this class fully controls — a far smaller, more certain problem
// than guessing ST's proprietary I2C ranging protocol from Linux. It also
// solves multi-sensor fusion for free: several ToF units share the I2C address
// 0x29 and need XSHUT-sequenced address reassignment at boot, which is fiddly
// from Linux GPIO and natural on a dedicated MCU — the MCU can read N sensors,
// stitch them into one wider grid, and stream just ONE clean frame here.
//
// Wire format (little-endian), designed to be trivial to emit from any MCU:
//
//   0xAA 0x55  rows:u8  cols:u8  maxRangeM:f32  data[rows*cols]:i16(mm)  crc:u8
//
//   - data[i] in millimetres; 0 or negative (0xFFFF/-1) = no valid return.
//   - crc = XOR of every byte from `rows` through the last data byte
//     (same convention as the MSP v1 checksum already used in this codebase).
//   - rows/cols bounded to MAX_DIM (64) to keep the parser's buffer bounded
//     against a corrupted length byte.
//
// The MCU-side firmware (STM32/RP2040/whatever) is the one piece that lives
// outside this repo: run the vendor ULD driver, pack its result into this
// frame, write it to UART. That firmware is NOT included here — this class is
// the Pi-side half of the link, ready the moment that firmware exists.
class McuTofSource : public ITofSource {
public:
    static constexpr int MAX_DIM = 64;

    bool connect(const std::string& dev, int baud = 460800);
    const char* name()      const override { return "mcu-tof"; }
    float       maxRangeM() const override { return maxRangeM_; }

    // Drains the UART and returns true if a full, checksum-valid frame arrived
    // since the last call (out is only updated on success).
    bool read(cv::Mat& out) override;

private:
    void drainRx_();
    void onFrame_();

    SerialPort serial_;

    enum class St { S1, S2, ROWS, COLS, RANGE, DATA, CRC } st_ = St::S1;
    uint8_t              rows_ = 0, cols_ = 0;
    uint8_t              rangeBytes_[4]{};
    int                  rangeCount_ = 0;
    std::vector<uint8_t> dataBuf_;
    int                  dataCount_  = 0;
    uint8_t              crc_        = 0;

    cv::Mat latest_;
    float   maxRangeM_ = 0.f;
    bool    fresh_     = false;
};
