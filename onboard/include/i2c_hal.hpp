#pragma once

#include <cstdint>
#include <string>

// Minimal Linux i2c-dev register HAL. This is the exact shape ST's ULD drivers
// (VL53L5CX/L8CX, and almost certainly L9CX) expect as their "platform" layer:
// multi-byte read/write at a device address + a millisecond delay. Implementing
// a vendor ULD against this HAL — instead of writing driver protocol from
// scratch — is what makes bringing up a new sensor a small, mechanical step
// rather than a rewrite once the vendor source is available.
class I2cHal {
public:
    bool open(const std::string& devPath, uint8_t addr7);   // e.g. "/dev/i2c-1", 0x29
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // ST ULD platform-layer shape: multi-byte read/write at a 16-bit register.
    bool readReg(uint16_t reg, uint8_t* buf, int len);
    bool writeReg(uint16_t reg, const uint8_t* buf, int len);
    static void delayMs(int ms);

private:
    int fd_ = -1;
};
