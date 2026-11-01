#include "i2c_hal.hpp"

#include <cstdio>
#include <cstring>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#endif

#include <chrono>
#include <thread>

bool I2cHal::open(const std::string& devPath, uint8_t addr7) {
#if defined(__linux__)
    close();
    fd_ = ::open(devPath.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::fprintf(stderr, "[i2c] cannot open %s\n", devPath.c_str());
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, addr7) < 0) {
        std::fprintf(stderr, "[i2c] cannot address 0x%02x on %s\n", addr7, devPath.c_str());
        close();
        return false;
    }
    return true;
#else
    (void)devPath; (void)addr7;
    return false;   // build host (not Linux): no i2c-dev, caller falls back to sim
#endif
}

void I2cHal::close() {
#if defined(__linux__)
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
#endif
}

bool I2cHal::readReg(uint16_t reg, uint8_t* buf, int len) {
#if defined(__linux__)
    if (fd_ < 0) return false;
    // ST ToF parts use a 16-bit big-endian register address, written before the
    // read (a repeated-start read), which is the ULD platform convention.
    const uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    if (::write(fd_, addr, 2) != 2) return false;
    return ::read(fd_, buf, len) == len;
#else
    (void)reg; (void)buf; (void)len; return false;
#endif
}

bool I2cHal::writeReg(uint16_t reg, const uint8_t* buf, int len) {
#if defined(__linux__)
    if (fd_ < 0 || len > 252) return false;
    uint8_t packet[254];
    packet[0] = (uint8_t)(reg >> 8);
    packet[1] = (uint8_t)(reg & 0xFF);
    std::memcpy(packet + 2, buf, len);
    return ::write(fd_, packet, len + 2) == len + 2;
#else
    (void)reg; (void)buf; (void)len; return false;
#endif
}

void I2cHal::delayMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
