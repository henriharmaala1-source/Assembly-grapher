#pragma once

#include <memory>
#include <string>

#include "i2c_hal.hpp"
#include "tof_source.hpp"

// ITofSource backends for ST's multizone dToF family (VL53L5CX / L8CX / L9CX).
//
// INTEGRATION STATUS: the I2cHal register plumbing (i2c_hal.*) is real and
// tested. The RANGING PROTOCOL is NOT — these ST parts boot a firmware blob
// over I2C at init and expose results through a wrapper API (the "ULD"), not a
// simple documented register map, so it can't be written from scratch here
// without the vendor source. Everything up to that seam is done: once you have
// ST's ULD driver (published for L5CX/L8CX today; L9CX likely follows the same
// shape), implement the two TODO-marked functions below against it and this
// class is live — the rest of the pipeline (DepthNav::updateFromGrid, VFH+, the
// TofNavigateModule) needs no changes.
//
// Until then, read() returns false (no data) — safe to build and ship: the
// scheduler simply won't get ToF frames, exactly like a disconnected sensor.
class Vl53Tof9Source : public ITofSource {
public:
    // devPath e.g. "/dev/i2c-1" (Pi 5 default I2C bus), addr7 default 0x29.
    bool connect(const std::string& devPath, uint8_t addr7 = 0x29);

    const char* name()      const override { return "vl53l9"; }
    float       maxRangeM() const override { return 9.0f; }

    bool read(cv::Mat& out) override;

private:
    // ---- vendor-driver seam (fill in once the ULD source is available) ------
    // 1. bootFirmware_(): upload the sensor firmware blob + start ranging.
    //    Mirrors ULD's vl53l5cx_init()/vl53l5cx_start_ranging() for the L9CX.
    bool bootFirmware_();
    // 2. pollFrame_(): poll for a new result, fill `out` (CV_32F, metres,
    //    higher=farther, <=0=invalid) at whatever zone count the part reports
    //    (up to 54x42 for L9CX). Mirrors ULD's vl53l5cx_check_data_ready() +
    //    vl53l5cx_get_ranging_data(), converting distance_mm[]/target_status[]
    //    per zone into metres (reject zones whose status isn't a valid return).
    bool pollFrame_(cv::Mat& out);

    I2cHal hal_;
    bool   booted_ = false;
};

// Same shape, proven part: VL53L5CX, 8x8 zones, ~4 m. The guaranteed-simple
// fallback if L9CX driver bring-up stalls — swap one line at the call site,
// nothing else in the pipeline changes.
class Vl53Tof5Source : public ITofSource {
public:
    bool connect(const std::string& devPath, uint8_t addr7 = 0x29);
    const char* name()      const override { return "vl53l5cx"; }
    float       maxRangeM() const override { return 4.0f; }
    bool read(cv::Mat& out) override;

private:
    bool bootFirmware_();
    bool pollFrame_(cv::Mat& out);

    I2cHal hal_;
    bool   booted_ = false;
};

// ---------------------------------------------------------------------------
// Synthetic source for developing/testing the rest of the pipeline (perception
// module wiring, DepthNav::updateFromGrid, VFH+ steer) with ZERO hardware —
// exactly the situation where the vendor driver isn't ready yet. Sweeps a
// closing gap across the grid so you can watch the steer track it.
class SimTofSource : public ITofSource {
public:
    SimTofSource(int cols = 54, int rows = 42, float maxRangeM = 9.f)
        : cols_(cols), rows_(rows), maxRange_(maxRangeM) {}

    const char* name()      const override { return "sim-tof"; }
    float       maxRangeM() const override { return maxRange_; }
    bool read(cv::Mat& out) override;

private:
    int   cols_, rows_;
    float maxRange_;
    int   tick_ = 0;
};
