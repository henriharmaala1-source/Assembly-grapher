#pragma once

#include <opencv2/core.hpp>

// Abstract source of a metric depth grid for DepthNav::updateFromGrid().
//
// An implementation wraps one physical depth sensor and returns a frame's worth
// of range data as a CV_32F grid in METRES (higher = farther; <= 0 = invalid /
// no return). The grid can be any size — DepthNav upsamples it:
//
//   VL53L5CX / VL53L8CX  → 8×8   grid   (ST ULD driver, I²C)
//   VL53L9CX             → up to 54×42  (ST driver, I²C/SPI/MIPI)
//   OAK-D / stereo       → arbitrary-size depth map (already metric)
//
// The ST ULD / vendor driver (the I²C reads on the Pi) is platform-specific and
// is the one piece that lives outside this repo. A backend's only job is to pull
// the sensor's per-zone distances (usually mm) and write them into `out` as
// metres — then DepthNav does the rest (openness → sectors → VFH+).
//
// Wiring (in a perception module): construct a source, call DepthNav::enableTof()
// once, then each tick do  `if (src->read(g)) nav.updateFromGrid(g, frameSize,
// src->maxRangeM());`  — the VFH+ steer comes out exactly as with the DNN path.
class ITofSource {
public:
    virtual ~ITofSource() = default;
    virtual const char* name() const = 0;

    // Fill `out` (CV_32F, metres, higher = farther, <= 0 = invalid). Returns
    // false if no fresh frame is available this tick (caller reuses the last).
    virtual bool read(cv::Mat& out) = 0;

    // Max reliable range (m) — used to normalise openness. VL53L5CX ≈ 4,
    // VL53L9 ≈ 9, a stereo cam whatever its useful depth is.
    virtual float maxRangeM() const = 0;
};

// ---- Example backend skeleton (the ST driver is the platform-specific part) --
//
// class Vl53l9Source : public ITofSource {
// public:
//     bool connect(const char* i2cDev);       // opens I²C, boots the VL53L9
//     const char* name() const override { return "vl53l9"; }
//     float maxRangeM() const override { return 9.0f; }
//     bool read(cv::Mat& out) override {
//         // 1. poll the ST driver for a new 54×42 result (distance_mm per zone)
//         // 2. out.create(42, 54, CV_32F);
//         // 3. for each zone: out.at<float>(r,c) = valid ? mm * 1e-3f : -1.f;
//         // 4. return true when a fresh frame arrived, false otherwise
//     }
// };
