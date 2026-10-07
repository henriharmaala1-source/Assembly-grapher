#include "vl53_tof_source.hpp"

#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------- VL53L9CX

bool Vl53Tof9Source::connect(const std::string& devPath, uint8_t addr7) {
    if (!hal_.open(devPath, addr7)) return false;
    booted_ = bootFirmware_();
    if (!booted_)
        std::fprintf(stderr,
            "[vl53l9] I2C link opened but no ranging driver wired in yet — "
            "see the TODOs in vl53_tof_source.cpp. Running dataless.\n");
    return true;   // link is open either way; read() just returns false until booted_
}

bool Vl53Tof9Source::bootFirmware_() {
    // TODO(vendor driver): call the ULD equivalent of
    //   vl53l5cx_init(&dev) / vl53l5cx_start_ranging(&dev)
    // against `hal_` (readReg/writeReg/delayMs already match the ULD platform
    // shape). Set the resolution to the part's max (4x4/8x8 on L5/L8CX; up to
    // 54x42 on L9CX) before starting.
    return false;
}

bool Vl53Tof9Source::pollFrame_(cv::Mat& out) {
    // TODO(vendor driver): call the ULD equivalent of
    //   vl53l5cx_check_data_ready(&dev, &ready)
    //   vl53l5cx_get_ranging_data(&dev, &results)
    // then, for the reported zone grid (rows x cols):
    //   out.create(rows, cols, CV_32F);
    //   out.at<float>(r, c) = (results.target_status[i] is a valid return)
    //                             ? results.distance_mm[i] * 1e-3f
    //                             : -1.f;   // no return -> DepthNav treats as blocked
    (void)out;
    return false;
}

bool Vl53Tof9Source::read(cv::Mat& out) {
    if (!booted_) return false;
    return pollFrame_(out);
}

// ---------------------------------------------------------------- VL53L5CX

bool Vl53Tof5Source::connect(const std::string& devPath, uint8_t addr7) {
    if (!hal_.open(devPath, addr7)) return false;
    booted_ = bootFirmware_();
    if (!booted_)
        std::fprintf(stderr,
            "[vl53l5cx] I2C link opened but no ranging driver wired in yet — "
            "the L5CX ULD is publicly available from ST today; port its "
            "platform layer onto i2c_hal.hpp and fill this in.\n");
    return true;
}

bool Vl53Tof5Source::bootFirmware_() {
    // TODO(ST VL53L5CX ULD): vl53l5cx_init() / vl53l5cx_start_ranging(),
    // 8x8 resolution. This part's ULD is the well-trodden one — start here if
    // the L9CX driver isn't available yet; identical ITofSource contract means
    // swapping which class the perception module owns is a one-line change.
    return false;
}

bool Vl53Tof5Source::pollFrame_(cv::Mat& out) {
    // TODO: same pattern as Vl53Tof9Source::pollFrame_, fixed 8x8 zones.
    (void)out;
    return false;
}

bool Vl53Tof5Source::read(cv::Mat& out) {
    if (!booted_) return false;
    return pollFrame_(out);
}

// -------------------------------------------------------------- SimTofSource

bool SimTofSource::read(cv::Mat& out) {
    out.create(rows_, cols_, CV_32F);
    ++tick_;
    // A gap that sweeps left-to-right and slowly closes, so the VFH+ steer can
    // be watched tracking it end-to-end with no hardware attached.
    const float phase   = (tick_ % 240) / 240.f;                 // 0..1 sweep
    const float gapCtr  = phase * (cols_ - 1);
    const float gapHalf = std::max(2.f, cols_ * 0.18f * (1.f - 0.5f * phase));
    for (int y = 0; y < rows_; ++y) {
        for (int x = 0; x < cols_; ++x) {
            const bool inGap = std::fabs(x - gapCtr) < gapHalf;
            out.at<float>(y, x) = inGap ? maxRange_ * 0.9f : 0.6f;  // open vs near wall
        }
    }
    return true;
}
