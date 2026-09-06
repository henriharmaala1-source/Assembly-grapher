#include "mcu_tof_source.hpp"

#include <cstring>

bool McuTofSource::connect(const std::string& dev, int baud) {
    return serial_.open(dev, baud);
}

bool McuTofSource::read(cv::Mat& out) {
    drainRx_();
    if (!fresh_) return false;
    fresh_ = false;
    out = latest_;
    return true;
}

void McuTofSource::drainRx_() {
    uint8_t buf[512];
    int n;
    while ((n = serial_.read(buf, sizeof(buf))) > 0) {
        for (int i = 0; i < n; ++i) {
            const uint8_t b = buf[i];
            switch (st_) {
                case St::S1: st_ = (b == 0xAA) ? St::S2 : St::S1; break;
                case St::S2: st_ = (b == 0x55) ? St::ROWS : St::S1; break;
                case St::ROWS:
                    rows_ = b; crc_ = b;
                    st_ = (rows_ > 0 && rows_ <= MAX_DIM) ? St::COLS : St::S1;
                    break;
                case St::COLS:
                    cols_ = b; crc_ ^= b;
                    if (cols_ == 0 || cols_ > MAX_DIM) { st_ = St::S1; break; }
                    rangeCount_ = 0;
                    st_ = St::RANGE;
                    break;
                case St::RANGE:
                    rangeBytes_[rangeCount_++] = b; crc_ ^= b;
                    if (rangeCount_ >= 4) {
                        dataBuf_.assign((size_t)rows_ * cols_ * 2, 0);
                        dataCount_ = 0;
                        st_ = St::DATA;
                    }
                    break;
                case St::DATA:
                    dataBuf_[dataCount_++] = b; crc_ ^= b;
                    if (dataCount_ >= (int)dataBuf_.size()) st_ = St::CRC;
                    break;
                case St::CRC:
                    if (b == crc_) onFrame_();
                    st_ = St::S1;
                    break;
            }
        }
    }
}

void McuTofSource::onFrame_() {
    float range;
    std::memcpy(&range, rangeBytes_, 4);
    if (range <= 0.f) return;

    cv::Mat grid(rows_, cols_, CV_32F);
    for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
            const int idx = (r * cols_ + c) * 2;
            const int16_t mm = (int16_t)(dataBuf_[idx] | (dataBuf_[idx + 1] << 8));
            grid.at<float>(r, c) = (mm > 0) ? mm * 1e-3f : -1.f;
        }
    }
    latest_    = grid;
    maxRangeM_ = range;
    fresh_     = true;
}
