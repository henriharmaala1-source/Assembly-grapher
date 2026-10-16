// The two depth colour ramps, moved here from voxel_live.cpp so `demo` and
// `sim` cannot disagree about what a given distance looks like. See the header.
#include "depth_vis.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace sim {

cv::Mat colourDepthEq(const cv::Mat& d, float maxM) {
    int hist[256] = {0};
    long n = 0;
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            int b = int(std::min(1.f, r[x] / std::max(0.1f, maxM)) * 255.f);
            ++hist[b]; ++n;
        }
    }
    float cdf[256];
    long acc = 0;
    for (int i = 0; i < 256; ++i) { acc += hist[i]; cdf[i] = n ? float(acc) / n : 0.f; }

    cv::Mat out(d.rows, d.cols, CV_8UC3, cv::Scalar(90, 90, 90));
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            int b = int(std::min(1.f, r[x] / std::max(0.1f, maxM)) * 255.f);
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(cdf[b] * 120.f), 200, 230);
        }
    }
    cv::Mat bgr; cv::cvtColor(out, bgr, cv::COLOR_HSV2BGR);
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x)
            if (!(r[x] > 0.f)) bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(90, 90, 90);
    }
    return bgr;
}

cv::Mat colourDepth(const cv::Mat& d, float maxM) {
    cv::Mat out(d.rows, d.cols, CV_8UC3, cv::Scalar(90, 90, 90));
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (!(r[x] > 0.f)) continue;
            const float t = std::min(1.f, r[x] / std::max(0.1f, maxM));
            // OpenCV 8-bit hue is 0..179, NOT 0..359. The first version added a
            // 120 offset on top of a 0..120 ramp, so everything past ~6 m wrapped
            // through magenta and read as NEARER than the foreground -- a colour
            // map that inverts its own meaning at range.
            const int hue = int(t * 120.f);          // 0 = red near, 120 = blue far
            out.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(hue), 200, 230);
        }
    }
    cv::Mat bgr;
    cv::cvtColor(out, bgr, cv::COLOR_HSV2BGR);
    // Repaint the invalid pixels AFTER the conversion, so they are a flat grey
    // rather than whatever the hue ramp does at the ends.
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        for (int x = 0; x < d.cols; ++x)
            if (!(r[x] > 0.f)) bgr.at<cv::Vec3b>(y, x) = cv::Vec3b(90, 90, 90);
    }
    return bgr;
}

}  // namespace sim
