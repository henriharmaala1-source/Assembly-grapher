#pragma once
// ---------------------------------------------------------------------------
// DarkFrameGate -- with the D435i's emitter STROBING, which IR frames are dark?
//
// Tracking (VIO, SLAM) must never see the projector's dots: they are fixed to
// the camera and track as zero motion. The strobe lights every other frame,
// and librealsense's per-frame metadata says which -- except that its polarity
// has been reported INVERTED in exactly this mode (realsense-ros #3040, D455,
// fw 5.15.1, SDK 2.54.2), and on Linux it is absent without the RSUSB backend.
// A gate that trusted it would hand the tracker precisely the dotted frames.
//
// So the image decides. The dots are hundreds of small bright points: a lit
// frame carries far more fine-scale energy (mean |Laplacian|) than the dark
// frame before or after it. Each frame is compared with the previous one:
//   clearly more energy  -> LIT, skip it
//   clearly less         -> DARK, use it
//   about the same       -> skip it (dropped frames put lit frames in a row,
//                           and they look like this) -- until 6 frames in a
//                           row show no alternation: then the dots are not
//                           visible (sunlight, nothing in range) and every
//                           frame is usable.
// Measured on the rendered strobe in test_voxel_nav and onboard/test.
// ---------------------------------------------------------------------------

#include <cmath>

#include <opencv2/core.hpp>

namespace sim {

class DarkFrameGate {
public:
    enum class Verdict { Lit, Dark, Usable, Unknown };

    // ratio: how much more fine energy makes a frame "clearly" lit (> 1).
    explicit DarkFrameGate(float ratio = 1.15f) : ratio_(ratio) {}

    Verdict classify(const cv::Mat& ir8) {
        const float e = energy(ir8);
        Verdict v = Verdict::Unknown;
        if (prevE_ > 0.f && e > 0.f) {
            if (e > prevE_ * ratio_) v = Verdict::Lit;
            else if (e * ratio_ < prevE_) v = Verdict::Dark;
            // About the same as the previous frame. Dropped frames put two
            // (or three) lit frames in a row, and those look exactly like
            // this -- so it is SKIPPED, until kQuiet frames in a row have
            // shown no alternation at all: then the dots are not visible
            // (sunlight, nothing in range), and every frame is usable.
            else v = (++quiet_ >= kQuiet) ? Verdict::Usable : Verdict::Unknown;
            if (v == Verdict::Lit || v == Verdict::Dark) quiet_ = 0;
        }
        prevE_ = e;
        return v;
    }
    static bool usable(Verdict v) { return v == Verdict::Dark || v == Verdict::Usable; }

    // Mean absolute 4-neighbour Laplacian over EVERY pixel: what a field of
    // one-to-two-pixel dots adds most of. Not subsampled -- a regular dot
    // lattice can fall entirely between the samples of a regular grid (the
    // test's did), and the full pass is under a millisecond at 848x480.
    static float energy(const cv::Mat& im) {
        if (im.empty() || im.type() != CV_8U || im.rows < 3 || im.cols < 3) return 0.f;
        double s = 0.0;
        long n = 0;
        for (int y = 1; y < im.rows - 1; ++y) {
            const uchar* a = im.ptr<uchar>(y - 1);
            const uchar* b = im.ptr<uchar>(y);
            const uchar* c = im.ptr<uchar>(y + 1);
            for (int x = 1; x < im.cols - 1; ++x) {
                s += std::abs(4 * int(b[x]) - int(b[x - 1]) - int(b[x + 1]) -
                              int(a[x]) - int(c[x]));
                ++n;
            }
        }
        return n ? float(s / double(n)) : 0.f;
    }

private:
    static constexpr int kQuiet = 6;   // 5 dropped frames in a row to fool it
    float   ratio_;
    float   prevE_ = -1.f;
    int     quiet_ = 0;
};

}  // namespace sim
