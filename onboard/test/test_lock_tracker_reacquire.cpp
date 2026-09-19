// The old tracker's template scale convention -- the one the flight runtime
// still instantiates (perception.hpp holds a LockOnTracker, not a LockTracker).
//
// saveTemplate caps tmpl_ at 96 px, so for any larger box the template is a
// DOWNSCALED rendition of the target. computeNCC honoured that by resizing the
// candidate ROI to tmpl_.size(); templateSearch did not -- it slid tmpl_ over
// the frame at native resolution. One member, two conventions, and past 96 px
// they disagree.
//
// This is not a flag-only path. The box has no upper bound and the flow backend
// scales it by up to 1.18x per frame, so an 80 px default crosses 96 in two
// frames as the target closes range.
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/imgproc.hpp>

#include "lock_tracker.hpp"

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}
static std::string f1(double v) { char b[48]; std::snprintf(b, sizeof b, "%.1f", v); return b; }

// A frame with a distinctive textured target at (cx,cy) of side `sz`, on a
// textured background -- a blank background would let any match win.
static cv::Mat scene(int w, int h, int cx, int cy, int sz, unsigned seed) {
    cv::Mat img(h, w, CV_8UC3);
    cv::RNG rng(seed);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const uchar v = uchar(90 + 30 * std::sin(x * 0.07) * std::cos(y * 0.05));
            img.at<cv::Vec3b>(y, x) = {v, v, v};
        }
    // The target: fixed-seed noise, so it is the same pattern every frame and
    // is genuinely distinctive against the smooth background.
    cv::RNG t(12345);
    for (int y = cy - sz / 2; y < cy + sz / 2; ++y)
        for (int x = cx - sz / 2; x < cx + sz / 2; ++x) {
            if (x < 0 || y < 0 || x >= w || y >= h) continue;
            const uchar v = uchar(t.uniform(0, 256));
            img.at<cv::Vec3b>(y, x) = {v, v, v};
        }
    return img;
}

int main() {
    std::printf("old tracker: template scale convention on re-acquire\n");

    // A box WELL over the 96 px cap, so the template is downscaled 128 -> 96.
    const int SZ = 128, W = 640, H = 480;
    int x = 200, y = 240;

    LockOnTracker trk;
    cv::Mat f0 = scene(W, H, x, y, SZ, 1);
    if (!trk.init(f0, {x, y}, Backend::FLOW, SZ)) {
        std::printf("  init failed\n"); return 1;
    }
    check("designated a 128 px box (over the 96 px template cap)",
          trk.bbox().width == SZ, std::to_string(trk.bbox().width));

    // Fly it. A jump per frame far larger than the flow backend can follow
    // forces the loss path, which is where the template search lives -- so
    // this exercises re-acquire through the public API rather than reaching
    // into a private method.
    for (int i = 0; i < 12; ++i) {
        x += 34; y += 6;
        cv::Mat f = scene(W, H, x, y, SZ, unsigned(2 + i));
        trk.update(f);
    }

    const cv::Rect b = trk.bbox();
    std::printf("  after 12 jumps: bbox %dx%d at (%d,%d), locked=%d, losses=%d\n",
                b.width, b.height, b.x, b.y, int(trk.locked()), trk.totalLosses());

    // THE SIZE IS THE POINT, and the assertion has to be TIGHT enough to see
    // it. templateSearch returned a rect sized tmpl_.cols x tmpl_.rows, so a
    // successful re-acquire silently reset the tracked box to the capped
    // template -- and reinitBackend then re-initialised the backend on a box
    // smaller than the target, which shrinks again on the next re-acquire.
    //
    // MEASURED both ways, so this is a regression test and not decoration:
    //   with the fix     124 x 124   (3 % under the designated 128)
    //   without it        95 x  95   (26 % under -- the 96 px cap, clipped)
    // A first attempt asserted "not exactly 96" and "within 2x", and BOTH
    // passed on the broken build. A test that passes either way proves
    // nothing; the band below is set between the two measurements.
    const int lo = int(SZ * 0.85), hi = int(SZ * 1.15);
    check("the re-acquired box keeps the target's size, not the template cap",
          b.width >= lo && b.width <= hi && b.height >= lo && b.height <= hi,
          std::to_string(b.width) + "x" + std::to_string(b.height)
          + " (want " + std::to_string(lo) + ".." + std::to_string(hi) + ")");

    // And it must still be ON the target. A box of the right size in the wrong
    // place is the other way this could pass while being broken.
    const double cx = b.x + b.width / 2.0, cy = b.y + b.height / 2.0;
    const double err = std::hypot(cx - x, cy - y);
    check("and it is still on the target", err < SZ,
          f1(err) + " px from truth");

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
