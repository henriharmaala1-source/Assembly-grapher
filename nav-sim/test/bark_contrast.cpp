// Measure how visible a real tree trunk is to a real stereo matcher.
//
//   ./bark_contrast photo.jpg  x0 y0 x1 y1  [x0 y0 x1 y1 ...]
//
// WHY THIS EXISTS. The forest world model assigns trunks a "texture" value that
// decides whether stereo can see them, and that value currently comes from ONE
// screenshot of somebody else's depth output, of unknown vintage, unknown
// pipeline, possibly a filter bug. Everything downstream -- whether the
// aircraft survives, how fast it may fly, how much clearance it needs -- rests
// on a number nobody has measured.
//
// This measures it. Photograph a trunk the way the aircraft would see it (from
// its altitude, against whatever background it will actually have, in the light
// it will actually fly in), mark a rectangle on the trunk and a rectangle on
// the ground, and this reports the quantity a block matcher actually cares
// about: how much local contrast there is inside a correlation window.
//
// WHAT THE NUMBER MEANS. A block matcher can only localise a window if that
// window varies. The standard measure is the normalised standard deviation
// inside the window; below roughly 2-3 grey levels of sigma on an 8-bit sensor
// a matcher has nothing to lock onto and returns no disparity. That maps onto
// the sim's `texThresh` and `trunkTexMin/Max`, which is what makes this
// actionable rather than interesting.
//
// TAKE THE PHOTOS PROPERLY, or the number is worse than none:
//   - from the aircraft's altitude and range, not standing under the tree
//   - BACKLIT is the case that matters; a trunk with the sun on it is easy
//   - same exposure the flight camera would choose, ideally the flight camera
//   - several trunks, several lighting conditions, and report the WORST
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

struct Stats { double meanSigma, p10Sigma, meanGrey; int windows, blind; };

// Local standard deviation inside each block-matcher-sized window, which is
// the quantity that decides whether a correlation has a unique maximum.
Stats windowContrast(const cv::Mat& grey, cv::Rect roi, int win, double blindSigma) {
    Stats s{0, 0, 0, 0, 0};
    std::vector<double> sig;
    for (int y = roi.y; y + win <= roi.y + roi.height; y += win / 2)
        for (int x = roi.x; x + win <= roi.x + roi.width; x += win / 2) {
            cv::Mat w = grey(cv::Rect(x, y, win, win));
            cv::Scalar m, sd;
            cv::meanStdDev(w, m, sd);
            sig.push_back(sd[0]);
            s.meanGrey += m[0];
            if (sd[0] < blindSigma) ++s.blind;
        }
    if (sig.empty()) return s;
    s.windows = int(sig.size());
    s.meanGrey /= sig.size();
    for (double v : sig) s.meanSigma += v;
    s.meanSigma /= sig.size();
    std::sort(sig.begin(), sig.end());
    // The 10th percentile, not the mean. A matcher fails on the WORST windows,
    // and a trunk that is textured on average but blank in patches produces
    // exactly the coherent holes that kill you.
    s.p10Sigma = sig[size_t(0.10 * (sig.size() - 1))];
    return s;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 6 || (argc - 2) % 4 != 0) {
        std::printf(
            "usage: %s photo.jpg x0 y0 x1 y1 [x0 y0 x1 y1 ...]\n"
            "  first rectangle is treated as the TRUNK, the rest as reference\n"
            "  (mark ground/litter for a reference the sim calls tex 0.7)\n", argv[0]);
        return 2;
    }
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (img.empty()) { std::printf("cannot read %s\n", argv[1]); return 3; }

    // Matcher window and blindness threshold. 7 px is a common StereoBM block;
    // 2.5 grey levels of sigma is about where an 8-bit sensor's own noise
    // starts to dominate the signal, so below that a "match" is matching noise.
    const int WIN = 7;
    const double BLIND = 2.5;

    std::printf("%s  %dx%d,  %d px matcher window, blind below sigma %.1f\n\n",
                argv[1], img.cols, img.rows, WIN, BLIND);
    std::printf("%-10s%9s%11s%10s%9s%10s\n",
                "region", "mean sd", "p10 sd", "mean grey", "blind%", "-> sim tex");

    for (int a = 2; a + 3 < argc; a += 4) {
        int x0 = std::atoi(argv[a]),   y0 = std::atoi(argv[a+1]);
        int x1 = std::atoi(argv[a+2]), y1 = std::atoi(argv[a+3]);
        cv::Rect roi(std::min(x0,x1), std::min(y0,y1),
                     std::abs(x1-x0), std::abs(y1-y0));
        roi &= cv::Rect(0, 0, img.cols, img.rows);
        if (roi.width < WIN || roi.height < WIN) { std::printf("  (region too small)\n"); continue; }

        Stats s = windowContrast(img, roi, WIN, BLIND);
        // Map onto the sim's texture scale. The sim treats tex as "richness"
        // with texThresh = 0.25 meaning no match; anchor that at the blindness
        // sigma and let 20 grey levels be a richly textured surface. This is a
        // calibration, so it is stated rather than hidden: change it here if
        // your sensor or matcher differs.
        double simTex = std::min(1.0, std::max(0.0, (s.p10Sigma - BLIND) / 20.0 * 0.75 + 0.25));
        if (s.p10Sigma < BLIND) simTex = s.p10Sigma / BLIND * 0.25;
        std::printf("%-10s%9.2f%11.2f%10.1f%9.0f%%%10.2f\n",
                    (a == 2 ? "TRUNK" : "ref"), s.meanSigma, s.p10Sigma, s.meanGrey,
                    100.0 * s.blind / std::max(1, s.windows), simTex);
    }

    std::printf("\nFeed the TRUNK figure to voxel_sim as --trunktex, then read the\n"
                "tolerance table in README_VOXEL.md to see whether the stack survives\n"
                "there. Photograph several trunks backlit and use the WORST one.\n");
    return 0;
}
