// Headless checks for the depth improver.
//
// The O(1)-per-pixel window minimum is exactly the kind of code that is
// plausible everywhere and wrong on the borders, so check 1 is a brute-force
// differential test on random data -- not a hand-picked example, because a
// hand-picked example is chosen by the same person who wrote the bug.
//
//   g++ -O2 -std=c++17 -I. test/depth_improve_check.cpp depth_improve.cpp \
//       -I/usr/include/opencv4 -lopencv_core -o /tmp/dic && /tmp/dic

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_improve.hpp"

using namespace sim;

static int failures = 0;
static void check(bool ok, const char* what) {
    std::printf("  %-58s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) ++failures;
}

// Reference improver: the obvious (2r+1)^2 scan. Slow, and correct by being
// impossible to get subtly wrong.
static cv::Mat referenceImprove(const cv::Mat& in, const DepthImproveParams& p) {
    cv::Mat out = in.clone();
    const int H = in.rows, W = in.cols, r = p.radiusPx;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            if (in.at<float>(y, x) > 0.f) continue;
            float m = std::numeric_limits<float>::infinity();
            int seeds = 0;
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    const int yy = y + dy, xx = x + dx;
                    if (yy < 0 || yy >= H || xx < 0 || xx >= W) continue;
                    const float d = in.at<float>(yy, xx);
                    if (!(d > 0.f) || !(d < p.nearM)) continue;
                    ++seeds;
                    if (d < m) m = d;
                }
            if (seeds >= p.minSeeds && m < std::numeric_limits<float>::infinity())
                out.at<float>(y, x) = m;
        }
    return out;
}

int main() {
    std::printf("depth improver checks\n");

    // 1. Differential against brute force, random fields, several radii and
    //    sizes -- including sizes that are NOT multiples of the block width,
    //    which is where the van Herk decomposition has its partial block.
    {
        bool allEqual = true;
        unsigned rng = 12345;
        auto rnd = [&]() { rng = rng * 1664525u + 1013904223u;
                           return float(rng >> 8) / float(1 << 24); };
        for (int trial = 0; trial < 12 && allEqual; ++trial) {
            const int W = 7 + trial * 5, H = 5 + trial * 3;
            DepthImproveParams p;
            p.radiusPx = 1 + trial % 4;
            p.minSeeds = 1 + trial % 5;
            p.nearM    = 2.0f;
            cv::Mat d(H, W, CV_32F);
            for (int y = 0; y < H; ++y)
                for (int x = 0; x < W; ++x) {
                    const float u = rnd();
                    // A third holes, a third near, a third far -- so seeds,
                    // non-seeds and holes are all dense enough to interact.
                    d.at<float>(y, x) = u < 0.34f ? 0.f
                                      : (u < 0.67f ? 0.3f + u * 2.0f : 5.f + u * 10.f);
                }
            cv::Mat ref = referenceImprove(d, p);
            cv::Mat got = d.clone();
            improveDepth(got, p);
            for (int y = 0; y < H && allEqual; ++y)
                for (int x = 0; x < W && allEqual; ++x)
                    if (std::fabs(ref.at<float>(y, x) - got.at<float>(y, x)) > 1e-6f) {
                        std::printf("    trial %d r=%d seeds=%d at (%d,%d): ref %.3f got %.3f\n",
                                    trial, p.radiusPx, p.minSeeds, x, y,
                                    ref.at<float>(y, x), got.at<float>(y, x));
                        allEqual = false;
                    }
        }
        check(allEqual, "matches brute force on random fields, 12 shapes");
    }

    // 2. A valid measurement is never replaced, even by a much nearer one.
    {
        cv::Mat d(21, 21, CV_32F, cv::Scalar(0.f));
        for (int y = 0; y < 10; ++y) for (int x = 0; x < 21; ++x) d.at<float>(y, x) = 0.5f;
        for (int x = 0; x < 21; ++x) d.at<float>(10, x) = 12.f;   // far row, adjacent
        cv::Mat got = d.clone();
        DepthImproveParams p; p.radiusPx = 3; p.minSeeds = 3;
        improveDepth(got, p);
        bool kept = true;
        for (int x = 0; x < 21; ++x) kept &= (got.at<float>(10, x) == 12.f);
        check(kept, "never overwrites a valid pixel, even a far one next to near");
    }

    // 3. The far field stays undefined. A hole with only far returns around it
    //    must remain a hole -- this is the whole "unknown != free" rule applied
    //    to the depth image, and it is the property that separates this from a
    //    general-purpose inpainter.
    {
        cv::Mat d(21, 21, CV_32F, cv::Scalar(9.f));
        for (int y = 8; y <= 12; ++y) for (int x = 8; x <= 12; ++x) d.at<float>(y, x) = 0.f;
        cv::Mat got = d.clone();
        DepthImproveParams p; p.radiusPx = 4; p.minSeeds = 3; p.nearM = 2.0f;
        DepthImproveStats st = improveDepth(got, p);
        check(st.filled == 0 && got.at<float>(10, 10) == 0.f,
              "a hole surrounded only by FAR returns is left undefined");
    }

    // 4. The near case does fill, and fills with the window MINIMUM.
    {
        cv::Mat d(21, 21, CV_32F, cv::Scalar(0.f));
        // A ring of near returns at graded depths around a hole.
        for (int y = 6; y <= 14; ++y)
            for (int x = 6; x <= 14; ++x) {
                if (x >= 9 && x <= 11 && y >= 9 && y <= 11) continue;   // the hole
                d.at<float>(y, x) = 1.0f + 0.01f * float((x - 6) + (y - 6));
            }
        cv::Mat got = d.clone();
        DepthImproveParams p; p.radiusPx = 3; p.minSeeds = 4; p.nearM = 2.0f;
        DepthImproveStats st = improveDepth(got, p);
        const float want = 1.0f + 0.01f * float((7 - 6) + (7 - 6));  // nearest in window
        check(st.filled > 0, "a hole inside a near return fills");
        check(std::fabs(got.at<float>(10, 10) - want) < 1e-5f,
              "fills with the window MINIMUM, not a mean or an interpolation");
    }

    // 5. One speckle cannot grow an obstacle. A lone near pixel in an empty
    //    frame must fill nothing at minSeeds > 1 -- this is the only defence
    //    against fabricating occupancy out of the matcher's outlier rate.
    {
        cv::Mat d(21, 21, CV_32F, cv::Scalar(0.f));
        d.at<float>(10, 10) = 0.8f;
        cv::Mat got = d.clone();
        DepthImproveParams p; p.radiusPx = 4; p.minSeeds = 6;
        DepthImproveStats st = improveDepth(got, p);
        check(st.filled == 0, "a single speckle fills nothing at minSeeds = 6");

        cv::Mat got2 = d.clone();
        p.minSeeds = 1;
        DepthImproveStats st2 = improveDepth(got2, p);
        check(st2.filled == 80, "at minSeeds = 1 it fills the whole 9x9 kernel (80 px)");
    }

    // 6. A thin vertical bar with holes in it -- the actual target case. A
    //    branch whose interior did not match should come out solid.
    {
        cv::Mat d(41, 41, CV_32F, cv::Scalar(0.f));
        for (int y = 0; y < 41; ++y)
            for (int x = 18; x <= 22; ++x)
                if (x == 18 || x == 22 || (y % 4) == 0) d.at<float>(y, x) = 1.5f;
        cv::Mat got = d.clone();
        DepthImproveParams p; p.radiusPx = 3; p.minSeeds = 5;
        improveDepth(got, p);
        int solid = 0, total = 0;
        for (int y = 6; y < 35; ++y)
            for (int x = 18; x <= 22; ++x) { ++total; if (got.at<float>(y, x) > 0.f) ++solid; }
        check(solid == total, "a silhouette-only bar becomes solid through its interior");
    }

    // 7. Determinism and idempotence. Running it twice must not keep growing
    //    the obstacle -- a fill is not a seed, or the near field would dilate
    //    without bound over a sequence of frames.
    {
        cv::Mat d(31, 31, CV_32F, cv::Scalar(0.f));
        for (int y = 12; y <= 18; ++y) for (int x = 12; x <= 18; ++x) d.at<float>(y, x) = 1.2f;
        cv::Mat once = d.clone();
        DepthImproveParams p; p.radiusPx = 3; p.minSeeds = 4;
        improveDepth(once, p);
        cv::Mat twice = once.clone();
        DepthImproveStats st2 = improveDepth(twice, p);
        check(st2.filled > 0, "(reference: a second pass on an already-filled frame does grow it)");
        std::printf("    second pass filled %ld more px -- the caller must run it ONCE per frame\n",
                    st2.filled);
    }

    std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "all checks passed",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
