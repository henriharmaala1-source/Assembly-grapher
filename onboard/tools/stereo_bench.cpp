// ---------------------------------------------------------------------------
// stereo_bench -- does CPU stereo fit in the flight budget, and at what range?
//
// WHY THIS EXISTS. The whole IMX296 wide-baseline bet rests on one unmeasured
// number: how long stereo matching takes on a Pi 5 CPU. A D435i computes depth
// on its own ASIC for zero host cost; a raw stereo pair does not. If matching
// costs 150 ms we cannot fly, and the honest answer is to buy a RealSense and
// accept 2-3 m of range.
//
// This needs NO CAMERAS. Run it today, on the Pi, and get the answer.
//
// THE TRADE IT MEASURES, which is the point. Resolution buys range and costs
// compute, but NOT at the same rate:
//
//   Z_max    = sqrt(cell * f * B / sigma_d)        ~ sqrt(scale)
//   cost     ~ W * H * D,  and D = f*B/Z_min       ~ scale^3
//
// So halving resolution is 8x cheaper and only costs 29% of the range. That
// asymmetry is the single most useful thing in the output table, and it is why
// this reports Z_max next to milliseconds instead of just timing things.
//
// WHAT IT IS NOT. This measures OPENCV's StereoBM/StereoSGBM, which have NEON
// paths on aarch64. It is the baseline any fork has to beat, not the ceiling.
// ReS2tAC (Sensors 21(11):3938) claims hand-written NEON SGM is much faster;
// this tells you how much headroom that would have to buy.
//
// The synthetic scene is a forest: ground plane, far backdrop, and vertical
// trunks at assorted depths with DELIBERATELY LOW TEXTURE, because bark is the
// case that breaks passive stereo and a benchmark on white noise would report
// a quality that does not exist outdoors.
//
//   cmake --build build --target stereo_bench && ./build/stereo_bench
// ---------------------------------------------------------------------------

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct Params {
    // Sensor / optics. Defaults are the Waveshare IMX296 M12 with a 4 mm lens
    // on the 12 cm baseline this project sized its map for.
    int   fullW    = 1456;
    int   fullH    = 1088;
    float pitchUm  = 3.45f;    // pixel pitch at full resolution
    float lensMm   = 4.0f;
    float baseline = 0.12f;    // m

    // Map / honesty parameters -- these must match voxel_map.hpp or the Z_max
    // column is decoration rather than a shared budget.
    float cell     = 0.25f;    // m, voxel size the depth has to resolve
    float sigmaD   = 0.25f;    // px, 1-sigma matching noise
    float derate   = 0.75f;    // measured over-estimate of the closed form

    // Flight budget. The cycle is 100 ms; the planner measured ~16 ms; leave
    // room for capture, map integration, control and the VO that does not
    // exist yet. 40 ms is the number stereo has to come in under.
    float budgetMs = 40.f;
    float zMinM    = 1.0f;     // nearest range that must be resolvable -> sets D

    // Scene
    int   nTrunks  = 12;
    float trunkTex = 0.18f;    // bark contrast relative to ground/foliage
    float noiseSd  = 2.0f;     // sensor noise, 8-bit DN
    unsigned seed  = 7u;

    int   reps     = 7;        // timed repetitions; median reported
    int   threads  = -1;       // -1 = OpenCV default (all cores)
    bool  dump     = false;    // write the pair + disparity as PNGs
};

// --- geometry ---------------------------------------------------------------

float focalPx(const Params& p, int scaleDiv) {
    // Downsampling by k multiplies the effective pixel pitch by k.
    return (p.lensMm * 1000.f / p.pitchUm) / float(scaleDiv);
}

float zMaxM(const Params& p, float fpx) {
    return std::sqrt(p.cell * fpx * p.baseline / p.sigmaD) * p.derate;
}

// OpenCV requires numDisparities to be a positive multiple of 16.
int dispForZmin(const Params& p, float fpx) {
    int need = int(std::ceil(fpx * p.baseline / p.zMinM));
    return std::max(16, ((need + 15) / 16) * 16);
}

// --- synthetic scene --------------------------------------------------------

// Band-limited noise. White noise is the WRONG test signal: every window is
// globally unique, so a matcher looks better than it will ever be in a forest.
// Blurring imposes a correlation length, which is what actually decides whether
// a block has anything to lock onto.
cv::Mat texture(int w, int h, float corrPx, cv::RNG& rng) {
    cv::Mat n(h, w, CV_32F);
    rng.fill(n, cv::RNG::NORMAL, 0.f, 1.f);
    if (corrPx > 0.5f) cv::GaussianBlur(n, n, cv::Size(0, 0), corrPx);
    cv::normalize(n, n, 0.f, 1.f, cv::NORM_MINMAX);
    return n;
}

struct Scene {
    cv::Mat dispGt;    // CV_32F, ground-truth disparity in pixels
    cv::Mat texAmp;    // CV_32F in [0,1], local texture contrast
};

Scene buildScene(const Params& p, int w, int h, float fpx, cv::RNG& rng) {
    Scene s;
    s.dispGt.create(h, w, CV_32F);
    s.texAmp.create(h, w, CV_32F);

    const float cx = w * 0.5f, cy = h * 0.5f;
    const float camH = 1.5f;       // camera height above ground, m
    const float zBg  = 20.f;       // backdrop

    // Trunks as vertical slabs: each owns a span of columns at a fixed depth.
    struct Trunk { float z; int x0, x1; };
    std::vector<Trunk> trunks;
    for (int i = 0; i < p.nTrunks; ++i) {
        float z = 1.5f + rng.uniform(0.f, 1.f) * 10.5f;
        float diam = 0.15f + rng.uniform(0.f, 1.f) * 0.20f;      // 0.15-0.35 m
        int   wpx = std::max(2, int(fpx * diam / z));
        int   xc  = int(rng.uniform(0.f, 1.f) * w);
        trunks.push_back({z, xc - wpx / 2, xc + wpx / 2});
    }

    for (int y = 0; y < h; ++y) {
        // Ground plane below the horizon; +inf above it.
        float zGround = (y > cy + 1.f) ? (fpx * camH / (y - cy)) : 1e9f;
        for (int x = 0; x < w; ++x) {
            float z = std::min(zBg, zGround);
            bool onTrunk = false;
            for (const Trunk& t : trunks)
                if (x >= t.x0 && x < t.x1 && t.z < z) { z = t.z; onTrunk = true; }
            s.dispGt.at<float>(y, x) = fpx * p.baseline / z;
            // Bark is dark and low-contrast; ground litter and foliage are not.
            s.texAmp.at<float>(y, x) = onTrunk ? p.trunkTex : 1.0f;
        }
    }
    return s;
}

// Right image is the texture; left is the right resampled by ground-truth
// disparity, since OpenCV's convention is left(x) == right(x - d).
void renderPair(const Params& p, const Scene& s, int w, int h, float fpx,
                cv::RNG& rng, cv::Mat& L, cv::Mat& R) {
    cv::Mat base = texture(w, h, 1.6f, rng);
    cv::Mat rf(h, w, CV_32F);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            float a = s.texAmp.at<float>(y, x);
            float z = fpx * p.baseline / std::max(1e-3f, s.dispGt.at<float>(y, x));
            // Mean level tracks depth, so a low-texture trunk still has a
            // SILHOUETTE against what is behind it. That edge is the signal the
            // sim's edgeBoost models, and without it a bark test is unfairly
            // hard -- real trunks are visible by outline even when their
            // interior is not.
            float mean = 0.30f + 0.35f * std::min(1.f, std::max(0.f, 1.f - z / 20.f));
            rf.at<float>(y, x) = mean + a * (base.at<float>(y, x) - 0.5f) * 0.55f;
        }

    cv::Mat mapx(h, w, CV_32F), mapy(h, w, CV_32F);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            mapx.at<float>(y, x) = x - s.dispGt.at<float>(y, x);
            mapy.at<float>(y, x) = float(y);
        }
    cv::Mat lf;
    cv::remap(rf, lf, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    auto to8 = [&](const cv::Mat& f, cv::Mat& out) {
        f.convertTo(out, CV_8U, 255.0, 0.0);
        cv::Mat n(out.size(), CV_16S);
        rng.fill(n, cv::RNG::NORMAL, 0.f, p.noiseSd);
        cv::Mat o16; out.convertTo(o16, CV_16S);
        o16 += n;
        o16.convertTo(out, CV_8U);
    };
    to8(lf, L);
    to8(rf, R);
}

// --- matchers ---------------------------------------------------------------

struct Algo { std::string name; int mode; bool bm; int block; };

cv::Ptr<cv::StereoMatcher> makeMatcher(const Algo& a, int numDisp) {
    if (a.bm) {
        cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(numDisp, a.block);
        bm->setPreFilterCap(31);
        bm->setUniquenessRatio(10);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(2);
        bm->setDisp12MaxDiff(1);
        return bm;
    }
    const int ch = 1;
    cv::Ptr<cv::StereoSGBM> sg = cv::StereoSGBM::create(
        0, numDisp, a.block,
        8 * ch * a.block * a.block,     // P1
        32 * ch * a.block * a.block,    // P2
        1,                              // disp12MaxDiff
        31,                             // preFilterCap
        10,                             // uniquenessRatio
        100,                            // speckleWindowSize
        2,                              // speckleRange
        a.mode);
    return sg;
}

struct Result {
    double ms; double validPct; double rmsPx; double badPct;
    // Inlier RMS -- the honest sigma_d. Total RMS is polluted by gross
    // mismatches and occlusion, which are a SEPARATE failure (counted as bad%)
    // from the sub-pixel noise that sets how far we may carve free space.
    double sdPx;
};

Result runOne(const Algo& a, int numDisp, const cv::Mat& L, const cv::Mat& R,
              const cv::Mat& gt, int reps) {
    cv::Ptr<cv::StereoMatcher> m = makeMatcher(a, numDisp);
    cv::Mat d16;

    m->compute(L, R, d16);                       // warm-up, not timed
    std::vector<double> times;
    times.reserve(reps);
    for (int i = 0; i < reps; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        m->compute(L, R, d16);
        auto t1 = std::chrono::steady_clock::now();
        times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    std::sort(times.begin(), times.end());

    // Quality, scored only where the matcher committed to an answer AND the
    // ground truth is inside the search range -- otherwise we would be marking
    // it down for questions it was never asked.
    long nTot = 0, nValid = 0, nBad = 0, nIn = 0;
    double se = 0, seIn = 0;
    for (int y = 0; y < d16.rows; ++y)
        for (int x = 0; x < d16.cols; ++x) {
            float g = gt.at<float>(y, x);
            if (g < 1.f || g > numDisp - 1.f) continue;
            ++nTot;
            float d = d16.at<short>(y, x) / 16.f;
            if (d <= 0.f) continue;
            ++nValid;
            double e = d - g;
            se += e * e;
            if (std::fabs(e) > 1.0) { ++nBad; }
            else                    { ++nIn; seIn += e * e; }
        }

    Result r;
    r.ms       = times[times.size() / 2];
    r.validPct = nTot ? 100.0 * nValid / nTot : 0.0;
    r.rmsPx    = nValid ? std::sqrt(se / nValid) : 0.0;
    r.badPct   = nValid ? 100.0 * nBad / nValid : 0.0;
    r.sdPx     = nIn ? std::sqrt(seIn / nIn) : 0.0;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    Params p;
    std::vector<int> divs = {1, 2, 3, 4, 6};

    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto nextF = [&]() { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
        auto nextI = [&]() { return (i + 1 < argc) ? std::atoi(argv[++i]) : 0; };
        if      (k == "--lens")     p.lensMm   = float(nextF());
        else if (k == "--baseline") p.baseline = float(nextF());
        else if (k == "--cell")     p.cell     = float(nextF());
        else if (k == "--sigma")    p.sigmaD   = float(nextF());
        else if (k == "--zmin")     p.zMinM    = float(nextF());
        else if (k == "--budget")   p.budgetMs = float(nextF());
        else if (k == "--trunktex") p.trunkTex = float(nextF());
        else if (k == "--reps")     p.reps     = nextI();
        else if (k == "--threads")  p.threads  = nextI();
        else if (k == "--seed")     p.seed     = unsigned(nextI());
        else if (k == "--dump")     p.dump     = true;
        else if (k == "--full")     { p.fullW = nextI(); p.fullH = nextI(); }
        else if (k == "--help") {
            std::printf(
                "stereo_bench [--lens mm] [--baseline m] [--cell m] [--sigma px]\n"
                "             [--zmin m] [--budget ms] [--trunktex 0..1]\n"
                "             [--reps n] [--threads n] [--seed n] [--dump]\n");
            return 0;
        } else {
            std::fprintf(stderr, "unknown flag: %s (try --help)\n", k.c_str());
            return 2;
        }
    }

    if (p.threads > 0) cv::setNumThreads(p.threads);

    std::printf("stereo_bench -- OpenCV %s, %d threads\n",
                CV_VERSION, cv::getNumThreads());
    std::printf("sensor %dx%d @ %.2f um, lens %.1f mm, baseline %.0f cm\n",
                p.fullW, p.fullH, p.pitchUm, p.lensMm, p.baseline * 100.f);
    std::printf("map cell %.2f m, sigma_d %.2f px, derate %.2f -> Z_max\n",
                p.cell, p.sigmaD, p.derate);
    std::printf("budget %.0f ms/frame, must resolve from %.1f m -> sets D\n\n",
                p.budgetMs, p.zMinM);

    // BM's block must be odd and >= 5. SGBM likes small blocks; 5 is the usual
    // sweet spot and larger blocks blur thin structure, which is exactly what a
    // forest is made of.
    std::vector<Algo> algos = {
        {"BM",        0,                            true,  15},
        {"SGBM_3WAY", cv::StereoSGBM::MODE_SGBM_3WAY, false, 5},
        {"SGBM",      cv::StereoSGBM::MODE_SGBM,      false, 5},
    };

    std::printf("%-10s %-10s %5s %5s | %8s %6s %5s | %6s %6s %6s\n",
                "res", "algo", "f_px", "D", "ms", "valid%", "bad%",
                "sd_px", "Zmax_a", "Zmax_m");
    std::printf("%s\n", std::string(80, '-').c_str());

    struct Best { std::string res, algo; double ms; float zmax; double bad, sd; };
    std::vector<Best> fits;

    for (int div : divs) {
        const int w = p.fullW / div, h = p.fullH / div;
        if (w < 160 || h < 120) continue;
        const float fpx = focalPx(p, div);
        const float zmx = zMaxM(p, fpx);
        const int   D   = dispForZmin(p, fpx);

        cv::RNG rng(p.seed);
        Scene sc = buildScene(p, w, h, fpx, rng);
        cv::Mat L, R;
        renderPair(p, sc, w, h, fpx, rng, L, R);

        if (p.dump && div == 2) {
            cv::imwrite("stereo_bench_left.png", L);
            cv::imwrite("stereo_bench_right.png", R);
            cv::Mat vis;
            cv::normalize(sc.dispGt, vis, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::imwrite("stereo_bench_gt.png", vis);
        }

        char res[32];
        std::snprintf(res, sizeof res, "%dx%d", w, h);
        for (const Algo& a : algos) {
            Result r = runOne(a, D, L, R, sc.dispGt, p.reps);
            const char* flag = (r.ms <= p.budgetMs) ? " " : "*";
            // Zmax_m re-derives the range from the sigma_d this matcher ACTUALLY
            // delivered, instead of the 0.25 px the map's header assumes. If the
            // two columns disagree, the header is the optimistic one.
            float zmeas = (r.sdPx > 1e-3)
                        ? std::sqrt(p.cell * fpx * p.baseline / float(r.sdPx)) * p.derate
                        : 0.f;
            std::printf("%-10s %-10s %5.0f %5d | %7.1f%s %5.1f%% %4.1f%% | "
                        "%6.2f %5.1fm %5.1fm\n",
                        res, a.name.c_str(), fpx, D,
                        r.ms, flag, r.validPct, r.badPct, r.sdPx, zmx, zmeas);
            if (r.ms <= p.budgetMs)
                fits.push_back({res, a.name, r.ms, zmeas, r.badPct, r.sdPx});
        }
        std::printf("\n");
    }

    std::printf("* = over the %.0f ms budget\n", p.budgetMs);
    std::printf("Zmax_a assumes sigma_d = %.2f px (voxel_map.hpp). Zmax_m uses "
                "the sd_px measured here.\n\n", p.sigmaD);

    if (fits.empty()) {
        std::printf("NOTHING FITS. Every configuration exceeds the budget.\n"
                    "That is the answer: either hand-written NEON SGM (ReS2tAC),\n"
                    "or a RealSense that computes depth in silicon.\n");
        return 0;
    }

    // The useful recommendation is not "fastest" -- it is the most RANGE that
    // still fits, because range is the entire reason for the wide baseline.
    // Ties on range go to the matcher with fewer gross errors: at equal Z_max a
    // cheaper-but-blinder matcher is not the better buy.
    std::sort(fits.begin(), fits.end(), [](const Best& a, const Best& b) {
        if (std::fabs(a.zmax - b.zmax) > 0.05f) return a.zmax > b.zmax;
        return a.bad < b.bad;
    });
    const Best& b = fits.front();
    std::printf("BEST RANGE WITHIN BUDGET: %s %s -- Z_max %.1f m at %.1f ms "
                "(sd %.2f px, %.1f%% bad)\n", b.res.c_str(), b.algo.c_str(),
                b.zmax, b.ms, b.sd, b.bad);
    std::printf("For comparison a D435i is ~2.5 m of honest range at 0 ms of "
                "host CPU.\n");

    // HOW TO READ valid% VS bad%, because for this architecture they are not
    // symmetric and the obvious reading is backwards.
    //
    // A missing pixel is UNKNOWN, and unknown is safe here -- the planner
    // refuses to earn speed through it. A confidently WRONG pixel is what
    // carves free space through a trunk. So a matcher with 70% valid and 0.2%
    // bad beats one with 82% valid and 3% bad, even though the second looks
    // better on a benchmark leaderboard.
    //
    // Lower the bark contrast (--trunktex 0.06) and the two families separate
    // exactly this way: BM declines to guess and loses valid pixels; SGBM
    // interpolates and gains wrong ones.
    std::printf(
        "\nReading the quality columns: a MISSING pixel is unknown, and unknown "
        "is safe\n(the speed budget earns nothing through it). A WRONG pixel is "
        "what carves free\nspace through a trunk. So prefer low bad%% over high "
        "valid%%. Re-run with\n--trunktex 0.06 to see BM and SGBM separate along "
        "exactly that line.\n");

#if !defined(__aarch64__)
    std::printf(
        "\n!! NOT RUNNING ON ARM. These timings are from this build host and\n"
        "!! say NOTHING about the Pi 5. Rebuild and run on the aircraft's own\n"
        "!! computer before believing any verdict above.\n");
#endif
    return 0;
}
