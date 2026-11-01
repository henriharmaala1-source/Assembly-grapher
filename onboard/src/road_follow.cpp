#include "road_follow.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

void RoadFollowModule::run(const cv::Mat& frame, WorldModel& wm) {
    // ---- 0. pre-process: half-res, denoise, CIELab (shadow tolerant) --------
    cv::Mat small, lab;
    cv::resize(frame, small, {W, H}, 0, 0, cv::INTER_AREA);
    cv::GaussianBlur(small, small, {3, 3}, 0);
    cv::cvtColor(small, lab, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> ch;        // L, a, b  (CV_8U)
    cv::split(lab, ch);

    cv::Mat lap, texAbs;
    cv::Laplacian(ch[0], lap, CV_32F, 3);
    texAbs = cv::abs(lap);          // local texture energy

    // feature planes as float32: [a, b, L, texture]
    cv::Mat f[4];
    ch[1].convertTo(f[0], CV_32F);
    ch[2].convertTo(f[1], CV_32F);
    ch[0].convertTo(f[2], CV_32F);
    texAbs.convertTo(f[3], CV_32F);

    // ---- 1. self-sample the road model from the bottom-centre seed ROI ------
    const cv::Rect seed(W / 4, H * 3 / 4, W / 2, H / 4);
    for (int c = 0; c < 4; ++c) {
        cv::Scalar mm, ss;
        cv::meanStdDev(f[c](seed), mm, ss);
        const float mu = (float)mm[0];
        const float sg = std::max((float)ss[0], 4.0f);   // floor avoids div-by-0
        if (!modelInit_) { mu_[c] = mu; sg_[c] = sg; }
        else {
            mu_[c] = (1 - MODEL_EMA) * mu_[c] + MODEL_EMA * mu;
            sg_[c] = (1 - MODEL_EMA) * sg_[c] + MODEL_EMA * sg;
        }
    }
    modelInit_ = true;

    // ---- 2. road-likeness score = exp(-0.5 * Mahalanobis-lite) --------------
    // Down-weight L (illumination) so shadows don't read as "not road".
    const float wts[4] = {1.0f, 1.0f, 0.3f, 0.5f};
    cv::Mat d2 = cv::Mat::zeros(H, W, CV_32F);
    for (int c = 0; c < 4; ++c) {
        cv::Mat z = (f[c] - mu_[c]) / sg_[c];
        d2 += z.mul(z) * wts[c];
    }
    cv::Mat score;
    cv::exp(-0.5f * d2, score);               // float [0,1]

    cv::Mat mask;
    cv::threshold(score, mask, 0.30, 255, cv::THRESH_BINARY);
    mask.convertTo(mask, CV_8U);
    const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  k5);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k5);

    // ---- 3. keep the component under the camera (bottom-centre) -------------
    cv::Mat labels, stats, cent;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, cent);
    int pick = labels.at<int>(H - 2, W / 2);
    if (pick == 0) {                           // seed not on a component — take largest
        int best = 0; long bestA = 0;
        for (int i = 1; i < n; ++i) {
            const long a = stats.at<int>(i, cv::CC_STAT_AREA);
            if (a > bestA) { bestA = a; best = i; }
        }
        pick = best;
    }
    cv::Mat road = (pick > 0) ? (labels == pick) : cv::Mat::zeros(H, W, CV_8U);

    // ---- 4. per-row centreline ---------------------------------------------
    cv::Mat roadF;
    road.convertTo(roadF, CV_32F, 1.0 / 255.0);

    static cv::Mat xramp;                       // 0..W-1 broadcast over rows (cached)
    if (xramp.rows != H) {
        cv::Mat row(1, W, CV_32F);
        for (int x = 0; x < W; ++x) row.at<float>(0, x) = (float)x;
        cv::repeat(row, H, 1, xramp);
    }
    cv::Mat rowMass, rowNum;
    cv::reduce(roadF,              rowMass, 1, cv::REDUCE_SUM, CV_32F);  // Hx1
    cv::reduce(roadF.mul(xramp),   rowNum,  1, cv::REDUCE_SUM, CV_32F);  // Hx1

    std::vector<std::pair<int, float>> line;    // (y, cx) for rows with enough road
    const float minMass = 6.f;
    for (int y = 0; y < H; ++y) {
        const float m = rowMass.at<float>(y, 0);
        if (m >= minMass) line.push_back({y, rowNum.at<float>(y, 0) / m});
    }

    // ---- 5. offset, heading, confidence ------------------------------------
    WorldState patch;
    if (line.size() >= 8) {
        // line is top→bottom (y ascending). near = large y, far = small y.
        const int K = std::max(2, (int)line.size() / 4);
        float nearCx = 0, farCx = 0;
        for (int i = 0; i < K; ++i) {                       // far band (top)
            farCx += line[i].second;
            nearCx += line[line.size() - 1 - i].second;     // near band (bottom)
        }
        nearCx /= K; farCx /= K;

        const float half   = W / 2.0f;
        float offset  = clampf((nearCx - half) / half, -1.f, 1.f);
        float heading = clampf((farCx - nearCx) / half, -1.f, 1.f);

        // confidence = coverage * separability * straightness * temporal
        const float coverage = clampf((float)line.size() / (H * 0.6f), 0.f, 1.f);

        const float inRoad = (float)cv::mean(score, road)[0];
        cv::Mat edges = cv::Mat::zeros(H, W, CV_8U);
        edges(cv::Rect(0, 0, W / 6, H)).setTo(255);
        edges(cv::Rect(W - W / 6, 0, W / 6, H)).setTo(255);
        const float atEdge = (float)cv::mean(score, edges)[0];
        const float separability = clampf(inRoad - atEdge, 0.f, 1.f);

        float mean = 0; for (auto& p : line) mean += p.second; mean /= line.size();
        float var = 0;  for (auto& p : line) var += (p.second - mean) * (p.second - mean);
        var /= line.size();
        const float straightness = clampf(1.0f - std::sqrt(var) / half, 0.f, 1.f);

        const float temporal = haveOutput_
            ? clampf(1.0f - std::fabs(offset - prevOffset_), 0.f, 1.f) : 0.7f;

        float conf = coverage * separability * straightness * temporal;

        if (haveOutput_) {
            offset  = (1 - OUT_EMA) * prevOffset_  + OUT_EMA * offset;
            heading = (1 - OUT_EMA) * prevHeading_ + OUT_EMA * heading;
        }
        prevOffset_ = offset; prevHeading_ = heading; haveOutput_ = true;

        patch.roadValid   = conf > 0.15f;
        patch.roadOffset  = offset;
        patch.roadHeading = heading;
        patch.roadConf    = conf;
    } else {
        haveOutput_ = false;
        patch.roadValid = false;
        patch.roadConf  = 0.f;
    }

    wm.with([&](WorldState& s) {
        s.roadValid   = patch.roadValid;
        s.roadOffset  = patch.roadOffset;
        s.roadHeading = patch.roadHeading;
        s.roadConf    = patch.roadConf;
        s.roadStampS  = monoNowS();
    });
}
