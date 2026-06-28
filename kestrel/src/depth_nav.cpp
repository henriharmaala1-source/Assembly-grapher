#include "depth_nav.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>

const char* depth_backend_name(DepthBackend b) {
    return b == DepthBackend::DEPTH_ANYTHING_V2 ? "DepthAnythingV2" : "MiDaS";
}

// -------------------------------------------------------------------- init

bool DepthNav::init(const std::string& modelPath, DepthBackend backend) {
    try {
        net_ = cv::dnn::readNetFromONNX(modelPath);
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "[depth] failed to load model: %s\n", e.what());
        return false;
    }
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    backend_ = backend;
    if (backend == DepthBackend::MIDAS_SMALL) {
        inputSize_   = {256, 256};
        invertDepth_ = true;   // MiDaS: higher output = closer to camera
    } else {
        inputSize_   = {518, 518};
        invertDepth_ = false;  // DAv2: higher output = farther from camera
    }
    return true;
}

// ------------------------------------------------------------ preprocessing

cv::Mat DepthNav::preprocess(const cv::Mat& frame) const {
    cv::Mat resized, rgb, fp;
    cv::resize(frame, resized, inputSize_, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(fp, CV_32FC3, 1.0 / 255.0);

    // ImageNet normalisation used by both models.
    const cv::Scalar mean(0.485f, 0.456f, 0.406f);
    const cv::Scalar std (0.229f, 0.224f, 0.225f);
    cv::subtract(fp, mean, fp);
    std::vector<cv::Mat> ch;
    cv::split(fp, ch);
    ch[0] /= (float)std[0];
    ch[1] /= (float)std[1];
    ch[2] /= (float)std[2];
    cv::merge(ch, fp);

    // (H,W,C) → (1,C,H,W)
    return cv::dnn::blobFromImage(fp);
}

// -------------------------------------------------------------- inference

bool DepthNav::update(const cv::Mat& frame) {
    if (net_.empty()) return false;

    net_.setInput(preprocess(frame));
    cv::Mat raw = net_.forward();  // (1,1,H,W) or (1,H,W) depending on model

    // Flatten to (H,W) regardless of output shape.
    const int* sz = raw.size.p;
    const int  nd = raw.dims;
    cv::Mat depth2d;
    if (nd == 4)
        depth2d = cv::Mat(sz[2], sz[3], CV_32F, raw.ptr<float>()).clone();
    else if (nd == 3)
        depth2d = cv::Mat(sz[1], sz[2], CV_32F, raw.ptr<float>()).clone();
    else
        depth2d = raw.reshape(1, (int)std::sqrt((double)raw.total())).clone();

    // Normalise to [0,1].
    double mn, mx;
    cv::minMaxLoc(depth2d, &mn, &mx);
    if (mx - mn < 1e-6) {
        depthMap_ = cv::Mat::zeros(frame.size(), CV_32F);
        sectors_.valid = false;
        return true;
    }
    cv::Mat norm;
    depth2d.convertTo(norm, CV_32F, 1.0 / (mx - mn), -mn / (mx - mn));

    // MiDaS convention: higher = closer — invert so that higher always = farther.
    if (invertDepth_) norm = 1.0f - norm;

    cv::resize(norm, depthMap_, frame.size(), 0, 0, cv::INTER_LINEAR);
    computeSectors(frame.size());
    computeTraverse(frame.size());
    return true;
}

// ----------------------------------------------------- corridor scoring

void DepthNav::buildBias(const cv::Size& sz) {
    // Radial falloff from centre: 1.0 at centre → (1 − BIAS_K) at the corners.
    // Multiplying the clearance field by this makes straight-ahead win ties.
    bias_.create(sz, CV_32F);
    const float cx = sz.width  / 2.f;
    const float cy = sz.height / 2.f;
    const float rmax = std::sqrt(cx * cx + cy * cy);
    for (int y = 0; y < sz.height; ++y) {
        float* row = bias_.ptr<float>(y);
        for (int x = 0; x < sz.width; ++x) {
            const float dx = x - cx, dy = y - cy;
            const float r  = std::sqrt(dx * dx + dy * dy) / rmax;
            row[x] = 1.f - BIAS_K * r;
        }
    }
}

void DepthNav::computeTraverse(const cv::Size& frameSize) {
    // Work on a small copy — corridor scoring doesn't need full resolution.
    cv::Mat small;
    cv::resize(depthMap_, small, {WORK_W, WORK_H}, 0, 0, cv::INTER_AREA);

    // 1. Clearance field: blur with a kernel ~ vehicle width. An isolated far
    //    pixel (needle gap) is averaged down; only a far region with open
    //    margin around it stays bright.
    const int k = (WORK_W / 6) | 1;            // odd kernel
    cv::Mat clear;
    cv::GaussianBlur(small, clear, {k, k}, 0);

    // 2. Forward bias: prefer straight-ahead when corridors tie.
    if (bias_.size() != clear.size()) buildBias(clear.size());
    cv::Mat steer = clear.mul(bias_);

    double mn, mx;
    cv::Point mxLoc;
    cv::minMaxLoc(steer, &mn, &mx, nullptr, &mxLoc);
    const float meanS = (float)cv::mean(steer)[0];

    // Map the peak back to frame coordinates (cell centre).
    const cv::Point2f raw(
        (mxLoc.x + 0.5f) / WORK_W * frameSize.width,
        (mxLoc.y + 0.5f) / WORK_H * frameSize.height);

    // 3. Temporal smoothing through the shared Kalman filter.
    if (!steerKalman_.initialized()) steerKalman_.init(raw);
    steerKalman_.predict();
    const cv::Point2f smooth = steerKalman_.correct(raw);

    traverse_.raw      = raw;
    traverse_.point    = smooth;
    traverse_.openness = clear.at<float>(mxLoc);  // clearance at peak
    traverse_.margin   = (float)(mx - meanS);     // decisiveness
    traverse_.valid    = true;
}

// --------------------------------------------------------- sector analysis

void DepthNav::computeSectors(const cv::Size& frameSize) {
    const int cellW = frameSize.width  / SectorMap::COLS;
    const int cellH = frameSize.height / SectorMap::ROWS;

    float best = -1.f;
    for (int r = 0; r < SectorMap::ROWS; ++r) {
        for (int c = 0; c < SectorMap::COLS; ++c) {
            const cv::Rect roi(c * cellW, r * cellH, cellW, cellH);
            sectors_.scores[r][c] = (float)cv::mean(depthMap_(roi))[0];
            if (sectors_.scores[r][c] > best) {
                best             = sectors_.scores[r][c];
                sectors_.bestRow = r;
                sectors_.bestCol = c;
            }
        }
    }
    sectors_.valid = true;
}

// ----------------------------------------------------------------- overlay

void DepthNav::drawOverlay(cv::Mat& frame) const {
    if (!sectors_.valid) return;

    const int W = frame.cols, H = frame.rows;
    const int cellW = W / SectorMap::COLS;
    const int cellH = H / SectorMap::ROWS;

    // Coloured sector rectangles: green (open) → red (blocked).
    for (int r = 0; r < SectorMap::ROWS; ++r) {
        for (int c = 0; c < SectorMap::COLS; ++c) {
            const float s = sectors_.scores[r][c];
            // green channel scales with openness, red with closedness
            const cv::Scalar col(0, (int)(s * 220), (int)((1.f - s) * 220));
            const cv::Rect   roi(c * cellW, r * cellH, cellW, cellH);
            cv::Mat overlay = frame(roi).clone();
            cv::rectangle(overlay, cv::Rect(0, 0, cellW, cellH), col, cv::FILLED);
            cv::addWeighted(frame(roi), 0.75, overlay, 0.25, 0, frame(roi));
            cv::rectangle(frame, roi, {200, 200, 200}, 1);

            // Score label
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%.2f", s);
            cv::putText(frame, buf,
                        {roi.x + 4, roi.y + cellH - 6},
                        cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        {255, 255, 255}, 1, cv::LINE_AA);
        }
    }

    // Corridor arrow: frame centre → smoothed traverse target.
    const cv::Point frameCentre(W / 2, H / 2);
    const bool decisive = traverse_.valid && traverse_.margin >= MIN_MARGIN;

    if (traverse_.valid) {
        const cv::Point target((int)traverse_.point.x, (int)traverse_.point.y);
        const cv::Point2f dir((float)(target.x - frameCentre.x),
                              (float)(target.y - frameCentre.y));
        const float len = (float)std::sqrt(dir.x * dir.x + dir.y * dir.y);

        // Decisive = green; indecisive (everything equally open) = amber.
        const cv::Scalar col = decisive ? cv::Scalar(0, 255, 128)
                                        : cv::Scalar(0, 200, 255);

        if (len > 4.f) {
            const float tip = std::max(len - 18.f, len * 0.6f);
            const cv::Point endpoint(
                (int)std::lround(frameCentre.x + dir.x / len * tip),
                (int)std::lround(frameCentre.y + dir.y / len * tip));
            cv::arrowedLine(frame, frameCentre, endpoint, col,
                            decisive ? 3 : 2, cv::LINE_AA, 0, 0.28);
        }
        // Ring at the target so the steer point is visible even when centred.
        cv::circle(frame, target, 6, col, decisive ? 2 : 1, cv::LINE_AA);
    }

    // Backend + decision readout (bottom-left, above the tracker HUD line).
    char lbl[64];
    std::snprintf(lbl, sizeof(lbl), "depth: %s   %s  open %.0f%%",
                  depth_backend_name(backend_),
                  decisive ? "TRAVERSE" : "SCANNING",
                  traverse_.openness * 100.f);
    cv::putText(frame, lbl, {8, H - 28},
                cv::FONT_HERSHEY_SIMPLEX, 0.42, {200, 200, 200}, 1, cv::LINE_AA);
}
