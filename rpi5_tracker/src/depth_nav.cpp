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
    return true;
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

    // Arrow from frame centre toward centre of best sector.
    const cv::Point frameCentre(W / 2, H / 2);
    const cv::Point sectorCentre(
        sectors_.bestCol * cellW + cellW / 2,
        sectors_.bestRow * cellH + cellH / 2);

    // Shorten so the arrowhead doesn't land on top of text.
    const cv::Point2f dir(
        (float)(sectorCentre.x - frameCentre.x),
        (float)(sectorCentre.y - frameCentre.y));
    const float len  = (float)std::sqrt(dir.x * dir.x + dir.y * dir.y);
    const float tip  = std::max(len - 20.f, len * 0.6f);
    const cv::Point endpoint(
        (int)std::lround(frameCentre.x + dir.x / (len + 1e-6f) * tip),
        (int)std::lround(frameCentre.y + dir.y / (len + 1e-6f) * tip));

    cv::arrowedLine(frame, frameCentre, endpoint,
                    {0, 255, 128}, 2, cv::LINE_AA, 0, 0.25);

    // Backend label (bottom-left corner, above the tracker HUD line)
    char lbl[32];
    std::snprintf(lbl, sizeof(lbl), "depth: %s",
                  depth_backend_name(backend_));
    cv::putText(frame, lbl, {8, H - 28},
                cv::FONT_HERSHEY_SIMPLEX, 0.42, {200, 200, 200}, 1, cv::LINE_AA);
}
