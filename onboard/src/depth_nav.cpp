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

// ------------------------------------------------ metric depth grid (ToF/stereo)

bool DepthNav::updateFromGrid(const cv::Mat& metricGrid, const cv::Size& frameSize,
                              float maxRangeM) {
    if (metricGrid.empty() || maxRangeM <= 0.f) return false;

    // Convert metres (higher = farther) → the internal openness map [0,1]
    // (higher = farther/more open), the same convention the DNN path produces.
    // A zone with no valid return (<= 0) is treated as BLOCKED (openness 0) so
    // the drone won't fly into unknown space — conservative by design.
    cv::Mat grid;
    metricGrid.convertTo(grid, CV_32F);
    cv::Mat openGrid(grid.size(), CV_32F);
    for (int y = 0; y < grid.rows; ++y) {
        const float* d = grid.ptr<float>(y);
        float*       o = openGrid.ptr<float>(y);
        for (int x = 0; x < grid.cols; ++x)
            o[x] = (d[x] > 0.f) ? std::min(1.f, d[x] / maxRangeM) : 0.f;
    }

    // Upsample the coarse grid (e.g. 54×42 VL53L9, 8×8 VL53L5) to frame size so
    // the sector overlay and VFH+ pipeline are identical to the DNN path.
    cv::resize(openGrid, depthMap_, frameSize, 0, 0, cv::INTER_LINEAR);
    computeSectors(frameSize);
    computeTraverse(frameSize);
    return true;
}

// ----------------------------------------------------- corridor scoring (VFH+)

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

    // 1b. Ego-motion de-rotation: undo the airframe's roll (rotate the map by
    //     -roll) and pitch (shift it vertically), so the horizon band below is
    //     WORLD-level forward regardless of how the drone is banked/pitched.
    //     Yaw needs no compensation here — VFH+ output is a heading relative to
    //     the current frame centre, so yaw is absorbed downstream. BORDER_REPLICATE
    //     extends edge openness rather than injecting black (= false obstacles).
    if (haveAtt_ && (std::fabs(roll_) > 0.5f || std::fabs(pitch_) > 0.5f)) {
        cv::Mat M = cv::getRotationMatrix2D(
            {WORK_W / 2.f, WORK_H / 2.f}, -roll_, 1.0);       // de-roll
        M.at<double>(1, 2) += (pitch_ / vFovDeg_) * WORK_H;  // de-pitch (vertical)
        cv::warpAffine(clear, clear, M, clear.size(),
                       cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    }

    // 2. VFH+ : collapse the clearance field to a 1-D polar openness histogram
    //    over headings. Use a horizon band so the (near) floor and ceiling don't
    //    blanket every column as blocked. openCol[x] = mean openness at heading x,
    //    bestRow[x] = the most-open height there (for the arrow tip).
    const int r0 = WORK_H / 5, r1 = WORK_H - WORK_H / 5;   // middle 60% band
    std::vector<float> openCol(WORK_W, 0.f);
    std::vector<int>   bestRow(WORK_W, WORK_H / 2);
    float maxO = 1e-6f, meanO = 0.f;
    for (int x = 0; x < WORK_W; ++x) {
        float sum = 0.f, best = -1.f; int br = WORK_H / 2;
        for (int y = r0; y < r1; ++y) {
            const float v = clear.at<float>(y, x);
            sum += v;
            if (v > best) { best = v; br = y; }
        }
        openCol[x] = sum / (r1 - r0);
        bestRow[x] = br;
        meanO += openCol[x];
        if (openCol[x] > maxO) maxO = openCol[x];
    }
    meanO /= WORK_W;

    // 3. Binary histogram with hysteresis (free/blocked thresholds relative to
    //    the frame's own max openness) so a heading doesn't flicker open/closed.
    if ((int)blocked_.size() != WORK_W) blocked_.assign(WORK_W, 0);
    const float tauFree  = VFH_FREE_FRAC  * maxO;
    const float tauBlock = VFH_BLOCK_FRAC * maxO;
    std::vector<uint8_t> blk(WORK_W);
    for (int x = 0; x < WORK_W; ++x) {
        if      (openCol[x] >= tauFree)  blk[x] = 0;
        else if (openCol[x] <= tauBlock) blk[x] = 1;
        else                             blk[x] = blocked_[x];   // hold (hysteresis)
    }
    blocked_ = blk;   // store un-widened state for next frame's hysteresis

    // 4. Vehicle-width compensation: widen blocked sectors by half a body so the
    //    chosen valley is actually wide enough to fly through.
    std::vector<uint8_t> blkW(blk);
    const int half = std::max(1, WORK_W / 16);
    for (int x = 0; x < WORK_W; ++x)
        if (blk[x])
            for (int d = -half; d <= half; ++d) {
                const int xx = x + d;
                if (xx >= 0 && xx < WORK_W) blkW[xx] = 1;
            }

    // 5. Pick the free heading nearest straight-ahead and the previous heading.
    const float ctr = (WORK_W - 1) * 0.5f;
    int   bestX = -1;
    float bestCost = 1e9f;
    for (int x = 0; x < WORK_W; ++x) {
        if (blkW[x]) continue;
        const float cost = VFH_W_FWD  * std::abs(x - ctr) +
                           VFH_W_PREV * std::abs(x - prevCol_);
        if (cost < bestCost) { bestCost = cost; bestX = x; }
    }
    const bool decisive = bestX >= 0;
    if (!decisive) bestX = (int)std::lround(ctr);   // fully blocked → hold centre
    prevCol_ = (float)bestX;

    const cv::Point2f raw(
        (bestX + 0.5f) / WORK_W * frameSize.width,
        (bestRow[bestX] + 0.5f) / WORK_H * frameSize.height);

    // 6. Temporal smoothing through the shared Kalman filter.
    if (!steerKalman_.initialized()) steerKalman_.init(raw);
    steerKalman_.predict();
    const cv::Point2f smooth = steerKalman_.correct(raw);

    traverse_.raw      = raw;
    traverse_.point    = smooth;
    traverse_.openness = openCol[bestX];
    traverse_.margin   = decisive ? std::max(0.f, openCol[bestX] - meanO) : 0.f;
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
