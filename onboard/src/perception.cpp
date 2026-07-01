#include "perception.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdio>

// =========================================================== TrackModule

TrackModule::TrackModule(Backend backend, int boxSize)
    : backend_(backend), boxSize_(boxSize) {}

void TrackModule::requestLock(cv::Point center) {
    pendingPt_   = center;
    pendingLock_ = true;
}

void TrackModule::setBackend(Backend b) {
    backend_ = b;
    trk_.reset();
}

void TrackModule::reset() { trk_.reset(); }

void TrackModule::run(const cv::Mat& frame, WorldModel& wm) {
    if (pendingLock_) {
        pendingLock_ = false;
        trk_.init(frame, pendingPt_, backend_, boxSize_);
    }
    if (trk_.hasTarget())
        trk_.update(frame);

    const bool      has = trk_.hasTarget();
    const cv::Rect  b   = trk_.bbox();
    const cv::Point ctr = (b.tl() + b.br()) / 2;
    const cv::Point2f vel = trk_.locked()
        ? trk_.projected(1.f) - cv::Point2f(ctr) : cv::Point2f(0, 0);

    wm.with([&](WorldState& s) {
        s.targetValid  = has;
        s.targetLocked = trk_.locked();
        s.targetCoast  = trk_.coasting();
        s.targetBox    = b;
        s.targetVel    = vel;
        s.targetConf   = trk_.confidence();
        s.targetAge    = trk_.age();
        s.targetLosses = trk_.totalLosses();
    });
}

// ========================================================= NavigateModule

NavigateModule::NavigateModule(const std::string& model, DepthBackend backend) {
    if (!model.empty())
        ready_ = nav_.init(model, backend);
    if (ready_)
        std::printf("[navigate] %s loaded\n", depth_backend_name(backend));
    else if (!model.empty())
        std::fprintf(stderr, "[navigate] failed to load %s\n", model.c_str());
}

void NavigateModule::run(const cv::Mat& frame, WorldModel& wm) {
    if (!ready_) return;
    { const auto s = wm.snapshot(); nav_.setAttitude(s.vehRollDeg, s.vehPitchDeg); }
    nav_.update(frame);
    const auto& t = nav_.traverse();

    wm.with([&](WorldState& s) {
        s.corridorValid    = t.valid;
        s.corridorDecisive = t.valid && t.margin >= 0.04f;
        s.corridorHeading  = t.point;
        s.corridorOffset   = (t.point.x - frame.cols * 0.5f) / (frame.cols * 0.5f);
        s.corridorOpen     = t.openness;
        s.corridorMargin   = t.margin;
    });
}

// ========================================================= TofNavigateModule

TofNavigateModule::TofNavigateModule(std::unique_ptr<ITofSource> src)
    : src_(std::move(src)) {
    nav_.enableTof();
    if (src_)
        std::printf("[tof-navigate] source: %s (max %.1fm)\n",
                    src_->name(), src_->maxRangeM());
}

void TofNavigateModule::run(const cv::Mat& frame, WorldModel& wm) {
    if (!src_ || !src_->read(grid_)) return;   // no fresh ToF frame this tick

    { const auto s = wm.snapshot(); nav_.setAttitude(s.vehRollDeg, s.vehPitchDeg); }
    nav_.updateFromGrid(grid_, frame.size(), src_->maxRangeM());
    const auto& t = nav_.traverse();

    wm.with([&](WorldState& s) {
        s.corridorValid    = t.valid;
        s.corridorDecisive = t.valid && t.margin >= 0.04f;
        s.corridorHeading  = t.point;
        s.corridorOffset   = (t.point.x - frame.cols * 0.5f) / (frame.cols * 0.5f);
        s.corridorOpen     = t.openness;
        s.corridorMargin   = t.margin;
    });
}

// =========================================================== DetectModule

DetectModule::DetectModule(const std::string& model,
                           std::vector<std::string> labels, float confThresh)
    : labels_(std::move(labels)), confThresh_(confThresh) {
    if (model.empty()) return;
    try {
        net_ = cv::dnn::readNetFromONNX(model);
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        ready_ = true;
        std::printf("[detect] %s loaded (%zu classes)\n",
                    model.c_str(), labels_.size());
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "[detect] failed to load model: %s\n", e.what());
    }
}

void DetectModule::run(const cv::Mat& frame, WorldModel& wm) {
    if (!ready_) return;

    cv::Mat blob = cv::dnn::blobFromImage(
        frame, 1.0 / 255.0, {inputSz_, inputSz_}, cv::Scalar(), true, false);
    net_.setInput(blob);

    cv::Mat out = net_.forward();   // YOLOv8: [1, 4+nc, 8400]
    if (out.dims != 3) return;

    // Squeeze to (4+nc, n) then transpose to (n, 4+nc).
    cv::Mat m(out.size[1], out.size[2], CV_32F, out.ptr<float>());
    cv::Mat dets;
    cv::transpose(m, dets);
    const int nc = m.rows - 4;
    if (nc <= 0) return;

    const float sx = (float)frame.cols / inputSz_;
    const float sy = (float)frame.rows / inputSz_;

    std::vector<cv::Rect> boxes;
    std::vector<float>    scores;
    std::vector<int>      classes;

    for (int i = 0; i < dets.rows; ++i) {
        const float* row = dets.ptr<float>(i);
        // best class score
        int   bestC = 0;
        float bestS = row[4];
        for (int c = 1; c < nc; ++c)
            if (row[4 + c] > bestS) { bestS = row[4 + c]; bestC = c; }
        if (bestS < confThresh_) continue;

        const float cx = row[0], cy = row[1], w = row[2], h = row[3];
        cv::Rect box(int((cx - w / 2) * sx), int((cy - h / 2) * sy),
                     int(w * sx), int(h * sy));
        boxes.push_back(box);
        scores.push_back(bestS);
        classes.push_back(bestC);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confThresh_, 0.45f, keep);

    std::vector<Detection> result;
    result.reserve(keep.size());
    for (int idx : keep) {
        Detection d;
        d.box        = boxes[idx];
        d.confidence = scores[idx];
        d.label      = (classes[idx] < (int)labels_.size())
                           ? labels_[classes[idx]]
                           : std::to_string(classes[idx]);
        result.push_back(std::move(d));
    }

    wm.with([&](WorldState& s) { s.detections = std::move(result); });
}
