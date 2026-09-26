#include "perception.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdio>

namespace {
// Publish DepthNav's openness histogram as the metric-ish polar scan the P5b
// occupancy grid integrates (openness×maxRange = clearance in metres; a bin at
// maxRange is a miss). Metric on the ToF path, nominal-scale on monocular.
void publishScan(WorldState& s, DepthNav& nav) {
    const auto& h = nav.openHist();
    if (h.empty()) { s.corridorScanN = 0; return; }
    const int n  = std::min((int)h.size(), WorldState::kScanMax);
    const float mx = nav.scanMaxM();
    for (int i = 0; i < n; ++i) {
        const int src = (n == 1) ? 0
            : (int)((float)i * (h.size() - 1) / (n - 1) + 0.5f);
        s.corridorScan[i] = std::max(0.f, std::min(1.f, h[src])) * mx;
    }
    s.corridorScanN      = n;
    s.corridorScanFovDeg = nav.hFovDeg();
    s.corridorScanMaxM   = mx;
}
}  // namespace

// =========================================================== TrackModule

TrackModule::TrackModule(Backend backend, int boxSize)
    : backend_(backend), boxSize_(boxSize) {}

void TrackModule::requestLock(cv::Point center) {
    pendingPt_   = center;
    pendingLock_ = true;
}

void TrackModule::rebuild() {
    core_ = (coreKind_ == Core::Fused) ? track::makeFusedCore()
                                       : track::makeLegacyCore(backend_);
}

void TrackModule::setCore(Core c) { coreKind_ = c; rebuild(); }

void TrackModule::setBackend(Backend b) {
    backend_ = b;
    rebuild();
}

void TrackModule::reset() { if (core_) core_->reset(); }

void TrackModule::run(const cv::Mat& frame, WorldModel& wm) {
    if (!core_) rebuild();
    // THIS IS THE PROCESSING CLOCK, NOT THE CAPTURE CLOCK, and the difference
    // is the one the tracker was just taught to care about. run() is handed a
    // cv::Mat with no timestamp on it, so the best available answer is "now",
    // which silently folds however long the frame spent in the camera stack
    // and the queue into zero. Closing that means carrying a capture time on
    // the frame itself, from the source; until then this is an upper bound on
    // freshness and is labelled as one rather than being called capture time.
    const double tNow = monoNowS();
    if (pendingLock_) {
        pendingLock_ = false;
        core_->designate(frame, pendingPt_, boxSize_);
    }
    const track::TrackObs o = core_->update(frame, tNow);

    wm.with([&](WorldState& s) {
        s.targetValid  = o.valid;
        s.targetLocked = o.locked;
        s.targetCoast  = o.coasting;
        s.targetBox    = o.box;
        s.targetVel    = o.vel;
        s.targetConf   = o.conf;
        s.targetAge    = o.age;
        s.targetLosses = o.losses;
        s.targetStampS = o.captureSec;
        // HOW LONG SINCE ANYTHING WAS ACTUALLY SEEN. targetStampS only says
        // when the tracker last SPOKE, and it speaks every frame whether or
        // not it has measured anything -- COASTING and SEARCHING both produce
        // a box from extrapolation. A consumer deciding how much to trust one
        // needs this, and could not previously compute it.
        s.targetFixAgeS = o.ageOfFixSec;
        s.targetCore    = o.core;
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
    float vehPitch = 0.f;
    // Feed the EFFECTIVE camera pitch (airframe + static mount up-tilt) into the
    // de-rotation, so the constant FPV up-tilt is compensated, not just dynamic
    // airframe pitch.
    { const auto s = wm.snapshot(); vehPitch = s.vehPitchDeg;
      nav_.setAttitude(s.vehRollDeg, s.vehPitchDeg + mountTiltDeg_); }
    nav_.update(frame);
    const auto& t = nav_.traverse();

    // Effective camera elevation above horizontal: up-tilt at hover, lowered as
    // the airframe pitches nose-down to fly forward. (Assumes vehPitchDeg > 0 =
    // nose-down/forward; flip the sign of camera.mount_tilt_deg if your FC
    // reports the opposite.) Outside the usable window the mono view is sky/
    // ground, not the forward obstacle field — suppress the grid scan.
    const float camElev = mountTiltDeg_ - vehPitch;
    const bool  scanUsable = (camElev <= camUpMaxDeg_) && (camElev >= -camDownMaxDeg_);

    wm.with([&](WorldState& s) {
        s.corridorValid    = t.valid;
        s.corridorDecisive = t.valid && t.margin >= 0.04f;
        s.corridorHeading  = t.point;
        s.corridorOffset   = (t.point.x - frame.cols * 0.5f) / (frame.cols * 0.5f);
        s.corridorOpen     = t.openness;
        s.corridorMargin   = t.margin;
        s.corridorStampS   = monoNowS();
        if (scanUsable) publishScan(s, nav_);   // else leave the grid uninformed
        else            s.corridorScanN = 0;
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
        s.corridorStampS   = monoNowS();
        publishScan(s, nav_);
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

    wm.with([&](WorldState& s) {
        s.detections = std::move(result);
        s.detStampS  = monoNowS();
    });
}
