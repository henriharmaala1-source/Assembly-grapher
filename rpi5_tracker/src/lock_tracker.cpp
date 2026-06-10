#include "lock_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>

const char* backend_name(Backend b) {
    switch (b) {
        case Backend::CSRT: return "CSRT";
        case Backend::KCF:  return "KCF";
        case Backend::FLOW: return "FLOW";
    }
    return "?";
}

// ------------------------------------------------------------ LKFlowTracker

namespace {

float median_of(std::vector<float> v) {
    if (v.empty()) return 0.f;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

}  // namespace

std::vector<cv::Point2f> LKFlowTracker::detect(const cv::Mat& gray,
                                               const cv::Rect& box) const {
    const cv::Rect r = box & cv::Rect(0, 0, gray.cols, gray.rows);
    if (r.empty()) return {};
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
    mask(r) = 255;
    std::vector<cv::Point2f> pts;
    cv::goodFeaturesToTrack(gray, pts, 80, 0.01, 5, mask, 7);
    return pts;
}

bool LKFlowTracker::init(const cv::Mat& frame, const cv::Rect& box) {
    box_ = box;
    cv::cvtColor(frame, prevGray_, cv::COLOR_BGR2GRAY);
    pts_ = detect(prevGray_, box_);
    return true;
}

bool LKFlowTracker::update(const cv::Mat& frame, cv::Rect& box) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Re-seed if too few points remain to be reliable.
    if ((int)pts_.size() < MIN_POINTS)
        pts_ = detect(prevGray_, box_);
    if ((int)pts_.size() < MIN_POINTS) {
        prevGray_ = gray;
        box = box_;
        return false;
    }

    const cv::Size  win(21, 21);
    const cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                30, 0.01);

    std::vector<cv::Point2f> nxt, back;
    std::vector<uchar> st, stB;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevGray_, gray, pts_, nxt, st, err, win, 3, crit);

    // Forward-backward check: track results back to the previous frame and
    // reject points that don't land where they started. Points that slid
    // onto the background still "track" — this is what catches them.
    cv::calcOpticalFlowPyrLK(gray, prevGray_, nxt, back, stB, err, win, 3, crit);

    std::vector<cv::Point2f> goodOld, goodNew;
    goodOld.reserve(pts_.size());
    goodNew.reserve(pts_.size());
    for (size_t i = 0; i < pts_.size(); ++i) {
        if (!st[i] || !stB[i]) continue;
        if (cv::norm(pts_[i] - back[i]) >= FB_MAX_ERR) continue;
        goodOld.push_back(pts_[i]);
        goodNew.push_back(nxt[i]);
    }

    if ((int)goodNew.size() < MIN_POINTS) {
        prevGray_ = gray;
        pts_.clear();
        box = box_;
        return false;
    }

    std::vector<float> dxs, dys;
    dxs.reserve(goodNew.size());
    dys.reserve(goodNew.size());
    for (size_t i = 0; i < goodNew.size(); ++i) {
        dxs.push_back(goodNew[i].x - goodOld[i].x);
        dys.push_back(goodNew[i].y - goodOld[i].y);
    }
    const float dx = median_of(dxs);
    const float dy = median_of(dys);

    // Scale estimation: compare median point-spread before and after.
    // When the object approaches or recedes the spread ratio gives the zoom
    // factor. Cap to ±18% per frame to prevent runaway box drift.
    cv::Point2f cOld(0, 0), cNew(0, 0);
    for (const auto& p : goodOld) cOld += p;
    for (const auto& p : goodNew) cNew += p;
    cOld *= 1.f / goodOld.size();
    cNew *= 1.f / goodNew.size();

    std::vector<float> spreadOldV, spreadNewV;
    for (const auto& p : goodOld) spreadOldV.push_back((float)cv::norm(p - cOld));
    for (const auto& p : goodNew) spreadNewV.push_back((float)cv::norm(p - cNew));
    const float spreadOld = median_of(spreadOldV);
    const float spreadNew = median_of(spreadNewV);

    if (spreadOld > 2.f) {
        const float scale = std::clamp(spreadNew / (spreadOld + 1e-6f), 0.82f, 1.18f);
        const float cx = box_.x + box_.width / 2.f + dx;
        const float cy = box_.y + box_.height / 2.f + dy;
        const int wNew = std::max(20, (int)std::lround(box_.width * scale));
        const int hNew = std::max(20, (int)std::lround(box_.height * scale));
        box_ = cv::Rect((int)std::lround(cx - wNew / 2.f),
                        (int)std::lround(cy - hNew / 2.f), wNew, hNew);
    } else {
        box_.x += (int)std::lround(dx);
        box_.y += (int)std::lround(dy);
    }

    prevGray_ = gray;
    pts_ = std::move(goodNew);
    box = box_;
    return true;
}

// ------------------------------------------------------------ LockOnTracker

bool LockOnTracker::init(const cv::Mat& frame, cv::Point center,
                         Backend backend, int boxSize) {
    const int bs = boxSize;
    const int x  = std::clamp(center.x - bs / 2, 0, std::max(0, frame.cols - bs));
    const int y  = std::clamp(center.y - bs / 2, 0, std::max(0, frame.rows - bs));
    const cv::Rect box(x, y, bs, bs);

    cvTracker_.release();
    lkTracker_.reset();

    try {
        if (backend == Backend::FLOW) {
            lkTracker_ = std::make_unique<LKFlowTracker>();
            lkTracker_->init(frame, box);
        } else {
            cvTracker_ = (backend == Backend::KCF)
                             ? cv::Ptr<cv::Tracker>(cv::TrackerKCF::create())
                             : cv::Ptr<cv::Tracker>(cv::TrackerCSRT::create());
            cvTracker_->init(frame, box);
        }
    } catch (const cv::Exception&) {
        return false;
    }

    bbox_        = box;
    hasTarget_   = true;
    locked_      = true;
    age_         = 0;
    lossFrames_  = 0;
    totalLosses_ = 0;
    return true;
}

void LockOnTracker::update(const cv::Mat& frame) {
    if (!hasTarget_) return;

    bool ok = false;
    cv::Rect box = bbox_;
    // KCF (and occasionally CSRT) throw cv::Exception instead of returning
    // false when the box leaves the frame — record it as a loss, the whole
    // point of this harness is counting failures, not crashing on them.
    try {
        ok = lkTracker_ ? lkTracker_->update(frame, box)
                        : cvTracker_->update(frame, box);
    } catch (const cv::Exception&) {
        ok = false;
    }

    if (ok) {
        bbox_ = box;
        ++age_;
        if (lossFrames_ > 0) ++totalLosses_;
        lossFrames_ = 0;
        locked_     = true;
    } else {
        ++lossFrames_;
        if (lossFrames_ >= LOSS_TIMEOUT) locked_ = false;
    }
}

void LockOnTracker::reset() {
    cvTracker_.release();
    lkTracker_.reset();
    bbox_        = cv::Rect();
    hasTarget_   = false;
    locked_      = false;
    age_         = 0;
    lossFrames_  = 0;
    totalLosses_ = 0;
}
