#include "lock_tracker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>

const char* backend_name(Backend b) {
    switch (b) {
        case Backend::CSRT:  return "CSRT";
        case Backend::KCF:   return "KCF";
        case Backend::FLOW:  return "FLOW";
        case Backend::MOSSE: return "MOSSE";
    }
    return "?";
}

// ---------------------------------------------------------------- helpers

namespace {

float median_of(std::vector<float> v) {
    if (v.empty()) return 0.f;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

cv::Rect clip_rect(const cv::Rect& r, const cv::Size& sz) {
    return r & cv::Rect(0, 0, sz.width, sz.height);
}

cv::Point2f rect_center(const cv::Rect& r) {
    return {r.x + r.width / 2.f, r.y + r.height / 2.f};
}

}  // namespace

// ------------------------------------------------------------ LKFlowTracker

std::vector<cv::Point2f> LKFlowTracker::detect(const cv::Mat& gray,
                                               const cv::Rect& box) const {
    const cv::Rect r = clip_rect(box, gray.size());
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

    if ((int)pts_.size() < MIN_POINTS)
        pts_ = detect(prevGray_, box_);
    if ((int)pts_.size() < MIN_POINTS) {
        prevGray_ = gray;
        box = box_;
        return false;
    }

    const cv::Size         win(21, 21);
    const cv::TermCriteria crit(
        cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01);

    std::vector<cv::Point2f> nxt, back;
    std::vector<uchar>       st, stB;
    std::vector<float>       err;
    cv::calcOpticalFlowPyrLK(prevGray_, gray, pts_, nxt, st, err, win, 3, crit);
    cv::calcOpticalFlowPyrLK(gray, prevGray_, nxt, back, stB, err, win, 3, crit);

    std::vector<cv::Point2f> goodOld, goodNew;
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
    for (size_t i = 0; i < goodNew.size(); ++i) {
        dxs.push_back(goodNew[i].x - goodOld[i].x);
        dys.push_back(goodNew[i].y - goodOld[i].y);
    }
    const float dx = median_of(dxs);
    const float dy = median_of(dys);

    // Scale estimation from median point-spread ratio (±18% cap per frame).
    cv::Point2f cOld(0, 0), cNew(0, 0);
    for (const auto& p : goodOld) cOld += p;
    for (const auto& p : goodNew) cNew += p;
    cOld *= 1.f / goodOld.size();
    cNew *= 1.f / goodNew.size();

    std::vector<float> sOld, sNew;
    for (const auto& p : goodOld) sOld.push_back((float)cv::norm(p - cOld));
    for (const auto& p : goodNew) sNew.push_back((float)cv::norm(p - cNew));
    const float spreadOld = median_of(sOld);
    const float spreadNew = median_of(sNew);

    if (spreadOld > 2.f) {
        const float scale = std::clamp(spreadNew / (spreadOld + 1e-6f), 0.82f, 1.18f);
        const float cx = box_.x + box_.width  / 2.f + dx;
        const float cy = box_.y + box_.height / 2.f + dy;
        const int wNew = std::max(20, (int)std::lround(box_.width  * scale));
        const int hNew = std::max(20, (int)std::lround(box_.height * scale));
        box_ = cv::Rect((int)std::lround(cx - wNew / 2.f),
                        (int)std::lround(cy - hNew / 2.f), wNew, hNew);
    } else {
        box_.x += (int)std::lround(dx);
        box_.y += (int)std::lround(dy);
    }

    prevGray_ = gray;
    pts_      = std::move(goodNew);
    box       = box_;
    return true;
}

// ------------------------------------------------------------ LockOnTracker

// ---- template management ---------------------------------------------------

void LockOnTracker::saveTemplate(const cv::Mat& frame, const cv::Rect& box) {
    const cv::Rect roi = clip_rect(box, frame.size());
    if (roi.empty()) return;
    cv::Mat crop, gray;
    cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
    // Cap template at 96×96 — keeps matchTemplate fast while preserving detail.
    const int tw = std::min(roi.width,  96);
    const int th = std::min(roi.height, 96);
    cv::resize(gray, tmpl_, {tw, th}, 0, 0, cv::INTER_LINEAR);
    tmplAge_ = 0;
}

void LockOnTracker::maybeUpdateTemplate(const cv::Mat& frame,
                                        const cv::Rect& box) {
    ++tmplAge_;
    // Only refresh when we've been locked long enough and confidence is solid.
    if (tmplAge_ < TMPL_UPDATE_FRAMES || confidence_ < 0.60f) return;
    saveTemplate(frame, box);
}

// ---- NCC-based confidence check --------------------------------------------

float LockOnTracker::computeNCC(const cv::Mat& frame,
                                const cv::Rect& box) const {
    if (tmpl_.empty()) return 0.f;
    const cv::Rect roi = clip_rect(box, frame.size());
    if (roi.empty()) return 0.f;

    cv::Mat gray, resized, result;
    cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
    cv::resize(gray, resized, tmpl_.size(), 0, 0, cv::INTER_LINEAR);

    // matchTemplate needs image > template; compare same-size via single pixel
    // result — pad one pixel so the function is happy.
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, 0, 1, 0, 1,
                       cv::BORDER_REPLICATE);
    cv::matchTemplate(padded, tmpl_, result, cv::TM_CCOEFF_NORMED);
    return std::clamp(result.at<float>(0, 0), 0.f, 1.f);
}

// ---- template re-detection -------------------------------------------------

float LockOnTracker::searchRadius() const {
    const auto v     = kalman_.velocity();
    const float speed = std::sqrt(v.x * v.x + v.y * v.y);
    // Hard cap: unbounded growth turns this into a full-frame search (at 8 px/
    // frame it covers 640x480 within ~1 s of the loss) run every frame forever.
    return std::clamp(60.f + speed * lossFrames_ * 1.5f, 80.f, REACQUIRE_MAX_RADIUS);
}

cv::Rect LockOnTracker::templateSearch(const cv::Mat& frame,
                                       float& outScore) const {
    outScore = 0.f;
    if (tmpl_.empty() || !kalman_.initialized()) return {};

    const float   rad    = searchRadius();
    const auto    pred   = kalman_.project((float)std::min(lossFrames_, 8));
    const cv::Rect sroi  = clip_rect(
        cv::Rect((int)(pred.x - rad), (int)(pred.y - rad),
                 (int)(rad * 2),      (int)(rad * 2)),
        frame.size());

    // Search region must be larger than the template.
    if (sroi.width <= tmpl_.cols || sroi.height <= tmpl_.rows) return {};

    cv::Mat gray, result;
    cv::cvtColor(frame(sroi), gray, cv::COLOR_BGR2GRAY);
    cv::matchTemplate(gray, tmpl_, result, cv::TM_CCOEFF_NORMED);

    double   maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(result, nullptr, &maxVal, nullptr, &maxLoc);
    outScore = (float)maxVal;

    if (outScore < REACQUIRE_THRESH) return {};

    // Translate template-match location back to frame coordinates.
    return cv::Rect(sroi.x + maxLoc.x, sroi.y + maxLoc.y,
                    tmpl_.cols, tmpl_.rows);
}

// ---- backend management ----------------------------------------------------

bool LockOnTracker::reinitBackend(const cv::Mat& frame,
                                  const cv::Rect& box) {
    lkTracker_.reset();
    cvTracker_.release();
    legacyTracker_.release();
    try {
        if (backend_ == Backend::FLOW) {
            lkTracker_ = std::make_unique<LKFlowTracker>();
            return lkTracker_->init(frame, box);
        }
        if (backend_ == Backend::MOSSE) {
            // MOSSE uses the legacy Tracker API (Rect2d init/update).
            legacyTracker_ = cv::legacy::TrackerMOSSE::create();
            return legacyTracker_->init(frame, cv::Rect2d(box));
        }
        cvTracker_ = (backend_ == Backend::KCF)
                         ? cv::Ptr<cv::Tracker>(cv::TrackerKCF::create())
                         : cv::Ptr<cv::Tracker>(cv::TrackerCSRT::create());
        cvTracker_->init(frame, box);
        return true;
    } catch (const cv::Exception&) { return false; }
}

// ---- public API ------------------------------------------------------------

bool LockOnTracker::init(const cv::Mat& frame, cv::Point center,
                         Backend backend, int boxSize) {
    const int bs = boxSize;
    const int x  = std::clamp(center.x - bs / 2,
                               0, std::max(0, frame.cols - bs));
    const int y  = std::clamp(center.y - bs / 2,
                               0, std::max(0, frame.rows - bs));
    const cv::Rect box(x, y, bs, bs);

    backend_ = backend;
    boxSize_ = boxSize;

    if (!reinitBackend(frame, box)) return false;

    kalman_.init(rect_center(box));
    saveTemplate(frame, box);

    bbox_        = box;
    hasTarget_   = true;
    locked_      = true;
    confidence_  = 1.f;
    age_         = 0;
    lossFrames_  = 0;
    totalLosses_ = 0;
    return true;
}

void LockOnTracker::update(const cv::Mat& frame) {
    if (!hasTarget_) return;

    // Step 1: Kalman predict — get expected position before any measurement.
    kalman_.predict();

    // Step 2: Run primary tracker.
    bool     ok  = false;
    cv::Rect box = bbox_;
    try {
        if (lkTracker_) {
            ok = lkTracker_->update(frame, box);
        } else if (legacyTracker_) {
            cv::Rect2d r2d(box);
            ok  = legacyTracker_->update(frame, r2d);
            box = cv::Rect((int)r2d.x, (int)r2d.y,
                           (int)r2d.width, (int)r2d.height);
        } else {
            ok = cvTracker_->update(frame, box);
        }
    } catch (const cv::Exception&) { ok = false; }

    if (ok) {
        // Step 3a: Tracker succeeded — fuse center with Kalman.
        kalman_.correct(rect_center(box));
        // A real target cannot cross more than ~0.9x its own size per frame at
        // 30 fps; anything faster is a noisy/false measurement pumping the
        // velocity, which the constant-velocity model then compounds until the
        // box flies off screen. Bound it. (Same defect and fix as the Kotlin
        // tracker's CenterFilter::clampSpeed, where it was measured to cut
        // worst-case error from 220 px to 24 px on a noisy feed.)
        kalman_.clampVelocity(std::max(bbox_.width, bbox_.height) * 0.9f);
        bbox_ = box;
        ++age_;
        if (lossFrames_ > 0) ++totalLosses_;
        lossFrames_ = 0;
        locked_     = true;

        confidence_ = computeNCC(frame, bbox_);
        maybeUpdateTemplate(frame, bbox_);

    } else {
        // Step 3b: Tracker failed — try template re-detection.
        ++lossFrames_;
        if (lossFrames_ >= LOSS_TIMEOUT) locked_ = false;
        // Coast decelerates instead of sailing off on stale velocity.
        kalman_.decayVelocity(COAST_DECAY);

        // Cost guard. searchRadius() grows with lossFrames_ without bound, so
        // after a permanent loss this degenerates into a FULL-FRAME
        // matchTemplate every single frame, forever — and the tracker sits in
        // the scheduler's alwaysOn slot, which runs unconditionally and ignores
        // the per-tick budget, so nothing upstream can throttle it. Once we've
        // given up on a quick re-acquire, retry on a cold cadence instead.
        const bool coldRetry = lossFrames_ >= LOSS_TIMEOUT &&
                               (lossFrames_ % REACQUIRE_COLD_INTERVAL) != 0;
        float    score = 0.f;
        cv::Rect found = coldRetry ? cv::Rect() : templateSearch(frame, score);

        if (!found.empty()) {
            // Re-acquired — reinit primary tracker at found location.
            reinitBackend(frame, found);
            kalman_.correct(rect_center(found));
            bbox_ = found;
            ++totalLosses_;
            lossFrames_ = 0;
            locked_     = true;
        } else {
            // Coast: move bbox to Kalman-predicted position.
            const auto pos = kalman_.position();
            bbox_.x = (int)(pos.x - bbox_.width  / 2.f);
            bbox_.y = (int)(pos.y - bbox_.height / 2.f);
        }

        confidence_ = score;
    }
}

void LockOnTracker::reset() {
    cvTracker_.release();
    legacyTracker_.release();
    lkTracker_.reset();
    kalman_      = KalmanCenter();
    tmpl_        = cv::Mat();
    tmplAge_     = 0;
    bbox_        = cv::Rect();
    hasTarget_   = false;
    locked_      = false;
    confidence_  = 0.f;
    age_         = 0;
    lossFrames_  = 0;
    totalLosses_ = 0;
}

cv::Point2f LockOnTracker::projected(float steps) const {
    return kalman_.initialized() ? kalman_.project(steps)
                                 : rect_center(bbox_);
}
