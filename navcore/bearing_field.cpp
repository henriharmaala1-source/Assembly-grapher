#include "bearing_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {

static const float PI_F_ = 3.14159265358979f;

void BearingField::init(const BearingFieldParams& p) {
    p_ = p;
    p_.nAz = std::max(8, p_.nAz);
    p_.nEl = std::max(1, p_.nEl);
    r_.assign(size_t(p_.nAz) * p_.nEl, kNone);
    age_.assign(r_.size(), -1);
    cur_.assign(r_.size(), kNone);
    cnt_.assign(r_.size(), 0);
    look_.assign(r_.size(), 0);
    sup_.assign(r_.size(), -1.f);
    conf_.assign(r_.size(), 0);
    frame_ = 0;
}

int BearingField::azIdx(float azDeg) const {
    float a = std::fmod(azDeg, 360.f);
    if (a < 0.f) a += 360.f;
    int i = int(a / 360.f * p_.nAz);
    return i >= p_.nAz ? p_.nAz - 1 : i;
}

int BearingField::elIdx(float elDeg) const {
    const float span = p_.elMaxDeg - p_.elMinDeg;
    if (span <= 0.f) return 0;
    int i = int((elDeg - p_.elMinDeg) / span * p_.nEl);
    return i < 0 ? -1 : (i >= p_.nEl ? -1 : i);
}

void BearingField::rebuildTable(const DepthCamera& cam, float pitchDeg,
                                float rollDeg, int rows, int cols) {
    tblRows_ = rows; tblCols_ = cols; tblPitch_ = pitchDeg; tblRoll_ = rollDeg;
    sigCoef_ = p_.subpixelPx / std::max(1e-6f, cam.fpx() * 0.05f);
    bin_.assign(size_t(rows) * cols, -1);
    const float cr = std::cos(rollDeg * PI_F_ / 180.f);
    const float sr = std::sin(rollDeg * PI_F_ / 180.f);
    const float cpi = std::cos(pitchDeg * PI_F_ / 180.f);
    const float spi = std::sin(pitchDeg * PI_F_ / 180.f);
    const float f = cam.fpx(), fy = cam.fy(), ppx = cam.ppx(), ppy = cam.ppy();
    const float elSpan = p_.elMaxDeg - p_.elMinDeg;
    for (int v = 0; v < rows; ++v) {
        const float ry0 = (float(v) - ppy) / fy;
        for (int u = 0; u < cols; ++u) {
            const float rx = (float(u) - ppx) / f;
            const float x1 = rx * cr - ry0 * sr;
            const float y1 = rx * sr + ry0 * cr;
            const float y2 = y1 * cpi - spi, z2 = y1 * spi + cpi;
            // Body frame, yaw NOT applied: that is what makes the shift exact.
            const float dx = x1, dy = z2, dz = -y2;
            const float L = std::sqrt(dx*dx + dy*dy + dz*dz);
            const float el = std::asin(std::max(-1.f, std::min(1.f, dz / L)))
                           * 180.f / PI_F_;
            if (el < p_.elMinDeg || el >= p_.elMaxDeg) continue;
            const int ie = int((el - p_.elMinDeg) / elSpan * p_.nEl);
            bin_[size_t(v) * cols + u] = int32_t(ie) * p_.nAz + int32_t(azIdx(
                std::atan2(dx, dy) * 180.f / PI_F_));
        }
    }
}

void BearingField::update(const cv::Mat& depth, const DepthCamera& cam,
                          const CamPose& pose, int stride) {
    if (depth.empty()) return;
    ++frame_;
    const int s = std::max(1, stride);
    if (tblRows_ != depth.rows || tblCols_ != depth.cols ||
        tblPitch_ != pose.pitchDeg || tblRoll_ != pose.rollDeg)
        rebuildTable(cam, pose.pitchDeg, pose.rollDeg, depth.rows, depth.cols);

    // PER-FRAME FIRST, THEN MERGE -- and this is not a detail.
    //
    // The obvious version keeps a running minimum per bin. That is wrong on a
    // noisy sensor: the minimum of N samples is biased low and the bias grows
    // with N, so every bin creeps toward the nearest outlier it has ever seen
    // and the render turns to salt and pepper. Observed exactly that.
    //
    // A bearing bin is also FULLY RE-OBSERVED whenever it is in frame, unlike a
    // voxel which may be occluded. So: within a frame take the nearest return,
    // across frames REPLACE. Persistence then means only "what I saw when I
    // last looked that way", which is all a bearing field can honestly claim.
    if (cur_.size() != r_.size()) cur_.assign(r_.size(), kNone);
    else std::fill(cur_.begin(), cur_.end(), kNone);
    if (cnt_.size() != r_.size()) cnt_.assign(r_.size(), 0);
    else std::fill(cnt_.begin(), cnt_.end(), 0);
    if (look_.size() != r_.size()) look_.assign(r_.size(), 0);
    else std::fill(look_.begin(), look_.end(), 0);

    const int yawShift = ((int(std::lround(pose.yawDeg / 360.f * p_.nAz))
                           % p_.nAz) + p_.nAz) % p_.nAz;
    for (int v = 0; v < depth.rows; v += s) {
        const float* row = depth.ptr<float>(v);
        const int32_t* brow = &bin_[size_t(v) * depth.cols];
        for (int u = 0; u < depth.cols; u += s) {
            const int32_t b = brow[u];
            if (b < 0) continue;
            const int ie = b / p_.nAz;
            int ia = b % p_.nAz + yawShift;
            if (ia >= p_.nAz) ia -= p_.nAz;
            const size_t k = size_t(ie) * p_.nAz + ia;
            // EVERY sampled pixel counts as a LOOK, whether it returned or not.
            // That denominator is the whole point: without it there is no way to
            // tell a surface from a few noise matches in an empty sky.
            ++look_[k];
            const float r = row[u];
            if (!(r > p_.minRangeM) || r > p_.maxRangeM) continue;
            ++cnt_[k];
            if (r < cur_[k]) cur_[k] = r;
        }
    }
    // A single stereo outlier is the nearest sample in its bin by definition,
    // so a bare minimum is maximally sensitive to exactly the thing this sensor
    // produces most. Require a handful of agreeing samples before a bin may
    // claim a surface at all.
    rej_ = Rejects();
    for (size_t k = 0; k < cur_.size(); ++k) {
        if (look_[k] > 0) ++rej_.looked;
        if (cnt_[k] < p_.minSamples || cur_[k] >= kNone) {
            if (look_[k] > 0) ++rej_.tooFewSamples;
            continue;
        }
        if (look_[k] > 0 &&
            float(cnt_[k]) / float(look_[k]) < p_.minFillFrac) {
            ++rej_.tooSparse; continue;
        }
        ++rej_.accepted;
        sup_[k] = look_[k] > 0 ? float(cnt_[k]) / float(look_[k]) : -1.f;
        // EXISTENCE confirms; range does not have to hold still.
        const bool wasLive = age_[k] >= 0 && frame_ - age_[k] <= 1;
        conf_[k] = wasLive ? conf_[k] + 1 : 1;
        const float tol = std::max(p_.agreeM,
                                   p_.agreeSigK * sigCoef_ * cur_[k] * cur_[k]);
        // Smooth only while the range is steady. Across a silhouette flip, take
        // the new value outright -- averaging a trunk with the gap behind it
        // would place a surface where there is none.
        r_[k] = (wasLive && std::fabs(cur_[k] - r_[k]) <= tol)
              ? 0.7f * r_[k] + 0.3f * cur_[k]
              : cur_[k];
        age_[k] = frame_;
    }
    // --- neighbour consensus ------------------------------------------------
    // Second pass, after every bin has had its own say, so a filled bin can
    // never seed another. One round only: a segment recovers its interior, and
    // a genuine gap between two surfaces stays a gap.
    if (p_.consensusMin > 0) {
        for (int ie = 1; ie + 1 < p_.nEl; ++ie)
            for (int ia = 0; ia < p_.nAz; ++ia) {
                const size_t k = size_t(ie) * p_.nAz + ia;
                if (age_[k] == frame_) continue;          // spoke for itself
                if (look_[k] < p_.minSamples) continue;   // nobody looked there
                float sum = 0.f, supN = 0.f, first = -1.f; int agree = 0;
                for (int de = -1; de <= 1; ++de)
                    for (int da = -1; da <= 1; ++da) {
                        if (!de && !da) continue;
                        int na = ia + da;
                        if (na < 0) na += p_.nAz; else if (na >= p_.nAz) na -= p_.nAz;
                        const size_t nk = size_t(ie + de) * p_.nAz + na;
                        if (age_[nk] != frame_ || conf_[nk] < p_.confirmFrames) continue;
                        if (first < 0.f) first = r_[nk];
                        else if (std::fabs(r_[nk] - first) >
                                 p_.consensusTolFrac * first) continue;
                        sum += r_[nk]; supN += sup_[nk]; ++agree;
                    }
                if (agree < p_.consensusMin) continue;
                ++rej_.filledByConsensus;
                r_[k] = sum / float(agree);
                sup_[k] = 0.5f * supN / float(agree);   // inferred, not measured
                conf_[k] = p_.confirmFrames;            // its neighbours vouched
                age_[k] = frame_;
            }
    }

}

float BearingField::rangeAt(float azDeg, float elDeg) const {
    const int ie = elIdx(elDeg);
    if (ie < 0) return -1.f;
    const size_t k = size_t(ie) * p_.nAz + azIdx(azDeg);
    if (age_[k] < 0 || frame_ - age_[k] > p_.forgetFrames) return -1.f;
    if (conf_[k] < p_.confirmFrames) return -1.f;
    return r_[k];

}

float BearingField::supportAt(float azDeg, float elDeg) const {
    const int ie = elIdx(elDeg);
    if (ie < 0) return -1.f;
    const size_t k = size_t(ie) * p_.nAz + azIdx(azDeg);
    if (age_[k] < 0 || frame_ - age_[k] > p_.forgetFrames) return -1.f;
    if (conf_[k] < p_.confirmFrames) return -1.f;
    return sup_[k];
}

void BearingField::supportHistogram(int& solid, int& partial, int& none) const {
    solid = partial = none = 0;
    for (size_t i = 0; i < r_.size(); ++i) {
        const bool live = age_[i] >= 0 && frame_ - age_[i] <= p_.forgetFrames
                       && conf_[i] >= p_.confirmFrames;
        if (!live) { ++none; continue; }
        if (sup_[i] >= p_.solidFillFrac) ++solid; else ++partial;
    }
}

std::vector<float> BearingField::obstacleDistance(float yawDeg, int bins) const {
    std::vector<float> out(std::max(1, bins), -1.f);
    // The band the aircraft could actually hit: a few degrees either side of
    // its own altitude. Taking the whole elevation range would report the
    // ground as an obstacle on every bearing.
    const float elBand = 8.f;
    for (int b = 0; b < int(out.size()); ++b) {
        const float az = yawDeg + 360.f * float(b) / float(out.size());
        float best = -1.f;
        for (float el = -elBand; el <= elBand; el += 2.f) {
            const float r = rangeAt(az, el);
            if (r > 0.f && (best < 0.f || r < best)) best = r;
        }
        out[b] = best;
    }
    return out;
}

void BearingField::occupancy(int& live, int& total) const {
    live = 0;
    total = int(r_.size());
    for (size_t i = 0; i < r_.size(); ++i)
        if (age_[i] >= 0 && frame_ - age_[i] <= p_.forgetFrames
            && conf_[i] >= p_.confirmFrames) ++live;
}

// --- first-person render ----------------------------------------------------
// Mirrors VoxelMap::fpvImageWH pixel for pixel in geometry and colour, so a
// side-by-side pane differs ONLY in where the range came from.
cv::Mat BearingField::render(const BearingField& bf,
                             float yawDeg, float pitchDeg,
                             int outW, int outH, float hfovDeg,
                             float minRange, float maxRange, float eyeAltM,
                             cv::Mat* hitMask) {
    cv::Mat img(outH, outW, CV_8UC3);
    if (hitMask) *hitMask = cv::Mat(outH, outW, CV_8U, cv::Scalar(0));
    const float HEIGHT_KEY_M = 3.5f;
    const cv::Vec3f LOW(90, 150, 80), AT(55, 60, 235), HIGH(210, 160, 90);
    const cv::Vec3f FOG(238, 240, 244);

    const float f = (outW * 0.5f) / std::tan(hfovDeg * 0.5f * PI_F_ / 180.f);
    const float cx_ = (outW - 1) * 0.5f, cy_ = (outH - 1) * 0.5f;
    const float cyw = std::cos(yawDeg * PI_F_ / 180.f), syw = std::sin(yawDeg * PI_F_ / 180.f);
    const float cp = std::cos(pitchDeg * PI_F_ / 180.f), sp = std::sin(pitchDeg * PI_F_ / 180.f);

    // Range per pixel first, then shade -- the slope term needs its neighbours.
    cv::Mat rng(outH, outW, CV_32F, cv::Scalar(-1.f));
    cv::Mat alt(outH, outW, CV_32F, cv::Scalar(0.f));
    cv::Mat sup(outH, outW, CV_32F, cv::Scalar(1.f));
    for (int v = 0; v < outH; ++v)
        for (int u = 0; u < outW; ++u) {
            const float rx = (float(u) - cx_) / f, ry = (float(v) - cy_) / f;
            const float y2 = ry * cp - sp, z2 = ry * sp + cp;
            const float fE = rx, fN = z2, fU = -y2;
            float dx = fE * cyw + fN * syw;
            float dy = -fE * syw + fN * cyw;
            float dz = fU;
            const float L = std::sqrt(dx*dx + dy*dy + dz*dz);
            dx /= L; dy /= L; dz /= L;
            const float az = std::atan2(dx, dy) * 180.f / PI_F_;
            const float el = std::asin(std::max(-1.f, std::min(1.f, dz))) * 180.f / PI_F_;
            const float r = bf.rangeAt(az, el);
            if (!(r > 0.f) || r > maxRange || r < minRange) continue;
            rng.at<float>(v, u) = r;
            sup.at<float>(v, u) = bf.supportAt(az, el);
            alt.at<float>(v, u) = dz * r;      // height of the surface above the eye
        }

    for (int v = 0; v < outH; ++v) {
        cv::Vec3b* row = img.ptr<cv::Vec3b>(v);
        uchar* mrow = hitMask ? hitMask->ptr<uchar>(v) : nullptr;
        for (int u = 0; u < outW; ++u) {
            const float r = rng.at<float>(v, u);
            if (!(r > 0.f)) {
                row[u] = cv::Vec3b(uchar(FOG[0]), uchar(FOG[1]), uchar(FOG[2]));
                if (mrow) mrow[u] = 0;
                continue;
            }
            float k = alt.at<float>(v, u) / HEIGHT_KEY_M;
            k = std::max(-1.f, std::min(1.f, k));
            cv::Vec3f col = (k < 0.f) ? LOW + (AT - LOW) * (1.f + k)
                                      : AT + (HIGH - AT) * k;
            // Slope shading in place of cube faces. A surface facing the camera
            // stays bright; one raking away darkens. Without it a bearing field
            // renders as a flat wash and reads as less informative than it is,
            // which would make the comparison unfair in the other direction.
            float gx = 0.f, gy = 0.f;
            if (u > 0 && u + 1 < outW) {
                const float a = rng.at<float>(v, u - 1), b = rng.at<float>(v, u + 1);
                if (a > 0.f && b > 0.f) gx = (b - a) * 0.5f;
            }
            if (v > 0 && v + 1 < outH) {
                const float a = rng.at<float>(v - 1, u), b = rng.at<float>(v + 1, u);
                if (a > 0.f && b > 0.f) gy = (b - a) * 0.5f;
            }
            const float g = std::sqrt(gx*gx + gy*gy) * f / std::max(0.5f, r);
            const float shade = 1.f / std::sqrt(1.f + g * g);
            col *= 0.42f + 0.58f * shade;
            const float d = std::min(1.f, r / maxRange);
            col = col * (1.f - 0.55f * d) + FOG * (0.55f * d);
            // SUPPORT AS TRANSPARENCY. A bin where only a third of the pixels
            // returned is a silhouette edge or foliage, and drawing it as
            // solidly as a wall asserts more than was measured. Fading it makes
            // the outline of found depth visible instead of implied.
            const float sv = sup.at<float>(v, u);
            if (sv >= 0.f) {
                const float a = 0.35f + 0.65f * std::min(1.f, sv / 0.7f);
                col = col * a + FOG * (1.f - a);
            }
            row[u] = cv::Vec3b(uchar(std::min(255.f, col[0])),
                               uchar(std::min(255.f, col[1])),
                               uchar(std::min(255.f, col[2])));
            if (mrow) mrow[u] = 255;
        }
    }
    (void)eyeAltM;
    return img;
}

}  // namespace sim
