#include "vio.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace sim {

namespace {

constexpr float kDeg = 3.14159265f / 180.f;

// Camera->world rotation for a CamPose, built from camToWorld itself so there
// is exactly ONE definition of the attitude convention in the tree.
cv::Matx33d rotWc(const CamPose& p) {
    cv::Matx33d R;
    const float basis[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int c = 0; c < 3; ++c) {
        float wx, wy, wz;
        DepthCamera::camToWorld(p, basis[c][0], basis[c][1], basis[c][2], wx, wy, wz);
        R(0, c) = wx; R(1, c) = wy; R(2, c) = wz;
    }
    return R;
}

// Yaw (clockwise from North) of a camera->world rotation: where the optical
// axis points, projected onto the horizontal.
float yawOf(const cv::Matx33d& Rwc) {
    return float(std::atan2(Rwc(0, 2), Rwc(1, 2)) / kDeg);
}

// Depth in a (2R+1)^2 window around (u,v) stays within a relative band of the
// centre: the corner sits on one surface, not on an edge. The band is wide
// enough for stereo range noise (which grows with r^2 and is ~10 % at the far
// end of the anchor band); a silhouette is a jump of metres. Stereo leaves
// holes on edges rather than wrong values, so a few invalid pixels are
// allowed but a window that is mostly hole is not.
bool smoothDepth(const cv::Mat& d, int u, int v, float r) {
    constexpr int R = 2;
    const float tol = 0.05f + 0.12f * r;
    if (u < R || v < R || u >= d.cols - R || v >= d.rows - R) return false;
    int holes = 0;
    for (int dv = -R; dv <= R; ++dv) {
        const float* row = d.ptr<float>(v + dv);
        for (int du = -R; du <= R; ++du) {
            const float q = row[u + du];
            if (!(q > 0.f)) { ++holes; continue; }
            if (std::fabs(q - r) > tol) return false;
        }
    }
    return holes <= 6;
}

// Subtract the local mean: what is left is texture. The low frequencies of an
// image are mostly LIGHTING -- projector falloff, vignetting, auto-exposure
// steps -- and lighting that moves with the camera pulls a pyramidal tracker
// toward zero motion at its coarse levels, where the texture has averaged out
// and only the lighting is left. Measured in the sim: at a 0.2 m step, LK
// seeded at the true position kept 17 of 139 corners within 2 px on the raw
// image and 135 on this one.
cv::Mat highPass(const cv::Mat& ir8, int box, float gain) {
    cv::Mat f, m;
    ir8.convertTo(f, CV_16S);
    cv::blur(ir8, m, cv::Size(box, box), cv::Point(-1, -1), cv::BORDER_REFLECT);
    m.convertTo(m, CV_16S);
    cv::Mat d = f - m, out;
    d.convertTo(out, CV_8U, gain, 128.0);
    return out;
}

float wrap180(float d) {
    while (d > 180.f) d -= 360.f;
    while (d <= -180.f) d += 360.f;
    return d;
}

}  // namespace

void DepthVio::init(const DepthCamera& cam, const VioParams& p) {
    cam_ = &cam;
    p_ = p;
    started_ = false;
}

void DepthVio::reset(const CamPose& start) {
    pose_ = start;
    kfPose_ = start;
    imuRef_ = false;
    vel_[0] = vel_[1] = vel_[2] = 0.f;
    lastValid_ = false;
    prevImg_.release();
    prevPts_.clear();
    worldPts_.clear();
    started_ = true;
    ++resets_;
}

void DepthVio::makeKeyframe(const cv::Mat& ir8, const cv::Mat& depthM) {
    // Keep the survivors: they already carry world points, and re-deriving
    // them from this frame would re-inject this frame's pose error into them.
    // Add fresh corners only where there are none, and only with good depth.
    cv::Mat mask(ir8.size(), CV_8U, cv::Scalar(0));
    for (int v = 0; v < depthM.rows; ++v) {
        const float* d = depthM.ptr<float>(v);
        uchar* m = mask.ptr<uchar>(v);
        for (int u = 0; u < depthM.cols; ++u)
            m[u] = (d[u] >= p_.minDepthM && d[u] <= p_.maxDepthM) ? 255 : 0;
    }
    for (const cv::Point2f& q : prevPts_)
        cv::circle(mask, q, int(p_.minDistPx), cv::Scalar(0), -1);
    const int want = p_.maxFeatures - int(prevPts_.size());
    if (want > 0) {
        std::vector<cv::Point2f> fresh;
        cv::goodFeaturesToTrack(ir8, fresh, want, p_.quality, p_.minDistPx, mask);
        const cv::Matx33d Rwc = rotWc(pose_);
        for (const cv::Point2f& q : fresh) {
            const int u = int(q.x + 0.5f), v = int(q.y + 0.5f);
            if (u < 0 || v < 0 || u >= depthM.cols || v >= depthM.rows) continue;
            const float r = depthM.at<float>(v, u);
            if (!(r >= p_.minDepthM && r <= p_.maxDepthM)) continue;
            // SILHOUETTES. The strongest corners in an IR image are object
            // edges against a far background, and there the depth pixel under
            // the corner belongs to either side -- a world point metres off.
            // Real stereo adds flying pixels on the same edges. Keep a corner
            // only where the depth around it is one surface.
            if (!smoothDepth(depthM, u, v, r)) continue;
            // RANGE along the ray -> camera-frame point on the unit ray.
            const double x = (q.x - cam_->ppx()) / cam_->fpx();
            const double y = (q.y - cam_->ppy()) / cam_->fy();
            const double n = std::sqrt(x * x + y * y + 1.0);
            const cv::Vec3d pc(x / n * r, y / n * r, 1.0 / n * r);
            const cv::Vec3d pw = Rwc * pc + cv::Vec3d(pose_.e, pose_.n, pose_.u);
            prevPts_.push_back(q);
            worldPts_.push_back(cv::Point3f(float(pw[0]), float(pw[1]), float(pw[2])));
        }
    }
    kfPose_ = pose_;
}

VioResult DepthVio::step(const cv::Mat& irRaw, const cv::Mat& depthM,
                         const CamPose& attitude) {
    const auto t0 = std::chrono::steady_clock::now();
    VioResult out;
    if (!cam_ || irRaw.empty() || depthM.empty() || irRaw.size() != depthM.size()) return out;
    const cv::Mat ir8 = p_.highPassPx > 1 ? highPass(irRaw, p_.highPassPx, p_.highPassGain) : irRaw;
    if (!started_) {
        CamPose start; start.rollDeg = attitude.rollDeg;
        start.pitchDeg = attitude.pitchDeg; start.yawDeg = attitude.yawDeg;
        reset(start);
    }

    // The IMU's rotation since the last frame, applied to our own yaw: the
    // prior. Roll and pitch are taken from the IMU outright.
    if (!imuRef_) { lastImuYaw_ = attitude.yawDeg; imuRef_ = true; }
    CamPose prior = pose_;
    prior.yawDeg   = pose_.yawDeg + wrap180(attitude.yawDeg - lastImuYaw_);
    prior.rollDeg  = attitude.rollDeg;
    prior.pitchDeg = attitude.pitchDeg;
    prior.e += vel_[0]; prior.n += vel_[1]; prior.u += vel_[2];
    lastImuYaw_ = attitude.yawDeg;

    if (prevImg_.empty() || prevPts_.empty()) {
        // First frame (or just re-anchored after a loss): nothing to track
        // yet. Anchor a keyframe at the prior and report the hold.
        pose_ = prior;
        prevPts_.clear(); worldPts_.clear();
        makeKeyframe(ir8, depthM);
        prevImg_ = ir8.clone();
        out.pose = pose_; out.newKeyframe = true; out.resets = resets_;
        out.valid = false;
        out.ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return out;
    }

    // 1. Track, with the forward-backward check.
    // PREDICTED: each corner's search starts where its world point projects
    // under the prior pose, not where it was last frame. At 3 m/s a near
    // floor corner moves tens of pixels per frame; the prediction takes that
    // out and leaves LK only the prior's error to find.
    const cv::Matx33d K(cam_->fpx(), 0, cam_->ppx(), 0, cam_->fy(), cam_->ppy(), 0, 0, 1);
    std::vector<cv::Point2f> cur(prevPts_), back(prevPts_);
    {
        const cv::Matx33d Rcw = rotWc(prior).t();
        const cv::Vec3d C(prior.e, prior.n, prior.u);
        for (size_t i = 0; i < prevPts_.size(); ++i) {
            const cv::Vec3d pc = Rcw * (cv::Vec3d(worldPts_[i].x, worldPts_[i].y,
                                                   worldPts_[i].z) - C);
            if (pc[2] < 0.1) continue;
            const cv::Point2f q(float(K(0, 0) * pc[0] / pc[2] + K(0, 2)),
                                float(K(1, 1) * pc[1] / pc[2] + K(1, 2)));
            if (q.x >= 0 && q.y >= 0 && q.x < ir8.cols && q.y < ir8.rows) cur[i] = q;
        }
    }
    std::vector<uchar> st1, st2;
    std::vector<float> err;
    const cv::Size win(p_.lkWin, p_.lkWin);
    const cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(prevImg_, ir8, prevPts_, cur, st1, err, win, p_.lkLevels,
                             term, cv::OPTFLOW_USE_INITIAL_FLOW);
    cv::calcOpticalFlowPyrLK(ir8, prevImg_, cur, back, st2, err, win, p_.lkLevels,
                             term, cv::OPTFLOW_USE_INITIAL_FLOW);
    std::vector<cv::Point2f> img;
    std::vector<cv::Point3f> obj;
    for (size_t i = 0; i < cur.size(); ++i) {
        if (!st1[i] || !st2[i]) continue;
        const float fb = std::hypot(back[i].x - prevPts_[i].x, back[i].y - prevPts_[i].y);
        if (fb > p_.fbMaxPx) continue;
        if (cur[i].x < 0 || cur[i].y < 0 || cur[i].x >= ir8.cols || cur[i].y >= ir8.rows) continue;
        img.push_back(cur[i]); obj.push_back(worldPts_[i]);
    }
    out.tracked = int(img.size());

    // 2+3. Pose. Roll and pitch are the IMU's; what is solved is yaw and
    // position, 4 DOF, by Gauss-Newton on reprojection error starting from the
    // prior. Frame-to-frame motion is centimetres and a degree or two, well
    // inside the basin, so no minimal-set search is needed to find it -- and a
    // 6-DOF PnP left free to trade tilt against translation is exactly what
    // produced wild solutions here. Outliers: Huber weights first, then a hard
    // residual gate and a clean re-solve on the survivors.
    CamPose est = prior;
    std::vector<int> inl;
    bool ok = false;
    double rms = 0;
    if (int(img.size()) >= p_.minInliers) {
        std::vector<char> use(img.size(), 1);
        auto solve = [&](int iters, double huber) {
            for (int it = 0; it < iters; ++it) {
                const cv::Matx33d Rcw = rotWc(est).t();
                // d(Rcw)/d(yaw) by a small finite step: one definition of the
                // attitude convention, rotWc, and no hand-derived copy of it.
                CamPose ey = est; ey.yawDeg += 0.01f;
                const cv::Matx33d dRcw = (rotWc(ey).t() - Rcw) * (1.0 / (0.01 * kDeg));
                const cv::Vec3d C(est.e, est.n, est.u);
                cv::Matx44d H = cv::Matx44d::zeros();
                cv::Vec4d g(0, 0, 0, 0);
                for (size_t k = 0; k < img.size(); ++k) {
                    if (!use[k]) continue;
                    const cv::Vec3d pw(obj[k].x, obj[k].y, obj[k].z);
                    const cv::Vec3d q = pw - C;
                    const cv::Vec3d pc = Rcw * q;
                    if (pc[2] < 0.1) continue;
                    const double iz = 1.0 / pc[2];
                    const double ru = K(0, 0) * pc[0] * iz + K(0, 2) - img[k].x;
                    const double rv = K(1, 1) * pc[1] * iz + K(1, 2) - img[k].y;
                    const cv::Matx23d Jp(K(0, 0) * iz, 0, -K(0, 0) * pc[0] * iz * iz,
                                         0, K(1, 1) * iz, -K(1, 1) * pc[1] * iz * iz);
                    const cv::Matx23d JC = Jp * (-Rcw);
                    const cv::Vec2d Jy = Jp * (dRcw * q);
                    const cv::Matx<double, 2, 4> J(Jy[0], JC(0, 0), JC(0, 1), JC(0, 2),
                                                   Jy[1], JC(1, 0), JC(1, 1), JC(1, 2));
                    const double e = std::sqrt(ru * ru + rv * rv);
                    const double wgt = (huber > 0 && e > huber) ? huber / e : 1.0;
                    H += wgt * (J.t() * J);
                    g += wgt * (J.t() * cv::Vec2d(ru, rv));
                }
                // The gyro's yaw, as one more (angular) observation.
                const double sy = double(p_.gyroYawSigmaDeg) * kDeg;
                const double eyaw = double(wrap180(est.yawDeg - prior.yawDeg)) * kDeg;
                H(0, 0) += 1.0 / (sy * sy);
                g[0]    += eyaw / (sy * sy);
                cv::Vec4d dx;
                if (!cv::solve(H, -g, dx, cv::DECOMP_CHOLESKY)) return false;
                est.yawDeg += float(dx[0] / kDeg);
                est.e += float(dx[1]); est.n += float(dx[2]); est.u += float(dx[3]);
                if (std::fabs(dx[0]) < 1e-6 && std::fabs(dx[1]) + std::fabs(dx[2]) +
                    std::fabs(dx[3]) < 1e-5) break;
            }
            return true;
        };
        auto residual = [&](size_t k, const cv::Matx33d& Rcw) {
            const cv::Vec3d pc = Rcw * (cv::Vec3d(obj[k].x, obj[k].y, obj[k].z) -
                                        cv::Vec3d(est.e, est.n, est.u));
            if (pc[2] < 0.1) return 1e9;
            const double ru = K(0, 0) * pc[0] / pc[2] + K(0, 2) - img[k].x;
            const double rv = K(1, 1) * pc[1] / pc[2] + K(1, 2) - img[k].y;
            return std::sqrt(ru * ru + rv * rv);
        };
        ok = solve(10, 1.5);
        if (ok) {
            const cv::Matx33d Rcw = rotWc(est).t();
            inl.clear();
            for (size_t k = 0; k < img.size(); ++k) {
                use[k] = residual(k, Rcw) <= p_.ransacPx;
                if (use[k]) inl.push_back(int(k));
            }
            ok = int(inl.size()) >= p_.minInliers && solve(5, 0.0);
        }
        if (ok) {
            const cv::Matx33d Rcw = rotWc(est).t();
            double se = 0;
            inl.clear();
            for (size_t k = 0; k < img.size(); ++k) {
                const double e = residual(k, Rcw);
                if (e <= p_.ransacPx) { inl.push_back(int(k)); se += e * e; }
            }
            rms = inl.empty() ? 1e9 : std::sqrt(se / double(inl.size()));
            // Sanity: a solve that jumps further than the aircraft can move in
            // a frame, or turns away from the gyro, is a wrong basin. LOST
            // beats confidently wrong -- downstream fuses what we say.
            const float jump = std::hypot(std::hypot(est.e - pose_.e, est.n - pose_.n),
                                          est.u - pose_.u);
            const float yawDev = std::fabs(wrap180(est.yawDeg - prior.yawDeg));
            ok = int(inl.size()) >= p_.minInliers && jump <= p_.maxJumpM &&
                 yawDev <= p_.maxYawDevDeg;
        }
    }

    if (ok) {
        est.yawDeg = wrap180(est.yawDeg);
        // Velocity: measured only across two consecutive good frames; after a
        // loss the previous pose was a coast, and differencing it would feed
        // the guess back in as a measurement.
        const float m[3] = {est.e - pose_.e, est.n - pose_.n, est.u - pose_.u};
        for (int a = 0; a < 3; ++a)
            vel_[a] = lastValid_ ? 0.5f * vel_[a] + 0.5f * m[a] : vel_[a];
        pose_ = est;
        out.inliers = int(inl.size());
        out.rmsPx = float(rms);
        out.valid = true;
        // Keep only the inliers as the live track.
        std::vector<cv::Point2f> ip; std::vector<cv::Point3f> iw;
        for (int k : inl) { ip.push_back(img[size_t(k)]); iw.push_back(obj[size_t(k)]); }
        prevPts_.swap(ip); worldPts_.swap(iw);
    } else {
        // LOST. Take the prior -- the IMU's rotation, the last velocity
        // coasting down -- and say so: the pose is not a measurement. Re-anchor from this frame, which
        // is a discontinuity in everything downstream of the anchor.
        pose_ = prior;
        for (float& v : vel_) v *= p_.coastDecay;
        prevPts_.clear(); worldPts_.clear();
        ++resets_;
    }
    lastValid_ = ok;

    // 4. Keyframe policy.
    const float moved = std::hypot(std::hypot(pose_.e - kfPose_.e, pose_.n - kfPose_.n),
                                   pose_.u - kfPose_.u);
    const float turned = std::fabs(wrap180(pose_.yawDeg - kfPose_.yawDeg));
    if (!ok || int(prevPts_.size()) < p_.kfMinTracked || moved > p_.kfDistM ||
        turned > p_.kfYawDeg) {
        makeKeyframe(ir8, depthM);
        out.newKeyframe = true;
    }
    prevImg_ = ir8.clone();
    out.pose = pose_;
    out.resets = resets_;
    out.ms = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

}  // namespace sim
