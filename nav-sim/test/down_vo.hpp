#pragma once
// ---------------------------------------------------------------------------
// down_vo.hpp -- the pieces of the downward-camera study that are not
// navcore's, shared by the simulator (downvio_sweep.cpp) and the harness that
// runs the same estimators on RECORDED footage (footage_vio.cpp), so the two
// cannot drift apart:
//
//   PnpVo        vision-only 6-DOF odometry (no IMU attitude at all)
//   planeDepth   MONO + KNOWN HEIGHT depth: each pixel's ray cut against a
//                flat ground plane H metres below the camera
//   rotOf        a navcore CamPose as a camera->world rotation matrix
//   tiltErrDeg   the angle between two optical axes
//
// Test-harness code, not the aircraft's.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "depth_camera.hpp"

namespace downvo {

constexpr double kPi = 3.14159265358979323846;

// Camera->world rotation of a CamPose, from navcore's own projection.
inline cv::Matx33d rotOf(const sim::CamPose& p) {
    cv::Matx33d R;
    const float b[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int c = 0; c < 3; ++c) {
        float x, y, z;
        sim::DepthCamera::camToWorld(p, b[c][0], b[c][1], b[c][2], x, y, z);
        R(0, c) = x; R(1, c) = y; R(2, c) = z;
    }
    return R;
}

// Angle between two optical axes (= the body's down axes), degrees: TILT error.
inline double tiltErrDeg(const cv::Matx33d& a, const cv::Matx33d& b) {
    const double d = a(0, 2) * b(0, 2) + a(1, 2) * b(1, 2) + a(2, 2) * b(2, 2);
    return std::acos(std::max(-1.0, std::min(1.0, d))) * 180.0 / kPi;
}

// Range along each pixel's ray to a flat ground plane `H` metres below the
// camera, for camera->world rotation `Rwc` (CV_32F, 0 = the ray never meets
// the ground, or meets it too obliquely to trust: above ~3 deg below the
// horizon).
inline cv::Mat planeDepth(const sim::DepthCamera& cam, int w, int h, const cv::Matx33d& Rwc,
                          double H) {
    cv::Mat dm(h, w, CV_32F);
    for (int v = 0; v < h; ++v) {
        float* row = dm.ptr<float>(v);
        for (int u = 0; u < w; ++u) {
            const double x = (u - cam.ppx()) / cam.fpx(), y = (v - cam.ppy()) / cam.fy();
            const double n = std::sqrt(x * x + y * y + 1.0);
            const double dz = (Rwc(2, 0) * x + Rwc(2, 1) * y + Rwc(2, 2)) / n;
            row[u] = (dz < -0.05) ? float(H / -dz) : 0.f;
        }
    }
    return dm;
}

// ---------------------------------------------------------------------------
// PnpVo -- VISION-ONLY attitude (--attitude vision): the question "can a wide
// lens stand in for the IMU?". The same front end as DepthVio -- Shi-Tomasi
// corners anchored at keyframes by depth, tracked by pyramidal LK seeded by a
// constant-velocity prediction, forward-backward checked, same keyframe
// policy -- but the pose is solved in FULL 6 DOF by PnP (RANSAC, then
// refined), so roll and pitch come from the image and nothing from an IMU.
// Depth is asked for at keyframes through `depthAt(Rwc, C)`, so the mono +
// rangefinder model builds its ground plane from THIS estimate's tilt, as it
// would have to with no IMU.
class PnpVo {
public:
    using DepthFn = std::function<cv::Mat(const cv::Matx33d& Rwc, const cv::Vec3d& C)>;
    void init(const sim::DepthCamera& cam, float maxDepthM, float maxJumpM) {
        cam_ = &cam; maxDepth_ = maxDepthM; maxJump_ = maxJumpM;
        K_ = cv::Matx33d(cam.fpx(), 0, cam.ppx(), 0, cam.fy(), cam.ppy(), 0, 0, 1);
    }
    void reset(const cv::Matx33d& Rwc, const cv::Vec3d& C) {
        R_ = Rprev_ = Rwc; C_ = Cprev_ = C; vel_ = cv::Vec3d(0, 0, 0); started_ = true;
        pts_.clear(); world_.clear(); prev_.release(); lostRun_ = 0;
    }
    // One frame. Returns true if the pose is a measurement.
    bool step(const cv::Mat& ir8raw, const DepthFn& depthAt) {
        cv::Mat ir8;
        {   // the same high-pass DepthVio applies: lighting is not texture
            cv::Mat f, m; ir8raw.convertTo(f, CV_16S);
            cv::blur(ir8raw, m, cv::Size(25, 25), cv::Point(-1, -1), cv::BORDER_REFLECT);
            m.convertTo(m, CV_16S); cv::Mat d = f - m; d.convertTo(ir8, CV_8U, 2.0, 128.0);
        }
        // Constant-velocity prediction, rotation included.
        const cv::Matx33d dR = R_ * Rprev_.t();
        const cv::Matx33d Rpred = dR * R_;
        const cv::Vec3d Cpred = C_ + vel_;
        Rprev_ = R_; Cprev_ = C_;
        bool ok = false;
        tracked_ = 0;
        if (!prev_.empty() && pts_.size() >= 20) {
            std::vector<cv::Point2f> cur(pts_.size()), back;
            const cv::Matx33d Rcw = Rpred.t();
            for (size_t i = 0; i < pts_.size(); ++i) {
                const cv::Vec3d pc = Rcw * (cv::Vec3d(world_[i].x, world_[i].y, world_[i].z) - Cpred);
                cur[i] = pc[2] > 0.1 ? cv::Point2f(float(K_(0, 0) * pc[0] / pc[2] + K_(0, 2)),
                                                   float(K_(1, 1) * pc[1] / pc[2] + K_(1, 2)))
                                     : pts_[i];
            }
            back = pts_;
            std::vector<uchar> s1, s2; std::vector<float> e;
            const cv::TermCriteria term(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
            cv::calcOpticalFlowPyrLK(prev_, ir8, pts_, cur, s1, e, cv::Size(21, 21), 3, term, cv::OPTFLOW_USE_INITIAL_FLOW);
            cv::calcOpticalFlowPyrLK(ir8, prev_, cur, back, s2, e, cv::Size(21, 21), 3, term, cv::OPTFLOW_USE_INITIAL_FLOW);
            std::vector<cv::Point2f> img; std::vector<cv::Point3f> obj; std::vector<int> idx;
            for (size_t i = 0; i < pts_.size(); ++i) {
                if (!s1[i] || !s2[i]) continue;
                if (std::hypot(back[i].x - pts_[i].x, back[i].y - pts_[i].y) > 1.f) continue;
                if (cur[i].x < 0 || cur[i].y < 0 || cur[i].x >= ir8.cols || cur[i].y >= ir8.rows) continue;
                img.push_back(cur[i]); obj.push_back(world_[i]); idx.push_back(int(i));
            }
            tracked_ = int(img.size());
            if (img.size() >= 20) {
                cv::Matx33d Rcw0 = Rpred.t();
                cv::Mat rvec, tvec;
                cv::Rodrigues(cv::Mat(Rcw0), rvec);
                tvec = cv::Mat(-(Rcw0 * Cpred));
                std::vector<int> inl;
                const bool sol = cv::solvePnPRansac(obj, img, cv::Mat(K_), cv::noArray(), rvec, tvec, true,
                                                    100, 2.0f, 0.99, inl, cv::SOLVEPNP_ITERATIVE);
                if (sol && inl.size() >= 20) {
                    std::vector<cv::Point2f> i2; std::vector<cv::Point3f> o2;
                    for (int k : inl) { i2.push_back(img[size_t(k)]); o2.push_back(obj[size_t(k)]); }
                    cv::solvePnPRefineLM(o2, i2, cv::Mat(K_), cv::noArray(), rvec, tvec);
                    cv::Mat Rm; cv::Rodrigues(rvec, Rm);
                    const cv::Matx33d Rcw(Rm);
                    const cv::Vec3d C = -(Rcw.t() * cv::Vec3d(tvec));
                    const cv::Matx33d Rwc = Rcw.t();
                    const double jump = cv::norm(C - C_);
                    const cv::Matx33d rel = Rwc * R_.t();
                    const double turn = std::acos(std::max(-1.0, std::min(1.0, (rel(0, 0) + rel(1, 1) + rel(2, 2) - 1) / 2)));
                    if (jump <= maxJump_ && turn < 10.0 * kPi / 180.0) {
                        vel_ = 0.5 * vel_ + 0.5 * (C - C_);
                        R_ = Rwc; C_ = C; ok = true; inliers_ = int(inl.size());
                        std::vector<cv::Point2f> np; std::vector<cv::Point3f> nw;
                        for (int k : inl) { np.push_back(img[size_t(k)]); nw.push_back(obj[size_t(k)]); }
                        pts_.swap(np); world_.swap(nw);
                    }
                }
            }
        }
        if (!ok && !prev_.empty()) {
            // LOST: coast on the prediction, re-anchor below.
            R_ = Rpred; C_ = Cpred; pts_.clear(); world_.clear(); ++lostRun_;
        } else if (ok) {
            lostRun_ = 0;
        }
        // Keyframe policy, as DepthVio's.
        const cv::Matx33d relK = R_ * kfR_.t();
        const double turnedK = std::acos(std::max(-1.0, std::min(1.0, (relK(0, 0) + relK(1, 1) + relK(2, 2) - 1) / 2)));
        if (prev_.empty() || !ok || pts_.size() < 80 || cv::norm(C_ - kfC_) > 0.6 || turnedK > 15.0 * kPi / 180.0)
            keyframe(ir8, depthAt(R_, C_));
        prev_ = ir8.clone();
        return ok;
    }
    const cv::Matx33d& R() const { return R_; }
    const cv::Vec3d& C() const { return C_; }
    int inliers() const { return inliers_; }
    int tracked() const { return tracked_; }

private:
    void keyframe(const cv::Mat& ir8, const cv::Mat& depth) {
        cv::Mat mask(ir8.size(), CV_8U, cv::Scalar(0));
        for (int v = 0; v < depth.rows; ++v)
            for (int u = 0; u < depth.cols; ++u) {
                const float r = depth.at<float>(v, u);
                if (r >= 0.3f && r <= maxDepth_) mask.at<uchar>(v, u) = 255;
            }
        for (const cv::Point2f& q : pts_) cv::circle(mask, q, 8, cv::Scalar(0), -1);
        const int want = 250 - int(pts_.size());
        if (want > 0) {
            std::vector<cv::Point2f> fresh;
            cv::goodFeaturesToTrack(ir8, fresh, want, 0.01, 8.0, mask);
            for (const cv::Point2f& q : fresh) {
                const float r = depth.at<float>(int(q.y + 0.5f), int(q.x + 0.5f));
                if (!(r >= 0.3f && r <= maxDepth_)) continue;
                const double x = (q.x - K_(0, 2)) / K_(0, 0), y = (q.y - K_(1, 2)) / K_(1, 1);
                const double n = std::sqrt(x * x + y * y + 1.0);
                const cv::Vec3d pw = R_ * cv::Vec3d(x / n * r, y / n * r, r / n) + C_;
                pts_.push_back(q);
                world_.push_back(cv::Point3f(float(pw[0]), float(pw[1]), float(pw[2])));
            }
        }
        kfR_ = R_; kfC_ = C_;
    }
    const sim::DepthCamera* cam_ = nullptr;
    cv::Matx33d K_, R_ = cv::Matx33d::eye(), Rprev_ = cv::Matx33d::eye(), kfR_ = cv::Matx33d::eye();
    cv::Vec3d C_, Cprev_, vel_, kfC_;
    float maxDepth_ = 15.f, maxJump_ = 0.5f;
    bool started_ = false;
    int lostRun_ = 0, tracked_ = 0, inliers_ = 0;
    cv::Mat prev_;
    std::vector<cv::Point2f> pts_;
    std::vector<cv::Point3f> world_;
};

}  // namespace downvo
