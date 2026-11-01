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
//   PlaceMemory  "I have been here": stored ground fingerprints that, when
//                recognised again, correct the drift since
//
// Test-harness code, not the aircraft's.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
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

// ---------------------------------------------------------------------------
// PlaceMemory -- "I have been here before": a landmark memory for the
// downward camera, the part of SLAM (RTAB-Map, ORB-SLAM3) that bounds drift,
// without the rest of SLAM.
//
// STORE: every spacingM of flight, ORB corners of the ground and where each
// one is in the world -- its pixel ray from the pose at that moment, cut
// against the ground plane H below. A place is a set of fingerprints with
// world positions, nothing more.
//
// RECOGNISE: every few frames, against the places old enough to be a real
// return (minAgeS) and near enough to be possible (searchBaseM + searchFrac
// of the distance flown since: odometry drifts a percentage of distance, so
// it cannot be further off than that). Fingerprints matched with a ratio test,
// then the GEOMETRY decides: PnP with RANSAC from the current estimate, and
// the result must agree with what is trusted INDEPENDENTLY of the odometry --
// the height source, and the tilt where a gimbal holds it -- before it counts.
// Not with the odometry's own estimate: that is what drifted, and the first
// version, gated on it, rejected a correct 63-inlier return to the start
// because the vision-only estimate it was correcting was 44 deg off in tilt. A look-alike patch of grass
// is the failure that matters, because a false "been here" teleports the
// estimate; every test after the fingerprint match is there for that.
//
// What a FIX gives is the pose relative to the place as it was stored, so it
// removes the drift accumulated SINCE then -- the start point, stored first,
// is the best place to come back to.
class PlaceMemory {
public:
    struct Params {
        float spacingM    = 2.0f;   // store a place every this far
        float minAgeS     = 15.0f;  // only places at least this old are a return
        float searchBaseM = 3.0f;   // candidates within base + frac * flown-since
        float searchFrac  = 0.2f;
        int   features    = 800;
        float ratio       = 0.8f;   // Lowe's ratio test
        int   minInliers  = 40;
        float maxTiltErrDeg = 4.0f; // the fix's optical axis against the trusted one
        float maxHeightErr  = 0.25f;  // fraction of the height
        float maxRangeH   = 4.0f;   // ground points no further than this x height
    };
    struct Fix {
        bool ok = false;
        cv::Matx33d R;              // camera -> world, in the memory's frame
        cv::Vec3d C;
        int inliers = 0, place = -1;
        double ageS = 0;
    };

    void init(const sim::DepthCamera& cam, const Params& p) {
        p_ = p;
        K_ = cv::Matx33d(cam.fpx(), 0, cam.ppx(), 0, cam.fy(), cam.ppy(), 0, 0, 1);
        // A LOWER CORNER THRESHOLD, after local contrast normalisation. At
        // ORB's default (FAST 20) the synthetic low-contrast ground gave ZERO
        // corners -- and so would grass from height or dusk -- while FAST at 5
        // found a thousand. CLAHE lifts the contrast locally; 10 keeps noise
        // out of the fingerprints.
        orb_ = cv::ORB::create(p.features, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 10);
        clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
    }

    // Every frame, with the CORRECTED pose. True if a place was stored.
    bool maybeStore(const cv::Mat& gray, double t, double flownM, const cv::Matx33d& Rwc,
                    const cv::Vec3d& C, double H) {
        if (!places_.empty() && flownM - lastStoreFlown_ < p_.spacingM) return false;
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc, eq;
        clahe_->apply(gray, eq);
        orb_->detectAndCompute(eq, cv::noArray(), kp, desc);
        Place pl;
        pl.t = t; pl.flownM = flownM; pl.C = C;
        for (size_t i = 0; i < kp.size(); ++i) {
            const double x = (kp[i].pt.x - K_(0, 2)) / K_(0, 0), y = (kp[i].pt.y - K_(1, 2)) / K_(1, 1);
            const cv::Vec3d d = Rwc * cv::Vec3d(x, y, 1.0);
            const double n = cv::norm(d);
            if (d[2] / n > -0.05) continue;                  // above or along the horizon
            const double s = H / -d[2];
            if (s * n > p_.maxRangeH * H) continue;          // too oblique to trust
            const cv::Vec3d w = C + s * d;
            pl.world.emplace_back(float(w[0]), float(w[1]), float(w[2]));
            pl.desc.push_back(desc.row(int(i)));
        }
        if (pl.world.size() < size_t(p_.minInliers)) return false;
        places_.push_back(std::move(pl));
        lastStoreFlown_ = flownM;
        return true;
    }

    // Look for a return. `Rwc`, `C`: the current corrected estimate (the
    // RANSAC starts from it). `H`: the height above the ground from the
    // height source; `tiltRef`: the camera rotation whose TILT is trusted (the
    // gimbal's), or null when there is none (vision-only attitude).
    Fix query(const cv::Mat& gray, double t, double flownM, const cv::Matx33d& Rwc, const cv::Vec3d& C,
              double H, const cv::Matx33d* tiltRef) {
        Fix f;
        std::vector<std::pair<double, int>> cand;
        for (int i = 0; i < int(places_.size()); ++i) {
            const Place& pl = places_[size_t(i)];
            if (t - pl.t < p_.minAgeS) continue;
            const double d = std::hypot(C[0] - pl.C[0], C[1] - pl.C[1]);
            if (d <= p_.searchBaseM + p_.searchFrac * (flownM - pl.flownM)) cand.emplace_back(d, i);
        }
        if (cand.empty()) return f;
        std::sort(cand.begin(), cand.end());
        ++queries_;
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc, eq;
        clahe_->apply(gray, eq);
        orb_->detectAndCompute(eq, cv::noArray(), kp, desc);
        if (desc.empty()) return f;
        cv::BFMatcher bf(cv::NORM_HAMMING);
        for (size_t c = 0; c < cand.size() && c < 3; ++c) {
            const Place& pl = places_[size_t(cand[c].second)];
            std::vector<std::vector<cv::DMatch>> km;
            bf.knnMatch(desc, pl.desc, km, 2);
            std::vector<cv::Point3f> obj;
            std::vector<cv::Point2f> img;
            for (const auto& m : km)
                if (m.size() == 2 && m[0].distance < p_.ratio * m[1].distance) {
                    obj.push_back(pl.world[size_t(m[0].trainIdx)]);
                    img.push_back(kp[size_t(m[0].queryIdx)].pt);
                }
            if (int(obj.size()) < p_.minInliers) continue;
            const cv::Matx33d Rcw0 = Rwc.t();
            cv::Mat rvec, tvec;
            cv::Rodrigues(cv::Mat(Rcw0), rvec);
            tvec = cv::Mat(-(Rcw0 * C));
            std::vector<int> inl;
            if (!cv::solvePnPRansac(obj, img, cv::Mat(K_), cv::noArray(), rvec, tvec, true, 200, 3.0f, 0.99,
                                    inl, cv::SOLVEPNP_ITERATIVE))
                continue;
            if (int(inl.size()) < p_.minInliers) { ++rejected_; continue; }
            cv::Mat Rm;
            cv::Rodrigues(rvec, Rm);
            const cv::Matx33d Rcw(Rm);
            const cv::Matx33d R = Rcw.t();
            const cv::Vec3d Cs = -(R * cv::Vec3d(tvec));
            // THE GATES: tilt and height are trusted outright; a fix that
            // disagrees with them is a look-alike, not a return.
            const double hFix = Cs[2] - double(pl.world.front().z);   // above that place's ground
            const bool tiltOk = !tiltRef || tiltErrDeg(R, *tiltRef) <= p_.maxTiltErrDeg;
            const bool heightOk = std::fabs(hFix - H) <= p_.maxHeightErr * std::max(0.5, H);
            const double moved = std::hypot(Cs[0] - C[0], Cs[1] - C[1]);
            const bool nearOk = moved <= p_.searchBaseM + p_.searchFrac * (flownM - pl.flownM);
            if (std::getenv("PLACE_TRACE"))
                std::fprintf(stderr, "place %d age %.1f s: %zu matches, %zu inliers, tilt err %.2f deg, height %.2f vs %.2f, "
                                     "moves %.2f m\n", cand[c].second, t - pl.t, obj.size(), inl.size(),
                             tiltRef ? tiltErrDeg(R, *tiltRef) : -1.0, hFix, H, moved);
            if (!(tiltOk && heightOk && nearOk)) { ++rejected_; continue; }
            f.ok = true; f.R = R; f.C = Cs; f.inliers = int(inl.size()); f.place = cand[c].second;
            f.ageS = t - pl.t;
            ++accepted_;
            return f;
        }
        return f;
    }

    int stored() const { return int(places_.size()); }
    double placeT(int i) const { return places_[size_t(i)].t; }
    double placeFlown(int i) const { return places_[size_t(i)].flownM; }
    int queries() const { return queries_; }
    int accepted() const { return accepted_; }
    int rejected() const { return rejected_; }

private:
    struct Place {
        double t = 0, flownM = 0;
        cv::Vec3d C;
        cv::Mat desc;
        std::vector<cv::Point3f> world;
    };
    Params p_;
    cv::Matx33d K_;
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::CLAHE> clahe_;
    std::vector<Place> places_;
    double lastStoreFlown_ = -1e9;
    int queries_ = 0, accepted_ = 0, rejected_ = 0;
};

}  // namespace downvo
