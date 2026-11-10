#pragma once
// ---------------------------------------------------------------------------
// SlamAnchor -- place an external SLAM's frame into local ENU.
//
// ORB-SLAM3 (stereo) starts its world at the first camera pose: metric, but
// with arbitrary heading and a tilt equal to however the camera was tilted.
// The aircraft wants ENU. So at one frame -- the first tracked one, and again
// after every map change -- the anchor pins "this SLAM pose IS this ENU pose":
// roll/pitch from the IMU (drift-free), heading from the compass or the last
// estimate, position where the estimate last was. Every later SLAM pose is
// carried through that fixed transform.
//
// Camera axes are navcore's and ORB-SLAM3's alike: +x right, +y down, +z
// forward. The rotation convention is taken from DepthCamera::camToWorld, the
// one definition in the tree.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"   // navcore

class SlamAnchor {
public:
    // Camera -> world rotation for a navcore pose.
    static cv::Matx33d rotWc(const sim::CamPose& p) {
        cv::Matx33d R;
        const float basis[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int c = 0; c < 3; ++c) {
            float x, y, z;
            sim::DepthCamera::camToWorld(p, basis[c][0], basis[c][1], basis[c][2], x, y, z);
            R(0, c) = x; R(1, c) = y; R(2, c) = z;
        }
        return R;
    }
    // Heading of the optical axis, degrees clockwise from North.
    static float yawOf(const cv::Matx33d& Rwc) {
        return float(std::atan2(Rwc(0, 2), Rwc(1, 2)) * 180.0 / 3.14159265358979);
    }

    // Twc: 3x4 row-major camera -> SLAM world (SlamLink's PoseReply::Twc).
    void anchor(const float* Twc, const sim::CamPose& enuPose) {
        cv::Matx33d Rsc; cv::Vec3d tsc;
        split(Twc, Rsc, tsc);
        R_ = rotWc(enuPose) * Rsc.t();
        t_ = cv::Vec3d(enuPose.e, enuPose.n, enuPose.u) - R_ * tsc;
        ok_ = true;
    }
    bool anchored() const { return ok_; }

    // JUMP GUARD. A pose further from the last accepted one than the aircraft
    // could have flown since (maxSpeedMs * dt, plus a slack for a loop
    // closure's drift correction) is not motion: it is the SLAM's frame
    // moving under us -- a re-initialisation, a map merge, a bad
    // relocalisation. Seen: two closed-loop ORB-SLAM3 runs under CPU
    // contention ended 10-19 m off while losing few frames, a failure the
    // same worlds do not show run alone; this is the defence against the class.
    static bool plausible(float de, float dn, float du, double dtS,
                          float maxSpeedMs = 4.f, float slackM = 0.5f) {
        const double d = std::sqrt(double(de) * de + double(dn) * dn + double(du) * du);
        return d <= maxSpeedMs * std::max(0.0, dtS) + slackM;
    }
    void clear() { ok_ = false; }

    // ENU position and heading of the camera for a SLAM pose. Roll/pitch are
    // NOT returned: the IMU's are better than a value carried through a
    // fixed transform from one noisy sample.
    void toEnu(const float* Twc, float& e, float& n, float& u, float& yawDeg) const {
        cv::Matx33d Rsc; cv::Vec3d tsc;
        split(Twc, Rsc, tsc);
        const cv::Vec3d p = R_ * tsc + t_;
        e = float(p[0]); n = float(p[1]); u = float(p[2]);
        yawDeg = yawOf(R_ * Rsc);
    }

private:
    static void split(const float* T, cv::Matx33d& R, cv::Vec3d& t) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) R(i, j) = T[i * 4 + j];
            t[i] = T[i * 4 + 3];
        }
    }
    cv::Matx33d R_ = cv::Matx33d::eye();
    cv::Vec3d   t_ = cv::Vec3d(0, 0, 0);
    bool        ok_ = false;
};
