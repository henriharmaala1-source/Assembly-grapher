#include "depth_camera.hpp"

#include <cmath>

namespace sim {

DepthCamera::DepthCamera(const CamParams& p) : p_(p), rng_(p.seed ? p.seed : 1u) {
    fpx_ = (p_.width * 0.5f) / std::tan(p_.hfovDeg * 0.5f * sim::PI_F / 180.f);
}

float DepthCamera::urand() const {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return float(rng_ & 0xFFFFFF) / float(0x1000000);
}

void DepthCamera::rayFor(const CamPose& pose, int u, int v,
                         float& dx, float& dy, float& dz) const {
    // Camera frame: +X right, +Y down, +Z forward. Pixel -> normalised ray.
    const float cx = (p_.width - 1) * 0.5f, cy = (p_.height - 1) * 0.5f;
    float rx = (u - cx) / fpx_;
    float ry = (v - cy) / fpx_;
    float rz = 1.f;

    // Rotate into world. Yaw is measured from +North, clockwise (matching
    // sim_world's bearing convention), so forward at yaw=0 is +y.
    const float cr = std::cos(pose.rollDeg  * sim::PI_F / 180.f);
    const float sr = std::sin(pose.rollDeg  * sim::PI_F / 180.f);
    const float cp = std::cos(pose.pitchDeg * sim::PI_F / 180.f);
    const float sp = std::sin(pose.pitchDeg * sim::PI_F / 180.f);
    const float cy_ = std::cos(pose.yawDeg  * sim::PI_F / 180.f);
    const float sy_ = std::sin(pose.yawDeg  * sim::PI_F / 180.f);

    // roll about forward axis
    float x1 = rx * cr - ry * sr;
    float y1 = rx * sr + ry * cr;
    float z1 = rz;
    // pitch about right axis (nose up = ray tilts up)
    float y2 = y1 * cp - z1 * sp;
    float z2 = y1 * sp + z1 * cp;
    float x2 = x1;
    // Camera axes -> ENU: forward(z2)->North, right(x2)->East, down(y2)->-Up
    float fE = x2, fN = z2, fU = -y2;
    // yaw clockwise from North
    dx = fE * cy_ + fN * sy_;
    dy = -fE * sy_ + fN * cy_;
    dz = fU;
}

cv::Mat DepthCamera::renderTruth(const VoxelWorld& w, const CamPose& pose) const {
    cv::Mat d(p_.height, p_.width, CV_32F, cv::Scalar(-1.f));
    for (int v = 0; v < p_.height; ++v) {
        float* row = d.ptr<float>(v);
        for (int u = 0; u < p_.width; ++u) {
            float dx, dy, dz; rayFor(pose, u, v, dx, dy, dz);
            float t = w.raycast(pose.e, pose.n, pose.u, dx, dy, dz, p_.maxRangeM, nullptr);
            row[u] = (t >= p_.maxRangeM) ? -1.f : t;
        }
    }
    return d;
}

cv::Mat DepthCamera::renderStereo(const VoxelWorld& w, const CamPose& pose,
                                  float* validFrac) const {
    cv::Mat d(p_.height, p_.width, CV_32F, cv::Scalar(-1.f));
    const float fB = fpx_ * p_.baselineM;
    const float minRange = fB / float(p_.maxDisp);   // near blind zone
    long valid = 0;

    for (int v = 0; v < p_.height; ++v) {
        float* row = d.ptr<float>(v);
        for (int u = 0; u < p_.width; ++u) {
            float dx, dy, dz; rayFor(pose, u, v, dx, dy, dz);
            float tex = 0.f;
            float t = w.raycast(pose.e, pose.n, pose.u, dx, dy, dz, p_.maxRangeM, &tex);

            if (t >= p_.maxRangeM) continue;         // nothing there: sky, far
            if (t < minRange) continue;              // inside the blind zone

            // Occlusion band: the left f*B/Z columns see surface the right
            // camera cannot. Nothing to match against, so no depth.
            if (u < fB / t) continue;

            // TEXTURE. The whole point. A featureless surface yields no match,
            // and the map must treat that as UNKNOWN, never as free.
            if (tex < p_.texThresh) continue;
            // Near the threshold matching is unreliable rather than impossible.
            float pFail = (tex < p_.texThresh * 2.f)
                        ? 1.f - (tex - p_.texThresh) / p_.texThresh : 0.f;
            if (pFail > 0.f && urand() < pFail * 0.8f) continue;

            // Speckle: a confidently WRONG match. Worse than a hole.
            if (urand() < p_.speckleFrac) {
                float bogus = minRange + urand() * (p_.maxRangeM - minRange);
                row[u] = bogus; ++valid;
                continue;
            }

            // Quantise through disparity so error grows as Z^2/(f*B), which is
            // what a real matcher does and what a naive "depth + x% noise"
            // model gets wrong.
            float disp = fB / t;
            // Box-Muller-free approximation: sum of two uniforms is close
            // enough to Gaussian for this and is cheaper.
            float noise = (urand() + urand() - 1.f) * p_.subpixelPx * 1.732f;
            disp += noise;
            if (disp < 1.f) continue;                 // beyond usable range
            row[u] = fB / disp;
            ++valid;
        }
    }
    if (validFrac) *validFrac = float(valid) / float(p_.width * p_.height);
    return d;
}

}  // namespace sim
