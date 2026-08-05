#include "depth_camera.hpp"

#include <cstdlib>
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

    // PASS 1: geometry. Raycast every pixel, keep the range and the surface's
    // own texture. No dropout decisions yet -- they need the neighbourhood.
    cv::Mat raw(p_.height, p_.width, CV_32F, cv::Scalar(-1.f));
    cv::Mat texM(p_.height, p_.width, CV_32F, cv::Scalar(0.f));
    for (int v = 0; v < p_.height; ++v) {
        float* rr = raw.ptr<float>(v);
        float* tr = texM.ptr<float>(v);
        for (int u = 0; u < p_.width; ++u) {
            float dx, dy, dz; rayFor(pose, u, v, dx, dy, dz);
            float tex = 0.f;
            float t = w.raycast(pose.e, pose.n, pose.u, dx, dy, dz, p_.maxRangeM, &tex);
            if (t >= p_.maxRangeM) continue;         // sky or beyond range
            if (t < minRange) continue;              // near blind zone
            if (u < fB / t) continue;                // occlusion band
            rr[u] = t; tr[u] = tex;
        }
    }

    // PASS 2: SILHOUETTE. A pixel whose neighbourhood spans a large depth step
    // sits on an edge, and an edge is the easiest thing in the scene to
    // correlate -- which is why real depth images of a forest show trunks as
    // black columns with bright stripes down one side. Raise the effective
    // texture there; the discontinuity IS the feature.
    if (p_.edgeWinPx > 0 && p_.edgeBoost > 0.f) {
        cv::Mat boosted = texM.clone();
        const int W = p_.edgeWinPx;
        for (int v = 0; v < p_.height; ++v)
            for (int u = 0; u < p_.width; ++u) {
                if (!(raw.at<float>(v, u) > 0)) continue;
                float lo = 1e9f, hi = -1e9f;
                bool sawSky = false;
                for (int dv = -W; dv <= W; ++dv) {
                    int yy = v + dv; if (yy < 0 || yy >= p_.height) continue;
                    for (int du = -W; du <= W; ++du) {
                        int xx = u + du; if (xx < 0 || xx >= p_.width) continue;
                        float q = raw.at<float>(yy, xx);
                        // No return next door is itself an edge: that is the
                        // trunk-against-sky case, the strongest cue there is.
                        if (!(q > 0)) { sawSky = true; continue; }
                        lo = std::min(lo, q); hi = std::max(hi, q);
                    }
                }
                if (sawSky || (hi - lo) > p_.edgeDepthM)
                    boosted.at<float>(v, u) = std::max(texM.at<float>(v, u), p_.edgeBoost);
            }
        texM = boosted;
    }

    // PASS 3: matching, using the effective texture from above.
    for (int v = 0; v < p_.height; ++v) {
        float* row = d.ptr<float>(v);
        for (int u = 0; u < p_.width; ++u) {
            float t = raw.at<float>(v, u);
            if (!(t > 0)) continue;
            float tex = texM.at<float>(v, u);

            // TEXTURE. The whole point. A featureless surface yields no match,
            // and the map must treat that as UNKNOWN, never as free.
            if (tex < p_.texThresh) continue;
            // Near the threshold matching is unreliable rather than impossible.
            // The draw is per BLOCK, not per pixel -- see blockPx. A matcher
            // correlates a window, so a marginal surface fails in window-sized
            // patches. Deriving the draw from the block index rather than a
            // running RNG also makes it stable frame to frame for a stationary
            // camera, which is what a real matcher does and what a per-pixel
            // coin flip conspicuously does not.
            float pFail = (tex < p_.texThresh * 2.f)
                        ? 1.f - (tex - p_.texThresh) / p_.texThresh : 0.f;
            if (pFail > 0.f) {
                float draw;
                if (p_.blockPx > 0) {
                    uint32_t bx = uint32_t(u / p_.blockPx), by = uint32_t(v / p_.blockPx);
                    uint32_t h = bx * 73856093u ^ by * 19349663u ^ p_.seed * 83492791u;
                    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
                    draw = float(h & 0xFFFFFF) / float(0x1000000);
                } else {
                    draw = urand();
                }
                if (draw < pFail * 0.8f) continue;
            }

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
    // SPECKLE / CONSISTENCY REJECTION, modelling what every real pipeline runs
    // after matching: left-right consistency plus filterSpeckles. A valid pixel
    // in a neighbourhood that is mostly invalid was almost certainly a bad
    // match, and throwing it away is what gives real depth images clean hole
    // edges instead of a fringe of survivors.
    //
    // It also removes most of the isolated wrong-depth speckle this model
    // injects above -- which is correct, and worth stating: the dangerous
    // artefact in practice is not the lone bad pixel that a filter catches, it
    // is the coherent hole that no filter can fill.
    if (p_.filterSpeckle && p_.speckleWin > 0) {
        cv::Mat src = d.clone();
        const int W = p_.speckleWin;
        const int need = int(p_.speckleKeep * float((2*W+1) * (2*W+1)) + 0.5f);
        for (int v = 0; v < p_.height; ++v) {
            float* row = d.ptr<float>(v);
            for (int u = 0; u < p_.width; ++u) {
                if (!(src.at<float>(v, u) > 0)) continue;
                int ok = 0;
                for (int dv = -W; dv <= W; ++dv) {
                    int yy = v + dv; if (yy < 0 || yy >= p_.height) continue;
                    const float* s = src.ptr<float>(yy);
                    for (int du = -W; du <= W; ++du) {
                        int xx = u + du; if (xx < 0 || xx >= p_.width) continue;
                        if (s[xx] > 0) ++ok;
                    }
                }
                if (ok < need) { row[u] = -1.f; --valid; }
            }
        }
    }
    if (validFrac) *validFrac = float(valid) / float(p_.width * p_.height);
    return d;
}

}  // namespace sim
