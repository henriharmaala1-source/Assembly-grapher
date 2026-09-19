#include "depth_camera.hpp"

#include <cstdlib>
#include <cmath>
#include <cstdint>

namespace sim {

DepthCamera::DepthCamera(const CamParams& p) : p_(p), rng_(p.seed ? p.seed : 1u) {
    fpx_ = (p_.fxPx > 0.f)
         ? p_.fxPx
         : (p_.width * 0.5f) / std::tan(p_.hfovDeg * 0.5f * sim::PI_F / 180.f);
    fy_  = (p_.fyPx > 0.f) ? p_.fyPx : fpx_;
    ppx_ = (p_.ppxPx > 0.f) ? p_.ppxPx : (p_.width  - 1) * 0.5f;
    ppy_ = (p_.ppyPx > 0.f) ? p_.ppyPx : (p_.height - 1) * 0.5f;
}

float DepthCamera::urand() const {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return float(rng_ & 0xFFFFFF) / float(0x1000000);
}

void DepthCamera::camToWorld(const CamPose& pose, float rx, float ry, float rz,
                             float& dx, float& dy, float& dz) {
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

void DepthCamera::rayFor(const CamPose& pose, int u, int v,
                         float& dx, float& dy, float& dz) const {
    // Camera frame: +X right, +Y down, +Z forward. Pixel -> normalised ray.
    camToWorld(pose, (u - ppx_) / fpx_, (v - ppy_) / fy_, 1.f, dx, dy, dz);
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

    // PASS 1b: OCCLUSION SHADOW. A pixel is visible to the left imager but
    // hidden from the right whenever some pixel FURTHER RIGHT maps to the same
    // or a smaller right-image coordinate -- i.e. something nearer has moved in
    // front of it by more than the disparity difference.
    //
    //     x_right(u) = u - f*B/Z(u)
    //     u is occluded  <=>  min over u' > u of x_right(u')  <=  x_right(u)
    //
    // One right-to-left sweep per row, O(1) per pixel. This is the same
    // left-right consistency test a real matcher runs, which is why real
    // hardware produces exactly this artefact rather than merely something
    // like it: the strip is not noise, it is the geometry admitting it has no
    // second view.
    //
    // The shadow falls on the LEFT of a near object because the right imager
    // sits at +X. Same convention as the frame-edge band above, and it is worth
    // stating because a shadow on the wrong side would look plausible and quietly
    // mirror every obstacle boundary in the map.
    if (p_.modelOcclusion) {
        for (int v = 0; v < p_.height; ++v) {
            float* rr = raw.ptr<float>(v);
            float* tr = texM.ptr<float>(v);
            float minXr = 1e30f;
            for (int u = p_.width - 1; u >= 0; --u) {
                if (!(rr[u] > 0.f)) continue;
                const float xr = float(u) - fB / rr[u];
                if (xr >= minXr) { rr[u] = -1.f; tr[u] = 0.f; }   // no second view
                else minXr = xr;
            }
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

            // IR PROJECTOR. It puts features on surfaces that have none, which
            // is the same quantity texThresh gates on -- so it enters as a
            // FLOOR under the scene's own texture, not as a multiplier. A
            // richly textured trunk gains nothing from it; a smooth shadowed
            // one gains everything, which is the actual asymmetry.
            if (p_.emitterOn) {
                const float amb = std::max(0.f, std::min(1.f, p_.ambientIR));
                const float rr = t / std::max(0.01f, p_.emitterRangeM);
                const float proj = p_.emitterTex * (1.f - amb) / (1.f + rr * rr);
                tex = std::max(tex, proj);
            }

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

// ---------------------------------------------------------------------------
// SYNTHETIC IR. See the header for why intensity must be a function of the
// world point rather than of the pixel.
namespace {

inline float hash3i(int x, int y, int z) {
    uint32_t h = uint32_t(x) * 73856093u ^ uint32_t(y) * 19349663u
               ^ uint32_t(z) * 83492791u;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return float(h & 0xFFFFFFu) / float(0xFFFFFF);
}

// Trilinear value noise. SMOOTH, and that is not cosmetic: two viewpoints hit
// a surface at slightly different points, and a discontinuous function would
// return unrelated values for neighbouring samples of the same patch. The
// matcher would then see noise where the world has texture.
inline float vnoise(float x, float y, float z) {
    const float fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);
    const int ix = int(fx), iy = int(fy), iz = int(fz);
    float tx = x - fx, ty = y - fy, tz = z - fz;
    tx = tx * tx * (3.f - 2.f * tx);
    ty = ty * ty * (3.f - 2.f * ty);
    tz = tz * tz * (3.f - 2.f * tz);
    auto L = [](float a, float b, float t) { return a + (b - a) * t; };
    const float c00 = L(hash3i(ix, iy,   iz),   hash3i(ix+1, iy,   iz),   tx);
    const float c10 = L(hash3i(ix, iy+1, iz),   hash3i(ix+1, iy+1, iz),   tx);
    const float c01 = L(hash3i(ix, iy,   iz+1), hash3i(ix+1, iy,   iz+1), tx);
    const float c11 = L(hash3i(ix, iy+1, iz+1), hash3i(ix+1, iy+1, iz+1), tx);
    return L(L(c00, c10, ty), L(c01, c11, ty), tz);
}

}  // namespace

cv::Mat DepthCamera::renderIR(const VoxelWorld& w, const CamPose& pose) const {
    cv::Mat ir(p_.height, p_.width, CV_8U, cv::Scalar(0));
    const float LAT = 1.f / 0.04f;      // 4 cm primary feature scale
    for (int v = 0; v < p_.height; ++v) {
        uchar* row = ir.ptr<uchar>(v);
        for (int u = 0; u < p_.width; ++u) {
            float dx, dy, dz; rayFor(pose, u, v, dx, dy, dz);
            float tex = 0.f;
            const float t = w.raycast(pose.e, pose.n, pose.u, dx, dy, dz,
                                      p_.maxRangeM, &tex);
            if (t >= p_.maxRangeM) {          // sky: dark, and flat on purpose
                row[u] = uchar(std::max(0.f, 6.f + 3.f * (urand() - 0.5f)));
                continue;
            }
            const float hx = pose.e + dx * t, hy = pose.n + dy * t,
                        hz = pose.u + dz * t;
            // Two octaves so the surface has structure at more than one scale,
            // which is what a correlation window actually locks onto.
            const float n = 0.65f * vnoise(hx * LAT, hy * LAT, hz * LAT)
                          + 0.35f * vnoise(hx * LAT * 3.f, hy * LAT * 3.f,
                                           hz * LAT * 3.f);
            // Active illumination falls off with range. Half brightness at 4 m.
            const float fall = 1.f / (1.f + (t * t) / 16.f);
            const float base = 30.f + 170.f * fall;
            const float amp  = std::min(1.f, std::max(0.f, tex));
            float val = base * (1.f - 0.5f * amp) + base * amp * n;
            val += 2.0f * (urand() - 0.5f) * 2.f;      // sensor noise
            row[u] = uchar(std::min(255.f, std::max(0.f, val)));
        }
    }
    return ir;
}

}  // namespace sim
