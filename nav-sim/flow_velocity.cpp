#include "flow_velocity.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

namespace {

// SSD over a patch, with early-out. Deliberately not a fancy matcher: the
// measurement above shows precision barely matters, so spending CPU on
// sub-pixel refinement would buy almost nothing against the cost of existing.
float ssd(const cv::Mat& a, int ax, int ay,
          const cv::Mat& b, int bx, int by, int half) {
    float s = 0.f;
    for (int dy = -half; dy <= half; ++dy)
        for (int dx = -half; dx <= half; ++dx) {
            const float d = float(a.at<uint8_t>(ay + dy, ax + dx)) -
                            float(b.at<uint8_t>(by + dy, bx + dx));
            s += d * d;
        }
    return s;
}

// Variance of a patch. A flat patch matches everywhere equally well, so its
// "best" offset is noise wearing the costume of a measurement.
float patchVar(const cv::Mat& m, int cx, int cy, int half) {
    float sum = 0.f, sum2 = 0.f; int n = 0;
    for (int dy = -half; dy <= half; ++dy)
        for (int dx = -half; dx <= half; ++dx) {
            const float v = float(m.at<uint8_t>(cy + dy, cx + dx));
            sum += v; sum2 += v * v; ++n;
        }
    const float mean = sum / n;
    return sum2 / n - mean * mean;
}

// Best integer offset of (cx,cy) from `from` into `to`. Returns false if the
// best match sits on the search boundary, which means the true match is
// probably outside it and the number would be a floor, not a measurement.
bool match(const cv::Mat& from, const cv::Mat& to, int cx, int cy,
           int half, int search, int& bx, int& by) {
    float best = 1e30f; bx = 0; by = 0;
    for (int dy = -search; dy <= search; ++dy)
        for (int dx = -search; dx <= search; ++dx) {
            const float s = ssd(from, cx, cy, to, cx + dx, cy + dy, half);
            if (s < best) { best = s; bx = dx; by = dy; }
        }
    return std::abs(bx) < search && std::abs(by) < search;
}

}  // namespace

FlowVelocity FlowVelocityEstimator::estimate(
        const cv::Mat& prevIr, const cv::Mat& curIr, const cv::Mat& depth,
        const DepthCamera& cam, float dYawDeg, float dPitchDeg, float dRollDeg,
        float dt) const {
    FlowVelocity out;
    if (prevIr.empty() || curIr.empty() || depth.empty() || dt <= 0.f) return out;
    if (prevIr.size() != curIr.size()) return out;

    const int W = prevIr.cols, H = prevIr.rows;
    const float f  = cam.fpx();
    const float cxp = cam.ppx(), cyp = cam.ppy();
    const float kR = 3.14159265358979f / 180.f;
    const float wy = dYawDeg * kR, wx = dPitchDeg * kR, wz = dRollDeg * kR;

    const int half = p_.patch;
    const int marg = half + p_.search + 1;

    // 3x3 normal equations for t = (tx,ty,tz).
    double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double b[3] = {0,0,0};
    int used = 0;

    for (int gy = 0; gy < p_.gridY; ++gy) {
        for (int gx = 0; gx < p_.gridX; ++gx) {
            const int px = marg + int((W - 2*marg) * (gx + 0.5f) / p_.gridX);
            const int py = marg + int((H - 2*marg) * (gy + 0.5f) / p_.gridY);
            if (px < marg || py < marg || px >= W-marg || py >= H-marg) continue;

            const float Z = depth.at<float>(py, px);
            if (!(Z > p_.minRangeM) || Z > p_.maxRangeM) continue;
            if (patchVar(prevIr, px, py, half) < p_.minVar) continue;

            int dx, dy;
            if (!match(prevIr, curIr, px, py, half, p_.search, dx, dy)) continue;

            // FORWARD-BACKWARD CHECK. Match the found point back and require it
            // to land where it started. This is what rejects a repeated texture
            // matching the wrong instance of itself -- bark and foliage are full
            // of those, and without it they vote confidently for nonsense.
            // The backward window must be at least as wide as the forward
            // displacement, or the round trip always lands on its own boundary
            // and every point is rejected. The first version used a smaller
            // window and yielded ZERO points on a pair with a known 7 px shift:
            // the check was not strict, it was broken.
            int rx, ry;
            if (!match(curIr, prevIr, px + dx, py + dy, half,
                       std::max(p_.fbSearch, p_.search), rx, ry))
                continue;
            if (std::hypot(float(dx + rx), float(dy + ry)) > p_.fbMaxErrPx) continue;

            // DE-ROTATE. Small-angle rotational image motion, subtracted before
            // anything is attributed to translation.
            const float u = float(px) - cxp, v = float(py) - cyp;
            // SIGNS, derived rather than guessed -- the first version had both
            // backwards, which made de-rotation ADD to the phantom translation
            // instead of removing it (4.1 m/s where 0 was expected).
            //   yaw RIGHT (+wy)  -> scene moves LEFT   -> du = -f*wy
            //   pitch UP (+wx)   -> scene moves DOWN   -> dv = +f*wx  (y is down)
            //   roll RIGHT (+wz) -> scene rotates CCW  -> du = +wz*v, dv = -wz*u
            const float duRot = -f * wy + wz * v;
            const float dvRot =  f * wx - wz * u;
            const float du = float(dx) - duRot;
            const float dv = float(dy) - dvRot;

            // du = (-f/Z) tx + (u/Z) tz ;  dv = (-f/Z) ty + (v/Z) tz
            const double r0[3] = { -f / Z, 0.0,    u / Z };
            const double r1[3] = { 0.0,    -f / Z, v / Z };
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) A[i][j] += r0[i]*r0[j] + r1[i]*r1[j];
                b[i] += r0[i] * du + r1[i] * dv;
            }
            ++used;
        }
    }

    out.points = used;
    if (used < p_.minPoints) return out;

    // Solve by Gauss-Jordan with partial pivoting. Three unknowns; a library
    // would be more code than the maths.
    double M[3][4] = {{A[0][0],A[0][1],A[0][2],b[0]},
                      {A[1][0],A[1][1],A[1][2],b[1]},
                      {A[2][0],A[2][1],A[2][2],b[2]}};
    double pivMin = 1e300, pivMax = 0.0;
    for (int c = 0; c < 3; ++c) {
        int piv = c;
        for (int r = c + 1; r < 3; ++r)
            if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
        if (piv != c) for (int k = 0; k < 4; ++k) std::swap(M[c][k], M[piv][k]);
        const double d = M[c][c];
        if (std::fabs(d) < 1e-12) return out;         // singular: refuse
        pivMin = std::min(pivMin, std::fabs(d));
        pivMax = std::max(pivMax, std::fabs(d));
        for (int k = c; k < 4; ++k) M[c][k] /= d;
        for (int r = 0; r < 3; ++r) {
            if (r == c) continue;
            const double fct = M[r][c];
            for (int k = c; k < 4; ++k) M[r][k] -= fct * M[c][k];
        }
    }
    // A pivot-ratio proxy for the condition number. Crude, and honest about it:
    // its job is to catch the degenerate case -- all points at one depth dead
    // ahead, where forward motion is unobservable -- not to be exact.
    out.condition = float(pivMax / std::max(1e-12, pivMin));
    if (out.condition > p_.maxCondition) return out;

    out.vx = float(M[0][3] / dt);
    out.vy = float(M[1][3] / dt);
    out.vz = float(M[2][3] / dt);
    out.valid = true;
    return out;
}

}  // namespace sim
