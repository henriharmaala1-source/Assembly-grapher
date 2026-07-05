#include "sim_world.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace sim {

namespace {
constexpr float kPi = 3.14159265358979323846f;

// Range from p along unit dir d to circle (c,r); +inf-ish (maxRange) if it misses.
float rayCircle(float pe, float pn, float de, float dn,
                const Circle& o, float maxRange) {
    const float me = pe - o.e, mn = pn - o.n;
    const float b  = me * de + mn * dn;
    const float cc = me * me + mn * mn - o.r * o.r;
    const float disc = b * b - cc;
    if (disc < 0.f) return maxRange;
    const float sq = std::sqrt(disc);
    const float t1 = -b - sq, t2 = -b + sq;
    if (t2 < 0.f) return maxRange;      // circle entirely behind
    if (t1 < 0.f) return 0.f;           // origin inside
    return std::min(t1, maxRange);
}

// Range from p along unit dir d to segment (a->b). maxRange if no hit ahead.
float raySegment(float pe, float pn, float de, float dn,
                 const Wall& w, float maxRange) {
    const float se = w.e1 - w.e0, sn = w.n1 - w.n0;   // segment dir
    const float denom = de * sn - dn * se;            // cross(d, s)
    if (std::fabs(denom) < 1e-9f) return maxRange;     // parallel
    const float qe = w.e0 - pe, qn = w.n0 - pn;
    const float t = (qe * sn - qn * se) / denom;       // along the ray
    const float u = (qe * dn - qn * de) / denom;       // along the segment
    if (t < 0.f || u < 0.f || u > 1.f) return maxRange;
    return std::min(t, maxRange);
}
}  // namespace

void World::advance(float dt) {
    for (auto& c : circles) { c.e += c.ve * dt; c.n += c.vn * dt; }
}

float World::rayRange(float pe, float pn, float bearingDeg, float maxRange) const {
    const float b  = bearingDeg * kPi / 180.f;
    const float de = std::sin(b), dn = std::cos(b);   // 0 deg = +N, +E clockwise
    float best = maxRange;
    for (const auto& c : circles) best = std::min(best, rayCircle(pe, pn, de, dn, c, maxRange));
    for (const auto& w : walls)   best = std::min(best, raySegment(pe, pn, de, dn, w, maxRange));
    return best;
}

float World::clearanceAt(float pe, float pn) const {
    float best = 1e9f;
    for (const auto& c : circles)
        best = std::min(best, std::hypot(pe - c.e, pn - c.n) - c.r);
    for (const auto& w : walls) {
        // point-to-segment distance
        const float se = w.e1 - w.e0, sn = w.n1 - w.n0;
        const float len2 = se * se + sn * sn;
        float t = (len2 > 1e-9f) ? ((pe - w.e0) * se + (pn - w.n0) * sn) / len2 : 0.f;
        t = std::max(0.f, std::min(1.f, t));
        const float ce = w.e0 + t * se, cn = w.n0 + t * sn;
        best = std::min(best, std::hypot(pe - ce, pn - cn));
    }
    return best;
}

void castScan(const World& w, float pe, float pn, float yawDeg,
              float hFovDeg, int n, float maxRange, std::vector<float>& ranges) {
    ranges.assign(n, maxRange);
    const float half = hFovDeg * 0.5f;
    for (int i = 0; i < n; ++i) {
        const float rel = (n == 1) ? 0.f : (-half + 2.f * half * i / (n - 1));
        ranges[i] = w.rayRange(pe, pn, yawDeg + rel, maxRange);
    }
}

cv::Mat renderFPV(const std::vector<float>& ranges, float maxRange, int w, int h) {
    cv::Mat img(h, w, CV_8UC3);
    // sky/ground gradient
    for (int y = 0; y < h; ++y) {
        const bool sky = y < h / 2;
        const float t = sky ? (float)y / (h / 2) : (float)(y - h / 2) / (h / 2);
        cv::Vec3b col = sky ? cv::Vec3b((uchar)(90 + 60 * t), (uchar)(70 + 50 * t), (uchar)(55 + 40 * t))
                            : cv::Vec3b((uchar)(45 - 20 * t), (uchar)(50 - 22 * t), (uchar)(55 - 24 * t));
        img.row(y).setTo(col);
    }
    const int n = (int)ranges.size();
    if (n == 0) return img;
    for (int x = 0; x < w; ++x) {
        const int ri = (n == 1) ? 0 : (int)((float)x * (n - 1) / (w - 1) + 0.5f);
        const float rng = std::max(0.05f, ranges[ri]);
        if (rng >= maxRange) continue;                       // open — no wall strip
        const float wallH = std::min((float)h, (h * 1.4f) / rng);   // ~1/range
        const int y0 = (int)((h - wallH) * 0.5f), y1 = (int)((h + wallH) * 0.5f);
        // shade darker with distance
        const float shade = std::max(0.15f, 1.f - rng / maxRange);
        const cv::Vec3b col((uchar)(70 * shade + 25), (uchar)(120 * shade + 25),
                            (uchar)(150 * shade + 25));
        for (int y = std::max(0, y0); y < std::min(h, y1); ++y) img.at<cv::Vec3b>(y, x) = col;
    }
    return img;
}

cv::Mat rangesToGrid(const std::vector<float>& ranges, int rows) {
    const int cols = (int)ranges.size();
    cv::Mat g(std::max(1, rows), std::max(1, cols), CV_32F);
    for (int r = 0; r < g.rows; ++r)
        for (int c = 0; c < cols; ++c)
            g.at<float>(r, c) = ranges[c] > 0.f ? ranges[c] : -1.f;
    return g;
}

cv::Mat renderTopDown(const World& w, float pe, float pn, float yawDeg,
                      float goalE, float goalN, float wpE, float wpN,
                      float planBearingDeg, bool planValid,
                      const std::vector<cv::Point2f>& trail,
                      const char* phase, int size, float spanM) {
    cv::Mat img(size, size, CV_8UC3, cv::Scalar(28, 28, 30));
    const float ppm = size / spanM;                       // pixels per metre
    const float cx = pe, cn = pn;                          // view centred on drone
    auto toPx = [&](float e, float n) {
        return cv::Point((int)(size / 2 + (e - cx) * ppm),
                         (int)(size / 2 - (n - cn) * ppm));   // N up
    };
    // grid
    for (float g = -spanM; g <= spanM; g += 5.f) {
        cv::line(img, toPx(cx + g, cn - spanM), toPx(cx + g, cn + spanM), {45, 45, 48}, 1);
        cv::line(img, toPx(cx - spanM, cn + g), toPx(cx + spanM, cn + g), {45, 45, 48}, 1);
    }
    // obstacles (truth)
    for (const auto& c : w.circles)
        cv::circle(img, toPx(c.e, c.n), (int)(c.r * ppm), {60, 90, 200}, -1, cv::LINE_AA);
    for (const auto& wl : w.walls)
        cv::line(img, toPx(wl.e0, wl.n0), toPx(wl.e1, wl.n1), {60, 90, 200}, 3, cv::LINE_AA);
    // flown trail
    for (size_t i = 1; i < trail.size(); ++i)
        cv::line(img, toPx(trail[i - 1].x, trail[i - 1].y), toPx(trail[i].x, trail[i].y),
                 {90, 200, 90}, 1, cv::LINE_AA);
    // goal + committed waypoint
    cv::drawMarker(img, toPx(goalE, goalN), {80, 255, 80}, cv::MARKER_STAR, 14, 2, cv::LINE_AA);
    cv::circle(img, toPx(wpE, wpN), 4, {80, 220, 255}, 1, cv::LINE_AA);
    // drone + heading, plus plan bearing
    const cv::Point d = toPx(pe, pn);
    const float yr = yawDeg * kPi / 180.f;
    cv::arrowedLine(img, d, cv::Point(d.x + (int)(20 * std::sin(yr)), d.y - (int)(20 * std::cos(yr))),
                    {255, 255, 255}, 2, cv::LINE_AA, 0, 0.3);
    if (planValid) {
        const float pr = planBearingDeg * kPi / 180.f;
        cv::arrowedLine(img, d, cv::Point(d.x + (int)(30 * std::sin(pr)), d.y - (int)(30 * std::cos(pr))),
                        {80, 220, 255}, 1, cv::LINE_AA, 0, 0.3);
    }
    cv::circle(img, d, 4, {255, 255, 255}, -1, cv::LINE_AA);
    if (phase) cv::putText(img, phase, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {230, 230, 230}, 1);
    return img;
}

}  // namespace sim
