#include "sim_world.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <queue>
#include <random>
#include <string>

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

// Given a filled occ bitmap, compute the clearance distance-transform, pick a
// roomy free START near the bottom-centre (origin placed so start = world (0,0)),
// and a reachable free GOAL farthest from it. Shared by the image loader and the
// procedural generators.
void finalizeOcc(World& w, float& startE, float& startN, float& goalE, float& goalN) {
    cv::Mat freeMask(w.oh, w.ow, CV_8U);
    for (int y=0;y<w.oh;++y) for (int x=0;x<w.ow;++x)
        freeMask.at<unsigned char>(y,x) = w.occ[(size_t)y*w.ow+x] ? 0 : 255;
    cv::Mat dist; cv::distanceTransform(freeMask, dist, cv::DIST_L2, 3);
    w.occDist.assign((size_t)w.ow*w.oh, 0.f);
    for (int y=0;y<w.oh;++y) for (int x=0;x<w.ow;++x)
        w.occDist[(size_t)y*w.ow+x] = dist.at<float>(y,x)*w.ocell;
    // start = the roomiest free cell near the bottom-centre (drone needs berth)
    int sx=w.ow/2, sy=1; float bestScore=-1e18f;
    for (int y=0;y<w.oh;++y) for (int x=0;x<w.ow;++x){
        if (w.occ[(size_t)y*w.ow+x]) continue;
        const float openM = w.occDist[(size_t)y*w.ow+x];
        if (openM < 1.6f) continue;
        const float score = openM*3.f - std::hypot((float)(x-w.ow/2),(float)y)*w.ocell;
        if (score>bestScore){ bestScore=score; sx=x; sy=y; }
    }
    w.oe0 = -sx*w.ocell; w.on0 = -sy*w.ocell;
    startE=0.f; startN=0.f;
    std::vector<int> dg((size_t)w.ow*w.oh,-1);
    std::queue<std::pair<int,int>> q; q.push({sx,sy}); dg[(size_t)sy*w.ow+sx]=0;
    int gx=sx, gy=sy, gmax=0; const int DX[4]={1,-1,0,0},DY[4]={0,0,1,-1};
    while(!q.empty()){ auto[cx,cy]=q.front(); q.pop(); int dd=dg[(size_t)cy*w.ow+cx];
        if(dd>gmax){gmax=dd;gx=cx;gy=cy;}
        for(int k=0;k<4;++k){int nx=cx+DX[k],ny=cy+DY[k];
            if(nx<0||ny<0||nx>=w.ow||ny>=w.oh)continue;
            if(w.occ[(size_t)ny*w.ow+nx]||dg[(size_t)ny*w.ow+nx]>=0)continue;
            dg[(size_t)ny*w.ow+nx]=dd+1; q.push({nx,ny});}}
    goalE=w.oe0+gx*w.ocell; goalN=w.on0+gy*w.ocell;
}

// Recursive-division maze into the occ bitmap: keep splitting each region with a
// wall that has ONE gap wide enough for the drone, until regions are corridor-
// sized. Always fully connected (every wall has a gap), so always solvable.
void mazeDivide(World& w, int N, int x0,int y0,int x1,int y1,
                int minCell,int gapCell, std::mt19937& rng) {
    const int wdt=x1-x0, hgt=y1-y0;
    if (wdt < 2*minCell && hgt < 2*minCell) return;
    const bool vert = (wdt > hgt);
    if (vert) {
        const int room = std::max(1, wdt-2*minCell);
        const int wx = x0+minCell + (int)(rng()%room);
        const int gr = std::max(1, hgt-gapCell);
        const int gy = y0 + (int)(rng()%gr);
        for (int y=y0;y<y1;++y) if (y<gy||y>=gy+gapCell) w.occ[(size_t)y*N+wx]=1;
        mazeDivide(w,N,x0,y0,wx,y1,minCell,gapCell,rng);
        mazeDivide(w,N,wx+1,y0,x1,y1,minCell,gapCell,rng);
    } else {
        const int room = std::max(1, hgt-2*minCell);
        const int wy = y0+minCell + (int)(rng()%room);
        const int gr = std::max(1, wdt-gapCell);
        const int gx = x0 + (int)(rng()%gr);
        for (int x=x0;x<x1;++x) if (x<gx||x>=gx+gapCell) w.occ[(size_t)wy*N+x]=1;
        mazeDivide(w,N,x0,y0,x1,wy,minCell,gapCell,rng);
        mazeDivide(w,N,x0,wy+1,x1,y1,minCell,gapCell,rng);
    }
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
    if (hasOcc()) {                                    // march the occupancy bitmap
        const float stepM = ocell * 0.5f;
        for (float m = 0.f; m < best; m += stepM) {
            const int cx = (int)((pe + de*m - oe0) / ocell);
            const int cy = (int)((pn + dn*m - on0) / ocell);
            if (cx<0 || cy<0 || cx>=ow || cy>=oh) break;   // off the map = open
            if (occ[(size_t)cy*ow+cx]) { best = std::min(best, m); break; }
        }
    }
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
    if (hasOcc() && !occDist.empty()) {                // precomputed distance transform
        const int cx = (int)((pe - oe0) / ocell), cy = (int)((pn - on0) / ocell);
        if (cx>=0 && cy>=0 && cx<ow && cy<oh) best = std::min(best, occDist[(size_t)cy*ow+cx]);
    }
    return best;
}

bool loadOccupancyImage(World& w, const std::string& path, float metersPerPixel,
                        float& startE, float& startN, float& goalE, float& goalN) {
    cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (img.empty()) return false;
    // Downscale very large maps so the sim stays fast (target ~200 px max side).
    const int maxSide = 220;
    if (std::max(img.cols, img.rows) > maxSide) {
        const float s = (float)maxSide / std::max(img.cols, img.rows);
        cv::resize(img, img, {}, s, s, cv::INTER_AREA);
        metersPerPixel /= s;                           // keep real-world scale
    }
    w.ow = img.cols; w.oh = img.rows; w.ocell = metersPerPixel;
    w.occ.assign((size_t)w.ow * w.oh, 0);
    for (int y = 0; y < img.rows; ++y)
        for (int x = 0; x < img.cols; ++x) {
            const bool solid = img.at<unsigned char>(y, x) < 128;   // dark = obstacle
            const int gy = img.rows - 1 - y;                        // flip: +n up
            w.occ[(size_t)gy*w.ow + x] = solid ? 1 : 0;
        }
    finalizeOcc(w, startE, startN, goalE, goalN);
    return true;
}

bool genMaze(World& w, float worldSizeM, float ocellM, unsigned seed,
             float& startE, float& startN, float& goalE, float& goalN) {
    std::mt19937 rng(seed ? seed : 1u);
    const int N = std::max(24, (int)(worldSizeM / ocellM));
    w.ow = N; w.oh = N; w.ocell = ocellM;
    w.occ.assign((size_t)N*N, 0);
    for (int x=0;x<N;++x){ w.occ[x]=1; w.occ[(size_t)(N-1)*N+x]=1; }   // border
    for (int y=0;y<N;++y){ w.occ[(size_t)y*N]=1; w.occ[(size_t)y*N+N-1]=1; }
    const int minCell = std::max(8, (int)(4.0f/ocellM));   // corridors ~4 m
    const int gapCell = std::max(8, (int)(4.0f/ocellM));   // gaps ~4 m (berth fits)
    mazeDivide(w, N, 1, 1, N-1, N-1, minCell, gapCell, rng);
    finalizeOcc(w, startE, startN, goalE, goalN);
    return true;
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
