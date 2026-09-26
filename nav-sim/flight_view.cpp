#include "flight_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <opencv2/imgproc.hpp>

#include "footage.hpp"
#include "voxel_map.hpp"

namespace kshow {

namespace {

constexpr float kPi = 3.14159265358979f;

// BGR. The trail is warm and the leg is cool so the two never read as one.
const cv::Scalar kTrail{40, 190, 255}, kLeg{255, 215, 70}, kInk{242, 240, 238};
const cv::Scalar kDim{170, 162, 156}, kShadowInk{0, 0, 0};

float wrap180(float d) {
    while (d > 180.f) d -= 360.f;
    while (d <= -180.f) d += 360.f;
    return d;
}

void label(cv::Mat& im, const std::string& s, cv::Point at, double sc,
           const cv::Scalar& c, int th = 1) {
    cv::putText(im, s, at, cv::FONT_HERSHEY_SIMPLEX, sc, kShadowInk, th + 2, cv::LINE_AA);
    cv::putText(im, s, at, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
}

cv::Scalar phaseColour(const std::string& ph) {
    if (ph == "MOVE")  return {110, 215, 110};
    if (ph == "THINK") return {70, 200, 250};
    if (ph == "SCAN")  return {230, 130, 220};
    if (ph == "STUCK") return {70, 70, 230};
    return {200, 180, 160};                       // SETTLE, ARRIVE, ARMED
}

// What each phase MEANS, in the words a viewer needs -- the mission's own
// states, not a narration invented for the screen.
const char* phaseLine(const std::string& ph) {
    if (ph == "SETTLE") return "SETTLE  hover until still";
    if (ph == "THINK")  return "THINK  map this stop, certify a leg";
    if (ph == "SCAN")   return "SCAN  nothing certified ahead: turn and look";
    if (ph == "MOVE")   return "MOVE  fly the certified leg";
    if (ph == "ARRIVE") return "ARRIVE  leg done, stop";
    if (ph == "STUCK")  return "STUCK  no way out found";
    return ph.c_str();
}

// A world point into a view rendered from `cam` at w x h, hfov `fov`. The
// renderers' own projection (voxel_map.hpp), so overlays land on geometry.
bool project(const sim::CamPose& cam, int w, int h, float fov,
             float x, float y, float z, cv::Point2f& out) {
    float u, v;
    const bool in = sim::VoxelMap::fpvProject(cam.e, cam.n, cam.u, cam.yawDeg, cam.pitchDeg,
                                              w, h, fov, x, y, z, u, v);
    out = {u, v};
    // Behind the camera is refused; off-image is kept so lines can clip.
    if (in) return true;
    const float yr = cam.yawDeg * kPi / 180.f;
    const float fwd = (x - cam.e) * std::sin(yr) + (y - cam.n) * std::cos(yr);
    return fwd > 0.2f && std::isfinite(u) && std::isfinite(v) &&
           std::fabs(u) < 20.f * w && std::fabs(v) < 20.f * h;
}

// Is (x,y,z) visible from the camera, or behind a wall of the TRUE world?
bool visible(const sim::VoxelWorld& w, const sim::CamPose& cam, float x, float y, float z) {
    const float dx = x - cam.e, dy = y - cam.n, dz = z - cam.u;
    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d < 0.05f) return true;
    const float t = w.raycast(cam.e, cam.n, cam.u, dx / d, dy / d, dz / d, d, nullptr);
    return t >= d - 0.3f;
}

// THE AIRFRAME, drawn: an X quad, 0.44 m motor to motor, nose marked. Its
// shadow goes on the floor straight below, which is what makes its height
// readable in a still frame.
void drawDrone(cv::Mat& im, const sim::CamPose& cam, float fov, const sim::CamPose& a,
               float floorZ, bool shadow) {
    const int w = im.cols, h = im.rows;
    const float yr = a.yawDeg * kPi / 180.f;
    const float fe = std::sin(yr), fn = std::cos(yr);     // forward
    const float re = std::cos(yr), rn = -std::sin(yr);    // right
    const float arm = 0.22f, rotor = 0.13f;
    auto at = [&](float f, float r, float z, cv::Point2f& p) {
        return project(cam, w, h, fov, a.e + fe * f + re * r, a.n + fn * f + rn * r, z, p);
    };
    const float mot[4][2] = {{arm, arm}, {arm, -arm}, {-arm, -arm}, {-arm, arm}};
    if (shadow) {
        std::vector<cv::Point> poly;
        for (int i = 0; i < 16; ++i) {
            const float t = 2.f * kPi * float(i) / 16.f;
            cv::Point2f p;
            if (at(0.42f * std::cos(t), 0.42f * std::sin(t), floorZ + 0.02f, p))
                poly.push_back(p);
        }
        if (poly.size() >= 3) {
            cv::Mat over = im.clone();
            cv::fillConvexPoly(over, poly, {10, 10, 10}, cv::LINE_AA);
            cv::addWeighted(over, 0.35, im, 0.65, 0.0, im);
        }
    }
    cv::Point2f c;
    if (!at(0.f, 0.f, a.u, c)) return;
    for (const auto& m : mot) {                            // arms
        cv::Point2f p;
        if (at(m[0], m[1], a.u, p)) cv::line(im, c, p, {45, 45, 50}, 3, cv::LINE_AA);
    }
    for (int k = 0; k < 4; ++k) {                          // rotors: a disc each
        std::vector<cv::Point> poly;
        for (int i = 0; i < 14; ++i) {
            const float t = 2.f * kPi * float(i) / 14.f;
            cv::Point2f p;
            if (at(mot[k][0] + rotor * std::cos(t), mot[k][1] + rotor * std::sin(t),
                   a.u + 0.03f, p))
                poly.push_back(p);
        }
        if (poly.size() >= 3) {
            cv::Mat over = im.clone();
            cv::fillConvexPoly(over, poly, k < 2 ? cv::Scalar(90, 90, 235)
                                                 : cv::Scalar(230, 230, 230), cv::LINE_AA);
            cv::addWeighted(over, 0.45, im, 0.55, 0.0, im);
            cv::polylines(im, poly, true, {250, 250, 250}, 1, cv::LINE_AA);
        }
    }
    std::vector<cv::Point> body;                           // the hull
    for (const auto& q : {std::array<float, 2>{0.10f, 0.f}, {0.f, 0.07f},
                          {-0.09f, 0.f}, {0.f, -0.07f}}) {
        cv::Point2f p;
        if (at(q[0], q[1], a.u + 0.02f, p)) body.push_back(p);
    }
    if (body.size() == 4) cv::fillConvexPoly(im, body, {30, 30, 34}, cv::LINE_AA);
}

// A polyline through world points, skipping the parts a wall hides.
void drawPath3d(cv::Mat& im, const sim::VoxelWorld* world, const sim::CamPose& cam, float fov,
                const std::vector<cv::Point3f>& pts, const cv::Scalar& col, int th,
                bool fadeOld) {
    cv::Point2f prev;
    bool have = false;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const cv::Point3f& q = pts[i];
        cv::Point2f p;
        const bool ok = project(cam, im.cols, im.rows, fov, q.x, q.y, q.z, p) &&
                        (!world || visible(*world, cam, q.x, q.y, q.z));
        if (ok && have) {
            cv::Scalar c = col;
            if (fadeOld) {
                const float age = float(n - 1 - i) / float(std::max<size_t>(1, n));
                c = col * (1.f - 0.6f * age);
            }
            cv::line(im, prev, p, c, th, cv::LINE_AA);
        }
        prev = p;
        have = ok;
    }
}

}  // namespace

// ------------------------------------------------------------------ ribbons
void drawRibbon(cv::Mat& im, const sim::CamPose& eye, float hfov, const cv::Mat& hitDist,
                const std::vector<std::array<float, 3>>& path, const cv::Scalar& col,
                float halfW, double alpha, bool arrow, float drop) {
    // IN 10 cm PIECES: a quad with a corner behind the camera cannot be
    // projected and is dropped whole, so a leg drawn as one long quad vanished
    // the moment the aircraft started along it. Short pieces lose only what is
    // really behind the eye.
    std::vector<std::array<float, 3>> pts;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const auto& a = path[i];
        const auto& b = path[i + 1];
        const float len = std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]) +
                                    (b[2] - a[2]) * (b[2] - a[2]));
        const int n = std::max(1, int(std::ceil(len / 0.1f)));
        for (int k = 0; k < n; ++k) {
            const float t = float(k) / float(n);
            pts.push_back({a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
                           a[2] + (b[2] - a[2]) * t});
        }
    }
    if (!path.empty()) pts.push_back(path.back());
    auto proj = [&](float x, float y, float z, cv::Point2f& o) {
        float u = std::numeric_limits<float>::quiet_NaN(), v = u;
        sim::VoxelMap::fpvProject(eye.e, eye.n, eye.u, eye.yawDeg, eye.pitchDeg, im.cols,
                                  im.rows, hfov, x, y, z, u, v);
        o = {u, v};
        return std::isfinite(u) && std::isfinite(v) && std::fabs(u) < 8.f * im.cols &&
               std::fabs(v) < 8.f * im.rows;
    };
    cv::Mat mask(im.size(), CV_8U, cv::Scalar(0));
    auto quad = [&](const cv::Point3f (&c)[4], double band) {
        std::vector<cv::Point> poly;
        for (const cv::Point3f& q : c) {
            cv::Point2f p;
            if (!proj(q.x, q.y, q.z, p)) return;
            poly.push_back(cv::Point(int(std::lround(p.x)), int(std::lround(p.y))));
        }
        const cv::Rect box = cv::boundingRect(poly) & cv::Rect(0, 0, im.cols, im.rows);
        if (box.area() <= 0) return;
        const cv::Point3f mid = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
        const float dq = std::sqrt((mid.x - eye.e) * (mid.x - eye.e) +
                                   (mid.y - eye.n) * (mid.y - eye.n) +
                                   (mid.z - eye.u) * (mid.z - eye.u));
        mask(box).setTo(0);
        cv::fillConvexPoly(mask, poly, cv::Scalar(255), cv::LINE_AA);
        const double shade = (1.0 - 0.45 * std::min(1.f, dq / 6.f)) * band;
        for (int v = box.y; v < box.y + box.height; ++v) {
            const uchar* m = mask.ptr<uchar>(v);
            const float* hd = hitDist.empty() ? nullptr : hitDist.ptr<float>(v);
            cv::Vec3b* px = im.ptr<cv::Vec3b>(v);
            for (int u = box.x; u < box.x + box.width; ++u) {
                if (!m[u]) continue;
                // Behind a voxel -- one clearly nearer, not the surface the
                // ribbon lies on or beside.
                if (hd && hd[u] > 0.f && hd[u] < dq - 0.15f) continue;
                const double a = alpha * m[u] / 255.0;
                for (int k = 0; k < 3; ++k)
                    px[u][k] = cv::saturate_cast<uchar>(px[u][k] * (1.0 - a) +
                                                        col[k] * shade * a);
            }
        }
    };
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto& a = pts[i];
        const auto& b = pts[i + 1];
        const float dx = b[0] - a[0], dy = b[1] - a[1];
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-3f) continue;
        const float nx = -dy / len * halfW, ny = dx / len * halfW;
        const cv::Point3f c[4] = {{a[0] + nx, a[1] + ny, a[2] - drop},
                                  {a[0] - nx, a[1] - ny, a[2] - drop},
                                  {b[0] - nx, b[1] - ny, b[2] - drop},
                                  {b[0] + nx, b[1] + ny, b[2] - drop}};
        // BANDS every 30 cm, a shade apart, like road markings: something
        // for the eye to count into the distance, which is most of what makes
        // a flat colour read as lying IN the scene.
        quad(c, ((i / 3) % 2) ? 0.80 : 1.0);
    }
    if (!arrow || pts.size() < 2) return;
    const auto& a = pts[pts.size() - 2];
    const auto& b = pts.back();
    const float dx = b[0] - a[0], dy = b[1] - a[1];
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-3f) return;
    // The head scales with the ribbon: twice its width across, four long.
    const float fx = dx / len, fy = dy / len, z = b[2] - drop, w = halfW * 2.f;
    const float tip = halfW * 4.f;
    const cv::Point3f c[4] = {{b[0] - fy * w, b[1] + fx * w, z},
                              {b[0] + fy * w, b[1] - fx * w, z},
                              {b[0] + fx * tip, b[1] + fy * tip, z},
                              {b[0] + fx * tip, b[1] + fy * tip, z}};
    quad(c, 1.0);
}

// ------------------------------------------------------------------ the fan
std::vector<Ray> legFan(const FlightShow& f) {
    const sim::NavPipeline& nav = f.module().pipeline();
    std::vector<Ray> fan;
    if (nav.frames() == 0) return fan;
    sim::CamPose from;                           // the vantage: origin of its map
    from.yawDeg = f.truth().yawDeg;
    const float half = f.camera().params().hfovDeg * 0.5f - 8.f;
    const float legMax = std::max(MissionController::Params().stepM +
                                  MissionController::Params().voxStopMarginM, 1.f);
    for (float o = -half; o <= half + 1e-3f; o += 3.f) {
        Ray r;
        r.bearingDeg = f.truth().yawDeg + o;
        r.freeM = nav.straightFreeM(from, r.bearingDeg, legMax);
        fan.push_back(r);
    }
    return fan;
}

// ----------------------------------------------------------------- snapshot
ShowSnap ShowSnap::of(const FlightShow& f, const ShowSnap* prev) {
    ShowSnap s;
    s.world = f.worldPtr();
    const sim::NavPipeline& nav = f.module().pipeline();
    s.mapFrames = nav.frames();
    s.mapKey = long(f.module().resets()) * 100000L + s.mapFrames;
    // The map is copied only when it has changed: a new frame folded in, or a
    // new stop begun. Otherwise the previous copy is shared.
    if (prev && prev->map && prev->mapKey == s.mapKey && prev->world == s.world) {
        s.map = prev->map;
        s.field = prev->field;
    } else if (s.mapFrames > 0) {
        s.map = std::make_shared<const sim::VoxelMap>(nav.map());
        s.field = std::make_shared<const sim::BearingField>(nav.field());
    }
    s.maxIntegM = nav.mapParams().maxIntegM;
    s.farRangeM = nav.params().farRangeM;
    s.cam = f.camera().params();
    s.floorZ = f.floorZ(); s.speed = f.speed();
    s.truth = f.truth(); s.vantage = f.vantage();
    s.phase = f.phase();
    s.worldName = f.params().world;
    s.stats = f.stats();
    s.trail = f.trail();
    s.legs = f.legs();
    s.depth = f.lastDepth();          // a new Mat every frame; sharing is safe
    s.depthSeq = f.depthFrames();
    // THE FAN is the module's search at THIS stop: recomputed while the
    // aircraft is choosing, kept through the leg it chose, gone with the map.
    if (s.phase == "THINK" || s.phase == "SCAN") s.fan = legFan(f);
    else if (prev && prev->world == s.world && prev->mapKey / 100000L == s.mapKey / 100000L)
        s.fan = prev->fan;
    return s;
}

sim::CamPose FlightView::chasePose(const ShowSnap& s) const {
    const sim::CamPose& a = s.truth;
    const float up = 2.1f;
    const float yr = camYaw_ * kPi / 180.f;
    sim::CamPose c;
    c.e = a.e - std::sin(yr) * boomM_;
    c.n = a.n - std::cos(yr) * boomM_;
    c.u = a.u + up;
    c.yawDeg = camYaw_;
    // Aim a little ahead of the aircraft, so the leg in front of it is in frame.
    c.pitchDeg = -std::atan2(up, boomM_ + 2.5f) * 180.f / kPi;
    return c;
}

void FlightView::update(const ShowSnap& s) {
    // THE PLAN of the true world, once per world: a slab 0.5 m either side of
    // flight altitude, so a lintel or a low block shows as the wall it is to
    // the aircraft. 1 px per cell.
    if (planOf_ != s.world.get()) {
        const sim::VoxelWorld& w = *s.world;
        const int nx = w.nx(), ny = w.ny();
        plan_ = cv::Mat(ny, nx, CV_8U, cv::Scalar(0));
        int z0, z1, dummy;
        w.worldToCell(0.f, 0.f, s.truth.u - 0.5f, dummy, dummy, z0);
        w.worldToCell(0.f, 0.f, s.truth.u + 0.5f, dummy, dummy, z1);
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                for (int z = z0; z <= z1; ++z)
                    if (w.solid(x, y, z)) { plan_.at<uchar>(ny - 1 - y, x) = 255; break; }
        planCell_ = w.cell();
        planOf_ = s.world.get();
        camYaw_ = s.truth.yawDeg;           // a new world: the camera starts behind
        lastT_ = -1.0;
    }
    // WHAT THE MAP HOLDS NEAR THE VANTAGE, listed once per map change rather
    // than scanned per picture: 5.5 M cells, of which everything a stop can
    // have marked lies within its few metres of honest range. +-10 m across,
    // +-4 m up and down, in the map's own frame.
    if (!s.map) {
        cacheKey_ = -1; free_.clear(); occ_.clear();
    } else if (s.mapKey != cacheKey_) {
        cacheKey_ = s.mapKey;
        free_.clear(); occ_.clear();
        const sim::VoxelMap& map = *s.map;
        const sim::VoxelMapParams& mp = map.params();
        int cx, cy, cz;
        map.worldToCell(0.f, 0.f, 0.f, cx, cy, cz);
        const int R = int(10.f / mp.cell), Rz = int(4.f / mp.cell);
        for (int y = cy - R; y <= cy + R; ++y)
            for (int x = cx - R; x <= cx + R; ++x) {
                if (!map.inBounds(x, y, cz)) continue;
                float wx, wy, wz;
                map.cellCentre(x, y, cz, wx, wy, wz);
                if (map.stateAt(wx, wy, wz) == sim::VoxelMap::FREE) free_.push_back({wx, wy});
                for (int z = std::max(0, cz - Rz); z <= std::min(mp.nz - 1, cz + Rz); ++z) {
                    map.cellCentre(x, y, z, wx, wy, wz);
                    if (map.stateAt(wx, wy, wz) == sim::VoxelMap::OCCUPIED)
                        occ_.push_back({wx, wy, wz});
                }
            }
    }
    const double t = s.stats.timeS;
    const float dt = lastT_ < 0 ? 10.f : float(t - lastT_);
    lastT_ = t;
    // THE CHASE CAMERA follows the heading with a lag (0.9 s), so a turn in
    // place reads as a turn instead of the world spinning round the aircraft.
    const float k = 1.f - std::exp(-std::max(0.f, dt) / 0.9f);
    camYaw_ += wrap180(s.truth.yawDeg - camYaw_) * k;
    // THE BOOM never goes through a wall: shorten it to what is clear behind
    // the aircraft, as a game camera does. Otherwise a stop beside a wall is
    // filmed from inside the wall.
    const sim::CamPose& a = s.truth;
    const float yr = camYaw_ * kPi / 180.f;
    float dx = -std::sin(yr) * 4.5f, dy = -std::cos(yr) * 4.5f, dz = 2.1f;
    const float L = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= L; dy /= L; dz /= L;
    const float hit = (*s.world).raycast(a.e, a.n, a.u, dx, dy, dz, L + 0.5f, nullptr);
    const float want = std::max(1.2f, std::min(4.5f, (hit - 0.5f) * 4.5f / L));
    boomM_ += (want - boomM_) * std::min(1.f, k * 3.f);

}

// THE SHOWCASE'S LOOK for the true scene: pale plaster walls in panels, a
// concrete floor ruled on the world's 2 m lattice, a soft contact shadow where
// the two meet. Colour only; renderFootage's geometry and light are unchanged.
static sim::FootageStyle showStyle() {
    sim::FootageStyle s;
    s.skyTop  = {0.86f, 0.58f, 0.30f}; s.skyHor = {0.95f, 0.86f, 0.74f};
    s.groundA = {0.33f, 0.34f, 0.35f}; s.groundB = {0.36f, 0.37f, 0.38f};
    s.warmA   = {0.60f, 0.72f, 0.84f}; s.warmB  = {0.64f, 0.76f, 0.87f};
    s.wallA   = {0.62f, 0.70f, 0.78f}; s.wallB  = {0.66f, 0.73f, 0.80f};
    s.hazeK = 0.5f;
    s.gridM = 2.f; s.gridDark = 0.25f;
    s.panelM = 2.f; s.panelTint = 0.10f;
    s.contactM = 0.8f;
    return s;
}

// -------------------------------------------------------------------- chase
cv::Mat FlightView::chase(const ShowSnap& s, int w, int h) const {
    // THE LENS FOLLOWS THE PICTURE: a fixed vertical field (36 deg), so a wide
    // pane shows more to either side rather than cropping the floor and the
    // aircraft off the bottom.
    const float fov = 2.f * std::atan(std::tan(18.f * kPi / 180.f) * float(w) / float(h)) *
                      180.f / kPi;
    const sim::CamPose cam = chasePose(s);
    // Cast at 3/4 of the pane and scaled up: 56 % of the rays, and the
    // overlays are drawn after, at full resolution, through the same
    // projection (focal length and centre scale with the image).
    cv::Mat im;
    cv::resize(sim::renderFootage((*s.world), cam, w * 3 / 4, h * 3 / 4, fov, 70.f,
                                  s.floorZ + 0.3f, showStyle()),
               im, {w, h}, 0, 0, cv::INTER_LINEAR);
    // WHAT THIS STOP'S MAP KNOWS, laid over the true scene: air it has
    // CONFIRMED free at flight height as a blue sheet, and every cell it has
    // marked OCCUPIED as a red point. Everything untinted is unknown to the
    // aircraft -- it is only visible here because this is the true scene.
    const sim::CamPose& v = s.vantage;
    if (s.mapFrames > 0 && s.map && cacheKey_ == s.mapKey) {
        const float c = s.map->params().cell;
        cv::Mat sheet = im.clone();
        bool any = false;
        for (const cv::Point2f& q0 : free_) {
            // Hidden behind a wall from here? Then it is not drawn over it.
            if (!visible((*s.world), cam, v.e + q0.x, v.n + q0.y, s.truth.u)) continue;
            std::vector<cv::Point> q;
            for (const auto& d : {std::array<float, 2>{-0.5f, -0.5f}, {0.5f, -0.5f},
                                  {0.5f, 0.5f}, {-0.5f, 0.5f}}) {
                cv::Point2f p;
                if (project(cam, w, h, fov, v.e + q0.x + d[0] * c, v.n + q0.y + d[1] * c,
                            s.truth.u, p))
                    q.push_back(p);
            }
            if (q.size() == 4) { cv::fillConvexPoly(sheet, q, {235, 170, 90}); any = true; }
        }
        if (any) cv::addWeighted(sheet, 0.35, im, 0.65, 0.0, im);
        for (const cv::Point3f& o : occ_) {
            const float ex = v.e + o.x, ny = v.n + o.y, uz = v.u + o.z;
            cv::Point2f p;
            if (!project(cam, w, h, fov, ex, ny, uz, p)) continue;
            if (p.x < 0 || p.y < 0 || p.x >= w || p.y >= h) continue;
            if (!visible((*s.world), cam, ex, ny, uz)) continue;
            cv::circle(im, p, 2, {60, 60, 235}, cv::FILLED, cv::LINE_AA);
        }
    }
    // THE LEG FAN while the aircraft is choosing: each bearing as far as it
    // was certified, at flight height, short red to long green.
    // Kept, dimmed, through the leg it produced: THINK lasts 0.3 s, and the
    // point -- the leg flown is the longest ray of this fan -- needs longer.
    const std::string phNow = s.phase;
    const bool choosing = phNow == "THINK" || phNow == "SCAN";
    if (choosing || phNow == "MOVE") {
        const float legMax = std::max(MissionController::Params().stepM +
                                      MissionController::Params().voxStopMarginM, 1.f);
        for (const Ray& r : s.fan) {
            const float br = r.bearingDeg * kPi / 180.f;
            const float q = std::min(1.f, r.freeM / legMax);
            std::vector<cv::Point3f> seg{{v.e, v.n, s.truth.u},
                                         {v.e + std::sin(br) * r.freeM,
                                          v.n + std::cos(br) * r.freeM, s.truth.u}};
            const float k = choosing ? 1.f : 0.55f;
            drawPath3d(im, nullptr, cam, fov, seg,
                       cv::Scalar(60, 60 + 170 * q, 230 - 180 * q) * k, 1, false);
        }
    }
    // Every leg flown so far, faint; the trail, warm and fading with age.
    const std::vector<cv::Point3f>& tr = s.trail;
    const size_t keep = 900;                              // ~90 m of trail
    std::vector<cv::Point3f> recent(tr.size() > keep ? tr.end() - keep : tr.begin(), tr.end());
    recent.push_back({s.truth.e, s.truth.n, s.truth.u});
    drawPath3d(im, s.world.get(), cam, fov, recent, kTrail, 2, true);
    // THE LEG BEING FLOWN (or just certified): a line at flight altitude to
    // the waypoint, and a ring where the aircraft will stop.
    const std::string ph = s.phase;
    if (!s.legs.empty() && ph == "MOVE") {
        const Leg& l = s.legs.back();
        const float br = l.bearingDeg * kPi / 180.f;
        const float ee = l.e0 + std::sin(br) * l.lengthM, ne = l.n0 + std::cos(br) * l.lengthM;
        std::vector<cv::Point3f> seg;
        for (int i = 0; i <= 12; ++i) {
            const float q = float(i) / 12.f;
            seg.push_back({l.e0 + (ee - l.e0) * q, l.n0 + (ne - l.n0) * q, s.truth.u});
        }
        drawPath3d(im, s.world.get(), cam, fov, seg, kLeg, 2, false);
        std::vector<cv::Point3f> ring;
        for (int i = 0; i <= 20; ++i) {
            const float t = 2.f * kPi * float(i) / 20.f;
            ring.push_back({ee + 0.35f * std::cos(t), ne + 0.35f * std::sin(t), s.truth.u});
        }
        drawPath3d(im, s.world.get(), cam, fov, ring, kLeg, 2, false);
    }
    drawDrone(im, cam, fov, s.truth, s.floorZ, true);

    // HUD: the mission's phase, what it means, and the airframe's numbers.
    const cv::Scalar pc = phaseColour(ph);
    cv::rectangle(im, {10, 10, 7, 36}, pc, cv::FILLED);
    label(im, phaseLine(ph), {24, 28}, 0.62, pc, 2);
    label(im, cv::format("%.1f m/s   alt %.1f m   hdg %03.0f   t %02d:%02d", s.speed,
                         s.truth.u - s.floorZ, s.truth.yawDeg,
                         int(s.stats.timeS) / 60, int(s.stats.timeS) % 60),
          {24, 46}, 0.45, kInk, 1);
    label(im, "SIMULATED SCENE -- the aircraft never sees this", {10, h - 12}, 0.42, kDim, 1);
    return im;
}

// ------------------------------------------------------------------- camera
cv::Mat FlightView::camera(const ShowSnap& s, int w, int h) const {
    // The left imager at 424x240 -- a real D435i IR mode -- scaled to the pane.
    sim::CamParams cp = s.cam;
    cp.width = 424; cp.height = 240;
    const sim::DepthCamera cam(cp);
    const cv::Mat ir = cam.renderIR((*s.world), s.truth);
    cv::Mat bgr, out;
    cv::cvtColor(ir, bgr, cv::COLOR_GRAY2BGR);
    cv::resize(bgr, out, {w, h}, 0, 0, cv::INTER_LINEAR);
    // THE LEG, where the aircraft will fly it, drawn onto the FLOOR below the
    // flight line (a line at camera height projects onto the horizon).
    const std::string ph = s.phase;
    if (!s.legs.empty() && ph == "MOVE") {
        const Leg& l = s.legs.back();
        const float br = l.bearingDeg * kPi / 180.f;
        std::vector<cv::Point3f> seg;
        const float z = s.floorZ + 0.02f;
        for (int i = 0; i <= 16; ++i) {
            const float q = l.lengthM * float(i) / 16.f;
            seg.push_back({l.e0 + std::sin(br) * q, l.n0 + std::cos(br) * q, z});
        }
        drawPath3d(out, s.world.get(), s.truth, cp.hfovDeg, seg, kLeg, 3, false);
    }
    label(out, "D435i left IR, 87 deg", {10, 22}, 0.5, kInk, 1);
    return out;
}

// -------------------------------------------------------------------- depth
cv::Mat FlightView::depth(const ShowSnap& s, int w, int h) const {
    const cv::Mat& d = s.depth;
    // Only a NEW frame is worth drawing again (a frame arrives at most every
    // tick, and during a leg every sixth).
    if (!d.empty() && s.depthSeq == depthOf_ && depthPane_.cols == w && depthPane_.rows == h)
        return depthPane_.clone();
    cv::Mat out(h, w, CV_8UC3, cv::Scalar(30, 27, 24));
    if (d.empty()) return out;
    // RED NEAR, BLUE FAR, on a FIXED scale -- a colour means a distance.
    // Holes are grey: unmeasured, not far (depth_vis.hpp, CLAUDE.md).
    const float maxM = 8.f;
    cv::Mat idx(d.size(), CV_8U), col;
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        uchar* o = idx.ptr<uchar>(y);
        for (int x = 0; x < d.cols; ++x)
            o[x] = r[x] > 0.f ? cv::saturate_cast<uchar>(255.f * (1.f - std::min(r[x], maxM) / maxM))
                              : 0;
    }
    cv::applyColorMap(idx, col, cv::COLORMAP_TURBO);
    int valid = 0;
    for (int y = 0; y < d.rows; ++y) {
        const float* r = d.ptr<float>(y);
        cv::Vec3b* c = col.ptr<cv::Vec3b>(y);
        for (int x = 0; x < d.cols; ++x) {
            if (r[x] > 0.f) ++valid;
            else c[x] = cv::Vec3b(72, 72, 72);
        }
    }
    // AREA when shrinking: nearest-neighbour turns a 0.4 % speckle into
    // confetti at a third of the size. Nearest when enlarging, so a hole
    // stays a hole.
    cv::resize(col, out, {w, h}, 0, 0,
               w < col.cols ? cv::INTER_AREA : cv::INTER_NEAREST);
    // The scale, on the image: a colour is a distance.
    const int bx = w - 190, by = h - 30, bw = 170, bh = 10;
    for (int i = 0; i < bw; ++i) {
        cv::Mat one(1, 1, CV_8U, cv::Scalar(uchar(255.f * (1.f - float(i) / bw)))), c;
        cv::applyColorMap(one, c, cv::COLORMAP_TURBO);
        const cv::Vec3b v = c.at<cv::Vec3b>(0, 0);
        cv::line(out, {bx + i, by}, {bx + i, by + bh}, cv::Scalar(v[0], v[1], v[2]));
    }
    cv::rectangle(out, {bx - 1, by - 1, bw + 2, bh + 2}, kInk, 1);
    label(out, "0", {bx - 4, by + bh + 14}, 0.4, kInk, 1);
    label(out, "4 m", {bx + bw / 2 - 12, by + bh + 14}, 0.4, kInk, 1);
    label(out, "8 m+", {bx + bw - 20, by + bh + 14}, 0.4, kInk, 1);
    label(out, cv::format("%.0f%% of pixels matched   grey = no match",
                          100.0 * valid / std::max(1, d.rows * d.cols)),
          {10, 22}, 0.45, kInk, 1);
    depthOf_ = s.depthSeq;
    depthPane_ = out.clone();
    return out;
}

// ---------------------------------------------------------------------- fpv
cv::Mat FlightView::fpv(const ShowSnap& s, int w, int h) const {
    const float fov = s.cam.hfovDeg;
    // THE CAMERA'S OWN IMAGE BEHIND THE VOXELS, as the live pane draws it: the
    // D435i's left IR (simulated here), the imager depth is computed in, so
    // every voxel lands on what it came from. Where nothing is mapped the pane
    // shows what the camera sees -- not a claim that it is empty.
    sim::CamParams cp = s.cam;
    cp.width = w; cp.height = h;
    const sim::DepthCamera cam(cp);
    cv::Mat view;
    cv::cvtColor(cam.renderIR(*s.world, s.truth), view, cv::COLOR_GRAY2BGR);
    view *= 0.85;
    if (s.mapFrames == 0 || !s.map) {
        label(view, "no map yet", {10, 22}, 0.5, kInk, 1);
        return view;
    }
    const sim::CamPose& v = s.vantage;
    sim::CamPose eye = s.truth;                    // the camera, in the map's frame
    eye.e -= v.e; eye.n -= v.n; eye.u -= v.u;
    std::vector<sim::VoxelMap::Layer> layers(1);
    layers[0].map = s.map.get();
    layers[0].minRange = 0.f;
    layers[0].range = s.maxIntegM > 0.f ? s.maxIntegM : 4.f;
    cv::Mat hitMask, hitDist;
    const cv::Mat vox = sim::VoxelMap::renderLadder(layers, eye.e, eye.n, eye.u, eye.yawDeg,
                                                    eye.pitchDeg, w, h, fov, sim::FpvStyle(),
                                                    &hitMask, &hitDist);
    vox.copyTo(view, hitMask);
    // THE LEG FAN, laid under the flight line: every bearing the module
    // certified, as far as it certified it, short red to long green; the leg it
    // chose (or is flying) wide and blue with its arrowhead. 0.6 m below the
    // eye: a level D435i sees the floor only from 2.7 m ahead, and the legs are
    // 2-3 m long, so on the floor they would be out of the picture.
    const float drop = std::min(0.6f, s.truth.u - s.floorZ);
    const float legMax = std::max(MissionController::Params().stepM +
                                  MissionController::Params().voxStopMarginM, 1.f);
    const Ray* best = nullptr;
    const bool choosing = s.phase == "THINK" || s.phase == "SCAN";
    for (const Ray& r : s.fan) {
        if (!best || r.freeM > best->freeM) best = &r;
        if (!choosing) continue;            // while flying, the leg alone
        const float br = r.bearingDeg * kPi / 180.f;
        const float q = std::min(1.f, r.freeM / legMax);
        const std::vector<std::array<float, 3>> path{
            {0.f, 0.f, 0.f}, {std::sin(br) * r.freeM, std::cos(br) * r.freeM, 0.f}};
        drawRibbon(view, eye, fov, hitDist, path, cv::Scalar(60, 60 + 170 * q, 230 - 180 * q),
                   0.03f, 0.6, false, drop);
    }
    std::vector<std::array<float, 3>> leg;
    if (s.phase == "MOVE" && !s.legs.empty()) {
        const Leg& l = s.legs.back();
        const float br = l.bearingDeg * kPi / 180.f;
        const float e0 = l.e0 - v.e, n0 = l.n0 - v.n;
        leg = {{e0, n0, 0.f}, {e0 + std::sin(br) * l.lengthM, n0 + std::cos(br) * l.lengthM, 0.f}};
    } else if (best && best->freeM > 0.5f) {
        const float br = best->bearingDeg * kPi / 180.f;
        leg = {{0.f, 0.f, 0.f}, {std::sin(br) * best->freeM, std::cos(br) * best->freeM, 0.f}};
    }
    if (!leg.empty())
        drawRibbon(view, eye, fov, hitDist, leg, cv::Scalar(255, 110, 40), 0.08f, 0.95, true, drop);
    label(view, cv::format("%d frames   voxels over the camera image", s.mapFrames),
          {10, 22}, 0.45, kInk, 1);
    label(view, "legs laid 0.6 m below the eye", {10, h - 12}, 0.42, kDim, 1);
    return view;
}

// ------------------------------------------------------------------ mission
cv::Mat FlightView::mission(const ShowSnap& s, int w, int h, bool compact) const {
    // A window round the aircraft, 1 px per cell, then scaled; north up.
    const float spanM = 36.f;
    const int n = int(spanM / planCell_);
    const sim::VoxelWorld& world = (*s.world);
    const sim::CamPose& a = s.truth;
    const float e0 = a.e - spanM * 0.5f, n0 = a.n - spanM * 0.5f;
    cv::Mat base(n, n, CV_8UC3, cv::Scalar(34, 30, 27));
    const sim::VoxelMap& map = (*s.map);
    const bool haveMap = s.mapFrames > 0 && s.map;
    const sim::CamPose& v = s.vantage;
    for (int y = 0; y < n; ++y) {
        cv::Vec3b* row = base.ptr<cv::Vec3b>(y);
        const float wn = n0 + (n - 1 - y + 0.5f) * planCell_;
        for (int x = 0; x < n; ++x) {
            const float we = e0 + (x + 0.5f) * planCell_;
            int cx, cy, cz;
            world.worldToCell(we, wn, a.u, cx, cy, cz);
            const bool solid = cx >= 0 && cy >= 0 && cx < plan_.cols && cy < plan_.rows &&
                               plan_.at<uchar>(plan_.rows - 1 - cy, cx) != 0;
            cv::Vec3b c = solid ? cv::Vec3b(120, 112, 104) : cv::Vec3b(46, 41, 37);
            // WHAT THIS STOP'S MAP SAYS about that cell, at flight altitude.
            if (haveMap) {
                const auto st = map.stateAt(we - v.e, wn - v.n, 0.f);
                if (st == sim::VoxelMap::FREE)
                    c = solid ? cv::Vec3b(60, 60, 220) : cv::Vec3b(150, 112, 62);
                else if (st == sim::VoxelMap::OCCUPIED)
                    c = cv::Vec3b(60, 60, 235);
            }
            row[x] = c;
        }
    }
    cv::Mat im;
    cv::resize(base, im, {w, h}, 0, 0, cv::INTER_NEAREST);
    const float sx = float(w) / spanM, sy = float(h) / spanM;
    auto px = [&](float e, float nn) {
        return cv::Point2f((e - e0) * sx, (spanM - (nn - n0)) * sy);
    };
    // The trail, and every leg flown as a tick where it began.
    const std::vector<cv::Point3f>& tr = s.trail;
    for (size_t i = 1; i < tr.size(); ++i)
        cv::line(im, px(tr[i - 1].x, tr[i - 1].y), px(tr[i].x, tr[i].y), kTrail, 2, cv::LINE_AA);
    for (const Leg& l : s.legs)
        cv::circle(im, px(l.e0, l.n0), 3, kInk, cv::FILLED, cv::LINE_AA);
    // The fan from this vantage.
    for (const Ray& r : s.fan) {
        const float br = r.bearingDeg * kPi / 180.f;
        cv::line(im, px(v.e, v.n), px(v.e + std::sin(br) * r.freeM, v.n + std::cos(br) * r.freeM),
                 {120, 200, 120}, 1, cv::LINE_AA);
    }
    // The aircraft: an arrowhead on its heading.
    const float yr = a.yawDeg * kPi / 180.f;
    const cv::Point2f c = px(a.e, a.n);
    const cv::Point2f fwd(std::sin(yr), -std::cos(yr)), rt(std::cos(yr), std::sin(yr));
    std::vector<cv::Point> arrow{c + fwd * 13.f, c - fwd * 8.f + rt * 8.f, c - fwd * 4.f,
                                 c - fwd * 8.f - rt * 8.f};
    cv::fillConvexPoly(im, arrow, {250, 250, 250}, cv::LINE_AA);
    cv::polylines(im, arrow, true, {20, 20, 20}, 1, cv::LINE_AA);

    if (compact) {
        label(im, "from above", {6, 16}, 0.4, kInk, 1);
        return im;
    }
    const FlightStats& st = s.stats;
    label(im, "from above, north up", {10, 22}, 0.5, kInk, 1);
    label(im, "blue: confirmed free   red: marked solid", {10, 42}, 0.42, kDim, 1);
    int y = h - 64;
    label(im, cv::format("flown %.0f m   %d m from start   %d cells", st.travelM, int(st.netM),
                         st.cells), {10, y}, 0.48, kInk, 1);
    label(im, cv::format("%d stops   %d legs   closest %.2f m", st.stops, st.legs,
                         st.minClearM > 100.f ? 0.f : st.minClearM), {10, y + 20}, 0.48, kInk, 1);
    label(im, cv::format("collisions %d", st.collisions), {10, y + 40}, 0.48,
          st.collisions ? cv::Scalar(80, 80, 240) : cv::Scalar(120, 210, 120), 1);
    return im;
}

}  // namespace kshow
