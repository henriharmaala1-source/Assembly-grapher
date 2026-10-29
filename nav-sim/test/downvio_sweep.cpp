// downvio_sweep -- VIO from a DOWNWARD camera: how much do the aircraft's
// tilt, rotation rate, the lens's field of view, speed, height and the ground
// matter?
//
// The estimator is the aircraft's own, unchanged: navcore's DepthVio (corners
// tracked in the IR image, each anchored at a metric 3D point by depth, then a
// 4-DOF pose solve with roll/pitch from the IMU). What changes is where the
// camera looks and where its depth comes from:
//
//   --depth plane    a MONOCULAR wide-FOV global-shutter camera plus a
//                    downward rangefinder: per-pixel range from the
//                    rangefinder, the IMU's (noisy) attitude and a FLAT-GROUND
//                    assumption -- what the cheap downward option has. Wrong
//                    wherever the ground is not flat (--world clutter).
//   --depth stereo   a downward stereo depth camera (navcore's D435-class
//                    failure model: holes, speckle, range noise).
//   --depth truth    perfect depth: the control.
//
// The camera is fixed to the airframe looking straight down (image top =
// nose), rendered every frame at the TRUE body attitude: a nose-down cruise
// tilt (--tilt), plus an optional roll/pitch wobble (--wobble AMP --wobble-hz
// F: turbulence, stick inputs; peak rate AMP*2*pi*F). The IMU hands the VIO
// that attitude with 0.3 deg noise (and --att-bias), and the rangefinder
// (plane mode) reads the true range along the body's down axis with 1 % noise.
//
// The world is navcore's IR model -- intensity a function of the WORLD POINT,
// band-limited to the pixel, so a surface looks the same from anywhere, as
// real ground does -- over a large floor:
//   --world flat     textured floor only
//   --world clutter  plus boxes 0.3-1.5 m tall over ~15 % of it (breaks the
//                    plane assumption, occludes, gives parallax)
//   --world lowtex   flat, but the floor's texture richness 0.15 (smooth
//                    concrete) instead of 0.5
//
// Flown: a straight line north at --speed for --dist metres at --alt above
// the floor. Scored against the truth: final horizontal error as % of the
// distance flown, worst error, fraction of frames LOST, mean inliers.
// One configuration per run, one CSV row on stdout (--header for the header):
// nav-sim/test/downvio_sweep.sh runs the grid.
//
// FOOTAGE OUT, for footage_vio.cpp (the same estimators on recorded video):
//   --gimbal DEG       the camera on a stabilised gimbal at this pitch (-90 =
//                      straight down, -45 oblique), turning with the heading,
//                      as on a DJI Neo 2 -- instead of fixed to the airframe
//   --path circle      one loop of circumference --dist, ending at the start
//   --write-video F    the rendered frames as a video (needs videoio)
//   --write-srt F      the truth as DJI-layout subtitles: position, height
//                      above the ground as rel_alt, gimbal angles
//
// PINHOLE ONLY. navcore's camera model and DepthVio are pinhole; real lenses
// past ~120 deg are fisheye, which concentrates resolution at the centre
// rather than stretching the edges. A pinhole at 140 deg is a pessimistic
// stand-in for the edges and an optimistic one for nothing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef DOWNVIO_HAVE_VIDEOIO
#include <opencv2/videoio.hpp>
#endif

#include "depth_camera.hpp"
#include "vio.hpp"
#include "voxel_world.hpp"
#include "ground_texture.hpp"
#include "down_vo.hpp"

using downvo::PnpVo;
using downvo::rotOf;
using downvo::tiltErrDeg;

namespace {

using downvo::kPi;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

struct V3 { double e, n, u; };
V3 add(V3 a, V3 b) { return {a.e + b.e, a.n + b.n, a.u + b.u}; }
V3 mul(V3 a, double s) { return {a.e * s, a.n * s, a.u * s}; }

// Body FRD axes in ENU from yaw (clockwise from North), pitch (nose up +) and
// roll (right wing down +): the same conventions as the FC and navcore.
void bodyAxes(double yawD, double pitchD, double rollD, V3& f, V3& r, V3& d) {
    const double ph = rad(rollD), th = rad(pitchD), ps = rad(yawD);
    V3 F{0, 1, 0}, R{1, 0, 0}, D{0, 0, -1};
    // roll about forward: right wing down -> right axis tips toward down
    V3 R1 = add(mul(R, std::cos(ph)), mul(D, std::sin(ph)));
    V3 D1 = add(mul(D, std::cos(ph)), mul(R, -std::sin(ph)));
    // pitch about the right axis: nose up -> forward tips toward up (= -down)
    V3 F2 = add(mul(F, std::cos(th)), mul(D1, -std::sin(th)));
    V3 D2 = add(mul(D1, std::cos(th)), mul(F, std::sin(th)));
    auto yaw = [&](V3 v) {
        return V3{v.e * std::cos(ps) + v.n * std::sin(ps),
                  -v.e * std::sin(ps) + v.n * std::cos(ps), v.u};
    };
    f = yaw(F2); r = yaw(R1); d = yaw(D2);
}

// A camera whose optical axis is `z`, image-right `x` (both ENU unit vectors)
// as a CamPose in navcore's Euler convention (depth_camera.cpp camToWorld):
// optical axis = (cp sy, cp cy, sp); image-right = (cr cy + sr sp sy,
// -cr sy + sr sp cy, -sr cp). Straight down is a gimbal lock (yaw and roll
// both turn the image about the vertical): there roll is 0 and yaw takes it.
sim::CamPose camPose(V3 pos, V3 z, V3 x) {
    sim::CamPose p;
    p.e = float(pos.e); p.n = float(pos.n); p.u = float(pos.u);
    const double sp = std::max(-1.0, std::min(1.0, z.u));
    const double pitch = std::asin(sp), cp = std::cos(pitch);
    double yaw, roll;
    if (cp > 1e-6) {
        yaw = std::atan2(z.e, z.n);
        const double sr = -x.u / cp;
        const double cr = x.e * std::cos(yaw) - x.n * std::sin(yaw);
        roll = std::atan2(sr, cr);
    } else {
        // E' = cos(yaw + roll * sign), N' = -sin(...): roll 0, yaw from x.
        roll = 0.0;
        yaw = std::atan2(-x.n, x.e);
        if (sp > 0) yaw = std::atan2(x.n, x.e);   // looking straight up (never, here)
    }
    p.yawDeg = float(deg(yaw)); p.pitchDeg = float(deg(pitch)); p.rollDeg = float(deg(roll));
    return p;
}

// The downward camera of a body at this attitude: optical axis = body down,
// image right = body right (so image top = the nose).
sim::CamPose downCam(V3 pos, double yawD, double pitchD, double rollD) {
    V3 f, r, d;
    bodyAxes(yawD, pitchD, rollD, f, r, d);
    return camPose(pos, d, r);
}

// `v` turned by `a` radians about the unit `axis` (Rodrigues).
V3 turn(V3 v, V3 k, double a) {
    const double c = std::cos(a), s = std::sin(a), kv = k.e * v.e + k.n * v.n + k.u * v.u;
    const V3 kxv{k.n * v.u - k.u * v.n, k.u * v.e - k.e * v.u, k.e * v.n - k.n * v.e};
    return add(add(mul(v, c), mul(kxv, s)), mul(k, kv * (1 - c)));
}

// A camera on a STABILISED gimbal: optical axis `pitchD` below the horizon
// toward the heading `yawD`, image level -- then tipped by `errA` degrees
// about its image-right axis and `errB` about the horizontal heading (the
// gimbal's, or the IMU's opinion of it, being off).
sim::CamPose gimbalCam(V3 pos, double yawD, double pitchD, double errA, double errB) {
    const double y = rad(yawD), p = rad(pitchD);
    V3 z{std::cos(p) * std::sin(y), std::cos(p) * std::cos(y), std::sin(p)};
    V3 x{std::cos(y), -std::sin(y), 0.0};
    const V3 f{std::sin(y), std::cos(y), 0.0};
    z = turn(z, x, rad(errA));
    z = turn(z, f, rad(errB)); x = turn(x, f, rad(errB));
    return camPose(pos, z, x);
}

void box(sim::VoxelWorld& w, float x0, float y0, float z0, float x1, float y1, float z1,
         float tex) {
    int ax, ay, az, bx, by, bz;
    w.worldToCell(x0, y0, z0, ax, ay, az);
    w.worldToCell(x1, y1, z1, bx, by, bz);
    for (int z = az; z <= bz; ++z)
        for (int y = ay; y <= by; ++y)
            for (int x = ax; x <= bx; ++x) { w.set(x, y, z); w.setTex(x, y, z, tex); }
}

struct Opt {
    float fov = 90.f, tilt = 0.f, wobble = 0.f, wobbleHz = 1.5f, speed = 2.f, alt = 1.5f;
    float dist = 30.f, attBias = 0.f, fps = 30.f, attNoise = 0.3f;
    int   w = 640, h = 400;
    unsigned seed = 1;
    std::string depth = "plane", world = "flat", dump;
    // --ground SCENE: real ground photographs under the sun (ground_texture.hpp)
    // instead of navcore's IR noise model ("noise", the default).
    std::string ground = "noise", texDir = "nav-sim/textures";
    float sunElev = 45.f, sunAz = 135.f, exposureMs = 0.f;
    float checkDrift = -1.f;      // > 0: exit 1 if the drift % exceeds it (ctest)
    // imu: DepthVio, roll/pitch from the IMU. vision: PnpVo, all 6 DOF from
    // the image, no IMU at all.
    std::string attitude = "imu";
    bool droneShadow = false, castShadows = true;
    // --gimbal DEG: the camera on a STABILISED gimbal at this pitch (-90 =
    // straight down), level and turning with the heading -- a DJI-style
    // camera, instead of one fixed to the airframe. Body tilt and wobble then
    // move only the rangefinder, and that reads the vertical height (1 %).
    bool gimbalOn = false;
    float gimbal = -90.f;
    // --path line: north for --dist. circle: one loop of circumference --dist,
    // turning right, heading along the track, ending where it started.
    std::string path = "line";
    // Footage out, for nav-sim/test/footage_vio.cpp: the rendered frames as a
    // video, and the truth as DJI-style subtitles (.SRT).
    std::string writeVideo, writeSrt;
};

// Arc length -> position (east, north) and heading for --path.
void pathAt(const Opt& o, double s, double& e, double& n, double& yawD) {
    if (o.path == "circle") {
        const double r = o.dist / (2 * kPi);
        e = r - r * std::cos(s / r); n = r * std::sin(s / r); yawD = s / r * 180.0 / kPi;
    } else {
        e = 0.0; n = s; yawD = 0.0;
    }
}

float floorTop = 0.1f;   // the floor slab's top, metres

// How far the lens can see across the ground: the steepest ray off the
// vertical is half the field of view plus the tilt (and the wobble), and the
// floor must reach past it -- or the "horizon" is the edge of the test world.
// (The first version used a fixed 30 m floor: a 140 deg lens at 15 m saw 29 %
// floor and drifted 12 %, for the world's sake, not the camera's.)
float reachM(const Opt& o) {
    const float off = o.gimbalOn ? 90.f + o.gimbal : o.tilt + o.wobble;   // optical axis off vertical
    const float a = std::min(80.f, 0.5f * o.fov * 1.25f + off);   // diagonal ~1.25x
    return std::min(150.f, o.alt * std::tan(a * float(kPi) / 180.f) + 10.f);
}

// The floor covers the path's extent [x0,x1] x [y0,y1] plus the lens's reach.
void buildWorld(sim::VoxelWorld& w, const Opt& o, float x0, float x1, float y0, float y1) {
    const float R = std::max(15.f, reachM(o));
    const float W = (x1 - x0) + 2.f * R, L = (y1 - y0) + 2.f * R, H = 3.f;
    const float xa = x0 - R, ya = y0 - R;
    // Cells only shape the boxes; the IR texture is a function of the world
    // point (band-limited), so a coarser lattice on a big floor costs nothing.
    const float c = W > 60.f ? 0.25f : 0.1f;
    floorTop = c;                          // one cell of slab: its top is the floor
    w.init(c, xa, ya, 0.f, int(W / c), int(L / c), int(H / c));
    const float floorTex = o.world == "lowtex" ? 0.15f : 0.5f;
    box(w, xa, ya, 0.f, xa + W, ya + L, floorTop - 0.01f, floorTex);
    if (o.world == "clutter") {
        std::mt19937 rng(o.seed * 7919u + 3u);
        std::uniform_real_distribution<float> X(xa, xa + W), Y(ya, ya + L),
            S(0.5f, 2.f), Hh(0.3f, 1.5f), T(0.35f, 0.65f);
        const int n = int(0.15f * W * L / (1.25f * 1.25f));   // ~15 % covered
        for (int i = 0; i < n; ++i) {
            const float x = X(rng), y = Y(rng), s = S(rng), hh = Hh(rng);
            box(w, x, y, floorTop, x + s, y + s, floorTop + hh, T(rng));
        }
    }
}

bool parse(int argc, char** argv, Opt& o, bool& header) {
    header = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto f = [&](float& v) { if (i + 1 < argc) v = float(std::atof(argv[++i])); };
        if (a == "--fov") f(o.fov);
        else if (a == "--tilt") f(o.tilt);
        else if (a == "--wobble") f(o.wobble);
        else if (a == "--wobble-hz") f(o.wobbleHz);
        else if (a == "--speed") f(o.speed);
        else if (a == "--alt") f(o.alt);
        else if (a == "--dist") f(o.dist);
        else if (a == "--att-bias") f(o.attBias);
        else if (a == "--att-noise") f(o.attNoise);
        else if (a == "--fps") f(o.fps);
        else if (a == "--seed" && i + 1 < argc) o.seed = unsigned(std::atoi(argv[++i]));
        else if (a == "--res" && i + 1 < argc) std::sscanf(argv[++i], "%dx%d", &o.w, &o.h);
        else if (a == "--depth" && i + 1 < argc) o.depth = argv[++i];
        else if (a == "--world" && i + 1 < argc) o.world = argv[++i];
        else if (a == "--dump" && i + 1 < argc) o.dump = argv[++i];
        else if (a == "--ground" && i + 1 < argc) o.ground = argv[++i];
        else if (a == "--tex-dir" && i + 1 < argc) o.texDir = argv[++i];
        else if (a == "--sun-elev") f(o.sunElev);
        else if (a == "--sun-az") f(o.sunAz);
        else if (a == "--exposure-ms") f(o.exposureMs);
        else if (a == "--drone-shadow") o.droneShadow = true;
        else if (a == "--no-cast-shadows") o.castShadows = false;
        else if (a == "--check-drift") f(o.checkDrift);
        else if (a == "--attitude" && i + 1 < argc) o.attitude = argv[++i];
        else if (a == "--gimbal") { o.gimbalOn = true; f(o.gimbal); }
        else if (a == "--path" && i + 1 < argc) o.path = argv[++i];
        else if (a == "--write-video" && i + 1 < argc) o.writeVideo = argv[++i];
        else if (a == "--write-srt" && i + 1 < argc) o.writeSrt = argv[++i];
        else if (a == "--header") header = true;
        else { std::fprintf(stderr, "downvio_sweep: unknown argument %s\n", a.c_str()); return false; }
    }
    return true;
}

}  // namespace

// The rotation above is the one thing here that is not navcore's: check it
// against navcore's own projection over a grid of attitudes, gimbal lock
// included. Returns the worst axis error, degrees.
double selfTest() {
    double worst = 0;
    for (int y = -180; y < 180; y += 30)
        for (int p = -60; p <= 60; p += 5)
            for (int r = -60; r <= 60; r += 5) {
                V3 f, rr, d;
                bodyAxes(y, p, r, f, rr, d);
                const sim::CamPose c = downCam({0, 0, 0}, y, p, r);
                float zx, zy, zz, xx, xy, xz;
                sim::DepthCamera::camToWorld(c, 0, 0, 1, zx, zy, zz);
                sim::DepthCamera::camToWorld(c, 1, 0, 0, xx, xy, xz);
                const double ez = std::acos(std::min(1.0, zx * d.e + zy * d.n + zz * d.u));
                const double ex = std::acos(std::min(1.0, xx * rr.e + xy * rr.n + xz * rr.u));
                worst = std::max({worst, deg(ez), deg(ex)});
            }
    return worst;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--selftest") == 0) {
        const double w = selfTest();
        std::printf("camera rotation vs navcore camToWorld: worst %.4f deg\n", w);
        return w < 0.05 ? 0 : 1;   // float CamPose: acos of 1 - 1e-7 is already 0.026 deg
    }
    Opt o;
    bool header;
    if (!parse(argc, argv, o, header)) return 2;
    const char* cols = "depth,world,fov,tilt,wobble,wobble_hz,peak_rate_dps,speed,alt,att_bias,"
                       "seed,frames,drift_pct,final_err_m,max_err_m,lost_pct,mean_inliers,"
                       "ground_frac,ms_per_frame,ground,sun_elev,drone_shadow,exposure_ms,"
                       "attitude,tilt_err_rms_deg,tilt_err_max_deg";
    if (header) { std::printf("%s\n", cols); return 0; }

    if (o.path != "line" && o.path != "circle") {
        std::fprintf(stderr, "downvio_sweep: --path takes line|circle\n");
        return 2;
    }
    sim::VoxelWorld world;
    if (o.path == "circle") {
        const float r = float(o.dist / (2 * kPi));
        buildWorld(world, o, 0.f, 2.f * r, -r, r);
    } else {
        buildWorld(world, o, 0.f, 0.f, 0.f, o.dist);
    }
    kground::GroundRenderer ground;
    if (o.ground != "noise") {
        std::string err;
        if (!ground.load(o.texDir, &err)) {
            std::fprintf(stderr, "downvio_sweep: %s\n", err.c_str());
            return o.checkDrift > 0 ? 77 : 2;          // a check without its data: skipped
        }
        if (!ground.setScene(o.ground, o.seed, floorTop)) {
            std::fprintf(stderr, "downvio_sweep: --ground takes noise|%s\n", kground::GroundRenderer::sceneNames());
            return 2;
        }
        ground.light.sunElevDeg = o.sunElev; ground.light.sunAzDeg = o.sunAz;
        ground.light.castShadows = o.castShadows;
    }
    sim::CamParams cp;
    cp.width = o.w; cp.height = o.h; cp.hfovDeg = o.fov;
    cp.baselineM = 0.05f; cp.irBandLimit = true; cp.maxRangeM = std::max(30.f, 2.f * reachM(o));
    cp.seed = o.seed;
    sim::DepthCamera cam(cp);

    sim::VioParams vp;
    vp.maxDepthM = std::max(15.f, 4.f * o.alt);  // a downward lens sees the ground obliquely
    // The per-frame jump gate (0.5 m, set for kestrel's slow legs) is about
    // wrong solves, not speed: at FPV cruise it would reject every frame.
    vp.maxJumpM = std::max(vp.maxJumpM, 2.5f * o.speed / o.fps);
    sim::DepthVio vio;
    vio.init(cam, vp);
    PnpVo pvo;
    pvo.init(cam, vp.maxDepthM, vp.maxJumpM);
    const bool visionAtt = o.attitude == "vision";
    double tiltSq = 0, tiltMax = 0; int tiltN = 0;
    std::vector<double> estMs;    // the ESTIMATOR alone (plus vision mode's own plane-depth build)

    std::mt19937 rng(o.seed * 104729u + 17u);
    std::normal_distribution<double> N(0.0, 1.0);
    const double dt = 1.0 / o.fps;
    // FROM A HOVER: the speed ramps up over 1.5 s, as an aircraft accelerates.
    // (Starting at full speed handed DepthVio 50 px of flow on its first frame
    // with no velocity prior: a 60 deg lens at 1.5 m and 4 m/s never caught on.)
    const double ramp = 1.5;
    auto along = [&](double t) {
        return t < ramp ? 0.5 * o.speed * t * t / ramp : o.speed * (t - 0.5 * ramp);
    };
    const int frames = int((o.dist / o.speed + 0.5 * ramp) * o.fps) + 1;
    const double u0 = floorTop + o.alt;
    double sumIn = 0, maxErr = 0, groundSum = 0, msSum = 0;
    int lost = 0, nIn = 0;
    double finalErr = 0;
    // A random phase per seed, so the wobble is not the same at the same place.
    const double ph1 = std::uniform_real_distribution<double>(0, 2 * kPi)(rng);
    const double ph2 = std::uniform_real_distribution<double>(0, 2 * kPi)(rng);
    const double peakRate = o.wobble * 2 * kPi * o.wobbleHz;
    // The camera at a body attitude, with attitude errors: fixed to the
    // airframe (errors on the body's pitch and roll), or on the gimbal (errors
    // on its two tilt axes; the body's attitude does not reach it).
    auto camAt = [&](V3 p, double yawD, double pitchD, double rollD, double ea, double eb) {
        return o.gimbalOn ? gimbalCam(p, yawD, o.gimbal, ea, eb)
                          : downCam(p, yawD, pitchD + ea, rollD + eb);
    };
    auto poseAt = [&](double tt, double& yawD) {
        double e, n;
        pathAt(o, along(std::max(0.0, tt)), e, n, yawD);
        return V3{e, n, u0};
    };

#ifdef DOWNVIO_HAVE_VIDEOIO
    cv::VideoWriter video;
#endif
    if (!o.writeVideo.empty()) {
        bool opened = false;
#ifdef DOWNVIO_HAVE_VIDEOIO
        opened = video.open(o.writeVideo, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), o.fps,
                            cv::Size(o.w, o.h), true);
#endif
        if (!opened) {
            std::fprintf(stderr, "downvio_sweep: cannot write %s (no videoio/FFmpeg in this build?)\n",
                         o.writeVideo.c_str());
            return 77;
        }
    }
    FILE* srt = nullptr;
    if (!o.writeSrt.empty() && !(srt = std::fopen(o.writeSrt.c_str(), "w"))) {
        std::fprintf(stderr, "downvio_sweep: cannot write %s\n", o.writeSrt.c_str());
        return 2;
    }

    for (int k = 0; k < frames; ++k) {
        const double t = k * dt;
        // Body attitude: nose-down cruise tilt plus the wobble on both axes.
        auto pitchAt = [&](double tt) { return -o.tilt + o.wobble * std::sin(2 * kPi * o.wobbleHz * tt + ph1); };
        auto rollAt = [&](double tt) { return o.wobble * std::sin(2 * kPi * o.wobbleHz * 0.83 * tt + ph2); };
        double yaw;
        const V3 pos = poseAt(t, yaw);
        const double pitch = pitchAt(t), roll = rollAt(t);
        const sim::CamPose truth = camAt(pos, yaw, pitch, roll, 0.0, 0.0);
        const double heightTrue = u0 - floorTop;

        // What the IMU says: 0.3 deg noise on roll and pitch, plus any bias.
        const double errP = o.attNoise * N(rng) + o.attBias;
        const double errR = o.attNoise * N(rng) + o.attBias;
        const double pitchI = pitch + errP, rollI = roll + errR;
        const sim::CamPose imuCam = camAt(pos, yaw, pitch, roll, errP, errR);

        const auto t0 = std::chrono::steady_clock::now();
        cv::Mat ir;
        if (o.ground == "noise") {
            ir = cam.renderIR(world, truth);
        } else {
            // Motion blur: the poses across the exposure, ending at this frame.
            std::vector<sim::CamPose> blur;
            if (o.exposureMs > 0.f) {
                const int K = ground.camera.blurSubFrames;
                for (int j = 0; j < K; ++j) {
                    const double tj = t - 1e-3 * o.exposureMs * double(K - 1 - j) / double(K - 1);
                    double yj;
                    const V3 pj = poseAt(tj, yj);
                    blur.push_back(camAt(pj, yj, pitchAt(tj), rollAt(tj), 0.0, 0.0));
                }
            }
            kground::DroneShadow sh;
            sh.on = o.droneShadow; sh.e = float(pos.e); sh.n = float(pos.n); sh.u = float(pos.u);
            ir = ground.render(world, cam, truth, blur, sh);
        }
        if (!o.writeVideo.empty()) {
#ifdef DOWNVIO_HAVE_VIDEOIO
            cv::Mat bgr;
            cv::cvtColor(ir, bgr, cv::COLOR_GRAY2BGR);
            video.write(bgr);
#endif
        }
        if (srt) {
            // DJI-style subtitle block (the Mini/Air layout; what a Neo 2
            // writes is unverified -- footage_vio.cpp parses fields by name).
            // Positions about an arbitrary origin; height above the ground as
            // rel_alt (took off from the ground); the gimbal's attitude as
            // gb_* (the optical axis' pitch, as DJI reports it).
            const double R = 6378137.0, lat0 = 60.17, lon0 = 24.94;
            const double lat = lat0 + pos.n / R * 180.0 / kPi;
            const double lon = lon0 + pos.e / (R * std::cos(rad(lat0))) * 180.0 / kPi;
            auto ts = [](double x, char* b) {
                const int ms = int(std::lround(x * 1000.0));
                std::snprintf(b, 32, "%02d:%02d:%02d,%03d", ms / 3600000, ms / 60000 % 60, ms / 1000 % 60, ms % 1000);
            };
            char a[32], b[32];
            ts(t, a); ts(t + dt, b);
            const double gp = o.gimbalOn ? o.gimbal : -90.0;   // body-fixed: "straight down"
            std::fprintf(srt,
                         "%d\n%s --> %s\n<font size=\"28\">FrameCnt: %d, DiffTime: %dms\n"
                         "2026-10-29 12:%02d:%06.3f\n"
                         "[iso: 100] [shutter: 1/1000.0] [fnum: 2.8] [ev: 0] [color_md: default] "
                         "[focal_len: 24.00] [latitude: %.6f] [longitude: %.6f] "
                         "[rel_alt: %.3f abs_alt: %.3f] [gb_yaw: %.1f gb_pitch: %.1f gb_roll: %.1f] "
                         "[ct: 5500] </font>\n\n",
                         k + 1, a, b, k + 1, int(std::lround(dt * 1000)), int(t / 60.0) % 60, std::fmod(t, 60.0), lat, lon,
                         heightTrue, heightTrue + 20.0, yaw > 180 ? yaw - 360 : yaw, gp, 0.0);
        }
        // Mono + rangefinder depth from a given camera attitude (rotation, C):
        // the IMU's in imu mode; the vision's OWN estimate in vision mode.
        float rngMeas = -1.f;
        // On the gimbal the rangefinder is not along the optical axis: it
        // reports the vertical height (as a DJI height estimate does), 1 %.
        auto planeDepth = [&](const cv::Matx33d& Rwc) {
            if (rngMeas < 0.f) {
                if (o.gimbalOn) {
                    rngMeas = float(heightTrue * (1.0 + 0.01 * N(rng)));
                } else {
                    V3 f0, r0, d0;
                    bodyAxes(yaw, pitch, roll, f0, r0, d0);
                    rngMeas = world.raycast(float(pos.e), float(pos.n), float(pos.u), float(d0.e),
                                            float(d0.n), float(d0.u), cp.maxRangeM);
                    rngMeas *= float(1.0 + 0.01 * N(rng));
                }
            }
            // Body down = the optical axis = Rwc's third column.
            const double H = o.gimbalOn ? rngMeas : rngMeas * (-Rwc(2, 2));
            return downvo::planeDepth(cam, o.w, o.h, Rwc, H);
        };
        cv::Mat depth;
        if (visionAtt) {
            // Built inside the VO, at keyframes, from its own attitude.
        } else if (o.depth == "truth") {
            depth = cam.renderTruth(world, truth);
        } else if (o.depth == "stereo") {
            depth = cam.renderStereo(world, truth, nullptr);
        } else {
            // MONO + RANGEFINDER: the true range along the body's down axis
            // (1 % noise), the plane it implies from the IMU's attitude, and
            // each pixel's ray -- by the IMU's attitude too -- cut against it.
            V3 f, r, d;
            bodyAxes(yaw, pitch, roll, f, r, d);
            float rng1 = o.gimbalOn ? float(heightTrue)
                                    : world.raycast(float(pos.e), float(pos.n), float(pos.u), float(d.e),
                                                    float(d.n), float(d.u), cp.maxRangeM);
            rng1 *= float(1.0 + 0.01 * N(rng));
            V3 fI, rI, dI;
            bodyAxes(yaw, pitchI, rollI, fI, rI, dI);
            const double H = o.gimbalOn ? double(rng1) : rng1 * (-dI.u);   // height above "the" plane
            depth.create(o.h, o.w, CV_32F);
            for (int v = 0; v < o.h; ++v) {
                float* row = depth.ptr<float>(v);
                for (int u = 0; u < o.w; ++u) {
                    float dx, dy, dz;
                    cam.rayFor(imuCam, u, v, dx, dy, dz);
                    row[u] = (dz < -0.05f) ? float(H / -dz) : 0.f;
                }
            }
        }
        // How much of the view is the floor at all (vs boxes, walls, or
        // nothing): a sparse grid of rays, not a second full render.
        {
            int g = 0, tot = 0;
            for (int v = 4; v < o.h; v += 16)
                for (int u = 4; u < o.w; u += 16) {
                    ++tot;
                    float dx, dy, dz;
                    cam.rayFor(truth, u, v, dx, dy, dz);
                    const float rr = world.raycast(truth.e, truth.n, truth.u, dx, dy, dz, cp.maxRangeM);
                    if (rr < cp.maxRangeM && truth.u + dz * rr < floorTop + 0.05f) ++g;
                }
            groundSum += double(g) / tot;
        }
        sim::VioResult res;
        const auto te0 = std::chrono::steady_clock::now();
        if (visionAtt) {
            const cv::Matx33d Rt = rotOf(truth);
            if (k == 0) pvo.reset(Rt, cv::Vec3d(truth.e, truth.n, truth.u));   // start pose known
            cv::Mat stereoD, truthD;
            res.valid = pvo.step(ir, [&](const cv::Matx33d& Rwc, const cv::Vec3d&) {
                if (o.depth == "plane") return planeDepth(Rwc);
                if (o.depth == "stereo") return stereoD.empty() ? (stereoD = cam.renderStereo(world, truth, nullptr)) : stereoD;
                return truthD.empty() ? (truthD = cam.renderTruth(world, truth)) : truthD;
            });
            res.inliers = pvo.inliers();
            estMs.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - te0).count());
            const double te = tiltErrDeg(pvo.R(), Rt);
            tiltSq += te * te; tiltMax = std::max(tiltMax, te); ++tiltN;
        } else {
            if (k == 0) vio.reset(truth);                // the start pose is known
            sim::CamPose imuIn = imuCam;
            res = vio.step(ir, depth, imuIn);
            estMs.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - te0).count());
            const double te = tiltErrDeg(rotOf(imuCam), rotOf(truth));   // the IMU's own tilt error
            tiltSq += te * te; tiltMax = std::max(tiltMax, te); ++tiltN;
        }
        msSum += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (!o.dump.empty() && k == frames / 2) {
            cv::imwrite(o.dump + "_ir.png", ir);
            if (!depth.empty()) {
                cv::Mat d8; depth.convertTo(d8, CV_8U, 255.0 / 15.0);
                cv::imwrite(o.dump + "_depth.png", d8);
            }
        }
        if (std::getenv("DOWNVIO_TRACE"))
            std::fprintf(stderr, "k %d valid %d tracked %d inliers %d kf %d err %.3f\n", k,
                         int(res.valid), res.tracked, res.inliers, int(res.newKeyframe),
                         std::hypot(vio.pose().e - truth.e, vio.pose().n - truth.n));
        if (k == 0) continue;
        if (!res.valid) ++lost; else { sumIn += res.inliers; ++nIn; }
        const double ee = visionAtt ? pvo.C()[0] : double(vio.pose().e);
        const double en = visionAtt ? pvo.C()[1] : double(vio.pose().n);
        const double err = std::hypot(ee - truth.e, en - truth.n);
        maxErr = std::max(maxErr, err);
        finalErr = err;
    }
    if (srt) std::fclose(srt);
    std::printf("%s,%s,%.0f,%.0f,%.0f,%.2f,%.0f,%.1f,%.1f,%.1f,%u,%d,%.2f,%.3f,%.3f,%.1f,%.0f,%.2f,%.0f,%s,%.0f,%d,%.1f,%s,%.3f,%.3f\n",
                o.depth.c_str(), o.world.c_str(), double(o.fov), double(o.tilt), double(o.wobble),
                double(o.wobbleHz), peakRate, double(o.speed), double(o.alt), double(o.attBias),
                o.seed, frames, 100.0 * finalErr / std::max(1e-6, along((frames - 1) * dt)), finalErr, maxErr,
                100.0 * lost / std::max(1, frames - 1), nIn ? sumIn / nIn : 0.0,
                groundSum / frames, msSum / frames, o.ground.c_str(), double(o.sunElev),
                o.droneShadow ? 1 : 0, double(o.exposureMs), o.attitude.c_str(),
                tiltN ? std::sqrt(tiltSq / tiltN) : 0.0, tiltMax);
    if (std::getenv("DOWNVIO_TIMING") && !estMs.empty()) {
        std::vector<double> v = estMs;
        std::sort(v.begin(), v.end());
        double sum = 0; for (double x : v) sum += x;
        std::fprintf(stderr, "estimator ms/frame: mean %.2f  median %.2f  p95 %.2f  max %.2f  (%zu frames)\n",
                     sum / double(v.size()), v[v.size() / 2], v[v.size() * 95 / 100], v.back(), v.size());
    }
    const double drift = 100.0 * finalErr / std::max(1e-6, along((frames - 1) * dt));
    if (o.checkDrift > 0 && drift > o.checkDrift) {
        std::fprintf(stderr, "downvio_sweep: drift %.2f %% > %.2f %%\n", drift, double(o.checkDrift));
        return 1;
    }
    return 0;
}
