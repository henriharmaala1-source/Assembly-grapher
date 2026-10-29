// footage_vio -- the downward-camera VIO of downvio_sweep, run on RECORDED
// footage: a DJI Neo 2 (or any camera) looking down from a known height.
//
// The simulator answered "how well should this work"; this is the harness for
// "how well does it work on a real camera, over real ground". It runs the SAME
// estimators on the video's frames, unchanged:
//
//   --attitude gimbal   navcore's DepthVio (4-DOF solve) with the camera's
//                       tilt from the GIMBAL: a stabilised camera holds the
//                       pitch you set (--gimbal-pitch, -90 straight down) and
//                       a level horizon, which is what an IMU would hand the
//                       solve -- the gimbal IS the IMU's attitude, applied.
//   --attitude srt      the same, with the gimbal's pitch/roll/yaw per frame
//                       from the subtitle file, where the drone writes them
//   --attitude vision   PnpVo (down_vo.hpp): all 6 DOF from the image, no
//                       attitude input at all past the first frame
//
// DEPTH is the mono + rangefinder model of the study: each pixel's ray cut
// against a FLAT GROUND a known height below the camera. The height is yours
// to set, because it is the one number the recording does not carry reliably:
//
//   --height M          a constant -- you hovered or flew at a set height
//   --height-csv FILE   time_s,height_m rows (a flight-log export), interpolated
//   (neither)           the subtitles' rel_alt: barometric, relative to TAKEOFF,
//                       so a hand launch needs --height-offset (palm at 1.2 m:
//                       rel_alt 0 is 1.2 m above the ground)
//
// A WRONG HEIGHT SCALES THE WHOLE TRACK: 10 % too high and every distance is
// 10 % long. So is a wrong lens (--hfov / --calib). With GPS in the subtitles
// the report fits the track to it and says which height would have made the
// scale right -- the check on both.
//
// HEADING. Nothing here knows north: the track starts at heading --start-yaw
// (or the subtitles' gb_yaw), and the comparison with GPS fits one rotation
// about the start. The camera's yaw between frames comes from the VIO itself
// (a constant-rate prediction stands in for the gyro the footage lacks), or
// from the subtitles' gb_yaw.
//
// WHAT THIS CANNOT SEE, and the footage can contain:
//   - ROLLING SHUTTER. The Neo 2's sensor reads out line by line; fast motion
//     skews the frame. The estimators assume a global shutter.
//   - ELECTRONIC STABILISATION (EIS / RockSteady) warps every frame by an
//     unknown homography. Record with it OFF.
//   - NOT-FLAT GROUND: anything standing up (trees, cars, walls, the pilot)
//     is placed on the ground plane at the wrong range.
//
// The lens: --hfov DEG for a rectilinear (already distortion-corrected)
// video, or --calib FILE from `footage_vio --calibrate CHECKERBOARD_VIDEO`.
//
// Output: a summary on stdout; --csv per-frame track; --plot a top-down
// picture of the track (and GPS); --shot one processed frame, to check the
// lens correction; --summary appends one row per run, for a table over clips.
//
// Validation without a drone: downvio_sweep --write-video/--write-srt renders
// the same flights the study measured, as a video and DJI-style subtitles,
// with the truth known (nav-sim/test/footage_vio_synthetic.sh).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "depth_camera.hpp"
#include "vio.hpp"
#include "down_vo.hpp"

namespace {

using downvo::kPi;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

struct Opt {
    std::string video, srt, calib, heightCsv, csv, plot, shot, summary;
    bool noSrt = false;
    float hfov = 0.f;                 // > 0: pinhole of this horizontal FOV
    float height = 0.f;               // > 0: constant height, metres
    float heightOffset = 0.f;
    float heightTimeOffset = 0.f;     // added to video time before the height CSV lookup
    float gimbalPitch = -90.f, gimbalRoll = 0.f;
    float startYaw = 0.f;
    bool startYawSet = false;
    std::string attitude = "gimbal";  // gimbal | srt | vision
    float start = 0.f, end = -1.f;
    int every = 1, width = 960;
    float maxSpeed = 12.f;            // m/s: sets the per-frame jump gate
    float checkDrift = -1.f;          // > 0: exit 1 if drift % exceeds it
    bool quiet = false;
    // --calibrate
    std::string calibVideo, calibOut = "calib.yml";
    int boardW = 9, boardH = 6, maxViews = 60;
    float squareMm = 25.f;
    bool fisheye = false;
};

void usage() {
    std::printf(
        "footage_vio -- downward VIO on recorded footage (DJI Neo 2 or any camera)\n\n"
        "  footage_vio --video FILE (--hfov DEG | --calib FILE) [options]\n\n"
        "lens (one of):\n"
        "  --hfov DEG             horizontal FOV of the frames as recorded (rectilinear)\n"
        "  --calib FILE           from --calibrate (pinhole or fisheye model)\n"
        "height above the ground (custom height wins):\n"
        "  --height M             constant\n"
        "  --height-csv FILE      time_s,height_m rows, interpolated\n"
        "  --height-time-offset S video time + S = CSV time\n"
        "  (neither)              the subtitles' rel_alt (relative to TAKEOFF)\n"
        "  --height-offset M      added to any of them (hand launch at 1.2 m: 1.2)\n"
        "attitude:\n"
        "  --attitude gimbal|srt|vision   (default gimbal)\n"
        "  --gimbal-pitch DEG     camera pitch, -90 = straight down (default -90)\n"
        "  --gimbal-roll DEG      horizon tilt of the image (default 0)\n"
        "  --start-yaw DEG        heading at the start (default: subtitles' gb_yaw, else 0)\n"
        "range and speed:\n"
        "  --start S --end S      part of the video to run (seconds)\n"
        "  --every N              every Nth frame (60 fps footage: 2)\n"
        "  --width PX             processing width (default 960; aspect kept)\n"
        "  --max-speed M/S        fastest the aircraft flew (default 12): the jump gate\n"
        "telemetry:\n"
        "  --srt FILE             DJI subtitles (default: the video's name with .SRT/.srt)\n"
        "  --no-srt               ignore them\n"
        "output:\n"
        "  --csv FILE             per-frame track\n"
        "  --plot FILE.png        top-down track, against GPS when there is one\n"
        "  --shot FILE.png        one processed frame (check the lens correction)\n"
        "  --summary FILE         append one CSV row per run\n"
        "  --check-drift PCT      exit 1 if the drift exceeds PCT %% of the path\n"
        "  --quiet\n\n"
        "  footage_vio --calibrate VIDEO [--board 9x6] [--square-mm 25] [--fisheye]\n"
        "              [--out calib.yml] [--every N] [--max-views 60]\n"
        "      lens calibration from a clip of a checkerboard (inner corners WxH),\n"
        "      shot in the SAME video mode as the flights\n\n"
        "  footage_vio --selftest\n");
}

bool parse(int argc, char** argv, Opt& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool more = i + 1 < argc;
        auto f = [&](float& v) { v = float(std::atof(argv[++i])); };
        if (!more && a != "--no-srt" && a != "--fisheye" && a != "--quiet" && a != "--help" && a != "-h") {
            std::fprintf(stderr, "footage_vio: %s needs a value\n", a.c_str());
            return false;
        }
        if (a == "--video") o.video = argv[++i];
        else if (a == "--srt") o.srt = argv[++i];
        else if (a == "--no-srt") o.noSrt = true;
        else if (a == "--calib") o.calib = argv[++i];
        else if (a == "--hfov") f(o.hfov);
        else if (a == "--height") f(o.height);
        else if (a == "--height-csv") o.heightCsv = argv[++i];
        else if (a == "--height-offset") f(o.heightOffset);
        else if (a == "--height-time-offset") f(o.heightTimeOffset);
        else if (a == "--attitude") o.attitude = argv[++i];
        else if (a == "--gimbal-pitch") f(o.gimbalPitch);
        else if (a == "--gimbal-roll") f(o.gimbalRoll);
        else if (a == "--start-yaw") { f(o.startYaw); o.startYawSet = true; }
        else if (a == "--start") f(o.start);
        else if (a == "--end") f(o.end);
        else if (a == "--every") o.every = std::max(1, std::atoi(argv[++i]));
        else if (a == "--width") o.width = std::max(64, std::atoi(argv[++i]));
        else if (a == "--max-speed") f(o.maxSpeed);
        else if (a == "--csv") o.csv = argv[++i];
        else if (a == "--plot") o.plot = argv[++i];
        else if (a == "--shot") o.shot = argv[++i];
        else if (a == "--summary") o.summary = argv[++i];
        else if (a == "--check-drift") f(o.checkDrift);
        else if (a == "--quiet") o.quiet = true;
        else if (a == "--calibrate") o.calibVideo = argv[++i];
        else if (a == "--board") std::sscanf(argv[++i], "%dx%d", &o.boardW, &o.boardH);
        else if (a == "--square-mm") f(o.squareMm);
        else if (a == "--fisheye") o.fisheye = true;
        else if (a == "--out") o.calibOut = argv[++i];
        else if (a == "--max-views") o.maxViews = std::max(10, std::atoi(argv[++i]));
        else if (a == "--help" || a == "-h") { usage(); std::exit(0); }
        else { std::fprintf(stderr, "footage_vio: unknown argument %s (--help)\n", a.c_str()); return false; }
    }
    return true;
}

// ---------------------------------------------------------------------------
// DJI subtitles. One block per frame (or per second, on some models):
//
//   12
//   00:00:00,400 --> 00:00:00,433
//   <font size="28">FrameCnt: 12, DiffTime: 33ms
//   2024-05-06 12:00:00.400
//   [iso: 100] ... [latitude: 60.170012] [longitude: 24.940018]
//   [rel_alt: 5.200 abs_alt: 30.100] [gb_yaw: 12.3 gb_pitch: -90.0 gb_roll: 0.0] </font>
//
// The fields differ by model and firmware, and what a Neo 2 writes is not
// verified here -- so every "name: number" pair is read by name, plus the two
// older layouts ("GPS(lon,lat,alt)", "H 5.2m") and DJI's own misspelling
// "longtitude". What was found is printed, so a missing field is visible.
// ---------------------------------------------------------------------------
struct SrtRec { double t = 0; std::map<std::string, double> f; };

double srtTime(const std::string& s) {
    int h = 0, m = 0, sec = 0, ms = 0;
    if (std::sscanf(s.c_str(), "%d:%d:%d%*[,.]%d", &h, &m, &sec, &ms) < 3) return -1;
    return h * 3600.0 + m * 60.0 + sec + ms / 1000.0;
}

// The fields of one block's text (tags stripped).
std::map<std::string, double> srtFields(std::string text) {
    std::map<std::string, double> f;
    text = std::regex_replace(text, std::regex("<[^>]*>"), " ");
    static const std::regex kv("([A-Za-z_][A-Za-z_.]*)\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    for (auto it = std::sregex_iterator(text.begin(), text.end(), kv); it != std::sregex_iterator(); ++it) {
        std::string k = (*it)[1];
        std::transform(k.begin(), k.end(), k.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        if (k == "longtitude") k = "longitude";
        if (k == "gimbal_pitch") k = "gb_pitch";
        if (k == "gimbal_roll") k = "gb_roll";
        if (k == "gimbal_yaw") k = "gb_yaw";
        f[k] = std::atof(std::string((*it)[2]).c_str());
    }
    static const std::regex gps("GPS\\s*\\(\\s*(-?[0-9.]+)\\s*,\\s*(-?[0-9.]+)\\s*(?:,\\s*(-?[0-9.]+))?\\s*\\)");
    std::smatch m;
    if (std::regex_search(text, m, gps)) {                // GPS(lon, lat, alt): DJI's order
        f["longitude"] = std::atof(std::string(m[1]).c_str());
        f["latitude"] = std::atof(std::string(m[2]).c_str());
    }
    static const std::regex hgt("(?:^|[\\s,])H\\s+(-?[0-9.]+)\\s*m");
    if (!f.count("rel_alt") && std::regex_search(text, m, hgt)) f["rel_alt"] = std::atof(std::string(m[1]).c_str());
    if (!f.count("rel_alt") && f.count("barometer")) f["rel_alt"] = f["barometer"];
    // (0, 0) is "no fix", not a place.
    if (f.count("latitude") && f.count("longitude") && f["latitude"] == 0.0 && f["longitude"] == 0.0) {
        f.erase("latitude"); f.erase("longitude");
    }
    return f;
}

std::vector<SrtRec> parseSrt(const std::string& all) {
    std::vector<SrtRec> out;
    std::istringstream in(all);
    std::string line, text;
    double t = -1;
    auto flush = [&]() {
        if (t >= 0) { SrtRec r; r.t = t; r.f = srtFields(text); out.push_back(r); }
        t = -1; text.clear();
    };
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const size_t arrow = line.find("-->");
        if (arrow != std::string::npos) { flush(); t = srtTime(line.substr(0, arrow)); continue; }
        if (t >= 0) text += line + "\n";
    }
    flush();
    std::stable_sort(out.begin(), out.end(), [](const SrtRec& a, const SrtRec& b) { return a.t < b.t; });
    return out;
}

// A field at video time t: linear between the blocks that have it.
bool srtAt(const std::vector<SrtRec>& s, const char* key, double t, double& v) {
    const SrtRec* a = nullptr; const SrtRec* b = nullptr;
    for (const SrtRec& r : s) {
        if (!r.f.count(key)) continue;
        if (r.t <= t) a = &r; else { b = &r; break; }
    }
    if (!a && !b) return false;
    if (!a) { v = b->f.at(key); return true; }
    if (!b) { v = a->f.at(key); return true; }
    const double w = (t - a->t) / std::max(1e-9, b->t - a->t);
    const double va = a->f.at(key), vb = b->f.at(key);
    if (std::strstr(key, "yaw")) {                       // headings wrap
        double d = vb - va;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        v = va + w * d;
    } else {
        v = va + w * (vb - va);
    }
    return true;
}

bool srtHas(const std::vector<SrtRec>& s, const char* key) {
    for (const SrtRec& r : s) if (r.f.count(key)) return true;
    return false;
}

// Latitude/longitude -> metres east/north of (lat0, lon0): local and flat,
// which is all a flight of a few hundred metres needs.
void enuOf(double lat, double lon, double lat0, double lon0, double& e, double& n) {
    const double R = 6378137.0;
    n = rad(lat - lat0) * R;
    e = rad(lon - lon0) * R * std::cos(rad(lat0));
}

// time,height rows; anything that does not start with two numbers is skipped.
bool loadHeightCsv(const std::string& path, std::vector<std::pair<double, double>>& rows) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    while (std::getline(in, line)) {
        for (char& c : line) if (c == ',' || c == ';' || c == '\t') c = ' ';
        double t, h;
        if (std::sscanf(line.c_str(), "%lf %lf", &t, &h) == 2) rows.emplace_back(t, h);
    }
    std::sort(rows.begin(), rows.end());
    return !rows.empty();
}

double interpRows(const std::vector<std::pair<double, double>>& r, double t) {
    if (t <= r.front().first) return r.front().second;
    if (t >= r.back().first) return r.back().second;
    const auto it = std::lower_bound(r.begin(), r.end(), std::make_pair(t, -1e300));
    const auto& b = *it; const auto& a = *(it - 1);
    return a.second + (t - a.first) / std::max(1e-9, b.first - a.first) * (b.second - a.second);
}

// Rotation about the world vertical from Ra to Rb, degrees, in CamPose's yaw
// sense (clockwise from North): rotUp(d) * Ra = Rb (vio.cpp).
double yawBetween(const cv::Matx33d& Ra, const cv::Matx33d& Rb) {
    const cv::Matx33d M = Rb * Ra.t();
    return deg(std::atan2(M(0, 1), M(0, 0)));
}

// ---------------------------------------------------------------------------
// The lens.
// ---------------------------------------------------------------------------
struct Lens {
    std::string model = "pinhole";   // pinhole | fisheye
    int w = 0, h = 0;                // the resolution K is for
    cv::Matx33d K = cv::Matx33d::eye();
    cv::Mat D;
    double rms = 0;
    int views = 0;
};

bool loadLens(const std::string& path, Lens& L, std::string& err) {
    cv::FileStorage fs;
    try { fs.open(path, cv::FileStorage::READ); } catch (const cv::Exception& e) { err = e.what(); return false; }
    if (!fs.isOpened()) { err = "cannot read " + path; return false; }
    cv::Mat K;
    fs["model"] >> L.model;
    fs["image_width"] >> L.w;
    fs["image_height"] >> L.h;
    fs["K"] >> K;
    fs["D"] >> L.D;
    fs["rms_px"] >> L.rms;
    if (K.rows != 3 || K.cols != 3 || L.w <= 0 || L.h <= 0) { err = path + ": no K / image size"; return false; }
    K.convertTo(K, CV_64F);
    L.K = cv::Matx33d(K);
    L.D.convertTo(L.D, CV_64F);
    if (L.model != "pinhole" && L.model != "fisheye") { err = path + ": model must be pinhole or fisheye"; return false; }
    return true;
}

// Undistortion maps at processing size (pw, ph) and the pinhole they produce.
// Only valid pixels are kept (alpha / balance 0): the edge of a fisheye
// stretched to a pinhole is mostly interpolation, not image.
void undistortMaps(const Lens& L, int pw, int ph, cv::Mat& m1, cv::Mat& m2, cv::Matx33d& Knew) {
    cv::Matx33d Ks = L.K;
    const double sx = double(pw) / L.w, sy = double(ph) / L.h;
    Ks(0, 0) *= sx; Ks(0, 2) *= sx; Ks(1, 1) *= sy; Ks(1, 2) *= sy;
    const cv::Size sz(pw, ph);
    cv::Mat Kn;
    if (L.model == "fisheye") {
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cv::Mat(Ks), L.D, sz, cv::Matx33d::eye(), Kn, 0.0, sz);
        cv::fisheye::initUndistortRectifyMap(cv::Mat(Ks), L.D, cv::Matx33d::eye(), Kn, sz, CV_16SC2, m1, m2);
    } else {
        Kn = cv::getOptimalNewCameraMatrix(cv::Mat(Ks), L.D, sz, 0.0, sz);
        cv::initUndistortRectifyMap(cv::Mat(Ks), L.D, cv::Mat(), Kn, sz, CV_16SC2, m1, m2);
    }
    Kn.convertTo(Kn, CV_64F);
    Knew = cv::Matx33d(Kn);
}

// ---------------------------------------------------------------------------
// --calibrate: OpenCV's checkerboard calibration over a clip. A board on a
// laptop screen is fine (flat, sharp, and its square size only scales the
// extrinsics, which are thrown away). Fill the frame's edges and corners
// with it, at several tilts: the distortion lives at the edges.
// ---------------------------------------------------------------------------
int calibrate(const Opt& o) {
    cv::VideoCapture cap(o.calibVideo);
    if (!cap.isOpened()) { std::fprintf(stderr, "footage_vio: cannot open %s\n", o.calibVideo.c_str()); return 2; }
    const double nf = cap.get(cv::CAP_PROP_FRAME_COUNT);
    const int every = o.every > 1 ? o.every : std::max(1, int(nf / 300.0));
    const cv::Size board(o.boardW, o.boardH);
    std::vector<std::vector<cv::Point2f>> img;
    std::vector<cv::Point2f> last;
    cv::Size sz;
    cv::Mat bgr, g;
    int idx = 0, tried = 0, found = 0;
    for (; cap.read(bgr); ++idx) {
        if (idx % every) continue;
        // Detect at no more than 1280 wide: a 4K checkerboard search is slow,
        // and K scales with the image (the run rescales it to its own size).
        const double s = std::min(1.0, 1280.0 / bgr.cols);
        cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
        if (s < 1.0) cv::resize(g, g, cv::Size(), s, s, cv::INTER_AREA);
        sz = g.size();
        ++tried;
        std::vector<cv::Point2f> c;
        if (!cv::findChessboardCorners(g, board, c, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
                                                        cv::CALIB_CB_FAST_CHECK))
            continue;
        ++found;
        cv::cornerSubPix(g, c, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
        // A new VIEW only if the board moved: forty copies of one pose
        // calibrate nothing but that pose.
        if (!last.empty()) {
            double d = 0;
            for (size_t i = 0; i < c.size(); ++i) d += std::hypot(c[i].x - last[i].x, c[i].y - last[i].y);
            if (d / double(c.size()) < 0.03 * g.cols) continue;
        }
        last = c;
        img.push_back(c);
    }
    std::printf("calibrate: %d frames looked at, board %dx%d found in %d, %zu distinct views\n", tried,
                o.boardW, o.boardH, found, img.size());
    if (img.size() < 8) {
        std::fprintf(stderr, "footage_vio: need at least 8 distinct views of the board (got %zu)\n", img.size());
        return 1;
    }
    if (int(img.size()) > o.maxViews) {                 // evenly thinned
        std::vector<std::vector<cv::Point2f>> t;
        for (int i = 0; i < o.maxViews; ++i) t.push_back(img[size_t(i * img.size() / o.maxViews)]);
        img.swap(t);
    }
    std::vector<cv::Point3f> b;
    for (int y = 0; y < o.boardH; ++y)
        for (int x = 0; x < o.boardW; ++x) b.emplace_back(x * o.squareMm * 1e-3f, y * o.squareMm * 1e-3f, 0.f);
    std::vector<std::vector<cv::Point3f>> obj(img.size(), b);
    Lens L;
    L.model = o.fisheye ? "fisheye" : "pinhole";
    L.w = sz.width; L.h = sz.height; L.views = int(img.size());
    cv::Mat K, D;
    std::vector<cv::Mat> rv, tv;
    try {
        if (o.fisheye) {
            L.rms = cv::fisheye::calibrate(obj, img, sz, K, D, rv, tv,
                                           cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW,
                                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
        } else {
            L.rms = cv::calibrateCamera(obj, img, sz, K, D, rv, tv);
        }
    } catch (const cv::Exception& e) {
        std::fprintf(stderr, "footage_vio: calibration failed: %s\n", e.what());
        return 1;
    }
    K.convertTo(K, CV_64F);
    L.K = cv::Matx33d(K);
    D.convertTo(L.D, CV_64F);
    cv::FileStorage fs(o.calibOut, cv::FileStorage::WRITE);
    fs << "model" << L.model << "image_width" << L.w << "image_height" << L.h << "K" << K << "D" << L.D
       << "rms_px" << L.rms << "views" << L.views << "board" << (std::to_string(o.boardW) + "x" + std::to_string(o.boardH));
    fs.release();
    cv::Mat m1, m2;
    cv::Matx33d Kn;
    undistortMaps(L, L.w, L.h, m1, m2, Kn);
    std::printf("calibrate: %s model, %d views, reprojection RMS %.3f px at %dx%d\n", L.model.c_str(), L.views,
                L.rms, L.w, L.h);
    std::printf("  fx %.1f fy %.1f cx %.1f cy %.1f   D", L.K(0, 0), L.K(1, 1), L.K(0, 2), L.K(1, 2));
    for (int i = 0; i < int(L.D.total()); ++i) std::printf(" %.4f", L.D.at<double>(i));
    std::printf("\n  after correction: horizontal FOV %.1f deg (pinhole, valid pixels only)\n",
                deg(2 * std::atan(0.5 * L.w / Kn(0, 0))));
    std::printf("  written to %s%s\n", o.calibOut.c_str(),
                L.rms > 1.0 ? "   (RMS above 1 px: blur, or a board that is not flat -- reshoot?)" : "");
    return 0;
}

// ---------------------------------------------------------------------------
// The plot: top-down, north up when GPS fixed the heading, metres.
// ---------------------------------------------------------------------------
void drawPlot(const std::string& path, const std::vector<cv::Point2d>& vio, const std::vector<char>& valid,
              const std::vector<cv::Point2d>& gps, const std::vector<std::string>& lines, bool northUp) {
    const int W = 900, H = 900, top = 30 + 22 * int(lines.size()), pad = 50;
    cv::Mat im(H, W, CV_8UC3, cv::Scalar(255, 255, 255));
    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    auto grow = [&](const std::vector<cv::Point2d>& v) {
        for (const auto& p : v) { x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y); }
    };
    grow(vio); grow(gps);
    if (x0 > x1) { x0 = y0 = -1; x1 = y1 = 1; }
    const double span = std::max({x1 - x0, y1 - y0, 2.0}) * 1.1;
    const double cx = 0.5 * (x0 + x1), cy = 0.5 * (y0 + y1);
    const double sc = std::min(W - 2 * pad, H - top - pad) / span;
    const int ox = W / 2, oy = top + (H - top - pad) / 2;
    auto P = [&](const cv::Point2d& p) { return cv::Point(int(ox + (p.x - cx) * sc), int(oy - (p.y - cy) * sc)); };
    double step = 1;                                     // grid: 1-2-5
    for (double s : {1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0})
        if (span / s <= 12) { step = s; break; }
    for (double g = std::floor((cx - span / 2) / step) * step; g <= cx + span / 2; g += step) {
        const int x = P({g, 0}).x;
        cv::line(im, {x, top}, {x, H - pad}, cv::Scalar(232, 232, 232), 1);
        cv::putText(im, cv::format("%g", g), {x + 2, H - pad + 14}, cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
    }
    for (double g = std::floor((cy - span / 2) / step) * step; g <= cy + span / 2; g += step) {
        const int y = P({0, g}).y;
        if (y < top) continue;
        cv::line(im, {pad, y}, {W - pad, y}, cv::Scalar(232, 232, 232), 1);
        cv::putText(im, cv::format("%g", g), {4, y - 2}, cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
    }
    const cv::Scalar cG(0, 140, 255), cV(200, 90, 20), cL(40, 40, 220);
    for (size_t i = 1; i < gps.size(); ++i) cv::line(im, P(gps[i - 1]), P(gps[i]), cG, 2, cv::LINE_AA);
    for (size_t i = 1; i < vio.size(); ++i) cv::line(im, P(vio[i - 1]), P(vio[i]), cV, 2, cv::LINE_AA);
    for (size_t i = 0; i < vio.size(); ++i) if (!valid[i]) cv::circle(im, P(vio[i]), 2, cL, -1, cv::LINE_AA);
    if (!vio.empty()) {
        cv::circle(im, P(vio.front()), 7, cv::Scalar(60, 160, 60), 2, cv::LINE_AA);
        cv::drawMarker(im, P(vio.back()), cV, cv::MARKER_TILTED_CROSS, 14, 2, cv::LINE_AA);
    }
    if (!gps.empty()) cv::drawMarker(im, P(gps.back()), cG, cv::MARKER_TILTED_CROSS, 14, 2, cv::LINE_AA);
    int y = 22;
    for (const std::string& l : lines) {
        cv::putText(im, l, {12, y}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
        y += 22;
    }
    int lx = W - 250, ly = top + 18;
    cv::line(im, {lx, ly - 4}, {lx + 30, ly - 4}, cV, 2); cv::putText(im, "VIO", {lx + 38, ly}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cV, 1, cv::LINE_AA);
    if (!gps.empty()) {
        ly += 18;
        cv::line(im, {lx, ly - 4}, {lx + 30, ly - 4}, cG, 2); cv::putText(im, "GPS (subtitles)", {lx + 38, ly}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cG, 1, cv::LINE_AA);
    }
    ly += 18;
    cv::circle(im, {lx + 15, ly - 4}, 3, cL, -1); cv::putText(im, "VIO lost", {lx + 38, ly}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cL, 1, cv::LINE_AA);
    ly += 18;
    cv::circle(im, {lx + 15, ly - 4}, 6, cv::Scalar(60, 160, 60), 2); cv::putText(im, "start", {lx + 38, ly}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    cv::putText(im, northUp ? "metres, north up" : "metres, VIO frame (heading at start = --start-yaw)",
                {pad, H - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
    cv::imwrite(path, im);
}

// ---------------------------------------------------------------------------
// --selftest: the parts here that are not the estimators -- the subtitle
// parser on the layouts DJI has used, the yaw sign against navcore, and the
// local projection.
// ---------------------------------------------------------------------------
int selfTest() {
    int bad = 0;
    auto check = [&](bool ok, const char* what) {
        std::printf("  %-58s %s\n", what, ok ? "ok" : "FAIL");
        if (!ok) ++bad;
    };
    const std::string a =
        "1\r\n00:00:00,000 --> 00:00:00,033\r\n<font size=\"28\">FrameCnt: 1, DiffTime: 33ms\r\n"
        "2024-05-06 12:00:00.000\r\n[iso: 100] [shutter: 1/500.0] [fnum: 2.8] [ev: 0] [focal_len: 24.00] "
        "[latitude: 60.170000] [longitude: 24.940000] [rel_alt: 5.200 abs_alt: 30.100] "
        "[gb_yaw: 170.0 gb_pitch: -90.0 gb_roll: 0.0] [ct: 5500] </font>\r\n\r\n"
        "2\r\n00:00:00,033 --> 00:00:00,066\r\n<font size=\"28\">FrameCnt: 2, DiffTime: 33ms\r\n"
        "2024-05-06 12:00:00.033\r\n[latitude: 60.170100] [longitude: 24.940000] [rel_alt: 5.400 abs_alt: 30.300] "
        "[gb_yaw: -170.0 gb_pitch: -88.0 gb_roll: 0.0] </font>\r\n";
    const auto s = parseSrt(a);
    double v;
    check(s.size() == 2 && std::fabs(s[1].t - 0.033) < 1e-9, "DJI Mini/Air layout: two blocks, times");
    check(srtAt(s, "latitude", 0.0, v) && std::fabs(v - 60.17) < 1e-9, "latitude");
    check(srtAt(s, "rel_alt", 0.0165, v) && std::fabs(v - 5.3) < 1e-6, "rel_alt, interpolated half way");
    check(s[0].f.count("abs_alt") && std::fabs(s[0].f.at("abs_alt") - 30.1) < 1e-9, "two fields in one bracket");
    check(srtAt(s, "gb_pitch", 0.0, v) && v == -90.0, "gb_pitch");
    check(srtAt(s, "gb_yaw", 0.0165, v) && std::fabs(std::fabs(v) - 180.0) < 1e-6, "gb_yaw interpolates across +-180");
    const auto o1 = parseSrt("1\n00:00:01,000 --> 00:00:02,000\nF/2.8, SS 320, ISO 100, EV 0, "
                             "GPS (24.9400, 60.1700, 19), D 25.3m, H 7.5m, H.S 0.0m/s\n");
    check(o1.size() == 1 && o1[0].f.count("latitude") && std::fabs(o1[0].f.at("latitude") - 60.17) < 1e-9 &&
              std::fabs(o1[0].f.at("longitude") - 24.94) < 1e-9, "old layout: GPS(lon, lat, alt)");
    check(o1.size() == 1 && o1[0].f.count("rel_alt") && std::fabs(o1[0].f.at("rel_alt") - 7.5) < 1e-9,
          "old layout: H 7.5m");
    const auto o2 = parseSrt("1\n00:00:00,000 --> 00:00:00,033\n[latitude : 0.000000] [longtitude : 0.000000]\n"
                             "2\n00:00:00,033 --> 00:00:00,066\n[latitude : 60.1] [longtitude : 24.9]\n");
    check(o2.size() == 2 && !o2[0].f.count("latitude") && o2[1].f.count("longitude"),
          "\"longtitude\", and (0, 0) = no fix");
    // Yaw: +10 deg in CamPose terms must read +10 here, straight down and oblique.
    for (float p : {-90.f, -60.f}) {
        sim::CamPose c0, c1;
        c0.pitchDeg = c1.pitchDeg = p; c0.yawDeg = 30.f; c1.yawDeg = 40.f;
        const double d = yawBetween(downvo::rotOf(c0), downvo::rotOf(c1));
        check(std::fabs(d - 10.0) < 1e-3, p < -89 ? "yaw between frames, camera straight down" : "yaw between frames, camera at -60");
    }
    double e, n;
    enuOf(60.171, 24.942, 60.17, 24.94, e, n);
    check(std::fabs(n - 111.3) < 0.5 && std::fabs(e - 111.3 * 2 * std::cos(rad(60.17))) < 0.5, "lat/lon -> metres");
    std::printf("footage_vio selftest: %s\n", bad ? "FAILED" : "ok");
    return bad ? 1 : 0;
}

struct Rec {
    double t = 0;
    bool valid = false;
    int tracked = 0, inliers = 0;
    double e = 0, n = 0, u = 0, yaw = 0, H = 0;
    bool gps = false;
    double ge = 0, gn = 0, galt = 0;
};

// Path length of a track, sampled every `everyS` seconds: hover jitter at
// frame rate is not distance flown.
double pathLen(const std::vector<Rec>& r, bool gps, double everyS) {
    double L = 0, lt = -1e9, le = 0, ln = 0;
    bool have = false;
    for (const Rec& x : r) {
        if (gps && !x.gps) continue;
        if (have && x.t - lt < everyS) continue;
        const double e = gps ? x.ge : x.e, n = gps ? x.gn : x.n;
        if (have) L += std::hypot(e - le, n - ln);
        le = e; ln = n; lt = x.t; have = true;
    }
    return L;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--selftest") == 0) return selfTest();
    Opt o;
    if (argc < 2) { usage(); return 2; }
    if (!parse(argc, argv, o)) return 2;
    if (!o.calibVideo.empty()) return calibrate(o);
    if (o.video.empty()) { std::fprintf(stderr, "footage_vio: --video FILE (or --calibrate VIDEO); --help\n"); return 2; }
    if (o.attitude != "gimbal" && o.attitude != "srt" && o.attitude != "vision") {
        std::fprintf(stderr, "footage_vio: --attitude takes gimbal|srt|vision\n");
        return 2;
    }
    if ((o.hfov > 0) == !o.calib.empty()) {
        std::fprintf(stderr, "footage_vio: give the lens: --hfov DEG or --calib FILE (one of them)\n");
        return 2;
    }
    if (o.height > 0 && !o.heightCsv.empty()) {
        std::fprintf(stderr, "footage_vio: --height and --height-csv both given; pick one\n");
        return 2;
    }

    // Telemetry.
    std::vector<SrtRec> srt;
    if (!o.noSrt) {
        std::string path = o.srt;
        if (path.empty()) {
            const size_t dot = o.video.find_last_of('.');
            const std::string stem = dot == std::string::npos ? o.video : o.video.substr(0, dot);
            for (const char* ext : {".SRT", ".srt"})
                if (std::ifstream(stem + ext)) { path = stem + ext; break; }
        }
        if (!path.empty()) {
            std::ifstream in(path, std::ios::binary);
            if (!in) { std::fprintf(stderr, "footage_vio: cannot read %s\n", path.c_str()); return 2; }
            std::stringstream ss; ss << in.rdbuf();
            srt = parseSrt(ss.str());
            std::set<std::string> keys;
            for (const SrtRec& r : srt) for (const auto& kv : r.f) keys.insert(kv.first);
            std::string ks;
            for (const char* k : {"latitude", "longitude", "rel_alt", "abs_alt", "gb_pitch", "gb_roll", "gb_yaw"})
                ks += std::string(" ") + k + (keys.count(k) ? "" : "(-)");
            if (!o.quiet) std::printf("subtitles: %s, %zu blocks; fields:%s\n", path.c_str(), srt.size(), ks.c_str());
        }
    }
    const bool haveGps = srtHas(srt, "latitude") && srtHas(srt, "longitude");
    const bool srtGimbal = srtHas(srt, "gb_pitch");
    const bool srtYaw = srtHas(srt, "gb_yaw");
    if (o.attitude == "srt" && !srtGimbal) {
        std::fprintf(stderr, "footage_vio: --attitude srt, but the subtitles carry no gb_pitch\n");
        return 2;
    }

    // Height source.
    std::vector<std::pair<double, double>> hcsv;
    std::string hsrc;
    if (o.height > 0) hsrc = "constant";
    else if (!o.heightCsv.empty()) {
        if (!loadHeightCsv(o.heightCsv, hcsv)) { std::fprintf(stderr, "footage_vio: no time,height rows in %s\n", o.heightCsv.c_str()); return 2; }
        hsrc = "csv";
    } else if (srtHas(srt, "rel_alt")) hsrc = "srt";
    else {
        std::fprintf(stderr, "footage_vio: no height: give --height M (or --height-csv, or subtitles with rel_alt)\n");
        return 2;
    }
    auto heightAt = [&](double t) {
        double h = o.height;
        if (hsrc == "csv") h = interpRows(hcsv, t + o.heightTimeOffset);
        else if (hsrc == "srt") srtAt(srt, "rel_alt", t, h);
        return h + o.heightOffset;
    };

    cv::VideoCapture cap(o.video);
    if (!cap.isOpened()) { std::fprintf(stderr, "footage_vio: cannot open %s\n", o.video.c_str()); return 2; }
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (!(fps > 1 && fps < 1000)) fps = 30;
    const double nFrames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cv::Mat bgr;
    if (!cap.read(bgr) || bgr.empty()) { std::fprintf(stderr, "footage_vio: no frames in %s\n", o.video.c_str()); return 2; }
    const int nw = bgr.cols, nh = bgr.rows;
    const int pw = std::min(o.width, nw) & ~1;
    const int ph = int(std::lround(double(nh) * pw / nw)) & ~1;

    // The lens at processing size.
    sim::CamParams cp;
    cp.width = pw; cp.height = ph;
    cv::Mat map1, map2;
    if (!o.calib.empty()) {
        Lens L;
        std::string err;
        if (!loadLens(o.calib, L, err)) { std::fprintf(stderr, "footage_vio: %s\n", err.c_str()); return 2; }
        if (std::fabs(double(L.w) / L.h - double(nw) / nh) > 0.01)
            std::fprintf(stderr, "footage_vio: WARNING: calibration is %dx%d, the video %dx%d -- a different "
                                 "aspect is a different video mode (crop), and its lens model does not apply\n",
                         L.w, L.h, nw, nh);
        cv::Matx33d Kn;
        undistortMaps(L, pw, ph, map1, map2, Kn);
        cp.fxPx = float(Kn(0, 0)); cp.fyPx = float(Kn(1, 1));
        cp.ppxPx = float(Kn(0, 2)); cp.ppyPx = float(Kn(1, 2));
        cp.hfovDeg = float(deg(2 * std::atan(0.5 * pw / Kn(0, 0))));
    } else {
        cp.hfovDeg = o.hfov;
    }
    const sim::DepthCamera cam(cp);
    const double hfovEff = deg(2 * std::atan(0.5 * pw / cam.fpx()));

    // Estimators. The height bound for the depth band is the largest the run
    // will see (from the source, sampled).
    double Hmax = 0;
    for (double t = o.start; t <= (o.end > 0 ? o.end : nFrames / fps) + 1e-9; t += 0.5) Hmax = std::max(Hmax, heightAt(t));
    Hmax = std::max(Hmax, heightAt(o.start));
    if (!(Hmax > 0.2)) {
        std::fprintf(stderr, "footage_vio: height %.2f m at the start -- below 0.2 m nothing here is valid "
                             "(rel_alt is relative to takeoff: --height-offset?)\n", Hmax);
        return 2;
    }
    const double fpsRun = fps / o.every;
    sim::VioParams vp;
    vp.maxDepthM = float(std::max(15.0, 4.0 * Hmax));
    vp.maxJumpM = std::max(vp.maxJumpM, float(2.5 * o.maxSpeed / fpsRun));
    // No gyro in a video file: the yaw prior below is a guess (a constant
    // rate, or the subtitles' gimbal yaw at 0.1 deg), so it is weighted as one.
    vp.gyroYawSigmaDeg = (o.attitude == "srt" && srtYaw) ? 1.f : 3.f;
    vp.maxYawDevDeg = 12.f;
    sim::DepthVio vio;
    vio.init(cam, vp);
    downvo::PnpVo pvo;
    pvo.init(cam, vp.maxDepthM, vp.maxJumpM);

    if (!o.quiet)
        std::printf("video: %s  %dx%d @ %.2f fps (%.0f frames); processing %dx%d, every %d -> %.2f fps; "
                    "hfov %.1f deg\nheight: %s%s; attitude: %s (gimbal %.0f deg)\n",
                    o.video.c_str(), nw, nh, fps, nFrames, pw, ph, o.every, fpsRun, hfovEff,
                    hsrc == "constant" ? cv::format("%.2f m constant", o.height).c_str()
                                       : hsrc == "csv" ? ("from " + o.heightCsv).c_str() : "subtitles' rel_alt",
                    o.heightOffset != 0 ? cv::format(" %+.2f m", o.heightOffset).c_str() : "",
                    o.attitude.c_str(), o.gimbalPitch);

    std::vector<Rec> recs;
    std::vector<double> estMs;
    double lat0 = 0, lon0 = 0;
    bool haveOrigin = false;
    double gyroYaw = 0, rate = 0, prevSrtYaw = 0;
    bool prevValid = false;
    cv::Matx33d prevR;
    cv::Mat depth, gray, small;
    double depthKeyP = 1e9, depthKeyR = 1e9, depthKeyH = -1;
    const double tEnd = o.end > 0 ? o.end : 1e18;
    double shotT = -1;
    if (!o.shot.empty()) shotT = o.start + 0.5 * (std::min(tEnd, nFrames / fps) - o.start);
    int idx = 0, first = -1;
    do {
        const double t = idx / fps;
        if (t + 1e-9 < o.start) continue;
        if (t > tEnd) break;
        if (first < 0) first = idx;
        if ((idx - first) % o.every) continue;

        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        cv::resize(gray, small, cv::Size(pw, ph), 0, 0, cv::INTER_AREA);
        if (!map1.empty()) { cv::Mat u; cv::remap(small, u, map1, map2, cv::INTER_LINEAR); small = u; }

        // The camera's attitude for this frame.
        double pitch = o.gimbalPitch, roll = o.gimbalRoll, v;
        if (o.attitude == "srt") {
            if (srtAt(srt, "gb_pitch", t, v)) pitch = v;
            if (srtAt(srt, "gb_roll", t, v)) roll = v;
        }
        const double H = heightAt(t);
        const bool firstFrame = recs.empty();
        if (firstFrame) {
            double y0 = 0;
            if (o.startYawSet) gyroYaw = o.startYaw;
            else if (srtYaw && srtAt(srt, "gb_yaw", t, y0)) gyroYaw = y0;
            if (srtYaw) srtAt(srt, "gb_yaw", t, prevSrtYaw);
        } else if (o.attitude == "srt" && srtYaw) {
            double y; srtAt(srt, "gb_yaw", t, y);         // the gimbal's yaw CHANGE is the gyro
            double d = y - prevSrtYaw;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            gyroYaw += d; prevSrtYaw = y;
        } else {
            gyroYaw += rate;                              // constant-rate stand-in for a gyro
        }
        sim::CamPose att;
        att.yawDeg = float(gyroYaw); att.pitchDeg = float(pitch); att.rollDeg = float(roll);
        att.u = float(H);

        Rec r;
        r.t = t; r.H = H;
        const auto te0 = std::chrono::steady_clock::now();
        if (o.attitude == "vision") {
            if (firstFrame) pvo.reset(downvo::rotOf(att), cv::Vec3d(0, 0, H));
            r.valid = pvo.step(small, [&](const cv::Matx33d& Rwc, const cv::Vec3d&) {
                return downvo::planeDepth(cam, pw, ph, Rwc, H);
            });
            r.tracked = pvo.tracked(); r.inliers = pvo.inliers();
            r.e = pvo.C()[0]; r.n = pvo.C()[1]; r.u = pvo.C()[2];
            const cv::Matx33d R = pvo.R();
            r.yaw = firstFrame ? gyroYaw : recs.back().yaw + yawBetween(prevR, R);
            prevR = R;
        } else {
            if (firstFrame) vio.reset(att);
            // The plane depth depends on the tilt and the height only: reuse it.
            if (std::fabs(pitch - depthKeyP) > 1e-3 || std::fabs(roll - depthKeyR) > 1e-3 ||
                std::fabs(H - depthKeyH) > 1e-3 * H) {
                depth = downvo::planeDepth(cam, pw, ph, downvo::rotOf(att), H);
                depthKeyP = pitch; depthKeyR = roll; depthKeyH = H;
            }
            const sim::VioResult res = vio.step(small, depth, att);
            r.valid = res.valid; r.tracked = res.tracked; r.inliers = res.inliers;
            r.e = vio.pose().e; r.n = vio.pose().n; r.u = vio.pose().u;
            const cv::Matx33d R = downvo::rotOf(vio.pose());
            r.yaw = firstFrame ? gyroYaw : recs.back().yaw + yawBetween(prevR, R);
            // The stand-in gyro learns the turn rate from the VIO's own
            // consecutive measured frames, and lets it fade when lost.
            if (!firstFrame && r.valid && prevValid) rate = 0.5 * rate + 0.5 * yawBetween(prevR, R);
            else if (!r.valid) rate *= 0.85;
            prevR = R;
        }
        estMs.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - te0).count());
        prevValid = r.valid;

        double lat, lon;
        if (haveGps && srtAt(srt, "latitude", t, lat) && srtAt(srt, "longitude", t, lon)) {
            if (!haveOrigin) { lat0 = lat; lon0 = lon; haveOrigin = true; }
            enuOf(lat, lon, lat0, lon0, r.ge, r.gn);
            r.gps = true;
            double a;
            if (srtAt(srt, "rel_alt", t, a)) r.galt = a;
        }
        recs.push_back(r);

        if (shotT >= 0 && t >= shotT) {
            // The processed frame, with what the flat-ground depth would NOT
            // anchor (the sky, rays too far out) dimmed.
            const cv::Mat d = o.attitude == "vision" ? downvo::planeDepth(cam, pw, ph, downvo::rotOf(att), H) : depth;
            cv::Mat out;
            cv::cvtColor(small, out, cv::COLOR_GRAY2BGR);
            for (int y = 0; y < ph; ++y)
                for (int x = 0; x < pw; ++x) {
                    const float z = d.at<float>(y, x);
                    if (!(z >= vp.minDepthM && z <= vp.maxDepthM)) out.at<cv::Vec3b>(y, x) = out.at<cv::Vec3b>(y, x) * 0.35;
                }
            cv::putText(out, cv::format("t %.2f s  %dx%d  hfov %.1f  height %.2f m  pitch %.0f", t, pw, ph, hfovEff, H, pitch),
                        {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
            cv::imwrite(o.shot, out);
            shotT = -1;
        }
    } while (++idx, cap.read(bgr));

    if (recs.size() < 2) { std::fprintf(stderr, "footage_vio: fewer than 2 frames in the range\n"); return 2; }

    // --- Report ---
    int nValid = 0; double sumIn = 0;
    for (size_t i = 1; i < recs.size(); ++i) if (recs[i].valid) { ++nValid; sumIn += recs[i].inliers; }
    const double validPct = 100.0 * nValid / double(recs.size() - 1);
    const double meanIn = nValid ? sumIn / nValid : 0;
    const Rec& R0 = recs.front();
    const Rec& R1 = recs.back();
    const double path = pathLen(recs, false, 0.5);
    const double closure = std::hypot(R1.e - R0.e, R1.n - R0.n);
    // A CLOSED LOOP HIDES A SCALE ERROR: a wrong height stretches the way
    // out and the way back alike, and the loop still closes. What shows it
    // without GPS is a distance measured on the ground -- fly out to a taped
    // mark and back, and compare this with the tape.
    double far = 0, farT = 0;
    for (const Rec& x : recs) {
        const double d = std::hypot(x.e - R0.e, x.n - R0.n);
        if (d > far) { far = d; farT = x.t; }
    }
    std::vector<double> ms = estMs;
    std::sort(ms.begin(), ms.end());
    double msSum = 0; for (double x : ms) msSum += x;
    const double msMean = msSum / double(ms.size()), msP95 = ms[ms.size() * 95 / 100];

    // Against GPS: one rotation about the start, no scale (the scale is what
    // is being tested), then the scale that WOULD fit, reported on its own.
    int nG = 0;
    double sDot = 0, sCross = 0, sPP = 0;
    for (const Rec& x : recs) {
        if (!x.gps) continue;
        const double pe = x.e - R0.e, pn = x.n - R0.n;
        sDot += pe * x.ge + pn * x.gn;
        sCross += pe * x.gn - pn * x.ge;
        sPP += pe * pe + pn * pn;
        ++nG;
    }
    const bool cmp = nG >= 2 && sPP > 1e-6;
    const double th = cmp ? std::atan2(sCross, sDot) : 0.0;
    const double scale = cmp ? std::hypot(sDot, sCross) / sPP : 1.0;   // GPS ~ scale * R * VIO
    double se = 0, maxE = 0, finE = 0;
    auto rot = [&](const Rec& x) {
        const double pe = x.e - R0.e, pn = x.n - R0.n;
        return cv::Point2d(std::cos(th) * pe - std::sin(th) * pn, std::sin(th) * pe + std::cos(th) * pn);
    };
    for (const Rec& x : recs) {
        if (!x.gps) continue;
        const cv::Point2d p = rot(x);
        const double e = std::hypot(p.x - x.ge, p.y - x.gn);
        se += e * e; maxE = std::max(maxE, e); finE = e;
    }
    const double rmsE = cmp ? std::sqrt(se / nG) : 0;
    const double gpsPath = cmp ? pathLen(recs, true, 0.5) : 0;
    const double gpsClosure = cmp ? std::hypot(R1.ge, R1.gn) : 0;
    double Hmean = 0; for (const Rec& x : recs) Hmean += x.H; Hmean /= double(recs.size());
    // Drift: against GPS where there is one, else the start-to-end gap (the
    // drift IF the flight ended over its start).
    const double drift = cmp ? 100.0 * finE / std::max(1e-6, gpsPath) : 100.0 * closure / std::max(1e-6, path);

    std::printf("\nframes %zu over %.1f s, %.1f %% tracked, mean inliers %.0f, estimator %.1f ms/frame (p95 %.1f)\n",
                recs.size(), R1.t - R0.t, validPct, meanIn, msMean, msP95);
    std::printf("VIO: flew %.1f m; start-to-end %.2f m = %.2f %% of the path  "
                "(the drift, IF it ended over its start)\n", path, closure, 100.0 * closure / std::max(1e-6, path));
    std::printf("VIO: furthest from the start %.2f m, at %.1f s  (against a TAPED distance this checks the "
                "scale -- height and lens -- which a closed loop cannot)\n", far, farT);
    if (cmp) {
        std::printf("GPS: flew %.1f m; start-to-end %.2f m\n", gpsPath, gpsClosure);
        std::printf("VIO vs GPS (one rotation about the start, %.1f deg; no scale): RMS %.2f m, max %.2f m, "
                    "final %.2f m = %.2f %% of the path\n", deg(th), rmsE, maxE, finE, drift);
        std::printf("scale GPS/VIO %.3f: a height of %.2f m (mean; used %.2f) would make it 1, if the lens is right\n",
                    scale, Hmean * scale, Hmean);
        if (gpsPath < 20)
            std::printf("  (GPS over %.0f m is metre-noise against metres: the comparison means little)\n", gpsPath);
    } else if (!srt.empty()) {
        std::printf("GPS: none in the subtitles -- the start-to-end gap is the only check\n");
    }

    if (!o.csv.empty()) {
        FILE* f = std::fopen(o.csv.c_str(), "w");
        if (!f) { std::fprintf(stderr, "footage_vio: cannot write %s\n", o.csv.c_str()); return 2; }
        std::fprintf(f, "t_s,valid,tracked,inliers,vio_e,vio_n,vio_u,vio_yaw_deg,height_m,gps,gps_e,gps_n,gps_rel_alt,vio_rot_e,vio_rot_n\n");
        for (const Rec& x : recs) {
            const cv::Point2d p = rot(x);
            std::fprintf(f, "%.4f,%d,%d,%d,%.4f,%.4f,%.4f,%.2f,%.3f,%d,%.3f,%.3f,%.3f,%.4f,%.4f\n", x.t, int(x.valid),
                         x.tracked, x.inliers, x.e, x.n, x.u, x.yaw, x.H, int(x.gps), x.ge, x.gn, x.galt, p.x, p.y);
        }
        std::fclose(f);
    }
    if (!o.plot.empty()) {
        std::vector<cv::Point2d> pv, pg;
        std::vector<char> va;
        for (const Rec& x : recs) {
            pv.push_back(cmp ? rot(x) : cv::Point2d(x.e - R0.e, x.n - R0.n));
            va.push_back(x.valid);
            if (x.gps) pg.emplace_back(x.ge, x.gn);
        }
        const size_t slash = o.video.find_last_of("/\\");
        std::vector<std::string> lines{
            cv::format("%s   %s attitude, height %s %.2f m, hfov %.0f",
                       o.video.substr(slash == std::string::npos ? 0 : slash + 1).c_str(), o.attitude.c_str(),
                       hsrc.c_str(), Hmean, hfovEff),
            cv::format("VIO %.1f m flown, start-to-end %.2f m (%.2f %%), %.0f %% tracked", path, closure,
                       100.0 * closure / std::max(1e-6, path), validPct)};
        if (cmp)
            lines.push_back(cv::format("vs GPS: RMS %.2f m, final %.2f m (%.2f %%), scale %.3f -> height %.2f m",
                                       rmsE, finE, drift, scale, Hmean * scale));
        drawPlot(o.plot, pv, va, pg, lines, cmp);
    }
    if (!o.summary.empty()) {
        const bool fresh = !std::ifstream(o.summary).good();
        FILE* f = std::fopen(o.summary.c_str(), "a");
        if (f) {
            if (fresh)
                std::fprintf(f, "video,attitude,height_src,height_mean_m,height_offset_m,gimbal_pitch,hfov,width,every,"
                                "frames,duration_s,valid_pct,mean_inliers,path_m,closure_m,closure_pct,gps_path_m,"
                                "gps_closure_m,gps_rms_m,gps_max_m,gps_final_m,drift_pct,scale,implied_height_m,ms_per_frame,furthest_m\n");
            std::fprintf(f, "%s,%s,%s,%.3f,%.3f,%.1f,%.2f,%d,%d,%zu,%.2f,%.1f,%.0f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,"
                            "%.2f,%.4f,%.3f,%.2f,%.3f\n",
                         o.video.c_str(), o.attitude.c_str(), hsrc.c_str(), Hmean, double(o.heightOffset),
                         double(o.gimbalPitch), hfovEff, pw, o.every, recs.size(), R1.t - R0.t, validPct, meanIn,
                         path, closure, 100.0 * closure / std::max(1e-6, path), gpsPath, gpsClosure, rmsE, maxE,
                         finE, drift, scale, Hmean * scale, msMean, far);
            std::fclose(f);
        }
    }
    if (o.checkDrift > 0 && !(drift <= o.checkDrift)) {
        std::fprintf(stderr, "footage_vio: drift %.2f %% > %.2f %%\n", drift, double(o.checkDrift));
        return 1;
    }
    return 0;
}
