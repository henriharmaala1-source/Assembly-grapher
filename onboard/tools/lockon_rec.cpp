// ---------------------------------------------------------------------------
// lockon_rec -- field-test recorder for the lock-on tracker.
//
// WHAT IT DOES. Watches an AUX switch on the flight controller. Flip it up and
// the tracker locks onto whatever is at the centre of frame; flip it down and
// it releases. Everything is recorded with the overlay burned in, plus a CSV of
// the same state, so a flight produces something you can watch AND something
// you can plot.
//
// THE POINT IS THE EVIDENCE, not the flying. Nothing here steers the aircraft.
//
// SAFETY -- THIS TOOL NEVER COMMANDS THE AIRCRAFT.
//
// It talks to the FC through MspBackend::tick() and poll() only. tick() drains
// RX and requests telemetry; it does not transmit RC. sendControl() is never
// called from this file, and there is no ControlCmd anywhere in it. The FC
// cannot be driven by this program even if the tracker goes berserk -- which is
// exactly the property you want the first time you fly a new perception module.
//
// The pilot keeps every stick. The AUX switch is an INPUT we observe, not a
// mode we command: pick a spare channel the FC does nothing with, so a flick
// mid-flight cannot change how the aircraft behaves.
//
// WHY A SEPARATE THREAD FOR THE FC. Same reason FcLink has one: a stalled
// camera read must not stall telemetry. Here the consequence is mild (a stale
// AUX reading, so a late lock) rather than an RC timeout, but the fix is the
// same and it keeps the AUX edge honest.
//
// WHICH TRACKER. track::LockTracker (lock_tracker_fused.hpp) -- the CURRENT
// design: cue fusion, anchor + adaptive + keyframe bank, STAPLE histogram cue,
// ego-motion feed-forward, occlusion-aware adaptation, and the zoomed-out
// SEARCHING re-acquire. NOT the older LockOnTracker the flight runtime still
// uses. Per-stage timings are on the overlay because the point of the first
// field test is to learn where the frame budget actually goes on a Pi.
//
//   ./lockon_rec --fc-port=/dev/ttyAMA0 --aux=5 --out=/home/pi/flights
//   ./lockon_rec --fc=none --autolock=3            # bench, no FC
//   ./lockon_rec --cues=none,edge,chroma           # fuse three channels
// ---------------------------------------------------------------------------

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "crop_filters.hpp"
#include "gray_frame.hpp"
#include "lock_tracker_fused.hpp"
#include "msp_backend.hpp"

namespace {

using clock_t_ = std::chrono::steady_clock;

double nowS() {
    static const auto t0 = clock_t_::now();
    return std::chrono::duration<double>(clock_t_::now() - t0).count();
}

std::string stamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[32];
    std::strftime(b, sizeof b, "%Y%m%d-%H%M%S", &tm);
    return b;
}

// --- FC reader -- READ ONLY -------------------------------------------------
// Owns the backend on its own thread and publishes a telemetry snapshot. The
// ONLY calls made on the backend are connect/tick/poll. Adding sendControl here
// would turn a passive recorder into something that can fly the aircraft; do
// not do it. Put control in the real runtime, behind the mode arbiter.
class FcReader {
public:
    bool open(const std::string& port, int baud) {
        fc_ = std::make_unique<MspBackend>();
        if (!fc_->connect(port, baud)) { fc_.reset(); return false; }
        return true;
    }
    bool haveFc() const { return (bool)fc_; }

    void start() {
        if (!fc_) return;
        run_ = true;
        thr_ = std::thread([this] {
            while (run_) {
                fc_->tick();
                FcTelemetry t;
                if (fc_->poll(t)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    tel_ = t;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }
    void stop() {
        run_ = false;
        if (thr_.joinable()) thr_.join();
        if (fc_) fc_->disconnect();
    }

    FcTelemetry snapshot() const {
        std::lock_guard<std::mutex> lk(mu_);
        return tel_;
    }

private:
    std::unique_ptr<MspBackend> fc_;
    mutable std::mutex mu_;
    FcTelemetry tel_{};
    std::atomic<bool> run_{false};
    std::thread thr_;
};

// --- AUX switch with hysteresis --------------------------------------------
// A 3-position switch parked in the middle, or a noisy link, will chatter
// across a single threshold and produce a burst of lock/release events. Two
// thresholds with a dead band in between means the state only changes when the
// operator actually moved the switch.
class AuxSwitch {
public:
    AuxSwitch(int hiUs, int loUs) : hi_(hiUs), lo_(loUs) {}

    // Returns true on the rising edge.
    bool update(int us, bool& outHigh) {
        if (us <= 0) { outHigh = high_; return false; }   // no RC data yet
        const bool was = high_;
        if (us >= hi_)      high_ = true;
        else if (us <= lo_) high_ = false;                // else: hold
        outHigh = high_;
        return high_ && !was;
    }
    bool high() const { return high_; }
    // Falling edge is polled separately by comparing to the caller's own copy;
    // keeping one edge here avoids a second bool that can get out of step.

private:
    int  hi_, lo_;
    bool high_ = false;
};

// --- cv::Mat -> GrayFrame, allocation-free after the first frame ------------
// The returned view borrows these buffers; it must not outlive the next call.
std::vector<float> gLuma, gU, gV;

track::GrayFrame makeFrameView(const cv::Mat& m) {
    const int w = m.cols, h = m.rows, n = w * h;
    const int ch = m.channels();
    if ((int)gLuma.size() != n) gLuma.assign(n, 0.f);
    track::GrayFrame f;
    f.w = w; f.h = h; f.d = gLuma.data();
    if (ch < 3) {
        for (int y = 0; y < h; ++y) {
            const unsigned char* row = m.ptr<unsigned char>(y);
            for (int x = 0; x < w; ++x) gLuma[y * w + x] = float(row[x]);
        }
        return f;                       // luma only: chroma cues skip themselves
    }
    if ((int)gU.size() != n) { gU.assign(n, 0.f); gV.assign(n, 0.f); }
    f.cu = gU.data(); f.cv = gV.data();
    // BT.601, matching what the sensor's own YUV path would give the Kotlin --
    // same coefficients means footage is comparable between the two, not merely
    // similar.
    for (int y = 0; y < h; ++y) {
        const unsigned char* row = m.ptr<unsigned char>(y);
        for (int x = 0; x < w; ++x) {
            const float b = row[x * ch + 0], g = row[x * ch + 1], r = row[x * ch + 2];
            const int o = y * w + x;
            const float Y = 0.299f * r + 0.587f * g + 0.114f * b;
            gLuma[o] = Y;
            gU[o] = 0.564f * (b - Y);
            gV[o] = 0.713f * (r - Y);
        }
    }
    return f;
}

// --- overlay ----------------------------------------------------------------
void putShadowed(cv::Mat& img, const std::string& s, cv::Point p,
                 double scale, cv::Scalar col, int thick = 1) {
    cv::putText(img, s, p + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(0, 0, 0), thick + 1, cv::LINE_AA);
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, col, thick,
                cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    const cv::String keys =
        "{help h        |       | show this }"
        "{camera        | 0     | V4L2 camera index }"
        "{width         | 640   | capture width }"
        "{height        | 480   | capture height }"
        "{fps           | 30    | capture fps request }"
        "{fc            | msp   | msp | none }"
        "{fc-port       | /dev/ttyAMA0 | FC serial port }"
        "{fc-baud       | 115200| FC serial baud }"
        "{aux           | 5     | RC channel index to watch; AUX1 = 4 }"
        "{aux-hi        | 1700  | us at/above which the switch reads HIGH }"
        "{aux-lo        | 1300  | us at/below which it reads LOW }"
        "{cues          | edge,chroma,none | fused channels: none,stretch,edge,threshold,sharpen,chroma }"
        "{boxsize       | 64    | lock box side, px }"
        "{out           | .     | output directory }"
        "{record        | session | session | always | off }"
        "{preroll       | 2.0   | seconds of video kept before each lock }"
        "{fourcc        | MJPG  | writer codec }"
        "{display       | false | show a window (needs a desktop) }"
        "{autolock      | 0     | with --fc=none: lock N seconds after start }"
        ;

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("lockon_rec -- record the lock-on tracker under AUX control");
    if (parser.has("help")) { parser.printMessage(); return 0; }

    const int    camIdx  = parser.get<int>("camera");
    const int    camW    = parser.get<int>("width");
    const int    camH    = parser.get<int>("height");
    const int    camFps  = parser.get<int>("fps");
    const int    auxIdx  = parser.get<int>("aux");
    const int    boxSize = parser.get<int>("boxsize");
    const bool   display = parser.get<bool>("display");
    const double preroll = parser.get<double>("preroll");
    const double autolock= parser.get<double>("autolock");
    const std::string outDir = parser.get<std::string>("out");
    const std::string recMode = parser.get<std::string>("record");

    // Cue set. Each name is a channel the tracker correlates on; they are FUSED,
    // each weighted by its own PSR, so a cue that is useless right now (no chroma
    // signal on a grey target) contributes nothing rather than voting wrong.
    std::vector<track::CropFilter> cues;
    {
        const std::string cs = parser.get<std::string>("cues");
        size_t p0 = 0;
        while (p0 <= cs.size()) {
            const size_t p1 = cs.find(',', p0);
            const std::string one = cs.substr(p0, p1 == std::string::npos ? std::string::npos : p1 - p0);
            track::CropFilter cf;
            if (!one.empty()) {
                if (!track::cropFilterFromName(one.c_str(), cf)) {
                    std::fprintf(stderr, "unknown cue '%s'\n", one.c_str());
                    return 2;
                }
                cues.push_back(cf);
            }
            if (p1 == std::string::npos) break;
            p0 = p1 + 1;
        }
        if (cues.empty()) cues.push_back(track::CropFilter::NONE);
    }

    // ---- camera
    cv::VideoCapture cap;
    cap.open(camIdx, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  camW);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, camH);
    cap.set(cv::CAP_PROP_FPS,          camFps);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "Cannot open camera %d\n", camIdx);
        return 1;
    }
    const int W = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const int H = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::printf("[cam] %dx%d requested %d fps\n", W, H, camFps);

    // ---- FC (read only)
    FcReader fcr;
    const bool wantFc = parser.get<std::string>("fc") != "none";
    if (wantFc) {
        const std::string port = parser.get<std::string>("fc-port");
        if (!fcr.open(port, parser.get<int>("fc-baud"))) {
            std::fprintf(stderr,
                "[fc] cannot open %s -- run with --fc=none to bench without it\n",
                port.c_str());
            return 1;
        }
        fcr.start();
        std::printf("[fc] %s, watching channel %d (AUX%d), hi>=%d lo<=%d us\n",
                    port.c_str(), auxIdx, auxIdx - 3,
                    parser.get<int>("aux-hi"), parser.get<int>("aux-lo"));
        std::printf("[fc] READ ONLY -- this tool never sends RC.\n");
    } else {
        std::printf("[fc] disabled. %s\n",
                    autolock > 0 ? "Auto-lock armed."
                                 : "Press 'l' to lock (needs --display).");
    }

    AuxSwitch aux(parser.get<int>("aux-hi"), parser.get<int>("aux-lo"));
    track::LockTracker trk;
    trk.setCues(cues);
    {
        std::string names;
        for (size_t i = 0; i < cues.size(); ++i)
            names += (i ? "+" : "") + std::string(track::cropFilterName(cues[i]));
        std::printf("[trk] fused cues: %s, box %d px\n", names.c_str(), boxSize);
    }

    // ---- recording
    // Pre-roll: a ring of raw frames so each clip starts BEFORE the lock. What
    // was in frame at the moment the switch flipped is usually the thing you
    // want to see, and it is gone by the time the writer opens.
    const size_t prerollN = (recMode == "session")
                          ? (size_t)std::max(0.0, preroll * camFps) : 0;
    std::deque<cv::Mat> ring;
    cv::VideoWriter writer;
    std::FILE* csv = nullptr;
    int   sessionNo = 0;
    long  framesWritten = 0, framesDropped = 0;
    const int fourcc = cv::VideoWriter::fourcc(
        parser.get<std::string>("fourcc")[0], parser.get<std::string>("fourcc")[1],
        parser.get<std::string>("fourcc")[2], parser.get<std::string>("fourcc")[3]);

    auto openSession = [&](const std::string& why) {
        const std::string base = outDir + "/lockon-" + stamp() + "-" +
                                 std::to_string(sessionNo++);
        if (!writer.open(base + ".avi", fourcc, camFps, cv::Size(W, H), true)) {
            std::fprintf(stderr, "[rec] cannot open %s.avi\n", base.c_str());
            return;
        }
        csv = std::fopen((base + ".csv").c_str(), "w");
        if (csv) std::fprintf(csv,
            "t_s,aux_us,state,cx,cy,w,h,conf,aim_x,aim_y,"
            "t_flow_ms,t_crop_ms,t_cue_ms,fps\n");
        std::printf("[rec] %s.avi  (%s)\n", base.c_str(), why.c_str());
    };
    auto closeSession = [&]() {
        if (writer.isOpened()) writer.release();
        if (csv) { std::fclose(csv); csv = nullptr; }
    };

    if (recMode == "always") openSession("continuous");

    // ---- loop
    const cv::Point centre(W / 2, H / 2);
    double  tPrev = nowS(), fps = 0;
    bool    prevHigh = false;
    bool    manualLock = false;
    const double tStart = nowS();
    long frameNo = 0;

    std::printf("[run] ctrl-C to stop\n");
    for (;;) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::fprintf(stderr, "[cam] read failed\n");
            break;
        }
        const double t = nowS();
        const double dt = t - tPrev; tPrev = t;
        if (dt > 1e-6) fps = 0.9 * fps + 0.1 * (1.0 / dt);
        ++frameNo;

        // ---- what does the switch say?
        int auxUs = 0;
        FcTelemetry tel{};
        if (fcr.haveFc()) {
            tel = fcr.snapshot();
            if (auxIdx >= 0 && auxIdx < tel.rcCount) auxUs = tel.rc[auxIdx];
        } else if (autolock > 0) {
            auxUs = (t - tStart >= autolock) ? 2000 : 1000;
        } else {
            auxUs = manualLock ? 2000 : 1000;
        }

        bool high = false;
        const bool rising = aux.update(auxUs, high);
        const bool falling = prevHigh && !high;
        prevHigh = high;

        // ---- lock / release
        // The tracker works on its own frame type, not cv::Mat -- it is
        // deliberately OpenCV-free so it can be unit-tested headless and share a
        // validation mirror with the Kotlin. Convert once, here, into buffers
        // that are allocated ONCE: fromBgr() would allocate three w*h float
        // planes per frame (3.7 MB at 640x480), which is 110 MB/s of churn at
        // 30 fps on the capture thread. That is the exact failure the Kotlin
        // side documents -- allocation, not arithmetic, is what collapsed its
        // frame rate.
        track::GrayFrame gf = makeFrameView(frame);

        if (rising) {
            trk.reset();
            trk.setCues(cues);
            trk.designate(gf, float(centre.x), float(centre.y), float(boxSize));
            if (recMode == "session") {
                openSession("AUX up");
                for (const cv::Mat& f : ring)          // flush the pre-roll
                    if (writer.isOpened()) writer.write(f);
            }
            std::printf("[lock] t=%.1f  centre (%d,%d) box %d\n",
                        t, centre.x, centre.y, boxSize);
        }
        if (falling) {
            trk.reset();
            std::printf("[lock] released t=%.1f\n", t);
            if (recMode == "session") closeSession();
        }

        track::LockTracker::Result tr{};
        if (rising) {
            // designate() just built the templates FROM this frame; running a
            // match against it would be a tautology and would nudge the centre
            // filter on zero evidence. Report the designated box instead.
            tr.state = track::LockTracker::State::LOCKED;
            tr.x = centre.x - boxSize / 2; tr.y = centre.y - boxSize / 2;
            tr.w = tr.h = boxSize;
            tr.conf = 1.f;
            tr.aimX = float(centre.x); tr.aimY = float(centre.y);
        } else if (trk.hasTarget()) {
            tr = trk.update(gf);
        }

        // ---- overlay
        cv::Mat vis = frame.clone();
        using St = track::LockTracker::State;
        const St st = high ? trk.state() : St::IDLE;
        const char* state = track::LockTracker::stateName(st);
        // COASTING is amber (riding the prediction, still fine); SEARCHING is
        // orange (zoomed out, anchor-only); LOST is red. Distinguishing the two
        // recovery states on the video is the whole point -- they fail for
        // different reasons and the fix for each is different.
        const cv::Scalar col =
              st == St::LOCKED    ? cv::Scalar(0, 230, 0)
            : st == St::COASTING  ? cv::Scalar(0, 200, 255)
            : st == St::SEARCHING ? cv::Scalar(0, 140, 255)
            : st == St::IDLE      ? cv::Scalar(180, 180, 180)
                                  : cv::Scalar(0, 0, 255);

        if (!high) {
            // Idle reticle: shows the operator exactly what will be grabbed.
            const int r = boxSize / 2;
            cv::rectangle(vis, cv::Rect(centre.x - r, centre.y - r, boxSize, boxSize),
                          cv::Scalar(120, 120, 120), 1);
            cv::drawMarker(vis, centre, cv::Scalar(200, 200, 200),
                           cv::MARKER_CROSS, 18, 1);
        } else if (trk.hasTarget()) {
            cv::rectangle(vis, cv::Rect(tr.x, tr.y, tr.w, tr.h), col, 2);
            const cv::Point c(tr.x + tr.w / 2, tr.y + tr.h / 2);
            cv::drawMarker(vis, c, col, cv::MARKER_CROSS, 14, 1);
            // Latency-compensated AIM point -- where the target will be by the
            // time a command based on this frame could act on it. This is the
            // number a future control loop would consume, so draw it, not the
            // box centre.
            cv::arrowedLine(vis, c, cv::Point(int(tr.aimX), int(tr.aimY)), col, 2,
                            cv::LINE_AA, 0, 0.3);
            cv::circle(vis, cv::Point(int(tr.aimX), int(tr.aimY)), 3, col, -1);
        }

        char l1[192], l2[192], l3[192];
        std::snprintf(l1, sizeof l1, "%-9s AUX%d %4dus  conf %.2f",
                      state, auxIdx - 3, auxUs, tr.conf);
        std::snprintf(l2, sizeof l2, "%.0f fps  flow %.1f crop %.1f cue %.1f ms",
                      fps, trk.tFlowMs(), trk.tCropMs(), trk.tCueMs());
        std::snprintf(l3, sizeof l3, "wrote %ld  drop %ld%s",
                      framesWritten, framesDropped,
                      fcr.haveFc() && !tel.linkUp ? "  [FC LINK DOWN]" : "");
        putShadowed(vis, l1, {8, 20}, 0.5, col);
        putShadowed(vis, l2, {8, 40}, 0.5, cv::Scalar(220, 220, 220));
        putShadowed(vis, l3, {8, 60}, 0.5, cv::Scalar(220, 220, 220));
        if (writer.isOpened())
            cv::circle(vis, cv::Point(W - 16, 16), 6, cv::Scalar(0, 0, 255), -1);

        // ---- record
        if (prerollN) {
            ring.push_back(vis.clone());
            while (ring.size() > prerollN) ring.pop_front();
        }
        if (writer.isOpened()) {
            const double tw0 = nowS();
            writer.write(vis);
            ++framesWritten;
            // A writer that cannot keep up is the difference between evidence
            // and a smear. Say so once rather than silently dropping quality.
            if (nowS() - tw0 > 1.5 / camFps) {
                ++framesDropped;
                if (framesDropped == 1)
                    std::fprintf(stderr,
                        "[rec] writer slower than capture -- try a smaller "
                        "--width/--height or a different --fourcc\n");
            }
            if (csv) {
                std::fprintf(csv,
                    "%.3f,%d,%s,%d,%d,%d,%d,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f\n",
                    t, auxUs, state, tr.x + tr.w / 2, tr.y + tr.h / 2, tr.w, tr.h,
                    tr.conf, tr.aimX, tr.aimY,
                    trk.tFlowMs(), trk.tCropMs(), trk.tCueMs(), fps);
            }
        }

        if (display) {
            cv::imshow("lockon_rec", vis);
            const int k = cv::waitKey(1) & 0xFF;
            if (k == 27 || k == 'q') break;
            if (k == 'l') manualLock = !manualLock;
        }
    }

    closeSession();
    fcr.stop();
    cap.release();
    std::printf("[done] %ld frames, %ld written, %ld slow writes\n",
                frameNo, framesWritten, framesDropped);
    return 0;
}
