// kestrel — modular drone-vision runtime for the Raspberry Pi 5 (CPU-only).
//
// P0+P1 skeleton: capture → perception scheduler → world model → telemetry.
// Wires the existing rpi5_tracker modules (lock-on tracker, monocular depth
// corridor) plus a YOLOv8 detector behind a single compute-budgeted scheduler.
// The behaviour arbiter here is a stub; the full FSM + MAVLink offboard control
// land in P2, and the on-device LLM supervisor in P3.
//
// Keys (with --display):  click = lock target   1/2/3/4 = backend
//                         r = reset   q/ESC = quit

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstdio>
#include <string>

#include "perception.hpp"
#include "scheduler.hpp"
#include "world_model.hpp"

namespace {

struct ClickState {
    cv::Point pt{-1, -1};
    bool      pending = false;
};

void on_mouse(int event, int x, int y, int, void* userdata) {
    auto* cs = static_cast<ClickState*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) { cs->pt = {x, y}; cs->pending = true; }
}

// P1 stub arbiter. Priority: a live lock wins, else a decisive corridor, else
// scan. P2 replaces this with a hysteretic FSM driven by events + LLM goals.
Behavior arbitrate(const WorldState& s) {
    if (s.targetValid && s.targetLocked)       return Behavior::TRACK;
    if (s.corridorValid && s.corridorDecisive) return Behavior::NAVIGATE;
    return Behavior::SEARCH;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string keys =
        "{help h         |       | show this message }"
        "{camera c       | 0     | V4L2 camera index }"
        "{width          | 640   | capture width }"
        "{height         | 480   | capture height }"
        "{backend b      | mosse | track backend: csrt|kcf|flow|mosse }"
        "{size s         | 80    | lock box size px }"
        "{depth-model    |       | ONNX depth model (enables navigate) }"
        "{depth-backend  | midas | midas|dav2 }"
        "{detect-model   |       | ONNX YOLOv8 model (enables detect) }"
        "{detect-labels  | drone,bird | comma-separated class labels }"
        "{budget         | 60    | per-tick CPU budget ms }"
        "{display        | false | show the video window (desk testing) }";

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) { parser.printMessage(); return 0; }

    Backend backend = Backend::MOSSE;
    const std::string be = parser.get<std::string>("backend");
    if (be == "csrt") backend = Backend::CSRT;
    if (be == "kcf")  backend = Backend::KCF;
    if (be == "flow") backend = Backend::FLOW;

    DepthBackend db = parser.get<std::string>("depth-backend") == "dav2"
                          ? DepthBackend::DEPTH_ANYTHING_V2
                          : DepthBackend::MIDAS_SMALL;

    // Split comma-separated labels.
    std::vector<std::string> labels;
    {
        std::string raw = parser.get<std::string>("detect-labels"), cur;
        for (char ch : raw) {
            if (ch == ',') { if (!cur.empty()) labels.push_back(cur); cur.clear(); }
            else cur += ch;
        }
        if (!cur.empty()) labels.push_back(cur);
    }

    const bool display = parser.get<bool>("display");

    // ---- modules
    TrackModule    track(backend, parser.get<int>("size"));
    NavigateModule navigate(parser.get<std::string>("depth-model"), db);
    DetectModule   detect(parser.get<std::string>("detect-model"), labels);

    // ---- scheduler
    PerceptionScheduler sched;
    sched.setBudgetMs(parser.get<float>("budget"));
    sched.add({&track,    /*alwaysOn*/ true});
    sched.add({&navigate, false, /*base*/ 12, /*hot*/ 3, Behavior::NAVIGATE});
    sched.add({&detect,   false, /*base*/ 10, /*hot*/ 4, Behavior::SEARCH});

    // ---- camera
    cv::VideoCapture cap(parser.get<int>("camera"), cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  parser.get<int>("width"));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, parser.get<int>("height"));
    if (!cap.isOpened()) {
        std::fprintf(stderr, "Cannot open camera %d\n", parser.get<int>("camera"));
        return 1;
    }

    WorldModel wm;
    ClickState click;
    const std::string win = "kestrel";
    if (display) {
        cv::namedWindow(win);
        cv::setMouseCallback(win, on_mouse, &click);
    }

    long   frameId = 0;
    double fps     = 30.0;
    auto   tPrev   = std::chrono::steady_clock::now();
    auto   tLog    = tPrev;

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            std::fprintf(stderr, "Camera read failed\n");
            break;
        }
        ++frameId;

        if (click.pending) { click.pending = false; track.requestLock(click.pt); }

        const Behavior beh = arbitrate(wm.snapshot());
        sched.tick(frame, wm, frameId, beh);

        const auto   tNow = std::chrono::steady_clock::now();
        const double dt   = std::chrono::duration<double>(tNow - tPrev).count();
        tPrev = tNow;
        fps   = 0.9 * fps + 0.1 / std::max(dt, 1e-6);

        wm.with([&](WorldState& s) { s.behavior = beh; s.fps = fps; s.frameId = frameId; });

        // Telemetry: one compact world-state line ~2x/sec (this is the LLM input).
        if (std::chrono::duration<double>(tNow - tLog).count() >= 0.5) {
            tLog = tNow;
            std::printf("%s\n", wm.snapshot().brief().c_str());
            std::fflush(stdout);
        }

        if (display) {
            if (navigate.isReady()) navigate.nav().drawOverlay(frame);
            const WorldState s = wm.snapshot();
            if (s.targetValid) {
                const cv::Scalar col = s.targetLocked
                    ? (s.targetCoast ? cv::Scalar(60, 180, 255) : cv::Scalar(80, 220, 80))
                    : cv::Scalar(60, 60, 230);
                cv::rectangle(frame, s.targetBox, col, 2);
            }
            for (const auto& d : s.detections) {
                cv::rectangle(frame, d.box, {0, 200, 255}, 2);
                cv::putText(frame, d.label, d.box.tl() + cv::Point(0, -4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 200, 255}, 1);
            }
            cv::putText(frame, behavior_name(beh), {8, 22},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2);
            cv::imshow(win, frame);
            const int k = cv::waitKey(1) & 0xFF;
            if (k == 27 || k == 'q') break;
            if (k == 'r') track.reset();
            if (k == '1') track.setBackend(Backend::CSRT);
            if (k == '2') track.setBackend(Backend::KCF);
            if (k == '3') track.setBackend(Backend::FLOW);
            if (k == '4') track.setBackend(Backend::MOSSE);
        }
    }
    return 0;
}
