// Raspberry Pi 5 click-to-lock tracker.
//
// Standalone C++ port of the Python app's drone mode: click the video to
// designate a fixed-size box, a lightweight tracker (CSRT / KCF / optical
// flow) follows it every frame, and the HUD shows lock age and loss count
// for reliability measurement. No neural networks, no GPU — the exact
// workload a sub-10W companion computer runs.
//
// Keys:  1=CSRT  2=KCF  3=Optical Flow  r=reset  ESC/q=quit
// Mouse: left click = lock onto that point

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstdio>
#include <string>

#include "lock_tracker.hpp"

namespace {

struct ClickState {
    cv::Point pt{-1, -1};
    bool      pending = false;
};

void on_mouse(int event, int x, int y, int, void* userdata) {
    auto* cs = static_cast<ClickState*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        cs->pt      = {x, y};
        cs->pending = true;
    }
}

void draw_hud(cv::Mat& frame, const LockOnTracker& trk, Backend backend,
              double fps) {
    const cv::Scalar white(255, 255, 255);

    if (trk.hasTarget()) {
        // Green while locked, orange while coasting through a loss, red lost.
        cv::Scalar color(80, 220, 80);
        const char* status = "LOCK";
        if (!trk.locked()) {
            color  = cv::Scalar(60, 60, 230);
            status = "LOST";
        } else if (trk.lossFrames() > 0) {
            color  = cv::Scalar(60, 180, 255);
            status = "COAST";
        }

        const cv::Rect& b = trk.bbox();
        cv::rectangle(frame, b, color, 2);
        cv::drawMarker(frame, (b.tl() + b.br()) / 2, color,
                       cv::MARKER_CROSS, 12, 1);

        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s  age %ld  losses %d", status,
                      trk.age(), trk.totalLosses());
        cv::putText(frame, buf, {b.x, std::max(14, b.y - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }

    char info[128];
    std::snprintf(info, sizeof(info),
                  "%s  %.0f fps   click=lock  1/2/3=backend  r=reset  q=quit",
                  backend_name(backend), fps);
    cv::putText(frame, info, {8, frame.rows - 10}, cv::FONT_HERSHEY_SIMPLEX,
                0.45, white, 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string keys =
        "{help h    |      | show this message }"
        "{camera c  | 0    | V4L2 camera index }"
        "{width     | 640  | capture width }"
        "{height    | 480  | capture height }"
        "{size s    | 80   | lock box size in px }"
        "{backend b | csrt | csrt | kcf | flow }";
    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }

    Backend backend = Backend::CSRT;
    const std::string be = parser.get<std::string>("backend");
    if (be == "kcf")  backend = Backend::KCF;
    if (be == "flow") backend = Backend::FLOW;
    const int boxSize = parser.get<int>("size");

    cv::VideoCapture cap(parser.get<int>("camera"), cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  parser.get<int>("width"));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, parser.get<int>("height"));
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    if (!cap.isOpened()) {
        std::fprintf(stderr, "Cannot open camera %d\n",
                     parser.get<int>("camera"));
        return 1;
    }

    const std::string win = "RPi5 Lock-On";
    cv::namedWindow(win);
    ClickState click;
    cv::setMouseCallback(win, on_mouse, &click);

    LockOnTracker tracker;
    double fps   = 30.0;
    auto   tPrev = std::chrono::steady_clock::now();

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            std::fprintf(stderr, "Camera read failed\n");
            break;
        }

        if (click.pending) {
            click.pending = false;
            tracker.init(frame, click.pt, backend, boxSize);
        }

        tracker.update(frame);

        const auto tNow = std::chrono::steady_clock::now();
        const double dt =
            std::chrono::duration<double>(tNow - tPrev).count();
        tPrev = tNow;
        fps   = 0.9 * fps + 0.1 / std::max(dt, 1e-6);

        draw_hud(frame, tracker, backend, fps);
        cv::imshow(win, frame);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
        if (key == 'r') tracker.reset();
        if (key == '1' || key == '2' || key == '3') {
            backend = (key == '1')   ? Backend::CSRT
                      : (key == '2') ? Backend::KCF
                                     : Backend::FLOW;
            tracker.reset();  // backend switch requires a fresh lock
        }
    }
    return 0;
}
