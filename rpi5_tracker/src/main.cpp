// Raspberry Pi 5 click-to-lock tracker with monocular depth navigation.
//
// Standalone C++ port of the Python app's drone mode: click the video to
// designate a fixed-size box, a lightweight tracker (CSRT / KCF / optical
// flow) follows it every frame. Optionally runs a monocular depth model
// (MiDaS Small or Depth Anything v2 Small) every N frames to show which
// sector of the frame is most open. No GPU — pure CPU.
//
// Keys:  1=CSRT  2=KCF  3=Optical Flow  4=MOSSE
//        d=toggle depth overlay   r=reset   ESC/q=quit
// Mouse: left click = lock onto that point

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstdio>
#include <string>

#include "depth_nav.hpp"
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

// Single-field deinterlace: drop odd lines and resize back up.
// Halves comb artifacts from analog capture dongles on moving objects.
void deinterlace(cv::Mat& frame) {
    const int H = frame.rows, W = frame.cols;
    cv::Mat even(H / 2, W, frame.type());
    for (int r = 0; r < H / 2; ++r)
        frame.row(r * 2).copyTo(even.row(r));
    cv::resize(even, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
}

void draw_hud(cv::Mat& frame, const LockOnTracker& trk, Backend backend,
              double fps, bool depthActive) {
    const cv::Scalar white(255, 255, 255);

    if (trk.hasTarget()) {
        cv::Scalar  color(80, 220, 80);
        const char* status = "LOCK";
        if (!trk.locked()) {
            color  = cv::Scalar(60, 60, 230);
            status = "LOST";
        } else if (trk.coasting()) {
            color  = cv::Scalar(60, 180, 255);
            status = "COAST";
        }

        const cv::Rect& b      = trk.bbox();
        const cv::Point centre = (b.tl() + b.br()) / 2;

        cv::rectangle(frame, b, color, 2);
        cv::drawMarker(frame, centre, color, cv::MARKER_CROSS, 12, 1);

        // Motion-vector arrow — projected centre 8 frames ahead.
        if (trk.locked()) {
            const auto proj = trk.projected(8.f);
            const cv::Point tip((int)proj.x, (int)proj.y);
            if (tip != centre)
                cv::arrowedLine(frame, centre, tip, {200, 200, 0},
                                1, cv::LINE_AA, 0, 0.3);
        }

        // Confidence bar (bottom of box).
        const int barW = (int)(b.width * trk.confidence());
        cv::rectangle(frame, {b.x, b.y + b.height + 3,
                               b.width, 4}, {60, 60, 60}, cv::FILLED);
        if (barW > 0)
            cv::rectangle(frame, {b.x, b.y + b.height + 3, barW, 4},
                          color, cv::FILLED);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s  age %ld  loss %d  conf %.0f%%",
                      status, trk.age(), trk.totalLosses(),
                      trk.confidence() * 100.f);
        cv::putText(frame, buf, {b.x, std::max(14, b.y - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
    }

    char info[160];
    std::snprintf(info, sizeof(info),
                  "%s  %.0f fps   click=lock  1234=backend  d=depth  r=reset  q=quit",
                  backend_name(backend), fps);
    cv::putText(frame, info,
                {8, frame.rows - (depthActive ? 46 : 10)},
                cv::FONT_HERSHEY_SIMPLEX, 0.42, white, 1, cv::LINE_AA);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string keys =
        "{help h           |       | show this message }"
        "{camera c         | 0     | V4L2 camera index }"
        "{width            | 640   | capture width }"
        "{height           | 480   | capture height }"
        "{size s           | 80    | lock box size in px }"
        "{backend b        | csrt  | tracking backend: csrt | kcf | flow | mosse }"
        "{deinterlace      | false | drop odd lines (analog capture dongle) }"
        "{depth-model      |       | path to ONNX model for depth nav }"
        "{depth-backend    | midas | depth model: midas | dav2 }"
        "{depth-interval   | 6     | run depth every N tracking frames }"
        "{depth-on         | false | start with depth overlay visible }";

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) { parser.printMessage(); return 0; }

    // ---- tracking backend
    Backend backend = Backend::CSRT;
    const std::string be = parser.get<std::string>("backend");
    if (be == "kcf")   backend = Backend::KCF;
    if (be == "flow")  backend = Backend::FLOW;
    if (be == "mosse") backend = Backend::MOSSE;
    const int boxSize = parser.get<int>("size");

    // ---- depth nav
    DepthNav   depthNav;
    const std::string depthModel = parser.get<std::string>("depth-model");
    const bool        depthOnArg = parser.get<bool>("depth-on");
    bool              depthShow  = false;
    const int         depthInterval = std::max(1, parser.get<int>("depth-interval"));
    int               depthAge   = 0;

    if (!depthModel.empty()) {
        DepthBackend db = DepthBackend::MIDAS_SMALL;
        if (parser.get<std::string>("depth-backend") == "dav2")
            db = DepthBackend::DEPTH_ANYTHING_V2;
        if (depthNav.init(depthModel, db)) {
            depthShow = depthOnArg;
            std::printf("[depth] %s loaded, interval=%d frames\n",
                        depth_backend_name(db), depthInterval);
        } else {
            std::fprintf(stderr, "[depth] failed to load %s\n", depthModel.c_str());
        }
    }

    // ---- camera
    const bool doDeinterlace = parser.get<bool>("deinterlace");
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

        if (doDeinterlace) deinterlace(frame);

        // ---- depth (every N frames, only when shown)
        if (depthNav.isReady()) {
            ++depthAge;
            if (depthAge >= depthInterval) {
                depthAge = 0;
                depthNav.update(frame);
            }
        }

        // ---- tracking
        if (click.pending) {
            click.pending = false;
            tracker.init(frame, click.pt, backend, boxSize);
        }
        tracker.update(frame);

        // ---- fps
        const auto   tNow = std::chrono::steady_clock::now();
        const double dt   = std::chrono::duration<double>(tNow - tPrev).count();
        tPrev = tNow;
        fps   = 0.9 * fps + 0.1 / std::max(dt, 1e-6);

        // ---- draw
        if (depthShow && depthNav.isReady()) depthNav.drawOverlay(frame);
        draw_hud(frame, tracker, backend, fps, depthShow && depthNav.isReady());
        cv::imshow(win, frame);

        // ---- keys
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
        if (key == 'r') tracker.reset();
        if (key == 'd' && depthNav.isReady()) depthShow = !depthShow;
        if (key == '1' || key == '2' || key == '3' || key == '4') {
            backend = (key == '1')   ? Backend::CSRT
                      : (key == '2') ? Backend::KCF
                      : (key == '3') ? Backend::FLOW
                                     : Backend::MOSSE;
            tracker.reset();
        }
    }
    return 0;
}
