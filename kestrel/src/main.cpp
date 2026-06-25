// kestrel — modular drone-vision + control runtime for the Raspberry Pi 5.
//
// Capture → perception scheduler → world model → behaviour FSM → controller
// → flight-controller backend (MSP / iNAV). All CPU, no GPU, no ROS.
//
// SAFETY: control output is DRY-RUN by default — the command is computed and
// displayed but NOT sent to the FC. Pass --allow-control (and toggle with the
// space bar) to actually send. Arm on the radio, props off, bench first.
//
// Keys (with --display):
//   click = lock target      1/2/3/4 = track backend (CSRT/KCF/FLOW/MOSSE)
//   m = MANUAL    h = HOLD    g = resume autonomy    space = arm/disarm control
//   r = reset     q/ESC = quit

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "controller.hpp"
#include "flight_controller.hpp"
#include "fsm.hpp"
#include "mavlink_backend.hpp"
#include "msp_backend.hpp"
#include "perception.hpp"
#include "road_follow.hpp"
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
        "{road           | true  | enable appearance road-follow module }"
        "{fc             | none  | flight controller: none|msp|mavlink }"
        "{fc-port        | /dev/ttyAMA0 | FC serial device }"
        "{fc-baud        | 115200 | FC serial baud }"
        "{allow-control  | false | actually SEND control to the FC (else dry-run) }"
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

    std::vector<std::string> labels;
    {
        std::string raw = parser.get<std::string>("detect-labels"), cur;
        for (char ch : raw) {
            if (ch == ',') { if (!cur.empty()) labels.push_back(cur); cur.clear(); }
            else cur += ch;
        }
        if (!cur.empty()) labels.push_back(cur);
    }

    const bool display      = parser.get<bool>("display");
    const bool roadOn        = parser.get<bool>("road");
    bool       allowControl = parser.get<bool>("allow-control");

    // ---- perception modules
    TrackModule    track(backend, parser.get<int>("size"));
    NavigateModule navigate(parser.get<std::string>("depth-model"), db);
    DetectModule   detect(parser.get<std::string>("detect-model"), labels);
    RoadFollowModule road;

    PerceptionScheduler sched;
    sched.setBudgetMs(parser.get<float>("budget"));
    sched.add({&track,    /*alwaysOn*/ true});
    sched.add({&navigate, false, 12, 3, Behavior::NAVIGATE});
    sched.add({&detect,   false, 10, 4, Behavior::SEARCH});
    if (roadOn) sched.add({&road, false, 6, 2, Behavior::ROAD_FOLLOW});

    // ---- flight controller backend
    std::unique_ptr<IFlightController> fc;
    const std::string fcSel = parser.get<std::string>("fc");
    if (fcSel == "msp")     fc = std::make_unique<MspBackend>();
    if (fcSel == "mavlink") fc = std::make_unique<MavlinkBackend>();
    if (fc && !fc->connect(parser.get<std::string>("fc-port"),
                           parser.get<int>("fc-baud"))) {
        std::fprintf(stderr, "[fc] %s connect failed — running FC-less\n", fcSel.c_str());
        fc.reset();
    }
    if (allowControl && !fc) {
        std::fprintf(stderr, "[fc] --allow-control set but no FC link; staying dry-run\n");
        allowControl = false;
    }

    BehaviorFsm fsm;
    Controller  controller;

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

        // ---- flight-controller telemetry
        if (fc) {
            fc->tick();
            FcTelemetry t;
            fc->poll(t);
            wm.with([&](WorldState& s) {
                s.vehArmed = t.armed; s.vehBattery = t.battPct; s.vehBattV = t.battV;
                s.vehAltM = t.altM;   s.vehRollDeg = t.rollDeg; s.vehPitchDeg = t.pitchDeg;
                s.vehYawDeg = t.yawDeg; s.vehLat = t.lat; s.vehLon = t.lon;
                s.vehSats = t.sats; s.vehFix = t.fixType;
                s.vehGroundspeed = t.groundspeedMs; s.vehLink = t.linkUp;
                s.vehMode = "FC";
            });
        }

        // ---- target designation
        OperatorCmd op;
        if (click.pending) { click.pending = false; track.requestLock(click.pt); }

        // ---- perception
        const Behavior prevBeh = fsm.current();
        sched.tick(frame, wm, frameId, prevBeh);

        // ---- behaviour + control
        const auto   tNow = std::chrono::steady_clock::now();
        const double dt   = std::chrono::duration<double>(tNow - tPrev).count();
        tPrev = tNow;
        fps   = 0.9 * fps + 0.1 / std::max(dt, 1e-6);

        std::string reason;
        const WorldState snap = wm.snapshot();
        const Behavior beh = fsm.update(snap, op, dt, reason);
        ControlCmd cmd = controller.compute(snap, beh, frame.cols, frame.rows);

        bool sent = false;
        if (allowControl && fc && fc->linkUp() && cmd.valid)
            sent = fc->sendControl(cmd);

        wm.with([&](WorldState& s) {
            s.behavior = beh; s.modeReason = reason; s.fps = fps; s.frameId = frameId;
            s.control = cmd;  s.controlActive = sent;
        });

        // ---- telemetry line ~2x/sec (LLM scene-state input)
        if (std::chrono::duration<double>(tNow - tLog).count() >= 0.5) {
            tLog = tNow;
            std::printf("%s | %s\n", wm.snapshot().brief().c_str(), reason.c_str());
            std::fflush(stdout);
        }

        // ---- display
        if (display) {
            if (navigate.isReady()) navigate.nav().drawOverlay(frame);
            if (snap.targetValid) {
                const cv::Scalar col = snap.targetLocked
                    ? (snap.targetCoast ? cv::Scalar(60, 180, 255) : cv::Scalar(80, 220, 80))
                    : cv::Scalar(60, 60, 230);
                cv::rectangle(frame, snap.targetBox, col, 2);
            }
            for (const auto& d : snap.detections) {
                cv::rectangle(frame, d.box, {0, 200, 255}, 2);
                cv::putText(frame, d.label, d.box.tl() + cv::Point(0, -4),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 200, 255}, 1);
            }
            if (snap.roadValid) {   // road centreline arrow from bottom centre
                const int bx = frame.cols / 2;
                const cv::Point tip(bx + (int)(snap.roadOffset * frame.cols / 2),
                                    frame.rows / 3);
                cv::arrowedLine(frame, {bx, frame.rows - 4}, tip,
                                {80, 255, 80}, 2, cv::LINE_AA, 0, 0.25);
            }
            char hud[160];
            std::snprintf(hud, sizeof(hud), "%s  %s  ctl:%s  %.0ffps",
                          behavior_name(beh), reason.c_str(),
                          (allowControl && sent) ? "LIVE" : "dry", fps);
            cv::putText(frame, hud, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        {255, 255, 255}, 2);

            cv::imshow(win, frame);
            const int k = cv::waitKey(1) & 0xFF;
            if (k == 27 || k == 'q') break;
            if (k == 'r') track.reset();
            if (k == '1') track.setBackend(Backend::CSRT);
            if (k == '2') track.setBackend(Backend::KCF);
            if (k == '3') track.setBackend(Backend::FLOW);
            if (k == '4') track.setBackend(Backend::MOSSE);
            if (k == 'm') { op.type = OperatorCmd::SET_MODE; op.mode = Behavior::MANUAL;
                            fsm.update(wm.snapshot(), op, 0, reason); }
            if (k == 'h') { op.type = OperatorCmd::SET_MODE; op.mode = Behavior::HOLD;
                            fsm.update(wm.snapshot(), op, 0, reason); }
            if (k == 'g') { op.type = OperatorCmd::SET_MODE; op.mode = Behavior::IDLE;
                            fsm.update(wm.snapshot(), op, 0, reason); }
            if (k == ' ') {
                allowControl = !allowControl && fc != nullptr;
                std::printf("[ctl] %s\n", allowControl ? "LIVE — sending control"
                                                       : "dry-run");
            }
        }
    }

    if (fc) fc->disconnect();
    return 0;
}
