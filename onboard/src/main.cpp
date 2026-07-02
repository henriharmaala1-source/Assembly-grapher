// kestrel — modular drone-vision + control runtime for the Raspberry Pi 5.
//
// Capture → perception scheduler → world model → behaviour FSM → controller
// → flight-controller backend (MSP / iNAV). All CPU, no GPU, no ROS.
//
// SAFETY: control output is DRY-RUN by default — the command is computed and
// displayed but NOT sent to the FC. Pass --allow-control (and toggle with the
// space bar) to actually send. Arm on the radio, props off, bench first.
//
// Target designation: click the view (with --display), or flip an RC switch
// (--lock-aux=<chan>) to lock onto the CENTRE of the view in flight — the
// on-aircraft equivalent of a click. The lock is a sensing output (a box in the
// world model for a gimbal/operator); TRACK observes from hover.
//
// Keys (with --display):
//   click = lock target      1/2/3/4 = track backend (CSRT/KCF/FLOW/MOSSE)
//   modes: f=FLY s=ASSIST k=LOCK_ON o=FOLLOW_ROAD w=WAYPOINT a=AUTONOMY
//          y=SHADOW (operator flies; autonomy intent drawn only) h=HOLD
//   AUTONOMY/SHADOW: ← → (or , .) steer the goal direction, g = GO / STOP (hover)
//   x = abort -> iNAV RTH     space = arm/disarm control     r = reset  q/ESC = quit
//
// Bench-test (--bench-test): no camera; connects to the FC and prints a live
// telemetry table + DRY-RUN RC channel map every 500 ms. Ctrl+C to exit.
// Use --fc=sim to run it (and the whole OS) with a SIMULATED FC — no hardware:
// the sim responds to control, so GPS/attitude/battery evolve like a real link.

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "config.hpp"
#include "controller.hpp"
#include "deliberator.hpp"
#include "fc_link.hpp"
#include "rc_command.hpp"
#include "flight_controller.hpp"
#include "frame_bus.hpp"
#include "mavlink_backend.hpp"
#include "modes.hpp"
#include "msp_backend.hpp"
#include "sim_fc_backend.hpp"
#include "perception.hpp"
#include "road_follow.hpp"
#include "scheduler.hpp"
#include "state_estimator.hpp"
#include "world_model.hpp"

namespace {

constexpr float kDeg2Rad = 3.14159265358979323846f / 180.f;

// ---- bench-test ----
volatile std::sig_atomic_t g_benchQuit = 0;
void bench_sig(int) { g_benchQuit = 1; }

// Axis helper (mirrors MspBackend internals for display).
static uint16_t axUs(float v) {
    v = std::max(-1.f, std::min(1.f, v));
    int us = 1500 + (int)(v * 500.f);
    return (uint16_t)std::max(1000, std::min(2000, us));
}
// Throttle [0,1] → [1500,2000] µs (0 = mid/hold, 1 = full climb).
// Matches MspBackend::thrToUs() exactly.
static uint16_t thrUs(float v) {
    v = std::max(0.f, std::min(1.f, v));
    int us = 1500 + (int)(v * 500.f);
    return (uint16_t)std::max(1000, std::min(2000, us));
}

static void printBenchRow(const char* label, float roll, float pitch,
                          float thr, float yaw) {
    std::printf("  %-12s  A=%-4u  E=%-4u  T=%-4u  R=%-4u µs\n",
                label, axUs(roll), axUs(pitch), thrUs(thr), axUs(yaw));
}

static void runBenchTest(IFlightController& fc) {
    using clock = std::chrono::steady_clock;
    std::signal(SIGINT,  bench_sig);
    std::signal(SIGTERM, bench_sig);

    std::printf("\n");
    std::printf("╔══════════════════════════════════════════════════════════════╗\n");
    std::printf("║        kestrel  MSP BENCH-TEST  (all control DRY-RUN)       ║\n");
    std::printf("║   Ctrl+C to exit. Arm on the radio — NOT via this tool.     ║\n");
    std::printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    auto tPrint = clock::now() - std::chrono::seconds(2);   // force immediate first print
    int  ticks  = 0;

    while (!g_benchQuit) {
        fc.tick();
        ++ticks;

        const auto now   = clock::now();
        const double age = std::chrono::duration<double>(now - tPrint).count();
        if (age < 0.5) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        tPrint = now;

        FcTelemetry t{};
        fc.poll(t);

        // ---- header
        std::printf("─────────────────────────────────── [tick %-5d] ─────\n", ticks);

        // ---- link / armed
        std::printf("link    : %-6s          armed   : %s\n",
                    t.linkUp ? "UP" : "DOWN",
                    t.armed  ? "ARMED (!!)" : "disarmed");

        // ---- attitude
        std::printf("roll    : %+7.1f °       pitch   : %+7.1f °       yaw : %+7.1f °\n",
                    t.rollDeg, t.pitchDeg, t.yawDeg);

        // ---- battery
        const int cellCount = (t.battV > 16.f) ? 4 : (t.battV > 12.f) ? 3 : 0;
        if (cellCount) {
            std::printf("battery : %5.2f V  [%d%%]   (%dS pack assumed)\n",
                        t.battV, (int)(t.battPct * 100.f), cellCount);
        } else {
            std::printf("battery : %5.2f V  [reading — verify cell count]\n", t.battV);
        }

        // ---- GPS
        const char* fixNames[] = {"NO FIX","NO FIX","2D","3D","DGPS","RTK"};
        const char* fixStr = (t.fixType < 6) ? fixNames[t.fixType] : "?";
        std::printf("GPS     : %-6s  sats=%-2d  lat=%+.6f  lon=%+.6f\n",
                    fixStr, t.sats, t.lat, t.lon);
        std::printf("          alt=%.0f m   spd=%.1f m/s\n", t.altM, t.groundspeedMs);

        // ---- DRY-RUN RC channel map
        std::printf("\n── DRY-RUN RC (AETR — throttle=Ch2, yaw=Ch3) ─────────────\n");
        std::printf("  %-12s  A=Roll  E=Pitch  T=Throttle  R=Yaw  (µs)\n", "maneuver");
        std::printf("  ────────────────────────────────────────────────────────\n");
        printBenchRow("neutral",     0.f,  0.f,  0.f,  0.f);
        printBenchRow("fwd cruise",  0.f,  0.25f, 0.f, 0.f);
        printBenchRow("climb +25%",  0.f,  0.f,  0.5f, 0.f);
        printBenchRow("yaw left",    0.f,  0.f,  0.f, -0.15f);
        printBenchRow("yaw right",   0.f,  0.f,  0.f, +0.15f);
        printBenchRow("roll right",  0.35f, 0.f, 0.f,  0.f);
        std::printf("  AUX Ch4–7 = 1000 µs (arm on the radio)\n\n");

        if (!t.linkUp) {
            std::printf("  [!] FC not responding — check baud rate and cable\n\n");
        }

        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::printf("\n[bench] exited cleanly.\n");
}

// ---- main loop display helpers ----
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
        "{lock-aux       | -1    | RC channel idx (0-based) that locks onto frame centre on a high switch; -1 = off }"
        "{lock-aux-us    | 1700  | µs threshold above which the lock switch reads high }"
        "{depth-model    |       | ONNX depth model (enables navigate) }"
        "{depth-backend  | midas | midas|dav2 }"
        "{detect-model   |       | ONNX YOLOv8 model (enables detect) }"
        "{detect-labels  | drone,bird | comma-separated class labels }"
        "{road           | true  | enable appearance road-follow module }"
        "{fc             | none  | flight controller: none|msp|mavlink|sim }"
        "{fc-port        | /dev/ttyAMA0 | FC serial device }"
        "{fc-baud        | 115200 | FC serial baud }"
        "{auto           | false | autonomous move-stop-sense cycle (hover→think→plan→move) }"
        "{allow-control  | false | actually SEND control to the FC (else dry-run) }"
        "{assist         | false | flight-assist: trim from the operator's current sticks (else total autonomy from neutral) }"
        "{bench-test     | false | connect FC, print live telemetry table, then exit (no camera) }"
        "{feed-gps       | false | inject the fused estimate into iNAV as MSP2_SENSOR_GPS }"
        "{feed-gps-hz    | 10    | rate to inject synthetic GPS (Hz) }"
        "{budget         | 60    | per-tick CPU budget ms }"
        "{config         |       | key=value tuning file (gains/mission/safety) }"
        "{dump-config    | false | print effective config (defaults+overrides) and exit }"
        "{display        | false | show the video window (desk testing) }";

    cv::CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")) { parser.printMessage(); return 0; }

    // ---- runtime config: load (if given), resolve all tunables. --dump-config
    // prints the effective values and exits — the canonical knob reference.
    Config   cfg;
    if (!parser.get<std::string>("config").empty())
        cfg.load(parser.get<std::string>("config"));
    const Tunables tune = load_tunables(cfg);
    cfg.warnUnused();
    if (parser.get<bool>("dump-config")) { cfg.dump(); return 0; }

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
    const bool feedGps       = parser.get<bool>("feed-gps");
    const double feedPeriod  = 1.0 / std::max(1.0, (double)parser.get<float>("feed-gps-hz"));
    const bool assistMode    = parser.get<bool>("assist");
    const int  lockAux       = parser.get<int>("lock-aux");
    const int  lockAuxUs     = parser.get<int>("lock-aux-us");

    // ---- perception modules
    TrackModule    track(backend, parser.get<int>("size"));
    NavigateModule navigate(parser.get<std::string>("depth-model"), db);
    DetectModule   detect(parser.get<std::string>("detect-model"), labels);
    RoadFollowModule road;

    // Separate thinking from flying: the cheap, control-relevant modules (track,
    // road-follow) run in the fast fly loop; the heavy ones (depth/navigate,
    // detect — and later SLAM/planner) run on the Deliberator's own thread so a
    // slow update can never stall control. Both share the thread-safe WorldModel.
    FrameBus    frameBus;
    Deliberator deliberator;
    deliberator.scheduler().setBudgetMs(parser.get<float>("budget"));
    deliberator.scheduler().add({&navigate, false, 12, 3, Behavior::NAVIGATE});
    deliberator.scheduler().add({&detect,   false, 10, 4, Behavior::SEARCH});

    // ---- flight controller backend
    std::unique_ptr<IFlightController> fc;
    const std::string fcSel = parser.get<std::string>("fc");
    if (fcSel == "msp")     fc = std::make_unique<MspBackend>();
    if (fcSel == "mavlink") fc = std::make_unique<MavlinkBackend>();
    if (fcSel == "sim")     fc = std::make_unique<SimFcBackend>();
    if (fc && !fc->connect(parser.get<std::string>("fc-port"),
                           parser.get<int>("fc-baud"))) {
        std::fprintf(stderr, "[fc] %s connect failed — running FC-less\n", fcSel.c_str());
        fc.reset();
    }
    if (allowControl && !fc) {
        std::fprintf(stderr, "[fc] --allow-control set but no FC link; staying dry-run\n");
        allowControl = false;
    }
    // ---- bench-test mode: live telemetry table, no camera, no vision pipeline
    // (uses the raw FC directly, before it's handed to the I/O thread).
    if (parser.get<bool>("bench-test")) {
        if (!fc) {
            std::fprintf(stderr,
                "[bench] --bench-test requires --fc=msp (or mavlink). "
                "Pass --fc=msp --fc-port=/dev/ttyAMA0 --fc-baud=115200\n");
            return 1;
        }
        runBenchTest(*fc);
        fc->disconnect();
        return 0;
    }

    // Hand the FC to its own I/O thread so a slow/hung camera can't stall RC
    // (iNAV failsafes below ~5 Hz). From here the fly loop talks only to fcLink.
    FcLink fcLink(std::move(fc));
    fcLink.configure(assistMode, tune.rthAuxIdx, tune.rthAuxUs);
    fcLink.start();

    Controller       controller(tune.gains);
    StateEstimator   est;
    ModeManager      modes(tune.mode);
    RcCommandSource  rc(tune.rc);   // radio as a command source (mode/GO/steer)
    if (rc.enabled()) std::printf("[rc] command source active (mode/go/steer via AUX)\n");
    register_standard_modes(modes, tune.mission);   // FLY ASSIST LOCK_ON HOLD FOLLOW_ROAD WAYPOINT AUTONOMY ...
    const bool     autoStart = parser.get<bool>("auto");
    auto           tFeed = std::chrono::steady_clock::now();

    // ---- camera
    const int camIndex = parser.get<int>("camera");
    const int camW = parser.get<int>("width"), camH = parser.get<int>("height");
    auto openCam = [&](cv::VideoCapture& c) {
        c.open(camIndex, cv::CAP_V4L2);
        c.set(cv::CAP_PROP_FRAME_WIDTH,  camW);
        c.set(cv::CAP_PROP_FRAME_HEIGHT, camH);
        return c.isOpened();
    };
    cv::VideoCapture cap;
    if (!openCam(cap)) {
        std::fprintf(stderr, "Cannot open camera %d\n", camIndex);
        return 1;
    }
    auto tCamRetry = std::chrono::steady_clock::now();

    WorldModel wm;
    ClickState click;
    const std::string win = "kestrel";
    if (display) {
        cv::namedWindow(win);
        cv::setMouseCallback(win, on_mouse, &click);
    }

    long   frameId    = 0;
    double fps        = 30.0;
    bool   wasAllow   = false;  // engage edge for the assist-baseline latch
    bool   auxWasHigh = false;  // edge for the AUX lock switch
    auto   tPrev   = std::chrono::steady_clock::now();
    auto   tLog    = tPrev;

    wm.with([&](WorldState& s){ modes.select(autoStart ? "AUTONOMY" : "FLY", s); });
    deliberator.start(frameBus, wm);   // heavy perception on its own thread

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            // Camera lost — do NOT exit the process. Command a safe HOLD (the
            // I/O thread keeps RC alive on its own; this just makes the intent
            // an explicit hover rather than the last motion command), and retry
            // opening the camera every 2 s. iNAV doesn't failsafe on a USB hiccup.
            if (fcLink.haveFc()) {
                ControlCmd hold; hold.valid = true;
                fcLink.command(hold, allowControl);
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - tCamRetry > std::chrono::seconds(2)) {
                tCamRetry = now;
                std::fprintf(stderr, "[camera] read failed — releasing control, retrying open\n");
                cap.release();
                openCam(cap);
            }
            if (display && (cv::waitKey(1) & 0xFF) == 'q') break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));  // ~50 Hz FC service
            continue;
        }
        ++frameId;

        // Stamp "now" for this tick — the timebase the perception staleness
        // checks (corridorFresh() etc.) compare their write-stamps against.
        wm.with([&](WorldState& s) { s.tickMonoS = monoNowS(); });

        // ---- flight-controller telemetry (snapshot from the I/O thread)
        FcTelemetry t;
        bool        haveTel = false;
        if (fcLink.haveFc()) {
            t = fcLink.telemetry();
            haveTel = t.linkUp;
            wm.with([&](WorldState& s) {
                s.vehArmed = t.armed; s.vehBattery = t.battPct; s.vehBattV = t.battV;
                s.vehAltM = t.altM;   s.vehRollDeg = t.rollDeg; s.vehPitchDeg = t.pitchDeg;
                s.vehYawDeg = t.yawDeg; s.vehLat = t.lat; s.vehLon = t.lon;
                s.vehSats = t.sats; s.vehFix = t.fixType;
                s.vehGroundspeed = t.groundspeedMs; s.vehLink = t.linkUp;
                s.vehMode = "FC";
            });
        }

        // ---- target designation: mouse click (display) or AUX switch (in-flight).
        // The AUX switch locks onto whatever is at the centre of the view — the
        // in-flight equivalent of a click, for designating a subject to the
        // tracker (a sensing output; TRACK observes from hover, see controller).
        if (click.pending) { click.pending = false; track.requestLock(click.pt); }
        if (lockAux >= 0 && haveTel && lockAux < t.rcCount) {
            const bool hi = t.rc[lockAux] >= lockAuxUs;
            if (hi && !auxWasHigh)
                track.requestLock({frame.cols / 2, frame.rows / 2});   // lock frame centre
            else if (!hi && auxWasHigh)
                track.reset();                                         // release on switch low
            auxWasHigh = hi;
        }

        // ---- perception. FLY here (cheap, control-relevant): the tracker and
        // road-follow run every fast tick. THINK on the Deliberator's thread:
        // hand it the newest frame; it runs depth/detect/SLAM/planning at its own
        // pace and publishes into the world model without stalling this loop.
        frameBus.publish(frame);
        track.run(frame, wm);
        if (roadOn) road.run(frame, wm);

        // ---- behaviour + control
        const auto   tNow = std::chrono::steady_clock::now();
        const double dt   = std::chrono::duration<double>(tNow - tPrev).count();
        tPrev = tNow;
        fps   = 0.9 * fps + 0.1 / std::max(dt, 1e-6);

        // ---- RC command source: the radio drives mode / GO / goal-steer (the
        // same world-model surfaces as the keyboard), so the drone is flyable
        // with no laptop. Runs before the arbiter so inputs take effect this tick.
        if (rc.enabled() && haveTel)
            wm.with([&](WorldState& s) { rc.update(t, modes, s, (float)dt); });

        // ---- state estimator (Pi-side EKF): predict, then fuse FC data.
        // Vision-odometry / SLAM fixes plug in here via est.updateVisionVelocity()
        // / est.updateVisionPose() once the P5 front-end exists.
        bool feeding = false;
        if (haveTel) {
            est.setHeading(t.yawDeg);
            est.predict((float)dt);
            if (t.fixType >= 3) {                       // GPS velocity from course+speed
                const float course = t.groundCourseDeg * kDeg2Rad;
                const float vN = t.groundspeedMs * std::cos(course);
                const float vE = t.groundspeedMs * std::sin(course);
                est.updateGps(t.lat, t.lon, t.altM, t.fixType, true, vN, vE, 0.f, 2.5f, 3.0f);
            }
            est.updateBaro(t.baroAltM);

            // Inject the fused estimate back as synthetic GPS, rate-limited.
            if (feedGps && fcLink.haveFc() && est.hasOrigin() &&
                std::chrono::duration<double>(tNow - tFeed).count() >= feedPeriod) {
                ExtGps g;
                if (est.makeExtGps(g)) { fcLink.feedGps(g); feeding = true; }
                tFeed = tNow;
            }
        }
        const StateEstimator::State es = est.state();
        wm.with([&](WorldState& s) {
            s.estValid = es.valid;
            s.estPe = es.pe; s.estPn = es.pn; s.estPu = es.pu;
            s.estVe = es.ve; s.estVn = es.vn; s.estVu = es.vu;
            s.estSpeed = es.speedMs; s.estEphM = es.ephM;
            s.estGpsDenied = es.gpsDenied; s.estFeedingFc = feeding;
        });

        // Single control arbiter: safety layers (failsafe → iNAV RTH; obstacle →
        // HOLD) then the active mode module. Writes opMode/behavior/modeReason
        // into the world model itself.
        bool       rthTrigger = false;
        ControlCmd cmd;
        ControlCtx cctx;
        cctx.controller = &controller;
        cctx.frameW = frame.cols; cctx.frameH = frame.rows; cctx.dt = (float)dt;
        wm.with([&](WorldState& s) { cmd = modes.tick(s, cctx, rthTrigger); });

        // On the dry→live engage edge in assist mode, latch the operator's
        // current sticks so the takeover starts from their hands, not neutral.
        if (allowControl && !wasAllow && assistMode && fcLink.haveFc()) fcLink.latchBaseline();
        wasAllow = allowControl;

        // Hand intent to the I/O thread. Failsafe → RTH; a released command
        // (valid=false, e.g. FLY/SHADOW) is passed as not-live so the thread
        // sends nothing (operator/iNAV flies); an active command sends live.
        if (rthTrigger && fcLink.haveFc()) {
            fcLink.commandRth(allowControl);
        } else if (fcLink.haveFc()) {
            fcLink.command(cmd, allowControl && cmd.valid);
        }
        const bool sent = allowControl && fcLink.linkUp() && (rthTrigger || cmd.valid);

        wm.with([&](WorldState& s) {
            s.fps = fps; s.frameId = frameId;
            s.control = cmd;  s.controlActive = sent;
        });
        const WorldState snap = wm.snapshot();   // post-arbiter, for display/telemetry

        // ---- telemetry line ~2x/sec (LLM scene-state input)
        if (std::chrono::duration<double>(tNow - tLog).count() >= 0.5) {
            tLog = tNow;
            std::printf("%s\n", wm.snapshot().brief().c_str());
            // Think-tier watchdog: the deliberator stamps every loop pass, so a
            // large age means a module has it wedged — its outputs are going
            // stale (the freshness gates are already ignoring them).
            if (deliberator.running() && deliberator.lastTickAgeS() > 2.0)
                std::fprintf(stderr, "[think] WARNING: deliberator stalled %.1fs — "
                             "perception outputs stale\n", deliberator.lastTickAgeS());
            std::fflush(stdout);
        }

        // ---- display. Reads ONLY the thread-safe world-model snapshot — never
        // the think-tier modules' internals (which another thread is writing).
        if (display) {
            // Overlays only draw FRESH data — a stale arrow looks live and
            // invites trusting a corridor nobody is computing anymore.
            if (snap.corridorFresh(0.7f)) {   // corridor steer target (think tier)
                const cv::Point ctr(frame.cols / 2, frame.rows / 2);
                const cv::Point tgt((int)snap.corridorHeading.x, (int)snap.corridorHeading.y);
                const cv::Scalar col = snap.corridorDecisive ? cv::Scalar(0, 255, 128)
                                                             : cv::Scalar(0, 200, 255);
                cv::arrowedLine(frame, ctr, tgt, col, 2, cv::LINE_AA, 0, 0.25);
                cv::circle(frame, tgt, 6, col, snap.corridorDecisive ? 2 : 1, cv::LINE_AA);
            }
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
            if (snap.roadFresh(1.0f)) {   // road centreline arrow, bottom centre
                const int bx = frame.cols / 2;
                const cv::Point tip(bx + (int)(snap.roadOffset * frame.cols / 2),
                                    frame.rows / 3);
                cv::arrowedLine(frame, {bx, frame.rows - 4}, tip,
                                {80, 255, 80}, 2, cv::LINE_AA, 0, 0.25);
            }
            char autohud[48] = "";
            if (snap.missionActive)
                std::snprintf(autohud, sizeof(autohud), "  %s:%s",
                              snap.shadowActive ? "SHADOW" : "AUTO", snap.missionPhase.c_str());
            const char* ctlStr = (allowControl && sent)
                ? (assistMode ? "LIVE/assist" : "LIVE/total") : "dry";
            char hud[160];
            std::snprintf(hud, sizeof(hud), "%s  %s  ctl:%s  %.0ffps",
                          snap.opMode.c_str(), snap.modeReason.c_str(), ctlStr, fps);
            if (autohud[0]) std::strncat(hud, autohud, sizeof(hud)-std::strlen(hud)-1);
            cv::putText(frame, hud, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        {255, 255, 255}, 2);

            if (lockAux >= 0) {   // centre designation reticle for the AUX lock
                cv::drawMarker(frame, {frame.cols / 2, frame.rows / 2},
                               {200, 200, 200}, cv::MARKER_CROSS, 18, 1, cv::LINE_AA);
            }

            if (snap.missionActive) {   // AUTONOMY goal-direction arrow (rel. to nose)
                float gr = snap.missionGoalBearing - snap.vehYawDeg;
                while (gr > 180.f) gr -= 360.f; while (gr <= -180.f) gr += 360.f;
                const float a = gr * kDeg2Rad;
                const cv::Point c0(frame.cols / 2, frame.rows / 2);
                const cv::Point tip(c0.x + (int)(70 * std::sin(a)),
                                    c0.y - (int)(70 * std::cos(a)));
                const cv::Scalar gcol = snap.missionGo ? cv::Scalar(80, 255, 80)
                                                       : cv::Scalar(60, 180, 255);
                cv::arrowedLine(frame, c0, tip, gcol, 3, cv::LINE_AA, 0, 0.3);
                cv::putText(frame, snap.missionGo ? "GO" : "ARMED (g=go , .=steer)",
                            {c0.x + 10, c0.y}, cv::FONT_HERSHEY_SIMPLEX, 0.45, gcol, 1);
            }

            // SHADOW: draw the intended (dry-run) command the autonomy WOULD send
            // — forward = pitch, lateral = yaw — in magenta so it can't be
            // mistaken for live control. The operator is flying; this is advisory.
            if (snap.shadowActive && snap.shadowCmd.valid) {
                const cv::Point c0(frame.cols / 2, frame.rows * 2 / 3);
                const cv::Point tip(c0.x + (int)(snap.shadowCmd.yaw   *  60.f),
                                    c0.y - (int)(snap.shadowCmd.pitch * 120.f));
                const cv::Scalar mag(230, 60, 230);
                cv::arrowedLine(frame, c0, tip, mag, 2, cv::LINE_AA, 0, 0.3);
                cv::putText(frame, "intent", {c0.x + 8, c0.y + 4},
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, mag, 1);
            }

            cv::imshow(win, frame);
            const int kx = cv::waitKeyEx(1);          // full code (keeps arrows)
            const int k  = kx & 0xFF;                 // letter keys

            // AUTONOMY steering: pick a direction (arrows or , . ), press GO (g).
            // Any command source could set these — here it's the keyboard.
            const int kLeft = 65361, kUp = 65362, kRight = 65363;  // GTK arrow codes
            if (kx == kLeft  || k == ',') wm.with([&](WorldState& s){ s.missionGoalBearing -= 15.f; });
            if (kx == kRight || k == '.') wm.with([&](WorldState& s){ s.missionGoalBearing += 15.f; });
            if (kx == kUp)                wm.with([&](WorldState& s){ s.missionGoalBearing = s.vehYawDeg; });
            if (k == 'g') { wm.with([&](WorldState& s){ s.missionGo = !s.missionGo; });
                            std::printf("[auto] %s\n",
                                        wm.snapshot().missionGo ? "GO" : "STOP (hover)"); }

            if (k == 27 || k == 'q') break;
            if (k == 'r') track.reset();
            if (k == '1') track.setBackend(Backend::CSRT);
            if (k == '2') track.setBackend(Backend::KCF);
            if (k == '3') track.setBackend(Backend::FLOW);
            if (k == '4') track.setBackend(Backend::MOSSE);
            // Mode selection by name (registry). x = abort → iNAV RTH.
            auto setm = [&](const char* n){
                wm.with([&](WorldState& s){ modes.select(n, s); });
                std::printf("[mode] %s\n", n); };
            if (k == 'f') setm("FLY");
            if (k == 's') setm("ASSIST");
            if (k == 'k') setm("LOCK_ON");
            if (k == 'o') setm("FOLLOW_ROAD");
            if (k == 'w') setm("WAYPOINT");
            if (k == 'a') setm("AUTONOMY");
            if (k == 'y') setm("SHADOW");
            if (k == 'h') setm("HOLD");
            if (k == 'x') { modes.requestAbort(); std::printf("[mode] ABORT -> RTH\n"); }
            if (k == ' ') {
                allowControl = !allowControl && fcLink.haveFc();
                std::printf("[ctl] %s\n", allowControl ? "LIVE — sending control"
                                                       : "dry-run");
            }
        }
    }

    deliberator.stop();          // join the think thread before tearing down
    fcLink.stop();               // join the FC I/O thread + disconnect
    return 0;
}
