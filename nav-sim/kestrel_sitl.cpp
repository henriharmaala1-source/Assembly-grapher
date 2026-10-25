// `kestrel sitl` -- the aircraft's own autonomy, flying ArduPilot.
//
//   kestrel sitl [--connect tcp:127.0.0.1:5760] [--world gallery|hall]
//                [--seed N] [--seconds S] [--alt M] [--script FILE.kms|.kmb]
//                [--lowres] [--keep-going] [--speedup X] [--realtime]
//                [--vio | --slam [--slam-socket PATH]] [--csv FILE] [--shot PREFIX]
//
// FlightShow -- the same object `demo` and `mission sim` fly, so the same
// VoxelNavModule, MissionController and ScriptMode from onboard/src -- with
// SitlPlant as its airframe: commands go to ArduCopter through onboard's own
// MavlinkBackend, and the simulated D435i is rendered from ArduPilot's TRUE
// pose (sitl_plant.hpp). What this proves that `mission sim` cannot: the
// commands mean what the autonomy thinks once a real flight stack's
// controllers, limits and lag are between them and the airframe.
//
// Scored as everything here is scored (CLAUDE.md): metres flown before a
// collision ends it, the collision itself, cells visited and net
// displacement. A wall is air to SITL, so contact is judged against the true
// voxels and the run ENDS at the first one (--keep-going to fly on). Then
// LAND, and wait for the FC to disarm. Contact cuts the motors (a forced
// disarm), so the airframe falls in SITL as it would off a real wall.
//
// SOFT LOCKSTEP (default). SITL keeps its own clock and cannot be stepped
// from MAVLink -- but it re-reads SIM_SPEEDUP at run time. So this loop
// paces itself on SITL's clock (ATTITUDE time_boot_ms: one tick per 50 ms of
// SIMULATED time) and sets SIM_SPEEDUP to what it can sustain, from the
// measured cost of a tick: a slow render slows the simulated world instead of
// making the frames late. The alternative, ArduPilot's JSON physics backend,
// is true lockstep but replaces ArduPilot's multicopter model with one of
// ours, which is the fidelity this exists to borrow. --realtime: pace to the
// wall at --speedup instead, late frames counted, as before.

#include "kestrel_sitl.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "flight_show.hpp"
#include "flight_view.hpp"
#include "kestrel_mission.hpp"
#include "mission_program.hpp"
#include "sitl_plant.hpp"

namespace ksitl {

namespace {

double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

struct Options {
    SitlParams sitl;
    kshow::FlightParams fp;
    float seconds = 120.f;
    std::string script, shot, csv;
    bool keepGoing = false, realtime = false;
    double speedup = 1.0;
};

int usage() {
    std::fputs(usageText(), stderr);
    return 2;
}

bool parse(const std::vector<std::string>& a, Options& o) {
    o.fp.world = "gallery";
    for (size_t i = 0; i < a.size(); ++i) {
        const bool more = i + 1 < a.size();
        if (a[i] == "--connect" && more) o.sitl.connect = a[++i];
        else if (a[i] == "--world" && more) o.fp.world = a[++i];
        else if (a[i] == "--seed" && more) o.fp.seed = unsigned(std::atoi(a[++i].c_str()));
        else if (a[i] == "--seconds" && more) o.seconds = float(std::atof(a[++i].c_str()));
        else if (a[i] == "--alt" && more) o.fp.altM = float(std::atof(a[++i].c_str()));
        else if (a[i] == "--script" && more) o.script = a[++i];
        else if (a[i] == "--shot" && more) o.shot = a[++i];
        else if (a[i] == "--csv" && more) o.csv = a[++i];
        else if (a[i] == "--speedup" && more) o.speedup = std::atof(a[++i].c_str());
        else if (a[i] == "--lowres") { o.fp.camW = 424; o.fp.camH = 240; }
        else if (a[i] == "--keep-going") o.keepGoing = true;
        else if (a[i] == "--vio") { o.fp.vio = true; o.sitl.vio = true; }
        else if (a[i] == "--slam") { o.fp.slam = true; o.sitl.vio = true; }
        else if (a[i] == "--slam-socket" && more) o.fp.slamSocket = a[++i];
        else if (a[i] == "--slam-inertial") {
            o.fp.slam = o.fp.slamInertial = true; o.sitl.vio = o.sitl.imu = true;
        }
        else if (a[i] == "--realtime") o.realtime = true;
        else { std::fprintf(stderr, "[sitl] unknown argument: %s\n", a[i].c_str()); return false; }
    }
    if (o.fp.world != "gallery" && o.fp.world != "hall") {
        std::fprintf(stderr, "[sitl] --world takes gallery or hall (got '%s')\n", o.fp.world.c_str());
        return false;
    }
    if (o.speedup <= 0.0 || o.seconds <= 0.f || o.fp.altM < 0.5f) {
        std::fprintf(stderr, "[sitl] --speedup and --seconds must be > 0, --alt at least 0.5 m\n");
        return false;
    }
    if (o.fp.vio && o.fp.slam) {
        std::fprintf(stderr, "[sitl] --vio and --slam are two estimators for one job: pick one\n");
        return false;
    }
    if (o.fp.slam && o.fp.camW < 848)
        std::fprintf(stderr, "[sitl] note: ORB-SLAM3 at %dx%d finds too few corners in dark IR to "
                             "initialise (needs > 500; measured ~10-50). Fly SLAM at 848x480, with "
                             "the bridge on --fast 12 5 (onboard/orbslam/README.md).\n",
                     o.fp.camW, o.fp.camH);
    o.sitl.takeoffM = o.fp.altM;
    return true;
}

}  // namespace

const char* usageText() {
    return
        "kestrel sitl [--connect SPEC] [--world gallery|hall] [--seed N] [--seconds S]\n"
        "             [--alt M] [--script FILE.kms|.kmb] [--lowres] [--keep-going]\n"
        "             [--speedup X] [--realtime] [--vio | --slam | --slam-inertial]\n"
        "             [--slam-socket P]\n"
        "             [--csv FILE] [--shot PREFIX]\n"
        "  The aircraft's own autonomy flying ArduPilot SITL: commands through\n"
        "  onboard's MAVLink backend, the simulated D435i rendered from SITL's TRUE\n"
        "  pose. Start SITL first (ArduCopter, SERIAL0 on tcp:5760 -- RUN_ME_windows.txt).\n"
        "  --connect   tcp:HOST:PORT (default tcp:127.0.0.1:5760), udpout:HOST:PORT, udp:PORT\n"
        "  --seconds   flight time after takeoff (default 120)\n"
        "  --alt       takeoff and flight height above the floor, m (default 1.5)\n"
        "  --script    fly a mission on SCRIPT mode instead of AUTONOMY's wandering\n"
        "  --keep-going  fly on through a collision (default: the first one ends it)\n"
        "  --speedup   the fastest SITL may run (default 1). Soft lockstep slows it\n"
        "              (SIM_SPEEDUP) to what this machine renders: no late frames\n"
        "  --realtime  no lockstep: pace to the wall at --speedup, count late frames\n"
        "  --vio       GNSS-DENIED: kestrel's VIO on the simulated IR is the FC's\n"
        "              position (ODOMETRY into EKF3). Start SITL with\n"
        "              nav-sim/sitl/kestrel_vio.parm (GPS off, ExternalNav on)\n"
        "  --slam      the same with ORB-SLAM3 on the simulated stereo IR pair, via a\n"
        "              running kestrel-orbslam (onboard/orbslam; Linux / WSL2)\n"
        "  --slam-inertial  ORB-SLAM3 stereo-inertial, on ArduPilot's own IMU\n"
        "              (HIGHRES_IMU, 200 Hz, on SITL's clock like the frames)\n"
        "  --csv       append one row of the result to FILE\n"
        "  --shot      write PREFIX_mission.png, _chase.png and _depth.png at the end\n"
        "  exit: 0 flown clean, 1 contact, 2 usage, 3 SITL not reached or not airborne\n";
}

int run(const std::vector<std::string>& args) {
    // A run lasts minutes and is mostly read from a log (CI, a pipe): line by
    // line, not in 4 KB blocks at the end.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    for (const std::string& a : args)
        if (a == "--help" || a == "-h") { std::fputs(usageText(), stdout); return 0; }
    Options o;
    if (!parse(args, o)) return usage();
    if (!o.script.empty()) {
        auto prog = std::make_shared<kms::Program>();
        if (!kmission::loadAny(o.script, *prog, true)) return 1;
        o.fp.script = prog;
    }
    auto plant = std::make_shared<SitlPlant>(o.sitl);
    std::printf("[sitl] connecting to %s ...\n", o.sitl.connect.c_str());
    if (!plant->connect()) {
        std::fprintf(stderr, "[sitl] %s\n", plant->failure().c_str());
        return 3;
    }
    o.fp.plant = plant;
    kshow::FlightShow f(o.fp);
    if (!o.script.empty() && !f.script()) return 1;
    std::printf("[sitl] %s (seed %u), %dx%d stereo depth, %s, up to %.0f s after takeoff\n",
                o.fp.world.c_str(), o.fp.seed, o.fp.camW, o.fp.camH,
                f.script() ? "SCRIPT mode" : "AUTONOMY", double(o.seconds));

    // THE LOOP: one FlightShow tick per kDt of SITL's time.
    const double period = kshow::FlightShow::kDt;
    const double wallPeriod = period / o.speedup;     // --realtime only
    double next = nowS();
    double nextSim = -1.0;                 // SITL time the next tick is due at
    double speed = o.speedup;              // SIM_SPEEDUP as last set
    double costSum = 0; int costN = 0;     // tick cost, for the speedup controller
    long ticks = 0, late = 0;
    if (!o.realtime) plant->fc().setParam("SIM_SPEEDUP", float(speed));
    float travelAtCrash = -1.f, crashT = -1.f;
    double lastPrint = 0, tickSum = 0, tickMax = 0;
    std::string lastPhase;
    // VIO against the truth: its frame starts where the first valid pose was.
    bool vioAnchored = false;
    float vioE0 = 0, vioN0 = 0, vioErrMax = 0, ekfErrMax = 0;
    // HOLDING STILL: while the mission holds a vantage (THINK / SCAN) it
    // commands zero velocity, so any true speed there is the FC failing to
    // hold -- the cleanest single number for how well its estimate closes
    // the loop. SITL's truth, not anything the aircraft believes.
    double holdSq = 0; long holdN = 0;
    while (true) {
        const double t0 = nowS();
        f.tick();
        const double tickS = nowS() - t0;
        costSum += tickS; ++costN;
        if (plant->stage() == SitlPlant::Stage::FAILED) {
            std::fprintf(stderr, "[sitl] %s\n", plant->failure().c_str());
            return 3;
        }
        const kshow::FlightStats& st = f.stats();
        if ((o.fp.vio || o.fp.slam) && !vioAnchored && f.state().vioValid) {
            // The VIO's origin, in the world: where the camera was at its
            // first valid pose (it estimates from there, North-up).
            const WorldState w = f.state();
            vioE0 = f.truth().e - w.vioPe; vioN0 = f.truth().n - w.vioPn;
            vioAnchored = true;
        }
        // Both estimates against the truth, as metres of horizontal error.
        float vioErr = -1.f, ekfErr = -1.f;
        if (vioAnchored) {
            const WorldState w = f.state();
            vioErr = std::hypot(vioE0 + w.vioPe - f.truth().e, vioN0 + w.vioPn - f.truth().n);
        }
        {
            FcTelemetry tel;
            plant->fc().poll(tel);
            if (tel.localValid)            // the EKF's NED, from the origin = SITL's start
                ekfErr = std::hypot(tel.localE - (f.truth().e - f.spawnE()),
                                    tel.localN - (f.truth().n - f.spawnN()));
        }
        if (plant->stage() == SitlPlant::Stage::FLYING) {
            ++ticks;
            tickSum += tickS; tickMax = std::max(tickMax, tickS);
            if (vioErr >= 0.f) vioErrMax = std::max(vioErrMax, vioErr);
            if (ekfErr >= 0.f) ekfErrMax = std::max(ekfErrMax, ekfErr);
            const std::string ph = f.state().missionPhase;
            if (ph == "THINK" || ph == "SCAN") {
                const MavlinkBackend::SimTruth& tr = plant->fc().simTruth();
                holdSq += double(tr.vN) * tr.vN + double(tr.vE) * tr.vE;
                ++holdN;
            }
            if (st.collisions > 0 && travelAtCrash < 0.f) {
                travelAtCrash = st.travelM; crashT = st.timeS;
                std::printf("[sitl] t=%.1f s  COLLISION after %.1f m (clearance %.2f m)\n",
                            double(st.timeS), double(st.travelM), double(st.minClearM));
                if (!o.keepGoing) break;
            }
            if (st.timeS >= o.seconds) break;
            if (f.scriptEnded() || st.stuck) break;
            const WorldState w = f.state();
            if (st.timeS - lastPrint >= 5.0 || w.missionPhase != lastPhase) {
                const sim::CamPose& p = f.truth();
                std::printf("[sitl] t=%5.1f s  %-6s  at %.1f,%.1f up %.2f m  hdg %3.0f  %.2f m/s  "
                            "flown %.1f m  net %.1f m  clear %.2f m\n",
                            double(st.timeS), w.missionPhase.c_str(), double(p.e), double(p.n),
                            double(p.u - f.floorZ()), double(p.yawDeg), double(f.speed()),
                            double(st.travelM), double(st.netM), double(st.minClearM));
                if (o.fp.vio || o.fp.slam)
                    std::printf("[sitl]          VIO error %.2f m (%d tracked)  EKF error %.2f m  "
                                "ODOMETRY sent %ld\n", double(vioErr), w.vioTracked,
                                double(ekfErr), plant->visionSent());
                if (st.timeS - lastPrint >= 5.0) lastPrint = st.timeS;
                lastPhase = w.missionPhase;
            }
        }
        const double simNow = plant->timeS();
        if (!o.realtime && simNow >= 0.0) {
            // SOFT LOCKSTEP. Late = SITL's clock ran a whole tick past when
            // this one was due: the controller below had not caught up yet.
            if (nextSim < 0.0) nextSim = simNow;
            if (plant->stage() == SitlPlant::Stage::FLYING && simNow > nextSim + period) ++late;
            nextSim = std::max(nextSim + period, simNow - period);
            // SIM_SPEEDUP from the measured tick cost, 60 % of what fits: the
            // cost swings 2-3x tick to tick (a SCAN renders more, a stop maps),
            // and the mean is not the worst. Down at once, up only by a margin,
            // so it does not hunt.
            if (costN >= 10) {
                const double cost = costSum / costN;
                const double want = std::max(0.03, std::min(o.speedup, 0.6 * period / cost));
                if (want < 0.9 * speed || want > 1.25 * speed) {
                    speed = want;
                    plant->fc().setParam("SIM_SPEEDUP", float(speed));
                    std::printf("[sitl] tick %.0f ms: SIM_SPEEDUP %.2f\n", 1e3 * cost, speed);
                }
                costSum = 0; costN = 0;
            }
            // Wait for SITL's clock, servicing the link. A SITL that stops
            // answering for 3 s is not waited for.
            const double w0 = nowS();
            while (plant->timeS() < nextSim && nowS() - w0 < 3.0) {
                plant->fc().tick();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } else {
            next += wallPeriod;
            const double wait = next - nowS();
            if (wait > 0) std::this_thread::sleep_for(std::chrono::duration<double>(wait));
            else if (plant->stage() == SitlPlant::Stage::FLYING) {
                ++late;
                if (wait < -1.0) next = nowS();            // far behind: do not try to catch up
            }
        }
    }

    const bool crashed = travelAtCrash >= 0.f;
    if (crashed && !o.keepGoing) plant->crash();
    const kshow::FlightStats& st = f.stats();
    const float before = crashed ? travelAtCrash : st.travelM;
    std::printf("\n[sitl] %s\n", f.script() ? f.script()->status().c_str()
                                             : (st.stuck ? "STUCK" : "autonomy"));
    std::printf("[sitl] %.0f s, flown %.1f m%s, net %.1f m, %d cells, %d legs, closest %.2f m, "
                "collisions %d\n",
                double(st.timeS), double(before), crashed ? " before the collision" : "",
                double(st.netM), st.cells, st.legs,
                double(st.minClearM > 100.f ? 0.f : st.minClearM), st.collisions);
    std::printf("[sitl] %ld ticks, %ld late (%.0f %%) -- frames rendered later than SITL's clock; "
                "tick mean %.0f ms, worst %.0f ms (budget %.0f)\n",
                ticks, late, ticks ? 100.0 * double(late) / double(ticks) : 0.0,
                ticks ? 1e3 * tickSum / double(ticks) : 0.0, 1e3 * tickMax, 1e3 * period);
    const double holdRms = holdN ? std::sqrt(holdSq / double(holdN)) : -1.0;
    std::printf("[sitl] holding still (THINK/SCAN, zero commanded): true speed rms %.2f m/s "
                "over %.1f s\n", holdRms, double(holdN) * kshow::FlightShow::kDt);
    if (!o.realtime)
        std::printf("[sitl] soft lockstep: SITL ran at x%.2f of real time to keep up\n", speed);
    if (o.fp.vio || o.fp.slam)
        std::printf("[sitl] GNSS-denied (%s): worst %s error %.2f m, worst EKF error %.2f m "
                    "(horizontal, against SITL's truth); %ld ODOMETRY frames sent\n",
                    o.fp.slam ? "ORB-SLAM3" : "VIO", o.fp.slam ? "SLAM" : "VIO", double(vioErrMax), double(ekfErrMax), plant->visionSent());
    if (!o.csv.empty()) {
        std::ifstream probe(o.csv);
        const bool fresh = !probe.good();
        probe.close();
        std::ofstream c(o.csv, std::ios::app);
        if (fresh) c << "world,seed,mode,seconds,travel_m,travel_before_crash_m,net_m,cells,legs,"
                        "min_clear_m,collisions,crash_t_s,late_pct,hold_rms_mps,estimator,sim_speedup,est_err_max_m,ekf_err_max_m\n";
        c << o.fp.world << ',' << o.fp.seed << ',' << (f.script() ? "script" : "autonomy") << ','
          << st.timeS << ',' << st.travelM << ',' << before << ',' << st.netM << ',' << st.cells << ','
          << st.legs << ',' << st.minClearM << ',' << st.collisions << ',' << crashT << ','
          << (ticks ? 100.0 * double(late) / double(ticks) : 0.0) << ',' << holdRms << ','
          << (o.fp.slam ? "slam" : o.fp.vio ? "vio" : "gps") << ','
          << (o.realtime ? o.speedup : speed) << ',' << vioErrMax << ',' << ekfErrMax << '\n';
        std::printf("[sitl] appended to %s\n", o.csv.c_str());
    }
    if (!o.shot.empty()) {
        const kshow::ShowSnap snap = kshow::ShowSnap::of(f, nullptr);
        kshow::FlightView view;
        view.update(snap);
        bool ok = cv::imwrite(o.shot + "_mission.png", view.mission(snap, 720, 720));
        ok = cv::imwrite(o.shot + "_chase.png", view.chase(snap, 960, 540)) && ok;
        if (!f.lastDepth().empty()) {
            cv::Mat d8, col;
            f.lastDepth().convertTo(d8, CV_8U, 255.0 / 10.0);   // 0-10 m
            cv::applyColorMap(d8, col, cv::COLORMAP_TURBO);
            col.setTo(cv::Scalar(128, 128, 128), f.lastDepth() <= 0.f);   // no return: grey, unknown
            ok = cv::imwrite(o.shot + "_depth.png", col) && ok;
        }
        std::printf("[sitl] %s %s_mission.png, _chase.png, _depth.png\n",
                    ok ? "wrote" : "could not write", o.shot.c_str());
    }
    // Nothing is rendered from here on: SITL back to full speed for the
    // landing (or the fall), whose wait is on the wall clock.
    if (!o.realtime) plant->fc().setParam("SIM_SPEEDUP", float(o.speedup));
    plant->land();
    return crashed ? 1 : 0;
}

}  // namespace ksitl
