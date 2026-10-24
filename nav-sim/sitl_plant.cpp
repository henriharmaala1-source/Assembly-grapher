#include "sitl_plant.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace ksitl {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMPerDegLat = 111319.49;   // WGS84 equatorial, as ArduPilot's own
double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

SitlPlant::SitlPlant(const SitlParams& p) : p_(p) {
    // THE KINEMATIC PLANT'S STICK, so every gain tuned on it means the same
    // speed here: pitch 1 = 4 m/s (MissionController cruise 0.25 = 1 m/s),
    // yaw 1 = 90 deg/s, throttle 1 = 0.8 m/s (FlightShow's script climb).
    MavlinkBackend::StickScale k;
    k.mps = 1.f / MissionController::Params().cruise;
    k.yawDps = 90.f;
    k.climbMps = 0.8f;
    fc_.setStickScale(k);
}

SitlPlant::~SitlPlant() = default;

bool SitlPlant::connect() {
    if (!fc_.connect(p_.connect, 115200)) {
        failure_ = "nothing answered at " + p_.connect +
                   " -- is ArduPilot SITL running? (nav-sim/RUN_ME_windows.txt, SITL)";
        stage_ = Stage::FAILED;
        return false;
    }
    fc_.requestSimTruth(50);
    t0_ = stageT_ = nowS();
    return true;
}

const char* SitlPlant::stageName() const {
    switch (stage_) {
    case Stage::LINK:    return "link + parameters";
    case Stage::GUIDED:  return "GUIDED";
    case Stage::ARM:     return "arming";
    case Stage::TAKEOFF: return "taking off";
    case Stage::FLYING:  return "flying";
    case Stage::FAILED:  return "failed";
    }
    return "?";
}

void SitlPlant::service() {
    fc_.tick();
    if (fc_.fcTextCount() != textSeen_) {
        textSeen_ = fc_.fcTextCount();
        if (p_.verbose) std::printf("[sitl] FC: %s\n", fc_.fcText().c_str());
    }
}

void SitlPlant::start(const sim::CamPose& spawn, float floorZ) {
    spawnE_ = spawn.e; spawnN_ = spawn.n; floorZ_ = floorZ;
}

// SITL truth (geodetic, NED attitude) -> the world's pose: ENU metres from the
// first fix, laid on the spawn point, height from the floor slab. Equirect-
// angular about the origin, which is exact to millimetres over a 160 m world.
void SitlPlant::truthToPose(sim::CamPose& out) const {
    const MavlinkBackend::SimTruth& t = fc_.simTruth();
    const double dN = (t.latDeg - lat0_) * kMPerDegLat;
    const double dE = (t.lonDeg - lon0_) * kMPerDegLat * std::cos(lat0_ * kPi / 180.0);
    out.e = spawnE_ + float(dE);
    out.n = spawnN_ + float(dN);
    out.u = floorZ_ + (t.altMslM - alt0_);
    // Same conventions both sides: yaw clockwise from North, nose-up pitch and
    // right-wing-down roll positive (depth_camera.cpp camToWorld).
    float yaw = float(t.yawRad * 180.0 / kPi);
    while (yaw < 0.f) yaw += 360.f;
    while (yaw >= 360.f) yaw -= 360.f;
    out.yawDeg = yaw;
    out.pitchDeg = float(t.pitchRad * 180.0 / kPi);
    out.rollDeg = float(t.rollRad * 180.0 / kPi);
}

bool SitlPlant::ready(sim::CamPose& truth) {
    service();
    if (stage_ == Stage::FLYING) { truthToPose(truth); return true; }
    if (stage_ == Stage::FAILED) return false;

    const double t = nowS();
    const auto next = [&](Stage s) {
        stage_ = s; stageT_ = t; lastAskS_ = -1e9;
        if (p_.verbose) std::printf("[sitl] %.1f s: %s\n", t - t0_, stageName());
    };
    if (t - t0_ > p_.startTimeoutS) {
        failure_ = std::string("not airborne after ") + std::to_string(int(p_.startTimeoutS)) +
                   " s (stuck at: " + stageName() + "; last FC message: \"" + fc_.fcText() + "\")";
        stage_ = Stage::FAILED;
        return false;
    }
    const bool ask = t - lastAskS_ > 2.0;
    const MavlinkBackend::SimTruth& st = fc_.simTruth();
    switch (stage_) {
    case Stage::LINK:
        if (fc_.linkUp() && fc_.paramsResolved() && st.valid) {
            // The origin: where SITL has the aircraft standing, before it moves.
            lat0_ = st.latDeg; lon0_ = st.lonDeg; alt0_ = st.altMslM;
            originSet_ = true;
            if (p_.verbose) std::printf("%s", fc_.paramReport().c_str());
            next(Stage::GUIDED);
        } else if (t - stageT_ > 10.0 && !st.valid && fc_.linkUp() && ask) {
            lastAskS_ = t;
            std::printf("[sitl] no SIM_STATE yet: is this ArduPilot SITL? (a real FC never sends it)\n");
        }
        break;
    case Stage::GUIDED:
        if (fc_.copterMode() == mav::COPTER_GUIDED) next(Stage::ARM);
        else if (ask) { lastAskS_ = t; fc_.setMode(FcMode::OFFBOARD); }
        break;
    case Stage::ARM: {
        FcTelemetry tel;
        fc_.poll(tel);
        if (tel.armed) { next(Stage::TAKEOFF); fc_.takeoff(p_.takeoffM); lastAskS_ = t; }
        else if (fc_.copterMode() != mav::COPTER_GUIDED) next(Stage::GUIDED);
        else if (ask) { lastAskS_ = t; fc_.arm(false); }   // never forced
        break;
    }
    case Stage::TAKEOFF: {
        const float up = st.altMslM - alt0_;
        FcTelemetry tel;
        fc_.poll(tel);
        if (!tel.armed) next(Stage::ARM);                    // disarmed on the ground
        else if (up > 0.85f * p_.takeoffM) next(Stage::FLYING);
        else if (t - lastAskS_ > 4.0 && up < 0.1f) { lastAskS_ = t; fc_.takeoff(p_.takeoffM); }
        break;
    }
    default: break;
    }
    if (originSet_) truthToPose(truth);
    return stage_ == Stage::FLYING;
}

void SitlPlant::step(const ControlCmd& c, float, sim::CamPose& truth, float& speed) {
    fc_.sendControl(c);
    service();
    truthToPose(truth);
    const MavlinkBackend::SimTruth& t = fc_.simTruth();
    // Signed along the nose, as the kinematic plant's v.
    const float a = truth.yawDeg * float(kPi) / 180.f;
    speed = t.vN * std::cos(a) + t.vE * std::sin(a);
}

void SitlPlant::handOff(WorldState::FcRequest r) {
    if (r == WorldState::FcRequest::LAND) fc_.setMode(FcMode::LAND);
    if (r == WorldState::FcRequest::RTL)  fc_.setMode(FcMode::RTL);
}

void SitlPlant::land(double timeoutS) {
    if (!fc_.linkUp()) return;
    fc_.setMode(FcMode::LAND);
    const double t0 = nowS();
    double ask = t0;
    for (;;) {
        service();
        FcTelemetry tel;
        fc_.poll(tel);
        if (!tel.armed) { if (p_.verbose) std::printf("[sitl] landed and disarmed\n"); return; }
        if (nowS() - t0 > timeoutS) { std::printf("[sitl] still armed after %.0f s of LAND\n", timeoutS); return; }
        if (nowS() - ask > 3.0 && fc_.copterMode() != mav::COPTER_LAND) { ask = nowS(); fc_.setMode(FcMode::LAND); }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

}  // namespace ksitl
