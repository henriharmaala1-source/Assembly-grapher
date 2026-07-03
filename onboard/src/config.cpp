#include "config.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace {
std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    const auto a = s.find_first_not_of(ws);
    if (a == std::string::npos) return "";
    const auto b = s.find_last_not_of(ws);
    return s.substr(a, b - a + 1);
}
}  // namespace

bool Config::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "[config] %s not found — using defaults\n", path.c_str());
        return false;
    }
    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        // Strip inline comments (# or ;) and surrounding whitespace.
        const auto hash = line.find_first_of("#;");
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            std::fprintf(stderr, "[config] %s:%d skipped (no '='): %s\n",
                         path.c_str(), lineNo, line.c_str());
            continue;
        }
        const std::string key = trim(line.substr(0, eq));
        const std::string val = trim(line.substr(eq + 1));
        if (!key.empty()) raw_[key] = val;
    }
    std::printf("[config] loaded %zu keys from %s\n", raw_.size(), path.c_str());
    return true;
}

void Config::record_(const std::string& key, const std::string& val) {
    queried_.insert(key);
    resolved_.emplace_back(key, val);
}

float Config::f(const std::string& key, float def) {
    auto it = raw_.find(key);
    const float v = (it != raw_.end()) ? std::strtof(it->second.c_str(), nullptr) : def;
    std::ostringstream os; os << v;
    record_(key, os.str());
    return v;
}

int Config::i(const std::string& key, int def) {
    auto it = raw_.find(key);
    const int v = (it != raw_.end()) ? (int)std::strtol(it->second.c_str(), nullptr, 10) : def;
    record_(key, std::to_string(v));
    return v;
}

bool Config::b(const std::string& key, bool def) {
    auto it = raw_.find(key);
    bool v = def;
    if (it != raw_.end()) {
        const std::string& s = it->second;
        v = (s == "1" || s == "true" || s == "yes" || s == "on");
    }
    record_(key, v ? "true" : "false");
    return v;
}

std::string Config::s(const std::string& key, const std::string& def) {
    auto it = raw_.find(key);
    const std::string v = (it != raw_.end()) ? it->second : def;
    record_(key, v);
    return v;
}

void Config::warnUnused() const {
    for (const auto& kv : raw_)
        if (!queried_.count(kv.first))
            std::fprintf(stderr, "[config] WARNING: unknown key '%s' (typo?)\n",
                         kv.first.c_str());
}

void Config::dump() const {
    std::printf("# effective configuration (defaults + overrides)\n");
    for (const auto& kv : resolved_)
        std::printf("%-28s = %s\n", kv.first.c_str(), kv.second.c_str());
}

Tunables load_tunables(Config& c) {
    Tunables t;

    // --- controller gains ---
    t.gains.kpYawNav    = c.f("controller.kp_yaw_nav",    t.gains.kpYawNav);
    t.gains.kpYawRoad   = c.f("controller.kp_yaw_road",   t.gains.kpYawRoad);
    t.gains.kHeadRoad   = c.f("controller.k_head_road",   t.gains.kHeadRoad);
    t.gains.cruise      = c.f("controller.cruise",        t.gains.cruise);
    t.gains.scanYaw     = c.f("controller.scan_yaw",      t.gains.scanYaw);
    t.gains.maxAuthority = c.f("controller.max_authority", t.gains.maxAuthority);

    // --- mission (move-stop-sense) ---
    t.mission.settleSec        = c.f("mission.settle_sec",         t.mission.settleSec);
    t.mission.settleSpeedMs    = c.f("mission.settle_speed_ms",    t.mission.settleSpeedMs);
    t.mission.stepM            = c.f("mission.step_m",             t.mission.stepM);
    t.mission.moveTimeoutSec   = c.f("mission.move_timeout_sec",   t.mission.moveTimeoutSec);
    t.mission.cruise           = c.f("mission.cruise",             t.mission.cruise);
    t.mission.kpYaw            = c.f("mission.kp_yaw",             t.mission.kpYaw);
    t.mission.hFovDeg          = c.f("mission.hfov_deg",           t.mission.hFovDeg);
    t.mission.corridorStaleSec = c.f("mission.corridor_stale_sec", t.mission.corridorStaleSec);
    t.mission.maxEphM          = c.f("mission.max_eph_m",          t.mission.maxEphM);
    t.mission.minOpenToMove    = c.f("mission.min_open_to_move",   t.mission.minOpenToMove);
    t.mission.minOpenToKeep    = c.f("mission.min_open_to_keep",   t.mission.minOpenToKeep);
    t.mission.scanYawRate      = c.f("mission.scan_yaw_rate",      t.mission.scanYawRate);
    t.mission.scanTimeoutSec   = c.f("mission.scan_timeout_sec",   t.mission.scanTimeoutSec);

    // P5b occupancy grid + planner
    t.mission.useMap           = c.b("nav.use_map",        t.mission.useMap);
    t.mission.planBerthM       = c.f("nav.plan_berth_m",   t.mission.planBerthM);
    t.mission.map.sizeM        = c.f("nav.grid_size_m",    t.mission.map.sizeM);
    t.mission.map.cellM        = c.f("nav.grid_cell_m",    t.mission.map.cellM);

    // --- mode-manager safety layers ---
    t.mode.rtlBattPct       = c.f("safety.rtl_batt_pct",        t.mode.rtlBattPct);
    t.mode.blockedOpen      = c.f("safety.blocked_open",        t.mode.blockedOpen);
    t.mode.corridorStaleSec = c.f("safety.corridor_stale_sec",  t.mode.corridorStaleSec);

    // --- failsafe RTH AUX (P2.1) ---
    t.rthAuxIdx = c.i("failsafe.rth_aux",    t.rthAuxIdx);
    t.rthAuxUs  = c.i("failsafe.rth_aux_us", t.rthAuxUs);

    // --- camera geometry (monocular tilt handling) ---
    t.cameraMountTiltDeg = c.f("camera.mount_tilt_deg",    t.cameraMountTiltDeg);
    t.scanCamUpMaxDeg    = c.f("nav.scan_cam_up_max_deg",  t.scanCamUpMaxDeg);
    t.scanCamDownMaxDeg  = c.f("nav.scan_cam_down_max_deg", t.scanCamDownMaxDeg);

    // --- RC command source (P2.2) ---
    t.rc.modeAux        = c.i("rc.mode_aux",         t.rc.modeAux);
    t.rc.goAux          = c.i("rc.go_aux",           t.rc.goAux);
    t.rc.goUs           = c.i("rc.go_us",            t.rc.goUs);
    t.rc.steerAux       = c.i("rc.steer_aux",        t.rc.steerAux);
    t.rc.steerRateDps   = c.f("rc.steer_rate_dps",   t.rc.steerRateDps);
    t.rc.steerDeadbandUs = c.i("rc.steer_deadband_us", t.rc.steerDeadbandUs);
    // mode_map: comma-separated mode names, low→high band order.
    const std::string map = c.s("rc.mode_map", "FLY,ASSIST,WAYPOINT,AUTONOMY,SHADOW,HOLD");
    { std::string cur; for (char ch : map) {
        if (ch == ',') { if (!cur.empty()) t.rc.modeMap.push_back(cur); cur.clear(); }
        else if (ch != ' ') cur += ch; }
      if (!cur.empty()) t.rc.modeMap.push_back(cur); }

    // --- real-time scheduling of the control threads (F9) ---
    t.rt.enable     = c.b("rt.enable",      t.rt.enable);
    t.rt.flyPrio    = c.i("rt.fly_prio",    t.rt.flyPrio);
    t.rt.fcLinkPrio = c.i("rt.fclink_prio", t.rt.fcLinkPrio);
    t.rt.controlCpu = c.i("rt.control_cpu", t.rt.controlCpu);

    return t;
}
