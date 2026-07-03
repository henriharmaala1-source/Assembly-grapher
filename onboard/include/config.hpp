#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "controller.hpp"
#include "control_mode.hpp"
#include "mission.hpp"
#include "rc_command.hpp"

// Runtime configuration — flat `key = value` text, no dependencies.
//
// Gains, thresholds and cadences are otherwise compile-time constants, so field
// tuning means a rebuild. A `--config=kestrel.conf` overrides them at launch;
// `--dump-config` prints the EFFECTIVE values (defaults + overrides) and exits,
// which doubles as the documentation of every knob.
//
//   # comments start with # or ; ; blank lines ignored; `key = value`
//   controller.cruise      = 0.22
//   mission.step_m         = 3.0
//   failsafe.rth_aux       = 4          # 0-based AUX index driven on RTH
//
// Precedence: CLI flags (main.cpp) win over the config file, which wins over
// the built-in defaults. Unknown keys in the file are warned about (typo catch).
class Config {
public:
    // Parse a file. Missing file is NOT an error (returns false, keeps defaults);
    // a malformed line is skipped with a warning.
    bool load(const std::string& path);

    // Typed lookup with a fallback. Each call records the key as "known" and the
    // resolved value (for --dump-config and the unused-key warning).
    float       f(const std::string& key, float def);
    int         i(const std::string& key, int def);
    bool        b(const std::string& key, bool def);
    std::string s(const std::string& key, const std::string& def);

    void warnUnused() const;   // file keys never queried → likely typos
    void dump() const;         // resolved key=value, in query order (effective)

private:
    void record_(const std::string& key, const std::string& val);

    std::map<std::string, std::string>              raw_;       // from file
    std::vector<std::pair<std::string, std::string>> resolved_; // effective
    std::set<std::string>                           queried_;
};

// The full set of runtime-tunable structs, resolved from a Config. FC link and
// mode wiring stay in main's CLI; this is the perception/control/safety tuning.
struct Tunables {
    Controller::Gains          gains;
    MissionController::Params  mission;
    ModeManager::Params        mode;

    // Failsafe → iNAV RTH via an AUX channel (consumed by the MSP backend, P2.1).
    int  rthAuxIdx = -1;    // 0-based AUX index to drive on RTH; <0 = disabled
    int  rthAuxUs  = 1800;  // µs to write on that channel when RTH is active

    // Camera geometry — FPV cameras are up-tilted; the monocular corridor/grid
    // scan is suppressed when the effective elevation leaves the usable window.
    float cameraMountTiltDeg = 0.f;   // + = up vs airframe forward axis
    float scanCamUpMaxDeg    = 12.f;  // suppress grid scan above this elevation
    float scanCamDownMaxDeg  = 40.f;  // …and below this depression

    RcConfig rc;           // radio-as-command-source (P2.2)

    // Real-time scheduling for the control threads (F9). SCHED_FIFO priorities
    // (1..99, 0 = leave normal); FcLink higher — RC cadence is the hard deadline.
    // controlCpu >= 0 pins fly+FcLink to that core (default off; see realtime.hpp).
    struct Rt { bool enable = true; int flyPrio = 10; int fcLinkPrio = 20;
                int controlCpu = -1; } rt;
};

// Read every tunable key out of `c` into a Tunables (also populates the dump /
// known-key set). Call warnUnused()/dump() on `c` afterwards.
Tunables load_tunables(Config& c);
