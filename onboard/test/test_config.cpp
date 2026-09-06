// Config parser + tunable resolution. Writes a temp file, checks overrides land
// in the right struct fields, defaults hold where unset, bad lines are skipped,
// and typo keys are detectable.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include "config.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

int main() {
    const std::string path = "/tmp/kestrel_test.conf";
    {
        std::ofstream o(path);
        o << "# a comment\n";
        o << "controller.cruise = 0.22   ; inline comment\n";
        o << "mission.step_m = 3.0\n";
        o << "mission.max_eph_m=2.5\n";               // no spaces
        o << "safety.blocked_open = 0.4\n";
        o << "failsafe.rth_aux = 4\n";
        o << "\n";                                     // blank
        o << "this line has no equals\n";             // malformed → skipped
        o << "mission.typo_key = 9\n";                // unknown → warned
    }

    Config c;
    CHECK(c.load(path));
    const Tunables t = load_tunables(c);

    CHECK(std::fabs(t.gains.cruise - 0.22f) < 1e-6);           // overridden
    CHECK(std::fabs(t.mission.stepM - 3.0f) < 1e-6);
    CHECK(std::fabs(t.mission.maxEphM - 2.5f) < 1e-6);         // no-space parse
    CHECK(std::fabs(t.mode.blockedOpen - 0.4f) < 1e-6);
    CHECK(t.rthAuxIdx == 4);

    // Defaults survive where unset.
    Tunables d;   // fresh defaults
    CHECK(std::fabs(t.mission.settleSec - d.mission.settleSec) < 1e-6);
    CHECK(std::fabs(t.gains.maxAuthority - d.gains.maxAuthority) < 1e-6);
    CHECK(t.rthAuxUs == d.rthAuxUs);

    // A missing file is non-fatal and yields pure defaults.
    Config miss;
    CHECK(!miss.load("/tmp/does_not_exist_kestrel.conf"));
    const Tunables md = load_tunables(miss);
    CHECK(std::fabs(md.mission.stepM - d.mission.stepM) < 1e-6);

    std::remove(path.c_str());
    std::printf("test_config: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
