// The showcase's two maps, flown by the aircraft's own autonomy (FlightShow:
// onboard's VoxelNavModule + MissionController on a simulated stereo D435i).
//
//   flight_show_check                       gallery + hall, seed 101, 45 s each
//   flight_show_check SECONDS SEED...       longer flights, more layouts
//   FLIGHT_LOWRES=1                         at 424x240 (default: 848x480, the
//                                           mode that flies)
//
// What it holds the maps to is CLAUDE.md's objective, not a goal: the aircraft
// must not touch anything (truth clearance under the 0.3 m airframe), and it
// must actually go somewhere -- hovering for ever is safe and useless. A map
// the real stack cannot fly is not a showcase of the real stack.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "flight_show.hpp"

int main(int argc, char** argv) {
    float seconds = 45.f;
    std::vector<unsigned> seeds;
    if (argc > 1) seconds = float(std::atof(argv[1]));
    for (int i = 2; i < argc; ++i) seeds.push_back(unsigned(std::atoi(argv[i])));
    if (seeds.empty()) seeds.push_back(101);
    const bool verbose = std::getenv("FLIGHT_VERBOSE") != nullptr;

    int fails = 0;
    for (const char* world : {"gallery", "hall"}) {
        for (unsigned seed : seeds) {
            kshow::FlightParams p;
            p.world = world; p.seed = seed;
            if (std::getenv("FLIGHT_LOWRES")) { p.camW = 424; p.camH = 240; }
            kshow::FlightShow f(p);
            std::string last;
            const int ticks = int(seconds / kshow::FlightShow::kDt);
            for (int i = 0; i < ticks && !f.stats().stuck; ++i) {
                f.tick();
                if (verbose && f.phase() != last) {
                    const WorldState s = f.state();
                    std::printf("   t=%5.1f %-6s -> %-6s at (%.1f,%.1f) yaw %5.1f  "
                                "leg %.0f deg %.2f m  frames %d\n",
                                f.stats().timeS, last.c_str(), f.phase().c_str(),
                                f.truth().e, f.truth().n, f.truth().yawDeg,
                                s.voxLegBearingDeg, s.voxLegFreeM, s.voxFrames);
                }
                last = f.phase();
            }
            const kshow::FlightStats& st = f.stats();
            std::printf("%-7s seed %u: %.0f s  travel %.1f m  net %.1f m  cells %d  legs %d  "
                        "stops %d  min clearance %.2f m  collisions %d%s\n",
                        world, seed, st.timeS, st.travelM, st.netM, st.cells, st.legs,
                        st.stops, st.minClearM, st.collisions, st.stuck ? "  STUCK" : "");
            const bool ok = st.collisions == 0 && st.legs >= 3 && st.travelM >= 5.f;
            if (!ok) { std::printf("  FAIL\n"); ++fails; }
        }
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
