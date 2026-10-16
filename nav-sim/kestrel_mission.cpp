// `kestrel mission` -- write a mission on the ground, check it, compile it,
// fly it in the simulator, and hand the aircraft only the compiled file.
//
//   kestrel mission check   FILE.kms             errors with line:col, then the listing
//   kestrel mission compile FILE.kms [-o OUT.kmb] the file the Pi loads (--script)
//   kestrel mission sim     FILE.kms|.kmb [--world gallery|hall] [--seed N]
//                           [--seconds S] [--lowres] [--shot PREFIX]
//   kestrel mission editor  [--print]            the visual editor, in the browser
//   kestrel mission playground [--print]         the 3D crosshair-and-throttle playground
//
// `sim` flies the program on the aircraft's own SCRIPT mode, MissionController
// and VoxelNavModule (flight_show.hpp) against a simulated D435i -- the same
// interpreter and the same certified legs the Pi runs, not a model of them.
// The language is documented in onboard/docs/mission-scripts.md.
#include "kestrel_mission.hpp"
#include "kestrel_scenarios.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "flight_show.hpp"
#include "flight_view.hpp"
#include "mission_compile.hpp"
#include "mission_program.hpp"

namespace kmission {

namespace {

bool endsWith(const std::string& s, const char* suf) {
    const size_t n = std::char_traits<char>::length(suf);
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
}

std::string swapExt(const std::string& path, const char* ext) {
    const size_t slash = path.find_last_of("/\\");
    const size_t dot = path.find_last_of('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
        return path.substr(0, dot) + ext;
    return path + ext;
}

bool fileExists(const std::string& p) { std::ifstream f(p); return bool(f); }

// A .kms is compiled (and its diagnostics printed); a .kmb is loaded and
// verified exactly as the Pi would.
bool loadAny(const std::string& path, kms::Program& out, bool quietOk) {
    if (endsWith(path, ".kmb")) {
        std::string err;
        if (!kms::loadProgram(path, out, &err)) {
            std::fprintf(stderr, "%s: %s\n", path.c_str(), err.c_str());
            return false;
        }
        return true;
    }
    const kms::CompileResult r = kms::compileFile(path);
    const std::string rep = r.report(path);
    if (!rep.empty() && (!r.ok || !quietOk)) std::fputs(rep.c_str(), stderr);
    if (!r.ok) {
        int n = 0;
        for (const auto& d : r.diags) n += d.error;
        std::fprintf(stderr, "%d error%s: nothing was compiled.\n", n, n == 1 ? "" : "s");
        return false;
    }
    out = r.program;
    return true;
}

int usage() {
    std::fprintf(stderr,
        "kestrel mission check   FILE.kms\n"
        "kestrel mission compile FILE.kms [-o OUT.kmb]\n"
        "kestrel mission sim     FILE.kms|FILE.kmb [--world gallery|hall] [--seed N]\n"
        "                        [--seconds S] [--lowres] [--shot PREFIX]\n"
        "kestrel mission scenarios FILE.kms [--runs N] [--seed S] [--target KIND] [--out DIR]\n"
        "                        [--no-video] [--show] [--tilt DEG] [--hfov DEG] [--det-hz HZ]\n"
        "kestrel mission editor  [--print]\n"
        "kestrel mission playground [--print]  (crosshair + throttle, in 3D)\n"
        "The language: onboard/docs/mission-scripts.md\n");
    return 2;
}

int check(const std::vector<std::string>& a) {
    if (a.size() < 1) return usage();
    const kms::CompileResult r = kms::compileFile(a[0]);
    std::fputs(r.report(a[0]).c_str(), stdout);
    if (!r.ok) {
        int n = 0;
        for (const auto& d : r.diags) n += d.error;
        std::printf("%d error%s.\n", n, n == 1 ? "" : "s");
        return 1;
    }
    std::fputs(kms::disassemble(r.program).c_str(), stdout);
    std::printf("OK: %zu instructions, %zu state%s. This is what the aircraft will run.\n",
                r.program.code.size(), r.program.states.size(),
                r.program.states.size() == 1 ? "" : "s");
    return 0;
}

int compile(const std::vector<std::string>& a) {
    if (a.size() < 1) return usage();
    std::string out;
    for (size_t i = 1; i + 1 < a.size(); ++i)
        if (a[i] == "-o") out = a[i + 1];
    if (out.empty()) out = swapExt(a[0], ".kmb");
    kms::Program p;
    if (!loadAny(a[0], p, false)) return 1;
    std::string err;
    if (!kms::saveProgram(p, out, &err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
    // Read it back exactly as the Pi will: what was written is what loads.
    kms::Program back;
    if (!kms::loadProgram(out, back, &err)) {
        std::fprintf(stderr, "wrote %s but it does not load back: %s\n", out.c_str(), err.c_str());
        return 1;
    }
    std::printf("wrote %s: \"%s\", %zu instructions, checksum verified.\n"
                "On the aircraft:  kestrel --script %s   (then select SCRIPT, then GO)\n",
                out.c_str(), p.name.c_str(), p.code.size(), out.c_str());
    return 0;
}

int sim(const std::vector<std::string>& a) {
    if (a.size() < 1) return usage();
    kshow::FlightParams fp;
    fp.world = "gallery";
    float seconds = 300.f;
    std::string shot;
    for (size_t i = 1; i < a.size(); ++i) {
        const bool more = i + 1 < a.size();
        if (a[i] == "--world" && more) fp.world = a[++i];
        else if (a[i] == "--seed" && more) fp.seed = unsigned(std::atoi(a[++i].c_str()));
        else if (a[i] == "--seconds" && more) seconds = float(std::atof(a[++i].c_str()));
        else if (a[i] == "--shot" && more) shot = a[++i];
        else if (a[i] == "--lowres") { fp.camW = 424; fp.camH = 240; }
        else { std::fprintf(stderr, "[mission] unknown argument: %s\n", a[i].c_str()); return usage(); }
    }
    if (fp.world != "gallery" && fp.world != "hall") {
        std::fprintf(stderr, "[mission] --world takes gallery or hall (got '%s')\n", fp.world.c_str());
        return 2;
    }
    auto prog = std::make_shared<kms::Program>();
    if (!loadAny(a[0], *prog, true)) return 1;
    if (prog->caps & kms::Program::NEEDS_DETECTOR)
        std::printf("[mission] note: the simulator has no detector -- `seen`/`track` never "
                    "fire here, so their else branches and timeouts are what you will see.\n");
    fp.script = prog;
    std::printf("[mission] flying \"%s\" in the %s (seed %u), up to %.0f s, on the aircraft's "
                "SCRIPT mode...\n", prog->name.c_str(), fp.world.c_str(), fp.seed, seconds);
    kshow::FlightShow f(fp);
    if (!f.script()) return 1;
    std::string last;
    const int maxTicks = int(seconds / kshow::FlightShow::kDt);
    for (int t = 0; t < maxTicks && !f.scriptEnded(); ++t) {
        f.tick();
        const WorldState w = f.state();
        const std::string now = cv::format("line %d%s%s  %s", w.scriptLine,
                                           w.scriptState.empty() ? "" : "  state ",
                                           w.scriptState.c_str(), f.script()->status().c_str());
        // Print a line when the instruction or state changes, not every tick.
        const std::string key = cv::format("%d|%s", w.scriptLine, w.scriptState.c_str());
        if (key != last) {
            std::printf("  t=%6.1f s  %s\n", f.stats().timeS, now.c_str());
            last = key;
        }
    }
    const kshow::FlightStats& st = f.stats();
    const WorldState w = f.state();
    const bool ok = f.scriptEnded() && !f.script()->failed() && st.collisions == 0;
    std::printf("\n[mission] %s\n", f.script()->status().c_str());
    std::printf("[mission] %.0f s, flown %.1f m, %d legs, closest %.2f m, collisions %d%s\n",
                st.timeS, st.travelM, st.legs, st.minClearM > 100.f ? 0.f : st.minClearM,
                st.collisions, f.scriptEnded() ? "" : "  (cut off by --seconds)");
    if (w.fcRequest == WorldState::FcRequest::LAND) std::printf("[mission] ended with LAND\n");
    if (w.fcRequest == WorldState::FcRequest::RTL) std::printf("[mission] ended with RTL\n");
    if (!shot.empty()) {
        const kshow::ShowSnap snap = kshow::ShowSnap::of(f, nullptr);
        kshow::FlightView view;
        view.update(snap);
        const bool a1 = cv::imwrite(shot + "_mission.png", view.mission(snap, 720, 720));
        const bool a2 = cv::imwrite(shot + "_chase.png", view.chase(snap, 960, 540));
        std::printf("[mission] %s %s_mission.png and %s_chase.png\n",
                    a1 && a2 ? "wrote" : "could not write", shot.c_str(), shot.c_str());
    }
    return ok ? 0 : 1;
}

int openPage(const std::vector<std::string>& a, const std::string& exeDir, const std::string& file,
             const char* what) {
    bool printOnly = false;
    for (const auto& s : a) printOnly |= s == "--print";
    const std::vector<std::string> cand = {exeDir + "/" + file, exeDir + "/../" + file, file,
                                           "nav-sim/" + file};
    std::string path;
    for (const auto& c : cand)
        if (fileExists(c)) { path = c; break; }
    if (path.empty()) {
        std::fprintf(stderr, "[mission] %s is not next to kestrel. It is nav-sim/%s in the "
                             "source tree.\n", file.c_str(), file.c_str());
        return 1;
    }
    std::printf("[mission] %s: %s\n", what, path.c_str());
    if (printOnly) return 0;
#ifdef _WIN32
    const std::string cmd = "start \"\" \"" + path + "\"";
#else
    const std::string cmd = "(command -v xdg-open >/dev/null && xdg-open \"" + path +
                            "\" || open \"" + path + "\") >/dev/null 2>&1 &";
#endif
    if (std::system(cmd.c_str()) != 0)
        std::printf("[mission] could not open a browser here: open that file by hand.\n");
    return 0;
}

}  // namespace

int run(const std::vector<std::string>& args, const std::string& exeDir) {
    if (args.empty()) return usage();
    const std::string sub = args[0];
    const std::vector<std::string> rest(args.begin() + 1, args.end());
    if (sub == "check") return check(rest);
    if (sub == "compile") return compile(rest);
    if (sub == "sim") return sim(rest);
    // Randomised encounters on the aircraft's own SCRIPT mode, as FPV video.
    if (sub == "scenarios") return kscen::run(rest);
    if (sub == "editor") return openPage(rest, exeDir, "mission_editor.html", "the visual editor");
    // The 3D world where a crosshair and a throttle are the controls: fly
    // it live, record the moves, copy them out as `fly crosshair` / `steer`.
    if (sub == "playground")
        return openPage(rest, exeDir, "control_playground.html", "the 3D control playground");
    std::fprintf(stderr, "[mission] unknown: %s\n", sub.c_str());
    return usage();
}

}  // namespace kmission
