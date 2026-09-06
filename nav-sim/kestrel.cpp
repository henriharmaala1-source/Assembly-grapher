// kestrel -- one binary for the three things this project can actually do.
//
//   kestrel                       an interactive menu
//   kestrel track  <frames...>    object lock over video or an image sequence
//   kestrel bench  [--worlds ...] the non-RL path-planner baselines
//   kestrel sim    [args...]      the live voxel sim  (spawns voxel_live)
//   kestrel train  [args...]      RL training         (spawns python train.py)
//
// WHY A LAUNCHER AND NOT ONE MONOLITH. Two of these already have a main() and a
// large argument surface of their own, and duplicating that here would create a
// second place for flags to drift out of sync. The two that do NOT have a home
// -- object lock over recorded frames, and the planner baselines -- are built in
// directly, because they are the ones a reviewer actually needs to run.
//
// VIDEO IS OPTIONAL, exactly as highgui already is in this tree. A build without
// OpenCV's videoio can still run `track` over an IMAGE SEQUENCE, which is what a
// frame dump or a set of extracted stills gives you. That is not a lesser mode:
// a sequence is deterministic and diffable where a video decode is neither, so
// it is the better thing to regression-test against.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// Where to send a probe's output so it does not clutter the console.
#ifdef _WIN32
#define NULLDEV "NUL"
#else
#define NULLDEV "/dev/null"
#endif

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#if KESTREL_HAVE_VIDEOIO
#include <opencv2/videoio.hpp>
#endif

#include "lock_tracker_fused.hpp"
#include "rl_env.hpp"

using namespace sim;

namespace {

// ---------------------------------------------------------------------- util
std::string join(const std::vector<std::string>& v, const char* sep) {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) { if (i) s += sep; s += v[i]; }
    return s;
}

int spawn(const std::string& exe, const std::vector<std::string>& args) {
    std::string cmd = "\"" + exe + "\"";
    for (const std::string& a : args) cmd += " " + a;
    std::printf("[kestrel] %s\n", cmd.c_str());
    const int rc = std::system(cmd.c_str());
    if (rc != 0)
        std::fprintf(stderr,
                     "[kestrel] '%s' exited %d. If it was not found, build it first "
                     "(cmake --build build) or pass its path explicitly.\n",
                     exe.c_str(), rc);
    return rc;
}

// ------------------------------------------------------------------- tracking
// BGR/gray -> the tracker's plane layout. The tracker is deliberately
// OpenCV-free (it has to run on a Pi and later an MCU), so the conversion lives
// at the caller, which is here.
track::GrayFrame toGray(const cv::Mat& bgr, std::vector<float>& d,
                        std::vector<float>& u, std::vector<float>& v) {
    cv::Mat yuv;
    if (bgr.channels() == 3) cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV);
    else                     cv::cvtColor(bgr, yuv, cv::COLOR_GRAY2BGR), 
                             cv::cvtColor(yuv, yuv, cv::COLOR_BGR2YUV);
    const int w = yuv.cols, h = yuv.rows;
    d.resize(size_t(w) * h); u.resize(size_t(w) * h); v.resize(size_t(w) * h);
    for (int y = 0; y < h; ++y) {
        const cv::Vec3b* row = yuv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < w; ++x) {
            const size_t i = size_t(y) * w + x;
            d[i] = float(row[x][0]);
            u[i] = float(row[x][1]) - 128.f;
            v[i] = float(row[x][2]) - 128.f;
        }
    }
    track::GrayFrame f;
    f.w = w; f.h = h; f.d = d.data(); f.cu = u.data(); f.cv = v.data();
    return f;
}

struct Source {
    std::vector<std::string> files;      // image-sequence mode
#if KESTREL_HAVE_VIDEOIO
    cv::VideoCapture cap;
#endif
    bool isVideo = false;
    size_t idx = 0;

    bool next(cv::Mat& out) {
#if KESTREL_HAVE_VIDEOIO
        if (isVideo) return cap.read(out) && !out.empty();
#endif
        while (idx < files.size()) {
            out = cv::imread(files[idx++], cv::IMREAD_COLOR);
            if (!out.empty()) return true;
            std::fprintf(stderr, "[kestrel] skipping unreadable %s\n",
                         files[idx - 1].c_str());
        }
        return false;
    }
};

int cmdTrack(std::vector<std::string> args) {
    Source src;
    float bx = -1, by = -1, bsize = 64.f;
    int limit = 0;
    std::string csv;

    for (size_t i = 0; i < args.size(); ++i) {
        auto next = [&](const char* d) { return (i + 1 < args.size()) ? args[++i] : std::string(d); };
        if (args[i] == "--box") {
            bx = std::stof(next("0")); by = std::stof(next("0")); bsize = std::stof(next("64"));
        } else if (args[i] == "--frames") limit = std::stoi(next("0"));
        else if (args[i] == "--csv")      csv = next("");
        else src.files.push_back(args[i]);
    }
    if (src.files.empty()) {
        std::fprintf(stderr,
            "usage: kestrel track [--box X Y SIZE] [--frames N] [--csv OUT] <video|frame...>\n"
            "  --box designates the target on the FIRST frame. Without it the centre\n"
            "  of the frame is used, which is only right if that is where it is.\n");
        return 2;
    }
#if KESTREL_HAVE_VIDEOIO
    if (src.files.size() == 1 && src.files[0].find('.') != std::string::npos) {
        src.cap.open(src.files[0]);
        src.isVideo = src.cap.isOpened();
    }
#else
    if (src.files.size() == 1)
        std::fprintf(stderr, "[kestrel] built without videoio: treating '%s' as an "
                             "image, not a video. Pass a frame sequence instead.\n",
                     src.files[0].c_str());
#endif

    track::LockTracker trk;
    std::vector<float> d, u, v;
    cv::Mat frame;
    std::FILE* out = csv.empty() ? nullptr : std::fopen(csv.c_str(), "w");
    if (out) std::fprintf(out, "frame,state,x,y,w,h,conf\n");

    int n = 0, locked = 0, coasting = 0, searching = 0, lost = 0;
    while (src.next(frame)) {
        track::GrayFrame g = toGray(frame, d, u, v);
        if (n == 0) {
            if (bx < 0) { bx = frame.cols * 0.5f; by = frame.rows * 0.5f; }
            trk.designate(g, bx, by, bsize);
            std::printf("[track] designated (%.0f, %.0f) size %.0f in %dx%d\n",
                        bx, by, bsize, frame.cols, frame.rows);
        }
        const track::LockTracker::Result r = trk.update(g);
        switch (r.state) {
            case track::LockTracker::State::LOCKED:    ++locked; break;
            case track::LockTracker::State::COASTING:  ++coasting; break;
            case track::LockTracker::State::SEARCHING: ++searching; break;
            case track::LockTracker::State::LOST:      ++lost; break;
            default: break;
        }
        if (out)
            std::fprintf(out, "%d,%s,%d,%d,%d,%d,%.3f\n", n,
                         track::LockTracker::stateName(r.state),
                         r.x, r.y, r.w, r.h, r.conf);
        ++n;
        if (limit && n >= limit) break;
    }
    if (out) std::fclose(out);

    if (!n) { std::fprintf(stderr, "[kestrel] no frames read\n"); return 1; }
    // LOCK RATE IS THE HEADLINE and the other states are printed beside it,
    // because "not locked" splits into three situations that want different
    // fixes: coasting is a brief miss, searching is a re-acquire in progress,
    // and lost is a give-up.
    std::printf("[track] %d frames: LOCKED %d (%.0f%%)  COASTING %d  SEARCHING %d  LOST %d\n",
                n, locked, 100.0 * locked / n, coasting, searching, lost);
    if (!csv.empty()) std::printf("[track] per-frame states -> %s\n", csv.c_str());
    return 0;
}

// ------------------------------------------------------------------ baselines
enum class Pol { Random, FreeM, Goal, Score };
const char* polName(Pol p) {
    switch (p) { case Pol::Random: return "random"; case Pol::FreeM: return "freeM";
                 case Pol::Goal: return "goal"; default: return "score"; }
}

int choose(Pol pol, const std::vector<float>& obs, const std::vector<uint8_t>& mask,
           int nPrims, unsigned& rng) {
    const int F = VoxelEnv::obsFeaturesPerPrim();
    std::vector<int> legal;
    for (int i = 0; i < nPrims; ++i) if (mask[i]) legal.push_back(i);
    if (legal.empty()) return 0;
    if (pol == Pol::Random) { rng = rng * 1664525u + 1013904223u; return legal[rng % legal.size()]; }
    int best = legal[0]; float bestV = -1e30f;
    for (int i : legal) {
        const float* o = &obs[size_t(i) * F];
        float vv = (pol == Pol::FreeM) ? o[0]
                 : (pol == Pol::Goal)  ? -o[3]
                 : 0.7f * o[1] - 1.0f * o[3] - 0.25f * std::fabs(o[6])
                   + 0.5f * o[2] - 2.0f * std::max(0.f, o[4]);
        if (vv > bestV) { bestV = vv; best = i; }
    }
    return best;
}

int cmdBench(std::vector<std::string> args) {
    std::vector<std::string> worlds = {"forest", "maze"};
    int s0 = 101, s1 = 104, maxSteps = 600;
    bool stereo = false;
    for (size_t i = 0; i < args.size(); ++i) {
        auto next = [&](const char* d) { return (i + 1 < args.size()) ? args[++i] : std::string(d); };
        if (args[i] == "--worlds") {
            worlds.clear();
            while (i + 1 < args.size() && args[i + 1][0] != '-') worlds.push_back(args[++i]);
        } else if (args[i] == "--seeds") { s0 = std::stoi(next("101")); s1 = std::stoi(next("104")); }
        else if (args[i] == "--steps")   maxSteps = std::stoi(next("600"));
        else if (args[i] == "--stereo")  stereo = true;
    }
    std::printf("baselines through VoxelEnv -- the same harness a learned policy uses\n");
    std::printf("%-8s %-8s %-5s %-16s %9s %9s %9s\n",
                "policy", "world", "seed", "outcome", "travel", "end-dist", "minClr");
    for (Pol pol : {Pol::Random, Pol::FreeM, Pol::Goal, Pol::Score}) {
        double sum = 0; int runs = 0, coll = 0, reach = 0;
        for (const std::string& w : worlds)
            for (int s = s0; s <= s1; ++s) {
                EnvConfig c;
                c.world = w; c.seed = unsigned(s); c.maxSteps = maxSteps;
                c.truthDepth = !stereo;
                c.horizonS = (w == "maze") ? 0.6f : 2.0f;
                VoxelEnv env(c);
                unsigned rng = unsigned(s) * 7919u + unsigned(pol) * 104729u;
                EnvStep st;
                for (int t = 0; t < maxSteps; ++t) {
                    st = env.step(choose(pol, env.observation(), env.actionMask(),
                                         env.nPrims(), rng));
                    if (st.done || st.truncated) break;
                }
                const char* oc = st.reachedGoal ? "reached goal"
                               : st.collisions  ? "COLLIDED" : "ran out of steps";
                std::printf("%-8s %-8s %-5d %-16s %9.1f %9.1f %9.2f\n",
                            polName(pol), w.c_str(), s, oc, st.travelM,
                            st.distToGoalM, st.minClearM);
                std::fflush(stdout);
                sum += st.travelM; ++runs;
                coll += st.collisions ? 1 : 0; reach += st.reachedGoal ? 1 : 0;
            }
        std::printf("  -> %-8s runs %d  collisions %d  goals %d  mean travel %.1f m\n\n",
                    polName(pol), runs, coll, reach, sum / std::max(1, runs));
    }
    return 0;
}

// -------------------------------------------------------------------- training
// PREFLIGHT BEFORE SPAWNING. Training is the one command whose dependencies
// live outside this binary, and a missing one otherwise surfaces as a Python
// traceback several screens long -- which for a double-click workflow reads as
// "it is broken" rather than "run one pip command". Check first, and say the
// command rather than the symptom.
int cmdTrain(const std::string& dir, const std::vector<std::string>& rest) {
    const std::string script = dir + "/../python/train.py";
    const std::string probe =
        "python3 -c \"import gymnasium, stable_baselines3, sb3_contrib\" "
        "> " NULLDEV " 2>&1";
    if (std::system(probe.c_str()) != 0) {
        std::fprintf(stderr,
            "[kestrel] the RL stack is not installed for this python.\n"
            "          pip install -r \"%s/../python/requirements.txt\"\n",
            dir.c_str());
        return 3;
    }
    // The extension module is built beside this exe; train.py adds that
    // directory to sys.path itself, so a missing module here means the build
    // did not produce it rather than a path problem.
    const std::string probe2 =
        "python3 -c \"import sys; sys.path.insert(0, r'" + dir + "'); import voxelenv\" "
        "> " NULLDEV " 2>&1";
    if (std::system(probe2.c_str()) != 0) {
        std::fprintf(stderr,
            "[kestrel] the voxelenv extension module is missing from %s.\n"
            "          Build it: cmake --build build --target voxelenv\n"
            "          (it needs pybind11: pip install pybind11, then re-run cmake)\n",
            dir.c_str());
        return 3;
    }
    std::vector<std::string> a{script};
    for (const std::string& r : rest) a.push_back(r);
    return spawn("python3", a);
}

// ----------------------------------------------------------------------- menu
std::string exeDir(const char* argv0) {
    std::string p(argv0 ? argv0 : "");
    const size_t c = p.find_last_of("/\\");
    return (c == std::string::npos) ? std::string(".") : p.substr(0, c);
}

int menu(const std::string& dir) {
    for (;;) {
        std::printf(
            "\n  KESTREL\n"
            "  1  object lock over recorded frames   (track)\n"
            "  2  path-planner baselines            (bench)\n"
            "  3  live voxel sim                    (spawns voxel_live)\n"
            "  4  RL training                       (spawns python train.py)\n"
            "  q  quit\n"
            "  > ");
        std::fflush(stdout);
        char line[64] = {0};
        if (!std::fgets(line, sizeof line, stdin)) return 0;
        switch (line[0]) {
            case '1': {
                std::printf("  frames (space-separated) or a video path: ");
                std::fflush(stdout);
                char buf[1024] = {0};
                if (!std::fgets(buf, sizeof buf, stdin)) break;
                std::vector<std::string> a;
                for (char* t = std::strtok(buf, " \t\r\n"); t; t = std::strtok(nullptr, " \t\r\n"))
                    a.push_back(t);
                if (a.empty()) { std::printf("  nothing given\n"); break; }
                cmdTrack(a);
                break;
            }
            case '2': cmdBench({}); break;
            case '3': spawn(dir + "/voxel_live", {}); break;
            case '4': cmdTrain(dir, {"--workers", "8"}); break;
            case 'q': case 'Q': return 0;
            default:  std::printf("  ?\n");
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string dir = exeDir(argc ? argv[0] : "");
    if (argc < 2) return menu(dir);

    std::vector<std::string> rest;
    for (int i = 2; i < argc; ++i) rest.push_back(argv[i]);
    const std::string cmd = argv[1];

    if (cmd == "track") return cmdTrack(rest);
    if (cmd == "bench") return cmdBench(rest);
    if (cmd == "sim")   return spawn(dir + "/voxel_live", rest);
    if (cmd == "train") return cmdTrain(dir, rest);
    if (cmd == "--help" || cmd == "-h" || cmd == "help") {
        std::printf("kestrel [track|bench|sim|train] ...   (no args: menu)\n");
        return 0;
    }
    std::fprintf(stderr, "unknown command '%s' -- try --help\n", cmd.c_str());
    return 2;
}
