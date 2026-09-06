// Point-and-click front end for kestrel. See kestrel_gui.hpp for why it is a
// separate file that only knows how to build argument lists.
//
// THE COMMAND LINE IS ALWAYS ON SCREEN. Every panel writes the exact
// `kestrel ...` invocation it is about to run into a strip along the bottom,
// so the window is a way to discover the CLI rather than a replacement for it.
// A screenshot of this window is a reproducible run; a screenshot of a GUI that
// hides its arguments is not.
#include "kestrel_gui.hpp"

#include <cstdio>

#ifndef SIM_HAVE_HIGHGUI
// Headless build. Not an error: `track` and `bench` are the two commands a
// reviewer runs in CI, and neither needs a window.
namespace kgui {
int shot(const std::string&, const std::string&) { return 0; }
int check() { return 0; }
int run(const Actions&, const std::string&) {
    std::fprintf(stderr,
        "[kestrel] this build has no OpenCV highgui, so there is no window to open.\n"
        "          Rebuild against an OpenCV with highgui, or use the text menu\n"
        "          and the subcommands -- they do everything the GUI does.\n");
    return -1;
}
}  // namespace kgui
#else

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <system_error>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace kgui {
namespace {

const char* WIN = "kestrel";
const int W = 1060, H = 660;

const cv::Scalar BG   {30, 30, 36};
const cv::Scalar INK  {238, 238, 240};
const cv::Scalar DIM  {150, 150, 160};
const cv::Scalar EDGE {120, 120, 130};
const cv::Scalar OFFB {58, 58, 66};
const cv::Scalar ONB  {70, 140, 60};
const cv::Scalar GO   {170, 110, 40};   // BGR: blue

// ------------------------------------------------------------------ widgets
struct Btn { cv::Rect r; std::string label; int id; bool on = false; bool go = false; };

struct { int x = 0, y = 0; bool clicked = false; } g_mouse;
void onMouse(int ev, int x, int y, int, void*) {
    g_mouse.x = x; g_mouse.y = y;
    if (ev == cv::EVENT_LBUTTONDOWN) g_mouse.clicked = true;
}

void txt(cv::Mat& im, const std::string& s, int x, int y, double sc,
         const cv::Scalar& c, int th = 1) {
    cv::putText(im, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
}

// Truncate to fit maxPx, keeping the TAIL. Paths are what overflow here and
// the distinguishing part of a path is its end, not its beginning.
std::string fit(const std::string& s, int maxPx, double sc, bool keepTail = true) {
    int base = 0;
    if (cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width <= maxPx)
        return s;
    std::string t = s;
    while (t.size() > 4) {
        t = keepTail ? t.substr(1) : t.substr(0, t.size() - 1);
        const std::string probe = keepTail ? "..." + t : t + "...";
        if (cv::getTextSize(probe, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width <= maxPx)
            return probe;
    }
    return t;
}

void drawBtn(cv::Mat& im, const Btn& b) {
    cv::rectangle(im, b.r, b.go ? GO : (b.on ? ONB : OFFB), cv::FILLED);
    cv::rectangle(im, b.r, EDGE, 1);
    const double sc = b.go ? 0.62 : 0.52;
    const std::string label = fit(b.label, b.r.width - 16, sc);
    int base = 0;
    cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base);
    txt(im, label, b.r.x + (b.r.width - ts.width) / 2,
        b.r.y + (b.r.height + ts.height) / 2, sc, INK, b.go ? 2 : 1);
}

// A -/+ pair with the value between them. Returns nothing; the caller matches
// on idMinus / idPlus.
void stepper(cv::Mat& im, std::vector<Btn>& bs, int x, int y, const char* label,
             const std::string& value, int idMinus, int idPlus,
             const char* hint = nullptr) {
    txt(im, label, x, y - 10, 0.5, DIM);
    bs.push_back({cv::Rect(x, y, 34, 34), "-", idMinus});
    bs.push_back({cv::Rect(x + 130, y, 34, 34), "+", idPlus});
    int base = 0;
    cv::Size ts = cv::getTextSize(value, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
    txt(im, value, x + 82 - ts.width / 2, y + 24, 0.6, INK, 2);
    if (hint) txt(im, hint, x, y + 54, 0.42, DIM);
}

// ------------------------------------------------------------ input scanning
bool hasExt(const fs::path& p, const std::vector<std::string>& exts) {
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return std::find(exts.begin(), exts.end(), e) != exts.end();
}

std::vector<std::string> filesIn(const std::string& dir,
                                 const std::vector<std::string>& exts, size_t cap) {
    std::vector<std::string> out;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return out;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        if (ec) break;
        if (!e.is_regular_file() || !hasExt(e.path(), exts)) continue;
        out.push_back(e.path().string());
        if (out.size() >= cap) break;
    }
    std::sort(out.begin(), out.end());
    return out;
}

const std::vector<std::string> IMG_EXT{".png", ".jpg", ".jpeg", ".bmp", ".pgm"};
const std::vector<std::string> VID_EXT{".mp4", ".avi", ".mov", ".mkv", ".m4v"};

// What `track` can be pointed at. A DIRECTORY of images is offered as one
// entry rather than N, because a sequence is a single input -- listing the
// frames individually would let you pick one frame, which tracks nothing.
struct TrackInput {
    std::string label;
    std::vector<std::string> args;   // what goes on the command line
};

std::vector<TrackInput> findTrackInputs(const std::string& exeDir) {
    std::vector<TrackInput> v;
    const std::string roots[] = {".", "frames", "captures", "recordings",
                                 exeDir, exeDir + "/frames"};
    std::vector<std::string> seen;
    for (const std::string& r : roots) {
        std::error_code ec;
        if (!fs::is_directory(r, ec)) continue;
        const std::string canon = fs::weakly_canonical(r, ec).string();
        if (std::find(seen.begin(), seen.end(), canon) != seen.end()) continue;
        seen.push_back(canon);

        const std::vector<std::string> imgs = filesIn(r, IMG_EXT, 4096);
        if (imgs.size() >= 2) {
            // The LAST component is the name; the rest is where it happens to
            // live. exeDir is absolute, so unshortened these all read the same.
            std::string name = fs::path(r).filename().string();
            if (name.empty() || name == ".") name = "./";
            v.push_back({name + "/   " + std::to_string(imgs.size()) + " frames", imgs});
        }
        for (const std::string& f : filesIn(r, VID_EXT, 8))
            v.push_back({fs::path(f).filename().string(), {f}});
        if (v.size() >= 8) break;
    }
    return v;
}

std::vector<std::string> findRecordings(const std::string& exeDir) {
    std::vector<std::string> v = filesIn(".", {".kdr"}, 6);
    for (const std::string& f : filesIn("recordings", {".kdr"}, 6)) v.push_back(f);
    for (const std::string& f : filesIn(exeDir, {".kdr"}, 6)) v.push_back(f);
    return v;
}

// ----------------------------------------------------------------- settings
enum Mode { TRACK = 0, BENCH, SIM, TRAIN, NMODES };
const char* MODE_NAME[NMODES] = {"track", "bench", "sim", "train"};

const int TRAIN_STEPS[] = {50000, 200000, 1000000, 5000000, 10000000, 20000000};
const int NTRAIN_STEPS = int(sizeof TRAIN_STEPS / sizeof *TRAIN_STEPS);

struct Cfg {
    int mode = TRACK;

    // track
    int   input = -1;             // index into inputs, -1 = none chosen
    int   boxSize = 64;
    bool  designate = true;       // click the target on the first frame
    int   frameLimit = 0;         // 0 = all
    bool  csv = true;

    // bench
    bool  forest = true, maze = true;
    int   seed0 = 101, seed1 = 104, steps = 600;
    bool  benchStereo = false;

    // sim
    int   simSource = 0;          // 0 raycaster, 1 live, 2 replay
    int   replay = -1;

    // train
    int   workers = 8, stepsIdx = 4;
    bool  trainStereo = true;
    bool  cuda = false;
};

std::string humanSteps(int n) {
    if (n >= 1000000) return std::to_string(n / 1000000) + " M";
    return std::to_string(n / 1000) + " k";
}

// The single source of truth for what RUN does. The strip along the bottom
// prints exactly this, so what you see is what executes.
std::vector<std::string> buildArgs(const Cfg& c,
                                   const std::vector<TrackInput>& inputs,
                                   const std::vector<std::string>& recs) {
    std::vector<std::string> a;
    switch (c.mode) {
        case TRACK:
            if (c.frameLimit) { a.push_back("--frames"); a.push_back(std::to_string(c.frameLimit)); }
            if (c.csv) { a.push_back("--csv"); a.push_back("track.csv"); }
            if (c.input >= 0 && c.input < int(inputs.size()))
                for (const std::string& f : inputs[c.input].args) a.push_back(f);
            break;
        case BENCH:
            a.push_back("--worlds");
            if (c.forest) a.push_back("forest");
            if (c.maze)   a.push_back("maze");
            a.push_back("--seeds"); a.push_back(std::to_string(c.seed0));
            a.push_back(std::to_string(c.seed1));
            a.push_back("--steps"); a.push_back(std::to_string(c.steps));
            if (c.benchStereo) a.push_back("--stereo");
            break;
        case SIM:
            if (c.simSource == 0) a.push_back("--sim");
            else if (c.simSource == 1) a.push_back("--live");
            else if (c.replay >= 0 && c.replay < int(recs.size())) {
                a.push_back("--replay"); a.push_back(recs[c.replay]);
            }
            break;
        case TRAIN:
            a.push_back("--workers"); a.push_back(std::to_string(c.workers));
            a.push_back("--steps");   a.push_back(std::to_string(TRAIN_STEPS[c.stepsIdx]));
            if (c.trainStereo) a.push_back("--stereo");
            if (c.cuda) { a.push_back("--device"); a.push_back("cuda"); }
            break;
    }
    return a;
}

// Why RUN is refused, or empty if it is not. Saying it beside a greyed button
// beats letting the click do nothing.
std::string blocker(const Cfg& c, const std::vector<TrackInput>& inputs,
                    const std::vector<std::string>& recs) {
    if (c.mode == TRACK && (c.input < 0 || inputs.empty()))
        return inputs.empty()
             ? "no frames or video found - put a folder of images beside this exe"
             : "pick an input first";
    if (c.mode == BENCH && !c.forest && !c.maze) return "pick at least one world";
    if (c.mode == SIM && c.simSource == 2 && (c.replay < 0 || recs.empty()))
        return recs.empty() ? "no .kdr recordings found here" : "pick a recording";
    return "";
}

// ------------------------------------------------------------------- panels
// Button ids. Kept in blocks of 100 per panel so a stray id cannot mean two
// things in two modes.
enum {
    ID_MODE = 0,          // +0..3
    ID_RUN = 10,
    ID_TRACK_INPUT = 100, // +index
    ID_TRACK_SIZE_M = 130, ID_TRACK_SIZE_P, ID_TRACK_DESIG,
    ID_TRACK_LIM_M, ID_TRACK_LIM_P, ID_TRACK_CSV,
    ID_BENCH_FOREST = 200, ID_BENCH_MAZE, ID_BENCH_S0M, ID_BENCH_S0P,
    ID_BENCH_S1M, ID_BENCH_S1P, ID_BENCH_STM, ID_BENCH_STP, ID_BENCH_STEREO,
    ID_SIM_SRC = 300,     // +0..2
    ID_SIM_REPLAY = 310,  // +index
    ID_TRAIN_WM = 400, ID_TRAIN_WP, ID_TRAIN_SM, ID_TRAIN_SP,
    ID_TRAIN_STEREO, ID_TRAIN_CUDA,
};

void panelTrack(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c,
                const std::vector<TrackInput>& inputs) {
    const int x = 266;
    txt(im, "object lock over recorded frames", x, 112, 0.62, INK, 1);
    txt(im, "The tracker that runs on the aircraft, over frames you already have.",
        x, 136, 0.44, DIM);

    txt(im, "input", x, 176, 0.5, DIM);
    if (inputs.empty()) {
        txt(im, "nothing found in ./ , ./frames , ./captures", x, 206, 0.46, DIM);
        txt(im, "a folder of .png/.jpg is a sequence; a .mp4 needs a videoio build",
            x, 228, 0.42, DIM);
    } else {
        int y = 188;
        for (size_t i = 0; i < inputs.size() && i < 5; ++i) {
            bs.push_back({cv::Rect(x, y, 470, 34), inputs[i].label,
                          ID_TRACK_INPUT + int(i), c.input == int(i)});
            y += 40;
        }
    }

    stepper(im, bs, x, 400, "box size (px)", std::to_string(c.boxSize),
            ID_TRACK_SIZE_M, ID_TRACK_SIZE_P, "how big the target is");
    stepper(im, bs, x + 220, 400, "frame limit",
            c.frameLimit ? std::to_string(c.frameLimit) : "all",
            ID_TRACK_LIM_M, ID_TRACK_LIM_P, "0 = the whole sequence");

    bs.push_back({cv::Rect(x, 480, 230, 36),
                  c.designate ? "Click the target" : "Use frame centre",
                  ID_TRACK_DESIG, c.designate});
    txt(im, c.designate ? "click it on frame 1" : "only if it IS centred",
        x, 534, 0.42, DIM);
    bs.push_back({cv::Rect(x + 250, 480, 230, 36),
                  c.csv ? "Write track.csv" : "No CSV", ID_TRACK_CSV, c.csv});
    txt(im, c.csv ? "per-frame state timeline" : "lock rate to the console",
        x + 250, 534, 0.42, DIM);
}

void panelBench(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "path-planner baselines", x, 112, 0.62, INK, 1);
    txt(im, "random / freeM / goal / weighted score, through the SAME environment",
        x, 136, 0.44, DIM);
    txt(im, "a learned policy uses. These are the numbers a policy has to beat.",
        x, 156, 0.44, DIM);

    txt(im, "worlds", x, 196, 0.5, DIM);
    bs.push_back({cv::Rect(x, 208, 140, 36), "Forest", ID_BENCH_FOREST, c.forest});
    bs.push_back({cv::Rect(x + 156, 208, 140, 36), "Maze", ID_BENCH_MAZE, c.maze});

    stepper(im, bs, x, 300, "first seed", std::to_string(c.seed0),
            ID_BENCH_S0M, ID_BENCH_S0P);
    stepper(im, bs, x + 220, 300, "last seed", std::to_string(c.seed1),
            ID_BENCH_S1M, ID_BENCH_S1P);
    stepper(im, bs, x + 440, 300, "steps/run", std::to_string(c.steps),
            ID_BENCH_STM, ID_BENCH_STP);

    const int runs = (c.forest + c.maze) * std::max(0, c.seed1 - c.seed0 + 1) * 4;
    txt(im, std::to_string(runs) + " runs (4 policies x " +
            std::to_string(c.forest + c.maze) + " world(s) x " +
            std::to_string(std::max(0, c.seed1 - c.seed0 + 1)) + " seed(s))",
        x, 392, 0.46, DIM);

    bs.push_back({cv::Rect(x, 430, 250, 36),
                  c.benchStereo ? "Simulated stereo" : "Perfect depth (control)",
                  ID_BENCH_STEREO, c.benchStereo});
    txt(im, "Run both. If it fails on perfect depth the planner is at fault;",
        x, 486, 0.42, DIM);
    txt(im, "if only on stereo, the sensor is the limit.", x, 506, 0.42, DIM);
    txt(im, "Output is a table in the console, not in this window.",
        x, 540, 0.42, DIM);
}

void panelSim(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c,
              const std::vector<std::string>& recs) {
    const int x = 266;
    txt(im, "live voxel sim", x, 112, 0.62, INK, 1);
    txt(im, "The real map, planner and veto over depth. Runs in THIS process --",
        x, 136, 0.44, DIM);
    txt(im, "it is the same code as the standalone voxel_live, not a copy.",
        x, 156, 0.44, DIM);

    txt(im, "depth source", x, 200, 0.5, DIM);
    const char* src[3] = {"Simulated raycaster", "Live D435i", "Replay a recording"};
    for (int i = 0; i < 3; ++i)
        bs.push_back({cv::Rect(x, 212 + i * 46, 250, 38), src[i], ID_SIM_SRC + i,
                      c.simSource == i});

    if (c.simSource == 1) {
        txt(im, "librealsense is loaded at RUN time, so this build needs no SDK.",
            x, 370, 0.42, DIM);
        txt(im, "If it is missing the sim says where it looked.", x, 390, 0.42, DIM);
    } else if (c.simSource == 2) {
        if (recs.empty()) {
            txt(im, "no .kdr files in ./ or ./recordings", x, 370, 0.44, DIM);
        } else {
            int y = 366;
            for (size_t i = 0; i < recs.size() && i < 4; ++i) {
                bs.push_back({cv::Rect(x, y, 400, 34),
                              fs::path(recs[i]).filename().string(),
                              ID_SIM_REPLAY + int(i), c.replay == int(i)});
                y += 40;
            }
        }
    } else {
        txt(im, "No camera needed. The raycaster is the control case: if the", x, 370, 0.42, DIM);
        txt(im, "planner fails here, the sensor is not what is wrong.", x, 390, 0.42, DIM);
    }
    txt(im, "In the sim window:  space pause   v first-person / overlay   s save PNG",
        x, 540, 0.42, DIM);
    txt(im, "                    m its own menu   q back to here", x, 560, 0.42, DIM);
}

void panelTrain(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "RL path-policy training", x, 112, 0.62, INK, 1);
    txt(im, "PyTorch and stable-baselines3 driving the C++ environment. This is",
        x, 136, 0.44, DIM);
    txt(im, "the one command that runs python -- see the note at the bottom.",
        x, 156, 0.44, DIM);

    stepper(im, bs, x, 220, "workers", std::to_string(c.workers),
            ID_TRAIN_WM, ID_TRAIN_WP, "parallel environments");
    stepper(im, bs, x + 220, 220, "steps", humanSteps(TRAIN_STEPS[c.stepsIdx]),
            ID_TRAIN_SM, ID_TRAIN_SP, "checkpointed as it goes");

    bs.push_back({cv::Rect(x, 320, 250, 38),
                  c.trainStereo ? "Simulated stereo" : "Perfect depth",
                  ID_TRAIN_STEREO, c.trainStereo});
    txt(im, "stereo is the honest setting and about 3x slower", x, 376, 0.42, DIM);

    bs.push_back({cv::Rect(x, 404, 250, 38), c.cuda ? "device: cuda" : "device: cpu",
                  ID_TRAIN_CUDA, c.cuda});
    txt(im, "The GPU WILL look idle and that is correct: the bottleneck is", x, 462, 0.42, DIM);
    txt(im, "environment steps, which are C++ on the CPU. cuda is here so you", x, 482, 0.42, DIM);
    txt(im, "can measure that rather than take the claim on trust.", x, 502, 0.42, DIM);

    txt(im, "First time: pip install -r python/requirements.txt", x, 528, 0.44, DIM);
    txt(im, "Progress prints in the console, not here.", x, 550, 0.42, DIM);
}

// ------------------------------------------------------------------- compose
// ONE FUNCTION DRAWS THE WHOLE WINDOW and hands back the buttons it drew, so
// hit-testing cannot disagree with what is on screen. It also means the layout
// can be rendered with no display at all -- see shot() -- which is the only way
// this window is checkable over ssh or in CI. gui_preview does the same thing
// for the sim's flight layout, and for the same reason.
cv::Mat compose(const Cfg& c, const std::vector<TrackInput>& inputs,
                const std::vector<std::string>& recs, std::vector<Btn>& bs) {
    cv::Mat im(H, W, CV_8UC3, BG);
    txt(im, "kestrel", 28, 48, 0.95, INK, 2);
    txt(im, "one binary: object lock, planner baselines, the live voxel sim, RL training",
        28, 74, 0.44, DIM);

    bs.clear();
    for (int i = 0; i < NMODES; ++i)
        bs.push_back({cv::Rect(28, 100 + i * 54, 210, 46), MODE_NAME[i],
                      ID_MODE + i, c.mode == i});

    switch (c.mode) {
        case TRACK: panelTrack(im, bs, c, inputs); break;
        case BENCH: panelBench(im, bs, c); break;
        case SIM:   panelSim(im, bs, c, recs); break;
        default:    panelTrain(im, bs, c); break;
    }

    const std::string why = blocker(c, inputs, recs);
    Btn runBtn{cv::Rect(28, 336, 210, 58), "RUN", ID_RUN};
    runBtn.go = why.empty();
    bs.push_back(runBtn);
    if (!why.empty()) txt(im, why, 28, 414, 0.4, DIM);
    txt(im, "q or esc  quit", 28, H - 26, 0.44, DIM);

    // The command strip. Not decoration: it is what RUN executes.
    cv::rectangle(im, {266, H - 96, W - 294, 44}, {22, 22, 26}, cv::FILLED);
    cv::rectangle(im, {266, H - 96, W - 294, 44}, EDGE, 1);
    std::string cmd = "kestrel " + std::string(MODE_NAME[c.mode]);
    for (const std::string& a : buildArgs(c, inputs, recs)) cmd += " " + a;
    if (c.mode == TRACK && c.designate) cmd += "   (+ --box from your click)";
    txt(im, fit(cmd, W - 318, 0.46, /*keepTail=*/false), 278, H - 68, 0.46, INK);
    txt(im, "this is the command RUN executes -- you can type it instead",
        266, H - 34, 0.4, DIM);

    for (const Btn& b : bs) drawBtn(im, b);
    return im;
}

// ------------------------------------------------------------ click handling
void apply(int id, Cfg& c, const std::vector<TrackInput>& inputs,
           const std::vector<std::string>& recs) {
    if (id >= ID_MODE && id < ID_MODE + NMODES) { c.mode = id - ID_MODE; return; }
    if (id >= ID_TRACK_INPUT && id < ID_TRACK_INPUT + 30) {
        c.input = id - ID_TRACK_INPUT; return;
    }
    if (id >= ID_SIM_SRC && id < ID_SIM_SRC + 3) { c.simSource = id - ID_SIM_SRC; return; }
    if (id >= ID_SIM_REPLAY && id < ID_SIM_REPLAY + 20) {
        c.replay = id - ID_SIM_REPLAY; return;
    }
    switch (id) {
        case ID_TRACK_SIZE_M: c.boxSize = std::max(16, c.boxSize - 16); break;
        case ID_TRACK_SIZE_P: c.boxSize = std::min(256, c.boxSize + 16); break;
        case ID_TRACK_DESIG:  c.designate = !c.designate; break;
        case ID_TRACK_LIM_M:  c.frameLimit = std::max(0, c.frameLimit - 50); break;
        case ID_TRACK_LIM_P:  c.frameLimit = std::min(5000, c.frameLimit + 50); break;
        case ID_TRACK_CSV:    c.csv = !c.csv; break;

        case ID_BENCH_FOREST: c.forest = !c.forest; break;
        case ID_BENCH_MAZE:   c.maze = !c.maze; break;
        case ID_BENCH_S0M:    c.seed0 = std::max(1, c.seed0 - 1);
                              c.seed1 = std::max(c.seed0, c.seed1); break;
        case ID_BENCH_S0P:    c.seed0 = std::min(999, c.seed0 + 1);
                              c.seed1 = std::max(c.seed0, c.seed1); break;
        case ID_BENCH_S1M:    c.seed1 = std::max(c.seed0, c.seed1 - 1); break;
        case ID_BENCH_S1P:    c.seed1 = std::min(999, c.seed1 + 1); break;
        case ID_BENCH_STM:    c.steps = std::max(100, c.steps - 100); break;
        case ID_BENCH_STP:    c.steps = std::min(5000, c.steps + 100); break;
        case ID_BENCH_STEREO: c.benchStereo = !c.benchStereo; break;

        case ID_TRAIN_WM:     c.workers = std::max(1, c.workers - 1); break;
        case ID_TRAIN_WP:     c.workers = std::min(32, c.workers + 1); break;
        case ID_TRAIN_SM:     c.stepsIdx = std::max(0, c.stepsIdx - 1); break;
        case ID_TRAIN_SP:     c.stepsIdx = std::min(NTRAIN_STEPS - 1, c.stepsIdx + 1); break;
        case ID_TRAIN_STEREO: c.trainStereo = !c.trainStereo; break;
        case ID_TRAIN_CUDA:   c.cuda = !c.cuda; break;
        default: break;
    }
    (void)inputs; (void)recs;
}

// ------------------------------------------------------------ designate flow
// Shows the first frame and waits for a click. Returns false if the user closed
// the window or the frame could not be read -- the caller then falls back to
// the frame centre, which is what `track` does with no --box anyway.
bool clickTarget(const std::string& firstFile, float& bx, float& by) {
    cv::Mat f = cv::imread(firstFile, cv::IMREAD_COLOR);
    if (f.empty()) return false;
    cv::Mat shown;
    const double sc = std::min(1.0, std::min(1200.0 / f.cols, 700.0 / f.rows));
    cv::resize(f, shown, {}, sc, sc, cv::INTER_AREA);
    g_mouse.clicked = false;
    for (;;) {
        cv::Mat im = shown.clone();
        cv::rectangle(im, {0, 0, im.cols, 34}, {24, 24, 28}, cv::FILLED);
        txt(im, "click the target   (esc = use the frame centre)", 12, 24, 0.55, INK);
        cv::imshow(WIN, im);
        const int k = cv::waitKey(20);
        if (k == 27 || k == 'q') return false;
        if (g_mouse.clicked) {
            g_mouse.clicked = false;
            if (g_mouse.y < 34) continue;
            bx = float(g_mouse.x / sc);
            by = float(g_mouse.y / sc);
            return true;
        }
    }
}

// IS THERE A SCREEN TO DRAW ON. This has to be answered BEFORE calling into
// highgui, not after: with no display OpenCV's Qt backend calls abort(), which
// no try/catch can intercept -- `kestrel` over ssh died with SIGABRT instead of
// falling back to the text menu it promises. Checking the environment first is
// the only thing that works.
//
// It does not cover a DISPLAY that is SET but unreachable (a stale forwarded
// one); that still aborts. Detecting it would mean opening the connection
// ourselves, and the common case by far is ssh without -X, where DISPLAY is
// simply absent.
bool haveDisplay() {
#if defined(_WIN32) || defined(__APPLE__)
    return true;                      // always a window server
#else
    const char* d = std::getenv("DISPLAY");
    const char* w = std::getenv("WAYLAND_DISPLAY");
    return (d && *d) || (w && *w);
#endif
}

}  // namespace

// ---------------------------------------------------------------------- run
int run(const Actions& act, const std::string& exeDir) {
    if (!haveDisplay()) {
        std::fprintf(stderr,
            "[kestrel] no display (DISPLAY and WAYLAND_DISPLAY are both unset),\n"
            "          so there is no window to open. Falling back to the text menu.\n");
        return -1;
    }
    Cfg c;
    std::vector<TrackInput> inputs = findTrackInputs(exeDir);
    std::vector<std::string> recs = findRecordings(exeDir);
    if (!inputs.empty()) c.input = 0;
    if (!recs.empty())   c.replay = 0;

    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(WIN, onMouse);

    for (;;) {
        std::vector<Btn> bs;
        const std::string why = blocker(c, inputs, recs);
        cv::Mat im = compose(c, inputs, recs, bs);
        cv::imshow(WIN, im);

        const int k = cv::waitKey(20);
        if (k == 'q' || k == 27) break;
        if (!g_mouse.clicked) continue;
        g_mouse.clicked = false;

        int hit = -1;
        for (const Btn& b : bs)
            if (b.r.contains({g_mouse.x, g_mouse.y})) { hit = b.id; break; }
        if (hit < 0) continue;
        if (hit != ID_RUN) { apply(hit, c, inputs, recs); continue; }
        if (!why.empty()) continue;

        std::vector<std::string> args = buildArgs(c, inputs, recs);

        // Designating happens IN this window, before it is torn down, because
        // it needs the first frame on screen and a click on it.
        if (c.mode == TRACK && c.designate && c.input >= 0) {
            float bx = 0, by = 0;
            if (clickTarget(inputs[c.input].args.front(), bx, by)) {
                std::vector<std::string> box{"--box", std::to_string(int(bx)),
                                             std::to_string(int(by)),
                                             std::to_string(c.boxSize)};
                args.insert(args.begin(), box.begin(), box.end());
            }
        } else if (c.mode == TRACK) {
            args.insert(args.begin(), {"--box", "-1", "-1", std::to_string(c.boxSize)});
        }

        // HAND THE SCREEN OVER. track and sim open windows of their own, and
        // bench and train print for minutes to hours; leaving a dead launcher
        // behind either fights for the window or looks hung. It comes back
        // when the command returns.
        cv::destroyWindow(WIN);
        cv::waitKey(1);
        std::printf("\n[kestrel] %s", MODE_NAME[c.mode]);
        for (const std::string& a : args) std::printf(" %s", a.c_str());
        std::printf("\n");
        std::fflush(stdout);

        int rc = 0;
        switch (c.mode) {
            case TRACK: rc = act.track(args); break;
            case BENCH: rc = act.bench(args); break;
            case SIM:   rc = act.sim(args);   break;
            default:    rc = act.train(args); break;
        }
        if (rc != 0) std::printf("[kestrel] %s exited %d\n", MODE_NAME[c.mode], rc);
        std::fflush(stdout);

        // Inputs may have appeared while we were away -- a run of `sim` writes
        // recordings, and `track` may have been pointed at a folder someone
        // filled in the meantime.
        inputs = findTrackInputs(exeDir);
        recs = findRecordings(exeDir);
        if (c.input >= int(inputs.size())) c.input = inputs.empty() ? -1 : 0;
        if (c.replay >= int(recs.size()))  c.replay = recs.empty() ? -1 : 0;

        cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(WIN, onMouse);
    }
    cv::destroyWindow(WIN);
    return 0;
}

int check() {
    // Synthetic inputs, so the result does not depend on what happens to be
    // lying in the working directory when it runs.
    const std::vector<TrackInput> inputs{
        {"frames/   240 frames", {"frames/a.png", "frames/b.png"}},
        {"a_rather_long_recording_name.mp4", {"a_rather_long_recording_name.mp4"}},
    };
    const std::vector<std::string> recs{"one.kdr", "two.kdr"};

    int bad = 0;
    for (int m = 0; m < NMODES; ++m)
        for (int variant = 0; variant < 3; ++variant) {
            // Panels change shape with their own settings -- sim grows a file
            // list, track swaps its hints -- so each is laid out in more than
            // one state rather than only its default.
            Cfg c;
            c.mode = m;
            c.input = 0; c.replay = 0;
            c.simSource = variant;
            c.designate = c.csv = c.forest = (variant != 1);
            c.maze = (variant != 2);
            c.benchStereo = c.trainStereo = c.cuda = (variant == 1);
            c.frameLimit = variant * 50;
            c.stepsIdx = variant;

            std::vector<Btn> bs;
            const cv::Mat im = compose(c, inputs, recs, bs);
            const std::string tag =
                std::string(MODE_NAME[m]) + "/" + std::to_string(variant);

            for (size_t i = 0; i < bs.size(); ++i) {
                const Btn& a = bs[i];
                if ((a.r & cv::Rect(0, 0, im.cols, im.rows)) != a.r) {
                    std::printf("%s: button '%s' is off the canvas\n",
                                tag.c_str(), a.label.c_str());
                    ++bad;
                }
                // A label truncated below this says nothing useful; the box is
                // simply too small for what was put in it.
                const std::string drawn = fit(a.label, a.r.width - 16,
                                              a.go ? 0.62 : 0.52);
                if (!a.label.empty() && drawn.size() < 5 &&
                    drawn.size() < a.label.size()) {
                    std::printf("%s: label '%s' does not fit its %d px button\n",
                                tag.c_str(), a.label.c_str(), a.r.width);
                    ++bad;
                }
                for (size_t j = i + 1; j < bs.size(); ++j) {
                    if (bs[j].id == a.id) {
                        std::printf("%s: id %d used by '%s' and '%s'\n",
                                    tag.c_str(), a.id, a.label.c_str(),
                                    bs[j].label.c_str());
                        ++bad;
                    }
                    if ((a.r & bs[j].r).area() > 0) {
                        std::printf("%s: '%s' overlaps '%s'\n", tag.c_str(),
                                    a.label.c_str(), bs[j].label.c_str());
                        ++bad;
                    }
                }
            }
        }
    std::printf("[gui check] %d layout violation(s)\n", bad);
    return bad;
}

int shot(const std::string& exeDir, const std::string& prefix) {
    const std::vector<TrackInput> inputs = findTrackInputs(exeDir);
    const std::vector<std::string> recs = findRecordings(exeDir);
    int n = 0;
    for (int m = 0; m < NMODES; ++m) {
        Cfg c;
        c.mode = m;
        if (!inputs.empty()) c.input = 0;
        if (!recs.empty())   c.replay = 0;
        if (m == SIM && !recs.empty()) c.simSource = 2;   // show the replay list
        std::vector<Btn> bs;
        const cv::Mat im = compose(c, inputs, recs, bs);
        const std::string f = prefix + "_" + MODE_NAME[m] + ".png";
        if (cv::imwrite(f, im)) { std::printf("%s\n", f.c_str()); ++n; }
    }
    return n;
}

}  // namespace kgui
#endif  // SIM_HAVE_HIGHGUI
