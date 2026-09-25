// Point-and-click front end for kestrel. See kestrel_gui.hpp for why it is a
// separate file that only knows how to build argument lists.
//
// THE COMMAND LINE IS ALWAYS ON SCREEN. Every panel writes the exact
// `kestrel ...` invocation it is about to run into a strip along the bottom,
// so the window is a way to discover the CLI rather than a replacement for it.
// A screenshot of this window is a reproducible run; a screenshot of a GUI that
// hides its arguments is not.
#include "kestrel_gui.hpp"
#include "kestrel_python.hpp"

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

// THE LOOK. BGR. A dark app palette: one accent (blue) for "selected", one
// (green) for the action, and everything else in greys, so colour always
// means something.
const cv::Scalar BG      {34, 30, 27};     // content
const cv::Scalar HEADER  {44, 39, 35};
const cv::Scalar SIDEBAR {40, 35, 32};
const cv::Scalar SURFACE {62, 56, 51};     // an idle button
const cv::Scalar HOVER   {80, 73, 66};
const cv::Scalar INK     {242, 240, 238};
const cv::Scalar DIM     {160, 152, 146};
const cv::Scalar LABEL   {196, 170, 132};  // section headings
const cv::Scalar EDGE    {84, 77, 70};
const cv::Scalar ACCENT  {214, 146, 60};   // selected
const cv::Scalar OFFB    = SURFACE;
const cv::Scalar ONB     = ACCENT;
const cv::Scalar GO      {92, 170, 78};    // RUN
const cv::Scalar CONSOLE {24, 21, 19};

// Layout of the chrome, shared by compose() and the check.
const int HEADER_H = 84;
const int SIDEBAR_W = 252;
const int NAV_Y0 = 100, NAV_PITCH = 46;

// ------------------------------------------------------------------ widgets
// kind: 0 a button, 1 a sidebar navigation row.
struct Btn { cv::Rect r; std::string label; int id; bool on = false; bool go = false;
             int kind = 0; };

struct { int x = 0, y = 0; bool clicked = false; } g_mouse;
void onMouse(int ev, int x, int y, int, void*) {
    g_mouse.x = x; g_mouse.y = y;
    if (ev == cv::EVENT_LBUTTONDOWN) g_mouse.clicked = true;
}

// WHAT THE LAYOUT CHECK CAN SEE. It compared buttons against buttons and
// nothing else, so a paragraph running underneath a button passed cleanly --
// which is exactly what the python status line did when the fifth mode arrived.
// Every txt() records its box here while checking, so text-over-button is now
// a violation too. Null in normal drawing; drawBtn suppresses it, since a
// button's own label is meant to be inside it.
std::vector<cv::Rect>* g_textBoxes = nullptr;

void txt(cv::Mat& im, const std::string& s, int x, int y, double sc,
         const cv::Scalar& c, int th = 1) {
    cv::putText(im, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
    if (g_textBoxes && !s.empty()) {
        int base = 0;
        const cv::Size ts = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, th, &base);
        g_textBoxes->push_back(cv::Rect(x, y - ts.height, ts.width, ts.height + base));
    }
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

// A filled rounded rectangle (OpenCV has none): two crossing rectangles and
// four anti-aliased corner discs.
void roundRect(cv::Mat& im, const cv::Rect& r, int rad, const cv::Scalar& c) {
    rad = std::max(0, std::min(rad, std::min(r.width, r.height) / 2));
    cv::rectangle(im, {r.x + rad, r.y, r.width - 2 * rad, r.height}, c, cv::FILLED);
    cv::rectangle(im, {r.x, r.y + rad, r.width, r.height - 2 * rad}, c, cv::FILLED);
    for (const cv::Point& p : {cv::Point(r.x + rad, r.y + rad),
                               cv::Point(r.x + r.width - 1 - rad, r.y + rad),
                               cv::Point(r.x + rad, r.y + r.height - 1 - rad),
                               cv::Point(r.x + r.width - 1 - rad, r.y + r.height - 1 - rad)})
        cv::circle(im, p, rad, c, cv::FILLED, cv::LINE_AA);
}

bool hovered(const cv::Rect& r) { return r.contains({g_mouse.x, g_mouse.y}); }

// A section heading: small capitals in the heading colour.
void section(cv::Mat& im, const std::string& s, int x, int y) {
    std::string u = s;
    for (char& ch : u) ch = char(std::toupper((unsigned char)ch));
    txt(im, u, x, y, 0.42, LABEL, 1);
}

void drawBtn(cv::Mat& im, const Btn& b) {
    std::vector<cv::Rect>* keep = g_textBoxes;
    g_textBoxes = nullptr;                       // a label belongs in its button
    struct Restore { std::vector<cv::Rect>*& g; std::vector<cv::Rect>* v;
                     ~Restore() { g = v; } } restore{g_textBoxes, keep};
    const bool hov = hovered(b.r);
    if (b.kind == 1) {
        // SIDEBAR ROW: no box unless selected or hovered; the selected one
        // gets a lighter row and an accent bar, like any app's navigation.
        if (b.on || hov) roundRect(im, b.r, 6, b.on ? SURFACE : HOVER);
        if (b.on) roundRect(im, {b.r.x, b.r.y + 8, 4, b.r.height - 16}, 2, ACCENT);
        std::string u = b.label;
        if (!u.empty()) u[0] = char(std::toupper((unsigned char)u[0]));
        int base = 0;
        const cv::Size ts = cv::getTextSize(u, cv::FONT_HERSHEY_SIMPLEX, 0.56, 1, &base);
        txt(im, u, b.r.x + 20, b.r.y + (b.r.height + ts.height) / 2, 0.56,
            b.on ? INK : DIM, b.on ? 2 : 1);
        return;
    }
    if (b.go) {
        // THE ACTION: green when it can run, a flat grey when it cannot.
        roundRect(im, b.r, 10, hov ? cv::Scalar(110, 190, 96) : GO);
        const int cy = b.r.y + b.r.height / 2, cx = b.r.x + b.r.width / 2 - 34;
        const std::vector<cv::Point> tri{{cx, cy - 11}, {cx, cy + 11}, {cx + 17, cy}};
        cv::fillConvexPoly(im, tri, INK, cv::LINE_AA);
        txt(im, b.label, cx + 28, cy + 9, 0.72, INK, 2);
        return;
    }
    roundRect(im, b.r, 7, b.on ? (hov ? cv::Scalar(230, 166, 86) : ONB)
                               : (hov ? HOVER : OFFB));
    const double sc = 0.52;
    // KEEP THE FRONT OF A BUTTON LABEL. fit() defaults to keeping the TAIL,
    // which is right for a file path -- you want the filename -- and exactly
    // wrong for a two-state toggle, where the FIRST word is the state. The
    // watch panel's sampling button rendered as "... (see what training does)":
    // the elision ate "sample", the only word that said which mode it was in,
    // and left the parenthetical that is identical in spirit for both states.
    const std::string label = fit(b.label, b.r.width - 16, sc, /*keepTail=*/false);
    int base = 0;
    cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base);
    txt(im, label, b.r.x + (b.r.width - ts.width) / 2,
        b.r.y + (b.r.height + ts.height) / 2, sc, INK, b.go ? 2 : 1);
}

// A -/+ pair with the value between them. Returns nothing; the caller matches
// on idMinus / idPlus.
// `w` is the gap between the - and + buttons. The train panel's second row
// carries five settings and the default 130 only fits four across the panel.
void stepper(cv::Mat& im, std::vector<Btn>& bs, int x, int y, const char* label,
             const std::string& value, int idMinus, int idPlus,
             const char* hint = nullptr, int w = 130) {
    section(im, label, x, y - 10);
    bs.push_back({cv::Rect(x, y, 34, 34), "-", idMinus});
    bs.push_back({cv::Rect(x + w, y, 34, 34), "+", idPlus});
    int base = 0;
    cv::Size ts = cv::getTextSize(value, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
    txt(im, value, x + (w + 34) / 2 - ts.width / 2, y + 24, 0.6, INK, 2);
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

// EXPORTED POLICIES. A .onnx beside the exe or under runs/ is a checkpoint
// someone has already put through export_onnx.py's agreement check, so it is
// safe to offer. Without one the demo flies a classical planner and says so --
// which is correct, and is also the single most likely reason for someone to
// come away thinking the learned policy is what they watched when it was not.
std::vector<std::string> findModels(const std::string& exeDir) {
    std::vector<std::string> v = filesIn(".", {".onnx"}, 6);
    for (const std::string& f : filesIn("runs", {".onnx"}, 6)) v.push_back(f);
    for (const std::string& f : filesIn(exeDir, {".onnx"}, 6)) v.push_back(f);
    // DEDUPE BY THE PATH THE FILESYSTEM AGREES ON. "." and exeDir are the same
    // directory whenever the exe is run from beside itself, which is the normal
    // case, so every model was listed twice -- two buttons, same file, one of
    // them lit. Comparing the strings would not catch it; comparing what they
    // resolve to does.
    std::vector<std::string> out;
    std::vector<std::string> seen;
    for (const std::string& f : v) {
        std::error_code ec;
        const std::string key = fs::weakly_canonical(f, ec).string();
        const std::string k = ec ? f : key;
        if (std::find(seen.begin(), seen.end(), k) != seen.end()) continue;
        seen.push_back(k);
        out.push_back(f);
    }
    return out;
}

// --------------------------------------------------------------- python state
// WHICH PYTHON, shown on the train panel. Discovery starts several
// interpreters, so it runs once when the window opens and again after anything
// that could change the answer -- never per frame.
struct PyState {
    std::vector<kpy::Py> pys;
    std::string abi;
    bool probed = false;
};
PyState g_py;

// FILE SCOPE, refreshed beside the python probe, for the same reason g_py is:
// threading a fifth list through compose(), panelX() and buildArgs() would
// touch every panel to serve one. Refreshed whenever a command returns, since
// running the export is exactly what creates one of these.
std::vector<std::string> g_models;
void refreshModels(const std::string& dir) { g_models = findModels(dir); }

void refreshPy(const std::string& dir) {
    g_py.pys = kpy::discover(dir);
    g_py.abi = kpy::moduleAbi(dir);
    g_py.probed = true;
}

// ----------------------------------------------------------------- settings
enum Mode { TRACK = 0, BENCH, SIM, DEMO, TRAIN, WATCH, EVAL, REPORT, NMODES };
const char* MODE_NAME[NMODES] = {"track", "bench", "sim", "demo", "train",
                                 "watch", "evaluate", "report"};

// 0 means FOREVER -- run until stopped by hand, saving on the way out. The
// rest are close enough together that a run can be sized without dropping to
// the command line, which the old six-entry list could not do.
const int TRAIN_STEPS[] = {20000, 50000, 100000, 250000, 500000, 1000000,
                           2000000, 5000000, 10000000, 20000000, 50000000,
                           100000000, 0};
const int NTRAIN_STEPS = int(sizeof TRAIN_STEPS / sizeof *TRAIN_STEPS);
// How often a checkpoint is written. Matters most for a forever run, where
// the checkpoints ARE the record of how it progressed.
const int SAVE_EVERY[] = {10000, 25000, 50000, 100000, 250000};
const int NSAVE_EVERY = int(sizeof SAVE_EVERY / sizeof *SAVE_EVERY);

// HOW MUCH RANDOM STUFF THE POLICY TRIES -- the PPO entropy bonus at the start
// of the run, decaying to a tenth of it over the anneal. 0 turns exploration
// pressure off. The old fixed value was 0.01 and never moved; 0.02 is the new
// default because a 210-way action space over six worlds is a lot to search.
const float EXPLORE[] = {0.f, 0.005f, 0.01f, 0.02f, 0.04f, 0.08f, 0.15f};
const int NEXPLORE = int(sizeof EXPLORE / sizeof *EXPLORE);
// Steps over which the learning rate and explore fall to a tenth. -1 is auto
// (the run length, or 20 M for a forever run); 0 holds both constant, which is
// what the 15 M run that degraded after 14 M did.
const int ANNEAL[] = {-1, 0, 1000000, 2000000, 5000000, 10000000, 20000000,
                      50000000, 100000000};
const int NANNEAL = int(sizeof ANNEAL / sizeof *ANNEAL);
// How far one update is allowed to move the policy before the rest of it is
// abandoned. 0 = off, which is what PPO does without it.
// HORIZONS IN STEPS, NOT AS 0.999. gamma = 1 - 1/steps, and the step count is
// the number that can be compared against something: a journey takes 573 to
// 2042 steps (the panel's own steps/episode, and journey_fit's table), so a
// value horizon of 200 -- which is what gamma 0.995 meant -- could not see the
// end of any of them. Naming the discount instead of the horizon is why that
// went unnoticed through a fifteen-million-step run.
const int VALUE_H[] = {100, 200, 500, 1000, 2000, 4000, 10000};
const int NVALUE_H = int(sizeof VALUE_H / sizeof *VALUE_H);
// 1/(1 - gamma*lambda): how far the credit for ONE action reaches. The old
// 0.995/0.95 pair gave 18 steps, i.e. 1.8 seconds of flight.
const int CREDIT_H[] = {20, 50, 100, 200, 500, 1000};
const int NCREDIT_H = int(sizeof CREDIT_H / sizeof *CREDIT_H);

const float TARGET_KL[] = {0.f, 0.01f, 0.02f, 0.03f, 0.05f, 0.1f};
const int NTARGET_KL = int(sizeof TARGET_KL / sizeof *TARGET_KL);

// THE SIX WORLDS, ONCE. This row was hand-written three times -- bench, watch
// and evaluate -- with three sets of typed-in offsets, three parallel six-bool
// blocks in Cfg, eighteen near-identical cases in apply() and three copies of
// the emit loop in buildArgs. The copies drifted, and the drift was not
// cosmetic: panelBench's run-count line still multiplied by (forest + maze)
// long after four more worlds existed, so with all six lit the panel said
// "32 runs" and launched 96.
//
// One row, one order, one count. WORLD_NAME is also the order buildArgs emits,
// so what the buttons say and what the command says cannot disagree again.
const char* WORLD_NAME[6] = {"forest", "maze", "city", "road", "culdesac", "corridor"};
const char* WORLD_LABEL[6] = {"Forest", "Maze", "City", "Road", "Cul-de-sac", "Corridors"};
const int NWORLDS = 6;
// Pane size, in pixels of ONE of the four. The window is 2x2 of these plus
// furniture, so 640 is a 1320x1500 window -- past what a laptop lid shows,
// which is why the default is 480 and not the biggest on the list.
// THE DEMO'S OWN WORLD LIST, and gallery/hall lead it. They are built on the
// 2 m lattice the coarsest map rung uses, so the first-person view subdivides
// as you approach a wall instead of the wall appearing to move -- which is
// what the six training worlds do through that ladder, correctly and
// unwatchably. The training worlds stay on the list because the demo should be
// able to show what the policy was actually measured in; they are just not the
// default. They are deliberately NOT added to WORLD_NAME: that list is what
// bench, watch, evaluate and report sample, and adding a world nothing has
// trained on would quietly change every one of those tables.
// "tour" first, as the default: gallery and hall in turn, a new layout each
// time, until the demo is closed (kestrel_demo.hpp).
const char* DEMO_WORLD[] = {"tour", "gallery", "hall", "forest", "maze", "city",
                            "road", "culdesac", "corridor"};
const int NDEMO_WORLD = int(sizeof DEMO_WORLD / sizeof *DEMO_WORLD);

// THE NINE, in the order bench runs them and named exactly as --policies takes
// them, so the row, the command and the binary cannot disagree about what a
// planner is called.
const char* POLICY_NAME[] = {"random", "freeM", "goal", "score",
                             "freeG", "novelG", "cover", "frontRaw", "circler"};
const int NPOLICY = int(sizeof POLICY_NAME / sizeof *POLICY_NAME);

// CORE FRACTION, AT THE REGIME BOUNDARIES rather than at round decimals.
// sphereClear walks INTEGER cell offsets, so the distances it can test are
// quantised: the core condition fires when d2 <= (robotR*coreFrac)^2 and d2 is
// k*cell^2, so the parameter only changes behaviour as it crosses
// sqrt(k)*cell/robotR. At 0.25 m cells and a 0.6 m body that is 0.417, 0.589,
// 0.722, 0.833, 0.932. Anything below 0.417 tests the centre cell alone and is
// a no-op in practice -- 0.30 is on the list to SHOW that, not because it is a
// setting anyone should pick.
const float CORE_FRAC[] = {0.f, 0.30f, 0.45f, 0.62f, 0.80f, 1.00f};
const int NCORE_FRAC = int(sizeof CORE_FRAC / sizeof *CORE_FRAC);

const int DEMO_PANE[] = {360, 480, 560, 640};
const int NDEMO_PANE = int(sizeof DEMO_PANE / sizeof *DEMO_PANE);

int nWorldsOn(const bool* w) {
    int n = 0;
    for (int i = 0; i < NWORLDS; ++i) n += w[i] ? 1 : 0;
    return n;
}

void worldRow(cv::Mat& im, std::vector<Btn>& bs, int x, int y, int idBase,
              const bool* w) {
    section(im, "worlds", x, y - 12);
    for (int i = 0; i < NWORLDS; ++i)
        bs.push_back({cv::Rect(x + i * 126, y, 118, 36), WORLD_LABEL[i],
                      idBase + i, w[i]});
}

void emitWorlds(std::vector<std::string>& a, const bool* w) {
    a.push_back("--worlds");
    for (int i = 0; i < NWORLDS; ++i) if (w[i]) a.push_back(WORLD_NAME[i]);
}

// THE STRIP IS THE CONTRACT, SO IT MUST NOT LIE.
//
// It was one line through fit(), which chops CHARACTERS. At default settings
// that truncated four of the six panels, and the train panel's strip ended
// "--max-steps 30..." -- which is not a shortened command but a DIFFERENT and
// still-parseable one: typed in, it runs 30-step episodes instead of 3000 and
// silently drops --explore and --target-kl, the two flags the panel's whole
// second stepper row exists to expose. A caption that says "you can type it
// instead" above a command that means something else is worse than no caption.
//
// So wrap at ARGUMENT boundaries, never inside a token, and when even two lines
// will not hold it drop whole arguments and say how many. A reader then sees
// either the real command or an honest "(+N more)", and never a wrong one.
std::vector<std::string> wrapCmd(const std::vector<std::string>& tok, int maxPx,
                                 double sc, int maxLines, int* dropped) {
    std::vector<std::string> lines;
    *dropped = 0;
    int base = 0;
    auto wide = [&](const std::string& t) {
        return cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width;
    };
    size_t i = 0;
    while (i < tok.size() && int(lines.size()) < maxLines) {
        std::string line = tok[i];
        // A single token longer than the whole strip still has to go somewhere;
        // it goes alone on its line rather than being silently halved.
        ++i;
        while (i < tok.size() && wide(line + " " + tok[i]) <= maxPx) {
            line += " " + tok[i];
            ++i;
        }
        lines.push_back(line);
    }
    if (i < tok.size()) {
        *dropped = int(tok.size() - i);
        const std::string tail = "(+" + std::to_string(*dropped) + " more)";
        if (!lines.empty()) {
            // Make room by giving back whole tokens, never characters.
            while (!lines.back().empty() &&
                   wide(lines.back() + " " + tail) > maxPx) {
                const size_t sp = lines.back().rfind(' ');
                if (sp == std::string::npos) break;
                lines.back().erase(sp);
                ++*dropped;
            }
            lines.back() += " " + tail;
        }
    }
    return lines;
}

std::string trimNum(float v) {
    char b[32]; std::snprintf(b, sizeof b, "%g", v);
    return std::string(b);
}

// CAMERA TILT for `sim`. Index 0 passes nothing: the sim's own default
// (-20 deg down, so the floor is inside the map's reach -- voxel_live.cpp says
// why). The rest pass --pitch explicitly, level included.
const int   NSIM_TILT = 5;
const char* SIM_TILT_ARG[NSIM_TILT]   = {nullptr, "0", "-10", "-20", "-30"};
const char* SIM_TILT_LABEL[NSIM_TILT] = {"camera tilt: default (-20)", "camera tilt: level",
                                         "camera tilt: -10 deg", "camera tilt: -20 deg",
                                         "camera tilt: -30 deg"};

// WHERE THE CAMERA IS: the three pose sources (navcore VisualPose).
const char* POSE_ARG[3]   = {"fixed", "vio", "slam"};
const char* POSE_LABEL[3] = {"Fixed", "VIO", "ORB-SLAM3"};
const char* POSE_HINT[3]  = {
    "the map is built where the camera stands; move it and the map smears",
    "DepthVio on the IR image + depth: this build, no other process",
    "ORB-SLAM3 on the stereo IR pair: start kestrel-orbslam first"};

struct Cfg {
    int mode = TRACK;

    // track
    int   input = -1;             // index into inputs, -1 = none chosen
    int   boxSize = 64;
    bool  designate = true;       // click the target on the first frame
    int   frameLimit = 0;         // 0 = all
    bool  csv = true;

    // bench
    // One array per panel, indexed like WORLD_NAME. See worldRow().
    bool  bw[NWORLDS] = {true, true, true, true, true, true};
    // 600 STEPS COULD NOT REACH THE GOAL IN FIVE OF THE SIX WORLDS, and this is
    // the panel whose whole job is measurement. journey_fit() puts the journeys
    // at 573 (maze), 1184 (corridor), 1722 (culdesac), 1944 (forest), 2000
    // (road) and 2042 (city) steps, so at 600 only the maze could finish and
    // every other row was scored on a task with an unreachable goal. Same class
    // of bug as the 1500-step training cap and the 368 m city goal, in the two
    // commands the policy is actually judged by.
    int   seed0 = 101, seed1 = 104, steps = 3000;
    bool  benchStereo = false;
    bool  benchOracle = false;    // --oracle: truth map within 8 m (a ceiling)
    // Which planners bench runs, and how much of the body's volume the veto
    // insists is CONFIRMED free. Both default to what bench has always done.
    bool  bp[9] = {true, true, true, true, true, true, true, true, true};
    int   coreIdx = 0;

    // sim
    int   simSource = 0;          // 0 raycaster, 1 live, 2 replay
    bool  simNear = false;        // --nearcell 0.10: the render-only near layer
    int   simTilt = 0;            // index into SIM_TILT; 0 = the sim's default
    // WHERE THE CAMERA IS (--pose): 0 fixed, 1 vio, 2 slam -- navcore's
    // VisualPose, the aircraft's own. Same meaning in the demo's live pane.
    int   simPose = 0;
    int   replay = -1;

    // train
    int   workers = 8, stepsIdx = 8, epLen = 3000, saveIdx = 2;
    // Defaults matching train.py: explore 0.02, anneal auto, target-kl 0.02,
    // clearance scaled with world size.
    int   exploreIdx = 3, annealIdx = 0, klIdx = 2;
    int   valueHIdx = 3, creditHIdx = 3;     // 1000 steps / 200 steps
    int   trainSeed = 0;                     // 0 = unseeded, as train.py
    bool  rawClear = false;
    // OFF by default, matching train.py: normalising the return was measured
    // and did not help -- see --norm-reward in train.py for the numbers.
    // THE DEFINING SETTING ON THIS PANEL, as a three-way cycle rather than a
    // toggle. 0 safe travel, 1 the same with the coverage reward tripled,
    // 2 the same charged for flying through unseen space, 3 reach a goal.
    // The two travel variants live here rather than in steppers of their own
    // because they are not independent knobs -- each trades one term of the
    // travel objective against another, and both are meaningless under the
    // goal one. There is also no free stepper slot: both rows are five wide
    // and the button grid is four by two.
    int   objMode = 0;
    bool  normReward = false;
    bool  clipVf = true;          // 0.2, and only applied with normReward
    bool  trainStereo = true;
    bool  cuda = false;
    bool  resume = false;
    bool  noVeto = false, varyGoal = false;

    // watch
    int   panes = 4, paneIdx = 1, layout = 0;   // layout 0 both, 1 fpv, 2 top
    bool  wDet = false;
    bool  ww[NWORLDS] = {true, true, true, true, true, true};

    // evaluate
    bool  ew[NWORLDS] = {true, true, true, true, true, true};
    bool  eRandom = false, eStereo = false;
    bool  eBaselines = true, eReward = false, eProgress = false, eNoVeto = false;
    bool  eVary = false;
    int   eSeed0 = 101, eSeed1 = 108, eSteps = 3000;   // see Cfg::steps

    // report
    bool  rw[NWORLDS] = {true, true, true, true, true, true};
    int   rSeed0 = 101, rSeed1 = 104, rSteps = 3000, rRepeats = 3;
    // demo: the showcase. Four panes, and the only choices that change what
    // it SHOWS rather than how fast it runs.
    int   dSource = 0;          // 0 sim, 1 live RealSense, 2 replay
    int   dWorld = 0;           // index into WORLD_NAME
    int   dPane = 1;            // index into DEMO_PANE
    // STORED AS THE FLAG, not as the feature. Green means the flag is on the
    // command line everywhere in this window, so a toggle whose ON state emits
    // NOTHING is the --raw-clear inversion again. Naming the field after the
    // argument makes the button, the flag and the green light one fact.
    // The people pane: 0 YOLOX-nano (built in), 1 HOG, 2 off.
    int   dPeople = 0;
    bool  dNoMirror = false, dNoEmitter = false;
    // 0 auto, 1 --cuda, 2 --no-cuda. Three states rather than a toggle because
    // "take the GPU if it is there" and "I am relying on the GPU" are different
    // intentions, and only the second should refuse to start without one.
    int   dCuda = 0;
    int   dPose = 0;            // 0 fixed, 1 vio, 2 slam (--pose)
    // -1 means "no model": fly the classical fallback, captioned as one.
    int   dModel = -1;
    bool  rDet = false, rProgress = false, rBaselines = false, rRandom = false;
    bool  rNoHome = false;
    bool  rStereo = false, rNoVeto = false, rVary = false;
};

const int PANE_PX[] = {240, 320, 420, 520};
const int NPANE_PX = int(sizeof PANE_PX / sizeof *PANE_PX);
// Matches watch.py --layout. "all" first because it is the default and
// the only one that shows depth, which is what an open world needs: the
// fpv there is mostly fog and correctly so.
const char* LAYOUT_NAME[5] = {"all", "both", "fpv", "top", "depth"};
const int NLAYOUT = 5;

std::string humanSteps(int n) {
    if (n == 0) return "forever";
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
            emitWorlds(a, c.bw);
            a.push_back("--seeds"); a.push_back(std::to_string(c.seed0));
            a.push_back(std::to_string(c.seed1));
            a.push_back("--steps"); a.push_back(std::to_string(c.steps));
            if (c.benchStereo) a.push_back("--stereo");
            if (c.benchOracle) a.push_back("--oracle");
            // Only when it is a SUBSET. Emitting all nine would be the same run
            // with a longer command line, and the strip is meant to be typed.
            {
                int on = 0;
                for (int i = 0; i < NPOLICY; ++i) on += c.bp[i] ? 1 : 0;
                if (on && on < NPOLICY) {
                    a.push_back("--policies");
                    for (int i = 0; i < NPOLICY; ++i)
                        if (c.bp[i]) a.push_back(POLICY_NAME[i]);
                }
            }
            if (CORE_FRAC[c.coreIdx] > 0.f) {
                a.push_back("--corefrac");
                a.push_back(trimNum(CORE_FRAC[c.coreIdx]));
            }
            break;
        case SIM:
            if (c.simSource == 0) a.push_back("--sim");
            else if (c.simSource == 1) a.push_back("--live");
            else if (c.replay >= 0 && c.replay < int(recs.size())) {
                a.push_back("--replay"); a.push_back(recs[c.replay]);
            }
            if (c.simNear) { a.push_back("--nearcell"); a.push_back("0.10"); }
            if (c.simTilt > 0 && c.simTilt < NSIM_TILT) {
                a.push_back("--pitch"); a.push_back(SIM_TILT_ARG[c.simTilt]);
            }
            if (c.simPose > 0) { a.push_back("--pose"); a.push_back(POSE_ARG[c.simPose]); }
            break;
        case DEMO:
            if (c.dSource == 1) a.push_back("--live");
            else if (c.dSource == 2) {
                a.push_back("--replay");
                a.push_back(c.replay >= 0 && c.replay < int(recs.size())
                                ? recs[c.replay] : std::string("(no recording)"));
            } else a.push_back("--sim");
            a.push_back("--world"); a.push_back(DEMO_WORLD[c.dWorld]);
            a.push_back("--pane");  a.push_back(std::to_string(DEMO_PANE[c.dPane]));
            if (c.dPeople == 2) a.push_back("--no-people");
            else if (c.dPeople == 1) { a.push_back("--detector"); a.push_back("hog"); }
            if (c.dNoMirror)  a.push_back("--no-mirror");
            if (c.dNoEmitter) a.push_back("--no-emitter");
            if (c.dPose > 0) { a.push_back("--pose"); a.push_back(POSE_ARG[c.dPose]); }
            if (c.dModel >= 0 && c.dModel < int(g_models.size())) {
                a.push_back("--model"); a.push_back(g_models[c.dModel]);
            }
            if (c.dCuda == 1) a.push_back("--cuda");
            else if (c.dCuda == 2) a.push_back("--no-cuda");
            break;
        case EVAL:
            // No --model: evaluate.py takes the newest checkpoint itself when
            // one is not named, which is what you want right after training.
            if (c.eRandom) a.push_back("--random");
            emitWorlds(a, c.ew);
            a.push_back("--seeds");
            for (int sd = c.eSeed0; sd <= c.eSeed1; ++sd) a.push_back(std::to_string(sd));
            a.push_back("--max-steps"); a.push_back(std::to_string(c.eSteps));
            if (c.eStereo) a.push_back("--stereo");
            if (c.eBaselines) a.push_back("--baselines");
            if (c.eReward) a.push_back("--reward");
            if (c.eProgress) a.push_back("--progress");
            if (c.eNoVeto) a.push_back("--no-veto");
            if (c.eVary) a.push_back("--vary-goal");
            break;
        case REPORT:
            emitWorlds(a, c.rw);
            a.push_back("--seeds");
            for (int sd = c.rSeed0; sd <= c.rSeed1; ++sd)
                a.push_back(std::to_string(sd));
            a.push_back("--max-steps"); a.push_back(std::to_string(c.rSteps));
            a.push_back("--repeats");   a.push_back(std::to_string(c.rRepeats));
            if (c.rDet) a.push_back("--deterministic");
            if (c.rProgress) a.push_back("--progress");
            if (c.rBaselines) a.push_back("--baselines");
            if (c.rNoHome)    a.push_back("--no-homeward");
            if (c.rRandom) a.push_back("--random");
            if (c.rStereo) a.push_back("--stereo");
            if (c.rNoVeto) a.push_back("--no-veto");
            if (c.rVary) a.push_back("--vary-goal");
            break;
        case WATCH:
            a.push_back("--panes");  a.push_back(std::to_string(c.panes));
            a.push_back("--px");     a.push_back(std::to_string(PANE_PX[c.paneIdx]));
            a.push_back("--layout"); a.push_back(LAYOUT_NAME[c.layout]);
            if (c.wDet) a.push_back("--deterministic");
            emitWorlds(a, c.ww);
            break;
        case TRAIN:
            a.push_back("--workers"); a.push_back(std::to_string(c.workers));
            if (TRAIN_STEPS[c.stepsIdx] == 0) {
                a.push_back("--forever");
            } else {
                a.push_back("--steps");
                a.push_back(std::to_string(TRAIN_STEPS[c.stepsIdx]));
            }
            a.push_back("--save-every");
            a.push_back(std::to_string(SAVE_EVERY[c.saveIdx]));
            if (c.trainStereo) a.push_back("--stereo");
            if (c.cuda) { a.push_back("--device"); a.push_back("cuda"); }
            if (c.resume) a.push_back("--resume");
            if (c.noVeto) a.push_back("--no-veto");
            a.push_back("--max-steps"); a.push_back(std::to_string(c.epLen));
            if (c.varyGoal) a.push_back("--vary-goal");
            a.push_back("--explore");
            a.push_back(trimNum(EXPLORE[c.exploreIdx]));
            // -1 is train.py's own default, so leaving it out keeps the printed
            // command as short as what a person would actually type.
            if (ANNEAL[c.annealIdx] >= 0) {
                a.push_back("--anneal");
                a.push_back(std::to_string(ANNEAL[c.annealIdx]));
            }
            a.push_back("--target-kl");
            a.push_back(trimNum(TARGET_KL[c.klIdx]));
            if (c.rawClear) a.push_back("--raw-clear");
            // range is train.py's default, so only the deviation prints.
            if (c.objMode == 3) {
                a.push_back("--objective"); a.push_back("goal");
            } else if (c.objMode == 1) {
                a.push_back("--coverage"); a.push_back("0.45");
            } else if (c.objMode == 2) {
                a.push_back("--seen"); a.push_back("0.3");
            }
            if (c.normReward) a.push_back("--norm-reward");
            // Inert without --norm-reward, and 0.2 is train.py's own
            // default, so it is only printed when it deviates AND applies --
            // the strip has to hold the whole command.
            if (c.normReward && !c.clipVf) {
                a.push_back("--clip-vf"); a.push_back("0");
            }
            if (c.trainSeed) {
                a.push_back("--seed");
                a.push_back(std::to_string(c.trainSeed));
            }
            // The panel names horizons; train.py takes the discounts. One
            // conversion, here, so the two can never mean different things.
            {
                const double g = 1.0 - 1.0 / double(VALUE_H[c.valueHIdx]);
                const double lam = (1.0 - 1.0 / double(CREDIT_H[c.creditHIdx])) / g;
                a.push_back("--gamma");      a.push_back(trimNum(float(g)));
                // Four decimals: the command strip is meant to be typed, and
                // 0.995996 is noise on a number whose whole meaning is
                // "about 200 steps".
                const double lr = std::round(std::min(0.9999, std::max(0.5, lam))
                                             * 10000.0) / 10000.0;
                a.push_back("--gae-lambda"); a.push_back(trimNum(float(lr)));
            }
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
    if (c.mode == BENCH && !nWorldsOn(c.bw)) return "pick at least one world";
    if (c.mode == WATCH && !nWorldsOn(c.ww)) return "pick at least one world";
    if (c.mode == EVAL  && !nWorldsOn(c.ew)) return "pick at least one world";
    if (c.mode == REPORT && !nWorldsOn(c.rw)) return "pick at least one world";
    if (c.mode == SIM && c.simSource == 2 && (c.replay < 0 || recs.empty()))
        return recs.empty() ? "no .kdr recordings found here" : "pick a recording";
    if (c.mode == DEMO && c.dSource == 2 && (c.replay < 0 || recs.empty()))
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
    ID_BENCH_S0M = 200, ID_BENCH_S0P,
    ID_BENCH_S1M, ID_BENCH_S1P, ID_BENCH_STM, ID_BENCH_STP, ID_BENCH_STEREO,
    // Six CONTIGUOUS ids per world row, so apply() is a range test rather than
    // six cases that can be typed wrong. The old bench ids were split either
    // side of the seed steppers -- FOREST/MAZE at 200-201 and CITY..CORR at
    // 209-212 -- which is exactly the shape that made a six-world row look
    // like a two-world row to the code that counted it.
    ID_BW = 250,          // +0..5, the order of WORLD_NAME
    ID_BENCH_POL = 260,   // +0..8, the order of POLICY_NAME
    ID_BENCH_CFM = 270, ID_BENCH_CFP, ID_BENCH_ORACLE,
    ID_WW = 560,          // +0..5
    ID_EW = 660,          // +0..5
    ID_RW = 760,          // +0..5
    ID_SIM_SRC = 300,     // +0..2
    ID_SIM_REPLAY = 310,  // +index
    ID_SIM_NEAR = 335,
    ID_SIM_TILT = 336,
    ID_SIM_POSE = 340,    // +0..2
    ID_TRAIN_WM = 400, ID_TRAIN_WP, ID_TRAIN_SM, ID_TRAIN_SP,
    ID_TRAIN_STEREO, ID_TRAIN_CUDA, ID_TRAIN_INSTALL, ID_TRAIN_PYTHONS,
    ID_TRAIN_RESUME, ID_TRAIN_NOVETO, ID_TRAIN_EPM, ID_TRAIN_EPP, ID_TRAIN_VARY, ID_TRAIN_SVM, ID_TRAIN_SVP,
    ID_TRAIN_EXM, ID_TRAIN_EXP, ID_TRAIN_ANM, ID_TRAIN_ANP, ID_TRAIN_KLM,
    ID_TRAIN_KLP, ID_TRAIN_RAWCLR, ID_TRAIN_VHM, ID_TRAIN_VHP,
    ID_TRAIN_CHM, ID_TRAIN_CHP, ID_TRAIN_SEEDM, ID_TRAIN_SEEDP,
    ID_TRAIN_NORMR, ID_TRAIN_CLIPVF, ID_TRAIN_OBJ,
    ID_W_PANES_M = 500, ID_W_PANES_P, ID_W_PX_M, ID_W_PX_P,
    ID_W_LAYOUT, ID_W_DET,
    ID_E_S0M = 600, ID_E_S0P, ID_E_S1M, ID_E_S1P,
    ID_E_STM, ID_E_STP, ID_E_RANDOM, ID_E_STEREO, ID_E_BASE, ID_E_REWARD,
    ID_E_PROGRESS, ID_E_NOVETO, ID_E_VARY,
    ID_R_S0M = 700, ID_R_S0P, ID_R_S1M, ID_R_S1P, ID_R_STM, ID_R_STP,
    ID_R_RPM, ID_R_RPP, ID_R_DET, ID_R_PROGRESS, ID_R_BASE, ID_R_RANDOM,
    ID_R_STEREO, ID_R_NOVETO, ID_R_VARY, ID_R_NOHOME,
    // ID_D_SOURCE OWNS THREE IDS, one per source button, because the buttons
    // are emitted as ID_D_SOURCE + i. The next name therefore starts at 810
    // rather than following on: written as a bare successor it WAS
    // ID_D_SOURCE + 1, so clicking "Live D435i" also cycled the world. This is
    // the duplicate-id case `gui --check` exists to catch, and it caught it.
    ID_D_SOURCE = 800,
    ID_D_WORLD = 810, ID_D_PANEM, ID_D_PANEP,
    ID_D_PEOPLE, ID_D_MIRROR, ID_D_EMITTER, ID_D_SHOT, ID_D_CUDA,
    ID_D_EXPORT,
    // A RANGE, like ID_SIM_REPLAY: one id per .onnx found, plus one for "none".
    ID_D_MODEL = 830,
    ID_D_POSE = 840,      // +0..2
};

void panelTrack(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c,
                const std::vector<TrackInput>& inputs) {
    const int x = 266;
    txt(im, "object lock over recorded frames", x, 112, 0.66, INK, 2);
    txt(im, "The tracker that runs on the aircraft, over frames you already have.",
        x, 136, 0.44, DIM);

    section(im, "input", x, 176);
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
    txt(im, "path-planner baselines", x, 112, 0.66, INK, 2);
    txt(im, "NINE classical planners through the SAME environment a learned policy",
        x, 136, 0.44, DIM);
    txt(im, "uses. Four optimise a goal nothing is scored on any more; five are",
        x, 156, 0.44, DIM);
    txt(im, "matched to safe travel. RUN_ME says what each one does.",
        x, 176, 0.44, DIM);

    worldRow(im, bs, x, 206, ID_BW, c.bw);        // label 194, buttons 206-242

    section(im, "planners", x, 268);
    for (int i = 0; i < NPOLICY; ++i)             // two rows, 280-314 and 320-354
        bs.push_back({cv::Rect(x + (i % 5) * 158, 280 + (i / 5) * 40, 150, 34),
                      POLICY_NAME[i], ID_BENCH_POL + i, c.bp[i]});

    stepper(im, bs, x, 386, "first seed", std::to_string(c.seed0),
            ID_BENCH_S0M, ID_BENCH_S0P, nullptr, 100);
    stepper(im, bs, x + 190, 386, "last seed", std::to_string(c.seed1),
            ID_BENCH_S1M, ID_BENCH_S1P, nullptr, 100);
    stepper(im, bs, x + 380, 386, "steps/run", std::to_string(c.steps),
            ID_BENCH_STM, ID_BENCH_STP, nullptr, 100);
    // THE VETO'S ATTITUDE TO UNKNOWN SPACE. 0 lets a primitive sweep through
    // air nothing has measured, which is why every collision ever measured in
    // this tree has been into unmapped space. Quantised by the voxel size --
    // see CORE_FRAC for why the list is not round numbers.
    stepper(im, bs, x + 570, 386, "corefrac", trimNum(CORE_FRAC[c.coreIdx]),
            ID_BENCH_CFM, ID_BENCH_CFP, "unknown is not free", 100);

    // FROM THE SAME ARRAYS the row and the command are built from. This line
    // once multiplied by (forest + maze) and reported "32 runs" for a job that
    // launched 96, and later still said "4 policies" when bench ran nine.
    const int nseeds = std::max(0, c.seed1 - c.seed0 + 1);
    const int nw = nWorldsOn(c.bw);
    int npol = 0;
    for (int i = 0; i < NPOLICY; ++i) npol += c.bp[i] ? 1 : 0;
    txt(im, std::to_string(nw * nseeds * npol) + " runs (" +
            std::to_string(npol) + " planner(s) x " + std::to_string(nw) +
            " world(s) x " + std::to_string(nseeds) + " seed(s))",
        x, 470, 0.46, DIM);

    bs.push_back({cv::Rect(x, 486, 250, 36),
                  c.benchStereo ? "Simulated stereo" : "Perfect depth (control)",
                  ID_BENCH_STEREO, c.benchStereo});
    // THE CEILING, not a planner: the fine map is the TRUTH within 8 m, so
    // the row says what perfect map knowledge would be worth.
    bs.push_back({cv::Rect(x + 270, 486, 250, 36),
                  c.benchOracle ? "Oracle map: ON (ceiling)" : "Oracle map: off",
                  ID_BENCH_ORACLE, c.benchOracle});
    txt(im, "Run both: a failure on perfect depth is the planner, a failure only on "
            "stereo is the sensor.", x, 546, 0.42, DIM);
}
void panelDemo(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c,
               const std::vector<std::string>& recs) {
    const int x = 266;
    txt(im, "the demo -- the aircraft, flying", x, 112, 0.66, INK, 2);
    txt(im, "Simulated: the aircraft's OWN autonomy (onboard's voxel module and mission,",
        x, 144, 0.44, DIM);
    txt(im, "compiled in) through the showcase worlds -- what it sees, the stereo depth,",
        x, 162, 0.44, DIM);
    txt(im, "the map it builds, the flight. A camera or a policy: the four-pane window.",
        x, 180, 0.44, DIM);

    section(im, "depth for the two live panes", x, 208);
    const char* src[3] = {"Simulated raycaster", "Live D435i", "Replay a recording"};
    for (int i = 0; i < 3; ++i)
        bs.push_back({cv::Rect(x + i * 260, 220, 250, 38), src[i],
                      ID_D_SOURCE + i, c.dSource == i});

    // WHICH WORLD THE POLICY FLIES IN, and it is a single choice rather than
    // the multi-select the measuring panels use: a demo shows one thing at a
    // time and a checklist would imply otherwise.
    section(im, "world for the SIM pane", x, 292);
    bs.push_back({cv::Rect(x, 304, 250, 38),
                  std::string("world: ") + DEMO_WORLD[c.dWorld], ID_D_WORLD, true});
    txt(im, c.dWorld == 0 ? "gallery + hall in turn, forever" : "click to cycle",
        x, 358, 0.42, DIM);

    // WHERE THE CAMERA IS for the live voxel pane (--pose): fixed, DepthVio,
    // or ORB-SLAM3 through kestrel-orbslam. Only a real source is tracked.
    section(im, "live pane position", x + 520, 292);
    const char* poseShort[3] = {"Fixed", "VIO", "SLAM"};
    for (int i = 0; i < 3; ++i)
        bs.push_back({cv::Rect(x + 520 + i * 84, 304, 80, 38), poseShort[i],
                      ID_D_POSE + i, c.dPose == i});
    stepper(im, bs, x + 300, 304, "pane px",
            std::to_string(DEMO_PANE[c.dPane]), ID_D_PANEM, ID_D_PANEP,
            "one of the four", 100);

    // PEOPLE: the built-in YOLOX-nano (Apache-2.0, compiled into this exe),
    // OpenCV's HOG, or the pane off.
    bs.push_back({cv::Rect(x, 382, 250, 36),
                  c.dPeople == 0 ? "people: YOLOX (built in)"
                : c.dPeople == 1 ? "people: HOG" : "people pane OFF",
                  ID_D_PEOPLE, c.dPeople != 0});
    bs.push_back({cv::Rect(x + 260, 382, 250, 36),
                  c.dNoMirror ? "camera as-is" : "mirror the camera",
                  ID_D_MIRROR, c.dNoMirror});
    // The projector is what makes a D435i work on a blank wall, and it is also
    // what puts a dot pattern in the infrared image the detector reads. It
    // matters here and nowhere else in this window.
    bs.push_back({cv::Rect(x + 520, 382, 250, 36),
                  c.dNoEmitter ? "IR emitter off" : "IR emitter on",
                  ID_D_EMITTER, c.dNoEmitter});

    // THE GPU, FOR THE TWO NETWORKS AND NOTHING ELSE. It does not touch the
    // sim's depth renderer: that has its own build switch guarding a kernel
    // its own header says has never been compiled or run.
    bs.push_back({cv::Rect(x, 428, 250, 36),
                  c.dCuda == 1 ? "cuda: required"
                : c.dCuda == 2 ? "cuda: off"
                               : "cuda: if present",
                  ID_D_CUDA, c.dCuda != 0});
    bs.push_back({cv::Rect(x + 260, 428, 250, 36), "Write the panes as PNG",
                  ID_D_SHOT, false});
    // EXPORT IS ONE BUTTON because the script defaults to the newest
    // checkpoint in the newest run, the same convention report and evaluate
    // use. It runs through train, which is where the interpreter is chosen.
    bs.push_back({cv::Rect(x + 520, 428, 250, 36), "Export newest -> .onnx",
                  ID_D_EXPORT, false});

    // WHICH POLICY, and "none" is a real choice rather than the absence of one:
    // it is the aircraft's own autonomy flying. A learned policy switches to
    // the research window, captioned as the policy it is.
    section(im, "who flies the SIM", x, 486);
    {
        const int n = std::min<int>(3, int(g_models.size()));
        bs.push_back({cv::Rect(x, 494, 180, 32), "the aircraft",
                      ID_D_MODEL, c.dModel < 0});
        for (int i = 0; i < n; ++i)
            bs.push_back({cv::Rect(x + 190 + i * 200, 494, 190, 32),
                          fs::path(g_models[i]).filename().string(),
                          ID_D_MODEL + 1 + i, c.dModel == i});
    }

    // BELOW the buttons, not beside them: at x+520 four lines of this length
    // ran 200 px off a 1060 px canvas, and gui --check caught it.
    txt(im, "cuda moves the POLICY and YOLOX onto the GPU; HOG has no GPU path.", x, 541, 0.42, DIM);

    if (c.dSource == 1) {
        txt(im, "librealsense loads at RUN time; with no camera the two live panes "
                "fall back to the sim.", x, 557, 0.42, DIM);
    } else if (c.dSource == 2 && recs.empty()) {
        txt(im, "no .kdr recordings found in ./ or ./recordings", x, 557, 0.42, DIM);
    } else {
        txt(im, "q quit, r next map. Every pane names what it is ACTUALLY showing.",
            x, 557, 0.42, DIM);
    }
}

void panelSim(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c,
              const std::vector<std::string>& recs) {
    const int x = 266;
    txt(im, "live voxel sim", x, 112, 0.66, INK, 2);
    txt(im, "The real map, planner and veto over depth, in THIS process -- the same",
        x, 142, 0.44, DIM);
    txt(im, "pipeline the aircraft flies (navcore), not a copy.", x, 160, 0.44, DIM);

    // THREE CHOICES, one row each, as segmented controls: where the depth
    // comes from, where the camera is, and how it is drawn.
    section(im, "depth source", x, 194);
    const char* src[3] = {"Simulated", "Live D435i", "Replay"};
    for (int i = 0; i < 3; ++i)
        bs.push_back({cv::Rect(x + i * 172, 204, 168, 36), src[i], ID_SIM_SRC + i,
                      c.simSource == i});

    section(im, "camera position", x, 268);
    for (int i = 0; i < 3; ++i)
        bs.push_back({cv::Rect(x + i * 172, 278, 168, 36), POSE_LABEL[i], ID_SIM_POSE + i,
                      c.simPose == i});
    txt(im, POSE_HINT[c.simPose], x, 334, 0.42, DIM);

    section(im, "display", x, 368);
    // OFF BY DEFAULT. The 0.10 m near layer is drawn but never planned on, and
    // its seam with the 0.25 m map put a round blind spot in the middle of the
    // first-person pane whenever a surface sat just past 2.2 m.
    bs.push_back({cv::Rect(x, 378, 252, 36),
                  c.simNear ? "0.10 m near layer: ON" : "0.10 m near layer: off",
                  ID_SIM_NEAR, c.simNear});
    // The DEFAULT differs by source: the sim looks 20 deg down; a real camera's
    // tilt is measured by its IMU, and nothing is assumed.
    bs.push_back({cv::Rect(x + 260, 378, 252, 36),
                  c.simTilt == 0 && c.simSource != 0 ? "camera tilt: from its IMU"
                                                     : SIM_TILT_LABEL[c.simTilt],
                  ID_SIM_TILT, c.simTilt != 0});
    txt(im, "the near layer is render only -- the planner never reads it", x, 432,
        0.4, DIM);

    if (c.simSource == 1) {
        txt(im, "librealsense loads at RUN time, so this build needs no SDK;",
            x, 470, 0.42, DIM);
        txt(im, "if it is missing, the sim says where it looked.", x, 490, 0.42, DIM);
    } else if (c.simSource == 2) {
        if (recs.empty()) {
            txt(im, "no .kdr files in ./ or ./recordings", x, 470, 0.44, DIM);
        } else {
            // Three at most, so the list cannot run into the key line below.
            section(im, "recording", x, 452);
            for (size_t i = 0; i < recs.size() && i < 3; ++i)
                bs.push_back({cv::Rect(x + int(i) * 172, 462, 168, 34),
                              fs::path(recs[i]).filename().string(),
                              ID_SIM_REPLAY + int(i), c.replay == int(i)});
        }
    } else {
        txt(im, "No camera needed. The raycaster is the control case: if the planner",
            x, 470, 0.42, DIM);
        txt(im, "fails here, the sensor is not what is wrong.", x, 490, 0.42, DIM);
    }
    txt(im, "In the sim window:  space pause   v view   s save PNG   m menu   q back",
        x, 540, 0.42, DIM);
}

void panelTrain(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "RL path-policy training", x, 106, 0.66, INK, 2);
    // BESIDE THE TITLE, not buried in the switch grid. This decides what the
    // policy is being paid for, and every other control on the panel is a
    // detail by comparison. The blurb below runs to about x+400, so this sits
    // clear of it.
    bs.push_back({cv::Rect(x + 540, 96, 240, 40),
                  c.objMode == 3 ? "objective: reach a goal"
                : c.objMode == 2 ? "safe travel: look first"
                : c.objMode == 1 ? "safe travel: explore+"
                                 : "objective: safe travel",
                  ID_TRAIN_OBJ, c.objMode != 0});
    txt(im, "PyTorch and stable-baselines3 driving the C++ environment. This is",
        x, 128, 0.44, DIM);
    txt(im, "the one command that runs python -- see the note at the bottom.",
        x, 146, 0.44, DIM);

    // ROW ONE: how big the run is.
    // BOTH ROWS ARE FIVE WIDE at the same pitch, so the panel reads as a grid
    // rather than as two different layouts stacked.
    const int P = 150, SW = 100;
    stepper(im, bs, x, 182, "workers", std::to_string(c.workers),
            ID_TRAIN_WM, ID_TRAIN_WP, "parallel envs", SW);
    stepper(im, bs, x + P, 182, "steps", humanSteps(TRAIN_STEPS[c.stepsIdx]),
            ID_TRAIN_SM, ID_TRAIN_SP, "saved as it goes", SW);
    // THE GOAL HAS TO FIT INSIDE AN EPISODE. At 1500 it did not: the forest
    // goal needs ~2500 steps, so every episode was cut off before arrival was
    // possible and the goal bonus was unreachable.
    stepper(im, bs, x + 2 * P, 182, "steps/episode", std::to_string(c.epLen),
            ID_TRAIN_EPM, ID_TRAIN_EPP,
            c.objMode == 3 ? "the goal must fit" : "caps how far it gets", SW);
    stepper(im, bs, x + 3 * P, 182, "save every",
            humanSteps(SAVE_EVERY[c.saveIdx]),
            ID_TRAIN_SVM, ID_TRAIN_SVP, "checkpoint interval", SW);
    // TWO RUNS THAT DIFFER ONLY IN ONE SETTING. Without a seed they also differ
    // by whatever the initialisation happened to be, which over a short run is
    // most of the difference -- so an A/B of a training change would be two
    // samples from a noisy distribution rather than a comparison.
    stepper(im, bs, x + 4 * P, 182, "seed",
            c.trainSeed ? std::to_string(c.trainSeed) : std::string("off"),
            ID_TRAIN_SEEDM, ID_TRAIN_SEEDP, "0 = unseeded", SW);

          // five across the panel
    stepper(im, bs, x, 268, "explore", trimNum(EXPLORE[c.exploreIdx]),
            ID_TRAIN_EXM, ID_TRAIN_EXP, "how random it stays", SW);
    stepper(im, bs, x + P, 268, "anneal",
            ANNEAL[c.annealIdx] < 0 ? std::string("auto")
          : ANNEAL[c.annealIdx] == 0 ? std::string("off")
          : humanSteps(ANNEAL[c.annealIdx]),
            ID_TRAIN_ANM, ID_TRAIN_ANP, "lr + explore decay", SW);
    stepper(im, bs, x + 2 * P, 268, "target-kl", trimNum(TARGET_KL[c.klIdx]),
            ID_TRAIN_KLM, ID_TRAIN_KLP, "cap on one update", SW);
    // IN STEPS, so it can be compared with steps/episode directly above it.
    // As a discount this was 0.995 and nobody noticed it meant 200.
    stepper(im, bs, x + 3 * P, 268, "value horizon",
            std::to_string(VALUE_H[c.valueHIdx]),
            ID_TRAIN_VHM, ID_TRAIN_VHP, "steps it sees ahead", SW);
    stepper(im, bs, x + 4 * P, 268, "credit horizon",
            std::to_string(CREDIT_H[c.creditHIdx]),
            ID_TRAIN_CHM, ID_TRAIN_CHP, "steps one act owns", SW);

    // FOUR ACROSS, not three. Two more settings had to land here and the panel
    // has no vertical room left -- the python block already ends 10 px above
    // the command strip. 184 wide at a 202 pitch fills the 794 px exactly.
    const int BW = 184, BP = 202;
    bs.push_back({cv::Rect(x, 356, BW, 38),
                  c.trainStereo ? "Simulated stereo" : "Perfect depth",
                  ID_TRAIN_STEREO, c.trainStereo});
    // WITHOUT THIS A RUN ALWAYS STARTS FROM ZERO. The trainer checkpoints as it
    // goes but had no way to read one back, so an interrupted overnight run
    // could only be started again from scratch with its weights on disk.
    bs.push_back({cv::Rect(x + BP, 356, BW, 38),
                  c.resume ? "resume newest" : "from scratch",
                  ID_TRAIN_RESUME, c.resume});
    bs.push_back({cv::Rect(x + 2 * BP, 356, BW, 38),
                  c.cuda ? "device: cuda" : "device: cpu",
                  ID_TRAIN_CUDA, c.cuda});
    // OFF, AND MEASURED. Normalising the return was added to rescue a critic
    // that looked broken at a 1000-step horizon, and did not rescue it:
    // explained_variance +0.286 -> +0.250 while the worst update went -8.75 ->
    // -75.60. explained_variance is 1 - Var(y-yhat)/Var(y) and therefore
    // scale-invariant by construction, so rescaling the targets could never
    // have moved it. Kept as a switch, not a default.
    //
    // GREEN MEANS THE FLAG IS ON THE COMMAND LINE, here as everywhere -- even
    // though "green = the good setting is on" reads more naturally on a button
    // like this. That reading is exactly the inversion --raw-clear shipped
    // with, and one rule that always holds beats two that each feel right on
    // their own button.
    bs.push_back({cv::Rect(x + 3 * BP, 356, BW, 38),
                  c.normReward ? "normalised returns" : "raw returns",
                  ID_TRAIN_NORMR, c.normReward});

    // THE SAFETY MASK, AS A SWITCH. On, the policy chooses among primitives the
    // geometry already approved and cannot collide by choosing. Off, it must
    // learn avoidance from the collision terminal: a measurement of what the
    // veto is worth, not a way to fly.
    bs.push_back({cv::Rect(x, 400, BW, 36),
                  c.noVeto ? "learn by crashing" : "geometric veto on",
                  ID_TRAIN_NOVETO, c.noVeto});
    bs.push_back({cv::Rect(x + BP, 400, BW, 36),
                  c.varyGoal ? "varied journey" : "one fixed journey",
                  ID_TRAIN_VARY, c.varyGoal});
    // The near-miss penalty is the only avoidance signal that arrives BEFORE
    // contact. Left in absolute units it was ~5x weaker against progress in a
    // tight world than an open one -- the wrong way round.
    bs.push_back({cv::Rect(x + 2 * BP, 400, BW, 36),
                  c.rawClear ? "raw clearance" : "scaled clearance",
                  ID_TRAIN_RAWCLR, c.rawClear});
    // PPO clips the policy update and, by default in SB3, not the value one --
    // which is the update that matters when the targets are large.
    // GREY WHEN IT DOES NOTHING. clip_range_vf is only applied with
    // --norm-reward, so lighting it up on its own claims an effect the run
    // will not have -- the same species of lie as a toggle whose green means
    // the opposite of its neighbour's.
    bs.push_back({cv::Rect(x + 3 * BP, 400, BW, 36),
                  !c.normReward ? "value clip: n/a"
                                : (c.clipVf ? "clip value updates"
                                            : "value clip off"),
                  ID_TRAIN_CLIPVF, c.clipVf && c.normReward});

    txt(im, "explore is the entropy bonus -- how much random stuff it tries. "
            "High early, a tenth of it after the anneal.", x, 452, 0.42, DIM);
    txt(im, c.objMode == 3
            ? "A goal beyond the value horizon is invisible to the value "
              "function; only the progress shaping reaches it."
            : "safe travel pays for DISPLACEMENT and NEW GROUND, not metres "
              "flown: a hoverer scores -34, freeM +132.",
        x, 470, 0.42, DIM);
    txt(im, "stereo is honest and ~3x slower. cuda will look idle: the "
            "bottleneck is environment steps, in C++.", x, 488, 0.42, DIM);

    // WHICH INTERPRETER, by absolute path. "python" is ambiguous on a machine
    // with several, and installing into the wrong one SUCCEEDS -- leaving the
    // packages present and the import still failing, which is the most
    // confusing state available. Naming the path removes the question.
    const kpy::Py* b = g_py.probed ? kpy::best(g_py.pys) : nullptr;
    // The buttons to the right start at x+266, so this column is 250 px wide.
    // Fitting to it rather than trusting the text to be short is the fix for a
    // status line that ran straight under "Install the RL stack".
    const int col = 250;
    section(im, "python", x, 506);
    if (!g_py.probed) {
        txt(im, fit("not checked yet", col, 0.44, false), x, 524, 0.44, DIM);
    } else if (b && b->rl) {
        txt(im, fit("ready: " + b->exe, col, 0.44), x, 524, 0.44, INK);
        txt(im, fit("CPython " + std::to_string(b->major) + "." +
                    std::to_string(b->minor) + " loads voxelenv", col, 0.42, false),
        x, 540, 0.42, DIM);
    } else if (b) {
        txt(im, fit("needs the RL stack: " + b->exe, col, 0.44), x, 524, 0.44, INK);
        txt(im, fit("Install targets THAT interpreter,", col, 0.42, false), x, 540, 0.42, DIM);
        txt(im, fit("not whatever 'python' means.", col, 0.42, false), x, 556, 0.42, DIM);
    } else if (!g_py.abi.empty()) {
        txt(im, fit("none can load voxelenv (needs " + g_py.abi + ")", col, 0.44, false),
            x, 524, 0.44, INK);
        txt(im, fit("a version mismatch, not a missing file", col, 0.42, false),
            x, 540, 0.42, DIM);
    } else {
        txt(im, fit("voxelenv is not beside this exe", col, 0.44, false), x, 524, 0.44, INK);
        txt(im, fit("the C++ env the trainer steps", col, 0.42, false), x, 544, 0.42, DIM);
    }
    bs.push_back({cv::Rect(x + 266, 508, 230, 34), "Install the RL stack",
                  ID_TRAIN_INSTALL, b && !b->rl});
    // The full listing, for when the one line above is not enough -- which is
    // whenever the machine has several pythons and the wrong one is winning.
    // Beside Install rather than under it: the panel has no vertical room left
    // and a row at 546 would reach 576, through the command strip at 564.
    bs.push_back({cv::Rect(x + 512, 508, 230, 34), "List every python",
                  ID_TRAIN_PYTHONS, false});
}

void panelWatch(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "watch the policy fly while it trains", x, 112, 0.66, INK, 2);
    txt(im, "A grid of live episodes in the FIRST-PERSON VOXEL VIEW -- what the",
        x, 136, 0.44, DIM);
    txt(im, "aircraft believes it can see. Run this beside a training run.",
        x, 156, 0.44, DIM);

worldRow(im, bs, x, 208, ID_WW, c.ww);

    stepper(im, bs, x, 300, "panes", std::to_string(c.panes),
            ID_W_PANES_M, ID_W_PANES_P, "one episode each");
    stepper(im, bs, x + 220, 300, "pane px", std::to_string(PANE_PX[c.paneIdx]),
            ID_W_PX_M, ID_W_PX_P, "bigger costs more CPU");

    bs.push_back({cv::Rect(x + 440, 300, 240, 36),
                  c.layout == 0 ? "fpv + plan + DEPTH"
                : c.layout == 1 ? "fpv + plan inset"
                : c.layout == 2 ? "fpv only"
                : c.layout == 3 ? "plan view only" : "depth only",
                  ID_W_LAYOUT, true});
    txt(im, "click to cycle", x + 440, 354, 0.42, DIM);

    // Early in training an argmaxed policy picks one primitive whatever it
    // sees, so every pane flies the same arc and the view looks frozen.
    // SHORT ENOUGH TO SURVIVE ITS BUTTON. Both labels used to carry their
    // reason in a parenthetical, ran past 250 px, and were elided from the
    // front -- so the button read "... (see what training does)" and the word
    // that named the state was the one thrown away. The reason moved to the
    // hint line, which has the whole panel width to spend.
    bs.push_back({cv::Rect(x, 360, 250, 34),
                  c.wDet ? "argmax (deterministic)" : "sampled (stochastic)",
                  ID_W_DET, c.wDet});
    txt(im, c.wDet ? "judging a finished policy"
                   : "what training actually does", x + 266, 382, 0.42, DIM);

    txt(im, "PALE IS UNKNOWN, drawn as fog and never as air. In an open world a",
        x, 404, 0.42, DIM);
    txt(im, "pane is mostly WHITE and that is correct: at 0.25 m voxels the map",
        x, 422, 0.42, DIM);
    txt(im, "only marks obstacles to about 3.5 m. The DEPTH strip is what shows",
        x, 440, 0.42, DIM);
    txt(im, "the sensor is returning anything at all.", x, 458, 0.42, DIM);

    txt(im, "It reloads the newest checkpoint as training writes them, on seeds", x, 492, 0.42, DIM);
    txt(im, "training never uses. Before the first one it flies random-legal --", x, 510, 0.42, DIM);
    txt(im, "the same baseline `bench` reports, so pane one is a fair 'before'.", x, 528, 0.42, DIM);
    txt(im, "q in the watch window closes it and comes back here.", x, 552, 0.42, DIM);
}

void panelEval(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "score a trained policy", x, 112, 0.66, INK, 2);
    txt(im, "The SAME columns `bench` reports the classical planners in, on",
        x, 136, 0.44, DIM);
    txt(im, "seeds held out from training. A comparison on new metrics is worth",
        x, 156, 0.44, DIM);
    txt(im, "nothing, so the scorecard is deliberately identical.", x, 176, 0.44, DIM);

worldRow(im, bs, x, 228, ID_EW, c.ew);

    stepper(im, bs, x, 320, "first seed", std::to_string(c.eSeed0),
            ID_E_S0M, ID_E_S0P);
    stepper(im, bs, x + 220, 320, "last seed", std::to_string(c.eSeed1),
            ID_E_S1M, ID_E_S1P);
    stepper(im, bs, x + 440, 320, "steps/run", std::to_string(c.eSteps),
            ID_E_STM, ID_E_STP);

    bs.push_back({cv::Rect(x, 420, 250, 36),
                  c.eRandom ? "random (the floor)" : "the trained policy",
                  ID_E_RANDOM, c.eRandom});
    bs.push_back({cv::Rect(x + 266, 420, 250, 36),
                  c.eStereo ? "Simulated stereo" : "Perfect depth",
                  ID_E_STEREO, c.eStereo});

    bs.push_back({cv::Rect(x, 476, 250, 34),
                  c.eBaselines ? "with the 9 baselines" : "the policy alone",
                  ID_E_BASE, c.eBaselines});
    bs.push_back({cv::Rect(x + 266, 476, 250, 34),
                  c.eReward ? "show reward per term" : "scorecard only",
                  ID_E_REWARD, c.eReward});

    // Which world the policy is being scored in. A --no-veto policy scored
    // with the veto on is measured in an easier world than it trained in.
    bs.push_back({cv::Rect(x + 266, 516, 250, 32),
                  c.eNoVeto ? "veto OFF while scoring" : "veto on while scoring",
                  ID_E_NOVETO, c.eNoVeto});
    bs.push_back({cv::Rect(x, 516, 250, 32),
                  c.eProgress ? "EVERY checkpoint" : "newest checkpoint",
                  ID_E_PROGRESS, c.eProgress});
    // Third column, not a fourth row: a row at 556 reaches y=588 and the
    // command strip's text starts at 582.
    bs.push_back({cv::Rect(x + 532, 516, 210, 32),
                  c.eVary ? "varied journey" : "fixed journey",
                  ID_E_VARY, c.eVary});

}

// GREEN MEANS THIS FLAG IS ON THE COMMAND LINE -- and now the check says so.
//
// Btn carries one `on` bool and the panels used it for several unrelated ideas:
// a flag being present, an item being selected out of N, the current value of a
// cycler, and "click me". The polarity of one of them was simply inverted --
// ID_TRAIN_RAWCLR lit green when --raw-clear was ABSENT -- and nothing could
// catch it, because "green" had no definition to check against.
//
// For the plain flag toggles it does now. Each row below names a button and the
// argument it is responsible for; check() builds the command for every mode and
// variant and asserts the button is green exactly when its flag is emitted. The
// cyclers, the radio lists and the Install button are deliberately absent: they
// are not flags, and claiming they were would make this table a lie rather than
// a test.
struct FlagBtn { int mode; int id; const char* flag; };
const FlagBtn FLAG_BTNS[] = {
    {BENCH, ID_BENCH_STEREO,  "--stereo"},
    {BENCH, ID_BENCH_ORACLE,  "--oracle"},
    {TRAIN, ID_TRAIN_STEREO,  "--stereo"},
    {TRAIN, ID_TRAIN_RESUME,  "--resume"},
    {TRAIN, ID_TRAIN_NOVETO,  "--no-veto"},
    {TRAIN, ID_TRAIN_VARY,    "--vary-goal"},
    {TRAIN, ID_TRAIN_RAWCLR,  "--raw-clear"},
    {TRAIN, ID_TRAIN_NORMR,   "--norm-reward"},
    {WATCH, ID_W_DET,         "--deterministic"},
    {EVAL,  ID_E_RANDOM,      "--random"},
    {EVAL,  ID_E_STEREO,      "--stereo"},
    {EVAL,  ID_E_BASE,        "--baselines"},
    {EVAL,  ID_E_REWARD,      "--reward"},
    {EVAL,  ID_E_PROGRESS,    "--progress"},
    {EVAL,  ID_E_NOVETO,      "--no-veto"},
    {EVAL,  ID_E_VARY,        "--vary-goal"},
    {REPORT, ID_R_DET,      "--deterministic"},
    {REPORT, ID_R_PROGRESS, "--progress"},
    {REPORT, ID_R_BASE,     "--baselines"},
    {REPORT, ID_R_RANDOM,   "--random"},
    {REPORT, ID_R_STEREO,   "--stereo"},
    {DEMO,  ID_D_PEOPLE,    "--no-people"},
    {DEMO,  ID_D_MIRROR,    "--no-mirror"},
    {DEMO,  ID_D_EMITTER,   "--no-emitter"},
    {BENCH, ID_BENCH_STEREO, "--stereo"},
    // Three-state, so the table cannot name one flag: lit for --cuda AND for
    // --no-cuda, dark only for auto, which emits nothing. Checking it against
    // a single flag would fail whichever of the two was not named.
    {DEMO,  ID_D_CUDA,      ""},
    {REPORT, ID_R_NOVETO,   "--no-veto"},
    {REPORT, ID_R_NOHOME,   "--no-homeward"},
    {REPORT, ID_R_VARY,     "--vary-goal"},
};
const int NFLAG_BTNS = int(sizeof FLAG_BTNS / sizeof *FLAG_BTNS);

void panelReport(cv::Mat& im, std::vector<Btn>& bs, const Cfg& c) {
    const int x = 266;
    txt(im, "how it fails, as pictures", x, 106, 0.66, INK, 2);
    txt(im, "Every other view here reports the policy as numbers, and a table "
            "cannot say", x, 128, 0.44, DIM);
    txt(im, "WHY. Six episodes came back as five rows all reading 'ran out of "
            "steps':", x, 146, 0.44, DIM);
    txt(im, "one never left the spawn, one flew the wrong way, one was still "
            "closing.", x, 164, 0.44, DIM);

    worldRow(im, bs, x, 194, ID_RW, c.rw);

    stepper(im, bs, x, 268, "first seed", std::to_string(c.rSeed0),
            ID_R_S0M, ID_R_S0P, "world instance", 100);
    stepper(im, bs, x + 150, 268, "last seed", std::to_string(c.rSeed1),
            ID_R_S1M, ID_R_S1P, nullptr, 100);
    stepper(im, bs, x + 300, 268, "steps/run", std::to_string(c.rSteps),
            ID_R_STM, ID_R_STP, "the goal must fit", 100);
    // A DETERMINISTIC POLICY GIVES ONE TRAIL PER WORLD however many times you
    // ask, and one trail says nothing about where failures cluster. Repeats
    // above 1 sample instead, which is what turns the map into a distribution.
    stepper(im, bs, x + 450, 268, "repeats", std::to_string(c.rRepeats),
            ID_R_RPM, ID_R_RPP, "runs per instance", 100);

    const int nrun = nWorldsOn(c.rw) * std::max(0, c.rSeed1 - c.rSeed0 + 1)
    // 10, NOT 5: one policy plus the NINE classical planners. This number is
    // the only warning before a 20-minute run, and it under-counted by half
    // the moment the bar stopped being four planners.
                   * std::max(1, c.rRepeats) * (c.rBaselines ? 10 : 1);
    txt(im, std::to_string(nrun) + " episodes", x + 610, 292, 0.5, INK);

    bs.push_back({cv::Rect(x, 356, 250, 36),
                  c.rDet ? "argmax (one trail each)" : "sampled (a spread)",
                  ID_R_DET, c.rDet});
    bs.push_back({cv::Rect(x + 266, 356, 250, 36),
                  c.rProgress ? "EVERY checkpoint" : "newest checkpoint",
                  ID_R_PROGRESS, c.rProgress});
    bs.push_back({cv::Rect(x + 532, 356, 210, 36),
                  c.rBaselines ? "with the 9 baselines" : "the policy alone",
                  ID_R_BASE, c.rBaselines});

    bs.push_back({cv::Rect(x, 400, 250, 36),
                  c.rRandom ? "random (the floor)" : "the trained policy",
                  ID_R_RANDOM, c.rRandom});
    bs.push_back({cv::Rect(x + 266, 400, 250, 36),
                  c.rStereo ? "Simulated stereo" : "Perfect depth",
                  ID_R_STEREO, c.rStereo});
    bs.push_back({cv::Rect(x + 532, 400, 210, 36),
                  c.rNoVeto ? "veto OFF" : "veto on", ID_R_NOVETO, c.rNoVeto});
    bs.push_back({cv::Rect(x, 444, 250, 36),
                  c.rVary ? "varied journey" : "fixed journey",
                  ID_R_VARY, c.rVary});
    // THE ONLY SETTING HERE THAT CAN MAKE A NUMBER MEANINGLESS RATHER THAN BAD.
    // g[22],g[23] say how far the aircraft is from its spawn and which way that
    // lies. A checkpoint trained before they carried signal has first-layer
    // weights on them that never saw a gradient, so scoring it with them live
    // feeds two untrained weights real numbers -- the run is noise, and it does
    // not look like noise, it looks like a bad policy.
    //
    // report reads the right answer out of the run's run.json and this button
    // is the override for a checkpoint that arrived without one (--model
    // pointing somewhere else). It does not touch the baselines: they read only
    // the per-primitive block.
    bs.push_back({cv::Rect(x + 266, 444, 250, 36),
                  c.rNoHome ? "spawn bearing OFF" : "spawn bearing on",
                  ID_R_NOHOME, c.rNoHome});

    txt(im, "Writes report.csv beside the weights and draws five panels from "
            "it: which failure is each", x, 494, 0.42, DIM);
    txt(im, "world's failure, every episode's distance-to-goal curve, where "
            "the reward went, and the", x, 512, 0.42, DIM);
    txt(im, "trails laid over the world they were flown in.", x, 530, 0.42, DIM);
    txt(im, "Redraw without flying again:  kestrel report --plot <dir>",
        x, 552, 0.42, DIM);
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
    // HEADER: the app's name, what it is, and which mode is open.
    cv::rectangle(im, {0, 0, W, HEADER_H}, HEADER, cv::FILLED);
    cv::line(im, {0, HEADER_H}, {W, HEADER_H}, EDGE, 1);
    roundRect(im, {26, 22, 34, 34}, 8, ACCENT);                  // the mark
    {
        const std::vector<cv::Point> k{{35, 30}, {35, 48}, {40, 43}, {50, 48}, {44, 39}, {50, 30}};
        cv::polylines(im, k, false, INK, 2, cv::LINE_AA);
    }
    txt(im, "kestrel", 72, 46, 0.95, INK, 2);
    txt(im, "object lock, planner baselines, the live voxel sim, RL training",
        74, 70, 0.44, DIM);
    {
        std::string m = MODE_NAME[c.mode];
        for (char& ch : m) ch = char(std::toupper((unsigned char)ch));
        int base = 0;
        const cv::Size ts = cv::getTextSize(m, cv::FONT_HERSHEY_SIMPLEX, 0.46, 1, &base);
        const cv::Rect pill(W - 28 - ts.width - 28, 30, ts.width + 28, 26);
        roundRect(im, pill, 13, SURFACE);
        txt(im, m, pill.x + 14, pill.y + 18, 0.46, INK, 1);
    }

    // SIDEBAR: navigation, then the action.
    cv::rectangle(im, {0, HEADER_H + 1, SIDEBAR_W, H - HEADER_H - 1}, SIDEBAR, cv::FILLED);
    cv::line(im, {SIDEBAR_W, HEADER_H + 1}, {SIDEBAR_W, H}, EDGE, 1);

    bs.clear();
    for (int i = 0; i < NMODES; ++i) {
        // Derived, not constants: runY below is computed from the same
        // numbers, which is what keeps a new mode from landing on RUN (it
        // once did, and `gui --check` caught it).
        Btn nb{cv::Rect(14, NAV_Y0 + i * NAV_PITCH, SIDEBAR_W - 28, 40), MODE_NAME[i],
               ID_MODE + i, c.mode == i};
        nb.kind = 1;
        bs.push_back(nb);
    }

    switch (c.mode) {
        case TRACK: panelTrack(im, bs, c, inputs); break;
        case BENCH: panelBench(im, bs, c); break;
        case SIM:   panelSim(im, bs, c, recs); break;
        case DEMO:  panelDemo(im, bs, c, recs); break;
        case WATCH: panelWatch(im, bs, c); break;
        case EVAL:  panelEval(im, bs, c); break;
        case REPORT: panelReport(im, bs, c); break;
        default:    panelTrain(im, bs, c); break;
    }

    const std::string why = blocker(c, inputs, recs);
    // Below the LAST mode button, computed rather than a constant: adding the
    // fifth mode put a button straight through RUN, and `gui --check` caught it
    // on the first run. Derive it and it cannot happen again.
    const int runY = NAV_Y0 + NMODES * NAV_PITCH + 14;
    Btn runBtn{cv::Rect(14, runY, SIDEBAR_W - 28, 54), "RUN", ID_RUN};
    runBtn.go = why.empty();
    bs.push_back(runBtn);
    if (!why.empty()) txt(im, why, 14, runY + 74, 0.4, cv::Scalar(90, 150, 230));
    txt(im, "q / esc  quit", 20, H - 22, 0.42, DIM);

    // The command strip, a console line. Not decoration: it is what RUN
    // executes, and you can type it instead.
    roundRect(im, {266, H - 98, W - 294, 48}, 8, CONSOLE);
    txt(im, "$", 278, H - 68, 0.5, GO, 2);
    std::vector<std::string> tok{"kestrel", MODE_NAME[c.mode]};
    for (const std::string& a : buildArgs(c, inputs, recs)) tok.push_back(a);
    if (c.mode == TRACK && c.designate) tok.push_back("(+ --box from your click)");
    int dropped = 0;
    const std::vector<std::string> lines =
        wrapCmd(tok, W - 336, 0.44, 2, &dropped);
    {
        // The strip's own lines live INSIDE the strip by construction, so they
        // must not be measured against it -- the same reason drawBtn keeps a
        // button's label out of the text boxes.
        std::vector<cv::Rect>* keep = g_textBoxes;
        g_textBoxes = nullptr;
        struct Restore { std::vector<cv::Rect>*& g; std::vector<cv::Rect>* v;
                         ~Restore() { g = v; } } restore{g_textBoxes, keep};
        for (size_t i = 0; i < lines.size(); ++i)
            txt(im, lines[i], 296, H - 78 + int(i) * 18, 0.44, INK);
    }
    txt(im, "the command RUN executes -- you can type it instead",
        268, H - 30, 0.4, DIM);

    for (const Btn& b : bs) drawBtn(im, b);
    return im;
}

// ------------------------------------------------------------ click handling
void apply(int id, Cfg& c, const std::vector<TrackInput>& inputs,
           const std::vector<std::string>& recs) {
    if (id >= ID_MODE && id < ID_MODE + NMODES) { c.mode = id - ID_MODE; return; }
    if (id >= ID_D_MODEL && id < ID_D_MODEL + 30) {
        c.dModel = id - ID_D_MODEL - 1;      // the first entry is "none"
        return;
    }
    if (id >= ID_TRACK_INPUT && id < ID_TRACK_INPUT + 30) {
        c.input = id - ID_TRACK_INPUT; return;
    }
    if (id >= ID_SIM_SRC && id < ID_SIM_SRC + 3) { c.simSource = id - ID_SIM_SRC; return; }
    if (id == ID_SIM_NEAR) { c.simNear = !c.simNear; return; }
    if (id == ID_SIM_TILT) { c.simTilt = (c.simTilt + 1) % NSIM_TILT; return; }
    if (id >= ID_SIM_POSE && id < ID_SIM_POSE + 3) { c.simPose = id - ID_SIM_POSE; return; }
    if (id >= ID_D_POSE && id < ID_D_POSE + 3) { c.dPose = id - ID_D_POSE; return; }
    if (id >= ID_SIM_REPLAY && id < ID_SIM_REPLAY + 20) {
        c.replay = id - ID_SIM_REPLAY; return;
    }
    // The three world rows, as ranges rather than eighteen hand-typed cases.
    if (id >= ID_BW && id < ID_BW + NWORLDS) { c.bw[id - ID_BW] ^= 1; return; }
    if (id >= ID_BENCH_POL && id < ID_BENCH_POL + NPOLICY) {
        c.bp[id - ID_BENCH_POL] ^= 1; return;
    }
    if (id >= ID_WW && id < ID_WW + NWORLDS) { c.ww[id - ID_WW] ^= 1; return; }
    if (id >= ID_EW && id < ID_EW + NWORLDS) { c.ew[id - ID_EW] ^= 1; return; }
    if (id >= ID_RW && id < ID_RW + NWORLDS) { c.rw[id - ID_RW] ^= 1; return; }
    switch (id) {
        case ID_TRACK_SIZE_M: c.boxSize = std::max(16, c.boxSize - 16); break;
        case ID_TRACK_SIZE_P: c.boxSize = std::min(256, c.boxSize + 16); break;
        case ID_TRACK_DESIG:  c.designate = !c.designate; break;
        case ID_TRACK_LIM_M:  c.frameLimit = std::max(0, c.frameLimit - 50); break;
        case ID_TRACK_LIM_P:  c.frameLimit = std::min(5000, c.frameLimit + 50); break;
        case ID_TRACK_CSV:    c.csv = !c.csv; break;

        case ID_BENCH_S0M:    c.seed0 = std::max(1, c.seed0 - 1);
                              c.seed1 = std::max(c.seed0, c.seed1); break;
        case ID_BENCH_S0P:    c.seed0 = std::min(999, c.seed0 + 1);
                              c.seed1 = std::max(c.seed0, c.seed1); break;
        case ID_BENCH_S1M:    c.seed1 = std::max(c.seed0, c.seed1 - 1); break;
        case ID_BENCH_S1P:    c.seed1 = std::min(999, c.seed1 + 1); break;
        case ID_BENCH_STM:    c.steps = std::max(100, c.steps - 100); break;
        case ID_BENCH_STP:    c.steps = std::min(5000, c.steps + 100); break;
        case ID_BENCH_STEREO: c.benchStereo = !c.benchStereo; break;
        case ID_BENCH_ORACLE: c.benchOracle = !c.benchOracle; break;

        case ID_TRAIN_WM:     c.workers = std::max(1, c.workers - 1); break;
        case ID_TRAIN_WP:     c.workers = std::min(32, c.workers + 1); break;
        case ID_TRAIN_SM:     c.stepsIdx = std::max(0, c.stepsIdx - 1); break;
        case ID_TRAIN_SP:     c.stepsIdx = std::min(NTRAIN_STEPS - 1, c.stepsIdx + 1); break;
        case ID_TRAIN_STEREO: c.trainStereo = !c.trainStereo; break;
        case ID_TRAIN_CUDA:   c.cuda = !c.cuda; break;
        case ID_TRAIN_RESUME: c.resume = !c.resume; break;
        case ID_TRAIN_NOVETO: c.noVeto = !c.noVeto; break;
        case ID_TRAIN_EPM:    c.epLen = std::max(500, c.epLen - 500); break;
        case ID_TRAIN_EPP:    c.epLen = std::min(10000, c.epLen + 500); break;
        case ID_TRAIN_VARY:   c.varyGoal = !c.varyGoal; break;
        case ID_TRAIN_SVM:    c.saveIdx = std::max(0, c.saveIdx - 1); break;
        case ID_TRAIN_SVP:    c.saveIdx = std::min(NSAVE_EVERY - 1, c.saveIdx + 1); break;
        case ID_TRAIN_EXM:    c.exploreIdx = std::max(0, c.exploreIdx - 1); break;
        case ID_TRAIN_EXP:    c.exploreIdx = std::min(NEXPLORE - 1, c.exploreIdx + 1); break;
        case ID_TRAIN_ANM:    c.annealIdx = std::max(0, c.annealIdx - 1); break;
        case ID_TRAIN_ANP:    c.annealIdx = std::min(NANNEAL - 1, c.annealIdx + 1); break;
        case ID_TRAIN_KLM:    c.klIdx = std::max(0, c.klIdx - 1); break;
        case ID_TRAIN_KLP:    c.klIdx = std::min(NTARGET_KL - 1, c.klIdx + 1); break;
        case ID_R_S0M: c.rSeed0 = std::max(1, c.rSeed0 - 1);
                       c.rSeed1 = std::max(c.rSeed1, c.rSeed0); break;
        case ID_R_S0P: c.rSeed0 += 1; c.rSeed1 = std::max(c.rSeed1, c.rSeed0); break;
        case ID_R_S1M: c.rSeed1 = std::max(c.rSeed0, c.rSeed1 - 1); break;
        case ID_R_S1P: c.rSeed1 += 1; break;
        case ID_R_STM: c.rSteps = std::max(100, c.rSteps - 250); break;
        case ID_R_STP: c.rSteps = std::min(10000, c.rSteps + 250); break;
        case ID_R_RPM: c.rRepeats = std::max(1, c.rRepeats - 1); break;
        case ID_R_RPP: c.rRepeats = std::min(20, c.rRepeats + 1); break;
        case ID_R_DET: c.rDet = !c.rDet; break;
        case ID_R_PROGRESS: c.rProgress = !c.rProgress; break;
        case ID_R_BASE: c.rBaselines = !c.rBaselines; break;
        case ID_R_NOHOME: c.rNoHome = !c.rNoHome; break;
        case ID_D_SOURCE:     c.dSource = 0; break;
        case ID_D_SOURCE + 1: c.dSource = 1; break;
        case ID_D_SOURCE + 2: c.dSource = 2; break;
        case ID_D_WORLD:  c.dWorld = (c.dWorld + 1) % NDEMO_WORLD; break;
        case ID_D_PANEM:  c.dPane = std::max(0, c.dPane - 1); break;
        case ID_D_PANEP:  c.dPane = std::min(NDEMO_PANE - 1, c.dPane + 1); break;
        case ID_D_PEOPLE:  c.dPeople = (c.dPeople + 1) % 3; break;
        case ID_D_MIRROR:  c.dNoMirror = !c.dNoMirror; break;
        case ID_D_EMITTER: c.dNoEmitter = !c.dNoEmitter; break;
        case ID_D_CUDA:    c.dCuda = (c.dCuda + 1) % 3; break;
        case ID_BENCH_CFM: c.coreIdx = std::max(0, c.coreIdx - 1); break;
        case ID_BENCH_CFP: c.coreIdx = std::min(NCORE_FRAC - 1, c.coreIdx + 1); break;
        case ID_R_RANDOM: c.rRandom = !c.rRandom; break;
        case ID_R_STEREO: c.rStereo = !c.rStereo; break;
        case ID_R_NOVETO: c.rNoVeto = !c.rNoVeto; break;
        case ID_R_VARY: c.rVary = !c.rVary; break;
        case ID_TRAIN_RAWCLR: c.rawClear = !c.rawClear; break;
        case ID_TRAIN_VHM: c.valueHIdx = std::max(0, c.valueHIdx - 1); break;
        case ID_TRAIN_VHP: c.valueHIdx = std::min(NVALUE_H - 1, c.valueHIdx + 1); break;
        case ID_TRAIN_CHM: c.creditHIdx = std::max(0, c.creditHIdx - 1); break;
        case ID_TRAIN_CHP: c.creditHIdx = std::min(NCREDIT_H - 1, c.creditHIdx + 1); break;
        case ID_TRAIN_SEEDM: c.trainSeed = std::max(0, c.trainSeed - 1); break;
        case ID_TRAIN_SEEDP: c.trainSeed = std::min(999, c.trainSeed + 1); break;
        case ID_TRAIN_OBJ: c.objMode = (c.objMode + 1) % 4; break;
        case ID_TRAIN_NORMR: c.normReward = !c.normReward; break;
        case ID_TRAIN_CLIPVF: c.clipVf = !c.clipVf; break;

        case ID_W_PANES_M: c.panes = std::max(1, c.panes - 1); break;
        case ID_W_PANES_P: c.panes = std::min(9, c.panes + 1); break;
        case ID_W_PX_M:    c.paneIdx = std::max(0, c.paneIdx - 1); break;
        case ID_W_PX_P:    c.paneIdx = std::min(NPANE_PX - 1, c.paneIdx + 1); break;
        case ID_W_LAYOUT:  c.layout = (c.layout + 1) % NLAYOUT; break;
        case ID_W_DET:     c.wDet = !c.wDet; break;

        case ID_E_S0M:    c.eSeed0 = std::max(1, c.eSeed0 - 1);
                          c.eSeed1 = std::max(c.eSeed0, c.eSeed1); break;
        case ID_E_S0P:    c.eSeed0 = std::min(999, c.eSeed0 + 1);
                          c.eSeed1 = std::max(c.eSeed0, c.eSeed1); break;
        case ID_E_S1M:    c.eSeed1 = std::max(c.eSeed0, c.eSeed1 - 1); break;
        case ID_E_S1P:    c.eSeed1 = std::min(999, c.eSeed1 + 1); break;
        case ID_E_STM:    c.eSteps = std::max(100, c.eSteps - 100); break;
        case ID_E_STP:    c.eSteps = std::min(5000, c.eSteps + 100); break;
        case ID_E_RANDOM: c.eRandom = !c.eRandom; break;
        case ID_E_STEREO: c.eStereo = !c.eStereo; break;
        case ID_E_BASE:   c.eBaselines = !c.eBaselines; break;
        case ID_E_REWARD: c.eReward = !c.eReward; break;
        case ID_E_PROGRESS: c.eProgress = !c.eProgress; break;
        case ID_E_NOVETO:   c.eNoVeto = !c.eNoVeto; break;
        case ID_E_VARY:     c.eVary = !c.eVary; break;
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
    refreshPy(exeDir);
    refreshModels(exeDir);

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
        // Two buttons act rather than set: install, and the interpreter
        // listing. Both go through the same hand-the-screen-over path as RUN so
        // their output lands in the console in the same place.
        const bool isInstall = (hit == ID_TRAIN_INSTALL);
        const bool isPythons = (hit == ID_TRAIN_PYTHONS);
        const bool isShot    = (hit == ID_D_SHOT);
        const bool isExport  = (hit == ID_D_EXPORT);
        if (hit != ID_RUN && !isInstall && !isPythons && !isShot && !isExport) {
            apply(hit, c, inputs, recs);
            continue;
        }
        if (hit == ID_RUN && !why.empty()) continue;

        // Install is dispatched through train, so there is exactly one place
        // that decides which interpreter is meant.
        std::vector<std::string> args =
            isExport  ? std::vector<std::string>{"--script", "export_onnx.py"}
          : isInstall ? std::vector<std::string>{"--install"}
          : isPythons ? std::vector<std::string>{}
                      : buildArgs(c, inputs, recs);
        // The shot writes PREFIX_*.png beside the exe and returns immediately,
        // so it keeps the rest of the panel's settings rather than being a
        // second, differently-configured demo.
        if (isShot) args.insert(args.begin(), {"--shot", "demo"});

        // Designating happens IN this window, before it is torn down, because
        // it needs the first frame on screen and a click on it.
        if (!isInstall && !isPythons && !isShot && c.mode == TRACK && c.designate && c.input >= 0) {
            float bx = 0, by = 0;
            if (clickTarget(inputs[c.input].args.front(), bx, by)) {
                std::vector<std::string> box{"--box", std::to_string(int(bx)),
                                             std::to_string(int(by)),
                                             std::to_string(c.boxSize)};
                args.insert(args.begin(), box.begin(), box.end());
            }
        } else if (!isInstall && !isPythons && !isShot && c.mode == TRACK) {
            args.insert(args.begin(), {"--box", "-1", "-1", std::to_string(c.boxSize)});
        }

        // HAND THE SCREEN OVER. track and sim open windows of their own, and
        // bench and train print for minutes to hours; leaving a dead launcher
        // behind either fights for the window or looks hung. It comes back
        // when the command returns.
        cv::destroyWindow(WIN);
        cv::waitKey(1);
        std::printf("\n[kestrel] %s",
                    isPythons ? "python" : isInstall ? "train" : MODE_NAME[c.mode]);
        for (const std::string& a : args) std::printf(" %s", a.c_str());
        std::printf("\n");
        std::fflush(stdout);

        int rc = 0;
        if (isPythons)      rc = act.pythons();
        else if (isInstall) rc = act.train(args);
        else if (isShot)    rc = act.demo(args);
        else if (isExport)  rc = act.train(args);
        else switch (c.mode) {
            case TRACK: rc = act.track(args); break;
            case BENCH: rc = act.bench(args); break;
            case SIM:   rc = act.sim(args);   break;
            case DEMO:  rc = act.demo(args);  break;
            case WATCH: rc = act.watch(args); break;
            case EVAL:  rc = act.eval(args);  break;
            // REPORT WAS NOT HERE, so it fell into `default` and RUN on the
            // report panel launched TRAINING with report's arguments. The
            // action was bound in kestrel.cpp and never called. `default` is
            // why it was silent -- a mode that forgets to list itself gets
            // whatever the last line does, so every mode is now named and
            // default only catches TRAIN.
            case REPORT: rc = act.report(args); break;
            default:    rc = act.train(args); break;
        }
        // The listing is read in the terminal, so hold the window closed until
        // it has been: reopening instantly would put it back over the output.
        if (isPythons) {
            std::printf("\n[kestrel] press Enter to return to the window ");
            std::fflush(stdout);
            int ch; while ((ch = std::getchar()) != '\n' && ch != EOF) {}
        }
        if (rc != 0) std::printf("[kestrel] %s exited %d\n", MODE_NAME[c.mode], rc);
        std::fflush(stdout);

        // Inputs may have appeared while we were away -- a run of `sim` writes
        // recordings, and `track` may have been pointed at a folder someone
        // filled in the meantime.
        inputs = findTrackInputs(exeDir);
        recs = findRecordings(exeDir);
        refreshPy(exeDir);
        refreshModels(exeDir);
        if (c.dModel >= int(g_models.size())) c.dModel = -1;
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
        // FOUR variants, not three. The fourth turns every world off, which is
        // the only state that makes blocker() return a string -- so until it
        // existed the "pick at least one world" line was never laid out by the
        // check at all, and could overflow its column unnoticed.
        for (int variant = 0; variant < 4; ++variant) {
            // Panels change shape with their own settings -- sim grows a file
            // list, track swaps its hints -- so each is laid out in more than
            // one state rather than only its default.
            Cfg c;
            c.mode = m;
            c.input = 0; c.replay = 0;
            // Variant 3 is the REFUSED state: nothing picked anywhere, so every
            // panel's blocker string is actually drawn and measured. The track
            // one is 436 px at its 0.40 scale and starts at x=28, so it reaches
            // well into the panel column -- exactly the kind of overrun the
            // check exists for, and it could not see it while every variant was
            // a runnable state.
            if (variant == 3) { c.input = -1; c.replay = -1; }
            c.simSource = variant;
            c.simNear = (variant == 2);      // lay out both toggle captions
            c.simTilt = variant % NSIM_TILT; // and the tilt captions
            c.designate = c.csv = (variant != 1);
            // Vary the world rows too, and include an ALL-OFF state -- without
            // one, blocker() returned "" in every checked variant and the
            // "pick at least one world" line was never laid out at all.
            for (int w = 0; w < NWORLDS; ++w) {
                c.bw[w] = c.ww[w] = c.ew[w] = (variant != 1) || (w == 0);
                if (variant == 3) c.bw[w] = c.ww[w] = c.ew[w] = false;
            }
            c.benchStereo = c.trainStereo = c.cuda = (variant == 1);
            c.benchOracle = (variant == 2);
            c.frameLimit = variant * 50;
            c.stepsIdx = variant;

            // A SYNTHETIC PYTHON STATE, and a different one per variant. The
            // check ran with g_py unprobed, so panelTrain always drew its short
            // "not checked yet" branch and the long interpreter paths -- the
            // ones that actually overflowed -- were never laid out at all. A
            // check that only ever sees the empty state is not checking the
            // panel, it is checking a placeholder.
            const PyState saved = g_py;
            kpy::Py fake;
            fake.exe = "C:\\Users\\Somebody\\AppData\\Local\\Programs\\"
                       "Python\\Python311\\python.exe";
            fake.major = 3; fake.minor = 11; fake.runs = true;
            fake.origin = "PATH, py -3.11";
            fake.voxelenv = (variant != 2);
            fake.rl = (variant == 0);
            g_py.probed = true;
            g_py.pys = {fake};
            g_py.abi = "3.11";

            std::vector<Btn> bs;
            std::vector<cv::Rect> texts;
            g_textBoxes = &texts;
            const cv::Mat im = compose(c, inputs, recs, bs);
            g_textBoxes = nullptr;
            const std::string tag =
                std::string(MODE_NAME[m]) + "/" + std::to_string(variant);

            // Text running underneath a button. This is the case the check was
            // blind to: it compared buttons against buttons, so a paragraph
            // drawn straight through "Install the RL stack" passed clean.
            for (const cv::Rect& t : texts)
                for (const Btn& b : bs)
                    if ((t & b.r).area() > 0) {
                        std::printf("%s: text overlaps button '%s'\n",
                                    tag.c_str(), b.label.c_str());
                        ++bad;
                    }
            // THE STRIP SHOWS THE WHOLE COMMAND. "you can type it instead" is
            // only true if nothing was dropped; TRACK is exempt because its
            // input is an arbitrary file name of any length.
            if (m != TRACK) {
                std::vector<std::string> tok{"kestrel", MODE_NAME[m]};
                for (const std::string& g : buildArgs(c, inputs, recs))
                    tok.push_back(g);
                int dropped = 0;
                wrapCmd(tok, W - 336, 0.44, 2, &dropped);
                if (dropped) {
                    std::printf("%s: command strip drops %d argument(s)\n",
                                tag.c_str(), dropped);
                    ++bad;
                }
            }
            // GREEN == THE FLAG IS EMITTED, for every plain flag toggle on
            // this panel. This is the rule that would have caught rawClear.
            {
                const std::vector<std::string> argv = buildArgs(c, inputs, recs);
                for (int f = 0; f < NFLAG_BTNS; ++f) {
                    if (FLAG_BTNS[f].mode != m) continue;
                    const bool emitted =
                        std::find(argv.begin(), argv.end(),
                                  std::string(FLAG_BTNS[f].flag)) != argv.end();
                    for (const Btn& b : bs) {
                        if (b.id != FLAG_BTNS[f].id) continue;
                        if (b.on != emitted) {
                            std::printf("%s: '%s' is %s but %s is %s the command\n",
                                        tag.c_str(), b.label.c_str(),
                                        b.on ? "GREEN" : "grey", FLAG_BTNS[f].flag,
                                        emitted ? "in" : "NOT in");
                            ++bad;
                        }
                    }
                }
            }
            // TEXT THAT LEAVES THE WINDOW. Buttons were checked against the
            // canvas from the start and text was not, and the difference
            // showed the moment a panel got a fourth column: three note lines
            // written at x+570 simply ran off the right-hand edge, and the
            // check passed them clean because they overlapped nothing.
            // Nothing is drawn there to overlap -- that is the whole problem.
            for (const cv::Rect& t : texts)
                if ((t & cv::Rect(0, 0, im.cols, im.rows)) != t) {
                    std::printf("%s: text runs off the canvas at (%d,%d %dx%d)\n",
                                tag.c_str(), t.x, t.y, t.width, t.height);
                    ++bad;
                }
            // TEXT ON TOP OF TEXT. The same fourth column put two stepper hints
            // into each other -- "how much random stuff it tries" ran straight
            // through "both come down over this" -- and neither is a button, so
            // the button test above could not see it either. Two labels in one
            // place are less readable than one.
            for (size_t i = 0; i < texts.size(); ++i)
                for (size_t j = i + 1; j < texts.size(); ++j)
                    if ((texts[i] & texts[j]).area() > 0) {
                        std::printf("%s: text overlaps text at (%d,%d) / (%d,%d)\n",
                                    tag.c_str(), texts[i].x, texts[i].y,
                                    texts[j].x, texts[j].y);
                        ++bad;
                    }
            // The command strip is drawn last and over everything, so a panel
            // note that reaches it is clipped in half rather than overlapping
            // visibly. It is a fixed rectangle; check against it too.
            const cv::Rect strip(266, H - 96, W - 294, 44);
            for (const cv::Rect& t : texts)
                if ((t & strip).area() > 0 && t.y < H - 80) {
                    std::printf("%s: text runs under the command strip\n",
                                tag.c_str());
                    ++bad;
                }
            g_py = saved;

            for (size_t i = 0; i < bs.size(); ++i) {
                const Btn& a = bs[i];
                if ((a.r & cv::Rect(0, 0, im.cols, im.rows)) != a.r) {
                    std::printf("%s: button '%s' is off the canvas\n",
                                tag.c_str(), a.label.c_str());
                    ++bad;
                }
                // ANY truncation at all. The old rule only fired below five
                // drawn characters, so "... (see what training does)" -- 27
                // characters that had lost the only word naming the state --
                // passed clean. A button label that does not fit its button is
                // a layout error whatever survives of it, and unlike a file
                // path there is no long tail here that was never meant to fit.
                const std::string drawn = fit(a.label, a.r.width - 16,
                                              a.go ? 0.62 : 0.52, false);
                if (!a.label.empty() && drawn != a.label) {
                    std::printf("%s: label '%s' does not fit its %d px button "
                                "(drawn as '%s')\n",
                                tag.c_str(), a.label.c_str(), a.r.width,
                                drawn.c_str());
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
    refreshPy(exeDir);
    // THE SHOT MUST SEE WHAT THE WINDOW SEES. g_models is file-scope state that
    // run() refreshes; without this line --shot rendered the demo panel with an
    // empty policy list however many .onnx files were sitting there, so the one
    // artefact that gets reviewed over ssh showed a control that looks broken.
    refreshModels(exeDir);
    int n = 0;
    for (int m = 0; m < NMODES; ++m) {
        Cfg c;
        c.mode = m;
        if (!inputs.empty()) c.input = 0;
        if (!recs.empty())   c.replay = 0;
        if (m == SIM && !recs.empty()) c.simSource = 2;   // show the replay list
        if (m == DEMO && !g_models.empty()) c.dModel = 0; // and the policy list
        std::vector<Btn> bs;
        const cv::Mat im = compose(c, inputs, recs, bs);
        const std::string f = prefix + "_" + MODE_NAME[m] + ".png";
        if (cv::imwrite(f, im)) { std::printf("%s\n", f.c_str()); ++n; }
    }
    return n;
}

}  // namespace kgui
#endif  // SIM_HAVE_HIGHGUI
