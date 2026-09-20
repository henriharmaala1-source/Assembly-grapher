// The demo. See kestrel_demo.hpp for what it shows and why those four things.
#include "kestrel_demo.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#if SIM_HAVE_HIGHGUI
#include <opencv2/highgui.hpp>
#endif
#if KESTREL_HAVE_OBJDETECT
#include <opencv2/objdetect.hpp>
#endif
#if KESTREL_HAVE_DNN
#include <opencv2/dnn.hpp>
#endif
// getCudaEnabledDeviceCount() is in core and returns 0 on a build without
// CUDA, so this header is safe everywhere and the question is answered at RUN
// time. That matters for the same reason librealsense is loaded at run time:
// the machine the demo is shown on is not the machine it was built on.
#include <opencv2/core/cuda.hpp>
#if KESTREL_HAVE_VIDEOIO
#include <opencv2/videoio.hpp>
#endif

#include "depth_vis.hpp"
#include "frame_source.hpp"
#include "rl_env.hpp"
#include "voxel_map.hpp"

namespace kdemo {
namespace {

// ------------------------------------------------------------------ painting
const cv::Scalar INK{240, 240, 240}, DIM{150, 150, 150}, EDGE{70, 70, 70};
const cv::Scalar BG{24, 24, 24}, WARN{60, 160, 250}, OK{120, 210, 120};

void txt(cv::Mat& im, const std::string& s, int x, int y, double sc,
         const cv::Scalar& c, int th = 1) {
    cv::putText(im, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, {0, 0, 0}, th + 2,
                cv::LINE_AA);
    cv::putText(im, s, {x, y}, cv::FONT_HERSHEY_SIMPLEX, sc, c, th, cv::LINE_AA);
}

// KEEP THE FRONT OF A CAPTION. Same rule as the GUI's buttons and for the same
// reason: the first words say which pane this is, and eliding them leaves a
// parenthetical that is identical for all four.
std::string fit(const std::string& s, int wpx, double sc) {
    int base = 0;
    if (cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width <= wpx)
        return s;
    std::string t = s;
    while (t.size() > 3 &&
           cv::getTextSize(t + "...", cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base).width > wpx)
        t.pop_back();
    return t + "...";
}

// Scale to fit WITHOUT stretching. A depth image and an FPV render have
// different aspects, and stretching one to the pane silently changes the
// apparent field of view -- which is exactly the quantity a viewer is trying
// to read off these panes.
cv::Mat letterbox(const cv::Mat& src, int w, int h) {
    cv::Mat out(h, w, CV_8UC3, BG);
    if (src.empty()) return out;
    const double s = std::min(double(w) / src.cols, double(h) / src.rows);
    cv::Mat r;
    cv::resize(src, r, {}, s, s, cv::INTER_NEAREST);
    r.copyTo(out(cv::Rect((w - r.cols) / 2, (h - r.rows) / 2, r.cols, r.rows)));
    return out;
}

// One pane: the image, a frame, a title and a subtitle. The subtitle is where
// a pane says what it is ACTUALLY showing -- "classical: cover" rather than
// "learned" when no network loaded. A demo that mislabels its own panes is
// worse than a demo with fewer panes.
struct Pane {
    std::string title, sub;
    cv::Mat     img;
    cv::Scalar  subColour = DIM;
};

const int CAPTION_H = 46;   // title + subtitle strip under each image

void drawPane(cv::Mat& canvas, const cv::Rect& r, const Pane& p) {
    cv::rectangle(canvas, r, BG, cv::FILLED);
    const cv::Rect img(r.x, r.y, r.width, r.height - CAPTION_H);
    letterbox(p.img, img.width, img.height).copyTo(canvas(img));
    cv::rectangle(canvas, r, EDGE, 1);
    txt(canvas, fit(p.title, r.width - 20, 0.56),
        r.x + 10, r.y + r.height - 26, 0.56, INK, 1);
    txt(canvas, fit(p.sub, r.width - 20, 0.42),
        r.x + 10, r.y + r.height - 8, 0.42, p.subColour, 1);
}

// THE LAYOUT, as data, so `check` can assert it without a display and without
// running a single frame.
struct Layout {
    cv::Size canvas;
    cv::Rect pane[4];
    cv::Rect strip;          // the status line along the bottom
};

Layout layoutFor(int paneW, int paneH) {
    Layout L;
    const int PAD = 8, TOP = 44;
    const int pw = paneW, ph = paneH + CAPTION_H;
    L.canvas = {PAD + 2 * pw + PAD + PAD, TOP + 2 * ph + PAD + 34 + PAD};
    for (int i = 0; i < 4; ++i)
        L.pane[i] = cv::Rect(PAD + (i % 2) * (pw + PAD), TOP + (i / 2) * (ph + PAD),
                             pw, ph);
    L.strip = cv::Rect(PAD, L.canvas.height - PAD - 26, L.canvas.width - 2 * PAD, 26);
    return L;
}

// ------------------------------------------------------------------ backends
// WHERE THE TWO NETWORKS RUN, decided at run time and REPORTED.
//
// A demo that claims CUDA and silently runs on the CPU is worse than one that
// never mentioned it: the number on screen is then a CPU number wearing a GPU
// label, and the first question anyone asks about a live demo is how fast it
// is. So the backend each net actually got is written on its pane.
//
// OpenCV will accept DNN_BACKEND_CUDA on a build without CUDA and quietly fall
// back, which is exactly the silent-wrong-label case -- hence the device count
// is checked first rather than the request being trusted.
int cudaDevices() {
    try { return cv::cuda::getCudaEnabledDeviceCount(); }
    catch (const cv::Exception&) { return 0; }
}

// Returns the label to print. `want` is Options::Cuda.
std::string applyBackend(void* netv, int want, bool* usedCuda) {
    *usedCuda = false;
#if KESTREL_HAVE_DNN
    cv::dnn::Net& net = *static_cast<cv::dnn::Net*>(netv);
    if (want == Options::CUDA_OFF) {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        return "cpu (--no-cuda)";
    }
    const int n = cudaDevices();
    if (n > 0) {
        try {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            // FP16 is not free accuracy-wise and this is a detector and a
            // policy, not a benchmark, so the default target is the plain one.
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            *usedCuda = true;
            return cv::format("cuda (%d device%s)", n, n == 1 ? "" : "s");
        } catch (const cv::Exception& e) {
            return std::string("cpu (cuda refused: ") + e.what() + ")";
        }
    }
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return want == Options::CUDA_ON
             ? "cpu -- NO CUDA DEVICE, and --cuda asked for one"
             : "cpu (no cuda device)";
#else
    (void)netv; (void)want;
    return "cpu (this build has no opencv dnn)";
#endif
}

// --------------------------------------------------------------- the detector
// PEOPLE, and only the parts of "people" this build can honestly do.
//
// HOG + a linear SVM (Dalal & Triggs 2005) ships inside OpenCV with trained
// weights, so the default path needs no model file, no download and no network
// at demo time -- which is the difference between a demo that runs on a strange
// laptop and one that does not. It is also genuinely mediocre: upright,
// unoccluded, roughly full-body people only, and it costs tens of milliseconds.
// Both facts are stated on the pane rather than hidden.
//
// An ONNX detector is strictly better when one is present, and `--detector
// FILE` takes it. It is not the default because a default that fails when a
// file is missing is not a default.
//
// RANGE COMES FROM THE DEPTH FRAME, not from the box size. Estimating distance
// from how tall a person looks requires assuming how tall they are; this stack
// has a calibrated depth image, so the box's median valid depth is a
// measurement. That is the one thing here a webcam demo cannot do.
struct Person {
    cv::Rect box;
    float    rangeM = -1.f;    // < 0 means the depth frame had nothing there
    float    score = 0.f;
};

class PersonDetector {
public:
    bool init(const std::string& onnx, int want, std::string& note) {
#if KESTREL_HAVE_DNN
        if (!onnx.empty()) {
            try {
                net_ = cv::dnn::readNet(onnx);
                if (!net_.empty()) {
                    kind_ = ONNX;
                    backend_ = applyBackend(&net_, want, &cuda_);
                    note = "onnx on " + backend_;
                    return true;
                }
            } catch (const cv::Exception& e) {
                note = std::string("onnx failed, using HOG: ") + e.what();
            }
        }
#else
        if (!onnx.empty()) note = "this build has no opencv dnn; using HOG";
#endif
#if KESTREL_HAVE_OBJDETECT
        hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        kind_ = HOG;
        // HOG IS CPU HERE AND THERE IS NO GPU PATH FOR IT in mainline OpenCV
        // -- cv::cuda::HOG lives in the contrib cudaobjdetect module, which
        // this build does not require. So --cuda does nothing for the default
        // detector, and the pane says cpu rather than implying otherwise. The
        // way to put the detector on the GPU is --detector FILE.onnx.
        backend_ = "cpu (HOG has no cuda path in this build)";
        if (note.empty())
            note = "HOG + linear SVM, upright people, no model file";
        return true;
#else
        kind_ = NONE;
        note = "this build has no opencv objdetect -- no detector";
        return false;
#endif
    }

    bool available() const { return kind_ != NONE; }
    const std::string& backend() const { return backend_; }
    bool onCuda() const { return cuda_; }
    const char* kindName() const {
        return kind_ == ONNX ? "onnx" : kind_ == HOG ? "HOG" : "none";
    }

    // `bgr` is what the camera saw; `depthM` may be empty, in which case the
    // boxes come back without a range rather than with a guessed one.
    std::vector<Person> detect(const cv::Mat& bgr, const cv::Mat& depthM) {
        std::vector<Person> out;
        if (kind_ == NONE || bgr.empty()) return out;
#if KESTREL_HAVE_OBJDETECT
        if (kind_ == HOG) {
            // DOWNSCALE FIRST. HOG on a 640x480 frame is ~120 ms on a laptop
            // core, which would make this the slowest stage by a factor of
            // four; at 320 wide it is ~35 ms and finds the same people at the
            // ranges a demo happens at.
            cv::Mat small;
            const double s = 320.0 / std::max(1, bgr.cols);
            cv::resize(bgr, small, {}, s, s, cv::INTER_AREA);
            std::vector<cv::Rect> found;
            std::vector<double> weights;
            hog_.detectMultiScale(small, found, weights, 0.0, cv::Size(8, 8),
                                  cv::Size(0, 0), 1.05, 2.0, false);
            for (size_t i = 0; i < found.size(); ++i) {
                Person p;
                p.box = cv::Rect(int(found[i].x / s), int(found[i].y / s),
                                 int(found[i].width / s), int(found[i].height / s));
                p.score = float(weights[i]);
                out.push_back(p);
            }
        }
#endif
        for (Person& p : out) p.rangeM = rangeIn(depthM, p.box, bgr.size());
        return out;
    }

private:
    // MEDIAN, not mean. A box around a person also contains whatever is behind
    // them, and a mean of "person at 2 m" and "wall at 8 m" is a number that
    // describes nothing in the scene. The median of the middle of the box is
    // the person whenever the person fills most of it, which is what a box
    // that fired on a person looks like.
    static float rangeIn(const cv::Mat& depthM, const cv::Rect& box,
                         const cv::Size& imgSize) {
        if (depthM.empty() || depthM.type() != CV_32F) return -1.f;
        const double sx = double(depthM.cols) / std::max(1, imgSize.width);
        const double sy = double(depthM.rows) / std::max(1, imgSize.height);
        cv::Rect r(int(box.x * sx), int(box.y * sy),
                   int(box.width * sx), int(box.height * sy));
        // The middle half, so the frame of the box (mostly background) is out.
        r = cv::Rect(r.x + r.width / 4, r.y + r.height / 4, r.width / 2, r.height / 2);
        r &= cv::Rect(0, 0, depthM.cols, depthM.rows);
        if (r.width < 2 || r.height < 2) return -1.f;
        std::vector<float> v;
        for (int y = r.y; y < r.y + r.height; ++y) {
            const float* row = depthM.ptr<float>(y);
            for (int x = r.x; x < r.x + r.width; ++x)
                if (row[x] > 0.f) v.push_back(row[x]);
        }
        if (v.size() < 8) return -1.f;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }

    enum Kind { NONE = 0, HOG, ONNX } kind_ = NONE;
    std::string backend_ = "cpu";
    bool cuda_ = false;
#if KESTREL_HAVE_OBJDETECT
    cv::HOGDescriptor hog_;
#endif
#if KESTREL_HAVE_DNN
    cv::dnn::Net net_;
#endif
};

void drawPeople(cv::Mat& bgr, const std::vector<Person>& ps) {
    for (const Person& p : ps) {
        cv::rectangle(bgr, p.box, OK, 2);
        // The range is the point, so it goes on the box and not in a corner.
        const std::string lab = p.rangeM > 0.f
            ? cv::format("person  %.2f m", p.rangeM)
            : std::string("person  (no depth)");
        txt(bgr, lab, p.box.x, std::max(14, p.box.y - 6), 0.5,
            p.rangeM > 0.f ? OK : WARN, 1);
    }
}


// ---------------------------------------------------------------- the stages
// ONE SLOT PER STAGE, holding the latest FINISHED frame. A queue would be
// wrong here: the compositor wants the newest thing, not the oldest unshown
// one, and a stage that falls behind should drop frames rather than build a
// backlog that makes the pane lag further the longer the demo runs.
template <class T>
class Slot {
public:
    void publish(T v) {
        std::lock_guard<std::mutex> lk(m_);
        v_ = std::move(v);
        ++gen_;
    }
    bool take(T& out, unsigned long& seen) const {
        std::lock_guard<std::mutex> lk(m_);
        if (gen_ == seen) return false;
        out = v_; seen = gen_; return true;
    }
    unsigned long gen() const { std::lock_guard<std::mutex> lk(m_); return gen_; }

private:
    mutable std::mutex m_;
    T             v_{};
    unsigned long gen_ = 0;
};

struct PlannerFrame {
    cv::Mat fpv, depth;
    float   travelM = 0, netM = 0;
    int     steps = 0, cells = 0, collisions = 0;
};
struct CameraFrame {
    cv::Mat depthVis, mapFpv, colour;
    cv::Mat depthRaw;            // CV_32F metres, for the detector's range
    float   validFrac = 0;
};
struct PeopleFrame {
    cv::Mat  image;
    int      n = 0;
    double   msPerFrame = 0;
    // Whether the image came from the camera that measured the depth. When it
    // did not, the boxes have no range and the pane says why.
    bool     aligned = false;
};

// ------------------------------------------------------------ policy loading
// The policy pane needs an ACTION each step. Three ways to get one, and the
// pane must always say which it used:
//
//   ONNX       the exported checkpoint, run through opencv dnn. No python at
//              demo time, so the binary stays the only thing to install.
//   classical  a named planner from rl_env.hpp. Not a stand-in for the policy
//              -- it is a different planner and is captioned as one.
//
// THE MASK IS NOT OPTIONAL. Selecting a masked primitive is treated as "hold"
// by the environment, so a network whose logits are used raw will appear to
// stall rather than to choose badly. Masked entries go to -inf before argmax,
// exactly as the trainer does it.
class Policy {
public:
    bool loadOnnx(const std::string& path, int want, std::string& note) {
#if KESTREL_HAVE_DNN
        if (path.empty()) return false;
        try {
            net_ = cv::dnn::readNet(path);
            if (net_.empty()) { note = "onnx loaded empty: " + path; return false; }
            haveNet_ = true;
            backend_ = applyBackend(&net_, want, &cuda_);
            note = "learned policy, onnx on " + backend_;
            return true;
        } catch (const cv::Exception& e) {
            note = std::string("onnx failed: ") + e.what();
            return false;
        }
#else
        (void)path;
        note = "this build has no opencv dnn";
        return false;
#endif
    }

    int act(const std::vector<float>& obs, const std::vector<uint8_t>& mask,
            int nPrims, sim::BaselinePolicy fallback, unsigned& rng) {
#if KESTREL_HAVE_DNN
        if (haveNet_) {
            try {
                cv::Mat in(1, int(obs.size()), CV_32F, (void*)obs.data());
                net_.setInput(in.clone());
                cv::Mat logits = net_.forward();
                logits = logits.reshape(1, 1);
                int best = -1; float bv = -1e30f;
                for (int i = 0; i < nPrims && i < logits.cols; ++i) {
                    if (i < int(mask.size()) && !mask[i]) continue;   // the mask
                    const float v = logits.at<float>(0, i);
                    if (v > bv) { bv = v; best = i; }
                }
                if (best >= 0) return best;
            } catch (const cv::Exception&) {
                // A network that throws mid-demo falls back rather than ending
                // the demo, and the caption already says which is flying.
                haveNet_ = false;
            }
        }
#endif
        return sim::chooseBaseline(fallback, obs, mask, nPrims, rng);
    }

    bool learned() const { return haveNet_; }
    const std::string& backend() const { return backend_; }
    bool onCuda() const { return cuda_; }

private:
    bool haveNet_ = false;
    std::string backend_ = "cpu";
    bool cuda_ = false;
#if KESTREL_HAVE_DNN
    cv::dnn::Net net_;
#endif
};

sim::BaselinePolicy baselineByName(const std::string& s, bool* found) {
    using P = sim::BaselinePolicy;
    const P all[] = {P::Random, P::FreeM, P::Goal, P::Score, P::FreeG,
                     P::NovelG, P::Cover, P::FrontRaw, P::Circler};
    for (P p : all)
        if (s == sim::baselineName(p)) { if (found) *found = true; return p; }
    if (found) *found = false;
    return P::Cover;
}

// ---------------------------------------------------------- synthetic camera
// WHAT --shot AND --check SEE. No device, no world, no policy: a frame that
// exercises every drawing path so the layout can be asserted in ctest. It is
// deliberately obviously fake -- a demo screenshot must never be mistakable
// for a measurement.
cv::Mat syntheticColour(int w, int h) {
    cv::Mat im(h, w, CV_8UC3, cv::Scalar(40, 44, 50));
    for (int y = 0; y < h; ++y)
        im.row(y) = cv::Scalar(34 + y * 30 / h, 38 + y * 34 / h, 46 + y * 40 / h);
    // A person-shaped silhouette, so the detector pane has something with the
    // right aspect in it even where no detector ran.
    const int cx = w / 2, cy = h / 2;
    cv::circle(im, {cx, cy - h / 6}, h / 14, cv::Scalar(180, 180, 185), cv::FILLED);
    cv::rectangle(im, cv::Rect(cx - h / 12, cy - h / 12, h / 6, h / 3),
                  cv::Scalar(170, 172, 180), cv::FILLED);
    txt(im, "synthetic frame -- no camera attached", 10, h - 12, 0.42, WARN, 1);
    return im;
}

cv::Mat syntheticDepth(int w, int h) {
    cv::Mat d(h, w, CV_32F, cv::Scalar(0.f));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const float r = std::hypot(float(x - w / 2), float(y - h / 2));
            // A hole in the middle, because a depth image with no invalid
            // pixels is the one case that never happens and the one case a
            // three-state map is built for.
            d.at<float>(y, x) = (r < h / 10.f) ? 0.f : 1.0f + r * 0.02f;
        }
    return d;
}

}  // namespace

// --------------------------------------------------------------------- parse
bool parse(const std::vector<std::string>& args, Options& o, std::string& err) {
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= args.size()) { err = std::string(what) + " needs a value"; return ""; }
            return args[++i];
        };
        if (a == "--live")         o.source = Options::LIVE;
        else if (a == "--sim")     o.source = Options::SIM;
        else if (a == "--replay") { o.source = Options::REPLAY; o.replayPath = next("--replay"); }
        else if (a == "--model")   o.model = next("--model");
        else if (a == "--fallback") o.fallback = next("--fallback");
        else if (a == "--world")   o.world = next("--world");
        else if (a == "--seed")    o.seed = unsigned(std::stoul(next("--seed")));
        else if (a == "--detector") o.detector = next("--detector");
        else if (a == "--webcam") { o.eyes = Options::WEBCAM; o.webcamIndex = std::stoi(next("--webcam")); }
        else if (a == "--infrared") o.eyes = Options::INFRARED;
        else if (a == "--no-people") o.eyes = Options::NOEYES;
        else if (a == "--pane")   { o.paneW = std::stoi(next("--pane")); o.paneH = o.paneW * 3 / 4; }
        else if (a == "--no-mirror") o.mirror = false;
        else if (a == "--no-emitter") o.emitter = false;
        else if (a == "--cuda")     o.cuda = Options::CUDA_ON;
        else if (a == "--no-cuda")  o.cuda = Options::CUDA_OFF;
        else { err = "unknown argument: " + a; return false; }
        if (!err.empty()) return false;
    }
    // --cuda MEANS "I INTEND TO RUN ON THE GPU". Letting it pass silently on a
    // machine with no device is how a rehearsal becomes a surprise, so it is
    // refused here rather than noted on a pane an hour later. AUTO is the
    // default precisely so this is opt-in.
    if (o.cuda == Options::CUDA_ON && cudaDevices() == 0) {
        err = "--cuda: no CUDA device visible to OpenCV. Drop the flag to run "
              "on the CPU, or --no-cuda to say so deliberately.";
        return false;
    }
    bool ok = false;
    baselineByName(o.fallback, &ok);
    if (!ok) { err = "--fallback: no such planner: " + o.fallback; return false; }
    return true;
}

// --------------------------------------------------------------------- check
int check() {
    int bad = 0;
    auto fail = [&](const std::string& m) { std::printf("  %s\n", m.c_str()); ++bad; };
    for (int pw : {320, 480, 640}) {
        const Layout L = layoutFor(pw, pw * 3 / 4);
        const cv::Rect canvas(0, 0, L.canvas.width, L.canvas.height);
        for (int i = 0; i < 4; ++i) {
            if ((L.pane[i] & canvas) != L.pane[i])
                fail(cv::format("pane %d off the canvas at paneW=%d", i, pw));
            for (int j = i + 1; j < 4; ++j)
                if ((L.pane[i] & L.pane[j]).area() > 0)
                    fail(cv::format("panes %d and %d overlap at paneW=%d", i, j, pw));
            if ((L.pane[i] & L.strip).area() > 0)
                fail(cv::format("pane %d runs into the status strip at paneW=%d", i, pw));
            // A caption strip that cannot hold two lines of text is a caption
            // strip that silently drops the subtitle -- which is the line that
            // says whether the policy is learned or classical.
            if (L.pane[i].height - CAPTION_H < 40)
                fail(cv::format("pane %d has no room for its image at paneW=%d", i, pw));
        }
        if ((L.strip & canvas) != L.strip)
            fail(cv::format("status strip off the canvas at paneW=%d", pw));
    }
    // Every caption this demo can show must survive fit() with its FIRST WORD
    // intact at the narrowest pane, or the pane cannot say what it is.
    const char* caps[] = {"LEARNED PLANNER", "LIVE DEPTH", "LIVE VOXEL MAP",
                          "HUMANS", "classical: cover", "learned policy (onnx)",
                          "no detector in this build"};
    for (const char* c : caps) {
        const std::string f = fit(c, 320 - 20, 0.56);
        if (f.size() < 4 || f.substr(0, 3) == "...")
            fail(std::string("caption elided to nothing: ") + c);
    }
    std::printf("[demo check] %d layout violation(s)\n", bad);
    return bad;
}

// ---------------------------------------------------------------------- shot
int shot(const Options& o, const std::string& prefix) {
    const Layout L = layoutFor(o.paneW, o.paneH);
    const int iw = o.paneW, ih = o.paneH;

    // The planner pane from a real VoxelEnv, stepped a little so the map has
    // something in it -- this part needs no camera and no policy, so it is the
    // same picture the demo shows.
    sim::EnvConfig cfg;
    cfg.world = o.world; cfg.seed = o.seed; cfg.maxSteps = 100000;
    cfg.objective = sim::EnvConfig::RANGE;
    cfg.horizonS = (o.world == "maze") ? 0.6f : 2.0f;
    sim::VoxelEnv env(cfg);
    unsigned rng = 7u;
    bool found = false;
    const sim::BaselinePolicy fb = baselineByName(o.fallback, &found);
    // FAR ENOUGH TO HAVE A MAP. At 60 steps both the FPV and the plan view are
    // almost entirely UNKNOWN -- which is correct, and reads as a broken
    // renderer to anyone who has not been told that pale means fog. The shot is
    // a picture of the layout, so it shows the layout with something in it.
    for (int i = 0; i < 500; ++i) {
        const sim::EnvStep st = env.step(
            sim::chooseBaseline(fb, env.observation(), env.actionMask(),
                                env.nPrims(), rng));
        if (st.done || st.truncated) break;
    }
    auto toMat = [&](std::vector<uint8_t> v, int w, int h) {
        return v.size() == size_t(w) * h * 3
                   ? cv::Mat(h, w, CV_8UC3, v.data()).clone()
                   : cv::Mat(h, w, CV_8UC3, BG);
    };
    Pane p[4];
    p[0].title = "LEARNED PLANNER";
    p[0].sub   = std::string("classical: ") + sim::baselineName(fb);
    p[0].subColour = WARN;
    p[0].img   = toMat(env.renderFrame(iw, ih, false), iw, ih);

    const cv::Mat dRaw = syntheticDepth(iw, ih);
    p[1].title = "LIVE DEPTH";
    p[1].sub   = "synthetic -- no camera attached";
    p[1].subColour = WARN;
    p[1].img   = sim::colourDepth(dRaw, 8.f);

    p[2].title = "LIVE VOXEL MAP";
    p[2].sub   = "grey is UNKNOWN, and unknown is not free";
    p[2].img   = toMat(env.renderFrame(iw, ih, true), iw, ih);

    PersonDetector det;
    std::string dnote;
    det.init(o.detector, o.cuda, dnote);
    cv::Mat colour = syntheticColour(iw, ih);
    drawPeople(colour, det.detect(colour, dRaw));
    p[3].title = "HUMANS";
    p[3].sub   = det.available() ? dnote : "no detector in this build";
    p[3].subColour = det.available() ? DIM : WARN;
    p[3].img   = colour;

    cv::Mat canvas(L.canvas, CV_8UC3, BG);
    txt(canvas, "kestrel demo -- synthetic input, nothing measured here",
        12, 28, 0.62, INK, 1);
    for (int i = 0; i < 4; ++i) drawPane(canvas, L.pane[i], p[i]);
    txt(canvas, "source: shot (no device)   planner: classical   people: "
                + std::string(det.kindName()),
        L.strip.x + 4, L.strip.y + 18, 0.44, DIM, 1);

    int n = 0;
    const char* names[4] = {"planner", "depth", "voxel", "humans"};
    for (int i = 0; i < 4; ++i)
        n += cv::imwrite(prefix + "_" + names[i] + ".png", p[i].img) ? 1 : 0;
    n += cv::imwrite(prefix + "_window.png", canvas) ? 1 : 0;
    std::printf("[demo shot] %d image(s) written as %s_*.png\n", n, prefix.c_str());
    return n;
}


// ----------------------------------------------------------------------- run
#if !SIM_HAVE_HIGHGUI
int run(const Options&) {
    std::printf("[demo] this build has no OpenCV highgui, so there is no window.\n"
                "       `kestrel demo --shot PREFIX` still writes every pane to PNG.\n");
    return -1;
}
#else
namespace {

// The colour image the detector reads. Kept behind its own tiny seam for the
// same reason FrameSource exists: the detector must not know or care whether
// the pixels came from a camera or from nothing, and a demo whose one webcam
// is missing should lose one pane rather than fail to start.
class ColourSource {
public:
    bool open(const Options& o, std::string& note) {
        if (o.eyes == Options::NOEYES) { note = "people pane off (--no-people)"; return false; }
#if KESTREL_HAVE_VIDEOIO
        if (o.eyes == Options::AUTO || o.eyes == Options::WEBCAM) {
            cap_.open(o.webcamIndex);
            if (cap_.isOpened()) {
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, o.camW);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, o.camH);
                note = cv::format("webcam %d", o.webcamIndex);
                live_ = true;
                return true;
            }
        }
        note = "no camera opened; showing a synthetic frame";
#else
        note = "this build has no opencv videoio; showing a synthetic frame";
#endif
        return false;
    }

    // Always returns something drawable, so the pane never goes blank. It does
    // NOT mirror: the caller detects in sensor orientation and mirrors once,
    // afterwards, so there is exactly one place that decides which way round
    // the pane is.
    cv::Mat grab(int w, int h) {
#if KESTREL_HAVE_VIDEOIO
        if (live_) {
            cv::Mat f;
            if (cap_.read(f) && !f.empty()) return f;
            live_ = false;      // it went away mid-demo; fall through, say so
        }
#endif
        return syntheticColour(w, h);
    }
    bool live() const { return live_; }

private:
#if KESTREL_HAVE_VIDEOIO
    cv::VideoCapture cap_;
#endif
    bool live_ = false;
};

cv::Mat matFrom(const std::vector<uint8_t>& v, int w, int h) {
    return v.size() == size_t(w) * h * 3 ? cv::Mat(h, w, CV_8UC3, (void*)v.data()).clone()
                                         : cv::Mat(h, w, CV_8UC3, BG);
}

}  // namespace

int run(const Options& o) {
    const Layout L = layoutFor(o.paneW, o.paneH);
    const int iw = o.paneW, ih = o.paneH;

    std::atomic<bool> stop{false};

    // --- the planner stage ---------------------------------------------------
    sim::EnvConfig cfg;
    cfg.world = o.world; cfg.seed = o.seed;
    cfg.maxSteps = o.maxSteps > 0 ? o.maxSteps : 100000;
    cfg.objective = sim::EnvConfig::RANGE;
    cfg.horizonS = (o.world == "maze") ? 0.6f : 2.0f;

    Policy pol;
    std::string polNote;
    const bool learned = pol.loadOnnx(o.model, o.cuda, polNote);
    bool fbFound = false;
    const sim::BaselinePolicy fb = baselineByName(o.fallback, &fbFound);
    const std::string polLabel =
        learned ? polNote : std::string("classical: ") + sim::baselineName(fb);
    if (!learned && !o.model.empty())
        std::printf("[demo] %s\n       flying %s instead, and the pane says so.\n",
                    polNote.c_str(), sim::baselineName(fb));

    Slot<PlannerFrame> planSlot;
    std::thread planThread([&] {
        sim::VoxelEnv env(cfg);
        unsigned rng = o.seed * 7919u + 13u;
        while (!stop.load()) {
            const sim::EnvStep st = env.step(
                pol.act(env.observation(), env.actionMask(), env.nPrims(), fb, rng));
            PlannerFrame f;
            f.fpv = matFrom(env.renderFrame(iw, ih, false), iw, ih);
            f.depth = matFrom(env.renderDepth(iw, ih), iw, ih);
            f.travelM = st.travelM; f.netM = st.netDispM;
            f.steps = st.steps; f.cells = st.cellsVisited; f.collisions = st.collisions;
            planSlot.publish(std::move(f));
            // A COLLISION IS NOT THE END OF THE DEMO. It is the failure the
            // whole project is about, so it is shown -- briefly -- and then the
            // episode restarts, rather than the pane freezing on a dead frame
            // that a viewer reads as the program having crashed.
            if (st.done || st.truncated) {
                std::this_thread::sleep_for(std::chrono::milliseconds(700));
                env.reset(o.world, ++rng);
            }
        }
    });

    // --- the camera stage ----------------------------------------------------
    // SIM and REPLAY exist so this runs with no hardware. LIVE is the D435i.
    std::string srcNote;
    std::unique_ptr<sim::FrameSource> src;
    if (o.source == Options::LIVE) {
        std::string err;
        src = sim::makeLiveSource(o.camW, o.camH, o.camFps, o.emitter, &err);
        srcNote = src ? "RealSense" : ("no live camera: " + err);
    } else if (o.source == Options::REPLAY) {
        auto r = std::make_unique<sim::ReplayFrameSource>();
        std::string err;
        if (r->open(o.replayPath, &err)) { srcNote = "replay " + o.replayPath; src = std::move(r); }
        else srcNote = "replay failed: " + err;
    } else {
        srcNote = "sim -- the planner's own camera";
    }

    Slot<CameraFrame> camSlot;
    std::thread camThread([&] {
        sim::VoxelMap map;
        sim::VoxelMapParams mp;
        bool inited = false;
        while (!stop.load()) {
            CameraFrame f;
            if (src && src->ok()) {
                cv::Mat depth;
                sim::PoseHint hint;
                if (!src->next(depth, hint) || depth.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                // POSE IS THE HONEST LIMIT, exactly as in voxel_live: a
                // handheld camera has none, so the map is built from a FIXED
                // pose and says so. Inventing motion here would produce a map
                // that looks plausible and means nothing.
                const sim::CamPose pose = hint.valid ? hint.pose : sim::CamPose{};
                if (!inited) { map.init(mp, pose.e, pose.n, pose.u); inited = true; }
                map.integrate(depth, src->camera(), pose);
                int valid = 0;
                for (int y = 0; y < depth.rows; ++y) {
                    const float* r = depth.ptr<float>(y);
                    for (int x = 0; x < depth.cols; ++x) valid += (r[x] > 0.f);
                }
                f.validFrac = float(valid) / std::max(1, depth.rows * depth.cols);
                f.depthRaw = depth.clone();
                f.depthVis = sim::colourDepthEq(depth, 8.f);
                // THE IMAGE THE DEPTH WAS MEASURED IN, when the source has
                // one. Registered with the depth by construction, so a box
                // found here indexes straight into f.depthRaw -- which is what
                // makes "person at 2.3 m" a measurement rather than an
                // estimate from apparent height. See FrameSource::intensity.
                cv::Mat ir;
                if (src->intensity(ir) && !ir.empty())
                    cv::cvtColor(ir, f.colour, cv::COLOR_GRAY2BGR);
                f.mapFpv = map.fpvImage(pose.e, pose.n, pose.u, pose.yawDeg,
                                        pose.pitchDeg, iw, 90.f, 8.f);
            } else {
                // No camera: mirror the planner's own sensor, which is the
                // honest thing to show -- it IS simulated stereo from a real
                // VoxelMap, just not from a real room.
                PlannerFrame pf;
                unsigned long seen = 0;
                if (planSlot.take(pf, seen)) {
                    f.depthVis = pf.depth;
                    f.mapFpv = pf.fpv;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
            camSlot.publish(std::move(f));
        }
    });

    // --- the detector stage --------------------------------------------------
    ColourSource eyes;
    std::string eyesNote;
    const bool eyesLive = eyes.open(o, eyesNote);
    PersonDetector det;
    std::string detNote;
    det.init(o.detector, o.cuda, detNote);

    Slot<PeopleFrame> pplSlot;
    std::thread detThread([&] {
        unsigned long seenCam = 0;
        CameraFrame cf;
        while (!stop.load()) {
            const auto t0 = std::chrono::steady_clock::now();
            camSlot.take(cf, seenCam);
            // PREFER THE CAMERA THAT MEASURED THE DEPTH. A webcam image is
            // not registered with the RealSense's depth frame, so a box found
            // in it cannot honestly be given a range -- rangeIn would be
            // indexing a different camera's pixels and would return a
            // confident wrong number, which is the one failure mode this tree
            // refuses everywhere else. The webcam is the fallback for when
            // there is no depth camera at all, and then boxes come back
            // without a range rather than with a guessed one.
            const bool aligned = !cf.colour.empty();
            PeopleFrame pf;
            // DETECT IN THE ORIENTATION THE SENSOR DELIVERED, always. The
            // mirror is a courtesy to the person standing in front of the
            // camera -- it makes waving match -- and a mirrored image does not
            // index the same pixels as the depth frame, so detecting in it
            // would look the boxes up in the wrong half of the scene and
            // return a confident wrong range.
            cv::Mat sensorView = aligned ? cf.colour : eyes.grab(iw, ih);
            std::vector<Person> ps =
                det.detect(sensorView, aligned ? cf.depthRaw : cv::Mat());

            cv::Mat shown;
            if (o.mirror) {
                cv::flip(sensorView, shown, 1);
                for (Person& pp : ps)
                    pp.box.x = shown.cols - pp.box.x - pp.box.width;
            } else {
                shown = sensorView.clone();
            }
            drawPeople(shown, ps);
            pf.image = shown;
            pf.n = int(ps.size());
            pf.aligned = aligned;
            pf.msPerFrame = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - t0).count();
            pplSlot.publish(std::move(pf));
            if (!det.available() || !eyesLive)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    // --- the window ----------------------------------------------------------
    const char* WIN = "kestrel demo";
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);
    cv::Mat canvas(L.canvas, CV_8UC3, BG);
    PlannerFrame pf; CameraFrame cf; PeopleFrame hf;
    unsigned long g0 = 0, g1 = 0, g2 = 0;
    int rc = 0;
    while (true) {
        planSlot.take(pf, g0);
        camSlot.take(cf, g1);
        pplSlot.take(hf, g2);

        canvas.setTo(BG);
        txt(canvas, "kestrel demo", 12, 28, 0.62, INK, 1);

        Pane p[4];
        p[0].title = "LEARNED PLANNER";
        p[0].sub = polLabel;
        p[0].subColour = learned ? OK : WARN;
        p[0].img = pf.fpv;
        p[1].title = "LIVE DEPTH";
        p[1].sub = srcNote + (cf.validFrac > 0.f
                       ? cv::format("   %.0f%% of pixels returned", cf.validFrac * 100.f)
                       : std::string());
        p[1].subColour = (o.source == Options::LIVE && src) ? DIM : WARN;
        p[1].img = cf.depthVis.empty() ? pf.depth : cf.depthVis;
        p[2].title = "LIVE VOXEL MAP";
        p[2].sub = "grey is UNKNOWN, and unknown is not free";
        p[2].img = cf.mapFpv.empty() ? pf.fpv : cf.mapFpv;
        p[3].title = "HUMANS";
        p[3].sub = det.available()
                     ? cv::format("%s on %s   %d found   %.0f ms", det.kindName(),
                                  hf.aligned ? "the depth camera's own imager"
                                             : eyesNote.c_str(),
                                  hf.n, hf.msPerFrame)
                     : detNote;
        p[3].subColour = det.available() ? DIM : WARN;
        p[3].img = hf.image;
        for (int i = 0; i < 4; ++i) drawPane(canvas, L.pane[i], p[i]);

        txt(canvas,
            cv::format("step %d   travel %.0f m   net %.0f m   cells %d   collisions %d"
                       "   |  policy %s   people %s      [q] quit  [r] restart",
                       pf.steps, pf.travelM, pf.netM, pf.cells, pf.collisions,
                       pol.backend().c_str(), det.backend().c_str()),
            L.strip.x + 4, L.strip.y + 18, 0.44, DIM, 1);

        cv::imshow(WIN, canvas);
        const int k = cv::waitKey(20);
        if (k == 'q' || k == 27) break;
    }
    stop.store(true);
    planThread.join();
    camThread.join();
    detThread.join();
    cv::destroyWindow(WIN);
    return rc;
}
#endif  // SIM_HAVE_HIGHGUI

}  // namespace kdemo
