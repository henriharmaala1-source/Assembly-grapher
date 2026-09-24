// The demo. See kestrel_demo.hpp for what it shows and why those four things.
#include "kestrel_demo.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
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
#include "nav_pipeline.hpp"
#include "visual_pose.hpp"
#include "rl_env.hpp"
#include "voxel_map.hpp"

namespace kdemo {
namespace {

// ------------------------------------------------------------------ painting
// The launcher's palette (kestrel_gui.cpp), so the two windows read as one app.
const cv::Scalar INK{242, 240, 238}, DIM{160, 152, 146}, EDGE{84, 77, 70};
const cv::Scalar BG{34, 30, 27}, WARN{60, 160, 250}, OK{120, 210, 120};
const cv::Scalar CARD{44, 39, 35}, HEADER{44, 39, 35}, ACCENT{214, 146, 60};

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
    cv::rectangle(canvas, r, CARD, cv::FILLED);                 // a card
    const cv::Rect img(r.x, r.y, r.width, r.height - CAPTION_H);
    letterbox(p.img, img.width, img.height).copyTo(canvas(img));
    cv::rectangle(canvas, {r.x, r.y + r.height - CAPTION_H, 3, CAPTION_H}, ACCENT,
                  cv::FILLED);                                  // caption marker
    cv::rectangle(canvas, r, EDGE, 1);
    txt(canvas, fit(p.title, r.width - 20, 0.56),
        r.x + 10, r.y + r.height - 26, 0.56, INK, 1);
    txt(canvas, fit(p.sub, r.width - 20, 0.42),
        r.x + 10, r.y + r.height - 8, 0.42, p.subColour, 1);
}

// The window's header bar: the app mark, the title, and what the input is.
void drawHeader(cv::Mat& canvas, const std::string& title, const std::string& note) {
    cv::rectangle(canvas, {0, 0, canvas.cols, 38}, HEADER, cv::FILLED);
    cv::line(canvas, {0, 38}, {canvas.cols, 38}, EDGE, 1);
    cv::rectangle(canvas, {10, 8, 22, 22}, ACCENT, cv::FILLED);
    const std::vector<cv::Point> k{{16, 12}, {16, 26}, {20, 22}, {27, 26}, {22, 19}, {27, 12}};
    cv::polylines(canvas, k, false, INK, 2, cv::LINE_AA);
    cv::putText(canvas, title, {40, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.62, INK, 2, cv::LINE_AA);
    int base = 0;
    const int tw = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, 0.62, 2, &base).width;
    cv::putText(canvas, note, {52 + tw, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.44, DIM, 1,
                cv::LINE_AA);
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

// THE SIM PANE: FPV footage of the simulated scene, with the map the
// aircraft is building inset beside it -- first-person, then from above.
// Footage alone would show a scene the aircraft cannot see; the map alone is
// mostly fog, correctly, and a viewer cannot tell fog from failure. Together
// the gap between them IS the demonstration: what is there, and how much of
// it the aircraft has actually measured.
struct SimInsets { cv::Rect fpv, top; };
SimInsets simInsets(int iw, int ih) {
    const int m = std::max(4, iw / 80);
    const int W = iw * 36 / 100, H = W * 3 / 4;
    const int S = std::min(H, ih - H - 3 * m);          // top-down is square
    SimInsets r;
    r.fpv = cv::Rect(iw - W - m, m, W, H);
    r.top = cv::Rect(iw - S - m, m + H + m, S, S);
    return r;
}
cv::Mat composeSim(const cv::Mat& footage, const cv::Mat& belief,
                   const cv::Mat& topDown, int iw, int ih) {
    cv::Mat out = footage.empty() ? cv::Mat(ih, iw, CV_8UC3, BG)
                                  : letterbox(footage, iw, ih);
    const SimInsets r = simInsets(iw, ih);
    auto inset = [&](const cv::Mat& src, const cv::Rect& at, const char* label) {
        if (!src.empty()) letterbox(src, at.width, at.height).copyTo(out(at));
        cv::rectangle(out, at, INK, 1);
        txt(out, label, at.x + 4, at.y + at.height - 5, 0.36, INK, 1);
    };
    inset(belief, r.fpv, "MAP: what it believes");
    inset(topDown, r.top, "MAP from above");
    // Never mistakable for a sensor: this is the true scene, rendered.
    txt(out, "SIMULATED SCENE (truth)", 8, 18, 0.42, INK, 1);
    return out;
}

// A SIMULATED D435i on the simulated aircraft, for when no camera is plugged
// in. It runs the LIVE path unchanged -- stereo depth into navcore's
// NavPipeline (the aircraft's own), both tiers in first person -- so the two live
// panes show what --live will show, instead of repeating the SIM pane's own
// belief view. It keeps its own copy of the world (same name, same seed, so the
// same geometry) because the planner thread rebuilds its world on reset.
//
// 424x240: a real D435i depth mode, and what a laptop can stereo-match at a
// usable rate here (45 ms; 848x480 is 194 ms). Its honest marking range is
// shorter, ~2.5 m against ~3.5 m, and the caption says which camera it is.
//
// MOUNTED 20 DEG DOWN, like the sim's camera (voxel_live.cpp says why). Level,
// at the forest's 1.5 m, the first floor this camera sees is 2.7 m ahead and
// its map reaches 2.5 m -- so the floor never registered at all. Tilted, the
// floor is in view from 1.3 m and marked out to its reach.
constexpr float kSimEyeTiltDeg = -20.f;

// THE DEPTH PANE IS ON AN ABSOLUTE SCALE: red is 0 m, blue is 8 m and beyond,
// grey is no return. It was histogram-equalised, which makes red mean "the
// nearest thing in THIS frame" -- so floor 5-6 m away, well past the map's
// reach, painted the bottom of the pane red beside a voxel pane with nothing
// there, and read as a close-range blind spot in the map. Measured at that
// pose: every one of those returns was 5-6.4 m out. Equalising suits a
// scene you are inspecting; beside a map with a fixed reach, the colour has
// to mean a distance.
constexpr float kDepthScaleM = 8.f;
class SimEye {
public:
    explicit SimEye(const sim::EnvConfig& cfg) : cfg_(cfg) {
        cp_.width = 424; cp_.height = 240; cp_.hfovDeg = 87.f; cp_.baselineM = 0.05f;
    }
    // One frame from `pose` in world `seed`. Returns false if nothing drawn.
    bool frame(const sim::CamPose& aircraft, unsigned seed, int outW, int outH,
               cv::Mat& depthVis, cv::Mat& mapFpv, float& validFrac) {
        sim::CamPose pose = aircraft;
        pose.pitchDeg += kSimEyeTiltDeg;      // the camera's mount, not the airframe
        if (!twin_ || seed != seed_) {
            sim::EnvConfig c = cfg_; c.seed = seed;
            twin_.reset(new sim::VoxelEnv(c));
            cam_.reset(new sim::SimFrameSource(twin_->world(), cp_, /*truth*/ false));
            nav_.init(cam_->camera(), sim::NavPipelineParams(), pose);
            seed_ = seed;
        }
        cam_->setPose(pose);
        cv::Mat depth; sim::PoseHint hint;
        if (!cam_->next(depth, hint) || depth.empty()) return false;
        // The sim KNOWS its pose, so this map follows the aircraft (step()
        // recentres on it); the real camera's cannot, and holds a fixed pose.
        nav_.step(depth, pose);
        int valid = 0;
        for (int y = 0; y < depth.rows; ++y) {
            const float* r = depth.ptr<float>(y);
            for (int x = 0; x < depth.cols; ++x) valid += (r[x] > 0.f);
        }
        validFrac = float(valid) / std::max(1, depth.rows * depth.cols);
        // ABSOLUTE colour, not equalised -- see kDepthScaleM.
        depthVis = letterbox(sim::colourDepth(depth, kDepthScaleM), outW, outH);
        // BOTH TIERS, as voxel_live draws them: the fine map to its honest
        // range and the bearing field beyond. The fine map alone is fog past
        // 2.5-3.5 m, which from flying height is nearly the whole frame.
        mapFpv = letterbox(nav_.renderFpv(pose, 320, 180, cp_.hfovDeg), outW, outH);
        return true;
    }
private:
    sim::EnvConfig cfg_;
    sim::CamParams cp_;
    std::unique_ptr<sim::VoxelEnv> twin_;
    std::unique_ptr<sim::SimFrameSource> cam_;
    sim::NavPipeline nav_;
    unsigned seed_ = ~0u;
};

struct PlannerFrame {
    cv::Mat fpv, depth, top, footage;
    sim::CamPose pose;           // where the aircraft is, for the sim D435i
    unsigned     seed = 0;       // which world it is flying
    float   travelM = 0, netM = 0;
    int     steps = 0, cells = 0, collisions = 0;
};
struct CameraFrame {
    cv::Mat depthVis, mapFpv, colour;
    cv::Mat depthRaw;            // CV_32F metres, for the detector's range
    float   validFrac = 0;
    std::string poseNote;        // what placed the map, for the pane caption
    bool    poseOk = true;
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
        else if (a == "--pose") {
            o.pose = next("--pose");
            sim::PoseMode m;
            if (err.empty() && !sim::parsePoseMode(o.pose, m))
                err = "--pose takes fixed, vio or slam (got '" + o.pose + "')";
        }
        else if (a == "--slam-socket") o.slamSocket = next("--slam-socket");
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
    // A MISTYPED WORLD IS SILENTLY A FOREST. VoxelEnv::reset falls through to
    // forest for anything it does not recognise, which is right for the
    // environment -- it must always build something -- and wrong here, where
    // `--world galery` would quietly show the exact thing gallery exists to
    // avoid and nothing would say so.
    static const char* WORLDS[] = {"gallery", "hall", "forest", "maze", "city",
                                   "road", "culdesac", "corridor"};
    bool knownWorld = false;
    for (const char* wn : WORLDS) if (o.world == wn) knownWorld = true;
    if (!knownWorld) {
        err = "--world: no such world: " + o.world + ". One of:";
        for (const char* wn : WORLDS) err += std::string(" ") + wn;
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
    // THE SIM PANE'S INSETS must sit inside its image, clear of each other
    // and of the caption burned into the footage's top-left corner.
    for (int pw : {320, 480, 640}) {
        const int iw = pw, ih = pw * 3 / 4;
        const SimInsets r = simInsets(iw, ih);
        const cv::Rect img(0, 0, iw, ih), caption(0, 0, iw / 2, 26);
        if ((r.fpv & img) != r.fpv || (r.top & img) != r.top)
            fail(cv::format("sim inset off its pane at paneW=%d", pw));
        if ((r.fpv & r.top).area() > 0)
            fail(cv::format("sim insets overlap at paneW=%d", pw));
        if ((r.fpv & caption).area() > 0 || (r.top & caption).area() > 0)
            fail(cv::format("sim inset covers the SIMULATED caption at paneW=%d", pw));
        if (r.top.width < 60)
            fail(cv::format("top-down inset too small to read at paneW=%d", pw));
    }
    const char* caps[] = {"SIM DEMONSTRATION", "LIVE DEPTH", "LIVE VOXEL -- first person",
                          "HUMANS", "flying freeM (classical)",
                          "learned policy (onnx)", "no detector in this build"};
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
    // THE SHOT FLIES WHAT THE DEMO WOULD FLY. Using the fallback here
    // regardless would mean --shot could never show the one thing the demo
    // exists for, and an exported policy would have no path through this
    // binary that runs without a display -- so nothing headless could ever
    // check that the .onnx loads, masks and steers.
    Policy pol;
    std::string polNote;
    const bool learned = pol.loadOnnx(o.model, o.cuda, polNote);
    // FAR ENOUGH TO HAVE A MAP. At 60 steps both the FPV and the plan view are
    // almost entirely UNKNOWN -- which is correct, and reads as a broken
    // renderer to anyone who has not been told that pale means fog. The shot is
    // a picture of the layout, so it shows the layout with something in it.
    for (int i = 0; i < 500; ++i) {
        const sim::EnvStep st = env.step(
            pol.act(env.observation(), env.actionMask(), env.nPrims(), fb, rng));
        if (st.done || st.truncated) break;
    }
    auto toMat = [&](std::vector<uint8_t> v, int w, int h) {
        return v.size() == size_t(w) * h * 3
                   ? cv::Mat(h, w, CV_8UC3, v.data()).clone()
                   : cv::Mat(h, w, CV_8UC3, BG);
    };
    Pane p[4];
    p[0].title = "SIM DEMONSTRATION";
    p[0].sub   = learned ? polNote
                         : std::string("flying ") + sim::baselineName(fb) + " (classical)";
    p[0].subColour = learned ? OK : DIM;
    p[0].img   = composeSim(toMat(env.renderFootage(iw, ih), iw, ih),
                            toMat(env.renderFrame(iw, ih, false), iw, ih),
                            toMat(env.renderFrame(ih, ih, true), ih, ih), iw, ih);

    // THE LIVE PANES FROM A SIMULATED D435i on the same aircraft, so the shot
    // shows the live path -- depth, map, and the map in FIRST PERSON -- rather
    // than a stand-in. A short yaw sweep first, so the map has more than one
    // frame's worth in it.
    const cv::Mat dRaw = syntheticDepth(iw, ih);
    {
        SimEye eye(cfg);
        cv::Mat dv, mf; float vf = 0.f;
        const sim::CamPose at = env.pose();
        for (int k = -3; k <= 3; ++k) {
            sim::CamPose q = at; q.yawDeg = at.yawDeg + 8.f * float(k);
            eye.frame(q, cfg.seed, iw, ih, dv, mf, vf);
        }
        eye.frame(at, cfg.seed, iw, ih, dv, mf, vf);
        p[1].img = dv; p[2].img = mf;
        p[1].sub = cv::format("simulated D435i 424x240, red 0 m .. blue 8 m   %.0f%% returned",
                              vf * 100.f);
    }
    p[1].title = "LIVE DEPTH";
    p[1].subColour = WARN;
    p[2].title = "LIVE VOXEL -- first person";
    p[2].sub   = "sim D435i: voxels to 2.5 m, far tier beyond; grey is UNKNOWN";
    p[2].subColour = WARN;

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
    drawHeader(canvas, "kestrel demo", "synthetic input -- nothing measured here");
    for (int i = 0; i < 4; ++i) drawPane(canvas, L.pane[i], p[i]);
    txt(canvas, "source: shot (no device)   planner: "
                + std::string(learned ? pol.backend() : "classical")
                + "   people: " + det.kindName(),
        L.strip.x + 4, L.strip.y + 18, 0.44, DIM, 1);

    int n = 0;
    const char* names[4] = {"sim", "depth", "voxel", "humans"};
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
        learned ? polNote : std::string("flying ") + sim::baselineName(fb) + " (classical)";
    if (!learned && !o.model.empty())
        std::printf("[demo] %s\n       flying %s instead, and the pane says so.\n",
                    polNote.c_str(), sim::baselineName(fb));

    Slot<PlannerFrame> planSlot;
    std::thread planThread([&] {
        sim::VoxelEnv env(cfg);
        unsigned rng = o.seed * 7919u + 13u;
        unsigned seedNow = cfg.seed;
        while (!stop.load()) {
            const sim::EnvStep st = env.step(
                pol.act(env.observation(), env.actionMask(), env.nPrims(), fb, rng));
            PlannerFrame f;
            f.fpv = matFrom(env.renderFrame(iw, ih, false), iw, ih);
            f.depth = matFrom(env.renderDepth(iw, ih), iw, ih);
            f.top = matFrom(env.renderFrame(ih, ih, true), ih, ih);
            f.footage = matFrom(env.renderFootage(iw, ih), iw, ih);
            f.pose = env.pose();
            f.seed = seedNow;
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
                seedNow = rng;
            }
        }
    });

    // --- the camera stage ----------------------------------------------------
    // SIM and REPLAY exist so this runs with no hardware. LIVE is the D435i.
    std::string srcNote;
    std::unique_ptr<sim::FrameSource> src;
    if (o.source == Options::LIVE) {
        std::string err;
        sim::PoseMode pm = sim::PoseMode::Fixed;
        sim::parsePoseMode(o.pose, pm);
        const bool track = pm != sim::PoseMode::Fixed;
        src = sim::makeLiveSource(o.camW, o.camH, o.camFps, o.emitter, &err,
                                  o.emitter && track, pm == sim::PoseMode::Slam);
        srcNote = src ? "RealSense" : ("no live camera: " + err);
    } else if (o.source == Options::REPLAY) {
        auto r = std::make_unique<sim::ReplayFrameSource>();
        std::string err;
        if (r->open(o.replayPath, &err)) { srcNote = "replay " + o.replayPath; src = std::move(r); }
        else srcNote = "replay failed: " + err;
    } else {
        srcNote = "simulated D435i 424x240 on the sim aircraft";
    }

    Slot<CameraFrame> camSlot;
    std::thread camThread([&] {
        SimEye simEye(cfg);
        sim::NavPipeline nav;
        bool inited = false;
        // THE POSE SOURCE for the live pane (VisualPose, as the aircraft).
        sim::VisualPose visual;
        sim::PoseMode poseMode = sim::PoseMode::Fixed;
        sim::parsePoseMode(o.pose, poseMode);
        if (src && src->ok()) {
            sim::VisualPoseParams vp;
            vp.mode = poseMode;
            vp.emitter = o.emitter ? sim::EmitterMode::Strobe : sim::EmitterMode::Off;
            vp.slamSocket = o.slamSocket;
            vp.fps = float(o.camFps);
            visual.init(*src, vp);
        }
        sim::CamPose origin, lastEst;
        const auto t0 = std::chrono::steady_clock::now();
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
                sim::CamPose pose = hint.valid ? hint.pose : sim::CamPose{};
                if (poseMode != sim::PoseMode::Fixed) {
                    // Attitude from the camera's IMU; POSITION (and heading)
                    // from the tracker, relative to where the map was begun.
                    const double tNow = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0).count();
                    const sim::PoseEstimate& ve = visual.step(*src, depth, pose,
                                                              pose.yawDeg, tNow);
                    if (ve.valid) {
                        pose.e = origin.e + ve.e; pose.n = origin.n + ve.n;
                        pose.u = origin.u + ve.u; pose.yawDeg = ve.yawDeg;
                        lastEst = pose;
                    } else if (inited) {
                        // No pose this frame: HOLD the last one (the map does
                        // not jump back to where it began).
                        pose.e = lastEst.e; pose.n = lastEst.n; pose.u = lastEst.u;
                    }
                    f.poseNote = std::string("pose ") + sim::poseModeName(poseMode) + ": " +
                                 ve.status + cv::format("   lost %d  resets %d", ve.lost,
                                                        ve.resets);
                    f.poseOk = ve.valid;
                } else {
                    f.poseNote = "pose FIXED: move the camera and the map smears";
                    f.poseOk = true;
                }
                // THE AIRCRAFT'S PIPELINE, not a default map: navcore's
                // NavPipeline, as onboard runs it -- the map configured from
                // THIS camera (it used bare VoxelMapParams: marking to 8 m with
                // no stereo noise model, a more confident map than the one that
                // flies), plus the bearing field. Fixed pose, as above.
                if (!inited) {
                    nav.init(src->camera(), sim::NavPipelineParams(), pose);
                    origin = pose; lastEst = pose;
                    inited = true;
                }
                nav.step(depth, pose);
                int valid = 0;
                for (int y = 0; y < depth.rows; ++y) {
                    const float* r = depth.ptr<float>(y);
                    for (int x = 0; x < depth.cols; ++x) valid += (r[x] > 0.f);
                }
                f.validFrac = float(valid) / std::max(1, depth.rows * depth.cols);
                f.depthRaw = depth.clone();
                f.depthVis = sim::colourDepth(depth, kDepthScaleM);
                // THE IMAGE THE DEPTH WAS MEASURED IN, when the source has
                // one. Registered with the depth by construction, so a box
                // found here indexes straight into f.depthRaw -- which is what
                // makes "person at 2.3 m" a measurement rather than an
                // estimate from apparent height. See FrameSource::intensity.
                cv::Mat ir;
                if (src->intensity(ir) && !ir.empty())
                    cv::cvtColor(ir, f.colour, cv::COLOR_GRAY2BGR);
                const float hf = src->camera().params().hfovDeg;
                f.mapFpv = letterbox(nav.renderFpv(pose, 320, 320 * depth.rows
                                                   / std::max(1, depth.cols), hf),
                                     iw, ih);
            } else {
                // No camera: mirror the planner's own sensor, which is the
                // honest thing to show -- it IS simulated stereo from a real
                // VoxelMap, just not from a real room.
                PlannerFrame pf;
                unsigned long seen = 0;
                if (planSlot.take(pf, seen) && !pf.fpv.empty())
                    simEye.frame(pf.pose, pf.seed, iw, ih, f.depthVis, f.mapFpv,
                                 f.validFrac);
                else
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
        drawHeader(canvas, "kestrel demo",
                   o.source == Options::LIVE ? "live D435i" :
                   o.source == Options::REPLAY ? "replay" : "simulated input");

        Pane p[4];
        p[0].title = "SIM DEMONSTRATION";
        p[0].sub = polLabel;
        p[0].subColour = learned ? OK : DIM;
        p[0].img = composeSim(pf.footage, pf.fpv, pf.top, iw, ih);
        p[1].title = "LIVE DEPTH";
        p[1].sub = srcNote + ", red 0 m .. blue 8 m" + (cf.validFrac > 0.f
                       ? cv::format("   %.0f%% of pixels returned", cf.validFrac * 100.f)
                       : std::string());
        p[1].subColour = (o.source == Options::LIVE && src) ? DIM : WARN;
        p[1].img = cf.depthVis.empty() ? pf.depth : cf.depthVis;
        p[2].title = "LIVE VOXEL -- first person";
        p[2].sub = (src && src->ok())
                   ? cf.poseNote
                   : std::string("sim D435i: voxels to the map's reach, far tier beyond; "
                                 "grey is UNKNOWN");
        p[2].subColour = (src && src->ok() && cf.poseOk) ? DIM : WARN;
        p[2].img = cf.mapFpv;
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
