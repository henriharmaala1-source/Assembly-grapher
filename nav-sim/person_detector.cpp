#include "person_detector.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace kdemo {

namespace {
// Anchor-free grid cells for strides 8/16/32 at a square input.
int gridCells(int s) { return (s / 8) * (s / 8) + (s / 16) * (s / 16) + (s / 32) * (s / 32); }
}  // namespace

#if !KESTREL_EMBED_YOLOX
// No dnn, no built-in model: the detector falls back to HOG and says so.
const unsigned char* yoloxNanoBytes() { return nullptr; }
size_t yoloxNanoSize() { return 0; }
#endif

const char* detLayoutName(DetLayout l) {
    switch (l) {
        case DetLayout::YoloxRaw: return "YOLOX";
        case DetLayout::YoloV5:   return "YOLOv5";
        case DetLayout::YoloV8:   return "YOLOv8";
        default:                  return "unknown";
    }
}

DetLayout detLayoutOf(const cv::Mat& out, int s) {
    if (out.dims != 3 || out.size[0] != 1) return DetLayout::Unknown;
    const int a = out.size[1], b = out.size[2];
    // [1, N, 5 + C]: YOLOX leaves its head RAW (one row per grid cell);
    // YOLOv5 exports decoded rows, three anchors per cell.
    if (b >= 6 && a == gridCells(s))     return DetLayout::YoloxRaw;
    if (b >= 6 && a == 3 * gridCells(s)) return DetLayout::YoloV5;
    // [1, 4 + C, N]: YOLOv8 / YOLO11, anchor-free, no objectness.
    if (a >= 5 && a < b && b == gridCells(s)) return DetLayout::YoloV8;
    return DetLayout::Unknown;
}

std::vector<Person> decodePersons(const cv::Mat& out, DetLayout layout, int s,
                                  const Letterbox& lb, cv::Size img,
                                  float scoreThr, float nmsThr) {
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    auto keep = [&](float cx, float cy, float w, float h, float score) {
        if (!(score >= scoreThr)) return;
        // Network pixels -> original pixels: undo the letterbox.
        const float x0 = (cx - 0.5f * w - lb.padX) / lb.scale;
        const float y0 = (cy - 0.5f * h - lb.padY) / lb.scale;
        cv::Rect r(int(std::lround(x0)), int(std::lround(y0)),
                   int(std::lround(w / lb.scale)), int(std::lround(h / lb.scale)));
        r &= cv::Rect(0, 0, img.width, img.height);
        if (r.width < 2 || r.height < 2) return;
        boxes.push_back(r);
        scores.push_back(score);
    };
    if (layout == DetLayout::YoloxRaw || layout == DetLayout::YoloV5) {
        const int n = out.size[1], stride = out.size[2];
        const float* p = out.ptr<float>();
        int level = 0, g = s / 8, inLevel = 0;
        for (int i = 0; i < n; ++i, p += stride) {
            float cx = p[0], cy = p[1], w = p[2], h = p[3];
            if (layout == DetLayout::YoloxRaw) {
                // The raw head: offsets in grid cells, log sizes, per stride.
                const int st = 8 << level;
                const int gx = inLevel % g, gy = inLevel / g;
                cx = (cx + float(gx)) * float(st);
                cy = (cy + float(gy)) * float(st);
                w = std::exp(w) * float(st);
                h = std::exp(h) * float(st);
                if (++inLevel == g * g) { inLevel = 0; ++level; g /= 2; }
            }
            // obj x class-0; both already through a sigmoid in these exports.
            keep(cx, cy, w, h, p[4] * p[5]);
        }
    } else if (layout == DetLayout::YoloV8) {
        const int n = out.size[2];
        const float* p = out.ptr<float>();
        for (int i = 0; i < n; ++i)
            keep(p[0 * n + i], p[1 * n + i], p[2 * n + i], p[3 * n + i], p[4 * n + i]);
    }
    std::vector<int> idx;
#if KESTREL_HAVE_DNN
    cv::dnn::NMSBoxes(boxes, scores, scoreThr, nmsThr, idx);
#else
    for (size_t i = 0; i < boxes.size(); ++i) idx.push_back(int(i));
#endif
    std::vector<Person> ps;
    for (int i : idx) { Person q; q.box = boxes[size_t(i)]; q.score = scores[size_t(i)]; ps.push_back(q); }
    return ps;
}

const char* PersonDetector::kindName() const {
    if (kind_ == ONNX) return builtin_ ? "YOLOX-nano" : detLayoutName(layout_);
    return kind_ == HOG ? "HOG" : "none";
}

bool PersonDetector::initHog(std::string& note) {
#if KESTREL_HAVE_OBJDETECT
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    kind_ = HOG;
    // HOG has no GPU path in mainline OpenCV (cv::cuda::HOG is contrib).
    backend_ = "cpu (HOG has no cuda path in this build)";
    if (note.empty()) note = "HOG + linear SVM, upright people, no model file";
    return true;
#else
    kind_ = NONE;
    if (note.empty()) note = "this build has no opencv objdetect -- no detector";
    return false;
#endif
}

#if KESTREL_HAVE_DNN
// One forward pass on a blank input at `size`: does the net take it, and what
// does it put out? That is how the layout -- and so the pre-processing -- is
// found for a file nobody described.
bool PersonDetector::probe(int size) {
    try {
        const cv::Mat blank(size, size, CV_8UC3, cv::Scalar(114, 114, 114));
        net_.setInput(cv::dnn::blobFromImage(blank, 1.0, cv::Size(size, size)));
        const cv::Mat out = net_.forward();
        const DetLayout l = detLayoutOf(out, size);
        if (l == DetLayout::Unknown) return false;
        inputSize_ = size;
        layout_ = l;
        return true;
    } catch (const cv::Exception&) {
        return false;
    }
}
#endif

bool PersonDetector::init(const std::string& spec, const BackendFn& backend,
                          std::string& note) {
    note.clear();
    if (spec == "hog") return initHog(note);
#if KESTREL_HAVE_DNN
    const bool builtin = spec.empty() || spec == "yolox";
    try {
        if (builtin) {
            if (yoloxNanoSize() == 0) {
                note = "no built-in YOLOX in this build; using HOG";
                return initHog(note);
            }
            net_ = cv::dnn::readNetFromONNX(reinterpret_cast<const char*>(yoloxNanoBytes()),
                                            yoloxNanoSize());
        } else {
            net_ = cv::dnn::readNet(spec);
        }
    } catch (const cv::Exception& e) {
        note = std::string("detector failed to load, using HOG: ") + e.what();
        return initHog(note);
    }
    // THE INPUT SIZE, found rather than assumed: YOLOX nano/tiny take 416,
    // most exports 640. A layout nobody recognises is refused, not guessed at
    // -- the old --detector path loaded a net and then never ran it.
    if (net_.empty() || !(builtin ? probe(416) : (probe(640) || probe(416) || probe(320)))) {
        note = builtin ? "built-in YOLOX did not run; using HOG"
                       : "unrecognised detector output layout (want YOLOX, YOLOv5 or "
                         "YOLOv8/11); using HOG";
        return initHog(note);
    }
    kind_ = ONNX;
    builtin_ = builtin;
    backend_ = backend ? backend(&net_, &cuda_) : "cpu";
    note = std::string(builtin ? "YOLOX-nano (built in, Apache-2.0)" : detLayoutName(layout_)) +
           " on " + backend_;
    return true;
#else
    if (!spec.empty() && spec != "yolox") note = "this build has no opencv dnn; using HOG";
    else note = "this build has no opencv dnn, so no YOLOX; using HOG";
    (void)backend;
    return initHog(note);
#endif
}

std::vector<Person> PersonDetector::detect(const cv::Mat& bgr, const cv::Mat& depthM) {
    std::vector<Person> out;
    if (kind_ == NONE || bgr.empty()) return out;
#if KESTREL_HAVE_DNN
    if (kind_ == ONNX) {
        // LETTERBOX into the square input, pad grey 114, as every one of
        // these families was trained. YOLOX: top-left, BGR, 0..255.
        // Ultralytics: centred, RGB, 0..1.
        const int s = inputSize_;
        const bool ultra = layout_ != DetLayout::YoloxRaw;
        Letterbox lb;
        lb.scale = std::min(float(s) / bgr.cols, float(s) / bgr.rows);
        const int w = std::max(1, int(bgr.cols * lb.scale)), h = std::max(1, int(bgr.rows * lb.scale));
        if (ultra) { lb.padX = float((s - w) / 2); lb.padY = float((s - h) / 2); }
        cv::Mat canvas(s, s, CV_8UC3, cv::Scalar(114, 114, 114)), small;
        cv::resize(bgr, small, {w, h}, 0, 0, cv::INTER_LINEAR);
        small.copyTo(canvas(cv::Rect(int(lb.padX), int(lb.padY), w, h)));
        net_.setInput(cv::dnn::blobFromImage(canvas, ultra ? 1.0 / 255.0 : 1.0, cv::Size(s, s),
                                             cv::Scalar(), /*swapRB=*/ultra));
        const cv::Mat o = net_.forward();
        out = decodePersons(o, layout_, s, lb, bgr.size());
    }
#endif
#if KESTREL_HAVE_OBJDETECT
    if (kind_ == HOG) {
        // DOWNSCALE FIRST: HOG at 640 wide is ~120 ms on a laptop core, at
        // 320 ~35 ms, and finds the same people at the ranges a demo sees.
        cv::Mat small;
        const double sc = 320.0 / std::max(1, bgr.cols);
        cv::resize(bgr, small, {}, sc, sc, cv::INTER_AREA);
        std::vector<cv::Rect> found;
        std::vector<double> weights;
        hog_.detectMultiScale(small, found, weights, 0.0, cv::Size(8, 8),
                              cv::Size(0, 0), 1.05, 2.0, false);
        for (size_t i = 0; i < found.size(); ++i) {
            Person p;
            p.box = cv::Rect(int(found[i].x / sc), int(found[i].y / sc),
                             int(found[i].width / sc), int(found[i].height / sc));
            p.score = float(weights[i]);
            out.push_back(p);
        }
    }
#endif
    for (Person& p : out) p.rangeM = rangeIn(depthM, p.box, bgr.size());
    return out;
}

// MEDIAN, not mean. A box around a person also contains whatever is behind
// them, and a mean of "person at 2 m" and "wall at 8 m" describes nothing in
// the scene. The median of the middle of the box is the person whenever the
// person fills most of it, which is what a box that fired on a person is.
float PersonDetector::rangeIn(const cv::Mat& depthM, const cv::Rect& box,
                              const cv::Size& imgSize) {
    if (depthM.empty() || depthM.type() != CV_32F) return -1.f;
    const double sx = double(depthM.cols) / std::max(1, imgSize.width);
    const double sy = double(depthM.rows) / std::max(1, imgSize.height);
    cv::Rect r(int(box.x * sx), int(box.y * sy), int(box.width * sx), int(box.height * sy));
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
    std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
    return v[v.size() / 2];
}

}  // namespace kdemo
