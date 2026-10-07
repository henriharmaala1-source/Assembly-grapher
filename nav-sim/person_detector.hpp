#pragma once
// ---------------------------------------------------------------------------
// PersonDetector -- people in an image, with their RANGE read off the depth.
//
// Three detectors, one interface:
//   yolox  YOLOX-nano (Megvii, Apache-2.0), COMPILED INTO THE BINARY: no model
//          file, no download, nothing to install. The default wherever this
//          build has OpenCV's dnn module. Finds partial, sitting, turned and
//          distant people; ~10 ms per frame on a 4-thread desktop CPU.
//   hog    HOG + linear SVM (Dalal & Triggs 2005), shipped inside OpenCV: the
//          fallback when there is no dnn. Upright, unoccluded, whole people.
//   FILE   any ONNX detector in one of the common layouts -- YOLOX (raw head,
//          as released), YOLOv5 (decoded), YOLOv8/YOLO11 (84 x N). Mind the
//          licence of what you load: Ultralytics' models are AGPL-3.0, which
//          reaches the whole program if it is distributed with them inside.
//
// RANGE COMES FROM THE DEPTH FRAME, not from the box size: the median valid
// depth in the middle half of the box. Distance from apparent height needs an
// assumed height; this stack has a calibrated depth image.
// ---------------------------------------------------------------------------

#include <functional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#if KESTREL_HAVE_DNN
#include <opencv2/dnn.hpp>
#endif
#if KESTREL_HAVE_OBJDETECT
#include <opencv2/objdetect.hpp>
#endif

namespace kdemo {

struct Person {
    cv::Rect box;
    float    rangeM = -1.f;    // < 0 means the depth frame had nothing there
    float    score = 0.f;
};

// How an ONNX detector lays out its output, told apart by shape alone.
enum class DetLayout { Unknown, YoloxRaw, YoloV5, YoloV8 };
DetLayout detLayoutOf(const cv::Mat& out, int inputSize);
const char* detLayoutName(DetLayout l);

// The letterbox that put the image into the network's square input.
struct Letterbox { float scale = 1.f, padX = 0.f, padY = 0.f; };

// COCO class 0 ("person") boxes from one forward pass, in ORIGINAL image
// pixels, after non-maximum suppression. Exposed for the tests.
std::vector<Person> decodePersons(const cv::Mat& out, DetLayout layout, int inputSize,
                                  const Letterbox& lb, cv::Size imgSize,
                                  float scoreThr = 0.35f, float nmsThr = 0.45f);

// The built-in model's bytes (embedded_yolox.cpp, generated at build time);
// size 0 when this build has none.
const unsigned char* yoloxNanoBytes();
size_t               yoloxNanoSize();

class PersonDetector {
public:
    // Puts a loaded net on a backend and says which (the demo's CUDA policy).
    using BackendFn = std::function<std::string(void* net, bool* usedCuda)>;

    // spec: "" or "yolox" (built in), "hog", or a path to an .onnx file.
    // Falls back -- yolox -> hog -- and says why in `note`.
    bool init(const std::string& spec, const BackendFn& backend, std::string& note);

    bool available() const { return kind_ != NONE; }
    const std::string& backend() const { return backend_; }
    bool onCuda() const { return cuda_; }
    const char* kindName() const;

    // `bgr` is what the camera saw; `depthM` may be empty, in which case the
    // boxes come back without a range rather than with a guessed one.
    std::vector<Person> detect(const cv::Mat& bgr, const cv::Mat& depthM);

    // Median depth in the middle half of `box` (image pixels), or -1.
    static float rangeIn(const cv::Mat& depthM, const cv::Rect& box, const cv::Size& imgSize);

private:
    enum Kind { NONE = 0, HOG, ONNX } kind_ = NONE;
    bool        builtin_ = false;
    std::string backend_ = "cpu";
    bool        cuda_ = false;
    int         inputSize_ = 416;
    DetLayout   layout_ = DetLayout::Unknown;
#if KESTREL_HAVE_OBJDETECT
    cv::HOGDescriptor hog_;
#endif
#if KESTREL_HAVE_DNN
    cv::dnn::Net net_;
    bool probe(int size);
#endif
    bool initHog(std::string& note);
};

}  // namespace kdemo
