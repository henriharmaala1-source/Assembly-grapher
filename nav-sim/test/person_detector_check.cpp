// The person detector: its decoders against outputs with a KNOWN answer, and
// the model compiled into the binary.
//
//   decode   a synthetic YOLOX raw head and a synthetic YOLOv8 output, each
//            with one person and one non-person placed in a known grid cell:
//            exactly the person comes back, where it was put, in ORIGINAL
//            image pixels after the letterbox is undone.
//   embedded the built-in YOLOX-nano: the bytes are an ONNX file of the
//            expected size, it loads from memory, its output is recognised as
//            the YOLOX layout, and a blank image has nobody in it.
//   photo    (PERSON_TEST_IMAGE=path) run it on a real picture and print what
//            it finds. Not in CI: this tree ships no photographs of people.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "person_detector.hpp"

using namespace kdemo;

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

int main() {
    std::printf("decoding\n");
    {
        // YOLOX raw at 416: 52x52 (stride 8), 26x26, 13x13 = 3549 rows of 85.
        const int S = 416, N = 52 * 52 + 26 * 26 + 13 * 13;
        int sz[3] = {1, N, 85};
        cv::Mat out(3, sz, CV_32F, cv::Scalar(0));
        auto row = [&](int i) { return out.ptr<float>(0, i); };
        // A person in stride-16 cell (gx 10, gy 5): row 2704 + 5*26 + 10.
        // Offsets 0.5 -> centre (10.5, 5.5)*16 = (168, 88); log size -> 64x128.
        float* p = row(2704 + 5 * 26 + 10);
        p[0] = 0.5f; p[1] = 0.5f; p[2] = std::log(4.f); p[3] = std::log(8.f);
        p[4] = 0.9f; p[5] = 0.95f;                     // obj, class 0 (person)
        // A confident CAR elsewhere (class 2): must not come back.
        float* c = row(100);
        c[0] = 0.5f; c[1] = 0.5f; c[2] = 1.f; c[3] = 1.f; c[4] = 0.99f; c[7] = 0.99f;
        CHECK(detLayoutOf(out, S) == DetLayout::YoloxRaw);
        Letterbox lb; lb.scale = 0.5f;                 // an 832x832 image, top-left
        const auto ps = decodePersons(out, DetLayout::YoloxRaw, S, lb, cv::Size(832, 832));
        CHECK(ps.size() == 1);
        if (!ps.empty()) {
            // (168-32, 88-64) = (136, 24) in the network, /0.5 in the image.
            std::printf("  YOLOX: box %d,%d %dx%d score %.3f\n", ps[0].box.x, ps[0].box.y,
                        ps[0].box.width, ps[0].box.height, ps[0].score);
            CHECK(ps[0].box == cv::Rect(272, 48, 128, 256));
            CHECK(std::fabs(ps[0].score - 0.855f) < 1e-3f);
        }
    }
    {
        // YOLOv8 at 640: [1, 84, 8400], rows are cx cy w h then 80 scores.
        const int S = 640, N = 80 * 80 + 40 * 40 + 20 * 20;
        int sz[3] = {1, 84, N};
        cv::Mat out(3, sz, CV_32F, cv::Scalar(0));
        float* d = out.ptr<float>();
        auto set = [&](int r, int i, float v) { d[size_t(r) * N + size_t(i)] = v; };
        set(0, 7, 320.f); set(1, 7, 330.f); set(2, 7, 100.f); set(3, 7, 200.f); set(4, 7, 0.8f);
        set(0, 9, 100.f); set(1, 9, 100.f); set(2, 9, 50.f); set(3, 9, 50.f); set(5, 9, 0.9f);  // a bicycle
        CHECK(detLayoutOf(out, S) == DetLayout::YoloV8);
        Letterbox lb; lb.scale = 1.f; lb.padX = 0.f; lb.padY = 80.f;   // 640x480, centred
        const auto ps = decodePersons(out, DetLayout::YoloV8, S, lb, cv::Size(640, 480));
        CHECK(ps.size() == 1);
        if (!ps.empty()) {
            std::printf("  YOLOv8: box %d,%d %dx%d\n", ps[0].box.x, ps[0].box.y,
                        ps[0].box.width, ps[0].box.height);
            CHECK(ps[0].box == cv::Rect(270, 150, 100, 200));
        }
    }

    std::printf("the built-in model\n");
    {
        const size_t n = yoloxNanoSize();
        std::printf("  %zu bytes compiled in\n", n);
        CHECK(n == 3659407);                            // the released yolox_nano.onnx
        PersonDetector det;
        std::string note;
        CHECK(det.init("yolox", nullptr, note));
        std::printf("  init: %s (%s)\n", note.c_str(), det.kindName());
        CHECK(std::string(det.kindName()) == "YOLOX-nano");
        const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
        CHECK(det.detect(blank, cv::Mat()).empty());
        // A person box whose middle is 2 m away reads 2 m.
        cv::Mat depth(480, 640, CV_32F, cv::Scalar(6.f));
        depth(cv::Rect(300, 200, 60, 120)).setTo(2.f);
        CHECK(std::fabs(PersonDetector::rangeIn(depth, cv::Rect(290, 180, 80, 160),
                                                cv::Size(640, 480)) - 2.f) < 1e-6f);
        // HOG still there as the fallback.
        PersonDetector hog;
        std::string hn;
        hog.init("hog", nullptr, hn);
        std::printf("  hog: %s\n", hn.c_str());

        if (const char* img = std::getenv("PERSON_TEST_IMAGE")) {
            const cv::Mat im = cv::imread(img);
            CHECK(!im.empty());
            if (!im.empty()) {
                for (PersonDetector* d : {&det, &hog}) {
                    const auto t0 = cv::getTickCount();
                    const auto ps = d->detect(im, cv::Mat());
                    const double ms = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
                    std::printf("  %s on %s: %zu people, %.1f ms\n", d->kindName(), img,
                                ps.size(), ms);
                    for (const Person& q : ps)
                        std::printf("    %d,%d %dx%d  %.2f\n", q.box.x, q.box.y, q.box.width,
                                    q.box.height, q.score);
                }
            }
        }
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
