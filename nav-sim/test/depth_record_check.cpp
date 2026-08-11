// .kdr round trip, and the failure modes that matter in the field.
//
// The format's whole justification is that both ends agree without a
// specification anyone has to read, so the checks are: C++ writes what C++
// reads, a TRUNCATED file (cable pulled mid-capture) stays readable, a file
// that is not a recording is refused rather than misread, and the replay source
// rebuilds a camera whose rays match the intrinsics that were recorded.
//
// The other half -- Python writes, C++ reads -- lives in
// test/kdr_crosscheck.py, because a round trip inside one language proves only
// that it is self-consistent.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "depth_record.hpp"
#include "frame_source.hpp"

using namespace sim;

static int fails = 0;
static void check(bool ok, const char* what, const std::string& detail = "") {
    std::printf("  %-58s %s%s%s\n", what, ok ? "ok" : "FAIL",
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++fails;
}

int main() {
    std::printf("depth recording (.kdr) checks\n");
    const std::string path = "/tmp/kdr_test.kdr";
    const int W = 64, H = 48, N = 7;

    DepthRecordHeader h;
    h.width = W; h.height = H;
    h.depthScale = 0.001f;
    h.fx = 425.3f; h.fy = 424.9f; h.ppx = 31.2f; h.ppy = 24.7f;
    h.baselineM = 0.0499f;
    h.flags = DepthRecordHeader::FLAG_EMITTER_ON;

    // --- write ------------------------------------------------------------
    {
        DepthRecordWriter w;
        std::string err;
        check(w.open(path, h, &err), "writer opens", err);
        std::vector<uint16_t> f(size_t(W) * H);
        for (int i = 0; i < N; ++i) {
            for (int k = 0; k < W * H; ++k)
                f[size_t(k)] = uint16_t((k + i * 13) % 60000);
            f[0] = 0;                                  // an invalid pixel
            check(w.writeFrame(f.data()) || i > 0, i == 0 ? "writes a frame" : "");
        }
        check(w.close(), "close patches the frame count");
    }

    // --- read back --------------------------------------------------------
    {
        DepthRecordReader r;
        std::string err;
        check(r.open(path, &err), "reader opens", err);
        check(r.frameCount() == N, "frame count round-trips",
              std::to_string(r.frameCount()));
        const DepthRecordHeader& g = r.header();
        check(g.width == uint32_t(W) && g.height == uint32_t(H), "size round-trips");
        check(std::fabs(g.fx - 425.3f) < 1e-4f && std::fabs(g.fy - 424.9f) < 1e-4f
              && std::fabs(g.ppx - 31.2f) < 1e-4f && std::fabs(g.ppy - 24.7f) < 1e-4f,
              "intrinsics round-trip exactly");
        check(std::fabs(g.baselineM - 0.0499f) < 1e-6f, "baseline round-trips");
        check((g.flags & DepthRecordHeader::FLAG_EMITTER_ON) != 0, "emitter flag survives");

        std::vector<float> px;
        check(r.readFrame(3, px), "seeks to an arbitrary frame");
        check(px.size() == size_t(W) * H, "frame size correct");
        check(std::fabs(px[1] - float((1 + 3 * 13) % 60000) * 0.001f) < 1e-6f,
              "pixel value and scale correct");
        check(px[0] < 0.f, "a stored 0 reads back as INVALID, not 0 metres");

        // Wrapping matters: the live view loops a recording, and an off-by-one
        // at the wrap would show as a stutter nobody would trace to the reader.
        r.readFrame(N - 1, px);
        std::vector<float> tmp;
        const int a = r.readNext(tmp);
        const int b = r.readNext(tmp);
        check(a == N - 1 && b == 0, "readNext wraps to 0 at the end",
              std::to_string(a) + "," + std::to_string(b));
    }

    // --- truncation -------------------------------------------------------
    // A capture killed by a pulled cable is the NORMAL way a recording ends.
    {
        const std::string tpath = "/tmp/kdr_trunc.kdr";
        FILE* in = std::fopen(path.c_str(), "rb");
        FILE* out = std::fopen(tpath.c_str(), "wb");
        // Header plus 3.5 frames: a partial frame at the end, and a header that
        // still claims 7.
        const long keep = DepthRecordHeader::HEADER_BYTES + long(W) * H * 2 * 3
                        + long(W) * H;      // half a frame
        std::vector<char> buf(static_cast<size_t>(keep), 0);
        std::fread(buf.data(), 1, buf.size(), in);
        std::fwrite(buf.data(), 1, buf.size(), out);
        std::fclose(in); std::fclose(out);

        DepthRecordReader r;
        std::string err;
        check(r.open(tpath, &err), "a truncated recording still opens", err);
        check(r.frameCount() == 3, "and reports only the COMPLETE frames",
              std::to_string(r.frameCount()));
        std::vector<float> px;
        check(r.readFrame(2, px) && !r.readFrame(3, px),
              "reads every complete frame and refuses the partial one");
    }

    // --- refusals ---------------------------------------------------------
    {
        const std::string bad = "/tmp/kdr_bad.kdr";
        FILE* f = std::fopen(bad.c_str(), "wb");
        const char junk[128] = {'n', 'o', 't', ' ', 'a', ' ', 'k', 'd', 'r'};
        std::fwrite(junk, 1, sizeof(junk), f);
        std::fclose(f);
        DepthRecordReader r;
        std::string err;
        check(!r.open(bad, &err), "refuses a file that is not a recording");
        check(err.find("magic") != std::string::npos, "and says why", err);

        DepthRecordReader r2;
        check(!r2.open("/tmp/definitely_not_here.kdr", &err), "refuses a missing file");
    }

    // --- the replay source rebuilds the right camera ----------------------
    {
        ReplayFrameSource src;
        std::string err;
        check(src.open(path, &err), "ReplayFrameSource opens the recording", err);
        const DepthCamera& c = src.camera();
        check(std::fabs(c.fpx() - 425.3f) < 1e-3f, "camera fx comes from the FILE",
              std::to_string(c.fpx()));
        check(std::fabs(c.fy() - 424.9f) < 1e-3f, "and fy, which is not equal to fx");
        check(std::fabs(c.ppx() - 31.2f) < 1e-3f && std::fabs(c.ppy() - 24.7f) < 1e-3f,
              "and the OFF-CENTRE principal point");

        // The ray through the principal point must be dead ahead. If this is
        // wrong every carve is skewed, and the symptom in the map looks like a
        // sensor fault rather than a frame convention.
        CamPose pose;   // yaw 0 = +y (North), level
        float dx, dy, dz;
        c.rayFor(pose, int(std::lround(c.ppx())), int(std::lround(c.ppy())), dx, dy, dz);
        check(std::fabs(dx) < 0.02f && std::fabs(dz) < 0.02f && dy > 0.99f,
              "the ray through the principal point is straight ahead");

        cv::Mat d;
        PoseHint hint;
        check(src.next(d, hint), "delivers a frame");
        check(d.type() == CV_32F && d.rows == H && d.cols == W, "as CV_32F metres");
        check(!hint.valid,
              "and reports NO pose -- a recording does not know where it was");
        check(src.frameCount() == N, "frame count exposed");
    }

    std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "all checks passed",
                fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
