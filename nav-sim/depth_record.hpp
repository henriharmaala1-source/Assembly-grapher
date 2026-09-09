#pragma once

#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// .kdr -- a depth recording, deliberately the dumbest format that works.
//
// WHY NOT .npz, WHICH WE ALREADY WRITE. Reading one from C++ means a ZIP
// reader plus a .npy header parser plus zlib, to open a file whose entire
// content is a header and a block of uint16. And why not librealsense's own
// .bag: that API turned out to be version-dependent (the wheel on the dev
// machine rejects .bag and wants .db3) and could not be exercised without
// hardware, which is exactly the kind of untestable dependency this project
// keeps refusing.
//
// So: a 64-byte fixed header and raw frames. Thirty lines of C++, thirty lines
// of Python, no dependencies on either side, and the round trip is checked
// BOTH WAYS in the test suite -- Python writes and C++ reads, C++ writes and
// Python reads. A format both ends agree on is worth more than a format with a
// specification.
//
// DEVICE UNITS, NOT METRES, and this is not laziness. The camera reports
// uint16; storing that verbatim makes the file half the size and bit-exact with
// what the sensor said, and pushes the scale factor into metadata where it can
// be wrong loudly rather than baked into every sample where it cannot be
// recovered.
//
// INTRINSICS TRAVEL WITH THE PIXELS. A depth frame without fx/ppx is not
// interpretable -- you cannot turn a pixel into a ray. Recording them in the
// header is what makes a file from six months ago still usable, and it is what
// lets the replay path build rays that match the camera that produced them
// rather than an assumed pinhole.
// ---------------------------------------------------------------------------

namespace sim {

struct DepthRecordHeader {
    uint32_t width = 0, height = 0, frames = 0;
    float depthScale = 0.001f;   // metres per stored unit
    float fx = 0, fy = 0;        // pixels
    float ppx = 0, ppy = 0;      // principal point, pixels
    float baselineM = 0;
    uint32_t flags = 0;          // bit0: IR emitter was ON
    static constexpr uint32_t FLAG_EMITTER_ON = 1u;
    static constexpr int HEADER_BYTES = 64;
};

// Streaming reader. Holds one frame at a time -- a 600-frame 848x480 recording
// is 488 MB and does not want to be resident.
class DepthRecordReader {
public:
    bool open(const std::string& path, std::string* err = nullptr);
    void close();
    bool isOpen() const { return f_ != nullptr; }

    const DepthRecordHeader& header() const { return h_; }
    int  frameCount() const { return int(h_.frames); }
    int  index() const { return idx_; }

    // Read frame `i` into `out` (metres, <=0 invalid). Seeks, so frames may be
    // read in any order -- the GUI needs to scrub and loop.
    bool readFrame(int i, std::vector<float>& out);
    // Next frame, wrapping to 0 at the end. Returns the index read.
    int  readNext(std::vector<float>& out);

    ~DepthRecordReader() { close(); }

private:
    void*  f_ = nullptr;         // FILE*, kept opaque so the header stays clean
    DepthRecordHeader h_;
    int    idx_ = 0;
    std::vector<uint16_t> raw_;
};

// Writer, used by the tests and by any C++-side capture.
class DepthRecordWriter {
public:
    bool open(const std::string& path, const DepthRecordHeader& h,
              std::string* err = nullptr);
    bool writeFrame(const uint16_t* px);         // width*height units
    bool close();                                // patches the frame count
    ~DepthRecordWriter() { close(); }

private:
    void* f_ = nullptr;
    DepthRecordHeader h_;
    uint32_t written_ = 0;
};

}  // namespace sim
