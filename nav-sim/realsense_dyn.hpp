#pragma once

#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// librealsense AT RUNTIME. No SDK at build time, no headers, no import library,
// no find_package.
//
// WHY. The build kept reporting "no RealSense SDK" on a machine where the SDK
// was installed and the Viewer streamed happily. Three rounds of increasingly
// clever CMake search paths did not fix it, and that is the signal that the
// approach is wrong rather than incomplete: a build that has to go looking for
// a dependency will keep finding new ways not to find it. The dependency here
// is not really needed at build time at all.
//
// WHAT MAKES THIS POSSIBLE. librealsense's C++ interface is a header-only
// inline wrapper over a flat C API -- that is why linking against the
// pyrealsense2 wheel worked earlier, and it is the same reason this does. The
// C API is a couple of dozen plain functions with opaque pointers, so they can
// be declared here and resolved from the shared library at RUN time. The
// declarations below are not a copy of the SDK; they are the ABI, which is what
// a shared library is for.
//
// CONSEQUENCES, and they are the point:
//   * voxel_live builds identically with or without the SDK installed
//   * --live works the moment realsense2.dll is present ANYWHERE the loader
//     looks, which includes the directory the Viewer installed it into
//   * there is no build-time search to get wrong, so no class of failure that
//     looks like a missing camera
//   * a version mismatch cannot happen: the API version handed to
//     rs2_create_context is read FROM the library itself
//
// The one thing given up is compile-time type checking against the SDK headers.
// Bought back with realsense_dyn_check, which resolves every symbol and
// exercises the whole path against a real librealsense binary.
// ---------------------------------------------------------------------------

namespace rsdyn {

// Enum values from rs_sensor.h / rs_option.h / rs_types.h. These are ABI, not
// implementation: changing them would break every compiled program using the
// library, which is why they can be written down.
enum : int {
    STREAM_DEPTH    = 1,
    STREAM_INFRARED = 3,
    FORMAT_Z16      = 1,
    OPTION_EMITTER_ENABLED = 12,
    CAMERA_INFO_NAME = 0,
    CAMERA_INFO_SERIAL = 1,
    CAMERA_INFO_FIRMWARE = 2,
    CAMERA_INFO_USB_TYPE = 12,
};

struct Intrinsics {          // layout of rs2_intrinsics, verified against the header
    int   width = 0, height = 0;
    float ppx = 0, ppy = 0, fx = 0, fy = 0;
    int   model = 0;
    float coeffs[5] = {0, 0, 0, 0, 0};
};

// Try to load the library. Returns false and fills `err` with the paths tried.
// Safe to call repeatedly; the handle is kept.
bool load(std::string* err = nullptr);
bool loaded();
const std::string& libraryPath();     // what actually got loaded, "" if none
int  apiVersion();                    // the LIBRARY's version, not ours

// A depth stream, opened. Everything here returns false rather than throwing;
// `error()` holds the last librealsense message.
class Pipeline {
public:
    ~Pipeline() { stop(); }

    bool start(int width, int height, int fps);
    void stop();
    bool running() const { return pipe_ != nullptr; }

    // Blocks up to timeoutMs. `out` receives width*height uint16 device units.
    bool waitDepth(std::vector<uint16_t>& out, int& w, int& h, int timeoutMs = 2000);

    float      depthScale() const { return depthScale_; }
    Intrinsics intrinsics() const { return intr_; }
    float      baselineM() const { return baseline_; }
    bool       setEmitter(bool on);
    std::string deviceInfo(int cameraInfo) const;

    const std::string& error() const { return err_; }

private:
    void* ctx_ = nullptr;
    void* pipe_ = nullptr;
    void* profile_ = nullptr;
    void* sensor_ = nullptr;
    float depthScale_ = 0.001f;
    float baseline_ = 0.05f;
    Intrinsics intr_;
    mutable std::string err_;
};

}  // namespace rsdyn
