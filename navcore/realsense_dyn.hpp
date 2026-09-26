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

// Enum values from rs_sensor.h / rs_option.h / rs_types.h / rs_frame.h. These
// are ABI, not implementation: changing them would break every compiled
// program using the library, which is why they can be written down.
//
// WRITE THEM DOWN FROM THE HEADER, NOT FROM MEMORY. Three of these were wrong
// until 2026-09-24 and each failed silently: the option was 12 (VISUAL_PRESET,
// so "emitter on" selected a depth preset), the depth-sensor extension was 12
// (DEPTH_FRAME, so no sensor ever matched: the emitter was never set and the
// depth scale was never read -- the 0.001 default happens to be the D435i's),
// and the USB-type info was 12 (FIRMWARE_UPDATE_ID). All re-checked against
// librealsense v2.55.1 include/librealsense2/h/.
enum : int {
    STREAM_DEPTH    = 1,             // rs2_stream
    STREAM_COLOR    = 2,
    STREAM_INFRARED = 3,
    STREAM_GYRO     = 5,
    STREAM_ACCEL    = 6,
    FORMAT_Z16      = 1,             // rs2_format
    FORMAT_BGR8     = 6,
    FORMAT_Y8       = 9,
    FORMAT_MOTION_XYZ32F = 16,
    OPTION_EMITTER_ENABLED = 18,     // rs2_option
    OPTION_EMITTER_ON_OFF  = 46,     // alternate emitter on/off every frame
    EXTENSION_DEPTH_SENSOR = 7,      // rs2_extension
    METADATA_LASER_POWER_MODE = 12,  // rs2_frame_metadata_value: 0 off, 1 on
    METADATA_EMITTER_MODE     = 29,  //   newer firmware: 0 off, >0 some emitter
    CAMERA_INFO_NAME = 0,            // rs2_camera_info
    CAMERA_INFO_SERIAL = 1,
    CAMERA_INFO_FIRMWARE = 2,
    CAMERA_INFO_USB_TYPE = 9,
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

    // `wantIR` adds infrared index 1 -- the LEFT imager, which is the one depth
    // is computed in and therefore the one already registered with it. `wantIMU`
    // adds the gyro and accelerometer.
    //
    // Both are optional because both can fail independently on a given device
    // or USB link: a D435 (no i) has no motion sensor at all, and a USB 2
    // connection routinely refuses the extra bandwidth. A failure to add them
    // must not cost the depth stream, so each is enabled in its own attempt.
    // `wantIR2` adds infrared index 2, the RIGHT imager: with index 1 it is
    // the rectified stereo pair a stereo SLAM tracks (onboard/orbslam).
    // `wantColor` adds the RGB camera at colorW x colorH, BGR8. It is a
    // separate USB stream on its own sensor; if the pipeline will not start
    // with it (a USB 2 link, usually), it starts again WITHOUT it rather than
    // losing depth -- haveColor() says which happened.
    bool start(int width, int height, int fps,
               bool wantIR = false, bool wantIMU = false, bool wantIR2 = false,
               bool wantColor = false, int colorW = 640, int colorH = 480);
    bool haveColor() const { return haveColor_; }
    bool haveIR()  const { return haveIR_; }
    bool haveIR2() const { return haveIR2_; }
    bool haveIMU() const { return haveIMU_; }
    void stop();
    bool running() const { return pipe_ != nullptr; }

    // Blocks up to timeoutMs. `out` receives width*height uint16 device units.
    bool waitDepth(std::vector<uint16_t>& out, int& w, int& h, int timeoutMs = 2000);

    // One motion sample as delivered. Motion frames arrive at their own rate --
    // typically 200 Hz gyro against 30 Hz depth -- so a frameset carries
    // several, and dropping all but the last would throw away most of the
    // rotation. They are accumulated and handed over whole.
    struct Motion {
        bool  isGyro = false;      // false = accelerometer
        float x = 0, y = 0, z = 0; // rad/s for gyro, m/s^2 for accel
        double tMs = 0;            // device timestamp
    };

    // Depth plus whatever else was enabled. `ir` is width*height uint8 and is
    // left untouched when infrared is off; `motion` is appended to, never
    // cleared, so a caller can drain several framesets before integrating.
    // `ir2` receives the right IR image when index 2 was enabled.
    // `color` receives the RGB frame (BGR8) when the colour stream is on.
    struct ColorFrame { std::vector<uint8_t> bgr; int w = 0, h = 0; };
    bool waitFrames(std::vector<uint16_t>& depth, int& w, int& h,
                    std::vector<uint8_t>* ir,
                    std::vector<Motion>* motion,
                    int timeoutMs = 2000,
                    std::vector<uint8_t>* ir2 = nullptr,
                    ColorFrame* color = nullptr);
    // The colour camera's calibration, from the device, once a colour frame
    // has arrived: its intrinsics, and depth -> colour extrinsics as
    // rs2_extrinsics lays them out (rotation[9] COLUMN-major, translation[3]
    // in metres). False until both are known.
    bool colorCalibration(Intrinsics& colorIntr, float depthToColor[12]) const;
    // Device timestamp (ms, the motion samples' clock) of the last left IR
    // frame delivered; < 0 if none.
    double lastIrTimeMs() const { return lastIrMs_; }

    float      depthScale() const { return depthScale_; }
    Intrinsics intrinsics() const { return intr_; }
    // Left -> right IR distance from the device's own extrinsics once both IR
    // streams have delivered a frame; the 50 mm nominal until then.
    float      baselineM() const { return baseline_; }
    bool       setEmitter(bool on);
    // STROBE: emitter on for one frame, off for the next, and so on. Depth
    // keeps the projector's texture on half the frames; the other half give
    // an IR image with no dots in it, which is the only kind visual odometry
    // can track (the dots move WITH the camera and read as zero motion).
    bool       setEmitterStrobe(bool on);
    // Emitter state of the last IR frame delivered, from frame metadata:
    // 1 lit, 0 dark, -1 unknown (metadata unsupported -- on Linux it needs
    // the patched kernel module or the RSUSB backend).
    int        lastIrEmitter() const { return lastIrEmitter_; }
    std::string deviceInfo(int cameraInfo) const;

    const std::string& error() const { return err_; }

private:
    void* ctx_ = nullptr;
    void* pipe_ = nullptr;
    void* profile_ = nullptr;
    void* sensor_ = nullptr;
    bool  haveIR_ = false, haveIMU_ = false, haveIR2_ = false, haveColor_ = false;
    bool  haveColorCal_ = false;
    Intrinsics colorIntr_;
    float d2c_[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    double lastIrMs_ = -1.0;
    bool  haveBaseline_ = false;
    int   lastIrEmitter_ = -1;
    float depthScale_ = 0.001f;
    float baseline_ = 0.05f;
    Intrinsics intr_;
    mutable std::string err_;
};

}  // namespace rsdyn
