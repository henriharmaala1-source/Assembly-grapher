#pragma once
// ---------------------------------------------------------------------------
// SlamLink -- the wire between onboard and an external SLAM process.
//
// WHY A SEPARATE PROCESS. ORB-SLAM3 is GPLv3; linking it into the flight
// binary would put all of onboard under GPLv3. So it runs as its own program,
// kestrel-orbslam (onboard/orbslam/), and the two exchange plain data over a
// local socket: frames in, poses out. The same seam would take any other SLAM
// (Basalt, OpenVINS) without onboard changing.
//
// WHO OWNS THE CAMERA. Onboard does: the D435i can be streamed by one process,
// and onboard needs its depth for the voxel map and proximity. It forwards the
// two IR images, their device timestamp, the IMU samples since the last frame
// and the intrinsics; the bridge builds its ORB-SLAM3 settings from the first
// frame's intrinsics, so there is no calibration file to keep in step.
//
// FLOW CONTROL is the client's: at most one frame in flight, the newest
// replacing any not yet sent (IMU samples are carried forward, never dropped).
// ORB-SLAM3 on a Pi 5 is slower than the camera; move-stop-sense is slow
// enough that tracking every second or third frame is what it can afford.
//
// Wire format: little-endian, native struct layout (both ends are built on
// the same machine), each message = header struct + payload.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace slamlink {

constexpr uint32_t kMagicFrame = 0x464C534Bu;   // "KSLF"
constexpr uint32_t kMagicPose  = 0x504C534Bu;   // "KSLP"
constexpr uint32_t kVersion    = 1;

// One IMU sample, ALREADY in the left IR camera's axes (librealsense rotates
// D435i motion data into the depth frame), device time in seconds.
struct ImuSample {
    double tS = 0;
    float  ax = 0, ay = 0, az = 0;      // m/s^2
    float  gx = 0, gy = 0, gz = 0;      // rad/s
};

enum FrameFlags : uint32_t {
    kFlagInertial = 1u,                 // ask for stereo-inertial (else stereo)
};

struct FrameHeader {
    uint32_t magic = kMagicFrame, version = kVersion;
    uint32_t seq = 0, flags = 0;
    double   tS = 0;                    // device time of the IR pair, seconds
    int32_t  width = 0, height = 0;     // both images, CV_8U, rectified
    int32_t  nImu = 0;
    float    fx = 0, fy = 0, cx = 0, cy = 0;
    float    baselineM = 0;             // left -> right, along +x
    float    fps = 30.f;
};
// payload: width*height left, width*height right, nImu * ImuSample

// ORB-SLAM3's Tracking::eTrackingState, mirrored so onboard need not include it.
enum TrackState : int32_t {
    kNotReady = -1, kNoImages = 0, kNotInitialized = 1, kOk = 2,
    kRecentlyLost = 3, kLost = 4, kOkKlt = 5,
};

struct PoseReply {
    uint32_t magic = kMagicPose, version = kVersion;
    uint32_t seq = 0;                   // the frame this answers
    int32_t  state = kNotReady;
    int32_t  mapId = -1;                // a new map is a NEW FRAME: re-anchor
    uint32_t mapChanges = 0;            // loop closures / merges: a jump
    float    Twc[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};  // 3x4 row-major,
                                        // left camera -> SLAM world, metres
    int32_t  tracked = 0;               // map points matched this frame
    float    ms = 0;                    // bridge's processing time
};

// ---- blocking socket helpers (both ends) ---------------------------------
bool writeAll(int fd, const void* p, size_t n);
bool readAll(int fd, void* p, size_t n);

bool sendFrame(int fd, const FrameHeader& h, const uint8_t* left, const uint8_t* right,
               const ImuSample* imu);
// Reads one frame; `left`, `right`, `imu` are resized to fit.
bool recvFrame(int fd, FrameHeader& h, std::vector<uint8_t>& left,
               std::vector<uint8_t>& right, std::vector<ImuSample>& imu);
bool sendPose(int fd, const PoseReply& r);
bool recvPose(int fd, PoseReply& r);

int  listenUnix(const std::string& path, std::string* err = nullptr);
int  connectUnix(const std::string& path, std::string* err = nullptr);
void closeFd(int fd);
void removePath(const std::string& path);

}  // namespace slamlink
