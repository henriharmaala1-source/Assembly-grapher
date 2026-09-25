// SPDX-License-Identifier: GPL-3.0-or-later
// ---------------------------------------------------------------------------
// kestrel-orbslam -- ORB-SLAM3 as a separate process behind SlamLink.
//
//   kestrel-orbslam --vocab ORBvoc.txt [--socket /tmp/kestrel-slam.sock]
//                   [--features 1000] [--settings-out FILE]
//
// Serves one client at a time on a local socket (navcore/slam_link.hpp):
// a stereo IR pair (+ IMU) in, the left camera's pose and ORB-SLAM3's tracking
// state out, one reply per frame. The ORB-SLAM3 settings are WRITTEN from the
// first frame's intrinsics -- no calibration file to drift out of step with
// the camera -- and the SLAM system lives for the life of the process, so a
// client that reconnects keeps its map.
//
// Stereo by default; stereo-inertial when the first frame asks for it
// (kFlagInertial). Stereo gives metric scale from the baseline on its own and
// needs no motion to initialise -- the right default for an aircraft that
// hovers and stops. Stereo-inertial initialises its IMU only after enough
// excitation and is the option for faster flight.
//
// GPLv3 because ORB-SLAM3 is. Nothing in onboard links this; they share only
// the protocol header.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <csignal>
#include <execinfo.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include <opencv2/core.hpp>

#include "System.h"
#include "ImuTypes.h"
#include "slam_link.hpp"

namespace {

struct Options {
    std::string vocab, socket = "/tmp/kestrel-slam.sock", settingsOut;
    int features = 1000;
};

// ORB-SLAM3 "1.0" settings for a RECTIFIED stereo pair. IMU noise is the
// figure ORB-SLAM3's own D435i example uses (from VINS-Mono): generous,
// because the D435i's BMI055 is a consumer part and is not calibrated at the
// factory (rs-imu-calibration writes its intrinsics). The IMU sits at the
// depth origin to within millimetres and librealsense already reports it in
// the depth frame's axes, so T_b_c1 is identity plus ORB-SLAM3's D435i offset.
std::string writeSettings(const slamlink::FrameHeader& h, bool inertial, int features,
                          const std::string& outPath) {
    std::string path = outPath;
    if (path.empty()) {
        char tmpl[] = "/tmp/kestrel-orbslam-XXXXXX.yaml";
        const int fd = mkstemps(tmpl, 5);
        if (fd >= 0) ::close(fd);
        path = tmpl;
    }
    std::ofstream f(path);
    f << "%YAML:1.0\n"
      << "File.version: \"1.0\"\n"
      << "Camera.type: \"Rectified\"\n"
      << "Camera1.fx: " << h.fx << "\nCamera1.fy: " << h.fy << "\n"
      << "Camera1.cx: " << h.cx << "\nCamera1.cy: " << h.cy << "\n"
      << "Stereo.b: " << h.baselineM << "\n"
      << "Camera.width: " << h.width << "\nCamera.height: " << h.height << "\n"
      << "Camera.fps: " << int(h.fps + 0.5f) << "\n"
      << "Camera.RGB: 0\n"
      << "Stereo.ThDepth: 40.0\n";
    if (inertial)
        f << "IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n"
          << "   data: [1.0, 0.0, 0.0, -0.005, 0.0, 1.0, 0.0, -0.005,"
             " 0.0, 0.0, 1.0, 0.0117, 0.0, 0.0, 0.0, 1.0]\n"
          << "IMU.InsertKFsWhenLost: 0\n"
          << "IMU.NoiseGyro: 1e-3\nIMU.NoiseAcc: 1e-2\n"
          << "IMU.GyroWalk: 1e-6\nIMU.AccWalk: 1e-4\nIMU.Frequency: 200.0\n";
    f << "ORBextractor.nFeatures: " << features << "\n"
      << "ORBextractor.scaleFactor: 1.2\nORBextractor.nLevels: 8\n"
      << "ORBextractor.iniThFAST: 20\nORBextractor.minThFAST: 7\n"
      << "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\n"
      << "Viewer.GraphLineWidth: 0.9\nViewer.PointSize: 2.0\n"
      << "Viewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
      << "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\n"
      << "Viewer.ViewpointZ: -1.8\nViewer.ViewpointF: 500.0\n";
    return path;
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](std::string& v) { if (i + 1 >= argc) return false; v = argv[++i]; return true; };
        std::string v;
        if (a == "--vocab" && next(v)) o.vocab = v;
        else if (a == "--socket" && next(v)) o.socket = v;
        else if (a == "--settings-out" && next(v)) o.settingsOut = v;
        else if (a == "--features" && next(v)) o.features = std::atoi(v.c_str());
        else return false;
    }
    return !o.vocab.empty();
}

}  // namespace

// A CRASH LEAVES A STACK TRACE on stderr, then dies as it would have. The
// bridge runs unattended aboard; "it exited" with nothing else is how a
// once-in-thirty startup crash went undiagnosed (only async-signal-safe calls
// here: backtrace_symbols_fd writes straight to the descriptor).
void onCrash(int sig) {
    const char msg[] = "\n[orbslam] FATAL signal -- stack:\n";
    if (::write(2, msg, sizeof msg - 1) < 0) {}
    void* frames[64];
    const int n = backtrace(frames, 64);
    backtrace_symbols_fd(frames, n, 2);
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

int main(int argc, char** argv) {
    for (int sig : {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL}) std::signal(sig, onCrash);
    Options opt;
    if (!parse(argc, argv, opt)) {
        std::fprintf(stderr, "usage: kestrel-orbslam --vocab ORBvoc.txt [--socket PATH] "
                             "[--features N] [--settings-out FILE]\n");
        return 2;
    }
    std::string err;
    const int lfd = slamlink::listenUnix(opt.socket, &err);
    if (lfd < 0) { std::fprintf(stderr, "[orbslam] %s\n", err.c_str()); return 1; }
    std::printf("[orbslam] listening on %s\n", opt.socket.c_str());
    std::fflush(stdout);

    std::unique_ptr<ORB_SLAM3::System> slam;
    slamlink::FrameHeader first;
    bool inertial = false;
    long lastMap = -1;
    uint32_t mapChanges = 0;
    std::vector<uint8_t> L, R;
    std::vector<slamlink::ImuSample> imu;
    for (;;) {
        const int fd = ::accept(lfd, nullptr, nullptr);
        if (fd < 0) continue;
        std::printf("[orbslam] client connected\n");
        std::fflush(stdout);
        slamlink::FrameHeader h;
        while (slamlink::recvFrame(fd, h, L, R, imu)) {
            slamlink::PoseReply rep;
            rep.seq = h.seq;
            if (!slam) {
                // The first frame fixes the camera. Loading the vocabulary takes
                // seconds (longer on a Pi): the client is waiting on this reply.
                first = h;
                inertial = (h.flags & slamlink::kFlagInertial) != 0;
                const std::string yaml = writeSettings(h, inertial, opt.features,
                                                       opt.settingsOut);
                slam.reset(new ORB_SLAM3::System(
                    opt.vocab, yaml,
                    inertial ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO,
                    /*bUseViewer=*/false));
                std::printf("[orbslam] %s %dx%d f %.1f b %.4f m\n",
                            inertial ? "stereo-inertial" : "stereo", h.width, h.height,
                            double(h.fx), double(h.baselineM));
                std::fflush(stdout);
            } else if (h.width != first.width || h.height != first.height ||
                       std::fabs(h.fx - first.fx) > 0.5f) {
                // A different camera geometry would silently corrupt the map.
                std::fprintf(stderr, "[orbslam] frame geometry changed; restart the bridge\n");
                break;
            }
            const auto t0 = std::chrono::steady_clock::now();
            const cv::Mat l(h.height, h.width, CV_8U, L.data());
            const cv::Mat r(h.height, h.width, CV_8U, R.data());
            std::vector<ORB_SLAM3::IMU::Point> meas;
            if (inertial) {
                meas.reserve(imu.size());
                for (const auto& s : imu)
                    meas.emplace_back(s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.tS);
            }
            const Sophus::SE3f Tcw = slam->TrackStereo(l, r, h.tS, meas);
            rep.ms = std::chrono::duration<float, std::milli>(
                         std::chrono::steady_clock::now() - t0).count();
            rep.state = slam->GetTrackingState();
            rep.mapId = int32_t(slam->GetCurrentMapIdHeadless());
            if (slam->MapChanged()) ++mapChanges;      // loop closure / merge
            if (lastMap >= 0 && rep.mapId != lastMap) ++mapChanges;
            lastMap = rep.mapId;
            rep.mapChanges = mapChanges;
            const Eigen::Matrix4f Twc = Tcw.inverse().matrix();
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 4; ++j) rep.Twc[i * 4 + j] = Twc(i, j);
            int tracked = 0;
            for (ORB_SLAM3::MapPoint* p : slam->GetTrackedMapPoints()) if (p) ++tracked;
            rep.tracked = tracked;
            if (!slamlink::sendPose(fd, rep)) break;
        }
        ::close(fd);
        std::printf("[orbslam] client gone; map kept\n");
        std::fflush(stdout);
    }
}
