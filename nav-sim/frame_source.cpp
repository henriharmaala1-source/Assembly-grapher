#include "frame_source.hpp"

#include <cmath>
#include <cstdio>

#include "realsense_dyn.hpp"

namespace sim {

bool SimFrameSource::next(cv::Mat& depth, PoseHint& hint) {
    depth = truth_ ? cam_.renderTruth(w_, pose_) : cam_.renderStereo(w_, pose_, nullptr);
    hint.valid = true;              // the sim knows exactly where it was
    hint.attitudeOnly = false;
    hint.pose = pose_;
    return !depth.empty();
}

bool ReplayFrameSource::open(const std::string& path, std::string* err) {
    if (!rd_.open(path, err)) return false;
    const DepthRecordHeader& h = rd_.header();

    // Rebuild the camera from the RECORDING's intrinsics, not from a guess.
    // This is the whole reason they are in the file: the mapper carves along
    // rays, and rays built from an assumed pinhole are systematically wrong in
    // a way that looks exactly like a bad calibration once it reaches the map.
    CamParams p;
    p.width = int(h.width);
    p.height = int(h.height);
    p.baselineM = h.baselineM > 0 ? h.baselineM : 0.05f;
    p.fxPx = h.fx; p.fyPx = h.fy; p.ppxPx = h.ppx; p.ppyPx = h.ppy;
    if (h.fx > 0.f)
        p.hfovDeg = 2.f * std::atan(p.width * 0.5f / h.fx) * 180.f / sim::PI_F;
    p.maxRangeM = 40.f;
    cam_.reset(new DepthCamera(p));
    return true;
}

bool ReplayFrameSource::next(cv::Mat& depth, PoseHint& hint) {
    if (rd_.readNext(buf_) < 0) return false;
    const DepthRecordHeader& h = rd_.header();
    depth.create(int(h.height), int(h.width), CV_32F);
    std::memcpy(depth.ptr<float>(0), buf_.data(), buf_.size() * sizeof(float));
    // A bare recording knows nothing about where it was. Saying so is the
    // point: the caller must decide, and a source that quietly returned the
    // origin would produce a map that looks plausible and is meaningless.
    hint.valid = false;
    hint.attitudeOnly = false;
    return true;
}

// ---------------------------------------------------------------------------
// LIVE. No #ifdef, no build-time SDK, no find_package: librealsense is loaded
// at RUN time through realsense_dyn. See that header for why -- briefly, the
// build kept failing to find an SDK that was installed, and a dependency that
// only has to exist at runtime should not be able to break a build at all.
//
// One consequence worth stating: this branch is now compiled on EVERY machine,
// so it cannot rot behind an #ifdef nobody defines. It already had.
class RealSenseSource : public FrameSource {
public:
    bool start(int w, int h, int fps, bool emitter, std::string* err) {
        if (!pipe_.start(w, h, fps)) {
            if (err) *err = pipe_.error();
            return false;
        }
        pipe_.setEmitter(emitter);

        // Pull one frame before reporting success. The intrinsics come from the
        // frame's own profile, so until a frame has arrived we do not actually
        // know the geometry -- and a device that starts but never delivers is a
        // failure the caller should hear about now rather than at frame 1.
        int fw = 0, fh = 0;
        if (!pipe_.waitDepth(raw_, fw, fh, 5000)) {
            if (err) *err = pipe_.error();
            pipe_.stop();
            return false;
        }
        const rsdyn::Intrinsics in = pipe_.intrinsics();

        CamParams p;
        p.width = fw; p.height = fh;
        if (in.fx > 0.f) {
            p.fxPx = in.fx; p.fyPx = in.fy; p.ppxPx = in.ppx; p.ppyPx = in.ppy;
            p.hfovDeg = 2.f * std::atan(fw * 0.5f / in.fx) * 180.f / sim::PI_F;
        } else {
            // No intrinsics is survivable but must be said out loud: the rays
            // will be built from an assumed pinhole and every carve is then
            // systematically off.
            std::fprintf(stderr, "[live] WARNING: no intrinsics reported; "
                                 "assuming a centred pinhole at %.0f deg\n",
                         p.hfovDeg);
        }
        p.baselineM = 0.05f;      // D435i nominal; not reported by this path
        p.maxRangeM = 40.f;
        cam_.reset(new DepthCamera(p));
        scale_ = pipe_.depthScale();
        pending_ = true;          // the frame just read is the first one out
        pendingW_ = fw; pendingH_ = fh;
        ok_ = true;
        return true;
    }

    const char* name() const override { return "realsense"; }
    bool ok() const override { return ok_; }
    const CamParams& params() const override { return cam_->params(); }
    const DepthCamera& camera() const override { return *cam_; }

    bool next(cv::Mat& depth, PoseHint& hint) override {
        int w = pendingW_, h = pendingH_;
        if (pending_) {
            pending_ = false;               // reuse the frame start() already took
        } else if (!pipe_.waitDepth(raw_, w, h, 2000)) {
            return false;
        }
        depth.create(h, w, CV_32F);
        for (int y = 0; y < h; ++y) {
            float* dst = depth.ptr<float>(y);
            const uint16_t* src = raw_.data() + size_t(y) * w;
            for (int x = 0; x < w; ++x) dst[x] = src[x] ? float(src[x]) * scale_ : -1.f;
        }
        ++idx_;
        hint.valid = false;      // no odometry yet -- the caller decides
        return true;
    }

    int index() const override { return idx_; }
    std::string info(int which) const { return pipe_.deviceInfo(which); }
    float depthScale() const { return scale_; }

private:
    rsdyn::Pipeline pipe_;
    std::vector<uint16_t> raw_;
    std::unique_ptr<DepthCamera> cam_;
    float scale_ = 0.001f;
    bool  ok_ = false, pending_ = false;
    int   pendingW_ = 0, pendingH_ = 0, idx_ = 0;
};

std::unique_ptr<FrameSource> makeLiveSource(int w, int h, int fps, bool emitter,
                                            std::string* err) {
    auto s = std::unique_ptr<RealSenseSource>(new RealSenseSource());
    if (!s->start(w, h, fps, emitter, err)) return nullptr;
    std::printf("[live] %s  serial %s  fw %s  usb %s\n",
                s->info(rsdyn::CAMERA_INFO_NAME).c_str(),
                s->info(rsdyn::CAMERA_INFO_SERIAL).c_str(),
                s->info(rsdyn::CAMERA_INFO_FIRMWARE).c_str(),
                s->info(rsdyn::CAMERA_INFO_USB_TYPE).c_str());
    return std::unique_ptr<FrameSource>(s.release());
}

// Runtime, not compile time: can the library be loaded right now.
bool haveLiveSupport() { return rsdyn::load(nullptr); }

std::string liveSupportDetail() {
    std::string err;
    if (rsdyn::load(&err))
        return "librealsense " + std::to_string(rsdyn::apiVersion() / 10000) + "." +
               std::to_string((rsdyn::apiVersion() / 100) % 100) + "." +
               std::to_string(rsdyn::apiVersion() % 100) + " (" +
               rsdyn::libraryPath() + ")";
    return err;
}

}  // namespace sim
