#include "frame_source.hpp"

#include <cmath>
#include <cstdio>

#ifdef NAVSIM_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

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
#ifdef NAVSIM_HAVE_REALSENSE

class RealSenseSource : public FrameSource {
public:
    bool start(int w, int h, int fps, bool emitter, std::string* err) {
        try {
            cfg_.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
            auto prof = pipe_.start(cfg_);
            // first_depth_sensor() exists in the PYTHON bindings, not in C++.
            // The C++ API is the templated device::first<T>(). Caught only by
            // compiling this block, which nothing had done until the headers
            // were fetched -- an #ifdef'd branch that never compiles anywhere
            // is not code, it is a plan.
            auto ds = prof.get_device().first<rs2::depth_sensor>();
            scale_ = ds.get_depth_scale();
            if (ds.supports(RS2_OPTION_EMITTER_ENABLED))
                ds.set_option(RS2_OPTION_EMITTER_ENABLED, emitter ? 1.f : 0.f);

            auto vsp = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            auto in  = vsp.get_intrinsics();
            CamParams p;
            p.width = in.width; p.height = in.height;
            p.fxPx = in.fx; p.fyPx = in.fy; p.ppxPx = in.ppx; p.ppyPx = in.ppy;
            p.hfovDeg = 2.f * std::atan(in.width * 0.5f / in.fx) * 180.f / sim::PI_F;
            p.baselineM = 0.05f;
            try {
                auto ir2 = prof.get_stream(RS2_STREAM_INFRARED, 2);
                p.baselineM = std::fabs(vsp.get_extrinsics_to(ir2).translation[0]);
            } catch (...) { /* nominal 50 mm; reported in the banner */ }
            p.maxRangeM = 40.f;
            cam_.reset(new DepthCamera(p));
            ok_ = true;
            return true;
        } catch (const std::exception& e) {
            if (err) *err = e.what();
            return false;
        }
    }

    const char* name() const override { return "realsense"; }
    bool ok() const override { return ok_; }
    const CamParams& params() const override { return cam_->params(); }
    const DepthCamera& camera() const override { return *cam_; }

    bool next(cv::Mat& depth, PoseHint& hint) override {
        try {
            rs2::frameset fs;
            if (!pipe_.try_wait_for_frames(&fs, 2000)) return false;
            rs2::depth_frame df = fs.get_depth_frame();
            if (!df) return false;
            const int w = df.get_width(), h = df.get_height();
            depth.create(h, w, CV_32F);
            const uint16_t* src = static_cast<const uint16_t*>(df.get_data());
            for (int y = 0; y < h; ++y) {
                float* dst = depth.ptr<float>(y);
                for (int x = 0; x < w; ++x) {
                    const uint16_t u = src[y * w + x];
                    dst[x] = u ? float(u) * scale_ : -1.f;
                }
            }
            ++idx_;
            hint.valid = false;      // no odometry yet -- the caller decides
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    int index() const override { return idx_; }
    float depthScale() const { return scale_; }

private:
    rs2::pipeline pipe_;
    rs2::config   cfg_;
    std::unique_ptr<DepthCamera> cam_;
    float scale_ = 0.001f;
    bool  ok_ = false;
    int   idx_ = 0;
};

std::unique_ptr<FrameSource> makeLiveSource(int w, int h, int fps, bool emitter,
                                            std::string* err) {
    auto s = std::unique_ptr<RealSenseSource>(new RealSenseSource());
    if (!s->start(w, h, fps, emitter, err)) return nullptr;
    return std::unique_ptr<FrameSource>(s.release());
}
bool haveLiveSupport() { return true; }

#else

std::unique_ptr<FrameSource> makeLiveSource(int, int, int, bool, std::string* err) {
    if (err)
        *err = "this build has no librealsense. Rebuild with the RealSense SDK "
               "installed (CMake finds it via find_package(realsense2)); until "
               "then use --replay on a .kdr recording.";
    return nullptr;
}
bool haveLiveSupport() { return false; }

#endif

}  // namespace sim
