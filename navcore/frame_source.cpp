#include "frame_source.hpp"

#include "attitude_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "realsense_dyn.hpp"

namespace sim {

bool SimFrameSource::next(cv::Mat& depth, PoseHint& hint) {
    depth = truth_ ? cam_.renderTruth(w_, pose_) : cam_.renderStereo(w_, pose_, nullptr);
    hint.valid = true;              // the sim knows exactly where it was
    hint.attitudeOnly = false;
    hint.pose = pose_;
    if (irOn_) {
        irL_ = cam_.renderIR(w_, pose_);
        if (irRight_) {
            // The right imager: the same attitude, the baseline along the
            // camera's own +x (camToWorld is the one rotation definition).
            float dx, dy, dz;
            DepthCamera::camToWorld(pose_, 1.f, 0.f, 0.f, dx, dy, dz);
            CamPose r = pose_;
            const float b = cam_.params().baselineM;
            r.e += dx * b; r.n += dy * b; r.u += dz * b;
            irR_ = cam_.renderIR(w_, r);
        }
    }
    return !depth.empty();
}

bool SimFrameSource::intensity(cv::Mat& out) const {
    if (!irOn_ || irL_.empty()) return false;
    out = irL_.clone();
    return true;
}

bool SimFrameSource::intensityRight(cv::Mat& out) const {
    if (!irOn_ || !irRight_ || irR_.empty()) return false;
    out = irR_.clone();
    return true;
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
    bool start(int w, int h, int fps, bool emitter, bool strobe, bool stereoIr,
               bool colour, std::string* err) {
        // Ask for the IMU. Both optional streams fail independently and
        // neither failure costs depth -- see rsdyn::Pipeline::start.
        // wantIR: the left imager. It costs a little USB bandwidth and it is
        // what `demo` runs its person detector on -- see FrameSource::
        // intensity(). A device or link that refuses it still gives depth,
        // which is why it is a separate attempt inside Pipeline::start.
        if (!pipe_.start(w, h, fps, true, true, stereoIr, colour)) {
            if (err) *err = pipe_.error();
            return false;
        }
        if (colour)
            std::fprintf(stderr, pipe_.haveColor()
                ? "[live] RGB camera streaming: people are found in colour.\n"
                : "[live] RGB camera NOT streaming (%s): people fall back to IR.\n",
                pipe_.error().c_str());
        pipe_.setEmitter(emitter);
        if (emitter && strobe) {
            // Said out loud either way: a strobe that silently did not engage
            // leaves VIO tracking the dot pattern.
            const bool s = pipe_.setEmitterStrobe(true);
            strobing_ = s;
            std::fprintf(stderr, s ? "[live] emitter STROBING: VIO uses the dark frames.\n"
                                   : "[live] emitter strobe NOT supported by this device/"
                                     "firmware: emitter left on, VIO will see the dots.\n");
        }

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
        att_.init(AttitudeParams{});
        haveImu_ = pipe_.haveIMU();
        std::fprintf(stderr, haveImu_
            ? "[live] IMU present: attitude will be estimated from it.\n"
            : "[live] NO IMU stream. Attitude is whatever the caller assumes.\n");
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
        } else if (!pipe_.waitFrames(raw_, w, h,
                                     pipe_.haveIR() ? &ir_ : nullptr,
                                     haveImu_ ? &motion_ : nullptr, 2000,
                                     pipe_.haveIR2() ? &ir2_ : nullptr,
                                     pipe_.haveColor() ? &color_ : nullptr)) {
            return false;
        }
        rawW_ = w; rawH_ = h;
        // Raw samples for a consumer that integrates them (stereo-inertial
        // SLAM). Bounded: nobody draining them must not grow memory.
        for (const auto& m : motion_) {
            if (rawImu_.size() >= 8000) rawImu_.erase(rawImu_.begin(), rawImu_.begin() + 4000);
            ImuRaw r; r.tS = m.tMs * 1e-3; r.gyro = m.isGyro; r.x = m.x; r.y = m.y; r.z = m.z;
            rawImu_.push_back(r);
        }
        irW_ = w; irH_ = h;

        // FEED THE FILTER EVERY SAMPLE, not one per depth frame. The gyro runs
        // at 200 Hz against 30 Hz of depth, so a frameset carries several and
        // keeping only the last would throw away most of the rotation -- which
        // is exactly the rotation the estimate exists to track.
        for (const auto& m : motion_) {
            if (m.isGyro) { gx_ = m.x; gy_ = m.y; gz_ = m.z; }
            else          { ax_ = m.x; ay_ = m.y; az_ = m.z; }
            const double t = m.tMs;
            if (lastImuMs_ > 0.0 && t > lastImuMs_) {
                att_.update(gx_, gy_, gz_, ax_, ay_, az_,
                            float((t - lastImuMs_) * 0.001));
            } else if (lastImuMs_ <= 0.0 && !m.isGyro) {
                att_.seed(ax_, ay_, az_);      // level once, immediately
            }
            if (t > 0.0) lastImuMs_ = t;
        }
        motion_.clear();
        // Z-DEPTH IN, RANGE OUT. librealsense reports the distance along the
        // OPTICAL AXIS; everything downstream of this seam works in range
        // along the ray, and VoxelMap::rayInsert normalises the direction
        // before marching. Handing it Z would place every off-centre return
        // nearer than it is -- 18% at the edge of a 70-degree frame, 25% at
        // the corner -- so a flat wall would map as a bowl curving toward the
        // camera. The sim never showed this because renderTruth already
        // returns range.
        depth.create(h, w, CV_32F);
        for (int y = 0; y < h; ++y) {
            float* dst = depth.ptr<float>(y);
            const uint16_t* src = raw_.data() + size_t(y) * w;
            for (int x = 0; x < w; ++x)
                dst[x] = src[x] ? float(src[x]) * scale_ * cam_->rangePerZ(x, y)
                                : -1.f;
        }
        ++idx_;
        // ATTITUDE ONLY, and the flag says so. There is no translation in here
        // and there must not appear to be: `attitudeOnly` is the seam the whole
        // frame_source header exists to describe.
        if (haveImu_ && att_.seeded()) {
            hint.valid = true;
            hint.attitudeOnly = true;
            hint.pose.rollDeg  = att_.rollDeg();
            hint.pose.pitchDeg = att_.pitchDeg();
            hint.pose.yawDeg   = att_.yawDeg();
        } else {
            hint.valid = false;   // no attitude either -- the caller decides
        }
        return true;
    }

    bool intensity(cv::Mat& out) const override {
        if (ir_.size() != size_t(irW_) * irH_ || irW_ <= 0) return false;
        out = cv::Mat(irH_, irW_, CV_8U, (void*)ir_.data()).clone();
        return true;
    }
    bool intensityRight(cv::Mat& out) const override {
        if (ir2_.size() != size_t(irW_) * irH_ || irW_ <= 0) return false;
        out = cv::Mat(irH_, irW_, CV_8U, (void*)ir2_.data()).clone();
        return true;
    }
    double intensityTimeS() const override {
        const double ms = pipe_.lastIrTimeMs();
        return ms < 0.0 ? -1.0 : ms * 1e-3;
    }
    void takeImu(std::vector<ImuRaw>& out) override { out.clear(); out.swap(rawImu_); }

    bool colour(cv::Mat& bgr, cv::Mat& rangeM) const override {
        if (color_.w <= 0 || color_.bgr.size() != size_t(color_.w) * color_.h * 3) return false;
        bgr = cv::Mat(color_.h, color_.w, CV_8UC3, (void*)color_.bgr.data()).clone();
        rsdyn::Intrinsics ci;
        float ex[12];
        const rsdyn::Intrinsics di = pipe_.intrinsics();
        if (pipe_.colorCalibration(ci, ex) && di.fx > 0.f &&
            raw_.size() == size_t(rawW_) * rawH_) {
            const float dk[4] = {di.fx, di.fy, di.ppx, di.ppy};
            const float ck[4] = {ci.fx, ci.fy, ci.ppx, ci.ppy};
            rangeM = registerDepthToColour(raw_.data(), rawW_, rawH_, scale_, dk, ck,
                                           color_.w, color_.h, ex);
        } else {
            rangeM.release();            // no calibration yet: boxes get no range
        }
        return true;
    }
    float stereoBaselineM() const override { return pipe_.baselineM(); }

    // Raw metadata -- informational. Its polarity has been reported INVERTED
    // under the strobe (realsense-ros #3040), so the consumer's dark-frame
    // gate (emitter_gate.hpp) decides from the image instead.
    int intensityEmitter() const override { return pipe_.lastIrEmitter(); }

    int index() const override { return idx_; }
    std::string info(int which) const { return pipe_.deviceInfo(which); }
    float depthScale() const { return scale_; }

private:
    rsdyn::Pipeline pipe_;
    std::vector<uint16_t> raw_;
    std::vector<uint8_t>  ir_, ir2_;
    rsdyn::Pipeline::ColorFrame color_;
    int  rawW_ = 0, rawH_ = 0;
    std::vector<ImuRaw>   rawImu_;
    int  irW_ = 0, irH_ = 0;
    std::unique_ptr<DepthCamera> cam_;
    float scale_ = 0.001f;
    std::vector<rsdyn::Pipeline::Motion> motion_;
    AttitudeFilter att_;
    double lastImuMs_ = 0.0;
    float gx_ = 0, gy_ = 0, gz_ = 0, ax_ = 0, ay_ = 0, az_ = 0;
    bool  haveImu_ = false;
    bool  strobing_ = false;
    bool  ok_ = false, pending_ = false;
    int   pendingW_ = 0, pendingH_ = 0, idx_ = 0;
};

std::unique_ptr<FrameSource> makeLiveSource(int w, int h, int fps, bool emitter,
                                            std::string* err, bool strobe, bool stereoIr,
                                            bool colour) {
    auto s = std::unique_ptr<RealSenseSource>(new RealSenseSource());
    if (!s->start(w, h, fps, emitter, strobe, stereoIr, colour, err)) return nullptr;
    std::printf("[live] %s  serial %s  fw %s  usb %s\n",
                s->info(rsdyn::CAMERA_INFO_NAME).c_str(),
                s->info(rsdyn::CAMERA_INFO_SERIAL).c_str(),
                s->info(rsdyn::CAMERA_INFO_FIRMWARE).c_str(),
                s->info(rsdyn::CAMERA_INFO_USB_TYPE).c_str());
    return std::unique_ptr<FrameSource>(s.release());
}

cv::Mat registerDepthToColour(const uint16_t* z16, int w, int h, float scale,
                              const float dk[4], const float ck[4],
                              int cw, int ch, const float ex[12], int stride) {
    cv::Mat out(ch, cw, CV_32F, cv::Scalar(-1.f));
    if (!z16 || w <= 0 || h <= 0 || cw <= 0 || ch <= 0 || dk[0] <= 0.f || ck[0] <= 0.f)
        return out;
    stride = std::max(1, stride);
    // How many colour pixels one depth sample spans -- the ratio of focal
    // lengths, times the stride -- and so how far to splat it to leave no
    // pinholes. Not at all when it spans a pixel or less: a splat there only
    // lets "nearest wins" hand a pixel its neighbour's (shorter, off-axis)
    // range, which measured 0.2 % low on a flat wall.
    const float foot = float(stride) * ck[0] / dk[0];
    const int rad = foot <= 1.0f ? 0 : int(std::ceil(0.5f * foot));
    for (int v = 0; v < h; v += stride) {
        const uint16_t* row = z16 + size_t(v) * w;
        for (int u = 0; u < w; u += stride) {
            if (!row[u]) continue;
            const float Z = float(row[u]) * scale;
            const float X = (float(u) - dk[2]) / dk[0] * Z;
            const float Y = (float(v) - dk[3]) / dk[1] * Z;
            // rs2_extrinsics: rotation is column-major, p' = R p + t.
            const float Xc = ex[0] * X + ex[3] * Y + ex[6] * Z + ex[9];
            const float Yc = ex[1] * X + ex[4] * Y + ex[7] * Z + ex[10];
            const float Zc = ex[2] * X + ex[5] * Y + ex[8] * Z + ex[11];
            if (Zc <= 0.05f) continue;
            const int uc = int(std::lround(ck[0] * Xc / Zc + ck[2]));
            const int vc = int(std::lround(ck[1] * Yc / Zc + ck[3]));
            const float r = std::sqrt(Xc * Xc + Yc * Yc + Zc * Zc);
            for (int dy = -rad; dy <= rad; ++dy) {
                const int y = vc + dy;
                if (y < 0 || y >= ch) continue;
                float* o = out.ptr<float>(y);
                for (int dx = -rad; dx <= rad; ++dx) {
                    const int x = uc + dx;
                    if (x < 0 || x >= cw) continue;
                    if (o[x] <= 0.f || r < o[x]) o[x] = r;     // nearest wins
                }
            }
        }
    }
    return out;
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
