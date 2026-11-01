#include "visual_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sim {

const char* poseModeName(PoseMode m) {
    switch (m) {
        case PoseMode::Vio:  return "vio";
        case PoseMode::Slam: return "slam";
        default:             return "fixed";
    }
}

bool parsePoseMode(const std::string& s, PoseMode& out) {
    if (s == "fixed") { out = PoseMode::Fixed; return true; }
    if (s == "vio")   { out = PoseMode::Vio;   return true; }
    if (s == "slam")  { out = PoseMode::Slam;  return true; }
    return false;
}

void VisualPose::init(const FrameSource& src, const VisualPoseParams& p) {
    src_ = &src;
    p_ = p;
    est_ = PoseEstimate{};
    if (p_.mode == PoseMode::Vio) vio_.init(src.camera(), p_.vio);
    if (p_.mode == PoseMode::Slam) slam_.reset(new SlamClient(p_.slamSocket));
    est_.status = p_.mode == PoseMode::Fixed ? "fixed position" : "starting";
}

const PoseEstimate& VisualPose::step(FrameSource& src, const cv::Mat& depth,
                                     const CamPose& attitude, float headingDeg,
                                     double nowS) {
    est_.fresh = false;
    if (p_.mode == PoseMode::Fixed) return est_;

    // WHICH FRAMES a tracker may see: all with the emitter off, none with it
    // on, the dark ones under the strobe -- from the IMAGE (emitter_gate.hpp).
    bool dotFree = true;
    if (p_.emitter == EmitterMode::On) dotFree = false;
    else if (p_.emitter == EmitterMode::Strobe) {
        cv::Mat ir;
        dotFree = src.intensity(ir) && DarkFrameGate::usable(gate_.classify(ir));
    }
    est_.dotFree = dotFree;

    if (p_.mode == PoseMode::Slam) stepSlam(src, attitude, headingDeg, nowS);
    else if (dotFree) stepVio(src, depth, attitude, headingDeg, nowS);
    if (p_.emitter == EmitterMode::On)
        est_.status = "emitter ON: the dots would be tracked -- use strobe or off";
    return est_;
}

void VisualPose::publish(bool valid, float e, float n, float u, float yaw, int tracked,
                         int resets, double nowS, double frameS) {
    est_.frameTimeS = frameS;
    if (!valid) ++est_.lost;
    if (valid && prevT_ > 0.0 && nowS > prevT_) {
        const float dt = float(nowS - prevT_);
        const float a = std::min(1.f, dt / 0.3f);          // ~0.3 s smoothing
        est_.ve += a * ((e - prevE_) / dt - est_.ve);
        est_.vn += a * ((n - prevN_) / dt - est_.vn);
    }
    if (valid) { prevT_ = nowS; prevE_ = e; prevN_ = n; }
    est_.fresh = true;
    est_.valid = valid;
    if (valid) haveFirst_ = true;
    if (valid) { est_.e = e; est_.n = n; est_.u = u; est_.yawDeg = yaw; }
    est_.tracked = tracked;
    est_.resets = resets;
}

void VisualPose::stepVio(FrameSource& src, const cv::Mat& depth, const CamPose& att,
                         float heading, double nowS) {
    cv::Mat ir;
    if (!src.intensity(ir) || ir.size() != depth.size()) {
        est_.status = "no IR image in this source";
        return;
    }
    if (!vio_.started()) {
        // Start at the origin facing the given heading: ENU from here on.
        CamPose start;
        start.rollDeg = att.rollDeg; start.pitchDeg = att.pitchDeg;
        start.yawDeg = heading;
        vio_.reset(start);
    }
    const VioResult r = vio_.step(ir, depth, att);
    publish(r.valid, r.pose.e, r.pose.n, r.pose.u, r.pose.yawDeg, r.tracked, r.resets, nowS,
            nowS);
    est_.status = r.valid ? "tracking" : "lost, coasting";
}

void VisualPose::stepSlam(FrameSource& src, const CamPose& att, float heading,
                          double nowS) {
    est_.linkUp = slam_ && slam_->connected();
    // IMU: drained EVERY frame, lit ones too -- stereo-inertial integrates
    // every sample between the frames it sees.
    if (p_.slamInertial) {
        std::vector<ImuRaw> raw;
        src.takeImu(raw);
        imuSync_.push(raw);
        imuSync_.drain(slamImu_);
    }
    // 1. SEND this pair if it is trackable.
    if (est_.dotFree) {
        cv::Mat l, r;
        if (src.intensity(l) && src.intensityRight(r) && l.size() == r.size()) {
            slamlink::FrameHeader h;
            const double t = src.intensityTimeS();
            h.tS = t >= 0.0 ? t : nowS;
            h.width = l.cols; h.height = l.rows;
            const DepthCamera& cam = src.camera();
            h.fx = cam.fpx(); h.fy = cam.fy(); h.cx = cam.ppx(); h.cy = cam.ppy();
            h.baselineM = src.stereoBaselineM();
            h.fps = p_.fps;
            h.flags = p_.slamInertial ? slamlink::kFlagInertial : 0u;
            const uint32_t seq = slam_->submit(h, l.data, r.data, slamImu_);
            slamImu_.clear();
            ctx_[seq] = Ctx{att, heading, nowS};
            while (ctx_.size() > 64) ctx_.erase(ctx_.begin());
        } else {
            est_.status = "no stereo IR pair in this source";
            if (!warnedStereo_) {
                std::fprintf(stderr, "[pose] slam: this source has no stereo IR pair "
                                     "(right imager not streaming?) -- nothing to track\n");
                warnedStereo_ = true;
            }
        }
    }
    if (!est_.linkUp) est_.status = "no kestrel-orbslam at " + p_.slamSocket;

    // 2. TAKE whatever came back.
    slamlink::PoseReply rep;
    long conn = 0;
    while (slam_->takeReply(rep, &conn)) {
        // A new connection after the first is, as far as anyone can tell, a
        // RESTARTED bridge: its map id starts again at 0 and its origin is
        // wherever the camera was when it came up. Same id, different frame --
        // and after a vocabulary load of 5-30 s the jump guard, at 4 m/s, would
        // wave through 20-120 m. So a new connection re-anchors like a new map.
        if (conn != slamConn_) {
            if (slamConn_ != 0) slamRestarted_ = true;
            slamConn_ = conn;
        }
        const auto it = ctx_.find(rep.seq);
        const bool ok = rep.state == slamlink::kOk || rep.state == slamlink::kOkKlt;
        if (!ok || it == ctx_.end()) {
            publish(false, 0, 0, 0, 0, rep.tracked, resets_, nowS,
                    it == ctx_.end() ? nowS : it->second.tS);
            est_.status = rep.state == slamlink::kNotInitialized ? "initialising"
                        : rep.state == slamlink::kRecentlyLost   ? "recently lost"
                                                                  : "lost";
            continue;
        }
        const Ctx& ctx = it->second;
        // Where the last estimate has got to by now, carried on the measured
        // velocity for at most half a second: a re-anchor pins HERE.
        const float carry = float(std::min(0.5, std::max(0.0, nowS - lastSlamS_)));
        const float predE = lastE_ + est_.ve * carry, predN = lastN_ + est_.vn * carry;
        // ANCHOR on the first tracked frame and on every new map -- a new map
        // is a new coordinate frame.
        if (!anchor_.anchored() || rep.mapId != slamMap_ || slamRestarted_) {
            CamPose a = ctx.att;
            a.yawDeg = haveLast_ ? lastYaw_ : ctx.heading;
            a.e = haveLast_ ? predE : 0.f;
            a.n = haveLast_ ? predN : 0.f;
            a.u = haveLast_ ? lastU_ : 0.f;
            anchor_.anchor(rep.Twc, a);
            if (slamMap_ != -1) ++resets_;
            slamMap_ = rep.mapId;
            // A new process counts its map changes from zero: that is not a
            // change, the re-anchor above already reported the discontinuity.
            if (slamRestarted_) slamChanges_ = rep.mapChanges;
            slamRestarted_ = false;
        }
        // A loop closure or merge moves the pose within its frame: a JUMP the
        // consumer must be told about.
        if (rep.mapChanges != slamChanges_) { ++resets_; slamChanges_ = rep.mapChanges; }
        float e, n, u, yaw;
        anchor_.toEnu(rep.Twc, e, n, u, yaw);
        // An impossible jump is the SLAM frame moving, not the camera.
        if (haveLast_ && !SlamAnchor::plausible(e - lastE_, n - lastN_, u - lastU_,
                                                 nowS - lastSlamS_)) {
            CamPose a = ctx.att;
            a.yawDeg = lastYaw_; a.e = predE; a.n = predN; a.u = lastU_;
            anchor_.anchor(rep.Twc, a);
            ++resets_;
            anchor_.toEnu(rep.Twc, e, n, u, yaw);
        }
        lastSlamS_ = nowS;
        haveLast_ = true;
        lastE_ = e; lastN_ = n; lastU_ = u; lastYaw_ = yaw;
        publish(true, e, n, u, yaw, rep.tracked, resets_, nowS, ctx.tS);
        est_.status = "tracking";
    }
}

}  // namespace sim
