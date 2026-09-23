#include "voxel_nav.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

VoxelNavModule::VoxelNavModule(std::unique_ptr<sim::FrameSource> src,
                               const Params& p)
    : src_(std::move(src)), p_(p) {
    p_.nav.seedBodyM = p_.bodyR;
    p_.nav.legCoreM  = p_.bodyR;
    if (src_ && src_->ok()) nav_.init(src_->camera(), p_.nav, sim::CamPose{});
}

std::unique_ptr<VoxelNavModule> VoxelNavModule::live(const Params& p, int width,
                                                     int height, int fps,
                                                     std::string* err) {
    // Emitter ON: the D435i's projector is what gives a blank wall texture to
    // match on, and a blank wall is the obstacle stereo is otherwise blind to.
    auto src = sim::makeLiveSource(width, height, fps, true, err);
    return std::unique_ptr<VoxelNavModule>(new VoxelNavModule(std::move(src), p));
}

void VoxelNavModule::run(const cv::Mat& /*colour -- see header*/, WorldModel& wm) {
    if (!isReady()) return;

    cv::Mat depth;
    sim::PoseHint hint;
    if (!src_->next(depth, hint) || depth.empty()) return;

    const WorldState s = wm.snapshot();

    // STILL OR NOT. Under a mission, only the phases the cycle defines as a
    // stable vantage count: THINK and SCAN (and ARMED, hovering for GO).
    // SETTLE does NOT -- it is the phase in which the aircraft is still
    // bleeding off the last leg's speed, and 0.3 m/s of drift across a 1.5 s
    // settle is two cells of smear in a map with no translation to correct
    // it. Without a mission the ground speed alone decides. Either way the
    // ground speed can veto: it catches motion nobody asked for (wind, a
    // pilot on the sticks).
    const std::string& ph = s.missionPhase;
    const bool vantage = !s.missionActive || ph == "THINK" || ph == "SCAN" ||
                         ph == "ARMED";
    const bool still = vantage && s.vehGroundspeed < p_.stillSpeedMs;

    if (!still) {
        still_ = false;
        wm.with([&](WorldState& w) {
            w.voxValid  = false;
            w.voxMoving = true;
            w.voxFrames = 0;          // the old vantage's map is gone
            w.voxStampS = monoNowS();
        });
        return;
    }
    // ATTITUDE. Origin fixed at (0,0,0); only the rotation varies.
    sim::CamPose pose;
    pose.yawDeg = s.vehYawDeg;           // heading: the FC's, compass-referenced
    if (p_.preferCameraImu && hint.valid && hint.attitudeOnly) {
        pose.rollDeg  = hint.pose.rollDeg;
        pose.pitchDeg = hint.pose.pitchDeg;
    } else {
        pose.rollDeg  = s.vehRollDeg;
        pose.pitchDeg = s.vehPitchDeg + p_.mountTiltDeg;
    }
    // A source that knows its pose outright (the sim) is believed entirely --
    // that is what makes the module testable against ground truth.
    if (hint.valid && !hint.attitudeOnly) pose = hint.pose;

    if (!still_) {                       // a new vantage: start a new map
        sim::CamPose origin;             // architecture C: the origin is here
        origin.e = pose.e; origin.n = pose.n; origin.u = pose.u;
        nav_.reset(origin);
        still_ = true;
        ++resets_;
    }

    const sim::GeneralResult r = nav_.step(depth, pose);
    // THE LEG. The mission flies a straight, level line, and the planner's
    // bearing is the endpoint of a curved primitive chosen for openness -- so
    // a straight leg on exactly that bearing is often short or clipped. Search
    // the bearings the camera can actually see, certify each one on the
    // straight line it would fly (nav_pipeline.hpp: straightFreeM), and take
    // the longest, ties to the planner's bearing. Params::farChoose instead
    // lets the far tier pick among near-equal legs; it measured worse and is
    // off -- the numbers are in voxel_nav.hpp.
    struct Cand { float brg, len, far, dev; };
    std::vector<Cand> cands;
    auto add = [&](float brg) {
        const float len = nav_.straightFreeM(pose, brg, p_.legMaxM);
        if (len <= 0.f) return;
        float d = brg - r.azDeg;
        while (d > 180.f) d -= 360.f;
        while (d <= -180.f) d += 360.f;
        const float fr = nav_.farRangeAt(brg, 0.f);
        const float far = fr < 0.f ? 0.f : std::min(fr, p_.nav.farRangeM);
        cands.push_back({brg, len, far, std::fabs(d)});
    };
    if (!r.blocked) add(r.azDeg);
    const float half = src_->camera().params().hfovDeg * 0.5f - p_.legFovMarginDeg;
    for (float o = -half; o <= half + 1e-3f; o += p_.legStepDeg)
        add(pose.yawDeg + o);

    float legBrg = r.azDeg, legFree = 0.f, legFar = 0.f;
    if (!cands.empty()) {
        float bestLen = 0.f;
        for (const Cand& c : cands) bestLen = std::max(bestLen, c.len);
        const Cand* best = nullptr;
        float bestScore = -1e9f;
        for (const Cand& c : cands) {
            float score;
            if (p_.farChoose) {
                if (c.len < p_.legKeepFrac * bestLen) continue;   // near decides first
                score = c.far - p_.legTieM * c.dev / 90.f;
            } else {
                score = c.len - p_.legTieM * c.dev / 90.f;
            }
            if (score > bestScore) { bestScore = score; best = &c; }
        }
        legBrg = best->brg; legFree = best->len; legFar = best->far;
    }

    wm.with([&](WorldState& w) {
        w.voxValid      = true;
        w.voxMoving     = false;
        w.voxBlocked    = r.blocked;
        w.voxBearingDeg = r.azDeg;
        w.voxElDeg      = r.elDeg;
        w.voxSpeed      = r.blocked ? 0.f : r.speed;
        w.voxFreeM      = r.freeM;
        w.voxOpenM      = r.openM;
        w.voxLegFreeM   = legFree;
        w.voxLegBearingDeg = legBrg;
        w.voxLegFarM    = legFar;
        w.voxFrames     = nav_.frames();
        w.voxStampS     = monoNowS();
    });
}
