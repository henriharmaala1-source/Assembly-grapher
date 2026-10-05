#include "script_mode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using kms::CondOp;
using kms::Instr;
using kms::Op;
using kms::Target;

namespace {
constexpr double kPi = 3.14159265358979;
constexpr double kD2R = kPi / 180.0;
// Instantaneous instructions run back to back within a tick, up to this many:
// a loop of nothing but jumps cannot hold the fly loop.
constexpr int kMaxStepsPerTick = 100;

double wrap180(double d) {
    while (d > 180.0) d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return d;
}
double wrap360(double d) {
    while (d >= 360.0) d -= 360.0;
    while (d < 0.0) d += 360.0;
    return d;
}
std::string fmt(const char* f, double a = 0, double b = 0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}
}  // namespace

ScriptMode::ScriptMode() : ScriptMode(Params()) {}

ScriptMode::ScriptMode(Params p, ModeLookup lookup)
    : p_(p), lookup_(std::move(lookup)), mission_(p.mission) {}

bool ScriptMode::load(const kms::Program& prog, std::string* err) {
    if (!kms::verify(prog, err)) return false;
    for (const Instr& in : prog.code)
        if (in.op == Op::RUN) {
            const std::string& m = prog.strings[size_t(in.text)];
            if (!lookup_ || !lookup_(m)) {
                if (err) *err = "line " + std::to_string(in.line) + ": this aircraft has no mode '" +
                                m + "'";
                return false;
            }
        }
    prog_ = prog;
    loaded_ = true;
    reset_();
    status_ = "loaded \"" + prog_.name + "\"";
    return true;
}

bool ScriptMode::loadFile(const std::string& path, std::string* err) {
    kms::Program p;
    if (!kms::loadProgram(path, p, err)) return false;
    return load(p, err);
}

void ScriptMode::reset_() {
    started_ = finished_ = failed_ = false;
    pc_ = 0; savedPc_ = -1; inHandler_ = false;
    state_ = -1; trackReqT_ = -1;
    direct_ = false;
    visited_.clear();
    gYaw_ = p_.yawPid; gStrafe_ = p_.strafePid; gVert_ = p_.divePid; gRange_ = p_.rangePid;
    anchorT_ = -1; anchorLabel_.clear();
    fired_.assign(prog_.handlers.size(), false);
    regs_.assign(size_t(std::max(0, prog_.registers)), 0.0);
    places_.assign(prog_.targets.size(), Place());
    t_ = opT_ = opAux_ = noPosT_ = 0;
    opBegun_ = false;
    delegate_ = nullptr;
    missionOn_ = false;
}

void ScriptMode::onEnter(WorldState& s) {
    reset_();
    s.missionGo = false;              // armed: nothing moves before GO
    s.missionGoalValid = false; s.missionLegCapM = 0.f;
    s.fcRequest = WorldState::FcRequest::NONE;
    s.scriptActive = loaded_;
    status_ = loaded_ ? "armed: \"" + prog_.name + "\" -- waiting for GO"
                      : "no mission loaded (kestrel --script FILE.kmb)";
}

void ScriptMode::onExit(WorldState& s) {
    endOp_(s);
    mission_.enable(false);
    s.missionGo = false;
    s.missionGoalValid = false; s.missionLegCapM = 0.f;
    s.fcRequest = WorldState::FcRequest::NONE;
    s.scriptActive = false;
}

// --------------------------------------------------------------- places
void ScriptMode::start_(const WorldState& s) {
    started_ = true;
    Place& st = places_[0];
    st.set = true;
    st.e = s.estValid ? s.estPe : 0.0;
    st.n = s.estValid ? s.estPn : 0.0;
    st.refYaw = s.vehYawDeg;
    st.u = s.vehAltM;
    for (size_t i = 1; i < prog_.targets.size(); ++i) {
        const Target& t = prog_.targets[i];
        Place& p = places_[i];
        if (t.kind == Target::ENU) {
            p.set = true; p.e = st.e + t.x; p.n = st.n + t.y; p.refYaw = st.refYaw;
        } else if (t.kind == Target::GPS && s.vehFix >= 3) {
            // Equirectangular about the start: exact enough inside a fence of
            // a few hundred metres, and the frame estPe/estPn are in.
            const double dn = (t.x - s.vehLat) * 111320.0;
            const double de = (t.y - s.vehLon) * 111320.0 * std::cos(s.vehLat * kD2R);
            p.set = true; p.e = st.e + de; p.n = st.n + dn; p.refYaw = st.refYaw;
        } else if (t.kind == Target::START) {
            p = st;
        }
    }
}

bool ScriptMode::resolve_(int i, Place& out) const {
    if (i < 0 || i >= int(places_.size())) return false;
    const Target& t = prog_.targets[size_t(i)];
    if (t.kind == Target::OFFSET || t.kind == Target::REL) {
        Place b;
        if (!resolve_(t.base, b)) return false;      // bases point backwards: terminates
        out = b;
        if (t.kind == Target::OFFSET) { out.e += t.x; out.n += t.y; }
        else {
            const double h = b.refYaw * kD2R;        // x ahead, y right of the base's heading
            out.e += t.x * std::sin(h) + t.y * std::cos(h);
            out.n += t.x * std::cos(h) - t.y * std::sin(h);
        }
        return true;
    }
    out = places_[size_t(i)];
    return out.set;
}

bool ScriptMode::targetPos(int i, double& e, double& n) const {
    Place p;
    if (!started_ || !resolve_(i, p)) return false;
    e = p.e; n = p.n;
    return true;
}

// ------------------------------------------------------------ conditions
bool ScriptMode::cond_(int c, const WorldState& s) const {
    const kms::CondRange& r = prog_.conds[size_t(c)];
    bool st[64]; int sp = 0;
    auto cmp = [](uint8_t k, double a, double b) {
        switch (k) {
            case CondOp::LT: return a < b;
            case CondOp::LE: return a <= b;
            case CondOp::GT: return a > b;
            default:         return a >= b;
        }
    };
    for (int j = r.start; j < r.start + r.count && sp < 64; ++j) {
        const CondOp& o = prog_.condOps[size_t(j)];
        switch (o.kind) {
            case CondOp::TRUE_: st[sp++] = true; break;
            case CondOp::POSITIONED: st[sp++] = s.estValid; break;
            case CondOp::SEEN: {
                bool any = false;
                if (o.var & kms::FLAG_NEW) {
                    any = seen_(s, prog_.strings[size_t(o.arg)], ctx_, nullptr, nullptr, nullptr, true);
                } else if (s.tickMonoS - s.detStampS <= p_.detStaleSec) {
                    for (const Detection& d : s.detections)
                        any |= d.label == prog_.strings[size_t(o.arg)];
                }
                st[sp++] = any;
                break;
            }
            case CondOp::VAR: {
                double v = 0;
                switch (o.var) {
                    case CondOp::BATTERY: v = s.vehBattery * 100.0; break;
                    case CondOp::ALT:     v = s.vehAltM; break;
                    case CondOp::TIME:    v = t_; break;
                    case CondOp::SPEED:   v = s.vehGroundspeed; break;
                    default:              v = s.vehYawDeg; break;
                }
                st[sp++] = cmp(o.cmp, v, o.value);
                break;
            }
            case CondOp::DIST: {
                Place p;
                // An unknown place, or no position: the comparison is FALSE --
                // a distance nobody can measure satisfies nothing.
                st[sp++] = s.estValid && resolve_(o.arg, p) &&
                           cmp(o.cmp, std::hypot(p.e - s.estPe, p.n - s.estPn), o.value);
                break;
            }
            case CondOp::TRACKING: st[sp++] = tracking_(s, prog_.strings[size_t(o.arg)]); break;
            case CondOp::RANGE: {
                float d = -1.f;
                // Unseen, or seen with no range: FALSE either way -- a range
                // nobody can measure is not below anything.
                const bool vis = seen_(s, prog_.strings[size_t(o.arg)], ctx_, nullptr, nullptr, nullptr,
                                       false, nullptr, &d);
                st[sp++] = vis && d > 0.f && (o.cmp == CondOp::LT ? d < o.value
                                            : o.cmp == CondOp::LE ? d <= o.value
                                            : o.cmp == CondOp::GT ? d > o.value : d >= o.value);
                break;
            }
            case CondOp::NOT: st[sp - 1] = !st[sp - 1]; break;
            case CondOp::AND: --sp; st[sp - 1] = st[sp - 1] && st[sp]; break;
            case CondOp::OR:  --sp; st[sp - 1] = st[sp - 1] || st[sp]; break;
        }
    }
    return sp > 0 && st[sp - 1];
}

// ------------------------------------------------------------ detections
bool ScriptMode::seen_(const WorldState& s, const std::string& label, const ControlCtx& ctx,
                       float* offDeg, float* distM, float* fill, bool onlyNew,
                       cv::Rect* boxOut, float* losM) const {
    if (ctx.frameW <= 0 || ctx.frameH <= 0) return false;
    const bool detFresh = s.tickMonoS - s.detStampS <= p_.detStaleSec;
    const double f = (ctx.frameW * 0.5) / std::tan(p_.detHfovDeg * 0.5 * kD2R);
    // A detection's own line-of-sight range: MEASURED (depth in the box), else
    // its known height over its box height -- refused when the box is cut by
    // the frame edge, where its height is not the object's.
    //   1 measured depth   2 a DECLARED size   3 the ground plane (below)
    //   4 a typical size the compiler assumed -- a guess never beats geometry
    auto losOf = [&](const Detection& d, bool allowAssumed) -> double {
        if (d.rangeM > 0.f) return d.rangeM;
        bool assumed = false;
        const float hM = sizeOf_(d.label, &assumed);
        if (assumed && !allowAssumed) return -1.0;
        const bool cut = d.box.y <= 2 || d.box.y + d.box.height >= ctx.frameH - 2;
        if (hM > 0.f && !cut && d.box.height >= 8) {
            const double r = f * hM / d.box.height;
            if (r <= p_.maxRangeM) return r;
        }
        return -1.0;
    };
    float usedLos = -1.f;
    // Bearing off the nose, horizontal distance (< 0 unknown) and fill, for a
    // box with a known line-of-sight range or none.
    auto measure = [&](const cv::Rect& box, const Detection* det, double los, float& off,
                       float& dist, float& fl) {
        const double cx = box.x + box.width * 0.5 - ctx.frameW * 0.5;
        off = float(std::atan(cx / f) / kD2R);
        fl = float(box.height) / float(ctx.frameH);
        dist = -1.f;
        const double cy = box.y + box.height * 0.5 - ctx.frameH * 0.5;
        const double elCentre = p_.detTiltDeg - std::atan(cy / f) / kD2R;
        // RANGE is straight-line, to the box's centre -- what "4 m from it"
        // means from above; dist is its horizontal part, for positions.
        if (los > 0.0) { dist = float(los * std::cos(elCentre * kD2R)); usedLos = float(los); return; }
        // THE GROUND PLANE, for something standing on it: the box's bottom
        // edge is where it meets the ground, so its depression below the
        // horizon and the altitude give the distance. Refused near the
        // horizon, where it explodes, and without an altitude.
        const double yb = box.y + box.height - ctx.frameH * 0.5;
        const double dep = -(p_.detTiltDeg - std::atan(yb / f) / kD2R);   // + below
        if (s.vehAltM > 0.5f && dep > 3.0) {
            const double d = s.vehAltM / std::tan(dep * kD2R);
            if (d <= p_.maxRangeM) dist = float(d);
        }
        if (dist < 0.f && det) {                       // 4: the typical size, last
            const double la = losOf(*det, true);
            if (la > 0.0) dist = float(la * std::cos(elCentre * kD2R));
        }
        usedLos = dist > 0.f ? float(dist / std::max(0.05, std::cos(elCentre * kD2R))) : -1.f;
    };
    cv::Rect used;
    auto give = [&](float off, float dist, float fl) {
        if (boxOut) *boxOut = used;
        if (losM) *losM = usedLos;
        if (offDeg) *offDeg = off;
        if (distM) *distM = dist;
        if (fill) *fill = fl;
        return true;
    };

    if (onlyNew) {
        // ONLY WHAT HAS NOT BEEN VISITED: every fresh detection of the label,
        // most confident first, placed on the ground; the first whose place is
        // not within newRadiusM of one already marked for this label. One
        // whose place cannot be had counts as new -- nothing says otherwise.
        if (!detFresh) return false;
        std::vector<const Detection*> c;
        for (const Detection& d : s.detections) if (d.label == label) c.push_back(&d);
        std::sort(c.begin(), c.end(), [](const Detection* a, const Detection* b) {
            return a->confidence > b->confidence; });
        for (const Detection* d : c) {
            float off, dist, fl;
            measure(d->box, d, losOf(*d, false), off, dist, fl);
            used = d->box;
            bool visited = false;
            if (dist > 0.f && s.estValid) {
                const double b = (s.vehYawDeg + off) * kD2R;
                const double e = s.estPe + dist * std::sin(b), n = s.estPn + dist * std::cos(b);
                for (const auto& v : visited_)
                    if (v.label == label && std::hypot(v.e - e, v.n - n) < p_.newRadiusM) visited = true;
            }
            if (!visited) return give(off, dist, fl);
        }
        return false;
    }

    const Detection* best = nullptr;
    if (detFresh)
        for (const Detection& d : s.detections)
            if (d.label == label && (!best || d.confidence > best->confidence)) best = &d;
    float off, dist, fl;
    if (tracking_(s, label)) {
        // THE TRACKER'S BOX, fresh every frame, for the bearing. Range through
        // its scale change, anchored on the detector (TrackRef).
        const bool anchor = best && (best->box & s.targetBox).area() > 0;
        // The tracker has no ground contact point, so an assumed size is
        // better than nothing for anchoring it.
        const double dl = anchor ? losOf(*best, true) : -1.0;
        if (dl > 0.0) { tref_.valid = true; tref_.losM = dl; tref_.size0 = float(s.targetBox.height); }
        // Only cores that estimate SCALE can carry it between detections;
        // a fixed-size box would freeze the range at its anchor, silently.
        const std::string core = s.targetCore ? s.targetCore : "";
        const bool scales = core == "fused" || core == "csrt";
        double los = -1.0;
        if (dl > 0.0) los = dl;
        else if (tref_.valid && scales && s.targetBox.height > 0)
            los = tref_.losM * tref_.size0 / double(s.targetBox.height);
        measure(s.targetBox, anchor ? best : nullptr, los, off, dist, fl);
        used = s.targetBox;
        return give(off, dist, fl);
    }
    if (!best) return false;
    measure(best->box, best, losOf(*best, false), off, dist, fl);
    used = best->box;
    return give(off, dist, fl);
}

bool ScriptMode::measureObject_(const WorldState& s, const std::string& label,
                                const ControlCtx& ctx, double& e, double& n,
                                double& bearing, double* topU) const {
    float off = 0, d = -1;
    cv::Rect box;
    if (!s.estValid || !seen_(s, label, ctx, &off, &d, nullptr, false, &box) || !(d > 0.f))
        return false;
    bearing = wrap360(s.vehYawDeg + off);
    e = s.estPe + d * std::sin(bearing * kD2R);
    n = s.estPn + d * std::cos(bearing * kD2R);
    if (topU) *topU = topHeight_(s, ctx, box, d);
    return true;
}

double ScriptMode::topHeight_(const WorldState& s, const ControlCtx& ctx, const cv::Rect& box,
                              double distM) const {
    const double f = (ctx.frameW * 0.5) / std::tan(p_.detHfovDeg * 0.5 * kD2R);
    const double elTop = p_.detTiltDeg - std::atan((box.y - ctx.frameH * 0.5) / f) / kD2R;
    return std::max(0.0, double(s.vehAltM) + distM * std::tan(elTop * kD2R));
}

ControlCmd ScriptMode::crosshair_(const WorldState& s, const ControlCtx& ctx, double px,
                                  double py, float throttle) {
    // Horizontal angle of the crosshair off the nose, and the ELEVATION of
    // the ray through it: the camera's tilt plus its angle above centre.
    const double f = (ctx.frameW * 0.5) / std::tan(p_.detHfovDeg * 0.5 * kD2R);
    const double az = std::atan(px / f) / kD2R;
    const double el = p_.detTiltDeg - std::atan(py / f) / kD2R;
    aimValid_ = true; aimPx_ = px; aimPy_ = py;
    ControlCmd c = hover_();
    c.yaw = aimYaw_(float(az), ctx.dt);
    const float t = std::max(0.f, std::min(1.f, throttle));
    // Along the ray: its horizontal part forward (eased while turning onto
    // it), its vertical part up or down -- the floor still holds.
    c.pitch = t * float(std::cos(el * kD2R)) * std::max(0.f, 1.f - float(std::fabs(az)) / 30.f);
    float v = t * p_.mpsPerStick * float(std::sin(el * kD2R)) / std::max(0.1f, p_.vertMpsPerStick);
    v = std::max(-1.f, std::min(1.f, v));
    if (v < 0.f && s.vehAltM <= p_.minAltM) v = 0.f;
    c.throttle = v;
    return c;
}

float ScriptMode::vertTo_(const WorldState& s, double heightM) const {
    const double target = std::max(double(p_.minAltM), heightM);
    float v = float(p_.altKp * (target - s.vehAltM));
    v = std::max(-p_.maxVert, std::min(p_.maxVert, v));
    if (v < 0.f && s.vehAltM <= p_.minAltM) v = 0.f;      // the floor
    return v;
}

float ScriptMode::sizeOf_(const std::string& label, bool* assumed) const {
    for (const kms::ObjectSize& z : prog_.sizes)
        if (prog_.strings[size_t(z.label)] == label) {
            if (assumed) *assumed = z.assumed != 0;
            return z.heightM;
        }
    return -1.f;
}

bool ScriptMode::tracking_(const WorldState& s, const std::string& label) const {
    return s.trackLabel == label && s.targetValid && s.targetLocked &&
           s.tickMonoS - s.targetStampS <= p_.detStaleSec &&
           s.targetFixAgeS >= 0.f && s.targetFixAgeS < 1.0f;
}

float ScriptMode::yawTo_(float errDeg) const {
    return std::max(-p_.maxYaw, std::min(p_.maxYaw, p_.yawKp * errDeg / 90.f));
}

ControlCmd ScriptMode::fly_(WorldState& s, float dt, bool goal, float bearing, float capM) {
    if (direct_ && goal) {
        // DIRECT: no cycle, no certificate -- turn onto the bearing and fly
        // at it, slowing over the last directSlowM metres. Only the
        // ModeManager's failsafes and the fence stand between this and
        // whatever is in the way; the compiler said so when it was written.
        if (missionOn_) { mission_.enable(false); missionOn_ = false; }
        const float err = float(wrap180(bearing - s.vehYawDeg));
        ControlCmd c = hover_();
        c.yaw = yawTo_(err);
        if (std::fabs(err) < 25.f) {
            const float ease = capM > 0.f ? std::max(0.25f, std::min(1.f, capM / p_.directSlowM)) : 1.f;
            c.pitch = p_.directPitch * ease * (1.f - std::fabs(err) / 25.f * 0.5f);
        }
        s.missionActive = true;
        s.missionPhase = "DIRECT";
        return c;
    }
    if (!missionOn_) { mission_.enable(true); missionOn_ = true; }
    s.missionGoalValid = goal;
    s.missionGoalBearing = bearing;
    s.missionLegCapM = goal ? capM : 0.f;
    s.missionGo = true;
    return mission_.update(s, dt);
}

bool ScriptMode::faceFirst_(const WorldState& s, float bearing, ControlCmd& c) {
    if (direct_) return false;
    const double err = wrap180(bearing - s.vehYawDeg);
    const MissionController::Phase ph = mission_.phase();
    const bool between = !missionOn_ || ph == MissionController::Phase::SETTLE;
    if (!facing_ && !(between && std::fabs(err) > p_.mission.hFovDeg * 0.5 - 10.0)) return false;
    if (std::fabs(err) < 5.0) { facing_ = false; return false; }
    facing_ = true;
    c = hover_();
    c.yaw = yawTo_(float(err));
    return true;
}

// ------------------------------------------------------------ control flow
void ScriptMode::endOp_(WorldState& s) {
    if (missionOn_) { mission_.enable(false); missionOn_ = false; }
    if (delegate_) { delegate_->onExit(s); delegate_ = nullptr; }
    s.missionGoalValid = false;
    s.missionLegCapM = 0.f;
    s.missionGlideDeg = 0.f;
}

void ScriptMode::next_(int pc) {
    pc_ = pc;
    opT_ = 0; opAux_ = 0; opAux2_ = 0; opBegun_ = false; facing_ = false;
    // Each step starts its loops from rest: no integral carried over, no
    // derivative kick from where the last one left the target point.
    pidYaw_.reset(); pidStrafe_.reset(); pidVert_.reset(); pidRange_.reset();
}

void ScriptMode::finish_(WorldState& s, bool failed, const std::string& why) {
    endOp_(s);
    finished_ = true;
    failed_ = failed;
    status_ = why;
}

ControlCmd ScriptMode::update(WorldState& s, const ControlCtx& ctx) {
    ctx_ = ctx;
    aimValid_ = false;
    s.scriptActive = loaded_;
    if (!loaded_) {
        s.scriptStatus = status_;
        return {};                                   // nothing to fly: release
    }
    auto out = [&](ControlCmd c) {
        s.scriptStatus = status_;
        s.scriptLine = (pc_ >= 0 && pc_ < int(prog_.code.size())) ? prog_.code[size_t(pc_)].line : 0;
        s.scriptState = stateName();
        return c;
    };
    if (finished_) {
        // LAND/RTL keep asking; anything else ends in a hover.
        if (s.fcRequest != WorldState::FcRequest::NONE) return out({});
        s.missionActive = true; s.missionPhase = "ARMED";
        return out(hover_());
    }
    if (!s.missionGo) {
        if (started_) {
            endOp_(s);
            status_ = "paused (GO is off) at line " +
                      std::to_string(prog_.code[size_t(pc_)].line);
        }
        s.missionActive = true; s.missionPhase = "ARMED";
        return out(hover_());
    }
    if (!started_) {
        if ((prog_.caps & kms::Program::NEEDS_POSITION) && !s.estValid) {
            status_ = "waiting for a position estimate before starting";
            s.missionActive = true; s.missionPhase = "ARMED";
            return out(hover_());
        }
        if ((prog_.caps & kms::Program::NEEDS_GPS) && s.vehFix < 3) {
            status_ = "waiting for a 3D GPS fix before starting (a gps() place)";
            s.missionActive = true; s.missionPhase = "ARMED";
            return out(hover_());
        }
        start_(s);
        status_ = "started \"" + prog_.name + "\"";
    }

    const double dt = std::max(0.f, ctx.dt);
    t_ += dt;
    if (t_ > prog_.timeoutS) {
        finish_(s, true, fmt("STOPPED: the mission timeout (%.0f s) ran out", prog_.timeoutS));
        return out(hover_());
    }
    if (s.estValid && places_[0].set &&
        std::hypot(s.estPe - places_[0].e, s.estPn - places_[0].n) > prog_.fenceM) {
        finish_(s, true, fmt("STOPPED: outside the %.0f m fence", prog_.fenceM));
        return out(hover_());
    }
    // HANDLERS: each fires once, and not while another is running.
    if (!inHandler_)
        for (size_t h = 0; h < prog_.handlers.size(); ++h)
            if (!fired_[h] && cond_(prog_.handlers[h].cond, s)) {
                fired_[h] = true;
                endOp_(s);
                savedPc_ = pc_;
                inHandler_ = true;
                next_(prog_.handlers[h].pc);
                status_ = "handler (line " + std::to_string(prog_.handlers[h].line) + ") fired";
                break;
            }

    // THE STATE'S TRIGGERS: the first that holds switches state, abandoning
    // whatever the state was in the middle of. Not while a handler runs, and
    // never into the state it is already in (that would restart it each tick).
    if (!inHandler_ && state_ >= 0) {
        const kms::State& st = prog_.states[size_t(state_)];
        for (int k = st.transStart; k < st.transStart + st.transCount; ++k) {
            const kms::Transition& tr = prog_.transitions[size_t(k)];
            if (tr.to != state_ && cond_(tr.cond, s)) {
                endOp_(s);
                anchorT_ = -1; anchorLabel_.clear();       // an abandoned path lets go
                state_ = tr.to;
                next_(prog_.states[size_t(tr.to)].pc);
                status_ = "-> state " + stateName() + " (line " + std::to_string(tr.line) + ")";
                std::printf("[script] %s\n", status_.c_str());
                break;
            }
        }
    }

    // RE-GROUNDING: while a path is anchored on an object, every tick that
    // object is measured moves the anchor toward the new measurement -- and
    // with it every point of the path. Its reference heading does not move.
    if (anchorT_ >= 0) {
        double e, n, b, u = 0;
        if (measureObject_(s, anchorLabel_, ctx, e, n, b, &u)) {
            Place& a = places_[size_t(anchorT_)];
            a.e += p_.anchorGain * (e - a.e);
            a.n += p_.anchorGain * (n - a.n);
            a.u += p_.anchorGain * (u - a.u);
        }
    }

    auto fail = [&](const Instr& in, const std::string& why) -> bool {
        endOp_(s);
        if (in.jump >= 0) { status_ = why + " -- taking the else"; next_(in.jump); return true; }
        finish_(s, true, "STOPPED at line " + std::to_string(in.line) + ": " + why);
        return false;
    };

    for (int steps = 0; steps < kMaxStepsPerTick; ++steps) {
        if (finished_) return out(s.fcRequest != WorldState::FcRequest::NONE ? ControlCmd{} : hover_());
        const Instr& in = prog_.code[size_t(pc_)];
        const std::string text = in.text >= 0 ? prog_.strings[size_t(in.text)] : std::string();
        const bool first = !opBegun_;
        opBegun_ = true;
        if (!first) opT_ += dt;
        s.missionActive = true;

        switch (in.op) {
            // ---------------------------------------------- instantaneous
            case Op::JUMP: next_(in.jump); continue;
            case Op::JUMP_IFNOT: next_(cond_(in.cond, s) ? pc_ + 1 : in.jump); continue;
            case Op::SET_REG: regs_[size_t(in.target)] = in.a; next_(pc_ + 1); continue;
            case Op::LOOP:
                regs_[size_t(in.target)] -= 1.0;
                next_(regs_[size_t(in.target)] > 0.5 ? in.jump : pc_ + 1);
                continue;
            case Op::TIMER: regs_[size_t(in.target)] = t_; next_(pc_ + 1); continue;
            case Op::JUMP_TIMEUP:
                next_(t_ - regs_[size_t(in.target)] >= in.a ? in.jump : pc_ + 1);
                continue;
            case Op::SAY:
                status_ = "\"" + text + "\"";
                std::printf("[script] line %d: %s\n", in.line, text.c_str());
                next_(pc_ + 1);
                continue;
            case Op::MARK: {
                if (!s.estValid) { if (!fail(in, "no position to mark")) return out(hover_()); continue; }
                Place& p = places_[size_t(in.target)];
                p.set = true; p.e = s.estPe; p.n = s.estPn; p.refYaw = s.vehYawDeg; p.u = s.vehAltM;
                next_(pc_ + 1);
                continue;
            }
            case Op::MARK_SEEN: {
                float off = 0, d = -1;
                if (!s.estValid) { if (!fail(in, "no position to place it from")) return out(hover_()); continue; }
                const bool onlyNew = (in.flags & kms::FLAG_NEW) != 0;
                cv::Rect mbox;
                if (!seen_(s, text, ctx, &off, &d, nullptr, onlyNew, &mbox)) {
                    if (!fail(in, onlyNew ? "no NEW '" + text + "' in view" : "'" + text + "' is not in view"))
                        return out(hover_());
                    continue;
                }
                if (!(d > 0.f)) {
                    if (!fail(in, "'" + text + "' seen but no range (no depth, and not on the ground below)"))
                        return out(hover_());
                    continue;
                }
                // THE OBJECT'S POSITION: along the bearing it is seen on, at
                // its range; its reference heading is that line of sight.
                const double brg = wrap360(s.vehYawDeg + off);
                Place& p = places_[size_t(in.target)];
                p.set = true;
                p.e = s.estPe + d * std::sin(brg * kD2R);
                p.n = s.estPn + d * std::cos(brg * kD2R);
                p.refYaw = brg;
                p.u = topHeight_(s, ctx, mbox, d);             // its top
                visited_.push_back({text, p.e, p.n});         // for `new` from now on
                status_ = fmt("marked it %.1f m away on %03.0f", d, brg);
                next_(pc_ + 1);
                continue;
            }
            case Op::GO_STATE:
                endOp_(s);
                anchorT_ = -1; anchorLabel_.clear();
                inHandler_ = false;              // a handler that goes somewhere is done
                state_ = in.target;
                next_(prog_.states[size_t(state_)].pc);
                status_ = "-> state " + stateName();
                continue;
            case Op::NAV:
                endOp_(s);
                direct_ = in.a > 0.5f;
                next_(pc_ + 1);
                continue;
            case Op::STEER: {
                if (in.cond >= 0 && cond_(in.cond, s)) { status_ = "steer: done"; next_(pc_ + 1); continue; }
                if ((in.flags & kms::FLAG_FOR) && opT_ >= in.c) { status_ = "steer: done"; next_(pc_ + 1); continue; }
                if (opT_ > in.c) {
                    if (!fail(in, "steer: its until never came true")) return out(hover_());
                    continue;
                }
                cv::Rect box;
                if (seen_(s, text, ctx, nullptr, nullptr, nullptr, false, &box)) {
                    opAux_ = opT_;
                    // The aim point: `a` box widths right of the box's centre,
                    // as a bearing off the nose.
                    const double f = (ctx.frameW * 0.5) / std::tan(p_.detHfovDeg * 0.5 * kD2R);
                    const double ax = box.x + box.width * (0.5 + double(in.a)) - ctx.frameW * 0.5;
                    const float aimDeg = float(std::atan(ax / f) / kD2R);
                    aimValid_ = true; aimPx_ = ax; aimPy_ = box.y + box.height * 0.5 - ctx.frameH * 0.5;
                    if (in.flags & kms::FLAG_RAY) {
                        // THE CROSSHAIR, PINNED TO THE LOCK: a box widths
                        // right and d box heights up of its centre.
                        const double ay = box.y + box.height * (0.5 - double(in.d)) - ctx.frameH * 0.5;
                        ControlCmd c = crosshair_(s, ctx, ax, ay, in.b);
                        s.missionActive = true;
                        s.missionPhase = "DIRECT";
                        status_ = "steer on '" + text + "' along the crosshair";
                        return out(c);
                    }
                    ControlCmd c = hover_();
                    if (in.flags & kms::FLAG_DIVE) {
                        // DOWN THE LINE OF SIGHT: keep the aim point centred
                        // vertically too, so flying forward also descends
                        // toward it (or climbs). The floor still holds.
                        const double ay = box.y + box.height * 0.5 - ctx.frameH * 0.5;
                        const float below = float(std::atan(ay / f) / kD2R);   // + = below centre
                        c.throttle = std::max(-p_.maxVert, std::min(p_.maxVert, aimVert_(-below, ctx.dt)));
                        if (c.throttle < 0.f && s.vehAltM <= p_.minAltM) c.throttle = 0.f;
                    }
                    const float fwd = std::min(1.f, in.b / std::max(0.1f, p_.mpsPerStick));
                    if (in.flags & kms::FLAG_STRAFE) {
                        c.roll = aimStrafe_(aimDeg, ctx.dt);
                        c.pitch = fwd;
                    } else {
                        c.yaw = aimYaw_(aimDeg, ctx.dt);
                        // Ease off while far off the aim: turn first, then go.
                        c.pitch = fwd * std::max(0.f, 1.f - std::fabs(aimDeg) / 30.f);
                    }
                    s.missionActive = true;
                    s.missionPhase = "DIRECT";
                    status_ = "steer on '" + text + "'" + fmt(" aim %+.0f deg", aimDeg);
                    return out(c);
                }
                if (opT_ - opAux_ > 2.0) {
                    if (!fail(in, "steer: lost '" + text + "'")) return out(hover_());
                    continue;
                }
                return out(hover_());                    // a moment out of view: hold still
            }
            case Op::GAINS: {
                PidGains* g = in.target == 0 ? &gYaw_ : in.target == 1 ? &gStrafe_
                            : in.target == 2 ? &gVert_ : &gRange_;
                if (in.flags & 1) g->kp = in.a;
                if (in.flags & 2) g->ki = in.b;
                if (in.flags & 4) g->kd = in.c;
                if (in.flags & 8) g->dTau = in.d;
                next_(pc_ + 1);
                continue;
            }
            case Op::CRUISE: {
                const bool forT = (in.flags & kms::FLAG_FOR) != 0;
                if (first) opAux_ = s.vehAltM;              // the height to hold
                if (forT && opT_ >= in.c) { next_(pc_ + 1); continue; }
                if (!forT && in.cond >= 0 && cond_(in.cond, s)) { next_(pc_ + 1); continue; }
                if (!forT && opT_ > in.c) {
                    if (!fail(in, "cruise: its until never came true")) return out(hover_());
                    continue;
                }
                ControlCmd c = hover_();
                c.pitch = std::min(1.f, in.a / std::max(0.1f, p_.mpsPerStick));
                c.throttle = vertTo_(s, opAux_);
                s.missionActive = true;
                s.missionPhase = "DIRECT";
                status_ = fmt("cruising %.1f m/s at %.1f m", in.a, opAux_);
                return out(c);
            }
            case Op::FLY: {
                const bool forT = (in.flags & kms::FLAG_FOR) != 0;
                if (forT && opT_ >= in.c) { next_(pc_ + 1); continue; }
                if (!forT && in.cond >= 0 && cond_(in.cond, s)) { next_(pc_ + 1); continue; }
                if (!forT && opT_ > in.c) {
                    if (!fail(in, "fly: its until never came true")) return out(hover_());
                    continue;
                }
                const double px = double(in.a) * ctx.frameW * 0.5;
                const double py = -double(in.d) * ctx.frameH * 0.5;
                ControlCmd c = crosshair_(s, ctx, px, py, in.b);
                s.missionActive = true;
                s.missionPhase = "DIRECT";
                status_ = fmt("fly at the crosshair, throttle %.2f", in.b);
                return out(c);
            }
            case Op::FOLLOW: {
                const bool forT = (in.flags & kms::FLAG_FOR) != 0;
                if (forT && opT_ >= in.c) { status_ = "followed for the time asked"; next_(pc_ + 1); continue; }
                if (!forT && in.cond >= 0 && cond_(in.cond, s)) { status_ = "follow: done"; next_(pc_ + 1); continue; }
                if (!forT && opT_ > in.c) {
                    if (!fail(in, "follow: its until never came true")) return out(hover_());
                    continue;
                }
                float off = 0, d = -1, dh = -1;
                cv::Rect box;
                if (first) { lastRange_ = -1; rangeRate_ = 0; }
                if (seen_(s, text, ctx, &off, &dh, nullptr, false, &box, &d)) {
                    opAux_ = opT_;
                    const double f = (ctx.frameW * 0.5) / std::tan(p_.detHfovDeg * 0.5 * kD2R);
                    const double ax = box.x + box.width * (0.5 + double(in.d)) - ctx.frameW * 0.5;
                    const float aimDeg = float(std::atan(ax / f) / kD2R);
                    aimValid_ = true; aimPx_ = ax; aimPy_ = box.y + box.height * 0.5 - ctx.frameH * 0.5;
                    ControlCmd c = hover_();
                    // Range error -> forward speed, BOTH ways: it backs off
                    // when the object comes closer. No range: hold the
                    // distance, keep it centred.
                    if (d > 0.f) {
                        // How fast the range itself is changing, minus our own
                        // part in it: the OBJECT's speed away, fed forward so
                        // a walking target is not trailed by a fixed lag.
                        const double ub = (s.vehYawDeg + off) * kD2R;
                        if (lastRange_ > 0.0 && dt > 0.0) {
                            // Our own motion toward it, from the estimate.
                            const double own = ((s.estPe - lastE_) * std::sin(ub) +
                                                (s.estPn - lastN_) * std::cos(ub)) / dt;
                            const double objRate = (d - lastRange_) / dt + own;
                            rangeRate_ += 0.15 * (objRate - rangeRate_);
                        }
                        lastRange_ = d; lastE_ = s.estPe; lastN_ = s.estPn;
                        // PD on the range (derivative on it, so closing fast
                        // brakes), on top of the object's own speed.
                        PidGains rg = gRange_;
                        rg.outMax = in.b;
                        const float vWant = std::max(-in.b, std::min(in.b,
                            float(rangeRate_) + pidRange_.step(d - in.a, -d, float(dt), rg)));
                        c.pitch = std::max(-1.f, std::min(1.f, vWant / std::max(0.1f, p_.mpsPerStick)));
                        if (std::fabs(aimDeg) > 30.f) c.pitch *= 0.3f;   // turn first
                    }
                    if (in.flags & kms::FLAG_STRAFE)
                        c.roll = aimStrafe_(aimDeg, ctx.dt);
                    else
                        c.yaw = aimYaw_(aimDeg, ctx.dt);
                    if (in.flags & (kms::FLAG_ALT_ABS | kms::FLAG_ALT_REL)) {
                        const double top = dh > 0.f ? topHeight_(s, ctx, box, dh) : double(s.vehAltM);
                        c.throttle = vertTo_(s, (in.flags & kms::FLAG_ALT_ABS) ? double(in.e)
                                                                                : top + in.e);
                    }
                    s.missionActive = true;
                    s.missionPhase = "DIRECT";
                    status_ = "following '" + text + "'" +
                              (d > 0.f ? fmt(" at %.1f m (want %.1f)", d, in.a) : std::string(" (no range)"));
                    return out(c);
                }
                if (opT_ - opAux_ > 2.0) {
                    if (!fail(in, "follow: lost '" + text + "'")) return out(hover_());
                    continue;
                }
                return out(hover_());
            }
            case Op::ANCHOR: {
                double e, n, b, u = 0;
                if (!measureObject_(s, text, ctx, e, n, b, &u)) {
                    if (!fail(in, "'" + text + "' not in view with a range to anchor the path on"))
                        return out(hover_());
                    continue;
                }
                Place& p = places_[size_t(in.target)];
                p.set = true; p.e = e; p.n = n; p.refYaw = b; p.u = u;   // line of sight: the path's frame
                anchorT_ = in.target; anchorLabel_ = text;
                visited_.push_back({text, e, n});
                status_ = "path anchored on '" + text + "'";
                next_(pc_ + 1);
                continue;
            }
            case Op::UNANCHOR:
                anchorT_ = -1; anchorLabel_.clear();
                next_(pc_ + 1);
                continue;
            case Op::TURN_TO_PLACE: {
                Place tp;
                if (!resolve_(in.target, tp) || !s.estValid) { next_(pc_ + 1); continue; }
                const double brg = wrap360(std::atan2(tp.e - s.estPe, tp.n - s.estPn) / kD2R);
                const double err = wrap180(brg - s.vehYawDeg);
                if (std::fabs(err) < 5.0) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { next_(pc_ + 1); continue; }   // best effort: never stops the path
                ControlCmd c = hover_();
                c.yaw = yawTo_(float(err));
                status_ = "facing it";
                s.missionPhase = "SCAN";
                return out(c);
            }
            case Op::UNTRACK:
                ++s.trackReleaseSeq;
                s.trackLabel.clear();
                tref_ = TrackRef();
                next_(pc_ + 1);
                continue;
            case Op::RESUME:
                inHandler_ = false;
                next_(savedPc_ >= 0 ? savedPc_ : pc_ + 1);
                continue;
            case Op::END:
                finish_(s, false, inHandler_ ? "finished (in a handler): hovering"
                                             : "finished: hovering");
                return out(hover_());
            case Op::LAND:
            case Op::RTL:
                finish_(s, false, in.op == Op::LAND ? "LAND handed to the flight controller"
                                                   : "RTL handed to the flight controller");
                s.fcRequest = in.op == Op::LAND ? WorldState::FcRequest::LAND
                                                : WorldState::FcRequest::RTL;
                return out({});

            // ---------------------------------------------- takes time
            case Op::HOLD:
                if (opT_ >= in.a) { next_(pc_ + 1); continue; }
                status_ = fmt("hold %.0f of %.0f s", opT_, in.a);
                s.missionPhase = "SCAN";             // still: a vantage for the map
                return out(hover_());

            case Op::TURN_TO:
            case Op::TURN_BY: {
                if (first) opAux_ = in.op == Op::TURN_TO ? in.a : wrap360(s.vehYawDeg + in.a);
                const double err = wrap180(opAux_ - s.vehYawDeg);
                if (std::fabs(err) < 4.0) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "turn timed out")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.yaw = yawTo_(float(err));
                status_ = fmt("turning to %03.0f", opAux_);
                s.missionPhase = "SCAN";
                return out(c);
            }

            case Op::TURN_REF: {
                Place ref;
                if (!resolve_(in.target, ref)) { next_(pc_ + 1); continue; }
                const double err = wrap180(ref.refYaw - s.vehYawDeg);
                if (std::fabs(err) < 4.0) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "turn timed out")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.yaw = yawTo_(float(err));
                status_ = fmt("turning back to %03.0f", ref.refYaw);
                s.missionPhase = "SCAN";
                return out(c);
            }

            case Op::CLIMB_BY: {
                if (first) opAux_ = std::max(double(p_.minAltM), double(s.vehAltM) + in.a);
                if (std::fabs(s.vehAltM - opAux_) < 0.15) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "height not reached in time")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.throttle = vertTo_(s, opAux_);
                status_ = std::string(in.a >= 0 ? "up" : "down") +
                          fmt(" to %.1f m (%.1f)", opAux_, s.vehAltM);
                s.missionPhase = "CLIMB";
                return out(c);
            }

            case Op::TRACK: {
                if (first) trackReqT_ = -1;
                // Locked on it since the request: the tracker has it.
                if (trackReqT_ >= 0 && tracking_(s, text) && s.targetStampS >= trackReqT_) {
                    status_ = "tracking '" + text + "'";
                    next_(pc_ + 1); continue;
                }
                if (opT_ > in.a) {
                    if (!fail(in, trackReqT_ < 0 ? "'" + text + "' never seen to track"
                                                 : "the tracker did not lock on '" + text + "'"))
                        return out(hover_());
                    continue;
                }
                if (trackReqT_ < 0 && s.tickMonoS - s.detStampS <= p_.detStaleSec) {
                    const Detection* best = nullptr;
                    for (const Detection& d : s.detections)
                        if (d.label == text && (!best || d.confidence > best->confidence)) best = &d;
                    if (best) {
                        s.trackRequestBox = best->box;
                        ++s.trackRequestSeq;
                        s.trackLabel = text;
                        trackReqT_ = s.tickMonoS;
                        tref_ = TrackRef();
                    }
                }
                status_ = trackReqT_ < 0 ? "waiting to see '" + text + "' to track it"
                                         : "handing '" + text + "' to the tracker";
                s.missionPhase = "SCAN";
                return out(hover_());
            }

            case Op::CLIMB: {
                // To a height, up or down -- never below the floor.
                const double want = std::max(double(p_.minAltM), double(in.a));
                if (std::fabs(s.vehAltM - want) < 0.2) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "height not reached in time")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.throttle = vertTo_(s, want);
                status_ = fmt("to %.1f m (%.1f)", want, s.vehAltM);
                s.missionPhase = "CLIMB";
                return out(c);
            }

            case Op::GOTO: {
                Place tg;
                if (!resolve_(in.target, tg)) {
                    if (!fail(in, "that place was never marked")) return out(hover_());
                    continue;
                }
                if (opT_ > in.b) {
                    if (!fail(in, fmt("goto timed out after %.0f s", in.b))) return out(hover_());
                    continue;
                }
                if (!s.estValid) {
                    noPosT_ += dt;
                    if (noPosT_ > p_.noPositionMaxS) {
                        if (!fail(in, "position estimate lost")) return out(hover_());
                        continue;
                    }
                    status_ = "goto: no position estimate -- hovering";
                    return out(hover_());
                }
                noPosT_ = 0;
                const double de = tg.e - s.estPe, dn = tg.n - s.estPn;
                const double dist = std::hypot(de, dn);
                // A HEIGHT too: above the ground, or above the place's own.
                const bool hasAlt = (in.flags & (kms::FLAG_ALT_ABS | kms::FLAG_ALT_REL)) != 0;
                const double wantU = !hasAlt ? 0.0
                    : std::max(double(p_.minAltM), (in.flags & kms::FLAG_ALT_ABS) ? double(in.c)
                                                                                    : tg.u + in.c);
                const double ev = hasAlt ? wantU - s.vehAltM : 0.0;
                if (dist <= in.a && std::fabs(ev) < 0.3) { endOp_(s); next_(pc_ + 1); continue; }
                if (dist <= in.a) {
                    // There across, not yet at height: the rest of the way is
                    // VERTICAL -- in certified mode, what the glide could not
                    // cover, and nothing checks it (the floor still holds).
                    if (missionOn_) { mission_.enable(false); missionOn_ = false; }
                    ControlCmd c = hover_();
                    c.throttle = vertTo_(s, wantU);
                    status_ = fmt("at it; height %.1f -> %.1f m (vertical, unchecked)", s.vehAltM, wantU);
                    s.missionPhase = "CLIMB";
                    return out(c);
                }
                if (!direct_ && mission_.phase() == MissionController::Phase::STUCK) {
                    if (!fail(in, "boxed in on the way (STUCK)")) return out(hover_());
                    continue;
                }
                const double brg = wrap360(std::atan2(de, dn) / kD2R);
                const double err = wrap180(brg - s.vehYawDeg);
                status_ = fmt("goto: %.1f m to go, bearing %03.0f", dist, brg);
                (void)err;
                // A target outside what the camera can certify: turn to face
                // it first (rotation keeps the map honest) -- between legs only.
                ControlCmd turn;
                if (faceFirst_(s, float(brg), turn)) { s.missionPhase = "SCAN"; return out(turn); }
                if (hasAlt && !direct_) {
                    // CERTIFIED: a glide toward the height, no steeper than
                    // maxGlideDeg -- certified along its slope by the map.
                    const double g = std::atan2(ev, dist) / kD2R;
                    s.missionGlideDeg = float(std::max(-double(p_.maxGlideDeg),
                                                       std::min(double(p_.maxGlideDeg), g)));
                }
                ControlCmd c = fly_(s, float(dt), true, float(brg), float(dist));
                if (hasAlt && direct_) c.throttle = vertTo_(s, wantU);   // DIRECT: together
                if (hasAlt) status_ += fmt(", height %.1f -> %.1f m", s.vehAltM, wantU);
                return out(c);
            }

            case Op::EXPLORE:
                if (opT_ >= in.a) { endOp_(s); next_(pc_ + 1); continue; }
                status_ = fmt("exploring %.0f of %.0f s", opT_, in.a);
                return out(fly_(s, float(dt), false, s.vehYawDeg, 0.f));

            case Op::SEARCH: {
                if (seen_(s, text, ctx, nullptr, nullptr, nullptr, (in.flags & kms::FLAG_NEW) != 0)) {
                    status_ = "found '" + text + "'";
                    next_(pc_ + 1); continue;
                }
                if (opT_ > in.a) {
                    if (!fail(in, "'" + text + "' not found")) return out(hover_());
                    continue;
                }
                ControlCmd c = hover_();
                c.yaw = float(in.b) * p_.searchYaw;
                status_ = "searching for '" + text + "'";
                s.missionPhase = "SCAN";
                return out(c);
            }

            case Op::FACE: {
                float off = 0;
                if (seen_(s, text, ctx, &off, nullptr, nullptr)) {
                    opAux_ = opT_;                   // last time it was seen
                    if (std::fabs(off) < 4.f) { next_(pc_ + 1); continue; }
                    ControlCmd c = hover_();
                    c.yaw = yawTo_(off);
                    status_ = "facing '" + text + "'" + fmt(" (%+.0f deg)", off);
                    s.missionPhase = "SCAN";
                    return out(c);
                }
                if (opT_ - opAux_ > 2.0 || opT_ > in.a) {
                    if (!fail(in, "lost '" + text + "'")) return out(hover_());
                    continue;
                }
                s.missionPhase = "SCAN";
                return out(hover_());
            }

            case Op::APPROACH: {
                float off = 0, fill = 0;
                if (opT_ > in.b) { if (!fail(in, "approach timed out")) return out(hover_()); continue; }
                if (seen_(s, text, ctx, &off, nullptr, &fill)) {
                    opAux_ = opT_;
                    if (fill >= in.a) { endOp_(s); status_ = "close to '" + text + "'"; next_(pc_ + 1); continue; }
                    // Onto it before each leg, as a goto turns onto its place.
                    const bool between = !missionOn_ ||
                                         mission_.phase() == MissionController::Phase::SETTLE;
                    if (between && std::fabs(off) > 12.f) {
                        ControlCmd c = hover_();
                        c.yaw = yawTo_(off);
                        s.missionPhase = "SCAN";
                        status_ = "approach: turning onto '" + text + "'";
                        return out(c);
                    }
                    status_ = "approaching '" + text + "'";
                    // Short certified legs, re-aimed at it after every one.
                    return out(fly_(s, float(dt), true, float(wrap360(s.vehYawDeg + off)), 1.5f));
                }
                if (opT_ - opAux_ > 3.0) {
                    if (!fail(in, "lost '" + text + "'")) return out(hover_());
                    continue;
                }
                return out(missionOn_ ? fly_(s, float(dt), s.missionGoalValid, s.missionGoalBearing,
                                             s.missionLegCapM)
                                      : hover_());
            }

            case Op::APPROACH_TO: {
                float off = 0, d = -1;
                if (opT_ > in.b) { if (!fail(in, "approach timed out")) return out(hover_()); continue; }
                if (seen_(s, text, ctx, &off, nullptr, nullptr, false, nullptr, &d)) {
                    opAux_ = opT_;
                    if (d > 0.f && d <= in.a + 0.15f) {
                        endOp_(s);
                        status_ = fmt("at %.1f m from '", d) + text + "'";
                        next_(pc_ + 1); continue;
                    }
                    const bool between = !missionOn_ ||
                                         mission_.phase() == MissionController::Phase::SETTLE;
                    if (between && std::fabs(off) > 12.f) {
                        ControlCmd c = hover_();
                        c.yaw = yawTo_(off);
                        s.missionPhase = "SCAN";
                        status_ = "approach: turning onto '" + text + "'";
                        return out(c);
                    }
                    if (!(d > 0.f)) {
                        // Seen, no range: nothing says when to stop, so it
                        // does not move -- and gives up after a while.
                        if (opT_ - opAux2_ > 3.0) {
                            if (!fail(in, "no range to '" + text + "' (no depth, no size)"))
                                return out(hover_());
                            continue;
                        }
                        status_ = "approach: '" + text + "' has no range yet";
                        return out(hover_());
                    }
                    opAux2_ = opT_;
                    status_ = "approaching '" + text + "'" + fmt(" %.1f m, stop at %.1f", d, in.a);
                    // A leg never longer than what is left to the stop.
                    const float cap = std::max(0.3f, std::min(1.5f, d - in.a));
                    return out(fly_(s, float(dt), true, float(wrap360(s.vehYawDeg + off)), cap));
                }
                if (opT_ - opAux_ > 3.0) {
                    if (!fail(in, "lost '" + text + "'")) return out(hover_());
                    continue;
                }
                return out(missionOn_ ? fly_(s, float(dt), s.missionGoalValid, s.missionGoalBearing,
                                             s.missionLegCapM)
                                      : hover_());
            }

            case Op::RUN: {
                if (first) {
                    delegate_ = lookup_ ? lookup_(text) : nullptr;
                    if (!delegate_) { if (!fail(in, "no mode '" + text + "'")) return out(hover_()); continue; }
                    delegate_->onEnter(s);
                    s.missionGo = true;            // a mission-like delegate is past GO already
                }
                if (opT_ >= in.a) { endOp_(s); next_(pc_ + 1); continue; }
                status_ = "running " + text + fmt(" (%.0f of %.0f s)", opT_, in.a);
                s.missionActive = false;            // its own phases, or none
                return out(delegate_->update(s, ctx));
            }

            case Op::WAIT:
                if (cond_(in.cond, s)) { next_(pc_ + 1); continue; }
                if (opT_ > in.a) {
                    if (in.jump >= 0) { next_(in.jump); continue; }
                    next_(pc_ + 1); continue;       // no else: carry on
                }
                status_ = fmt("waiting (%.0f of %.0f s)", opT_, in.a);
                s.missionPhase = "SCAN";
                return out(hover_());

            default:
                finish_(s, true, "STOPPED: unknown instruction");
                return out(hover_());
        }
    }
    status_ = "yielding (many instant steps)";
    return out(hover_());
}
