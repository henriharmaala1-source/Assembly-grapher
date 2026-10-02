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
                if (s.tickMonoS - s.detStampS <= p_.detStaleSec)
                    for (const Detection& d : s.detections)
                        any |= d.label == prog_.strings[size_t(o.arg)];
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
                const bool vis = seen_(s, prog_.strings[size_t(o.arg)], ctx_, nullptr, &d, nullptr);
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
                       float* offDeg, float* distM, float* fill) const {
    if (ctx.frameW <= 0 || ctx.frameH <= 0) return false;
    const bool detFresh = s.tickMonoS - s.detStampS <= p_.detStaleSec;
    const Detection* best = nullptr;
    if (detFresh)
        for (const Detection& d : s.detections)
            if (d.label == label && (!best || d.confidence > best->confidence)) best = &d;
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
    double los = -1.0;
    cv::Rect box;
    if (tracking_(s, label)) {
        // THE TRACKER'S BOX, fresh every frame, for the bearing. Range through
        // its scale change, anchored on the detector (TrackRef).
        box = s.targetBox;
        const bool anchor = best && (best->box & s.targetBox).area() > 0;
        // The tracker has no ground contact point, so an assumed size is
        // better than nothing for anchoring it.
        const double dl = anchor ? losOf(*best, true) : -1.0;
        if (dl > 0.0) { tref_.valid = true; tref_.losM = dl; tref_.size0 = float(s.targetBox.height); }
        // Only cores that estimate SCALE can carry it between detections;
        // a fixed-size box would freeze the range at its anchor, silently.
        const std::string core = s.targetCore ? s.targetCore : "";
        const bool scales = core == "fused" || core == "csrt";
        if (dl > 0.0) los = dl;
        else if (tref_.valid && scales && s.targetBox.height > 0)
            los = tref_.losM * tref_.size0 / double(s.targetBox.height);
    } else if (best) {
        box = best->box;
        los = losOf(*best, false);
    } else {
        return false;
    }
    const double cx = box.x + box.width * 0.5 - ctx.frameW * 0.5;
    if (offDeg) *offDeg = float(std::atan(cx / f) / kD2R);
    if (fill) *fill = float(box.height) / float(ctx.frameH);
    if (distM) {
        *distM = -1.f;
        const double cy = box.y + box.height * 0.5 - ctx.frameH * 0.5;
        const double elCentre = p_.detTiltDeg - std::atan(cy / f) / kD2R;
        if (los > 0.0) {
            *distM = float(los * std::cos(elCentre * kD2R));     // its horizontal part
        } else {
            // THE GROUND PLANE, for something standing on it: the box's
            // bottom edge is where it meets the ground, so its depression
            // below the horizon and the altitude give the distance. Refused
            // near the horizon, where it explodes, and without an altitude.
            const double yb = box.y + box.height - ctx.frameH * 0.5;
            const double dep = -(p_.detTiltDeg - std::atan(yb / f) / kD2R);   // + below
            if (s.vehAltM > 0.5f && dep > 3.0) {
                const double d = s.vehAltM / std::tan(dep * kD2R);
                if (d <= p_.maxRangeM) *distM = float(d);
            }
            if (*distM < 0.f && best) {               // 4: the typical size, last
                const double la = losOf(*best, true);
                if (la > 0.0) *distM = float(la * std::cos(elCentre * kD2R));
            }
        }
    }
    return true;
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
    if (!missionOn_) { mission_.enable(true); missionOn_ = true; }
    s.missionGoalValid = goal;
    s.missionGoalBearing = bearing;
    s.missionLegCapM = goal ? capM : 0.f;
    s.missionGo = true;
    return mission_.update(s, dt);
}

bool ScriptMode::faceFirst_(const WorldState& s, float bearing, ControlCmd& c) {
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
}

void ScriptMode::next_(int pc) {
    pc_ = pc;
    opT_ = 0; opAux_ = 0; opAux2_ = 0; opBegun_ = false; facing_ = false;
}

void ScriptMode::finish_(WorldState& s, bool failed, const std::string& why) {
    endOp_(s);
    finished_ = true;
    failed_ = failed;
    status_ = why;
}

ControlCmd ScriptMode::update(WorldState& s, const ControlCtx& ctx) {
    ctx_ = ctx;
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
                state_ = tr.to;
                next_(prog_.states[size_t(tr.to)].pc);
                status_ = "-> state " + stateName() + " (line " + std::to_string(tr.line) + ")";
                std::printf("[script] %s\n", status_.c_str());
                break;
            }
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
                p.set = true; p.e = s.estPe; p.n = s.estPn; p.refYaw = s.vehYawDeg;
                next_(pc_ + 1);
                continue;
            }
            case Op::MARK_SEEN: {
                float off = 0, d = -1;
                if (!s.estValid) { if (!fail(in, "no position to place it from")) return out(hover_()); continue; }
                if (!seen_(s, text, ctx, &off, &d, nullptr)) {
                    if (!fail(in, "'" + text + "' is not in view")) return out(hover_());
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
                status_ = fmt("marked it %.1f m away on %03.0f", d, brg);
                next_(pc_ + 1);
                continue;
            }
            case Op::GO_STATE:
                endOp_(s);
                inHandler_ = false;              // a handler that goes somewhere is done
                state_ = in.target;
                next_(prog_.states[size_t(state_)].pc);
                status_ = "-> state " + stateName();
                continue;
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
                if (first) opAux_ = s.vehAltM + in.a;
                if (s.vehAltM >= opAux_ - 0.15) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "climb timed out")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.throttle = p_.climbThrottle;
                status_ = fmt("up to %.1f m (%.1f)", opAux_, s.vehAltM);
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
                if (s.vehAltM >= in.a - 0.2f) { next_(pc_ + 1); continue; }
                if (opT_ > in.b) { if (!fail(in, "climb timed out")) return out(hover_()); continue; }
                ControlCmd c = hover_();
                c.throttle = p_.climbThrottle;
                status_ = fmt("climbing to %.1f m (%.1f)", in.a, s.vehAltM);
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
                if (dist <= in.a) { endOp_(s); next_(pc_ + 1); continue; }
                if (mission_.phase() == MissionController::Phase::STUCK) {
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
                return out(fly_(s, float(dt), true, float(brg), float(dist)));
            }

            case Op::EXPLORE:
                if (opT_ >= in.a) { endOp_(s); next_(pc_ + 1); continue; }
                status_ = fmt("exploring %.0f of %.0f s", opT_, in.a);
                return out(fly_(s, float(dt), false, s.vehYawDeg, 0.f));

            case Op::SEARCH: {
                if (seen_(s, text, ctx, nullptr, nullptr, nullptr)) {
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
                if (seen_(s, text, ctx, &off, &d, nullptr)) {
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
