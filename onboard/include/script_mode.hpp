#pragma once
// ---------------------------------------------------------------------------
// SCRIPT -- the control mode that flies a compiled mission (mission_program.hpp).
//
// It interprets a program the compiler already checked; it never sees text.
// Each tick it runs instructions until one takes time (a goto, a hold, a
// search), and returns that one's command. It invents no flying of its own:
//
//   goto / over / orbit / survey / explore / approach
//            the move-stop-sense MissionController, the same one AUTONOMY
//            runs, with a GOAL (WorldState::missionGoalBearing +
//            missionGoalValid) and a leg cap so it stops on the target. Every
//            leg is still certified by whichever planner the aircraft has --
//            voxel map, corridor, occupancy grid -- exactly as in AUTONOMY.
//   run MODE for T
//            any other registered mode (FOLLOW_ROAD, AUTONOMY, ...), driven
//            for T seconds -- the manager's safety layers wrap it as usual.
//   land / rtl
//            the flight controller's own LAND / RTL (WorldState::fcRequest).
//   turn / search / face / climb / hold
//            in place: yaw, a climb, a hover.
//
// It STOPS -- hovers and says why on scriptStatus -- rather than guess: a
// failure with no `else` (a goto that timed out, an object it could not
// range), the fence, the mission timeout, or a lost position estimate for too
// long. The ModeManager's failsafe (low battery, operator abort -> RTH) sits
// above all of it, unchanged.
//
// Nothing happens before GO (WorldState::missionGo), like AUTONOMY: select
// SCRIPT, check the status line, then GO. GO off pauses it in a hover.
// ---------------------------------------------------------------------------

#include <functional>
#include <string>
#include <vector>

#include "control_mode.hpp"
#include "mission.hpp"
#include "mission_program.hpp"

class ScriptMode : public IControlMode {
public:
    struct Params {
        MissionController::Params mission;   // the cycle every goto flies
        // THE DETECTION CAMERA, for turning a box into a bearing and, with no
        // measured range, a ground-plane range: its horizontal FoV and its
        // pitch on the airframe (negative = looking down).
        float detHfovDeg   = 60.f;
        float detTiltDeg   = 0.f;
        float detStaleSec  = 1.0f;     // a detection older than this is not "seen"
        float maxRangeM    = 60.f;     // a ground-plane range beyond this is a guess
        float yawKp        = 1.2f;     // heading error -> yaw stick, per 90 deg
        float maxYaw       = 0.5f;     // yaw stick cap for turns, faces, searches
        float searchYaw    = 0.3f;
        float climbThrottle = 0.4f;
        float noPositionMaxS = 20.f;   // stop if the estimate is gone this long mid-goto
        // `nav direct`: forward stick at full speed, eased off over the last
        // few metres so it stops near the target rather than through it.
        float directPitch  = 0.35f;
        float directSlowM  = 4.f;
        float newRadiusM   = 3.f;      // `new`: farther than this from anything visited
        // A path's anchor follows each new measurement of its object by this
        // fraction per tick it is in view (a smoothed re-grounding, not a jump).
        float anchorGain   = 0.15f;
        // STEER: metres per second for a full forward stick (the airframe's
        // own number; the sim's is 4), and the roll stick per degree of aim
        // error when strafing.
        float mpsPerStick  = 4.f;
        float strafeKp     = 0.02f;
        // HEIGHT. Never commanded below minAltM (only `land` goes lower --
        // the forward camera cannot see what is under the aircraft); a
        // certified glide no steeper than maxGlideDeg; vertical stick per
        // metre of height error, capped at maxVert.
        float minAltM      = 1.0f;
        float maxGlideDeg  = 25.f;
        float altKp        = 0.8f;
        float maxVert      = 0.6f;
        float diveKp       = 0.08f;  // vertical stick per degree the aim sits below centre
        // THE CROSSHAIR LAW: vertical metres per second at full stick (the
        // airframe's own; ArduPilot PILOT_SPEED_UP/DN), for turning the
        // ray's climb into a stick.
        float vertMpsPerStick = 1.5f;
    };
    using ModeLookup = std::function<IControlMode*(const std::string&)>;

    ScriptMode();
    explicit ScriptMode(Params p, ModeLookup lookup = {});

    // Load a compiled program (the Pi: from a .kmb). Refuses a program that
    // names a mode this aircraft does not have, so the gap shows on the
    // ground, not at the `run` line.
    bool load(const kms::Program& prog, std::string* err);
    bool loadFile(const std::string& kmbPath, std::string* err);
    bool loaded() const { return loaded_; }
    const kms::Program& program() const { return prog_; }

    const char* name() const override { return "SCRIPT"; }
    bool isMotion() const override { return true; }
    // Its legs are the MissionController's, which keeps its own standoff; a
    // delegated mode decides for itself.
    bool ownsObstacleAvoidance() const override {
        return delegate_ ? delegate_->ownsObstacleAvoidance() : true;
    }
    Behavior heatBehavior() const override { return Behavior::NAVIGATE; }
    void onEnter(WorldState& s) override;
    void onExit(WorldState& s) override;
    ControlCmd update(WorldState& s, const ControlCtx& ctx) override;

    // For tests and the sim.
    bool finished() const { return finished_; }
    bool failed() const { return failed_; }
    int  pc() const { return pc_; }
    // The current state's name, or "" outside the state machine.
    std::string stateName() const {
        return state_ >= 0 ? prog_.strings[size_t(prog_.states[size_t(state_)].name)] : std::string();
    }
    const std::string& status() const { return status_; }
    // Where a target is now, if it can be said (the sim draws them).
    bool targetPos(int i, double& e, double& n) const;

private:
    // u: HEIGHT above the ground (the vehAltM frame) -- for a place marked
    // from the camera, the top of the object; for `here`, where it was flown.
    struct Place { bool set = false; double e = 0, n = 0, refYaw = 0, u = 0; };

    void reset_();
    void start_(const WorldState& s);
    bool resolve_(int i, Place& out) const;
    bool cond_(int c, const WorldState& s) const;
    ControlCmd hover_() const { ControlCmd c; c.valid = true; return c; }
    void finish_(WorldState& s, bool failed, const std::string& why);
    void next_(int pc);
    void endOp_(WorldState& s);
    // The best fresh detection of `label`, its bearing off the nose (deg, +
    // right) and, if it can be had, its horizontal distance.
    // The tracker's box is preferred when it is locked on `label` (every
    // frame, cheap); the detector's otherwise.
    // onlyNew: skip objects whose ground position is within newRadiusM of a
    // place already marked for this label (`new`).
    bool seen_(const WorldState& s, const std::string& label, const ControlCtx& ctx,
               float* offDeg, float* distM, float* fill, bool onlyNew = false,
               cv::Rect* boxOut = nullptr, float* losM = nullptr) const;
    bool tracking_(const WorldState& s, const std::string& label) const;
    // Metres tall, or -1; *assumed says the compiler filled it in.
    float sizeOf_(const std::string& label, bool* assumed = nullptr) const;

    // RANGE THROUGH THE TRACKER. A detection gives an absolute range (depth,
    // or its known height over its box height); the tracker's box is square
    // and its own, so its SIZE means nothing absolute -- but its CHANGE does:
    // range = range0 * size0 / size. Anchored whenever a fresh detection lands
    // on the tracked box, so tracker scale drift is reset each time YOLO runs.
    struct TrackRef { bool valid = false; double losM = 0; float size0 = 0; };
    mutable TrackRef tref_;
    ControlCtx ctx_;          // the last tick's frame size, for conditions
    float yawTo_(float errDeg) const;
    // One tick of a mission-cycle leg toward `bearing` capped at `capM`
    // (capM <= 0, goal off: explore).
    ControlCmd fly_(WorldState& s, float dt, bool goal, float bearing, float capM);
    // Turn onto `bearing` first, if it is outside what the camera can certify
    // -- but only BETWEEN legs (op start, or the cycle settling after one),
    // never mid-leg and never while the cycle is looking round (THINK/SCAN):
    // fighting its scan reset it every few seconds and flew nothing.
    bool faceFirst_(const WorldState& s, float bearing, ControlCmd& c);

    Params p_;
    ModeLookup lookup_;
    kms::Program prog_;
    bool loaded_ = false;
    MissionController mission_;
    bool missionOn_ = false;
    IControlMode* delegate_ = nullptr;

    bool started_ = false, finished_ = false, failed_ = false;
    int pc_ = 0, savedPc_ = -1;
    int state_ = -1;          // current state, -1 = none (top level)
    double trackReqT_ = -1;   // when this TRACK asked for a lock
    bool inHandler_ = false;
    std::vector<bool> fired_;
    std::vector<double> regs_;
    std::vector<Place> places_;
    struct Visited { std::string label; double e, n; };
    std::vector<Visited> visited_;   // every object a `seen` marked, for `new`
    // THE ACTIVE PATH ANCHOR: which place, and what object re-grounds it.
    int anchorT_ = -1;
    std::string anchorLabel_;
    // Where the anchor's object is now (e/n), measured from the camera: for
    // re-grounding, and for a path's first point.
    bool measureObject_(const WorldState& s, const std::string& label, const ControlCtx& ctx,
                        double& e, double& n, double& bearing, double* topU = nullptr) const;
    // Height of a box's top edge above the ground, `distM` away.
    double topHeight_(const WorldState& s, const ControlCtx& ctx, const cv::Rect& box,
                      double distM) const;
    // Vertical stick toward a height, the floor applied.
    float vertTo_(const WorldState& s, double heightM) const;
    // THE CROSSHAIR LAW, shared by `fly crosshair` and `steer ... aim up`:
    // a point in the image at (px, py) pixels from the centre (+ right,
    // + down) and a throttle 0..1 -> fly along the 3D ray through it.
    ControlCmd crosshair_(const WorldState& s, const ControlCtx& ctx, double px, double py,
                          float throttle) const;
    double t_ = 0;           // mission seconds since GO (paused time excluded)
    double opT_ = 0;         // seconds in the current instruction
    double opAux_ = 0;       // per-op scratch (a turn's goal heading, last-seen time)
    double opAux2_ = 0;      // ...and a second (last time a range was had)
    double lastRange_ = -1, rangeRate_ = 0;   // FOLLOW: the object's own closing speed
    double lastE_ = 0, lastN_ = 0;
    bool opBegun_ = false;
    bool facing_ = false;     // turning onto a goal, until within 5 deg
    bool direct_ = false;     // `nav direct`: no certified legs
    double noPosT_ = 0;
    std::string status_;
};
