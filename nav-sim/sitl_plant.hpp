#pragma once
// ---------------------------------------------------------------------------
// SitlPlant -- ArduPilot SITL as FlightShow's airframe (`kestrel sitl`).
//
// The aircraft's own autonomy (VoxelNavModule, MissionController, ScriptMode)
// flies exactly as in `kestrel demo` and `mission sim`; what changes is the
// thing under it:
//
//   commands  ControlCmd -> onboard's MavlinkBackend -> ArduCopter, over the
//             same code that flies the aircraft (tcp:/udp: instead of a UART,
//             byte_link.hpp). GUIDED, so the velocity uplink: a full stick is
//             the kinematic plant's 4 m/s, 90 deg/s, 0.8 m/s climb.
//   pose      ArduPilot's SIMULATOR TRUTH (SIM_STATE), not its EKF: the
//             stereo pair and depth are rendered from where its physics put
//             the airframe. Origin = where it stood at start, mapped onto the
//             world's spawn point; heights from the floor slab.
//
// Not lockstep: SITL runs on its own clock and this one paces itself to the
// wall (kestrel_sitl.cpp), so a frame that takes too long to render is a late
// frame, as it would be aboard. SITL knows nothing of the world -- a wall is
// air to it -- so contact is judged here against the true voxels, the same
// clearance test FlightShow always used, and a run ends at the first one by
// default.
//
// Getting airborne (ready() false until then): wait for the link, the
// parameter readback and a truth fix; ask for GUIDED until the FC is in it;
// arm until it is armed (ArduPilot refuses until its EKF has a position, which
// takes SITL ~20-30 s from boot); NAV_TAKEOFF to the flight height; ready once
// within 85 % of it. Every step is retried and every FC STATUSTEXT printed, so
// a refusal says why.
// ---------------------------------------------------------------------------

#include <memory>
#include <string>

#include "flight_show.hpp"
#include "mavlink_backend.hpp"   // onboard

namespace ksitl {

struct SitlParams {
    std::string connect = "tcp:127.0.0.1:5760";   // ArduPilot SITL's SERIAL0
    float takeoffM = 1.5f;     // above the ground; FlightParams::altM
    double startTimeoutS = 120.0;
    bool verbose = true;
};

class SitlPlant : public kshow::Plant {
public:
    explicit SitlPlant(const SitlParams& p);
    ~SitlPlant() override;

    const char* name() const override { return "ArduPilot SITL"; }
    bool connect();                         // false: nothing at p.connect
    void start(const sim::CamPose& spawn, float floorZ) override;
    bool ready(sim::CamPose& truth) override;
    void step(const ControlCmd& c, float dt, sim::CamPose& truth, float& speed) override;
    void handOff(WorldState::FcRequest r) override;

    // LAND, and service the link until the FC is down and disarmed (or the
    // timeout). For the end of a run.
    void land(double timeoutS = 30.0);

    enum class Stage { LINK, GUIDED, ARM, TAKEOFF, FLYING, FAILED };
    Stage stage() const { return stage_; }
    const char* stageName() const;
    const std::string& failure() const { return failure_; }
    MavlinkBackend& fc() { return fc_; }

private:
    void service();                         // tick the link, echo FC text
    void truthToPose(sim::CamPose& out) const;

    SitlParams p_;
    MavlinkBackend fc_;
    Stage stage_ = Stage::LINK;
    std::string failure_;
    double t0_ = 0, stageT_ = 0, lastAskS_ = -1e9;
    long textSeen_ = 0;
    // Truth origin (the first fix) and where it lands in the world.
    bool originSet_ = false;
    double lat0_ = 0, lon0_ = 0;
    float alt0_ = 0;
    float spawnE_ = 0, spawnN_ = 0, floorZ_ = 0;
};

}  // namespace ksitl
