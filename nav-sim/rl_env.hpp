#pragma once

#include <memory>
#include <string>
#include <vector>

#include "bearing_field.hpp"
#include "depth_camera.hpp"
#include "voxel_map.hpp"
#include "voxel_traj.hpp"
#include "voxel_world.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// VoxelEnv -- the RL environment, in C++ where the sim already is.
//
// THE POLICY RANKS PRIMITIVES; IT DOES NOT FLY. The action is an index into the
// trajectory library, and only primitives sphereClear has ALREADY ADMITTED are
// selectable. The hard veto stays geometric, so a learned policy can advise
// among options the geometry approved and can never grant permission of its
// own. A bad policy picks a worse admissible primitive; it cannot pick an
// inadmissible one.
//
// That is not a safety nicety, it is the whole reason this is worth doing on
// this stack rather than copying a PyBullet end-to-end controller: the failure
// mode where a learned model steers into space nothing measured is structurally
// unreachable here.
//
// EVERY SPATIAL CHANNEL CARRIES A MASK. A single scalar cannot distinguish
// "clear" from "unmeasured", and a policy trained where ground truth is
// complete will never discover the difference -- it will simply learn that the
// value is always trustworthy, and then meet a real sensor. Same rule as
// three-state occupancy, one level up. See obsFeaturesPerPrim().
//
// THE ENV OWNS THE SENSING LOOP. Depth is rendered and integrated every step,
// so the map the policy sees is one it built, with the real dropout, Z_max and
// occlusion of the stereo model. Training against a preloaded map trains a
// KNOWN-MAP planner, and partial observability is the entire problem.
// ---------------------------------------------------------------------------

struct EnvConfig {
    std::string world = "forest";
    unsigned    seed  = 1;
    int   maxSteps    = 1500;
    float dt          = 0.1f;
    float cell        = 0.25f;
    int   camW        = 160, camH = 120;   // training res; validate at 320x240
    bool  truthDepth  = false;             // cheap render for stage 1
    float robotR      = 0.6f;
    float horizonS    = 0.6f;
    // Visit-count memory. The maze failure is that a stateless planner
    // re-derives the same local preference every time it returns to a junction.
    float visitCellM  = 1.0f;
    float visitDecay  = 0.999f;
    // Reward weights. Distance is split into PROGRESS and COVERAGE on purpose:
    // rewarding raw distance travelled pays a policy that orbits forever.
    float wProgress   = 1.0f;
    float wCoverage   = 0.15f;
    float wTime       = 0.01f;
    float wStop       = 0.05f;
    float wClear      = 0.20f;
    float clearTarget = 0.8f;
    float rGoal       = 50.f;
    float rCollide    = 50.f;
    float goalTolM    = 3.0f;

    // THE VETO, AS AN EXPERIMENT RATHER THAN AN ASSUMPTION.
    //
    // Normally sphereClear marks a primitive inadmissible and the action mask
    // hides it, so the policy never even offers an action that flies into
    // something it can see. It learns PREFERENCE among options geometry has
    // already approved, and it cannot collide by choosing -- which is the whole
    // safety argument of this architecture, and also the reason a trained
    // policy here has never once hit a wall on purpose.
    //
    // With maskUnsafe=false every primitive is selectable. The policy can fly
    // straight into a trunk, take -rCollide, end the episode, and has to learn
    // avoidance from the consequence instead of being handed it. That is the
    // "train from zero, hit walls, work it out" experiment.
    //
    // It is a MEASUREMENT, not a deployment mode. The point of running it is to
    // put a number on what the veto is worth: if a policy that had to learn
    // avoidance matches one that was given it, the mask is buying nothing but
    // sample efficiency; if it does not, the gap is the argument for keeping a
    // hard geometric veto under a learned planner.
    bool  maskUnsafe  = true;
};

struct EnvStep {
    float reward = 0.f;
    bool  done = false, truncated = false;
    // Scorecard fields, so evaluation can emit the SAME columns sweep.sh does.
    // A comparison against the classical planners on new metrics is worthless.
    float travelM = 0.f, distToGoalM = 0.f, minClearM = 0.f;
    int   collisions = 0, stoppedSteps = 0, steps = 0;
    bool  reachedGoal = false;
};

class VoxelEnv {
public:
    explicit VoxelEnv(const EnvConfig& c = {});
    ~VoxelEnv();

    void  reset(const std::string& world, unsigned seed);
    EnvStep step(int primitiveIndex);

    // Flat observation: [nPrims * featuresPerPrim] then [globalFeatures].
    const std::vector<float>& observation() const { return obs_; }
    // 1 where the primitive is admissible this step, 0 otherwise. A policy MUST
    // mask its logits with this; selecting a masked index is treated as "hold".
    const std::vector<uint8_t>& actionMask() const { return mask_; }

    int nPrims() const;
    static int obsFeaturesPerPrim() { return 9; }
    static int obsGlobalFeatures()  { return 24; }
    int obsSize() const { return nPrims() * obsFeaturesPerPrim() + obsGlobalFeatures(); }

    const EnvStep& last() const { return last_; }
    const EnvConfig& config() const { return cfg_; }

    // --- watching a run -----------------------------------------------------
    // One small pane of what the aircraft BELIEVES: by default the first-person
    // voxel view, the same renderLadder the sim labels "VOXEL FPV". The map and
    // not the world on purpose -- what the policy steers on is what is worth
    // looking at, and a picture of the true world would hide exactly the
    // mistakes worth seeing (pale is UNKNOWN, and unknown is not free).
    //
    // Returns tightly packed BGR bytes, w*h*3, so the caller needs no OpenCV
    // type in its interface -- this header stays free of cv::Mat and the python
    // binding can hand numpy a buffer without a copy through a Mat.
    // topDown swaps the FPV for the plan view, which is the better picture for
    // judging whether a run actually went anywhere.
    std::vector<uint8_t> renderFrame(int w = 320, int h = 240,
                                     bool topDown = false) const;

private:
    struct Impl;
    std::unique_ptr<Impl> im_;
    EnvConfig cfg_;
    std::vector<float>   obs_;
    std::vector<uint8_t> mask_;
    EnvStep last_;

    void buildObservation();
};

}  // namespace sim
