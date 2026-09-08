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
    // METRES TOWARD THE GOAL ARE THE POINT -- but as a FRACTION OF THE JOURNEY,
    // not as raw metres, or the big worlds drown the small ones.
    //
    // Progress telescopes to (start distance - end distance). In raw metres a
    // 340 m city journey therefore pays an order of magnitude more than a 30 m
    // maze for the same quality of flying, and measurement across the six
    // worlds put the spread of mean episode return at 7.6x: road 66.3 against
    // corridor 8.7. PPO normalises advantages per batch, but a world
    // contributing 7x larger advantages still dominates the update -- so the
    // open worlds would set the weights and the tight ones would be noise.
    // That is "open maps poisoning the policy", and it is arithmetic rather
    // than bad luck.
    //
    // Each step now pays wProgress * closed / startDist * progressScaleM, so
    // every world pays the same for closing the same FRACTION of its journey
    // and the maximum an episode can earn from progress is the same
    // everywhere: wProgress * progressScaleM. The scale is in metres purely so
    // the numbers stay in a familiar range -- a full journey pays what a 100 m
    // one used to.
    float wProgress   = 2.0f;
    float progressScaleM = 100.f;
    float wCoverage   = 0.15f;
    float wTime       = 0.01f;
    float wStop       = 0.05f;
    float wClear      = 0.20f;
    float clearTarget = 0.8f;
    // TERMINALS ARE A FLOOR, NOT THE VALUE. A fixed penalty cannot punish hard
    // in worlds of different size, and measurement said so plainly: progress
    // telescopes to (start - end) distance, so in the 175 m forest a policy
    // banked +47 before hitting a tree and still finished the episode POSITIVE
    // at +5.6, while the same -50 in the 35 m maze scored -48.7. One number
    // meant "barely a scratch" in one world and "catastrophe" in the other.
    //
    // The effective penalty is now whichever is larger: this floor, or a
    // multiple of every metre the episode could possibly have earned. That
    // makes crashing strictly worse than any amount of progress, in any world,
    // whatever wProgress is set to -- rather than depending on the goal
    // happening to be close enough.
    float rGoal       = 50.f;
    float rCollide    = 50.f;
    // Multiples of (wProgress * progressScaleM), the most progress an episode
    // can earn -- which is now the same number in every world, so these are too.
    // 1.5 for collision so it can never be bought; 0.5 for arrival so reaching
    // the goal keeps a clear edge over merely getting near it.
    float collideScale = 1.5f;
    float goalScale    = 0.5f;
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

    // VARY THE JOURNEY, not just the obstacles. Every episode used a fixed
    // start and a fixed goal: forest was always 175 m away on a bearing of
    // 36.9 deg, maze always 31.8 m at 45 deg, on every seed. Only the trees and
    // walls moved. A policy can score well on that by learning a compass
    // heading and never using the goal channels at all, and nothing in the
    // scorecard would notice -- so "it navigates" was never actually tested.
    //
    // With this, start and goal are sampled per episode (deterministically from
    // the seed) with real clearance and a minimum separation. It makes the task
    // strictly harder and the numbers not comparable with fixed-geometry runs,
    // which is why it is opt-in rather than the new default.
    bool  varyGoal    = false;
};

struct EnvStep {
    float reward = 0.f;
    bool  done = false, truncated = false;
    // Scorecard fields, so evaluation can emit the SAME columns sweep.sh does.
    // A comparison against the classical planners on new metrics is worthless.
    float travelM = 0.f, distToGoalM = 0.f, minClearM = 0.f;
    // CLOSEST APPROACH, and when. Final distance alone cannot tell "never got
    // there" from "got there and drifted off", and those are different
    // failures: one is navigation, the other is termination. A run that
    // reached 10.7 m at step 2400 and ended 52 m out reads as a failure to
    // navigate if you only look at the last number.
    float minDistToGoalM = 0.f;
    int   minDistStep = 0;
    int   collisions = 0, stoppedSteps = 0, steps = 0;
    bool  reachedGoal = false;

    // WHERE THE REWARD ACTUALLY WENT, accumulated over the episode. A scalar
    // return says a policy improved; it cannot say which term it improved, and
    // when reward rises while the scorecard falls that is the only question
    // worth asking. These are diagnostics -- nothing reads them back.
    float rProgress = 0.f;   // closing distance to goal (telescopes to start-end)
    float rCoverage = 0.f;   // newly visited cells
    float rTime     = 0.f;   // per-step cost of existing
    float rStop     = 0.f;   // standing still, or picking a masked action
    float rClear    = 0.f;   // flying closer to things than clearTarget
    float rTerminal = 0.f;   // the goal bonus or the collision penalty
};

// The classical planners the learned one is measured against. ONE definition,
// shared by `kestrel bench` and by the python evaluator through the extension
// module -- a second copy in python would be free to drift, and a comparison
// against a baseline that is not the baseline is worth nothing.
enum class BaselinePolicy { Random = 0, FreeM, Goal, Score };
const char* baselineName(BaselinePolicy p);

// rng is advanced in place; only Random uses it. Returns a primitive index,
// always one the mask admits when any is admitted.
int chooseBaseline(BaselinePolicy pol, const std::vector<float>& obs,
                   const std::vector<uint8_t>& mask, int nPrims, unsigned& rng);

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
