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
    // WHAT THE POLICY IS FOR. The simulator has a goal in it and everything
    // here was scored on reaching one, which was the wrong target: the goal is
    // scaffolding. What is wanted is an aircraft that keeps flying, gets away
    // from where it started, covers ground, and does not hit anything.
    //
    // GOAL  -- the old objective. Pays for closing distance to one point, pays
    //          a bonus for arriving, and ends the episode there.
    // RANGE -- pays for DISPLACEMENT from the spawn and for NEW GROUND, and the
    //          goal is ignored entirely (the episode does not end on it).
    //
    // Why displacement and not path length: the greedy openness planner freeM
    // flies 81.6 m of a 90 m ceiling with zero collisions and ends 9.9 m from
    // where it started -- a path 8.2x its own displacement. It is circling a
    // safe clearing and banking metres. Paying for path length makes that the
    // optimum; paying for displacement makes it worth nothing, because a loop
    // returns you to where you began.
    //
    // Displacement alone would reward one straight dash and then hovering at
    // the far end, so coverage pays for new ground alongside it. Between them:
    // go somewhere, keep going somewhere new, do not come back, do not hover.
    enum Objective { GOAL = 0, RANGE = 1 };
    int   objective   = GOAL;
    // Per metre of net displacement GAINED. Telescopes to the final
    // displacement, so it is path-independent by construction -- a detour round
    // a tree costs nothing beyond the time it takes.
    float wRange      = 2.0f;
    // Both range and coverage are scaled by rangeScaleM / worldSpan, so a full
    // traverse of a 60 m maze and of a 300 m city pay the same. Without it the
    // big worlds would set the weights and the tight ones would be noise --
    // the same arithmetic that made progress scale-free.
    float rangeScaleM = 100.f;
    // FLY WHERE YOU HAVE LOOKED. Every collision measured in this tree is into
    // UNMAPPED space -- across 108 held-out episodes and five planners, not one
    // was into a cell the map had already marked OCCUPIED. The geometric veto
    // has never failed; it simply cannot veto what nothing has seen.
    //
    // Speed is not the lever either. The `score` baseline flies 0.081 m/step
    // and collides in 18 of 18 episodes; freeM flies 0.082 m/step and collides
    // in none. What separates them is that freeM maximises CONFIRMED-FREE path
    // length -- it goes fast only where it has looked.
    //
    // This charges for the part of the chosen primitive's rollout that was not
    // confirmed free, which is the one thing the policy is never paid to care
    // about. 0 disables it.
    float wSeen       = 0.f;
    // DON'T LOITER. Measured over 20,000-step episodes, the policy has exactly
    // two behaviours: explore briefly and die, or stop exploring and live
    // forever. Five of seven survivors found their last new cell inside the
    // first 800 steps and then orbited for the remaining 96% of the flight --
    // forest/101 flew 821 m and covered 19 distinct cells.
    //
    // It is behaving OPTIMALLY for the reward it was given. Coverage pays
    // 0.15 * (100/44) = +0.34 per new cell in the maze; collisions run at ~5
    // per 1125 new cells and cost 300, so exploring one new cell is worth
    // +0.34 and costs 0.0044 * 300 = 1.33 in expectation. Net -0.99. Hovering
    // is worth 0. Zero beats negative, so it parks.
    //
    // This charges for every step spent in a cell already visited, which turns
    // hovering from free into costly. At ~20 steps per new cell during genuine
    // exploration, 0.05/step makes exploring the better bet by the same
    // arithmetic that currently forbids it.
    float wRevisit    = 0.f;
    // DISTANCE TIMES NEW GROUND, as one term rather than two additive ones.
    // Coverage pays the same for a cell 2 m from the spawn as for one 100 m
    // out, so "explore a ring around home" scores identically to "go
    // somewhere". With this, a new cell is worth (1 + wFar * disp/span) times
    // as much, so the two things being asked for cannot be satisfied
    // separately. 0 leaves coverage flat.
    float wFar        = 0.f;
    float wProgress   = 2.0f;
    float progressScaleM = 100.f;
    float wCoverage   = 0.15f;
    float wTime       = 0.01f;
    float wStop       = 0.05f;
    float wClear      = 0.20f;
    float clearTarget = 0.8f;
    // THE CLEARANCE PENALTY IS THE ONLY THING THAT PUSHES AWAY FROM AN OBSTACLE
    // BEFORE CONTACT. rCollide is a cliff -- it arrives once, at the end, after
    // the mistake is unrecoverable. This term is the gradient: it costs a
    // little, every step, for flying closer to something than clearTarget.
    //
    // It was left in absolute units when progress was made scale-free, and that
    // reintroduced exactly the disease the progress fix cured. Progress now
    // pays wProgress * closed * (progressScaleM / startDist), so a step of it
    // is worth ~0.17 in the 175 m forest and ~0.86 in a 35 m maze, while a
    // near-miss cost a flat ~0.16 in both. Against progress the avoidance
    // signal was therefore about FIVE TIMES WEAKER in the tight worlds than in
    // the open ones -- the wrong way round, since tight is where clearance is
    // the whole problem.
    //
    // Scaled by the same factor, a near-miss costs the same relative to a step
    // of progress in every world. Set false to restore the old absolute term
    // for comparison with runs made before this existed.
    bool  scaleClear  = true;
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
    // ITS OWN COLUMN. This was folded into rClear, so the one diagnostic built
    // to say WHICH term drove an episode was conflating two of them -- and it
    // mattered immediately: the combined figure read -142.49 against a
    // displacement reward of +97.81, and only separating them shows that
    // almost all of it was this.
    float rSeen     = 0.f;   // flying through space nothing confirmed free
    float rRevisit  = 0.f;   // steps spent in ground already covered
    float rTerminal = 0.f;   // the goal bonus or the collision penalty

    // WHAT MAKES "OUT OF STEPS" READABLE. One truncation column covered at
    // least five different failures -- never left the spawn, orbited in place,
    // flew the wrong way, got close and drifted off, was still closing when
    // cut off -- and they need completely different fixes. The fields above
    // already separate some of them; these four separate the rest.
    //
    // startDistM  the journey. Without it "end-dist 177.8" is unreadable: only
    //             knowing the forest journey is 175.0 makes that row say "ended
    //             further out than it started".
    // netDispM    straight-line distance from the spawn. travelM alone cannot
    //             tell a policy orbiting its spawn from one that flew 100 m
    //             sideways, because distToGoalM only measures motion along the
    //             goal axis.
    // cellsVisited how much distinct ground was covered, without the reader
    //             having to divide rCoverage by a weight that a flag can change.
    // hitUnknown  on a collision: was the cell flown into UNKNOWN in the map the
    //             policy had built, or OCCUPIED? With the veto on, hitting
    //             something already mapped should be impossible, so this splits
    //             "the veto has a bug" from "the veto was blind" -- the honest
    //             failure, whose fix is sensing range and not the policy.
    float startDistM  = 0.f;
    float netDispM    = 0.f;
    int   cellsVisited = 0;
    bool  hitUnknown  = false;
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
    // WHERE IT IS AND WHERE IT IS GOING. Nothing outside the C++ renderer could
    // see either, so no tool could draw a trail over the world it was flown in
    // -- the one picture that says whether a world fails at one choke point or
    // everywhere. Metres, world frame, east/north/up.
    void position(float& e, float& n, float& u) const;
    void goal(float& e, float& n, float& u) const;

    // IS THERE A PATH AT ALL? journey_fit already asks whether the goal fits in
    // the step budget, which is a question about DISTANCE. This is the other
    // one, and nothing in this project has ever asked it: a flood fill from the
    // spawn over cells with at least robotR of true clearance, reporting
    // whether the goal is in the same connected component and how long the
    // shortest such path is.
    //
    // It is asked because of a picture. Fifteen held-out maze episodes flown by
    // five completely different planners -- openness-seeking, goal-seeking,
    // weighted-score, uniform random and a learned network -- all ran the same
    // corridor, turned at the same wall and stopped, with the goal ring
    // somewhere none of them went. Five objectives do not agree on a wrong turn
    // by coincidence. Unreachable goals have already cost this project three
    // times (the 1500-step forest cap, the 368 m city goal, the 600-step bench
    // default), and every one of them was found late because nothing checked.
    //
    // Returns the shortest free-space path length in metres; -1 if the goal is
    // not reachable from the spawn at all; -2 if the check was not attempted
    // because the lattice would be too large. "No route" and "not checked" are
    // different answers and must not share a value.
    float goalPathM(float cellM = 1.0f) const;
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

    // WHAT THE CAMERA RETURNED this step, colourised the way the sim's DEPTH
    // pane does it: near warm, far cool, and GREY MEANS NO RETURN rather than
    // far away. Worth having beside the FPV view because the FPV is mostly fog
    // in an open world and correctly so -- at 0.25 m voxels the map can only
    // honestly mark obstacles to about 3.5 m, so watching it alone tells you
    // very little about whether the sensor is seeing anything at all.
    std::vector<uint8_t> renderDepth(int w = 320, int h = 240) const;

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
