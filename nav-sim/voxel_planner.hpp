#pragma once

#include <array>
#include <vector>

#include "voxel_map.hpp"

// ---------------------------------------------------------------------------
// Two planners over the voxel map, for two genuinely different jobs.
//
// GeneralPlanner  — "go roughly that way, don't hit anything."
//     A 3D openness histogram over azimuth x elevation, scored against a goal
//     direction with hysteresis toward the previous choice. Reactive, runs every
//     frame, needs no path and no goal position — only a direction. This is the
//     layer that keeps the aircraft alive, and it is deliberately the one that
//     governs SPEED, because it is the one reading the live sensor.
//
// PrecisePlanner  — "fly to that point."
//     3D A* over a coarsened copy of the map, with obstacle inflation and an
//     explicit cost for traversing UNKNOWN. Runs occasionally, produces a
//     waypoint list. Its output is a SUGGESTION, handed to the general planner
//     as a direction to prefer.
//
// THE ARBITRATION RULE, which is the whole safety argument:
//     The precise planner never commands the vehicle. It only changes which
//     direction the general planner is biased toward. So a stale, wrong or
//     empty path can slow the aircraft down or send it the long way round, but
//     it can never drive it into something the live map can see. This mirrors
//     the rule the 2D stack in this directory already follows, and it is the
//     reason a map built from noisy stereo is safe to use at all.
//
// UNKNOWN IS NOT FREE, and it is not blocked either. Treating it as free flies
// you into untextured walls; treating it as blocked means you can never enter
// space you have not already seen, which on a forward-facing camera means you
// can never move. Both planners therefore price it: traversable, expensive.
// ---------------------------------------------------------------------------

#include <cmath>

namespace sim {

// --- general (reactive direction) planner -----------------------------------

struct GeneralParams {
    // 48 bins is 7.5 deg -> 0.65 m between rays at 5 m, wider than a forest
    // trunk. Raised, and combined with the swept-sphere test in plan().
    int   nAz         = 96;      // azimuth bins over 360 deg
    int   nEl         = 9;       // elevation bins
    float elMinDeg    = -35.f;
    float elMaxDeg    =  35.f;
    float horizonM    = 12.f;    // how far to probe
    float sweepM      = 7.f;     // out to here, probe the robot's full width
    float robotR      = 0.6f;    // clearance the aircraft needs
    float unknownCost = 0.45f;   // an unknown cell counts as this fraction of a
                                 // real obstacle when scoring openness
    // Direction must be scored on BOTH openness and confirmed-free room. See
    // the comment in plan(): scoring on openness alone picks the most promising
    // direction and then discovers it has nowhere to move into.
    float freeWeight  = 1.4f;
    float goalWeight  = 1.0f;    // pull toward the requested direction
    // COMMITMENT. The planner used to re-run its 864-bin argmax every step --
    // 10 Hz, while moving 0.3 m per step. The scene only changes meaningfully
    // every ~1 s of travel, so it was re-deciding roughly ten times per real
    // change, and the winning bin flipped between gaps as each depth frame
    // landed. That is the "spinning between gaps" behaviour, and it is a
    // decision-RATE problem rather than a weight-tuning one.
    //
    // Hold the chosen heading for commitSteps unless it actually becomes
    // blocked, then re-decide.
    //
    // The length was swept rather than picked (commit_sweep.sh; forest, 600
    // steps, 4 seeds). Commitment is NOT free -- it buys smoothness with
    // progress -- so the frontier matters more than any single number:
    //
    //     hold   churn   reversals   advance            collisions
    //      0     30.01      7.6%     0.579 [0.49,0.68]      1/4
    //      1     18.89      2.2%     0.592 [0.57,0.61]      1/4   <- shipped
    //      2     17.09      1.3%     0.534 [0.52,0.56]      0/4
    //      3     13.65      1.3%     0.524 [0.48,0.61]      1/4
    //      5     11.78      0.5%     0.510 [0.47,0.54]      0/4
    //      8     10.18      0.2%     0.482 [0.44,0.50]      1/4
    //
    // One step is the only arm that DOMINATES no-commitment: less churn, fewer
    // reversals, more progress, and a much tighter spread across seeds. Every
    // longer hold from 2 upward is a trade, buying churn with advance at a
    // steadily worse rate -- going 1 -> 8 halves the churn and costs a fifth of
    // the progress.
    //
    // Halving the decision rate does most of the work, which is the useful
    // lesson: the problem was never that the planner decided badly, it was that
    // it decided again before its last decision had produced any motion.
    //
    // Collisions do not separate the arms at four seeds -- 1,1,0,1,0,1 is noise
    // at that count, and should not be read as a safety ordering.
    int   commitSteps = 1;       // 0.1 s at dt = 0.1
    float breakFreeM  = 1.2f;    // re-decide early if committed free run drops below this
    // ------------------------------------------------------------------
    // THE THREE MECHANISMS BELOW ARE OFF BY DEFAULT BECAUSE THEY WERE
    // MEASURED AND DID NOT WORK. They are kept, and kept switchable
    // (--ema / --dwell / --revpen on voxel_sim), because the measurement is
    // worth more than the code and the next person will otherwise think of
    // all three again. `ablate.sh` reproduces it.
    //
    // Forest, 600 steps, seeds 1-3, one factor at a time. "churn" is mean
    // |dyaw| per step; "advance" is net displacement / distance travelled:
    //
    //     arm                 churn   reversals   advance
    //     commit 0 (off)      26.92       5.9%      0.530
    //     commit 5            11.39       0.5%      0.531   <- shipped
    //     commit 5 + ema      10.94       0.4%      0.452
    //     commit 5 + dwell    10.93       0.5%      0.492
    //     commit 5 + revpen   11.20       0.4%      0.479
    //     all three           ~11         ~0.5%     0.507
    //
    // No collisions in any arm, 3/3 seeds.
    //
    // Commitment alone accounts for the entire churn improvement -- 58% less
    // yaw churn and 92% fewer reversals, at no cost to progress. None of the
    // other three removes any further churn, and all three cost advance.
    //
    // Read the spread before reading the means, though: across the three
    // baseline seeds advance ranged 0.460-0.642, a spread of 0.182, while the
    // largest gap between arm MEANS is 0.080. Three seeds cannot resolve a
    // 0.08 effect. So the honest statement is "no measurable benefit, possible
    // harm", not "these are 8% worse" -- and a bigger sweep is what would be
    // needed to say anything stronger.
    // ------------------------------------------------------------------

    // FIELD SMOOTHING. The reasoning was: with commitment alone the planner
    // still re-decided every 6th step and 39% of those re-decisions swung the
    // command more than 90 deg, so score on a time-averaged field instead of
    // the instantaneous one. The first guess at the cause -- stereo speckle
    // collapsing a direction's free run for one frame -- was refuted by the
    // perfect-depth control (34% on truth depth against 39% on stereo). The
    // variance is geometry sampled from a moving origin: translate 0.3 m
    // sideways in a stand with a 2.67 m mean gap and the azimuth of every clear
    // lane shifts. Real, but averaging it did not help.
    //
    // If it is ever switched back on, the safety argument holds and is worth
    // stating: the smoothed field only ever chooses a HEADING. Commanded SPEED
    // is computed from the raw, current, unsmoothed free run, so a genuinely
    // new obstacle cuts the speed on the frame it appears. Smooth what you
    // steer by, never what you brake by.
    float fieldEma    = 1.00f;   // 1 = no smoothing (the default); 0.3 was tried
    // A near-tie must not be able to flip the command -- the standard dwell
    // rule. Made no difference to churn on top of commitment, which in
    // hindsight is unsurprising: commitment already caps the decision rate, and
    // a dwell margin is a weaker version of the same idea.
    float switchMargin= 0.00f;   // 0 = off; 0.12 was tried
    // REVERSING AS A LAST RESORT. A linear angular cost does not say that: at
    // goalWeight 1.0 a 180 deg reversal costs 1.0 while the openness terms are
    // worth up to 2.4, so the widest lane in the plot can win even when it
    // points backwards. This charges extra on the deviation beyond 90 deg, to
    // take the NEAREST acceptable lane rather than the globally most open one
    // -- VFH+'s nearest-valley rule. Sound in principle; not measurable here,
    // because with commitment on, the aircraft rarely reverses anyway.
    float revPenalty  = 0.00f;   // 0 = off; 1.2 was tried
    float hystWeight  = 0.55f;   // stickiness toward last frame's choice; this
                                 // is what stops oscillation between two
                                 // equally-open gaps, and it matters more than
                                 // it looks like it should
    // 6 m/s was reckless for this environment. At 1200 stems/ha the typical gap
    // between trunks is 2.67 m, so 6 m/s crosses a gap in under half a second
    // while the map updates at 10 Hz. Skydio manages ~5 m/s in forest with SIX
    // cameras and far better perception; 3 m/s is the honest ceiling here.
    float vMax        = 3.0f;    // m/s
    float decelMs2    = 3.0f;    // usable deceleration
    float reactS      = 0.25f;   // sense+plan+actuate latency before decel starts
    float minFreeM    = 0.4f;    // below this confirmed-free range, hold
};

// Smallest signed difference between two bearings, degrees, in (-180, 180].
// Positive means `a` is CLOCKWISE of `b` -- to the RIGHT, since every bearing
// in this tree is clockwise from North.
//
// In the header rather than a file-static because three places now need it and
// two of them are not this file: the planner's own heading-hysteresis, and the
// first-person turn HUD, which would otherwise carry a second copy of a
// convention that inverts silently. `chase_turn_check` pins the wrap.
// fmod FIRST, then one correction. The obvious one-liner --
// `fmod(a - b + 540, 360) - 180` -- is only right while `a - b` stays inside
// +-540, because fmod keeps the sign of its argument. It was the planner's
// internal form and safe there, since both arguments come out of atan2 and are
// bounded. It is NOT safe for the turn HUD, whose second argument is a heading
// that accumulates: at yaw 3000 deg it returns -480 and the aircraft is told to
// turn the wrong way. Found by asserting the range over +-720 rather than by
// noticing it in a picture, which is the only way it was ever going to be found.
inline float angDiffDeg(float a, float b) {
    float d = std::fmod(a - b, 360.f);          // (-360, 360)
    if (d > 180.f) d -= 360.f;
    else if (d <= -180.f) d += 360.f;
    return d;                                    // (-180, 180]
}

struct GeneralResult {
    float azDeg = 0, elDeg = 0;  // chosen direction
    float speed = 0;             // commanded speed, m/s
    // TWO different ranges, and conflating them is a collision.
    //   openM — unknown-discounted openness. Used to CHOOSE a direction, because
    //           refusing to steer toward unseen space means never moving.
    //   freeM — contiguous CONFIRMED-FREE distance. Used to set SPEED, because
    //           the aircraft may only travel as fast as it can stop within what
    //           it has actually seen. Unknown space earns no speed at all.
    float openM = 0;
    float freeM = 0;
    bool  blocked = false;       // nothing acceptable anywhere -> hold
    // Which branch produced this heading. Logged, because "the aircraft is
    // spinning" is a symptom with at least three possible causes -- argmax
    // churn, the escape branch firing, and velocity-lag overshoot -- and they
    // need completely different fixes. Guessing which one it is has already
    // cost this project a day.
    enum Source { SCORED, HELD, ESCAPE, BLOCKED } src = SCORED;
};

class GeneralPlanner {
public:
    explicit GeneralPlanner(const GeneralParams& p = GeneralParams()) : p_(p) {}
    const GeneralParams& params() const { return p_; }

    // goalAz/goalEl: the direction you would like to go if the world were empty.
    GeneralResult plan(const VoxelMap& m, float px, float py, float pz,
                       float goalAzDeg, float goalElDeg);

    // Last openness field, for drawing. Row-major [el][az], metres.
    const std::vector<float>& field() const { return field_; }
    const std::vector<float>& freeField() const { return free_; }
    int nAz() const { return p_.nAz; }
    int nEl() const { return p_.nEl; }

private:
    // One direction's reach (unknown-discounted) and confirmed-free run.
    // Public-ish only in the sense that plan() calls it 864 times on a decision
    // step and exactly ONCE on a held step -- which is where the onboard budget
    // came from.
    void probe(const VoxelMap& m, float px, float py, float pz,
               float azDeg, float elDeg, float& reachOut, float& freeOut) const;
    float elForIndex(int i) const;

    GeneralParams p_;
    std::vector<float> field_, free_;
    // Time-averaged copies, used ONLY for direction scoring. Kept separate from
    // field_/free_ so that nothing which gates speed can accidentally read a
    // smoothed value -- the separation is the safety property, so it is a
    // separate buffer rather than a flag.
    std::vector<float> fieldS_, freeS_;
    float lastAz_ = 0, lastEl_ = 0;
    bool  haveLast_ = false;
    int   held_ = 0;             // steps the current heading has been held
};

// --- precise (A* to a point) planner ----------------------------------------

struct PreciseParams {
    int   coarsen      = 2;     // plan on every Nth voxel; 0.25 m -> 0.5 m
    float robotR       = 0.6f;  // physical clearance
    // Plan with a WIDER margin than the reactive layer uses. If they are equal,
    // the planned path skims obstacle faces, the reactive layer then fights it
    // every frame, and the aircraft crawls. The 2D stack in this directory
    // learned this the same way.
    float planMarginM  = 0.9f;
    float unknownCost  = 2.5f;  // multiplier on step cost through unknown space
    int   maxExpand    = 250000;
    float goalTolM     = 1.5f;
};

struct PrecisePath {
    std::vector<std::array<float, 3>> pts;   // world ENU waypoints
    bool  found = false;
    int   expanded = 0;
    float costM = 0;
    // Set when the start cell itself was blocked and had to be nudged. Worth
    // surfacing rather than hiding: it usually means the map thinks you are
    // inside an obstacle, which is either drift or a bad inflation radius.
    bool  startWasBlocked = false;
};

class PrecisePlanner {
public:
    explicit PrecisePlanner(const PreciseParams& p = PreciseParams()) : p_(p) {}
    const PreciseParams& params() const { return p_; }

    PrecisePath plan(const VoxelMap& m,
                     float sx, float sy, float sz,
                     float gx, float gy, float gz);

private:
    PreciseParams p_;
};

}  // namespace sim
