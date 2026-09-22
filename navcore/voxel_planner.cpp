#include "voxel_planner.hpp"

#include <cstdint>
#include <vector>
#include <algorithm>
#include <array>
#include <cmath>

namespace sim {

static inline float deg2rad(float d) { return d * sim::PI_F / 180.f; }

// Direction from azimuth (clockwise from +North) and elevation.
static inline void dirFrom(float azDeg, float elDeg, float& dx, float& dy, float& dz) {
    float a = deg2rad(azDeg), e = deg2rad(elDeg);
    float ch = std::cos(e);
    dx = ch * std::sin(a);   // East
    dy = ch * std::cos(a);   // North
    dz = std::sin(e);        // Up
}

// Is a SPHERE of radius r around this point clear? A single-ray probe is what
// let the aircraft fly into trees: 48 azimuth bins is 7.5 deg, so adjacent rays
// are 0.65 m apart at 5 m, and a 0.10-0.35 m forest trunk fits between them
// completely unseen. Subtracting robotR from the ray length afterwards does not
// help -- the obstacle was never detected at all.
//
// Seven samples (centre + six axis offsets) rather than a full ball: 7x the
// probe cost instead of 125x at 0.25 m cells, and for convex obstacles larger
// than a voxel it finds the same things.
// EXHAUSTIVE over the cells in the ball, not 7 sample points. The 7-point
// version was the same sparse-sampling error that had already appeared twice in
// this codebase: adjacent axis samples on a 0.6 m sphere are 0.85 m apart, so a
// 0.10-0.35 m trunk at 45 deg sits between them and is invisible. That is why
// the aircraft still hit trees with PERFECT depth, a map with 0.000% false-free
// cells, and voxels finer than the trunks.
// sphereClear lives on VoxelMap. This file used to carry its own copy with
// the same two defects; see voxel_map.hpp for what they were.

// --- general planner --------------------------------------------------------

// Probe one (azimuth, elevation) direction: how far it REACHES with unknown
// space discounted, and how far the robot-sized volume is CONFIRMED free.
// Factored out so the held-heading fast path below can evaluate a single bin
// without paying for all 864.
void GeneralPlanner::probe(const VoxelMap& m, float px, float py, float pz,
                           float az, float el, float& reachOut, float& freeOut) const {
    const float step = m.params().cell * 0.75f;   // sub-cell so we cannot skip a voxel
    float dx, dy, dz; dirFrom(az, el, dx, dy, dz);
    float reach = 0.f, t = 0.f, freeRun = 0.f;
    bool stillFree = true;
    while (t < p_.horizonM) {
        t += step;
        float qx = px + dx * t, qy = py + dy * t, qz = pz + dz * t;
        VoxelMap::State s = m.stateAt(qx, qy, qz);
        if (s == VoxelMap::OCCUPIED) { stillFree = false; break; }
        // freeRun gates SPEED, so it gets the expensive correct test: the whole
        // robot-sized volume must be clear, not a centre line. Only out to
        // sweepM -- past that it cannot affect the command.
        if (stillFree) {
            if (t <= p_.sweepM && !m.sphereClear(qx, qy, qz, p_.robotR)) stillFree = false;
            else if (s == VoxelMap::FREE) freeRun = t;
            else stillFree = false;
        }
        reach = (s == VoxelMap::FREE) ? t : reach + step * (1.f - p_.unknownCost);
    }
    reachOut = std::min(reach, p_.horizonM);
    freeOut  = std::min(freeRun, p_.horizonM);
}

float GeneralPlanner::elForIndex(int i) const {
    return p_.elMinDeg + (p_.elMaxDeg - p_.elMinDeg) *
           (p_.nEl > 1 ? float(i) / (p_.nEl - 1) : 0.5f);
}

GeneralResult GeneralPlanner::plan(const VoxelMap& m, float px, float py, float pz,
                                   float goalAzDeg, float goalElDeg) {
    // HELD-HEADING FAST PATH, and this is the difference between "runs on a Pi"
    // and "nearly runs on a Pi". Commitment was added to stop the aircraft
    // spinning and, as first written, saved no CPU whatsoever: the 864-bin
    // probe ran unconditionally and commitment only skipped the scoring loop,
    // which is microseconds. Measured before: 13.99 ms/step, unchanged by
    // commitment.
    //
    // But a held step does not need the field. It needs exactly one question
    // answered -- is the direction I am already flying still clear? -- and that
    // is one bin out of 864. Probing only that bin is not a corner cut: it is
    // precisely the safety test the commitment logic was already doing, with
    // the 863 bins it was ignoring anyway left uncomputed.
    //
    // Skipped when the EMA is enabled, because that genuinely does need every
    // bin updated every step to mean anything.
    if (haveLast_ && held_ < p_.commitSteps && p_.fieldEma >= 1.f) {
        float reach = 0, fr = 0;
        probe(m, px, py, pz, lastAz_, lastEl_, reach, fr);
        if (fr - p_.robotR >= p_.breakFreeM) {
            ++held_;
            GeneralResult rc;
            rc.src = GeneralResult::HELD;
            rc.azDeg = lastAz_; rc.elDeg = lastEl_;
            rc.freeM = fr; rc.openM = reach;
            float usable = std::max(0.f, rc.freeM - p_.robotR);
            const float a2 = p_.decelMs2, t2 = p_.reactS;
            float v2 = -a2 * t2 + std::sqrt(a2 * t2 * a2 * t2 + 2.f * a2 * usable);
            rc.speed = std::min(p_.vMax, std::max(0.f, v2));
            return rc;
        }
        // Not clear any more: fall through and re-decide with the full field.
    }

    field_.assign(size_t(p_.nAz) * p_.nEl, 0.f);
    free_.assign(size_t(p_.nAz) * p_.nEl, 0.f);

    // Probe every (azimuth, elevation) bin. An UNKNOWN cell does not stop the
    // probe, but it accrues cost -- so a corridor of unknown space is usable
    // and a corridor of free space is preferred, which is exactly the ordering
    // you want when the camera only sees forward.
    for (int ie = 0; ie < p_.nEl; ++ie) {
        float el = elForIndex(ie);
        for (int ia = 0; ia < p_.nAz; ++ia) {
            float az = 360.f * float(ia) / p_.nAz;
            probe(m, px, py, pz, az, el,
                  field_[size_t(ie) * p_.nAz + ia], free_[size_t(ie) * p_.nAz + ia]);
        }
    }

    // Blend into the steering field. First call seeds it, so the aircraft is
    // not steered by a half-formed average on step 0.
    const size_t NB = field_.size();
    if (fieldS_.size() != NB || p_.fieldEma >= 1.f) { fieldS_ = field_; freeS_ = free_; }
    else {
        const float a = p_.fieldEma;
        for (size_t i = 0; i < NB; ++i) {
            fieldS_[i] += (field_[i] - fieldS_[i]) * a;
            freeS_[i]  += (free_[i]  - freeS_[i])  * a;
        }
    }

    // COMMITMENT CHECK, for the EMA configuration only. The ordinary path takes
    // the single-bin fast path at the top of plan() and never reaches here; this
    // exists because the EMA needs the whole field refreshed every step, so with
    // it enabled the hold decision has to be made after the full probe instead
    // of instead of it. Same test, same result, different cost.
    if (haveLast_ && held_ < p_.commitSteps && p_.fieldEma < 1.f) {
        int ia = int(std::lround(lastAz_ / 360.f * p_.nAz)) % p_.nAz;
        if (ia < 0) ia += p_.nAz;
        int ie = 0; float bd = 1e9f;
        for (int i = 0; i < p_.nEl; ++i) {
            float d = std::fabs(elForIndex(i) - lastEl_);
            if (d < bd) { bd = d; ie = i; }
        }
        float fr = free_[size_t(ie) * p_.nAz + ia];
        if (fr - p_.robotR >= p_.breakFreeM) {
            ++held_;
            GeneralResult rc;
            rc.src = GeneralResult::HELD;
            rc.azDeg = lastAz_; rc.elDeg = lastEl_;
            rc.freeM = fr; rc.openM = field_[size_t(ie) * p_.nAz + ia];
            float usable = std::max(0.f, rc.freeM - p_.robotR);
            const float a2 = p_.decelMs2, t2 = p_.reactS;
            float v2 = -a2 * t2 + std::sqrt(a2 * t2 * a2 * t2 + 2.f * a2 * usable);
            rc.speed = std::min(p_.vMax, std::max(0.f, v2));
            return rc;
        }
    }
    held_ = 0;

    // Score. Openness dominates; goal alignment and hysteresis break ties.
    GeneralResult r;
    float best = -1e30f;
    float incumbent = -1e30f;    // score of the heading we are already flying
    for (int ie = 0; ie < p_.nEl; ++ie) {
        float el = p_.elMinDeg + (p_.elMaxDeg - p_.elMinDeg) *
                   (p_.nEl > 1 ? float(ie) / (p_.nEl - 1) : 0.5f);
        for (int ia = 0; ia < p_.nAz; ++ia) {
            float az = 360.f * float(ia) / p_.nAz;
            // BOTH terms, and this cost a second failure to learn. Scoring on
            // openness alone made the planner choose whichever direction held
            // the most UNSEEN space -- which is exactly the direction with the
            // least confirmed-free room -- and then the stopping-distance gate
            // refused to move. Measured: wedged at 0.38 m of free run with 8.71 m
            // of openness, stationary for the remaining 780 steps.
            // Steer by the smoothed field; the raw one is reserved for speed.
            const size_t k = size_t(ie) * p_.nAz + ia;
            float open = fieldS_[k] / p_.horizonM;
            float fr   = freeS_[k]  / p_.horizonM;
            // Admissibility, though, is a safety test, so it reads the CURRENT
            // measurement: a direction that has just become blocked must be
            // rejected this frame even if its average still looks generous.
            if (field_[k] < p_.robotR * 2.f) continue;
            float dAz = std::fabs(angDiffDeg(az, goalAzDeg));
            float gd = dAz / 180.f + std::fabs(el - goalElDeg) / 90.f * 0.5f;
            float rev = std::max(0.f, dAz - 90.f) / 90.f;   // 0 ahead, 1 behind
            float hd = 0.f;
            if (haveLast_)
                hd = std::fabs(angDiffDeg(az, lastAz_)) / 180.f
                   + std::fabs(el - lastEl_) / 90.f * 0.5f;
            float score = open + p_.freeWeight * fr
                        - p_.goalWeight * gd - p_.hystWeight * hd
                        - p_.revPenalty * rev;
            if (score > best) {
                best = score; r.azDeg = az; r.elDeg = el;
                r.openM = field_[k];
                r.freeM = free_[k];
            }
            // Is this the heading we are already flying? Scored identically, so
            // the margin below compares like with like.
            if (haveLast_ && std::fabs(angDiffDeg(az, lastAz_)) < 180.f / p_.nAz
                          && std::fabs(el - lastEl_) < 1e-3f)
                incumbent = score;
        }
    }

    // DWELL. A challenger must beat the incumbent by switchMargin, not merely
    // tie with it. Without this the argmax flips on differences far smaller
    // than the map's own accuracy, which is how a planner ends up "spinning
    // between gaps" while every individual decision looks defensible.
    if (haveLast_ && incumbent > -1e29f && best - incumbent < p_.switchMargin) {
        int ia = int(std::lround(lastAz_ / 360.f * p_.nAz)) % p_.nAz;
        if (ia < 0) ia += p_.nAz;
        int ie = 0; float bd = 1e9f;
        for (int i = 0; i < p_.nEl; ++i) {
            float el = p_.elMinDeg + (p_.elMaxDeg - p_.elMinDeg) *
                       (p_.nEl > 1 ? float(i) / (p_.nEl - 1) : 0.5f);
            float d = std::fabs(el - lastEl_);
            if (d < bd) { bd = d; ie = i; }
        }
        const size_t k = size_t(ie) * p_.nAz + ia;
        if (field_[k] >= p_.robotR * 2.f) {
            best = incumbent;
            r.azDeg = 360.f * float(ia) / p_.nAz;
            r.elDeg = p_.elMinDeg + (p_.elMaxDeg - p_.elMinDeg) *
                      (p_.nEl > 1 ? float(ie) / (p_.nEl - 1) : 0.5f);
            r.openM = field_[k];
            r.freeM = free_[k];
        }
    }
    if (best <= -1e29f) {
        r.blocked = true; r.speed = 0.f; r.src = GeneralResult::BLOCKED;
        r.azDeg = haveLast_ ? lastAz_ : goalAzDeg;
        r.elDeg = haveLast_ ? lastEl_ : goalElDeg;
        return r;
    }
    // ESCAPE. If the goal-directed choice still has no room to move into, the
    // vehicle is wedged, and continuing to face the goal just holds it there
    // forever. Abandon the goal for this frame and take the single most open
    // direction available -- i.e. back out into space we have already seen.
    if (r.freeM - p_.robotR < p_.minFreeM) {
        float bf = -1.f; int bi = -1, be = -1;
        for (int ie = 0; ie < p_.nEl; ++ie)
            for (int ia = 0; ia < p_.nAz; ++ia)
                if (free_[size_t(ie) * p_.nAz + ia] > bf) {
                    bf = free_[size_t(ie) * p_.nAz + ia]; bi = ia; be = ie;
                }
        if (bi >= 0 && bf - p_.robotR >= p_.minFreeM) {
            r.azDeg = 360.f * float(bi) / p_.nAz;
            r.elDeg = p_.elMinDeg + (p_.elMaxDeg - p_.elMinDeg) *
                      (p_.nEl > 1 ? float(be) / (p_.nEl - 1) : 0.5f);
            r.freeM = bf;
            r.openM = field_[size_t(be) * p_.nAz + bi];
            r.src = GeneralResult::ESCAPE;
        }
    }
    lastAz_ = r.azDeg; lastEl_ = r.elDeg; haveLast_ = true;

    // SPEED IS SET BY CONFIRMED-FREE RANGE -- freeM, not openM, and this
    // distinction was worth a collision to learn. The first version gated speed
    // on the unknown-discounted openness, so an ENTIRELY UNMAPPED direction
    // scored ~6.6 m of "clearance" out of a 12 m horizon, cleared the 2 m stop
    // threshold, and the aircraft flew into a tree at 1.5 m/s on step 18 --
    // with PERFECT depth. Unknown space must earn zero speed.
    // It must be a STOPPING-DISTANCE budget, not a threshold. A threshold
    // deadlocks: with a forward-facing camera the confirmed-free distance in a
    // dense forest sits around 2 m, so "stop below 2 m" means stop forever --
    // and you cannot see further without moving. Measured: 638 of 700 steps
    // stationary, 7.3 m travelled in a 175 m run.
    //
    // Solving  d = v*t_react + v^2/(2a)  for v gives a speed that always has
    // room to stop inside what has actually been seen, and is positive for any
    // positive free distance, so the vehicle creeps rather than freezing.
    float usable = std::max(0.f, r.freeM - p_.robotR);
    if (usable < p_.minFreeM) { r.speed = 0.f; return r; }
    const float a = p_.decelMs2, tr = p_.reactS;
    float v = -a * tr + std::sqrt(a * tr * a * tr + 2.f * a * usable);
    r.speed = std::min(p_.vMax, std::max(0.f, v));
    return r;
}

}  // namespace sim
