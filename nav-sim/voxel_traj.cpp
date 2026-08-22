#include "voxel_traj.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

static inline float deg2rad(float d) { return d * sim::PI_F / 180.f; }
static inline float wrapDeg(float d) { return std::fmod(d + 540.f, 360.f) - 180.f; }

// Swept-volume test, identical in standard to the one GeneralPlanner uses: an
// EXHAUSTIVE scan of the cells in the ball, not sample points. The sampled
// version was the same mistake this codebase made three times -- adjacent axis
// samples on a 0.6 m sphere are 0.85 m apart, and a 0.2 m trunk sits between
// them unseen.
static inline bool sphereClear(const VoxelMap& m, float x, float y, float z,
                               float r, float coreFrac) {
    int cx, cy, cz; m.worldToCell(x, y, z, cx, cy, cz);
    const float cell = m.params().cell;
    const int R = int(std::ceil(r / cell));
    const float r2 = r * r;
    const float core2 = (r * coreFrac) * (r * coreFrac);
    for (int dz = -R; dz <= R; ++dz)
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                float ox = dx * cell, oy = dy * cell, oz = dz * cell;
                float d2 = ox*ox + oy*oy + oz*oz;
                if (d2 > r2) continue;
                if (!m.inBounds(cx+dx, cy+dy, cz+dz)) {
                    // Outside the map is unknown. Treat it like any unknown.
                    if (d2 <= core2 && coreFrac > 0.f) return false;
                    continue;
                }
                float l = m.logAt(cx+dx, cy+dy, cz+dz);
                if (l > m.params().occThresh) return false;              // blocked
                if (d2 <= core2 && !(l < m.params().freeThresh)) return false;  // not confirmed
            }
    return true;
}

TrajectoryPlanner::TrajectoryPlanner(const TrajParams& p) : p_(p) {
    // Roll out every primitive ONCE, here, in the body frame with +y forward.
    // The integration uses the same first-order velocity lag the vehicle has,
    // so a primitive is a path the aircraft can actually fly rather than a
    // geometric arc it would have to teleport onto.
    const int steps = std::max(1, int(p_.horizonS / p_.dt));
    const float k = std::min(1.f, p_.dt / std::max(1e-3f, p_.tau));

    for (int is = 0; is < p_.nSpeed; ++is) {
        // Speeds spread over (0, vMax]. The slowest primitive matters more than
        // it looks: it is the one that survives in tight cover, and without it
        // the library has nothing admissible exactly when it is needed most.
        float speed = p_.vMax * float(is + 1) / float(p_.nSpeed);
        for (int iy = 0; iy < p_.nYaw; ++iy) {
            float fy = (p_.nYaw > 1) ? (2.f * iy / float(p_.nYaw - 1) - 1.f) : 0.f;
            // Cube the fan parameter so candidates bunch near straight ahead,
            // where resolution actually buys something, and thin out toward the
            // extremes where one primitive either fits or does not.
            float yawRate = p_.maxYawRate * fy * fy * fy;
            for (int ic = 0; ic < p_.nClimb; ++ic) {
                float fc = (p_.nClimb > 1) ? (2.f * ic / float(p_.nClimb - 1) - 1.f) : 0.f;
                float climb = p_.maxClimb * fc;

                Prim pr; pr.speed = speed; pr.yawRate = yawRate; pr.climb = climb;
                pr.pts.reserve(steps);
                float x = 0, y = 0, z = 0, yaw = 0;
                float vx = 0, vy = 0, vz = 0;
                // Sideslip: the velocity vector leads the heading by beta while
                // turning, and |v| is unchanged so the speed budget still
                // means what it says. Sign follows the turn -- a right turn
                // slides right. Zero for a straight primitive at any latSlipDeg,
                // which is why this cannot affect the forward-flight case.
                const float beta = (p_.latSlipDeg != 0.f)
                    ? deg2rad(p_.latSlipDeg) * (2.f / sim::PI_F)
                        * std::atan(yawRate / std::max(1e-3f, p_.latKneeDps))
                    : 0.f;
                for (int s = 0; s < steps; ++s) {
                    yaw += deg2rad(yawRate) * p_.dt;
                    const float crs = yaw + beta;      // course, not heading
                    float cx = std::sin(crs) * speed, cy = std::cos(crs) * speed;
                    vx += (cx - vx) * k; vy += (cy - vy) * k; vz += (climb - vz) * k;
                    x += vx * p_.dt; y += vy * p_.dt; z += vz * p_.dt;
                    pr.pts.push_back({x, y, z});
                }
                prims_.push_back(std::move(pr));
            }
        }
    }

    // Escape set: pure translation in the body frame, heading unchanged, only
    // directions well off the nose (the forward arcs already cover the front).
    for (int i = 0; i < p_.nEscape; ++i) {
        float ang = 360.f * float(i) / float(std::max(1, p_.nEscape));
        float off = std::fabs(wrapDeg(ang));
        if (off < p_.escapeMinDeg) continue;      // forward arcs own that sector
        for (int ic = 0; ic < 3; ++ic) {
            float climb = p_.maxClimb * (ic - 1) * 0.5f;
            Prim pr; pr.speed = p_.escapeSpeed; pr.yawRate = 0; pr.climb = climb;
            pr.pts.reserve(steps);
            float x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;
            const float cxd = std::sin(deg2rad(ang)), cyd = std::cos(deg2rad(ang));
            for (int s = 0; s < steps; ++s) {
                vx += (cxd * p_.escapeSpeed - vx) * k;
                vy += (cyd * p_.escapeSpeed - vy) * k;
                vz += (climb - vz) * k;
                x += vx * p_.dt; y += vy * p_.dt; z += vz * p_.dt;
                pr.pts.push_back({x, y, z});
            }
            prims_.push_back(std::move(pr));
        }
    }
}

GeneralResult TrajectoryPlanner::plan(const VoxelMap& m, float px, float py, float pz,
                                      float curYawDeg, float goalAzDeg, float goalElDeg,
                                      const std::vector<CoarseLevel>& coarse,
                                      const FarBearings* far) {
    GeneralResult r;
    chosen_.clear();
    cands_.clear();

    const float ca = std::cos(deg2rad(curYawDeg)), sa = std::sin(deg2rad(curYawDeg));
    // Body +y is forward. Rotate into world by the current heading, clockwise
    // from North, matching every other frame convention in this tree.
    auto toWorld = [&](const std::array<float, 3>& b, float& wx, float& wy, float& wz) {
        wx = px + b[0] * ca + b[1] * sa;
        wy = py - b[0] * sa + b[1] * ca;
        wz = pz + b[2];
    };

    reject_ = Reject();
    float best = -1e30f;
    const Prim* bestPrim = nullptr;
    float bestFree = 0;
    size_t bestClear = 0;

    for (const Prim& pr : prims_) {
        // How far along does the swept volume stay clear? Stop at the first
        // blocked sample; everything past it is unreachable, not merely costly.
        size_t nClear = 0;
        float freeLen = 0, prevX = px, prevY = py, prevZ = pz;
        int why = 0;                       // 1 occupied, 2 unknown
        for (size_t i = 0; i < pr.pts.size(); ++i) {
            float wx, wy, wz; toWorld(pr.pts[i], wx, wy, wz);
            if (!sphereClear(m, wx, wy, wz, p_.robotR, p_.coreFrac)) { why = 1; break; }
            // Only CONFIRMED-FREE length earns speed. Unknown space is
            // traversable but pays nothing, which is the rule that stopped this
            // aircraft flying into a tree at 1.5 m/s on perfect depth.
            if (m.stateAt(wx, wy, wz) != VoxelMap::FREE) { why = 2; break; }
            freeLen += std::sqrt((wx-prevX)*(wx-prevX) + (wy-prevY)*(wy-prevY)
                               + (wz-prevZ)*(wz-prevZ));
            prevX = wx; prevY = wy; prevZ = wz;
            ++nClear;
        }
        if (nClear == 0) {
            if (why == 1) ++reject_.occupied; else ++reject_.unknown;
            ++reject_.atStart;
            continue;
        }
        if (why == 1) ++reject_.occupied; else if (why == 2) ++reject_.unknown;

        // Score on where it ENDS UP, not where it points. A primitive that
        // curves toward the goal beats one that starts toward it and turns
        // away, and only an endpoint test can tell them apart.
        float ex, ey, ez; toWorld(pr.pts[nClear - 1], ex, ey, ez);
        float endAz = std::atan2(ex - px, ey - py) * 180.f / sim::PI_F;
        float endEl = std::atan2(ez - pz, std::hypot(ex - px, ey - py)) * 180.f / sim::PI_F;
        float gd = std::fabs(wrapDeg(endAz - goalAzDeg)) / 180.f
                 + std::fabs(endEl - goalElDeg) / 90.f * 0.5f;
        float clear = freeLen / std::max(0.1f, p_.vMax * p_.horizonS);
        float smooth = std::fabs(pr.yawRate) / std::max(1.f, p_.maxYawRate);

        // Coarse-map openness along the bearing this primitive ends on. Reward
        // only -- it cannot veto, and it cannot raise the speed budget.
        float farOpen = 0.f;
        if (far && far->field && p_.farWeight > 0.f) {
            // Bearing lookup instead of a march: the field already answers
            // "nearest surface on this bearing", so openness along a heading is
            // one query rather than a walk through cubes.
            // UNKNOWN IS NOT OPEN. `rangeAt` returns < 0 for a bin holding
            // nothing, and that used to be mapped to FULL reach -- so a bearing
            // with no information scored the maximum, and the largest region of
            // "no information" in any outdoor scene is THE SKY. Every upward
            // bearing scored 1.0, the argmax went up, and the aircraft climbed
            // because the emptiest thing it could see was air.
            //
            // The rule is already written thirty lines above for the near
            // field: unknown space is traversable but pays nothing. This layer
            // has no free space at all -- it only ever knows "nearest surface
            // on this bearing" -- so absence of a return is absence of
            // knowledge, never evidence of clearance.
            //
            // Openness is therefore EARNED by a confirmed distance and by
            // nothing else. With no far information the term simply vanishes
            // and the goal direction decides, which is the correct behaviour in
            // open ground rather than a special case for it.
            const float r = far->field->rangeAt(endAz, endEl);
            farOpen = (r < 0.f)
                    ? 0.f
                    : std::min(r, far->rangeM) / std::max(0.1f, far->rangeM);
        } else if (!coarse.empty() && p_.farWeight > 0.f) {
            const float dx = std::sin(deg2rad(endAz)) * std::cos(deg2rad(endEl));
            const float dy = std::cos(deg2rad(endAz)) * std::cos(deg2rad(endEl));
            const float dz = std::sin(deg2rad(endEl));
            // Step with the FINEST level in play, so resolution degrades with
            // distance instead of falling off a cliff at the first handover.
            float reach = 0.f, t = std::max(0.5f, coarse.front().map->params().cell * 0.5f);
            int nSamp = 0, nOcc = 0;
            while (t <= p_.farRangeM) {
                const VoxelMap* lv = nullptr;
                float step = 0.f;
                for (const CoarseLevel& c : coarse)       // fine first
                    if (t <= c.rangeM) { lv = c.map; step = std::max(0.5f, c.map->params().cell * 0.5f); break; }
                if (!lv) break;                            // past every level's honest range
                // Same rule as the bearing path above, for the same reason: a
                // cell that is merely NOT OCCUPIED is not thereby open -- most
                // of a coarse map is unknown, and treating unknown as reach is
                // how sky wins an openness argmax. Only FREE extends reach.
                const VoxelMap::State st =
                    lv->stateAt(ex + dx * t, ey + dy * t, ez + dz * t);
                const bool occ  = st == VoxelMap::OCCUPIED;
                const bool free = st == VoxelMap::FREE;
                ++nSamp; if (!free) ++nOcc;
                if (occ && p_.farMode == TrajParams::FarMode::FIRST_BLOCKED) break;
                if (free) reach = t; else break;   // stop at the first non-FREE
                t += step;
            }
            farOpen = (p_.farMode == TrajParams::FarMode::DENSITY)
                    ? (nSamp ? 1.f - float(nOcc) / float(nSamp) : 0.f)
                    : reach / p_.farRangeM;
        }

        // Charged on the climb ALONE, not on |elevation|, so descending is left
        // to the goal term and to the map.
        //
        // And charged relative to the COMMANDED elevation, not to the horizon.
        // A flat penalty on absolute climb would also forbid climbing when the
        // mission explicitly asks for it, which turns a bias into a ceiling.
        // What must be suppressed is climbing the planner talked ITSELF into --
        // so the aircraft pays only for the climb it was not told to make.
        //
        // Sized by a rule rather than by taste: at 15 deg of uncommanded climb
        // the penalty is 1.0, which is twice the most the far term can ever
        // offer at its default weight. No openness reading, however open, can
        // buy a climb on its own.
        const float dEl     = endEl - goalElDeg;
        const float climbUp = std::max(0.f,  dEl) / 90.f;
        const float sinkDn  = std::max(0.f, -dEl) / 90.f;

        float score = p_.clearWeight * clear
                    - p_.goalWeight * gd
                    - p_.smoothWeight * smooth
                    + p_.farWeight * farOpen
                    - p_.climbPenalty * climbUp
                    - p_.descentPenalty * sinkDn;
        if (score > best) {
            best = score; bestPrim = &pr; bestFree = freeLen; bestClear = nClear;
        }
        // Keep admissible candidates for drawing. Capped, because this is a
        // debug aid and an unbounded one would dominate the frame time it is
        // meant to help you understand.
        if (cands_.size() < 64) {
            std::vector<std::array<float, 3>> w;
            w.reserve(nClear);
            for (size_t i = 0; i < nClear; ++i) {
                float wx, wy, wz; toWorld(pr.pts[i], wx, wy, wz);
                w.push_back({wx, wy, wz});
            }
            cands_.push_back(std::move(w));
        }
    }

    if (!bestPrim) {
        // Nothing in the library is flyable. Hold, and keep the last heading so
        // the aircraft does not snap to an arbitrary bearing while stopped.
        r.blocked = true; r.speed = 0; r.src = GeneralResult::BLOCKED;
        r.azDeg = haveLast_ ? lastAz_ : goalAzDeg;
        r.elDeg = haveLast_ ? lastEl_ : goalElDeg;
        return r;
    }

    for (size_t i = 0; i < bestClear; ++i) {
        float wx, wy, wz; toWorld(bestPrim->pts[i], wx, wy, wz);
        chosen_.push_back({wx, wy, wz});
    }

    // The COMMAND is the direction to a point a short way along the winning
    // path -- not to its endpoint. Aiming at the endpoint of a curved rollout
    // cuts the corner and flies outside the volume that was actually checked.
    size_t aim = std::min(bestClear - 1, size_t(std::max(1, int(p_.aimS / p_.dt))));
    float ax, ay, az; toWorld(bestPrim->pts[aim], ax, ay, az);
    r.azDeg = std::atan2(ax - px, ay - py) * 180.f / sim::PI_F;
    r.elDeg = std::atan2(az - pz, std::hypot(ax - px, ay - py)) * 180.f / sim::PI_F;
    r.freeM = bestFree;
    r.openM = bestFree;
    r.src = GeneralResult::SCORED;

    // Speed: the primitive's own speed, capped by the stopping-distance budget
    // over what was confirmed clear. Same rule as before -- solving
    // d = v*t_react + v^2/(2a) for v -- so this cannot command faster than it
    // can stop inside what it has actually seen.
    float usable = std::max(0.f, bestFree - p_.robotR);
    if (usable < p_.minFreeM) { r.speed = 0; }
    else {
        const float a = p_.decelMs2, tr = p_.reactS;
        float v = -a * tr + std::sqrt(a * tr * a * tr + 2.f * a * usable);
        r.speed = std::min(bestPrim->speed, std::max(0.f, v));
    }
    lastAz_ = r.azDeg; lastEl_ = r.elDeg; haveLast_ = true;
    return r;
}

}  // namespace sim
