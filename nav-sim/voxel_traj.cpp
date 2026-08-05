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
                for (int s = 0; s < steps; ++s) {
                    yaw += deg2rad(yawRate) * p_.dt;
                    float cx = std::sin(yaw) * speed, cy = std::cos(yaw) * speed;
                    vx += (cx - vx) * k; vy += (cy - vy) * k; vz += (climb - vz) * k;
                    x += vx * p_.dt; y += vy * p_.dt; z += vz * p_.dt;
                    pr.pts.push_back({x, y, z});
                }
                prims_.push_back(std::move(pr));
            }
        }
    }
}

GeneralResult TrajectoryPlanner::plan(const VoxelMap& m, float px, float py, float pz,
                                      float curYawDeg, float goalAzDeg, float goalElDeg) {
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

    float best = -1e30f;
    const Prim* bestPrim = nullptr;
    float bestFree = 0;
    size_t bestClear = 0;

    for (const Prim& pr : prims_) {
        // How far along does the swept volume stay clear? Stop at the first
        // blocked sample; everything past it is unreachable, not merely costly.
        size_t nClear = 0;
        float freeLen = 0, prevX = px, prevY = py, prevZ = pz;
        for (size_t i = 0; i < pr.pts.size(); ++i) {
            float wx, wy, wz; toWorld(pr.pts[i], wx, wy, wz);
            if (!sphereClear(m, wx, wy, wz, p_.robotR, p_.coreFrac)) break;
            // Only CONFIRMED-FREE length earns speed. Unknown space is
            // traversable but pays nothing, which is the rule that stopped this
            // aircraft flying into a tree at 1.5 m/s on perfect depth.
            if (m.stateAt(wx, wy, wz) != VoxelMap::FREE) break;
            freeLen += std::sqrt((wx-prevX)*(wx-prevX) + (wy-prevY)*(wy-prevY)
                               + (wz-prevZ)*(wz-prevZ));
            prevX = wx; prevY = wy; prevZ = wz;
            ++nClear;
        }
        if (nClear == 0) continue;

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

        float score = p_.clearWeight * clear
                    - p_.goalWeight * gd
                    - p_.smoothWeight * smooth;
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
