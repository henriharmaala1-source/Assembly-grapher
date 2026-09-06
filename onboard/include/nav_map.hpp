#pragma once

#include <cstdint>
#include <vector>

// Local occupancy grid + wavefront planner (P5b) — the memory the reactive
// corridor layer lacks. The VFH+ corridor sees only the current field of view
// and forgets everything else, so it traps in a local minimum on an obstacle
// sitting directly between the drone and the goal (it re-approaches the same
// obstacle after scanning). LocalMap accumulates observations into a rolling
// occupancy grid in the local ENU frame, so a planner can route AROUND an
// obstacle it can no longer see.
//
// Division of labour with the reactive layer:
//   - LocalMap.plan() gives the GLOBAL direction (which way, around known
//     obstacles) — grid memory is the authority for direction.
//   - the live corridor still governs SPEED and the STOP reflex — the grid can
//     be stale/wrong about the immediate front, so the live sensor has the last
//     word on whether it is safe to move forward right now.
//
// Metric scale note: the grid is only as metric as its input. The ToF path and
// the SITL synthetic perception feed true metres; monocular depth feeds a
// NOMINAL scale (relative openness × max range) until a ToF sensor or VIO scale
// exists (ROADMAP P5a). The planner's routing is unaffected by absolute scale;
// only the physical cell size is.
class LocalMap {
public:
    struct Params {
        float sizeM     = 80.f;    // grid is sizeM × sizeM, centred on originE/N
        float cellM     = 0.5f;    // cell edge length
        float originE   = 0.f;     // ENU coord the grid is centred on
        float originN   = 0.f;
        float robotR    = 1.5f;    // obstacle inflation for planning (vehicle+margin)
        float lOcc      = 0.85f;   // log-odds added to a hit cell
        float lFree     = 0.40f;   // log-odds removed from a free cell
        float lClamp    = 5.0f;    // |log-odds| clamp
        float occThresh = 0.30f;   // log-odds above this = obstacle (for planning)
        int   descentSteps = 6;    // gradient look-ahead cells → a smoother bearing
    };

    LocalMap() { reset(); }
    explicit LocalMap(Params p) : p_(p) { reset(); }

    void reset();

    // Integrate a polar depth scan taken at pose (pe,pn,yawDeg). scanClear[i] is
    // the metric clearance of ray i; the scan spans fovDeg centred on yawDeg,
    // ray 0 at yaw−fov/2. A ray at (or beyond) maxM is a miss — free along its
    // length, no hit cell. Cells beyond a hit stay unknown (never marked).
    void integrate(float pe, float pn, float yawDeg,
                   const float* scanClear, int n, float fovDeg, float maxM);

    // Route from (pe,pn) toward goalBearingDeg (a far cell in that compass
    // direction), around known obstacles. Sets outBearingDeg (deg, 0=N) and
    // returns true, or returns false if there is no map yet / the goal is
    // unreachable / the drone cell is blocked — caller falls back to reactive.
    bool plan(float pe, float pn, float goalBearingDeg, float& outBearingDeg);

    // --- introspection (display / tests) ---
    int   width()  const { return W_; }
    int   height() const { return H_; }
    float cellM()  const { return p_.cellM; }
    bool  occupied(int ix, int iy) const;         // planning-threshold occupancy
    float logodds(int ix, int iy) const;

private:
    bool  inBounds_(int ix, int iy) const { return ix >= 0 && ix < W_ && iy >= 0 && iy < H_; }
    int   idx_(int ix, int iy) const { return iy * W_ + ix; }
    void  worldToCell_(float e, float n, int& ix, int& iy) const;
    void  cellToWorld_(int ix, int iy, float& e, float& n) const;

    Params             p_;
    int                W_ = 0, H_ = 0;
    float              e0_ = 0.f, n0_ = 0.f;   // world coord of cell (0,0) centre
    std::vector<float> log_;                   // log-odds, row-major (iy*W+ix)
    std::vector<int>   dist_;                  // scratch: BFS distance field
    std::vector<uint8_t> blocked_;             // scratch: inflated obstacle mask
};
