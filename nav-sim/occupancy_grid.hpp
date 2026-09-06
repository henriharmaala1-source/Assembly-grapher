#pragma once

#include <cstdint>
#include <vector>

#include "sim_world.hpp"

namespace navsim {

// A small standalone log-odds occupancy grid. Self-contained (no dependency on
// the drone flight stack): the drone integrates what its forward scan sees as it
// moves, so planners work on a PARTIALLY-OBSERVED map and must replan — which is
// the honest test, and the reason a global plan can be stale.
//
// World frame: ENU metres (E = +x, N = +y). Grid centred on `originE/originN`.
class OccupancyGrid {
public:
    struct Params {
        int   cells   = 160;    // cells per side (square grid)
        float cellM   = 0.5f;   // metres per cell
        float lHit    = 0.85f;  // log-odds added to a cell a ray terminates in
        float lMiss   = 0.40f;  // log-odds removed along the free ray
        float lClamp  = 6.0f;   // saturation
        float occThresh = 0.0f; // log-odds above this = occupied
    };

    OccupancyGrid();                     // default params
    explicit OccupancyGrid(Params p);

    void setOrigin(float e, float n) { originE_ = e; originN_ = n; }
    int   cells() const { return p_.cells; }
    float cellM() const { return p_.cellM; }
    const Params& params() const { return p_; }

    // Integrate a forward polar scan (ranges in metres across the FoV centred on
    // yaw) taken from world position (pe,pn). Free space along each ray, an
    // obstacle at the hit.
    void integrate(float pe, float pn, float yawDeg, const std::vector<float>& ranges,
                   float hFovDeg, float maxRange);

    // World <-> cell.
    bool worldToCell(float e, float n, int& cx, int& cy) const;
    void cellToWorld(int cx, int cy, float& e, float& n) const;
    bool inBounds(int cx, int cy) const {
        return cx >= 0 && cy >= 0 && cx < p_.cells && cy < p_.cells;
    }

    bool occupied(int cx, int cy) const {
        return inBounds(cx, cy) && log_[cy * p_.cells + cx] > p_.occThresh;
    }
    // Occupied OR within `inflateCells` of an occupied cell (robot radius berth).
    bool blocked(int cx, int cy, int inflateCells) const;

    float logAt(int cx, int cy) const { return inBounds(cx,cy) ? log_[cy*p_.cells+cx] : 0.f; }

private:
    Params p_;
    std::vector<float> log_;
    float originE_, originN_;
};

}  // namespace navsim
