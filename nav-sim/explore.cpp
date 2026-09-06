#include "explore.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace navsim {

namespace {
const int DX8[8] = {1,-1,0,0,1,1,-1,-1};
const int DY8[8] = {0,0,1,-1,1,-1,1,-1};
}

const char* explorePhaseName(Explore::Phase p) {
    switch (p) { case Explore::Phase::EXPLORE: return "EXPLORE";
                 case Explore::Phase::RETURN:  return "RETURN";
                 case Explore::Phase::DONE:    return "DONE"; }
    return "?";
}

bool Explore::nearestFrontier(const OccupancyGrid& g, Vec2 pos, int inflate, Vec2& out) const {
    int sx, sy; if (!g.worldToCell(pos.e, pos.n, sx, sy)) return false;
    // snap start into free space if the exact cell reads blocked/unknown
    if (g.blocked(sx, sy, inflate)) {
        bool ok=false; for(int r=1;r<=20&&!ok;++r) for(int dy=-r;dy<=r&&!ok;++dy) for(int dx=-r;dx<=r&&!ok;++dx)
            if(std::max(std::abs(dx),std::abs(dy))==r && g.inBounds(sx+dx,sy+dy) && !g.blocked(sx+dx,sy+dy,inflate)){ sx+=dx;sy+=dy;ok=true; }
        if(!ok) return false;
    }
    const int C = g.cells();
    auto knownFree = [&](int x,int y){ return g.inBounds(x,y) && g.logAt(x,y) < -0.3f; };
    auto unknown   = [&](int x,int y){ return g.inBounds(x,y) && std::fabs(g.logAt(x,y)) < 0.1f; };
    std::vector<char> seen(C*C, 0);
    std::queue<std::pair<int,int>> q; q.push({sx,sy}); seen[sy*C+sx]=1;
    // The drone's OWN cell (and its close neighbours) almost always borders
    // unknown space, since a forward-cone sensor never scans a full 360 in one
    // look — so testing the start cell as a candidate returns "the frontier is
    // where you're already standing" on effectively every call, especially the
    // very first one. That's the "goal sits at spawn" bug: subgoal = pos, zero
    // displacement, so the caller "arrives" instantly and re-picks, camping
    // near the start instead of pushing outward. Require a real minimum
    // distance so a returned frontier is always somewhere actually worth flying
    // to (comfortably past the "arrived" radius the caller checks against).
    constexpr float kMinFrontierM = 3.0f;
    while (!q.empty()) {
        auto [cx,cy] = q.front(); q.pop();
        // a frontier: known-free and adjacent to unknown
        bool border=false; for(int k=0;k<8;++k) if(unknown(cx+DX8[k],cy+DY8[k])){border=true;break;}
        if (border && knownFree(cx,cy) && !g.blocked(cx,cy,inflate)) {
            float e,n; g.cellToWorld(cx,cy,e,n);
            if (std::hypot(e-pos.e, n-pos.n) >= kMinFrontierM) { out={e,n}; return true; }
        }
        for(int k=0;k<8;++k){ int nx=cx+DX8[k], ny=cy+DY8[k];
            if(!g.inBounds(nx,ny)||seen[ny*C+nx]) continue;
            if(g.blocked(nx,ny,inflate)) continue;
            if(!knownFree(nx,ny)) continue;          // only travel proven-free space
            seen[ny*C+nx]=1; q.push({nx,ny});
        }
    }
    return false;   // no frontier at least kMinFrontierM away (fully mapped, or boxed in tight)
}

Explore::Out Explore::step(const OccupancyGrid& g, Vec2 pos, int inflate) {
    if (phase_ == Phase::DONE) return { home_, explorePhaseName(phase_), true };
    maxDist_ = std::max(maxDist_, std::hypot(pos.e-home_.e, pos.n-home_.n));
    bool justEnteredReturn = false;   // true the tick EXPLORE->RETURN happens (either path below)

    if (phase_ == Phase::EXPLORE) {
        // Progress watchdog: a frontier is only reachable IN THE GRID (known-free,
        // BFS-connected) -- the driver actually flying to it can still get wedged
        // in practice (the funnel/local-minimum class of problem MoveStopSense's
        // own STUCK phase exists for), and unlike that driver, Explore has no
        // visibility into its state to notice. So it watches its own net
        // displacement directly: no real progress for kStuckTicksMax ticks means
        // this frontier is unreachable in practice, however "connected" it reads on
        // the map -- give up on it and try to get home rather than re-issuing the
        // same stuck target forever. The threshold is well above the driver's own
        // SETTLE/SCAN timeouts (so a normal in-place scan never false-triggers it)
        // and well below what a permanently wedged run would otherwise burn.
        constexpr float kStuckEscapeM  = 1.0f;
        constexpr int   kStuckTicksMax = 800;   // ~40s at the sim's usual dt=0.05
        if (std::hypot(pos.e-stuckAnchor_.e, pos.n-stuckAnchor_.n) > kStuckEscapeM) {
            stuckAnchor_ = pos; stuckTicks_ = 0;
        } else if (++stuckTicks_ > kStuckTicksMax) {
            phase_ = Phase::RETURN; haveTgt_ = false; stuckTicks_ = 0; justEnteredReturn = true;
        }
    }

    if (phase_ == Phase::EXPLORE) {
        const bool nearTgt = haveTgt_ && std::hypot(pos.e-tgt_.e, pos.n-tgt_.n) < 2.5f;
        if (!haveTgt_ || nearTgt) {
            Vec2 f;
            if (nearestFrontier(g, pos, inflate, f)) { tgt_ = f; haveTgt_ = true; }
            else if (maxDist_ < 3.f) {
                // Haven't really explored yet (map still tiny) — push forward to
                // bootstrap the map rather than prematurely declaring the area done.
                return { { home_.e, home_.n + 10.f }, "EXPLORE", false };
            }
            else { phase_ = Phase::RETURN; haveTgt_ = false; justEnteredReturn = true; }  // area mapped -> go home
        }
        if (phase_ == Phase::EXPLORE) return { tgt_, "EXPLORE", false };
    }

    // RETURN: head home; done once we're on it. justEnteredReturn tells the
    // caller to reset its driver -- see the Out::resetDriver comment in the
    // header for why a fresh start is required here specifically, not assumed.
    if (std::hypot(pos.e-home_.e, pos.n-home_.n) < 1.5f) { phase_ = Phase::DONE; return { home_, "DONE", true }; }
    return { home_, "RETURN", false, justEnteredReturn };
}

}  // namespace navsim
