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
    while (!q.empty()) {
        auto [cx,cy] = q.front(); q.pop();
        // a frontier: known-free and adjacent to unknown
        bool border=false; for(int k=0;k<8;++k) if(unknown(cx+DX8[k],cy+DY8[k])){border=true;break;}
        if (border && knownFree(cx,cy) && !g.blocked(cx,cy,inflate)) {
            float e,n; g.cellToWorld(cx,cy,e,n); out={e,n}; return true;
        }
        for(int k=0;k<8;++k){ int nx=cx+DX8[k], ny=cy+DY8[k];
            if(!g.inBounds(nx,ny)||seen[ny*C+nx]) continue;
            if(g.blocked(nx,ny,inflate)) continue;
            if(!knownFree(nx,ny)) continue;          // only travel proven-free space
            seen[ny*C+nx]=1; q.push({nx,ny});
        }
    }
    return false;   // reachable area fully mapped
}

Explore::Out Explore::step(const OccupancyGrid& g, Vec2 pos, int inflate) {
    if (phase_ == Phase::DONE) return { home_, explorePhaseName(phase_), true };
    maxDist_ = std::max(maxDist_, std::hypot(pos.e-home_.e, pos.n-home_.n));

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
            else { phase_ = Phase::RETURN; haveTgt_ = false; }   // area mapped -> go home
        }
        if (phase_ == Phase::EXPLORE) return { tgt_, "EXPLORE", false };
    }

    // RETURN: head home; done once we're on it.
    if (std::hypot(pos.e-home_.e, pos.n-home_.n) < 1.5f) { phase_ = Phase::DONE; return { home_, "DONE", true }; }
    return { home_, "RETURN", false };
}

}  // namespace navsim
