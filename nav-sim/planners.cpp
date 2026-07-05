#include "planners.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <random>

namespace navsim {

namespace {
constexpr float kSqrt2 = 1.41421356f;
struct Cell { int x, y; };

double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// Nearest free (unblocked) cell to (cx,cy) by an outward spiral — used when the
// start or goal falls inside the inflated obstacle mask.
bool nearestFree(const OccupancyGrid& g, int& cx, int& cy, int inflate, int maxR = 40) {
    if (!g.blocked(cx, cy, inflate)) return true;
    for (int r = 1; r <= maxR; ++r)
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
                const int nx = cx + dx, ny = cy + dy;
                if (g.inBounds(nx, ny) && !g.blocked(nx, ny, inflate)) { cx = nx; cy = ny; return true; }
            }
    return false;
}

std::vector<Vec2> cellsToWorld(const OccupancyGrid& g, const std::vector<Cell>& cs) {
    std::vector<Vec2> out; out.reserve(cs.size());
    for (auto& c : cs) { float e, n; g.cellToWorld(c.x, c.y, e, n); out.push_back({e, n}); }
    return out;
}

const int DX8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int DY8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
inline float stepCost(int k) { return k < 4 ? 1.f : kSqrt2; }

// --------------------------------------------------------------- Wavefront (BFS)
// The onboard method: flood integer distance from the GOAL over free cells, then
// the drone descends the gradient. Unweighted — treats all steps equal.
class Wavefront : public IPlanner {
public:
    const char* name() const override { return "wavefront"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0 = nowMs(); PlanResult R;
        int sx, sy, gx, gy;
        if (!g.worldToCell(s.e, s.n, sx, sy) || !g.worldToCell(gl.e, gl.n, gx, gy)) { R.planMs = nowMs()-t0; return R; }
        nearestFree(g, sx, sy, infl); nearestFree(g, gx, gy, infl);
        const int C = g.cells();
        std::vector<int> dist(C * C, -1);
        std::queue<Cell> q; q.push({gx, gy}); dist[gy*C+gx] = 0;
        int expanded = 0;
        while (!q.empty()) {
            Cell c = q.front(); q.pop(); ++expanded;
            for (int k = 0; k < 8; ++k) {
                const int nx = c.x+DX8[k], ny = c.y+DY8[k];
                if (!g.inBounds(nx,ny) || g.blocked(nx,ny,infl) || dist[ny*C+nx]>=0) continue;
                dist[ny*C+nx] = dist[c.y*C+c.x] + 1; q.push({nx, ny});
            }
        }
        R.expanded = expanded;
        if (dist[sy*C+sx] < 0) { R.planMs = nowMs()-t0; return R; }   // unreachable
        // gradient descent from start to goal
        std::vector<Cell> path; Cell cur{sx, sy};
        for (int guard = 0; guard < C*C; ++guard) {
            path.push_back(cur);
            if (cur.x==gx && cur.y==gy) break;
            int best=-1, bx=cur.x, by=cur.y;
            for (int k=0;k<8;++k){ int nx=cur.x+DX8[k], ny=cur.y+DY8[k];
                if (!g.inBounds(nx,ny)||dist[ny*C+nx]<0) continue;
                if (best<0||dist[ny*C+nx]<best){best=dist[ny*C+nx];bx=nx;by=ny;} }
            if (best<0) break; cur={bx,by};
        }
        R.path = cellsToWorld(g, path); R.ok = !path.empty(); R.planMs = nowMs()-t0; return R;
    }
};

// ------------------------------------------------------ Dijkstra / A* (shared)
class GridSearch : public IPlanner {
public:
    explicit GridSearch(bool useHeuristic) : astar_(useHeuristic) {}
    const char* name() const override { return astar_ ? "astar" : "dijkstra"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0 = nowMs(); PlanResult R;
        int sx, sy, gx, gy;
        if (!g.worldToCell(s.e, s.n, sx, sy) || !g.worldToCell(gl.e, gl.n, gx, gy)) { R.planMs=nowMs()-t0; return R; }
        nearestFree(g, sx, sy, infl); nearestFree(g, gx, gy, infl);
        const int C = g.cells();
        std::vector<float> gcost(C*C, 1e18f);
        std::vector<int>   parent(C*C, -1);
        auto h = [&](int x,int y){ if(!astar_) return 0.f;
            const float dx=std::abs(x-gx), dy=std::abs(y-gy);
            return (dx+dy) + (kSqrt2-2.f)*std::min(dx,dy); };   // octile
        typedef std::pair<float,int> PQe;
        std::priority_queue<PQe, std::vector<PQe>, std::greater<PQe>> pq;
        gcost[sy*C+sx]=0.f; pq.push({h(sx,sy), sy*C+sx});
        int expanded=0; bool found=false;
        while(!pq.empty()){
            auto [f, id]=pq.top(); pq.pop();
            const int cx=id%C, cy=id/C;
            if (f - h(cx,cy) > gcost[id] + 1e-4f) continue;   // stale
            ++expanded;
            if (cx==gx&&cy==gy){found=true;break;}
            for(int k=0;k<8;++k){ int nx=cx+DX8[k], ny=cy+DY8[k];
                if(!g.inBounds(nx,ny)||g.blocked(nx,ny,infl)) continue;
                const float ng=gcost[id]+stepCost(k);
                if(ng<gcost[ny*C+nx]){ gcost[ny*C+nx]=ng; parent[ny*C+nx]=id;
                    pq.push({ng+h(nx,ny), ny*C+nx}); } }
        }
        R.expanded=expanded;
        if(!found){ R.planMs=nowMs()-t0; return R; }
        std::vector<Cell> path; for(int id=gy*C+gx; id>=0; id=parent[id]) path.push_back({id%C,id/C});
        std::reverse(path.begin(), path.end());
        R.path=cellsToWorld(g,path); R.ok=!path.empty(); R.planMs=nowMs()-t0; return R;
    }
private:
    bool astar_;
};

// ------------------------------------------------------------- Potential field
// Reactive: attractive pull to the goal + repulsive push from nearby occupied
// cells; returns a single step. No global plan — cheap, but traps in local minima
// (the classic failure the grid methods above avoid). Included precisely so the
// comparison SHOWS that.
class PotentialField : public IPlanner {
public:
    const char* name() const override { return "potential"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0=nowMs(); PlanResult R;
        float ae = gl.e - s.e, an = gl.n - s.n;
        const float ad = std::hypot(ae,an); if(ad>1e-3f){ae/=ad;an/=ad;}
        // repulsion from occupied cells within a small window around the drone
        int sx, sy; g.worldToCell(s.e, s.n, sx, sy);
        const int Rc = infl + 6; float re=0, rn=0;
        for(int dy=-Rc;dy<=Rc;++dy)for(int dx=-Rc;dx<=Rc;++dx){
            if(!g.occupied(sx+dx,sy+dy)) continue;
            const float d = std::hypot((float)dx,(float)dy)*g.cellM();
            if(d<1e-3f) continue;
            const float w = 1.f/(d*d);
            re += -(dx*g.cellM())/d * w; rn += -(dy*g.cellM())/d * w;
        }
        const float kRep = 4.0f;
        float ve = ae + kRep*re, vn = an + kRep*rn;
        const float vd = std::hypot(ve,vn); if(vd>1e-3f){ve/=vd;vn/=vd;}
        R.path = { s, { s.e + ve*g.cellM()*3.f, s.n + vn*g.cellM()*3.f } };
        R.ok = true; R.expanded = (2*Rc+1)*(2*Rc+1); R.planMs=nowMs()-t0; return R;
    }
};

// ----------------------------------------------------------------------- RRT
// Sampling-based: grow a tree of free-space samples until one reaches near the
// goal, then backtrack. Not resolution-optimal (jagged paths) but scales to open
// space without a full grid flood — the contrast to the exhaustive methods.
class RRT : public IPlanner {
public:
    const char* name() const override { return "rrt"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0=nowMs(); PlanResult R;
        int sx,sy,gx,gy;
        if(!g.worldToCell(s.e,s.n,sx,sy)||!g.worldToCell(gl.e,gl.n,gx,gy)){R.planMs=nowMs()-t0;return R;}
        nearestFree(g,sx,sy,infl); nearestFree(g,gx,gy,infl);
        // Deterministic per-call PRNG seeded from the start+goal cells so runs
        // are reproducible (no wall-clock seed).
        std::mt19937 rng((uint32_t)(sx*73856093 ^ sy*19349663 ^ gx*83492791 ^ gy*2971215073u));
        std::uniform_int_distribution<int> ux(0,g.cells()-1), uy(0,g.cells()-1);
        std::vector<Cell> nodes = {{sx,sy}}; std::vector<int> parent = {-1};
        const int stepC = 4; const int maxIter = 4000; int expanded=0; int goalNode=-1;
        for(int it=0; it<maxIter && goalNode<0; ++it){
            ++expanded;
            int tx = (it%10==0)?gx:ux(rng), ty=(it%10==0)?gy:uy(rng);   // goal bias 10%
            // nearest existing node
            int best=0; float bd=1e18f;
            for(size_t i=0;i<nodes.size();++i){ float d=std::hypot((float)(nodes[i].x-tx),(float)(nodes[i].y-ty));
                if(d<bd){bd=d;best=(int)i;} }
            const float dx=tx-nodes[best].x, dy=ty-nodes[best].y; const float dd=std::hypot(dx,dy);
            if(dd<1e-3f) continue;
            const int nx=nodes[best].x+(int)std::round(dx/dd*stepC);
            const int ny=nodes[best].y+(int)std::round(dy/dd*stepC);
            if(!g.inBounds(nx,ny)||g.blocked(nx,ny,infl)) continue;
            // collision-check the segment coarsely
            bool clear=true; for(int q=1;q<=stepC;++q){
                int ix=nodes[best].x+(int)std::round(dx/dd*q), iy=nodes[best].y+(int)std::round(dy/dd*q);
                if(g.blocked(ix,iy,infl)){clear=false;break;} }
            if(!clear) continue;
            nodes.push_back({nx,ny}); parent.push_back(best);
            if(std::hypot((float)(nx-gx),(float)(ny-gy))<=stepC) goalNode=(int)nodes.size()-1;
        }
        R.expanded=expanded;
        if(goalNode<0){R.planMs=nowMs()-t0;return R;}
        std::vector<Cell> path; for(int i=goalNode;i>=0;i=parent[i]) path.push_back(nodes[i]);
        std::reverse(path.begin(),path.end());
        R.path=cellsToWorld(g,path); R.ok=true; R.planMs=nowMs()-t0; return R;
    }
};
}  // namespace

std::unique_ptr<IPlanner> makePlanner(const std::string& n) {
    if (n == "wavefront") return std::make_unique<Wavefront>();
    if (n == "dijkstra")  return std::make_unique<GridSearch>(false);
    if (n == "astar")     return std::make_unique<GridSearch>(true);
    if (n == "potential") return std::make_unique<PotentialField>();
    if (n == "rrt")       return std::make_unique<RRT>();
    return nullptr;
}

std::vector<std::string> plannerNames() {
    return {"wavefront", "dijkstra", "astar", "potential", "rrt"};
}

}  // namespace navsim
