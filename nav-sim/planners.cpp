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
constexpr float kPi    = 3.14159265358979323846f;
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

// Grid line-of-sight (supercover Bresenham): is the straight segment (x0,y0)->
// (x1,y1) free of blocked cells? Used by any-angle (Theta*) and sampling (RRT*).
bool lineOfSight(const OccupancyGrid& g, int x0, int y0, int x1, int y1, int infl) {
    int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1, err = dx-dy;
    while (true) {
        if (g.blocked(x0,y0,infl)) return false;
        if (x0==x1 && y0==y1) return true;
        const int e2 = 2*err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

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

// ------------------- Weighted best-first family (dijkstra/astar/greedy/wA*)
// One search parametrised by priority = wG*g + wH*h:
//   dijkstra    wG=1 wH=0   (uniform cost, no heuristic)
//   astar       wG=1 wH=1   (optimal, admissible)
//   greedy      wG=0 wH=1   (pure heuristic — fast, ignores cost, suboptimal)
//   weighted-A* wG=1 wH=2.5 (inflated heuristic — faster, bounded-suboptimal)
class GridSearch : public IPlanner {
public:
    GridSearch(const char* nm, float wG, float wH) : name_(nm), wG_(wG), wH_(wH) {}
    const char* name() const override { return name_; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0 = nowMs(); PlanResult R;
        int sx, sy, gx, gy;
        if (!g.worldToCell(s.e, s.n, sx, sy) || !g.worldToCell(gl.e, gl.n, gx, gy)) { R.planMs=nowMs()-t0; return R; }
        nearestFree(g, sx, sy, infl); nearestFree(g, gx, gy, infl);
        const int C = g.cells();
        std::vector<float> gcost(C*C, 1e18f);
        std::vector<int>   parent(C*C, -1);
        std::vector<char>  closed(C*C, 0);
        auto h = [&](int x,int y){ const float dx=std::abs(x-gx), dy=std::abs(y-gy);
            return (dx+dy) + (kSqrt2-2.f)*std::min(dx,dy); };   // octile
        typedef std::pair<float,int> PQe;
        std::priority_queue<PQe, std::vector<PQe>, std::greater<PQe>> pq;
        gcost[sy*C+sx]=0.f; pq.push({wH_*h(sx,sy), sy*C+sx});
        int expanded=0; bool found=false;
        while(!pq.empty()){
            const int id=pq.top().second; pq.pop();
            if (closed[id]) continue; closed[id]=1;
            const int cx=id%C, cy=id/C; ++expanded;
            if (cx==gx&&cy==gy){found=true;break;}
            for(int k=0;k<8;++k){ int nx=cx+DX8[k], ny=cy+DY8[k];
                if(!g.inBounds(nx,ny)||g.blocked(nx,ny,infl)||closed[ny*C+nx]) continue;
                const float ng=gcost[id]+stepCost(k);
                if(ng<gcost[ny*C+nx]){ gcost[ny*C+nx]=ng; parent[ny*C+nx]=id;
                    pq.push({wG_*ng + wH_*h(nx,ny), ny*C+nx}); } }
        }
        R.expanded=expanded;
        if(!found){ R.planMs=nowMs()-t0; return R; }
        std::vector<Cell> path; for(int id=gy*C+gx; id>=0; id=parent[id]) path.push_back({id%C,id/C});
        std::reverse(path.begin(), path.end());
        R.path=cellsToWorld(g,path); R.ok=!path.empty(); R.planMs=nowMs()-t0; return R;
    }
private:
    const char* name_; float wG_, wH_;
};

// ---------------------------------------------------------- Theta* (any-angle)
// A* whose relaxation shortcuts through the parent's parent when there's line of
// sight — so the path isn't locked to 8 grid directions and comes out shorter
// and smoother (fewer, longer straight legs) than A*.
class ThetaStar : public IPlanner {
public:
    const char* name() const override { return "theta*"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0=nowMs(); PlanResult R;
        int sx,sy,gx,gy;
        if(!g.worldToCell(s.e,s.n,sx,sy)||!g.worldToCell(gl.e,gl.n,gx,gy)){R.planMs=nowMs()-t0;return R;}
        nearestFree(g,sx,sy,infl); nearestFree(g,gx,gy,infl);
        const int C=g.cells();
        std::vector<float> gcost(C*C,1e18f); std::vector<int> parent(C*C,-1); std::vector<char> closed(C*C,0);
        auto dist=[&](int x0,int y0,int x1,int y1){ return std::hypot((float)(x0-x1),(float)(y0-y1)); };
        auto h=[&](int x,int y){ return dist(x,y,gx,gy); };
        typedef std::pair<float,int> PQe;
        std::priority_queue<PQe,std::vector<PQe>,std::greater<PQe>> pq;
        gcost[sy*C+sx]=0.f; parent[sy*C+sx]=sy*C+sx; pq.push({h(sx,sy),sy*C+sx});
        int expanded=0; bool found=false;
        while(!pq.empty()){
            const int id=pq.top().second; pq.pop(); if(closed[id])continue; closed[id]=1;
            const int cx=id%C, cy=id/C; ++expanded;
            if(cx==gx&&cy==gy){found=true;break;}
            for(int k=0;k<8;++k){ int nx=cx+DX8[k], ny=cy+DY8[k];
                if(!g.inBounds(nx,ny)||g.blocked(nx,ny,infl)||closed[ny*C+nx])continue;
                const int par=parent[id], px=par%C, py=par/C;
                float ng; int chosenPar;
                if(lineOfSight(g,px,py,nx,ny,infl)){ ng=gcost[par]+dist(px,py,nx,ny); chosenPar=par; }  // any-angle
                else                               { ng=gcost[id]+dist(cx,cy,nx,ny);  chosenPar=id; }
                if(ng<gcost[ny*C+nx]){ gcost[ny*C+nx]=ng; parent[ny*C+nx]=chosenPar; pq.push({ng+h(nx,ny),ny*C+nx}); } }
        }
        R.expanded=expanded; if(!found){R.planMs=nowMs()-t0;return R;}
        std::vector<Cell> path; for(int id=gy*C+gx;;){ path.push_back({id%C,id/C}); if(id==parent[id])break; id=parent[id]; }
        std::reverse(path.begin(),path.end());
        R.path=cellsToWorld(g,path); R.ok=!path.empty(); R.planMs=nowMs()-t0; return R;
    }
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

// ---------------------------------------------------------------------- RRT*
// RRT with asymptotic optimality: each new node picks the cheapest parent among
// nearby nodes (with line of sight), then rewires nearby nodes through it if
// that's cheaper. Straighter, shorter trees than plain RRT for the same samples.
class RRTStar : public IPlanner {
public:
    const char* name() const override { return "rrt*"; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0=nowMs(); PlanResult R;
        int sx,sy,gx,gy;
        if(!g.worldToCell(s.e,s.n,sx,sy)||!g.worldToCell(gl.e,gl.n,gx,gy)){R.planMs=nowMs()-t0;return R;}
        nearestFree(g,sx,sy,infl); nearestFree(g,gx,gy,infl);
        std::mt19937 rng((uint32_t)(sx*73856093 ^ sy*19349663 ^ gx*83492791 ^ gy*2971215073u));
        std::uniform_int_distribution<int> ux(0,g.cells()-1), uy(0,g.cells()-1);
        std::vector<Cell> nd={{sx,sy}}; std::vector<int> par={-1}; std::vector<float> cost={0.f};
        auto dist=[&](Cell a,Cell b){ return std::hypot((float)(a.x-b.x),(float)(a.y-b.y)); };
        const int stepC=4, maxIter=3000; const float rewireR=6.f; int expanded=0, goalNode=-1;
        for(int it=0; it<maxIter; ++it){ ++expanded;
            int tx=(it%10==0)?gx:ux(rng), ty=(it%10==0)?gy:uy(rng);
            int best=0; float bd=1e18f;
            for(size_t i=0;i<nd.size();++i){ float d=dist(nd[i],{tx,ty}); if(d<bd){bd=d;best=(int)i;} }
            const float dx=tx-nd[best].x, dy=ty-nd[best].y, dd=std::hypot(dx,dy); if(dd<1e-3f)continue;
            Cell nn{ nd[best].x+(int)std::round(dx/dd*stepC), nd[best].y+(int)std::round(dy/dd*stepC) };
            if(!g.inBounds(nn.x,nn.y)||g.blocked(nn.x,nn.y,infl)) continue;
            if(!lineOfSight(g,nd[best].x,nd[best].y,nn.x,nn.y,infl)) continue;
            // choose the cheapest parent among near nodes with LOS
            int bp=best; float bc=cost[best]+dist(nd[best],nn);
            for(size_t i=0;i<nd.size();++i){ if(dist(nd[i],nn)>rewireR) continue;
                const float c=cost[i]+dist(nd[i],nn);
                if(c<bc && lineOfSight(g,nd[i].x,nd[i].y,nn.x,nn.y,infl)){ bc=c; bp=(int)i; } }
            const int ni=(int)nd.size(); nd.push_back(nn); par.push_back(bp); cost.push_back(bc);
            // rewire near nodes through the new node
            for(size_t i=0;i<nd.size()-1;++i){ if(dist(nd[i],nn)>rewireR) continue;
                const float c=bc+dist(nn,nd[i]);
                if(c<cost[i] && lineOfSight(g,nn.x,nn.y,nd[i].x,nd[i].y,infl)){ par[i]=ni; cost[i]=c; } }
            if(dist(nn,{gx,gy})<=stepC && (goalNode<0 || bc<cost[goalNode])) goalNode=ni;
        }
        R.expanded=expanded; if(goalNode<0){R.planMs=nowMs()-t0;return R;}
        std::vector<Cell> path; for(int i=goalNode;i>=0;i=par[i]) path.push_back(nd[i]);
        std::reverse(path.begin(),path.end());
        R.path=cellsToWorld(g,path); R.ok=true; R.planMs=nowMs()-t0; return R;
    }
};

// ----------------------------------------------------------------------- Bug2
// A classic REACTIVE method with memory: head along the start->goal "m-line";
// when blocked, follow the obstacle boundary (right-hand rule) until back on the
// m-line closer to the goal, then resume. It works on simple/sparse boundaries
// (e.g. the `rooms` doorway) but — like all bug algorithms — is sensitive to the
// boundary-following heuristic, and here also to partial observability and the
// vehicle's turn-rate limit, so it can fail to trace complex or concave
// boundaries (trap / maze). Included to show the reactive family honestly, not
// as a trap-escaping hero. Stateful -> uses reset() per episode.
class Bug2 : public IPlanner {
public:
    const char* name() const override { return "bug2"; }
    void reset() override { mode_=SEEK; haveM_=false; }
    PlanResult plan(const OccupancyGrid& g, Vec2 s, Vec2 gl, int infl) override {
        const double t0=nowMs(); PlanResult R;
        if(!haveM_){ s0_=s; g0_=gl; haveM_=true; }   // m-line = first start -> goal
        const float toGoal = std::atan2(gl.e-s.e, gl.n-s.n);   // rad, 0=N
        int sx,sy; g.worldToCell(s.e,s.n,sx,sy);
        auto blockedAt=[&](float bearingRad,float dm){
            const float de=std::sin(bearingRad), dn=std::cos(bearingRad);
            int cx,cy; return g.worldToCell(s.e+de*dm, s.n+dn*dm, cx,cy) ? g.blocked(cx,cy,infl) : true; };
        const float probe = 1.6f;
        // distance from a point to the m-line (start->goal)
        auto mlineDist=[&](Vec2 p){ const float ae=g0_.e-s0_.e, an=g0_.n-s0_.n; const float L=std::hypot(ae,an);
            if(L<1e-3f) return 0.f; return std::fabs((p.e-s0_.e)*an-(p.n-s0_.n)*ae)/L; };
        const float distGoalNow=std::hypot(gl.e-s.e, gl.n-s.n);

        const float toGoalDeg = toGoal*180.f/kPi;
        float bearing;
        if(mode_==SEEK){
            if(!blockedAt(toGoal,probe)){ bearing=toGoalDeg; }
            else { mode_=FOLLOW; hitGoalDist_=distGoalNow; followHead_=toGoalDeg; bearing=wallFollow_(blockedAt); }
        } else {  // FOLLOW
            // leave the wall when back on the m-line AND closer than the hit point
            if(mlineDist(s)<0.7f && distGoalNow < hitGoalDist_-0.3f && !blockedAt(toGoal,probe)){
                mode_=SEEK; bearing=toGoalDeg;
            } else bearing=wallFollow_(blockedAt);
        }
        const float b=bearing*kPi/180.f;
        R.path={ s, { s.e+std::sin(b)*g.cellM()*3.f, s.n+std::cos(b)*g.cellM()*3.f } };
        R.ok=true; R.expanded=1; R.planMs=nowMs()-t0; return R;
    }
private:
    enum Mode{SEEK,FOLLOW} mode_=SEEK;
    bool haveM_=false; Vec2 s0_, g0_; float hitGoalDist_=0.f; float followHead_=0.f;
    // Right-hand wall follow with a PERSISTENT heading: try to turn right (hug
    // the wall), else straight, else left, else back — the classic boundary
    // tracer. Persisting the heading is what makes it trace a boundary instead
    // of jittering. `blockedAt` takes a bearing in RADIANS.
    template<class F> float wallFollow_(F blockedAt){
        const int order[] = {-90,-45,0,45,90,135,180,-135};
        for(int t : order){ const float cand=followHead_+t;
            if(!blockedAt(cand*kPi/180.f, 1.6f)){ followHead_=cand; return cand; } }
        followHead_+=180.f; return followHead_;
    }
};
}  // namespace

std::unique_ptr<IPlanner> makePlanner(const std::string& n) {
    if (n == "wavefront")      return std::make_unique<Wavefront>();
    if (n == "dijkstra")       return std::make_unique<GridSearch>("dijkstra", 1.f, 0.f);
    if (n == "astar")          return std::make_unique<GridSearch>("astar",    1.f, 1.f);
    if (n == "greedy")         return std::make_unique<GridSearch>("greedy",   0.f, 1.f);
    if (n == "weighted-astar") return std::make_unique<GridSearch>("weighted-astar", 1.f, 2.5f);
    if (n == "theta*")         return std::make_unique<ThetaStar>();
    if (n == "potential")      return std::make_unique<PotentialField>();
    if (n == "rrt")            return std::make_unique<RRT>();
    if (n == "rrt*")           return std::make_unique<RRTStar>();
    if (n == "bug2")           return std::make_unique<Bug2>();
    return nullptr;
}

std::vector<std::string> plannerNames() {
    return {"wavefront", "dijkstra", "astar", "greedy", "weighted-astar",
            "theta*", "potential", "rrt", "rrt*", "bug2"};
}

}  // namespace navsim
