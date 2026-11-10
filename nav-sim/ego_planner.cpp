#include "ego_planner.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

#include "voxel_map.hpp"

namespace sim {

namespace {
using V3 = EgoPlanner::V3;
V3 sub(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 add(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 mul(const V3& a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
float dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
float norm(const V3& a) { return std::sqrt(dot(a, a)); }
}  // namespace

float EgoPlanner::duration() const {
    return q_.size() >= 4 ? float(q_.size() - 3) * ts_ : 0.f;
}

// Uniform cubic B-spline, segment k covers [k ts, (k+1) ts].
V3 EgoPlanner::sample(float t) const {
    if (q_.size() < 4) return q_.empty() ? V3{0, 0, 0} : q_.front();
    const float T = duration();
    t = std::max(0.f, std::min(t, T));
    int k = int(t / ts_);
    if (k > int(q_.size()) - 4) k = int(q_.size()) - 4;
    const float u = t / ts_ - float(k);
    const float u2 = u * u, u3 = u2 * u;
    const float b0 = (1 - u) * (1 - u) * (1 - u) / 6.f;
    const float b1 = (3 * u3 - 6 * u2 + 4) / 6.f;
    const float b2 = (-3 * u3 + 3 * u2 + 3 * u + 1) / 6.f;
    const float b3 = u3 / 6.f;
    V3 r{0, 0, 0};
    for (int a = 0; a < 3; ++a)
        r[a] = b0 * q_[k][a] + b1 * q_[k + 1][a] + b2 * q_[k + 2][a] + b3 * q_[k + 3][a];
    return r;
}

// ------------------------------------------------------------------ the grid
// A box of the map's own cells around the aircraft, obstacles inflated by
// robotR - safeDistM (EGO inflates its map and then asks for a safe distance
// beyond the inflated surface). Unknown counts as free unless told otherwise.
void EgoPlanner::buildGrid(const VoxelMap& m, const V3& c) {
    map_ = &m;
    const VoxelMapParams& mp = m.params();
    cell_ = mp.cell;
    const int hx = int(std::ceil(p_.localHalfM / cell_));
    const int hz = int(std::ceil(p_.localHalfZM / cell_));
    int cx, cy, cz;
    m.worldToCell(c[0], c[1], c[2], cx, cy, cz);
    gx0_ = cx - hx; gy0_ = cy - hx; gz0_ = cz - hz;
    gnx_ = gny_ = 2 * hx + 1; gnz_ = 2 * hz + 1;
    std::vector<unsigned char> raw(size_t(gnx_) * gny_ * gnz_, 0);
    auto at = [&](int x, int y, int z) -> unsigned char& { return raw[(size_t(z) * gny_ + y) * gnx_ + x]; };
    for (int z = 0; z < gnz_; ++z)
        for (int y = 0; y < gny_; ++y)
            for (int x = 0; x < gnx_; ++x) {
                const int mx = gx0_ + x, my = gy0_ + y, mz = gz0_ + z;
                bool o;
                if (!m.inBounds(mx, my, mz)) o = !p_.unknownFree;
                else {
                    const float l = m.logAt(mx, my, mz);
                    o = p_.unknownFree ? (l > mp.occThresh) : !(l < mp.freeThresh);
                }
                at(x, y, z) = o ? 1 : 0;
            }
    occ_.assign(raw.size(), 0);
    const float rinf = std::max(0.f, p_.robotR - p_.safeDistM);
    const int r = int(std::floor(rinf / cell_ + 1e-4f));
    std::vector<std::array<int, 3>> offs;
    for (int dz = -r; dz <= r; ++dz)
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
                if ((dx * dx + dy * dy + dz * dz) * cell_ * cell_ <= rinf * rinf + 1e-6f) offs.push_back({dx, dy, dz});
    for (int z = 0; z < gnz_; ++z)
        for (int y = 0; y < gny_; ++y)
            for (int x = 0; x < gnx_; ++x) {
                if (!at(x, y, z)) continue;
                for (const auto& o : offs) {
                    const int X = x + o[0], Y = y + o[1], Z = z + o[2];
                    if (X < 0 || Y < 0 || Z < 0 || X >= gnx_ || Y >= gny_ || Z >= gnz_) continue;
                    occ_[(size_t(Z) * gny_ + Y) * gnx_ + X] = 1;
                }
            }
}

bool EgoPlanner::occCell(int x, int y, int z) const {
    x -= gx0_; y -= gy0_; z -= gz0_;
    if (x < 0 || y < 0 || z < 0 || x >= gnx_ || y >= gny_ || z >= gnz_) return !p_.unknownFree;
    return occ_[(size_t(z) * gny_ + y) * gnx_ + x] != 0;
}

bool EgoPlanner::occupied(const V3& w) const {
    int x, y, z;
    map_->worldToCell(w[0], w[1], w[2], x, y, z);
    return occCell(x, y, z);
}

// 26-connected A* on the inflated grid; the end points themselves may sit in
// an obstacle (a control point just outside a trunk often does).
bool EgoPlanner::astar(const V3& a, const V3& b, std::vector<V3>& path) {
    path.clear();
    int ax, ay, az, bx, by, bz;
    map_->worldToCell(a[0], a[1], a[2], ax, ay, az);
    map_->worldToCell(b[0], b[1], b[2], bx, by, bz);
    auto inBox = [&](int x, int y, int z) {
        return x >= gx0_ && y >= gy0_ && z >= gz0_ && x < gx0_ + gnx_ && y < gy0_ + gny_ && z < gz0_ + gnz_;
    };
    if (!inBox(ax, ay, az) || !inBox(bx, by, bz)) return false;
    auto key = [&](int x, int y, int z) { return (long long)((z - gz0_) * gny_ + (y - gy0_)) * gnx_ + (x - gx0_); };
    struct N { float f; long long k; };
    auto cmp = [](const N& l, const N& r) { return l.f > r.f; };
    std::priority_queue<N, std::vector<N>, decltype(cmp)> open(cmp);
    std::unordered_map<long long, float> g;
    std::unordered_map<long long, long long> parent;
    auto h = [&](int x, int y, int z) {
        return std::sqrt(float((x - bx) * (x - bx) + (y - by) * (y - by) + (z - bz) * (z - bz)));
    };
    const long long ks = key(ax, ay, az), kg = key(bx, by, bz);
    g[ks] = 0.f; parent[ks] = -1;
    open.push({h(ax, ay, az), ks});
    int expanded = 0;
    while (!open.empty()) {
        const N n = open.top(); open.pop();
        if (n.k == kg) break;
        if (++expanded > p_.astarMaxExpand) return false;
        const int x = int(n.k % gnx_) + gx0_;
        const int y = int((n.k / gnx_) % gny_) + gy0_;
        const int z = int(n.k / (long long)(gnx_ * gny_)) + gz0_;
        const float gn = g[n.k];
        if (n.f - h(x, y, z) > gn + 1e-4f) continue;          // stale entry
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy && !dz) continue;
                    const int X = x + dx, Y = y + dy, Z = z + dz;
                    if (!inBox(X, Y, Z)) continue;
                    const long long k = key(X, Y, Z);
                    if (k != kg && occCell(X, Y, Z)) continue;
                    const float c = gn + std::sqrt(float(dx * dx + dy * dy + dz * dz));
                    auto it = g.find(k);
                    if (it != g.end() && it->second <= c) continue;
                    g[k] = c; parent[k] = n.k;
                    open.push({c + h(X, Y, Z), k});
                }
    }
    if (!parent.count(kg)) return false;
    for (long long k = kg; k != -1; k = parent[k]) {
        const int x = int(k % gnx_) + gx0_;
        const int y = int((k / gnx_) % gny_) + gy0_;
        const int z = int(k / (long long)(gnx_ * gny_)) + gz0_;
        float wx, wy, wz;
        map_->cellCentre(x, y, z, wx, wy, wz);
        path.push_back({wx, wy, wz});
    }
    std::reverse(path.begin(), path.end());
    return true;
}

// EGO's {p, v} pairs: for each run of colliding control points that has none,
// an A* path round it; per point, where the plane through it normal to the
// trajectory meets that path gives the escape direction v, and p is where the
// march from the point along v leaves the obstacle.
int EgoPlanner::addPairs() {
    const int n = int(q_.size());
    int added = 0;
    auto needs = [&](int i) {
        if (!occupied(q_[i])) return false;
        for (const Pair& pr : pairs_[i])                  // still pushing outward?
            if (dot(sub(q_[i], pr.p), pr.v) < p_.safeDistM) return false;
        return true;
    };
    for (int i = 3; i < n; ++i) {
        if (!needs(i)) continue;
        int j = i;
        while (j + 1 < n && needs(j + 1)) ++j;
        const int a = i - 1, b = std::min(n - 1, j + 1);
        std::vector<V3> path;
        if (b == j || !astar(q_[a], q_[b], path) || path.size() < 2) {
            ++st_.astarFails;
            i = j;
            continue;
        }
        for (int k = i; k <= j; ++k) {
            V3 R = sub(q_[std::min(n - 1, k + 1)], q_[k - 1]);
            const float rn = norm(R);
            if (rn < 1e-6f) continue;
            R = mul(R, 1.f / rn);
            // where the path crosses the normal plane through q_k
            V3 A{0, 0, 0};
            bool found = false;
            for (size_t s = 1; s < path.size(); ++s) {
                const float d0 = dot(sub(path[s - 1], q_[k]), R), d1 = dot(sub(path[s], q_[k]), R);
                if ((d0 <= 0 && d1 >= 0) || (d0 >= 0 && d1 <= 0)) {
                    const float w = std::fabs(d0 - d1) > 1e-6f ? d0 / (d0 - d1) : 0.f;
                    A = add(path[s - 1], mul(sub(path[s], path[s - 1]), w));
                    found = true;
                    break;
                }
            }
            if (!found) continue;
            V3 v = sub(A, q_[k]);
            const float vn = norm(v);
            if (vn < 1e-3f) continue;
            v = mul(v, 1.f / vn);
            V3 p = A;
            for (float s = 0.f; s <= vn; s += 0.5f * cell_) {
                const V3 w = add(q_[k], mul(v, s));
                if (!occupied(w)) { p = w; break; }
            }
            pairs_[k].push_back({p, v});
            ++added;
        }
        i = j;
    }
    return added;
}

// The objective over the FREE control points (the first three are pinned by
// the start position and velocity).
double EgoPlanner::cost(const std::vector<double>& x, std::vector<double>& g) const {
    const int n = int(q_.size());
    std::vector<std::array<double, 3>> Q(n), G(n, {0, 0, 0});
    for (int i = 0; i < n; ++i)
        for (int a = 0; a < 3; ++a) Q[i][a] = i < 3 ? q_[i][a] : x[size_t(i - 3) * 3 + a];
    double J = 0;
    // smoothness: squared jerk of the control polygon
    for (int i = 0; i + 3 < n; ++i)
        for (int a = 0; a < 3; ++a) {
            const double j = Q[i + 3][a] - 3 * Q[i + 2][a] + 3 * Q[i + 1][a] - Q[i][a];
            J += p_.wSmooth * j * j;
            const double d = 2 * p_.wSmooth * j;
            G[i + 3][a] += d; G[i + 2][a] -= 3 * d; G[i + 1][a] += 3 * d; G[i][a] -= d;
        }
    // collision: EGO's piecewise cubic/quadratic on c = sf - d
    const double sf = p_.safeDistM;
    for (int i = 3; i < n; ++i)
        for (const Pair& pr : pairs_[i]) {
            double d = 0;
            for (int a = 0; a < 3; ++a) d += (Q[i][a] - pr.p[a]) * pr.v[a];
            const double c = sf - d;
            if (c <= 0) continue;
            double f, df;
            if (c <= sf) { f = c * c * c; df = 3 * c * c; }
            else { f = 3 * sf * c * c - 3 * sf * sf * c + sf * sf * sf; df = 6 * sf * c - 3 * sf * sf; }
            J += p_.wColl * f;
            for (int a = 0; a < 3; ++a) G[i][a] += p_.wColl * df * -pr.v[a];
        }
    // feasibility: per-axis velocity and acceleration over the limits, as a
    // quadratic hinge. SOFT on purpose: a curve that is too aggressive for the
    // time it was given is fixed by giving it more time (plan(), after the
    // optimisation, as EGO reallocates), not by refusing to bend round a trunk.
    // The first version squared (v^2 - vmax^2), and at 6 m/s^2 against a
    // 3 m/s^2 limit that gradient was ~200x the collision push: the curve
    // could not leave the straight line.
    const double ts = ts_;
    for (int i = 0; i + 1 < n; ++i)
        for (int a = 0; a < 3; ++a) {
            const double v = (Q[i + 1][a] - Q[i][a]) / ts;
            const double e = std::fabs(v) - p_.maxVel;
            if (e <= 0) continue;
            J += p_.wFeas * e * e;
            const double dv = p_.wFeas * 2 * e * (v > 0 ? 1 : -1) / ts;
            G[i + 1][a] += dv; G[i][a] -= dv;
        }
    for (int i = 0; i + 2 < n; ++i)
        for (int a = 0; a < 3; ++a) {
            const double ac = (Q[i + 2][a] - 2 * Q[i + 1][a] + Q[i][a]) / (ts * ts);
            const double e = std::fabs(ac) - p_.maxAcc;
            if (e <= 0) continue;
            J += p_.wFeas * e * e;
            const double da = p_.wFeas * 2 * e * (ac > 0 ? 1 : -1) / (ts * ts);
            G[i + 2][a] += da; G[i + 1][a] -= 2 * da; G[i][a] += da;
        }
    // the end of the curve at the local target
    for (int a = 0; a < 3; ++a) {
        const double e = (Q[n - 3][a] + 4 * Q[n - 2][a] + Q[n - 1][a]) / 6.0 - goal_[a];
        J += p_.wGoal * e * e;
        const double d = 2 * p_.wGoal * e / 6.0;
        G[n - 3][a] += d; G[n - 2][a] += 4 * d; G[n - 1][a] += d;
    }
    g.assign(size_t(n - 3) * 3, 0.0);
    for (int i = 3; i < n; ++i)
        for (int a = 0; a < 3; ++a) g[size_t(i - 3) * 3 + a] = G[i][a];
    return J;
}

// L-BFGS (memory 8) with a backtracking Armijo line search.
bool EgoPlanner::optimise() {
    const int n = int(q_.size());
    const size_t N = size_t(n - 3) * 3;
    std::vector<double> x(N), g, xn(N), gn;
    for (int i = 3; i < n; ++i)
        for (int a = 0; a < 3; ++a) x[size_t(i - 3) * 3 + a] = q_[i][a];
    double f = cost(x, g);
    const int M = 8;
    std::vector<std::vector<double>> S, Y;
    std::vector<double> rho;
    for (int it = 0; it < p_.maxIters; ++it) {
        double gmax = 0;
        for (double v : g) gmax = std::max(gmax, std::fabs(v));
        if (gmax < 1e-5) break;
        std::vector<double> d(g);
        std::vector<double> al(S.size());
        for (int k = int(S.size()) - 1; k >= 0; --k) {
            double s = 0; for (size_t j = 0; j < N; ++j) s += S[k][j] * d[j];
            al[k] = rho[k] * s;
            for (size_t j = 0; j < N; ++j) d[j] -= al[k] * Y[k][j];
        }
        if (!S.empty()) {
            double sy = 0, yy = 0;
            for (size_t j = 0; j < N; ++j) { sy += S.back()[j] * Y.back()[j]; yy += Y.back()[j] * Y.back()[j]; }
            const double gam = yy > 0 ? sy / yy : 1.0;
            for (double& v : d) v *= gam;
        } else {
            const double s = 1.0 / std::max(1.0, gmax);
            for (double& v : d) v *= s;
        }
        for (size_t k = 0; k < S.size(); ++k) {
            double s = 0; for (size_t j = 0; j < N; ++j) s += Y[k][j] * d[j];
            const double b = rho[k] * s;
            for (size_t j = 0; j < N; ++j) d[j] += S[k][j] * (al[k] - b);
        }
        for (double& v : d) v = -v;
        double gd = 0; for (size_t j = 0; j < N; ++j) gd += g[j] * d[j];
        if (gd >= 0) { for (size_t j = 0; j < N; ++j) d[j] = -g[j]; gd = 0; for (double v : g) gd -= v * v; S.clear(); Y.clear(); rho.clear(); }
        double step = 1.0, fn = f;
        bool ok = false;
        for (int ls = 0; ls < 30; ++ls) {
            for (size_t j = 0; j < N; ++j) xn[j] = x[j] + step * d[j];
            fn = cost(xn, gn);
            if (fn <= f + 1e-4 * step * gd) { ok = true; break; }
            step *= 0.5;
        }
        ++st_.iters;
        if (!ok) break;
        std::vector<double> s(N), y(N);
        double sy = 0;
        for (size_t j = 0; j < N; ++j) { s[j] = xn[j] - x[j]; y[j] = gn[j] - g[j]; sy += s[j] * y[j]; }
        if (sy > 1e-12) {
            S.push_back(s); Y.push_back(y); rho.push_back(1.0 / sy);
            if (int(S.size()) > M) { S.erase(S.begin()); Y.erase(Y.begin()); rho.erase(rho.begin()); }
        }
        const double df = f - fn;
        x.swap(xn); g.swap(gn); f = fn;
        if (df < 1e-9 * std::max(1.0, std::fabs(f))) break;
    }
    for (int i = 3; i < n; ++i)
        for (int a = 0; a < 3; ++a) q_[i][a] = float(x[size_t(i - 3) * 3 + a]);
    return std::isfinite(f);
}

bool EgoPlanner::plan(const VoxelMap& m, const V3& pos, const V3& vel, const V3& waypoint) {
    ++st_.plans;
    valid_ = false;
    buildGrid(m, pos);
    // The local target: planHorizonM along the straight line to the waypoint.
    V3 dir = sub(waypoint, pos);
    const float dist = norm(dir);
    goal_ = dist > p_.planHorizonM ? add(pos, mul(dir, p_.planHorizonM / dist)) : waypoint;
    const float span = norm(sub(goal_, pos));
    ts_ = p_.ctrlDistM / p_.maxVel;
    // Pinned start: position pos, velocity vel, zero acceleration.
    const int nFree = std::max(4, int(std::ceil(span / p_.ctrlDistM)));
    q_.assign(size_t(nFree) + 3, pos);
    q_[0] = sub(pos, mul(vel, ts_));
    q_[1] = pos;
    q_[2] = add(pos, mul(vel, ts_));
    for (int i = 3; i < int(q_.size()); ++i) {
        const float w = float(i - 2) / float(nFree);
        q_[i] = add(q_[2], mul(sub(goal_, q_[2]), w));
    }
    pairs_.assign(q_.size(), {});
    addPairs();
    if (!optimise()) { ++st_.failures; return false; }
    for (int r = 0; r < p_.rebounds; ++r) {
        if (addPairs() == 0) break;
        ++st_.rebounds;
        if (!optimise()) { ++st_.failures; return false; }
    }
    // TIME REALLOCATION, as EGO does: if the curve needs more speed or
    // acceleration than allowed, stretch the knot interval until it does not.
    // The geometry is kept; only the time along it changes.
    {
        float vmax = 0.f, amax = 0.f;
        for (size_t i = 0; i + 1 < q_.size(); ++i)
            for (int a = 0; a < 3; ++a) vmax = std::max(vmax, std::fabs(q_[i + 1][a] - q_[i][a]) / ts_);
        for (size_t i = 0; i + 2 < q_.size(); ++i)
            for (int a = 0; a < 3; ++a)
                amax = std::max(amax, std::fabs(q_[i + 2][a] - 2 * q_[i + 1][a] + q_[i][a]) / (ts_ * ts_));
        const float r = std::max({1.f, vmax / p_.maxVel, std::sqrt(amax / p_.maxAcc)});
        if (r > 1.01f) { ts_ *= r; ++st_.reallocations; }
    }
    int still = 0;
    for (size_t i = 3; i < q_.size(); ++i) still += occupied(q_[i]) ? 1 : 0;
    st_.collidingAfter += still;
    valid_ = true;
    return true;
}

}  // namespace sim
