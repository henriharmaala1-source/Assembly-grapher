// LocalMap (P5b): occupancy-grid integration + wavefront planning. Feeds
// synthetic ray scans of a known obstacle field and checks the planner routes
// around what the grid has learned — the memory the reactive layer lacks.

#include <cmath>
#include <cstdio>
#include <vector>

#include "nav_map.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {
constexpr float kPi = 3.14159265358979323846f;
struct Obs { float e, n, r; };

float wrap180(float d) { while (d > 180) d -= 360; while (d <= -180) d += 360; return d; }

// Clearance from p along unit (de,dn) to circle (o), capped at maxRange.
float rayClearance(float pe, float pn, float de, float dn, const Obs& o, float maxRange) {
    const float me = pe - o.e, mn = pn - o.n;
    const float b = me * de + mn * dn, cc = me * me + mn * mn - o.r * o.r;
    const float disc = b * b - cc;
    if (disc < 0) return maxRange;
    const float sq = std::sqrt(disc), t1 = -b - sq, t2 = -b + sq;
    if (t2 < 0) return maxRange;
    if (t1 < 0) return 0.f;
    return std::min(t1, maxRange);
}

// A 41-ray, 60° scan of the obstacle field from pose (pe,pn,yaw).
std::vector<float> synthScan(float pe, float pn, float yaw,
                             const std::vector<Obs>& obs, float maxM) {
    const int N = 41; const float half = 30.f;
    std::vector<float> scan(N, maxM);
    for (int i = 0; i < N; ++i) {
        const float rel = -half + 60.f * i / (N - 1);
        const float b = (yaw + rel) * kPi / 180.f, de = std::sin(b), dn = std::cos(b);
        float c = maxM;
        for (const auto& o : obs) c = std::min(c, rayClearance(pe, pn, de, dn, o, maxM));
        scan[i] = c;
    }
    return scan;
}
}  // namespace

int main() {
    const float maxM = 8.f;

    { // empty grid → plan points essentially straight at the goal bearing
        LocalMap m;
        float bearing = 0.f;
        CHECK(m.plan(0, 0, 90.f, bearing));           // goal due East
        CHECK(std::fabs(wrap180(bearing - 90.f)) < 12.f);
    }

    { // obstacle dead ahead → after one scan, the plan deflects off the goal
        LocalMap m;
        const std::vector<Obs> obs = {{6.f, 0.f, 2.f}};   // front at 4 m, within range
        auto scan = synthScan(0, 0, 90.f, obs, maxM);
        m.integrate(0, 0, 90.f, scan.data(), (int)scan.size(), 60.f, maxM);

        // The grid learned the obstacle's front face.
        int hit = 0;
        for (int ix = 0; ix < m.width(); ++ix)
            for (int iy = 0; iy < m.height(); ++iy)
                if (m.occupied(ix, iy)) ++hit;
        CHECK(hit > 0);

        float bearing = 0.f;
        CHECK(m.plan(0, 0, 90.f, bearing));               // still want to go East
        CHECK(std::fabs(wrap180(bearing - 90.f)) > 3.f);  // but routed around
    }

    { // accumulate scans while circling toward a centred obstacle; the plan
      // should stay valid and keep deflecting to one side (never straight in)
        LocalMap m;
        const std::vector<Obs> obs = {{12.f, 0.f, 3.f}};   // inflated r = 4.5
        // Integrate from a few approach points, all OUTSIDE the inflated disc.
        const float pts[][2] = {{0, 0}, {3, 0.5f}, {6, 1.5f}};
        for (auto& pt : pts) {
            auto scan = synthScan(pt[0], pt[1], 90.f, obs, maxM);
            m.integrate(pt[0], pt[1], 90.f, scan.data(), (int)scan.size(), 60.f, maxM);
        }
        float bearing = 0.f;
        CHECK(m.plan(6, 1.5f, 90.f, bearing));            // NW of the disc, clear of it
        // Routed bearing must not aim into the disc centre (bearing to centre
        // from (6,1.5) ≈ 104°); a good plan heads more northward/east around it.
        const float toCentre = std::atan2(12.f - 6.f, 0.f - 1.5f) * 180.f / kPi;
        CHECK(std::fabs(wrap180(bearing - toCentre)) > 10.f);
    }

    { // a determinstic obstacle-free lane: plan should thread it
        LocalMap m;
        // Two obstacles leaving a gap to the north-east.
        const std::vector<Obs> obs = {{10.f, -2.f, 2.f}, {10.f, 6.f, 2.f}};
        for (float px = 0; px <= 4; px += 2) {
            auto scan = synthScan(px, 0, 90.f, obs, maxM);
            m.integrate(px, 0, 90.f, scan.data(), (int)scan.size(), 60.f, maxM);
        }
        float bearing = 0.f;
        CHECK(m.plan(0, 0, 90.f, bearing));
        // The gap centre is near N=2; a sane plan biases toward it, not into
        // either disc (centres at N=-2 and N=6).
        CHECK(bearing < 90.f);   // gap is left/north of due-East from (0,0)
    }

    std::printf("test_nav_map: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
