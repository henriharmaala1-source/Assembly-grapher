// The voxel layer aboard: VoxelNavModule (D435i -> voxel -> plan) and the
// mission flying its plan, with no camera and no aircraft.
//
// The module takes any FrameSource. The one used here is navcore's simulated
// camera wrapped so that it reports ONLY ATTITUDE -- exactly what the D435i
// gives the aircraft -- so what is under test is the architecture-C path that
// flies: a map per vantage at a fixed origin, integrated only while still, and
// legs certified on the straight line they fly. Ground truth is kept on the
// side, and it is what every safety assertion is checked against.
//
//   1. the vantage contract: moving publishes nothing valid, and stopping
//      again starts a NEW map
//   2. a certified leg is clear IN GROUND TRUTH, facing a wall
//   3. the mission gates: THINK waits for a mature map, a blocked plan turns
//      to look, a leg is only as long as its certificate, no forward pitch
//      until the nose is on the certified line
//   4. closed loop in a pillar field: module + mission + a lagged kinematic
//      airframe, scored on what this project scores on -- distance flown
//      without hitting anything
//
// Env: VOXTEST_WORLD=<seed> re-rolls the pillars, VOXTEST_STEREO=1 renders
// through the simulated stereo matcher instead of perfect depth,
// VOXTEST_TRACE=1 prints every leg and phase change. What this found when it
// was first run is recorded in navcore/nav_pipeline.{hpp,cpp}: legs certified
// past the marking range (0.27 m truth clearance, then STUCK), and a wall at
// grazing incidence that stereo never returned (0.03 m).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "frame_source.hpp"
#include "mission.hpp"
#include "voxel_nav.hpp"
#include "voxel_world.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

namespace {

constexpr float kPi = 3.14159265358979f;

// The D435i as the aircraft sees it: depth plus attitude, and no position.
// The truth pose is held by the test and rendered from, then withheld.
class AttitudeOnlySource : public sim::FrameSource {
public:
    AttitudeOnlySource(const sim::VoxelWorld& w, const sim::CamParams& p,
                       const sim::CamPose* truth)
        : sim_(w, p, /*truth depth*/ !std::getenv("VOXTEST_STEREO")), truth_(truth) {}
    const char* name() const override { return "sim-attitude-only"; }
    bool ok() const override { return true; }
    const sim::CamParams& params() const override { return sim_.params(); }
    const sim::DepthCamera& camera() const override { return sim_.camera(); }
    bool next(cv::Mat& depth, sim::PoseHint& hint) override {
        sim_.setPose(*truth_);
        sim::PoseHint full;
        if (!sim_.next(depth, full)) return false;
        hint.valid = true;
        hint.attitudeOnly = true;          // translation is NOT given
        hint.pose = sim::CamPose{};
        hint.pose.rollDeg  = full.pose.rollDeg;
        hint.pose.pitchDeg = full.pose.pitchDeg;
        hint.pose.yawDeg   = full.pose.yawDeg;
        ++frames;
        return true;
    }
    int frames = 0;
private:
    sim::SimFrameSource sim_;
    const sim::CamPose* truth_;
};

sim::CamParams d435i() {
    // The D435i's depth geometry: 87 deg horizontal, 50 mm baseline. 424x240
    // rather than 848x480 so the suite stays quick; fineMapParams derives the
    // honest range from THIS camera, so the map is configured for it.
    sim::CamParams p;
    p.width = 424; p.height = 240; p.hfovDeg = 87.f; p.baselineM = 0.05f;
    // VOXTEST_FULLRES=1: the resolution onboard actually runs (848x480), which
    // halves stereo error at range and lengthens the honest marking range.
    if (std::getenv("VOXTEST_FULLRES")) { p.width = 848; p.height = 480; }
    return p;
}

// SURFACE TEXTURE, which the stereo model matches on. The first version of
// this test set none: every surface was texture 0, below the matcher's 0.25
// threshold, so under VOXTEST_STEREO the camera saw nothing but silhouette
// edges -- the most pessimistic world there is, and every stereo number it
// produced was measured in it. Values follow navcore's own worlds: plaster
// walls 0.35 (voxel_world.hpp says why not 0.10), ground and bark richer.
constexpr float kGroundTex = 0.5f, kWallTex = 0.35f, kPillarTex = 0.55f;

// Ground slab + perimeter walls: returns everywhere, so free space can be
// carved. Unknown is not free, and an empty world would be all unknown.
void buildRoom(sim::VoxelWorld& w, float sizeM, float heightM, float cell) {
    const int n = int(sizeM / cell), nz = int(heightM / cell);
    w.init(cell, 0.f, 0.f, 0.f, n, n, nz);
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            w.set(x, y, 0); w.setTex(x, y, 0, kGroundTex);
            if (x < 2 || y < 2 || x >= n - 2 || y >= n - 2)
                for (int z = 0; z < nz; ++z) { w.set(x, y, z); w.setTex(x, y, z, kWallTex); }
        }
}

void box(sim::VoxelWorld& w, float x0, float y0, float x1, float y1, float h,
         float tex) {
    int ax, ay, az, bx, by, bz;
    w.worldToCell(x0, y0, 0.f, ax, ay, az);
    w.worldToCell(x1, y1, h, bx, by, bz);
    for (int z = 0; z <= bz; ++z)
        for (int y = ay; y <= by; ++y)
            for (int x = ax; x <= bx; ++x) { w.set(x, y, z); w.setTex(x, y, z, tex); }
}

// Distance from a point to the nearest solid voxel (box, not centre), up to
// `upTo`. Ground truth -- the map never sees this.
float clearance(const sim::VoxelWorld& w, float px, float py, float pz, float upTo) {
    const float c = w.cell();
    int cx, cy, cz; w.worldToCell(px, py, pz, cx, cy, cz);
    const int r = int(std::ceil(upTo / c)) + 1;
    float best = upTo;
    for (int z = cz - r; z <= cz + r; ++z)
        for (int y = cy - r; y <= cy + r; ++y)
            for (int x = cx - r; x <= cx + r; ++x) {
                if (!w.solid(x, y, z)) continue;
                const float bx0 = w.ox() + x * c, by0 = w.oy() + y * c, bz0 = w.oz() + z * c;
                const float dx = std::max({bx0 - px, 0.f, px - (bx0 + c)});
                const float dy = std::max({by0 - py, 0.f, py - (by0 + c)});
                const float dz = std::max({bz0 - pz, 0.f, pz - (bz0 + c)});
                best = std::min(best, std::sqrt(dx*dx + dy*dy + dz*dz));
            }
    return best;
}

// Worst truth clearance along a straight, level segment.
float segmentClearance(const sim::VoxelWorld& w, float e, float n, float u,
                       float azDeg, float len, float upTo) {
    const float a = azDeg * kPi / 180.f;
    float worst = upTo;
    for (float t = 0.f; t <= len; t += 0.05f)
        worst = std::min(worst, clearance(w, e + std::sin(a) * t,
                                          n + std::cos(a) * t, u, upTo));
    return worst;
}

WorldState stillState() {
    WorldState s;
    s.tickMonoS = monoNowS();
    s.vehGroundspeed = 0.f;
    return s;
}

}  // namespace

int main() {
    const sim::CamParams cp = d435i();
    VoxelNavModule::Params vp;
    vp.legMaxM = 6.f;

    // ---------------------------------------------------------------- 1
    std::printf("vantage contract\n");
    {
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        sim::CamPose truth; truth.e = 8.f; truth.n = 4.f; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);
        CHECK(mod.isReady());
        WorldModel wm;
        wm.with([](WorldState& s) { s = stillState(); });
        for (int i = 0; i < 4; ++i) mod.run(cv::Mat(), wm);
        WorldState s = wm.snapshot();
        CHECK(s.voxValid && !s.voxMoving);
        CHECK(s.voxFrames == 4);
        CHECK(mod.resets() == 1);

        wm.with([](WorldState& s) { s.vehGroundspeed = 1.2f; });   // flying
        mod.run(cv::Mat(), wm);
        s = wm.snapshot();
        CHECK(!s.voxValid && s.voxMoving && s.voxFrames == 0);

        wm.with([](WorldState& s) { s.vehGroundspeed = 0.f; });    // stopped
        mod.run(cv::Mat(), wm);
        s = wm.snapshot();
        CHECK(s.voxValid && s.voxFrames == 1);   // a NEW map, not the old one
        CHECK(mod.resets() == 2);

        // Under a mission, SETTLE is not a vantage even at zero speed: that is
        // where the last leg's drift is still bleeding off.
        wm.with([](WorldState& s) { s.missionActive = true; s.missionPhase = "SETTLE"; });
        mod.run(cv::Mat(), wm);
        CHECK(!wm.snapshot().voxValid);
        wm.with([](WorldState& s) { s.missionPhase = "THINK"; });
        mod.run(cv::Mat(), wm);
        CHECK(wm.snapshot().voxValid && wm.snapshot().voxFrames == 1);
        wm.with([](WorldState& s) { s.missionPhase = "SCAN"; });   // rotation:
        mod.run(cv::Mat(), wm);                                   // same map
        CHECK(wm.snapshot().voxFrames == 2);
        std::printf("  ok: moving -> invalid; still again -> new map; SETTLE excluded\n");
    }

    // ---------------------------------------------------------------- 2
    std::printf("a certified leg is clear in ground truth\n");
    {
        sim::VoxelWorld w; buildRoom(w, 16.f, 4.f, 0.2f);
        // A wall 2 m ahead, 3 m wide, centred on the nose. Open either side.
        box(w, 6.5f, 6.0f, 9.5f, 6.4f, 4.f, kWallTex);
        sim::CamPose truth; truth.e = 8.f; truth.n = 4.f; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);
        WorldModel wm;
        wm.with([](WorldState& s) { s = stillState(); });
        for (int i = 0; i < 8; ++i) mod.run(cv::Mat(), wm);
        const WorldState s = wm.snapshot();
        std::printf("  planner %.1f deg (freeM %.2f, blocked %d)  leg %.1f deg "
                    "certified %.2f m\n", s.voxBearingDeg, s.voxFreeM,
                    int(s.voxBlocked), s.voxLegBearingDeg, s.voxLegFreeM);
        CHECK(s.voxValid);
        // Straight ahead is the wall: whatever it chose, its certificate must
        // not reach through it.
        const float ahead = mod.pipeline().straightFreeM(sim::CamPose{}, 0.f, 6.f);
        std::printf("  straight ahead certified %.2f m (wall face at 2.0 m)\n", ahead);
        CHECK(ahead < 2.0f - 0.3f);
        if (!std::getenv("VOXTEST_STEREO"))
            CHECK(s.voxLegFreeM > 1.f);       // there IS a way round it
        if (s.voxLegFreeM > 0.f) {
            const float worst = segmentClearance(w, truth.e, truth.n, truth.u,
                                                 s.voxLegBearingDeg, s.voxLegFreeM, 2.f);
            std::printf("  truth clearance along the certified leg: %.2f m\n", worst);
            CHECK(worst >= 0.3f);
        }
    }

    // ---------------------------------------------------------------- 3
    std::printf("mission gates on the voxel plan\n");
    {
        MissionController::Params mp;
        mp.useVoxel = true; mp.useMap = false; mp.settleSec = 0.2f;
        MissionController m(mp);
        m.enable(true);
        WorldState s;
        s.estValid = true; s.missionGo = true; s.tickMonoS = 50.0;
        auto fresh = [&](int frames, bool blocked, float legFree, float brg) {
            s.voxValid = true; s.voxStampS = s.tickMonoS; s.voxFrames = frames;
            s.voxBlocked = blocked; s.voxLegFreeM = legFree; s.voxLegBearingDeg = brg;
        };
        for (int i = 0; i < 5; ++i) m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);

        fresh(2, false, 3.f, 0.f);                  // young map: keep hovering
        ControlCmd c = m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);
        CHECK(c.pitch == 0.f && c.yaw == 0.f);

        fresh(8, true, 0.f, 0.f);                   // mature, nothing: look round
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::SCAN);
        c = m.update(s, 0.05f);
        CHECK(c.yaw != 0.f && c.pitch == 0.f);      // yaw only, no translation

        fresh(9, false, 2.5f, 40.f);                // an opening mid-sweep
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::THINK);
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::MOVE);
        // Leg = certificate - stop margin, NOT stepM: 2.5 - 0.5 = 2.0 m.
        const float wpDist = std::hypot(s.missionWpE, s.missionWpN);
        std::printf("  leg %.2f m on bearing 40 (stepM %.1f)\n", wpDist, mp.stepM);
        CHECK(std::fabs(wpDist - 2.0f) < 0.01f);

        s.vehYawDeg = 0.f;                          // nose 40 deg off the line
        c = m.update(s, 0.05f);
        CHECK(c.yaw > 0.f && c.pitch == 0.f);       // turn first, do not fly
        s.vehYawDeg = 36.f;
        c = m.update(s, 0.05f);
        CHECK(c.pitch > 0.f);                       // on the line: go

        s.estPe = 2.0f * std::sin(40.f * kPi / 180.f);
        s.estPn = 2.0f * std::cos(40.f * kPi / 180.f);
        m.update(s, 0.05f);
        CHECK(m.phase() == MissionController::Phase::ARRIVE);

        // A dead module: THINK gives it the stale window, then re-settles.
        MissionController m2(mp); m2.enable(true);
        WorldState s2; s2.estValid = true; s2.missionGo = true; s2.tickMonoS = 50.0;
        for (int i = 0; i < 5; ++i) m2.update(s2, 0.05f);
        CHECK(m2.phase() == MissionController::Phase::THINK);
        m2.update(s2, 0.05f);
        CHECK(m2.phase() == MissionController::Phase::THINK);   // not yet
        for (int i = 0; i < 20; ++i) m2.update(s2, 0.05f);
        CHECK(m2.phase() != MissionController::Phase::MOVE);
        std::printf("  ok\n");
    }

    // ---------------------------------------------------------------- 4
    std::printf("closed loop: pillar field, attitude only\n");
    {
        // VOXTEST_WORLD=<n> re-rolls the obstacles; VOXTEST_STEREO=1 swaps the
        // perfect depth for the simulated stereo matcher (holes, speckle,
        // range-dependent noise); VOXTEST_BIG=1 is a 48 m field with walls
        // that make dead ends, where seeing past the near map should matter;
        // VOXTEST_FAR=0 turns off the far tier's choice among safe legs.
        // CI runs the defaults (24 m room, perfect depth, far tier on).
        const bool big = std::getenv("VOXTEST_BIG") != nullptr;
        if (const char* f = std::getenv("VOXTEST_FAR")) vp.farChoose = std::atoi(f) != 0;
        const float sizeM = big ? 48.f : 24.f;
        const float spawnE = sizeM * 0.5f, spawnN = 4.f;
        sim::VoxelWorld w; buildRoom(w, sizeM, 5.f, 0.2f);
        const char* ws = std::getenv("VOXTEST_WORLD");
        std::mt19937 rng(ws ? unsigned(std::atoi(ws)) : 7u);
        std::uniform_real_distribution<float> U(3.f, sizeM - 3.f);
        int pillars = 0, walls = 0;
        const int wantPillars = big ? 45 : 22, wantWalls = big ? 10 : 0;
        for (int i = 0; i < 400 && walls < wantWalls; ++i) {
            // 6 m walls, either axis: the shapes that make a dead end, which a
            // 2.5 m near map walks into and a 20 m far tier can see.
            const float x = U(rng), y = U(rng);
            const bool alongE = (rng() & 1u) != 0;
            const float x1 = alongE ? x + 6.f : x + 0.4f, y1 = alongE ? y + 0.4f : y + 6.f;
            if (x1 > sizeM - 1.f || y1 > sizeM - 1.f) continue;
            const float cx = std::max(x, std::min(spawnE, x1));
            const float cy = std::max(y, std::min(spawnN, y1));
            if (std::hypot(cx - spawnE, cy - spawnN) < 3.f) continue;   // spawn clear
            box(w, x, y, x1, y1, 5.f, kWallTex);
            ++walls;
        }
        for (int i = 0; i < 400 && pillars < wantPillars; ++i) {
            const float x = U(rng), y = U(rng);
            if (std::hypot(x - spawnE, y - spawnN) < 2.5f) continue;   // spawn clear
            box(w, x - 0.2f, y - 0.2f, x + 0.2f, y + 0.2f, 5.f, kPillarTex);
            ++pillars;
        }
        sim::CamPose truth; truth.e = spawnE; truth.n = spawnN; truth.u = 1.5f;
        auto* src = new AttitudeOnlySource(w, cp, &truth);
        VoxelNavModule mod(std::unique_ptr<sim::FrameSource>(src), vp);

        MissionController::Params mp;
        mp.useVoxel = true; mp.useMap = false;
        MissionController m(mp);
        m.enable(true);

        WorldModel wm;
        const float dt = 0.05f, tau = 0.35f, vPerPitch = 1.0f / mp.cruise;
        float v = 0.f, travelled = 0.f, minClear = 1e9f;
        int legs = 0, ticks = 0;
        std::string lastPhase, minPhase;
        float minT = 0.f;
        // What this project scores on (CLAUDE.md): distance before a
        // collision, and -- against the degenerate answer of circling in a
        // safe clearing -- how far it ends from where it began and how much
        // ground it covers, in 1 m cells.
        std::vector<char> visited(size_t(sizeM) * size_t(sizeM), 0);
        int cells = 0;
        const float simS = 150.f;
        for (float t = 0.f; t < simS; t += dt, ++ticks) {
            // Telemetry the aircraft would have: attitude, speed, and (for
            // the mission's leg-length gate only) where it is.
            wm.with([&](WorldState& s) {
                s.tickMonoS = monoNowS();
                s.vehYawDeg = truth.yawDeg; s.vehGroundspeed = std::fabs(v);
                s.estValid = true; s.estPe = truth.e; s.estPn = truth.n;
                s.estSpeed = std::fabs(v); s.missionGo = true;
            });
            // The think tier runs at camera rate while a vantage is open, and
            // occasionally otherwise (it must still say "moving").
            const std::string ph = wm.snapshot().missionPhase;
            if (ph == "THINK" || ph == "SCAN" || ticks % 6 == 0) mod.run(cv::Mat(), wm);

            ControlCmd c;
            wm.with([&](WorldState& s) { c = m.update(s, dt); });
            const std::string now = wm.snapshot().missionPhase;
            if (now == "MOVE" && lastPhase != "MOVE") {
                ++legs;
                if (std::getenv("VOXTEST_TRACE")) {
                    const WorldState q = wm.snapshot();
                    std::printf("   t=%.1f leg from (%.2f,%.2f) brg %.1f cert %.2f "
                                "freeM %.2f frames %d truthSeg %.2f\n", t, truth.e,
                                truth.n, q.voxLegBearingDeg, q.voxLegFreeM, q.voxFreeM,
                                q.voxFrames,
                                segmentClearance(w, truth.e, truth.n, truth.u,
                                                 q.voxLegBearingDeg, q.voxLegFreeM, 2.f));
                }
            }
            if (std::getenv("VOXTEST_TRACE") && now != lastPhase)
                std::printf("   t=%.1f %s -> %s at (%.2f,%.2f) yaw %.1f v %.2f clr %.2f\n",
                            t, lastPhase.c_str(), now.c_str(), truth.e, truth.n,
                            truth.yawDeg, v, clearance(w, truth.e, truth.n, truth.u, 2.f));
            lastPhase = now;
            if (now == "STUCK") break;

            // Lagged kinematics: pitch -> forward speed, yaw -> yaw rate.
            truth.yawDeg += c.yaw * 90.f * dt;
            const float vCmd = c.pitch * vPerPitch;
            v += (vCmd - v) * dt / tau;
            const float a = truth.yawDeg * kPi / 180.f;
            truth.e += std::sin(a) * v * dt;
            truth.n += std::cos(a) * v * dt;
            travelled += std::fabs(v) * dt;
            const float cl = clearance(w, truth.e, truth.n, truth.u, 2.f);
            if (cl < minClear) { minClear = cl; minPhase = now; minT = t; }
            const int ce = int(truth.e), cn = int(truth.n);
            if (ce >= 0 && cn >= 0 && ce < int(sizeM) && cn < int(sizeM) &&
                !visited[size_t(cn) * size_t(sizeM) + size_t(ce)]) {
                visited[size_t(cn) * size_t(sizeM) + size_t(ce)] = 1; ++cells;
            }
        }
        const float net = std::hypot(truth.e - spawnE, truth.n - spawnN);
        std::printf("  %d pillars %d walls  %.0f s  travelled %.1f m  net %.1f m  "
                    "cells %d  legs %d  min truth clearance %.2f m (in %s at %.1f s)  "
                    "final %s  frames %d\n",
                    pillars, walls, simS, travelled, net, cells, legs, minClear,
                    minPhase.c_str(), minT, lastPhase.c_str(), src->frames);
        // One machine-readable line, for sweeps.
        std::printf("RESULT far=%d big=%d stereo=%d world=%s travel=%.2f net=%.2f "
                    "cells=%d legs=%d minclr=%.3f stuck=%d\n",
                    int(vp.farChoose), int(big), int(std::getenv("VOXTEST_STEREO") != nullptr),
                    ws ? ws : "7", travelled, net, cells, legs, minClear,
                    int(lastPhase == "STUCK"));
        // The airframe is 0.3 m in radius; the planner's 0.6 m includes the
        // margin. Closer than 0.3 m to a solid voxel is a collision.
        CHECK(minClear >= 0.3f);
        // And it must actually fly: a planner that hovers for ever is safe
        // and useless, which is the degenerate answer to this objective.
        CHECK(legs >= 5);
        CHECK(travelled >= 8.f);
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
