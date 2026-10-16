// tracker_bench -- quality, timing and a BIT-EXACT TRACE of track::LockTracker
// (the fused tracker) over synthetic scenarios x seeds. No camera, no OpenCV.
//
//   tracker_bench [trace.bin] [seeds=4]
//   tracker_bench --ab <nccFloorB> [draws=16] [chromaNoise=0]
//
// --ab is a PAIRED A/B in the style of desktop/eval_ab.py: the tracker is
// chaotic (a tiny input change decides which frame re-locks), so one run per
// arm proves nothing. Both arms run every scenario under the SAME perturbation
// -- the designation jittered by U(-0.25, +0.25) px -- and the per-draw
// difference is averaged over many draws. Arm A is plain NCC (nccFloor 0).
//
// Prints lock rate, mean centre error, losses and ms/frame per scenario, and
// writes every per-frame Result (state, box, conf, prediction, aim) to
// trace.bin. A refactor that claims "no behaviour change" must reproduce the
// trace byte for byte (cmp old.bin new.bin); a change that only moves rounding
// is checked with tools/tracker_trace_diff.py (same states, tiny deltas).
// Run it on the Pi for real per-frame cost.
//
// The scenes are SYNTHETIC: smoothed value-noise clutter, a textured target of
// flat colour. "fast" teleports the target every 60 frames and "vanish" removes
// it -- both exist to exercise re-acquire and loss, not to score tracking.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "lock_tracker_fused.hpp"
using namespace track;

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed * 2654435761u + 12345u) {}
    float next() { s = s * 1664525u + 1013904223u; return (s >> 8) * (1.f / 16777216.f); }
};

// Smoothed value noise: broadband but not white (like real clutter).
std::vector<float> valueNoise(int w, int h, int cell, Rng& r) {
    const int gw = w / cell + 2, gh = h / cell + 2;
    std::vector<float> g(size_t(gw) * gh);
    for (auto& v : g) v = r.next();
    std::vector<float> o(size_t(w) * h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const float fx = float(x) / cell, fy = float(y) / cell;
            const int ix = int(fx), iy = int(fy);
            const float ax = fx - ix, ay = fy - iy;
            const float a = g[iy * gw + ix], b = g[iy * gw + ix + 1];
            const float c = g[(iy + 1) * gw + ix], d = g[(iy + 1) * gw + ix + 1];
            o[size_t(y) * w + x] = (a * (1 - ax) + b * ax) * (1 - ay) + (c * (1 - ax) + d * ax) * ay;
        }
    return o;
}

struct Scene {
    int w = 320, h = 240;
    std::vector<float> bg, tex;          // background, target texture (TEX x TEX)
    int TEX = 64;
};

struct Obj { float x, y, size; bool visible; };

GrayFrame render(const Scene& sc, const std::vector<Obj>& objs, float panX, float panY,
                 bool colour, float noise, Rng& r, const Obj* occluder,
                 float chromaNoise = 0.f) {
    GrayFrame f; f.w = sc.w; f.h = sc.h;
    const size_t n = size_t(sc.w) * sc.h;
    f.ownD = std::make_shared<std::vector<float>>(n); f.d = f.ownD->data();
    if (colour) {
        f.ownU = std::make_shared<std::vector<float>>(n); f.ownV = std::make_shared<std::vector<float>>(n);
        f.cu = f.ownU->data(); f.cv = f.ownV->data();
    }
    for (int y = 0; y < sc.h; ++y)
        for (int x = 0; x < sc.w; ++x) {
            // Background moves with the camera pan (wraps).
            int bx = (x + int(panX)) % (sc.w * 2); if (bx < 0) bx += sc.w * 2;
            int by = (y + int(panY)) % (sc.h * 2); if (by < 0) by += sc.h * 2;
            float v = 60.f + 100.f * sc.bg[size_t(by) * (sc.w * 2) + bx];
            float cu = 5.f, cv = -5.f;
            for (const Obj& o : objs) {
                if (!o.visible) continue;
                const float ox = x - o.x, oy = y - o.y;
                if (std::fabs(ox) <= o.size / 2 && std::fabs(oy) <= o.size / 2) {
                    const int ix = std::min(std::max(int((ox / o.size + 0.5f) * sc.TEX), 0), sc.TEX - 1);
                    const int iy = std::min(std::max(int((oy / o.size + 0.5f) * sc.TEX), 0), sc.TEX - 1);
                    v = 120.f + 120.f * sc.tex[size_t(iy) * sc.TEX + ix];
                    cu = -40.f; cv = 55.f;
                }
            }
            if (occluder) {
                const float ox = x - occluder->x, oy = y - occluder->y;
                if (std::fabs(ox) <= occluder->size / 2 && std::fabs(oy) <= occluder->size * 0.8f) {
                    v = 70.f + 20.f * sc.bg[size_t((y * 7) % (sc.h * 2)) * (sc.w * 2) + (x * 3) % (sc.w * 2)];
                    cu = 20.f; cv = 10.f;
                }
            }
            v += noise * (r.next() - 0.5f);
            const size_t o = size_t(y) * sc.w + x;
            f.d[o] = std::min(std::max(v, 0.f), 255.f);
            if (colour) {
                // Real cameras are never chroma-flat; the clean default is the
                // stress case (flat patches are exactly flat).
                if (chromaNoise > 0.f) {
                    cu += chromaNoise * (r.next() - 0.5f) * 3.46f;   // unit-variance uniform
                    cv += chromaNoise * (r.next() - 0.5f) * 3.46f;
                }
                f.cu[o] = cu; f.cv[o] = cv;
            }
        }
    return f;
}

struct ScenarioResult {
    double errSum = 0; int errN = 0; int locked = 0, frames = 0, lostAt = -1; double ms = 0;
    int onTarget = 0, visible = 0;     // centre within 25 px, of frames the target is visible
};
struct Arm { float nccFloor = 0.f; float jx = 0.f, jy = 0.f; float chromaNoise = 0.f; };

// One scenario: returns per-frame Results appended to `trace`.
ScenarioResult runScenario(const std::string& name, uint32_t seed, bool colour,
                           std::vector<LockTracker::Result>& trace, const Arm& arm = Arm()) {
    Rng r(seed + 7u * uint32_t(name.size()));
    Scene sc;
    Rng rb(seed);
    sc.bg = valueNoise(sc.w * 2, sc.h * 2, 9, rb);
    sc.tex = valueNoise(sc.TEX, sc.TEX, 4, rb);
    LockTracker t;
    t.nccFloor = arm.nccFloor;
    if (colour) t.setCues({CropFilter::EDGE, CropFilter::CHROMA, CropFilter::NONE});
    else        t.setCues({CropFilter::EDGE, CropFilter::NONE});
    const int N = 150;
    ScenarioResult res;
    Obj target{100.f, 120.f, 34.f, true};
    for (int k = 0; k < N; ++k) {
        float panX = 0, panY = 0;
        std::vector<Obj> objs;
        Obj occ{0, 0, 0, false}; const Obj* occP = nullptr;
        float noise = 6.f;
        const float tt = float(k);
        if (name == "linear") { target.x = 70.f + 1.3f * tt; target.y = 120.f + 15.f * std::sin(tt * 0.04f); }
        else if (name == "fast") { target.x = 60.f + 3.2f * std::fmod(tt, 60.f); target.y = 120.f + 40.f * std::sin(tt * 0.1f); }
        else if (name == "pan") { panX = 2.5f * tt; target.x = 160.f + 20.f * std::sin(tt * 0.05f); target.y = 120.f; }
        else if (name == "occlusion") {
            target.x = 80.f + 1.2f * tt; target.y = 120.f;
            if (k >= 50 && k < 62) { occ = Obj{target.x, target.y, 46.f, true}; occP = &occ; }
        }
        else if (name == "distractor") {
            target.x = 100.f + 0.8f * tt; target.y = 110.f;
            objs.push_back(Obj{target.x + 75.f - 0.6f * tt, 130.f, 34.f, true});   // twin crossing
        }
        else if (name == "scale") { target.x = 160.f; target.y = 120.f; target.size = 34.f + 0.12f * tt; }
        else if (name == "noisy") { target.x = 90.f + 1.0f * tt; target.y = 125.f; noise = 40.f; }
        else if (name == "vanish") { target.x = 100.f + 1.f * tt; target.y = 120.f; target.visible = k < 60; }
        objs.push_back(target);
        GrayFrame f = render(sc, objs, panX, panY, colour, noise, r, occP, arm.chromaNoise);
        if (k == 0) {
            t.designate(f, target.x + 1.5f + arm.jx, target.y - 1.f + arm.jy, target.size * 1.2f);
            continue;
        }
        const auto a = std::chrono::steady_clock::now();
        const LockTracker::Result out = t.update(f, k / 30.0);
        res.ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
        trace.push_back(out);
        ++res.frames;
        if (out.state == LockTracker::State::LOCKED) ++res.locked;
        if (out.state == LockTracker::State::LOST && res.lostAt < 0) res.lostAt = k;
        const bool tracking = out.state != LockTracker::State::LOST &&
                              out.state != LockTracker::State::IDLE;
        const float cx = out.x + out.w / 2.f, cy = out.y + out.h / 2.f;
        const float err = std::hypot(cx - target.x, cy - target.y);
        if (target.visible) {
            ++res.visible;
            if (tracking && err < 25.f) ++res.onTarget;
            if (tracking) { res.errSum += err; ++res.errN; }
        }
        if (out.state == LockTracker::State::LOST) break;
    }
    return res;
}

const char* kNames[] = {"linear", "fast", "pan", "occlusion", "distractor", "scale", "noisy", "vanish"};

// Mean on-target % of one arm over every scenario x cue set, for one draw.
double armScore(const Arm& arm, uint32_t draw, std::vector<double>* perCase) {
    std::vector<LockTracker::Result> sink;
    double sum = 0; int n = 0;
    for (const char* nm : kNames)
        for (int colour = 0; colour <= 1; ++colour) {
            sink.clear();
            const ScenarioResult r = runScenario(nm, 2000u + draw, colour, sink, arm);
            const double on = 100.0 * r.onTarget / std::max(r.visible, 1);
            if (perCase) perCase->push_back(on);
            sum += on; ++n;
        }
    return sum / n;
}

int abMain(float floorB, int draws, float chromaNoise) {
    std::printf("paired A/B: nccFloor 0 vs %.2f, %d draws, chroma noise %.1f\n",
                double(floorB), draws, double(chromaNoise));
    std::vector<double> d;
    std::vector<std::vector<double>> caseA, caseB;
    for (int k = 0; k < draws; ++k) {
        Rng jr(uint32_t(77 + k));
        Arm a, b;
        a.jx = b.jx = (jr.next() - 0.5f) * 0.5f;      // same draw in both arms
        a.jy = b.jy = (jr.next() - 0.5f) * 0.5f;
        a.chromaNoise = b.chromaNoise = chromaNoise;
        b.nccFloor = floorB;
        std::vector<double> ca, cb;
        const double sa = armScore(a, uint32_t(k), &ca), sb = armScore(b, uint32_t(k), &cb);
        caseA.push_back(ca); caseB.push_back(cb);
        d.push_back(sb - sa);
        std::printf("  draw %2d  A %6.2f%%  B %6.2f%%  delta %+6.2f\n", k, sa, sb, sb - sa);
    }
    double m = 0; for (double x : d) m += x; m /= d.size();
    double v = 0; for (double x : d) v += (x - m) * (x - m);
    const double se = d.size() > 1 ? std::sqrt(v / (d.size() - 1) / d.size()) : 0.0;
    int pos = 0, neg = 0; for (double x : d) { pos += x > 0; neg += x < 0; }
    std::printf("paired delta %+.3f +/- %.3f (SE), t %+.2f; B better in %d, worse in %d of %d draws\n",
                m, se, se > 0 ? m / se : 0.0, pos, neg, draws);
    std::printf("per case (B wins / losses over draws):\n");
    for (size_t c = 0; c < caseA[0].size(); ++c) {
        int w = 0, l = 0; double ma = 0, mb = 0;
        for (int k = 0; k < draws; ++k) {
            w += caseB[k][c] > caseA[k][c]; l += caseB[k][c] < caseA[k][c];
            ma += caseA[k][c]; mb += caseB[k][c];
        }
        std::printf("  %-11s %-6s %3dW %3dL   mean %6.1f%% -> %6.1f%%\n", kNames[c / 2],
                    c % 2 ? "3-cue" : "2-cue", w, l, ma / draws, mb / draws);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 2 && std::strcmp(argv[1], "--ab") == 0)
        return abMain(float(std::atof(argv[2])), argc > 3 ? std::atoi(argv[3]) : 16,
                      argc > 4 ? float(std::atof(argv[4])) : 0.f);
    const char* out = argc > 1 ? argv[1] : nullptr;
    const int seeds = argc > 2 ? std::atoi(argv[2]) : 4;
    const char* const* names = kNames;
    std::vector<LockTracker::Result> trace;
    double totalMs = 0; int totalFrames = 0;
    std::printf("%-11s %-6s %8s %8s %9s %8s\n", "scenario", "cues", "lock%", "err px", "lostAt", "ms/frm");
    for (int ni = 0; ni < 8; ++ni)
        for (int colour = 0; colour <= 1; ++colour) {
            const char* nm = names[ni];
            double eS = 0; int eN = 0, lk = 0, fr = 0, lost = 0; double ms = 0;
            for (int s = 0; s < seeds; ++s) {
                const ScenarioResult r = runScenario(nm, uint32_t(1000 + s), colour, trace);
                eS += r.errSum; eN += r.errN; lk += r.locked; fr += r.frames; ms += r.ms;
                lost += r.lostAt >= 0;
            }
            totalMs += ms; totalFrames += fr;
            std::printf("%-11s %-6s %7.1f%% %8.2f %6d/%d %8.3f\n", nm, colour ? "3-cue" : "2-cue",
                        100.0 * lk / std::max(fr, 1), eN ? eS / eN : -1.0, lost, seeds, ms / std::max(fr, 1));
        }
    std::printf("TOTAL %d frames, mean %.3f ms/frame\n", totalFrames, totalMs / totalFrames);
    if (out) {
        FILE* fp = std::fopen(out, "wb");
        for (const auto& r : trace) {
            // Field by field: the struct has padding, which must not enter the hash.
            const int st = int(r.state);
            std::fwrite(&st, 4, 1, fp);
            std::fwrite(&r.x, 4, 1, fp); std::fwrite(&r.y, 4, 1, fp);
            std::fwrite(&r.w, 4, 1, fp); std::fwrite(&r.h, 4, 1, fp);
            std::fwrite(&r.conf, 4, 1, fp);
            std::fwrite(&r.predX, 4, 1, fp); std::fwrite(&r.predY, 4, 1, fp);
            std::fwrite(&r.aimX, 4, 1, fp); std::fwrite(&r.aimY, 4, 1, fp);
        }
        std::fclose(fp);
        std::printf("trace: %zu frames -> %s\n", trace.size(), out);
    }
}
