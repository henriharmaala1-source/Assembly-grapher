// Headless checks on the fused tracker. No OpenCV, no camera -- the tracker core
// is deliberately dependency-free so it can be tested exactly like this.
//
// These are not a substitute for desktop/simtrack.py, which is the behavioural
// mirror and where the A/B sweeps live. They exist to catch the things a port
// gets wrong: index arithmetic, buffer reuse, state machine edges, and the
// numerical trap in nccAt.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "lock_tracker_fused.hpp"

using namespace track;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  ", __FILE__, __LINE__); std::printf(__VA_ARGS__); \
    std::printf("\n"); ++failures; } } while (0)

namespace {

// Deterministic PRNG -- a fixed seed matters more than quality here.
struct Rng {
    unsigned s = 12345u;
    float next() { s = s * 1664525u + 1013904223u; return float((s >> 8) & 0xFFFF) / 65535.f; }
};

// The target's own texture, generated ONCE and indexed in target-local
// coordinates so it moves rigidly with the target.
//
// Getting this right matters more than it looks. Two earlier versions of this
// test measured the TARGET rather than the tracker:
//   - a uniform bright square is an aperture-problem target: the NCC response is
//     flat across its interior, so the peak wanders +-half the square;
//   - two sinusoids at 12-20 px periods inside a 23 px template are
//     quasi-periodic, so sliding by one period re-matches and the sidelobes
//     crush PSR to 2.5 even at a peak correlation of 0.99 -- below the lock
//     floor, so a perfect static match read as a total miss.
// Broadband, non-repeating texture (smoothed value noise) is what a real vehicle
// or building presents, and what gives a sharp autocorrelation peak.
const int TEX = 96;
const std::vector<float>& targetTexture() {
    static std::vector<float> t;
    if (!t.empty()) return t;
    std::vector<float> raw(TEX * TEX);
    Rng r; r.s = 987654321u;
    for (auto& v : raw) v = r.next();
    t.assign(TEX * TEX, 0.f);
    // 3x3 box blur -> correlation length ~2 px: broadband but not white.
    for (int y = 0; y < TEX; ++y)
        for (int x = 0; x < TEX; ++x) {
            float s = 0; int n = 0;
            for (int j = -1; j <= 1; ++j)
                for (int i = -1; i <= 1; ++i) {
                    const int yy = y + j, xx = x + i;
                    if (yy < 0 || yy >= TEX || xx < 0 || xx >= TEX) continue;
                    s += raw[yy * TEX + xx]; ++n;
                }
            t[y * TEX + x] = s / n;
        }
    return t;
}

// A frame with band-limited background texture and a textured square target.
// Band-limited, not white: white noise makes every patch globally unique and
// flatters any correlation tracker.
GrayFrame makeFrame(int w, int h, float tx, float ty, float tsize,
                    Rng& rng, bool colour, float noise = 3.f) {
    GrayFrame f;
    f.w = w; f.h = h;
    f.ownD = std::make_shared<std::vector<float>>(size_t(w) * h);
    f.d = f.ownD->data();
    if (colour) {
        f.ownU = std::make_shared<std::vector<float>>(size_t(w) * h);
        f.ownV = std::make_shared<std::vector<float>>(size_t(w) * h);
        f.cu = f.ownU->data(); f.cv = f.ownV->data();
    }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const int o = y * w + x;
            // Smooth background + a little noise.
            float v = 90.f + 30.f * std::sin(x * 0.07f) * std::cos(y * 0.05f)
                    + noise * (rng.next() - 0.5f);
            float cu = 0.f, cv = 0.f;
            const float ox = x - tx, oy = y - ty;
            if (std::fabs(ox) <= tsize / 2 && std::fabs(oy) <= tsize / 2) {
                // Target-local index, so the texture translates rigidly with the
                // target instead of the target sliding over a fixed pattern.
                const int ix = std::min(std::max(int(ox + TEX / 2), 0), TEX - 1);
                const int iy = std::min(std::max(int(oy + TEX / 2), 0), TEX - 1);
                v = 130.f + 110.f * targetTexture()[iy * TEX + ix]
                          + noise * (rng.next() - 0.5f);
                cu = -40.f; cv = 55.f;                  // distinctly coloured target
            }
            f.d[o] = std::min(std::max(v, 0.f), 255.f);
            if (colour) { f.cu[o] = cu; f.cv[o] = cv; }
        }
    return f;
}

float boxErr(const LockTracker::Result& r, float tx, float ty) {
    return std::hypot(r.x + r.w / 2.f - tx, r.y + r.h / 2.f - ty);
}

// --- tests ---------------------------------------------------------------

void testTracksLinearMotion() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE, CropFilter::EDGE});
    float tx = 120, ty = 120;
    GrayFrame f = makeFrame(320, 240, tx, ty, 40, rng, false);
    t.designate(f, tx, ty, 48);
    CHECK(t.state() == LockTracker::State::LOCKED, "designate did not lock");

    float worst = 0;
    for (int i = 0; i < 40; ++i) {
        tx += 2.0f; ty += 1.0f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, false);
        const auto r = t.update(g);
        if (i > 3) worst = std::max(worst, boxErr(r, tx, ty));
    }
    CHECK(t.state() == LockTracker::State::LOCKED, "lost a clean linear target");
    CHECK(worst < 5.f, "linear-motion box error %.1f px too high", worst);
    std::printf("  linear motion: worst err %.1f px, state %s\n",
                worst, LockTracker::stateName(t.state()));
}

// The target vanishes entirely. The tracker must COAST, then escalate to
// SEARCHING, then declare LOST -- and reset itself when it does.
void testCoastThenSearchThenLost() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE});
    GrayFrame f = makeFrame(320, 240, 160, 120, 40, rng, false);
    t.designate(f, 160, 120, 48);

    bool sawCoast = false, sawSearch = false;
    LockTracker::State last = LockTracker::State::IDLE;
    for (int i = 0; i < 60; ++i) {
        // Blank-ish frame: background only, target removed.
        GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, false);
        last = t.update(g).state;
        if (last == LockTracker::State::COASTING)  sawCoast = true;
        if (last == LockTracker::State::SEARCHING) sawSearch = true;
        if (last == LockTracker::State::LOST) break;
    }
    CHECK(sawCoast,  "never entered COASTING");
    CHECK(sawSearch, "never escalated to SEARCHING");
    CHECK(last == LockTracker::State::LOST, "never declared LOST");
    CHECK(!t.hasTarget(), "still claims a target after LOST");
    std::printf("  coast->search->lost: coast %d search %d final %s\n",
                sawCoast, sawSearch, LockTracker::stateName(last));
}

// nccAt must not blow up on a FLAT patch. This is the specific numerical trap
// documented in the Kotlin: the single-pass identity produces a huge spurious
// correlation exactly on low-contrast regions, which the argmax then takes.
void testFlatPatchDoesNotProduceSpuriousPeak() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE});
    GrayFrame f = makeFrame(320, 240, 160, 120, 40, rng, false);
    t.designate(f, 160, 120, 48);

    // Uniform grey -- every NCC position is degenerate.
    GrayFrame flat;
    flat.w = 320; flat.h = 240;
    flat.ownD = std::make_shared<std::vector<float>>(320 * 240, 128.f);
    flat.d = flat.ownD->data();
    const auto r = t.update(flat);
    CHECK(std::isfinite(r.conf), "confidence went non-finite on a flat frame");
    CHECK(r.conf >= 0.f && r.conf <= 1.f, "confidence %.3f out of range", r.conf);
    CHECK(std::isfinite(float(r.x)) && std::abs(r.x) < 10000,
          "box ran away on a flat frame: x=%d", r.x);
    std::printf("  flat frame: conf %.3f state %s (no spurious lock)\n",
                r.conf, LockTracker::stateName(r.state));
}

// The chroma histogram cue must engage on a colour frame and must be skipped
// (not crash, not corrupt) on a luma-only one.
void testHistogramCueColourAndLumaOnly() {
    for (int colour = 0; colour < 2; ++colour) {
        Rng rng;
        LockTracker t;
        t.setCues({CropFilter::NONE});
        float tx = 150, ty = 110;
        GrayFrame f = makeFrame(320, 240, tx, ty, 40, rng, colour != 0);
        t.designate(f, tx, ty, 48);
        float worst = 0;
        for (int i = 0; i < 20; ++i) {
            tx += 1.5f;
            GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, colour != 0);
            const auto r = t.update(g);
            if (i > 3) worst = std::max(worst, boxErr(r, tx, ty));
        }
        CHECK(t.state() == LockTracker::State::LOCKED,
              "%s: lost lock", colour ? "colour" : "luma-only");
        CHECK(worst < 5.f, "%s: err %.1f px", colour ? "colour" : "luma", worst);
        std::printf("  %-10s worst err %.1f px\n",
                    colour ? "colour:" : "luma:", worst);
    }
}

// Occlusion: the target is covered for a stretch, then reappears where the
// constant-velocity prediction says it should be. The point is that adaptation
// is frozen so the template is not contaminated and recovery still works.
void testOcclusionRecovery() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE, CropFilter::EDGE});
    float tx = 100, ty = 120;
    GrayFrame f = makeFrame(320, 240, tx, ty, 40, rng, true);
    t.designate(f, tx, ty, 48);
    for (int i = 0; i < 10; ++i) {                       // establish a clean lock
        tx += 2.f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        t.update(g);
    }
    for (int i = 0; i < 8; ++i) {                        // occluded
        tx += 2.f;
        GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, true);
        t.update(g);
    }
    float err = 1e9f;
    for (int i = 0; i < 12; ++i) {                       // reappears
        tx += 2.f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        const auto r = t.update(g);
        err = boxErr(r, tx, ty);
    }
    CHECK(t.hasTarget(), "did not survive an 8-frame occlusion");
    std::printf("  occlusion: final err %.1f px, state %s\n",
                err, LockTracker::stateName(t.state()));
}

// Scale must not ratchet down on a static target -- the dead-band exists because
// without it feed noise makes 0.9 win by a hair every frame.
void testScaleDoesNotRatchet() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE});
    GrayFrame f = makeFrame(320, 240, 160, 120, 40, rng, false);
    t.designate(f, 160, 120, 48);
    LockTracker::Result r{};
    for (int i = 0; i < 50; ++i) {
        GrayFrame g = makeFrame(320, 240, 160, 120, 40, rng, false);
        r = t.update(g);
    }
    CHECK(r.w >= 40 && r.w <= 60, "box ratcheted to %d px from 48", r.w);
    std::printf("  scale stability: 48 -> %d px over 50 static frames\n", r.w);
}

// Cue switching must not drop the lock (templates rebuild from the last crop).
void testCueSwitchKeepsLock() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE});
    float tx = 140, ty = 120;
    GrayFrame f = makeFrame(320, 240, tx, ty, 40, rng, true);
    t.designate(f, tx, ty, 48);
    for (int i = 0; i < 8; ++i) {
        tx += 1.5f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        t.update(g);
    }
    t.setCues({CropFilter::EDGE, CropFilter::CHROMA});
    CHECK(t.hasTarget(), "cue switch dropped the target");
    for (int i = 0; i < 10; ++i) {
        tx += 1.5f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        t.update(g);
    }
    CHECK(t.state() == LockTracker::State::LOCKED, "lost lock after cue switch");
    std::printf("  cue switch: survived, state %s\n",
                LockTracker::stateName(t.state()));
}

// Reuse of the exactly-sized scratch buffers must survive a crop-size change
// (normal FOV -> wide re-acquire -> back), which is where a stale-buffer bug in
// the EDGE filter would show up as a stationary phantom in the response map.
void testWideSearchScratchReuse() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::EDGE});
    GrayFrame f = makeFrame(320, 240, 160, 120, 40, rng, false);
    t.designate(f, 160, 120, 48);
    for (int i = 0; i < 10; ++i) {          // force the wide search open
        GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, false);
        t.update(g);
    }
    CHECK(t.state() == LockTracker::State::SEARCHING,
          "expected SEARCHING, got %s", LockTracker::stateName(t.state()));
    float err = 1e9f;
    for (int i = 0; i < 15; ++i) {          // put it back and re-acquire
        GrayFrame g = makeFrame(320, 240, 160, 120, 40, rng, false);
        const auto r = t.update(g);
        err = boxErr(r, 160, 120);
    }
    std::printf("  wide search: re-acquire err %.1f px, state %s\n",
                err, LockTracker::stateName(t.state()));
    CHECK(std::isfinite(err), "re-acquire produced a non-finite box");
}

// A caller that REUSES its conversion buffers -- which every real caller does,
// because allocating three float planes per frame is 110 MB/s at 640x480/30fps.
//
// WHAT THIS DOES AND DOES NOT COVER, stated plainly. It verifies that buffer
// reuse does not break tracking. It does NOT prove the tracker deep-copies its
// previous frame, and it is not able to: holding a VIEW instead makes optical
// flow compare a frame against ITSELF, which returns median (0,0) at consensus
// 1.0 -- a silent no-op, not a wrong answer.
//
// MEASURED, so nobody re-derives it: reintroducing the aliasing bug changes
// nothing here, and nothing under a sudden 11 px/frame camera pan either (7.0 px
// worst error, identical to three significant figures). The reason is structural
// -- OpticalFlow can only measure +-12 px (its `search` half-window) while the
// normal-FOV search window already absorbs +-18 frame px, so the ego estimate is
// always inside what the search would have found anyway. That agrees with the
// Kotlin's own A/B: "sim pan edge 3.7->0.4 px, zero change elsewhere".
//
// The deep copy in stashPrev() is kept regardless: a caller must not be able to
// silently disable a feature by doing the thing the API encourages.
void testCallerBufferReuse() {
    Rng rng;
    LockTracker t;
    t.setCues({CropFilter::NONE});

    // One set of buffers, overwritten in place every frame.
    std::vector<float> luma(320 * 240);
    auto fill = [&](float tx, float ty) {
        GrayFrame tmp = makeFrame(320, 240, tx, ty, 40, rng, false);
        std::copy(tmp.d, tmp.d + 320 * 240, luma.begin());
        GrayFrame view;                       // borrows `luma`, owns nothing
        view.d = luma.data(); view.w = 320; view.h = 240;
        return view;
    };

    float tx = 120, ty = 120;
    GrayFrame f0 = fill(tx, ty);
    t.designate(f0, tx, ty, 48);

    float worst = 0;
    for (int i = 0; i < 30; ++i) {
        tx += 2.0f; ty += 1.0f;
        GrayFrame g = fill(tx, ty);           // overwrites the buffer designate saw
        const auto r = t.update(g);
        if (i > 3) worst = std::max(worst, boxErr(r, tx, ty));
    }
    CHECK(t.state() == LockTracker::State::LOCKED,
          "lost lock when the caller reused its frame buffer");
    CHECK(worst < 5.f, "buffer-reuse box error %.1f px", worst);
    std::printf("  caller buffer reuse: worst err %.1f px, state %s\n",
                worst, LockTracker::stateName(t.state()));
}

}  // namespace

int main() {
    std::printf("fused lock tracker -- headless checks\n");
    testTracksLinearMotion();
    testCoastThenSearchThenLost();
    testFlatPatchDoesNotProduceSpuriousPeak();
    testHistogramCueColourAndLumaOnly();
    testOcclusionRecovery();
    testScaleDoesNotRatchet();
    testCueSwitchKeepsLock();
    testWideSearchScratchReuse();
    testCallerBufferReuse();
    if (failures) { std::printf("\n%d FAILURE(S)\n", failures); return 1; }
    std::printf("\nall checks passed\n");
    return 0;
}
