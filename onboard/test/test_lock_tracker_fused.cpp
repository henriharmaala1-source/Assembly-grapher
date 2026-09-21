// Headless checks on the fused tracker. No OpenCV, no camera -- the tracker core
// is deliberately dependency-free so it can be tested exactly like this.
//
// These are not a substitute for desktop/simtrack.py, which is the behavioural
// mirror and where the A/B sweeps live. They exist to catch the things a port
// gets wrong: index arithmetic, buffer reuse, state machine edges, and the
// numerical trap in nccAt.

#include <cmath>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <vector>

#include "lock_tracker_fused.hpp"

using namespace track;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  ", __FILE__, __LINE__); std::printf(__VA_ARGS__); \
    std::printf("\n"); ++failures; } } while (0)

namespace {
// A NOMINAL 30 fps CLOCK. update() now requires a capture time; these cases
// were all written against a fixed rate, so feeding them one preserves exactly
// what they were asserting. The cases that care about elapsed time set their
// own gaps.
double g_t = 0.0;
double tick(double dt = 1.0 / 30.0) { g_t += dt; return g_t; }


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


// --- recovery, measured rather than assumed ------------------------------
// The suite used to pass while printing "re-acquire err 19.4 px, state IDLE":
// the only assertion was that the error was FINITE. hasTarget() is no better,
// because it is true in COASTING and SEARCHING -- states in which the tracker
// is explicitly NOT holding the target. A recovery check has to name the
// state, the error and the time.
//
// The bounds below are MEASURED, not wished for: each case prints what the
// tracker actually did and then asserts against it, so the capability is
// described by the same numbers that pin it.
void testRecoveryReach() {
    struct Case { const char* name; int gapFrames; float reappearDx; };
    const Case cases[] = {
        {"same place, 8 frame gap",   8,   0.f},
        {"same place, 30 frame gap", 30,   0.f},
        {"25 px away, 8 frame gap",   8,  25.f},
        {"60 px away, 8 frame gap",   8,  60.f},
    };
    std::printf("  -- recovery reach --\n");
    for (const Case& c : cases) {
        Rng rng;
        LockTracker t;
        GrayFrame f0 = makeFrame(320, 240, 160, 120, 40, rng, true);
        t.designate(f0, 160, 120, 40);
        for (int i = 0; i < 10; ++i) {
            GrayFrame g = makeFrame(320, 240, 160, 120, 40, rng, true);
            t.update(g, tick());
        }
        for (int i = 0; i < c.gapFrames; ++i) {
            GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, true);
            t.update(g, tick());
        }
        const float rx = 160.f + c.reappearDx;
        int lockedAt = -1; float err = 1e9f;
        for (int i = 0; i < 20; ++i) {
            GrayFrame g = makeFrame(320, 240, rx, 120, 40, rng, true);
            const auto r = t.update(g, tick());
            err = boxErr(r, rx, 120);
            if (lockedAt < 0 && t.state() == LockTracker::State::LOCKED) lockedAt = i;
        }
        std::printf("     %-24s -> state %-9s err %6.1f px  locked after %s frames\n",
                    c.name, LockTracker::stateName(t.state()), err,
                    lockedAt < 0 ? "never" : (std::to_string(lockedAt)).c_str());
        if (c.gapFrames <= 8) {
            // WITHIN REACH: a gap under about a third of a second is recovered
            // from whatever the displacement, because the search widens around
            // a prediction that is still riding with the target. Measured: 0,
            // 25 and 60 px all re-LOCK on the first frame back, within 1.4 px.
            CHECK(t.state() == LockTracker::State::LOCKED, c.name);
            CHECK(lockedAt >= 0 && lockedAt <= 2, "re-LOCK took too long");
            CHECK(err <= 5.f, "re-LOCKed away from the target");
        } else {
            // PAST REACH, and this is the boundary worth pinning. SEARCHING is
            // LOCAL recovery: it expands around the prediction and matches the
            // original anchor, so a gap long enough for the prediction to drift
            // off is not recoverable by design. What the tracker must not do is
            // claim a lock anyway -- giving up is the correct answer and a
            // confident wrong box is the dangerous one.
            CHECK(!(t.state() == LockTracker::State::LOCKED && err > 8.f),
                  "claims LOCKED past its documented reach");
        }
    }
}

// THE LOSS TIMEOUT IS 1.5 SECONDS, not 45 frames. Fed at 10 fps it must give
// up after about 15 frames, not 45 -- which is the whole point of putting the
// tracker on a capture clock, and is the case the old frame-counting code got
// exactly three times wrong.
void testLossTimeoutIsSeconds() {
    std::printf("  -- loss timeout, in SECONDS --\n");
    for (double fps : {30.0, 10.0}) {
        Rng rng;
        LockTracker t;
        GrayFrame f0 = makeFrame(320, 240, 160, 120, 40, rng, true);
        t.designate(f0, 160, 120, 40);
        double clock = 0.0;
        for (int i = 0; i < 10; ++i) {
            GrayFrame g = makeFrame(320, 240, 160, 120, 40, rng, true);
            t.update(g, clock += 1.0 / fps);
        }
        // THE RETURNED state, not the tracker's. On LOST the tracker resets
        // itself to IDLE and reports LOST once, in that frame's Result -- so a
        // loop watching t.state() waits for a value that is never observable
        // and runs to its cap. My first version of this probe did exactly that
        // and read 13.3 s where the answer was 1.5.
        int n = 0; double t0 = clock; bool lost = false;
        while (!lost && n < 400) {
            GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, true);
            const auto r = t.update(g, clock += 1.0 / fps);
            lost = (r.state == LockTracker::State::LOST);
            ++n;
        }
        const double held = clock - t0;
        std::printf("     %4.0f fps: LOST after %3d frames = %.2f s\n",
                    fps, n, held);
        // THE TIMEOUT IS 1.5 SECONDS AT EVERY RATE. As a 45-frame count it was
        // 1.5 s at 30 fps and 4.5 s at 10 -- so the tracker held a target it
        // could not see three times longer exactly when the machine was too
        // busy to deliver frames, which is the opposite of what a timeout is
        // for. This is the regression test for that.
        CHECK(held > 1.3 && held < 1.8, "loss timeout is not 1.5 s at this rate");
    }
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
        const auto r = t.update(g, tick());
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
        last = t.update(g, tick()).state;
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
    const auto r = t.update(flat, tick());
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
            const auto r = t.update(g, tick());
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
        t.update(g, tick());
    }
    for (int i = 0; i < 8; ++i) {                        // occluded
        tx += 2.f;
        GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, true);
        t.update(g, tick());
    }
    float err = 1e9f;
    for (int i = 0; i < 12; ++i) {                       // reappears
        tx += 2.f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        const auto r = t.update(g, tick());
        err = boxErr(r, tx, ty);
    }
    // hasTarget() IS NOT RECOVERY. It is true in COASTING and SEARCHING,
    // states in which the tracker is explicitly not holding the target -- so
    // the old assertion passed for a tracker that had lost it and was still
    // looking. Recovery means the LOCKED state at the right place.
    std::printf("  occlusion: final err %.1f px, state %s\n",
                err, LockTracker::stateName(t.state()));
    CHECK(t.state() == LockTracker::State::LOCKED,
          "did not re-LOCK after an 8-frame occlusion");
    CHECK(err <= 5.f, "re-locked, but not on the target");
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
        r = t.update(g, tick());
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
        t.update(g, tick());
    }
    t.setCues({CropFilter::EDGE, CropFilter::CHROMA});
    CHECK(t.hasTarget(), "cue switch dropped the target");
    for (int i = 0; i < 10; ++i) {
        tx += 1.5f;
        GrayFrame g = makeFrame(320, 240, tx, ty, 40, rng, true);
        t.update(g, tick());
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
    bool sawWide = false;
    int  framesToWide = -1;                 // LATENCY, not just whether
    for (int i = 0; i < 30; ++i) {          // force the wide search open
        GrayFrame g = makeFrame(320, 240, -999, -999, 40, rng, false);
        const auto r = t.update(g, tick());
        if (r.state == LockTracker::State::SEARCHING) {
            if (!sawWide) framesToWide = i;
            sawWide = true;
        }
    }
    std::printf("  wide search: frames of empty scene before SEARCHING = %d\n",
                framesToWide);
    // THE LATTICE_FIX REGRESSION DOES NOT REPRODUCE HERE, and that is worth
    // recording because the header still warns about it.
    //
    // The claim carried over from the Python reference is that PSR roughly
    // doubling made the ABSOLUTE psrWarn/psrLock gates half as strict, so lock
    // is held on background too long on an empty scene (z_below_floor 15 -> 1).
    // Measured in this port: SEARCHING opens after 6 frames of a textured empty
    // scene, and FOV_DELAY is 6 -- it fails over at the fastest the constant
    // permits. There is no latency left for a stricter gate to recover.
    //
    // A RELATIVE FLOOR WAS BUILT AND REVERTED. Gating on a fraction of the
    // running clean baseline (psrEma_, which the occlusion detector already
    // uses) is scale-invariant where a doubled constant is not, so it looked
    // like the right shape of fix. Measured: no change at all here -- still 6
    // frames -- and the occlusion scenario regressed hard, final error 0.0 px
    // LOCKED to 59.1 px SEARCHING, because PSR legitimately falls below the
    // baseline during an occlusion and the floor rejected the very frames the
    // tracker is supposed to coast through. One measured loss, no measured win.
    //
    // WHAT IS STILL UNKNOWN. This is a synthetic unit test; the reference's
    // z_below_floor runs on clips. A background that produces a plausible FALSE
    // PEAK, rather than the smooth sinusoid here, could still hold lock. That
    // needs eval_tracker.py on real footage -- the same blocker as the MOSSE
    // swap and HIST_WEIGHT_CAP. Do not re-derive the relative floor from the
    // header's warning alone; it has been tried.
    std::printf("  wide search: reached SEARCHING = %d\n", sawWide);
    float err = 1e9f;
    for (int i = 0; i < 15; ++i) {          // put it back and re-acquire
        GrayFrame g = makeFrame(320, 240, 160, 120, 40, rng, false);
        const auto r = t.update(g, tick());
        err = boxErr(r, 160, 120);
    }
    std::printf("  wide search: re-acquire err %.1f px, state %s\n",
                err, LockTracker::stateName(t.state()));
    CHECK(std::isfinite(err), "re-acquire produced a non-finite box");
    // AND IT MUST NOT CLAIM A LOCK IT DOES NOT HAVE. This scenario is past the
    // tracker's documented reach -- 30 frames of empty scene, see
    // testRecoveryReach -- so the honest outcomes are COASTING, SEARCHING or
    // LOST. What would be a defect is LOCKED on the wrong thing, and nothing
    // here checked for that: the old assertion was that the error was FINITE.
    CHECK(!(t.state() == LockTracker::State::LOCKED && err > 8.f),
          "claims LOCKED while far from the target");
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
        const auto r = t.update(g, tick());
        if (i > 3) worst = std::max(worst, boxErr(r, tx, ty));
    }
    CHECK(t.state() == LockTracker::State::LOCKED,
          "lost lock when the caller reused its frame buffer");
    CHECK(worst < 5.f, "buffer-reuse box error %.1f px", worst);
    std::printf("  caller buffer reuse: worst err %.1f px, state %s\n",
                worst, LockTracker::stateName(t.state()));
}

// CoastFlow in isolation: seed on one frame, step to a frame shifted by a KNOWN
// amount, and require the median displacement to be exactly that. Deterministic
// -- no chaos, no battery -- which is what makes it able to catch the defect the
// reference found: mode 2 must REPLACE the extrapolation, and a matcher that is
// off by a pixel per frame compounds into a box-width of drift in three frames.
//
// 18 px is the case that motivates the pyramid at all: a single-scale search
// would need +-20 px (1681 positions) and would still lose the blurred target.
void testCoastFlowMeasuresKnownShift() {
    const int shifts[] = {0, 5, 11, 18};
    for (int sh : shifts) {
        Rng rng;
        CoastFlow cf;
        GrayFrame a = makeFrame(320, 240, 160, 120, 60, rng, false, 1.f);
        cf.seed(a, 160.f, 120.f, 60.f);
        CHECK(cf.ready(), "shift %d: seed found no points", sh);
        GrayFrame b = makeFrame(320, 240, 160.f + sh, 120.f + sh / 2.f, 60, rng, false, 1.f);
        float dx = 0, dy = 0;
        const bool ok = cf.step(b, dx, dy);
        CHECK(ok, "shift %d: no verdict", sh);
        if (!ok) continue;
        CHECK(std::fabs(dx - sh) <= 1.f, "shift %d: dx = %.1f", sh, dx);
        CHECK(std::fabs(dy - sh / 2.f) <= 1.f, "shift %d: dy = %.1f", sh, dy);
        std::printf("  coast flow: truth (%d,%.1f) measured (%.1f,%.1f)\n",
                    sh, sh / 2.f, dx, dy);
    }
}

}  // namespace

int main() {
    testRecoveryReach();
    testLossTimeoutIsSeconds();
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
    testCoastFlowMeasuresKnownShift();
    if (failures) { std::printf("\n%d FAILURE(S)\n", failures); return 1; }
    std::printf("\nall checks passed\n");
    return 0;
}
