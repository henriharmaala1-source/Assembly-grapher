#pragma once

#include <vector>

#include "center_filter.hpp"
#include "crop_filters.hpp"
#include "gray_frame.hpp"
#include "optical_flow.hpp"

// ---------------------------------------------------------------------------
// LockTracker -- the CURRENT lock-on tracker, ported from
// android-tracker/.../track/LockTracker.kt.
//
// THIS SUPERSEDES LockOnTracker in lock_tracker.hpp. That one is the older
// design (single CSRT/KCF/MOSSE backend + Kalman + NCC re-detect); the flight
// runtime still uses it, so both live here until the runtime is switched over.
// New work goes here.
//
// The validated architecture, in the order it runs:
//
//   1. EGO-MOTION feed-forward (P1-A) -- median grid flow, box-EXCLUDED and
//      forward-backward gated, added to the prediction so a camera pan does not
//      push the target out of the crop before the filter catches up.
//   2. FOLLOWED CROP sized from the box, so the target stays ~constant size from
//      50 to 800 m and one template works across the range.
//   3. CUE FUSION -- several channels (luma / edge / chroma) matched at once,
//      each weighted by its OWN PSR and by agreement with the prediction. A cue
//      that is useless right now contributes nothing instead of voting wrong.
//   4. ANCHOR + ADAPTIVE + KEYFRAME BANK (P1-B) -- the fixed anchor re-anchors a
//      drifted adaptive template; diverse-pose keyframes are consulted only as a
//      targeted fallback when the primary response is weak.
//   5. STAPLE-style HISTOGRAM cue -- chroma fg/bg with no spatial layout, so it
//      survives the deformation and occlusion boundaries that break spatial NCC.
//      Biggest single win of the research round (sim occlusion lock 51%->95%).
//   6. OCCLUSION-AWARE ADAPTATION (P2-B) -- a PSR collapse against the running
//      clean baseline freezes adaptation, banking and scale, with hysteresis on
//      BOTH ends so a target that legitimately gets harder is not frozen forever.
//   7. SEARCHING (P0-A) -- once coasting has clearly failed, zoom the FOV out and
//      re-acquire with the ANCHOR ALONE, because a drifted adaptive template
//      driving a big coarse scan is how you re-lock onto background.
//
// DELIBERATELY OPENCV-FREE. The Kotlin runs on a phone and this must run on a Pi
// and, later, an MCU; keeping it to plain float arrays is also what lets it be
// unit-tested headless and diffed against desktop/simtrack.py, which is the
// validation mirror for BOTH implementations. Any cv::Mat belongs at the caller.
//
// The tunables below are NOT free parameters -- most were swept in simtrack.py
// and several are non-monotonic. See android-tracker/TRACKER_PLAN.md before
// changing one, and re-run the A/B rather than reasoning about it.
// ---------------------------------------------------------------------------

namespace track {

class LockTracker {
public:
    // COASTING = briefly lost, riding the constant-velocity prediction in the
    // normal crop. SEARCHING = coasting failed; the search has zoomed out and is
    // scanning a wide region with the fixed ANCHOR templates to re-acquire.
    enum class State { IDLE, LOCKED, COASTING, SEARCHING, LOST };
    static const char* stateName(State s);

    struct Result {
        State state = State::IDLE;
        int   x = 0, y = 0, w = 0, h = 0;     // box, full-frame px
        float conf = 0.f;                     // 0..1 from fused PSR
        float predX = 0.f, predY = 0.f;       // near-term prediction
        float aimX = 0.f, aimY = 0.f;         // latency-compensated aim
    };

    // --- tunables (see header note before touching) --------------------------
    float psrLock       = 5.5f;
    float psrWarn       = 3.8f;
    float latencyFrames = 4.5f;   // ~150 ms at 30 fps -- output aim leads by this

    void setCues(const std::vector<CropFilter>& c);
    const std::vector<CropFilter>& cues() const { return cues_; }

    void reset();
    void designate(const GrayFrame& frame, float px, float py, float size = 64.f);
    Result update(const GrayFrame& frame);

    State state() const { return state_; }
    float conf()  const { return conf_; }
    bool  hasTarget() const {
        return state_ == State::LOCKED || state_ == State::COASTING
            || state_ == State::SEARCHING;
    }
    // Per-stage cost, ms, EMA. Split finely because the stages scale with
    // different things: flow with the camera stream size, cue with crop x
    // template^2 x cues.
    float tFlowMs() const { return tFlowMs_; }
    float tCropMs() const { return tCropMs_; }
    float tCueMs()  const { return tCueMs_; }

private:
    // Perf-tuned in simulation: ~8.7x cheaper per cue than 40/30/2 with
    // equal-or-better accuracy (a smaller template is less scale-sensitive).
    // NCC cost ~ positions x template^2, so these three dominate frame cost.
    static constexpr int   CROP   = 128;
    static constexpr int   TMPL   = 28;
    static constexpr float MARGIN = 2.2f;
    static constexpr int   SEARCH = 22;
    static constexpr int   STRIDE = 3;
    static constexpr int   LOSS_TIMEOUT = 45;  // coasting frames before giving up
    static constexpr int   FOV_DELAY    = 6;   // coasting frames before zooming out
    static constexpr float TMPL_EMA     = 0.08f;
    static constexpr int   K_KEYFRAMES  = 2;
    static constexpr float KF_THRESH    = 0.55f;
    static constexpr float KF_ADD_CONF  = 0.80f;
    static constexpr float OCC_FRAC     = 0.55f;
    static constexpr int   OCC_ENTER    = 2;
    static constexpr int   OCC_MAX      = 20;
    static constexpr float EGO_CONS     = 0.6f;
    static constexpr float EGO_DEAD     = 1.5f;
    static constexpr float EARLY_TERM_PSR = 10.f;
    static constexpr int   HIST_BINS    = 64;   // 8x8 quantised (cu,cv)
    static constexpr float HIST_LAMBDA  = 1.f;
    static constexpr float HIST_EMA     = 0.08f;
    // Sim-swept and NON-MONOTONIC: 1.0 let the histogram DOMINATE a merely-noisy
    // (not truly occluded) spatial cue and regressed noisy-feed lock 98->86 %;
    // 0.5 keeps most of the occlusion win (51->95 %) and recovers the noisy case.
    static constexpr float HIST_WEIGHT_CAP = 0.5f;

    using Patch = std::vector<float>;

    void  buildTemplates(const GrayFrame& rawCrop);
    void  adaptTemplates(const GrayFrame& rawCrop, float cx, float cy);
    void  maybeBankKeyframe(int ci, const Patch& fresh);
    void  updateScale(const GrayFrame& crop, float cx, float cy);
    void  applyDistractorPrior(std::vector<float>& fused, int gw, float cc) const;

    std::vector<float> responseMap(const GrayFrame& crop, const Patch& t, float tn,
                                   int g0, int g1, int gw, int stride) const;
    float nccAt(const GrayFrame& crop, float cx, float cy,
                const Patch& t, float tn) const;
    std::vector<float> histResponse(const GrayFrame& crop, int g0, int g1,
                                    int gw, int stride);
    int   histBinIndex(const GrayFrame& crop, int idx) const;
    void  histCountsAt(const GrayFrame& crop, float cx, float cy,
                       Patch& fg, Patch& bg) const;

    static void  subPixelPeak(const std::vector<float>& resp, int gw,
                              float& sx, float& sy);
    static float psrOf(const std::vector<float>& resp, int gw);
    static float psrToConf(float psr);
    static Patch meanSub(const float* a, int n);
    static float normOf(const Patch& a);
    static float dot(const Patch& a, const Patch& b);
    Patch normPatch(const GrayFrame& g, float cx, float cy, int size) const;

    GrayFrame workingCropRaw(const GrayFrame& frame, float cx, float cy,
                             float size) const;
    Result    result() const;
    float     confFloor() const { return psrToConf(psrWarn); }
    float     confLock()  const { return psrToConf(psrLock); }

    std::vector<CropFilter> cues_{CropFilter::NONE};

    std::vector<Patch> templates_, anchors_;
    std::vector<float> tmplNorms_, anchorNorms_;
    std::vector<std::vector<Patch>> keyframes_;
    std::vector<std::vector<float>> keyframeNorms_;
    Patch lumaTmpl_;
    float lumaNorm_ = 0.f;
    Patch histFg_, histBg_;
    std::vector<float> histBeta_, histII_;      // reused scratch, no per-frame alloc

    GrayFrame lastRawCrop_, prevFrame_;
    bool  haveLastCrop_ = false, havePrev_ = false;
    // The previous frame's LUMA, deep-copied. Callers are encouraged to reuse
    // their conversion buffers (that is the whole point of cropResampleInto), so
    // holding a VIEW of the caller's frame would silently make prevFrame_ alias
    // the CURRENT frame -- optical flow would then compare a frame with itself,
    // return (0,0) at consensus 1.0, and the ego-motion feed-forward would be
    // dead with no symptom. Owning the copy makes that unrepresentable.
    std::vector<float> prevLuma_;
    void stashPrev(const GrayFrame& f);

    float bcx_ = 0, bcy_ = 0, bsize_ = 0;
    CenterFilter cf_;
    OpticalFlow  flow_;
    int   badFrames_ = 0;
    float psrEma_ = 0.f;
    int   occLow_ = 0;
    State state_ = State::IDLE;
    float conf_  = 0.f;
    float tFlowMs_ = 0, tCropMs_ = 0, tCueMs_ = 0;

    // Exactly-sized crop scratch (see GrayFrame::cropResampleInto).
    std::vector<float> cropD_, cropU_, cropV_, filtBuf_;
    void ensureScratch(int n, bool colour);
};

}  // namespace track
