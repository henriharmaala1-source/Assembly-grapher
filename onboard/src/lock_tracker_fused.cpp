#include "lock_tracker_fused.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

namespace track {

namespace {
inline double nowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

const char* LockTracker::stateName(State s) {
    switch (s) {
        case State::IDLE:      return "IDLE";
        case State::LOCKED:    return "LOCKED";
        case State::COASTING:  return "COASTING";
        case State::SEARCHING: return "SEARCHING";
        case State::LOST:      return "LOST";
    }
    return "?";
}

// Deep-copy the frame's luma so prevFrame_ cannot alias caller scratch. Only
// luma is kept: OpticalFlow reads .d and nothing else.
void LockTracker::stashPrev(const GrayFrame& f) {
    const int n = f.w * f.h;
    if ((int)prevLuma_.size() != n) prevLuma_.assign(n, 0.f);
    std::memcpy(prevLuma_.data(), f.d, size_t(n) * sizeof(float));
    prevFrame_ = GrayFrame{};
    prevFrame_.d = prevLuma_.data();
    prevFrame_.w = f.w; prevFrame_.h = f.h;
    havePrev_ = true;
}

void LockTracker::ensureScratch(int n, bool colour) {
    if ((int)cropD_.size() != n)   cropD_.assign(n, 0.f);
    if ((int)filtBuf_.size() != n) filtBuf_.assign(n, 0.f);
    if (colour) {
        if ((int)cropU_.size() != n) cropU_.assign(n, 0.f);
        if ((int)cropV_.size() != n) cropV_.assign(n, 0.f);
    }
}

void LockTracker::reset() {
    templates_.clear(); tmplNorms_.clear();
    anchors_.clear();   anchorNorms_.clear();
    keyframes_.clear(); keyframeNorms_.clear();
    lumaTmpl_.clear();  lumaNorm_ = 0.f;
    histFg_.clear();    histBg_.clear();
    haveLastCrop_ = false; havePrev_ = false; prevLuma_.clear();
    state_ = State::IDLE; badFrames_ = 0; conf_ = 0.f; psrEma_ = 0.f; occLow_ = 0;
}

// Change the fused cue set. If locked, templates are rebuilt from the last crop
// so lock survives the switch (lets you A/B cues without re-designating).
void LockTracker::setCues(const std::vector<CropFilter>& c) {
    cues_ = c.empty() ? std::vector<CropFilter>{CropFilter::NONE} : c;
    if (hasTarget() && haveLastCrop_) buildTemplates(lastRawCrop_);
}

void LockTracker::designate(const GrayFrame& frame, float px, float py, float size) {
    bcx_ = px; bcy_ = py;
    bsize_ = std::min(std::max(size, 36.f), float(std::min(frame.w, frame.h)));
    lastRawCrop_ = workingCropRaw(frame, bcx_, bcy_, bsize_);
    haveLastCrop_ = true;
    buildTemplates(lastRawCrop_);
    cf_.start(px, py);
    badFrames_ = 0; conf_ = 1.f; state_ = State::LOCKED;
    psrEma_ = 0.f; occLow_ = 0;
    stashPrev(frame);
}

LockTracker::Result LockTracker::update(const GrayFrame& frame) {
    if (templates_.empty()) { stashPrev(frame); return Result{}; }

    // --- P1-A ego-motion feed-forward -------------------------------------
    // Gated on flow CONSENSUS: high on a rigid pan, low under noise or a large
    // occluder, so this fires only on a real camera pan and is a clean no-op on a
    // static/noisy feed. Deadband drops sub-pixel jitter; the cap stops a bad
    // estimate throwing the crop.
    const double tFlow0 = nowMs();
    float edx = 0.f, edy = 0.f;
    if (havePrev_ && prevFrame_.w == frame.w && prevFrame_.h == frame.h) {
        float fx = 0, fy = 0;
        flow_.estimate(prevFrame_, frame, bcx_, bcy_, bsize_ * MARGIN * 0.5f, fx, fy);
        if (flow_.consensus() >= EGO_CONS) {
            if (std::fabs(fx) < EGO_DEAD) fx = 0.f;
            if (std::fabs(fy) < EGO_DEAD) fy = 0.f;
            const float cap = bsize_ * MARGIN * 0.4f;   // never shift >~0.4 crop
            edx = std::min(std::max(fx, -cap), cap);
            edy = std::min(std::max(fy, -cap), cap);
        }
    }
    stashPrev(frame);
    tFlowMs_ = 0.9f * tFlowMs_ + 0.1f * float(nowMs() - tFlow0);

    float pcx = 0, pcy = 0;
    cf_.predict(edx, edy, pcx, pcy);

    // --- P0-A zoom the search out, but only once coasting has clearly FAILED --
    // For the first few lost frames the constant-velocity prediction still rides
    // along with the target, so the normal crop re-finds it; zooming out early
    // only adds a coarser peak and a bigger area to false-lock onto. After
    // FOV_DELAY misses, widen the FOV (to ~3x) by covering more frame area in a
    // proportionally larger buffer -- the target keeps its apparent SIZE so the
    // template still matches -- with a coarser stride to hold cost flat.
    const bool wide = badFrames_ >= FOV_DELAY;
    const float fov = wide
        ? std::min(1.f + 0.3f * (badFrames_ - FOV_DELAY + 1), 3.f) : 1.f;
    const int cropPix = (fov > 1.f) ? ((int(CROP * fov) / 2) * 2) : CROP;   // even
    const int strideEff = std::max(1, int(STRIDE * fov));
    const float regionW = bsize_ * MARGIN * fov;

    const double tCrop0 = nowMs();
    ensureScratch(cropPix * cropPix, frame.hasColor());
    const GrayFrame crop = frame.cropResampleInto(
        pcx - regionW / 2.f, pcy - regionW / 2.f, regionW, regionW,
        cropPix, cropPix, cropD_.data(),
        frame.hasColor() ? cropU_.data() : nullptr,
        frame.hasColor() ? cropV_.data() : nullptr);
    tCropMs_ = 0.9f * tCropMs_ + 0.1f * float(nowMs() - tCrop0);
    const double tCue0 = nowMs();

    // Search window: base + velocity, opened fully while coasting to re-find.
    const float velCrop = cf_.speed() * CROP / (bsize_ * MARGIN);
    const int maxHalf = cropPix / 2 - TMPL / 2;
    const int searchHalf = (badFrames_ > 0)
        ? maxHalf
        : std::min(std::max(int(SEARCH + velCrop * 2.f), SEARCH), maxHalf);

    // GRID_SYM. Round the half-width DOWN to a whole number of strides,
    // otherwise the grid is asymmetric about the prediction whenever
    // searchHalf % stride != 0 -- which is every default configuration:
    // [-22,+20] locked, [-50,+49] coasting, [-178,+173] in a 3x wide search. The
    // search then reached further one way than the other, and cc = (gw-1)/2
    // mislocated the prediction by up to 2.5 crop px, biasing the proximity
    // weight and the distractor prior DIRECTIONALLY. Centring error goes to 0 in
    // all three regimes. See the LATTICE_FIX note -- they ship together.
    const int searchHalfSym =
        std::max(strideEff, (searchHalf / strideEff) * strideEff);
    const int c0 = cropPix / 2;
    const int g0 = c0 - searchHalfSym, g1 = c0 + searchHalfSym;
    const int gw = (g1 - g0) / strideEff + 1;
    const float cc = (gw - 1) / 2.f;
    const float sigP = (badFrames_ > 0) ? gw / 1.4f : gw / 2.5f;

    std::vector<float> fused(size_t(gw) * gw, 0.f);
    float anyWeight = 0.f;

    for (size_t ci = 0; ci < cues_.size(); ++ci) {
        // One shared filter buffer is enough: only one filtered cue is live at a
        // time (anchor/adaptive/keyframe matches for cue ci all finish before
        // cue ci+1 is built).
        const GrayFrame cueCrop = applyFilter(crop, cues_[ci], filtBuf_.data(),
                                              (int)filtBuf_.size());
        // Match against the fixed ANCHOR and -- only when NOT wide-searching --
        // the adaptive template too, taking the better per position: the anchor
        // re-anchors when the adaptive has drifted, extending the lock. During a
        // wide re-acquire we use the ANCHOR ALONE, because the adaptive may have
        // drifted onto background before we lost the target and letting a drifted
        // template drive a big coarse scan is how you re-lock onto junk.
        std::vector<float> resp =
            responseMap(cueCrop, anchors_[ci], anchorNorms_[ci], g0, g1, gw, strideEff);
        if (!wide) {
            const std::vector<float> respA =
                responseMap(cueCrop, templates_[ci], tmplNorms_[ci], g0, g1, gw, strideEff);
            for (size_t i = 0; i < resp.size(); ++i)
                if (respA[i] > resp[i]) resp[i] = respA[i];
            // P1-B: consult diverse keyframes only as a TARGETED fallback --
            // while locked and not occluded AND the primary anchor+adaptive
            // response is weak (the pose-shift signature). An always-on max just
            // raises the response noise floor (sim: hurt occlusion and noisy).
            if (badFrames_ == 0 && !keyframes_[ci].empty() && psrOf(resp, gw) < psrLock) {
                for (size_t k = 0; k < keyframes_[ci].size(); ++k) {
                    const std::vector<float> rk = responseMap(
                        cueCrop, keyframes_[ci][k], keyframeNorms_[ci][k],
                        g0, g1, gw, strideEff);
                    for (size_t i = 0; i < resp.size(); ++i)
                        if (rk[i] > resp[i]) resp[i] = rk[i];
                }
            }
        }
        // Prediction-proximity: down-weight a cue whose peak drifts off-centre (a
        // distractor lock, or a confidently-wrong edge under scale -- PSR alone
        // cannot catch a sharp-but-wrong peak).
        size_t pk = 0; float pv = resp[0];
        for (size_t i = 0; i < resp.size(); ++i) if (resp[i] > pv) { pv = resp[i]; pk = i; }
        const float dxp = float(pk % gw) - cc, dyp = float(pk / gw) - cc;
        const float prox = std::exp(-(dxp * dxp + dyp * dyp) / (2.f * sigP * sigP));
        const float cuePsr = psrOf(resp, gw);
        const float wgt = std::max(cuePsr - 3.f, 0.f) * prox;
        if (wgt <= 0.f) continue;
        anyWeight += wgt;
        for (size_t i = 0; i < resp.size(); ++i) fused[i] += wgt * resp[i];
        // Early termination: once one cue is overwhelmingly dominant the
        // remaining cues' NCC is spent for negligible marginal fusion weight.
        if (cuePsr > EARLY_TERM_PSR) break;
    }
    tCueMs_ = 0.9f * tCueMs_ + 0.1f * float(nowMs() - tCue0);

    // STAPLE-style histogram cue -- chroma fg/bg, no spatial layout, so it
    // survives deformation and occlusion boundaries the spatial cues cannot. Only
    // during a normal-FOV search (the crop is CROP-sized, matching how the fg/bg
    // masks were built) and only when the frame carries chroma; the wide
    // re-acquire stays anchor-NCC-only. Weight is damped -- see HIST_WEIGHT_CAP.
    if (!wide && crop.hasColor() && !histFg_.empty()) {
        const std::vector<float> hresp = histResponse(crop, g0, g1, gw, strideEff);
        size_t hpk = 0; float hpv = hresp[0];
        for (size_t i = 0; i < hresp.size(); ++i)
            if (hresp[i] > hpv) { hpv = hresp[i]; hpk = i; }
        const float hdxp = float(hpk % gw) - cc, hdyp = float(hpk / gw) - cc;
        const float hprox = std::exp(-(hdxp * hdxp + hdyp * hdyp) / (2.f * sigP * sigP));
        const float hw = std::max(psrOf(hresp, gw) - 3.f, 0.f) * hprox * HIST_WEIGHT_CAP;
        if (hw > 0.f) {
            anyWeight += hw;
            for (size_t i = 0; i < hresp.size(); ++i) fused[i] += hw * hresp[i];
        }
    }

    if (anyWeight > 0.f) applyDistractorPrior(fused, gw, cc);
    const float curPsr = (anyWeight > 0.f) ? psrOf(fused, gw) : 0.f;
    conf_ = (anyWeight > 0.f) ? psrToConf(curPsr) : 0.f;

    // --- P2-B occlusion detection -----------------------------------------
    // A sharp PSR drop against the running CLEAN baseline is the occlusion
    // signature (the peak collapses, energy spreads). While occluded we still
    // track the visible part for POSITION but freeze appearance adaptation,
    // keyframe banking and scale, so the template cannot drift onto the occluder
    // and wreck recovery.
    //
    // HYSTERESIS ON BOTH ENDS. Enter: a one-frame dip is sensor noise, not an
    // occluder. Exit: the baseline could only ratchet UP, so a target that
    // legitimately gets harder (recedes, fades) parked its PSR in the band and
    // was flagged occluded FOREVER, adaptation frozen for the rest of the flight
    // with no way back. An occlusion is transient by definition; a lasting drop
    // means the target changed, so re-baseline after OCC_MAX.
    const bool low = psrEma_ > 0.f && curPsr < OCC_FRAC * psrEma_;
    occLow_ = low ? occLow_ + 1 : 0;
    if (occLow_ > OCC_MAX) { psrEma_ = curPsr; occLow_ = 0; }
    const bool occluded = occLow_ >= OCC_ENTER && occLow_ <= OCC_MAX;
    if (!occluded && curPsr > psrLock)
        psrEma_ = (psrEma_ <= 0.f) ? curPsr : 0.9f * psrEma_ + 0.1f * curPsr;

    // Re-locking from a zoomed-out search demands a STRONG match -- a coarse scan
    // over a large area would otherwise re-lock onto background.
    const float acceptConf = wide ? confLock() : confFloor();
    if (anyWeight > 0.f && conf_ >= acceptConf) {
        float sx = 0, sy = 0;
        subPixelPeak(fused, gw, sx, sy);
        const float cxCrop = g0 + sx * strideEff;
        const float cyCrop = g0 + sy * strideEff;
        const float nx = pcx + (cxCrop / cropPix - 0.5f) * regionW;
        const float ny = pcy + (cyCrop / cropPix - 0.5f) * regionW;
        cf_.correct(nx, ny);
        // A real target cannot cross more than ~0.9x its own size per frame at
        // 30 fps; anything faster is a noisy peak pumping the velocity.
        cf_.clampSpeed(bsize_ * 0.9f);
        bcx_ = cf_.x(); bcy_ = cf_.y();
        if (!occluded) updateScale(crop, cxCrop, cyCrop);
        if (conf_ >= confLock() && !occluded) adaptTemplates(crop, cxCrop, cyCrop);
        state_ = State::LOCKED; badFrames_ = 0;
    } else {
        cf_.decay(0.6f);                  // coast decelerates instead of flying off
        bcx_ = pcx; bcy_ = pcy;
        ++badFrames_;
        state_ = (badFrames_ >= LOSS_TIMEOUT) ? State::LOST
               : wide                        ? State::SEARCHING
                                             : State::COASTING;
        if (state_ == State::LOST) { Result r = result(); reset(); r.state = State::LOST; return r; }
    }

    if (state_ == State::LOCKED) {
        lastRawCrop_ = workingCropRaw(frame, bcx_, bcy_, bsize_);
        haveLastCrop_ = true;
    }
    return result();
}

// --- fusion helpers --------------------------------------------------------

void LockTracker::buildTemplates(const GrayFrame& rawCrop) {
    const size_t n = cues_.size();
    templates_.resize(n); tmplNorms_.resize(n);
    anchors_.resize(n);   anchorNorms_.resize(n);
    keyframes_.assign(n, {}); keyframeNorms_.assign(n, {});
    for (size_t ci = 0; ci < n; ++ci) {
        const GrayFrame f = applyFilter(rawCrop, cues_[ci], nullptr, 0);
        templates_[ci] = normPatch(f, CROP / 2.f, CROP / 2.f, TMPL);
        tmplNorms_[ci] = normOf(templates_[ci]);
        // Fixed anchors = the original views. Matching anchor-OR-adaptive stops
        // the adaptive template drifting onto background over a long hold.
        anchors_[ci] = templates_[ci];
        anchorNorms_[ci] = tmplNorms_[ci];
    }
    // Scale runs on LUMA -- the scale-robust channel; edge blurs under downsample
    // and mis-scales.
    lumaTmpl_ = normPatch(rawCrop, CROP / 2.f, CROP / 2.f, TMPL);
    lumaNorm_ = normOf(lumaTmpl_);
    if (rawCrop.hasColor()) histCountsAt(rawCrop, CROP / 2.f, CROP / 2.f, histFg_, histBg_);
    else { histFg_.clear(); histBg_.clear(); }
}

void LockTracker::adaptTemplates(const GrayFrame& rawCrop, float cx, float cy) {
    // Only bank on a very clean lock -- a partial-occlusion / ambiguous view has
    // degraded PSR, so this gate keeps a contaminated patch out of the bank (sim:
    // without it an occluder-half keyframe wrecked recovery).
    const bool canBank = conf_ >= KF_ADD_CONF && badFrames_ == 0;
    for (size_t ci = 0; ci < cues_.size(); ++ci) {
        const GrayFrame f = applyFilter(rawCrop, cues_[ci], filtBuf_.data(),
                                        (int)filtBuf_.size());
        const Patch fresh = normPatch(f, cx, cy, TMPL);
        Patch& cur = templates_[ci];
        for (size_t i = 0; i < cur.size(); ++i)
            cur[i] = (1.f - TMPL_EMA) * cur[i] + TMPL_EMA * fresh[i];
        tmplNorms_[ci] = normOf(cur);
        if (canBank) maybeBankKeyframe((int)ci, fresh);
    }
    const Patch fl = normPatch(rawCrop, cx, cy, TMPL);
    for (size_t i = 0; i < lumaTmpl_.size(); ++i)
        lumaTmpl_[i] = (1.f - TMPL_EMA) * lumaTmpl_[i] + TMPL_EMA * fl[i];
    lumaNorm_ = normOf(lumaTmpl_);
    // Refresh the histogram only on the same very-clean gate -- an
    // occlusion-tainted frame must not corrupt the cumulative fg/bg model.
    if (canBank && rawCrop.hasColor() && !histFg_.empty()) {
        Patch fg, bg;
        histCountsAt(rawCrop, cx, cy, fg, bg);
        for (int i = 0; i < HIST_BINS; ++i) {
            histFg_[i] = (1.f - HIST_EMA) * histFg_[i] + HIST_EMA * fg[i];
            histBg_[i] = (1.f - HIST_EMA) * histBg_[i] + HIST_EMA * bg[i];
        }
    }
}

// Add `fresh` iff it is a genuinely new appearance (below KF_THRESH similarity to
// every current slot), keeping the bank diverse.
void LockTracker::maybeBankKeyframe(int ci, const Patch& fresh) {
    const float fn = normOf(fresh);
    // KF_DEGEN_GUARD. A patch that is locally FLAT in this cue (a uniform target
    // in chroma, a texture-free one in edge) has norm 1e-6 from normOf's floor,
    // so its similarity to every stored slot is 0/(1e-6*1e-6) = 0 -- read as
    // MAXIMALLY NOVEL, so it always passes the gate below. Worse, eviction keeps
    // it forever: a zero patch is never the most-redundant slot. Measured in the
    // Python reference: both chroma slots fill with zeros by update frame 12,
    // and are then max'd into the response exactly when the primary is weak --
    // the case the bank exists for -- rectifying every negative correlation to 0
    // and corrupting the sidelobe statistics psrOf measures.
    if (fn < 1e-3f) return;
    float maxSim = dot(fresh, anchors_[ci]) / (fn * anchorNorms_[ci]);
    maxSim = std::max(maxSim, dot(fresh, templates_[ci]) / (fn * tmplNorms_[ci]));
    auto& kf = keyframes_[ci];
    auto& kn = keyframeNorms_[ci];
    for (size_t k = 0; k < kf.size(); ++k)
        maxSim = std::max(maxSim, dot(fresh, kf[k]) / (fn * kn[k]));
    if (maxSim >= KF_THRESH) return;
    kf.push_back(fresh); kn.push_back(fn);
    if ((int)kf.size() > K_KEYFRAMES) {
        // Evict the most REDUNDANT slot (highest similarity to some other kept
        // slot), not simply the oldest -- keeps distinct poses (front + side)
        // instead of whichever happens to be newest.
        size_t worstIdx = 0; float worstSim = -1.f;
        for (size_t i = 0; i < kf.size(); ++i) {
            float s = -1.f;
            for (size_t j = 0; j < kf.size(); ++j) {
                if (j == i) continue;
                const float sim = dot(kf[i], kf[j]) / (kn[i] * kn[j]);
                if (sim > s) s = sim;
            }
            if (s > worstSim) { worstSim = s; worstIdx = i; }
        }
        kf.erase(kf.begin() + worstIdx);
        kn.erase(kn.begin() + worstIdx);
    }
}

// --- STAPLE-style histogram appearance cue ---------------------------------

int LockTracker::histBinIndex(const GrayFrame& crop, int idx) const {
    const int cu = std::min(std::max(int((crop.cu[idx] + 128.f) / 32.f), 0), 7);
    const int cv = std::min(std::max(int((crop.cv[idx] + 128.f) / 32.f), 0), 7);
    return cu * 8 + cv;
}

// fg/bg per-bin pixel counts: fg = TMPL box centred at the ACTUAL found position,
// not the crop centre (which drifts from prediction error).
void LockTracker::histCountsAt(const GrayFrame& crop, float cx, float cy,
                               Patch& fg, Patch& bg) const {
    fg.assign(HIST_BINS, 0.f);
    bg.assign(HIST_BINS, 0.f);
    const float half = TMPL / 2.f, bgMargin = TMPL * 0.75f;
    for (int y = 0; y < crop.h; ++y)
        for (int x = 0; x < crop.w; ++x) {
            const float dx = std::fabs(x - cx), dy = std::fabs(y - cy);
            const int idx = y * crop.w + x;
            if (dx <= half && dy <= half) fg[histBinIndex(crop, idx)] += 1.f;
            else if (dx > half + bgMargin || dy > half + bgMargin)
                bg[histBinIndex(crop, idx)] += 1.f;
        }
}

// Per-candidate mean fg-probability via an integral image, so cost is
// O(crop + positions) instead of O(positions x template^2) like NCC.
std::vector<float> LockTracker::histResponse(const GrayFrame& crop, int g0, int g1,
                                             int gw, int stride) {
    const int w = crop.w, h = crop.h;
    if ((int)histBeta_.size() != w * h) histBeta_.assign(size_t(w) * h, 0.f);
    if ((int)histII_.size() != (w + 1) * (h + 1))
        histII_.assign(size_t(w + 1) * (h + 1), 0.f);
    for (int i = 0; i < w * h; ++i) {
        const int b = histBinIndex(crop, i);
        histBeta_[i] = histFg_[b] / (histFg_[b] + histBg_[b] + HIST_LAMBDA);
    }
    // NOTE: row 0 and column 0 of histII_ must stay zero (the box-sum reads them
    // as the integral-image border). The fill below only ever writes (y+1,x+1),
    // so they stay zero across reuse -- do not "optimise" the indexing without
    // re-zeroing that border.
    for (int y = 0; y < h; ++y) {
        float rowSum = 0.f;
        for (int x = 0; x < w; ++x) {
            rowSum += histBeta_[y * w + x];
            histII_[(y + 1) * (w + 1) + (x + 1)] = histII_[y * (w + 1) + (x + 1)] + rowSum;
        }
    }
    std::vector<float> resp(size_t(gw) * gw, 0.f);
    const int half = TMPL / 2;
    int gy = 0;
    for (int y = g0; y <= g1 && gy < gw; y += stride, ++gy) {
        int gx = 0;
        for (int x = g0; x <= g1 && gx < gw; x += stride, ++gx) {
            const int x0 = std::min(std::max(x - half, 0), w);
            const int y0 = std::min(std::max(y - half, 0), h);
            const int x1 = std::min(std::max(x + half, 0), w);
            const int y1 = std::min(std::max(y + half, 0), h);
            const int area = (x1 - x0) * (y1 - y0);
            if (area > 0) {
                const float s = histII_[y1 * (w + 1) + x1] - histII_[y0 * (w + 1) + x1]
                              - histII_[y1 * (w + 1) + x0] + histII_[y0 * (w + 1) + x0];
                resp[size_t(gy) * gw + gx] = s / area;
            }
        }
    }
    return resp;
}

// NCC of a template over the search grid. Stride is passed in -- it widens while
// coasting so the zoomed-out re-acquire costs the same as a normal locked search.
std::vector<float> LockTracker::responseMap(const GrayFrame& crop, const Patch& t,
                                            float tn, int g0, int g1, int gw,
                                            int stride) const {
    std::vector<float> resp(size_t(gw) * gw, 0.f);
    int gy = 0;
    for (int y = g0; y <= g1 && gy < gw; y += stride, ++gy) {
        int gx = 0;
        for (int x = g0; x <= g1 && gx < gw; x += stride, ++gx)
            resp[size_t(gy) * gw + gx] = nccAt(crop, float(x), float(y), t, tn);
    }
    return resp;
}

float LockTracker::nccAt(const GrayFrame& crop, float cx, float cy,
                         const Patch& t, float tn) const {
    const int hh = TMPL / 2;
    const int x0 = int(cx - hh), y0 = int(cy - hh);
    if (x0 < 0 || y0 < 0 || x0 + TMPL > crop.w || y0 + TMPL > crop.h) return -2.f;
    // TWO passes, deliberately. Folding them into one using
    //     sum((v-mean)*t) == sum(v*t)            (template is zero-mean)
    //     sum((v-mean)^2) == sum(v^2) - sum(v)^2/N
    // is true in exact arithmetic and UNSAFE in floating point on real inputs.
    // The template is only zero-mean to rounding, harmless until the patch is
    // FLAT: then the true numerator is zero, the denominator is zero, and the
    // rounding residual becomes the whole numerator divided by nothing. And the
    // variance identity subtracts two numbers both ~5e7 for 8-bit pixels over a
    // 28x28 patch to get a much smaller difference -- catastrophic cancellation
    // exactly on low-contrast patches. Measured on real frames in the Python
    // mirror: off by 2.9e+08 in the worst position, a spurious peak the argmax
    // would take. Real crops are full of flat regions (sky, saturated ground), so
    // this is a live failure mode. The saving was one read-only pass; not worth it.
    const float* cd = crop.d;
    const int cw = crop.w;
    float sum = 0.f;
    for (int j = 0; j < TMPL; ++j) {
        const float* row = cd + size_t(y0 + j) * cw + x0;
        for (int i = 0; i < TMPL; ++i) sum += row[i];
    }
    const float mean = sum / (TMPL * TMPL);
    float d = 0.f, pn = 0.f;
    int ti = 0;
    for (int j = 0; j < TMPL; ++j) {
        const float* row = cd + size_t(y0 + j) * cw + x0;
        for (int i = 0; i < TMPL; ++i, ++ti) {
            const float v = row[i] - mean;
            d += v * t[ti];
            pn += v * v;
        }
    }
    return d / (tn * (std::sqrt(pn) + 1e-6f));
}

void LockTracker::subPixelPeak(const std::vector<float>& resp, int gw,
                               float& sx, float& sy) {
    size_t pk = 0; float peak = resp[0];
    for (size_t i = 0; i < resp.size(); ++i) if (resp[i] > peak) { peak = resp[i]; pk = i; }
    const int px = int(pk % gw), py = int(pk / gw);
    float dx = 0.f, dy = 0.f;
    if (px >= 1 && px < gw - 1) {
        const float l = resp[size_t(py) * gw + px - 1];
        const float r = resp[size_t(py) * gw + px + 1];
        const float den = l - 2 * peak + r;
        if (std::fabs(den) > 1e-6f)
            dx = std::min(std::max(0.5f * (l - r) / den, -1.f), 1.f);
    }
    if (py >= 1 && py < gw - 1) {
        const float u = resp[size_t(py - 1) * gw + px];
        const float dn = resp[size_t(py + 1) * gw + px];
        const float den = u - 2 * peak + dn;
        if (std::fabs(den) > 1e-6f)
            dy = std::min(std::max(0.5f * (u - dn) / den, -1.f), 1.f);
    }
    sx = px + dx; sy = py + dy;
}

// Conditional spatial prior: if a rival peak exists (an identical distractor
// appearance cannot separate), bias the fused map toward the prediction. Only
// when a rival is present, so it never fights a lone target under scale.
void LockTracker::applyDistractorPrior(std::vector<float>& fused, int gw,
                                       float cc) const {
    size_t pk = 0; float pv = fused[0];
    for (size_t i = 0; i < fused.size(); ++i) if (fused[i] > pv) { pv = fused[i]; pk = i; }
    if (pv <= 0.1f) return;
    const int px = int(pk % gw), py = int(pk / gw);
    float pv2 = -1e9f; long pk2 = -1;
    for (size_t i = 0; i < fused.size(); ++i) {
        const int x = int(i % gw), y = int(i / gw);
        if (std::abs(x - px) <= 4 && std::abs(y - py) <= 4) continue;
        if (fused[i] > pv2) { pv2 = fused[i]; pk2 = long(i); }
    }
    if (pk2 < 0 || pv2 <= 0.6f * pv) return;
    const float d = std::hypot(float(pk2 % gw - px), float(pk2 / gw - py));
    if (d <= 5.f) return;
    const float sig = (badFrames_ > 0) ? gw / 1.5f : gw / 2.6f;
    for (size_t i = 0; i < fused.size(); ++i) {
        const float x = float(i % gw) - cc, y = float(i / gw) - cc;
        fused[i] *= std::exp(-(x * x + y * y) / (2.f * sig * sig));
    }
}

// Scale on LUMA -- the scale-robust channel; edge mis-scales.
void LockTracker::updateScale(const GrayFrame& crop, float cx, float cy) {
    static const float SCALES[3] = {0.9f, 1.0f, 1.11f};
    float best = -2.f, bestS = 1.f, ncc1 = 0.f;
    for (float s : SCALES) {
        const float ts = TMPL * s;
        const GrayFrame patch = crop.cropResample(cx - ts / 2.f, cy - ts / 2.f,
                                                  ts, ts, TMPL, TMPL);
        const Patch p = meanSub(patch.d, TMPL * TMPL);
        const float ncc = dot(p, lumaTmpl_) / (normOf(lumaTmpl_) * (normOf(p) + 1e-6f));
        if (s == 1.f) ncc1 = ncc;
        if (ncc > best) { best = ncc; bestS = s; }
    }
    // Dead-band: only rescale if a non-unity scale CLEARLY beats staying put.
    // Without this, feed noise makes 0.9 win by a hair most frames and the box
    // ratchets down to the floor every time (over-zoom + a tiny, unstable lock).
    if (bestS != 1.f && best < ncc1 + 0.03f) bestS = 1.f;
    const float lim = std::min(float(crop.w * 4), 2000.f);
    bsize_ = std::min(std::max(bsize_ * (1.f + (bestS - 1.f) * 0.5f), 36.f), lim);
}

// --- shared ----------------------------------------------------------------

float LockTracker::psrOf(const std::vector<float>& resp, int gw) {
    size_t pk = 0; float peak = resp[0];
    for (size_t i = 0; i < resp.size(); ++i) if (resp[i] > peak) { peak = resp[i]; pk = i; }
    const int px = int(pk % gw), py = int(pk / gw);
    float sum = 0, sum2 = 0; int n = 0;
    for (int y = 0; y < gw; ++y)
        for (int x = 0; x < gw; ++x) {
            if (std::abs(x - px) <= 3 && std::abs(y - py) <= 3) continue;
            const float v = resp[size_t(y) * gw + x];
            sum += v; sum2 += v * v; ++n;
        }
    if (n < 4) return 0.f;
    const float mean = sum / n;
    const float std_ = std::sqrt(std::max(sum2 / n - mean * mean, 1e-6f));
    return (peak - mean) / std_;
}

float LockTracker::psrToConf(float psr) {
    return std::min(std::max((psr - 3.f) / (12.f - 3.f), 0.f), 1.f);
}

LockTracker::Patch LockTracker::meanSub(const float* a, int n) {
    float s = 0.f;
    for (int i = 0; i < n; ++i) s += a[i];
    const float m = s / n;
    Patch o(n);
    for (int i = 0; i < n; ++i) o[i] = a[i] - m;
    return o;
}

float LockTracker::normOf(const Patch& a) {
    float s = 0.f;
    for (float v : a) s += v * v;
    return std::sqrt(s) + 1e-6f;
}

float LockTracker::dot(const Patch& a, const Patch& b) {
    float s = 0.f;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

LockTracker::Patch LockTracker::normPatch(const GrayFrame& g, float cx, float cy,
                                          int size) const {
    // LATTICE_FIX. The -0.5 puts the samples on the INTEGER lattice nccAt slices
    // from, instead of on pixel CENTRES half a pixel to the right of it.
    // cropResample samples centres (rx + i + 0.5) while nccAt indexes integer
    // rows, so without this every template was a half-pixel bilinear BLUR of the
    // source, offset half a pixel from every candidate it is compared against.
    //
    // Measured on an identical noiseless crop, where self-match must be 1.000:
    //     cue      peak unfixed -> fixed     PSR unfixed -> fixed
    //     none         0.4991 -> 1.0000        14.41 -> 28.57
    //     edge         0.5433 -> 1.0000        12.42 -> 27.39
    //     chroma       0.5017 -> 1.0000        13.87 -> 27.15
    // Every template was matching ITSELF at half strength, and PSR -- which
    // drives the fusion weight, the accept gate, the adapt gate and the
    // occlusion baseline -- was halved with it.
    //
    // Ships together with GRID_SYM in update(): the two are biases of OPPOSITE
    // sign that partly cancel, and fixing either alone measures worse than
    // fixing neither.
    const GrayFrame patch = g.cropResample(cx - size / 2.f - 0.5f,
                                           cy - size / 2.f - 0.5f,
                                           float(size), float(size), size, size);
    return meanSub(patch.d, size * size);
}

// Raw followed crop (luma + chroma, NO filter) -- filters are applied per cue.
GrayFrame LockTracker::workingCropRaw(const GrayFrame& frame, float cx, float cy,
                                      float size) const {
    const float r = size * MARGIN;
    return frame.cropResample(cx - r / 2.f, cy - r / 2.f, r, r, CROP, CROP);
}

LockTracker::Result LockTracker::result() const {
    Result out;
    const float half = bsize_ / 2.f;
    out.state = state_;
    out.x = int(bcx_ - half); out.y = int(bcy_ - half);
    out.w = int(bsize_);      out.h = int(bsize_);
    out.conf = conf_;
    cf_.project(2, out.predX, out.predY);
    cf_.project(int(latencyFrames + 0.5f), out.aimX, out.aimY);
    return out;
}

}  // namespace track
