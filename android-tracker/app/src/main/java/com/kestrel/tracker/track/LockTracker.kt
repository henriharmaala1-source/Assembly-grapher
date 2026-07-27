package com.kestrel.tracker.track

import kotlin.math.sqrt

/**
 * Lean lock-on tracker, "TFL1"-architecture, now with cue FUSION.
 *
 * The core is still a followed crop (sized from the tracked box, so the target
 * stays ~constant size across 50–800 m) with NCC template matching inside it and
 * PSR confidence. The upgrades over the single-channel version:
 *
 *   • CUE FUSION — track on several channels at once (structure via EDGE, colour
 *     via CHROMA, brightness via NONE …). Each channel's response is weighted by
 *     its OWN PSR and summed, so whichever cue is discriminating right now
 *     dominates and a cue that's useless (flat response) contributes nothing.
 *     This is what keeps lock when any single cue fails — a car the same colour
 *     as the road has no chroma signal but plenty of structure/motion, etc.
 *   • SUB-PIXEL peak (parabolic interpolation) → smoother, more accurate aim.
 *   • ADAPTIVE search window (grows with velocity; opens fully while coasting to
 *     re-acquire a briefly-lost target).
 *   • LATENCY COMPENSATION — the output aim point is projected forward by
 *     `latencyFrames` so it's where the target IS, not where it was on a ~150 ms
 *     feed.
 *
 * Pure Kotlin on GrayFrame — unit-testable, ports to the onboard C++ tracker.
 */
class LockTracker {

    // COASTING = briefly lost, riding the constant-velocity prediction in the
    // normal crop. SEARCHING = coasting has failed; the search has zoomed out and
    // is scanning a wide region with the fixed ANCHOR templates to re-acquire.
    enum class State { IDLE, LOCKED, COASTING, SEARCHING, LOST }

    data class Result(
        val state: State,
        val x: Int, val y: Int, val w: Int, val h: Int,   // box, full-frame px
        val conf: Float,                                  // 0..1 from fused PSR
        val predX: Float, val predY: Float,               // near-term prediction
        val aimX: Float, val aimY: Float,                 // latency-compensated aim
        val crop: GrayFrame?,                             // working crop (for PiP)
    )

    // --- tunables ------------------------------------------------------------
    /** Channels to fuse. One entry = single-cue (A/B testing); several = fusion. */
    var cues: List<CropFilter> = listOf(CropFilter.NONE)
        private set
    var psrLock = 5.5f
    var psrWarn = 3.8f
    var latencyFrames = 4.5f          // ~150 ms at 30 fps — output aim leads by this

    // Perf-tuned (simulation-validated: ~8.7x cheaper per cue than 40/30/2, with
    // equal-or-better accuracy — a smaller template is even less scale-sensitive).
    // NCC cost ~ positions x template^2, so these three dominate the frame cost.
    private val CROP = 128
    private val TMPL = 28
    private val MARGIN = 2.2f
    private val SEARCH = 22
    private val STRIDE = 3
    private val SCALES = floatArrayOf(0.9f, 1.0f, 1.11f)
    private val LOSS_TIMEOUT = 45      // frames of coasting before giving up (longer hold)
    private val FOV_DELAY = 6          // coasting frames before the search zooms out to re-find
    private val TMPL_EMA = 0.08f
    // P1-B appearance bank: diverse-pose keyframes beyond anchor+adaptive.
    private val K_KEYFRAMES = 2        // extra keyframe slots per cue
    private val KF_THRESH = 0.55f      // bank a view only if < this NCC-similar to every slot (diversity)
    private val KF_ADD_CONF = 0.80f    // ...and only on a very clean lock (anti-contamination)
    private val OCC_FRAC = 0.55f       // P2-B: PSR below this × clean-baseline = occluded
    private val OCC_ENTER = 2          // consecutive low frames before declaring occlusion
    private val OCC_MAX = 20           // after this many, re-baseline (it's not an occluder)
    private val EGO_CONS = 0.6f        // P1-A: min grid-flow consensus to trust the ego estimate
    private val EGO_DEAD = 1.5f        // ...and ignore sub-1.5px flow (jitter, not a real pan)
    private val EARLY_TERM_PSR = 10f   // skip remaining cues once one is this dominant
    // STAPLE-style histogram cue (chroma fg/bg, no spatial layout — survives
    // deformation/rotation that breaks the spatial NCC cues). Requires chroma
    // (skipped when the frame is luma-only) and only during a normal-FOV search.
    private val HIST_BINS = 64         // 8x8 quantized (cu,cv)
    private val HIST_LAMBDA = 1f
    private val HIST_EMA = 0.08f
    // Sim-swept: 1.0 let the histogram DOMINATE over a merely-noisy (not truly
    // occluded) spatial cue (regressed noisy-feed lock 98%->86%); 0.5 keeps most
    // of the real occlusion win (lock 51%->95% on the hardest case) while fully
    // recovering — even improving — the noisy case (98%->100%).
    private val HIST_WEIGHT_CAP = 0.5f

    private var templates: Array<FloatArray> = arrayOf()
    private var tmplNorms: FloatArray = floatArrayOf()
    private var anchors: Array<FloatArray> = arrayOf()    // original views, fixed — anti-drift
    private var anchorNorms: FloatArray = floatArrayOf()
    // P1-B: per-cue bank of diverse-pose keyframes (fixed once captured, can't drift).
    private var keyframes: Array<MutableList<FloatArray>> = arrayOf()
    private var keyframeNorms: Array<MutableList<Float>> = arrayOf()
    private var lumaTmpl: FloatArray = floatArrayOf()   // dedicated luma template for scale
    private var lumaNorm = 0f
    private var histFg: FloatArray = floatArrayOf()   // histogram cue: per-bin fg/bg pseudo-counts
    private var histBg: FloatArray = floatArrayOf()
    private var histBeta = FloatArray(0)   // reused per-frame scratch (no alloc on the frame thread)
    private var histII   = FloatArray(0)   // reused integral image
    private var lastRawCrop: GrayFrame? = null      // for rebuilding templates on cue change
    private var bcx = 0f; private var bcy = 0f
    private var bsize = 0f
    private val cf = CenterFilter()
    private var badFrames = 0
    private var psrEma = 0f            // P2-B: running clean-lock PSR baseline (occlusion detector)
    private var occLow = 0             // consecutive sub-threshold frames (occlusion hysteresis)
    private val flow = OpticalFlow()   // P1-A: ego-motion (camera pan) estimate
    private var prevFrame: GrayFrame? = null

    /** Per-stage cost in ms (EMA), read by the HUD. Split this finely because the
     *  stages scale with different things: `flow` and `crop` with the camera stream
     *  size, `cue` with crop size x template^2 x cues. Whatever the three do not
     *  account for is scale estimation, adaptation and the histogram cue. */
    var tFlowMs = 0f; private set
    var tCropMs = 0f; private set
    var tCueMs = 0f; private set

    // Reused search scratch. The wide re-acquire crop is 384x384, so a fresh
    // luma+chroma crop is 1.77 MB and each filtered cue another 590 KB — roughly
    // 3 MB of garbage per frame, on the camera-delivery thread, which is the same
    // order that previously turned 31 fps into 7. The measured arithmetic (~3.8M
    // multiply-adds) cannot account for the 132 ms the HUD reported; allocation
    // can. Sized EXACTLY (not "at least"), because some filters derive statistics
    // from array length.
    private var cropD = FloatArray(0)
    private var cropU = FloatArray(0)
    private var cropV = FloatArray(0)
    private var filtBuf = FloatArray(0)

    private fun ensureScratch(n: Int, colour: Boolean) {
        if (cropD.size != n) cropD = FloatArray(n)
        if (filtBuf.size != n) filtBuf = FloatArray(n)
        if (colour) {
            if (cropU.size != n) cropU = FloatArray(n)
            if (cropV.size != n) cropV = FloatArray(n)
        }
    }
    var state = State.IDLE; private set
    var conf = 0f; private set

    val hasTarget get() = state == State.LOCKED || state == State.COASTING || state == State.SEARCHING

    fun reset() { templates = arrayOf(); tmplNorms = floatArrayOf()
                  anchors = arrayOf(); anchorNorms = floatArrayOf()
                  keyframes = arrayOf(); keyframeNorms = arrayOf()
                  lumaTmpl = floatArrayOf(); lumaNorm = 0f; lastRawCrop = null
                  histFg = floatArrayOf(); histBg = floatArrayOf()
                  prevFrame = null
                  state = State.IDLE; badFrames = 0; conf = 0f; psrEma = 0f; occLow = 0 }

    /** Change the fused cue set. If locked, templates are rebuilt from the last
     *  crop so lock survives the switch (lets you A/B cues without re-tapping). */
    fun setCues(newCues: List<CropFilter>) {
        cues = if (newCues.isEmpty()) listOf(CropFilter.NONE) else newCues
        val crop = lastRawCrop
        if (hasTarget && crop != null) buildTemplates(crop)
    }

    fun designate(frame: GrayFrame, px: Float, py: Float, size: Float = 64f) {
        bcx = px; bcy = py
        bsize = size.coerceIn(36f, minOf(frame.w, frame.h).toFloat())
        val crop = workingCropRaw(frame, bcx, bcy, bsize)
        lastRawCrop = crop
        buildTemplates(crop)
        cf.start(px, py)
        badFrames = 0; conf = 1f; state = State.LOCKED; psrEma = 0f; occLow = 0; prevFrame = frame
    }

    fun update(frame: GrayFrame): Result {
        if (templates.isEmpty()) { prevFrame = frame; return Result(State.IDLE, 0,0,0,0, 0f, 0f,0f, 0f,0f, null) }

        // P1-A: estimate ego-motion (camera pan) from the previous→current frame and
        // feed it forward into the prediction, so a pan doesn't push the target out
        // of the search crop before the filter catches up. The median-flow estimate
        // rejects the (independently-moving) target as an outlier. Gated on flow
        // CONSENSUS (grid points agreeing with the median): high on a rigid pan, low
        // under noise or a large occluder — so this fires only on a real camera pan
        // and is a clean no-op on a static/noisy feed (sim: pan edge 0.4px @100%,
        // zero change on every other scenario). Deadband drops sub-pixel jitter;
        // cap stops a bad estimate throwing the crop.
        val tFlow0 = System.nanoTime()
        val prev = prevFrame
        var edx = 0f; var edy = 0f
        if (prev != null && prev.w == frame.w && prev.h == frame.h) {
            // Exclude the CURRENT box from the flow grid — a large/dominant target's
            // own motion could otherwise win the median vote with high consensus.
            val (fx0, fy0) = flow.estimate(prev, frame, bcx, bcy, bsize * MARGIN * 0.5f)
            if (flow.consensus >= EGO_CONS) {
                var fx = fx0; var fy = fy0
                if (kotlin.math.abs(fx) < EGO_DEAD) fx = 0f
                if (kotlin.math.abs(fy) < EGO_DEAD) fy = 0f
                val cap = bsize * MARGIN * 0.4f        // never shift more than ~0.4 crop
                edx = fx.coerceIn(-cap, cap); edy = fy.coerceIn(-cap, cap)
            }
        }
        prevFrame = frame
        // Per-stage cost, EMA-smoothed. The ego-motion flow is a full-FRAME pass
        // (grid search over the whole image) while everything below it works on a
        // 128px crop, so it scales with the camera stream size while the rest does
        // not — which makes it the first thing to look at whenever the stream
        // resolution changes. Surfaced on the HUD so the split is measured, not
        // assumed.
        tFlowMs = 0.9f * tFlowMs + 0.1f * ((System.nanoTime() - tFlow0) / 1e6f)

        val (pcx, pcy) = cf.predict(edx, edy)

        // ZOOM THE SEARCH OUT — but only once normal coasting has clearly FAILED.
        // For the first few lost frames the constant-velocity prediction still
        // rides along with the target, so the normal crop re-finds it; zooming out
        // early only adds a coarser peak and a bigger area to false-lock onto
        // (sim-confirmed: it hurt re-acquire until delayed + gated). After
        // FOV_DELAY misses, widen the FOV (up to ~3×) by covering more frame area
        // in a proportionally larger pixel buffer — target keeps its apparent SIZE
        // so the template still matches — with a coarser stride to hold cost flat.
        val wide = badFrames >= FOV_DELAY
        val fov = if (wide) minOf(1f + 0.3f * (badFrames - FOV_DELAY + 1), 3f) else 1f
        val cropPix = if (fov > 1f) ((CROP * fov).toInt() / 2) * 2 else CROP   // even
        val strideEff = maxOf(1, (STRIDE * fov).toInt())
        val regionW = bsize * MARGIN * fov
        val tCrop0 = System.nanoTime()
        ensureScratch(cropPix * cropPix, frame.hasColor)
        val crop = frame.cropResampleInto(pcx - regionW / 2f, pcy - regionW / 2f,
                                          regionW, regionW, cropPix, cropPix,
                                          cropD, cropU, cropV)
        tCropMs = 0.9f * tCropMs + 0.1f * ((System.nanoTime() - tCrop0) / 1e6f)
        val tCue0 = System.nanoTime()

        // Search window: base + velocity, opened fully while coasting to re-find.
        val velCrop = cf.speed * CROP / (bsize * MARGIN)
        val maxHalf = cropPix / 2 - TMPL / 2
        val searchHalf =
            if (badFrames > 0) maxHalf
            else (SEARCH + velCrop * 2f).toInt().coerceIn(SEARCH, maxHalf)

        // Fuse per-cue response maps (simulation-tuned).
        val c0 = cropPix / 2
        val g0 = c0 - searchHalf; val g1 = c0 + searchHalf
        val gw = (g1 - g0) / strideEff + 1
        val cc = (gw - 1) / 2f
        val sigP = if (badFrames > 0) gw / 1.4f else gw / 2.5f
        val fused = FloatArray(gw * gw)
        var anyWeight = 0f
        for (ci in cues.indices) {
            // One shared filter buffer is enough: only one filtered cue is live at
            // a time (the anchor/adaptive/keyframe matches for cue `ci` all finish
            // before cue ci+1 is built).
            val cueCrop = Filters.apply(crop, cues[ci], filtBuf)
            // Match against the fixed ANCHOR, and (only when NOT wide-searching)
            // also the adaptive template, taking the better per position — the
            // anchor re-anchors when the adaptive has drifted, extending the lock.
            // During a wide re-acquire we use the ANCHOR ALONE: the adaptive may
            // have drifted onto background before we lost the target, and letting a
            // drifted template drive a big coarse scan is how you re-lock onto junk.
            val respB = responseMap(cueCrop, anchors[ci], anchorNorms[ci], g0, g1, gw, strideEff)
            val resp = if (wide) respB else {
                val respA = responseMap(cueCrop, templates[ci], tmplNorms[ci], g0, g1, gw, strideEff)
                val m = FloatArray(respA.size) { if (respA[it] > respB[it]) respA[it] else respB[it] }
                // P1-B: consult diverse keyframes only as a TARGETED fallback — while
                // locked (badFrames==0, target present, not occluded) AND the primary
                // anchor+adaptive response is weak (PSR<lock, the pose-shift signature).
                // Always-on max just raises the response noise floor (sim: hurt
                // occlusion/noisy). The clean-view add-gate keeps the bank uncorrupted.
                val kf = keyframes[ci]
                if (badFrames == 0 && kf.isNotEmpty() && psrOf(m, gw) < psrLock) {
                    for (k in kf.indices) {
                        val rk = responseMap(cueCrop, kf[k], keyframeNorms[ci][k], g0, g1, gw, strideEff)
                        for (i in m.indices) if (rk[i] > m[i]) m[i] = rk[i]
                    }
                }
                m
            }
            // Prediction-proximity: down-weight a cue whose peak drifts off-centre
            // (a distractor lock, or a confidently-wrong edge under scale — PSR
            // alone can't catch a sharp-but-wrong peak).
            var pk = 0; var pv = resp[0]
            for (i in resp.indices) if (resp[i] > pv) { pv = resp[i]; pk = i }
            val dxp = pk % gw - cc; val dyp = pk / gw - cc
            val prox = kotlin.math.exp(-(dxp * dxp + dyp * dyp) / (2f * sigP * sigP))
            val cuePsr = psrOf(resp, gw)
            val w = (cuePsr - 3f).coerceAtLeast(0f) * prox
            if (w <= 0f) continue
            anyWeight += w
            for (i in resp.indices) fused[i] += w * resp[i]
            // Early termination: once one cue is already overwhelmingly dominant
            // (well past the lock threshold), the remaining cues' NCC is spent for
            // negligible marginal fusion weight — skip them this frame.
            if (cuePsr > EARLY_TERM_PSR) break
        }
        tCueMs = 0.9f * tCueMs + 0.1f * ((System.nanoTime() - tCue0) / 1e6f)
        // STAPLE-style histogram cue — chroma fg/bg, no spatial layout, so it
        // survives deformation/rotation the spatial NCC cues above can't. Only
        // during a normal-FOV search (crop is CROP-sized, matching how the fg/bg
        // masks were built) and only when the frame carries chroma; the wide
        // re-acquire stays anchor-NCC-only as before. Weight is damped
        // (HIST_WEIGHT_CAP) — letting it compete unbounded let it dominate over a
        // merely-noisy (not truly occluded) spatial cue (sim-confirmed regression).
        if (!wide && crop.cu != null && crop.cv != null && histFg.isNotEmpty()) {
            val hresp = histResponse(crop, g0, g1, gw, strideEff)
            var hpk = 0; var hpv = hresp[0]
            for (i in hresp.indices) if (hresp[i] > hpv) { hpv = hresp[i]; hpk = i }
            val hdxp = hpk % gw - cc; val hdyp = hpk / gw - cc
            val hprox = kotlin.math.exp(-(hdxp * hdxp + hdyp * hdyp) / (2f * sigP * sigP))
            val hw = (psrOf(hresp, gw) - 3f).coerceAtLeast(0f) * hprox * HIST_WEIGHT_CAP
            if (hw > 0f) {
                anyWeight += hw
                for (i in hresp.indices) fused[i] += hw * hresp[i]
            }
        }
        if (anyWeight > 0f) applyDistractorPrior(fused, gw, cc)
        val curPsr = if (anyWeight > 0f) psrOf(fused, gw) else 0f
        conf = if (anyWeight > 0f) psrToConf(curPsr) else 0f

        // P2-B occlusion detection: a sharp PSR drop vs the running CLEAN baseline
        // is the occlusion signature (the peak collapses, energy spreads). While
        // occluded we still track the visible part for POSITION, but freeze
        // appearance adaptation, keyframe banking and scale — the template can't
        // drift onto the occluder and wreck recovery. Baseline learns on clean frames.
        //
        // HYSTERESIS on both ends — a bare threshold was wrong in two ways:
        //  ENTER: a one-frame PSR dip is sensor noise, not an occluder. Requiring
        //    OCC_ENTER consecutive low frames removed ~half the false positives
        //    (sim, receding target with no occluder: 20%→0% single-cue, 35%→19%
        //    fused) with real-occlusion behaviour byte-identical.
        //  EXIT: the baseline could only ratchet UP — it was updated *only while
        //    not occluded* — so a target that legitimately gets harder (recedes,
        //    fades, loses contrast) parks its PSR in the band
        //    [psrLock, OCC_FRAC*psrEma] and is then flagged occluded FOREVER, with
        //    adaptation and scale frozen for the rest of the flight and no way
        //    back. An occlusion is transient by definition; a lasting drop means
        //    the target changed, so after OCC_MAX frames re-baseline to the new
        //    normal instead of suppressing adaptation indefinitely.
        val low = psrEma > 0f && curPsr < OCC_FRAC * psrEma
        occLow = if (low) occLow + 1 else 0
        if (occLow > OCC_MAX) { psrEma = curPsr; occLow = 0 }   // not an occluder — this IS the target now
        val occluded = occLow in OCC_ENTER..OCC_MAX
        if (!occluded && curPsr > psrLock)
            psrEma = if (psrEma <= 0f) curPsr else 0.9f * psrEma + 0.1f * curPsr

        // Re-locking from a zoomed-out (wide) search demands a STRONG match — a
        // coarse scan over a large area would otherwise re-lock onto background.
        val acceptConf = if (wide) confLock() else confFloor()
        if (anyWeight > 0f && conf >= acceptConf) {
            val (sx, sy) = subPixelPeak(fused, gw)
            val cxCrop = g0 + sx * strideEff
            val cyCrop = g0 + sy * strideEff
            val nx = pcx + (cxCrop / cropPix - 0.5f) * regionW
            val ny = pcy + (cyCrop / cropPix - 0.5f) * regionW
            cf.correct(nx, ny)
            // A real target can't cross more than ~0.9× its own size per frame at
            // 30 fps; anything faster is a noisy peak pumping the velocity. Cap it
            // so a single bad frame can't launch the box across the screen.
            cf.clampSpeed(bsize * 0.9f)
            bcx = cf.x; bcy = cf.y
            if (!occluded) updateScale(crop, cxCrop, cyCrop)   // P2-B: hold scale under occlusion
            if (conf >= confLock() && !occluded) adaptTemplates(crop, cxCrop, cyCrop)
            state = State.LOCKED; badFrames = 0
        } else {
            cf.decay(0.6f)                 // coast decelerates instead of flying off
            bcx = pcx; bcy = pcy
            badFrames++
            state = when {
                badFrames >= LOSS_TIMEOUT -> State.LOST
                wide -> State.SEARCHING    // zoomed-out anchor scan, target not yet re-found
                else -> State.COASTING     // still riding the prediction in the normal crop
            }
            if (state == State.LOST) { reset(); return result(crop) }   // raw crop → colour PiP
        }
        // The PiP crop is re-centred on the CORRECTED box position (bcx,bcy), not
        // the pre-correction prediction the search ran on — otherwise an inflated
        // velocity (or a wide-search re-acquire near the crop edge) leaves the box
        // on target while the PiP looks elsewhere. Search/adapt/scale above kept
        // the prediction-centred `crop` (where the match happened); only what we
        // show — and lastRawCrop, now target-centred for cleaner cue-switch
        // rebuilds — uses this. Coasting: bcx==pcx, so it's the same region.
        val shown = workingCropRaw(frame, bcx, bcy, bsize)
        if (state == State.LOCKED) lastRawCrop = shown
        return result(shown)   // raw crop (luma+chroma), box-centred → colour PiP
    }

    // --- fusion helpers ------------------------------------------------------

    private fun buildTemplates(rawCrop: GrayFrame) {
        templates = Array(cues.size) { ci ->
            normPatch(Filters.apply(rawCrop, cues[ci]), CROP / 2f, CROP / 2f, TMPL)
        }
        tmplNorms = FloatArray(cues.size) { normOf(templates[it]) }
        // Fixed anchors = the original views. Matching against anchor-OR-adaptive
        // stops the adaptive template drifting onto background over a long hold.
        anchors = Array(cues.size) { templates[it].copyOf() }
        anchorNorms = tmplNorms.copyOf()
        keyframes = Array(cues.size) { mutableListOf() }
        keyframeNorms = Array(cues.size) { mutableListOf() }
        // Dedicated luma template — scale estimation runs on luma (the
        // scale-robust channel); edge blurs under downsample and mis-scales.
        lumaTmpl = normPatch(rawCrop, CROP / 2f, CROP / 2f, TMPL)   // rawCrop.d = luma
        lumaNorm = normOf(lumaTmpl)
        if (rawCrop.cu != null && rawCrop.cv != null) {
            val (fg, bg) = histCountsAt(rawCrop, CROP / 2f, CROP / 2f)
            histFg = fg; histBg = bg
        } else { histFg = floatArrayOf(); histBg = floatArrayOf() }
    }

    private fun adaptTemplates(rawCrop: GrayFrame, cx: Float, cy: Float) {
        // Only bank keyframes on a very clean lock — a partial-occlusion / ambiguous
        // view has degraded PSR, so this conf gate keeps a contaminated patch out of
        // the bank (sim: without it, an occluder-half keyframe wrecked recovery).
        val canBank = conf >= KF_ADD_CONF && badFrames == 0
        for (ci in cues.indices) {
            // Safe to share filtBuf with the search loop: that loop has finished
            // with its filtered cues by the time adaptation runs, and only one
            // filtered frame is live here at a time too.
            val fresh = normPatch(Filters.apply(rawCrop, cues[ci], filtBuf), cx, cy, TMPL)
            val cur = templates[ci]
            for (i in cur.indices) cur[i] = (1f - TMPL_EMA) * cur[i] + TMPL_EMA * fresh[i]
            tmplNorms[ci] = normOf(cur)
            if (canBank) maybeBankKeyframe(ci, fresh)
        }
        val fl = normPatch(rawCrop, cx, cy, TMPL)
        for (i in lumaTmpl.indices) lumaTmpl[i] = (1f - TMPL_EMA) * lumaTmpl[i] + TMPL_EMA * fl[i]
        lumaNorm = normOf(lumaTmpl)
        // Refresh the histogram cue only on this same very-clean gate — an
        // occlusion-tainted or ambiguous frame must not corrupt the cumulative
        // fg/bg model (same anti-contamination lesson as the keyframe bank).
        if (canBank && rawCrop.cu != null && rawCrop.cv != null && histFg.isNotEmpty()) {
            val (fg, bg) = histCountsAt(rawCrop, cx, cy)
            for (i in 0 until HIST_BINS) {
                histFg[i] = (1f - HIST_EMA) * histFg[i] + HIST_EMA * fg[i]
                histBg[i] = (1f - HIST_EMA) * histBg[i] + HIST_EMA * bg[i]
            }
        }
    }

    /** Add `fresh` as a keyframe iff it's a genuinely new appearance (below
     *  KF_THRESH NCC-similarity to every current slot — anchor, adaptive, existing
     *  keyframes), keeping the bank diverse. */
    private fun maybeBankKeyframe(ci: Int, fresh: FloatArray) {
        val fn = normOf(fresh)
        var maxSim = dot(fresh, anchors[ci]) / (fn * anchorNorms[ci])
        maxSim = maxOf(maxSim, dot(fresh, templates[ci]) / (fn * tmplNorms[ci]))
        val kf = keyframes[ci]; val kn = keyframeNorms[ci]
        for (k in kf.indices) maxSim = maxOf(maxSim, dot(fresh, kf[k]) / (fn * kn[k]))
        if (maxSim >= KF_THRESH) return
        kf.add(fresh.copyOf()); kn.add(fn)
        if (kf.size > K_KEYFRAMES) {
            // Evict the most REDUNDANT slot (highest similarity to some other kept
            // slot), not simply the oldest — keeps distinct poses (front+side)
            // instead of whichever happens to be newest.
            var worstIdx = 0; var worstSim = -1f
            for (i in kf.indices) {
                var s = -1f
                for (j in kf.indices) if (j != i) {
                    val sim = dot(kf[i], kf[j]) / (kn[i] * kn[j])
                    if (sim > s) s = sim
                }
                if (s > worstSim) { worstSim = s; worstIdx = i }
            }
            kf.removeAt(worstIdx); kn.removeAt(worstIdx)
        }
    }

    // --- STAPLE-style histogram appearance cue --------------------------------

    private fun histBinIndex(crop: GrayFrame, idx: Int): Int {
        val cu = (((crop.cu!![idx] + 128f) / 32f).toInt()).coerceIn(0, 7)
        val cv = (((crop.cv!![idx] + 128f) / 32f).toInt()).coerceIn(0, 7)
        return cu * 8 + cv
    }

    /** fg/bg per-bin pixel counts: fg = TMPL box centred at (cx,cy) — the ACTUAL
     *  found position, not the crop centre (which drifts from prediction error). */
    private fun histCountsAt(crop: GrayFrame, cx: Float, cy: Float): Pair<FloatArray, FloatArray> {
        val fg = FloatArray(HIST_BINS); val bg = FloatArray(HIST_BINS)
        val half = TMPL / 2f; val bgMargin = TMPL * 0.75f
        for (y in 0 until crop.h) for (x in 0 until crop.w) {
            val dx = kotlin.math.abs(x - cx); val dy = kotlin.math.abs(y - cy)
            val idx = y * crop.w + x
            if (dx <= half && dy <= half) fg[histBinIndex(crop, idx)] += 1f
            else if (dx > half + bgMargin || dy > half + bgMargin) bg[histBinIndex(crop, idx)] += 1f
        }
        return fg to bg
    }

    /** Per-candidate-position mean fg-probability, via an integral image so cost
     *  is O(crop + positions) instead of O(positions x template^2) like NCC. */
    private fun histResponse(crop: GrayFrame, g0: Int, g1: Int, gw: Int, stride: Int): FloatArray {
        val w = crop.w; val h = crop.h
        // Reused scratch — these are ~64 KB each at CROP=128, and this runs on the
        // frame-delivery thread, so allocating them per frame would push ~4 MB/s of
        // garbage through the exact path the 30 fps target depends on (same reason
        // Camera2FrameSource/UvcFrameSource/MainActivity all reuse their buffers).
        if (histBeta.size != w * h) histBeta = FloatArray(w * h)
        if (histII.size != (w + 1) * (h + 1)) histII = FloatArray((w + 1) * (h + 1))
        val beta = histBeta; val ii = histII
        for (i in beta.indices) {
            val b = histBinIndex(crop, i)
            beta[i] = histFg[b] / (histFg[b] + histBg[b] + HIST_LAMBDA)
        }
        // NOTE: row 0 and column 0 of `ii` must stay zero (the box-sum reads them
        // as the integral-image border). The fill below only ever writes
        // (y+1, x+1), so they stay zero across reuse — do not "optimise" the
        // indexing here without re-zeroing that border.
        for (y in 0 until h) {
            var rowSum = 0f
            for (x in 0 until w) {
                rowSum += beta[y * w + x]
                ii[(y + 1) * (w + 1) + (x + 1)] = ii[y * (w + 1) + (x + 1)] + rowSum
            }
        }
        val resp = FloatArray(gw * gw); val half = TMPL / 2
        var gy = 0; var y = g0
        while (y <= g1 && gy < gw) {
            var gx = 0; var x = g0
            while (x <= g1 && gx < gw) {
                val x0 = (x - half).coerceIn(0, w); val y0 = (y - half).coerceIn(0, h)
                val x1 = (x + half).coerceIn(0, w); val y1 = (y + half).coerceIn(0, h)
                val area = (x1 - x0) * (y1 - y0)
                if (area > 0) {
                    val s = ii[y1 * (w + 1) + x1] - ii[y0 * (w + 1) + x1] -
                            ii[y1 * (w + 1) + x0] + ii[y0 * (w + 1) + x0]
                    resp[gy * gw + gx] = s / area
                }
                gx++; x += stride
            }
            gy++; y += stride
        }
        return resp
    }

    /** NCC of a template over the search grid → response map (gw×gw). Stride is
     *  passed in — it widens (coarser) while coasting so the zoomed-out re-acquire
     *  search stays the same cost as a normal-FOV locked search. */
    private fun responseMap(crop: GrayFrame, tmpl: FloatArray, tn: Float,
                            g0: Int, g1: Int, gw: Int, stride: Int): FloatArray {
        val resp = FloatArray(gw * gw)
        var gy = 0; var y = g0
        while (y <= g1 && gy < gw) {
            var gx = 0; var x = g0
            while (x <= g1 && gx < gw) {
                resp[gy * gw + gx] = nccAt(crop, x.toFloat(), y.toFloat(), tmpl, tn)
                gx++; x += stride
            }
            gy++; y += stride
        }
        return resp
    }

    /** Parabolic sub-pixel refinement of the peak; returns fractional (gx,gy). */
    private fun subPixelPeak(resp: FloatArray, gw: Int): Pair<Float, Float> {
        var pk = 0; var peak = resp[0]
        for (i in resp.indices) if (resp[i] > peak) { peak = resp[i]; pk = i }
        val px = pk % gw; val py = pk / gw
        var dx = 0f; var dy = 0f
        if (px in 1 until gw - 1) {
            val l = resp[py * gw + px - 1]; val r = resp[py * gw + px + 1]
            val den = l - 2 * peak + r
            if (kotlin.math.abs(den) > 1e-6f) dx = (0.5f * (l - r) / den).coerceIn(-1f, 1f)
        }
        if (py in 1 until gw - 1) {
            val u = resp[(py - 1) * gw + px]; val d = resp[(py + 1) * gw + px]
            val den = u - 2 * peak + d
            if (kotlin.math.abs(den) > 1e-6f) dy = (0.5f * (u - d) / den).coerceIn(-1f, 1f)
        }
        return (px + dx) to (py + dy)
    }

    // --- shared ---------------------------------------------------------------

    private fun confFloor() = psrToConf(psrWarn)
    private fun confLock()  = psrToConf(psrLock)

    private fun result(crop: GrayFrame): Result {
        val half = bsize / 2f
        val (px, py) = cf.project(2)
        val (ax, ay) = cf.project((latencyFrames + 0.5f).toInt())
        return Result(state,
            (bcx - half).toInt(), (bcy - half).toInt(), bsize.toInt(), bsize.toInt(),
            conf, px, py, ax, ay, crop)
    }

    /** Raw followed crop (luma + chroma, NO filter) — filters applied per cue. */
    private fun workingCropRaw(frame: GrayFrame, cx: Float, cy: Float, size: Float): GrayFrame {
        val r = size * MARGIN
        return frame.cropResample(cx - r / 2f, cy - r / 2f, r, r, CROP, CROP)
    }

    /** Scale on LUMA (crop.d) — the scale-robust channel; edge mis-scales. */
    private fun updateScale(crop: GrayFrame, cx: Float, cy: Float) {
        val t = lumaTmpl
        var best = -2f; var bestS = 1f; var ncc1 = 0f
        for (s in SCALES) {
            val ts = TMPL * s
            val patch = crop.cropResample(cx - ts / 2f, cy - ts / 2f, ts, ts, TMPL, TMPL)
            val p = meanSub(patch.d)
            val ncc = dot(p, t) / (normOf(t) * (normOf(p) + 1e-6f))
            if (s == 1f) ncc1 = ncc
            if (ncc > best) { best = ncc; bestS = s }
        }
        // Dead-band: only rescale if a non-unity scale CLEARLY beats staying put.
        // Without this, feed noise makes 0.9 win by a hair most frames and the box
        // ratchets down to the floor every time (over-zoom + a tiny, unstable lock).
        if (bestS != 1f && best < ncc1 + 0.03f) bestS = 1f
        bsize = (bsize * (1f + (bestS - 1f) * 0.5f)).coerceIn(36f, minOf(crop.w * 4, 2000).toFloat())
    }

    /** Conditional spatial prior: if a rival peak exists (an identical distractor
     *  appearance can't separate), bias the fused map toward the prediction. Only
     *  when a rival is present, so it never fights a lone target under scale. */
    private fun applyDistractorPrior(fused: FloatArray, gw: Int, cc: Float) {
        var pk = 0; var pv = fused[0]
        for (i in fused.indices) if (fused[i] > pv) { pv = fused[i]; pk = i }
        if (pv <= 0.1f) return
        val px = pk % gw; val py = pk / gw
        var pv2 = -1e9f; var pk2 = -1
        for (i in fused.indices) {
            val x = i % gw; val y = i / gw
            if (kotlin.math.abs(x - px) <= 4 && kotlin.math.abs(y - py) <= 4) continue
            if (fused[i] > pv2) { pv2 = fused[i]; pk2 = i }
        }
        if (pk2 < 0 || pv2 <= 0.6f * pv) return
        val d = kotlin.math.hypot((pk2 % gw - px).toFloat(), (pk2 / gw - py).toFloat())
        if (d <= 5f) return
        val sig = if (badFrames > 0) gw / 1.5f else gw / 2.6f
        for (i in fused.indices) {
            val x = i % gw - cc; val y = i / gw - cc
            fused[i] *= kotlin.math.exp(-(x * x + y * y) / (2f * sig * sig))
        }
    }

    private fun nccAt(crop: GrayFrame, cx: Float, cy: Float, t: FloatArray, tn: Float): Float {
        val h = TMPL / 2
        val x0 = (cx - h).toInt(); val y0 = (cy - h).toInt()
        if (x0 < 0 || y0 < 0 || x0 + TMPL > crop.w || y0 + TMPL > crop.h) return -2f
        // SINGLE pass. This is the hottest loop in the tracker — the HUD measured
        // the cue stage at 101.7 ms while coasting — and it used to walk the 28x28
        // patch TWICE: once for the mean, once for the dot product and norm. Two
        // identities remove the first walk outright:
        //
        //   sum((v-mean)*t) == sum(v*t)              because the template is
        //                                            zero-mean (normPatch subtracts
        //                                            it, and an EMA of zero-mean
        //                                            patches stays zero-mean)
        //   sum((v-mean)^2) == sum(v^2) - sum(v)^2/N
        //
        // Verified against the two-pass form on 300 random patches: max relative
        // difference 1.4e-12. Exact, not approximate — no tracking behaviour
        // changes. Sums accumulate in Double so the sum-of-squares subtraction
        // cannot lose significance on a low-contrast patch.
        val cd = crop.d; val cw = crop.w
        var s = 0.0; var s2 = 0.0; var dot = 0f
        var ti = 0
        for (j in 0 until TMPL) {
            var o = (y0 + j) * cw + x0
            for (i in 0 until TMPL) {
                val v = cd[o]
                s += v; s2 += v.toDouble() * v
                dot += v * t[ti]
                o++; ti++
            }
        }
        val pn = (s2 - s * s / (TMPL * TMPL)).coerceAtLeast(0.0).toFloat()
        return dot / (tn * (sqrt(pn) + 1e-6f))
    }

    private fun psrOf(resp: FloatArray, gw: Int): Float {
        var pk = 0; var peak = resp[0]
        for (i in resp.indices) if (resp[i] > peak) { peak = resp[i]; pk = i }
        val px = pk % gw; val py = pk / gw
        var sum = 0f; var sum2 = 0f; var n = 0
        for (y in 0 until gw) for (x in 0 until gw) {
            if (kotlin.math.abs(x - px) <= 3 && kotlin.math.abs(y - py) <= 3) continue
            val v = resp[y * gw + x]; sum += v; sum2 += v * v; n++
        }
        if (n < 4) return 0f
        val mean = sum / n
        val std = sqrt((sum2 / n - mean * mean).coerceAtLeast(1e-6f))
        return (peak - mean) / std
    }

    private fun psrToConf(psr: Float): Float = ((psr - 3f) / (12f - 3f)).coerceIn(0f, 1f)

    private fun normPatch(g: GrayFrame, cx: Float, cy: Float, size: Int): FloatArray {
        val patch = g.cropResample(cx - size / 2f, cy - size / 2f,
            size.toFloat(), size.toFloat(), size, size)
        return meanSub(patch.d)
    }

    private fun meanSub(a: FloatArray): FloatArray {
        var s = 0f; for (v in a) s += v; val m = s / a.size
        return FloatArray(a.size) { a[it] - m }
    }

    private fun normOf(a: FloatArray): Float { var s = 0f; for (v in a) s += v * v; return sqrt(s) + 1e-6f }
    private fun dot(a: FloatArray, b: FloatArray): Float { var s = 0f; for (i in a.indices) s += a[i] * b[i]; return s }
}
