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

    enum class State { IDLE, LOCKED, COASTING, LOST }

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

    private var templates: Array<FloatArray> = arrayOf()
    private var tmplNorms: FloatArray = floatArrayOf()
    private var anchors: Array<FloatArray> = arrayOf()    // original views, fixed — anti-drift
    private var anchorNorms: FloatArray = floatArrayOf()
    private var lumaTmpl: FloatArray = floatArrayOf()   // dedicated luma template for scale
    private var lumaNorm = 0f
    private var lastRawCrop: GrayFrame? = null      // for rebuilding templates on cue change
    private var bcx = 0f; private var bcy = 0f
    private var bsize = 0f
    private val cf = CenterFilter()
    private var badFrames = 0
    var state = State.IDLE; private set
    var conf = 0f; private set

    val hasTarget get() = state == State.LOCKED || state == State.COASTING

    fun reset() { templates = arrayOf(); tmplNorms = floatArrayOf()
                  anchors = arrayOf(); anchorNorms = floatArrayOf()
                  lumaTmpl = floatArrayOf(); lumaNorm = 0f; lastRawCrop = null
                  state = State.IDLE; badFrames = 0; conf = 0f }

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
        badFrames = 0; conf = 1f; state = State.LOCKED
    }

    fun update(frame: GrayFrame): Result {
        if (templates.isEmpty()) return Result(State.IDLE, 0,0,0,0, 0f, 0f,0f, 0f,0f, null)

        val (pcx, pcy) = cf.predict()

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
        val crop = frame.cropResample(pcx - regionW / 2f, pcy - regionW / 2f,
                                      regionW, regionW, cropPix, cropPix)

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
            val cueCrop = Filters.apply(crop, cues[ci])
            // Match against BOTH the adaptive template and the fixed anchor; take
            // the better per position — the anchor re-anchors when the adaptive
            // has drifted, extending how long the lock survives.
            val respA = responseMap(cueCrop, templates[ci], tmplNorms[ci], g0, g1, gw, strideEff)
            val respB = responseMap(cueCrop, anchors[ci], anchorNorms[ci], g0, g1, gw, strideEff)
            val resp = FloatArray(respA.size) { if (respA[it] > respB[it]) respA[it] else respB[it] }
            // Prediction-proximity: down-weight a cue whose peak drifts off-centre
            // (a distractor lock, or a confidently-wrong edge under scale — PSR
            // alone can't catch a sharp-but-wrong peak).
            var pk = 0; var pv = resp[0]
            for (i in resp.indices) if (resp[i] > pv) { pv = resp[i]; pk = i }
            val dxp = pk % gw - cc; val dyp = pk / gw - cc
            val prox = kotlin.math.exp(-(dxp * dxp + dyp * dyp) / (2f * sigP * sigP))
            val w = (psrOf(resp, gw) - 3f).coerceAtLeast(0f) * prox
            if (w <= 0f) continue
            anyWeight += w
            for (i in resp.indices) fused[i] += w * resp[i]
        }
        if (anyWeight > 0f) applyDistractorPrior(fused, gw, cc)
        conf = if (anyWeight > 0f) psrToConf(psrOf(fused, gw)) else 0f

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
            updateScale(crop, cxCrop, cyCrop)
            if (conf >= confLock()) adaptTemplates(crop, cxCrop, cyCrop)
            state = State.LOCKED; badFrames = 0
        } else {
            cf.decay(0.6f)                 // coast decelerates instead of flying off
            bcx = pcx; bcy = pcy
            badFrames++
            state = if (badFrames >= LOSS_TIMEOUT) State.LOST else State.COASTING
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
        // Dedicated luma template — scale estimation runs on luma (the
        // scale-robust channel); edge blurs under downsample and mis-scales.
        lumaTmpl = normPatch(rawCrop, CROP / 2f, CROP / 2f, TMPL)   // rawCrop.d = luma
        lumaNorm = normOf(lumaTmpl)
    }

    private fun adaptTemplates(rawCrop: GrayFrame, cx: Float, cy: Float) {
        for (ci in cues.indices) {
            val fresh = normPatch(Filters.apply(rawCrop, cues[ci]), cx, cy, TMPL)
            val cur = templates[ci]
            for (i in cur.indices) cur[i] = (1f - TMPL_EMA) * cur[i] + TMPL_EMA * fresh[i]
            tmplNorms[ci] = normOf(cur)
        }
        val fl = normPatch(rawCrop, cx, cy, TMPL)
        for (i in lumaTmpl.indices) lumaTmpl[i] = (1f - TMPL_EMA) * lumaTmpl[i] + TMPL_EMA * fl[i]
        lumaNorm = normOf(lumaTmpl)
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
        var sum = 0f
        for (j in 0 until TMPL) for (i in 0 until TMPL) sum += crop.d[(y0 + j) * crop.w + (x0 + i)]
        val mean = sum / (TMPL * TMPL)
        var dot = 0f; var pn = 0f
        for (j in 0 until TMPL) for (i in 0 until TMPL) {
            val v = crop.d[(y0 + j) * crop.w + (x0 + i)] - mean
            dot += v * t[j * TMPL + i]; pn += v * v
        }
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
