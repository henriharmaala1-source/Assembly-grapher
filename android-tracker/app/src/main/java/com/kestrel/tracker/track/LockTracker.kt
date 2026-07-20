package com.kestrel.tracker.track

import kotlin.math.sqrt

/**
 * Lean lock-on tracker built to the "TFL1" architecture we reverse-engineered
 * from the reference terminal-guidance footage — the design that gets solid
 * lock on a low-Hz, low-contrast analog/thermal feed WITHOUT a gimbal or a
 * heavy model:
 *
 *   • A followed CROP around the predicted target is resampled to a fixed
 *     working size every frame. Because the crop is sized from the tracked box
 *     (boxSize × MARGIN), the target stays a ~constant size inside it whatever
 *     the range — the "zoom window that never changes size" behaviour, and what
 *     makes correlation stable from 50 m to 800 m.
 *   • NCC template matching inside that crop (target large + centred → sharp,
 *     trustworthy peak), with an appearance filter applied to template AND crop.
 *   • PSR (peak-to-sidelobe ratio) as the per-frame confidence — the cheap,
 *     reliable health signal a boolean tracker flag never gives you.
 *   • Coarse multi-scale check to follow the target's apparent-size change
 *     (= range), and an alpha-beta centre filter to predict + coast.
 *
 * Pure Kotlin on GrayFrame — no Android deps — so it unit-tests here and ports
 * straight to the onboard C++ lock tracker.
 */
class LockTracker {

    enum class State { IDLE, LOCKED, COASTING, LOST }

    data class Result(
        val state: State,
        val x: Int, val y: Int, val w: Int, val h: Int,   // box in full-frame px
        val conf: Float,                                  // 0..1 from PSR
        val predX: Float, val predY: Float,               // predicted centre
        val crop: GrayFrame?,                             // the working crop (for the PiP)
    )

    // --- tunables (exposed; MainActivity wires the filter + thresholds) ------
    var filter: CropFilter = CropFilter.NONE
    var psrLock = 5.5f
    var psrWarn = 3.8f

    private val CROP = 128           // working-crop size (px)
    private val TMPL = 40            // template size (px, in the crop)
    private val MARGIN = 2.2f        // crop region = boxSize * MARGIN
    private val SEARCH = 30          // half search window in the crop (px)
    private val STRIDE = 2
    private val SCALES = floatArrayOf(0.9f, 1.0f, 1.11f)
    private val LOSS_TIMEOUT = 20
    private val TMPL_EMA = 0.08f

    private var tmpl: FloatArray? = null    // mean-subtracted template
    private var tmplNorm = 0f
    private var bcx = 0f; private var bcy = 0f      // box centre, full-frame
    private var bsize = 0f                          // box size (px), full-frame
    private val cf = CenterFilter()
    private var badFrames = 0
    var state = State.IDLE; private set
    var conf = 0f; private set

    val hasTarget get() = state == State.LOCKED || state == State.COASTING

    fun reset() { tmpl = null; state = State.IDLE; badFrames = 0; conf = 0f }

    /** Designate a target at (px,py) in full-frame coords, initial size `size`. */
    fun designate(frame: GrayFrame, px: Float, py: Float, size: Float = 64f) {
        bcx = px; bcy = py
        bsize = size.coerceIn(12f, minOf(frame.w, frame.h).toFloat())
        val crop = workingCrop(frame, bcx, bcy, bsize)
        tmpl = normPatch(crop, CROP / 2f, CROP / 2f, TMPL).also { tmplNorm = normOf(it) }
        cf.start(px, py)
        badFrames = 0; conf = 1f; state = State.LOCKED
    }

    fun update(frame: GrayFrame): Result {
        val t = tmpl ?: return Result(State.IDLE, 0, 0, 0, 0, 0f, 0f, 0f, null)

        // Predict where the target is, and follow the crop there.
        val (pcx, pcy) = cf.predict()
        val crop = workingCrop(frame, pcx, pcy, bsize)

        // NCC response map over the search window (target should be near centre).
        val c0 = CROP / 2
        val g0 = c0 - SEARCH; val g1 = c0 + SEARCH
        val gw = (g1 - g0) / STRIDE + 1
        val resp = FloatArray(gw * gw)
        var peak = -2f; var pxi = c0; var pyi = c0
        var gy = 0
        var y = g0
        while (y <= g1) {
            var gx = 0; var x = g0
            while (x <= g1) {
                val ncc = nccAt(crop, x.toFloat(), y.toFloat(), t, tmplNorm)
                resp[gy * gw + gx] = ncc
                if (ncc > peak) { peak = ncc; pxi = x; pyi = y }
                gx++; x += STRIDE
            }
            gy++; y += STRIDE
        }
        conf = psrConf(resp, gw, gw, peak)

        if (peak >= 0f && conf >= confFloor()) {
            // Map the crop peak back to full-frame coords.
            val rw = bsize * MARGIN
            val nx = pcx + (pxi / CROP.toFloat() - 0.5f) * rw
            val ny = pcy + (pyi / CROP.toFloat() - 0.5f) * rw
            cf.correct(nx, ny)
            bcx = cf.x; bcy = cf.y
            updateScale(crop, pxi.toFloat(), pyi.toFloat(), t)
            if (conf >= confLock()) {
                adaptTemplate(crop, pxi.toFloat(), pyi.toFloat())
                state = State.LOCKED
            } else state = State.LOCKED
            badFrames = 0
        } else {
            // Lost the peak: coast on prediction, widen tolerance, count down.
            bcx = pcx; bcy = pcy
            badFrames++
            state = if (badFrames >= LOSS_TIMEOUT) State.LOST else State.COASTING
            if (state == State.LOST) { tmpl = null; return result(frame, crop) }
        }
        return result(frame, crop)
    }

    // --- helpers -------------------------------------------------------------

    private fun confFloor() = psrToConf(psrWarn)
    private fun confLock()  = psrToConf(psrLock)

    private fun result(frame: GrayFrame, crop: GrayFrame): Result {
        val half = bsize / 2f
        val (px, py) = cf.project(6)
        return Result(state,
            (bcx - half).toInt(), (bcy - half).toInt(), bsize.toInt(), bsize.toInt(),
            conf, px, py, crop)
    }

    /** Extract the followed crop (boxSize×MARGIN region) at fixed working size,
     *  filtered. This is the scale-normalising step — the target's size in the
     *  crop stays roughly constant as bsize tracks its apparent size. */
    private fun workingCrop(frame: GrayFrame, cx: Float, cy: Float, size: Float): GrayFrame {
        val r = size * MARGIN
        val raw = frame.cropResample(cx - r / 2f, cy - r / 2f, r, r, CROP, CROP)
        return Filters.apply(raw, filter)
    }

    private fun updateScale(crop: GrayFrame, cx: Float, cy: Float, t: FloatArray) {
        var best = -2f; var bestS = 1f
        for (s in SCALES) {
            val ts = TMPL * s
            val patch = crop.cropResample(cx - ts / 2f, cy - ts / 2f, ts, ts, TMPL, TMPL)
            val p = meanSub(patch.d)
            val ncc = dot(p, t) / (normOf(t) * (normOf(p) + 1e-6f))
            if (ncc > best) { best = ncc; bestS = s }
        }
        // Damp the size update so a single noisy frame can't jump the scale.
        bsize = (bsize * (1f + (bestS - 1f) * 0.5f))
            .coerceIn(12f, minOf(crop.w * 4, 2000).toFloat())
    }

    private fun adaptTemplate(crop: GrayFrame, cx: Float, cy: Float) {
        val fresh = normPatch(crop, cx, cy, TMPL)
        val cur = tmpl ?: return
        for (i in cur.indices) cur[i] = (1f - TMPL_EMA) * cur[i] + TMPL_EMA * fresh[i]
        tmplNorm = normOf(cur)
    }

    /** NCC of the template against a TMPL patch centred at (cx,cy) in the crop. */
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

    /** Peak-to-sidelobe ratio → confidence in [0,1]. Sidelobe = response map
     *  outside an exclusion window around the peak. This is the health signal. */
    private fun psrConf(resp: FloatArray, w: Int, h: Int, peak: Float): Float {
        var pk = 0
        for (i in resp.indices) if (resp[i] == peak) { pk = i; break }
        val px = pk % w; val py = pk / w
        val excl = 3
        var sum = 0f; var sum2 = 0f; var n = 0
        for (y in 0 until h) for (x in 0 until w) {
            if (kotlin.math.abs(x - px) <= excl && kotlin.math.abs(y - py) <= excl) continue
            val v = resp[y * w + x]; sum += v; sum2 += v * v; n++
        }
        if (n < 4) return 0f
        val mean = sum / n
        val std = sqrt((sum2 / n - mean * mean).coerceAtLeast(1e-6f))
        val psr = (peak - mean) / std
        return psrToConf(psr)
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
