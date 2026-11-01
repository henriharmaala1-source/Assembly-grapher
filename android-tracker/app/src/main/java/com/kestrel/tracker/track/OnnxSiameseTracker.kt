package com.kestrel.tracker.track

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import java.nio.FloatBuffer
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sqrt

/**
 * Learned single-object tracker over an ONNX Siamese / one-stream graph.
 *
 * This is a line-for-line port of `desktop/siamese_onnx.py`, which was measured
 * against OpenCV's own TrackerVit on the 10-clip battery and tracks equivalently
 * (85% mean on-target vs 83%). Every constant here was recovered from OpenCV's
 * tracker_vit.cpp and then checked by that measurement rather than by reading —
 * see the Python module's header for exactly how far the verification goes and
 * where it stops.
 *
 * WHY THIS EXISTS ALONGSIDE LockTracker, not instead of it
 * --------------------------------------------------------
 * On the battery the two fail on disjoint scenarios, and the split is sharp:
 *
 *   learned wins   fast motion (>~8 px/frame), camera pan/shake, motion blur,
 *                  target rotation. f_maneuver 19% -> 100%.
 *   classical wins occlusion (79% vs 39%) and LOW CONTRAST (a 14 px target at
 *                  contrast 26: 95% vs 58%). The pixel tracker's contrast floor
 *                  is LOWER than the network's.
 *
 * The best measured configuration is neither alone: let the network lead and
 * fall back to LockTracker on frames where the network declines to answer. That
 * scored 93% against 70% classical and 83% learned — equal to a per-clip oracle.
 * Gating on the network's own score works where gating on measured speed does
 * NOT: speed read from the selected tracker's output is circular, because when
 * the pixel tracker loses a fast target its output stops moving fast and the
 * signal to switch away from it disappears exactly when it is needed (measured:
 * 68%, worse than either half alone).
 *
 * SWAPPING IN LightFC
 * -------------------
 * Replace the model file and adjust [decode] for its head layout. The crop,
 * normalisation and windowing below are the shared Siamese scaffolding and
 * should not need to change.
 *
 * Dependency: com.microsoft.onnxruntime:onnxruntime-android
 */
class OnnxSiameseTracker(
    modelBytes: ByteArray,
    private val scoreThreshold: Float = 0.20f,
    /**
     * Hard cap on the per-frame box size ratio. 1.05 = 5% per frame.
     *
     * The box-regression head is unconstrained and its output feeds the NEXT
     * crop, so an over-estimate enlarges the crop, which enlarges the next
     * estimate — positive feedback with nothing opposing it. Observed on real
     * footage: the box inflates until it covers most of the frame while the
     * tracker still reports LOCKED, because "LOCKED" here only means score >
     * 0.20 and says nothing about the box.
     *
     * Measured on the battery: 85% undamped -> 88% with this cap (c_lowcontrast
     * 84 -> 100, i_worst 70 -> 87). An EMA on the size was also tried and is
     * NOT used — it gives a nicer median box but is too slow for genuine scale
     * change, collapsing z_below_floor from 54% to 13%.
     *
     * 0 disables the cap.
     */
    private val maxScaleStep: Float = 1.05f,
    /**
     * Absolute bound on box size as a multiple of the DESIGNATED size.
     *
     * The rate cap above is not sufficient, for a structural reason. The search
     * crop is 4*sqrt(w*h) and the head predicts w RELATIVE TO THAT CROP, so the
     * fixed point is exactly w_pred = 1/SEARCH_FACTOR = 0.25. Any systematic
     * over-prediction above that is exponential feedback, and a per-frame cap
     * only sets how FAST it diverges. Observed on real footage WITH the 5% cap
     * already in place: a 48 px designation reached about 8x, which is ~43
     * frames compounding at 1.05.
     *
     * 0.25-4x is wide enough for genuine range change and costs nothing on the
     * battery (88% with it, 88% without, worst box 2.5x so it never binds
     * there). It converts an unbounded divergence into a bounded error.
     *
     * Fixing the size outright (bounds 1.0-1.0) was also measured: 84% against
     * 88%, losing z_below_floor 54% -> 13%. So the head IS worth having; it just
     * has to be fenced.
     */
    private val scaleBounds: ClosedFloatingPointRange<Float> = 0.25f..4.0f,
) {
    data class Box(val x: Int, val y: Int, val w: Int, val h: Int)

    private val env: OrtEnvironment = OrtEnvironment.getEnvironment()
    private val session: OrtSession =
        env.createSession(modelBytes, OrtSession.SessionOptions().apply {
            // The camera thread is the one that matters; leave the rest of the
            // cores for LockTracker, which runs on the same frame.
            setIntraOpNumThreads(2)
        })

    /** The template tensor, built once at designate time.
     *
     *  Unlike OpenCV's dnn::Net — which retains a named input across forward()
     *  calls — ONNX Runtime has no persistent input state: every run() must be
     *  handed BOTH inputs. So the template tensor is kept alive here for the
     *  life of the lock. This is the single most likely thing to get wrong when
     *  porting from the OpenCV version, because omitting it does not crash; it
     *  silently tracks against a zero template. */
    private var template: OnnxTensor? = null

    private var rect = Box(0, 0, 0, 0)
    private var initSize = 0f
    var trackingScore = 0f; private set
    val hasTarget get() = template != null

    private val hann = FloatArray(SCORE * SCORE).also { w ->
        // Centred Hann: note the (SCORE + 1) denominator and (i + 1) numerator —
        // neither endpoint is zero.
        val r = FloatArray(SCORE) { i ->
            0.5f * (1f - cos((2.0 * Math.PI / (SCORE + 1)) * (i + 1)).toFloat())
        }
        for (y in 0 until SCORE) for (x in 0 until SCORE) w[y * SCORE + x] = r[y] * r[x]
    }

    // Reused so the camera thread never allocates per frame — the same reason
    // LockTracker and MainActivity reuse theirs.
    private val searchBuf = FloatArray(3 * SEARCH_SIZE * SEARCH_SIZE)
    private val templBuf = FloatArray(3 * TEMPLATE_SIZE * TEMPLATE_SIZE)

    fun reset() {
        template?.close(); template = null; trackingScore = 0f
    }

    fun designate(nv21: ByteArray, fw: Int, fh: Int, cx: Float, cy: Float, size: Float) {
        val s = size.toInt().coerceAtLeast(2)
        rect = Box((cx - size / 2).toInt(), (cy - size / 2).toInt(), s, s)
        initSize = sqrt((s * s).toFloat())
        val cropSz = cropSize(rect, TEMPLATE_FACTOR)
        sampleCrop(nv21, fw, fh, rect, cropSz, TEMPLATE_SIZE, templBuf)
        template?.close()
        template = OnnxTensor.createTensor(
            env, FloatBuffer.wrap(templBuf),
            longArrayOf(1, 3, TEMPLATE_SIZE.toLong(), TEMPLATE_SIZE.toLong()))
    }

    /** @return the new box, or null when the network declines to answer — which
     *  is the signal the caller should fall back to the classical tracker. */
    fun update(nv21: ByteArray, fw: Int, fh: Int): Box? {
        val tmpl = template ?: return null
        val cropSz = cropSize(rect, SEARCH_FACTOR)
        sampleCrop(nv21, fw, fh, rect, cropSz, SEARCH_SIZE, searchBuf)

        val search = OnnxTensor.createTensor(
            env, FloatBuffer.wrap(searchBuf),
            longArrayOf(1, 3, SEARCH_SIZE.toLong(), SEARCH_SIZE.toLong()))
        search.use {
            session.run(mapOf("template" to tmpl, "search" to it),
                        setOf("output1", "output2", "output3")).use { res ->
                @Suppress("UNCHECKED_CAST")
                val conf = res.get(0).value as Array<Array<Array<FloatArray>>>
                @Suppress("UNCHECKED_CAST")
                val size = res.get(1).value as Array<Array<Array<FloatArray>>>
                @Suppress("UNCHECKED_CAST")
                val off = res.get(2).value as Array<Array<Array<FloatArray>>>
                return decode(conf[0][0], size[0], off[0], cropSz)
            }
        }
    }

    private fun decode(conf: Array<FloatArray>, size: Array<Array<FloatArray>>,
                       off: Array<Array<FloatArray>>, cropSz: Int): Box? {
        // The Hann window suppresses peaks at the edge of the search region: a
        // response that far from the prediction is far more likely a distractor
        // than the target having moved most of a crop in one frame.
        var best = -Float.MAX_VALUE; var px = 0; var py = 0
        for (y in 0 until SCORE) for (x in 0 until SCORE) {
            val v = conf[y][x] * hann[y * SCORE + x]
            if (v > best) { best = v; px = x; py = y }
        }
        trackingScore = best
        if (best < scoreThreshold) return null

        val cx = (px + off[0][py][px]) / SCORE
        val cy = (py + off[1][py][px]) / SCORE
        val w = size[0][py][px]
        val h = size[1][py][px]
        // NOTE: x0/y0 come from the box BEFORE this update, matching the C++.
        val x0 = rect.x + truncDiv2(rect.w - cropSz)
        val y0 = rect.y + truncDiv2(rect.h - cropSz)
        var nw = w * cropSz
        var nh = h * cropSz
        if (maxScaleStep > 1f && rect.w > 0 && rect.h > 0) {
            nw = nw.coerceIn(rect.w / maxScaleStep, rect.w * maxScaleStep)
            nh = nh.coerceIn(rect.h / maxScaleStep, rect.h * maxScaleStep)
        }
        if (initSize > 0f) {
            val lo = scaleBounds.start * initSize
            val hi = scaleBounds.endInclusive * initSize
            nw = nw.coerceIn(lo, hi); nh = nh.coerceIn(lo, hi)
        }
        // Centre from the decoded position and the ACCEPTED size, so clamping
        // the size cannot shift the box off the peak the network actually found.
        val ccx = cx * cropSz + x0
        val ccy = cy * cropSz + y0
        rect = Box(floor(ccx - nw / 2).toInt(), floor(ccy - nh / 2).toInt(),
                   nw.toInt().coerceAtLeast(2), nh.toInt().coerceAtLeast(2))
        return rect
    }

    private fun cropSize(r: Box, factor: Int): Int =
        ceil(sqrt(r.w.toDouble() * r.h.toDouble()) * factor).toInt().coerceAtLeast(1)

    /**
     * Crop a square region around [r] and resample it straight into a normalised
     * NCHW float buffer — one pass, no intermediate bitmap.
     *
     * Reading directly out of NV21 at the OUTPUT grid is deliberate: the
     * alternative (crop to an RGB bitmap, then resize) touches every source
     * pixel of a crop that can be 4x the box, whereas this touches exactly
     * out*out of them. On the camera thread that difference is the whole point.
     *
     * Out-of-frame samples are clamped to black, matching the zero-padded
     * BORDER_CONSTANT in the reference.
     */
    private fun sampleCrop(nv21: ByteArray, fw: Int, fh: Int, r: Box,
                           cropSz: Int, out: Int, dst: FloatArray) {
        val x1 = r.x + truncDiv2(r.w - cropSz)
        val y1 = r.y + truncDiv2(r.h - cropSz)
        val step = cropSz.toFloat() / out
        val plane = out * out
        val uvBase = fw * fh
        for (j in 0 until out) {
            val sy = (y1 + (j + 0.5f) * step).toInt()
            for (i in 0 until out) {
                val sx = (x1 + (i + 0.5f) * step).toInt()
                var rr = 0f; var gg = 0f; var bb = 0f
                if (sx in 0 until fw && sy in 0 until fh) {
                    val y = (nv21[sy * fw + sx].toInt() and 0xFF).toFloat()
                    val uvi = uvBase + (sy shr 1) * fw + (sx and 1.inv())
                    val v = ((nv21[uvi].toInt() and 0xFF) - 128).toFloat()
                    val u = ((nv21[uvi + 1].toInt() and 0xFF) - 128).toFloat()
                    rr = y + 1.402f * v
                    gg = y - 0.344f * u - 0.714f * v
                    bb = y + 1.772f * u
                }
                // Channel order and statistics copied from the reference: OpenCV
                // feeds BGR and applies the mean/std Scalars positionally, so the
                // value nominally belonging to R lands on B. That is what the
                // model was calibrated against, so it is reproduced rather than
                // "corrected".
                val o = j * out + i
                dst[o] = (bb.coerceIn(0f, 255f) - MEAN0) / STD0
                dst[plane + o] = (gg.coerceIn(0f, 255f) - MEAN1) / STD1
                dst[2 * plane + o] = (rr.coerceIn(0f, 255f) - MEAN2) / STD2
            }
        }
    }

    companion object {
        const val TEMPLATE_SIZE = 128
        const val SEARCH_SIZE = 256
        const val SCORE = 16
        const val TEMPLATE_FACTOR = 2
        const val SEARCH_FACTOR = 4

        private const val MEAN0 = 0.485f * 255f
        private const val MEAN1 = 0.456f * 255f
        private const val MEAN2 = 0.406f * 255f
        private const val STD0 = 0.229f * 255f
        private const val STD1 = 0.224f * 255f
        private const val STD2 = 0.225f * 255f

        /** C++ integer division truncates TOWARD ZERO; Kotlin's `/` on Int does
         *  too, but this is spelled out because the Python reference had to use
         *  int(a/2) instead of //, and the two must stay in step. */
        fun truncDiv2(a: Int): Int = a / 2
    }
}
