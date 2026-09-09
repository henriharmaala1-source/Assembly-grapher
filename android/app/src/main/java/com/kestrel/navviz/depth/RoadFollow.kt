package com.kestrel.navviz.depth

import android.graphics.Bitmap
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * Kotlin port of the onboard appearance-based road follower
 * (onboard/src/road_follow.cpp). Same idea: self-supervise a road-colour model
 * in CIELab from a bottom-centre "we're on the road now" seed ROI (shadow
 * tolerant), score every pixel against it, keep the road-coloured mass, read a
 * per-row centreline, and derive a lateral offset + near->far heading +
 * confidence. Runs on a small downscaled frame, pure Kotlin (no OpenCV).
 *
 * Faithful to the algorithm, NOT the exact C++ code — and one deliberate
 * simplification: the C++ isolates the road via connectedComponents; here the
 * per-row mass-weighted centroid of the thresholded score stands in, which is
 * cheaper and adequate for a single dominant road but less robust to stray
 * road-coloured patches. Flagged so it isn't mistaken for the real module.
 */
class RoadFollow {

    data class Result(
        val valid: Boolean,
        val offset: Float,      // lateral offset of the near road [-1,1] (L..R)
        val heading: Float,     // near->far bend [-1,1]
        val conf: Float,        // [0,1]
        val centerline: FloatArray, // per-row centre x in [0,1], or -1 = no road
        val w: Int, val h: Int,
    )

    private val W = 128
    private val H = 96
    private val modelEma = 0.10f
    private val outEma = 0.40f
    private val featW = floatArrayOf(1.0f, 1.0f, 0.3f, 0.5f) // a, b, L, texture

    private var modelInit = false
    private val mu = FloatArray(4)
    private val sg = floatArrayOf(1f, 1f, 1f, 1f)
    private var prevOffset = 0f
    private var prevHeading = 0f
    private var haveOutput = false

    /** Reset the self-supervised road model (e.g. operator says "re-learn here"). */
    fun relearn() { modelInit = false; haveOutput = false }

    fun analyze(bitmap: Bitmap): Result {
        val scaled = Bitmap.createScaledBitmap(bitmap, W, H, true)
        val px = IntArray(W * H); scaled.getPixels(px, 0, W, 0, 0, W, H)

        // Feature planes: [a, b, L, texture] (CIELab + horizontal L-Laplacian).
        val fa = FloatArray(W * H); val fb = FloatArray(W * H)
        val fl = FloatArray(W * H); val ft = FloatArray(W * H)
        for (i in px.indices) {
            val p = px[i]
            val lab = rgbToLab((p shr 16) and 0xFF, (p shr 8) and 0xFF, p and 0xFF)
            fl[i] = lab[0]; fa[i] = lab[1]; fb[i] = lab[2]
        }
        for (y in 0 until H) for (x in 1 until W - 1) {
            val i = y * W + x
            ft[i] = abs(fl[i - 1] - 2f * fl[i] + fl[i + 1])   // texture energy
        }
        val planes = arrayOf(fa, fb, fl, ft)

        // Self-sample the road model from the bottom-centre seed ROI.
        val sx0 = W / 4; val sx1 = W * 3 / 4; val sy0 = H * 3 / 4
        for (c in 0 until 4) {
            var sum = 0f; var n = 0
            for (y in sy0 until H) for (x in sx0 until sx1) { sum += planes[c][y * W + x]; n++ }
            val m = sum / n
            var vs = 0f
            for (y in sy0 until H) for (x in sx0 until sx1) { val d = planes[c][y * W + x] - m; vs += d * d }
            val s = maxOf(sqrt(vs / n), 4.0f)   // std floor avoids div-by-0
            if (!modelInit) { mu[c] = m; sg[c] = s }
            else { mu[c] = (1 - modelEma) * mu[c] + modelEma * m
                   sg[c] = (1 - modelEma) * sg[c] + modelEma * s }
        }
        modelInit = true

        // Road-likeness score = exp(-0.5 * weighted Mahalanobis-lite).
        val score = FloatArray(W * H)
        for (i in 0 until W * H) {
            var d2 = 0f
            for (c in 0 until 4) { val z = (planes[c][i] - mu[c]) / sg[c]; d2 += z * z * featW[c] }
            score[i] = exp(-0.5f * d2)
        }

        // Per-row mass-weighted centreline over score>=0.30.
        val centerline = FloatArray(H) { -1f }
        var covered = 0
        for (y in 0 until H) {
            var mass = 0f; var num = 0f
            for (x in 0 until W) { val v = score[y * W + x]; if (v >= 0.30f) { mass += v; num += v * x } }
            if (mass >= 6f) { centerline[y] = (num / mass) / W; covered++ }
        }

        if (covered < 8) {
            haveOutput = false
            return Result(false, 0f, 0f, 0f, centerline, W, H)
        }

        // Near (bottom) vs far (top) bands from the covered rows.
        val rows = (0 until H).filter { centerline[it] >= 0f }
        val k = maxOf(2, rows.size / 4)
        var nearCx = 0f; var farCx = 0f
        for (j in 0 until k) {
            farCx += centerline[rows[j]] * W                  // top = far
            nearCx += centerline[rows[rows.size - 1 - j]] * W  // bottom = near
        }
        nearCx /= k; farCx /= k
        val half = W / 2f
        var offset = ((nearCx - half) / half).coerceIn(-1f, 1f)
        var heading = ((farCx - nearCx) / half).coerceIn(-1f, 1f)

        // Confidence = coverage * separability * straightness * temporal.
        val coverage = (covered / (H * 0.6f)).coerceIn(0f, 1f)
        var inRoad = 0f; var inN = 0; var edge = 0f; var edgeN = 0
        val ew = W / 6
        for (y in 0 until H) for (x in 0 until W) {
            val v = score[y * W + x]
            if (centerline[y] >= 0f) { inRoad += v; inN++ }
            if (x < ew || x >= W - ew) { edge += v; edgeN++ }
        }
        val separability = (((if (inN > 0) inRoad / inN else 0f)) - (if (edgeN > 0) edge / edgeN else 0f)).coerceIn(0f, 1f)
        var mean = 0f; for (y in rows) mean += centerline[y] * W; mean /= rows.size
        var varc = 0f; for (y in rows) { val d = centerline[y] * W - mean; varc += d * d }; varc /= rows.size
        val straightness = (1f - sqrt(varc) / half).coerceIn(0f, 1f)
        val temporal = if (haveOutput) (1f - abs(offset - prevOffset)).coerceIn(0f, 1f) else 0.7f
        val conf = coverage * separability * straightness * temporal

        if (haveOutput) {
            offset = (1 - outEma) * prevOffset + outEma * offset
            heading = (1 - outEma) * prevHeading + outEma * heading
        }
        prevOffset = offset; prevHeading = heading; haveOutput = true

        return Result(conf > 0.15f, offset, heading, conf, centerline, W, H)
    }

    private fun rgbToLab(r: Int, g: Int, b: Int): FloatArray {
        fun lin(c: Int): Float { val v = c / 255f; return if (v <= 0.04045f) v / 12.92f else Math.pow(((v + 0.055) / 1.055), 2.4).toFloat() }
        val rl = lin(r); val gl = lin(g); val bl = lin(b)
        val x = (rl * 0.4124f + gl * 0.3576f + bl * 0.1805f) / 0.95047f
        val y = (rl * 0.2126f + gl * 0.7152f + bl * 0.0722f)
        val z = (rl * 0.0193f + gl * 0.1192f + bl * 0.9505f) / 1.08883f
        fun f(t: Float): Float = if (t > 0.008856f) Math.cbrt(t.toDouble()).toFloat() else 7.787f * t + 16f / 116f
        val fx = f(x); val fy = f(y); val fz = f(z)
        return floatArrayOf(116f * fy - 16f, 500f * (fx - fy), 200f * (fy - fz))
    }
}
