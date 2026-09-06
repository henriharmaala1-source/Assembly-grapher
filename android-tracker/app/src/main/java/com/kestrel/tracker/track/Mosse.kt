package com.kestrel.tracker.track

import kotlin.math.cos
import kotlin.math.ln
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * MOSSE adaptive correlation filter (Bolme et al. 2010), pure Kotlin.
 *
 * Replaces the brute-force NCC search — O(positions × template²) — with a single
 * FFT correlation over the whole window, O(N log N). It searches the ENTIRE
 * `sz × sz` window every frame (not just a small grid), adapts every frame, and
 * yields PSR natively. Validated in desktop/mosse_bench.py: equal-or-better
 * localisation than NCC, and ~4× cheaper for the same search coverage — the
 * frame-cost headroom for a bigger re-acquire search, and the biggest onboard
 * C++/NEON win.
 *
 * Not the default matcher yet: the NCC + fusion + bank + occlusion pipeline is the
 * validated tracker; swapping MOSSE in wholesale is the measured follow-up (P0-B
 * on real footage). This module is the ready, unit-checkable building block.
 *
 * Complex arrays are parallel float[] (re, im), row-major sz×sz. sz must be a
 * power of two.
 */
class Mosse(val sz: Int = 64, sigma: Float = 2f, private val eta: Float = 0.125f,
            private val lambda: Float = 1e-2f) {

    private val n = sz * sz
    private val win = FloatArray(n)          // cosine (Hann) window
    private val gr = FloatArray(n); private val gi = FloatArray(n)   // FFT of desired Gaussian
    private val ar = FloatArray(n); private val ai = FloatArray(n)   // numerator A
    private val b = FloatArray(n)            // denominator B (real: |F|² + λ)
    private var trained = false

    init {
        require(sz > 0 && (sz and (sz - 1)) == 0) { "sz must be a power of two" }
        val c = (sz - 1).toFloat()
        for (y in 0 until sz) for (x in 0 until sz) {
            win[y * sz + x] = (0.5f - 0.5f * cos(2f * Math.PI.toFloat() * x / c)) *
                              (0.5f - 0.5f * cos(2f * Math.PI.toFloat() * y / c))
        }
        // Desired output g: Gaussian peaked at the window CENTRE. Its FFT is G.
        val two = 2f * sigma * sigma
        for (y in 0 until sz) for (x in 0 until sz) {
            val dx = (x - sz / 2).toFloat(); val dy = (y - sz / 2).toFloat()
            gr[y * sz + x] = kotlin.math.exp(-(dx * dx + dy * dy) / two)
        }
        fft2(gr, gi, false)
    }

    val trainedOk get() = trained

    /** Train on the first patch (sz×sz luma). */
    fun init(patch: FloatArray) {
        val (fr, fi) = fft2Of(preprocess(patch))
        for (i in 0 until n) {
            // A = G ⊙ conj(F);  B = |F|² + λ
            ar[i] = gr[i] * fr[i] + gi[i] * fi[i]
            ai[i] = gi[i] * fr[i] - gr[i] * fi[i]
            b[i] = fr[i] * fr[i] + fi[i] * fi[i] + lambda
        }
        trained = true
    }

    /** Correlate a new patch; returns (dx, dy) of the target from the window centre
     *  and the response PSR. Response = ifft2(Z ⊙ H*), H* = A/B. */
    fun track(patch: FloatArray): FloatArray {   // [dx, dy, psr]
        val (zr, zi) = fft2Of(preprocess(patch))
        val rr = FloatArray(n); val ri = FloatArray(n)
        for (i in 0 until n) {
            val hr = ar[i] / b[i]; val hi = ai[i] / b[i]     // H* = A/B
            rr[i] = zr[i] * hr - zi[i] * hi
            ri[i] = zr[i] * hi + zi[i] * hr
        }
        fft2(rr, ri, true)                                   // inverse → real correlation in rr
        var pk = 0; var peak = rr[0]
        for (i in 1 until n) if (rr[i] > peak) { peak = rr[i]; pk = i }
        val px = pk % sz; val py = pk / sz
        return floatArrayOf((px - sz / 2).toFloat(), (py - sz / 2).toFloat(), psr(rr, px, py))
    }

    /** Adapt the filter to a patch re-centred on the found target. */
    fun update(patch: FloatArray) {
        val (fr, fi) = fft2Of(preprocess(patch))
        for (i in 0 until n) {
            val newAr = gr[i] * fr[i] + gi[i] * fi[i]
            val newAi = gi[i] * fr[i] - gr[i] * fi[i]
            val newB = fr[i] * fr[i] + fi[i] * fi[i] + lambda
            ar[i] = eta * newAr + (1 - eta) * ar[i]
            ai[i] = eta * newAi + (1 - eta) * ai[i]
            b[i] = eta * newB + (1 - eta) * b[i]
        }
    }

    // --- helpers -------------------------------------------------------------

    /** log-transform, mean/std normalise, cosine-window (MOSSE preprocessing). */
    private fun preprocess(patch: FloatArray): FloatArray {
        val p = FloatArray(n)
        var mean = 0f
        for (i in 0 until n) { p[i] = ln((if (patch[i] > 0f) patch[i] else 0f) + 1f); mean += p[i] }
        mean /= n
        var v = 0f
        for (i in 0 until n) { val d = p[i] - mean; v += d * d }
        val std = sqrt(v / n) + 1e-5f
        for (i in 0 until n) p[i] = (p[i] - mean) / std * win[i]
        return p
    }

    private fun psr(r: FloatArray, px: Int, py: Int): Float {
        val peak = r[py * sz + px]
        var sum = 0f; var sum2 = 0f; var cnt = 0
        for (y in 0 until sz) for (x in 0 until sz) {
            if (kotlin.math.abs(x - px) <= 5 && kotlin.math.abs(y - py) <= 5) continue
            val vv = r[y * sz + x]; sum += vv; sum2 += vv * vv; cnt++
        }
        if (cnt < 4) return 0f
        val m = sum / cnt
        val s = sqrt((sum2 / cnt - m * m).coerceAtLeast(1e-6f))
        return (peak - m) / s
    }

    private fun fft2Of(real: FloatArray): Pair<FloatArray, FloatArray> {
        val re = real.copyOf(); val im = FloatArray(n)
        fft2(re, im, false)
        return re to im
    }

    /** In-place 2D FFT (row FFTs then column FFTs). */
    private fun fft2(re: FloatArray, im: FloatArray, inv: Boolean) {
        val tr = FloatArray(sz); val ti = FloatArray(sz)
        for (y in 0 until sz) {                              // rows
            val off = y * sz
            for (x in 0 until sz) { tr[x] = re[off + x]; ti[x] = im[off + x] }
            fft1(tr, ti, inv)
            for (x in 0 until sz) { re[off + x] = tr[x]; im[off + x] = ti[x] }
        }
        for (x in 0 until sz) {                              // columns
            for (y in 0 until sz) { tr[y] = re[y * sz + x]; ti[y] = im[y * sz + x] }
            fft1(tr, ti, inv)
            for (y in 0 until sz) { re[y * sz + x] = tr[y]; im[y * sz + x] = ti[y] }
        }
    }

    /** In-place iterative radix-2 Cooley–Tukey FFT (length = sz, power of two). */
    private fun fft1(re: FloatArray, im: FloatArray, inv: Boolean) {
        val len = re.size
        var j = 0
        for (i in 1 until len) {                             // bit-reversal permutation
            var bit = len shr 1
            while (j and bit != 0) { j = j xor bit; bit = bit shr 1 }
            j = j or bit
            if (i < j) {
                val tr = re[i]; re[i] = re[j]; re[j] = tr
                val ti = im[i]; im[i] = im[j]; im[j] = ti
            }
        }
        var l = 2
        while (l <= len) {
            val ang = (if (inv) 2.0 else -2.0) * Math.PI / l
            val wr = cos(ang); val wi = sin(ang)             // double twiddle for accuracy
            var i = 0
            while (i < len) {
                var curR = 1.0; var curI = 0.0
                val half = l / 2
                for (k in 0 until half) {
                    val a = i + k; val c = i + k + half
                    val vr = re[c] * curR - im[c] * curI
                    val vi = re[c] * curI + im[c] * curR
                    re[c] = (re[a] - vr).toFloat(); im[c] = (im[a] - vi).toFloat()
                    re[a] = (re[a] + vr).toFloat(); im[a] = (im[a] + vi).toFloat()
                    val nr = curR * wr - curI * wi; curI = curR * wi + curI * wr; curR = nr
                }
                i += l
            }
            l = l shl 1
        }
        if (inv) for (i in 0 until len) { re[i] /= len; im[i] /= len }
    }
}
