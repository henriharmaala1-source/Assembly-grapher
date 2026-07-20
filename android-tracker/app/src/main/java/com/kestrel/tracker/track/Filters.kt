package com.kestrel.tracker.track

import kotlin.math.sqrt

/**
 * Appearance filters applied to the working crop before correlation — the same
 * "which filter best reveals the target on a low-contrast / thermal-like feed"
 * A/B test from the desktop tool, but here feeding the TRACKER, not just the
 * display. Applying the same filter to template and search crop makes the
 * correlation invariant to whatever the filter removes (e.g. absolute contrast).
 * All operate on a GrayFrame and return a new GrayFrame of the same size.
 */
enum class CropFilter { NONE, STRETCH, EDGE, THRESHOLD, SHARPEN, CHROMA }

object Filters {

    fun apply(g: GrayFrame, f: CropFilter): GrayFrame = when (f) {
        CropFilter.NONE      -> g
        CropFilter.STRETCH   -> stretch(g)     // percentile contrast stretch (CLAHE-lite)
        CropFilter.EDGE      -> sobel(g)       // gradient magnitude — structure
        CropFilter.THRESHOLD -> otsu(g)        // hot-blob binary — thermal style
        CropFilter.SHARPEN   -> sharpen(g)
        CropFilter.CHROMA    -> chroma(g)      // colourfulness — colour-distinct target pops
    }

    /** Track on colour instead of luma: per-pixel chroma magnitude
     *  sqrt((U-128)^2+(V-128)^2). A saturated target on a desaturated background
     *  pops even when they're the same brightness. Brightness-invariant (helps
     *  under changing light); wraparound-free (unlike hue), so NCC behaves.
     *  Passthrough to luma if the frame carries no colour (e.g. thermal). */
    private fun chroma(g: GrayFrame): GrayFrame {
        val cu = g.cu; val cv = g.cv
        if (cu == null || cv == null) return g
        val out = FloatArray(g.d.size) {
            (sqrt(cu[it] * cu[it] + cv[it] * cv[it]) * 1.41f).coerceIn(0f, 255f)
        }
        return GrayFrame(out, g.w, g.h, cu, cv)
    }

    /** Robust 2nd/98th-percentile contrast stretch — cheap CLAHE stand-in. */
    private fun stretch(g: GrayFrame): GrayFrame {
        val n = g.d.size
        val sample = FloatArray((n + 7) / 8) { g.d[it * 8] }
        sample.sort()
        val lo = sample[sample.size * 2 / 100]
        val hi = sample[sample.size * 98 / 100]
        val range = (hi - lo).coerceAtLeast(1e-3f)
        val out = FloatArray(n) { ((g.d[it] - lo) / range * 255f).coerceIn(0f, 255f) }
        return GrayFrame(out, g.w, g.h)
    }

    private fun sobel(g: GrayFrame): GrayFrame {
        val w = g.w; val h = g.h; val out = FloatArray(w * h)
        for (y in 1 until h - 1) for (x in 1 until w - 1) {
            val gx = -g.at(x-1,y-1) - 2*g.at(x-1,y) - g.at(x-1,y+1) +
                      g.at(x+1,y-1) + 2*g.at(x+1,y) + g.at(x+1,y+1)
            val gy = -g.at(x-1,y-1) - 2*g.at(x,y-1) - g.at(x+1,y-1) +
                      g.at(x-1,y+1) + 2*g.at(x,y+1) + g.at(x+1,y+1)
            out[y * w + x] = sqrt(gx * gx + gy * gy).coerceAtMost(255f)
        }
        return GrayFrame(out, w, h)
    }

    private fun otsu(g: GrayFrame): GrayFrame {
        val hist = IntArray(256)
        for (v in g.d) hist[v.toInt().coerceIn(0, 255)]++
        val total = g.d.size
        var sum = 0.0; for (i in 0..255) sum += i.toDouble() * hist[i]
        var sumB = 0.0; var wB = 0; var maxVar = 0.0; var thresh = 127
        for (i in 0..255) {
            wB += hist[i]; if (wB == 0) continue
            val wF = total - wB; if (wF == 0) break
            sumB += i.toDouble() * hist[i]
            val mB = sumB / wB; val mF = (sum - sumB) / wF
            val between = wB.toDouble() * wF * (mB - mF) * (mB - mF)
            if (between > maxVar) { maxVar = between; thresh = i }
        }
        val out = FloatArray(g.d.size) { if (g.d[it] > thresh) 255f else 0f }
        return GrayFrame(out, g.w, g.h)
    }

    private fun sharpen(g: GrayFrame): GrayFrame {
        val w = g.w; val h = g.h; val out = g.d.copyOf()
        for (y in 1 until h - 1) for (x in 1 until w - 1) {
            val lap = 4*g.at(x,y) - g.at(x-1,y) - g.at(x+1,y) - g.at(x,y-1) - g.at(x,y+1)
            out[y * w + x] = (g.at(x, y) + 0.7f * lap).coerceIn(0f, 255f)
        }
        return GrayFrame(out, w, h)
    }
}
