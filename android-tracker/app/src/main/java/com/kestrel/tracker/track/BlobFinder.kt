package com.kestrel.tracker.track

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Forgiving tap-to-lock: given a rough tap, find the distinct blob near it and
 * return its extent as a box, so you don't have to hit a target pixel-precisely
 * (hard on a touchscreen, harder cueing from a moving drone).
 *
 * Method: in a window around the tap, saliency = each pixel's deviation from the
 * window mean (luma, plus chroma when present) — the target pops because it's a
 * minority of a mostly-background window. Otsu-threshold that, connected-
 * components, and pick the blob nearest the tap. Returns [cx,cy,size] in frame
 * coords (size = the blob's larger side), or null if nothing distinct is found.
 *
 * Pure Kotlin on GrayFrame — testable, ports to the onboard tracker.
 */
object BlobFinder {

    fun findBlob(g: GrayFrame, px: Float, py: Float, radius: Int = 55): FloatArray? {
        val cx0 = px.toInt().coerceIn(0, g.w - 1); val cy0 = py.toInt().coerceIn(0, g.h - 1)
        val x0 = (cx0 - radius).coerceAtLeast(0); val x1 = (cx0 + radius).coerceAtMost(g.w - 1)
        val y0 = (cy0 - radius).coerceAtLeast(0); val y1 = (cy0 + radius).coerceAtMost(g.h - 1)
        val ww = x1 - x0 + 1; val hh = y1 - y0 + 1
        if (ww < 8 || hh < 8) return null
        val hasColor = g.cu != null && g.cv != null
        val cu = g.cu; val cv = g.cv

        // Window mean = background estimate.
        var mL = 0f; var mU = 0f; var mV = 0f
        for (y in y0..y1) for (x in x0..x1) {
            val p = y * g.w + x; mL += g.d[p]
            if (hasColor) { mU += cu!![p]; mV += cv!![p] }
        }
        val n = ww * hh; mL /= n; mU /= n; mV /= n

        // Saliency = deviation from the window mean.
        val sal = FloatArray(ww * hh)
        var maxS = 1e-6f
        for (y in 0 until hh) for (x in 0 until ww) {
            val p = (y0 + y) * g.w + (x0 + x)
            var s = abs(g.d[p] - mL)
            if (hasColor) { val du = cu!![p] - mU; val dv = cv!![p] - mV; s += sqrt(du * du + dv * dv) }
            sal[y * ww + x] = s
            if (s > maxS) maxS = s
        }

        // Otsu threshold on the saliency map.
        val hist = IntArray(256)
        for (v in sal) hist[(v / maxS * 255f).toInt().coerceIn(0, 255)]++
        val thr = otsu(hist, ww * hh) / 255f * maxS
        val mask = BooleanArray(ww * hh) { sal[it] > thr }

        // Connected components → pick the one nearest the tap (min size, not
        // whole-window). Flood fill.
        val tapLx = cx0 - x0; val tapLy = cy0 - y0
        val visited = BooleanArray(ww * hh); val stack = ArrayDeque<Int>()
        var best: IntArray? = null; var bestD = Float.MAX_VALUE
        for (start in 0 until ww * hh) {
            if (!mask[start] || visited[start]) continue
            var minx = ww; var miny = hh; var maxx = 0; var maxy = 0; var area = 0
            stack.addLast(start); visited[start] = true
            while (stack.isNotEmpty()) {
                val idx = stack.removeLast(); val lx = idx % ww; val ly = idx / ww; area++
                if (lx < minx) minx = lx; if (lx > maxx) maxx = lx
                if (ly < miny) miny = ly; if (ly > maxy) maxy = ly
                if (lx > 0)      push(mask, visited, stack, idx - 1)
                if (lx < ww - 1) push(mask, visited, stack, idx + 1)
                if (ly > 0)      push(mask, visited, stack, idx - ww)
                if (ly < hh - 1) push(mask, visited, stack, idx + ww)
            }
            if (area < 9) continue
            if (maxx - minx >= ww - 2 && maxy - miny >= hh - 2) continue   // fills window → no blob
            val ccx = (minx + maxx) / 2f; val ccy = (miny + maxy) / 2f
            val d = (ccx - tapLx) * (ccx - tapLx) + (ccy - tapLy) * (ccy - tapLy)
            if (d < bestD) { bestD = d; best = intArrayOf(minx, miny, maxx, maxy) }
        }
        val b = best ?: return null
        val bcx = x0 + (b[0] + b[2]) / 2f; val bcy = y0 + (b[1] + b[3]) / 2f
        // Margin for context + a floor so the crop isn't tiny/over-zoomed (which
        // leaves too small a search window to survive movement).
        val size = (maxOf(b[2] - b[0] + 1, b[3] - b[1] + 1) * 1.3f).coerceAtLeast(48f)
        return floatArrayOf(bcx, bcy, size)
    }

    private fun push(mask: BooleanArray, visited: BooleanArray, stack: ArrayDeque<Int>, ni: Int) {
        if (mask[ni] && !visited[ni]) { visited[ni] = true; stack.addLast(ni) }
    }

    private fun otsu(hist: IntArray, total: Int): Int {
        var sum = 0.0; for (i in 0..255) sum += i.toDouble() * hist[i]
        var sumB = 0.0; var wB = 0; var maxVar = 0.0; var thr = 127
        for (i in 0..255) {
            wB += hist[i]; if (wB == 0) continue
            val wF = total - wB; if (wF == 0) break
            sumB += i.toDouble() * hist[i]
            val mB = sumB / wB; val mF = (sum - sumB) / wF
            val v = wB.toDouble() * wF * (mB - mF) * (mB - mF)
            if (v > maxVar) { maxVar = v; thr = i }
        }
        return thr
    }
}
