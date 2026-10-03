package com.kestrel.navviz.depth

import android.graphics.Bitmap

/**
 * Lightweight click-to-lock tracker.
 *
 * SCOPE, stated honestly: the onboard LockOnTracker
 * (onboard/src/lock_tracker.cpp) is a three-layer stack — a CSRT/KCF/optical-
 * flow primary backend, a Kalman [x,y,vx,vy] filter, and NCC template
 * re-detection. Only that THIRD layer — normalised cross-correlation template
 * matching — is portable to pure Kotlin (CSRT/KCF need OpenCV-contrib). So this
 * is the NCC re-detection layer used as the whole tracker: keep a grayscale
 * template of the designated patch, each frame search a window around the last
 * position for the NCC peak, move the box there, refresh the template
 * periodically, and drop the lock when the peak score stays below threshold.
 * Good enough to demonstrate lock-on on the phone; NOT the full onboard tracker.
 */
class LockTracker {

    data class Box(val cx: Float, val cy: Float, val size: Int, val conf: Float, val locked: Boolean)

    private val tmplSize = 48        // template patch (px, in the working gray image)
    private val searchPad = 40       // +/- search window around last centre
    private val reacquire = 0.45f    // NCC below this for LOSS_TIMEOUT frames -> lost
    private val lossTimeout = 15
    private val tmplRefresh = 30

    private var tmpl: FloatArray? = null   // mean-subtracted template, tmplSize^2
    private var tmplNorm = 0f
    private var cx = 0f; private var cy = 0f
    private var frames = 0; private var lossFrames = 0
    private var lastConf = 0f
    private var active = false

    val hasTarget get() = active

    fun reset() { tmpl = null; active = false; lossFrames = 0 }

    /** Designate a target at (px,py) in the given frame's pixel coords. */
    fun designate(bmp: Bitmap, px: Float, py: Float) {
        val g = toGray(bmp)
        cx = px.coerceIn(tmplSize / 2f, (g.w - tmplSize / 2).toFloat())
        cy = py.coerceIn(tmplSize / 2f, (g.h - tmplSize / 2).toFloat())
        tmpl = patch(g, cx.toInt(), cy.toInt(), tmplSize)?.also { tmplNorm = normOf(it) }
        active = tmpl != null; frames = 0; lossFrames = 0; lastConf = 1f
    }

    /** Track into the current frame. Returns the box, or null if no active lock. */
    fun update(bmp: Bitmap): Box? {
        val t = tmpl ?: return null
        val g = toGray(bmp)
        // Search the window around the last centre for the NCC peak.
        var best = -1f; var bx = cx.toInt(); var by = cy.toInt()
        val x0 = (cx.toInt() - searchPad).coerceAtLeast(tmplSize / 2)
        val x1 = (cx.toInt() + searchPad).coerceAtMost(g.w - tmplSize / 2)
        val y0 = (cy.toInt() - searchPad).coerceAtLeast(tmplSize / 2)
        val y1 = (cy.toInt() + searchPad).coerceAtMost(g.h - tmplSize / 2)
        var y = y0
        while (y <= y1) {
            var x = x0
            while (x <= x1) {
                val ncc = nccAt(g, x, y, t, tmplNorm)
                if (ncc > best) { best = ncc; bx = x; by = y }
                x += 2   // stride 2 for speed; sub-pixel isn't needed here
            }
            y += 2
        }
        lastConf = best.coerceIn(0f, 1f)
        if (best >= reacquire) { cx = bx.toFloat(); cy = by.toFloat(); lossFrames = 0 }
        else lossFrames++

        if (lossFrames > lossTimeout) { active = false; return null }

        frames++
        if (frames % tmplRefresh == 0 && lossFrames == 0) {
            patch(g, cx.toInt(), cy.toInt(), tmplSize)?.let { tmpl = it; tmplNorm = normOf(it) }
        }
        // Scale box coords back to the source bitmap.
        val sx = bmp.width.toFloat() / g.w; val sy = bmp.height.toFloat() / g.h
        return Box(cx * sx, cy * sy, (tmplSize * sx).toInt(), lastConf, lossFrames == 0)
    }

    // --- helpers ---------------------------------------------------------

    private class Gray(val d: FloatArray, val w: Int, val h: Int)

    private val gw = 240
    private fun toGray(bmp: Bitmap): Gray {
        val gh = (gw * bmp.height / bmp.width).coerceAtLeast(1)
        val s = Bitmap.createScaledBitmap(bmp, gw, gh, true)
        val px = IntArray(gw * gh); s.getPixels(px, 0, gw, 0, 0, gw, gh)
        val d = FloatArray(gw * gh)
        for (i in px.indices) {
            val p = px[i]
            d[i] = 0.299f * ((p shr 16) and 0xFF) + 0.587f * ((p shr 8) and 0xFF) + 0.114f * (p and 0xFF)
        }
        return Gray(d, gw, gh)
    }

    private fun patch(g: Gray, ccx: Int, ccy: Int, sz: Int): FloatArray? {
        val h = sz / 2
        if (ccx - h < 0 || ccy - h < 0 || ccx + h > g.w || ccy + h > g.h) return null
        val out = FloatArray(sz * sz); var sum = 0f
        for (j in 0 until sz) for (i in 0 until sz) {
            val v = g.d[(ccy - h + j) * g.w + (ccx - h + i)]; out[j * sz + i] = v; sum += v
        }
        val mean = sum / (sz * sz)
        for (i in out.indices) out[i] -= mean   // mean-subtract for NCC
        return out
    }

    private fun normOf(t: FloatArray): Float { var s = 0f; for (v in t) s += v * v; return kotlin.math.sqrt(s) + 1e-6f }

    private fun nccAt(g: Gray, ccx: Int, ccy: Int, t: FloatArray, tn: Float): Float {
        val sz = tmplSize; val h = sz / 2
        var sum = 0f
        for (j in 0 until sz) for (i in 0 until sz) sum += g.d[(ccy - h + j) * g.w + (ccx - h + i)]
        val mean = sum / (sz * sz)
        var dot = 0f; var wn = 0f
        for (j in 0 until sz) for (i in 0 until sz) {
            val v = g.d[(ccy - h + j) * g.w + (ccx - h + i)] - mean
            dot += v * t[j * sz + i]; wn += v * v
        }
        return dot / (tn * (kotlin.math.sqrt(wn) + 1e-6f))
    }
}
