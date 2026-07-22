package com.kestrel.tracker.track

import kotlin.math.roundToInt

/**
 * Sparse optical flow → global (ego-motion) translation estimate.
 *
 * Samples a grid of TEXTURED points, matches each into the next frame by a small
 * SSD search, and takes the MEDIAN displacement. The median is the key: an
 * independently-moving object (or a bad match) is an outlier the median ignores,
 * so what's left is the camera's own motion. Used to cancel ego-motion before
 * frame-differencing, so MOTION acquisition works on a moving/vibrating drone —
 * not just a tripod. At 50–800 m the scene is far (little parallax), so a single
 * global translation is a good model of yaw/pitch pan.
 *
 * Pure Kotlin on GrayFrame — MCU-portable; the (dx,dy) is also a velocity cue
 * for nav VIO (P5a) later.
 */
class OpticalFlow {

    var patch = 5           // half-size of the match patch
    var search = 12         // half-size of the search window
    var gridX = 8; var gridY = 6
    var minVar = 40f        // skip flat (ambiguous) patches
    var fbSearch = 4        // backward-match half-window (TLD-style FB check)
    var fbMaxError = 1.5f   // discard a point if the round trip exceeds this

    /** Fraction of grid points that agreed with the median of the last estimate()
     *  — high on a rigid camera pan, low under noise or a large independently-
     *  moving occluder. A caller trusts the ego translation only when this is high. */
    var consensus = 0f; private set

    /** Global translation (dx,dy) mapping `prev` onto `cur`, in px (0,0 if weak).
     *
     *  `exCx,exCy,exHalf` EXCLUDE grid points inside the current tracked box (half
     *  size in px, 0 = no exclusion) — if the target is a large fraction of the
     *  frame, its own motion could otherwise win the median vote with high
     *  consensus even though it isn't camera pan at all.
     *
     *  Each surviving point is also checked FORWARD-BACKWARD (TLD-style): the
     *  found position is re-matched back toward its origin; a round trip that
     *  doesn't return close to the start means the match was ambiguous (aliased
     *  texture, not real motion) and is discarded before it can pollute the
     *  median/consensus. */
    fun estimate(prev: GrayFrame, cur: GrayFrame,
                exCx: Float = -1f, exCy: Float = -1f, exHalf: Float = 0f): Pair<Float, Float> {
        consensus = 0f
        val w = prev.w; val h = prev.h
        val m = patch + search
        if (w <= 2 * m || h <= 2 * m) return 0f to 0f
        val dxs = ArrayList<Float>(); val dys = ArrayList<Float>()
        for (gy in 1..gridY) for (gx in 1..gridX) {
            val cx = m + (w - 2 * m) * gx / (gridX + 1)
            val cy = m + (h - 2 * m) * gy / (gridY + 1)
            if (exHalf > 0f && kotlin.math.abs(cx - exCx) <= exHalf && kotlin.math.abs(cy - exCy) <= exHalf)
                continue                                            // skip the tracked target region
            if (patchVar(prev, cx, cy) < minVar) continue          // skip flat
            var best = Float.MAX_VALUE; var bdx = 0; var bdy = 0
            for (oy in -search..search) for (ox in -search..search) {
                val s = ssd(prev, cx, cy, cur, cx + ox, cy + oy)
                if (s < best) { best = s; bdx = ox; bdy = oy }
            }
            if (best == Float.MAX_VALUE) continue                  // no valid forward match (edge)
            // forward-backward check: re-match the found patch in `cur` back toward
            // the origin in `prev` — a large round-trip error means an unreliable point.
            var bestBack = Float.MAX_VALUE; var bbx = 0; var bby = 0
            for (oy in -fbSearch..fbSearch) for (ox in -fbSearch..fbSearch) {
                val s = ssd(cur, cx + bdx, cy + bdy, prev, cx + ox, cy + oy)
                if (s < bestBack) { bestBack = s; bbx = ox; bby = oy }
            }
            if (kotlin.math.hypot(bbx.toFloat(), bby.toFloat()) > fbMaxError) continue
            dxs.add(bdx.toFloat()); dys.add(bdy.toFloat())
        }
        if (dxs.size < 4) return 0f to 0f
        // median on COPIES so the original dxs/dys pairing survives for consensus.
        val mdx = median(ArrayList(dxs)); val mdy = median(ArrayList(dys))
        var inl = 0
        for (i in dxs.indices) if (kotlin.math.hypot(dxs[i] - mdx, dys[i] - mdy) <= 2f) inl++
        consensus = inl.toFloat() / dxs.size
        return mdx to mdy
    }

    private fun patchVar(g: GrayFrame, cx: Int, cy: Int): Float {
        var s = 0f; var s2 = 0f; var n = 0
        for (j in -patch..patch) for (i in -patch..patch) {
            val v = g.d[(cy + j) * g.w + (cx + i)]; s += v; s2 += v * v; n++
        }
        val mean = s / n; return s2 / n - mean * mean
    }

    private fun ssd(a: GrayFrame, ax: Int, ay: Int, b: GrayFrame, bx: Int, by: Int): Float {
        if (bx - patch < 0 || by - patch < 0 || bx + patch >= b.w || by + patch >= b.h)
            return Float.MAX_VALUE
        var s = 0f
        for (j in -patch..patch) for (i in -patch..patch) {
            val d = a.d[(ay + j) * a.w + (ax + i)] - b.d[(by + j) * b.w + (bx + i)]
            s += d * d
        }
        return s
    }

    private fun median(v: ArrayList<Float>): Float { v.sort(); return v[v.size / 2] }

    companion object {
        /** Shift `img` (w×h) in place by (dx,dy) px — align an accumulated
         *  background to the current view. Edge pixels clamp. */
        fun shift(img: FloatArray, w: Int, h: Int, dx: Float, dy: Float) {
            val idx = dx.roundToInt(); val idy = dy.roundToInt()
            if (idx == 0 && idy == 0) return
            val src = img.copyOf()
            for (y in 0 until h) for (x in 0 until w) {
                val sx = (x - idx).coerceIn(0, w - 1)
                val sy = (y - idy).coerceIn(0, h - 1)
                img[y * w + x] = src[sy * w + sx]
            }
        }
    }
}
