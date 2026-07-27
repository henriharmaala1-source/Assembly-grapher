package com.kestrel.tracker.track

/**
 * A frame the tracker works on: luminance (row-major floats [0,255]) plus,
 * optionally, per-pixel centred chroma (cu = U-128, cv = V-128) carried
 * alongside so colour-based filters can run. Decoupled from Android types so it
 * unit-tests and ports to the onboard C++ tracker.
 */
class GrayFrame(
    val d: FloatArray, val w: Int, val h: Int,
    val cu: FloatArray? = null, val cv: FloatArray? = null,
) {
    val hasColor get() = cu != null && cv != null

    fun at(x: Int, y: Int): Float = d[y * w + x]

    /** Region (rx,ry,rw,rh) resampled to (outW,outH), bilinear. Chroma is
     *  resampled too when present, so a colour filter on the crop still works. */
    fun cropResample(rx: Float, ry: Float, rw: Float, rh: Float,
                     outW: Int, outH: Int): GrayFrame =
        cropResampleInto(rx, ry, rw, rh, outW, outH, null, null, null)

    /**
     * As [cropResample], but writes into CALLER-OWNED buffers when they are
     * supplied and large enough (only the first outW*outH entries are touched).
     *
     * This exists purely to keep the tracker off the allocator. In a wide
     * re-acquire the crop is 384x384, so a fresh luma+chroma set is 1.77 MB —
     * every frame, on the camera-delivery thread. That is the same order as the
     * per-frame garbage that previously collapsed the app from 31 fps to 7, and it
     * lands in the same place: GC pauses attributed to the tracker, with the
     * measured arithmetic (a few million multiply-adds) nowhere near able to
     * explain the time.
     */
    fun cropResampleInto(rx: Float, ry: Float, rw: Float, rh: Float,
                         outW: Int, outH: Int,
                         dstD: FloatArray?, dstU: FloatArray?, dstV: FloatArray?): GrayFrame {
        val n = outW * outH
        val outD = if (dstD != null && dstD.size >= n) dstD else FloatArray(n)
        val outU = if (cu == null) null else if (dstU != null && dstU.size >= n) dstU else FloatArray(n)
        val outV = if (cv == null) null else if (dstV != null && dstV.size >= n) dstV else FloatArray(n)
        val sx = rw / outW
        val sy = rh / outH
        for (j in 0 until outH) {
            val fy = (ry + (j + 0.5f) * sy).coerceIn(0f, (h - 1).toFloat())
            val y0 = fy.toInt(); val y1 = (y0 + 1).coerceAtMost(h - 1); val ty = fy - y0
            for (i in 0 until outW) {
                val fx = (rx + (i + 0.5f) * sx).coerceIn(0f, (w - 1).toFloat())
                val x0 = fx.toInt(); val x1 = (x0 + 1).coerceAtMost(w - 1); val tx = fx - x0
                val o = j * outW + i
                outD[o] = bilerp(d, x0, x1, y0, y1, tx, ty)
                if (outU != null) outU[o] = bilerp(cu!!, x0, x1, y0, y1, tx, ty)
                if (outV != null) outV[o] = bilerp(cv!!, x0, x1, y0, y1, tx, ty)
            }
        }
        return GrayFrame(outD, outW, outH, outU, outV)
    }

    private fun bilerp(a: FloatArray, x0: Int, x1: Int, y0: Int, y1: Int, tx: Float, ty: Float): Float {
        val p = a[y0 * w + x0]; val q = a[y0 * w + x1]
        val r = a[y1 * w + x0]; val s = a[y1 * w + x1]
        val top = p + (q - p) * tx
        val bot = r + (s - r) * tx
        return top + (bot - top) * ty
    }

    companion object {
        /** Build from an NV21 buffer: Y plane = luma; VU (2×2-subsampled) →
         *  per-pixel centred chroma. Returns luma-only if the buffer lacks UV. */
        fun fromNv21(nv21: ByteArray, w: Int, h: Int): GrayFrame {
            val n = w * h
            val d = FloatArray(n) { (nv21[it].toInt() and 0xFF).toFloat() }
            if (nv21.size < n + n / 2) return GrayFrame(d, w, h)
            val cu = FloatArray(n); val cv = FloatArray(n)
            for (j in 0 until h) {
                val uvRow = n + (j shr 1) * w
                for (i in 0 until w) {
                    val uv = uvRow + (i and 1.inv())        // (i/2)*2; NV21 = V,U
                    cv[j * w + i] = ((nv21[uv].toInt() and 0xFF) - 128).toFloat()
                    cu[j * w + i] = ((nv21[uv + 1].toInt() and 0xFF) - 128).toFloat()
                }
            }
            return GrayFrame(d, w, h, cu, cv)
        }
    }
}
