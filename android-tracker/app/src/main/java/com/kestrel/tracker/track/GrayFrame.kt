package com.kestrel.tracker.track

/**
 * A single-channel (luminance) frame: row-major floats in [0,255], plus size.
 * The whole tracker works on this — decoupled from Android Bitmap/camera types
 * so the logic is unit-testable and ports directly to the onboard C++ tracker.
 */
class GrayFrame(val d: FloatArray, val w: Int, val h: Int) {

    fun at(x: Int, y: Int): Float = d[y * w + x]

    /**
     * Extract the region (rx,ry,rw,rh) from this frame and resample it to
     * (outW,outH) with bilinear interpolation — the "followed crop" the tracker
     * runs on. The region is clamped to the frame; sampling outside reads the
     * nearest edge pixel. Returns a new GrayFrame of size outW×outH.
     */
    fun cropResample(rx: Float, ry: Float, rw: Float, rh: Float,
                     outW: Int, outH: Int): GrayFrame {
        val out = FloatArray(outW * outH)
        val sx = rw / outW
        val sy = rh / outH
        for (j in 0 until outH) {
            val fy = (ry + (j + 0.5f) * sy).coerceIn(0f, (h - 1).toFloat())
            val y0 = fy.toInt(); val y1 = (y0 + 1).coerceAtMost(h - 1); val ty = fy - y0
            for (i in 0 until outW) {
                val fx = (rx + (i + 0.5f) * sx).coerceIn(0f, (w - 1).toFloat())
                val x0 = fx.toInt(); val x1 = (x0 + 1).coerceAtMost(w - 1); val tx = fx - x0
                val a = d[y0 * w + x0]; val b = d[y0 * w + x1]
                val c = d[y1 * w + x0]; val e = d[y1 * w + x1]
                val top = a + (b - a) * tx
                val bot = c + (e - c) * tx
                out[j * outW + i] = top + (bot - top) * ty
            }
        }
        return GrayFrame(out, outW, outH)
    }
}
