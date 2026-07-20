package com.kestrel.tracker.ui

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.graphics.RectF
import android.view.View
import com.kestrel.tracker.track.GrayFrame
import com.kestrel.tracker.track.LockTracker

/**
 * Draws the (grayscale) feed + the tracked box + a top-right zoom PiP of the
 * followed crop — the reference-footage layout. Everything is luminance: the
 * analog/thermal feed is effectively mono, and the tracker only needs luma, so
 * we never pay for YUV→RGB or a JPEG round-trip (the navviz framerate lesson).
 */
class TrackerOverlayView(context: Context) : View(context) {

    private var frameBmp: Bitmap? = null
    private var frameW = 0
    private var frameH = 0
    private var framePx = IntArray(0)

    private var pipBmp: Bitmap? = null
    private var result: LockTracker.Result? = null
    private var filterName = "none"
    private var fps = 0f

    private val p = Paint(Paint.ANTI_ALIAS_FLAG)
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 34f; color = Color.WHITE }
    private val PIP = 300

    /** Called from the camera thread each frame with luminance bytes. */
    fun submit(luma: ByteArray, w: Int, h: Int, r: LockTracker.Result?, filter: String, fps: Float) {
        if (frameW != w || frameH != h) {
            frameW = w; frameH = h; framePx = IntArray(w * h)
            frameBmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        }
        val px = framePx
        for (i in 0 until w * h) {
            val v = luma[i].toInt() and 0xFF
            px[i] = (0xFF shl 24) or (v shl 16) or (v shl 8) or v
        }
        frameBmp?.setPixels(px, 0, w, 0, 0, w, h)
        pipBmp = r?.crop?.let { grayToBitmap(it) }
        result = r; filterName = filter; this.fps = fps
        postInvalidate()
    }

    /** Map a touch point (view px) back to full-frame px, undoing the letterbox.
     *  Returns null if no frame has arrived yet. */
    fun viewToFrame(vx: Float, vy: Float): Pair<Float, Float>? {
        if (frameW == 0) return null
        val s = minOf(width.toFloat() / frameW, height.toFloat() / frameH)
        val ox = (width - frameW * s) / 2f; val oy = (height - frameH * s) / 2f
        return ((vx - ox) / s) to ((vy - oy) / s)
    }

    private fun grayToBitmap(g: GrayFrame): Bitmap {
        val px = IntArray(g.w * g.h)
        for (i in px.indices) {
            val v = g.d[i].toInt().coerceIn(0, 255)
            px[i] = (0xFF shl 24) or (v shl 16) or (v shl 8) or v
        }
        return Bitmap.createBitmap(px, g.w, g.h, Bitmap.Config.ARGB_8888)
    }

    override fun onDraw(canvas: Canvas) {
        val bmp = frameBmp
        if (bmp == null) {
            canvas.drawColor(Color.BLACK)
            canvas.drawText("waiting for USB camera…", 40f, height / 2f, text)
            return
        }
        // Letterbox the feed into the view.
        val s = minOf(width.toFloat() / frameW, height.toFloat() / frameH)
        val dw = frameW * s; val dh = frameH * s
        val ox = (width - dw) / 2f; val oy = (height - dh) / 2f
        canvas.drawBitmap(bmp, Rect(0, 0, frameW, frameH),
            RectF(ox, oy, ox + dw, oy + dh), null)

        fun fx(x: Float) = ox + x * s
        fun fy(y: Float) = oy + y * s

        val r = result
        if (r != null && (r.state == LockTracker.State.LOCKED || r.state == LockTracker.State.COASTING)) {
            val col = when {
                r.state == LockTracker.State.COASTING -> Color.rgb(255, 180, 0)
                r.conf >= 0.5f -> Color.rgb(40, 230, 70)
                else -> Color.rgb(0, 200, 255)
            }
            p.style = Paint.Style.STROKE; p.strokeWidth = 3f; p.color = col
            canvas.drawRect(fx(r.x.toFloat()), fy(r.y.toFloat()),
                fx((r.x + r.w).toFloat()), fy((r.y + r.h).toFloat()), p)
            val ccx = fx((r.x + r.w / 2).toFloat()); val ccy = fy((r.y + r.h / 2).toFloat())
            canvas.drawLine(ccx - 16, ccy, ccx + 16, ccy, p)
            canvas.drawLine(ccx, ccy - 16, ccx, ccy + 16, p)
            // predicted-heading arrow
            p.strokeWidth = 2f
            canvas.drawLine(ccx, ccy, fx(r.predX), fy(r.predY), p)

            // PiP top-right — the tracking window (like the reference footage).
            pipBmp?.let { pip ->
                val px1 = width - PIP - 16f; val py1 = 16f
                canvas.drawBitmap(pip, Rect(0, 0, pip.width, pip.height),
                    RectF(px1, py1, px1 + PIP, py1 + PIP), null)
                p.color = Color.LTGRAY; p.strokeWidth = 2f
                canvas.drawRect(px1, py1, px1 + PIP, py1 + PIP, p)
                p.color = Color.rgb(240, 40, 40); p.style = Paint.Style.FILL
                canvas.drawRect(px1 + PIP / 2 - 4, py1 + PIP / 2 - 4,
                    px1 + PIP / 2 + 4, py1 + PIP / 2 + 4, p)
                canvas.drawText("ZOOM $filterName", px1, py1 + PIP + 30, text)
            }
        }

        val st = r?.state?.name ?: "IDLE"
        val cf = ((r?.conf ?: 0f) * 100).toInt()
        canvas.drawText("$st   conf $cf%   filter $filterName   ${fps.toInt()} fps", 24f, 44f, text)
        canvas.drawText("tap=lock  double-tap=filter  long-press=reset", 24f, height - 24f, text)
    }
}
