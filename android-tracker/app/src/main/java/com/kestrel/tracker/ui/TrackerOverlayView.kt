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
import com.kestrel.tracker.track.MotionDetector

/**
 * Transparent overlay drawn ON TOP of the camera TextureView. It renders only
 * vector graphics — the tracked box, the zoom PiP, motion blobs, the HUD and the
 * buttons/chips. The camera image itself is shown by the TextureView behind
 * (GPU, sharp, native rate), so this view does NO per-pixel work and never
 * touches the frame delivery rate.
 *
 * Coordinates: the TextureView stretches the frame to fill the view, so frame→
 * view is a non-uniform scale (sx=width/frameW, sy=height/frameH); viewToFrame
 * inverts it. Tracking runs on the undistorted frame, so only the display shows
 * any aspect stretch.
 */
class TrackerOverlayView(context: Context) : View(context) {

    private var frameW = 0
    private var frameH = 0

    private var pipBmp: Bitmap? = null
    private var result: LockTracker.Result? = null
    private var fps = 0f

    private var filterNames: List<String> = emptyList()
    private var filterIdx = 0
    private val chipRects = ArrayList<RectF>()

    private var motionMode = false
    private var blobs: List<MotionDetector.Blob> = emptyList()
    private val modeRect = RectF()
    private var modeLabel = "LOCK"

    private var srcLabel = "PHONE"
    private val srcRect = RectF()
    private var zoomLabel = "1x"
    private val zoomRect = RectF()

    private val p = Paint(Paint.ANTI_ALIAS_FLAG)
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 34f; color = Color.WHITE }
    private val PIP = 300

    fun setFilters(names: List<String>) { filterNames = names }
    fun filterButtonAt(vx: Float, vy: Float): Int? {
        for (i in chipRects.indices) if (chipRects[i].contains(vx, vy)) return i
        return null
    }
    fun modeButtonAt(vx: Float, vy: Float): Boolean = modeRect.contains(vx, vy)
    fun setMode(label: String) { modeLabel = label }
    fun setSrc(label: String) { srcLabel = label }
    fun srcButtonAt(vx: Float, vy: Float): Boolean = srcRect.contains(vx, vy)
    fun setZoom(label: String) { zoomLabel = label }
    fun zoomButtonAt(vx: Float, vy: Float): Boolean = zoomRect.contains(vx, vy)

    private fun fname(i: Int) = filterNames.getOrElse(i) { "?" }

    /** Called from the camera thread each tracker frame. No image data — the
     *  TextureView shows the feed; this only updates the overlay graphics. */
    fun submit(w: Int, h: Int, r: LockTracker.Result?, filterIdx: Int,
               fps: Float, motionMode: Boolean, blobs: List<MotionDetector.Blob>) {
        frameW = w; frameH = h
        pipBmp = r?.crop?.let { grayToBitmap(it) }
        result = r; this.filterIdx = filterIdx; this.fps = fps
        this.motionMode = motionMode; this.blobs = blobs
        postInvalidate()
    }

    /** Map a touch point (view px) back to frame px (inverts the stretch). */
    fun viewToFrame(vx: Float, vy: Float): Pair<Float, Float>? {
        if (frameW == 0 || width == 0 || height == 0) return null
        return (vx * frameW / width) to (vy * frameH / height)
    }

    /** Crop -> bitmap in COLOUR when the crop carries chroma (YUV->RGB), else grey. */
    private fun grayToBitmap(g: GrayFrame): Bitmap {
        val px = IntArray(g.w * g.h)
        val cu = g.cu; val cv = g.cv
        if (cu != null && cv != null) {
            for (i in px.indices) {
                val y = g.d[i].toInt()
                val u = cu[i]; val v = cv[i]                 // centred (-128..127)
                val r = (y + 1.402f * v).toInt().coerceIn(0, 255)
                val gg = (y - 0.344f * u - 0.714f * v).toInt().coerceIn(0, 255)
                val b = (y + 1.772f * u).toInt().coerceIn(0, 255)
                px[i] = (0xFF shl 24) or (r shl 16) or (gg shl 8) or b
            }
        } else {
            for (i in px.indices) {
                val v = g.d[i].toInt().coerceIn(0, 255)
                px[i] = (0xFF shl 24) or (v shl 16) or (v shl 8) or v
            }
        }
        return Bitmap.createBitmap(px, g.w, g.h, Bitmap.Config.ARGB_8888)
    }

    override fun onDraw(canvas: Canvas) {
        val sx = if (frameW > 0) width.toFloat() / frameW else 1f
        val sy = if (frameH > 0) height.toFloat() / frameH else 1f
        fun fx(x: Float) = x * sx
        fun fy(y: Float) = y * sy

        val r = result
        if (frameW > 0 && motionMode) {
            for ((idx, b) in blobs.withIndex()) {
                val primary = idx == 0
                p.style = Paint.Style.STROKE; p.strokeWidth = if (primary) 3f else 2f
                p.color = if (primary) Color.rgb(60, 220, 255) else Color.rgb(90, 150, 170)
                canvas.drawRect(fx(b.x.toFloat()), fy(b.y.toFloat()),
                    fx((b.x + b.w).toFloat()), fy((b.y + b.h).toFloat()), p)
            }
        } else if (frameW > 0 && r != null &&
                   (r.state == LockTracker.State.LOCKED || r.state == LockTracker.State.COASTING)) {
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
            p.strokeWidth = 2f
            canvas.drawLine(ccx, ccy, fx(r.predX), fy(r.predY), p)
            // latency-compensated aim (magenta ✕)
            val ax = fx(r.aimX); val ay = fy(r.aimY)
            p.color = Color.rgb(255, 0, 200)
            canvas.drawLine(ax - 12, ay - 12, ax + 12, ay + 12, p)
            canvas.drawLine(ax - 12, ay + 12, ax + 12, ay - 12, p)
            // zoom PiP top-right
            pipBmp?.let { pip ->
                val px1 = width - PIP - 16f; val py1 = 16f
                canvas.drawBitmap(pip, Rect(0, 0, pip.width, pip.height),
                    RectF(px1, py1, px1 + PIP, py1 + PIP), null)
                p.color = Color.LTGRAY; p.strokeWidth = 2f
                canvas.drawRect(px1, py1, px1 + PIP, py1 + PIP, p)
                p.color = Color.rgb(240, 40, 40); p.style = Paint.Style.FILL
                canvas.drawRect(px1 + PIP / 2 - 4, py1 + PIP / 2 - 4,
                    px1 + PIP / 2 + 4, py1 + PIP / 2 + 4, p)
                canvas.drawText("ZOOM ${fname(filterIdx)}", px1, py1 + PIP + 30, text)
            }
        }

        // HUD + controls (always shown).
        val hud = if (motionMode) {
            "MOTION   ${blobs.size} movers   ${fps.toInt()} fps"
        } else {
            val st = r?.state?.name ?: "IDLE"
            "$st   conf ${((r?.conf ?: 0f) * 100).toInt()}%   ${fname(filterIdx)}   ${fps.toInt()} fps"
        }
        canvas.drawText(hud, 24f, 44f, text)
        canvas.drawText(
            if (motionMode) "tap a mover to lock" else "tap target=lock   long-press=reset",
            24f, height - 24f, text)
        drawButtons(canvas)
        drawFilterChips(canvas)
    }

    /** Top-right MODE (LOCK/MOTION) + SRC (PHONE/UVC) buttons. */
    private fun drawButtons(canvas: Canvas) {
        val ml = "MODE: $modeLabel"
        val mw = text.measureText(ml) + 36f
        val mx = width - mw - 16f; val my = 16f; val bh = 58f
        modeRect.set(mx, my, mx + mw, my + bh)
        button(canvas, modeRect, ml, mx, my, bh, Color.argb(200, 20, 60, 90), Color.rgb(120, 200, 240))

        val sl = "SRC: $srcLabel"
        val sw = text.measureText(sl) + 36f
        val sxb = width - sw - 16f; val syb = my + bh + 8f
        srcRect.set(sxb, syb, sxb + sw, syb + bh)
        button(canvas, srcRect, sl, sxb, syb, bh, Color.argb(200, 70, 45, 20), Color.rgb(240, 190, 120))

        val zl = "ZOOM: $zoomLabel"
        val zw = text.measureText(zl) + 36f
        val zx = width - zw - 16f; val zy = syb + bh + 8f
        zoomRect.set(zx, zy, zx + zw, zy + bh)
        button(canvas, zoomRect, zl, zx, zy, bh, Color.argb(200, 30, 60, 30), Color.rgb(150, 230, 150))
    }

    private fun button(c: Canvas, rect: RectF, label: String, x: Float, y: Float, bh: Float,
                       fill: Int, stroke: Int) {
        p.style = Paint.Style.FILL; p.color = fill
        c.drawRoundRect(rect, 12f, 12f, p)
        p.style = Paint.Style.STROKE; p.strokeWidth = 2f; p.color = stroke
        c.drawRoundRect(rect, 12f, 12f, p)
        c.drawText(label, x + 18f, y + bh - 18f, text)
    }

    private fun drawFilterChips(canvas: Canvas) {
        chipRects.clear()
        if (filterNames.isEmpty()) return
        val chipH = 58f; val padX = 20f; val gap = 10f
        var x = 24f; val y = height - chipH - 74f
        for (i in filterNames.indices) {
            val label = fname(i)
            val w = text.measureText(label) + padX * 2
            val rect = RectF(x, y, x + w, y + chipH)
            chipRects.add(rect)
            p.style = Paint.Style.FILL
            p.color = if (i == filterIdx) Color.rgb(40, 150, 220) else Color.argb(180, 30, 30, 30)
            canvas.drawRoundRect(rect, 12f, 12f, p)
            p.style = Paint.Style.STROKE; p.strokeWidth = 2f
            p.color = if (i == filterIdx) Color.WHITE else Color.rgb(90, 90, 90)
            canvas.drawRoundRect(rect, 12f, 12f, p)
            canvas.drawText(label, x + padX, y + chipH - 18f, text)
            x += w + gap
        }
    }
}
