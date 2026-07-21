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
    private var fps = 0f

    // Filter selector — tap a chip to change or turn OFF (no cycling required).
    private var filterNames: List<String> = emptyList()
    private var filterIdx = 0
    private val chipRects = ArrayList<RectF>()

    // Mode (LOCK vs MOTION) — top-left button toggles it.
    private var motionMode = false
    private var blobs: List<MotionDetector.Blob> = emptyList()
    private val modeRect = RectF()

    private val p = Paint(Paint.ANTI_ALIAS_FLAG)
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 34f; color = Color.WHITE }
    private val PIP = 300

    /** Register the filter names once (e.g. ["off","stretch","edge",...]). */
    fun setFilters(names: List<String>) { filterNames = names }

    /** If (vx,vy) hit a filter chip, its index; else null (→ treat as a lock tap). */
    fun filterButtonAt(vx: Float, vy: Float): Int? {
        for (i in chipRects.indices) if (chipRects[i].contains(vx, vy)) return i
        return null
    }

    /** True if (vx,vy) hit the mode toggle button. */
    fun modeButtonAt(vx: Float, vy: Float): Boolean = modeRect.contains(vx, vy)

    private fun fname(i: Int) = filterNames.getOrElse(i) { "?" }

    /** Called from the camera thread each frame with an NV21 buffer. */
    fun submit(nv21: ByteArray, w: Int, h: Int, r: LockTracker.Result?, filterIdx: Int,
               fps: Float, motionMode: Boolean, blobs: List<MotionDetector.Blob>) {
        if (frameW != w || frameH != h) {
            frameW = w; frameH = h; framePx = IntArray(w * h)
            frameBmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        }
        nv21ToArgb(nv21, w, h, framePx)
        frameBmp?.setPixels(framePx, 0, w, 0, 0, w, h)
        pipBmp = r?.crop?.let { grayToBitmap(it) }
        result = r; this.filterIdx = filterIdx; this.fps = fps
        this.motionMode = motionMode; this.blobs = blobs
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

    /** NV21 -> ARGB (integer YUV→RGB). Falls back to grey if the buffer is
     *  luma-only. This is the colour feed the operator sees. */
    private fun nv21ToArgb(nv21: ByteArray, w: Int, h: Int, out: IntArray) {
        val n = w * h
        val color = nv21.size >= n + n / 2
        for (j in 0 until h) {
            val uvRow = n + (j shr 1) * w
            val row = j * w
            for (i in 0 until w) {
                val y = nv21[row + i].toInt() and 0xFF
                if (color) {
                    val uv = uvRow + (i and 1.inv())
                    val v = (nv21[uv].toInt() and 0xFF) - 128
                    val u = (nv21[uv + 1].toInt() and 0xFF) - 128
                    val r = (y + (1436 * v shr 10)).coerceIn(0, 255)
                    val g = (y - (352 * u shr 10) - (731 * v shr 10)).coerceIn(0, 255)
                    val b = (y + (1815 * u shr 10)).coerceIn(0, 255)
                    out[row + i] = (0xFF shl 24) or (r shl 16) or (g shl 8) or b
                } else {
                    out[row + i] = (0xFF shl 24) or (y shl 16) or (y shl 8) or y
                }
            }
        }
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

        if (motionMode) {
            // MOTION mode: draw candidate movers; the largest is the primary.
            for ((idx, b) in blobs.withIndex()) {
                val primary = idx == 0
                p.style = Paint.Style.STROKE; p.strokeWidth = if (primary) 3f else 2f
                p.color = if (primary) Color.rgb(60, 220, 255) else Color.rgb(90, 150, 170)
                canvas.drawRect(fx(b.x.toFloat()), fy(b.y.toFloat()),
                    fx((b.x + b.w).toFloat()), fy((b.y + b.h).toFloat()), p)
            }
            canvas.drawText("MOTION   ${blobs.size} movers   ${fps.toInt()} fps", 24f, 44f, text)
            canvas.drawText("tap a mover to lock it   (button: switch mode)", 24f, height - 24f, text)
            drawModeButton(canvas)
            drawFilterChips(canvas)
            return
        }

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

            // Latency-compensated AIM point (where to steer NOW on a ~150ms feed),
            // magenta so it can't be confused with the box centre.
            val ax = fx(r.aimX); val ay = fy(r.aimY)
            p.color = Color.rgb(255, 0, 200); p.strokeWidth = 2f
            canvas.drawLine(ax - 12, ay - 12, ax + 12, ay + 12, p)
            canvas.drawLine(ax - 12, ay + 12, ax + 12, ay - 12, p)

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
                canvas.drawText("ZOOM ${fname(filterIdx)}", px1, py1 + PIP + 30, text)
            }
        }

        val st = r?.state?.name ?: "IDLE"
        val cf = ((r?.conf ?: 0f) * 100).toInt()
        canvas.drawText("$st   conf $cf%   filter ${fname(filterIdx)}   ${fps.toInt()} fps", 24f, 44f, text)
        canvas.drawText("tap target=lock   long-press=reset", 24f, height - 24f, text)

        drawModeButton(canvas)
        drawFilterChips(canvas)
    }

    /** Top-right MODE toggle (LOCK / MOTION). */
    private fun drawModeButton(canvas: Canvas) {
        val label = if (motionMode) "MODE: MOTION" else "MODE: LOCK"
        val bw = text.measureText(label) + 36f
        val x1 = width - bw - 16f; val y1 = 16f; val bh = 58f
        modeRect.set(x1, y1, x1 + bw, y1 + bh)
        p.style = Paint.Style.FILL; p.color = Color.argb(200, 20, 60, 90)
        canvas.drawRoundRect(modeRect, 12f, 12f, p)
        p.style = Paint.Style.STROKE; p.strokeWidth = 2f; p.color = Color.rgb(120, 200, 240)
        canvas.drawRoundRect(modeRect, 12f, 12f, p)
        canvas.drawText(label, x1 + 18f, y1 + bh - 18f, text)
    }

    /** A row of tappable filter chips along the bottom — tap any to switch, tap
     *  "off" to disable filtering. The active chip is highlighted. */
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
