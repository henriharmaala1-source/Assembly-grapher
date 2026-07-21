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
 * Draws the colour feed + the tracked box + a top-right zoom PiP of the followed
 * crop. The feed bitmap is built at HALF resolution: the per-pixel YUV→RGB pass
 * runs on the frame-delivery thread, so a full-res conversion throttles the
 * camera (the navviz framerate lesson, again). The tracker still uses full-res
 * luma — only the displayed bitmap is downsampled, drawn scaled.
 */
class TrackerOverlayView(context: Context) : View(context) {

    private var frameBmp: Bitmap? = null
    private var frameW = 0            // full frame size (coordinate mapping)
    private var frameH = 0
    private var dispW = 0             // downsampled display-bitmap size
    private var dispH = 0
    private var dispPx = IntArray(0)

    private var pipBmp: Bitmap? = null
    private var result: LockTracker.Result? = null
    private var fps = 0f

    // Filter selector — tap a chip to change or turn OFF (no cycling required).
    private var filterNames: List<String> = emptyList()
    private var filterIdx = 0
    private val chipRects = ArrayList<RectF>()

    // Mode (LOCK vs MOTION) — top-right button toggles it.
    private var motionMode = false
    private var blobs: List<MotionDetector.Blob> = emptyList()
    private val modeRect = RectF()

    // Camera source (PHONE / UVC) — button below MODE.
    private var srcLabel = "PHONE"
    private val srcRect = RectF()

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

    fun setSrc(label: String) { srcLabel = label }
    /** True if (vx,vy) hit the camera-source toggle button. */
    fun srcButtonAt(vx: Float, vy: Float): Boolean = srcRect.contains(vx, vy)

    private fun fname(i: Int) = filterNames.getOrElse(i) { "?" }

    /** Called from the camera thread each frame with an NV21 buffer. */
    fun submit(nv21: ByteArray, w: Int, h: Int, r: LockTracker.Result?, filterIdx: Int,
               fps: Float, motionMode: Boolean, blobs: List<MotionDetector.Blob>) {
        frameW = w; frameH = h
        // Full-res display now that the tracker is cheap — half-res looked bad and
        // the display was never the real bottleneck (the NCC search was).
        val dw = w; val dh = h
        if (dispW != dw || dispH != dh) {
            dispW = dw; dispH = dh; dispPx = IntArray(dw * dh)
            frameBmp = Bitmap.createBitmap(dw, dh, Bitmap.Config.ARGB_8888)
        }
        nv21ToArgb(nv21, w, h, dispPx, dw, dh)
        frameBmp?.setPixels(dispPx, 0, dw, 0, 0, dw, dh)
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
    /** NV21 -> ARGB at a downsampled (dw×dh) target — samples the source on a
     *  stride so the conversion is 1/(step^2) the work. Grey if luma-only. */
    private fun nv21ToArgb(nv21: ByteArray, w: Int, h: Int, out: IntArray, dw: Int, dh: Int) {
        val n = w * h
        val color = nv21.size >= n + n / 2
        val stepX = w / dw; val stepY = h / dh
        for (dj in 0 until dh) {
            val sy = dj * stepY
            val uvRow = n + (sy shr 1) * w
            val srow = sy * w
            val orow = dj * dw
            for (di in 0 until dw) {
                val sx = di * stepX
                val y = nv21[srow + sx].toInt() and 0xFF
                if (color) {
                    val uv = uvRow + (sx and 1.inv())
                    val v = (nv21[uv].toInt() and 0xFF) - 128
                    val u = (nv21[uv + 1].toInt() and 0xFF) - 128
                    val r = (y + (1436 * v shr 10)).coerceIn(0, 255)
                    val g = (y - (352 * u shr 10) - (731 * v shr 10)).coerceIn(0, 255)
                    val b = (y + (1815 * u shr 10)).coerceIn(0, 255)
                    out[orow + di] = (0xFF shl 24) or (r shl 16) or (g shl 8) or b
                } else {
                    out[orow + di] = (0xFF shl 24) or (y shl 16) or (y shl 8) or y
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
        // Letterbox the feed into the view (bitmap is half-res, drawn scaled up).
        val s = minOf(width.toFloat() / frameW, height.toFloat() / frameH)
        val vw = frameW * s; val vh = frameH * s
        val ox = (width - vw) / 2f; val oy = (height - vh) / 2f
        canvas.drawBitmap(bmp, Rect(0, 0, bmp.width, bmp.height),
            RectF(ox, oy, ox + vw, oy + vh), null)

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

    /** Top-right MODE toggle (LOCK / MOTION) + SRC toggle (PHONE / UVC) below it. */
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

        val sl = "SRC: $srcLabel"
        val sw = text.measureText(sl) + 36f
        val sx = width - sw - 16f; val sy = y1 + bh + 8f
        srcRect.set(sx, sy, sx + sw, sy + bh)
        p.style = Paint.Style.FILL; p.color = Color.argb(200, 70, 45, 20)
        canvas.drawRoundRect(srcRect, 12f, 12f, p)
        p.style = Paint.Style.STROKE; p.strokeWidth = 2f; p.color = Color.rgb(240, 190, 120)
        canvas.drawRoundRect(srcRect, 12f, 12f, p)
        canvas.drawText(sl, sx + 18f, sy + bh - 18f, text)
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
