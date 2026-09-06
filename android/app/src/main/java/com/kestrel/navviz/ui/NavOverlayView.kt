package com.kestrel.navviz.ui

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Rect
import android.view.View
import com.kestrel.navviz.NavCore
import com.kestrel.navviz.depth.LockTracker
import com.kestrel.navviz.depth.Openness
import com.kestrel.navviz.depth.RoadFollow
import kotlin.math.cos
import kotlin.math.sin

/**
 * Live nav HUD over the camera:
 *  - a translucent DEPTH HEATMAP (red = near/blocked, green = far/open) so the
 *    depth model is visibly working, not just a thin bar
 *  - a big STEER ARROW from screen centre showing where MoveStopSense wants to
 *    go (relative to the nose), plus a goal marker
 *  - the phase, openness flag/%, per-column openness bar, and a small compass
 *    (yaw / goal / steer bearings)
 */
private const val CAM_HFOV_DEG = 64f   // approx phone back-camera horizontal FoV;
                                       // maps corridorOffset [-1,1] -> a bearing

class NavOverlayView(context: Context) : View(context) {

    private var result: NavCore.Result? = null
    private var open: Openness.Result? = null
    private var roadR: RoadFollow.Result? = null
    private var box: LockTracker.Box? = null
    private var yawDeg = 0f
    private var goalDeg = 0f
    private var status = ""
    private var modeName = "NAV"

    fun setMode(m: String) { modeName = m; invalidate() }

    // Depth heatmap: a small source bitmap rebuilt per depth frame, scaled up
    // at draw time. Reused to avoid per-frame allocation churn.
    private var heat: Bitmap? = null
    // 0 = camera only, 1 = translucent heatmap over camera, 2 = full depth map.
    private var heatMode = 1

    /** Double-tap cycles the heatmap display mode. Returns the new mode's label. */
    fun cycleHeat(): String {
        heatMode = (heatMode + 1) % 3
        invalidate()
        return when (heatMode) { 0 -> "camera"; 1 -> "depth overlay"; else -> "full depth map" }
    }

    private val text = Paint().apply { color = Color.WHITE; textSize = 46f; isAntiAlias = true }
    private val sub = Paint().apply { color = Color.LTGRAY; textSize = 32f; isAntiAlias = true }
    private val stroke = Paint().apply { style = Paint.Style.STROKE; strokeWidth = 4f; isAntiAlias = true }
    private val fill = Paint().apply { style = Paint.Style.FILL; isAntiAlias = true }
    private val heatPaint = Paint().apply { isAntiAlias = false; alpha = 115 }

    /**
     * @param depth row-major [0,1] depth (0 near .. 1 far), or null if none this frame
     */
    fun render(
        r: NavCore.Result, o: Openness.Result?,
        depth: FloatArray?, dw: Int, dh: Int,
        yaw: Float, goal: Float, statusLine: String,
        rr: RoadFollow.Result?, lockBox: LockTracker.Box?,
    ) {
        result = r; open = o; yawDeg = yaw; goalDeg = goal; status = statusLine
        roadR = rr; box = lockBox
        if (depth != null && depth.size == dw * dh) heat = buildHeat(depth, dw, dh)
        invalidate()
    }

    private fun buildHeat(depth: FloatArray, w: Int, h: Int): Bitmap {
        val px = IntArray(w * h)
        for (i in depth.indices) {
            val v = depth[i].coerceIn(0f, 1f)
            val g = (kotlin.math.min(2f * v, 1f) * 255).toInt()
            val red = (kotlin.math.min(2f * (1f - v), 1f) * 255).toInt()
            px[i] = Color.rgb(red, g, 0)
        }
        val b = heat?.takeIf { it.width == w && it.height == h }
            ?: Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        b.setPixels(px, 0, w, 0, 0, w, h)
        return b
    }

    override fun onDraw(canvas: Canvas) {
        val vw = width.toFloat(); val vh = height.toFloat()

        // Mode banner (top-right) — always shown so the active behaviour is clear.
        text.color = Color.rgb(180, 200, 255)
        canvas.drawText(modeName, vw - 150f, 62f, text)

        when (modeName) {
            "ROAD" -> drawRoad(canvas, vw, vh)
            "LOCK" -> drawLock(canvas, vw, vh)
            else   -> drawNav(canvas, vw, vh)
        }

        status.takeIf { it.isNotEmpty() }?.let {
            sub.color = Color.argb(200, 190, 200, 210); canvas.drawText(it, 24f, vh - 26f, sub)
        }
        sub.color = Color.argb(200, 190, 200, 210)
        canvas.drawText("swipe: mode   ${hint()}", 24f, vh - 62f, sub)
    }

    private fun hint() = when (modeName) {
        "ROAD" -> "tap: re-learn road   hold: reset"
        "LOCK" -> "tap: lock target   hold: reset"
        else   -> "tap: goal   2-tap: depth view   hold: reset"
    }

    // ---- NAV: depth heatmap + openness + steer arrows -------------------
    private fun drawNav(canvas: Canvas, vw: Float, vh: Float) {
        if (heatMode != 0) heat?.let {
            heatPaint.alpha = if (heatMode == 2) 255 else 115
            canvas.drawBitmap(it, Rect(0, 0, it.width, it.height), Rect(0, 0, width, height), heatPaint)
        }
        val r = result ?: run {
            sub.color = Color.LTGRAY; canvas.drawText("waiting for camera…", 24f, 60f, sub); return
        }
        val phaseCol = when (r.phase) {
            NavCore.Phase.MOVE -> Color.rgb(90, 230, 90)
            NavCore.Phase.STUCK -> Color.rgb(240, 70, 70)
            NavCore.Phase.SCAN -> Color.rgb(255, 200, 60)
            else -> Color.WHITE
        }
        text.color = phaseCol; canvas.drawText(r.phase.name, 24f, 62f, text)
        sub.color = Color.WHITE
        canvas.drawText("bearing ${r.bearingDeg.toInt()}°  speed ${"%.2f".format(r.speedScale)}", 24f, 104f, sub)
        open?.let { o ->
            val flagCol = when (o.flag) {
                Openness.Flag.USABLE -> Color.rgb(128, 255, 0)
                Openness.Flag.MARGINAL -> Color.rgb(0, 200, 255)
                Openness.Flag.SUSPECT -> Color.rgb(240, 70, 70)
            }
            sub.color = flagCol
            canvas.drawText("depth ${o.flag}  open ${(o.corridorOpen * 100).toInt()}%", 24f, 144f, sub)
            sub.color = Color.LTGRAY
            drawBarStrip(canvas, o.perColumn, 24f, vh - 140f, vw - 48f, 74f)
            val openRel = o.corridorOffset * (CAM_HFOV_DEG / 2f)
            drawBigArrow(canvas, vw / 2f, vh / 2f, openRel,
                minOf(vw, vh) * (0.16f + 0.10f * o.corridorOpen), Color.rgb(0, 200, 255))
        }
        val rel = wrap180(r.bearingDeg - yawDeg)
        val steerCol = if (r.speedScale > 0.01f) Color.rgb(90, 230, 90) else Color.rgb(255, 200, 60)
        drawBigArrow(canvas, vw / 2f, vh / 2f, rel, minOf(vw, vh) * 0.28f, steerCol)
        val steerWord = when { rel > 3f -> "→R ${rel.toInt()}°"; rel < -3f -> "←L ${(-rel).toInt()}°"; else -> "▲straight" }
        val openWord = open?.let {
            val od = it.corridorOffset * (CAM_HFOV_DEG / 2f)
            when { od > 3f -> "→R ${od.toInt()}°"; od < -3f -> "←L ${(-od).toInt()}°"; else -> "▲ahead" }
        } ?: "—"
        sub.color = Color.WHITE
        canvas.drawText("open $openWord   steer $steerWord   goalΔ ${wrap180(goalDeg - yawDeg).toInt()}°", 24f, vh - 168f, sub)
        sub.color = Color.rgb(0, 200, 255); canvas.drawText("● openest", 24f, vh - 100f, sub)
        sub.color = Color.rgb(128, 255, 0); canvas.drawText("        ● steer", 24f, vh - 100f, sub)
    }

    // ---- ROAD: centreline dots + steer-to-centre arrow -----------------
    private fun drawRoad(canvas: Canvas, vw: Float, vh: Float) {
        val rr = roadR
        if (rr == null || !rr.valid) {
            text.color = Color.rgb(255, 200, 60); canvas.drawText("no road", 24f, 62f, text); return
        }
        // Centreline: draw a dot per covered row, mapped to the view.
        fill.color = Color.rgb(255, 220, 0)
        for (y in 0 until rr.h) {
            val cx = rr.centerline[y]; if (cx < 0f) continue
            canvas.drawCircle(cx * vw, (y.toFloat() / rr.h) * vh, 5f, fill)
        }
        // Steer toward centring on the road: offset drives lateral, heading the bend.
        val steer = (rr.offset * (CAM_HFOV_DEG / 2f)) + rr.heading * 15f
        drawBigArrow(canvas, vw / 2f, vh / 2f, steer, minOf(vw, vh) * 0.26f, Color.rgb(255, 220, 0))
        text.color = Color.rgb(255, 220, 0); canvas.drawText("ROAD ${(rr.conf * 100).toInt()}%", 24f, 62f, text)
        sub.color = Color.WHITE
        canvas.drawText("offset ${"%.2f".format(rr.offset)}  bend ${"%.2f".format(rr.heading)}", 24f, 104f, sub)
    }

    // ---- LOCK: tracked target box + status -----------------------------
    private fun drawLock(canvas: Canvas, vw: Float, vh: Float) {
        val b = box
        if (b == null) {
            text.color = Color.LTGRAY; canvas.drawText("tap a target to lock", 24f, 62f, text); return
        }
        val half = b.size / 2f
        val col = if (b.locked) Color.rgb(90, 230, 90) else Color.rgb(255, 200, 60)   // coasting = amber
        stroke.color = col; stroke.strokeWidth = 5f
        canvas.drawRect(b.cx - half, b.cy - half, b.cx + half, b.cy + half, stroke)
        // Crosshair to the target from centre.
        stroke.color = Color.argb(140, Color.red(col), Color.green(col), Color.blue(col)); stroke.strokeWidth = 3f
        canvas.drawLine(vw / 2f, vh / 2f, b.cx, b.cy, stroke)
        text.color = col; canvas.drawText(if (b.locked) "LOCKED" else "COASTING", 24f, 62f, text)
        sub.color = Color.WHITE; canvas.drawText("conf ${(b.conf * 100).toInt()}%", 24f, 104f, sub)
    }

    private fun drawBigArrow(c: Canvas, cx: Float, cy: Float, bearingDeg: Float, len: Float, col: Int) {
        val a = Math.toRadians((bearingDeg - 90f).toDouble())
        val tx = cx + len * cos(a).toFloat(); val ty = cy + len * sin(a).toFloat()
        stroke.color = Color.argb(160, 0, 0, 0); stroke.strokeWidth = 14f
        c.drawLine(cx, cy, tx, ty, stroke)
        stroke.color = col; stroke.strokeWidth = 9f
        c.drawLine(cx, cy, tx, ty, stroke)
        // arrowhead
        fill.color = col
        val ah = Math.toRadians((bearingDeg - 90f).toDouble())
        val h1 = ah + 2.5; val h2 = ah - 2.5; val hl = 34f
        val p = android.graphics.Path().apply {
            moveTo(tx, ty)
            lineTo(tx + hl * cos(h1).toFloat(), ty + hl * sin(h1).toFloat())
            lineTo(tx + hl * cos(h2).toFloat(), ty + hl * sin(h2).toFloat())
            close()
        }
        c.drawPath(p, fill)
    }

    private fun drawBarStrip(c: Canvas, cols: FloatArray, x0: Float, y0: Float, w: Float, h: Float) {
        fill.color = Color.argb(140, 30, 30, 34); c.drawRect(x0, y0, x0 + w, y0 + h, fill)
        val n = cols.size
        for (i in cols.indices) {
            val v = cols[i].coerceIn(0f, 1f)
            val bx = x0 + i / n.toFloat() * w
            fill.color = Color.rgb(
                (kotlin.math.min(2f * (1 - v), 1f) * 255).toInt(),
                (kotlin.math.min(2f * v, 1f) * 255).toInt(), 0)
            c.drawRect(bx, y0 + h - v * h, bx + (w / n) + 1f, y0 + h, fill)
        }
    }

    private fun wrap180(d: Float): Float {
        var x = d; while (x > 180f) x -= 360f; while (x <= -180f) x += 360f; return x
    }
}
