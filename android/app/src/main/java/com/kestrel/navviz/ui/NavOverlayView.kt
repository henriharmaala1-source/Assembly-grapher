package com.kestrel.navviz.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.view.View
import com.kestrel.navviz.NavCore
import com.kestrel.navviz.depth.Openness
import kotlin.math.cos
import kotlin.math.sin

/**
 * Draws the live nav state on top of the camera preview: the phase
 * (SETTLE/THINK/SCAN/MOVE/ARRIVE/STUCK), the openness usability flag, the
 * per-column openness bar strip (same signal the corridor scan consumes), and
 * a radar-style compass showing yaw, the goal bearing, and the controller's
 * chosen steer bearing. Intentionally plain Canvas — no dependency on the exact
 * screen/foldable geometry, so it renders on cover or unfolded.
 */
class NavOverlayView(context: Context) : View(context) {

    private var result: NavCore.Result? = null
    private var open: Openness.Result? = null
    private var yawDeg = 0f
    private var goalDeg = 0f

    private val text = Paint().apply { color = Color.WHITE; textSize = 44f; isAntiAlias = true }
    private val sub = Paint().apply { color = Color.LTGRAY; textSize = 30f; isAntiAlias = true }
    private val stroke = Paint().apply { style = Paint.Style.STROKE; strokeWidth = 4f; isAntiAlias = true }
    private val fill = Paint().apply { style = Paint.Style.FILL; isAntiAlias = true }

    fun render(r: NavCore.Result, o: Openness.Result?, yaw: Float, goal: Float) {
        result = r; open = o; yawDeg = yaw; goalDeg = goal; invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val r = result ?: return
        val w = width.toFloat(); val h = height.toFloat()

        // Phase + flag banner
        val phaseCol = when (r.phase) {
            NavCore.Phase.MOVE -> Color.rgb(90, 230, 90)
            NavCore.Phase.STUCK -> Color.rgb(240, 70, 70)
            NavCore.Phase.SCAN -> Color.rgb(255, 200, 60)
            else -> Color.WHITE
        }
        text.color = phaseCol
        canvas.drawText(r.phase.name, 24f, 60f, text)
        text.color = Color.WHITE
        canvas.drawText("bearing ${r.bearingDeg.toInt()}°  speed ${"%.2f".format(r.speedScale)}",
            24f, 110f, sub)

        open?.let { o ->
            val flagCol = when (o.flag) {
                Openness.Flag.USABLE -> Color.rgb(128, 255, 0)
                Openness.Flag.MARGINAL -> Color.rgb(0, 200, 255)
                Openness.Flag.SUSPECT -> Color.rgb(240, 70, 70)
            }
            sub.color = flagCol
            canvas.drawText("depth ${o.flag}  open ${(o.corridorOpen * 100).toInt()}%  spread ${"%.2f".format(o.spread)}",
                24f, 150f, sub)
            sub.color = Color.LTGRAY
            drawBarStrip(canvas, o.perColumn, 24f, h - 120f, w - 48f, 70f)
        }

        drawRadar(canvas, w - 190f, 190f, 150f, r)
    }

    private fun drawBarStrip(canvas: Canvas, cols: FloatArray, x0: Float, y0: Float, w: Float, h: Float) {
        fill.color = Color.argb(140, 30, 30, 34)
        canvas.drawRect(x0, y0, x0 + w, y0 + h, fill)
        val n = cols.size
        for (i in cols.indices) {
            val v = cols[i].coerceIn(0f, 1f)
            val bx = x0 + i / n.toFloat() * w
            val bh = v * h
            fill.color = Color.rgb(
                (kotlin.math.min(2f * (1 - v), 1f) * 255).toInt(),
                (kotlin.math.min(2f * v, 1f) * 255).toInt(), 0)
            canvas.drawRect(bx, y0 + h - bh, bx + (w / n) + 1f, y0 + h, fill)
        }
    }

    private fun drawRadar(canvas: Canvas, cx: Float, cy: Float, rad: Float, r: NavCore.Result) {
        stroke.color = Color.argb(160, 120, 120, 130)
        canvas.drawCircle(cx, cy, rad, stroke)
        // yaw (white), goal (cyan), steer bearing (green) — bearings are 0=N, y-up.
        drawNeedle(canvas, cx, cy, rad, yawDeg, Color.WHITE, 4f)
        drawNeedle(canvas, cx, cy, rad, goalDeg, Color.rgb(0, 200, 255), 3f)
        drawNeedle(canvas, cx, cy, rad, r.bearingDeg, Color.rgb(128, 255, 0), 5f)
    }

    private fun drawNeedle(canvas: Canvas, cx: Float, cy: Float, rad: Float, bearingDeg: Float, col: Int, wdt: Float) {
        val a = Math.toRadians((bearingDeg - 90f).toDouble())
        stroke.color = col; stroke.strokeWidth = wdt
        canvas.drawLine(cx, cy, cx + rad * cos(a).toFloat(), cy + rad * sin(a).toFloat(), stroke)
    }
}
