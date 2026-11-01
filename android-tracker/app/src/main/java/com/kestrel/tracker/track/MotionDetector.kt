package com.kestrel.tracker.track

import kotlin.math.abs

/**
 * Motion-based target acquisition — the cue that works when colour, brightness
 * and texture all fail to separate the target (a car the same colour as the
 * road). A running-average background is subtracted from each frame; what's left
 * is what moved. Connected-components on that mask gives candidate boxes.
 *
 * EGO-MOTION: with `egoComp` on (default), sparse optical flow estimates the
 * camera's own translation each frame and shifts the accumulated background to
 * match before differencing — so only INDEPENDENTLY-moving objects survive, and
 * MOTION acquisition works on a drifting/vibrating drone, not just a still one.
 * (Global-translation model; good at 50–800 m where parallax is small. Turn off
 * for a genuinely fixed camera to save the flow cost.)
 *
 * Pure Kotlin on GrayFrame — unit-testable, ports to the onboard tracker.
 */
class MotionDetector {

    data class Blob(val x: Int, val y: Int, val w: Int, val h: Int, val area: Int)

    var threshold = 16f       // luma diff to count as motion
    var minAreaFrac = 0.0008f // ignore blobs smaller than this fraction of frame
    var learnRate = 0.02f     // background EMA; low so movers aren't absorbed
    var maxBlobs = 12
    var egoComp = true        // cancel camera motion before differencing

    private val W = 160        // work resolution (cheap connected-components)
    private val H = 120
    private var bg: FloatArray? = null
    private var prev: FloatArray? = null              // previous work frame, for flow
    private val flow = OpticalFlow()
    var lastFlow = 0f to 0f; private set              // last ego-motion (dx,dy), work px

    fun reset() { bg = null; prev = null; lastFlow = 0f to 0f }

    fun detect(frame: GrayFrame): List<Blob> {
        val small = frame.cropResample(0f, 0f, frame.w.toFloat(), frame.h.toFloat(), W, H)
        val cur = small.d
        val b = bg
        if (b == null) { bg = cur.copyOf(); prev = cur.copyOf(); return emptyList() }

        // Ego-motion: estimate the camera's own shift and align the background to
        // the current view, so the frame-diff only lights up independent movers.
        if (egoComp) {
            prev?.let { p ->
                val (dx, dy) = flow.estimate(GrayFrame(p, W, H), GrayFrame(cur, W, H))
                lastFlow = dx to dy
                OpticalFlow.shift(b, W, H, dx, dy)
            }
        }

        val mask = BooleanArray(W * H) { abs(cur[it] - b[it]) > threshold }
        for (i in cur.indices) b[i] = (1 - learnRate) * b[i] + learnRate * cur[i]
        prev = cur.copyOf()

        val minArea = (minAreaFrac * W * H).toInt().coerceAtLeast(6)
        val comps = connectedComponents(mask).filter { it[4] >= minArea }

        val sx = frame.w.toFloat() / W; val sy = frame.h.toFloat() / H
        return comps
            .sortedByDescending { it[4] }
            .take(maxBlobs)
            .map { Blob((it[0] * sx).toInt(), (it[1] * sy).toInt(),
                        (it[2] * sx).toInt(), (it[3] * sy).toInt(), it[4]) }
    }

    /** 4-connected flood-fill labelling → list of [x,y,w,h,area] in work coords. */
    private fun connectedComponents(mask: BooleanArray): List<IntArray> {
        val visited = BooleanArray(W * H)
        val out = ArrayList<IntArray>()
        val stack = ArrayDeque<Int>()
        for (start in 0 until W * H) {
            if (!mask[start] || visited[start]) continue
            var minx = W; var miny = H; var maxx = 0; var maxy = 0; var area = 0
            stack.addLast(start); visited[start] = true
            while (stack.isNotEmpty()) {
                val idx = stack.removeLast()
                val x = idx % W; val y = idx / W
                area++
                if (x < minx) minx = x; if (x > maxx) maxx = x
                if (y < miny) miny = y; if (y > maxy) maxy = y
                if (x > 0)     push(mask, visited, stack, idx - 1)
                if (x < W - 1) push(mask, visited, stack, idx + 1)
                if (y > 0)     push(mask, visited, stack, idx - W)
                if (y < H - 1) push(mask, visited, stack, idx + W)
            }
            out.add(intArrayOf(minx, miny, maxx - minx + 1, maxy - miny + 1, area))
        }
        return out
    }

    private fun push(mask: BooleanArray, visited: BooleanArray, stack: ArrayDeque<Int>, ni: Int) {
        if (mask[ni] && !visited[ni]) { visited[ni] = true; stack.addLast(ni) }
    }
}
