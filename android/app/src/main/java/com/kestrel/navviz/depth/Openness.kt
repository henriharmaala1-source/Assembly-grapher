package com.kestrel.navviz.depth

import kotlin.math.abs

/**
 * Kotlin port of desktop/tilt_bench.py's horizon_band_openness() +
 * desktop/spin_map.py's per-column reduction — the SAME math the onboard C++
 * VFH+ corridor scan uses (depth_nav.cpp's "middle 60% band" crop), and the
 * same logic the Python tools already validated against synthetic scenes.
 *
 * Input depth is row-major [h*w], float in [0,1], 0 = near/blocked,
 * 1 = far/open (the convention DepthNav produces after its invert step).
 *
 * Produces exactly the two numbers MoveStopSense consumes:
 *   corridorOpen   — forward-cone clearance in [0,1]
 *   corridorOffset — openest direction in [-1,1] over the FoV
 * plus a usability flag, because a depth model pointed at featureless sky
 * outputs a CONFIDENT flat "everything is far" — a falsely-open corridor, not
 * visible noise — so a flat, high signal is the danger sign, not a clear one.
 */
object Openness {

    /** Width-normalised centre-bias strength for the openest-direction search.
     *  0.1 on the [0,1] openness scale: breaks near-ties toward straight-ahead,
     *  but a clearly-open edge (e.g. 0.9 vs a 0.5 centre) still wins. */
    private const val CENTER_BIAS_K = 0.1f

    enum class Flag { USABLE, MARGINAL, SUSPECT }

    data class Result(
        val corridorOpen: Float,    // forward-cone clearance [0,1]
        val corridorOffset: Float,  // openest direction [-1,1]
        val perColumn: FloatArray,  // openness per image column (for the bar strip)
        val meanOpen: Float,
        val spread: Float,          // low = flat/uninformative
        val fracFar: Float,         // fraction of columns reading "confidently far"
        val flag: Flag,
    )

    /**
     * @param depth  row-major depth, size = width*height, values [0,1]
     * @param width  image columns
     * @param height image rows
     * @param bandFrac fraction cropped off each vertical edge (0.2 == the
     *   onboard code's fixed "middle 60%" band)
     * @param forwardConeCols how many centre columns define "forward" openness
     *   (mirrors corridorFromScan's forward-cone averaging)
     */
    fun analyze(
        depth: FloatArray, width: Int, height: Int,
        bandFrac: Float = 0.2f, forwardConeFrac: Float = 0.15f,
    ): Result {
        val r0 = (height * bandFrac).toInt()
        val r1 = (height * (1f - bandFrac)).toInt().coerceAtLeast(r0 + 1)
        val bandRows = r1 - r0

        val perCol = FloatArray(width)
        for (x in 0 until width) {
            var sum = 0f
            for (y in r0 until r1) sum += depth[y * width + x]
            perCol[x] = sum / bandRows
        }

        // Forward-cone openness: mean of the centre columns (robust-ish; the
        // onboard corridorFromScan uses the 2nd-lowest of a few centre rays, but
        // a centre mean is the right first cut for a wide phone camera).
        val coneHalf = (width * forwardConeFrac / 2f).toInt().coerceAtLeast(1)
        val c = width / 2
        var coneSum = 0f; var coneN = 0
        for (x in (c - coneHalf)..(c + coneHalf)) {
            if (x in 0 until width) { coneSum += perCol[x]; coneN++ }
        }
        val corridorOpen = (coneSum / coneN).coerceIn(0f, 1f)

        // Openest direction with a mild centre bias (break near-ties toward
        // straight-ahead), mapped to [-1,1] across FoV. NOTE: the onboard C++
        // corridorFromScan uses `- 0.01*abs(i - N/2)` where i indexes ~60 ray
        // columns; here we index full IMAGE columns (256+ wide), so that raw
        // constant would be ~6x too strong and pin every decision to centre.
        // Use a width-NORMALISED bias instead (fraction of half-width), small
        // relative to the [0,1] openness range so a genuinely-open edge still
        // wins but a true tie centres.
        val halfW = (width / 2f).coerceAtLeast(1f)
        var bestX = c; var bestScore = -1e9f
        for (x in 0 until width) {
            val score = perCol[x] - CENTER_BIAS_K * (abs(x - c) / halfW)
            if (score > bestScore) { bestScore = score; bestX = x }
        }
        val corridorOffset = if (width <= 1) 0f
            else ((bestX.toFloat() / (width - 1)) * 2f - 1f).coerceIn(-1f, 1f)

        // Stats for the usability flag.
        var mean = 0f; for (v in perCol) mean += v; mean /= width
        var varSum = 0f; for (v in perCol) varSum += (v - mean) * (v - mean)
        val spread = kotlin.math.sqrt(varSum / width)
        var far = 0; for (v in perCol) if (v > 0.85f) far++
        val fracFar = far.toFloat() / width

        val flag = when {
            fracFar > 0.85f && spread < 0.08f -> Flag.SUSPECT  // blown out / sky-like
            spread < 0.04f                    -> Flag.SUSPECT  // flat, no structure
            spread < 0.10f || fracFar > 0.6f  -> Flag.MARGINAL
            else                              -> Flag.USABLE
        }

        return Result(corridorOpen, corridorOffset, perCol, mean, spread, fracFar, flag)
    }
}
