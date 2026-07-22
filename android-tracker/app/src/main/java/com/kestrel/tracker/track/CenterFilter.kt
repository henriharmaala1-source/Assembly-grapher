package com.kestrel.tracker.track

/**
 * Alpha-beta filter on the target centre — a lightweight constant-velocity
 * estimator (the practical equivalent of a Kalman [x,y,vx,vy] for this use).
 * Gives a smoothed position + velocity so the tracker can predict where the
 * target is heading and coast through a few missed frames. Pure Kotlin, no deps.
 */
class CenterFilter(private val alpha: Float = 0.5f, private val beta: Float = 0.15f) {

    var x = 0f; private set
    var y = 0f; private set
    var vx = 0f; private set
    var vy = 0f; private set
    var initialized = false; private set

    fun start(px: Float, py: Float) {
        x = px; y = py; vx = 0f; vy = 0f; initialized = true
    }

    /** Advance one step with no measurement (coast). Returns predicted (x,y). */
    fun predict(): Pair<Float, Float> {
        x += vx; y += vy
        return x to y
    }

    /** Fold in a measurement after predict(). Returns filtered (x,y). */
    fun correct(mx: Float, my: Float): Pair<Float, Float> {
        val rx = mx - x; val ry = my - y
        x += alpha * rx; y += alpha * ry
        vx += beta * rx; vy += beta * ry
        return x to y
    }

    /** Where the centre is projected `steps` frames ahead. */
    fun project(steps: Int): Pair<Float, Float> = (x + vx * steps) to (y + vy * steps)

    /** Cap the per-frame velocity. A noisy peak can inject a huge residual into
     *  `correct()`, and the constant-velocity prediction then compounds it frame
     *  after frame until the crop flies off the target ("box wanders away"). A
     *  ceiling (sized from the target) keeps a real fast target moving while
     *  stopping the runaway. */
    fun clampSpeed(maxV: Float) {
        val s = kotlin.math.sqrt(vx * vx + vy * vy)
        if (s > maxV && s > 1e-6f) { val k = maxV / s; vx *= k; vy *= k }
    }

    /** Bleed off velocity while coasting so a lost target's crop decelerates to a
     *  stop near the last sighting instead of sailing out of frame on stale speed. */
    fun decay(f: Float) { vx *= f; vy *= f }

    val speed: Float get() = kotlin.math.sqrt(vx * vx + vy * vy)
}
