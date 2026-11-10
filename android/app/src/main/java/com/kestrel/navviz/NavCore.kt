package com.kestrel.navviz

/**
 * Kotlin wrapper over the native move-stop-sense controller (nav_bridge.cpp).
 * Owns one native MoveStopSense instance for a nav session; every field of
 * [update] maps 1:1 to the C++ MssInput, and [Result] to MssOutput. No decision
 * logic lives here — it's a thin marshalling layer over the real algorithm.
 */
class NavCore : AutoCloseable {

    /** Matches the phase index order the bridge's phaseIndex() emits. */
    enum class Phase { SETTLE, THINK, SCAN, MOVE, ARRIVE, STUCK, UNKNOWN }

    data class Result(
        val bearingDeg: Float,   // absolute heading to steer toward (0 = North)
        val speedScale: Float,   // [0,1] forward translation (0 = hover / scan-in-place)
        val yawScan: Boolean,    // rotating in place to look around
        val phase: Phase,
        val wpE: Float,          // committed leg target (for display)
        val wpN: Float,
    )

    private var handle: Long = nativeCreate()

    fun reset() = nativeReset(handle)

    /**
     * One controller tick. Pose (e,n,yawDeg) comes from the pose provider,
     * corridor* from the depth/openness stage, goalBearing from the operator.
     * planValid=false runs the pure-reactive path (no occupancy-grid goal bias);
     * wiring the grid planner in is a documented follow-on (see android/README).
     */
    fun update(
        e: Float, n: Float, yawDeg: Float, speedMs: Float,
        corridorOpen: Float, corridorOffset: Float,
        goalBearing: Float, planValid: Boolean = false, planBearing: Float = 0f,
        dt: Float,
    ): Result {
        val o = nativeUpdate(
            handle, e, n, yawDeg, speedMs, corridorOpen, corridorOffset,
            goalBearing, planValid, planBearing, dt,
        )
        val idx = o[3].toInt()
        val phase = if (idx in 0..5) Phase.values()[idx] else Phase.UNKNOWN
        return Result(o[0], o[1], o[2] != 0f, phase, o[4], o[5])
    }

    override fun close() {
        if (handle != 0L) { nativeDestroy(handle); handle = 0L }
    }

    private external fun nativeCreate(): Long
    private external fun nativeReset(handle: Long)
    private external fun nativeDestroy(handle: Long)
    private external fun nativeUpdate(
        handle: Long,
        e: Float, n: Float, yawDeg: Float, speedMs: Float,
        corridorOpen: Float, corridorOffset: Float,
        goalBearing: Float, planValid: Boolean, planBearing: Float,
        dt: Float,
    ): FloatArray

    companion object {
        init { System.loadLibrary("navviz") }
    }
}
