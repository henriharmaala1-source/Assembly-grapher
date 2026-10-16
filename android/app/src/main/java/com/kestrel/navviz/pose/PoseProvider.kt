package com.kestrel.navviz.pose

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager

/**
 * Supplies the (e, n, yawDeg) pose MoveStopSense needs each tick, in a local ENU
 * frame with the origin at wherever nav started.
 *
 * Two implementations, matching the staged plan (see android/README):
 *
 *  - [GyroPoseProvider]  — rotation only, yaw from the fused rotation-vector
 *    sensor; position stays at origin. Works TODAY with a plain CameraX preview,
 *    no ARCore. This is the SCAN-phase-equivalent testbed: the aircraft's real
 *    SCAN rotates in place, and so does this. Good first running build.
 *
 *  - ArCorePoseProvider — full 6-DoF from ARCore VIO (position + heading), the
 *    upgrade that makes MOVE legs and real walk-around mapping meaningful. It is
 *    a deliberate ARCHITECTURAL change, not a drop-in: ARCore OWNS the camera,
 *    so switching to it means the depth stage reads ARCore's acquired frames
 *    instead of CameraX. Left as a documented Stage-3 step rather than a
 *    half-working stub, so the first build actually runs.
 */
interface PoseProvider {
    /** East, North (metres) and yaw (deg, 0 = North) in the local ENU frame. */
    val e: Float
    val n: Float
    val yawDeg: Float
    fun start()
    fun stop()
    /** Re-zero the frame origin/heading to the current pose (new nav session). */
    fun recenter()
}

/**
 * Yaw-only pose from the rotation-vector sensor. Position is held at origin —
 * honest about what it is: this validates the reactive + SCAN behaviour against
 * real rotation, not translation. Heading is taken relative to the yaw at the
 * last recenter() so "forward at start" reads as 0.
 */
class GyroPoseProvider(context: Context) : PoseProvider, SensorEventListener {
    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rotSensor = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    override var e: Float = 0f; private set
    override var n: Float = 0f; private set
    override var yawDeg: Float = 0f; private set

    private var yawZeroDeg = 0f
    private var lastRawYawDeg = 0f

    override fun start() {
        rotSensor?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }
    override fun stop() = sm.unregisterListener(this)
    override fun recenter() { yawZeroDeg = lastRawYawDeg; e = 0f; n = 0f }

    override fun onSensorChanged(ev: SensorEvent) {
        if (ev.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        val R = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(R, ev.values)
        val orientation = FloatArray(3)
        SensorManager.getOrientation(R, orientation)  // [azimuth, pitch, roll], rad
        val rawYaw = Math.toDegrees(orientation[0].toDouble()).toFloat()  // azimuth, 0=N
        lastRawYawDeg = rawYaw
        yawDeg = wrap180(rawYaw - yawZeroDeg)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun wrap180(d: Float): Float {
        var x = d
        while (x > 180f) x -= 360f
        while (x <= -180f) x += 360f
        return x
    }
}
