package com.kestrel.navviz

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.graphics.Rect
import android.graphics.YuvImage
import android.os.Bundle
import android.os.SystemClock
import android.view.GestureDetector
import android.view.MotionEvent
import android.widget.FrameLayout
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.kestrel.navviz.depth.LockTracker
import com.kestrel.navviz.depth.MidasDepth
import com.kestrel.navviz.depth.Openness
import com.kestrel.navviz.depth.RoadFollow
import com.kestrel.navviz.pose.GyroPoseProvider
import com.kestrel.navviz.ui.NavOverlayView
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors
import kotlin.math.abs

/**
 * CameraX preview -> mode-gated perception -> overlay, a phone test rig for the
 * kestrel navigation stack. Three swipe-selectable modes:
 *   NAV  — MiDaS depth -> openness -> the REAL MoveStopSense (NavCore/JNI). The
 *          "run the actual Pi controller against live data" case.
 *   ROAD — appearance road-follow (RoadFollow.kt, a Kotlin port of the onboard
 *          road_follow.cpp CIELab follower). Its own steer, not move-stop-sense.
 *   LOCK — click-to-lock target tracking (LockTracker.kt, the NCC template layer
 *          of the onboard tracker; the CSRT/KCF/Kalman layers can't port to Kotlin).
 *
 * Controls: SWIPE left/right cycles mode. TAP = per-mode (NAV: set goal to
 * current heading; ROAD: re-learn the road model; LOCK: designate a target).
 * DOUBLE-TAP (NAV): cycle the depth view. LONG-PRESS: reset everything.
 *
 * ROAD/LOCK are Kotlin reimplementations (like Openness.kt), so they validate
 * the ALGORITHM, not the exact C++ — only MoveStopSense (NAV) is the real code.
 * ARCore 6-DoF is still the deferred Stage-4 upgrade (android/README.md).
 */
class MainActivity : ComponentActivity() {

    /** Which perception+behaviour is active. Swipe left/right to cycle. */
    enum class Mode { NAV, ROAD, LOCK }

    private lateinit var overlay: NavOverlayView
    private lateinit var previewView: PreviewView
    private val analysisExecutor = Executors.newSingleThreadExecutor()

    private lateinit var nav: NavCore
    private var depth: MidasDepth? = null
    private val road = RoadFollow()
    private val lock = LockTracker()
    private lateinit var pose: GyroPoseProvider

    private var lastTickMs = 0L
    private var lastPercMs = 0L

    @Volatile private var mode = Mode.NAV
    // Operator goal (deg, 0 = N in the pose frame). Volatile: written from the
    // UI thread (tap), read from the analysis thread each tick.
    @Volatile private var goalBearingDeg = 0f
    @Volatile private var lastOpen: Openness.Result? = null
    @Volatile private var lastDepth: FloatArray? = null
    @Volatile private var lastRoad: RoadFollow.Result? = null
    @Volatile private var lastBox: LockTracker.Box? = null
    @Volatile private var pendingDesignate: Pair<Float, Float>? = null   // fractional tap point
    @Volatile private var depthStatus = "depth: loading…"

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Locals, not the fields, inside apply{}: the FrameLayout receiver has
        // its own `overlay` property (View.getOverlay) that silently shadows
        // the field there — a real compile error caught by validation.
        val pv = PreviewView(this)
        val ov = NavOverlayView(this)
        previewView = pv
        overlay = ov
        setContentView(FrameLayout(this).apply {
            addView(pv); addView(ov)
        })

        nav = NavCore()
        pose = GyroPoseProvider(this).also { it.start(); it.recenter() }
        // Best-effort model load: any failure -> app still runs (controller sees
        // a fully-open corridor) and the REASON is shown on the HUD, not swallowed.
        depth = runCatching { MidasDepth(this) }
            .onSuccess { depthStatus = "depth: ${it.provider}" }
            .onFailure {
                depthStatus = "depth OFF: ${it.message ?: it.javaClass.simpleName}"
                Toast.makeText(this, depthStatus, Toast.LENGTH_LONG).show()
            }.getOrNull()

        val gestures = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                // Tap means different things per mode. Confirmed (not the first
                // half of a double-tap) so double-tap owns the depth-view toggle.
                when (mode) {
                    Mode.NAV -> {
                        goalBearingDeg = pose.yawDeg
                        toast("goal set: ${goalBearingDeg.toInt()}° (heading)")
                    }
                    Mode.ROAD -> { road.relearn(); toast("road model re-learned") }
                    Mode.LOCK -> {
                        // Hand the fractional tap point to the analysis thread to
                        // designate on the next real (rotated) frame.
                        pendingDesignate = (e.x / overlay.width) to (e.y / overlay.height)
                        toast("locking on…")
                    }
                }
                return true
            }
            override fun onDoubleTap(e: MotionEvent): Boolean {
                toast("view: ${overlay.cycleHeat()}"); return true
            }
            override fun onLongPress(e: MotionEvent) {
                pose.recenter(); nav.reset(); lock.reset(); road.relearn(); goalBearingDeg = 0f
                toast("reset")
            }
            override fun onFling(e1: MotionEvent?, e2: MotionEvent, vx: Float, vy: Float): Boolean {
                if (abs(vx) < abs(vy) || abs(vx) < 800f) return false   // horizontal flings only
                val order = Mode.values()
                mode = order[(mode.ordinal + if (vx < 0) 1 else order.size - 1) % order.size]
                lock.reset()
                overlay.setMode(mode.name)
                toast("mode: ${mode.name}")
                return true
            }
        })
        overlay.setOnTouchListener { _, ev -> gestures.onTouchEvent(ev); true }

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            == PackageManager.PERMISSION_GRANTED) startCamera()
        else ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
    }

    override fun onRequestPermissionsResult(rc: Int, p: Array<String>, r: IntArray) {
        super.onRequestPermissionsResult(rc, p, r)
        if (rc == 1 && r.firstOrNull() == PackageManager.PERMISSION_GRANTED) startCamera()
        else if (rc == 1) Toast.makeText(this, "camera permission is required", Toast.LENGTH_LONG).show()
    }

    private fun startCamera() {
        val future = ProcessCameraProvider.getInstance(this)
        future.addListener({
            val provider = future.get()
            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(previewView.surfaceProvider)
            }
            val analysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build().also { it.setAnalyzer(analysisExecutor, ::analyze) }
            provider.unbindAll()
            provider.bindToLifecycle(
                this, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
        }, ContextCompat.getMainExecutor(this))
    }

    private fun analyze(image: ImageProxy) {
        image.use {
            val now = SystemClock.elapsedRealtime()

            // Heavy perception is throttled (~10 Hz) and MODE-GATED: only the
            // active mode's perception runs, so we never pay for depth in ROAD
            // mode or vice-versa — the cheap/heavy split the Deliberator uses.
            // ROTATION MATTERS: CameraX frames are in sensor (landscape)
            // orientation; rotate upright first or the analysis is sideways.
            if (now - lastPercMs > 100) {
                val bmp = it.toUprightBitmap()
                if (bmp != null) {
                    when (mode) {
                        Mode.NAV -> depth?.let { d ->
                            try {
                                val dep = d.infer(bmp)
                                lastDepth = dep
                                lastOpen = Openness.analyze(dep, d.inW, d.inH)
                                depthStatus = "depth: ${d.provider}"
                            } catch (e: Throwable) {
                                depthStatus = "depth ERR: ${e.message ?: e.javaClass.simpleName}"
                            }
                        }
                        Mode.ROAD -> lastRoad = road.analyze(bmp)
                        Mode.LOCK -> {
                            pendingDesignate?.let { (fx, fy) ->
                                lock.designate(bmp, fx * bmp.width, fy * bmp.height)
                                pendingDesignate = null
                            }
                            lastBox = lock.update(bmp)
                        }
                    }
                    lastPercMs = now
                }
            }

            val dt = if (lastTickMs == 0L) 0.05f else ((now - lastTickMs) / 1000f).coerceIn(0.001f, 0.5f)
            lastTickMs = now

            // The real MoveStopSense controller only drives NAV mode (it steers
            // on the depth corridor). ROAD/LOCK are their own behaviours.
            val open = lastOpen
            val result = nav.update(
                e = pose.e, n = pose.n, yawDeg = pose.yawDeg, speedMs = 0f,
                corridorOpen = open?.corridorOpen ?: 1f,
                corridorOffset = open?.corridorOffset ?: 0f,
                goalBearing = goalBearingDeg, dt = dt,
            )
            val dW = depth?.inW ?: 0; val dH = depth?.inH ?: 0
            overlay.post {
                overlay.render(result, open, lastDepth, dW, dH,
                    pose.yawDeg, goalBearingDeg, depthStatus, lastRoad, lastBox)
            }
        }
    }

    private fun toast(m: String) =
        overlay.post { Toast.makeText(this, m, Toast.LENGTH_SHORT).show() }

    override fun onDestroy() {
        super.onDestroy()
        pose.stop(); depth?.close(); nav.close(); analysisExecutor.shutdown()
    }
}

/**
 * YUV_420_888 -> upright ARGB Bitmap. Stride-aware NV21 repack (plane
 * layouts differ per device; naive buffer concatenation breaks on padded rows),
 * then JPEG round-trip for the colour conversion, then rotate by the frame's
 * reported rotationDegrees so the output is upright regardless of how the
 * phone is held.
 */
private fun ImageProxy.toUprightBitmap(): Bitmap? = runCatching {
    val nv21 = yuv420ToNv21(this)
    val yuv = YuvImage(nv21, ImageFormat.NV21, width, height, null)
    val out = ByteArrayOutputStream()
    yuv.compressToJpeg(Rect(0, 0, width, height), 85, out)
    val bytes = out.toByteArray()
    var bmp = BitmapFactory.decodeByteArray(bytes, 0, bytes.size) ?: return null
    val deg = imageInfo.rotationDegrees
    if (deg != 0) {
        val m = Matrix().apply { postRotate(deg.toFloat()) }
        bmp = Bitmap.createBitmap(bmp, 0, 0, bmp.width, bmp.height, m, true)
    }
    bmp
}.getOrNull()

private fun yuv420ToNv21(image: ImageProxy): ByteArray {
    val w = image.width; val h = image.height
    val nv21 = ByteArray(w * h * 3 / 2)
    val yPlane = image.planes[0]
    var pos = 0
    // Y: copy row by row (rowStride may exceed width on many sensors).
    val yBuf = yPlane.buffer
    for (row in 0 until h) {
        yBuf.position(row * yPlane.rowStride)
        yBuf.get(nv21, pos, w); pos += w
    }
    // Interleaved VU at half resolution, honouring pixel + row strides.
    val uPlane = image.planes[1]; val vPlane = image.planes[2]
    val uBuf = uPlane.buffer; val vBuf = vPlane.buffer
    for (row in 0 until h / 2) {
        for (col in 0 until w / 2) {
            val vIdx = row * vPlane.rowStride + col * vPlane.pixelStride
            val uIdx = row * uPlane.rowStride + col * uPlane.pixelStride
            nv21[pos++] = vBuf.get(vIdx)
            nv21[pos++] = uBuf.get(uIdx)
        }
    }
    return nv21
}
