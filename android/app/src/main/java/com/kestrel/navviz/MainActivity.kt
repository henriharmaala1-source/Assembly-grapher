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
import com.kestrel.navviz.depth.MidasDepth
import com.kestrel.navviz.depth.Openness
import com.kestrel.navviz.pose.GyroPoseProvider
import com.kestrel.navviz.ui.NavOverlayView
import java.io.ByteArrayOutputStream
import java.util.concurrent.Executors

/**
 * Stage 1–3 host: CameraX preview -> MiDaS depth (throttled) -> openness ->
 * gyro yaw -> the REAL MoveStopSense (NavCore/JNI) -> overlay. This is the
 * "run the actual Pi navigation code against live phone data" test rig — the
 * phone stands in for the aircraft's sensors, nothing more.
 *
 * Controls:
 *   TAP        — set the mission goal bearing to the direction the camera is
 *                facing right now ("fly that way"), like the onboard Up-arrow /
 *                N-E-S-W quick-set.
 *   LONG-PRESS — re-zero the pose frame and reset the controller (fresh nav
 *                session; clears a STUCK latch the honest way, via reset).
 *
 * ARCore (full 6-DoF, walk-around MOVE legs) is the Stage-4 upgrade documented
 * in android/README.md — deliberately not wired here so this build runs first.
 */
class MainActivity : ComponentActivity() {

    private lateinit var overlay: NavOverlayView
    private lateinit var previewView: PreviewView
    private val analysisExecutor = Executors.newSingleThreadExecutor()

    private lateinit var nav: NavCore
    private var depth: MidasDepth? = null
    private lateinit var pose: GyroPoseProvider

    private var lastTickMs = 0L
    private var lastDepthMs = 0L

    // Operator goal (deg, 0 = N in the pose frame). Volatile: written from the
    // UI thread (tap), read from the analysis thread each tick.
    @Volatile private var goalBearingDeg = 0f
    @Volatile private var lastOpen: Openness.Result? = null
    @Volatile private var lastDepth: FloatArray? = null
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
                // Confirmed (not the first half of a double-tap) so double-tap can
                // own the depth-view toggle without also re-setting the goal.
                goalBearingDeg = pose.yawDeg
                Toast.makeText(this@MainActivity,
                    "goal set: ${goalBearingDeg.toInt()}° (current heading)", Toast.LENGTH_SHORT).show()
                return true
            }
            override fun onDoubleTap(e: MotionEvent): Boolean {
                val mode = overlay.cycleHeat()
                Toast.makeText(this@MainActivity, "view: $mode", Toast.LENGTH_SHORT).show()
                return true
            }
            override fun onLongPress(e: MotionEvent) {
                pose.recenter(); nav.reset(); goalBearingDeg = 0f
                Toast.makeText(this@MainActivity,
                    "pose re-zeroed, controller reset", Toast.LENGTH_SHORT).show()
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

            // Depth is the expensive stage — throttle it (~10 Hz max) and reuse
            // the last openness between depth frames: the same cheap/heavy tier
            // split the onboard Deliberator uses. corridor* defaults to "open"
            // until the first depth frame lands.
            val d = depth
            if (d != null && now - lastDepthMs > 100) {
                // ROTATION MATTERS: CameraX delivers frames in the SENSOR's
                // orientation (landscape). Without applying rotationDegrees the
                // portrait-held image is sideways and the horizon-band analysis
                // slices vertical stripes — silently garbage. Rotate first.
                val bmp = it.toUprightBitmap()
                if (bmp != null) {
                    try {
                        val dep = d.infer(bmp)
                        lastDepth = dep
                        lastOpen = Openness.analyze(dep, d.inW, d.inH)
                        depthStatus = "depth: ${d.provider}"
                    } catch (e: Throwable) {
                        // Surface an inference failure (e.g. tensor-shape mismatch)
                        // instead of silently killing the analysis loop.
                        depthStatus = "depth ERR: ${e.message ?: e.javaClass.simpleName}"
                    }
                    lastDepthMs = now
                }
            }

            val dt = if (lastTickMs == 0L) 0.05f else ((now - lastTickMs) / 1000f).coerceIn(0.001f, 0.5f)
            lastTickMs = now

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
                    pose.yawDeg, goalBearingDeg, depthStatus)
            }
        }
    }

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
