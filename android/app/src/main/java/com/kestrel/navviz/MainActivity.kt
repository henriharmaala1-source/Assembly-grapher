package com.kestrel.navviz

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import android.os.SystemClock
import android.widget.FrameLayout
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
import java.util.concurrent.Executors

/**
 * Stage 1–3 host: CameraX preview -> MiDaS depth (throttled) -> openness ->
 * gyro yaw -> the REAL MoveStopSense (NavCore/JNI) -> overlay. This is the
 * "run the actual nav algorithm on live phone data" loop.
 *
 * ARCore (full 6-DoF, walk-around mapping) is the Stage-3 upgrade documented in
 * android/README — it replaces CameraX as the frame source, so it's a deliberate
 * swap, not wired here, to keep this first build actually runnable.
 *
 * UNBUILT/UNTESTED in this session (no SDK or device available) — every device
 * interaction below follows documented CameraX/TFLite patterns but needs the
 * on-device iterate loop to shake out. Treat as a scaffold, not a finished app.
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
    private var goalBearingDeg = 0f     // operator goal; TODO: wire a UI control

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        previewView = PreviewView(this)
        overlay = NavOverlayView(this)
        setContentView(FrameLayout(this).apply {
            addView(previewView); addView(overlay)
        })

        nav = NavCore()
        pose = GyroPoseProvider(this).also { it.start(); it.recenter() }
        // Depth model load is best-effort: if the asset is missing the app still
        // runs (openness just reports nothing) rather than crashing on launch.
        depth = runCatching { MidasDepth(this) }.getOrNull()

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            == PackageManager.PERMISSION_GRANTED) startCamera()
        else ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
    }

    override fun onRequestPermissionsResult(rc: Int, p: Array<out String>, r: IntArray) {
        super.onRequestPermissionsResult(rc, p, r)
        if (rc == 1 && r.firstOrNull() == PackageManager.PERMISSION_GRANTED) startCamera()
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

            // Depth is the expensive stage — throttle it (~10 Hz), reuse the last
            // openness between depth frames, exactly the cheap/heavy tier split
            // the onboard scheduler uses. corridor* default to "open" until the
            // first depth frame lands.
            var open = lastOpen
            if (depth != null && now - lastDepthMs > 100) {
                val bmp = it.toBitmap()
                if (bmp != null) {
                    val d = depth!!.infer(bmp)
                    open = Openness.analyze(d, depth!!.inW, depth!!.inH)
                    lastOpen = open
                    lastDepthMs = now
                }
            }

            val dt = if (lastTickMs == 0L) 0.05f else (now - lastTickMs) / 1000f
            lastTickMs = now

            val corridorOpen = open?.corridorOpen ?: 1f
            val corridorOffset = open?.corridorOffset ?: 0f
            val result = nav.update(
                e = pose.e, n = pose.n, yawDeg = pose.yawDeg, speedMs = 0f,
                corridorOpen = corridorOpen, corridorOffset = corridorOffset,
                goalBearing = goalBearingDeg, dt = dt,
            )
            overlay.post { overlay.render(result, open, pose.yawDeg, goalBearingDeg) }
        }
    }

    @Volatile private var lastOpen: Openness.Result? = null

    override fun onDestroy() {
        super.onDestroy()
        pose.stop(); depth?.close(); nav.close(); analysisExecutor.shutdown()
    }
}

/** Minimal YUV_420_888 -> ARGB Bitmap. TODO(device): verify against the real
 *  ImageProxy format the camera delivers; some devices need a rotation apply. */
private fun ImageProxy.toBitmap(): Bitmap? = runCatching {
    val yBuffer = planes[0].buffer
    val vuBuffer = planes[2].buffer
    val ySize = yBuffer.remaining(); val vuSize = vuBuffer.remaining()
    val nv21 = ByteArray(ySize + vuSize)
    yBuffer.get(nv21, 0, ySize); vuBuffer.get(nv21, ySize, vuSize)
    val yuv = android.graphics.YuvImage(
        nv21, android.graphics.ImageFormat.NV21, width, height, null)
    val out = java.io.ByteArrayOutputStream()
    yuv.compressToJpeg(android.graphics.Rect(0, 0, width, height), 85, out)
    val bytes = out.toByteArray()
    android.graphics.BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
}.getOrNull()
