package com.kestrel.tracker.camera

import android.content.Context
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.ImageReader
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.Surface

/**
 * Frame source over the platform Camera2 API — no external UVC library.
 *
 * The analog capture dongle enumerates as a Camera2 camera with
 * LENS_FACING_EXTERNAL on phones that support USB video (the same capability the
 * generic USB-camera app used). We prefer that external camera when present, and
 * fall back to the built-in back camera otherwise — so the app always runs, and
 * uses the feed-faithful dongle path whenever it's plugged in.
 *
 * Frames are read as YUV_420_888; the Y plane (row-stride-aware copy) is the
 * luminance the tracker wants — no colour conversion, no JPEG.
 */
class Camera2FrameSource(private val context: Context) : FrameSource {

    private val mgr = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var reader: ImageReader? = null
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var onFrame: ((ByteArray, Int, Int) -> Unit)? = null
    private var displaySurface: Surface? = null
    private var reqBuilder: CaptureRequest.Builder? = null
    private var zoom = 1f
    private var maxZoom = 1f
    private var camId: String? = null

    override fun start(onFrame: (ByteArray, Int, Int) -> Unit, display: SurfaceTexture?) {
        this.onFrame = onFrame
        thread = HandlerThread("cam").also { it.start() }
        handler = Handler(thread!!.looper)

        val id = pickCameraId() ?: run { Log.e("Camera2", "no camera found"); return }
        camId = id
        maxZoom = if (Build.VERSION.SDK_INT >= 30)
            mgr.getCameraCharacteristics(id)
                .get(CameraCharacteristics.CONTROL_ZOOM_RATIO_RANGE)?.upper ?: 1f
        else 1f
        Log.i("Camera2", "max zoom ${maxZoom}x")
        val size = pickSize(id)
        reader = ImageReader.newInstance(size.width, size.height, ImageFormat.YUV_420_888, 2).apply {
            setOnImageAvailableListener({ r -> deliver(r) }, handler)
        }
        // Display surface (GPU preview) sized to the camera output.
        if (display != null) {
            display.setDefaultBufferSize(size.width, size.height)
            displaySurface = Surface(display)
        }
        try {
            mgr.openCamera(id, object : CameraDevice.StateCallback() {
                override fun onOpened(cam: CameraDevice) { device = cam; startSession(cam) }
                override fun onDisconnected(cam: CameraDevice) { cam.close() }
                override fun onError(cam: CameraDevice, error: Int) {
                    Log.e("Camera2", "open error $error"); cam.close()
                }
            }, handler)
        } catch (e: SecurityException) {
            Log.e("Camera2", "CAMERA permission not granted", e)
        }
    }

    /** External camera (the dongle) if present, else back, else the first one. */
    private fun pickCameraId(): String? {
        val ids = mgr.cameraIdList
        var back: String? = null
        for (id in ids) {
            val facing = mgr.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_EXTERNAL) {
                Log.i("Camera2", "using EXTERNAL camera $id (dongle)"); return id
            }
            if (facing == CameraCharacteristics.LENS_FACING_BACK) back = id
        }
        return back ?: ids.firstOrNull()
    }

    private fun pickSize(id: String): Size {
        val map = mgr.getCameraCharacteristics(id)
            .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        val sizes = map?.getOutputSizes(ImageFormat.YUV_420_888)
        // CAP at 640x480: a bigger frame makes every per-pixel pass (colour
        // render, chroma build) proportionally slower for no tracking benefit
        // (the tracker works on a 128px crop regardless). Largest size <= cap;
        // else the smallest offered.
        val cap = 640 * 480
        val chosen = sizes?.filter { it.width * it.height <= cap }?.maxByOrNull { it.width * it.height }
            ?: sizes?.minByOrNull { it.width * it.height }
            ?: Size(640, 480)
        Log.i("Camera2", "preview size ${chosen.width}x${chosen.height}")
        return chosen
    }

    private fun startSession(cam: CameraDevice) {
        // Two targets: the ImageReader (tracker frames) + the display surface
        // (GPU preview). The display path never touches Kotlin/YUV conversion.
        val targets = listOfNotNull(reader!!.surface, displaySurface)
        @Suppress("DEPRECATION")
        cam.createCaptureSession(targets, object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(s: CameraCaptureSession) {
                session = s
                val b = cam.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
                targets.forEach { b.addTarget(it) }
                applyZoom(b)
                reqBuilder = b
                s.setRepeatingRequest(b.build(), null, handler)
            }
            override fun onConfigureFailed(s: CameraCaptureSession) {
                Log.e("Camera2", "session configure failed")
            }
        }, handler)
    }

    private fun applyZoom(b: CaptureRequest.Builder) {
        if (Build.VERSION.SDK_INT >= 30 && maxZoom > 1f)
            b.set(CaptureRequest.CONTROL_ZOOM_RATIO, zoom.coerceIn(1f, maxZoom))
    }

    /** Real sensor zoom (crops the sensor → full-quality magnification). Zooms
     *  both the display surface and the tracker frames together. */
    override fun setZoom(ratio: Float) {
        zoom = ratio
        val s = session ?: return
        val b = reqBuilder ?: return
        applyZoom(b)
        try { s.setRepeatingRequest(b.build(), null, handler) } catch (_: Exception) {}
    }

    private var nv21Buf = ByteArray(0)     // reused per frame — no per-frame alloc

    private fun deliver(r: ImageReader) {
        val img = r.acquireLatestImage() ?: return
        try {
            val w = img.width; val h = img.height
            val ySize = w * h
            if (nv21Buf.size != ySize * 3 / 2) nv21Buf = ByteArray(ySize * 3 / 2)
            val nv21 = nv21Buf

            // Y plane (row-stride aware).
            val yP = img.planes[0]; val yb = yP.buffer; val yRow = yP.rowStride
            if (yRow == w) { yb.get(nv21, 0, ySize) }
            else for (row in 0 until h) { yb.position(row * yRow); yb.get(nv21, row * w, w) }

            // Real chroma: pack the U/V planes into NV21's VU order (colour feed).
            val uP = img.planes[1]; val vP = img.planes[2]
            val ub = uP.buffer; val vb = vP.buffer
            val uRow = uP.rowStride; val uPix = uP.pixelStride
            val vRow = vP.rowStride; val vPix = vP.pixelStride
            var pos = ySize
            for (row in 0 until h / 2) {
                val uBase = row * uRow; val vBase = row * vRow
                for (col in 0 until w / 2) {
                    val vi = vBase + col * vPix; val ui = uBase + col * uPix
                    nv21[pos++] = if (vi < vb.limit()) vb.get(vi) else 128.toByte()   // V
                    nv21[pos++] = if (ui < ub.limit()) ub.get(ui) else 128.toByte()   // U
                }
            }
            onFrame?.invoke(nv21, w, h)
        } finally {
            img.close()
        }
    }

    override fun stop() {
        session?.close(); device?.close(); reader?.close(); displaySurface?.release()
        thread?.quitSafely()
        session = null; device = null; reader = null; displaySurface = null
        thread = null; onFrame = null
    }
}
