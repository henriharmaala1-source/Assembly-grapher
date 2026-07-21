package com.kestrel.tracker.camera

import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size

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

    override fun start(onFrame: (ByteArray, Int, Int) -> Unit) {
        this.onFrame = onFrame
        thread = HandlerThread("cam").also { it.start() }
        handler = Handler(thread!!.looper)

        val id = pickCameraId() ?: run { Log.e("Camera2", "no camera found"); return }
        val size = pickSize(id)
        reader = ImageReader.newInstance(size.width, size.height, ImageFormat.YUV_420_888, 2).apply {
            setOnImageAvailableListener({ r -> deliver(r) }, handler)
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
        val surface = reader!!.surface
        @Suppress("DEPRECATION")
        cam.createCaptureSession(listOf(surface), object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(s: CameraCaptureSession) {
                session = s
                val req = cam.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW)
                    .apply { addTarget(surface) }.build()
                s.setRepeatingRequest(req, null, handler)
            }
            override fun onConfigureFailed(s: CameraCaptureSession) {
                Log.e("Camera2", "session configure failed")
            }
        }, handler)
    }

    private fun deliver(r: ImageReader) {
        val img = r.acquireLatestImage() ?: return
        try {
            val w = img.width; val h = img.height
            val plane = img.planes[0]              // Y plane = luminance
            val buf = plane.buffer
            val rowStride = plane.rowStride
            // Emit NV21: fill Y, leave UV neutral (grey). This fallback path is
            // luma-only — colour display / the chroma filter light up on the UVC
            // (dongle) path, which delivers real chroma.
            val nv21 = ByteArray(w * h * 3 / 2)
            if (rowStride == w) {
                buf.get(nv21, 0, w * h)
            } else {
                for (row in 0 until h) {
                    buf.position(row * rowStride)
                    buf.get(nv21, row * w, w)
                }
            }
            for (i in w * h until nv21.size) nv21[i] = 128.toByte()   // neutral chroma
            onFrame?.invoke(nv21, w, h)
        } finally {
            img.close()
        }
    }

    override fun stop() {
        session?.close(); device?.close(); reader?.close()
        thread?.quitSafely()
        session = null; device = null; reader = null; thread = null; onFrame = null
    }
}
