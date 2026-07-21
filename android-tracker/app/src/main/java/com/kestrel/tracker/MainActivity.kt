package com.kestrel.tracker

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.os.Bundle
import android.os.SystemClock
import android.util.Log
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.TextureView
import android.widget.FrameLayout
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import com.kestrel.tracker.camera.Camera2FrameSource
import com.kestrel.tracker.camera.FrameSource
import com.kestrel.tracker.camera.UvcFrameSource
import com.kestrel.tracker.track.CropFilter
import com.kestrel.tracker.track.GrayFrame
import com.kestrel.tracker.track.LockTracker
import com.kestrel.tracker.track.MotionDetector
import com.kestrel.tracker.ui.TrackerOverlayView
import kotlin.math.max

/**
 * Kestrel Tracker — a portable lock-on test rig for the Pi.
 *
 * Display: the camera renders straight to a TextureView (GPU, sharp, native
 * rate) with the tracker overlay drawn transparently on top. The tracker gets
 * its own frame stream (ImageReader / UVC callback) — so display and tracking
 * run on separate paths and never throttle each other.
 *
 * Feed: SRC button toggles PHONE (built-in camera) or UVC (the analog dongle,
 * feed-faithful to the Pi's sensor chain). Controls: TAP = lock (or a mover in
 * MOTION mode). Buttons top-right: MODE, SRC. Cue chips bottom. LONG-PRESS reset.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var view: TrackerOverlayView
    private val tracker = LockTracker()
    private var source: FrameSource? = null
    private var displayTexture: SurfaceTexture? = null

    private var srcKind = SrcKind.PHONE
    private val zoomLevels = floatArrayOf(1f, 2f, 4f)   // Camera2 clamps to the sensor max
    private var zoomIdx = 0
    private val camPerm = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted -> if (granted) reallyStart() }

    // FUSE = luma + chroma (both scale-robust) — the simulation-validated default.
    // edge is a scale-fragile specialist kept as a single-cue chip, not in FUSE.
    private val cueModes = listOf(
        CueMode("FUSE",      listOf(CropFilter.NONE, CropFilter.CHROMA)),
        CueMode("off",       listOf(CropFilter.NONE)),
        CueMode("edge",      listOf(CropFilter.EDGE)),
        CueMode("chroma",    listOf(CropFilter.CHROMA)),
        CueMode("sharpen",   listOf(CropFilter.SHARPEN)),
        CueMode("threshold", listOf(CropFilter.THRESHOLD)),
    )
    private var filterIdx = 0
    private val needColor get() = cueModes[filterIdx].cues.contains(CropFilter.CHROMA)

    @Volatile private var pendingDesignate: FloatArray? = null
    private var lastMs = 0L
    private var fps = 0f
    private var loggedSize = false
    private var lumaBuf = FloatArray(0)

    @Volatile private var motionMode = false
    private val motion = MotionDetector()
    @Volatile private var lastBlobs: List<MotionDetector.Blob> = emptyList()

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val texture = TextureView(this)
        view = TrackerOverlayView(this)
        val root = FrameLayout(this).apply { addView(texture); addView(view) }  // overlay on top
        setContentView(root)

        tracker.setCues(cueModes[filterIdx].cues)
        view.setFilters(cueModes.map { it.label })
        view.setSrc(srcKind.name)

        val gestures = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent) = true
            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                if (view.zoomButtonAt(e.x, e.y)) {
                    zoomIdx = (zoomIdx + 1) % zoomLevels.size
                    source?.setZoom(zoomLevels[zoomIdx])
                    view.setZoom("${zoomLevels[zoomIdx].toInt()}x")
                    return true
                }
                if (view.srcButtonAt(e.x, e.y)) {
                    srcKind = if (srcKind == SrcKind.PHONE) SrcKind.UVC else SrcKind.PHONE
                    view.setSrc(srcKind.name); startSource(); return true
                }
                if (view.modeButtonAt(e.x, e.y)) {
                    motionMode = !motionMode
                    if (motionMode) motion.reset()
                    return true
                }
                val chip = view.filterButtonAt(e.x, e.y)
                if (chip != null) { filterIdx = chip; tracker.setCues(cueModes[chip].cues); return true }

                val fp = view.viewToFrame(e.x, e.y) ?: return true
                if (motionMode) {
                    val b = lastBlobs.firstOrNull {
                        fp.first in it.x.toFloat()..(it.x + it.w).toFloat() &&
                        fp.second in it.y.toFloat()..(it.y + it.h).toFloat()
                    }
                    if (b != null) {
                        pendingDesignate = floatArrayOf(
                            b.x + b.w / 2f, b.y + b.h / 2f, maxOf(b.w, b.h).toFloat())
                        motionMode = false
                    }
                } else {
                    pendingDesignate = floatArrayOf(fp.first, fp.second, 64f)
                }
                return true
            }
            override fun onDoubleTap(e: MotionEvent): Boolean {
                filterIdx = (filterIdx + 1) % cueModes.size
                tracker.setCues(cueModes[filterIdx].cues); return true
            }
            override fun onLongPress(e: MotionEvent) { tracker.reset(); motion.reset() }
        })
        view.setOnTouchListener { _, ev -> gestures.onTouchEvent(ev); true }

        // Start the camera once the display surface exists.
        texture.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) {
                displayTexture = st; startSource()
            }
            override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
            override fun onSurfaceTextureDestroyed(st: SurfaceTexture) = true
            override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
        }
    }

    private fun startSource() {
        source?.stop(); source = null
        if (srcKind == SrcKind.PHONE &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED) {
            camPerm.launch(Manifest.permission.CAMERA); return
        }
        reallyStart()
    }

    private fun reallyStart() {
        val src = when (srcKind) {
            SrcKind.PHONE -> Camera2FrameSource(this)
            SrcKind.UVC   -> UvcFrameSource(this)
        }
        source = src
        src.start(::onFrame, displayTexture)
        src.setZoom(zoomLevels[zoomIdx])   // reapply zoom across source switches
    }

    /** Camera-thread callback: NV21 -> GrayFrame -> tracker -> overlay graphics.
     *  (Display is handled by the TextureView directly; this is tracker-only.) */
    private fun onFrame(nv21: ByteArray, w: Int, h: Int) {
        if (!loggedSize) { loggedSize = true; Log.i("MainActivity", "frame ${w}x$h  nv21=${nv21.size}") }
        val now = SystemClock.elapsedRealtime()
        if (lastMs != 0L) fps = 0.9f * fps + 0.1f * (1000f / max(1L, now - lastMs))
        lastMs = now

        val wantChroma = !motionMode && needColor && (tracker.hasTarget || pendingDesignate != null)
        val gf = if (wantChroma) {
            GrayFrame.fromNv21(nv21, w, h)
        } else {
            if (lumaBuf.size != w * h) lumaBuf = FloatArray(w * h)
            for (i in 0 until w * h) lumaBuf[i] = (nv21[i].toInt() and 0xFF).toFloat()
            GrayFrame(lumaBuf, w, h)
        }

        val res: LockTracker.Result?
        if (motionMode) {
            lastBlobs = motion.detect(gf); res = null
        } else {
            pendingDesignate?.let { d ->
                tracker.designate(gf, d[0], d[1], d[2]); pendingDesignate = null
            }
            res = tracker.update(gf); lastBlobs = emptyList()
        }
        view.submit(w, h, res, filterIdx, fps, motionMode, lastBlobs)
    }

    override fun onDestroy() {
        super.onDestroy()
        source?.stop()
    }
}

/** A selectable tracking cue set: one channel (A/B testing) or several (fusion). */
private data class CueMode(val label: String, val cues: List<CropFilter>)

/** Camera source: the phone's built-in camera or the USB (UVC) dongle. */
private enum class SrcKind { PHONE, UVC }
