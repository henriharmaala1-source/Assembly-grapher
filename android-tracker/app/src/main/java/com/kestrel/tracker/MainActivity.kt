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
import com.kestrel.tracker.track.BlobFinder
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
    private lateinit var texture: TextureView
    private val tracker = LockTracker()
    private var source: FrameSource? = null
    private var displayTexture: SurfaceTexture? = null

    private var srcKind = SrcKind.PHONE
    private val zoomLevels = floatArrayOf(1f, 2f, 4f)   // Camera2 clamps to the sensor max
    private var zoomIdx = 0
    private val camPerm = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted -> if (granted) reallyStart() }

    // FUSE = structure (edge) + colour (chroma) + brightness (luma), each weighted
    // by its own confidence AND agreement with the prediction every frame — so
    // edge helps on structured targets (real-footage feedback) but the per-cue
    // gating stops a scale-drifted edge from dragging the lock. L+C keeps the
    // pure luma+chroma option (best on fast pure-zoom); single chips for A/B.
    private val cueModes = listOf(
        CueMode("FUSE",      listOf(CropFilter.NONE, CropFilter.CHROMA, CropFilter.EDGE)),
        CueMode("L+C",       listOf(CropFilter.NONE, CropFilter.CHROMA)),
        CueMode("off",       listOf(CropFilter.NONE)),
        CueMode("edge",      listOf(CropFilter.EDGE)),
        CueMode("chroma",    listOf(CropFilter.CHROMA)),
        CueMode("threshold", listOf(CropFilter.THRESHOLD)),
    )
    private var filterIdx = 0
    private val needColor get() = cueModes[filterIdx].cues.contains(CropFilter.CHROMA)

    @Volatile private var pendingDesignate: FloatArray? = null
    private var lastMs = 0L
    private var fps = 0f
    private var loggedSize = false
    // frame size the display transform is set for (written UI thread, read cam thread)
    @Volatile private var fitW = 0
    @Volatile private var fitH = 0
    private var lumaBuf = FloatArray(0)

    // Three tap modes: LOCK (precise box at the tap), BLOB (find the distinct
    // blob near the tap — forgiving), MOTION (acquire movers, tap one to lock).
    @Volatile private var mode = TrackMode.LOCK
    private val motionMode get() = mode == TrackMode.MOTION
    private val motion = MotionDetector()
    @Volatile private var lastBlobs: List<MotionDetector.Blob> = emptyList()

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        texture = TextureView(this)
        view = TrackerOverlayView(this)
        val root = FrameLayout(this).apply { addView(texture); addView(view) }  // overlay on top
        setContentView(root)

        tracker.setCues(cueModes[filterIdx].cues)
        view.setFilters(cueModes.map { it.label })
        view.setSrc(srcKind.name)
        view.setMode(mode.name)

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
                    mode = TrackMode.values()[(mode.ordinal + 1) % TrackMode.values().size]
                    if (mode == TrackMode.MOTION) motion.reset()
                    view.setMode(mode.name)
                    return true
                }
                val chip = view.filterButtonAt(e.x, e.y)
                if (chip != null) { filterIdx = chip; tracker.setCues(cueModes[chip].cues); return true }

                val fp = view.viewToFrame(e.x, e.y) ?: return true
                when (mode) {
                    TrackMode.MOTION -> {
                        val b = lastBlobs.firstOrNull {
                            fp.first in it.x.toFloat()..(it.x + it.w).toFloat() &&
                            fp.second in it.y.toFloat()..(it.y + it.h).toFloat()
                        }
                        if (b != null) {
                            pendingDesignate = floatArrayOf(
                                b.x + b.w / 2f, b.y + b.h / 2f, maxOf(b.w, b.h).toFloat())
                            mode = TrackMode.LOCK; view.setMode(mode.name)
                        }
                    }
                    // BLOB: size 0 signals "find the blob near the tap" in onFrame.
                    TrackMode.BLOB -> pendingDesignate = floatArrayOf(fp.first, fp.second, 0f)
                    TrackMode.LOCK -> pendingDesignate = floatArrayOf(fp.first, fp.second, 64f)
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

    /** Aspect-correct the TextureView (fit-center) to match the overlay mapping.
     *  Returns false if the view isn't laid out yet, so the caller retries on the
     *  next frame. Rotation is separate (needs the sensor orientation, logged). */
    private fun fitCenter(pw: Int, ph: Int): Boolean {
        val vw = texture.width; val vh = texture.height
        if (vw == 0 || vh == 0) return false
        val scale = minOf(vw.toFloat() / pw, vh.toFloat() / ph)
        val m = android.graphics.Matrix()
        m.setScale(pw * scale / vw, ph * scale / vh, vw / 2f, vh / 2f)
        texture.setTransform(m)
        return true
    }

    /** Camera-thread callback: NV21 -> GrayFrame -> tracker -> overlay graphics.
     *  (Display is handled by the TextureView directly; this is tracker-only.) */
    private fun onFrame(nv21: ByteArray, w: Int, h: Int) {
        if (!loggedSize) { loggedSize = true; Log.i("MainActivity", "frame ${w}x$h  nv21=${nv21.size}") }
        // Re-fit whenever the frame size changes (source switch, webcam renegotiates
        // resolution) — a stale transform would offset the box from the shown feed.
        if (w != fitW || h != fitH) runOnUiThread { if (fitCenter(w, h)) { fitW = w; fitH = h } }
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
                if (d[2] <= 0f) {   // BLOB mode: snap to the distinct blob near the tap
                    val box = BlobFinder.findBlob(gf, d[0], d[1])
                    if (box != null) tracker.designate(gf, box[0], box[1], box[2])
                    else tracker.designate(gf, d[0], d[1], 64f)
                } else tracker.designate(gf, d[0], d[1], d[2])
                pendingDesignate = null
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

/** Tap mode: precise box, forgiving blob-detect, or movement acquisition. */
private enum class TrackMode { LOCK, BLOB, MOTION }
