package com.kestrel.tracker

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.os.Build
import android.os.Bundle
import android.os.SystemClock
import android.util.Log
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.Surface
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
 * MOTION mode). Buttons top-right: MODE, SRC, ZOOM, ROT (cycles the display
 * rotation until the feed is upright). Cue chips bottom. LONG-PRESS reset.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var view: TrackerOverlayView
    private lateinit var texture: TextureView
    private val tracker = LockTracker()
    private var source: FrameSource? = null
    private var displayTexture: SurfaceTexture? = null
    // rotationDeg = auto (sensor - display) + rotOffset. Recomputed every re-fit so
    // it tracks device rotation; rotOffset is the user's ROT-button correction and
    // PERSISTS across rotations and source switches (it used to be overwritten on
    // every reallyStart(), so a manual fix never survived a SRC toggle).
    private var rotationDeg = 0
    private var rotOffset = 0

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
    private var tBuildMs = 0f
    private var tTrackMs = 0f
    private var loggedSize = false
    // frame size the display transform is set for (written UI thread, read cam thread)
    @Volatile private var fitW = 0
    @Volatile private var fitH = 0
    // PING-PONG frame buffers. Two sets, alternated every frame, because
    // LockTracker keeps `prevFrame` for the ego-motion flow: with a single shared
    // buffer prev.d and cur.d are the SAME array, so flow compares a frame against
    // itself and always reports zero motion (P1-A was silently dead in luma mode).
    // Reusing them also removes GrayFrame.fromNv21's 3.5 MiB/frame of allocation,
    // which was ~105 MiB/s of garbage on the camera thread and the real reason the
    // frame rate collapsed from 31 to 7 fps the moment a target was locked.
    private val bufD = arrayOf(FloatArray(0), FloatArray(0))
    private val bufU = arrayOf(FloatArray(0), FloatArray(0))
    private val bufV = arrayOf(FloatArray(0), FloatArray(0))
    private var bufIdx = 0

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
                if (view.rotButtonAt(e.x, e.y)) {
                    rotOffset = (rotOffset + 90) % 360          // persistent correction on top of auto
                    invalidateTransform()                       // re-fit picks it up next frame
                    return true
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
            override fun onLongPress(e: MotionEvent) {
                // Long-press ROT clears the manual correction. The offset persists
                // across rotations and source switches by design, which means a
                // correction made while the AUTO value was wrong keeps overriding
                // it after the auto value is fixed — exactly the stale "off 270"
                // that turned a correct rot=0 into a 16%-of-screen strip. There has
                // to be a way back to zero that isn't cycling four times.
                if (view.rotButtonAt(e.x, e.y)) { rotOffset = 0; invalidateTransform(); return }
                tracker.reset(); motion.reset()
            }
        })
        view.setOnTouchListener { _, ev -> gestures.onTouchEvent(ev); true }

        // Start the camera once the display surface exists.
        texture.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) {
                displayTexture = st; startSource()
            }
            override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {
                // TextureView just overwrote the SurfaceTexture's default buffer
                // size with the VIEW size (it does that in onSizeChanged, right
                // before calling us). Put the camera's own preview size back,
                // otherwise the buffer aspect stops matching the tracker frame and
                // the picture is squashed after a rotation.
                source?.onDisplayViewResized()
                invalidateTransform()          // view resized (rotation) -> re-fit
            }
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
        invalidateTransform()              // fitCenter() recomputes rotation for the new source
    }

    /** Device rotated. The activity survives (configChanges) so the camera keeps
     *  streaming; we only need to recompute the preview transform for the new
     *  window shape and display rotation. */
    override fun onConfigurationChanged(newConfig: android.content.res.Configuration) {
        super.onConfigurationChanged(newConfig)
        invalidateTransform()
    }

    /** Force the display transform + rotation to be recomputed on the next frame. */
    private fun invalidateTransform() { fitW = 0; fitH = 0 }

    /** Current rotation of the DEVICE display relative to its natural orientation. */
    private fun displayRotationDeg(): Int {
        @Suppress("DEPRECATION")
        val r = if (Build.VERSION.SDK_INT >= 30) display?.rotation
                else windowManager.defaultDisplay.rotation
        return when (r) {
            Surface.ROTATION_90  -> 90
            Surface.ROTATION_180 -> 180
            Surface.ROTATION_270 -> 270
            else                 -> 0
        }
    }

    /**
     * How far to rotate the camera frame so it appears upright ON SCREEN.
     *
     * SENSOR_ORIENTATION is defined as the rotation needed when the device is in
     * its NATURAL orientation, and the activity is pinned to portrait — which on a
     * phone IS natural — so the display rotation is 0 and this reduces to the
     * sensor constant. The subtraction is kept rather than hard-coded because the
     * value is then still correct if the window is ever resized/rotated anyway
     * (split screen, foldable), and because it makes the two terms visible in the
     * diagnostics instead of collapsing them into one unexplained number.
     *
     * The UVC dongle is not mounted in the phone at all, so its feed is already
     * upright and gets no rotation.
     */
    private var lastSensor = 0                 // for the on-screen geometry read-out
    private var lastDisp = 0

    private fun autoRotationDeg(src: FrameSource?): Int {
        lastDisp = displayRotationDeg()
        lastSensor = (src as? Camera2FrameSource)?.sensorOrientation ?: 0
        val rot = if (src is Camera2FrameSource) ((lastSensor - lastDisp) % 360 + 360) % 360 else 0
        Log.i("MainActivity", "rotation: sensor=$lastSensor display=$lastDisp -> frame rotation=$rot")
        return rot
    }

    /** Set the TextureView display matrix = the SAME canonical frame→view transform
     *  the overlay draws the box with (rotation + fit-center), composed with the
     *  inverse of the view's default fill so it lands correctly. Because both come
     *  from `frameToView`, the shown image and the box rotate together and stay
     *  aligned. Returns false if the view isn't laid out yet (caller retries). */
    private fun fitCenter(pw: Int, ph: Int): Boolean {
        val vw = texture.width; val vh = texture.height
        if (vw == 0 || vh == 0) return false
        // Recompute the rotation HERE, not once at source start: the display
        // rotation changes when the device is turned, and the activity now
        // survives that (configChanges), so a value cached at start-up goes stale.
        // rotOffset is the user's ROT-button correction and rides on top, so it
        // persists across rotations and source switches.
        rotationDeg = ((autoRotationDeg(source) + rotOffset) % 360 + 360) % 360
        val auto = ((rotationDeg - rotOffset) % 360 + 360) % 360
        runOnUiThread {
            view.setFrameRotation(rotationDeg)
            // Show the manual correction separately — "90" alone hides whether the
            // value came from the sensor or from a stale button press.
            view.setRot(if (rotOffset == 0) "$rotationDeg" else "$auto+$rotOffset")
        }
        val m = TrackerOverlayView.frameToView(rotationDeg, pw, ph, vw, vh)

        // Geometry read-out, shown on screen. Every "is it rotated / squashed /
        // letterboxed" question reduces to these numbers, and a photo of the phone
        // carries them — no cable, no logcat, no inference from a picture of a desk.
        val corners = floatArrayOf(0f, 0f, pw.toFloat(), 0f, pw.toFloat(), ph.toFloat(), 0f, ph.toFloat())
        m.mapPoints(corners)
        var x0 = corners[0]; var x1 = corners[0]; var y0 = corners[1]; var y1 = corners[1]
        for (i in 0 until 4) {
            val x = corners[i * 2]; val y = corners[i * 2 + 1]
            if (x < x0) x0 = x; if (x > x1) x1 = x
            if (y < y0) y0 = y; if (y > y1) y1 = y
        }
        val rw = (x1 - x0).toInt(); val rh = (y1 - y0).toInt()
        val (bw, bh) = source?.displayBufferSize ?: (0 to 0)
        val fill = if (vw > 0 && vh > 0) 100L * rw * rh / (vw.toLong() * vh) else 0
        // An upright frame that is portrait inside a landscape window (or vice
        // versa) is not a preference — it is a CONTRADICTION. The rotation says the
        // device is one way up and the window shape says the other, which can only
        // happen when the window is not following the device. Name it, so the
        // failure is legible from a photo instead of being inferred from a desk.
        val uprightPortrait = (rotationDeg % 180 != 0) == (pw >= ph)
        val windowPortrait = vh > vw
        val bad = if (uprightPortrait != windowPortrait) "  <<MISMATCH" else ""
        // The buffer must share the tracker frame's aspect or the picture is
        // squashed and the box is offset (16:9 is a vertical CROP of 4:3 here).
        val skew = if (bw > 0 && kotlin.math.abs(bw.toFloat() / bh - pw.toFloat() / ph) > 0.02f)
            "  <<ASPECT" else ""
        val diag = "cam ${pw}x$ph ${"%.2f".format(pw.toFloat() / ph)}  " +
            "buf ${bw}x$bh  view ${vw}x$vh  " +
            "sens $lastSensor disp $lastDisp off $rotOffset rot $rotationDeg  " +
            "img ${rw}x$rh ${fill}%$bad$skew"
        Log.i("MainActivity", diag)

        m.preScale(pw.toFloat() / vw, ph.toFloat() / vh)   // undo the default fill: M = frameToView · fill⁻¹
        texture.setTransform(m)
        runOnUiThread { view.setDiag(diag) }
        return true
    }

    /** NV21 -> GrayFrame with NO per-frame array allocation.
     *
     *  Alternates between two buffer sets so the frame the tracker kept as
     *  `prevFrame` is never the one we're overwriting — that aliasing is what made
     *  the ego-motion flow see zero movement. Only the small GrayFrame wrapper is
     *  allocated (a few dozen bytes); the 1.2 MiB planes are reused. */
    private fun buildFrame(nv21: ByteArray, w: Int, h: Int, color: Boolean): GrayFrame {
        val n = w * h
        bufIdx = bufIdx xor 1
        if (bufD[bufIdx].size != n) bufD[bufIdx] = FloatArray(n)
        val d = bufD[bufIdx]
        for (i in 0 until n) d[i] = (nv21[i].toInt() and 0xFF).toFloat()
        if (!color || nv21.size < n + n / 2) return GrayFrame(d, w, h)

        if (bufU[bufIdx].size != n) bufU[bufIdx] = FloatArray(n)
        if (bufV[bufIdx].size != n) bufV[bufIdx] = FloatArray(n)
        val cu = bufU[bufIdx]; val cv = bufV[bufIdx]
        for (j in 0 until h) {
            val uvRow = n + (j shr 1) * w
            val row = j * w
            for (i in 0 until w) {
                val uv = uvRow + (i and 1.inv())            // (i/2)*2; NV21 = V,U
                cv[row + i] = ((nv21[uv].toInt() and 0xFF) - 128).toFloat()
                cu[row + i] = ((nv21[uv + 1].toInt() and 0xFF) - 128).toFloat()
            }
        }
        return GrayFrame(d, w, h, cu, cv)
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
        val tb0 = System.nanoTime()
        val gf = buildFrame(nv21, w, h, wantChroma)
        val tb1 = System.nanoTime()
        tBuildMs = 0.9f * tBuildMs + 0.1f * ((tb1 - tb0) / 1e6f)

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
            val tt0 = System.nanoTime()
            res = tracker.update(gf); lastBlobs = emptyList()
            tTrackMs = 0.9f * tTrackMs + 0.1f * ((System.nanoTime() - tt0) / 1e6f)
        }
        // Cost breakdown on the HUD. `nv` is the NV21→float conversion (scales with
        // the stream size), `flow` the ego-motion full-frame pass (also scales),
        // `trk` the whole tracker update including flow. Anything the three don't
        // account for is delivery/GC/display — which is exactly the distinction
        // needed to tell "too much work" from "stalled waiting on buffers".
        view.setTiming(tBuildMs, tracker.tFlowMs, tTrackMs)
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
