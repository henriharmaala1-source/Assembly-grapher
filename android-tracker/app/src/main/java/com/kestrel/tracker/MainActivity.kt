package com.kestrel.tracker

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.SystemClock
import android.view.GestureDetector
import android.view.MotionEvent
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
 * Kestrel Tracker — a portable, feed-faithful lock-on test rig for the Pi.
 *
 * Feed: selectable via the SRC button — PHONE (built-in camera, always works,
 * for testing the tracker anywhere) or UVC (the analog capture dongle over
 * USB-OTG, the feed-faithful path that matches the Pi's sensor chain). Defaults
 * to PHONE so the tracker runs out of the box.
 *
 * Controls: TAP = lock (or tap a mover in MOTION mode). Buttons top-right: MODE
 * (LOCK/MOTION) and SRC (PHONE/UVC). Cue chips bottom. LONG-PRESS = reset.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var view: TrackerOverlayView
    private val tracker = LockTracker()
    private var source: FrameSource? = null

    // Camera source — PHONE (built-in, always works) or UVC (the dongle). Default
    // PHONE so the tracker is testable out of the box; SRC button toggles. UVC is
    // the feed-faithful path when the dongle cooperates.
    private var srcKind = SrcKind.PHONE
    private val camPerm = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { granted -> if (granted) reallyStart() }

    // Each chip = a fused cue set for the tracker. "FUSE" combines structure +
    // colour + brightness so lock survives when any single cue goes flat.
    // FUSE = luma + chroma (both scale-robust) — the simulation-validated default
    // (best or tied in every scenario). edge is a scale-fragile SPECIALIST kept
    // as a single-cue chip for same-brightness targets, not in the blend.
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

    // Pending designation [cx, cy, size] in frame coords (from a tap or a blob).
    @Volatile private var pendingDesignate: FloatArray? = null
    private var lastMs = 0L
    private var fps = 0f

    // MOTION mode — acquire targets by movement (works when colour/texture can't).
    @Volatile private var motionMode = false
    private val motion = MotionDetector()
    @Volatile private var lastBlobs: List<MotionDetector.Blob> = emptyList()

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        view = TrackerOverlayView(this)
        setContentView(view)
        tracker.setCues(cueModes[filterIdx].cues)
        view.setFilters(cueModes.map { it.label })

        val gestures = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent) = true    // required to receive the rest
            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                // Priority: source button, mode button, filter chip, then a tap.
                if (view.srcButtonAt(e.x, e.y)) {
                    srcKind = if (srcKind == SrcKind.PHONE) SrcKind.UVC else SrcKind.PHONE
                    view.setSrc(srcKind.name)
                    startSource()
                    return true
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
                    // Tap a mover to lock it: hand its box to the tracker + switch to LOCK.
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
                filterIdx = (filterIdx + 1) % cueModes.size   // quick cycle, still available
                tracker.setCues(cueModes[filterIdx].cues)
                return true
            }
            override fun onLongPress(e: MotionEvent) { tracker.reset(); motion.reset() }
        })
        view.setOnTouchListener { _, ev -> gestures.onTouchEvent(ev); true }

        view.setSrc(srcKind.name)
        startSource()
    }

    /** (Re)start the selected camera source. PHONE needs CAMERA permission; UVC
     *  is handled by the library on attach. Toggling re-registers the UVC monitor,
     *  which also picks up an already-plugged dongle. */
    private fun startSource() {
        source?.stop(); source = null
        if (srcKind == SrcKind.PHONE &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED) {
            camPerm.launch(Manifest.permission.CAMERA)
            return
        }
        reallyStart()
    }

    private fun reallyStart() {
        source = when (srcKind) {
            SrcKind.PHONE -> Camera2FrameSource(this)
            SrcKind.UVC   -> UvcFrameSource(this)
        }.also { it.start(::onFrame) }
    }

    /** Camera-thread callback: NV21 -> GrayFrame -> tracker -> overlay. */
    private fun onFrame(nv21: ByteArray, w: Int, h: Int) {
        val now = SystemClock.elapsedRealtime()
        if (lastMs != 0L) fps = 0.9f * fps + 0.1f * (1000f / max(1L, now - lastMs))
        lastMs = now

        // Split out chroma only when an active cue needs it — otherwise a cheap
        // luma-only frame keeps the tracker loop fast.
        val gf = if (!motionMode && needColor)
            GrayFrame.fromNv21(nv21, w, h)
        else
            GrayFrame(FloatArray(w * h) { (nv21[it].toInt() and 0xFF).toFloat() }, w, h)

        val res: LockTracker.Result?
        if (motionMode) {
            lastBlobs = motion.detect(gf)      // acquire by movement; no lock update
            res = null
        } else {
            pendingDesignate?.let { d ->
                tracker.designate(gf, d[0], d[1], d[2]); pendingDesignate = null
            }
            res = tracker.update(gf)
            lastBlobs = emptyList()
        }
        view.submit(nv21, w, h, res, filterIdx, fps, motionMode, lastBlobs)
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
