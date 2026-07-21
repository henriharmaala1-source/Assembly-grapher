package com.kestrel.tracker

import android.annotation.SuppressLint
import android.os.Bundle
import android.os.SystemClock
import android.view.GestureDetector
import android.view.MotionEvent
import androidx.appcompat.app.AppCompatActivity
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
 * Feed: the analog capture dongle over USB-OTG (UvcFrameSource) — the SAME
 * sensor path the Pi will fly (30 fps, ~150 ms, interlaced), so what locks here
 * locks there, as long as the tracker stays lean enough for the Pi to run at
 * 30 fps — which this one is (pure NCC-in-a-crop, no model).
 *
 * Controls: TAP = lock the target under your finger. DOUBLE-TAP = cycle the
 * crop filter (none/stretch/edge/threshold/sharpen — test which reveals the
 * target best on this feed). LONG-PRESS = reset.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var view: TrackerOverlayView
    private val tracker = LockTracker()
    private var source: FrameSource? = null

    // Each chip = a fused cue set for the tracker. "FUSE" combines structure +
    // colour + brightness so lock survives when any single cue goes flat.
    private val cueModes = listOf(
        CueMode("off",       listOf(CropFilter.NONE)),
        CueMode("edge",      listOf(CropFilter.EDGE)),
        CueMode("sharpen",   listOf(CropFilter.SHARPEN)),
        CueMode("chroma",    listOf(CropFilter.CHROMA)),
        CueMode("threshold", listOf(CropFilter.THRESHOLD)),
        CueMode("FUSE",      listOf(CropFilter.EDGE, CropFilter.CHROMA, CropFilter.NONE)),
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
                // Priority: mode button, then filter chip, then a target tap.
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

        // USB permission is handled by the UVC library on device attach.
        source = UvcFrameSource(this).also { it.start(::onFrame) }
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
