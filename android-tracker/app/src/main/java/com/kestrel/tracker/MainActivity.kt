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

    private val filters = CropFilter.values()
    private var filterIdx = 0

    @Volatile private var pendingTap: Pair<Float, Float>? = null
    private var lastMs = 0L
    private var fps = 0f

    @SuppressLint("ClickableViewAccessibility")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        view = TrackerOverlayView(this)
        setContentView(view)
        tracker.filter = filters[filterIdx]
        // Chip labels; NONE shows as "off" so turning filtering off is one tap.
        view.setFilters(filters.map { if (it == CropFilter.NONE) "off" else it.name.lowercase() })

        val gestures = GestureDetector(this, object : GestureDetector.SimpleOnGestureListener() {
            override fun onDown(e: MotionEvent) = true    // required to receive the rest
            override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
                // A tap on a filter chip switches/disables the filter; anywhere
                // else designates the target under the finger.
                val chip = view.filterButtonAt(e.x, e.y)
                if (chip != null) {
                    filterIdx = chip
                    tracker.filter = filters[chip]
                } else {
                    pendingTap = view.viewToFrame(e.x, e.y)
                }
                return true
            }
            override fun onDoubleTap(e: MotionEvent): Boolean {
                filterIdx = (filterIdx + 1) % filters.size   // quick cycle, still available
                tracker.filter = filters[filterIdx]
                return true
            }
            override fun onLongPress(e: MotionEvent) { tracker.reset() }
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

        // Only split out chroma when the colour filter needs it — otherwise a
        // cheap luma-only frame keeps the tracker loop fast.
        val gf = if (filters[filterIdx] == CropFilter.CHROMA)
            GrayFrame.fromNv21(nv21, w, h)
        else
            GrayFrame(FloatArray(w * h) { (nv21[it].toInt() and 0xFF).toFloat() }, w, h)

        pendingTap?.let { (fx, fy) ->
            if (fx in 0f..w.toFloat() && fy in 0f..h.toFloat())
                tracker.designate(gf, fx, fy, 64f)
            pendingTap = null
        }

        val res = tracker.update(gf)
        view.submit(nv21, w, h, res, filterIdx, fps)
    }

    override fun onDestroy() {
        super.onDestroy()
        source?.stop()
    }
}
