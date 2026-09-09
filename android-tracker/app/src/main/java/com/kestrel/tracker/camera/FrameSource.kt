package com.kestrel.tracker.camera

import android.graphics.SurfaceTexture

/**
 * Abstraction over "where frames come from". The tracker, overlay and activity
 * depend only on this.
 *
 * Two outputs per source:
 *  - onFrame(nv21,w,h): NV21 frames for the TRACKER (Y + 2×2-subsampled VU).
 *  - `display`: a SurfaceTexture the camera renders to DIRECTLY (GPU) for a
 *    sharp, full-res preview at native rate — decoupled from the frame-callback
 *    thread, so the display never competes with tracker processing.
 */
interface FrameSource {
    fun start(onFrame: (ByteArray, Int, Int) -> Unit, display: SurfaceTexture?)
    fun stop()
    /** Set optical/sensor zoom ratio (1 = none). No-op for fixed-lens sources. */
    fun setZoom(ratio: Float) {}
    /** The view holding the display SurfaceTexture was resized (device rotation).
     *
     *  TextureView.onSizeChanged calls `setDefaultBufferSize(viewW, viewH)` on the
     *  SurfaceTexture it owns — silently discarding the size we chose for the
     *  preview stream. That never mattered while the activity was orientation-
     *  locked (the view never resized); now that the window follows the device it
     *  happens on every rotation, so the source must re-assert its own size. */
    fun onDisplayViewResized() {}
    /** Resolution of the preview (display) buffer, for the on-screen diagnostics.
     *  0x0 when there is no display surface yet. */
    val displayBufferSize: Pair<Int, Int> get() = 0 to 0
}
