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
}
