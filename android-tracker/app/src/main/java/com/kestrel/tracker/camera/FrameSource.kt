package com.kestrel.tracker.camera

/**
 * Abstraction over "where frames come from". The tracker, overlay and activity
 * depend only on this — so swapping the analog dongle for the built-in camera or
 * a test pattern touches nothing else.
 *
 * Frames are delivered as NV21 (Y plane + 2×2-subsampled VU): the first
 * width*height bytes are luma (all the tracker needs by default), the rest carry
 * chroma for colour display and the `chroma` filter. A source with no real
 * colour may fill UV neutral (grey).
 */
interface FrameSource {
    /** onFrame(nv21, width, height): NV21 bytes (>= width*height; full is width*height*3/2). */
    fun start(onFrame: (ByteArray, Int, Int) -> Unit)
    fun stop()
}
