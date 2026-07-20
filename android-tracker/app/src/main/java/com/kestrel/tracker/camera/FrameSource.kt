package com.kestrel.tracker.camera

/**
 * Abstraction over "where luminance frames come from". The tracker, overlay and
 * activity depend only on this — so swapping the analog dongle for the phone's
 * built-in camera (a fallback FrameSource) or a test pattern touches nothing
 * else. The whole app is luminance-only (the feed is effectively mono), so a
 * source just delivers a Y-plane byte array + dimensions.
 */
interface FrameSource {
    /** onLuma(luma, width, height): luma is width*height bytes, row-major Y. */
    fun start(onLuma: (ByteArray, Int, Int) -> Unit)
    fun stop()
}
