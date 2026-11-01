package com.kestrel.tracker.track

import android.util.Log
import org.bytedeco.javacpp.BytePointer
import org.bytedeco.opencv.global.opencv_imgproc.COLOR_YUV2BGR_NV21
import org.bytedeco.opencv.global.opencv_imgproc.cvtColor
import org.bytedeco.opencv.opencv_core.Mat
import org.bytedeco.opencv.opencv_core.Rect
import org.bytedeco.opencv.opencv_tracking.TrackerCSRT

/**
 * CSRT (CSR-DCF) — a discriminative correlation filter, for A/B against
 * [LockTracker] on real footage.
 *
 * Why it is here
 * --------------
 * Measured on the 10-clip battery, mean on-target:
 *
 *     LockTracker (NCC) 70%    KCF 40%    CSRT 88%    ONNX network 83%
 *
 * CSRT beats this project's tracker on eight of ten clips and beats the network
 * on the mean, with no training, no model file and no NPU. The one clip where
 * LockTracker is the best of all is g_occlusion (79% against CSRT's 31%), which
 * is the whole argument for testing rather than switching: CSRT is a better
 * MATCHER, but it is not a tracking SYSTEM. It has no occlusion handling, no
 * coasting, no re-acquisition and no aim-point prediction — all of which live
 * in LockTracker and would sit on top of it.
 *
 * What it is, precisely
 * ---------------------
 * Not a neural network. HOG + Colour Names + greyscale features, with a filter
 * learned online per target by ridge regression in the Fourier domain, plus a
 * spatial reliability map and per-channel reliability weights. It learns — but
 * at runtime, from the operator's designation, with no pretrained weights. That
 * matters here: the class-agnostic property this rig needs is preserved.
 *
 * Cost, measured on a 2.1 GHz x86 core (frame size does NOT matter — it
 * resamples to a fixed template_size; the TARGET box does):
 *
 *     16-28 px box  ~24 ms      48 px+ box  ~150 ms
 *     stripped (HOG only, single scale)  ~15 ms
 *
 * Expect roughly 1.5-2x that on a Cortex-A76. On this phone it should be
 * comfortable; on a Pi it is marginal at 30 fps and would want the parameters
 * trimmed.
 *
 * VERIFICATION STATUS — read before trusting this
 * -----------------------------------------------
 * This file has NOT been compiled or run. There is no Android SDK in the
 * environment it was written in. The algorithm behaviour is measured (the
 * numbers above are real), but the JavaCPP binding, the NV21 conversion and the
 * native library loading are unexercised. If the dependency fails to resolve,
 * deleting this file and the four `org.bytedeco` lines in build.gradle.kts
 * restores the previous build exactly.
 *
 * CSRT is not in the official OpenCV Android AAR — that ships only MIL, GOTURN,
 * DaSiamRPN, Nano and Vit, because CSRT lives in opencv_contrib which the
 * official Android build excludes (verified by unpacking the AAR). The bytedeco
 * artifact does carry it: libopencv_tracking.so, 1.9 MB, arm64-v8a.
 */
class CsrtTracker {

    data class Box(val x: Int, val y: Int, val w: Int, val h: Int)

    private var tracker: TrackerCSRT? = null
    private var yuvMat: Mat? = null
    private var bgrMat: Mat? = null
    private var ptr: BytePointer? = null
    private var box = Rect()

    val hasTarget get() = tracker != null
    var available = true; private set

    fun reset() {
        tracker?.close(); tracker = null
    }

    /** NV21 -> BGR Mat, reusing the native buffers. CSRT wants colour: Colour
     *  Names is one of its feature channels and dropping it measurably hurts. */
    private fun toBgr(nv21: ByteArray, w: Int, h: Int): Mat? {
        return try {
            val need = w * h * 3 / 2
            var p = ptr
            if (p == null || p.capacity() < need) {
                p?.close(); p = BytePointer(need.toLong()); ptr = p
                yuvMat?.close(); yuvMat = Mat(h * 3 / 2, w, org.bytedeco.opencv.global.opencv_core.CV_8UC1, p)
                bgrMat?.close(); bgrMat = Mat()
            }
            p.put(nv21, 0, need)
            val dst = bgrMat!!
            cvtColor(yuvMat!!, dst, COLOR_YUV2BGR_NV21)
            dst
        } catch (t: Throwable) {
            // Missing native library, unsupported ABI, anything: disable rather
            // than take the camera thread down with us. This is an A/B extra,
            // not part of the flight path.
            Log.e("CsrtTracker", "native init failed — disabling CSRT", t)
            available = false
            null
        }
    }

    fun designate(nv21: ByteArray, w: Int, h: Int, cx: Float, cy: Float, size: Float) {
        if (!available) return
        val img = toBgr(nv21, w, h) ?: return
        val s = size.toInt().coerceAtLeast(4)
        box = Rect((cx - size / 2).toInt().coerceIn(0, w - s - 1),
                   (cy - size / 2).toInt().coerceIn(0, h - s - 1), s, s)
        try {
            tracker?.close()
            tracker = TrackerCSRT.create().also { it.init(img, box) }
        } catch (t: Throwable) {
            Log.e("CsrtTracker", "create/init failed — disabling CSRT", t)
            available = false; tracker = null
        }
    }

    /** @return the new box, or null when CSRT reports failure.
     *
     *  Note it has NO confidence score — unlike the ONNX network, which reports
     *  one and is therefore the thing that can drive a confidence gate. CSRT
     *  gives a bare boolean, so it cannot arbitrate between itself and a
     *  fallback; it can only be arbitrated BY something else. */
    fun update(nv21: ByteArray, w: Int, h: Int): Box? {
        val t = tracker ?: return null
        val img = toBgr(nv21, w, h) ?: return null
        return try {
            if (t.update(img, box)) Box(box.x(), box.y(), box.width(), box.height()) else null
        } catch (e: Throwable) {
            null                       // throws when the box walks out of frame
        }
    }
}
