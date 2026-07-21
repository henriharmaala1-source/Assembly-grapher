package com.kestrel.tracker.camera

import android.content.Context
import android.graphics.SurfaceTexture
import android.hardware.usb.UsbDevice
import android.util.Log
import android.view.Surface
import com.herohan.uvcapp.CameraHelper
import com.herohan.uvcapp.ICameraHelper
import com.serenegiant.usb.IFrameCallback
import com.serenegiant.usb.UVCCamera
import java.nio.ByteBuffer

/**
 * UVC frame source via herohan/UVCAndroid (com.herohan:UVCAndroid, Maven
 * Central). This phone doesn't expose the capture dongle as a Camera2 external
 * camera, so we open it over USB Host with libuvc — the same path the generic
 * USB-camera app used.
 *
 * We request NV21 preview frames; the first width*height bytes of each frame are
 * the Y (luma) plane — handed straight to the tracker with no colour conversion.
 *
 * API confirmed from the UVCAndroid demo (CameraHelper + ICameraHelper.State-
 * Callback + IFrameCallback). The frame-callback wiring is the most likely spot
 * to need a small tweak against the resolved library version — it's isolated
 * here behind FrameSource, so nothing else is affected.
 */
private const val TAG = "UvcFrameSource"

class UvcFrameSource(@Suppress("unused") private val context: Context) : FrameSource {

    private var helper: CameraHelper? = null
    private var onFrame: ((ByteArray, Int, Int) -> Unit)? = null
    private var displayTex: SurfaceTexture? = null
    @Volatile private var w = 0
    @Volatile private var h = 0

    private var gotFrame = false
    private var reuseBuf = ByteArray(0)          // reused per callback — no per-frame alloc

    private val frameCb = object : IFrameCallback {
        override fun onFrame(frame: ByteBuffer) {
            val cb = onFrame ?: return
            val ww = w; val hh = h
            if (ww == 0 || hh == 0) { Log.w(TAG, "frame but size 0 — previewSize null"); return }
            if (frame.capacity() < ww * hh) return
            if (!gotFrame) { gotFrame = true; Log.i(TAG, "FIRST FRAME ${ww}x$hh cap=${frame.capacity()}") }
            // Full NV21 (Y + VU) so the pipeline has colour. Reuse the buffer;
            // it's consumed synchronously in cb() before the next callback (single
            // callback thread), so allocating ~460KB per frame was pure waste.
            val take = minOf(frame.capacity(), ww * hh * 3 / 2)
            if (reuseBuf.size != take) reuseBuf = ByteArray(take)
            frame.position(0)
            frame.get(reuseBuf, 0, take)
            cb(reuseBuf, ww, hh)
        }
    }

    private val stateCb = object : ICameraHelper.StateCallback {
        override fun onAttach(device: UsbDevice?) {
            Log.i(TAG, "onAttach ${device?.deviceName}")
            device ?: return
            helper?.selectDevice(device)      // pick the dongle that attached
        }
        override fun onDeviceOpen(device: UsbDevice?, isFirstOpen: Boolean) {
            Log.i(TAG, "onDeviceOpen first=$isFirstOpen -> openCamera()")
            helper?.openCamera()
        }
        override fun onCameraOpen(device: UsbDevice?) {
            helper?.let { hpr ->
                val ps = hpr.previewSize
                if (ps != null) { w = ps.width; h = ps.height } else { w = 640; h = 480 }
                // GPU display surface for the dongle preview — sharp, native rate,
                // and off the frame-callback thread that feeds the tracker.
                displayTex?.let { tex ->
                    tex.setDefaultBufferSize(w, h)
                    hpr.addSurface(Surface(tex), false)
                }
                hpr.startPreview()
                hpr.setFrameCallback(frameCb, UVCCamera.PIXEL_FORMAT_NV21)
                Log.i(TAG, "onCameraOpen -> preview ${w}x$h (previewSize=$ps)")
            }
        }
        override fun onCameraClose(device: UsbDevice?) { Log.i(TAG, "onCameraClose") }
        override fun onDeviceClose(device: UsbDevice?) { Log.i(TAG, "onDeviceClose") }
        override fun onDetach(device: UsbDevice?) { Log.i(TAG, "onDetach"); w = 0; h = 0 }
        override fun onCancel(device: UsbDevice?) { Log.w(TAG, "onCancel — USB permission denied?") }
    }

    override fun start(onFrame: (ByteArray, Int, Int) -> Unit, display: SurfaceTexture?) {
        this.onFrame = onFrame
        this.displayTex = display
        helper = CameraHelper().also { it.setStateCallback(stateCb) }
        Log.i(TAG, "start(): waiting for UVC dongle attach")
    }

    override fun stop() {
        helper?.closeCamera()
        helper?.release()
        helper = null; onFrame = null
    }
}
