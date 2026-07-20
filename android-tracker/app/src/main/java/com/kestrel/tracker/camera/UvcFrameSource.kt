package com.kestrel.tracker.camera

import android.content.Context
import android.hardware.usb.UsbDevice
import android.util.Log
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
class UvcFrameSource(@Suppress("unused") private val context: Context) : FrameSource {

    private var helper: CameraHelper? = null
    private var onFrame: ((ByteArray, Int, Int) -> Unit)? = null
    @Volatile private var w = 0
    @Volatile private var h = 0

    private val frameCb = object : IFrameCallback {
        override fun onFrame(frame: ByteBuffer) {
            val cb = onFrame ?: return
            val ww = w; val hh = h
            if (ww == 0 || hh == 0) return
            if (frame.capacity() < ww * hh) return
            // Pass the FULL NV21 frame (Y + VU) so the pipeline has colour; Y is
            // the first w*h bytes, chroma follows. GrayFrame.fromNv21 splits it.
            val take = minOf(frame.capacity(), ww * hh * 3 / 2)
            val buf = ByteArray(take)
            frame.position(0)
            frame.get(buf, 0, take)
            cb(buf, ww, hh)
        }
    }

    private val stateCb = object : ICameraHelper.StateCallback {
        override fun onAttach(device: UsbDevice?) {
            device ?: return
            helper?.selectDevice(device)      // pick the dongle that attached
        }
        override fun onDeviceOpen(device: UsbDevice?, isFirstOpen: Boolean) {
            helper?.openCamera()
        }
        override fun onCameraOpen(device: UsbDevice?) {
            helper?.let { hpr ->
                hpr.startPreview()
                hpr.previewSize?.let { w = it.width; h = it.height }
                hpr.setFrameCallback(frameCb, UVCCamera.PIXEL_FORMAT_NV21)
                Log.i("UvcFrameSource", "UVC preview ${w}x$h")
            }
        }
        override fun onCameraClose(device: UsbDevice?) {}
        override fun onDeviceClose(device: UsbDevice?) {}
        override fun onDetach(device: UsbDevice?) { w = 0; h = 0 }
        override fun onCancel(device: UsbDevice?) {}
    }

    override fun start(onFrame: (ByteArray, Int, Int) -> Unit) {
        this.onFrame = onFrame
        helper = CameraHelper().also { it.setStateCallback(stateCb) }
        Log.i("UvcFrameSource", "waiting for UVC dongle attach")
    }

    override fun stop() {
        helper?.closeCamera()
        helper?.release()
        helper = null; onFrame = null
    }
}
