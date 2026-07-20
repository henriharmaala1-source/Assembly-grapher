package com.kestrel.tracker.camera

import android.content.Context
import android.util.Log
import com.jiangdg.ausbc.MultiCameraClient
import com.jiangdg.ausbc.callback.ICameraStateCallBack
import com.jiangdg.ausbc.callback.IPreviewDataCallBack
import com.jiangdg.ausbc.camera.CameraUVC
import com.jiangdg.ausbc.camera.bean.CameraRequest

/**
 * UVC (USB-camera) frame source backed by AUSBC (AndroidUSBCamera / libausbc).
 * We use the library ONLY to open the dongle and receive raw preview frames;
 * rendering and tracking are ours. NV21 is requested, whose first width*height
 * bytes are the Y (luma) plane — handed straight to the tracker with no colour
 * conversion (cheap, and the analog/thermal feed is mono anyway).
 *
 * >>> THIS FILE IS THE ONE DEVICE-VERIFY SEAM <<<
 * It couldn't be compile-checked in the build environment. The AUSBC API surface
 * (class/callback names, CameraRequest builder, the preview-format enum) can
 * shift between library versions — if the build complains here, reconcile these
 * calls with the libausbc version resolved by Gradle (see README). Everything
 * else in the app is validated and independent of this file via FrameSource.
 */
class UvcFrameSource(private val context: Context) : FrameSource {

    private var client: MultiCameraClient? = null
    private var camera: CameraUVC? = null
    private var onLuma: ((ByteArray, Int, Int) -> Unit)? = null

    private val previewCb = object : IPreviewDataCallBack {
        override fun onPreviewData(
            data: ByteArray?, width: Int, height: Int,
            format: IPreviewDataCallBack.DataFormat,
        ) {
            val d = data ?: return
            // NV21 Y-plane = first width*height bytes = luminance. Copy it out
            // (the library reuses its buffer) and hand it to the tracker.
            val need = width * height
            if (d.size < need) return
            onLuma?.invoke(d.copyOf(need), width, height)
        }
    }

    private val stateCb = ICameraStateCallBack { cam, state, _ ->
        if (state == ICameraStateCallBack.State.OPENED) {
            (cam as? CameraUVC)?.let {
                camera = it
                it.addPreviewDataCallBack(previewCb)
            }
        }
    }

    override fun start(onLuma: (ByteArray, Int, Int) -> Unit) {
        this.onLuma = onLuma
        val request = CameraRequest.Builder()
            .setPreviewWidth(640)
            .setPreviewHeight(480)
            .create()
        client = MultiCameraClient(context, object : MultiCameraClient.IDeviceConnectCallBack {
            override fun onAttachDev(device: android.hardware.usb.UsbDevice?) {
                // Request permission + open the first UVC device that attaches.
                device ?: return
                client?.requestPermission(device)
            }
            override fun onDetachDec(device: android.hardware.usb.UsbDevice?) { camera = null }
            override fun onConnectDev(
                device: android.hardware.usb.UsbDevice?,
                ctrlBlock: com.jiangdg.usb.USBMonitor.UsbControlBlock?,
            ) {
                device ?: return
                val cam = CameraUVC(context, device)
                cam.openCamera(null, request)
                cam.setCameraStateCallBack(stateCb)
            }
            override fun onDisConnectDec(
                device: android.hardware.usb.UsbDevice?,
                ctrlBlock: com.jiangdg.usb.USBMonitor.UsbControlBlock?,
            ) { camera = null }
            override fun onCancelDev(device: android.hardware.usb.UsbDevice?) {}
        })
        client?.register()
        Log.i("UvcFrameSource", "UVC client registered; waiting for dongle attach")
    }

    override fun stop() {
        camera?.closeCamera()
        client?.unRegister()
        client?.destroy()
        camera = null; client = null; onLuma = null
    }
}
