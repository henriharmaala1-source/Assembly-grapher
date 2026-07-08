package com.kestrel.navviz.depth

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.graphics.Bitmap
import java.nio.FloatBuffer

/**
 * On-device MiDaS depth inference via ONNX Runtime.
 *
 * Runs the EXACT SAME `midas_small.onnx` the desktop tools use
 * (tilt_bench.py / spin_map.py, via OpenCV DNN) — so there's no separate model
 * to fetch and no preprocessing to reconcile: the ImageNet mean/std NCHW input
 * and inverse-depth output below match the desktop `DepthNav` byte for byte.
 * Copy the file you already have at ~/depth_models/midas_small.onnx into
 * app/src/main/assets/midas_small.onnx (see android/README).
 *
 * SCAFFOLD — written without an Android device in the loop. The ORT API calls
 * follow the documented onnxruntime-android pattern, but the input/output
 * tensor NAMES and the exact output rank are read back from the session at load
 * time (rather than hard-coded) precisely because that's the part most likely to
 * vary; still, treat first-run behaviour as needing on-device confirmation.
 *
 * Output convention: 0 = near/blocked, 1 = far/open (MiDaS emits inverse depth,
 * higher = closer, so we invert) — the layout Openness.analyze() expects.
 */
class MidasDepth(context: Context, modelAsset: String = "midas_small.onnx") {

    private val env = OrtEnvironment.getEnvironment()
    private val session: OrtSession
    private val inputName: String

    // MiDaS small is 256x256, ImageNet-normalised NCHW (matches desktop DepthNav).
    val inW = 256
    val inH = 256

    init {
        val bytes = context.assets.open(modelAsset).use { it.readBytes() }
        val opts = OrtSession.SessionOptions()
        // NNAPI = the phone's NPU/GPU/DSP accel path; fall back to CPU if the
        // device or model can't take it (some ops aren't NNAPI-supported).
        runCatching { opts.addNnapi() }
        session = env.createSession(bytes, opts)
        inputName = session.inputNames.iterator().next()
    }

    /**
     * @return depth as row-major FloatArray of size inW*inH, values [0,1],
     *   0 = near/blocked, 1 = far/open — the layout Openness.analyze() expects.
     */
    fun infer(bitmap: Bitmap): FloatArray {
        val scaled = Bitmap.createScaledBitmap(bitmap, inW, inH, true)
        val input = OnnxTensor.createTensor(
            env, buildNchwInput(scaled), longArrayOf(1, 3, inH.toLong(), inW.toLong()))

        input.use {
            session.run(mapOf(inputName to it)).use { result ->
                val raw = (result[0].value)  // shape [1,H,W] or [1,1,H,W] -> flatten
                val flat = flatten(raw)
                // Normalise to [0,1] then invert (MiDaS higher=closer).
                var min = Float.MAX_VALUE; var max = -Float.MAX_VALUE
                for (v in flat) { if (v < min) min = v; if (v > max) max = v }
                val range = (max - min).coerceAtLeast(1e-6f)
                val out = FloatArray(flat.size)
                for (i in flat.indices) out[i] = 1f - (flat[i] - min) / range
                return out
            }
        }
    }

    /** Flatten ONNX output of unknown nesting ([1,H,W] or [1,1,H,W]) to row-major. */
    private fun flatten(v: Any?): FloatArray {
        val list = ArrayList<Float>(inW * inH)
        fun rec(o: Any?) {
            when (o) {
                is FloatArray -> for (f in o) list.add(f)
                is Array<*> -> for (e in o) rec(e)
            }
        }
        rec(v)
        return list.toFloatArray()
    }

    private fun buildNchwInput(bmp: Bitmap): FloatBuffer {
        val mean = floatArrayOf(0.485f, 0.456f, 0.406f)
        val std = floatArrayOf(0.229f, 0.224f, 0.225f)
        val buf = FloatBuffer.allocate(3 * inW * inH)
        val px = IntArray(inW * inH); bmp.getPixels(px, 0, inW, 0, 0, inW, inH)
        for (ch in 0 until 3) for (i in px.indices) {
            val p = px[i]
            val v = when (ch) {
                0 -> ((p shr 16) and 0xFF)   // R
                1 -> ((p shr 8) and 0xFF)    // G
                else -> (p and 0xFF)         // B
            } / 255f
            buf.put((v - mean[ch]) / std[ch])
        }
        buf.rewind()
        return buf
    }

    fun close() { session.close() }
}
