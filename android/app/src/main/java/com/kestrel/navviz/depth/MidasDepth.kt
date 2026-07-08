package com.kestrel.navviz.depth

import android.content.Context
import android.graphics.Bitmap
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * On-device MiDaS depth inference via TFLite.
 *
 * SCAFFOLD — the surrounding structure is right, but the exact input/output
 * tensor layout and normalisation MUST be verified against the specific .tflite
 * you ship (the qualcomm/Midas-V2 and isl-org MiDaS exports differ in size,
 * NCHW-vs-NHWC, and whether output is inverse depth). Cross-check against the
 * official isl-org/MiDaS Android sample and the model card. On a Snapdragon 8
 * Elite class NPU this model runs in single-digit ms — orders of magnitude
 * faster than the Pi 5 CPU path, which is the whole reason a phone is a good
 * validation vehicle.
 *
 * Output convention normalised to match Openness/DepthNav: 0 = near/blocked,
 * 1 = far/open. MiDaS emits inverse depth (higher = closer), so we invert.
 */
class MidasDepth(context: Context, modelAsset: String = "midas_small.tflite") {

    private val interpreter: Interpreter
    private var gpuDelegate: GpuDelegate? = null

    // TODO(device): set from the actual model's input tensor. 256 for MiDaS-small.
    val inW = 256
    val inH = 256

    init {
        val model = context.assets.openFd(modelAsset).use { fd ->
            fd.createInputStream().channel.map(
                java.nio.channels.FileChannel.MapMode.READ_ONLY, fd.startOffset, fd.declaredLength)
        }
        val opts = Interpreter.Options()
        if (CompatibilityList().isDelegateSupportedOnThisDevice) {
            gpuDelegate = GpuDelegate()
            opts.addDelegate(gpuDelegate)   // NPU/GPU path where available
        } else {
            opts.setNumThreads(4)           // CPU fallback
        }
        interpreter = Interpreter(model, opts)
    }

    /**
     * @return depth as row-major FloatArray of size inW*inH, values [0,1],
     *   0 = near/blocked, 1 = far/open. Same layout Openness.analyze() expects.
     */
    fun infer(bitmap: Bitmap): FloatArray {
        val scaled = Bitmap.createScaledBitmap(bitmap, inW, inH, true)
        val input = toNormalizedInput(scaled)

        // Output shape is model-specific; MiDaS-small is typically [1, inH, inW].
        val out = Array(1) { Array(inH) { FloatArray(inW) } }
        interpreter.run(input, out)

        val flat = FloatArray(inW * inH)
        var min = Float.MAX_VALUE; var max = -Float.MAX_VALUE
        for (y in 0 until inH) for (x in 0 until inW) {
            val v = out[0][y][x]; if (v < min) min = v; if (v > max) max = v
        }
        val range = (max - min).coerceAtLeast(1e-6f)
        for (y in 0 until inH) for (x in 0 until inW) {
            val norm = (out[0][y][x] - min) / range
            flat[y * inW + x] = 1f - norm   // invert: MiDaS higher=closer -> we want higher=farther
        }
        return flat
    }

    private fun toNormalizedInput(bmp: Bitmap): ByteBuffer {
        // ImageNet mean/std, NCHW float32 — matches the Python DepthNav preprocess.
        val mean = floatArrayOf(0.485f, 0.456f, 0.406f)
        val std = floatArrayOf(0.229f, 0.224f, 0.225f)
        val buf = ByteBuffer.allocateDirect(4 * 3 * inW * inH).order(ByteOrder.nativeOrder())
        val px = IntArray(inW * inH); bmp.getPixels(px, 0, inW, 0, 0, inW, inH)
        for (ch in 0 until 3) for (i in px.indices) {
            val p = px[i]
            val v = when (ch) {
                0 -> ((p shr 16) and 0xFF)   // R
                1 -> ((p shr 8) and 0xFF)    // G
                else -> (p and 0xFF)         // B
            } / 255f
            buf.putFloat((v - mean[ch]) / std[ch])
        }
        buf.rewind()
        return buf
    }

    fun close() { interpreter.close(); gpuDelegate?.close() }
}
