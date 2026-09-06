package com.kestrel.navviz.depth

import ai.onnxruntime.OnnxTensor
import ai.onnxruntime.OrtEnvironment
import ai.onnxruntime.OrtSession
import android.content.Context
import android.graphics.Bitmap
import java.nio.FloatBuffer

/**
 * On-device MiDaS depth inference via ONNX Runtime — runs the SAME
 * midas_small.onnx the desktop tools use.
 *
 * Session creation tries NNAPI (the phone's NPU/GPU/DSP) first, then falls
 * back to plain CPU if NNAPI can't compile the model — this fallback is the
 * fix for "depth silently disabled": createSession() throws (not addNnapi())
 * when NNAPI rejects the graph, and without the fallback that killed depth
 * entirely. `provider` records which path won, for the on-screen status line.
 *
 * Output convention: 0 = near/blocked, 1 = far/open (MiDaS emits inverse depth,
 * higher = closer, so we invert) — the layout Openness.analyze() expects.
 */
class MidasDepth(context: Context, modelAsset: String = "midas_small.onnx") {

    private val env = OrtEnvironment.getEnvironment()
    private val session: OrtSession
    private val inputName: String

    /** "nnapi" or "cpu" — surfaced in the overlay so it's clear what ran. */
    val provider: String

    val inW = 256
    val inH = 256

    init {
        val bytes = context.assets.open(modelAsset).use { it.readBytes() }

        // Attempt NNAPI; on ANY failure during option-build or session compile,
        // fall back to a clean CPU session. Never let acceleration take depth down.
        val (sess, prov) = try {
            val o = OrtSession.SessionOptions()
            o.addNnapi()
            env.createSession(bytes, o) to "nnapi"
        } catch (_: Throwable) {
            env.createSession(bytes, OrtSession.SessionOptions()) to "cpu"
        }
        session = sess
        provider = prov
        inputName = session.inputNames.iterator().next()
    }

    /**
     * @return depth as row-major FloatArray of size inW*inH, values [0,1],
     *   0 = near/blocked, 1 = far/open. Throws on tensor-shape mismatch — the
     *   caller surfaces that rather than swallowing it, so a wrong model layout
     *   is visible instead of silently blank.
     */
    fun infer(bitmap: Bitmap): FloatArray {
        val scaled = Bitmap.createScaledBitmap(bitmap, inW, inH, true)
        val input = OnnxTensor.createTensor(
            env, buildNchwInput(scaled), longArrayOf(1, 3, inH.toLong(), inW.toLong()))

        input.use {
            session.run(mapOf(inputName to it)).use { result ->
                val flat = flatten(result[0].value)
                if (flat.size != inW * inH) {
                    throw IllegalStateException(
                        "depth output ${flat.size} != ${inW * inH}; model layout differs")
                }
                // ROBUST normalisation: 2nd/98th percentiles, not raw min/max.
                // MiDaS relative depth had a single very-near or very-far pixel
                // stretch the whole [0,1] range, so mid-distance obstacles
                // collapsed toward "far/open" and only things RIGHT in front
                // registered ("only detects things very nearby"). Percentile
                // bounds ignore those outliers so the usable range spreads across
                // real scene depth. Subsampled sort keeps it cheap.
                val sample = FloatArray((flat.size + 7) / 8) { flat[it * 8] }
                sample.sort()
                val lo = sample[sample.size * 2 / 100]
                val hi = sample[sample.size * 98 / 100]
                val range = (hi - lo).coerceAtLeast(1e-6f)
                val out = FloatArray(flat.size)
                for (i in flat.indices) out[i] = (1f - (flat[i] - lo) / range).coerceIn(0f, 1f)
                return out
            }
        }
    }

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
