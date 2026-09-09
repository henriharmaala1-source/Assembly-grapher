#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

// ---------------------------------------------------------------------------
// GrayFrame -- the frame the fused tracker works on. Direct port of
// android-tracker/.../track/GrayFrame.kt; the two must stay in step, because
// desktop/simtrack.py is the validation mirror for BOTH.
//
// Luminance as row-major floats in [0,255], plus OPTIONAL per-pixel centred
// chroma (cu = U-128, cv = V-128). Chroma is carried per-pixel rather than
// subsampled so the CHROMA cue and the STAPLE histogram can index it with the
// same offset as luma -- the tracker never has to think about 4:2:0 geometry.
//
// Deliberately NOT a cv::Mat. The tracker is meant to be portable to a phone and
// an MCU, and every buffer here is caller-ownable so the frame path can run
// allocation-free (see cropResampleInto). Conversion from cv::Mat happens once,
// at the edge, in fromBgr().
// ---------------------------------------------------------------------------

namespace track {

struct GrayFrame {
    // Views, not owners: a crop can write into caller scratch. `own*` hold
    // storage only when this frame allocated it.
    float* d  = nullptr;          // luma  [0,255]
    float* cu = nullptr;          // U-128, or null
    float* cv = nullptr;          // V-128, or null
    int w = 0, h = 0;

    std::shared_ptr<std::vector<float>> ownD, ownU, ownV;

    bool hasColor() const { return cu && cv; }
    float at(int x, int y) const { return d[y * w + x]; }

    // Region (rx,ry,rw,rh) resampled to (outW,outH), bilinear. Allocates.
    GrayFrame cropResample(float rx, float ry, float rw, float rh,
                           int outW, int outH) const {
        return cropResampleInto(rx, ry, rw, rh, outW, outH, nullptr, nullptr, nullptr);
    }

    // As above, but writes into CALLER-OWNED buffers when supplied and large
    // enough. This exists purely to keep the tracker off the allocator: a wide
    // re-acquire crop is 384x384, so a fresh luma+chroma set is 1.77 MB per
    // frame on the capture thread.
    GrayFrame cropResampleInto(float rx, float ry, float rw, float rh,
                               int outW, int outH,
                               float* dstD, float* dstU, float* dstV) const;

    // Build from an OpenCV BGR or single-channel image. BGR is converted to
    // BT.601 luma + centred chroma so the CHROMA cue and the histogram cue work;
    // a 1-channel input yields a luma-only frame and those cues skip themselves.
    static GrayFrame fromBgr(const unsigned char* bgr, int w, int h, int stride,
                             int channels);

private:
    static float bilerp(const float* a, int w, int x0, int x1, int y0, int y1,
                        float tx, float ty) {
        const float p = a[y0 * w + x0], q = a[y0 * w + x1];
        const float r = a[y1 * w + x0], s = a[y1 * w + x1];
        const float top = p + (q - p) * tx;
        const float bot = r + (s - r) * tx;
        return top + (bot - top) * ty;
    }
};

}  // namespace track
