#include "crop_filters.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace track {

const char* cropFilterName(CropFilter f) {
    switch (f) {
        case CropFilter::NONE:      return "none";
        case CropFilter::STRETCH:   return "stretch";
        case CropFilter::EDGE:      return "edge";
        case CropFilter::THRESHOLD: return "threshold";
        case CropFilter::SHARPEN:   return "sharpen";
        case CropFilter::CHROMA:    return "chroma";
    }
    return "?";
}

bool cropFilterFromName(const char* s, CropFilter& out) {
    const std::string k(s ? s : "");
    if (k == "none")      { out = CropFilter::NONE;      return true; }
    if (k == "stretch")   { out = CropFilter::STRETCH;   return true; }
    if (k == "edge")      { out = CropFilter::EDGE;      return true; }
    if (k == "threshold") { out = CropFilter::THRESHOLD; return true; }
    if (k == "sharpen")   { out = CropFilter::SHARPEN;   return true; }
    if (k == "chroma")    { out = CropFilter::CHROMA;    return true; }
    return false;
}

namespace {

// Wrap a result buffer in a GrayFrame that carries the SOURCE chroma through.
// CHROMA needs that (it returns a frame whose luma is colourfulness but whose
// chroma planes are still the originals, because the histogram cue reads them).
GrayFrame wrap(float* buf, std::shared_ptr<std::vector<float>> own,
               const GrayFrame& g, bool keepChroma) {
    GrayFrame o;
    o.d = buf; o.ownD = std::move(own);
    o.w = g.w; o.h = g.h;
    if (keepChroma) { o.cu = g.cu; o.cv = g.cv; o.ownU = g.ownU; o.ownV = g.ownV; }
    return o;
}

// Pick the destination: caller scratch when EXACTLY the right size, else fresh.
float* dest(float* dst, int dstN, int n, std::shared_ptr<std::vector<float>>& own) {
    if (dst && dstN == n) return dst;
    own = std::make_shared<std::vector<float>>(n);
    return own->data();
}

}  // namespace

GrayFrame applyFilter(const GrayFrame& g, CropFilter f, float* dst, int dstN) {
    const int w = g.w, h = g.h, n = w * h;
    std::shared_ptr<std::vector<float>> own;

    switch (f) {
    case CropFilter::NONE:
        return g;

    // Track on colour instead of luma: per-pixel chroma magnitude. A saturated
    // target on a desaturated background pops even at equal brightness.
    // Brightness-invariant and wraparound-free (unlike hue), so NCC behaves.
    // Passthrough to luma when the frame carries no colour (thermal).
    case CropFilter::CHROMA: {
        if (!g.hasColor()) return g;
        float* out = dest(dst, dstN, n, own);
        for (int i = 0; i < n; ++i) {
            const float m = std::sqrt(g.cu[i] * g.cu[i] + g.cv[i] * g.cv[i]) * 1.41f;
            out[i] = std::min(std::max(m, 0.f), 255.f);
        }
        return wrap(out, own, g, true);
    }

    // Gradient magnitude -- structure.
    case CropFilter::EDGE: {
        float* out = dest(dst, dstN, n, own);
        // The loop skips the 1px border. A fresh buffer is zero there; a REUSED
        // one still holds the previous frame's edges, which would sit in the
        // response map as a stationary phantom. Clear it.
        std::memset(out, 0, size_t(n) * sizeof(float));
        for (int y = 1; y < h - 1; ++y)
            for (int x = 1; x < w - 1; ++x) {
                const float gx = -g.at(x-1,y-1) - 2*g.at(x-1,y) - g.at(x-1,y+1)
                               +  g.at(x+1,y-1) + 2*g.at(x+1,y) + g.at(x+1,y+1);
                const float gy = -g.at(x-1,y-1) - 2*g.at(x,y-1) - g.at(x+1,y-1)
                               +  g.at(x-1,y+1) + 2*g.at(x,y+1) + g.at(x+1,y+1);
                out[y * w + x] = std::min(std::sqrt(gx * gx + gy * gy), 255.f);
            }
        return wrap(out, own, g, true);
    }

    // Robust 2nd/98th-percentile contrast stretch -- cheap CLAHE stand-in.
    case CropFilter::STRETCH: {
        std::vector<float> sample((n + 7) / 8);
        for (size_t i = 0; i < sample.size(); ++i) sample[i] = g.d[i * 8];
        std::sort(sample.begin(), sample.end());
        const float lo = sample[sample.size() * 2 / 100];
        const float hi = sample[sample.size() * 98 / 100];
        const float range = std::max(hi - lo, 1e-3f);
        float* out = dest(dst, dstN, n, own);
        for (int i = 0; i < n; ++i)
            out[i] = std::min(std::max((g.d[i] - lo) / range * 255.f, 0.f), 255.f);
        return wrap(out, own, g, true);
    }

    case CropFilter::THRESHOLD: {
        int hist[256] = {0};
        for (int i = 0; i < n; ++i)
            hist[std::min(std::max(int(g.d[i]), 0), 255)]++;
        double sum = 0; for (int i = 0; i < 256; ++i) sum += double(i) * hist[i];
        double sumB = 0; int wB = 0; double maxVar = 0; int thresh = 127;
        for (int i = 0; i < 256; ++i) {
            wB += hist[i]; if (wB == 0) continue;
            const int wF = n - wB; if (wF == 0) break;
            sumB += double(i) * hist[i];
            const double mB = sumB / wB, mF = (sum - sumB) / wF;
            const double between = double(wB) * wF * (mB - mF) * (mB - mF);
            if (between > maxVar) { maxVar = between; thresh = i; }
        }
        float* out = dest(dst, dstN, n, own);
        for (int i = 0; i < n; ++i) out[i] = (g.d[i] > thresh) ? 255.f : 0.f;
        return wrap(out, own, g, true);
    }

    case CropFilter::SHARPEN: {
        float* out = dest(dst, dstN, n, own);
        std::memcpy(out, g.d, size_t(n) * sizeof(float));
        for (int y = 1; y < h - 1; ++y)
            for (int x = 1; x < w - 1; ++x) {
                const float lap = 4*g.at(x,y) - g.at(x-1,y) - g.at(x+1,y)
                                - g.at(x,y-1) - g.at(x,y+1);
                out[y * w + x] = std::min(std::max(g.at(x,y) + 0.7f * lap, 0.f), 255.f);
            }
        return wrap(out, own, g, true);
    }
    }
    return g;
}

}  // namespace track
