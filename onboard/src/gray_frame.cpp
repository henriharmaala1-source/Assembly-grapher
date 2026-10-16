#include "gray_frame.hpp"

namespace track {

GrayFrame GrayFrame::cropResampleInto(float rx, float ry, float rw, float rh,
                                      int outW, int outH,
                                      float* dstD, float* dstU, float* dstV) const {
    const int n = outW * outH;
    GrayFrame o;
    o.w = outW; o.h = outH;

    if (dstD) { o.d = dstD; }
    else { o.ownD = std::make_shared<std::vector<float>>(n); o.d = o.ownD->data(); }

    if (cu) {
        if (dstU) o.cu = dstU;
        else { o.ownU = std::make_shared<std::vector<float>>(n); o.cu = o.ownU->data(); }
    }
    if (cv) {
        if (dstV) o.cv = dstV;
        else { o.ownV = std::make_shared<std::vector<float>>(n); o.cv = o.ownV->data(); }
    }

    const float sx = rw / outW, sy = rh / outH;
    // The horizontal sample position depends only on the COLUMN, so it is
    // worked out once per column rather than once per pixel -- the same
    // arithmetic, so the same result, a sixth of the work per output pixel.
    struct Col { int x0, x1; float tx; };
    std::vector<Col> cols(static_cast<size_t>(outW));
    for (int i = 0; i < outW; ++i) {
        float fx = rx + (i + 0.5f) * sx;
        fx = std::min(std::max(fx, 0.f), float(w - 1));
        const int x0 = int(fx);
        cols[size_t(i)] = {x0, std::min(x0 + 1, w - 1), fx - x0};
    }
    for (int j = 0; j < outH; ++j) {
        float fy = ry + (j + 0.5f) * sy;
        fy = std::min(std::max(fy, 0.f), float(h - 1));
        const int y0 = int(fy);
        const int y1 = std::min(y0 + 1, h - 1);
        const float ty = fy - y0;
        const int row = j * outW;
        for (int i = 0; i < outW; ++i) {
            const Col& c = cols[size_t(i)];
            o.d[row + i] = bilerp(d, w, c.x0, c.x1, y0, y1, c.tx, ty);
        }
        if (o.cu)
            for (int i = 0; i < outW; ++i) {
                const Col& c = cols[size_t(i)];
                o.cu[row + i] = bilerp(cu, w, c.x0, c.x1, y0, y1, c.tx, ty);
            }
        if (o.cv)
            for (int i = 0; i < outW; ++i) {
                const Col& c = cols[size_t(i)];
                o.cv[row + i] = bilerp(cv, w, c.x0, c.x1, y0, y1, c.tx, ty);
            }
    }
    return o;
}

GrayFrame GrayFrame::fromBgr(const unsigned char* bgr, int w, int h, int stride,
                             int channels) {
    const int n = w * h;
    GrayFrame f;
    f.w = w; f.h = h;
    f.ownD = std::make_shared<std::vector<float>>(n);
    f.d = f.ownD->data();

    if (channels < 3) {
        for (int y = 0; y < h; ++y) {
            const unsigned char* row = bgr + size_t(y) * stride;
            for (int x = 0; x < w; ++x) f.d[y * w + x] = float(row[x]);
        }
        return f;                       // luma only: chroma cues skip themselves
    }

    f.ownU = std::make_shared<std::vector<float>>(n);
    f.ownV = std::make_shared<std::vector<float>>(n);
    f.cu = f.ownU->data();
    f.cv = f.ownV->data();

    // BT.601, matching what a camera's NV21 path would have produced -- the
    // Kotlin side gets YUV straight from the sensor, so converting here with the
    // same coefficients keeps the two implementations comparable on the same
    // footage rather than merely similar.
    for (int y = 0; y < h; ++y) {
        const unsigned char* row = bgr + size_t(y) * stride;
        for (int x = 0; x < w; ++x) {
            const float b = row[x * channels + 0];
            const float g = row[x * channels + 1];
            const float r = row[x * channels + 2];
            const int o = y * w + x;
            const float Y = 0.299f * r + 0.587f * g + 0.114f * b;
            f.d[o]  = Y;
            f.cu[o] = 0.564f * (b - Y);          // U-128, centred
            f.cv[o] = 0.713f * (r - Y);          // V-128, centred
        }
    }
    return f;
}

}  // namespace track
