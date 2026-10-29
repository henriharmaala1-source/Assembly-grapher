#include "ground_texture.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace kground {

namespace {

constexpr float kPi = 3.14159265358979f;

// Deterministic value noise in 2D, smooth (C1), in [0,1].
float hash2(int x, int y, unsigned seed) {
    unsigned h = unsigned(x) * 374761393u + unsigned(y) * 668265263u + seed * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return float((h ^ (h >> 16)) & 0xFFFFFF) / float(0x1000000);
}
float vnoise(float x, float y, unsigned seed) {
    const int ix = int(std::floor(x)), iy = int(std::floor(y));
    float fx = x - float(ix), fy = y - float(iy);
    fx = fx * fx * (3.f - 2.f * fx); fy = fy * fy * (3.f - 2.f * fy);
    const float a = hash2(ix, iy, seed), b = hash2(ix + 1, iy, seed);
    const float c = hash2(ix, iy + 1, seed), d = hash2(ix + 1, iy + 1, seed);
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}
// Two octaves at `scaleM` metres: a patch map.
float patches(float x, float y, float scaleM, unsigned seed) {
    return 0.65f * vnoise(x / scaleM, y / scaleM, seed) +
           0.35f * vnoise(2.1f * x / scaleM, 2.1f * y / scaleM, seed + 101u);
}
float smoothstep(float a, float b, float x) {
    const float t = std::min(1.f, std::max(0.f, (x - a) / (b - a)));
    return t * t * (3.f - 2.f * t);
}

float bilinearWrap(const cv::Mat& m, float px, float py) {
    const int W = m.cols, H = m.rows;
    px -= 0.5f; py -= 0.5f;
    const float fx = std::floor(px), fy = std::floor(py);
    const float ax = px - fx, ay = py - fy;
    auto wrap = [](int v, int n) { v %= n; return v < 0 ? v + n : v; };
    const int x0 = wrap(int(fx), W), x1 = wrap(int(fx) + 1, W);
    const int y0 = wrap(int(fy), H), y1 = wrap(int(fy) + 1, H);
    const float* r0 = m.ptr<float>(y0);
    const float* r1 = m.ptr<float>(y1);
    return (r0[x0] * (1 - ax) + r0[x1] * ax) * (1 - ay) + (r1[x0] * (1 - ax) + r1[x1] * ax) * ay;
}

}  // namespace

bool Texture::load(const std::string& path, float size, std::string* err) {
    cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
    if (bgr.empty()) { if (err) *err = "cannot read " + path; return false; }
    cv::Mat g;
    cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);          // luminance, visible band
    g.convertTo(g, CV_32F, 1.0 / 255.0);
    // Square power of two, so every mip level halves exactly.
    int s = 1;
    while (s * 2 <= std::min(g.cols, g.rows)) s *= 2;
    cv::resize(g, g, cv::Size(s, s), 0, 0, cv::INTER_AREA);
    cv::Scalar mu, sd;
    cv::meanStdDev(g, mu, sd);
    mean = float(mu[0]); stddev = float(sd[0]);
    sizeM = size;
    mip.clear();
    mip.push_back(g);
    while (mip.back().cols > 2) {
        cv::Mat h;
        cv::resize(mip.back(), h, cv::Size(mip.back().cols / 2, mip.back().rows / 2), 0, 0,
                   cv::INTER_AREA);                     // box filter: the area a texel covers
        mip.push_back(h);
    }
    return true;
}

float Texture::sample(float x, float y, float footM) const {
    const float texel = sizeM / float(mip[0].cols);
    float lod = std::log2(std::max(1e-6f, footM / texel));
    lod = std::min(float(mip.size() - 1), std::max(0.f, lod));
    const int l0 = int(lod);
    const int l1 = std::min(int(mip.size()) - 1, l0 + 1);
    const float f = lod - float(l0);
    auto at = [&](int l) {
        const cv::Mat& m = mip[size_t(l)];
        return bilinearWrap(m, x / sizeM * float(m.cols), y / sizeM * float(m.rows));
    };
    return f > 0.f && l1 != l0 ? at(l0) * (1.f - f) + at(l1) * f : at(l0);
}

bool GroundRenderer::load(const std::string& dir, std::string* err) {
    // Physical size of each photograph, metres per width, judged from what it
    // shows: grass blades, stones, cracks, paving slabs.
    struct Spec { const char* file; const char* name; float sizeM; };
    const Spec specs[] = {
        {"lawn.jpg", "lawn", 2.0f},       {"grass.jpg", "grass", 0.6f},
        {"grass_pd.png", "grass2", 0.45f}, {"gravel.png", "gravel", 0.7f},
        {"dirt.jpg", "dirt", 2.0f},       {"rocky.png", "rocky", 2.0f},
        {"sand.jpg", "sand", 0.4f},       {"paving.png", "paving", 2.4f},
        {"brick.jpg", "brick", 2.0f},
    };
    tex_.clear();
    for (const Spec& s : specs) {
        Texture t;
        t.name = s.name;
        if (!t.load(dir + "/" + s.file, s.sizeM, err)) {
            if (err) *err += " -- run nav-sim/textures/fetch_ground_textures.sh";
            return false;
        }
        tex_.push_back(std::move(t));
    }
    return true;
}

const Texture* GroundRenderer::find(const char* name) const {
    for (const Texture& t : tex_) if (t.name == name) return &t;
    return nullptr;
}

const char* GroundRenderer::sceneNames() { return "lawn|field|gravel|dirt|sand|paving|park"; }

bool GroundRenderer::setScene(const std::string& name, unsigned seed, float floorTopM) {
    const std::string all = std::string("|") + sceneNames() + "|";
    if (all.find("|" + name + "|") == std::string::npos) return false;
    scene_ = name; seed_ = seed; floorTop_ = floorTopM; gain_ = -1.f;
    return true;
}

float GroundRenderer::groundAlbedo(float x, float y, float foot) const {
    // A material: its photograph by STOCHASTIC TILING. The ground is cut into
    // cells a little smaller than the photograph, each showing the photograph at its own random offset and
    // rotation; a point blends the four nearest cells with smooth weights, and
    // the blend is renormalised to the photograph's own contrast. No period
    // survives, at any height. (Two fixed layers did not: from 15 m a 2 m tile
    // is a dozen pixels, and the lattice showed.)
    auto mat = [&](const char* nm) {
        const Texture* t = find(nm);
        // Built surfaces ARE periodic: paving laid in a pattern is one layer.
        if (std::string(nm) == "paving") return t->sample(x, y, foot);
        // Cells SMALLER than the photograph: a tile that repeats even twice
        // inside one cell shows as a lattice from height.
        const float cellM = 0.8f * t->sizeM;
        const float gx = x / cellM - 0.5f, gy = y / cellM - 0.5f;
        const int ix = int(std::floor(gx)), iy = int(std::floor(gy));
        float fx = gx - float(ix), fy = gy - float(iy);
        fx = fx * fx * (3.f - 2.f * fx); fy = fy * fy * (3.f - 2.f * fy);
        float acc = 0.f, w2 = 0.f;
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) {
                const float w = (i ? fx : 1.f - fx) * (j ? fy : 1.f - fy);
                if (w < 1e-4f) continue;
                const unsigned k = seed_ * 7u + 31u;
                const float ang = 2.f * kPi * hash2(ix + i, iy + j, k);
                const float ox = 97.f * hash2(ix + i, iy + j, k + 1u);
                const float oy = 97.f * hash2(ix + i, iy + j, k + 2u);
                const float ca = std::cos(ang), sa = std::sin(ang);
                acc += w * (t->sample(ca * x - sa * y + ox, sa * x + ca * y + oy, foot) - t->mean);
                w2 += w * w;
            }
        return t->mean + acc / std::sqrt(std::max(1e-6f, w2));
    };
    // Large-scale brightness: moisture, wear, shade -- metres to tens.
    auto macro = [&](float scaleM, float amp, unsigned k) {
        return 1.f + amp * (2.f * patches(x, y, scaleM, seed_ + k) - 1.f);
    };
    float a;
    if (scene_ == "lawn") {
        // Mowing stripes: alternate passes lay the blades two ways, ~8 %.
        const float stripe = 0.08f * std::sin(2.f * kPi * x / 1.6f);
        a = mat("lawn") * (1.f + stripe) * macro(8.f, 0.10f, 1);
    } else if (scene_ == "field") {
        const float dirtW = smoothstep(0.58f, 0.68f, patches(x, y, 7.f, seed_ + 2));
        const float worn = smoothstep(0.62f, 0.72f, patches(x, y, 3.f, seed_ + 3));
        a = mat("grass") * (1.f - dirtW) + mat("dirt") * dirtW;
        a = a * (1.f - 0.5f * worn) + mat("grass2") * 0.5f * worn;
        a *= macro(12.f, 0.12f, 4);
    } else if (scene_ == "gravel") {
        const float r = smoothstep(0.6f, 0.7f, patches(x, y, 5.f, seed_ + 5));
        a = (mat("gravel") * (1.f - r) + mat("rocky") * r) * macro(5.f, 0.08f, 6);
    } else if (scene_ == "dirt") {
        const float r = smoothstep(0.62f, 0.72f, patches(x, y, 6.f, seed_ + 7));
        a = (mat("dirt") * (1.f - r) + mat("rocky") * r) * macro(10.f, 0.10f, 8);
    } else if (scene_ == "sand") {
        a = mat("sand") * macro(15.f, 0.05f, 9);   // the hard case: little to see from up high
    } else if (scene_ == "paving") {
        a = mat("paving") * macro(6.f, 0.06f, 10);
    } else {   // park: grass, a paved path, gravel and dirt patches
        const float path = 1.f - smoothstep(1.3f, 1.6f, std::fabs(std::fmod(x + 1000.f, 18.f) - 9.f));
        const float grav = smoothstep(0.62f, 0.7f, patches(x, y, 6.f, seed_ + 11));
        const float dirtW = smoothstep(0.64f, 0.72f, patches(x, y, 4.f, seed_ + 12));
        a = mat("grass");
        a = a * (1.f - dirtW) + mat("dirt") * dirtW;
        a = a * (1.f - grav) + mat("gravel") * grav;
        a = a * (1.f - path) + mat("paving") * path;
        a *= macro(10.f, 0.10f, 13);
    }
    return std::max(0.f, a);
}

float GroundRenderer::radiance(const sim::VoxelWorld& w, const sim::DepthCamera& cam,
                               const sim::CamPose& pose, int u, int v,
                               const DroneShadow& sh) const {
    float dx, dy, dz;
    cam.rayFor(pose, u, v, dx, dy, dz);
    const float maxR = cam.params().maxRangeM;
    const float t = w.raycast(pose.e, pose.n, pose.u, dx, dy, dz, maxR);
    if (t >= maxR) return 1.4f;                         // sky: brighter than any ground
    const float hx = pose.e + dx * t, hy = pose.n + dy * t, hz = pose.u + dz * t;
    // Which face was hit: the coordinate sitting on a cell boundary.
    const float c = w.cell();
    auto edge = [c](float q, float o) { const float f = (q - o) / c - std::floor((q - o) / c);
                                        return std::min(f, 1.f - f); };
    const float ex = edge(hx, w.ox()), ey = edge(hy, w.oy()), ez = edge(hz, w.oz());
    const int axis = (ez <= ex && ez <= ey) ? 2 : (ex <= ey ? 0 : 1);
    float nrm[3] = {0, 0, 0};
    nrm[axis] = (axis == 0 ? dx : axis == 1 ? dy : dz) > 0 ? -1.f : 1.f;
    const float cosInc = std::max(0.1f, std::fabs(axis == 0 ? dx : axis == 1 ? dy : dz));
    const float foot = t / (cam.fpx() * cosInc);
    // Material: the floor slab's top is ground; anything else is an object --
    // rocky on top, brick on the sides.
    float alb;
    if (axis == 2 && nrm[2] > 0 && hz < floorTop_ + 0.5f * c) alb = groundAlbedo(hx, hy, foot);
    else if (axis == 2) alb = find("rocky")->sample(hx, hy, foot);
    else alb = 0.8f * find("brick")->sample(axis == 0 ? hy : hx, hz, foot);
    // The sun.
    const float el = light.sunElevDeg * kPi / 180.f, az = light.sunAzDeg * kPi / 180.f;
    const float s[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
    float lit = std::max(0.f, nrm[0] * s[0] + nrm[1] * s[1] + nrm[2] * s[2]);
    if (lit > 0.f && light.castShadows) {
        const float o = 0.02f;
        const float ts = w.raycast(hx + nrm[0] * o, hy + nrm[1] * o, hz + nrm[2] * o,
                                   s[0], s[1], s[2], 20.f);
        if (ts < 20.f) lit = 0.f;
    }
    if (lit > 0.f && sh.on) {
        // The aircraft between this point and the sun?
        const float px = sh.e - hx, py = sh.n - hy, pz = sh.u - hz;
        const float along = px * s[0] + py * s[1] + pz * s[2];
        if (along > 0.f) {
            const float qx = px - along * s[0], qy = py - along * s[1], qz = pz - along * s[2];
            if (qx * qx + qy * qy + qz * qz < sh.radiusM * sh.radiusM) lit = 0.f;
        }
    }
    return alb * (light.sky + light.sun * lit);
}

cv::Mat GroundRenderer::render(const sim::VoxelWorld& w, const sim::DepthCamera& cam,
                               const sim::CamPose& pose, const std::vector<sim::CamPose>& blur,
                               const DroneShadow& sh) {
    const int W = cam.params().width, H = cam.params().height;
    cv::Mat rad(H, W, CV_32F, cv::Scalar(0));
    const std::vector<sim::CamPose> poses = blur.empty() ? std::vector<sim::CamPose>{pose} : blur;
    for (const sim::CamPose& p : poses)
        for (int v = 0; v < H; ++v) {
            float* row = rad.ptr<float>(v);
            for (int u = 0; u < W; ++u) row[u] += radiance(w, cam, p, u, v, sh);
        }
    rad *= 1.0 / double(poses.size());
    // Vignetting, then auto-exposure on the central half (as cameras meter).
    for (int v = 0; v < H; ++v) {
        float* row = rad.ptr<float>(v);
        for (int u = 0; u < W; ++u) {
            const float rx = (u - 0.5f * W) / (0.5f * W), ry = (v - 0.5f * H) / (0.5f * W);
            row[u] *= 1.f - camera.vignette * std::min(1.f, 0.5f * (rx * rx + ry * ry));
        }
    }
    const cv::Scalar m = cv::mean(rad(cv::Rect(W / 4, H / 4, W / 2, H / 2)));
    const float want = camera.targetMean / float(std::max(1e-4, m[0]));
    gain_ = gain_ < 0.f ? want : gain_ + camera.aeRate * (want - gain_);
    cv::Mat out(H, W, CV_8U);
    std::normal_distribution<float> N(0.f, 1.f);
    for (int v = 0; v < H; ++v) {
        const float* r = rad.ptr<float>(v);
        uchar* o = out.ptr<uchar>(v);
        for (int u = 0; u < W; ++u) {
            float dn = 255.f * gain_ * r[u];
            dn += N(rng_) * std::sqrt(camera.readNoiseDN * camera.readNoiseDN +
                                      camera.shotGain * std::max(0.f, dn));
            o[u] = uchar(std::min(255.f, std::max(0.f, dn + 0.5f)));
        }
    }
    return out;
}

}  // namespace kground
