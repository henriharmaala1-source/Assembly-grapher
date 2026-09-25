// DEPTH INTO THE COLOUR CAMERA (navcore registerDepthToColour), against
// geometry with a known answer. It is what gives a person found in the D435i's
// RGB image a MEASURED range, and no camera is attached to this check -- so
// the parts that are easy to get backwards are pinned here:
//
//   wall       identity calibration: a flat wall at Z comes back as its RANGE
//              (Z at the centre, more off-axis -- range, not Z)
//   baseline   a colour camera 15 mm to the side: a near box shifts by
//              fx * t / Z pixels and a far wall by less, and where they
//              overlap the NEAR one wins
//   rotation   a yawed colour camera: the rs2_extrinsics rotation is
//              COLUMN-major, and a transposed matrix lands on the wrong side
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/core.hpp>

#include "frame_source.hpp"

static int fails = 0;
static void check(const char* what, bool ok, double got, double want) {
    std::printf("  %-58s %s  %.4f (want %.4f)\n", what, ok ? "ok  " : "FAIL", got, want);
    if (!ok) ++fails;
}

int main() {
    const int W = 424, H = 240;
    const float scale = 0.001f;                       // Z16 in millimetres
    const float K[4] = {300.f, 300.f, (W - 1) * 0.5f, (H - 1) * 0.5f};
    float I[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    std::printf("depth -> colour registration\n");

    // ------------------------------------------------------------ wall
    std::vector<uint16_t> z(size_t(W) * H, 2000);     // a wall at Z = 2 m
    cv::Mat r = sim::registerDepthToColour(z.data(), W, H, scale, K, K, W, H, I, 1);
    const float c = r.at<float>(int(K[3]), int(K[2]));
    check("wall: range at the centre is Z", std::fabs(c - 2.f) < 1e-3f, c, 2.0);
    {
        const int u = 20, v = 20;
        const float xn = (u - K[2]) / K[0], yn = (v - K[3]) / K[1];
        const float want = 2.f * std::sqrt(1.f + xn * xn + yn * yn);
        const float got = r.at<float>(v, u);
        check("wall: off-axis it is RANGE, not Z", std::fabs(got - want) < 2e-3f, got, want);
    }
    int holes = 0;
    for (int v = 10; v < H - 10; ++v)
        for (int u = 10; u < W - 10; ++u) holes += r.at<float>(v, u) <= 0.f;
    // Stride 2 into a camera with a longer focal length must splat, not leave
    // pinholes that make a box's median pick up nothing.
    const float Kc[4] = {450.f, 450.f, (W - 1) * 0.5f, (H - 1) * 0.5f};
    cv::Mat r2 = sim::registerDepthToColour(z.data(), W, H, scale, K, Kc, W, H, I, 2);
    int holes2 = 0;
    for (int v = 40; v < H - 40; ++v)
        for (int u = 60; u < W - 60; ++u) holes2 += r2.at<float>(v, u) <= 0.f;
    check("wall: stride 2, 1.5x focal length -- no pinholes", holes == 0 && holes2 == 0,
          holes + holes2, 0);

    // -------------------------------------------------------- baseline
    // A box at Z = 1 m, 40 x 40 px, in front of a wall at 3 m.
    std::fill(z.begin(), z.end(), uint16_t(3000));
    for (int v = 100; v < 140; ++v)
        for (int u = 190; u < 230; ++u) z[size_t(v) * W + u] = 1000;
    // t = (+15 mm, 0, 0): p_colour = p_depth + t, so every point's x grows and
    // lands further RIGHT in the colour image (the colour camera sits 15 mm to
    // the depth camera's left). The shift is fx * t / Z: bigger when nearer.
    float T[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0.015f, 0, 0};
    r = sim::registerDepthToColour(z.data(), W, H, scale, K, K, W, H, T, 1);
    const float shiftNear = K[0] * 0.015f / 1.f;      // 4.5 px
    // The box's left edge was u = 190; in colour it starts ~4.5 px right.
    const int u0 = 190 + int(std::lround(shiftNear));
    const float inside = r.at<float>(120, u0 + 2), before = r.at<float>(120, u0 - 3);
    // Just left of it: the wall, or a disocclusion HOLE (the box shifts 4.5 px,
    // the wall behind it only 1.5 px, and nothing lands in between) -- never
    // the box's own range.
    check("baseline: the near box moved fx*t/Z = 4.5 px",
          inside < 1.1f && !(before > 0.f && before < 1.5f), inside, 1.0);
    // Where near and far both land, the nearer range is kept.
    int nearWins = 0, cells = 0;
    for (int v = 105; v < 135; ++v)
        for (int u = u0; u < u0 + 30; ++u) { ++cells; nearWins += r.at<float>(v, u) < 1.2f; }
    check("baseline: overlapping samples -- nearest wins", nearWins == cells, nearWins, cells);

    // -------------------------------------------------------- rotation
    // Colour camera yawed +5 deg about its Y axis (row-major R below), stored
    // COLUMN-major as librealsense does. A point straight ahead of the depth
    // camera must land where R says -- to the side R puts it.
    const float a = 5.f * 3.14159265f / 180.f;
    const float Rrow[9] = {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};
    float Rc[12];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) Rc[j * 3 + i] = Rrow[i * 3 + j];   // column-major
    Rc[9] = Rc[10] = Rc[11] = 0.f;
    std::fill(z.begin(), z.end(), uint16_t(0));
    const int pu = int(K[2]), pv = int(K[3]);
    z[size_t(pv) * W + pu] = 2000;                     // one point, 2 m ahead
    r = sim::registerDepthToColour(z.data(), W, H, scale, K, K, W, H, Rc, 1);
    // Expected: p = (x,0,2) with x = (pu - ppx)/fx * 2 ~ 0; p' = Rrow * p.
    const float X = (pu - K[2]) / K[0] * 2.f, Zp = 2.f;
    const float Xc = Rrow[0] * X + Rrow[2] * Zp, Zc = Rrow[6] * X + Rrow[8] * Zp;
    const int ue = int(std::lround(K[0] * Xc / Zc + K[2]));
    const float at = r.at<float>(pv, ue), mirrored = r.at<float>(pv, 2 * pu - ue);
    check("rotation: column-major R lands the point where R says",
          std::fabs(at - 2.f) < 1e-2f && mirrored <= 0.f, ue, K[0] * Xc / Zc + K[2]);

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
