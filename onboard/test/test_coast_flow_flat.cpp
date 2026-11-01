// COAST FLOW MUST NOT INVENT MOTION FROM AN IMAGE THAT CANNOT SHOW IT.
//
// CoastFlow measures image displacement so a COASTING tracker can keep its
// search box on a target it cannot currently see. When the scene goes
// featureless, there is nothing to measure -- and the failure was not that it
// returned noise. It returned a CONSTANT, MAXIMAL displacement toward one
// corner, on every point, every frame.
//
// The mechanism is exact. matchPoint scores the SSD over the search window and
// keeps a candidate only on a STRICT improvement:
//
//     if (s < best) { best = s; bi = ox; bj = oy; }
//
// Against a uniform current frame every offset scores identically -- sum of
// (patch - constant)^2 does not depend on where you put it -- so no candidate
// ever improves on the first one tested, and the first one tested is the
// window's negative corner. With search 12 over 3 levels that accumulates to
// -3, doubled to -6, refined to -8, doubled to -16, refined to -18: the
// tracker's search centre walks 18 px per frame, diagonally, forever.
//
// This test pins BOTH halves of the repair: a tie must resolve to NO motion,
// and a patch with nothing in it must be refused outright so the caller keeps
// the prediction it already had.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gray_frame.hpp"
#include "optical_flow.hpp"

using namespace track;

namespace {
int failures = 0;
void check(const char* what, bool ok, const std::string& detail = "") {
    std::printf("  %-56s %s  %s\n", what, ok ? "ok  " : "FAIL", detail.c_str());
    if (!ok) ++failures;
}

// A textured frame: something CoastFlow can legitimately seed corners on.
GrayFrame textured(int w, int h, int shift) {
    GrayFrame g; g.w = w; g.h = h;
    g.ownD = std::make_shared<std::vector<float>>(size_t(w) * h, 0.f);
    g.d = g.ownD->data();
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const int u = x - shift;
            // Deterministic high-frequency pattern with real corners in it.
            g.d[size_t(y) * w + x] =
                float(((u * 73 + y * 151) ^ (u * 17 + y * 29)) & 0xFF);
        }
    return g;
}

GrayFrame uniform(int w, int h, float v) {
    GrayFrame g; g.w = w; g.h = h;
    g.ownD = std::make_shared<std::vector<float>>(size_t(w) * h, v);
    g.d = g.ownD->data();
    return g;
}
}  // namespace

int main() {
    std::printf("coast flow on an image with nothing in it\n");
    const int W = 320, H = 240;

    {
        CoastFlow cf;
        const GrayFrame seedF = textured(W, H, 0);
        cf.seed(seedF, W * 0.5f, H * 0.5f, 120.f);
        check("seeds on a textured frame", cf.ready());

        // The scene goes flat. There is no measurable motion, and there is no
        // motion to measure -- these must not be confused.
        float dx = 0, dy = 0;
        const GrayFrame flat = uniform(W, H, 128.f);
        const bool got = cf.step(flat, dx, dy);
        check("refuses a frame with no texture to match against", !got,
              got ? "reported " + std::to_string(dx) + "," + std::to_string(dy)
                  : "");
        if (got)
            check("...and at least does not invent a large displacement",
                  std::fabs(dx) < 1.f && std::fabs(dy) < 1.f,
                  std::to_string(dx) + "," + std::to_string(dy));
    }

    {
        // Two IDENTICAL textured frames: the honest answer is zero, and this is
        // the case the tie-break has to get right even where texture exists.
        CoastFlow cf;
        const GrayFrame a = textured(W, H, 0);
        cf.seed(a, W * 0.5f, H * 0.5f, 120.f);
        float dx = 0, dy = 0;
        const bool got = cf.step(a, dx, dy);
        check("identical frames report no motion", got &&
              std::fabs(dx) < 0.51f && std::fabs(dy) < 0.51f,
              std::to_string(dx) + "," + std::to_string(dy));
    }

    {
        // And it must still MEASURE when there is something to measure, or the
        // fix above would be a tracker that never coasts.
        CoastFlow cf;
        const GrayFrame s0 = textured(W, H, 0);
        cf.seed(s0, W * 0.5f, H * 0.5f, 120.f);
        float dx = 0, dy = 0;
        const GrayFrame s4 = textured(W, H, 4);
        const bool got = cf.step(s4, dx, dy);
        check("a real 4 px shift is still recovered", got &&
              std::fabs(dx - 4.f) < 1.01f && std::fabs(dy) < 1.01f,
              std::to_string(dx) + "," + std::to_string(dy));
    }

    std::printf(failures ? "FAILURES (%d)\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
