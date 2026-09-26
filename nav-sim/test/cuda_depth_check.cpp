// Is the GPU depth render the same render?
//
// A HARD GATE, not a benchmark. The .cu kernel was written on a machine with no
// CUDA toolkit and no GPU, so it has never been compiled or run by its author.
// This is the check that makes that acceptable rather than reckless: on a
// machine with a device it renders the same scene both ways and FAILS if they
// disagree. Until it has passed on your hardware, the GPU path is unverified.
//
// Without CUDA it still earns its place by testing the fallback, which is the
// path everyone else gets: renderRangesCuda must decline cleanly rather than
// crash, so a caller may invoke it unconditionally.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "depth_camera.hpp"
#include "depth_cuda.hpp"
#include "voxel_world.hpp"

using namespace sim;

static int failures = 0;
static void check(const char* what, bool ok, const std::string& d = "") {
    std::printf("  %-56s %s%s%s\n", what, ok ? "ok" : "FAIL", d.empty() ? "" : "  ", d.c_str());
    if (!ok) ++failures;
}

int main() {
    std::printf("GPU depth render vs CPU\n");
    std::printf("  %s\n", cudaDepthStatus());

    CamParams cp;
    cp.width = 160; cp.height = 120; cp.hfovDeg = 70.f;
    cp.baselineM = 0.12f; cp.maxRangeM = 12.f;
    DepthCamera cam(cp);

    ForestParams fp; fp.cell = 0.25f; fp.seed = 5;
    VoxelWorld w; genForest(w, fp, nullptr);
    CamPose pose; pose.e = 15.f; pose.n = 10.f; pose.u = 6.f; pose.yawDeg = 40.f;

    std::vector<float> gr, gt;
    const bool got = renderRangesCuda(w, pose, cp.width, cp.height, cp.hfovDeg,
                                      cam.fpx(), cam.fpy(), cam.ppx(), cam.ppy(),
                                      cp.maxRangeM, gr, gt);

    if (!got) {
        // The path everyone without a GPU takes. It must decline, not crash,
        // and must not leave a half-filled buffer behind.
        check("no GPU: the call declines cleanly", !cudaDepthAvailable());
        check("and reports why", std::string(cudaDepthStatus()).size() > 8,
              cudaDepthStatus());
        std::printf("  (GPU equality NOT tested -- no device on this machine)\n");
        std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed",
                    failures);
        return failures ? 1 : 0;
    }

    // --- a device is present: the two renders must agree -------------------
    cv::Mat cpu = cam.renderTruth(w, pose);
    check("the GPU returned the right number of pixels",
          gr.size() == size_t(cp.width) * cp.height, std::to_string(gr.size()));

    long agree = 0, disagreeHit = 0, disagreeMiss = 0;
    float worst = 0.f;
    for (int v = 0; v < cp.height; ++v)
        for (int u = 0; u < cp.width; ++u) {
            const float a = cpu.at<float>(v, u);
            const float b = gr[size_t(v) * cp.width + u];
            const bool aHit = a > 0.f, bHit = b > 0.f;
            if (aHit != bHit) { ++(aHit ? disagreeMiss : disagreeHit); continue; }
            if (!aHit) { ++agree; continue; }
            // One cell of tolerance: the DDA takes the same steps but the
            // float arithmetic is not bit-identical across compilers, and a
            // boundary sample can land either side. A whole cell out is a bug.
            const float e = std::fabs(a - b);
            worst = std::max(worst, e);
            if (e <= fp.cell) ++agree; else ++disagreeHit;
        }
    const long total = long(cp.width) * cp.height;
    std::printf("  agree %ld/%ld  worst range error %.4f m  "
                "hit/miss disagreements %ld/%ld\n",
                agree, total, worst, disagreeHit, disagreeMiss);

    check("ranges agree with the CPU render", agree > total * 99 / 100,
          std::to_string(agree) + "/" + std::to_string(total));
    check("no range is more than a cell out", worst <= fp.cell,
          std::to_string(worst));
    // A hit where the CPU saw sky is the dangerous direction: a phantom
    // obstacle would be integrated as a real one.
    check("the GPU invents no surfaces the CPU did not see", disagreeHit == 0,
          std::to_string(disagreeHit));

    std::printf("%s (%d failures)\n", failures ? "FAILURES" : "all checks passed", failures);
    return failures ? 1 : 0;
}
