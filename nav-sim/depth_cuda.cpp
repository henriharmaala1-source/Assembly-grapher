// The no-CUDA half of the pair. Always compiled, so every caller has a symbol
// to link against and the tree builds identically with or without a toolkit.
#include "depth_cuda.hpp"

#ifndef NAVSIM_HAVE_CUDA

namespace sim {

bool cudaDepthAvailable() { return false; }

const char* cudaDepthStatus() {
    return "CPU only (built without CUDA; -DNAVSIM_WITH_CUDA=ON to enable)";
}

bool renderRangesCuda(const VoxelWorld&, const CamPose&, int, int, float,
                      float, float, float, float, float,
                      std::vector<float>&, std::vector<float>&) {
    return false;
}

}  // namespace sim

#endif  // !NAVSIM_HAVE_CUDA
