#pragma once

#include <cstdint>
#include <vector>

namespace sim {

class VoxelWorld;
struct CamPose;

// ---------------------------------------------------------------------------
// GPU DEPTH RENDER -- the one part of this sim that may leave the CPU.
//
// WHY THIS PIECE AND NOTHING ELSE. frame_source.hpp states the discipline the
// whole tree is built on: the simulator runs THE SAME CODE THAT FLIES, so a bug
// seen on real data is a bug in the code that would fly. A CUDA VoxelMap or a
// CUDA sphereClear would be a SECOND implementation of the flight code; it
// would drift, and a policy would then be trained against a world model that is
// not the one deployed. That is Langostino's failure with the serial numbers
// filed off, and it is not worth a speedup.
//
// The depth renderer is the exception, on three counts:
//   1. It is the dominant cost -- voxel_sim reports 46.8 ms/step in forest
//      against ~14 ms for everything else, so roughly 75 % of a step.
//   2. It is purely functional: world in, ranges out. No state, no scattered
//      writes, no atomics, so a GPU version is a transliteration rather than a
//      redesign.
//   3. IT IS NOT FLIGHT CODE. voxel_sim labels it "[sim-only]" in its own
//      output, because the real aircraft gets depth from a D435i. A CUDA
//      version forks nothing that ever flies.
//
// STATUS, STATED PLAINLY: the .cu kernel in this pair has NEVER BEEN COMPILED
// OR RUN by its author -- there was no CUDA toolkit or GPU on the machine it
// was written on. Everything around it is exercised (the dispatch, the
// fallback, the equality harness); the kernel body is not. cuda_depth_check is
// a HARD GATE for exactly that reason: on a machine with CUDA it renders the
// same scene both ways and fails the build's test suite if they disagree.
// Do not trust the GPU path until that check has passed on your hardware.
// ---------------------------------------------------------------------------

// True when a CUDA device was found AND the module was built with a toolkit.
bool cudaDepthAvailable();
// Human-readable reason, for the status line. Never empty.
const char* cudaDepthStatus();

// Cast one ray per pixel and return the range at which each first hits solid
// geometry, plus that surface's texture richness. `out`/`tex` are resized to
// w*h. A miss writes a negative range, matching DepthCamera::renderTruth.
//
// Returns false when there is no GPU path, in which case the caller must use
// the CPU renderer -- so this is safe to call unconditionally.
bool renderRangesCuda(const VoxelWorld& w, const CamPose& pose,
                      int width, int height, float hfovDeg,
                      float fxPx, float fyPx, float ppxPx, float ppyPx,
                      float maxRangeM,
                      std::vector<float>& out, std::vector<float>& tex);

}  // namespace sim
