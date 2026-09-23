#pragma once
// ---------------------------------------------------------------------------
// FPV FOOTAGE of the simulated world -- what a camera on the aircraft would
// film, rendered from the TRUE world, not from the map.
//
// Everything else in this tree draws what the aircraft BELIEVES: the voxel
// map, with unknown as fog. That is the right picture for judging the planner
// and the wrong one for showing what the aircraft is flying through, because
// most of a fresh map is fog and a viewer cannot tell a mapping failure from
// air nobody has looked at. This is the other half of that comparison: the
// scene itself, lit and hazed, so the belief can be shown BESIDE it.
//
// IT IS NEVER AN INPUT. Nothing plans on it, nothing maps from it. It is
// ground truth by construction, and the one rule it must keep is that it is
// never mistaken for a sensor: callers caption it as the simulated scene.
//
// Deliberately a plain ray caster through VoxelWorld::raycast -- the same
// traversal the depth camera uses, so the footage and the depth the aircraft
// maps from are of exactly the same geometry. Shading is Lambert from a fixed
// sun on the face each ray enters, surface colour from height and the voxel's
// texture byte (the same byte the stereo model matches on), and distance haze
// to the sky colour.
// ---------------------------------------------------------------------------

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "voxel_world.hpp"

namespace sim {

// BGR, `w` x `h`, from `pose` with horizontal field of view `hfovDeg`.
// `groundZ` is the world height treated as terrain (coloured as ground).
cv::Mat renderFootage(const VoxelWorld& world, const CamPose& pose,
                      int w, int h, float hfovDeg = 87.f,
                      float maxRangeM = 60.f, float groundZ = 0.3f);

}  // namespace sim
