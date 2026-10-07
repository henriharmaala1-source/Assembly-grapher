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

// THE LOOK, and only the look: colours in BGR 0..1. The defaults are the
// original footage exactly, so every caller that does not pass one is
// unchanged. The showcase passes an architectural one (flight_view.cpp):
// geometry, lighting and haze are the same in every style.
struct FootageStyle {
    cv::Vec3f skyTop{0.86f, 0.62f, 0.38f}, skyHor{0.95f, 0.88f, 0.80f};
    cv::Vec3f groundA{0.24f, 0.47f, 0.36f}, groundB{0.27f, 0.52f, 0.40f};
    cv::Vec3f warmA{0.36f, 0.46f, 0.57f}, warmB{0.40f, 0.50f, 0.61f};   // tex >= 0.45
    cv::Vec3f wallA{0.66f, 0.68f, 0.70f}, wallB{0.72f, 0.73f, 0.74f};
    float gridM = 0.f;          // > 0: a line on the ground every gridM metres
    float gridDark = 0.22f;     //   ... this much darker
    float panelM = 0.f;         // > 0: walls tinted per panel of this size
    float panelTint = 0.f;      //   ... by up to this much
    float contactM = 0.f;       // > 0: walls darken within this of the ground
    float hazeK = 1.f;          // distance haze strength (1 = the original)
};

// BGR, `w` x `h`, from `pose` with horizontal field of view `hfovDeg`.
// `groundZ` is the world height treated as terrain (coloured as ground).
cv::Mat renderFootage(const VoxelWorld& world, const CamPose& pose,
                      int w, int h, float hfovDeg = 87.f,
                      float maxRangeM = 60.f, float groundZ = 0.3f,
                      const FootageStyle& style = FootageStyle());

}  // namespace sim
