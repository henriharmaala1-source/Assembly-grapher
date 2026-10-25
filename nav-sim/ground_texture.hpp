#pragma once
// ---------------------------------------------------------------------------
// GroundRenderer -- what a DOWNWARD daylight camera sees: real ground
// photographs at their physical size, lit by the sun.
//
// navcore's IR model (DepthCamera::renderIR) is the D435's: intensity from
// band-limited value noise, lit by its own projector that fades with range.
// Right for the forward depth camera; wrong for a belly camera in daylight,
// which sees grass, gravel, paving and dirt under sunlight that does not fade
// with height. So this is a second renderer for that camera, beside navcore's
// and touching nothing in it:
//
//   MATERIALS  real photographs (nav-sim/textures, fetch_ground_textures.sh),
//              each tiled at a PHYSICAL size (a lawn photo spans 2 m, gravel
//              0.7 m), as two layers at different scale and rotation so the
//              tile never repeats visibly -- a periodic floor is an artefact a
//              tracker can lock onto wrongly, and real ground is not periodic.
//   SCENES     materials laid out the way real ground is: a lawn with mowing
//              stripes, a field with dirt and worn patches, gravel, dry dirt,
//              sand, paving, and a park that mixes them; plus large-scale
//              brightness variation (moisture, wear, shade) at metres to tens of
//              metres, which is what is left to track from altitude once the
//              fine texture has blurred away.
//   SAMPLING   mipmapped by each pixel's ground FOOTPRINT (range / focal
//              length, stretched by obliquity): texture finer than a pixel
//              averages out as it does through a real lens, instead of
//              aliasing into shimmering detail that moves with the camera.
//   LIGHT      a sun (elevation, azimuth) and sky ambient; Lambert shading on
//              objects; their cast shadows; and optionally THE AIRCRAFT'S OWN
//              SHADOW -- a dark blob that moves with the aircraft, the classic
//              false feature for downward flow.
//   CAMERA     auto-exposure with lag, vignetting, shot and read noise, 8-bit;
//              optional motion blur over the exposure time.
//
// Objects in the world (VoxelWorld cells above the floor slab) are rocky
// ground on top and brick on the sides.
// ---------------------------------------------------------------------------

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "voxel_world.hpp"

namespace kground {

// One photograph, grey, as a mip pyramid; tiled at `sizeM` metres per width.
struct Texture {
    std::string name;
    float sizeM = 1.f;
    float mean = 0.5f, stddev = 0.1f;
    std::vector<cv::Mat> mip;            // CV_32F, [0] full resolution
    bool load(const std::string& path, float sizeM, std::string* err);
    // Albedo at ground coordinates (metres), for a pixel footprint `footM`.
    float sample(float x, float y, float footM) const;
};

struct Light {
    float sunElevDeg = 45.f, sunAzDeg = 135.f;   // azimuth clockwise from North
    float sun = 0.75f, sky = 0.35f;              // irradiance shares
    bool  castShadows = true;
};

struct CameraModel {
    float targetMean = 0.45f;     // auto-exposure aims the mean here (of full scale)
    float aeRate = 0.15f;         // per frame: auto-exposure lags, as real AE does
    float vignette = 0.35f;       // brightness loss at the image corner
    float readNoiseDN = 1.5f;     // 8-bit units
    float shotGain = 0.35f;       // shot-noise variance per DN
    float exposureMs = 0.f;       // > 0: motion blur over this
    int   blurSubFrames = 4;
};

// The aircraft's own shadow: a disc of radius `radiusM` at the airframe.
struct DroneShadow { bool on = false; float e = 0, n = 0, u = 0, radiusM = 0.15f; };

class GroundRenderer {
public:
    // Every texture from `dir` (nav-sim/textures). False + `err` if missing:
    // run fetch_ground_textures.sh.
    bool load(const std::string& dir, std::string* err);
    // lawn | field | gravel | dirt | sand | paving | park
    bool setScene(const std::string& name, unsigned seed, float floorTopM);
    static const char* sceneNames();

    // One frame. `pose` is the camera's (navcore CamPose); `blur` the poses
    // across the exposure, oldest first (empty: no blur).
    cv::Mat render(const sim::VoxelWorld& w, const sim::DepthCamera& cam,
                   const sim::CamPose& pose, const std::vector<sim::CamPose>& blur,
                   const DroneShadow& shadow);

    Light light;
    CameraModel camera;

private:
    // Radiance (before exposure) of the world along one pixel's ray.
    float radiance(const sim::VoxelWorld& w, const sim::DepthCamera& cam,
                   const sim::CamPose& pose, int u, int v, const DroneShadow& sh) const;
    float groundAlbedo(float x, float y, float footM) const;

    std::vector<Texture> tex_;
    const Texture* find(const char* name) const;
    std::string scene_ = "field";
    unsigned seed_ = 1;
    float floorTop_ = 0.1f;
    float gain_ = -1.f;                   // auto-exposure state
    std::mt19937 rng_{7};
};

}  // namespace kground
