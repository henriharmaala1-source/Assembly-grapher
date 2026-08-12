#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "depth_camera.hpp"
#include "voxel_world.hpp"

// ---------------------------------------------------------------------------
// The ESTIMATED 3D world model — log-odds occupancy built from noisy depth.
//
// This is the thing under test. VoxelWorld is truth; this is what the aircraft
// would actually believe, and score() measures the difference.
//
// THREE STATES, NOT TWO. Every cell is free, occupied, or UNKNOWN, and unknown
// is the default. This is not a stylistic choice: a two-state map has no way to
// say "I cannot see here", so an untextured wall that returns no disparity is
// indistinguishable from open air. Free space is carved ONLY along rays that
// terminated in a valid measurement; a ray with no return contributes nothing.
// That single rule is the difference between a map that is merely incomplete
// and one that will fly an aircraft into a glass facade.
//
// EGOCENTRIC AND SHORT-LIVED. The grid is world-anchored but recentred when the
// vehicle approaches an edge, and shifting decays what scrolls back in. The map
// is only ever as good as the pose estimate, and pretending otherwise is how
// map-driven avoidance kills aircraft. Keep it local, keep it recent.
// ---------------------------------------------------------------------------

namespace sim {

struct VoxelMapParams {
    float cell     = 0.25f;
    int   nx       = 240, ny = 240, nz = 96;   // ~60 x 60 x 24 m at 0.25 m
    // MINIMUM TRUSTED RANGE. Below this a return is NOT A MEASUREMENT and is
    // treated exactly like an invalid pixel: no mark, no carve, no information.
    //
    // Every stereo camera has one, and inside it the sensor does not fail
    // quietly -- it fails CONFIDENTLY. At 0.15 m on the D435i the disparity is
    // past the matcher's search range, the occlusion band is f*B/Z = 447*0.05
    // /0.15 = 149 px (18 % of an 848-wide frame with no counterpart in the
    // right image at all), and a close surface reflects enough of the ~1 W
    // projector to saturate both imagers. What comes back is the speckle class
    // depth_camera.hpp already warns about: "a hole is safe, a confident wrong
    // depth is not."
    //
    // Marking those returns is worse than useless, and the log-odds asymmetry
    // is why. lHit/lMiss = 0.85/0.40, so ONE spurious hit cancels two carving
    // passes: a spurious rate above roughly one frame in three holds a cell
    // OCCUPIED indefinitely. Near-field garbage is far above that, so it wins
    // the argument -- and it wins it in the cells the aircraft is sitting in,
    // where sphereClear's core test then rejects every primitive at its first
    // point. Observed on a real D435i by holding a hand near the lens: voxels
    // appear at the eye, flickering, and the first-person view goes solid.
    //
    // 0 disables, restoring the old behaviour exactly.
    float minIntegM = 0.25f;
    float lHit     = 0.85f;    // log-odds added at a ray's termination
    float lMiss    = 0.40f;    // log-odds removed along the free part of a ray
    float lClamp   = 4.0f;     // saturation, so the map can still change its mind
    float occThresh= 0.0f;     // > this = occupied
    float freeThresh = -0.4f;  // < this = free; between the two = unknown
    // MEASURED, and the single most important number in this file.
    //
    // Depth error grows as dZ = Z^2 * sigma_d / (f*B). Carving free space is
    // safe only while dZ stays under about one voxel: past that the ray's
    // endpoint lands SHORT of the real surface and the DDA carves free space
    // straight through a solid obstacle. Solving dZ = cell:
    //
    //     Z_max = sqrt(cell * f * B / sigma_d)
    //
    // Sweep on the forest world (f=228 px, B=0.12 m, cell=0.25 m, so the
    // formula predicts 7.4 m), scoring FALSE-FREE cells -- solid in truth,
    // called free by the map, i.e. places a planner would fly into something:
    //
    //     integrate to    4 m     6 m     8 m    12 m    18 m    25 m
    //     truth depth   0.00%   0.00%   0.00%   0.00%   0.00%   0.00%
    //     stereo        0.00%   0.07%   2.95%   7.69%  10.81%  13.22%
    //
    // Truth depth is 0.00% at EVERY range, which is what proves the mapper and
    // the three-state rule are correct. Everything above is the sensor.
    //
    // So the formula is a slight over-estimate; derate it ~25%. Practical
    // Z_max at sigma_d = 0.25 px:
    //
    //     f px   B      cell 0.15   0.25    0.5
    //      228   12 cm       4.1     5.2    7.4    (this demo)
    //      520   12 cm       6.1     7.9   11.2    (IMX296 at 728x544)
    //     1040   12 cm       8.7    11.2   15.8    (IMX296 full res)
    //      520   25 cm       8.8    11.4   16.1
    //
    // The blunt consequence: a 12 cm baseline supports voxel mapping to roughly
    // 8-11 m, NOT to the 20-30 m the raw depth range suggests. Returns beyond
    // Z_max are not merely imprecise, they are actively dangerous, because the
    // error is what carves the hole. If you need map out to 20 m, either take
    // coarser voxels at range or stop carving and only mark occupancy.
    float maxIntegM= 8.f;
    // FREE-SPACE CARVING IS A SEPARATE DECISION FROM MARKING AN OBSTACLE, and
    // conflating them was a paralysing bug.
    //
    // maxIntegM exists because a far return's POSITION is untrustworthy -- the
    // error grows as Z^2 and past Z_max it carves through the very obstacle it
    // found. But the original code discarded the ENTIRE RAY when the endpoint
    // was too far, throwing away the free space along it as well. Measured at
    // the city spawn: the nearest truth return is 10.39 m and ZERO pixels are
    // under 8 m, so with maxIntegM = 8 not one ray was integrated, the map
    // stayed completely empty, nothing was ever FREE, and the aircraft sat
    // still for 1200 steps. The stereo runs only moved because 0.4% speckle
    // outliers landed inside 8 m -- they were flying on sensor noise.
    //
    // A ray that returns 30 m still proves the first several metres are empty.
    // So carve free space out to (r - k*sigma(r)), the point beyond which the
    // endpoint's own uncertainty makes the claim unsafe, capped at maxCarveM;
    // and mark OCCUPIED only when r <= maxIntegM.
    float maxCarveM = 25.f;   // free space may be claimed this far out
    float carveSigK = 2.0f;   // stop carving this many depth-sigmas short of the hit
    // sigma(Z) = Z^2 * subpixel / (f*B); supplied by the camera so the map does
    // not have to know the optics. 0 disables the shortening.
    float depthSigCoef = 0.f;  // = subpixelPx / (f_px * baseline_m)

    // CARVE ONLY TO THE LOCAL NEAREST RETURN. This is the guard against the
    // failure that an unmatched obstacle is not merely unknown -- it is carved
    // FREE by the rays that see past it.
    //
    // Measured: with trunks made hard for stereo, perfect depth gives 0.000%
    // false-free and no collision, while stereo gives 7.8% and flies into a
    // tree. The map is not failing to see the trunk; it is actively claiming
    // the trunk's cells are empty, because the pixels either side of it
    // returned the background and the DDA carved right through.
    //
    // The rule: a ray may claim free space only as far as the NEAREST valid
    // return in its pixel neighbourhood, plus a little slack. On smooth open
    // ground the neighbourhood minimum is within centimetres of the centre
    // pixel and nothing changes. At a depth discontinuity -- which is exactly
    // what the silhouette of an unmatched object looks like -- it clamps hard,
    // and the cells behind stay UNKNOWN instead of becoming a lie.
    //
    // This is deliberately independent of any belief about how visible bark
    // is. It says "do not claim free space across a depth edge", which is true
    // whatever the sensor turns out to do.
    int   carveWinPx  = 5;     // neighbourhood width in pixels; 0 disables
    float carveSlackM = 0.5f;  // tolerance so smooth surfaces are unaffected

    // Use every Nth pixel of the depth image. 1 is every pixel.
    //
    // This is what makes a second, coarser map affordable. A 2 m map does not
    // need 76,800 rays: at 10 m, adjacent pixels are 4 cm apart and hundreds of
    // them land in the same 2 m cell, so all but a handful are redundant work.
    // Stride 4 is a sixteenth of the rays for a map whose cells are 8x wider --
    // still several rays per cell at every range that matters.
    //
    // On the FINE map leave this at 1. There the rays are not redundant; that
    // is the whole point of a fine map.
    int   integrateStride = 1;
};

// Rendering knobs for the first-person raycast render. The defaults ARE the
// first-person ones, so every caller that does not mention this gets exactly
// what it got before.
//
// Only the chase view overrides them, and only the unknown fog. That fog is an
// honesty device for a view from the aircraft: it says "you are looking through
// space nobody has measured". Move the camera outside the aircraft and the same
// number stops meaning that -- the unknown between a chase camera and the scene
// is unknown because I chose to stand back there, not because the map is
// ignorant of it. Charged at first-person strength it saturates on the first
// ray and whites out the pane, which is exactly how the chase view looked until
// I rendered it and went looking for the term responsible.
struct FpvStyle {
    float unknownFogM   = 6.f;      // metres of UNKNOWN that saturates the fog
    float unknownFogMax = 0.85f;    // and how opaque saturation is
};

class VoxelMap {
public:
    void init(const VoxelMapParams& p, float cx, float cy, float cz);
    const VoxelMapParams& params() const { return p_; }

    // Fold one depth image in. `cam` supplies the ray directions so the mapper
    // and the renderer cannot disagree about geometry.
    void integrate(const cv::Mat& depth, const DepthCamera& cam, const CamPose& pose);
    // Same, but also records a per-cell INTENSITY from an aligned image, so the
    // map can be drawn with the world's own appearance instead of a synthetic
    // height ramp. One byte per cell -- 5.5 MB at 240x240x96, nothing on a Pi.
    //
    // Be clear what this is for: it buys the AVIONICS nothing. It exists so a
    // human can look at the reconstruction and tell at a glance whether the map
    // resembles the place, which a height-coloured blob cannot show. Treat it
    // as debug instrumentation, not perception.
    void integrate(const cv::Mat& depth, const cv::Mat& intensity,
                   const DepthCamera& cam, const CamPose& pose);
    // 0 if never observed.
    uint8_t texAt(int x, int y, int z) const {
        return (!tex_.empty() && inBounds(x, y, z)) ? tex_[idx(x, y, z)] : 0;
    }
    bool hasTexture() const { return !tex_.empty(); }

    // Recentre on the vehicle, decaying anything that scrolls in from outside.
    void recentre(float cx, float cy, float cz);

    // Mark a ball as FREE. Exactly one legitimate use: the aircraft knows it is
    // not inside an obstacle at the moment it takes off, and without that one
    // fact a planner that requires positively-confirmed free space can never
    // start -- it cannot confirm without moving and cannot move without
    // confirming. Measured: 0.0 m travelled, stopped on 400 of 400 steps.
    //
    // Do not reach for this anywhere else. Asserting free space you have not
    // observed is the exact failure this whole map exists to prevent.
    void seedFree(float cx, float cy, float cz, float radiusM);

    enum State : uint8_t { UNKNOWN = 0, FREE = 1, OCCUPIED = 2 };
    State stateAt(float wx, float wy, float wz) const;
    float logAt(int x, int y, int z) const {
        return inBounds(x, y, z) ? log_[idx(x, y, z)] : 0.f;
    }
    bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < p_.nx && y < p_.ny && z < p_.nz;
    }
    void worldToCell(float wx, float wy, float wz, int& x, int& y, int& z) const;
    void cellCentre(int x, int y, int z, float& wx, float& wy, float& wz) const;

    // Result of marching a ray through the map. `face` is which side of the
    // cell the ray entered through (0/1 = -x/+x, 2/3 = -y/+y, 4/5 = -z/+z),
    // which is what makes a first-person render read as cubes rather than as a
    // depth field: shading by face is the whole trick.
    struct Hit {
        float t = 0;                 // distance along the ray, metres
        int   face = 5;
        int   x = 0, y = 0, z = 0;   // cell hit
        // How much UNKNOWN the ray crossed before it hit anything. Rendered as
        // fog. The map's central claim is that unknown is NOT free, and a
        // first-person view that draws unknown as clear air would be the most
        // convincing possible way to tell that lie.
        float unknownM = 0;
        bool  hit = false;
    };
    // Amanatides & Woo, same traversal as VoxelWorld::raycast. Stops on the
    // first OCCUPIED cell; UNKNOWN does not stop the ray, it accumulates.
    Hit raycast(float px, float py, float pz,
                float dx, float dy, float dz, float maxRange) const;

    // --- scoring against truth ---------------------------------------------
    //
    // The numbers that actually answer "does the voxel model work".
    struct Score {
        long occTP = 0, occFP = 0, occFN = 0;   // occupied-cell confusion
        long freeTP = 0, freeFP = 0;            // free-cell confusion
        long unknown = 0, total = 0;
        // Cells the map has an OPINION about. IoU must be computed over these,
        // not over every solid cell in the region: in a forest most solid
        // voxels are inside a canopy, behind a trunk or underground, and no
        // camera can ever see them. Scoring them as misses measures occlusion,
        // not mapping quality, and buries the signal you actually want.
        long observed = 0;
        // THE SAFETY NUMBER. Cells the map calls FREE that are actually SOLID.
        // Every one of these is a potential collision. Weight it above all else.
        long falseFree = 0;
        // IoU over OBSERVED cells only (see `observed`).
        double iou() const {
            double d = double(occTP + occFP + falseFree);
            return d > 0 ? double(occTP) / d : 0.0;
        }
        // Of the cells the map calls occupied, how many really are.
        double precision() const {
            double d = double(occTP + occFP);
            return d > 0 ? double(occTP) / d : 0.0;
        }
        // Fraction of the scored region the map has any opinion about.
        double coverage() const {
            return total > 0 ? double(observed) / double(total) : 0.0;
        }
        double falseFreeRate() const {
            double d = double(freeTP + falseFree);
            return d > 0 ? double(falseFree) / d : 0.0;
        }
    };
    // Compares only cells within `radiusM` of (cx,cy,cz) and below `maxZ`, so
    // the score is about the region the vehicle could actually hit rather than
    // being dominated by unobserved sky.
    Score score(const VoxelWorld& truth, float cx, float cy, float cz,
                float radiusM, float maxZ) const;

    // --- visualisation ------------------------------------------------------
    // Horizontal slice at height wz: grey = unknown, white = free, black = occ.
    cv::Mat sliceImage(float wz, int outPx = 480) const;
    // Isometric projection of OCCUPIED cells, height-coloured, rotatable about
    // the vertical axis by yawDeg. Cheap, no GPU, no scene graph -- painter's
    // algorithm back-to-front. Enough to see whether the map the aircraft built
    // actually resembles the world it flew through, which a 2D slice cannot
    // show: a slice at one height hides everything above and below it.
    // The projection isoImage used, so a caller can draw INTO that image in
    // world coordinates. Handing back the parameters rather than exposing a
    // draw-a-line-for-me method keeps the renderer ignorant of paths, trails
    // and goals -- and it guarantees the overlay cannot drift out of register
    // with the cubes, because there is only one projection to get wrong.
    struct IsoView {
        int   outPx = 0;
        int   step  = 1;
        float s = 0;                 // cube edge, pixels
        float ca = 1, sa = 0;        // yaw rotation
        float cxg = 0, cyg = 0, czg = 0;   // view centre, in block units
        float ox = 0, oy = 0, oz = 0, cell = 1;
        bool  valid = false;
        // Everything is expressed RELATIVE to the view centre. Adding the
        // centre back after rotating (which an earlier version did) makes the
        // vertical placement depend on the block pitch, so changing the pitch
        // slid the model up and down the pane for no reason a viewer could see.
        cv::Point2f project(float wx, float wy, float wz) const {
            const float b = cell * step;
            float dx = (wx - ox) / b - cxg, dy = (wy - oy) / b - cyg;
            float dz = (wz - oz) / b - czg;
            float rx = dx * ca - dy * sa, ry = dx * sa + dy * ca;
            return {(rx - ry) * s * 0.5f + outPx * 0.5f,
                    (rx + ry) * s * 0.25f - dz * s * 0.5f + outPx * 0.58f};
        }
    };
    // blockM is the DISPLAY pitch in metres, not the map resolution. At the
    // map's own 0.25 m a 60 m map is 240 blocks across a 440 px pane -- one
    // pixel each, which is why this pane read as noise. Blocks are OR-reduced
    // from the cells inside them, so nothing solid is ever lost to the display.
    // spanM is how many metres across to show, centred on the vehicle -- the
    // render distance. <=0 means the whole map. It matters because the map is
    // 60 m wide while the mapped part is often 10-15 m across, so scaling to
    // the map extent draws the model as a thumbnail in the middle of an empty
    // pane. Cropping to a span makes the blocks big enough to read AND is the
    // honest framing: it says "this is what the aircraft knows within N metres".
    // colourByTexture: draw cells with their recorded intensity instead of the
    // height ramp. Falls back to height if no intensity was ever integrated.
    cv::Mat isoImage(int outPx = 640, float maxZ = 30.f, float yawDeg = 0.f,
                     IsoView* view = nullptr, float blockM = 1.5f,
                     float spanM = 0.f, bool colourByTexture = false) const;

    // FIRST PERSON, out of the aircraft's own eyes, through the map it built.
    // One raycast per pixel, shaded by which cube face was hit -- the same
    // three brightnesses the isometric cubes use, for the same reason.
    //
    // This renders the MAP, never the world, and the distinction is the entire
    // point: the isometric pane shows you the model from outside, and this
    // shows you what flying inside that model would look like. Where the model
    // is wrong you fly into fog, and fog you can see is worth more than a
    // false-free percentage you have to interpret.
    //
    // Cost is ~0.5 us/ray, so a 320 px square pane is roughly 50 ms. That is
    // fine for a desktop window and would never run onboard; it is a
    // visualisation, not part of the flight loop.
    //
    // `maxRange` should be set from what the map can honestly know rather than
    // from what looks good: at 0.25 m cells on a 12 cm baseline that is about
    // 5 m, and the view SHOULD look short. The horizon here is a sensor
    // property, and seeing it is the useful part.
    cv::Mat fpvImage(float px, float py, float pz,
                     float yawDeg, float pitchDeg,
                     int outPx = 320, float hfovDeg = 90.f,
                     float maxRange = 25.f) const;

    // Non-square variant, for OVERLAYING the map on the depth image it was
    // built from. Alignment is the whole point of the overlay, so the render
    // has to match the camera's aspect AND its horizontal FOV -- a square
    // render letterboxed onto a 16:9 frame is wrong in the vertical by
    // whatever the aspect ratio is, which looks like a calibration error and
    // is really a rendering shortcut. The vertical FOV follows from the
    // horizontal one and the aspect, exactly as it does in a real pinhole.
    // Pixel -> world ray for the first-person render, and its exact inverse.
    // Both are here, next to each other and used by the renderer itself, so a
    // path drawn into that view lands where the geometry says it does. A
    // projection written separately from the render it draws into is a sign
    // error waiting to happen, and one that looks plausible.
    static void fpvRay(float yawDeg, float pitchDeg, int outW, int outH,
                       float hfovDeg, float u, float v,
                       float& dx, float& dy, float& dz);
    // Returns false when the point is behind the camera or off the image.
    static bool fpvProject(float px, float py, float pz,
                           float yawDeg, float pitchDeg, int outW, int outH,
                           float hfovDeg, float wx, float wy, float wz,
                           float& u, float& v);

    cv::Mat fpvImageWH(float px, float py, float pz,
                       float yawDeg, float pitchDeg,
                       int outW, int outH, float hfovDeg,
                       float maxRange, cv::Mat* hitMask = nullptr,
                       const FpvStyle& style = FpvStyle(),
                       cv::Mat* hitT = nullptr) const;

    // Render several maps from one eye, FINEST FIRST, taking the first level
    // that has an opinion inside its own band.
    //
    // This exists because the ladder is otherwise invisible. Every view here
    // used to draw ONE level, and the level it picked was the finest -- whose
    // honest marking range is 3.5 m at 0.25 m cells, or 2.2 m at 0.10 m. So a
    // scene whose nearest trunk is four metres away rendered as an empty room,
    // in the sim and on the real camera both, and looked like a broken mapper
    // rather than a correctly-scoped one. The coarse level DOES mark that trunk
    // -- 2 m cells reach 10 m -- it simply had nowhere to appear.
    //
    // Order matters: pass the levels finest first. The first one whose hit
    // falls inside its own band wins, so fine detail is used wherever it exists
    // and the coarse level carries only the range beyond it.
    // A level is consulted ONLY inside its own honest BAND. Both ends matter,
    // and the far end was the obvious one.
    //
    // The near end is the one that produced a real fault. The first version of
    // this took the NEAREST hit across all levels, on the argument that "where
    // two levels both know about a surface the finer one is in front of the
    // coarser one's blocky approximation and wins on its own merits". **That is
    // false.** A 2 m cell containing a surface at 3 m has its near FACE at up to
    // 2 m — nearer than the fine level's accurate 3 m hit — so the coarse level
    // systematically wins in the near field and draws the surface far too close.
    // Indoors, where everything is within a few metres, single 2 m cubes then
    // subtend 50 deg or more and the pane goes solid. Observed on a real camera
    // in a room, with a clean 97 % valid depth image going in.
    //
    // The planner already had the right rule: `voxel_traj.cpp` marches "fine
    // first ... if (t <= c.rangeM)". This is that rule, in the renderer.
    struct Layer {
        const VoxelMap* map = nullptr;
        float minRange = 0.f;           // start of this level's band
        float range = 0.f;              // end of it — its honest range
    };
    static cv::Mat renderLadder(const std::vector<Layer>& layers,
                                float px, float py, float pz,
                                float yawDeg, float pitchDeg,
                                int outW, int outH, float hfovDeg,
                                const FpvStyle& style = FpvStyle(),
                                cv::Mat* hitMask = nullptr);

private:
    size_t idx(int x, int y, int z) const { return (size_t(z) * p_.ny + y) * p_.nx + x; }
    void rayInsert(float px, float py, float pz, float dx, float dy, float dz,
                   float carveTo, float hitAt);

    VoxelMapParams p_;
    std::vector<float> log_;
    std::vector<uint8_t> tex_;         // per-cell intensity; empty until used
    float ox_ = 0, oy_ = 0, oz_ = 0;   // world coord of cell (0,0,0) min corner
};

// Side-by-side truth vs estimate isometric view, with the score printed on it.
cv::Mat compareImage(const VoxelWorld& truth, const VoxelMap& map,
                     const VoxelMap::Score& s, int outPx = 640);

}  // namespace sim
