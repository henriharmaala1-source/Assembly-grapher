#pragma once

#include <cstdint>
#include <vector>

#include "voxel_map.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// PLACE RECOGNITION FROM GEOMETRY -- "have I been here before?", answered by
// the shape of the world rather than by its appearance.
//
// WHY GEOMETRY AND NOT PICTURES. Appearance-based place recognition is strongly
// viewpoint-dependent: a corridor traversed in reverse does not look like
// itself, which is the single most common way a drone's loop closure fails
// indoors. Geometry does not care which way you were facing when you built it.
//
// THE DESCRIPTOR is Scan Context (Kim & Kim, IROS 2018): bin the surroundings
// into a polar grid of azimuth x range, and record the highest occupied cell in
// each bin. Two things make it the right shape here:
//
//   * ROTATION IS A COLUMN SHIFT. A yaw difference rotates the descriptor
//     round its azimuth axis, so comparing at every shift and keeping the best
//     both matches regardless of heading AND returns the heading offset. That
//     is exactly the trick BearingField already uses to make yaw an index shift
//     rather than a re-projection -- the same idea, one level up.
//   * IT READS THE MAP, NOT A FRAME. One depth frame is a keyhole: 70 degrees
//     wide and honest to 3.54 m. The accumulated MAP is not -- it holds
//     everything the vehicle has driven past. So the aperture argument that
//     kills frame-to-frame place recognition on this sensor does not
//     automatically kill it here, and whether that is enough is a measurement
//     rather than an opinion. See loop_closure_check.
//
// UNKNOWN IS NOT EMPTY, and a descriptor that cannot say so is dangerous in a
// specific way: unexplored space would read as "no obstacle here", which is a
// strong and completely unearned signal. Every bin therefore carries a MASK,
// and similarity is computed only over bins BOTH descriptors have looked at.
// ---------------------------------------------------------------------------

struct PlaceParams {
    int   nAz    = 60;      // azimuth bins; 60 -> 6 degrees each
    int   nRing  = 20;      // range rings
    float maxR   = 10.f;    // metres; beyond this the map holds little
    float minR   = 0.5f;    // ignore the vehicle's own immediate surroundings
    float zLo    = -3.f;    // height band relative to the vehicle
    float zHi    =  3.f;
    // A bin needs this fraction of its samples KNOWN before it counts as
    // observed. Without it a single stray carve would mark a whole bin as
    // looked-at and let a mostly-unexplored place match anything.
    float minKnownFrac = 0.25f;
};

struct PlaceDescriptor {
    // ONE ENTRY PER AZIMUTH, not a full polar grid. Range is the channel that
    // discriminates indoors -- see the note in describePlace about why storing
    // height per range-ring did not work.
    std::vector<float>   range;    // nAz, metres to the nearest occupied cell
    std::vector<float>   height;   // nAz, its height above the vehicle
    std::vector<uint8_t> known;    // nAz, 1 = this bearing was observed
    // 1 only when the ray actually TERMINATED ON A SURFACE. A bearing whose ray
    // stopped at the first unknown cell, or ran to maxR without hitting
    // anything, is observed but carries no range measurement -- and comparing
    // those is what made an early visit and a later one describe the same place
    // differently, because the later one had explored more. Range is only
    // comparable where both visits actually saw a wall.
    std::vector<uint8_t> hit;
    float e = 0, n = 0, u = 0;     // where it was taken (truth, for scoring only)
    float yawDeg = 0.f;            // and the heading it was built relative to
    int   nAz = 0, nRing = 0;
    int   observedBins = 0;

    bool valid() const { return observedBins > 0; }
};

struct PlaceMatch {
    float score = 0.f;        // 0..1 over commonly-observed bins
    int   shift = 0;          // azimuth bins; the heading offset it implies
    float yawDeg = 0.f;       // that shift in degrees
    int   overlapBins = 0;    // how many bins BOTH had looked at
    bool  usable = false;     // enough overlap to mean anything
};

// Build a descriptor from the map as it stands, centred on the vehicle.
// `yawDeg` is the vehicle's heading. The descriptor is built RELATIVE to it,
// which is what makes the column-shift comparison mean something: a
// world-aligned descriptor would need a known heading to build, and heading is
// precisely the thing that drifts, so aligning to it would assume the answer.
// Built vehicle-relative, two visits to one place differ by a rotation, the
// shift that best matches them IS the heading difference, and nothing had to
// know north.
PlaceDescriptor describePlace(const VoxelMap& map, float px, float py, float pz,
                              float yawDeg, const PlaceParams& p = {});

// Best similarity over all azimuth shifts. `minOverlap` is the number of
// commonly-observed bins below which the comparison is refused outright rather
// than reported with a low score -- two places that have barely looked at the
// same space are not dissimilar, they are unjudged, and those are different
// answers.
PlaceMatch matchPlace(const PlaceDescriptor& a, const PlaceDescriptor& b,
                      int minOverlap = 20);

}  // namespace sim
