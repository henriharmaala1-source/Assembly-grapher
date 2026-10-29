#include "place_descriptor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

PlaceDescriptor describePlace(const VoxelMap& map, float px, float py, float pz,
                              float yawDeg, const PlaceParams& p) {
    PlaceDescriptor d;
    d.nAz = p.nAz; d.nRing = 1;
    d.e = px; d.n = py; d.u = pz; d.yawDeg = yawDeg;
    d.range.assign(p.nAz, 0.f);
    d.height.assign(p.nAz, 0.f);
    d.known.assign(p.nAz, 0);
    d.hit.assign(p.nAz, 0);
    const float cell = map.params().cell;

    // WHAT IS STORED PER AZIMUTH: the RANGE to the nearest occupied cell,
    // plus the max height along that bearing. A first version stored height
    // per (azimuth, range) bin, following Scan Context directly, and it did not
    // work here -- indoor heights are nearly binary (a wall, or nothing), so
    // every descriptor was a similar vector of similar numbers and cosine
    // similarity crushed everything to ~0.95 regardless of place.
    //
    // Range is the discriminative channel for this data. A large square room, a
    // narrow corridor and a wide shallow room have plainly different range
    // signatures, and the map already holds the geometry to read them off. This
    // is effectively a 360-degree virtual scan taken from the ACCUMULATED map
    // rather than from one keyhole frame -- which is the whole reason this can
    // work on a 70-degree sensor at all.
    const int nZ = std::max(1, int((p.zHi - p.zLo) / cell));
    for (int a = 0; a < p.nAz; ++a) {
        const float az = yawDeg + (a + 0.5f) * 360.f / p.nAz;
        const float sa = std::sin(az * sim::PI_F / 180.f);
        const float ca = std::cos(az * sim::PI_F / 180.f);

        float hitR = -1.f, hitH = 0.f;
        int   knownSteps = 0, steps = 0;
        for (float rr = p.minR; rr < p.maxR; rr += cell * 0.5f) {
            const float wx = px + sa * rr, wy = py + ca * rr;
            bool anyKnown = false, anyOcc = false; float h = 0.f;
            for (int k = 0; k < nZ; ++k) {
                const float wz = pz + p.zLo + (k + 0.5f) * cell;
                const VoxelMap::State st = map.stateAt(wx, wy, wz);
                if (st == VoxelMap::UNKNOWN) continue;
                anyKnown = true;
                if (st == VoxelMap::OCCUPIED) { anyOcc = true; h = std::max(h, wz - pz); }
            }
            ++steps;
            if (anyKnown) ++knownSteps;
            if (anyOcc) { hitR = rr; hitH = h; break; }
            // STOP AT THE FIRST UNKNOWN STEP. Marching past it would measure a
            // range through space never looked at, which is the same error as
            // treating unknown as free -- the ray would report the far wall of
            // a room it has not entered.
            if (!anyKnown) break;
        }
        const bool observed = steps > 0
            && float(knownSteps) / float(steps) >= p.minKnownFrac;
        d.known[a]  = observed ? 1 : 0;
        // An observed bearing with no hit records maxR: "I looked this far and
        // found nothing", which is a real measurement. An unobserved one
        // records the same number but is masked out, and the mask is the only
        // thing that separates them.
        d.range[a]  = observed ? ((hitR < 0.f) ? p.maxR : hitR) : 0.f;
        d.hit[a]    = (observed && hitR >= 0.f) ? 1 : 0;
        d.height[a] = observed ? hitH : 0.f;
        if (observed) ++d.observedBins;
    }
    return d;
}

PlaceMatch matchPlace(const PlaceDescriptor& a, const PlaceDescriptor& b,
                      int minOverlap) {
    PlaceMatch m;
    if (!a.valid() || !b.valid() || a.nAz != b.nAz) return m;

    float bestScore = -1e9f;
    int   bestShift = 0, bestOverlap = 0;

    for (int s = 0; s < a.nAz; ++s) {
        // Mean absolute range difference over commonly-observed bearings,
        // turned into a similarity. Absolute difference rather than cosine
        // because range profiles are non-negative and of similar magnitude
        // everywhere, so cosine reads ~0.95 for any two indoor places -- the
        // first version of this measured exactly that and could not separate
        // three deliberately different rooms.
        double err = 0.0; int n = 0;
        for (int col = 0; col < a.nAz; ++col) {
            const int bcol = (col + s) % a.nAz;
            // BOTH must have hit a surface. See PlaceDescriptor::hit.
            if (!a.hit[col] || !b.hit[bcol]) continue;
            err += std::fabs(double(a.range[col]) - double(b.range[bcol]));
            ++n;
        }
        if (n < a.nAz / 6) continue;
        const float sc = -float(err / n);          // less error = better
        if (sc > bestScore) { bestScore = sc; bestShift = s; bestOverlap = n; }
    }
    if (bestOverlap == 0) return m;

    // Report as 0..1 so a threshold is readable. 3 m of mean range disagreement
    // is the scale over which two indoor places stop resembling each other;
    // beyond it the score simply floors rather than going negative.
    m.score = std::max(0.f, 1.f + bestScore / 3.f);
    m.shift = bestShift;
    m.yawDeg = bestShift * 360.f / a.nAz;
    m.overlapBins = bestOverlap;
    m.usable = bestOverlap >= minOverlap;
    return m;
}

}  // namespace sim
