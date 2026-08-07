#pragma once

#include "gray_frame.hpp"

// ---------------------------------------------------------------------------
// Appearance filters applied to the working crop before correlation. Direct port
// of track/Filters.kt.
//
// Applying the SAME filter to template and search crop makes the correlation
// invariant to whatever the filter removes -- e.g. EDGE discards absolute
// brightness, so a target that dims does not lose lock.
// ---------------------------------------------------------------------------

namespace track {

enum class CropFilter { NONE, STRETCH, EDGE, THRESHOLD, SHARPEN, CHROMA };

const char* cropFilterName(CropFilter f);
bool cropFilterFromName(const char* s, CropFilter& out);

// `dst`, when non-null and EXACTLY g.w*g.h long, receives the result instead of
// a fresh buffer. Exact size is required, not merely sufficient: STRETCH and
// THRESHOLD derive statistics from the buffer length, so a longer buffer would
// fold stale tail values into a percentile or histogram.
GrayFrame applyFilter(const GrayFrame& g, CropFilter f, float* dst, int dstN);

}  // namespace track
