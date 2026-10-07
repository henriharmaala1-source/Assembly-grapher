#pragma once
// HOW A DEPTH IMAGE IS COLOURED, in one place.
//
// This lived inside voxel_live.cpp, which was right while voxel_live was the
// only thing that drew depth. `demo` draws the same frames beside the map they
// build, and two colour ramps for one quantity is the specific failure this
// tree keeps guarding against: the panes would disagree about what 3 m looks
// like and the disagreement would read as a mapping bug.
//
// GREY IS NO RETURN, in both, and it is repainted AFTER the HSV conversion so
// it is a flat grey rather than whatever the hue ramp does at its ends. An
// invalid pixel is not far away; it is unmeasured, and the two must not share
// a colour. (Same rule as the map's UNKNOWN. See CLAUDE.md.)
#include <opencv2/core.hpp>

namespace sim {

// Linear ramp, red near to blue far, saturating at maxM.
cv::Mat colourDepth(const cv::Mat& depthM, float maxM);

// HISTOGRAM-EQUALISED, which is what a real scene usually wants: a fixed ramp
// has to be told the range in advance and then wastes most of its colours on
// the part of it nothing occupies. Found by noticing a lamppost was visible
// only at the longest scale setting -- the sensor had measured it the whole
// time and the ramp could not show it.
cv::Mat colourDepthEq(const cv::Mat& depthM, float maxM);

}  // namespace sim
