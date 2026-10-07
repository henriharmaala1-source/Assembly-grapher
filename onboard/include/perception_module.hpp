#pragma once
// The perception-module interface alone, without the modules. perception.hpp
// defines the tracker, detector and ToF modules, which pull in OpenCV's contrib
// tracking headers; a module that needs none of that (VoxelNavModule) includes
// this instead -- and so can be compiled into nav-sim's kestrel, whose Windows
// build uses OpenCV's prebuilt release (no contrib). The showcase flies the
// aircraft's own voxel module and mission, not a copy of them.

#include <opencv2/core.hpp>

#include "world_model.hpp"

// A perception module reads the current frame and writes its findings into the
// WorldModel. The scheduler decides which modules run on a given tick based on
// their cost and the active behaviour, so on a CPU-only Pi 5 the heavy models
// never all fire on the same frame.
class IPerceptionModule {
public:
    virtual ~IPerceptionModule() = default;
    virtual const char* name()   const = 0;
    virtual float       costMs() const = 0;   // approx CPU cost per run
    virtual bool        isReady() const { return true; }
    virtual void        run(const cv::Mat& frame, WorldModel& wm) = 0;
};
