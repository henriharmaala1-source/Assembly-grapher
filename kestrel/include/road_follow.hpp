#pragma once

#include <opencv2/core.hpp>

#include "perception.hpp"
#include "world_model.hpp"

// Appearance-based road / track follower (CPU, no neural net).
//
// Self-supervises a road-colour model from a bottom-centre "we are on the road
// now" ROI in CIELab (shadow-tolerant), scores every pixel against it, keeps the
// connected region under the camera, then reads a per-row centreline. Outputs a
// normalised lateral offset, a near→far heading, and a multiplicative confidence
// that collapses when no road is visible — at which point the behaviour FSM
// falls back to the depth corridor or HOLD.
//
// Pairs with NavigateModule: road follow uses APPEARANCE, the corridor uses
// DEPTH. Different failure modes, so together they cover more ground.
class RoadFollowModule : public IPerceptionModule {
public:
    const char* name()   const override { return "road"; }
    float       costMs() const override { return 6.f; }
    void        run(const cv::Mat& frame, WorldModel& wm) override;

private:
    // Working resolution — half of 640x480; a steering signal doesn't need more.
    static constexpr int   W = 320;
    static constexpr int   H = 240;
    static constexpr float MODEL_EMA = 0.10f;   // road-model temporal blend
    static constexpr float OUT_EMA   = 0.40f;   // output smoothing

    // EMA road-appearance model: [a, b, L, texture] mean + std (CIELab).
    bool  modelInit_ = false;
    float mu_[4]{}, sg_[4]{1, 1, 1, 1};

    float prevOffset_  = 0.f;
    float prevHeading_ = 0.f;
    bool  haveOutput_  = false;
};
