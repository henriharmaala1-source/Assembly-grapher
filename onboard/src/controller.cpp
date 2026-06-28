#include "controller.hpp"

#include <algorithm>

float Controller::clamp_(float v) const {
    return std::max(-g_.maxAuthority, std::min(g_.maxAuthority, v));
}

ControlCmd Controller::compute(const WorldState& s, Behavior beh,
                               int frameW, int frameH) const {
    ControlCmd c;
    const float halfW = frameW / 2.0f;

    switch (beh) {
        case Behavior::TRACK: {
            // Observe-from-hover. Target tracking is a SENSING capability: the
            // world model carries the target (s.targetBox) for a camera/gimbal to
            // point at and for the operator to see, but the airframe deliberately
            // does NOT translate or steer toward it. Driving the aircraft onto a
            // tracked target is steer-to-target guidance and is intentionally not
            // implemented here — TRACK holds a level hover and watches.
            c.valid = true;   // all axes zero → stable hover while observing
            break;
        }
        case Behavior::NAVIGATE: {
            if (s.corridorValid) {
                const float ex = (s.corridorHeading.x - halfW) / halfW;
                c.yaw   = clamp_(g_.kpYawNav * ex);
                c.pitch = clamp_(g_.cruise * std::max(0.3f, s.corridorOpen));
                c.valid = true;
            }
            break;
        }
        case Behavior::ROAD_FOLLOW: {
            if (s.roadValid) {
                c.yaw   = clamp_(g_.kpYawRoad * s.roadOffset +
                                 g_.kHeadRoad * s.roadHeading);
                c.pitch = clamp_(g_.cruise);
                c.valid = true;
            }
            break;
        }
        case Behavior::SEARCH: {
            c.yaw   = clamp_(g_.scanYaw);   // slow yaw scan, no translation
            c.valid = true;
            break;
        }
        case Behavior::EVADE: {
            c.pitch = clamp_(-g_.cruise);   // back off
            c.valid = true;
            break;
        }
        case Behavior::HOLD: {
            c.valid = true;                 // level hover, all zero
            break;
        }
        case Behavior::MANUAL:
        case Behavior::IDLE:
        case Behavior::RTL:
        default:
            c.valid = false;                // no override
            break;
    }
    (void)frameH;
    return c;
}
