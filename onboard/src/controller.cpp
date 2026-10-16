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
            // Pure SENSING — no control output. The tracker's only product is the
            // bounding box (s.targetBox, with s.targetVel/Conf/Age) in the world
            // model, for a camera/gimbal, the operator, telemetry, or a logger to
            // read. The controller emits NO flight command here (valid=false → no
            // override): the airframe is not driven by the tracked target at all.
            // Steer-to-target guidance is deliberately not implemented.
            c.valid = false;
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
