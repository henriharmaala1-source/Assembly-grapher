#pragma once

#include <algorithm>
#include <cmath>

// VioFrameAligner -- what the VIO pose goes through on its way into EKF3.
//
// TWO THINGS ARDUPILOT DOES NOT DO FOR A COMPANION, done here instead:
//
//   CONTINUITY ACROSS RESETS. kestrel's VIO counts a re-anchor (tracking lost
//   and regained, a SLAM map change) as a "reset". ArduPilot reads a change of
//   reset_counter as "snap your position to my new one" (VISO_TYPE 1:
//   ResetPositionNE onto it) or "freeze where I was" (VISO_TYPE 3: re-anchor
//   to the last pose received). Neither is what happened -- the aircraft kept
//   moving through the gap. So a reset is absorbed HERE: the new frame is
//   offset so the output continues from where the last output was heading
//   (constant-velocity across the gap, capped), and the counter sent to
//   ArduPilot never changes. The FC sees one continuous track; the VIO's
//   jump becomes an offset, which is what a re-anchor is.
//
//   YAW HELD TO THE FC'S. With compass yaw (EK3_SRC1_YAW = 1) the EKF's
//   heading and the VIO's drift apart, and every degree between them is a
//   degree of rotation between "where the VIO says I am" and "which way the
//   EKF thinks I face": position hold circles ("toilet bowling"). ArduPilot
//   aligns VIO yaw only once (VOXL backend) or never (MAVLink backend). Here
//   the VIO frame is snapped to the FC heading at the first sample -- so
//   ArduPilot's own one-shot trim comes out zero -- and then slewed toward it,
//   slowly and rate-limited, rotating about the CURRENT position so the
//   output never jumps. With external-nav yaw (EK3_SRC1_YAW = 6) the FC's
//   heading IS the VIO's: pass no reference and nothing is rotated.
//
// Frame: local NED metres, yaw radians clockwise from north (what goes out).
class VioFrameAligner {
public:
    struct Params {
        float slewGain   = 0.2f;                   // 1/s: ~5 s to close a yaw gap
        float maxSlewRad = 2.f * 3.14159265f / 180.f;   // per second, at most
        float maxGapS    = 0.5f;                   // extrapolate a reset gap this far, no more
    };
    struct Pose { float n = 0, e = 0, d = 0, yaw = 0, vn = 0, ve = 0, vd = 0; };

    VioFrameAligner() = default;
    explicit VioFrameAligner(Params p) : p_(p) {}

    // One VIO sample. `refYaw`: the FC's heading now (radians), or NaN for
    // "do not align". Returns the pose to send.
    Pose update(const Pose& in, int resets, double tS, float refYaw) {
        const bool haveRef = std::isfinite(refYaw);
        if (!started_) {
            started_ = true;
            dyaw_ = haveRef ? wrap(refYaw - in.yaw) : 0.f;
            tn_ = te_ = td_ = 0.f;
            resets_ = resets;
            tLast_ = tS;
            return last_ = apply(in);
        }
        const float dt = std::max(0.f, float(tS - tLast_));
        tLast_ = tS;

        if (resets != resets_) {
            // Where the track was going, and where the new frame puts it now:
            // the difference becomes part of the offset.
            resets_ = resets;
            const float g = std::min(dt, p_.maxGapS);
            const Pose raw = apply(in);
            tn_ += last_.n + last_.vn * g - raw.n;
            te_ += last_.e + last_.ve * g - raw.e;
            td_ += last_.d + last_.vd * g - raw.d;
            ++absorbed_;
        }

        if (haveRef && dt > 0.f) {
            const Pose before = apply(in);
            const float err  = wrap(refYaw - before.yaw);
            const float step = std::max(-p_.maxSlewRad * dt,
                                        std::min(p_.maxSlewRad * dt, err * p_.slewGain * dt));
            dyaw_ = wrap(dyaw_ + step);
            // Rotate about where we are: re-solve the offset so this sample's
            // position is unchanged by the new rotation.
            const Pose after = apply(in);
            tn_ += before.n - after.n;
            te_ += before.e - after.e;
        }
        return last_ = apply(in);
    }

    float yawOffsetRad() const { return dyaw_; }
    int   resetsAbsorbed() const { return absorbed_; }

private:
    static float wrap(float a) {
        while (a >  3.14159265f) a -= 6.2831853f;
        while (a < -3.14159265f) a += 6.2831853f;
        return a;
    }
    Pose apply(const Pose& in) const {
        const float c = std::cos(dyaw_), s = std::sin(dyaw_);
        Pose o;
        o.n  = c * in.n - s * in.e + tn_;
        o.e  = s * in.n + c * in.e + te_;
        o.d  = in.d + td_;
        o.yaw = wrap(in.yaw + dyaw_);
        o.vn = c * in.vn - s * in.ve;
        o.ve = s * in.vn + c * in.ve;
        o.vd = in.vd;
        return o;
    }

    Params p_;
    bool   started_ = false;
    float  dyaw_ = 0.f, tn_ = 0.f, te_ = 0.f, td_ = 0.f;
    int    resets_ = 0, absorbed_ = 0;
    double tLast_ = 0.0;
    Pose   last_;
};
