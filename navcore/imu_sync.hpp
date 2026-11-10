#pragma once
// ---------------------------------------------------------------------------
// ImuSync -- the D435i's two motion streams as one.
//
// The D435i sends gyro (200/400 Hz) and accelerometer (63/250 Hz) as separate
// streams with their own timestamps; a visual-inertial estimator wants both
// at each instant. As ORB-SLAM3's own D435i example does, the accelerometer is
// linearly interpolated onto each gyro timestamp. A gyro sample newer than the
// newest accelerometer sample is HELD until one arrives after it: extrapolating
// acceleration would invent motion.
// ---------------------------------------------------------------------------

#include <deque>
#include <vector>

#include "frame_source.hpp"   // navcore: sim::ImuRaw
#include "slam_link.hpp"

class ImuSync {
public:
    // Feed raw samples (any order within a batch; streams are each monotonic).
    void push(const std::vector<sim::ImuRaw>& raw) {
        for (const sim::ImuRaw& r : raw) {
            if (r.gyro) gyro_.push_back(r);
            else        acc_.push_back(r);
        }
    }

    // Emit every gyro sample that can now be paired, oldest first.
    void drain(std::vector<slamlink::ImuSample>& out) {
        while (!gyro_.empty() && acc_.size() >= 2) {
            const sim::ImuRaw& g = gyro_.front();
            if (g.tS < acc_.front().tS) { gyro_.pop_front(); continue; }  // before any accel
            if (g.tS > acc_.back().tS) break;                               // wait for accel
            while (acc_.size() >= 2 && acc_[1].tS < g.tS) acc_.pop_front();
            const sim::ImuRaw& a0 = acc_[0];
            const sim::ImuRaw& a1 = acc_.size() > 1 ? acc_[1] : acc_[0];
            const double span = a1.tS - a0.tS;
            const float k = span > 1e-9 ? float((g.tS - a0.tS) / span) : 0.f;
            slamlink::ImuSample s;
            s.tS = g.tS;
            s.ax = a0.x + k * (a1.x - a0.x);
            s.ay = a0.y + k * (a1.y - a0.y);
            s.az = a0.z + k * (a1.z - a0.z);
            s.gx = g.x; s.gy = g.y; s.gz = g.z;
            out.push_back(s);
            gyro_.pop_front();
        }
        // Bounded: a stalled accelerometer must not grow memory for ever.
        while (gyro_.size() > 2000) gyro_.pop_front();
    }

private:
    std::deque<sim::ImuRaw> gyro_, acc_;
};
