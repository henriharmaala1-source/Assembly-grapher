#pragma once
// ---------------------------------------------------------------------------
// PerfStats -- where the frame time goes, on the aircraft's own board.
//
// Every compute decision this project has to make (stay on the Pi 5, step to
// a Zero 2 W or a Jetson, add an NPU for the detector) turns on one number
// nobody has measured: the per-frame cost of each stage ON THE REAL BOARD.
// The published figures are other people's code on other people's data
// (ROADMAP.md P7.1). So the stages time themselves here, and `--perf N`
// prints the breakdown every N seconds:
//
//   [perf] 10.0 s  voxel 38.2 ms (p95 51.0, max 63.1, 29.8/s) | vox.map 21.4 ...
//
// Cheap enough to leave compiled in: two steady_clock reads per stage and a
// short mutex-guarded append. Thread-safe: the Deliberator thread records,
// the fly loop prints.
// ---------------------------------------------------------------------------

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class PerfStats {
public:
    // One stage's duration, milliseconds.
    void add(const std::string& name, double ms) {
        std::lock_guard<std::mutex> lk(mx_);
        Stat& s = stats_[name];
        ++s.n;
        s.sum += ms;
        s.max = std::max(s.max, ms);
        if (s.ring.size() < kRing) s.ring.push_back(ms);
        else s.ring[s.next] = ms;
        s.next = (s.next + 1) % kRing;
    }

    // Times a scope: PerfStats::Scope t(perf(), "vox.map");
    class Scope {
    public:
        Scope(PerfStats& p, const char* name)
            : p_(p), name_(name), t0_(std::chrono::steady_clock::now()) {}
        ~Scope() {
            p_.add(name_, std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0_).count());
        }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
    private:
        PerfStats& p_;
        const char* name_;
        std::chrono::steady_clock::time_point t0_;
    };

    struct Row { std::string name; long n = 0; double mean = 0, p95 = 0, max = 0, perS = 0; };

    // The stages since the last reset, slowest mean first. `windowS` is the
    // wall time they cover, for the per-second rate.
    std::vector<Row> rows(double windowS) const {
        std::lock_guard<std::mutex> lk(mx_);
        std::vector<Row> out;
        for (const auto& kv : stats_) {
            const Stat& s = kv.second;
            if (!s.n) continue;
            Row r;
            r.name = kv.first;
            r.n = s.n;
            r.mean = s.sum / double(s.n);
            r.max = s.max;
            std::vector<double> v = s.ring;
            std::sort(v.begin(), v.end());
            r.p95 = v[std::min(v.size() - 1, size_t(0.95 * double(v.size())))];
            r.perS = windowS > 0 ? double(s.n) / windowS : 0;
            out.push_back(r);
        }
        std::sort(out.begin(), out.end(), [](const Row& a, const Row& b) { return a.mean > b.mean; });
        return out;
    }

    // One line for the console.
    std::string line(double windowS) const {
        const std::vector<Row> rs = rows(windowS);
        char buf[160];
        std::snprintf(buf, sizeof buf, "[perf] %.1f s ", windowS);
        std::string s = buf;
        if (rs.empty()) return s + " (no stages ran)";
        for (size_t i = 0; i < rs.size(); ++i) {
            std::snprintf(buf, sizeof buf, "%s%s %.1f ms (p95 %.1f, max %.1f, %.1f/s)", i ? " | " : " ",
                          rs[i].name.c_str(), rs[i].mean, rs[i].p95, rs[i].max, rs[i].perS);
            s += buf;
        }
        return s;
    }

    void reset() {
        std::lock_guard<std::mutex> lk(mx_);
        stats_.clear();
    }

private:
    static constexpr size_t kRing = 512;   // the p95 is over the last 512 samples
    struct Stat {
        long n = 0;
        double sum = 0, max = 0;
        std::vector<double> ring;
        size_t next = 0;
    };
    mutable std::mutex mx_;
    std::map<std::string, Stat> stats_;
};

// The process's one instance: stages anywhere record into it, main prints it.
inline PerfStats& perf() {
    static PerfStats p;
    return p;
}
