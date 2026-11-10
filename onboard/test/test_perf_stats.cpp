// PerfStats (perf_stats.hpp): the per-stage timing behind `--perf N`. The
// numbers it prints decide hardware purchases, so its arithmetic is checked:
// mean, p95 and max over known samples, the per-second rate, slowest-first
// ordering, reset, and that concurrent writers lose nothing.

#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "perf_stats.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

int main() {
    PerfStats p;
    // Off by default: nothing is recorded, by add() or by a Scope.
    p.add("map", 5.0);
    { PerfStats::Scope t(p, "scope"); }
    CHECK(!p.enabled() && p.rows(1.0).empty());
    p.enable(true);
    for (int i = 1; i <= 100; ++i) p.add("map", double(i));   // 1..100 ms
    for (int i = 0; i < 10; ++i) p.add("track", 2.0);
    const auto rows = p.rows(10.0);
    CHECK(rows.size() == 2);
    CHECK(rows[0].name == "map");                    // slowest mean first
    CHECK(std::fabs(rows[0].mean - 50.5) < 1e-9);
    CHECK(rows[0].max == 100.0);
    CHECK(rows[0].p95 >= 95.0 && rows[0].p95 <= 96.0);
    CHECK(std::fabs(rows[0].perS - 10.0) < 1e-9);    // 100 samples over 10 s
    CHECK(rows[1].name == "track" && rows[1].mean == 2.0 && rows[1].n == 10);

    const std::string line = p.line(10.0);
    CHECK(line.find("map 50.5 ms") != std::string::npos);
    CHECK(line.find("track 2.0 ms") != std::string::npos);

    // p95 is over the most recent 512 samples: old slow ones age out.
    PerfStats q;
    q.enable(true);
    for (int i = 0; i < 600; ++i) q.add("s", 100.0);
    for (int i = 0; i < 512; ++i) q.add("s", 1.0);
    CHECK(q.rows(1.0)[0].p95 == 1.0);
    CHECK(q.rows(1.0)[0].max == 100.0);              // max is over the whole window

    // A scope records itself.
    { PerfStats::Scope t(q, "scope"); }
    bool sawScope = false;
    for (const auto& r : q.rows(1.0)) sawScope |= (r.name == "scope" && r.n == 1);
    CHECK(sawScope);

    p.reset();
    CHECK(p.rows(1.0).empty());
    CHECK(p.line(1.0).find("no stages ran") != std::string::npos);

    // Concurrent writers (the Deliberator and the fly loop) lose no samples.
    PerfStats c;
    c.enable(true);
    std::vector<std::thread> th;
    for (int k = 0; k < 4; ++k)
        th.emplace_back([&c] { for (int i = 0; i < 10000; ++i) c.add("x", 1.0); });
    for (auto& t : th) t.join();
    CHECK(c.rows(1.0)[0].n == 40000);

    std::printf("test_perf_stats: %s\n", fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}
